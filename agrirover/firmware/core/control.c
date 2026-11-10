#include "control.h"
#include <math.h>

#define DT (1.0f / AR_CTRL_HZ)
#define TWO_PI 6.28318530718f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void ar_ctrl_init(ar_ctrl_t *c) {
    *c = (ar_ctrl_t){0};
    /* Starting gains from the motor model (12 V, 150 rpm no-load => ~15.7 rad/s per unit duty).
     * They are a bench starting point, not tuned values. */
    c->kff = 1.0f / 15.7f;
    c->kp = 0.08f;
    c->ki = 0.60f;
    c->state = AR_ST_INIT;
}

void ar_ctrl_on_cmd_wheels(ar_ctrl_t *c, const ar_cmd_wheels_t *m, uint32_t t_ms) {
    if (isfinite(m->w_left) && isfinite(m->w_right)) {
        c->w_set[0] = clampf(m->w_left, -AR_W_MAX, AR_W_MAX);
        c->w_set[1] = clampf(m->w_right, -AR_W_MAX, AR_W_MAX);
        c->t_last_cmd = t_ms;
    }
}

void ar_ctrl_on_cmd_mode(ar_ctrl_t *c, const ar_cmd_mode_t *m, uint32_t t_ms) {
    switch (m->request) {
    case AR_REQ_DISABLE: c->enable_req = 0; break;
    case AR_REQ_ENABLE:
        c->enable_req = 1;
        c->t_last_cmd = t_ms;
        break;
    case AR_REQ_CLEAR_FAULT:
        /* latched faults clear only from a stopped, disabled robot; live conditions re-assert on the next tick */
        if (c->state == AR_ST_FAULT || c->state == AR_ST_ESTOP) {
            c->faults = 0;
            c->enable_req = 0;
            c->state = AR_ST_IDLE;
            c->t_state = t_ms;
        }
        break;
    default: break;
    }
}

void ar_ctrl_on_cmd_gains(ar_ctrl_t *c, const ar_cmd_gains_t *m) {
    if (m->kp >= 0 && m->kp < 2 && m->ki >= 0 && m->ki < 20 && m->kff >= 0 && m->kff < 1) {
        c->kp = m->kp;
        c->ki = m->ki;
        c->kff = m->kff;
    }
}

static void enter(ar_ctrl_t *c, uint8_t st, uint32_t t) {
    if (c->state != st) {
        c->state = st;
        c->t_state = t;
    }
}

static float slew(float cur, float target, int stopping) {
    float d = target - cur;
    /* moving towards zero (or stopping) may use the higher deceleration limit */
    int decel = stopping || (fabsf(target) < fabsf(cur)) || (target * cur < 0);
    float lim = (decel ? AR_DECEL_MAX : AR_ACCEL_MAX) * DT;
    return cur + clampf(d, -lim, lim);
}

void ar_ctrl_tick(ar_ctrl_t *c, const ar_inputs_t *in, ar_outputs_t *out) {
    uint32_t t = in->t_ms;
    /* ---- speed measurement */
    if (!c->have_prev) {
        c->enc_prev[0] = in->enc_left;
        c->enc_prev[1] = in->enc_right;
        c->have_prev = 1;
    }
    float raw0 = (float)(in->enc_left - c->enc_prev[0]) * (TWO_PI / AR_ENC_CPR) * AR_CTRL_HZ;
    float raw1 = (float)(in->enc_right - c->enc_prev[1]) * (TWO_PI / AR_ENC_CPR) * AR_CTRL_HZ;
    c->enc_prev[0] = in->enc_left;
    c->enc_prev[1] = in->enc_right;
    c->w_meas[0] += 0.35f * (raw0 - c->w_meas[0]); /* 1st-order low-pass, ~7 Hz */
    c->w_meas[1] += 0.35f * (raw1 - c->w_meas[1]);

    /* ---- fault detection (conditions are evaluated in every state) */
    uint16_t stop_faults = 0;
    if (in->estop_open) stop_faults |= AR_FAULT_ESTOP;
    if (in->bumper_open) stop_faults |= AR_FAULT_BUMPER;
    if (in->driver_fault && c->state == AR_ST_ACTIVE) stop_faults |= AR_FAULT_DRIVER;
    int oc = (in->i_left_ma > AR_I_LIMIT_MA || in->i_left_ma < -AR_I_LIMIT_MA ||
              in->i_right_ma > AR_I_LIMIT_MA || in->i_right_ma < -AR_I_LIMIT_MA);
    if (!oc) c->t_overcurrent = t;
    else if (t - c->t_overcurrent > AR_I_TRIP_MS) stop_faults |= AR_FAULT_OVERCURRENT;
    if (in->vbat_mv >= AR_VBAT_LOW_MV || in->vbat_mv == 0) c->t_lowbat = t; /* 0 = monitor absent */
    else if (t - c->t_lowbat > AR_VBAT_LOW_MS) stop_faults |= AR_FAULT_LOW_BATTERY;
    if (c->state == AR_ST_ACTIVE) {
        if (t - c->t_last_cmd > AR_LINK_LOST_MS) stop_faults |= AR_FAULT_LINK_LOST;
        if (in->vmot_mv > 8000) c->t_rail = t;
        else if (t - c->t_rail > 500 && t - c->t_state > 500) stop_faults |= AR_FAULT_RAIL;
    } else {
        c->t_rail = t;
    }

    /* ---- state machine */
    switch (c->state) {
    case AR_ST_INIT:
        enter(c, AR_ST_IDLE, t);
        break;
    case AR_ST_IDLE:
        if (stop_faults & (AR_FAULT_ESTOP | AR_FAULT_BUMPER | AR_FAULT_LOW_BATTERY)) {
            c->faults |= stop_faults;
            enter(c, (stop_faults & AR_FAULT_ESTOP) ? AR_ST_ESTOP : AR_ST_FAULT, t);
        } else if (c->enable_req) {
            c->integ[0] = c->integ[1] = 0;
            c->w_ramp[0] = c->w_ramp[1] = 0;
            c->t_last_cmd = t;
            enter(c, AR_ST_ACTIVE, t);
        }
        break;
    case AR_ST_ACTIVE:
        if (stop_faults) {
            c->faults |= stop_faults;
            enter(c, AR_ST_STOPPING, t);
        } else if (!c->enable_req) {
            enter(c, AR_ST_STOPPING, t);
        }
        break;
    case AR_ST_STOPPING:
        c->faults |= stop_faults;
        if (t - c->t_state >= AR_STOP_HOLD_MS && fabsf(c->w_ramp[0]) < 0.01f && fabsf(c->w_ramp[1]) < 0.01f) {
            c->enable_req = 0;
            if (c->faults & AR_FAULT_ESTOP) enter(c, AR_ST_ESTOP, t);
            else if (c->faults) enter(c, AR_ST_FAULT, t);
            else enter(c, AR_ST_IDLE, t);
        }
        break;
    case AR_ST_ESTOP:
    case AR_ST_FAULT:
        c->faults |= stop_faults; /* stays latched until AR_REQ_CLEAR_FAULT */
        c->enable_req = 0;
        break;
    default:
        enter(c, AR_ST_FAULT, t);
        break;
    }

    /* ---- set-point shaping and PI + feed-forward */
    int driving = (c->state == AR_ST_ACTIVE);
    int stopping = (c->state == AR_ST_STOPPING);
    int timed_out = (t - c->t_last_cmd > AR_CMD_TIMEOUT_MS);
    for (int k = 0; k < 2; k++) {
        float target = (driving && !timed_out) ? c->w_set[k] : 0.0f;
        c->w_ramp[k] = (driving || stopping) ? slew(c->w_ramp[k], target, stopping) : 0.0f;
    }
    float duty[2] = {0, 0};
    if (driving || stopping) {
        for (int k = 0; k < 2; k++) {
            float err = c->w_ramp[k] - c->w_meas[k];
            float u = c->kff * c->w_ramp[k] + c->kp * err + c->integ[k];
            float u_sat = clampf(u, -1.0f, 1.0f);
            /* anti-windup: integrate only when not saturated or when the error drives out of saturation */
            if (u == u_sat || (u > 1.0f && err < 0) || (u < -1.0f && err > 0)) c->integ[k] += c->ki * err * DT;
            c->integ[k] = clampf(c->integ[k], -0.6f, 0.6f);
            if (fabsf(c->w_ramp[k]) < 0.01f && fabsf(c->w_meas[k]) < 0.2f) { /* standstill: brake, bleed integrator */
                u_sat = 0;
                c->integ[k] *= 0.8f;
            }
            duty[k] = u_sat;
        }
    }
    out->duty_left = duty[0];
    out->duty_right = duty[1];
    out->driver_enable = (uint8_t)(driving || stopping);
    /* The rail stays commanded on while stopping so the drivers can brake; hardware (E-stop pole 1 + C3/R3)
     * removes it ~0.3 s after an E-stop regardless of what this code does. */
    out->rail_enable = (uint8_t)(driving || stopping);
}

void ar_ctrl_fill_state(const ar_ctrl_t *c, const ar_inputs_t *in, ar_state_t *s) {
    s->t_ms = in->t_ms;
    s->enc_left = in->enc_left;
    s->enc_right = in->enc_right;
    s->w_left = c->w_meas[0];
    s->w_right = c->w_meas[1];
    s->gyro_z = in->gyro_z;
    s->i_left_ma = in->i_left_ma;
    s->i_right_ma = in->i_right_ma;
    s->vbat_mv = in->vbat_mv;
    s->ibat_ma = in->ibat_ma;
    s->vmot_mv = in->vmot_mv;
    s->state = c->state;
    s->faults = c->faults;
}
