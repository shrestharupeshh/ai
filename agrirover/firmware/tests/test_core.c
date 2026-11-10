/* Host unit tests for the firmware core (protocol, control, safety) with a simulated motor plant.
 * Build + run: make test */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../core/control.h"
#include "../core/protocol.h"
#include "../hal/hal.h"

int app_run(int max_ticks);

static int fails, checks;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------------------------------------------------------- simulated plant */
typedef struct {
    double w[2], pos[2];
    uint32_t t_ms;
    int estop, bumper, blocked, vbat_mv;
    double rail;   /* motor rail voltage */
    ar_outputs_t last;
} plant_t;

static void plant_step(plant_t *p, const ar_outputs_t *o) {
    const double dt = 0.01, tau = 0.08, k = 15.7 * 12.8 / 12.0;
    p->rail += ((o->rail_enable && !p->estop) ? (12.8 - p->rail) : -p->rail) * 0.5;
    for (int i = 0; i < 2; i++) {
        double duty = i ? o->duty_right : o->duty_left;
        double target = (o->driver_enable && p->rail > 6.0) ? k * duty : 0.0;
        if (p->blocked) target = 0;
        p->w[i] += (target - p->w[i]) * dt / tau;
        p->pos[i] += p->w[i] * dt;
    }
    p->t_ms += 10;
    p->last = *o;
}

static void plant_inputs(const plant_t *p, ar_inputs_t *in) {
    memset(in, 0, sizeof *in);
    in->t_ms = p->t_ms;
    in->enc_left = (int32_t)lround(p->pos[0] * AR_ENC_CPR / (2 * M_PI));
    in->enc_right = (int32_t)lround(p->pos[1] * AR_ENC_CPR / (2 * M_PI));
    for (int i = 0; i < 2; i++) {
        double duty = i ? p->last.duty_right : p->last.duty_left;
        double cur = (duty * 12.8 - p->w[i] / 15.7 * 12.0) / 2.18 * 1000.0;
        if (!p->last.driver_enable) cur = 0;
        if (i) in->i_right_ma = (int16_t)cur; else in->i_left_ma = (int16_t)cur;
    }
    in->vbat_mv = (uint16_t)p->vbat_mv;
    in->vmot_mv = (uint16_t)(p->rail * 1000);
    in->estop_open = (uint8_t)p->estop;
    in->bumper_open = (uint8_t)p->bumper;
    in->gyro_z = 0;
}

static void run(ar_ctrl_t *c, plant_t *p, int ticks, float wl, float wr, int send_cmd) {
    ar_inputs_t in;
    ar_outputs_t out;
    for (int i = 0; i < ticks; i++) {
        if (send_cmd) {
            ar_cmd_wheels_t m = {wl, wr};
            ar_ctrl_on_cmd_wheels(c, &m, p->t_ms);
        }
        plant_inputs(p, &in);
        ar_ctrl_tick(c, &in, &out);
        plant_step(p, &out);
    }
}

static void fresh(ar_ctrl_t *c, plant_t *p) {
    memset(p, 0, sizeof *p);
    p->vbat_mv = 12800;
    ar_ctrl_init(c);
    run(c, p, 2, 0, 0, 0);
}

static void enable(ar_ctrl_t *c, plant_t *p) {
    ar_cmd_mode_t m = {AR_REQ_ENABLE};
    ar_ctrl_on_cmd_mode(c, &m, p->t_ms);
    run(c, p, 2, 0, 0, 1);
}

/* ---------------------------------------------------------------- tests */
static void test_protocol(void) {
    uint8_t frame[AR_MAX_FRAME], payload[AR_MAX_PAYLOAD], type, seq;
    size_t len;
    ar_parser_t ps = {0};
    ar_state_t s = {.t_ms = 123456, .enc_left = -5, .enc_right = 70000, .w_left = 1.5f, .vbat_mv = 12800, .state = 2, .faults = 0x21};
    size_t n = ar_frame_encode(AR_MSG_STATE, 7, &s, sizeof s, frame, sizeof frame);
    CHECK(n > 0 && frame[n - 1] == 0, "frame encode");
    int zeros = 0;
    for (size_t i = 0; i + 1 < n; i++) zeros += frame[i] == 0;
    CHECK(zeros == 0, "COBS output must not contain 0x00 before the delimiter");
    int got = 0;
    for (size_t i = 0; i < n; i++) got += ar_parser_feed(&ps, frame[i], &type, &seq, payload, &len);
    CHECK(got == 1 && type == AR_MSG_STATE && seq == 7 && len == sizeof s && !memcmp(&s, payload, len), "round trip");
    frame[3] ^= 0x10; /* corrupt */
    got = 0;
    for (size_t i = 0; i < n; i++) got += ar_parser_feed(&ps, frame[i], &type, &seq, payload, &len);
    CHECK(got == 0 && ps.rx_crc_err == 1, "corrupted frame rejected (crc_err=%u)", (unsigned)ps.rx_crc_err);
    /* garbage then a good frame: the parser must resynchronise */
    for (int i = 0; i < 200; i++) ar_parser_feed(&ps, (uint8_t)(i * 37 + 1) | 1, &type, &seq, payload, &len);
    ar_parser_feed(&ps, 0, &type, &seq, payload, &len);
    ar_cmd_wheels_t cw = {2.0f, -2.0f};
    n = ar_frame_encode(AR_MSG_CMD_WHEELS, 9, &cw, sizeof cw, frame, sizeof frame);
    got = 0;
    for (size_t i = 0; i < n; i++) got += ar_parser_feed(&ps, frame[i], &type, &seq, payload, &len);
    CHECK(got == 1 && type == AR_MSG_CMD_WHEELS, "resync after garbage");
    CHECK(ar_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE check value");
    CHECK(sizeof(ar_state_t) == 37, "ar_state_t wire size = %zu", sizeof(ar_state_t));
}

static void test_speed_tracking(void) {
    ar_ctrl_t c; plant_t p;
    fresh(&c, &p);
    CHECK(c.state == AR_ST_IDLE, "idle after init");
    run(&c, &p, 50, 6, 6, 1);
    CHECK(fabs(p.w[0]) < 1e-6 && p.last.rail_enable == 0, "no motion and rail off while not enabled");
    enable(&c, &p);
    CHECK(c.state == AR_ST_ACTIVE, "active after enable");
    double w_prev = 0, max_acc = 0;
    for (int i = 0; i < 300; i++) { run(&c, &p, 1, 6, 6, 1); double a = (p.w[0] - w_prev) / 0.01; if (a > max_acc) max_acc = a; w_prev = p.w[0]; }
    CHECK(fabs(p.w[0] - 6.0) < 0.15 && fabs(p.w[1] - 6.0) < 0.15, "steady-state speed %.3f / %.3f (want 6.0)", p.w[0], p.w[1]);
    CHECK(max_acc < AR_ACCEL_MAX * 1.6, "acceleration limited: %.1f rad/s^2", max_acc);
    run(&c, &p, 300, 4, -4, 1);
    CHECK(fabs(p.w[0] - 4.0) < 0.15 && fabs(p.w[1] + 4.0) < 0.15, "turn in place %.2f / %.2f", p.w[0], p.w[1]);
}

static void test_cmd_timeout_and_link_loss(void) {
    ar_ctrl_t c; plant_t p;
    fresh(&c, &p); enable(&c, &p);
    run(&c, &p, 200, 8, 8, 1);
    run(&c, &p, 100, 0, 0, 0); /* 1 s without commands */
    CHECK(fabs(p.w[0]) < 0.3, "stops after command timeout (w=%.2f)", p.w[0]);
    CHECK(c.state == AR_ST_ACTIVE, "still active inside the link-loss window");
    run(&c, &p, 200, 0, 0, 0);
    CHECK(c.state == AR_ST_FAULT && (c.faults & AR_FAULT_LINK_LOST) && p.last.rail_enable == 0, "link loss latches a fault and drops the rail");
}

static void test_estop(void) {
    ar_ctrl_t c; plant_t p;
    fresh(&c, &p); enable(&c, &p);
    run(&c, &p, 300, 10, 10, 1); /* 0.5 m/s */
    double pos0 = p.pos[0];
    p.estop = 1;
    run(&c, &p, 1, 10, 10, 1);
    CHECK(c.state == AR_ST_STOPPING, "E-stop -> controlled stop");
    int ticks = 1;
    while (fabs(p.w[0]) > 0.2 && ticks < 200) { run(&c, &p, 1, 10, 10, 1); ticks++; }
    double dist = (p.pos[0] - pos0) * 0.05;
    CHECK(dist < 0.15, "stop distance %.3f m from 0.5 m/s (requirement 0.15 m; simulated plant)", dist);
    run(&c, &p, 60, 10, 10, 1);
    CHECK(c.state == AR_ST_ESTOP && p.last.rail_enable == 0 && p.last.driver_enable == 0, "E-stop state reached, outputs off");
    ar_cmd_mode_t en = {AR_REQ_ENABLE};
    ar_ctrl_on_cmd_mode(&c, &en, p.t_ms);
    run(&c, &p, 50, 5, 5, 1);
    CHECK(c.state == AR_ST_ESTOP && fabs(p.w[0]) < 0.05, "cannot re-enable while latched");
    ar_cmd_mode_t clr = {AR_REQ_CLEAR_FAULT};
    ar_ctrl_on_cmd_mode(&c, &clr, p.t_ms);
    run(&c, &p, 5, 0, 0, 1);
    CHECK(c.state == AR_ST_ESTOP, "clear is refused while the E-stop is still pressed");
    p.estop = 0;
    ar_ctrl_on_cmd_mode(&c, &clr, p.t_ms);
    run(&c, &p, 5, 0, 0, 1);
    CHECK(c.state == AR_ST_IDLE && c.faults == 0, "clear works after release");
    ar_ctrl_on_cmd_mode(&c, &en, p.t_ms);
    run(&c, &p, 200, 5, 5, 1);
    CHECK(c.state == AR_ST_ACTIVE && fabs(p.w[0] - 5) < 0.2, "drives again after clear + enable");
}

static void test_bumper_stall_lowbat(void) {
    ar_ctrl_t c; plant_t p;
    fresh(&c, &p); enable(&c, &p);
    run(&c, &p, 200, 6, 6, 1);
    p.bumper = 1;
    run(&c, &p, 100, 6, 6, 1);
    CHECK(c.state == AR_ST_FAULT && (c.faults & AR_FAULT_BUMPER) && fabs(p.w[0]) < 0.05, "bumper latches a fault and stops");
    fresh(&c, &p); enable(&c, &p);
    p.blocked = 1;
    run(&c, &p, 400, 10, 10, 1);
    CHECK(c.state == AR_ST_FAULT && (c.faults & AR_FAULT_OVERCURRENT), "stall trips over-current (state %d faults 0x%x)", c.state, c.faults);
    fresh(&c, &p); enable(&c, &p);
    p.vbat_mv = 10900;
    run(&c, &p, 700, 3, 3, 1);
    CHECK(c.state == AR_ST_FAULT && (c.faults & AR_FAULT_LOW_BATTERY), "low battery trips after the hold time");
    fresh(&c, &p);
    ar_cmd_wheels_t bad = {NAN, 1e9f};
    ar_ctrl_on_cmd_wheels(&c, &bad, 0);
    CHECK(c.w_set[0] == 0 && c.w_set[1] == 0, "non-finite command ignored");
}

/* ---------------------------------------------------------------- host HAL for the app-level smoke test */
static plant_t hp;
static uint8_t rxq[512]; static size_t rxn, rxi;
static ar_parser_t txp; static int tx_states, tx_infos; static ar_state_t last_state;
void hal_init(void) { memset(&hp, 0, sizeof hp); hp.vbat_mv = 12800; }
uint32_t hal_millis(void) { return hp.t_ms; }
void hal_wait_tick(void) {}
void hal_read_inputs(ar_inputs_t *in) { plant_inputs(&hp, in); }
void hal_write_outputs(const ar_outputs_t *o) {
    plant_step(&hp, o);
    if (rxi >= rxn && hp.t_ms % 50 == 0) { /* host keeps commanding 5 rad/s at 20 Hz */
        ar_cmd_wheels_t m = {5, 5};
        rxn = ar_frame_encode(AR_MSG_CMD_WHEELS, 0, &m, sizeof m, rxq, sizeof rxq); rxi = 0;
    }
}
int hal_uart_read(uint8_t *b) { if (rxi < rxn) { *b = rxq[rxi++]; return 1; } return 0; }
void hal_uart_write(const uint8_t *d, size_t n) {
    uint8_t type, seq, payload[AR_MAX_PAYLOAD]; size_t len;
    for (size_t i = 0; i < n; i++)
        if (ar_parser_feed(&txp, d[i], &type, &seq, payload, &len)) {
            if (type == AR_MSG_STATE && len == sizeof last_state) { memcpy(&last_state, payload, len); tx_states++; }
            if (type == AR_MSG_INFO) tx_infos++;
        }
}
void hal_watchdog_kick(void) {}
void hal_led(int on) { (void)on; }

static void test_app_loop(void) {
    ar_cmd_mode_t en = {AR_REQ_ENABLE};
    rxn = ar_frame_encode(AR_MSG_CMD_MODE, 0, &en, sizeof en, rxq, sizeof rxq); rxi = 0;
    app_run(300);
    CHECK(tx_infos == 1, "INFO sent at boot");
    CHECK(tx_states == 150, "50 Hz telemetry: %d frames in 3 s", tx_states);
    CHECK(last_state.state == AR_ST_ACTIVE && fabsf(last_state.w_left - 5.0f) < 0.2f, "app loop drives the plant: state %d w %.2f", last_state.state, last_state.w_left);
    CHECK(last_state.enc_left > 1000, "encoder counts reported (%d)", (int)last_state.enc_left);
}

int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "--emit")) { /* reference frames for the Python cross-check */
        uint8_t f[AR_MAX_FRAME];
        ar_state_t s = {.t_ms = 1000, .enc_left = 4480, .enc_right = -4480, .w_left = 1.25f, .w_right = -1.25f, .gyro_z = 0.5f,
                        .i_left_ma = 350, .i_right_ma = -340, .vbat_mv = 13100, .ibat_ma = 1500, .vmot_mv = 13000, .state = AR_ST_ACTIVE, .faults = AR_FAULT_BUMPER};
        size_t n = ar_frame_encode(AR_MSG_STATE, 42, &s, sizeof s, f, sizeof f);
        for (size_t i = 0; i < n; i++) printf("%02x", f[i]);
        printf("\n");
        ar_cmd_wheels_t w = {3.5f, -1.0f};
        n = ar_frame_encode(AR_MSG_CMD_WHEELS, 1, &w, sizeof w, f, sizeof f);
        for (size_t i = 0; i < n; i++) printf("%02x", f[i]);
        printf("\n");
        return 0;
    }
    test_protocol();
    test_speed_tracking();
    test_cmd_timeout_and_link_loss();
    test_estop();
    test_bumper_stall_lowbat();
    test_app_loop();
    printf("%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
