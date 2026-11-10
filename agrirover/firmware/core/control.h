/* Wheel-speed control and safety supervisor - hardware independent. */
#ifndef AR_CONTROL_H
#define AR_CONTROL_H
#include <stdint.h>
#include "protocol.h"

#define AR_CTRL_HZ 100
#define AR_ENC_CPR 4480             /* 64 CPR x 70:1 */
#define AR_W_MAX 12.0f              /* rad/s set-point clamp (0.6 m/s on 100 mm wheels) */
#define AR_ACCEL_MAX 10.0f          /* rad/s^2  (0.5 m/s^2) */
#define AR_DECEL_MAX 30.0f          /* rad/s^2  (1.5 m/s^2), used for every stop */
#define AR_CMD_TIMEOUT_MS 300       /* no command -> ramp to zero */
#define AR_LINK_LOST_MS 2000        /* no command -> drop the motor rail */
#define AR_STOP_HOLD_MS 300         /* controlled-stop window before the rail is released */
#define AR_I_LIMIT_MA 3000
#define AR_I_TRIP_MS 1500
#define AR_VBAT_LOW_MV 11200
#define AR_VBAT_LOW_MS 5000

enum ar_safety_state { AR_ST_INIT = 0, AR_ST_IDLE, AR_ST_ACTIVE, AR_ST_STOPPING, AR_ST_ESTOP, AR_ST_FAULT };

#define AR_FAULT_ESTOP (1u << 0)
#define AR_FAULT_BUMPER (1u << 1)
#define AR_FAULT_OVERCURRENT (1u << 2)
#define AR_FAULT_DRIVER (1u << 3)
#define AR_FAULT_LOW_BATTERY (1u << 4)
#define AR_FAULT_LINK_LOST (1u << 5)
#define AR_FAULT_RAIL (1u << 6) /* motor rail commanded on but VMOT absent (fuse / E-stop wiring) */

typedef struct {            /* sampled inputs for one control tick */
    uint32_t t_ms;
    int32_t enc_left, enc_right;
    int16_t i_left_ma, i_right_ma;
    uint16_t vbat_mv, vmot_mv;
    int16_t ibat_ma;
    float gyro_z;
    uint8_t estop_open;     /* 1 = E-stop pressed or loop broken */
    uint8_t bumper_open;    /* 1 = bumper hit or loop broken */
    uint8_t driver_fault;   /* 1 = driver DIAG asserted */
} ar_inputs_t;

typedef struct {            /* actuator commands produced by one tick */
    float duty_left, duty_right; /* -1..+1, 0 = brake (both half-bridge outputs low) */
    uint8_t driver_enable;       /* EN pin of the motor drivers */
    uint8_t rail_enable;         /* MOTOR_EN: gate of the motor-rail switch */
} ar_outputs_t;

typedef struct {
    float kp, ki, kff;
    float integ[2];
    float w_set[2];   /* requested */
    float w_ramp[2];  /* slew-limited set-point */
    float w_meas[2];
    int32_t enc_prev[2];
    uint8_t have_prev;
    uint8_t state;
    uint16_t faults;
    uint32_t t_last_cmd, t_state, t_overcurrent, t_lowbat, t_rail;
    uint8_t enable_req;
} ar_ctrl_t;

void ar_ctrl_init(ar_ctrl_t *c);
void ar_ctrl_on_cmd_wheels(ar_ctrl_t *c, const ar_cmd_wheels_t *m, uint32_t t_ms);
void ar_ctrl_on_cmd_mode(ar_ctrl_t *c, const ar_cmd_mode_t *m, uint32_t t_ms);
void ar_ctrl_on_cmd_gains(ar_ctrl_t *c, const ar_cmd_gains_t *m);
void ar_ctrl_tick(ar_ctrl_t *c, const ar_inputs_t *in, ar_outputs_t *out);
void ar_ctrl_fill_state(const ar_ctrl_t *c, const ar_inputs_t *in, ar_state_t *s);
#endif
