/* AgriRover GH-1 base controller - application loop (hardware independent). */
#include "../core/control.h"
#include "../core/protocol.h"
#include "../hal/hal.h"

#define FW_MAJOR 0
#define FW_MINOR 1

static ar_ctrl_t ctrl;
static ar_parser_t parser;
static uint8_t tx_seq;

static void send(uint8_t type, const void *payload, size_t len) {
    uint8_t frame[AR_MAX_FRAME];
    size_t n = ar_frame_encode(type, tx_seq++, payload, len, frame, sizeof frame);
    if (n) hal_uart_write(frame, n);
}

static void send_info(void) {
    ar_info_t info = {AR_PROTO_VERSION, FW_MAJOR, FW_MINOR, AR_ENC_CPR};
    send(AR_MSG_INFO, &info, sizeof info);
}

static void poll_uart(uint32_t now) {
    uint8_t b, type, seq, payload[AR_MAX_PAYLOAD];
    size_t len;
    while (hal_uart_read(&b)) {
        if (!ar_parser_feed(&parser, b, &type, &seq, payload, &len)) continue;
        (void)seq;
        if (type == AR_MSG_CMD_WHEELS && len == sizeof(ar_cmd_wheels_t)) {
            ar_ctrl_on_cmd_wheels(&ctrl, (const ar_cmd_wheels_t *)payload, now);
        } else if (type == AR_MSG_CMD_MODE && len == sizeof(ar_cmd_mode_t)) {
            if (((const ar_cmd_mode_t *)payload)->request == AR_REQ_INFO) send_info();
            else ar_ctrl_on_cmd_mode(&ctrl, (const ar_cmd_mode_t *)payload, now);
        } else if (type == AR_MSG_CMD_GAINS && len == sizeof(ar_cmd_gains_t)) {
            ar_ctrl_on_cmd_gains(&ctrl, (const ar_cmd_gains_t *)payload);
        }
    }
}

int app_run(int max_ticks) { /* max_ticks < 0: run forever (target); >= 0: bounded (host tests) */
    ar_inputs_t in;
    ar_outputs_t out;
    ar_state_t st;
    uint32_t tick = 0;
    hal_init();
    ar_ctrl_init(&ctrl);
    send_info();
    while (max_ticks < 0 || (int)tick < max_ticks) {
        hal_wait_tick();
        uint32_t now = hal_millis();
        poll_uart(now);
        hal_read_inputs(&in);
        ar_ctrl_tick(&ctrl, &in, &out);
        hal_write_outputs(&out);
        if ((tick & 1u) == 0) { /* 50 Hz telemetry */
            ar_ctrl_fill_state(&ctrl, &in, &st);
            send(AR_MSG_STATE, &st, sizeof st);
        }
        hal_led(ctrl.state == AR_ST_ACTIVE ? 1 : ((tick / 50) & 1));
        hal_watchdog_kick();
        tick++;
    }
    return 0;
}

#ifndef AR_HOST_BUILD
int main(void) { return app_run(-1); }
#endif
