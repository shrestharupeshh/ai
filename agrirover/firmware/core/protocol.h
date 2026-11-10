/* AgriRover GH-1 base-controller serial protocol (MCU <-> SBC).
 *
 * Frame on the wire:  COBS( type:u8 | seq:u8 | payload | crc16:le )  0x00
 * CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.
 * All multi-byte fields little-endian, structs packed.
 * The Python mirror lives in software/agrirover/hw/protocol.py; the two are
 * cross-checked by tests/test_protocol_xcheck.py.
 */
#ifndef AR_PROTOCOL_H
#define AR_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#define AR_PROTO_VERSION 1
#define AR_MAX_PAYLOAD 48
#define AR_MAX_FRAME (AR_MAX_PAYLOAD + 4 + 2 + 1) /* +hdr +crc, +COBS overhead, +delimiter */

enum ar_msg_type {
    AR_MSG_CMD_WHEELS = 0x01, /* ar_cmd_wheels_t   host -> mcu */
    AR_MSG_CMD_MODE = 0x02,   /* ar_cmd_mode_t     host -> mcu */
    AR_MSG_CMD_GAINS = 0x03,  /* ar_cmd_gains_t    host -> mcu */
    AR_MSG_STATE = 0x81,      /* ar_state_t        mcu -> host, 50 Hz */
    AR_MSG_INFO = 0x82        /* ar_info_t         mcu -> host, on request / at boot */
};

enum ar_mode_request { AR_REQ_DISABLE = 0, AR_REQ_ENABLE = 1, AR_REQ_CLEAR_FAULT = 2, AR_REQ_INFO = 3 };

#pragma pack(push, 1)
typedef struct {
    float w_left;  /* wheel angular velocity set-point [rad/s], + = forward */
    float w_right;
} ar_cmd_wheels_t;

typedef struct {
    uint8_t request; /* enum ar_mode_request */
} ar_cmd_mode_t;

typedef struct {
    float kp, ki, kff; /* duty per rad/s, duty per rad, duty per rad/s */
} ar_cmd_gains_t;

typedef struct {
    uint32_t t_ms;
    int32_t enc_left; /* cumulative encoder counts (4x decoding) */
    int32_t enc_right;
    float w_left; /* measured wheel speed [rad/s] */
    float w_right;
    float gyro_z;      /* [rad/s], NaN if the IMU is absent */
    int16_t i_left_ma; /* motor currents from the driver current-monitor outputs */
    int16_t i_right_ma;
    uint16_t vbat_mv;
    int16_t ibat_ma;
    uint16_t vmot_mv;
    uint8_t state;   /* enum ar_safety_state */
    uint16_t faults; /* AR_FAULT_* bit mask */
} ar_state_t;

typedef struct {
    uint8_t proto_version;
    uint8_t fw_major, fw_minor;
    uint16_t enc_cpr; /* counts per wheel revolution */
} ar_info_t;
#pragma pack(pop)

uint16_t ar_crc16(const uint8_t *data, size_t len);
size_t ar_cobs_encode(const uint8_t *in, size_t len, uint8_t *out);
size_t ar_cobs_decode(const uint8_t *in, size_t len, uint8_t *out);

/* Build a complete wire frame (including the trailing 0x00). Returns its length, 0 on error. */
size_t ar_frame_encode(uint8_t type, uint8_t seq, const void *payload, size_t len, uint8_t *out, size_t out_cap);

typedef struct {
    uint8_t buf[AR_MAX_FRAME];
    size_t n;
    uint32_t rx_ok, rx_crc_err, rx_overrun;
} ar_parser_t;

/* Feed one received byte. Returns 1 when a valid frame is available in type/seq/payload/len. */
int ar_parser_feed(ar_parser_t *p, uint8_t byte, uint8_t *type, uint8_t *seq, uint8_t *payload, size_t *len);
#endif
