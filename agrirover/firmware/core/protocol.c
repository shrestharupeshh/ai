#include "protocol.h"
#include <string.h>

uint16_t ar_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t ar_cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t code_idx = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_idx] = code;
            code_idx = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[code_idx] = code;
                code_idx = o++;
                code = 1;
            }
        }
    }
    out[code_idx] = code;
    return o;
}

size_t ar_cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t i = 0, o = 0;
    while (i < len) {
        uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > len) return 0;
        for (uint8_t k = 1; k < code; k++) out[o++] = in[i++];
        if (code != 0xFF && i < len) out[o++] = 0;
    }
    return o;
}

size_t ar_frame_encode(uint8_t type, uint8_t seq, const void *payload, size_t len, uint8_t *out, size_t out_cap) {
    uint8_t raw[AR_MAX_PAYLOAD + 4];
    if (len > AR_MAX_PAYLOAD || out_cap < len + 4 + 2 + 1) return 0;
    raw[0] = type;
    raw[1] = seq;
    if (len) memcpy(raw + 2, payload, len);
    uint16_t crc = ar_crc16(raw, len + 2);
    raw[len + 2] = (uint8_t)(crc & 0xFF);
    raw[len + 3] = (uint8_t)(crc >> 8);
    size_t n = ar_cobs_encode(raw, len + 4, out);
    out[n++] = 0;
    return n;
}

int ar_parser_feed(ar_parser_t *p, uint8_t byte, uint8_t *type, uint8_t *seq, uint8_t *payload, size_t *len) {
    if (byte != 0) {
        if (p->n < sizeof p->buf) {
            p->buf[p->n++] = byte;
        } else {
            p->rx_overrun++;
            p->n = sizeof p->buf + 1; /* poison until the next delimiter */
        }
        return 0;
    }
    size_t n = p->n;
    p->n = 0;
    if (n == 0) return 0;
    if (n > sizeof p->buf) return 0;
    uint8_t raw[AR_MAX_FRAME];
    size_t m = ar_cobs_decode(p->buf, n, raw);
    if (m < 4 || m - 4 > AR_MAX_PAYLOAD) {
        p->rx_crc_err++;
        return 0;
    }
    uint16_t crc = (uint16_t)(raw[m - 2] | (raw[m - 1] << 8));
    if (crc != ar_crc16(raw, m - 2)) {
        p->rx_crc_err++;
        return 0;
    }
    *type = raw[0];
    *seq = raw[1];
    *len = m - 4;
    memcpy(payload, raw + 2, m - 4);
    p->rx_ok++;
    return 1;
}
