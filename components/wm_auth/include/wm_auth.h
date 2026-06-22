#ifndef WM_AUTH_H
#define WM_AUTH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef WM_CHALLENGE_LEN
#define WM_CHALLENGE_LEN 8
#endif

#ifndef WM_RESPONSE_LEN
#define WM_RESPONSE_LEN 8
#endif

#ifndef WM_AUTH_LABEL_READER
#define WM_AUTH_LABEL_READER "READER"
#endif

#ifndef WM_AUTH_LABEL_METER
#define WM_AUTH_LABEL_METER "METER"
#endif

/* Same master key must be compiled into BOTH reader and meter */
#define WM_MASTER_KEY { \
    0x11,0x22,0x33,0x44, \
    0x55,0x66,0x77,0x88, \
    0x99,0xAA,0xBB,0xCC, \
    0xDD,0xEE,0xFF,0x10, \
    0x21,0x32,0x43,0x54, \
    0x65,0x76,0x87,0x98, \
    0xA9,0xBA,0xCB,0xDC, \
    0xED,0xFE,0x0F,0x1E  \
}

void wm_auth_derive_key(
    const char *meter_id,
    uint8_t     out_key[32]);

void wm_auth_compute_response(
    const uint8_t per_meter_key[32],
    const uint8_t challenge[WM_CHALLENGE_LEN],
    uint8_t       out_response[WM_RESPONSE_LEN]);

void wm_auth_compute_proof(
    const uint8_t per_meter_key[32],
    const uint8_t challenge[WM_CHALLENGE_LEN],
    uint8_t       out_proof[WM_RESPONSE_LEN]);

void wm_auth_encode_hex(
    const uint8_t response[WM_RESPONSE_LEN],
    char         *out_hex);

bool wm_auth_decode_hex(
    const char *hex,
    uint8_t     out_response[WM_RESPONSE_LEN]);

bool wm_auth_compare(
    const uint8_t *a,
    const uint8_t *b,
    size_t         len);

#endif /* WM_AUTH_H */