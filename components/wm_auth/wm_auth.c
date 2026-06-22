#include "include/wm_auth.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define ROR32(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

typedef struct {
    uint32_t state[8];
    uint64_t count;
    uint8_t  buf[64];
} sha256_ctx_t;

static void sha256_init(sha256_ctx_t *ctx)
{
    ctx->state[0] = 0x6a09e667;
    ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;
    ctx->count = 0;
}

static void sha256_transform(sha256_ctx_t *ctx, const uint8_t *blk)
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, h, t1, t2;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ((uint32_t)blk[i*4+0] << 24)
             | ((uint32_t)blk[i*4+1] << 16)
             | ((uint32_t)blk[i*4+2] <<  8)
             | ((uint32_t)blk[i*4+3]);
    }
    for (i = 16; i < 64; i++) {
        uint32_t s0 = ROR32(w[i-15], 7) ^ ROR32(w[i-15],18) ^ (w[i-15] >> 3);
        uint32_t s1 = ROR32(w[i-2], 17) ^ ROR32(w[i-2], 19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }

    a = ctx->state[0]; b = ctx->state[1];
    c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5];
    g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 64; i++) {
        uint32_t S1  = ROR32(e,6) ^ ROR32(e,11) ^ ROR32(e,25);
        uint32_t ch  = (e & f) ^ (~e & g);
        t1 = h + S1 + ch + K[i] + w[i];
        uint32_t S0  = ROR32(a,2) ^ ROR32(a,13) ^ ROR32(a,22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        t2 = S0 + maj;

        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    ctx->state[0] += a; ctx->state[1] += b;
    ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f;
    ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, size_t len)
{
    size_t fill = (size_t)(ctx->count & 63);
    ctx->count += (uint64_t)len;

    if (fill && (fill + len) >= 64) {
        memcpy(ctx->buf + fill, data, 64 - fill);
        sha256_transform(ctx, ctx->buf);
        data += 64 - fill;
        len  -= 64 - fill;
        fill  = 0;
    }

    while (len >= 64) {
        sha256_transform(ctx, data);
        data += 64;
        len  -= 64;
    }

    if (len) {
        memcpy(ctx->buf + fill, data, len);
    }
}

static void sha256_final(sha256_ctx_t *ctx, uint8_t out[32])
{
    uint64_t bits = ctx->count * 8;
    size_t fill = (size_t)(ctx->count & 63);
    uint8_t pad[64] = {0};
    pad[0] = 0x80;

    size_t pad_len = (fill < 56) ? (56 - fill) : (120 - fill);
    sha256_update(ctx, pad, pad_len);

    uint8_t bits_be[8];
    bits_be[0] = (uint8_t)(bits >> 56);
    bits_be[1] = (uint8_t)(bits >> 48);
    bits_be[2] = (uint8_t)(bits >> 40);
    bits_be[3] = (uint8_t)(bits >> 32);
    bits_be[4] = (uint8_t)(bits >> 24);
    bits_be[5] = (uint8_t)(bits >> 16);
    bits_be[6] = (uint8_t)(bits >>  8);
    bits_be[7] = (uint8_t)(bits);
    sha256_update(ctx, bits_be, 8);

    for (int i = 0; i < 8; i++) {
        out[i*4+0] = (uint8_t)(ctx->state[i] >> 24);
        out[i*4+1] = (uint8_t)(ctx->state[i] >> 16);
        out[i*4+2] = (uint8_t)(ctx->state[i] >>  8);
        out[i*4+3] = (uint8_t)(ctx->state[i]);
    }
}

static void sha256(const uint8_t *data, size_t len, uint8_t out[32])
{
    sha256_ctx_t ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    sha256_final(&ctx, out);
}

#define HMAC_BLOCK 64

static void hmac_sha256(
    const uint8_t *key,  size_t key_len,
    const uint8_t *msg,  size_t msg_len,
    uint8_t        out[32])
{
    uint8_t k[HMAC_BLOCK];
    uint8_t inner[32];

    memset(k, 0, HMAC_BLOCK);
    if (key_len > HMAC_BLOCK) {
        sha256(key, key_len, k);
    } else {
        memcpy(k, key, key_len);
    }

    uint8_t ipad[HMAC_BLOCK];
    for (int i = 0; i < HMAC_BLOCK; i++) {
        ipad[i] = k[i] ^ 0x36;
    }

    sha256_ctx_t ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, ipad, HMAC_BLOCK);
    sha256_update(&ctx, msg, msg_len);
    sha256_final(&ctx, inner);

    uint8_t opad[HMAC_BLOCK];
    for (int i = 0; i < HMAC_BLOCK; i++) {
        opad[i] = k[i] ^ 0x5C;
    }

    sha256_ctx_t ctx2;
    sha256_init(&ctx2);
    sha256_update(&ctx2, opad, HMAC_BLOCK);
    sha256_update(&ctx2, inner, 32);
    sha256_final(&ctx2, out);
}

static void compute_typed_response(
    const uint8_t per_meter_key[32],
    const uint8_t challenge[WM_CHALLENGE_LEN],
    const char *role,
    uint8_t out_response[WM_RESPONSE_LEN])
{
    uint8_t msg[32];
    size_t role_len = strlen(role);

    if (role_len > 16) {
        role_len = 16;
    }

    memcpy(msg, role, role_len);
    msg[role_len] = 0x00;
    memcpy(msg + role_len + 1, challenge, WM_CHALLENGE_LEN);

    uint8_t full[32];
    hmac_sha256(
        per_meter_key,
        32,
        msg,
        role_len + 1 + WM_CHALLENGE_LEN,
        full);

    memcpy(out_response, full, WM_RESPONSE_LEN);
}

void wm_auth_derive_key(
    const char *meter_id,
    uint8_t out_key[32])
{
    static const uint8_t master_key[32] = WM_MASTER_KEY;

    hmac_sha256(
        master_key,
        sizeof(master_key),
        (const uint8_t *)meter_id,
        strlen(meter_id),
        out_key);
}

void wm_auth_compute_response(
    const uint8_t per_meter_key[32],
    const uint8_t challenge[WM_CHALLENGE_LEN],
    uint8_t out_response[WM_RESPONSE_LEN])
{
    compute_typed_response(
        per_meter_key,
        challenge,
        WM_AUTH_LABEL_READER,
        out_response);
}

void wm_auth_compute_proof(
    const uint8_t per_meter_key[32],
    const uint8_t challenge[WM_CHALLENGE_LEN],
    uint8_t out_proof[WM_RESPONSE_LEN])
{
    compute_typed_response(
        per_meter_key,
        challenge,
        WM_AUTH_LABEL_METER,
        out_proof);
}

void wm_auth_encode_hex(
    const uint8_t response[WM_RESPONSE_LEN],
    char *out_hex)
{
    for (int i = 0; i < WM_RESPONSE_LEN; i++) {
        snprintf(out_hex + (i * 2), 3, "%02X", response[i]);
    }
    out_hex[WM_RESPONSE_LEN * 2] = '\0';
}

bool wm_auth_decode_hex(
    const char *hex,
    uint8_t out_response[WM_RESPONSE_LEN])
{
    if (!hex || strlen(hex) != WM_RESPONSE_LEN * 2) {
        return false;
    }

    for (int i = 0; i < WM_RESPONSE_LEN; i++) {
        unsigned int v = 0;
        if (sscanf(hex + (i * 2), "%02X", &v) != 1) {
            return false;
        }
        out_response[i] = (uint8_t)v;
    }

    return true;
}

bool wm_auth_compare(
    const uint8_t *a,
    const uint8_t *b,
    size_t len)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < len; i++) {
        diff |= a[i] ^ b[i];
    }
    return (diff == 0);
}