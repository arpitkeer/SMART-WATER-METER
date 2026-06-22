#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define PAYLOAD_VERSION_DEFAULT 1

#include "../../wm_auth/include/wm_auth.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * PAYLOAD
 * ========================================================= */

typedef struct __attribute__((packed))
{
    uint8_t tamper_fw;

    uint8_t meter_type;

    uint8_t timestamp[5];

    uint8_t volume_multiplier;

    uint8_t meter_counter[3];

    uint16_t interval_minutes;

    uint8_t battery_level;

    uint8_t meter_id_len;

    char meter_id[16];

    uint8_t install_date[3];

    uint16_t install_counter;

    uint8_t challenge[WM_CHALLENGE_LEN];

    uint16_t crc;

} water_meter_payload_t;

/* =========================================================
 * CONFIG
 * ========================================================= */

typedef struct
{
    const char *device_name;
} nimble_adv_config_t;

typedef void (*ble_cmd_cb_t)(const char *cmd);

/* =========================================================
 * LIFECYCLE
 * ========================================================= */

esp_err_t nimble_adv_init(
    const nimble_adv_config_t *config);

void nimble_adv_deinit(void);

bool nimble_adv_start(void);

void nimble_adv_stop(void);

/* =========================================================
 * CONNECTION STATE
 * ========================================================= */

bool nimble_adv_is_connected(void);

bool nimble_adv_is_authorized(void);

bool nimble_adv_is_notify_enabled(void);

void nimble_adv_authorize(
    bool enabled);

void nimble_adv_disconnect(void);

int64_t nimble_adv_get_connect_time_us(void);

/* =========================================================
 * COMMANDS
 * ========================================================= */

void nimble_adv_set_cmd_cb(
    ble_cmd_cb_t cb);

/* =========================================================
 * DATA
 * ========================================================= */

void nimble_adv_update_data(
    uint32_t total,
    bool tamper);

void nimble_adv_flush(void);

/* =========================================================
 * TEXT NOTIFICATIONS
 * ========================================================= */

void nimble_adv_send_text(
    const char *msg);

void nimble_adv_send_control_text(
    const char *msg);

/* =========================================================
 * AUTH
 * ========================================================= */

void nimble_adv_get_challenge(
    uint8_t out[WM_CHALLENGE_LEN]);
uint8_t build_tamper_fw(
	    bool case_tamper,
	    bool magnetic_tamper,
	    bool removal_tamper);
void put_u40(
		    uint8_t *buf,
		    uint64_t value);
void put_u24(
			    uint8_t *buf,
			    uint32_t value);

/* =========================================================
 * PAYLOAD
 * ========================================================= */

void build_payload(
    water_meter_payload_t *p);

#ifdef __cplusplus
}
#endif