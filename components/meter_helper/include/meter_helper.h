#ifndef METER_HELPER_H
#define METER_HELPER_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"
#include "esp_attr.h"
#include "../../meter_storage/include/meter_storage.h"
#include "../../lora_manager/include/lora_manager.h"

/* Keep your fake-time constants if you still use them */
#define FAKE_YEAR   2026
#define FAKE_MONTH  5
#define FAKE_DAY    31
#define FAKE_HOUR   20
#define FAKE_MIN    50
#define FAKE_SEC    56

#define AUTH_WINDOW_US (5ULL * 60ULL * 1000000ULL)

/* Shared feature configuration (defined once in main.c) */
typedef struct
{
    bool ble_enabled;
    bool ble_mag_window_enabled;
    bool ble_auth_window_enabled;
    bool ble_month_lock_enabled;
    bool ble_month_lock_auto_release;
    uint32_t ble_mag_window_sec;
    uint32_t ble_auth_window_sec;

    bool board_has_lora;
    bool lora_enabled;
    bool lora_uplink_window_enabled;
    bool lora_downlink_window_enabled;
    bool lora_listen_enabled;
    uint32_t lora_listen_ms;
    uint32_t lora_tx_period_sec;
    bool lora_month_lock_enabled;
    bool lora_month_lock_auto_release;
    uint8_t lora_day_start;
    uint8_t lora_day_end;
    uint8_t lora_hour_start;
    uint8_t lora_hour_end;

    bool save_on_sleep;
} wm_feature_cfg_t;

/* defined in main.c */
extern wm_feature_cfg_t s_features;

/* shared month-lock state, defined once in main.c */
extern bool s_ble_month_locked;
extern bool s_lora_month_locked;
extern uint16_t s_ble_lock_year;
extern uint8_t s_ble_lock_month;
extern uint16_t s_lora_lock_year;
extern uint8_t s_lora_lock_month;

/* shared auth time */
extern int64_t auth_start_us;
extern uint8_t first_mag_tamper;

/* Tamper / time / BLE helpers */
const tamper_history_t *meter_helper_get_tamper_history(void);
void meter_helper_set_tamper_history(const tamper_history_t *history);

bool meter_helper_get_case_tamper(void);
bool meter_helper_get_magnetic_tamper(void);
bool meter_helper_get_removal_tamper(void);

void meter_helper_set_case_tamper(bool v);
void meter_helper_set_magnetic_tamper(bool v);
void meter_helper_set_removal_tamper(bool v);
void meter_helper_clear_all_tampers(void);

void meter_helper_lock_ble_until_next_month(uint16_t year, uint8_t month);
bool meter_helper_ble_month_locked(void);
void meter_helper_unlock_ble_month(void);
bool meter_helper_should_unlock_ble(uint16_t current_year, uint8_t current_month);

bool auth_session_active(int64_t now_us);

bool get_local_time(time_t *epoch_out, struct tm *tm_out);
void initialize_rtc_time(void);

bool is_ble_window_allowed(const struct tm *t);
bool is_ble_adv_slot_now(const struct tm *t);

time_t compute_next_ble_epoch(struct tm *now_tm, bool ble_month_locked);
uint64_t compute_sleep_us_after_now(time_t now_epoch);

void publish_current_state(void);
void ble_command_handler(const char *cmd);
void start_ble_if_needed(void);
void stop_ble_if_needed(void);
void meter_helper_poll_ble_tx(void);
void check_and_store_monthly_history(time_t epoch, const struct tm *tm_now);
uint32_t meter_total(uint32_t rtc_blocks, uint32_t pulse_bucket, uint32_t block_size);
void ble_flush_buffer(uint32_t send_gap_ms);
void save_rtc_to_eeprom(
    uint32_t rtc_blocks,
    uint32_t pulse_bucket,
    bool tamper_case,
    bool tamper_magnetic,
    bool tamper_removal,
    int64_t last_epoch);

esp_err_t meter_helper_flush_runtime_save(bool force);
bool meter_helper_runtime_save_pending(void);
void ble_lock_month(uint16_t year, uint8_t month);
void ble_unlock_month(void);

void lora_lock_month(uint16_t year, uint8_t month);
void lora_unlock_month(void);
void request_sleep_safe(
    int gpio1,
    int gpio2,
    bool *ble_started,
    uint32_t rtc_blocks,
    uint32_t pulse_bucket,
    bool tamper_flag,
    uint32_t stable_ms,
    bool save_state);

void request_sleep_stuck(
    int gpio1,
    int gpio2,
    int stuck_gpio,
    bool *ble_started,
    uint32_t rtc_blocks,
    uint32_t pulse_bucket,
    bool tamper_flag,
    bool save_state);

#endif