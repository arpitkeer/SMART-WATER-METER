#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_sleep.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "cyclic_counter.h"
#include "deep_sleep_manager.h"
#include "lwip/sys.h"
#include "soc/gpio_num.h"
#include "tamper.h"
#include "nimble_adv.h"
#include "ble_buffer.h"
#include "meter_storage.h"
#include "meter_helper.h"
#include "watchdog.h"
#include "lora_manager.h"
#include "sdkconfig.h"
#include "adc_monitor.h"
#include "esp_ota_ops.h"
/* =========================================================
 * PIN / BUILD CONFIG
 * ========================================================= */
#define GPIO1       CONFIG_WM_SENSOR1_GPIO
#define GPIO2       CONFIG_WM_SENSOR2_GPIO
#define TAMPER_GPIO CONFIG_WM_CASE_OPEN_TAMPER_GPIO

#ifndef BLOCK_SIZE
#define BLOCK_SIZE CONFIG_WM_CYCLIC_RESET_COUNT
#endif

#define WATCHDOG_TIMER_HANG_TIME 15
#define DEBOUNCE_MS              150
#define BLE_SEND_GAP_MS          50
#define CASE_TAMPER_COOL_DOWN    60

#ifndef INACTIVITY_TIMEOUT_US
#define INACTIVITY_TIMEOUT_US (5ULL * 1000000ULL) //5SECONDS
#endif

#ifndef AUTH_WINDOW_US
#define AUTH_WINDOW_US (5ULL * 60ULL * 1000000ULL)
#endif

#ifndef UNAUTH_TIMEOUT_US
#define UNAUTH_TIMEOUT_US (5ULL * 1000000ULL) //5 SECONDS
#endif

#define LORA_TX_PERIOD_SEC 120U
#define LORA_RX_LISTEN_PERIOD_MS 15000

/* =========================================================
 * LORA MANAGER CONFIG
 * ========================================================= */
void lora_manager_set_config(const lora_manager_config_t *cfg);

/* =========================================================
 * FEATURE FLAGS / PROVISIONING
 * ========================================================= */
wm_feature_cfg_t s_features = {
    .ble_enabled = true,
    .ble_mag_window_enabled = true,
    .ble_auth_window_enabled = true,
    .ble_month_lock_enabled = true,
    .ble_month_lock_auto_release = true,
    .ble_mag_window_sec = 3 * 60,
    .ble_auth_window_sec = 5 * 60,

    .board_has_lora = true,
    .lora_enabled = true,
    .lora_uplink_window_enabled = true,
    .lora_downlink_window_enabled = true,
    .lora_listen_enabled = true,
    .lora_listen_ms = LORA_RX_LISTEN_PERIOD_MS,
    .lora_tx_period_sec = LORA_TX_PERIOD_SEC,
    .lora_month_lock_enabled = true,
    .lora_month_lock_auto_release = true,
    .lora_day_start = 1,
    .lora_day_end = 31,
    .lora_hour_start = 0,
    .lora_hour_end = 23,

    .save_on_sleep = false
};

/* =========================================================
 * FORWARD DECLARATIONS
 * ========================================================= */
bool meter_helper_ble_month_locked(void);
bool meter_helper_should_unlock_ble(uint16_t current_year, uint8_t current_month);
void meter_helper_unlock_ble_month(void);
bool auth_session_active(int64_t now_us);
void start_ble_if_needed(void);
void stop_ble_if_needed(void);
void nimble_adv_deinit(void);
void nimble_adv_disconnect(void);
int64_t nimble_adv_get_connect_time_us(void);
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
uint64_t compute_sleep_us_after_now(time_t now_epoch);
time_t compute_next_ble_epoch(struct tm *now_tm, bool ble_month_locked);
void publish_current_state(void);
void ble_command_handler(const char *cmd);
esp_err_t lora_manager_send_meter_payload(void);
esp_err_t meter_helper_flush_runtime_save(bool force);
bool meter_helper_runtime_save_pending(void);

/* =========================================================
 * RTC DATA
 * ========================================================= */
RTC_DATA_ATTR uint32_t pulse_bucket = 0;
RTC_DATA_ATTR uint32_t rtc_blocks = 0;
RTC_DATA_ATTR bool ble_started = false;
RTC_DATA_ATTR bool rtc_loaded_from_eeprom = false;
RTC_DATA_ATTR int last_saved_month = -1;
RTC_DATA_ATTR int last_saved_year = -1;
RTC_DATA_ATTR bool s_time_set_once = false;

/* BLE windows */
RTC_DATA_ATTR bool mag_ble_event_latched = false;
RTC_DATA_ATTR bool mag_ble_window_open = false;
RTC_DATA_ATTR int64_t mag_ble_window_start_us = 0;

/* LoRa scheduling */
RTC_DATA_ATTR bool s_lora_sent_this_boot = false;
RTC_DATA_ATTR bool s_mag_ble_enabled = true;

/* BLE month lock */
RTC_DATA_ATTR bool s_ble_month_locked = false;
RTC_DATA_ATTR uint16_t s_ble_lock_year = 0;
RTC_DATA_ATTR uint8_t s_ble_lock_month = 0;

/* LoRa month lock */
RTC_DATA_ATTR bool s_lora_month_locked = false;
RTC_DATA_ATTR uint16_t s_lora_lock_year = 0;
RTC_DATA_ATTR uint8_t s_lora_lock_month = 0;

RTC_DATA_ATTR time_t s_next_lora_epoch = 0;

/* =========================================================
 * GLOBAL STATE
 * ========================================================= */
static RTC_DATA_ATTR uint32_t last_count;
bool ble_inited = false;
bool last_tamper_state = false;
int64_t last_activity = 0;
static bool prev_authorized = false;
static uint32_t blocks_since_save = 0;
bool cfg_command = false;
/* =========================================================
 * STATE MACHINE
 * ========================================================= */
typedef enum
{
    WM_STATE_BOOT = 0,
    WM_STATE_RUN,
    WM_STATE_BLE_ADV,
    WM_STATE_BLE_CONNECTED,
    WM_STATE_LORA_WINDOW,
    WM_STATE_PREPARE_SLEEP,
    WM_STATE_DEEP_SLEEP,
    WM_STATE_RESTART
} wm_state_t;

static const char *wm_state_name(wm_state_t s)
{
    switch (s)
    {
        case WM_STATE_BOOT:          return "BOOT";
        case WM_STATE_RUN:           return "RUN";
        case WM_STATE_BLE_ADV:       return "BLE_ADV";
        case WM_STATE_BLE_CONNECTED: return "BLE_CONNECTED";
        case WM_STATE_LORA_WINDOW:   return "LORA_WINDOW";
        case WM_STATE_PREPARE_SLEEP: return "PREPARE_SLEEP";
        case WM_STATE_DEEP_SLEEP:    return "DEEP_SLEEP";
        case WM_STATE_RESTART:       return "RESTART";
        default:                     return "UNKNOWN";
    }
}

typedef struct
{
    int64_t now_us;
    time_t now_epoch;
    struct tm now_tm;
    bool time_valid;

    bool ble_allowed_now;
    bool ble_month_locked;
    bool lora_month_locked;
    bool authorized_now;
    bool auth_active;

    bool ble_should_run;
    bool lora_should_run;
    bool lora_tx_pending;
    bool request_sleep;
    uint64_t sleep_us;
    uint32_t count_now;
} wm_cycle_ctx_t;

static wm_state_t s_state = WM_STATE_BOOT;
static wm_cycle_ctx_t s_ctx;

/* FIX-7: remember which state to return to after a LoRa window
 * fired mid-BLE so we restore BLE_ADV instead of always going to RUN. */
static wm_state_t s_lora_return_state = WM_STATE_RUN;

static void wm_set_state(wm_state_t next)
{
    if (s_state != next)
    {
        printf("\n[STATE]\n[STATE] %s -> %s\n\n",
               wm_state_name(s_state),
               wm_state_name(next));
    }
    s_state = next;
}

/* =========================================================
 * TIME / MONTH HELPERS
 * ========================================================= */
static void wm_init_time_base(void)
{
    initialize_rtc_time();
}

static bool month_changed(uint16_t y1, uint8_t m1, uint16_t y2, uint8_t m2)
{
    return (y1 != y2) || (m1 != m2);
}

void ble_lock_month(uint16_t year, uint8_t month)
{
    s_ble_month_locked = true;
    s_ble_lock_year    = year;
    s_ble_lock_month   = month;
}

void ble_unlock_month(void)
{
    s_ble_month_locked = false;
    s_ble_lock_year    = 0;
    s_ble_lock_month   = 0;
}

void lora_lock_month(uint16_t year, uint8_t month)
{
    s_lora_month_locked = true;
    s_lora_lock_year    = year;
    s_lora_lock_month   = month;
}

void lora_unlock_month(void)
{
    s_lora_month_locked = false;
    s_lora_lock_year    = 0;
    s_lora_lock_month   = 0;
}

/* =========================================================
 * LORA WINDOW PREDICATE
 * ========================================================= */
static bool lora_window_now(const struct tm *t)
{
    if (!t) return false;

    if (!s_features.board_has_lora    ||
        !s_features.lora_enabled      ||
        !s_features.lora_uplink_window_enabled)
    {
        return false;
    }

    if (s_features.lora_month_lock_enabled && s_lora_month_locked)
        return false;

    if (t->tm_mday < s_features.lora_day_start ||
        t->tm_mday > s_features.lora_day_end)
        return false;

    if (t->tm_hour < s_features.lora_hour_start ||
        t->tm_hour >= s_features.lora_hour_end)
        return false;

    return true;
}

/* =========================================================
 * NEXT LORA EPOCH  (absolute grid-snap, no drift)
 * ========================================================= */
static time_t compute_next_lora_epoch(struct tm *now_tm)
{
    if (!now_tm) return time(NULL) + 1;

    struct tm next = *now_tm;
    time_t now = mktime(now_tm);

    /* Month lock: skip to start of next month's window */
    if (s_features.lora_month_lock_enabled && s_lora_month_locked)
    {
        next.tm_mon += 1;
        next.tm_mday = s_features.lora_day_start;
        next.tm_hour = s_features.lora_hour_start;
        next.tm_min  = 0;
        next.tm_sec  = 0;
        next.tm_isdst = -1;
        time_t target = mktime(&next);
        return (target <= now) ? (now + 24 * 3600) : target;
    }

    /* Before allowed day range */
    if (next.tm_mday < s_features.lora_day_start)
    {
        next.tm_mday  = s_features.lora_day_start;
        next.tm_hour  = s_features.lora_hour_start;
        next.tm_min   = 0;
        next.tm_sec   = 0;
        next.tm_isdst = -1;
        return mktime(&next);
    }

    /* Past allowed day range: roll to next month */
    if (next.tm_mday > s_features.lora_day_end)
    {
        next.tm_mon  += 1;
        next.tm_mday  = s_features.lora_day_start;
        next.tm_hour  = s_features.lora_hour_start;
        next.tm_min   = 0;
        next.tm_sec   = 0;
        next.tm_isdst = -1;
        return mktime(&next);
    }

    /* Before daily window start */
    if (next.tm_hour < s_features.lora_hour_start)
    {
        next.tm_hour  = s_features.lora_hour_start;
        next.tm_min   = 0;
        next.tm_sec   = 0;
        next.tm_isdst = -1;
        return mktime(&next);
    }

    /* Past daily window end: advance to next day's window */
    if (next.tm_hour >= s_features.lora_hour_end)
    {
        next.tm_mday += 1;
        next.tm_hour  = s_features.lora_hour_start;
        next.tm_min   = 0;
        next.tm_sec   = 0;
        next.tm_isdst = -1;
        return mktime(&next);
    }

    /* -------------------------------------------------------
     * INSIDE the allowed window: snap to the next grid line.
     *
     * Formula:  next_grid = floor(now / period) * period + period
     *
     * This is purely arithmetic — it never drifts regardless
     * of how late the device wakes up or how long TX takes.
     * ------------------------------------------------------- */
    uint32_t period = s_features.lora_tx_period_sec;
    if (period == 0) period = 120;

    return (now - (now % period)) + period;
}

/* =========================================================
 * SLEEP DURATION CALCULATION
 * ========================================================= */
static uint64_t wm_compute_next_sleep_us(void)
{
    uint64_t next_ble_us  = UINT64_MAX;
    uint64_t next_lora_us = UINT64_MAX;

    time_t next_ble_epoch  = compute_next_ble_epoch(&s_ctx.now_tm, s_ble_month_locked);
    time_t next_lora_epoch = compute_next_lora_epoch(&s_ctx.now_tm);

    if (next_ble_epoch > s_ctx.now_epoch)
        next_ble_us  = (uint64_t)(next_ble_epoch  - s_ctx.now_epoch) * 1000000ULL;

    if (next_lora_epoch > s_ctx.now_epoch)
        next_lora_us = (uint64_t)(next_lora_epoch - s_ctx.now_epoch) * 1000000ULL;

    return (next_ble_us < next_lora_us) ? next_ble_us : next_lora_us;
}

/* =========================================================
 * BOOT HELPERS
 * ========================================================= */
static void wm_load_eeprom_once(void)
{
    meter_storage_record_t nv;
    memset(&nv, 0, sizeof(nv));

    if (meter_storage_get_copy(&nv) == ESP_OK)
    {
        rtc_blocks   = nv.rtc_blocks;
        pulse_bucket = nv.pulse_bucket;

        meter_helper_set_case_tamper(nv.tamper_case != 0);
        meter_helper_set_magnetic_tamper(nv.tamper_magnetic != 0);
        meter_helper_set_removal_tamper(nv.tamper_removal != 0);
        meter_helper_set_tamper_history(&nv.tamper_history);

        printf(
            "\n[BOOT]\n"
            "[BOOT] EEPROM LOADED\n"
            "[BOOT] Meter ID : %s\n"
            "[BOOT] Blocks   : %" PRIu32 "\n"
            "[BOOT] Bucket   : %" PRIu32 "\n"
            "[BOOT] Tamper   : %d/%d/%d\n\n",
            nv.meter_id,
            rtc_blocks,
            pulse_bucket,
            meter_helper_get_case_tamper()    ? 1 : 0,
            meter_helper_get_magnetic_tamper() ? 1 : 0,
            meter_helper_get_removal_tamper()  ? 1 : 0);
    }
    else
    {
        meter_helper_clear_all_tampers();
    }

    rtc_loaded_from_eeprom = true;
}

static void wm_boot_once(void)
{
    esp_reset_reason_t rst = esp_reset_reason();

    if (rst != ESP_RST_DEEPSLEEP)
    {
        /* -------------------------------------------------------
         * Cold boot / manual reset.
         * Clear all retained scheduling state that must not
         * survive a power cycle.
         * ------------------------------------------------------- */
        s_next_lora_epoch      = 0;
        s_lora_sent_this_boot  = false;
        mag_ble_event_latched  = false;
        mag_ble_window_open    = false;
        mag_ble_window_start_us = 0;
        s_ble_month_locked     = false;
        s_ble_lock_year        = 0;
        s_ble_lock_month       = 0;
        s_lora_month_locked    = false;
        s_lora_lock_year       = 0;
        s_lora_lock_month      = 0;
        rtc_loaded_from_eeprom = false;
        auth_start_us          = 0;   /* FIX-4 */

        /* FIX-3: single authoritative init — no duplicate call */
        esp_err_t ret = meter_storage_init();
        if (ret == ESP_ERR_INVALID_CRC)
            printf("[EEPROM] Invalid CRC — using defaults\n");
        else if (ret != ESP_OK)
            printf("[EEPROM] init failed: %s\n", esp_err_to_name(ret));

        wm_init_time_base();

        if (!rtc_loaded_from_eeprom)
            wm_load_eeprom_once();

        printf("\n[BOOT]\n[BOOT] Cold boot — EEPROM loaded\n\n");
    }
    else
    {
        /* Deep-sleep resume: storage already initialised; just restore time */
        wm_init_time_base();
        printf(
            "\n[BOOT]\n"
            "[BOOT] Deep sleep resume\n"
            "[BOOT] Blocks=%" PRIu32 "\n"
            "[BOOT] Bucket=%" PRIu32 "\n\n",
            rtc_blocks,
            pulse_bucket);
    }

    ble_buffer_init();

    tamper_config_t tcfg = {
        .gpio          = TAMPER_GPIO,
        .magnetic_gpio = CONFIG_WM_MAGNETIC_TAMPER_GPIO,
        .removal_gpio  = CONFIG_WM_REMOVAL_TAMPER_GPIO,
        .stable_time_us = 300000ULL
    };
    tamper_init(&tcfg);

    if (s_features.board_has_lora && s_features.lora_enabled && !s_lora_month_locked)
    {
        lora_manager_config_t lcfg = {
            .board_has_lora  = s_features.board_has_lora,
            .uplink_enabled  = s_features.lora_uplink_window_enabled,
            .downlink_enabled = s_features.lora_downlink_window_enabled,
            .listen_enabled  = s_features.lora_listen_enabled,
            .listen_ms       = s_features.lora_listen_ms,
        };
        lora_manager_set_config(&lcfg);
        ESP_ERROR_CHECK(lora_manager_init());
    }

    if (rst != ESP_RST_DEEPSLEEP){
        last_count = cyclic_counter_get_count();
	ESP_ERROR_CHECK(adc_monitor_init());}

    last_activity  = esp_timer_get_time();
    prev_authorized = false;
    s_lora_sent_this_boot = false;

    printf("\n[SYSTEM]\n[SYSTEM] INIT COMPLETE\n\n");
}

/* =========================================================
 * PER-CYCLE: SAMPLE TIME + MONTH LOCKS
 * ========================================================= */
static void wm_sample_time_and_month(void)
{
    s_ctx.now_us = esp_timer_get_time();

    memset(&s_ctx.now_tm, 0, sizeof(s_ctx.now_tm));
    s_ctx.time_valid = get_local_time(&s_ctx.now_epoch, &s_ctx.now_tm);

    if (!s_ctx.time_valid)
    {
        printf("\n[TIME]\n[TIME] RTC invalid, waiting\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    check_and_store_monthly_history(s_ctx.now_epoch, &s_ctx.now_tm);

    s_ctx.ble_month_locked  = s_ble_month_locked;
    s_ctx.lora_month_locked = s_lora_month_locked;

    /* Auto-release BLE monthly lock on new calendar month */
    if (s_features.ble_month_lock_enabled   &&
        s_features.ble_month_lock_auto_release &&
        s_ble_month_locked &&
        month_changed(s_ble_lock_year, s_ble_lock_month,
                      (uint16_t)(s_ctx.now_tm.tm_year + 1900),
                      (uint8_t) (s_ctx.now_tm.tm_mon  + 1)))
    {
        ble_unlock_month();
        s_ctx.ble_month_locked = false;
        printf("\n[BLE]\n[BLE] Monthly lock cleared\n\n");
    }

    /* Auto-release LoRa monthly lock on new calendar month */
    if (s_features.lora_month_lock_enabled   &&
        s_features.lora_month_lock_auto_release &&
        s_lora_month_locked &&
        month_changed(s_lora_lock_year, s_lora_lock_month,
                      (uint16_t)(s_ctx.now_tm.tm_year + 1900),
                      (uint8_t) (s_ctx.now_tm.tm_mon  + 1)))
    {
        lora_unlock_month();
        s_ctx.lora_month_locked = false;
        printf("\n[LORA]\n[LORA] Monthly lock cleared\n\n");
    }

    s_ctx.ble_allowed_now = is_ble_adv_slot_now(&s_ctx.now_tm);
    s_ctx.authorized_now  = nimble_adv_is_authorized();
    s_ctx.auth_active     = auth_session_active(s_ctx.now_us);
}

/* =========================================================
 * TAMPER HANDLER
 * ========================================================= */
static void wm_handle_tamper(void)
{
    tamper_update(s_ctx.now_us);

    bool current_magnetic_tamper = tamper_magnetic_is_triggered();
    bool current_case_tamper     = tamper_case_is_triggered();
    bool current_removal_tamper  = tamper_removal_is_triggered();

    if (!current_magnetic_tamper) mag_ble_event_latched = false;
    if (!current_case_tamper)     meter_helper_set_case_tamper(false);
    if (!current_removal_tamper)  meter_helper_set_removal_tamper(false);

    /* ---- CASE TAMPER ---- */
    if (current_case_tamper)
    {
        tamper_history_t hist = *meter_helper_get_tamper_history();
        bool should_save = false;

        if (!meter_helper_get_case_tamper())
        {
            meter_helper_set_case_tamper(true);

            if (hist.first_case_epoch == 0)
                hist.first_case_epoch = s_ctx.now_epoch;

            if ((s_ctx.now_epoch - hist.last_case_epoch) >= CASE_TAMPER_COOL_DOWN)
            {
                hist.last_case_epoch = s_ctx.now_epoch;
                hist.case_count++;

                save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
                    meter_helper_get_case_tamper(),
                    meter_helper_get_magnetic_tamper(),
                    meter_helper_get_removal_tamper(),
                    (int64_t)s_ctx.now_epoch);

                (void)meter_helper_flush_runtime_save(true);
                should_save = true;

                printf("\n[CASE TAMPER]\n[CASE TAMPER] OPEN DETECTED\n\n");
            }
        }
        else
        {
            if ((s_ctx.now_epoch - hist.last_case_epoch) >= CASE_TAMPER_COOL_DOWN)
            {
                hist.last_case_epoch = s_ctx.now_epoch;
                hist.case_count++;
                should_save = true;

                printf("\n[CASE TAMPER]\n[CASE TAMPER] STILL OPEN (60s)\n\n");
            }
        }

        if (should_save)
            meter_helper_set_tamper_history(&hist);

        publish_current_state();
    }

    /* ---- MAGNETIC TAMPER ---- */
    if (current_magnetic_tamper && !mag_ble_event_latched)
    {
        mag_ble_event_latched = true;

        if (!meter_helper_get_magnetic_tamper())
            meter_helper_set_magnetic_tamper(true);

        tamper_history_t hist = *meter_helper_get_tamper_history();

        if (hist.first_magnetic_epoch == 0)
            hist.first_magnetic_epoch = s_ctx.now_epoch;

        hist.last_magnetic_epoch = s_ctx.now_epoch;
        hist.magnetic_count++;
        meter_helper_set_tamper_history(&hist);

        save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
            meter_helper_get_case_tamper(),
            true,
            meter_helper_get_removal_tamper(),
            (int64_t)s_ctx.now_epoch);

        (void)meter_helper_flush_runtime_save(true);

        printf(
            "\n[MAGNETIC TAMPER]\n"
            "[MAGNETIC TAMPER] DETECTED\n"
            "[MAGNETIC TAMPER] Blocks : %" PRIu32 "\n"
            "[MAGNETIC TAMPER] Bucket : %" PRIu32 "\n\n",
            rtc_blocks, pulse_bucket);

        if (s_features.ble_mag_window_enabled)
        {
            mag_ble_window_open     = true;
            mag_ble_window_start_us = s_ctx.now_us;
            last_activity           = s_ctx.now_us;

            printf("\n[BLE]\n[BLE] Magnetic BLE window opened for %u sec\n\n",
                   (unsigned)s_features.ble_mag_window_sec);

            if (!ble_inited && s_features.ble_enabled)
            {
                nimble_adv_config_t ble_cfg = { .device_name = "WM" };
                ESP_ERROR_CHECK(nimble_adv_init(&ble_cfg));
                nimble_adv_set_cmd_cb(ble_command_handler);
                ble_inited = true;
            }

            if (s_features.ble_enabled)
                start_ble_if_needed();
        }

        publish_current_state();
    }

    /* Magnetic window expiry */
    if (mag_ble_window_open && s_features.ble_mag_window_enabled)
    {
        if (s_ctx.now_us < mag_ble_window_start_us)
            mag_ble_window_start_us = s_ctx.now_us;   /* RTC wrap guard */

        if ((s_ctx.now_us - mag_ble_window_start_us) >=
            ((int64_t)s_features.ble_mag_window_sec * 1000000LL))
        {
            mag_ble_window_open = false;
            printf("\n[BLE]\n[BLE] Magnetic BLE window expired\n\n");
            stop_ble_if_needed();
        }
    }

    /* ---- REMOVAL TAMPER ---- */
    if (current_removal_tamper)
    {
        tamper_history_t hist = *meter_helper_get_tamper_history();
        bool should_save = false;

        if (!meter_helper_get_removal_tamper())
        {
            meter_helper_set_removal_tamper(true);

            if (hist.first_removal_epoch == 0)
                hist.first_removal_epoch = s_ctx.now_epoch;

            hist.last_removal_epoch = s_ctx.now_epoch;
            hist.removal_count++;
            should_save = true;

            printf("\n[REMOVAL TAMPER]\n[REMOVAL TAMPER] FIRST DETECT\n\n");
        }
        else
        {
            if ((s_ctx.now_epoch - hist.last_removal_epoch) >= 3600)
            {
                hist.last_removal_epoch = s_ctx.now_epoch;
                hist.removal_count++;
                should_save = true;

                printf("\n[REMOVAL TAMPER]\n[REMOVAL TAMPER] STILL ACTIVE\n\n");
            }
        }

        if (should_save)
        {
            meter_helper_set_tamper_history(&hist);

            save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
                meter_helper_get_case_tamper(),
                meter_helper_get_magnetic_tamper(),
                true,
                (int64_t)s_ctx.now_epoch);

            (void)meter_helper_flush_runtime_save(true);

            printf(
                "\n[REMOVAL TAMPER]\n"
                "[REMOVAL TAMPER] SAVED\n"
                "[REMOVAL TAMPER] Count : %" PRIu32 "\n"
                "[REMOVAL TAMPER] First : %" PRIi64 "\n"
                "[REMOVAL TAMPER] Last  : %" PRIi64 "\n\n",
                hist.removal_count,
                hist.first_removal_epoch,
                hist.last_removal_epoch);
        }

        publish_current_state();
    }
}

/* =========================================================
 * PULSE HANDLER
 *
 * FIX-2: last_count is updated unconditionally FIRST so that
 * even if time is invalid (and we skip the save/log path) we
 * never let the delta accumulate invisibly across ticks.
 *
 * The ISR-driven cyclic_counter is free-running; the only job
 * of wm_handle_pulses() is to drain whatever has accumulated
 * since last_count was last sampled.  On a 50 ms BLE tick the
 * device can service multiple pulses per call — the delta math
 * already handles this correctly.
 * ========================================================= */
static void wm_handle_pulses(void)
{
    s_ctx.count_now = cyclic_counter_get_count();

    if (s_ctx.count_now == last_count)
        return;   /* nothing new */

    uint32_t delta = s_ctx.count_now - last_count;
    last_count     = s_ctx.count_now;  

    if (!s_ctx.time_valid)
    {
        pulse_bucket += delta;

        while (pulse_bucket >= BLOCK_SIZE)
        {
            rtc_blocks++;
            blocks_since_save++;
            pulse_bucket -= BLOCK_SIZE;
        }

        printf("[PULSE] Time invalid — counted delta %" PRIu32 " (no EEPROM save)\n", delta);
        return;
    }

    /* Normal path: time is valid */
    pulse_bucket  += delta;
    last_activity  = s_ctx.now_us;

    while (pulse_bucket >= BLOCK_SIZE)
    {
        rtc_blocks++;
        blocks_since_save++;
        pulse_bucket -= BLOCK_SIZE;
    }

    uint32_t total = meter_total(rtc_blocks, pulse_bucket, BLOCK_SIZE);

    printf(
        "\n[PULSE]\n"
        "Delta   : %" PRIu32 "\n"
        "Total   : %" PRIu32 "\n"
        "Blocks  : %" PRIu32 "\n"
        "Bucket  : %" PRIu32 "\n\n",
        delta, total, rtc_blocks, pulse_bucket);

    publish_current_state();

    if (blocks_since_save > 0)
    {
        save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
            meter_helper_get_case_tamper(),
            meter_helper_get_magnetic_tamper(),
            meter_helper_get_removal_tamper(),
            (int64_t)s_ctx.now_epoch);

        (void)meter_helper_flush_runtime_save(true);

        printf("[EEPROM] Block threshold reached. Saved to EEPROM.\n");
        blocks_since_save = 0;
    }
}

static void wm_update_auth_session(void)
{
    bool authorized_now = nimble_adv_is_authorized();

    if (authorized_now)
    {
        if (auth_start_us <= 0)
        {
            auth_start_us = s_ctx.now_us;
            printf("\n[BLE]\n[BLE] AUTH SESSION START\n\n");
        }

        last_activity = s_ctx.now_us;
    }
    else
    {
        auth_start_us   = 0;
        prev_authorized = false;
    }

    prev_authorized       = authorized_now;
    s_ctx.authorized_now  = authorized_now;
    s_ctx.auth_active     = auth_session_active(s_ctx.now_us);
}

static void wm_update_ble_state(void)
{
    bool mag_window_ok  = s_features.ble_mag_window_enabled  && mag_ble_window_open;
    bool auth_window_ok = s_features.ble_auth_window_enabled && s_ctx.auth_active;

    s_ctx.ble_should_run =
        s_features.ble_enabled &&
        (
            mag_window_ok  ||
            auth_window_ok ||
            (!s_ctx.ble_month_locked && s_ctx.ble_allowed_now)
        );

    if (s_ctx.ble_should_run)
    {
        if (!ble_inited)
        {
            nimble_adv_config_t ble_cfg = { .device_name = "WM" };
            ESP_ERROR_CHECK(nimble_adv_init(&ble_cfg));
            nimble_adv_set_cmd_cb(ble_command_handler);
            ble_inited = true;
        }

        start_ble_if_needed();
        ble_started   = true;
        last_activity = s_ctx.now_us;
    }
    else
    {
        stop_ble_if_needed();
        ble_started = false;
    }

    if (nimble_adv_is_connected() && !nimble_adv_is_authorized())
    {
        int64_t conn_us = nimble_adv_get_connect_time_us();

        if (conn_us > 0 &&
            (s_ctx.now_us - conn_us) >
            (int64_t)(s_features.ble_auth_window_sec * 1000000ULL))
        {
            printf("\n[BLE]\n[BLE] UNAUTHORIZED TIMEOUT\n\n");
            nimble_adv_disconnect();
        }
    }
}

static void wm_request_lora_window_if_needed(void)
{
    if (!s_features.board_has_lora            ||
        !s_features.lora_enabled              ||
        !s_features.lora_uplink_window_enabled)
    {
        s_ctx.lora_tx_pending = false;
        s_ctx.lora_should_run = false;
        return;
    }

    if (s_features.lora_month_lock_enabled && s_lora_month_locked)
    {
        s_ctx.lora_tx_pending = false;
        s_ctx.lora_should_run = false;
        return;
    }

    if (!lora_window_now(&s_ctx.now_tm))
    {
        s_ctx.lora_tx_pending = false;
        s_ctx.lora_should_run = false;
        return;
    }

    if (s_next_lora_epoch == 0)
        s_next_lora_epoch = s_ctx.now_epoch;

    s_ctx.lora_tx_pending = (s_ctx.now_epoch >= s_next_lora_epoch);
    s_ctx.lora_should_run = s_ctx.lora_tx_pending;
}

static void wm_run_lora_window(void)
{
    if (!s_features.board_has_lora  ||
        !s_features.lora_enabled    ||
        !s_ctx.lora_tx_pending)
    {
        return;
    }
	int min_mv = 0, max_mv = 0, avg_mv = 0;
	    // We call adc_monitor_sample directly (assuming your adc_monitor.c 
	    // provides this signature) to get high-resolution stats.
	    if (adc_monitor_sample(NULL, NULL, NULL, &min_mv, &max_mv, &avg_mv) == ESP_OK) {
	        printf("[LORA TX] Battery Min during TX: %d mV | Avg: %d mV\n", min_mv, avg_mv);
	        
	        // Optionally: Store this in a global or add it to your LoRa payload
	        // meter_helper_set_last_tx_battery(min_mv); 
	    }

    esp_err_t ret = lora_manager_send_meter_payload();

    if (ret != ESP_OK)
        printf("[LORA] TX FAILED: %s\n", esp_err_to_name(ret));
    else
        printf("[LORA] TX OK\n");

    uint32_t period = s_features.lora_tx_period_sec;
    if (period == 0) period = 120;

    s_next_lora_epoch =
        (s_ctx.now_epoch - (s_ctx.now_epoch % period)) + period;

    printf("[LORA] Next TX epoch: %lld\n", (long long)s_next_lora_epoch);

    s_ctx.lora_tx_pending = false;
    s_ctx.lora_should_run = false;
}

static void wm_request_sleep_if_needed(void)
{
    s_ctx.request_sleep = false;
    s_ctx.sleep_us      = 0;

    if (s_ctx.lora_tx_pending)
        return;

    if (nimble_adv_is_connected())
        return;

    if (!mag_ble_window_open &&
        !auth_session_active(s_ctx.now_us) &&
        ((s_ctx.now_us - last_activity) > INACTIVITY_TIMEOUT_US))
    {
        s_ctx.request_sleep = true;

        printf("\n[SLEEP]\n[SLEEP] Inactivity timeout — entering deep sleep\n\n");

        if (s_ctx.ble_should_run)
            stop_ble_if_needed();

        s_ctx.sleep_us = wm_compute_next_sleep_us();

        if (s_ctx.sleep_us == 0)
            s_ctx.sleep_us = 86400ULL * 1000000ULL;

        printf(
            "\n[SLEEP DEBUG]\n"
            "BLE LOCKED      = %d\n"
            "BLE LOCK MONTH  = %u\n"
            "NOW HOUR        = %d\n"
            "LORA WINDOW NOW = %d\n"
            "NEXT LORA EPOCH = %lld\n"
            "SLEEP_US        = %llu\n\n",
            s_ble_month_locked,
            s_ble_lock_month,
            s_ctx.now_tm.tm_hour,
            lora_window_now(&s_ctx.now_tm),
            (long long)s_next_lora_epoch,
            (unsigned long long)s_ctx.sleep_us);

        printf("\n[SLEEP]\n[SLEEP] Sleeping for %" PRIu64 " us\n\n",
               s_ctx.sleep_us);
    }
}

static void wm_prepare_sleep(void)
{
    if (s_features.save_on_sleep && meter_helper_runtime_save_pending())
        (void)meter_helper_flush_runtime_save(true);
    else if (meter_helper_runtime_save_pending())
        (void)meter_helper_flush_runtime_save(false);

    if (ble_inited)
    {
        stop_ble_if_needed();
        ble_started = false;
    }

    if (s_features.board_has_lora && s_features.lora_enabled)
        lora_manager_deinit();

    if (s_ctx.sleep_us == 0)
    {
        s_ctx.sleep_us = wm_compute_next_sleep_us();

        if (s_ctx.sleep_us == 0)
            s_ctx.sleep_us = 86400ULL * 1000000ULL;
    }

    wm_set_state(WM_STATE_DEEP_SLEEP);
}

static void wm_enter_sleep(void)
{
    deep_sleep_enter_for_us(s_ctx.sleep_us, true);
}

static bool wm_should_poll_hardware(void)
{
    switch (s_state)
    {
        case WM_STATE_BOOT:
        case WM_STATE_DEEP_SLEEP:
        case WM_STATE_RESTART:
        case WM_STATE_PREPARE_SLEEP:  
        case WM_STATE_LORA_WINDOW:    
            return false;
        default:
            return true;
    }
}
void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    printf("\n[WAKEUP]\n[WAKEUP] Cause = %d\n\n", cause);
    printf("\n[RESET]\nReset reason=%d\n", esp_reset_reason());
	// If you don't call this, the ESP32 will roll back to the old version on the next reboot.
	// 1. Properly handle OTA Validation first
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t ota_state;

	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
	    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
	        printf("New update detected. Validating image...\n");
	        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
	            printf("OTA Update Verified and Locked In!\n");
	        } else {
	            printf("Error: Failed to mark app as valid!\n");
	        }
	    } else {
	        printf("OTA State: %d (No pending verify required)\n", ota_state);
	    }
	} else {
	    printf("Warning: Could not get OTA state. Partition might be factory default.\n");
	}


	if (cause == ESP_SLEEP_WAKEUP_EXT1) {
	    uint64_t wake_mask = esp_sleep_get_ext1_wakeup_status();
	    printf("\n[WAKE MASK] 0x%llX\n", wake_mask);
	    
	    // Only process wakeup if we aren't in the middle of a critical OTA verify
	    cyclic_counter_process_wakeup(GPIO1, GPIO2);
	}

    cyclic_counter_process_wakeup(GPIO1, GPIO2);

    if (cause == ESP_SLEEP_WAKEUP_EXT1)
    {
        uint64_t wake_mask = esp_sleep_get_ext1_wakeup_status();
        printf("\n[WAKE MASK]\n0x%llX\n\n", wake_mask);
    }

    deep_sleep_handle_wakeup(GPIO1, GPIO2);

    cyclic_counter_config_t cc_cfg = {
        .gpio1       = GPIO1,
        .gpio2       = GPIO2,
        .debounce_ms = 50,
    };
    ESP_ERROR_CHECK(cyclic_counter_init(&cc_cfg));
	ESP_ERROR_CHECK(meter_storage_init());
    while (1)
    {
        if (wm_should_poll_hardware())
        {
            wm_sample_time_and_month();

            if (s_ctx.time_valid)
                wm_handle_tamper();

            wm_handle_pulses();
        }
        switch (s_state)
        {
            case WM_STATE_BOOT:
                wm_boot_once();
                last_tamper_state = meter_helper_get_case_tamper();
                wm_set_state(WM_STATE_RUN);
                break;

            case WM_STATE_RUN:
                if (s_ctx.time_valid)
                {
                    wm_update_auth_session();
                    wm_update_ble_state();
                    wm_request_lora_window_if_needed();
                    wm_request_sleep_if_needed();

                    if (s_ctx.request_sleep)
                    {
                        wm_set_state(WM_STATE_PREPARE_SLEEP);
                    }
                    else if (s_ctx.lora_tx_pending)
                    {
                        s_lora_return_state = WM_STATE_RUN;
                        wm_set_state(WM_STATE_LORA_WINDOW);
                    }
                    else if (s_ctx.ble_should_run)
                    {
                        wm_set_state(WM_STATE_BLE_ADV);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case WM_STATE_BLE_ADV:
                if (s_ctx.time_valid)
                {
                    wm_update_auth_session();
                    wm_update_ble_state();
                    wm_request_lora_window_if_needed();
                    wm_request_sleep_if_needed();

                    if (nimble_adv_is_connected()) 
                    {
                        wm_set_state(WM_STATE_BLE_CONNECTED);
                    }
                    /* 2. Standard Timeouts */
                    else if (s_ctx.request_sleep)
                    {
                        wm_set_state(WM_STATE_PREPARE_SLEEP);
                    }
                    /* 3. LoRa Interruptions */
                    else if (s_ctx.lora_tx_pending)
                    {
                        s_lora_return_state = WM_STATE_BLE_ADV; 
                        wm_set_state(WM_STATE_LORA_WINDOW);
                    }
                    /* 4. Natural Window Expiration */
                    else if (!s_ctx.ble_should_run) 
                    {
                        wm_set_state(WM_STATE_RUN);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case WM_STATE_BLE_CONNECTED:

                if (s_ctx.time_valid)
                {
                    wm_update_auth_session();
                    wm_update_ble_state();

                    wm_request_lora_window_if_needed();
                    if (s_ctx.lora_tx_pending)
                    {
                        printf("\n[LORA]\n[LORA] Firing inline during BLE session\n\n");
                        wm_run_lora_window();
                    }

                    meter_helper_poll_ble_tx();

                    wm_request_sleep_if_needed();

                    if (!nimble_adv_is_connected())
                    {
                        wm_set_state(WM_STATE_RUN);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(50));   /* 50 ms for live responsiveness */
                break;

            case WM_STATE_LORA_WINDOW:
                wm_run_lora_window();
                wm_set_state(s_lora_return_state);
                break;

            case WM_STATE_PREPARE_SLEEP:
                wm_prepare_sleep();
                break;

            case WM_STATE_DEEP_SLEEP:
                wm_enter_sleep();
                break;

            case WM_STATE_RESTART:
                printf("\n[SYSTEM]\n[SYSTEM] Executing Software Restart...\n\n");
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
                break;

            default:
                printf("\n[STATE] Invalid state — forcing reboot\n\n");
                wm_set_state(WM_STATE_RESTART);
                break;
        }
    }
}