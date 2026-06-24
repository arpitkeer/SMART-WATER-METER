#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/meter_helper.h"
#include "driver/gpio.h"

#include "host/ble_gap.h"
#include "lwip/sys.h"
#include "meter_storage.h"
#include "../nimble_adv/include/nimble_adv.h"
#include "ble_buffer.h"
#include "deep_sleep_manager.h"
#include "../meter_storage/include/meter_storage.h"

#include "../nimble_adv/include/nimble_adv.h"
#include "time.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "sdkconfig.h"
#include "../tamper/include/tamper.h"
#include "sys/time.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
/* =========================================================
 * DEFAULTS
 * ========================================================= */

#ifndef BLOCK_SIZE
#define BLOCK_SIZE CONFIG_WM_CYCLIC_RESET_COUNT
#endif

#ifndef BLE_SEND_GAP_MS
#define BLE_SEND_GAP_MS 50
#endif

#define BLE_ADV_ALLOWED_FOR_SEC 6
/* =========================================================
 * EXTERNAL STATE
 * ========================================================= */

extern uint32_t rtc_blocks;
extern uint32_t pulse_bucket;
extern bool ble_started;
extern bool last_tamper_state;
extern int64_t last_activity;
extern bool cfg_command;
extern int last_saved_month;
extern int last_saved_year;
extern bool s_ble_month_locked;
extern bool s_lora_month_locked;
extern uint32_t rtc_blocks;
extern uint32_t pulse_bucket;
RTC_DATA_ATTR uint8_t first_mag_tamper = 0;
int64_t auth_start_us = 0;
extern bool g_force_run_state;
extern int64_t last_ble_activity;
/* =========================================================
 * INTERNAL RTC-PERSISTENT TAMPER STATE
 * ========================================================= */
 extern wm_feature_cfg_t s_features;
 extern bool s_ble_month_locked;
 extern bool s_lora_month_locked;
 extern uint16_t s_ble_lock_year;
 extern uint8_t s_ble_lock_month;
 extern uint16_t s_lora_lock_year;
 extern uint8_t s_lora_lock_month;
RTC_DATA_ATTR static bool tamper_case_flag = false;
RTC_DATA_ATTR static bool tamper_magnetic_flag = false;
RTC_DATA_ATTR static bool tamper_removal_flag = false;
RTC_DATA_ATTR static tamper_history_t s_tamper_history = {0};

#include "freertos/portmacro.h"

typedef struct
{
    uint32_t rtc_blocks;
    uint32_t pulse_bucket;
    bool tamper_case;
    bool tamper_magnetic;
    bool tamper_removal;
    int64_t last_epoch;
} runtime_save_t;

static runtime_save_t s_pending_runtime_save = {0};
static bool s_runtime_save_pending = false;
static bool s_runtime_save_in_progress = false;
static int64_t s_last_runtime_save_req_us = 0;

static portMUX_TYPE s_runtime_save_mux = portMUX_INITIALIZER_UNLOCKED;

#define RUNTIME_SAVE_DEBOUNCE_US    (3000000ULL)   /* 3 seconds */

static volatile wm_cmd_source_t s_cmd_source = WM_CMD_SRC_BLE;

void ble_command_handler_set_source(wm_cmd_source_t src)
{
    s_cmd_source = src;
}

wm_cmd_source_t ble_command_handler_get_source(void)
{
    return s_cmd_source;
}


static void stage_runtime_save(
    uint32_t rtc_blocks,
    uint32_t pulse_bucket,
    bool tamper_case,
    bool tamper_magnetic,
    bool tamper_removal,
    int64_t last_epoch)
{
    portENTER_CRITICAL(&s_runtime_save_mux);

    s_pending_runtime_save.rtc_blocks = rtc_blocks;
    s_pending_runtime_save.pulse_bucket = pulse_bucket;
    s_pending_runtime_save.tamper_case = tamper_case;
    s_pending_runtime_save.tamper_magnetic = tamper_magnetic;
    s_pending_runtime_save.tamper_removal = tamper_removal;
    s_pending_runtime_save.last_epoch = last_epoch;

    s_runtime_save_pending = true;
    s_last_runtime_save_req_us = esp_timer_get_time();

    portEXIT_CRITICAL(&s_runtime_save_mux);
}

bool meter_helper_runtime_save_pending(void)
{
    bool pending;

    portENTER_CRITICAL(&s_runtime_save_mux);
    pending = s_runtime_save_pending;
    portEXIT_CRITICAL(&s_runtime_save_mux);

    return pending;
}

esp_err_t meter_helper_flush_runtime_save(bool force)
{
    runtime_save_t snap;
    bool do_flush = false;
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_runtime_save_mux);

    if (s_runtime_save_pending && !s_runtime_save_in_progress)
    {
        if (force ||
            s_last_runtime_save_req_us == 0 ||
            (now_us - s_last_runtime_save_req_us) >= (int64_t)RUNTIME_SAVE_DEBOUNCE_US)
        {
            snap = s_pending_runtime_save;
            s_runtime_save_in_progress = true;
            do_flush = true;
        }
    }

    portEXIT_CRITICAL(&s_runtime_save_mux);

    if (!do_flush)
    {
        return ESP_OK;
    }

    esp_err_t ret = meter_storage_set_runtime_state(
        snap.rtc_blocks,
        snap.pulse_bucket,
        snap.tamper_case,
        snap.tamper_magnetic,
        snap.tamper_removal,
        snap.last_epoch);

    portENTER_CRITICAL(&s_runtime_save_mux);

    s_runtime_save_in_progress = false;

    if (ret == ESP_OK)
    {
        s_runtime_save_pending = false;
    }
    else
    {
        s_last_runtime_save_req_us = esp_timer_get_time();
    }

    portEXIT_CRITICAL(&s_runtime_save_mux);

    return ret;
}


const tamper_history_t *meter_helper_get_tamper_history(void)
{
    return &s_tamper_history;
}

void meter_helper_set_tamper_history(const tamper_history_t *history)
{
    if (!history) return;
    s_tamper_history = *history;
}

void meter_helper_lock_ble_until_next_month(uint16_t year, uint8_t month)
{
    s_ble_month_locked = true;
    s_ble_lock_year = year;
    s_ble_lock_month = month;
}

bool meter_helper_ble_month_locked(void)
{
    return s_ble_month_locked;
}

void meter_helper_unlock_ble_month(void)
{
    s_ble_month_locked = false;
    s_ble_lock_year = 0;
    s_ble_lock_month = 0;
}

bool meter_helper_should_unlock_ble(uint16_t current_year, uint8_t current_month)
{
    return s_ble_month_locked &&
           (current_year != s_ble_lock_year || current_month != s_ble_lock_month);
}

void meter_helper_poll_ble_tx(void)
{
    if (!nimble_adv_is_connected() || !nimble_adv_is_authorized()) return;
    ble_flush_buffer(BLE_SEND_GAP_MS);
}

time_t compute_next_ble_epoch(struct tm *now_tm, bool ble_month_locked)
{
    if (!now_tm) return time(NULL) + 1;

    struct tm next = *now_tm;
    time_t now = mktime(now_tm);

    if (ble_month_locked)
    {
        next.tm_mon  += 1;
        next.tm_mday  = CONFIG_WM_SCHEDULE_DAY_START;
        next.tm_hour  = CONFIG_WM_DAYTIME_START_HOUR;
        next.tm_min   = 0;
        next.tm_sec   = 0;
        next.tm_isdst = -1;
        time_t target = mktime(&next);
        if (target <= now) target = now + (24 * 60 * 60);
        return target;
    }

    if (next.tm_mday < CONFIG_WM_SCHEDULE_DAY_START)
    {
        next.tm_mday  = CONFIG_WM_SCHEDULE_DAY_START;
        next.tm_hour  = CONFIG_WM_DAYTIME_START_HOUR;
        next.tm_min   = 0; next.tm_sec = 0; next.tm_isdst = -1;
        return mktime(&next);
    }

    if (next.tm_mday > CONFIG_WM_SCHEDULE_DAY_END)
    {
        next.tm_mon  += 1;
        next.tm_mday  = CONFIG_WM_SCHEDULE_DAY_START;
        next.tm_hour  = CONFIG_WM_DAYTIME_START_HOUR;
        next.tm_min   = 0; next.tm_sec = 0; next.tm_isdst = -1;
        return mktime(&next);
    }

    if (next.tm_hour < CONFIG_WM_DAYTIME_START_HOUR)
    {
        next.tm_hour  = CONFIG_WM_DAYTIME_START_HOUR;
        next.tm_min   = 0; next.tm_sec = 0; next.tm_isdst = -1;
        return mktime(&next);
    }

    if (next.tm_hour >= CONFIG_WM_DAYTIME_END_HOUR)
    {
        next.tm_mday += 1;
        next.tm_hour  = CONFIG_WM_DAYTIME_START_HOUR;
        next.tm_min   = 0; next.tm_sec = 0; next.tm_isdst = -1;
        return mktime(&next);
    }

    int interval = CONFIG_WM_PERIODIC_INTERVAL_MIN;
    if (interval <= 0) interval = 5;

    int next_min = ((next.tm_min / interval) + 1) * interval;
    next.tm_min   = next_min;
    next.tm_sec   = 0;
    next.tm_isdst = -1;

    time_t target = mktime(&next);
    if (target <= now) target = now + 1;
    return target;
}

/* =========================================================
 * TAMPER ACCESSORS
 * ========================================================= */

bool meter_helper_get_case_tamper(void)    { return tamper_case_flag; }
bool meter_helper_get_magnetic_tamper(void){ return tamper_magnetic_flag; }
bool meter_helper_get_removal_tamper(void) { return tamper_removal_flag; }
void meter_helper_set_case_tamper(bool v)    { tamper_case_flag = v; }
void meter_helper_set_magnetic_tamper(bool v){ tamper_magnetic_flag = v; }
void meter_helper_set_removal_tamper(bool v) { tamper_removal_flag = v; }

void meter_helper_clear_all_tampers(void)
{
    tamper_case_flag = false;
    tamper_magnetic_flag = false;
    tamper_removal_flag = false;
    memset(&s_tamper_history, 0, sizeof(s_tamper_history));
    save_rtc_to_eeprom(rtc_blocks, pulse_bucket, false, false, false, (int64_t)time(NULL));
}

/* =========================================================
 * EMITTERS
 *
 * ble_emit_line / ble_emit_control_line — BLE or LoRa verbose
 * lora_emit — LoRa only, hard-capped at 50 bytes, compact format
 * ========================================================= */

/* Add near the top of meter_helper.c */
static char s_last_ble_msg[128];
static int64_t s_last_ble_msg_us = 0;
#define BLE_DUP_SUPPRESS_US 1000000LL   /* 1 second */

/* Helper: true if this BLE message should be sent */
static bool ble_should_send_msg(const char *msg)
{
    if (!msg || msg[0] == '\0') {
        return false;
    }

    int64_t now_us = esp_timer_get_time();

    if (strcmp(msg, s_last_ble_msg) == 0 &&
        (now_us - s_last_ble_msg_us) < BLE_DUP_SUPPRESS_US)
    {
        return false;
    }

    strlcpy(s_last_ble_msg, msg, sizeof(s_last_ble_msg));
    s_last_ble_msg_us = now_us;
    return true;
}

/* Replace ble_emit_line() with this */
static void ble_emit_line(const char *msg)
{
    if (!msg) return;
    printf("%s\n", msg);

    if (ble_command_handler_get_source() == WM_CMD_SRC_LORA)
    {
        lora_manager_send_text(msg);
        return;
    }

    if (!ble_should_send_msg(msg)) {
        return;
    }

    nimble_adv_send_control_text(msg);
}

/* Replace ble_emit_control_line() with this */
static void ble_emit_control_line(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    printf("%s\n", buffer);

    if (ble_command_handler_get_source() == WM_CMD_SRC_LORA)
    {
        lora_manager_send_text(buffer);
        return;
    }

    if (!ble_should_send_msg(buffer)) {
        return;
    }

    nimble_adv_send_control_text(buffer);
}

/*
 * Send a compact uplink over LoRa only.
 * Hard cap: 50 bytes. No-op when source is BLE.
 */
static void lora_emit(const char *fmt, ...)
{
    if (ble_command_handler_get_source() != WM_CMD_SRC_LORA) return;

    char buf[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    printf("[LORA TX] %s\n", buf);
    lora_manager_send_text(buf);
}

/* =========================================================
 * TIMEZONE / FAKE TIME
 * ========================================================= */

static void init_timezone(void)
{
    setenv("TZ", "UTC0", 1);
    tzset();
    printf("\n[TIME]\n[TIME] Timezone set to GMT/UTC\n\n");
}

static void build_fake_time(time_t *epoch_out, struct tm *tm_out)
{
    struct tm fake;
    memset(&fake, 0, sizeof(fake));
    fake.tm_year = FAKE_YEAR - 1900;
    fake.tm_mon  = FAKE_MONTH - 1;
    fake.tm_mday = FAKE_DAY;
    fake.tm_hour = FAKE_HOUR;
    fake.tm_min  = FAKE_MIN;
    fake.tm_sec  = FAKE_SEC;
    fake.tm_isdst = -1;

    time_t epoch = mktime(&fake);
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    if (epoch_out) *epoch_out = epoch;
    if (tm_out)    *tm_out    = fake;

    printf("\n[FAKE TIME]\n[FAKE TIME] Applied\n[FAKE TIME] Epoch : %lld\n"
           "[FAKE TIME] %04d-%02d-%02d %02d:%02d:%02d\n\n",
           (long long)epoch,
           fake.tm_year + 1900, fake.tm_mon + 1, fake.tm_mday,
           fake.tm_hour, fake.tm_min, fake.tm_sec);
}

bool get_local_time(time_t *epoch_out, struct tm *tm_out)
{
    time_t now = time(NULL);
    if (now < 1700000000) return false;

    struct tm local_tm;
    memset(&local_tm, 0, sizeof(local_tm));
    localtime_r(&now, &local_tm);

    if (epoch_out) *epoch_out = now;
    if (tm_out)    *tm_out    = local_tm;
    return true;
}

void initialize_rtc_time(void)
{
    init_timezone();
    time_t now = time(NULL);
    if (now > 1700000000) {
        printf("\n[RTC]\n[RTC] Existing RTC valid\n\n");
        return;
    }
    printf("\n[RTC]\n[RTC] Forcing hard-coded fake time\n\n");
    build_fake_time(NULL, NULL);
}

/* =========================================================
 * BLE WINDOW LOGIC
 * ========================================================= */

static bool is_within_month_window(const struct tm *t)
{
    if (!t) return false;
    bool day_ok  = (t->tm_mday >= CONFIG_WM_SCHEDULE_DAY_START) && (t->tm_mday <= CONFIG_WM_SCHEDULE_DAY_END);
    bool hour_ok = (t->tm_hour >= CONFIG_WM_DAYTIME_START_HOUR) && (t->tm_hour <= CONFIG_WM_DAYTIME_END_HOUR);
    return day_ok && hour_ok;
}

bool is_ble_window_allowed(const struct tm *t) { return is_within_month_window(t); }

bool is_ble_adv_slot_now(const struct tm *t)
{
    if (!t || !is_within_month_window(t)) return false;
    return ((t->tm_min % CONFIG_WM_PERIODIC_INTERVAL_MIN) == 0) && (t->tm_sec <= BLE_ADV_ALLOWED_FOR_SEC);
}

static time_t next_ble_boundary_epoch(time_t now_epoch)
{
    struct tm t;
    localtime_r(&now_epoch, &t);
    int interval = CONFIG_WM_PERIODIC_INTERVAL_MIN;
    if (interval <= 0) interval = 5;
    t.tm_min = ((t.tm_min / interval) + 1) * interval;
    t.tm_sec = 0; t.tm_isdst = -1;
    return mktime(&t);
}

static time_t next_month_window_start_epoch(time_t now_epoch)
{
    struct tm t;
    localtime_r(&now_epoch, &t);
    t.tm_sec = 0; t.tm_min = 0;
    t.tm_hour = CONFIG_WM_DAYTIME_START_HOUR;
    t.tm_mday = CONFIG_WM_SCHEDULE_DAY_START;
    t.tm_isdst = -1;
    time_t candidate = mktime(&t);
    if (candidate <= now_epoch) { t.tm_mon += 1; candidate = mktime(&t); }
    return candidate;
}

uint64_t compute_sleep_us_after_now(time_t now_epoch)
{
    struct tm now_tm;
    localtime_r(&now_epoch, &now_tm);

    if (!is_within_month_window(&now_tm))
    {
        time_t next_start = next_month_window_start_epoch(now_epoch);
        if (next_start <= now_epoch) return 5ULL * 60ULL * 1000000ULL;
        return (uint64_t)(next_start - now_epoch) * 1000000ULL;
    }

    time_t next_boundary = next_ble_boundary_epoch(now_epoch);
    struct tm boundary_tm;
    localtime_r(&next_boundary, &boundary_tm);

    if (is_within_month_window(&boundary_tm))
        return (uint64_t)(next_boundary - now_epoch) * 1000000ULL;

    time_t next_start = next_month_window_start_epoch(now_epoch);
    if (next_start <= now_epoch) return 5ULL * 60ULL * 1000000ULL;
    return (uint64_t)(next_start - now_epoch) * 1000000ULL;
}

/* =========================================================
 * BLE / STORAGE HELPERS
 * ========================================================= */

void publish_current_state(void)
{
    uint32_t total = meter_total(rtc_blocks, pulse_bucket, BLOCK_SIZE);

    char msg[128];
    snprintf(msg, sizeof(msg), "TOTAL=%" PRIu32 " TAMPER=%d/%d/%d",
             total,
             meter_helper_get_case_tamper()    ? 1 : 0,
             meter_helper_get_magnetic_tamper() ? 1 : 0,
             meter_helper_get_removal_tamper()  ? 1 : 0);

    ble_buffer_push(msg);
    nimble_adv_update_data(total, meter_helper_get_case_tamper());
    if (ble_started) nimble_adv_flush();
}

/* =========================================================
 * COMMAND HANDLER
 * ========================================================= */

void ble_command_handler(const char *cmd)
{
	last_ble_activity = esp_timer_get_time();
    if (!cmd) return;

    char local[96];
    memset(local, 0, sizeof(local));
    strncpy(local, cmd, sizeof(local) - 1);

    /* Strip trailing whitespace */
    for (int i = (int)strlen(local) - 1; i >= 0; i--)
    {
        if (local[i] == '\r' || local[i] == '\n' ||
            local[i] == ' '  || local[i] == '\t')
            local[i] = '\0';
        else
            break;
    }

    bool from_lora = (ble_command_handler_get_source() == WM_CMD_SRC_LORA);

    if (!from_lora) ble_emit_control_line("[BLE CMD] %s", local);
    else            printf("[LORA CMD] %s\n", local);

    last_activity = esp_timer_get_time();

    /* LOCK — no auth required */
    if (strcmp(local, "LOCK") == 0)
    {
        nimble_adv_authorize(false);
        nimble_adv_disconnect();
        ble_emit_control_line("AUTH:FAIL");
        ble_emit_line("[BLE] LOCKED");
        return;
    }

    if (!from_lora && !nimble_adv_is_authorized())
    {
        ble_emit_line("[BLE] NOT AUTHORIZED");
        return;
    }

    /* -------------------------------------------------------
     * SET_FEATURE
     * LoRa: OK / ERR
     * ----------------------------------------------------- */
    if (strncmp(cmd, "SET_FEATURE,", 12) == 0)
    {
        char feature[16];
        int  value;
        if (sscanf(cmd, "SET_FEATURE,%15[^,],%d", feature, &value) == 2)
        {
            bool enable = (value != 0);
            if      (strcasecmp(feature, "BLE")  == 0) { s_features.ble_enabled = enable; }
            else if (strcasecmp(feature, "LORA") == 0) { s_features.lora_enabled = enable; }
            else if (strcasecmp(feature, "BOTH") == 0) { s_features.ble_enabled = enable; s_features.lora_enabled = enable; }
            else { lora_emit("ERR"); if (!from_lora) ble_emit_control_line("[BLE] ERR:UNKNOWN_FEATURE"); return; }
            lora_emit("OK");
        }
        else
        {
            if (!from_lora) ble_emit_control_line("[BLE] ERR:BAD_FORMAT");
            lora_emit("ERR feature");
        }
        return;
    }

    /* -------------------------------------------------------
     * DISC
     * ----------------------------------------------------- */
    if (strcmp(cmd, "DISC") == 0)
    {
		
        nimble_adv_authorize(false);
        nimble_adv_disconnect();
		g_force_run_state = false;
        return;
    }

    /* -------------------------------------------------------
     * SET_LOCATION
     * LoRa: OK / ERR
     * ----------------------------------------------------- */
    if (strncmp(local, "SET_LOCATION ", 13) == 0)
    {
        float lat = 0.0f, lon = 0.0f;
        char *data = local + 13;
        for (int i = 0; data[i]; i++) if (data[i] == ',') data[i] = ' ';

        if (sscanf(data, "%f %f", &lat, &lon) == 2 &&
            meter_storage_set_location_coords(lat, lon) == ESP_OK)
        {
            if (!from_lora) ble_emit_control_line("[EEPROM] LOCATION UPDATED : %.6f, %.6f", lat, lon);
            
			if(cfg_command){
						return;
					}
            lora_emit("OK");
        }
        else
        {
            if (!from_lora) ble_emit_line("[EEPROM] SET_LOCATION FAILED");
            lora_emit("ERR");
        }
        return;
    }

    /* -------------------------------------------------------
     * LOCK/UNLOCK BLE MONTH
     * LoRa: OK
     * ----------------------------------------------------- */
    if (strcmp(local, "LOCK_BLE_MONTH") == 0)
    {
        time_t now = time(NULL); struct tm t; localtime_r(&now, &t);
        ble_lock_month((uint16_t)(t.tm_year + 1900), (uint8_t)(t.tm_mon + 1));
        nimble_adv_disconnect();
        stop_ble_if_needed();
        lora_emit("OK");
        return;
    }
    if (strcmp(local, "UNLOCK_BLE_MONTH") == 0)
    {
        ble_unlock_month();
        lora_emit("OK");
        return;
    }

    /* -------------------------------------------------------
     * LOCK/UNLOCK LORA MONTH
     * LoRa: OK
     * ----------------------------------------------------- */
    if (strcmp(local, "LOCK_LORA_MONTH") == 0)
    {
        time_t now = time(NULL); struct tm t; localtime_r(&now, &t);
        lora_lock_month((uint16_t)(t.tm_year + 1900), (uint8_t)(t.tm_mon + 1));
        lora_emit("OK");
        return;
    }
    if (strcmp(local, "UNLOCK_LORA_MONTH") == 0)
    {
        lora_unlock_month();
        lora_emit("OK");
        return;
    }

    /* -------------------------------------------------------
     * GET_FEATURES
     * BLE: BLE=1,LORA=1,BLE_LOCK=1,LORA_LOCK=1
     * LoRa: BLE=1,LOR=1,BL=1,LL=1  (~21 bytes)
     * ----------------------------------------------------- */
    if (strcmp(cmd, "GET_FEATURES") == 0)
    {
        if (!from_lora)
            ble_emit_control_line("BLE=%d,LORA=%d,BLE_LOCK=%d,LORA_LOCK=%d",
                s_features.ble_enabled  ? 1 : 0, s_features.lora_enabled ? 1 : 0,
                s_ble_month_locked ? 1 : 0,       s_lora_month_locked ? 1 : 0);
        else
            lora_emit("OK BLE=%d,LOR=%d,BL=%d,LL=%d",
                s_features.ble_enabled  ? 1 : 0, s_features.lora_enabled ? 1 : 0,
                s_ble_month_locked ? 1 : 0,       s_lora_month_locked ? 1 : 0);
        return;
    }

    /* -------------------------------------------------------
     * MONTH_HIS
     * LoRa: one uplink per record (MH,12,16777215 = 14 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "MONTH_HIS") == 0)
    {
        monthly_history_record_t hist[MONTHLY_HISTORY_MAX];
        esp_err_t ret = meter_storage_read_monthly_history_raw(hist, MONTHLY_HISTORY_MAX);

        if (ret != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_control_line("ERR:EEPROM_READ_FAILED"); return; }

        char msg[32];
        for (size_t i = 0; i < MONTHLY_HISTORY_MAX; i++)
        {
            snprintf(msg, sizeof(msg), "MH,%02u,%" PRIu32, hist[i].month, hist[i].total_counter);
            if (!from_lora) { ble_emit_control_line(msg); vTaskDelay(pdMS_TO_TICKS(20)); }
            else            { lora_emit("%s", msg);       vTaskDelay(pdMS_TO_TICKS(200)); }
        }
        if (!from_lora) ble_buffer_push("MH_END");
        else            lora_emit("MH_END");
        return;
    }

    /* -------------------------------------------------------
     * IS_FIRST_MAG
     * LoRa: MAG=0/1  (5 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "IS_FIRST_MAG") == 0)
    {
        if (!from_lora)
        {
            if      (first_mag_tamper == 0) ble_emit_control_line("OK NO MAG TAMPER");
            else if (first_mag_tamper == 1) ble_emit_control_line("OK YES");
            else                            ble_emit_control_line("OK NO");
        }
        else { lora_emit("OK MAG=%d", first_mag_tamper ? 1 : 0); }
        return;
    }

    /* -------------------------------------------------------
     * GET
     * BLE: TOTAL=n TAMPER=c/m/r
     * LoRa: TOT=n,T=b  where b = bitmask (bit2=case,bit1=mag,bit0=rem)
     *       e.g. TOT=16777215,T=7  (17 bytes worst case)
     * ----------------------------------------------------- */
    if (strcmp(local, "GET") == 0)
    {
        
        if (!from_lora) ble_emit_control_line("[BLE] GET OK");
        else
        {
            uint32_t total = meter_total(rtc_blocks, pulse_bucket, BLOCK_SIZE);
            uint8_t  tb    = (meter_helper_get_case_tamper()     ? 4 : 0)
                           | (meter_helper_get_magnetic_tamper()  ? 2 : 0)
                           | (meter_helper_get_removal_tamper()   ? 1 : 0);
            lora_emit("TOT=%lu,T=%u", (unsigned long)total, tb);
        }
        return;
    }

    /* -------------------------------------------------------
     * GET_CONFIG
     * BLE: one verbose line (unchanged)
     * LoRa: 2 uplinks
     *   CFG1: ID,type,mult,counter,tamper_bits,battery,fw
     *         worst case: CFG1:WM000001,255,255,16777215,7,100,99 = 39 bytes
     *   CFG2: DD-MM-YYYY,tz,epoch,lat,lon
     *         worst case: CFG2:31-12-2099,330,9999999999,23.2599,77.4126 = 46 bytes
     * ----------------------------------------------------- */
    if (strcmp(local, "GET_CONFIG") == 0)
    {
        meter_storage_record_t nv;
        esp_err_t err = meter_storage_get_copy(&nv);

        if (!from_lora) ble_emit_control_line("[GET_CONFIG] ERR = %s", esp_err_to_name(err));

        if (err != ESP_OK) { lora_emit("ERR"); return; }

        if (!from_lora)
        {
            ble_emit_control_line(
                "CFG:%s,%u,%u,%02u-%02u-%04u,%u,%" PRIu32 ",%u/%u/%u,%u,%" PRId32 ",%" PRIi64 ",%.6f,%.6f,%d",
                nv.meter_id, nv.meter_type, nv.volume_multiplier,
                nv.install_day, nv.install_month, nv.install_year,
                nv.install_counter,
                (uint32_t)((rtc_blocks * 10) + pulse_bucket),
                nv.tamper_case, nv.tamper_magnetic, nv.tamper_removal,
                nv.battery_level, nv.timezone_min, nv.last_epoch,
                nv.location.latitude, nv.location.longitude,
                CONFIG_WM_FW_VERSION);
        }
        else
        {
            uint8_t tb = (nv.tamper_case    ? 4 : 0)
                       | (nv.tamper_magnetic ? 2 : 0)
                       | (nv.tamper_removal  ? 1 : 0);

            /* Uplink 1 — identity & meter data */
            lora_emit("CFG1:%s,%u,%u,%lu,%u,%u,%d",
                nv.meter_id, nv.meter_type, nv.volume_multiplier,
                (unsigned long)nv.install_counter,
                tb, nv.battery_level, CONFIG_WM_FW_VERSION);

            vTaskDelay(pdMS_TO_TICKS(500));

            /* Uplink 2 — date, time & location */
            lora_emit("CFG2:%02u-%02u-%04u,%" PRId32 ",%" PRIi64 ",%.4f,%.4f",
                nv.install_day, nv.install_month, nv.install_year,
                nv.timezone_min, nv.last_epoch,
                nv.location.latitude, nv.location.longitude);
        }
        return;
    }

    /* -------------------------------------------------------
     * GET_LOC
     * LoRa: LOC:lat,lon  (~20 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "GET_LOC") == 0)
    {
        meter_storage_record_t nv;
        esp_err_t err = meter_storage_get_copy(&nv);

        if (err == ESP_OK)
        {
            if (!from_lora)
            {
                ble_emit_control_line("GET LOC ERR = %s", esp_err_to_name(err));
                ble_emit_control_line("LOC:%.6f,%.6f", nv.location.latitude, nv.location.longitude);
            }
            else { lora_emit("LOC:%.4f,%.4f", nv.location.latitude, nv.location.longitude); }
        }
        else
        {
            if (!from_lora) ble_emit_control_line("[EEPROM] GET_LOC ERR = %s", esp_err_to_name(err));
            lora_emit("ERR");
        }
        return;
    }

    /* -------------------------------------------------------
     * GET_TAMPER
     * BLE: verbose multi-line
     * LoRa: TC=n,TM=n,TR=n  (~20 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "GET_TAMPER") == 0)
    {
        meter_storage_record_t nv;
        esp_err_t err = meter_storage_get_copy(&nv);

        if (!from_lora)
        {
            ble_emit_control_line("[GET_CONFIG] ERR = %s", esp_err_to_name(err));
            ble_emit_control_line("Case Count : %" PRIu32, nv.tamper_history.case_count);
            ble_emit_control_line("Case First : %" PRIi64, nv.tamper_history.first_case_epoch);
            ble_emit_control_line("Case Last  : %" PRIi64, nv.tamper_history.last_case_epoch);
            ble_emit_control_line("Mag Count  : %" PRIu32, nv.tamper_history.magnetic_count);
            ble_emit_control_line("Mag First  : %" PRIi64, nv.tamper_history.first_magnetic_epoch);
            ble_emit_control_line("Mag Last   : %" PRIi64, nv.tamper_history.last_magnetic_epoch);
            ble_emit_control_line("Rem Count  : %" PRIu32, nv.tamper_history.removal_count);
            ble_emit_control_line("Rem First  : %" PRIi64, nv.tamper_history.first_removal_epoch);
            ble_emit_control_line("Rem Last   : %" PRIi64, nv.tamper_history.last_removal_epoch);
        }
        else
        {
            lora_emit("TC=%lu,TM=%lu,TR=%lu",
                (unsigned long)nv.tamper_history.case_count,
                (unsigned long)nv.tamper_history.magnetic_count,
                (unsigned long)nv.tamper_history.removal_count);
        }
        return;
    }

    /* -------------------------------------------------------
     * CLR_TAMPER
     * LoRa: OK
     * ----------------------------------------------------- */
    if (strcmp(local, "CLR_TAMPER") == 0)
    {
        meter_helper_clear_all_tampers();
        tamper_reset();
        (void)meter_helper_flush_runtime_save(true);

        if (!from_lora) ble_emit_line("[TAMPER] CLEARED");
        lora_emit("OK TAMPER");
        return;
    }

    /* -------------------------------------------------------
     * SAVE
     * LoRa: OK / ERR
     * ----------------------------------------------------- */
    if (strcmp(local, "SAVE") == 0)
    {
        save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
            meter_helper_get_case_tamper(),
            meter_helper_get_magnetic_tamper(),
            meter_helper_get_removal_tamper(),
            (int64_t)time(NULL));

        esp_err_t ret = meter_helper_flush_runtime_save(true);
        if (!from_lora)
        {
            if (ret == ESP_OK) ble_emit_line("[EEPROM] STATE SAVED");
            else               ble_emit_control_line("[EEPROM] SAVE FAILED: %s", esp_err_to_name(ret));
        }
        lora_emit(ret == ESP_OK ? "OK" : "ERR");
        return;
    }

    /* -------------------------------------------------------
     * SET_METER_ID  /  SET_METER_TYPE  /  SET_MULTIPLIER
     * SET_BATTERY  /  SET_TZ  /  SET_INSTALL_DATE
     * SET_INSTALL_COUNTER  /  SET_TIME
     * LoRa: OK / ERR for all
     * ----------------------------------------------------- */
    if (strncmp(local, "SET_METER_ID ", 13) == 0)
    {
        const char *id = local + 13;
        if (meter_storage_set_meter_id(id) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_METER_ID FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] METER ID UPDATED : %s", id);
        
		if(cfg_command){
					return;
				}
		lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_METER_TYPE ", 15) == 0)
    {
        uint32_t v = strtoul(local + 15, NULL, 10);
        if (meter_storage_set_meter_type((uint8_t)v) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_METER_TYPE FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] METER TYPE UPDATED : %u", (unsigned)v);
        
		if(cfg_command){
			return;
		}
		lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_MULTIPLIER ", 15) == 0)
    {
        uint32_t v = strtoul(local + 15, NULL, 10);
        if (meter_storage_set_volume_multiplier((uint8_t)v) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_MULTIPLIER FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] MULTIPLIER UPDATED : %u", (unsigned)v);
        
		if(cfg_command){
					return;
				}
		lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_BATTERY ", 12) == 0)
    {
        uint32_t v = strtoul(local + 12, NULL, 10);
        if (meter_storage_set_battery_level((uint8_t)v) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_BATTERY FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] BATTERY UPDATED : %u", (unsigned)v);
         lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_TZ ", 7) == 0)
    {
        int32_t v = (int32_t)strtol(local + 7, NULL, 10);
        if (meter_storage_set_timezone_min(v) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_TZ FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] TIMEZONE UPDATED : %" PRId32, v);
         lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_INSTALL_DATE ", 17) == 0)
    {
        unsigned d = 0, m = 0, y = 0;
        if (sscanf(local + 17, "%u %u %u", &d, &m, &y) != 3) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_INSTALL_DATE FORMAT: DD MM YYYY"); return; }
        if (meter_storage_set_install_date((uint8_t)d, (uint8_t)m, (uint16_t)y) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_INSTALL_DATE FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] INSTALL DATE UPDATED : %02u-%02u-%04u", d, m, y);
        
		if(cfg_command){
			return;
		}

		lora_emit("OK"); return;
    }

    if (strncmp(local, "SET_INSTALL_COUNTER ", 20) == 0)
    {
        uint32_t v = strtoul(local + 20, NULL, 10);
        rtc_blocks = v / BLOCK_SIZE; pulse_bucket = v % BLOCK_SIZE;
        if (meter_storage_set_install_counter((uint16_t)v) != ESP_OK) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[EEPROM] SET_INSTALL_COUNTER FAILED"); return; }
        if (!from_lora) ble_emit_control_line("[EEPROM] INSTALL COUNTER UPDATED : %u", (unsigned)v);
        
		if(cfg_command){
					return;
				}
		lora_emit("OK");
		return;
    }

    if (strncmp(local, "SET_TIME ", 9) == 0)
    {
        int64_t epoch = strtoll(local + 9, NULL, 10);
        if (epoch <= 0) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[TIME] INVALID EPOCH"); return; }

        struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
        if (settimeofday(&tv, NULL) != 0) { lora_emit("ERR"); if (!from_lora) ble_emit_line("[TIME] SET FAILED"); return; }

        save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
            meter_helper_get_case_tamper(), meter_helper_get_magnetic_tamper(),
            meter_helper_get_removal_tamper(), epoch);

        if (meter_helper_flush_runtime_save(true) != ESP_OK)
            if (!from_lora) ble_emit_line("[EEPROM] TIME SAVE FAILED");

        if (!from_lora) ble_emit_control_line("[TIME] UPDATED EPOCH=%" PRIi64, epoch);
         lora_emit("OK"); return;
    }

    /* -------------------------------------------------------
     * GET_TOTAL
     * LoRa: TOT=n  (≤13 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "GET_TOTAL") == 0)
    {
        uint32_t total = meter_total(rtc_blocks, pulse_bucket, BLOCK_SIZE);
        if (!from_lora) ble_emit_control_line("[BLE] TOTAL : %" PRIu32, total);
        
        lora_emit("OK TOT=%lu", (unsigned long)total);
        return;
    }

    /* -------------------------------------------------------
     * 03FFFF binary config result signalling
     * lora_manager calls ble_command_handler("CFG_OK") or
     * ble_command_handler("CFG_FAIL") after parsing the packet.
     * LoRa reply: 03FFFF:1 or 03FFFF:0  (8 bytes)
     * ----------------------------------------------------- */
    if (strcmp(local, "CFG_OK") == 0)   { lora_emit("03FFFF:1"); return; }
    if (strcmp(local, "CFG_FAIL") == 0) { lora_emit("03FFFF:0"); return; }

    /* -------------------------------------------------------
     * Unknown
     * ----------------------------------------------------- */
    if (!from_lora) ble_emit_control_line("[BLE] UNKNOWN COMMAND");
    lora_emit("ERR");
}

/* =========================================================
 * BLE START/STOP
 * ========================================================= */

bool auth_session_active(int64_t now_us)
{
    if (!nimble_adv_is_authorized() || auth_start_us <= 0) return false;
    return (now_us - auth_start_us) < AUTH_WINDOW_US;
}

void start_ble_if_needed(void)
{
    if (!ble_started)
    {
        printf("\n[BLE]\n[BLE] START REQUESTED\n\n");
        if (nimble_adv_start()) { ble_started = true; printf("\n[BLE]\n[BLE] STARTED\n\n"); }
        else printf("\n[BLE]\n[BLE] NOT READY YET, WILL RETRY\n\n");
    }
}

void stop_ble_if_needed(void)
{
    if (ble_started)
    {
        printf("\n[BLE]\n[BLE] STOPPED\n\n");
        nimble_adv_stop();
        ble_started = false;
    }
}

void check_and_store_monthly_history(time_t epoch, const struct tm *tm_now)
{
    if (!tm_now) return;

    int current_month = tm_now->tm_mon + 1;
    int current_year  = tm_now->tm_year + 1900;

    if (last_saved_month < 0) { last_saved_month = current_month; last_saved_year = current_year; return; }

    if (current_month == last_saved_month && current_year == last_saved_year) return;

    uint32_t total = meter_total(rtc_blocks, pulse_bucket, BLOCK_SIZE);
    esp_err_t err  = meter_storage_add_monthly_record(last_saved_month, total);

    if (err == ESP_OK)
        printf("\n[MONTHLY HISTORY]\nSaved Month : %02d-%04d\nTotal        : %" PRIu32 "\n\n",
               last_saved_month, last_saved_year, total);
    else
        printf("\n[MONTHLY HISTORY]\nSAVE FAILED\n\n");

    last_saved_month = current_month;
    last_saved_year  = current_year;
}

/* =========================================================
 * TOTAL METER VALUE
 * ========================================================= */

uint32_t meter_total(uint32_t rtc_blocks, uint32_t pulse_bucket, uint32_t block_size)
{
    return (rtc_blocks * block_size) + pulse_bucket;
}

/* =========================================================
 * BLE BUFFER FLUSH
 * ========================================================= */

void ble_flush_buffer(uint32_t send_gap_ms)
{
    if (!nimble_adv_is_connected() || !nimble_adv_is_authorized()) { printf("\n[BLE TX BLOCKED]\n"); return; }

    char msg[128];
    while (ble_buffer_pop(msg, sizeof(msg)))
    {
        printf("\n[BLE TX] %s\n", msg);
        nimble_adv_send_text(msg);
        vTaskDelay(pdMS_TO_TICKS(send_gap_ms));
    }
}

/* =========================================================
 * SAVE RTC STATE
 * ========================================================= */

void save_rtc_to_eeprom(uint32_t rtc_blocks, uint32_t pulse_bucket,
                        bool tamper_case, bool tamper_magnetic, bool tamper_removal,
                        int64_t last_epoch)
{
    printf("\n[EEPROM]\n[EEPROM] SAVE QUEUED\n"
           "[EEPROM] Blocks : %" PRIu32 "\n[EEPROM] Bucket : %" PRIu32 "\n"
           "[EEPROM] Tamper : %d/%d/%d\n[EEPROM] Epoch  : %" PRIi64 "\n\n",
           rtc_blocks, pulse_bucket,
           tamper_case ? 1 : 0, tamper_magnetic ? 1 : 0, tamper_removal ? 1 : 0,
           last_epoch);

    stage_runtime_save(rtc_blocks, pulse_bucket, tamper_case, tamper_magnetic, tamper_removal, last_epoch);
}

/* =========================================================
 * NORMAL SLEEP
 * ========================================================= */

void request_sleep_safe(int gpio1, int gpio2, bool *ble_started,
                        uint32_t rtc_blocks, uint32_t pulse_bucket,
                        bool tamper_flag, uint32_t stable_ms, bool save_state)
{
    (void)stable_ms; (void)tamper_flag;
    printf("\n[SLEEP]\n[SLEEP] NORMAL SLEEP REQUEST\n\n");
    nimble_adv_stop(); *ble_started = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("\n[SYSTEM]\n[SYSTEM] CALLING DEEP SLEEP\n\n");
    deep_sleep_enter(gpio1, gpio2, -1);
}

/* =========================================================
 * STUCK GPIO SLEEP
 * ========================================================= */

void request_sleep_stuck(int gpio1, int gpio2, int stuck_gpio, bool *ble_started,
                         uint32_t rtc_blocks, uint32_t pulse_bucket,
                         bool tamper_flag, bool save_state)
{
    (void)tamper_flag;
    printf("\n[SLEEP]\n[SLEEP] STUCK GPIO REQUEST\n[SLEEP] GPIO : %d\n\n", stuck_gpio);

    if (save_state)
    {
        save_rtc_to_eeprom(rtc_blocks, pulse_bucket,
            meter_helper_get_case_tamper(), meter_helper_get_magnetic_tamper(),
            meter_helper_get_removal_tamper(), (int64_t)time(NULL));
        (void)meter_helper_flush_runtime_save(true);
    }

    nimble_adv_stop(); *ble_started = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("\n[SYSTEM]\n[SYSTEM] CALLING STUCK DEEP SLEEP\n\n");
    deep_sleep_enter(gpio1, gpio2, stuck_gpio);
}