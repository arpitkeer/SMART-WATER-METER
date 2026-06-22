#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <limits.h>
#include "meter_storage.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <inttypes.h>

#include "eeprom_24cxx.h"
#include "soc/gpio_num.h"
#include "../meter_helper/include/meter_helper.h"

static const char *TAG = "meter_storage";

#ifndef CONFIG_WM_DEVICE_NAME_PREFIX
#define CONFIG_WM_DEVICE_NAME_PREFIX "WM000001"
#endif

#ifndef CONFIG_WM_METER_TYPE
#define CONFIG_WM_METER_TYPE 0xAC
#endif

#ifndef CONFIG_WM_VOLUME_MULTIPLIER
#define CONFIG_WM_VOLUME_MULTIPLIER 1
#endif

#define EEPROM_I2C_PORT      I2C_NUM_0
#define EEPROM_SDA           GPIO_NUM_27
#define EEPROM_SCL           GPIO_NUM_26
#define EEPROM_POWER_GPIO    CONFIG_WM_EEPROM_POWER_GPIO
#define EEPROM_ADDR          0x50

#define EEPROM_BASE_ADDR     0x00

#define EEPROM_LOAD_RETRIES 5
#define EEPROM_RETRY_DELAY_MS 100

static meter_storage_record_t s_nv;
static bool s_inited = false;
static esp_err_t commit_internal(void);

esp_err_t meter_storage_read_monthly_history_raw(
    monthly_history_record_t *history,
    size_t max_entries)
{
    if (!history) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t count =
        (max_entries > MONTHLY_HISTORY_MAX)
            ? MONTHLY_HISTORY_MAX
            : max_entries;

    size_t offset =
        offsetof(
            meter_storage_record_t,
            monthly_history);

    return eeprom_24cxx_read(
        EEPROM_BASE_ADDR + offset,
        (uint8_t *)history,
        count * sizeof(monthly_history_record_t));
}

esp_err_t meter_storage_add_monthly_record(
    uint8_t month,
    uint32_t total)
{
    uint16_t idx = s_nv.monthly_history_index % MONTHLY_HISTORY_MAX;

    s_nv.monthly_history[idx].month = month;
    s_nv.monthly_history[idx].total_counter = total;

    s_nv.monthly_history_index++;

    printf(     "Monthly history saved: %02u Total=%" PRIu32 "\n",
                month,
                total);

    return meter_storage_commit();
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void fill_defaults(void)
{
    memset(&s_nv, 0, sizeof(s_nv));

    s_nv.magic = METER_STORAGE_MAGIC;
    s_nv.version = METER_STORAGE_VERSION;
    s_nv.struct_size = sizeof(s_nv);

    memset(s_nv.meter_id, 0, sizeof(s_nv.meter_id));
    strncpy(s_nv.meter_id, CONFIG_WM_DEVICE_NAME_PREFIX, sizeof(s_nv.meter_id) - 1);

    s_nv.meter_type = CONFIG_WM_METER_TYPE;
    s_nv.volume_multiplier = CONFIG_WM_VOLUME_MULTIPLIER;

    s_nv.install_day = 1;
    s_nv.install_month = 1;
    s_nv.install_year = 2026;
    s_nv.install_counter = 0;

    s_nv.rtc_blocks = 0;
    s_nv.pulse_bucket = 0;

    s_nv.tamper_case = 0;
    s_nv.tamper_magnetic = 0;
    s_nv.tamper_removal = 0;   /* use this slot as removal tamper */
    memset(
        &s_nv.tamper_history,
        0,
        sizeof(s_nv.tamper_history));

    s_nv.battery_level = 95;
    s_nv.timezone_min = 330;   /* India default */
    s_nv.last_epoch = 0;
	s_nv.location.latitude = 0.0f;
	s_nv.location.longitude = 0.0f;

    s_nv.crc16 = crc16_ccitt(
        (const uint8_t *)&s_nv,
        offsetof(meter_storage_record_t, crc16));
}
esp_err_t meter_storage_set_location_coords(float lat, float lon)
{
    s_nv.location.latitude = lat;
    s_nv.location.longitude = lon;

    return commit_internal();
}

static bool is_valid_record(const meter_storage_record_t *nv)
{
    if (!nv) {
        return false;
    }

    if (nv->magic != METER_STORAGE_MAGIC) {
        return false;
    }

    if (nv->version != METER_STORAGE_VERSION) {
        return false;
    }

    if (nv->struct_size != sizeof(meter_storage_record_t)) {
        return false;
    }

    uint16_t crc = crc16_ccitt(
        (const uint8_t *)nv,
        offsetof(meter_storage_record_t, crc16));

    return (crc == nv->crc16);
}

static esp_err_t commit_internal(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    s_nv.magic = METER_STORAGE_MAGIC;
    s_nv.version = METER_STORAGE_VERSION;
    s_nv.struct_size = sizeof(s_nv);

    s_nv.crc16 = crc16_ccitt(
        (const uint8_t *)&s_nv,
        offsetof(meter_storage_record_t, crc16));

    esp_err_t ret = eeprom_24cxx_write(
        EEPROM_BASE_ADDR,
        (const uint8_t *)&s_nv,
        sizeof(s_nv));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM save failed: %s", esp_err_to_name(ret));
    }
    else{
        ESP_LOGI(TAG, "eeprom save successful");
    }

    return ret;
}

static esp_err_t init_eeprom_hw(void)
{
    eeprom_24cxx_config_t rom = {
        .port = EEPROM_I2C_PORT,
        .sda_io = EEPROM_SDA,
        .scl_io = EEPROM_SCL,
        .scl_speed_hz = 50000,
        .i2c_addr_7bit = EEPROM_ADDR,
        .addr_bytes = EEPROM_ADDR_1BYTE,
        .enable_internal_pullups = true
    };

    return eeprom_24cxx_init(&rom);
}

esp_err_t meter_storage_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t ret = init_eeprom_hw();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = meter_storage_load();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load EEPROM (Err: %s). Formatting with defaults.", esp_err_to_name(ret));
        fill_defaults();
        s_inited = true; // Force initialization to true to allow commit_internal to pass
        commit_internal(); // Save the defaults immediately
        return ESP_OK; // Return OK because we recovered
    }

    s_inited = true;
    return ESP_OK;
}

esp_err_t meter_storage_get_copy(meter_storage_record_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        ESP_LOGE(TAG, "Get failed: Storage NOT initialized! Returning zeros.");
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_nv;
    return ESP_OK;
}

esp_err_t meter_storage_load(void)
{
    meter_storage_record_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < EEPROM_LOAD_RETRIES; i++)
    {
        ret = eeprom_24cxx_read(
            EEPROM_BASE_ADDR,
            (uint8_t *)&tmp,
            sizeof(tmp));

        if (ret == ESP_OK)
        {
            break;
        }

        printf(
            "[EEPROM] Read failed (%d/%d): %s\n",
            i + 1,
            EEPROM_LOAD_RETRIES,
            esp_err_to_name(ret));

        vTaskDelay(
            pdMS_TO_TICKS(EEPROM_RETRY_DELAY_MS));
    }

    if (ret != ESP_OK)
    {
        printf(
            "[EEPROM] Communication failure\n");

        return ret;
    }

    if (!is_valid_record(&tmp))
    {
        printf("[EEPROM] Record invalid\n");
        return ESP_ERR_INVALID_CRC;
    }

    s_nv = tmp;
    s_inited = true;

    return ESP_OK;
}

esp_err_t meter_storage_commit(void)
{
    return commit_internal();
}

const meter_storage_record_t *meter_storage_get(void)
{
    return &s_nv;
}

esp_err_t meter_storage_set_runtime_state(
    uint32_t blocks,
    uint32_t bucket,
    bool tamper_case,
    bool tamper_magnetic,
    bool tamper_removal,
    int64_t last_epoch)
{
    s_nv.rtc_blocks = blocks;
    s_nv.pulse_bucket = bucket;
    s_nv.tamper_case = tamper_case ? 1 : 0;
    s_nv.tamper_magnetic = tamper_magnetic ? 1 : 0;
    s_nv.tamper_removal = tamper_removal ? 1 : 0;   /* removal stored here */
    s_nv.tamper_history = *meter_helper_get_tamper_history();
    s_nv.last_epoch = last_epoch;
    return commit_internal();
}

esp_err_t meter_storage_set_meter_id(const char *meter_id)
{
    if (!meter_id || meter_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    memset(s_nv.meter_id, 0, sizeof(s_nv.meter_id));
    strncpy(s_nv.meter_id, meter_id, sizeof(s_nv.meter_id) - 1);

    return commit_internal();
}

esp_err_t meter_storage_set_meter_type(uint8_t meter_type)
{
    s_nv.meter_type = meter_type;
    return commit_internal();
}

esp_err_t meter_storage_set_volume_multiplier(uint8_t mult)
{
    if (mult == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_nv.volume_multiplier = mult;
    return commit_internal();
}

esp_err_t meter_storage_set_install_date(uint8_t day, uint8_t month, uint16_t year)
{
    if (day < 1 || day > 31) {
        return ESP_ERR_INVALID_ARG;
    }

    if (month < 1 || month > 12) {
        return ESP_ERR_INVALID_ARG;
    }

    if (year < 2000) {
        return ESP_ERR_INVALID_ARG;
    }

    s_nv.install_day = day;
    s_nv.install_month = month;
    s_nv.install_year = year;

    return commit_internal();
}

esp_err_t meter_storage_set_install_counter(uint16_t counter)
{
    s_nv.install_counter = counter;
    return commit_internal();
}

esp_err_t meter_storage_set_battery_level(uint8_t battery_level)
{
    if (battery_level > 100) {
        battery_level = 100;
    }

    s_nv.battery_level = battery_level;
    return commit_internal();
}

esp_err_t meter_storage_set_timezone_min(int32_t tz_min)
{
    if (tz_min < -1439 || tz_min > 1439) {
        return ESP_ERR_INVALID_ARG;
    }

    s_nv.timezone_min = tz_min;
    return commit_internal();
}

esp_err_t meter_storage_set_last_epoch(int64_t epoch)
{
    s_nv.last_epoch = epoch;
    return commit_internal();
}

uint32_t meter_storage_get_total(uint32_t block_size)
{
    uint64_t total = ((uint64_t)s_nv.rtc_blocks * (uint64_t)block_size)
                   + (uint64_t)s_nv.pulse_bucket;

    if (total > UINT32_MAX) {
        total = UINT32_MAX;
    }

    return (uint32_t)total;
}