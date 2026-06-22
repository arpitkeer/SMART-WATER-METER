#ifndef METER_STORAGE_H
#define METER_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define METER_STORAGE_MAGIC       0x574D4552UL  /* "WMER" */
#define METER_STORAGE_VERSION 0x0002 //this was changed for storing the history of tamper and count of tamper, just in case if there 
                                      // were multiple attempt for the tampers, earlier we could have only known the last tamper.
#define METER_STORAGE_MAX_ID_LEN   16
#ifndef MONTHLY_HISTORY_MAX
#define MONTHLY_HISTORY_MAX 6
#endif

esp_err_t meter_storage_add_monthly_record(
    uint8_t month,
    uint32_t total);

typedef struct __attribute__((packed))
{

    uint8_t month;

    uint32_t total_counter;

} monthly_history_record_t;


typedef struct
{
    int64_t first_case_epoch;
    int64_t first_magnetic_epoch;
    int64_t first_removal_epoch;

    int64_t last_case_epoch;
    int64_t last_magnetic_epoch;
    int64_t last_removal_epoch;

    uint32_t case_count;
    uint32_t magnetic_count;
    uint32_t removal_count;

} tamper_history_t;

typedef struct {
    float latitude;
    float longitude;
} meter_location_t;

// ... inside meter_storage_record_t, you still keep:
// meter_location_t location;

// Update the function prototype
esp_err_t meter_storage_set_location_coords(float lat, float lon);

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t struct_size;

    char     meter_id[METER_STORAGE_MAX_ID_LEN];
    uint8_t  meter_type;
    uint8_t  volume_multiplier;

    uint8_t  install_day;
    uint8_t  install_month;
    uint16_t install_year;
    uint16_t install_counter;

    uint32_t rtc_blocks;
    uint32_t pulse_bucket;

	uint8_t tamper_case;
	uint8_t tamper_magnetic;
	uint8_t tamper_removal;
	tamper_history_t tamper_history;
    uint8_t  battery_level;
    int32_t  timezone_min;
    int64_t  last_epoch;


    uint16_t monthly_history_index;

    monthly_history_record_t monthly_history[MONTHLY_HISTORY_MAX];
	meter_location_t location;

    uint16_t crc16;

} meter_storage_record_t;

esp_err_t meter_storage_init(void);
esp_err_t meter_storage_load(void);
esp_err_t meter_storage_commit(void);

const meter_storage_record_t *meter_storage_get(void);
esp_err_t meter_storage_get_copy(meter_storage_record_t *out);

esp_err_t meter_storage_set_runtime_state(
    uint32_t blocks,
    uint32_t bucket,
    bool tamper_case,
    bool tamper_magnetic,
    bool tamper_reverse,
    int64_t last_epoch);
esp_err_t meter_storage_read_monthly_history_raw(monthly_history_record_t *history, size_t max_entries);
esp_err_t meter_storage_set_meter_id(const char *meter_id);
esp_err_t meter_storage_set_meter_type(uint8_t meter_type);
esp_err_t meter_storage_set_volume_multiplier(uint8_t mult);
esp_err_t meter_storage_set_install_date(uint8_t day, uint8_t month, uint16_t year);
esp_err_t meter_storage_set_install_counter(uint16_t counter);
esp_err_t meter_storage_set_battery_level(uint8_t battery_level);
esp_err_t meter_storage_set_timezone_min(int32_t tz_min);
esp_err_t meter_storage_set_last_epoch(int64_t epoch);
uint32_t meter_storage_get_total(uint32_t block_size);

#ifdef __cplusplus
}
#endif

#endif /* METER_STORAGE_H */