#include "adc_monitor.h"
#include <stdio.h>
#include <limits.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ADC_MONITOR_UNIT        ADC_UNIT_1
#define ADC_MONITOR_CHAN        ADC_CHANNEL_4
#define ADC_MONITOR_ATTEN       ADC_ATTEN_DB_12
#define ADC_SAMPLE_COUNT        32

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_calibrated = false;
static const char *TAG = "ADC_MONITOR";

esp_err_t adc_monitor_init(void)
{
    if (s_adc_handle != NULL) return ESP_OK;

    // 1. Initialize Unit
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_MONITOR_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

    // 2. Configure Channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_MONITOR_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_MONITOR_CHAN, &config));

    // 3. Curve Fitting Calibration (Required for ESP32-H2)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_MONITOR_UNIT,
        .chan = ADC_MONITOR_CHAN,
        .atten = ADC_MONITOR_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle) == ESP_OK) {
        s_calibrated = true;
        ESP_LOGI(TAG, "Calibration initialized successfully");
    }

    return ESP_OK;
}

esp_err_t adc_monitor_sample(
    int *raw_min, int *raw_max, int *raw_avg,
    int *mv_min, int *mv_max, int *mv_avg)
{
    if (!s_adc_handle) return ESP_ERR_INVALID_STATE;

    int r_min = INT_MAX, r_max = 0;
    int m_min = INT_MAX, m_max = 0;
    long long r_sum = 0, m_sum = 0;

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        int raw_val = 0;
        int mv_val = 0;

        adc_oneshot_read(s_adc_handle, ADC_MONITOR_CHAN, &raw_val);

        if (s_calibrated) {
            adc_cali_raw_to_voltage(s_cali_handle, raw_val, &mv_val);
        } else {
            mv_val = raw_val; // Fallback
        }

        // Track raw limits
        if (raw_val < r_min) r_min = raw_val;
        if (raw_val > r_max) r_max = raw_val;
        r_sum += raw_val;

        // Track voltage limits
        if (mv_val < m_min) m_min = mv_val;
        if (mv_val > m_max) m_max = mv_val;
        m_sum += mv_val;
    }

    if (raw_min) *raw_min = r_min;
    if (raw_max) *raw_max = r_max;
    if (raw_avg) *raw_avg = (int)(r_sum / ADC_SAMPLE_COUNT);

    if (mv_min)  *mv_min = m_min;
    if (mv_max)  *mv_max = m_max;
    if (mv_avg)  *mv_avg = (int)(m_sum / ADC_SAMPLE_COUNT);

    return ESP_OK;
}

esp_err_t adc_monitor_get_voltage(int *mv_avg)
{
    return adc_monitor_sample(NULL, NULL, NULL, NULL, NULL, mv_avg);
}

void adc_monitor_log(void)
{
    int m_avg = 0;
    if (adc_monitor_get_voltage(&m_avg) == ESP_OK) {
        printf("[ADC] Pin: %d mV | Battery: %d mV\n", m_avg, m_avg * 2);
    }
}