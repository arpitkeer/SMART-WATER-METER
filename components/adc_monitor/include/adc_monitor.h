#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include "esp_err.h"

esp_err_t adc_monitor_init(void);

// Add this line below:
esp_err_t adc_monitor_sample(
    int *raw_min, int *raw_max, int *raw_avg,
    int *mv_min, int *mv_max, int *mv_avg);

esp_err_t adc_monitor_get_voltage(int *mv_avg);
void adc_monitor_log(void);

#endif