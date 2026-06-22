#ifndef TAMPER_H
#define TAMPER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_attr.h"

extern bool mag_ble_window_open;
extern int64_t mag_ble_window_start_us;
extern uint8_t first_mag_tamper;
typedef struct
{
    gpio_num_t gpio;           /* case-open tamper */
    gpio_num_t magnetic_gpio;  /* magnetic tamper */
    gpio_num_t removal_gpio;   /* removal tamper */
    uint64_t stable_time_us;   /* debounce / confirm time */
} tamper_config_t;

void tamper_init(const tamper_config_t *cfg);
void tamper_update(int64_t now_us);

/* Case tamper */
bool tamper_case_is_triggered(void);
void tamper_case_reset(void);

/* Magnetic tamper */
bool tamper_magnetic_is_triggered(void);
void tamper_magnetic_reset(void);

/* Removal tamper */
bool tamper_removal_is_triggered(void);
void tamper_removal_reset(void);

/* Convenience */
bool tamper_any_triggered(void);
void tamper_reset(void);       /* resets all */
bool tamper_is_triggered(void); /* legacy alias = case tamper */

#endif