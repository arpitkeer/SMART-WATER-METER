/* cyclic_counter.h */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CYCLE_NONE = 0,
    CYCLE_G1   = 1,
    CYCLE_G2   = 2,
} cyclic_state_t;

typedef struct {
    int      gpio1;
    int      gpio2;
    uint32_t debounce_ms;
} cyclic_counter_config_t;

/**
 * Call FIRST in app_main(), before any gpio_config()
 * or peripheral init.
 */
void cyclic_counter_process_wakeup(int gpio1, int gpio2);

/**
 * Initialise GPIO inputs and install ISR handlers.
 * Call after cyclic_counter_process_wakeup().
 */
esp_err_t cyclic_counter_init(const cyclic_counter_config_t *cfg);

/**
 * Call just before esp_deep_sleep_start().
 * Removes ISR handlers and arms EXT1 wakeup on both GPIOs.
 */
void cyclic_counter_prepare_sleep(gpio_num_t gpio1, gpio_num_t gpio2);

/**
 * Current pulse count.
 */
uint32_t cyclic_counter_get_count(void);

/**
 * Last confirmed sensor state.
 */
cyclic_state_t cyclic_counter_get_last_state(void);

/**
 * True if one half of a cycle has been seen.
 */
bool cyclic_counter_half_cycle_pending(void);

/**
 * Reset count and state to zero.
 */
void cyclic_counter_reset(void);

#ifdef __cplusplus
}
#endif