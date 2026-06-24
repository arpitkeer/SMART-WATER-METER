#include "tamper.h"
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "../lora_manager/include/lora_manager.h"
extern bool s_mag_ble_enabled;
extern bool mag_ble_window_open; 

static gpio_num_t s_master_gpio = GPIO_NUM_NC;
static gpio_num_t s_mag_gpio    = GPIO_NUM_NC;
static gpio_num_t s_rem_gpio    = GPIO_NUM_NC;

static bool s_case_latched = false;
static bool s_mag_latched  = false;
static bool s_rem_latched  = false;

/* Master Pin Debounce State */
static bool s_master_last_sample_active = false;
static int64_t s_master_stable_since_us = 0;

static uint64_t s_stable_time_us = 50000ULL;
static bool s_inited = false;

static void config_input_gpio(gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) {
        return;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, /* CRITICAL: Keep enabled during RUN state */
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io);
}
void tamper_init(const tamper_config_t *cfg)
{
    if (!cfg) {
        return;
    }

    s_master_gpio = cfg->gpio;             /* GPIO 12 (Master OR-Gate) */
    s_mag_gpio    = cfg->magnetic_gpio;    /* Sub-pin for Magnetic */
    s_rem_gpio    = cfg->removal_gpio;     /* Sub-pin for Removal */

    config_input_gpio(s_master_gpio);
    config_input_gpio(s_mag_gpio);
    config_input_gpio(s_rem_gpio);

    s_stable_time_us = (cfg->stable_time_us > 0) ? cfg->stable_time_us : 300000ULL;
    s_inited = true;

    printf(
        "\n[TAMPER]\n"
        "[TAMPER] Init complete (Master OR-Gate Mode)\n"
        "[TAMPER] Stable time = %llu us\n\n",
        (unsigned long long)s_stable_time_us);
}

void tamper_update(int64_t now_us)
{
    if (!s_inited || s_master_gpio == GPIO_NUM_NC) {
        return;
    }

    /* All tampers are active HIGH */
    bool master_active = (gpio_get_level(s_master_gpio) == 1);

    if (master_active != s_master_last_sample_active) {
        s_master_last_sample_active = master_active;
        s_master_stable_since_us = now_us;
    }

    /* Wait for the Master pin to stabilize before interrogating the sub-pins */
    if (master_active && (now_us - s_master_stable_since_us) >= (int64_t)s_stable_time_us)
    {
        bool mag_active = (s_mag_gpio != GPIO_NUM_NC) && (gpio_get_level(s_mag_gpio) == 1);
        bool rem_active = (s_rem_gpio != GPIO_NUM_NC) && (gpio_get_level(s_rem_gpio) == 1);

        /* 1. Classify Magnetic */
        if (mag_active && !s_mag_latched)
        {
            s_mag_latched = true;
            if (s_mag_ble_enabled) {
                mag_ble_window_open = true;
            }
			first_mag_tamper++;
            printf("\n[TAMPER]\n[TAMPER] MAGNETIC DETECTED\n\n");
        }

        /* 2. Classify Removal */
        if (rem_active && !s_rem_latched)
        {
            s_rem_latched = true;
            printf("\n[TAMPER]\n[TAMPER] REMOVAL DETECTED\n\n");
        }

        /* 3. Classify Case (Master is HIGH, but neither sub-pin claimed it) */
        if (!mag_active && !rem_active && !s_case_latched)
        {
            s_case_latched = true;
            printf("\n[TAMPER]\n[TAMPER] CASE DETECTED\n\n");
        }
    }
}

bool tamper_case_is_triggered(void)     { return s_case_latched; }
bool tamper_magnetic_is_triggered(void) { return s_mag_latched; }
bool tamper_removal_is_triggered(void)  { return s_rem_latched; }

void tamper_case_reset(void)     { s_case_latched = false; }
void tamper_magnetic_reset(void) { 
	s_mag_latched = false; 
	first_mag_tamper = 0;
}
void tamper_removal_reset(void)  { s_rem_latched = false; }

bool tamper_any_triggered(void)
{
    return s_case_latched || s_mag_latched || s_rem_latched;
}

void tamper_reset(void)
{
    tamper_case_reset();
    tamper_magnetic_reset();
    tamper_removal_reset();

    /* Force the debounce tracker to re-evaluate the pins if the magnet/case is still active */
    s_master_last_sample_active = false;
    s_master_stable_since_us = 0;

    printf("\n[TAMPER]\n[TAMPER] ALL RESET\n\n");
}

/* Legacy alias for your existing main.c */
bool tamper_is_triggered(void)
{
    return tamper_case_is_triggered();
}