#include "deep_sleep_manager.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/gpio_types.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/gpio_num.h"

static const char *TAG = "deep_sleep_manager";

/* GPIO10 + GPIO11 + CASE + MAGNETIC + REMOVAL */

#define WAKE_GPIO_A ((gpio_num_t)CONFIG_WM_SENSOR1_GPIO)
#define WAKE_GPIO_B ((gpio_num_t)CONFIG_WM_SENSOR2_GPIO)
#define WAKE_GPIO_C ((gpio_num_t)CONFIG_WM_CASE_OPEN_TAMPER_GPIO)
#define WAKE_GPIO_D ((gpio_num_t)CONFIG_WM_MAGNETIC_TAMPER_GPIO)
#define WAKE_GPIO_E ((gpio_num_t)CONFIG_WM_REMOVAL_TAMPER_GPIO)

static const gpio_num_t s_wake_gpios[] = {
    WAKE_GPIO_A,
    WAKE_GPIO_B,
    WAKE_GPIO_C,
//    WAKE_GPIO_D,
//    WAKE_GPIO_E,
};

#define STABILITY_SAMPLES     5
#define STABILITY_DELAY_MS    20
#define STABILITY_TIMEOUT_MS   500

static void clear_ext1_wakeup(void)
{
    /*
     * Disable all currently configured EXT1 wake IOs.
     * Passing 0 is the safe way used in your current codebase.
     */
    esp_err_t ret = esp_sleep_disable_ext1_wakeup_io(0);

    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "esp_sleep_disable_ext1_wakeup_io failed: %s",
                 esp_err_to_name(ret));
    }
}

static void ensure_gpio_inputs(void)
{
    for (size_t i = 0; i < sizeof(s_wake_gpios) / sizeof(s_wake_gpios[0]); i++)
    {
        gpio_num_t pin = s_wake_gpios[i];

        gpio_config_t io = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        if (pin == WAKE_GPIO_C) {
            io.pull_down_en = GPIO_PULLDOWN_ENABLE;
        }

        esp_err_t ret = gpio_config(&io);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
        }
    }
}
static bool pins_stable(void)
{
    int prev[sizeof(s_wake_gpios) / sizeof(s_wake_gpios[0])];
    memset(prev, -1, sizeof(prev));
	

    int stable_count = 0;
    int waited_ms = 0;

    while (waited_ms < STABILITY_TIMEOUT_MS)
    {
        bool same = true;

        for (size_t i = 0; i < sizeof(s_wake_gpios) / sizeof(s_wake_gpios[0]); i++)
        {
            int level = gpio_get_level(s_wake_gpios[i]);
            if (level != prev[i])
            {
                same = false;
                prev[i] = level;
            }
        }

        if (same)
        {
            stable_count++;
            if (stable_count >= STABILITY_SAMPLES)
            {
                return true;
            }
        }
        else
        {
            stable_count = 1;
        }

        vTaskDelay(pdMS_TO_TICKS(STABILITY_DELAY_MS));
        waited_ms += STABILITY_DELAY_MS;
    }

    return false;
}

static void configure_ext1_for_next_transition(void)
{
    uint64_t high_mask = 0;
    uint64_t low_mask = 0;

    int levels[sizeof(s_wake_gpios) / sizeof(s_wake_gpios[0])];

    for (size_t i = 0; i < sizeof(s_wake_gpios) / sizeof(s_wake_gpios[0]); i++)
    {
        levels[i] = gpio_get_level(s_wake_gpios[i]);

        if (levels[i]) {
            low_mask |= (1ULL << s_wake_gpios[i]);
        } else {
            high_mask |= (1ULL << s_wake_gpios[i]);
        }
    }

    clear_ext1_wakeup();

    if (high_mask) {
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(high_mask, ESP_EXT1_WAKEUP_ANY_HIGH));
    }
    if (low_mask) {
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(low_mask, ESP_EXT1_WAKEUP_ANY_LOW));
    }

    printf(
        "\n[WAKE CONFIG]\n"
        "[WAKE CONFIG] GPIO1=%d GPIO2=%d CASE=%d\n"
        "[WAKE CONFIG] HIGH MASK : 0x%llX\n"
        "[WAKE CONFIG] LOW  MASK : 0x%llX\n\n",
        levels[0], levels[1], levels[2],
        (unsigned long long)high_mask, (unsigned long long)low_mask);
}
static void enter_gpio_only_deep_sleep(void)
{
    ensure_gpio_inputs();

    if (!pins_stable())
    {
        ESP_LOGW(TAG, "GPIOs unstable before sleep, proceeding anyway");
    }

    configure_ext1_for_next_transition();

    printf(
        "\n[SYSTEM]\n"
        "[SYSTEM] ENTERING GPIO-ONLY DEEP SLEEP\n\n");

    vTaskDelay(pdMS_TO_TICKS(50));

    esp_deep_sleep_start();
}

void deep_sleep_enter_for_us(uint64_t sleep_us,
                             bool use_gpio_wake)
{
    ensure_gpio_inputs();

    if (use_gpio_wake)
    {
        configure_ext1_for_next_transition();
    }
    else
    {
        clear_ext1_wakeup();
    }

    if (sleep_us <= 0)
    {
        sleep_us = 3000000ULL;
    }

    ESP_ERROR_CHECK(
        esp_sleep_enable_timer_wakeup(sleep_us));

    printf(
        "\n[SLEEP]\n"
        "Timer wake in %" PRIu64 " sec\n",
        sleep_us / 1000000ULL);

    esp_deep_sleep_start();
}

void deep_sleep_enter(int gpio1, int gpio2, int stuck_gpio)
{
    (void)gpio1;
    (void)gpio2;
    (void)stuck_gpio;

    printf(
        "\n[SLEEP]\n"
        "[SLEEP] SIMPLE GPIO ONLY DEEP SLEEP MODE\n\n");

    enter_gpio_only_deep_sleep();
}

void deep_sleep_handle_wakeup(int gpio1, int gpio2)
{
    (void)gpio1;
    (void)gpio2;

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    printf(
        "\n[WAKEUP]\n"
        "[WAKEUP] Cause = %d\n\n",
        cause);

    switch (cause)
    {
        case ESP_SLEEP_WAKEUP_EXT1:
        {
            uint64_t wake_mask = esp_sleep_get_ext1_wakeup_status();

            printf(
                "\n[WAKEUP]\n"
                "[WAKEUP] EXT1 WAKE DETECTED\n"
                "[WAKEUP] MASK : 0x%llX\n\n",
                (unsigned long long)wake_mask);

            if (wake_mask & (1ULL << WAKE_GPIO_A))
            {
                printf(
                    "\n[WAKEUP]\n"
                    "[WAKEUP] SENSOR1 ACTIVITY\n\n");
            }

            if (wake_mask & (1ULL << WAKE_GPIO_B))
            {
                printf(
                    "\n[WAKEUP]\n"
                    "[WAKEUP] SENSOR2 ACTIVITY\n\n");
            }

            if (wake_mask & (1ULL << WAKE_GPIO_C))
            {
                printf(
                    "\n[WAKEUP]\n"
                    "[WAKEUP] TAMPER ACTIVITY\n\n");
            }

            break;
        }

        case ESP_SLEEP_WAKEUP_TIMER:
        {
            printf(
                "\n[WAKEUP]\n"
                "[WAKEUP] TIMER WAKE DETECTED\n\n");
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:{
			printf("\n[WAKEUP]\n"
			       "[WAKEUP] UNDEFINED WAKE DETECTED\n\n"
			        "%d", ESP_SLEEP_WAKEUP_UNDEFINED);
		}
        default:
        {
            printf(
                "\n[WAKEUP]\n"
                "[WAKEUP] COLD BOOT / POWER ON\n\n");
            break;
        }
    }
}