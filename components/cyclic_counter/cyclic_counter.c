/* cyclic_counter.c */

#include "cyclic_counter.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"

static const char *TAG = "cyclic_counter";

/* =========================================================
 * RTC STATE 
 * ========================================================= */

RTC_DATA_ATTR static volatile cyclic_state_t s_last_state  = CYCLE_NONE;
RTC_DATA_ATTR static volatile uint32_t       s_count       = 0;
RTC_DATA_ATTR static volatile int64_t        s_last_us_g1  = 0;
RTC_DATA_ATTR static volatile int64_t        s_last_us_g2  = 0;

/* =========================================================
 * RUNTIME CONTEXT — rebuilt every boot
 * ========================================================= */

static gpio_num_t   s_gpio1       = GPIO_NUM_NC;
static gpio_num_t   s_gpio2       = GPIO_NUM_NC;
static uint32_t     s_debounce_us = 50000;
static portMUX_TYPE s_mux         = portMUX_INITIALIZER_UNLOCKED;

/* =========================================================
 * CORE TRANSITION LOGIC
 *
 * Rules:
 *   CYCLE_NONE -> G1 or G2  : seed state, no count
 *   G1         -> G2        : count++
 *   G2         -> G1        : count++
 *   same       -> same      : ignored (bounce / duplicate)
 *
 * Must be called with s_mux held.
 * Must NOT call any logging — safe for both ISR and task context.
 * ========================================================= */

static void IRAM_ATTR process_edge_locked(cyclic_state_t current)
{
    if (current == CYCLE_NONE) return;

    if (s_last_state == CYCLE_NONE) {
        s_last_state = current;
        return;
    }

    if (current == s_last_state) return;

    /* Only count G1 -> G2 transition */
    if (s_last_state == CYCLE_G1 && current == CYCLE_G2) {
        s_count++;
    }

    s_last_state = current;
}

 static bool IRAM_ATTR debounce_ok(gpio_num_t gpio, int64_t now_us)
 {
     volatile int64_t *last_us =
         (gpio == s_gpio1) ? &s_last_us_g1 : &s_last_us_g2;

     /* CRITICAL FIX: esp_timer_get_time() resets to 0 after Deep Sleep.
      * If now_us is smaller than the RTC-retained last_us, a sleep/boot 
      * cycle occurred. We must accept the edge and sync the timestamp. */
     if (now_us < *last_us) {
         *last_us = now_us;
         return true;
     }

     if ((now_us - *last_us) < (int64_t)s_debounce_us) {
         return false;
     }

     *last_us = now_us;
     return true;
 }


 static void IRAM_ATTR isr_handler(void *arg)
 {
     gpio_num_t fired = (gpio_num_t)(uintptr_t)arg;
     int64_t    now   = esp_timer_get_time();

     if (s_gpio1 == GPIO_NUM_NC || s_gpio2 == GPIO_NUM_NC) {
         return;
     }

     /* 1. Debounce handles mechanical noise and bounces */
     if (!debounce_ok(fired, now)) {
         return;
     }
     cyclic_state_t current =
         (fired == s_gpio1) ? CYCLE_G1 : CYCLE_G2;

     portENTER_CRITICAL_ISR(&s_mux);
     process_edge_locked(current);
     portEXIT_CRITICAL_ISR(&s_mux);
 }
 


void cyclic_counter_process_wakeup(int gpio1, int gpio2)
{
    s_gpio1 = (gpio_num_t)gpio1;
    s_gpio2 = (gpio_num_t)gpio2;

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == 0) {
        /* Cold boot — reset everything */
        portENTER_CRITICAL(&s_mux);
        s_count      = 0;
        s_last_state = CYCLE_NONE;
        s_last_us_g1 = 0;
        s_last_us_g2 = 0;
        portEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "cold boot — state reset");
        return;
    }

    if (cause != ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGI(TAG, "wakeup cause %d — no pulse to process", (int)cause);
        return;
    }

    /*
     * Read BEFORE any gpio_config() — some IDF versions clear
     * this register when GPIOs are reconfigured.
     */
    uint64_t wake_mask = esp_sleep_get_ext1_wakeup_status();

    bool g1_fired = (wake_mask & (1ULL << gpio1)) != 0;
    bool g2_fired = (wake_mask & (1ULL << gpio2)) != 0;


    if (g1_fired && g2_fired && !gpio_get_level(GPIO_NUM_12)) {
        /* Both low simultaneously — ambiguous, keep existing state */
        ESP_LOGW(TAG, "both GPIOs low — ambiguous, state kept");
        return;
    }

    cyclic_state_t current = CYCLE_NONE;

    if (g1_fired) {
        current = CYCLE_G1;
    } else if (g2_fired) {
        current = CYCLE_G2;
    } else {
        ESP_LOGI(TAG, "EXT1 wakeup but wake_up with tamper key");
        return;
    }

    portENTER_CRITICAL(&s_mux);
    process_edge_locked(current);
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "wakeup pulse processed — count=%" PRIu32, (uint32_t)s_count);
}

/* =========================================================
 * INIT
 * ========================================================= */

esp_err_t cyclic_counter_init(const cyclic_counter_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    s_gpio1       = (gpio_num_t)cfg->gpio1;
    s_gpio2       = (gpio_num_t)cfg->gpio2;
    s_debounce_us = (cfg->debounce_ms == 0 ? 50u : cfg->debounce_ms) * 1000U;

    gpio_config_t io = {
        .pin_bit_mask  = (1ULL << s_gpio1) | (1ULL << s_gpio2),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,  /* falling edge only */
    };

	esp_err_t ret = gpio_config(&io);
	if (ret != ESP_OK) {
	    ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
	    return ret;
	}

	    /* THE CRITICAL FIX: Tell the OS to put this interrupt in IRAM */
	ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
	     ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
	     return ret;
	}

    ret = gpio_isr_handler_add(s_gpio1, isr_handler, (void *)(uintptr_t)s_gpio1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "isr_handler_add gpio1 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(s_gpio2, isr_handler, (void *)(uintptr_t)s_gpio2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "isr_handler_add gpio2 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG,
             "init OK — GPIO%d GPIO%d debounce=%lums count=%" PRIu32,
             cfg->gpio1, cfg->gpio2,
             (unsigned long)cfg->debounce_ms,
             (uint32_t)s_count);

    return ESP_OK;
}

/* =========================================================
 * PREPARE SLEEP
 * ========================================================= */

void cyclic_counter_prepare_sleep(gpio_num_t gpio1, gpio_num_t gpio2)
{
    gpio_isr_handler_remove(gpio1);
    gpio_isr_handler_remove(gpio2);

    esp_err_t ret = esp_sleep_enable_ext1_wakeup_io(
        (1ULL << gpio1) | (1ULL << gpio2),
        ESP_EXT1_WAKEUP_ANY_LOW);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ext1 wakeup arm failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "sleep armed — GPIO%d GPIO%d count=%" PRIu32
                      " last_state=%d",
                 (int)gpio1, (int)gpio2,
                 (uint32_t)s_count,
                 (int)s_last_state);
    }
}

/* =========================================================
 * GETTERS / RESET
 * ========================================================= */

uint32_t cyclic_counter_get_count(void)
{
    uint32_t v;
    portENTER_CRITICAL(&s_mux);
    v = (uint32_t)s_count;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

cyclic_state_t cyclic_counter_get_last_state(void)
{
    cyclic_state_t v;
    portENTER_CRITICAL(&s_mux);
    v = s_last_state;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool cyclic_counter_half_cycle_pending(void)
{
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = (s_last_state != CYCLE_NONE);
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void cyclic_counter_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    s_count      = 0;
    s_last_state = CYCLE_NONE;
    s_last_us_g1 = 0;
    s_last_us_g2 = 0;
    portEXIT_CRITICAL(&s_mux);
}