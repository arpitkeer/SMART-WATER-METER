#include "watchdog.h"

#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

static const char *TAG = "watchdog";

static bool s_inited = false;

static esp_task_wdt_user_handle_t s_user_handle = NULL;

/* =========================================================
 * RESET REASON
 * ========================================================= */

void watchdog_print_reset_reason(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    printf("\n[WATCHDOG]\n" "[WATCHDOG] Reset reason = %d\n\n", reason);

    switch (reason)
    {
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "POWERON RESET");
            break;

        case ESP_RST_SW:
            ESP_LOGI(TAG, "SOFTWARE RESET");
            break;

        case ESP_RST_PANIC:
            ESP_LOGI(TAG, "PANIC RESET");
            break;

        case ESP_RST_TASK_WDT:
            ESP_LOGI(TAG, "TASK WATCHDOG RESET");
            break;

        case ESP_RST_WDT:
            ESP_LOGI(TAG, "WATCHDOG RESET");
            break;

        case ESP_RST_DEEPSLEEP:
            ESP_LOGI(TAG, "WAKE FROM DEEP SLEEP");
            break;

        default:
            ESP_LOGI(TAG, "OTHER RESET");
            break;
    }
}

/* =========================================================
 * INIT
 * ========================================================= */

esp_err_t watchdog_init(uint32_t timeout_sec)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    if (timeout_sec == 0)
    {
        timeout_sec = 15;
    }

    esp_task_wdt_config_t cfg = {

        .timeout_ms =
            timeout_sec * 1000,

        .idle_core_mask = 0,

        .trigger_panic = true
    };

    esp_err_t ret =
        esp_task_wdt_init(&cfg);

    if (ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(
            TAG,
            "esp_task_wdt_init failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    ret =
        esp_task_wdt_add_user(
            "main_loop",
            &s_user_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "esp_task_wdt_add_user failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    s_inited = true;

    ESP_LOGI(
        TAG,
        "WATCHDOG INIT OK (%lu sec)",
        (unsigned long)timeout_sec);

    return ESP_OK;
}

/* =========================================================
 * FEED
 * ========================================================= */

esp_err_t watchdog_feed(void)
{
    if (!s_inited ||
        !s_user_handle)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_task_wdt_reset_user(
        s_user_handle);
}

/* =========================================================
 * DEINIT
 * ========================================================= */

esp_err_t watchdog_deinit(void)
{
    if (!s_inited)
    {
        return ESP_OK;
    }

    if (s_user_handle)
    {
        esp_err_t ret =
            esp_task_wdt_delete_user(
                s_user_handle);

        if (ret != ESP_OK)
        {
            ESP_LOGW(
                TAG,
                "esp_task_wdt_delete_user failed: %s",
                esp_err_to_name(ret));
        }

        s_user_handle = NULL;
    }

    esp_err_t ret =
        esp_task_wdt_deinit();

    if (ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(
            TAG,
            "esp_task_wdt_deinit failed: %s",
            esp_err_to_name(ret));

        return ret;
    }

    s_inited = false;

    ESP_LOGI(
        TAG,
        "WATCHDOG DEINIT OK");

    return ESP_OK;
}