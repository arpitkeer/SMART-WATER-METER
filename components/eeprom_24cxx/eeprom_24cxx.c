#include "eeprom_24cxx.h"

#include <string.h>

#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "soc/gpio_num.h"
#include "esp_timer.h"

static const char *TAG = "eeprom_24cxx";

/* =========================================================
 * EEPROM POWER GPIO
 * ========================================================= */

#ifndef CONFIG_WM_EEPROM_POWER_GPIO
#define CONFIG_WM_EEPROM_POWER_GPIO GPIO_NUM_NC
#endif

static gpio_num_t s_power_gpio = GPIO_NUM_22;
static bool s_powered = false;

/* =========================================================
 * INTERNAL STATE
 * ========================================================= */

static eeprom_24cxx_config_t s_cfg;
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static bool s_inited = false;

/* =========================================================
 * SERIAL ACCESS GUARD
 * ========================================================= */

static SemaphoreHandle_t s_lock = NULL;
static int64_t s_last_write_us = 0;

#define EEPROM_POST_WRITE_GAP_US    2000ULL

static esp_err_t eeprom_lock(void)
{
    if (!s_lock) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(4000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void eeprom_unlock(void)
{
    vTaskDelay(pdMS_TO_TICKS(30));
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

static void eeprom_wait_post_write_gap(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed = now_us - s_last_write_us;

    if (s_last_write_us > 0 && elapsed < (int64_t)EEPROM_POST_WRITE_GAP_US) {
        int64_t wait_us = (int64_t)EEPROM_POST_WRITE_GAP_US - elapsed;
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((wait_us + 999ULL) / 1000ULL)));
    }
}


/* =========================================================
 * POWER CONTROL
 * ========================================================= */

static esp_err_t eeprom_power_gpio_init(void)
{
    s_power_gpio = (gpio_num_t)CONFIG_WM_EEPROM_POWER_GPIO;

    if (s_power_gpio == GPIO_NUM_NC) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_power_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "power gpio config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(s_power_gpio, 0);
    s_powered = false;

    return ESP_OK;
}

static void eeprom_power_on(void)
{
    if (s_powered) {
        return;
    }

    if (s_power_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_power_gpio, 1);
        ESP_LOGI(TAG, "EEPROM power ON  (GPIO %d = 1)", s_power_gpio);
        vTaskDelay(pdMS_TO_TICKS(50));   /* Increased to 50ms to ensure EEPROM stability before I2C access */
    }

    s_powered = true;
}

static void eeprom_power_off(void)
{
    if (!s_powered) {
        return;
    }

    if (s_power_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_power_gpio, 0);
        ESP_LOGI(TAG, "EEPROM power OFF (GPIO %d = 0)", s_power_gpio);
    }

    s_powered = false;
}

/* =========================================================
 * BUILD MEMORY ADDRESS
 * ========================================================= */

static esp_err_t build_mem_addr(
    uint16_t  mem_addr,
    uint8_t  *buf,
    size_t   *addr_len)
{
    if (!buf || !addr_len) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_cfg.addr_bytes == EEPROM_ADDR_1BYTE) {
        if (mem_addr > 0xFF) {
            return ESP_ERR_INVALID_ARG;
        }
        buf[0]    = (uint8_t)(mem_addr & 0xFF);
        *addr_len = 1;
        return ESP_OK;
    }

    if (s_cfg.addr_bytes == EEPROM_ADDR_2BYTE) {
        buf[0]    = (uint8_t)((mem_addr >> 8) & 0xFF);
        buf[1]    = (uint8_t)(mem_addr & 0xFF);
        *addr_len = 2;
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

/* =========================================================
 * EEPROM READY POLLING
 * ========================================================= */

static esp_err_t eeprom_wait_ready(void)
{
    /*
     * EEPROM NACKs while internal write cycle is active.
     */
    for (int i = 0; i < 50; i++) {
        esp_err_t ret = i2c_master_probe(
            s_bus,
            s_cfg.i2c_addr_7bit,
            200);

        if (ret == ESP_OK) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(6));
    }

    return ESP_ERR_TIMEOUT;
}

/* =========================================================
 * INIT
 * ========================================================= */

esp_err_t eeprom_24cxx_init(
    const eeprom_24cxx_config_t *cfg)
{
    if (s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->addr_bytes != EEPROM_ADDR_1BYTE &&
        cfg->addr_bytes != EEPROM_ADDR_2BYTE) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }

    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = eeprom_power_gpio_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* =====================================================
     * I2C BUS
     * ===================================================== */

    i2c_master_bus_config_t bus_cfg = {
        .clk_source              = I2C_CLK_SRC_DEFAULT,
        .i2c_port                = s_cfg.port,
        .sda_io_num              = s_cfg.sda_io,
        .scl_io_num              = s_cfg.scl_io,
        .glitch_ignore_cnt       = 7,
        .flags.enable_internal_pullup = s_cfg.enable_internal_pullups,
    };

    ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* =====================================================
     * I2C DEVICE
     * ===================================================== */

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = s_cfg.i2c_addr_7bit,
        .scl_speed_hz    = s_cfg.scl_speed_hz,
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "EEPROM init OK");
    return ESP_OK;
}

/* =========================================================
 * READ
 * ========================================================= */

esp_err_t eeprom_24cxx_read(
    uint16_t  mem_addr,
    uint8_t  *data,
    size_t    len)
{
    if (!s_inited || !s_dev || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lock_ret = eeprom_lock();
    if (lock_ret != ESP_OK) {
        return lock_ret;
    }

    uint8_t addr_buf[2] = {0};
    size_t  addr_len    = 0;

    esp_err_t ret = build_mem_addr(mem_addr, addr_buf, &addr_len);
    if (ret != ESP_OK) {
        eeprom_unlock();
        return ret;
    }

    eeprom_wait_post_write_gap();
    eeprom_power_on();

    ret = eeprom_wait_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM not ready after power on");
        eeprom_power_off();
        eeprom_unlock();
        return ret;
    }

    ret = i2c_master_transmit_receive(
        s_dev,
        addr_buf,
        addr_len,
        data,
        len,
        -1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read failed: %s", esp_err_to_name(ret));
    }

    eeprom_power_off();
    eeprom_unlock();
    return ret;
}

/* =========================================================
 * WRITE
 * ========================================================= */

esp_err_t eeprom_24cxx_write(
    uint16_t       mem_addr,
    const uint8_t *data,
    size_t         len)
{
    if (!s_inited || !s_dev || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lock_ret = eeprom_lock();
    if (lock_ret != ESP_OK) {
        return lock_ret;
    }

    esp_err_t ret = ESP_OK;

    eeprom_wait_post_write_gap();
    eeprom_power_on();

    ret = eeprom_wait_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM not ready after power on");
        eeprom_power_off();
        eeprom_unlock();
        return ret;
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t tx_buf[3]  = {0};
        size_t  addr_len   = 0;

        ret = build_mem_addr(
            (uint16_t)(mem_addr + i),
            tx_buf,
            &addr_len);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "addr build failed: %s", esp_err_to_name(ret));
            break;
        }

        tx_buf[addr_len] = data[i];

        ret = i2c_master_transmit(
            s_dev,
            tx_buf,
            addr_len + 1,
            100);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "write failed at 0x%04X: %s",
                     (unsigned)(mem_addr + i),
                     esp_err_to_name(ret));
            break;
        }

        /* Wait internal write cycle after every byte */
        ret = eeprom_wait_ready();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM not ready after write");
            break;
        }
    }

    if (ret == ESP_OK) {
        s_last_write_us = esp_timer_get_time();
    }

    eeprom_power_off();
    eeprom_unlock();
    return ret;
}

/* =========================================================
 * DEINIT
 * ========================================================= */

esp_err_t eeprom_24cxx_deinit(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }

    eeprom_power_off();

    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }

    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }

    s_inited = false;

    if (s_lock) {
        xSemaphoreGive(s_lock);
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
    }

    return ESP_OK;
}