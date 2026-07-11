#include "i2c_bus_service.h"

#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c_bus_service";

enum {
    APP_I2C_PORT = 0,
};

static bool s_initialized;
static i2c_master_bus_handle_t s_bus;
static SemaphoreHandle_t s_mutex;

esp_err_t i2c_bus_service_get(i2c_master_bus_handle_t *bus)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized && s_bus != NULL) {
        *bus = s_bus;
        return ESP_OK;
    }

    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = i2c_master_get_bus_handle(APP_I2C_PORT, &s_bus);
    if (err == ESP_ERR_INVALID_STATE) {
        const i2c_master_bus_config_t bus_config = {
            .i2c_port = APP_I2C_PORT,
            .sda_io_num = BOARD_CONFIG_I2C_SDA_GPIO,
            .scl_io_num = BOARD_CONFIG_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = true,
            },
        };
        err = i2c_new_master_bus(&bus_config, &s_bus);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C bus initialized: port=%d SDA=%d SCL=%d",
                     APP_I2C_PORT, BOARD_CONFIG_I2C_SDA_GPIO,
                     BOARD_CONFIG_I2C_SCL_GPIO);
        }
    }
    ESP_RETURN_ON_ERROR(err, TAG, "I2C bus get/init failed");

    s_initialized = true;
    *bus = s_bus;
    return ESP_OK;
}

bool i2c_bus_service_lock(TickType_t timeout)
{
    return s_mutex != NULL && xSemaphoreTake(s_mutex, timeout) == pdTRUE;
}

void i2c_bus_service_unlock(void)
{
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}
