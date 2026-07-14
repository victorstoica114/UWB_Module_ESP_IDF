#include "i2c_bus_service.h"

#include "app_config.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "i2c_bus_service";

enum {
    APP_I2C_PORT = 0,
    BACKGROUND_LOCK_WAIT_TICKS = 1,
};

static bool s_initialized;
static i2c_master_bus_handle_t s_bus;
static SemaphoreHandle_t s_mutex;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_realtime_waiters;
static uint32_t s_realtime_period_us;
static int64_t s_last_realtime_activity_us;
static uint32_t s_realtime_lock_count;
static uint32_t s_background_lock_count;
static uint32_t s_background_deferred_count;

static void realtime_waiter_add(void)
{
    taskENTER_CRITICAL(&s_state_mux);
    s_realtime_waiters++;
    taskEXIT_CRITICAL(&s_state_mux);
}

static void realtime_waiter_remove(void)
{
    taskENTER_CRITICAL(&s_state_mux);
    if (s_realtime_waiters > 0) {
        s_realtime_waiters--;
    }
    taskEXIT_CRITICAL(&s_state_mux);
}

static void counter_increment(uint32_t *counter)
{
    taskENTER_CRITICAL(&s_state_mux);
    (*counter)++;
    taskEXIT_CRITICAL(&s_state_mux);
}

static void realtime_state_snapshot(uint32_t *waiters, uint32_t *period_us,
                                    int64_t *last_activity_us)
{
    taskENTER_CRITICAL(&s_state_mux);
    if (waiters != NULL) {
        *waiters = s_realtime_waiters;
    }
    if (period_us != NULL) {
        *period_us = s_realtime_period_us;
    }
    if (last_activity_us != NULL) {
        *last_activity_us = s_last_realtime_activity_us;
    }
    taskEXIT_CRITICAL(&s_state_mux);
}

static uint32_t realtime_waiter_count(void)
{
    uint32_t count = 0;
    realtime_state_snapshot(&count, NULL, NULL);
    return count;
}

static int32_t realtime_time_to_next_us_from(int64_t now_us, uint32_t period_us,
                                             int64_t last_activity_us)
{
    if (period_us == 0 || last_activity_us == 0) {
        return -1;
    }

    const int64_t elapsed_us = now_us - last_activity_us;
    if (elapsed_us < 0) {
        return (int32_t)period_us;
    }
    if (elapsed_us >= (int64_t)period_us) {
        return 0;
    }
    return (int32_t)((int64_t)period_us - elapsed_us);
}

static int32_t realtime_background_window_us_from(int64_t now_us,
                                                  uint32_t period_us,
                                                  int64_t last_activity_us)
{
    if (period_us == 0 || last_activity_us == 0) {
        return INT32_MAX;
    }

    const int64_t elapsed_us = now_us - last_activity_us;
    if (elapsed_us < 0) {
        return 0;
    }

    const int64_t stale_us =
        (int64_t)period_us * (int64_t)APP_I2C_REALTIME_STALE_PERIODS;
    if (stale_us > 0 && elapsed_us >= stale_us) {
        return INT32_MAX;
    }

    if (elapsed_us >= (int64_t)period_us) {
        return 0;
    }

    const int64_t time_to_next_us = (int64_t)period_us - elapsed_us;
    const int64_t window_us =
        time_to_next_us - (int64_t)APP_I2C_BACKGROUND_GUARD_US;
    if (window_us <= 0) {
        return 0;
    }
    if (window_us > INT32_MAX) {
        return INT32_MAX;
    }
    return (int32_t)window_us;
}

static int32_t realtime_background_window_us(void)
{
    uint32_t period_us = 0;
    int64_t last_activity_us = 0;
    realtime_state_snapshot(NULL, &period_us, &last_activity_us);
    return realtime_background_window_us_from(esp_timer_get_time(), period_us,
                                             last_activity_us);
}

static bool realtime_background_window_open_for(uint32_t estimated_transfer_us)
{
    const int32_t window_us = realtime_background_window_us();
    if (window_us == INT32_MAX) {
        return true;
    }
    return window_us > 0 && (estimated_transfer_us == 0 ||
                             (uint32_t)window_us >= estimated_transfer_us);
}

static bool timeout_elapsed(TickType_t start_tick, TickType_t timeout)
{
    return timeout != portMAX_DELAY &&
           (xTaskGetTickCount() - start_tick) >= timeout;
}

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
    return i2c_bus_service_lock_background(timeout);
}

bool i2c_bus_service_lock_realtime(TickType_t timeout)
{
    if (s_mutex == NULL) {
        return false;
    }

    realtime_waiter_add();
    const bool locked = xSemaphoreTake(s_mutex, timeout) == pdTRUE;
    realtime_waiter_remove();
    if (locked) {
        counter_increment(&s_realtime_lock_count);
    }
    return locked;
}

bool i2c_bus_service_lock_background(TickType_t timeout)
{
    return i2c_bus_service_lock_background_for(timeout, 0);
}

bool i2c_bus_service_lock_background_for(TickType_t timeout,
                                         uint32_t estimated_transfer_us)
{
    if (s_mutex == NULL) {
        return false;
    }

    const TickType_t start_tick = xTaskGetTickCount();
    while (true) {
        if (realtime_waiter_count() == 0 &&
            realtime_background_window_open_for(estimated_transfer_us) &&
            xSemaphoreTake(s_mutex, BACKGROUND_LOCK_WAIT_TICKS) == pdTRUE) {
            if (realtime_waiter_count() == 0 &&
                realtime_background_window_open_for(estimated_transfer_us)) {
                counter_increment(&s_background_lock_count);
                return true;
            }
            xSemaphoreGive(s_mutex);
            counter_increment(&s_background_deferred_count);
        } else {
            counter_increment(&s_background_deferred_count);
        }

        if (timeout == 0 || timeout_elapsed(start_tick, timeout)) {
            return false;
        }

        taskYIELD();
    }
}

void i2c_bus_service_unlock(void)
{
    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}

void i2c_bus_service_set_realtime_period_us(uint32_t period_us)
{
    const int64_t now_us = period_us > 0 ? esp_timer_get_time() : 0;
    taskENTER_CRITICAL(&s_state_mux);
    s_realtime_period_us = period_us;
    s_last_realtime_activity_us = now_us;
    taskEXIT_CRITICAL(&s_state_mux);
}

void i2c_bus_service_note_realtime_activity(void)
{
    const int64_t now_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_state_mux);
    if (s_realtime_period_us > 0) {
        s_last_realtime_activity_us = now_us;
    }
    taskEXIT_CRITICAL(&s_state_mux);
}

int32_t i2c_bus_service_background_window_us(void)
{
    return realtime_background_window_us();
}

void i2c_bus_service_get_stats(i2c_bus_service_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_state_mux);
    stats->realtime_waiters = s_realtime_waiters;
    stats->realtime_period_us = s_realtime_period_us;
    stats->realtime_lock_count = s_realtime_lock_count;
    stats->background_lock_count = s_background_lock_count;
    stats->background_deferred_count = s_background_deferred_count;
    const int64_t last_activity_us = s_last_realtime_activity_us;
    taskEXIT_CRITICAL(&s_state_mux);

    stats->realtime_time_to_next_us =
        realtime_time_to_next_us_from(esp_timer_get_time(),
                                      stats->realtime_period_us,
                                      last_activity_us);
    stats->background_window_us =
        realtime_background_window_us_from(esp_timer_get_time(),
                                           stats->realtime_period_us,
                                           last_activity_us);
}
