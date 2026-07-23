#include "i2c_bus_service.h"

#include "app_config.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/i2c_ll.h"
#include <string.h>

static const char *TAG = "i2c_bus_service";

enum {
    APP_I2C_PORT = 0,
    BACKGROUND_LOCK_WAIT_TICKS = 1,
    DIRECT_READ_MAX_BYTES = I2C_LL_FIFO_LEN,
    DIRECT_WRITE_MAX_DATA_BYTES = I2C_LL_FIFO_LEN - 2,
    DIRECT_SOURCE_HZ = 40000000,
    DIRECT_DEFAULT_TIMEOUT_US = 20000,
    DIRECT_HS_ENTRY_CLOCK_HZ = 1000000,
    DIRECT_HS_MASTER_CODE = 0x08,
    DIRECT_HS_MASTER_DONE_MASK = (1UL << 2),
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

static uint32_t direct_command_done_mask(i2c_dev_t *hw)
{
    uint32_t mask = 0;
    for (int i = 0; i < I2C_LL_CMD_REG_NUM; ++i) {
        if (hw->comd[i].command_done) {
            mask |= (1UL << i);
        }
    }
    return mask;
}

static uint32_t direct_wait_raw(i2c_dev_t *hw, int64_t start_us,
                                uint32_t timeout_us)
{
    uint32_t int_raw = 0;
    do {
        int_raw = hw->int_raw.val;
        if ((int_raw & (I2C_LL_INTR_MST_COMPLETE | I2C_LL_INTR_TIMEOUT |
                        I2C_LL_INTR_NACK |
                        I2C_LL_INTR_ARBITRATION)) != 0U) {
            break;
        }
        taskYIELD();
    } while ((uint32_t)(esp_timer_get_time() - start_us) < timeout_us);
    return int_raw;
}

static uint32_t direct_wait_done_mask(i2c_dev_t *hw, int64_t start_us,
                                      uint32_t timeout_us,
                                      uint32_t expected_done_mask,
                                      uint32_t *done_mask)
{
    uint32_t int_raw = 0;
    uint32_t current_done = 0;
    do {
        int_raw = hw->int_raw.val;
        current_done = direct_command_done_mask(hw);
        if ((current_done & expected_done_mask) == expected_done_mask) {
            break;
        }
        if ((int_raw & (I2C_LL_INTR_TIMEOUT |
                        I2C_LL_INTR_ARBITRATION)) != 0U) {
            break;
        }
        taskYIELD();
    } while ((uint32_t)(esp_timer_get_time() - start_us) < timeout_us);
    if (done_mask != NULL) {
        *done_mask = current_done;
    }
    return int_raw;
}

esp_err_t i2c_bus_service_scl_measure_start(
    int gpio, i2c_bus_service_scl_measure_t *measure)
{
    if (measure == NULL || gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *measure = (i2c_bus_service_scl_measure_t){0};

    const pcnt_unit_config_t unit_config = {
        .low_limit = -1,
        .high_limit = 32767,
        .intr_priority = 0,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &measure->unit);
    if (err != ESP_OK) {
        return err;
    }

    const pcnt_chan_config_t chan_config = {
        .edge_gpio_num = gpio,
        .level_gpio_num = -1,
    };
    err = pcnt_new_channel(measure->unit, &chan_config, &measure->channel);
    if (err != ESP_OK) {
        goto fail;
    }

    err = pcnt_channel_set_edge_action(
        measure->channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_channel_set_level_action(
        measure->channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_unit_enable(measure->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    measure->enabled = true;
    err = pcnt_unit_clear_count(measure->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_unit_start(measure->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    measure->started = true;
    return ESP_OK;

fail:
    if (measure->started) {
        (void)pcnt_unit_stop(measure->unit);
    }
    if (measure->enabled) {
        (void)pcnt_unit_disable(measure->unit);
    }
    if (measure->channel != NULL) {
        (void)pcnt_del_channel(measure->channel);
    }
    if (measure->unit != NULL) {
        (void)pcnt_del_unit(measure->unit);
    }
    *measure = (i2c_bus_service_scl_measure_t){0};
    return err;
}

esp_err_t i2c_bus_service_scl_measure_stop(
    i2c_bus_service_scl_measure_t *measure, uint32_t elapsed_us,
    i2c_bus_service_scl_measure_result_t *result)
{
    if (measure == NULL || measure->unit == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    if (measure->started) {
        err = pcnt_unit_stop(measure->unit);
        measure->started = false;
    }

    int count = 0;
    const esp_err_t count_err = pcnt_unit_get_count(measure->unit, &count);
    if (err == ESP_OK) {
        err = count_err;
    }

    if (result != NULL) {
        result->elapsed_us = elapsed_us;
        result->edges = count > 0 ? (uint32_t)count : 0U;
        if (elapsed_us > 0 && count > 0) {
            result->measured_hz =
                (uint32_t)(((uint64_t)count * 1000000ULL +
                            (elapsed_us / 2U)) /
                           elapsed_us);
        }
    }

    if (measure->enabled) {
        const esp_err_t disable_err = pcnt_unit_disable(measure->unit);
        if (err == ESP_OK) {
            err = disable_err;
        }
        measure->enabled = false;
    }
    if (measure->channel != NULL) {
        const esp_err_t channel_err = pcnt_del_channel(measure->channel);
        if (err == ESP_OK) {
            err = channel_err;
        }
        measure->channel = NULL;
    }
    if (measure->unit != NULL) {
        const esp_err_t unit_err = pcnt_del_unit(measure->unit);
        if (err == ESP_OK) {
            err = unit_err;
        }
        measure->unit = NULL;
    }
    return err;
}

static void direct_set_timing(i2c_dev_t *hw,
                              const i2c_bus_service_direct_config_t *config,
                              uint32_t clock_hz)
{
    i2c_hal_clk_config_t clk_cal = {0};
    i2c_ll_master_cal_bus_clk(DIRECT_SOURCE_HZ, clock_hz, &clk_cal);
    i2c_ll_master_set_bus_timing(hw, &clk_cal);
    if (config->manual_timing) {
        hw->scl_low_period.scl_low_period = config->scl_low_period;
        hw->scl_high_period.scl_high_period = config->scl_high_period;
        hw->scl_high_period.scl_wait_high_period =
            config->scl_wait_high_period;
    }
    i2c_ll_master_set_fractional_divider(hw, 0, 0);
}

static void direct_prepare_controller(i2c_dev_t *hw)
{
    i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);
    i2c_ll_set_mode(hw, I2C_BUS_MODE_MASTER);
    i2c_ll_enable_pins_open_drain(hw, true);
    i2c_ll_enable_arbitration(hw, false);
    i2c_ll_set_data_mode(hw, I2C_DATA_MODE_MSB_FIRST,
                         I2C_DATA_MODE_MSB_FIRST);
    i2c_ll_master_rx_full_ack_level(hw, false);
    i2c_ll_enable_controller_clock(hw, true);
    i2c_ll_set_source_clk(hw, I2C_CLK_SRC_XTAL);
    i2c_ll_master_set_filter(hw, 0);
}

static esp_err_t direct_send_hs_master_code(
    i2c_dev_t *hw, const i2c_bus_service_direct_config_t *config,
    i2c_bus_service_direct_result_t *result, uint32_t timeout_us)
{
    const uint32_t entry_clock_hz =
        config->hs_entry_clock_hz != 0 ? config->hs_entry_clock_hz
                                       : DIRECT_HS_ENTRY_CLOCK_HZ;
    i2c_hal_clk_config_t entry_clk = {0};
    i2c_ll_master_cal_bus_clk(DIRECT_SOURCE_HZ, entry_clock_hz, &entry_clk);
    i2c_ll_master_set_bus_timing(hw, &entry_clk);
    i2c_ll_master_set_fractional_divider(hw, 0, 0);
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);

    const uint8_t hs_code = DIRECT_HS_MASTER_CODE;
    i2c_ll_write_txfifo(hw, &hs_code, sizeof(hs_code));

    const i2c_ll_hw_cmd_t restart_cmd = {
        .op_code = I2C_LL_CMD_RESTART,
    };
    const i2c_ll_hw_cmd_t write_cmd = {
        .byte_num = 1,
        .ack_en = 0,
        .ack_exp = 0,
        .op_code = I2C_LL_CMD_WRITE,
    };
    const i2c_ll_hw_cmd_t stop_cmd = {
        .op_code = I2C_LL_CMD_STOP,
    };
    const i2c_ll_hw_cmd_t end_cmd = {
        .op_code = I2C_LL_CMD_END,
    };

    for (int i = 0; i < I2C_LL_CMD_REG_NUM; ++i) {
        i2c_ll_master_write_cmd_reg(hw, end_cmd, i);
    }
    i2c_ll_master_write_cmd_reg(hw, restart_cmd, 0);
    i2c_ll_master_write_cmd_reg(hw, write_cmd, 1);
    if (config->hs_master_stop) {
        i2c_ll_master_write_cmd_reg(hw, stop_cmd, 2);
    }

    const int64_t start_us = esp_timer_get_time();
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    uint32_t done_mask = 0;
    const uint32_t int_raw =
        direct_wait_done_mask(hw, start_us, timeout_us,
                              DIRECT_HS_MASTER_DONE_MASK, &done_mask);
    const int64_t elapsed_raw = esp_timer_get_time() - start_us;
    const uint32_t elapsed_us =
        elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U;

    if (result != NULL) {
        result->hs_master_elapsed_us = elapsed_us;
        result->hs_master_int_raw = int_raw;
        result->hs_master_command_done_mask = done_mask;
    }

    i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);

    if ((done_mask & DIRECT_HS_MASTER_DONE_MASK) == DIRECT_HS_MASTER_DONE_MASK) {
        return ESP_OK;
    }
    if ((int_raw & I2C_LL_INTR_TIMEOUT) != 0U || elapsed_us >= timeout_us) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t i2c_bus_service_direct_read_reg(
    uint8_t address, uint8_t start_reg, uint8_t *data, size_t data_len,
    const i2c_bus_service_direct_config_t *config,
    i2c_bus_service_direct_result_t *result)
{
    if (data == NULL || config == NULL || data_len == 0 ||
        data_len > DIRECT_READ_MAX_BYTES || address > 0x7fU ||
        config->clock_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->manual_timing &&
        (config->scl_low_period > 0x1ffU ||
         config->scl_high_period > 0x1ffU ||
         config->scl_wait_high_period > 0x7fU)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (result != NULL) {
        *result = (i2c_bus_service_direct_result_t){0};
    }

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_bus_service_get(&bus);
    if (err != ESP_OK) {
        return err;
    }
    (void)bus;

    if (!i2c_bus_service_lock_background_for(pdMS_TO_TICKS(1000),
                                             config->estimated_transfer_us)) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_dev_t *hw = I2C_LL_GET_HW(APP_I2C_PORT);
    const uint32_t saved_filter_cfg = hw->filter_cfg.val;
    const uint32_t timeout_us =
        config->timeout_us != 0 ? config->timeout_us : DIRECT_DEFAULT_TIMEOUT_US;

    uint32_t int_raw = 0;
    uint32_t elapsed_us = 0;
    esp_err_t measure_err = ESP_ERR_NOT_SUPPORTED;
    i2c_bus_service_scl_measure_t scl_measure = {0};
    i2c_bus_service_scl_measure_result_t scl_measure_result = {0};
    direct_prepare_controller(hw);

    if (config->hs_master_code) {
        err = direct_send_hs_master_code(hw, config, result, timeout_us);
        if (err != ESP_OK) {
            goto cleanup;
        }
    }

    direct_set_timing(hw, config, config->clock_hz);

    const uint8_t tx_data[] = {
        (uint8_t)(address << 1U),
        start_reg,
        (uint8_t)((address << 1U) | 0x01U),
    };

    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_write_txfifo(hw, tx_data, sizeof(tx_data));

    const i2c_ll_hw_cmd_t restart_cmd = {
        .op_code = I2C_LL_CMD_RESTART,
    };
    const i2c_ll_hw_cmd_t write_reg_cmd = {
        .byte_num = 2,
        .ack_en = 1,
        .ack_exp = 0,
        .op_code = I2C_LL_CMD_WRITE,
    };
    const i2c_ll_hw_cmd_t write_addr_read_cmd = {
        .byte_num = 1,
        .ack_en = 1,
        .ack_exp = 0,
        .op_code = I2C_LL_CMD_WRITE,
    };
    const i2c_ll_hw_cmd_t read_ack_cmd = {
        .byte_num = (uint32_t)(data_len > 1 ? data_len - 1U : 0U),
        .ack_val = I2C_ACK_VAL,
        .op_code = I2C_LL_CMD_READ,
    };
    const i2c_ll_hw_cmd_t read_nack_cmd = {
        .byte_num = 1,
        .ack_val = I2C_NACK_VAL,
        .op_code = I2C_LL_CMD_READ,
    };
    const i2c_ll_hw_cmd_t stop_cmd = {
        .op_code = I2C_LL_CMD_STOP,
    };
    const i2c_ll_hw_cmd_t end_cmd = {
        .op_code = I2C_LL_CMD_END,
    };

    for (int i = 0; i < I2C_LL_CMD_REG_NUM; ++i) {
        i2c_ll_master_write_cmd_reg(hw, end_cmd, i);
    }
    i2c_ll_master_write_cmd_reg(hw, restart_cmd, 0);
    i2c_ll_master_write_cmd_reg(hw, write_reg_cmd, 1);
    i2c_ll_master_write_cmd_reg(hw, restart_cmd, 2);
    i2c_ll_master_write_cmd_reg(hw, write_addr_read_cmd, 3);
    if (data_len > 1U) {
        i2c_ll_master_write_cmd_reg(hw, read_ack_cmd, 4);
        i2c_ll_master_write_cmd_reg(hw, read_nack_cmd, 5);
        i2c_ll_master_write_cmd_reg(hw, stop_cmd, 6);
    } else {
        i2c_ll_master_write_cmd_reg(hw, read_nack_cmd, 4);
        i2c_ll_master_write_cmd_reg(hw, stop_cmd, 5);
    }

    if (config->measure_scl_gpio >= 0) {
        measure_err =
            i2c_bus_service_scl_measure_start(config->measure_scl_gpio,
                                              &scl_measure);
    }

    const int64_t start_us = esp_timer_get_time();
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);
    int_raw = direct_wait_raw(hw, start_us, timeout_us);

    const int64_t elapsed_raw = esp_timer_get_time() - start_us;
    elapsed_us = elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U;

    if (scl_measure.unit != NULL) {
        measure_err = i2c_bus_service_scl_measure_stop(
            &scl_measure, elapsed_us, &scl_measure_result);
        if (result != NULL) {
            result->scl_measure_elapsed_us = scl_measure_result.elapsed_us;
            result->scl_measure_edges = scl_measure_result.edges;
            result->scl_measure_hz = scl_measure_result.measured_hz;
        }
    }

    const uint32_t rx_count = hw->sr.rxfifo_cnt;
    if (rx_count >= data_len) {
        i2c_ll_read_rxfifo(hw, data, data_len);
    }

    if ((int_raw & I2C_LL_INTR_MST_COMPLETE) != 0U &&
        rx_count >= data_len) {
        err = ESP_OK;
    } else if ((int_raw & I2C_LL_INTR_TIMEOUT) != 0U ||
               elapsed_us >= timeout_us) {
        err = ESP_ERR_TIMEOUT;
    } else if ((int_raw & I2C_LL_INTR_NACK) != 0U ||
               (int_raw & I2C_LL_INTR_ARBITRATION) != 0U) {
        err = ESP_ERR_INVALID_RESPONSE;
    } else {
        err = ESP_ERR_INVALID_STATE;
    }

cleanup:
    if (result != NULL) {
        result->elapsed_us = elapsed_us;
        result->int_raw = int_raw;
        result->command_done_mask = direct_command_done_mask(hw);
        result->scl_measure_error = measure_err;
    }
    i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);
    hw->filter_cfg.val = saved_filter_cfg;
    i2c_bus_service_unlock();
    return err;
}

esp_err_t i2c_bus_service_direct_write_reg(
    uint8_t address, uint8_t start_reg, const uint8_t *data, size_t data_len,
    const i2c_bus_service_direct_config_t *config,
    i2c_bus_service_direct_result_t *result)
{
    if (data == NULL || config == NULL || data_len == 0 ||
        data_len > DIRECT_WRITE_MAX_DATA_BYTES || address > 0x7fU ||
        config->clock_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->manual_timing &&
        (config->scl_low_period > 0x1ffU ||
         config->scl_high_period > 0x1ffU ||
         config->scl_wait_high_period > 0x7fU)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (result != NULL) {
        *result = (i2c_bus_service_direct_result_t){0};
    }

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_bus_service_get(&bus);
    if (err != ESP_OK) {
        return err;
    }
    (void)bus;

    if (!i2c_bus_service_lock_background_for(pdMS_TO_TICKS(1000),
                                             config->estimated_transfer_us)) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_dev_t *hw = I2C_LL_GET_HW(APP_I2C_PORT);
    const uint32_t saved_filter_cfg = hw->filter_cfg.val;
    const uint32_t timeout_us =
        config->timeout_us != 0 ? config->timeout_us : DIRECT_DEFAULT_TIMEOUT_US;

    uint32_t int_raw = 0;
    uint32_t elapsed_us = 0;
    esp_err_t measure_err = ESP_ERR_NOT_SUPPORTED;
    i2c_bus_service_scl_measure_t scl_measure = {0};
    i2c_bus_service_scl_measure_result_t scl_measure_result = {0};
    direct_prepare_controller(hw);

    if (config->hs_master_code) {
        err = direct_send_hs_master_code(hw, config, result, timeout_us);
        if (err != ESP_OK) {
            goto cleanup;
        }
    }

    direct_set_timing(hw, config, config->clock_hz);

    uint8_t tx_data[I2C_LL_FIFO_LEN] = {0};
    const size_t tx_len = data_len + 2U;
    tx_data[0] = (uint8_t)(address << 1U);
    tx_data[1] = start_reg;
    memcpy(&tx_data[2], data, data_len);

    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_write_txfifo(hw, tx_data, tx_len);

    const i2c_ll_hw_cmd_t restart_cmd = {
        .op_code = I2C_LL_CMD_RESTART,
    };
    const i2c_ll_hw_cmd_t write_cmd = {
        .byte_num = (uint32_t)tx_len,
        .ack_en = 1,
        .ack_exp = 0,
        .op_code = I2C_LL_CMD_WRITE,
    };
    const i2c_ll_hw_cmd_t stop_cmd = {
        .op_code = I2C_LL_CMD_STOP,
    };
    const i2c_ll_hw_cmd_t end_cmd = {
        .op_code = I2C_LL_CMD_END,
    };

    for (int i = 0; i < I2C_LL_CMD_REG_NUM; ++i) {
        i2c_ll_master_write_cmd_reg(hw, end_cmd, i);
    }
    i2c_ll_master_write_cmd_reg(hw, restart_cmd, 0);
    i2c_ll_master_write_cmd_reg(hw, write_cmd, 1);
    i2c_ll_master_write_cmd_reg(hw, stop_cmd, 2);

    if (config->measure_scl_gpio >= 0) {
        measure_err =
            i2c_bus_service_scl_measure_start(config->measure_scl_gpio,
                                              &scl_measure);
    }

    const int64_t start_us = esp_timer_get_time();
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);
    int_raw = direct_wait_raw(hw, start_us, timeout_us);

    const int64_t elapsed_raw = esp_timer_get_time() - start_us;
    elapsed_us = elapsed_raw > 0 ? (uint32_t)elapsed_raw : 0U;

    if (scl_measure.unit != NULL) {
        measure_err = i2c_bus_service_scl_measure_stop(
            &scl_measure, elapsed_us, &scl_measure_result);
        if (result != NULL) {
            result->scl_measure_elapsed_us = scl_measure_result.elapsed_us;
            result->scl_measure_edges = scl_measure_result.edges;
            result->scl_measure_hz = scl_measure_result.measured_hz;
        }
    }

    if ((int_raw & I2C_LL_INTR_MST_COMPLETE) != 0U) {
        err = ESP_OK;
    } else if ((int_raw & I2C_LL_INTR_TIMEOUT) != 0U ||
               elapsed_us >= timeout_us) {
        err = ESP_ERR_TIMEOUT;
    } else if ((int_raw & I2C_LL_INTR_NACK) != 0U ||
               (int_raw & I2C_LL_INTR_ARBITRATION) != 0U) {
        err = ESP_ERR_INVALID_RESPONSE;
    } else {
        err = ESP_ERR_INVALID_STATE;
    }

cleanup:
    if (result != NULL) {
        result->elapsed_us = elapsed_us;
        result->int_raw = int_raw;
        result->command_done_mask = direct_command_done_mask(hw);
        result->scl_measure_error = measure_err;
    }
    i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);
    hw->filter_cfg.val = saved_filter_cfg;
    i2c_bus_service_unlock();
    return err;
}
