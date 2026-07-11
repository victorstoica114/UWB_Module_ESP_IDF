#ifndef CHARGER_SERVICE_H
#define CHARGER_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHARGER_SERVICE_REGISTER_MAP_SIZE 0x49U

typedef struct {
    uint8_t reg;
    uint8_t requested_value;
    uint8_t mask;
    uint8_t before;
    uint8_t after;
    bool mask_used;
    bool changed;
    int err;
} charger_service_write_result_t;

typedef struct {
    bool monitor_enabled;
    bool present;
    bool read_ok;
    bool raw_valid;
    bool config_write_supported;
    bool config_writes_enabled;
    int last_error;
    uint32_t read_count;
    uint32_t error_count;
    uint32_t last_update_age_ms;
    int int_gpio_level;
    uint32_t int_irq_count;
    uint32_t int_last_irq_age_ms;
    int pg_gpio_level;
    bool pg_asserted;
    bool pg_stat;
    int qon_gpio_level;
    bool qon_asserted;
    uint8_t part_info;
    uint8_t part_number;
    uint8_t device_revision;
    uint8_t charger_status[5];
    uint8_t fault_status[2];
    uint8_t charger_flag[4];
    uint8_t fault_flag[2];
    uint8_t reg10_charger_control_1;
    uint8_t reg14_charger_control_5;
    uint8_t reg2e_adc_control;
    uint8_t reg2f_adc_disable_0;
    uint8_t reg30_adc_disable_1;
    uint16_t minimal_system_voltage_mv;
    uint16_t charge_voltage_limit_mv;
    uint16_t charge_current_limit_ma;
    uint16_t input_voltage_limit_mv;
    uint16_t input_current_limit_ma;
    uint8_t watchdog_setting;
    bool watchdog_disabled;
    uint8_t adc_sample;
    bool adc_continuous;
    bool adc_running_average;
    bool ibat_discharge_sense_enabled;
    bool adc_enabled;
    int16_t ibus_ma;
    int16_t ibat_ma;
    uint16_t vbus_mv;
    uint16_t vac1_mv;
    uint16_t vac2_mv;
    uint16_t vbat_mv;
    uint16_t vsys_mv;
    bool battery_soc_valid;
    uint8_t battery_soc_percent;
    double ts_percent;
    double tdie_c;
    uint16_t dp_mv;
    uint16_t dm_mv;
    uint32_t write_count;
    uint32_t write_error_count;
    uint32_t last_write_age_ms;
    uint8_t last_write_reg;
    uint8_t last_write_requested_value;
    uint8_t last_write_mask;
    uint8_t last_write_before;
    uint8_t last_write_after;
    bool last_write_mask_used;
    bool last_write_changed;
    int last_write_error;
    uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE];
} charger_service_snapshot_t;

esp_err_t charger_service_start(void);
void charger_service_get_snapshot(charger_service_snapshot_t *snapshot);
void charger_service_format_raw_hex(const charger_service_snapshot_t *snapshot,
                                    char *buffer, size_t buffer_size);
esp_err_t charger_service_read_register(uint8_t reg, uint8_t *value);
esp_err_t charger_service_write_register(
    uint8_t reg, uint8_t value, charger_service_write_result_t *result);
esp_err_t charger_service_update_register_bits(
    uint8_t reg, uint8_t mask, uint8_t value,
    charger_service_write_result_t *result);
esp_err_t charger_service_set_watchdog_disabled(
    charger_service_write_result_t *result);
esp_err_t charger_service_set_adc(bool enabled, bool continuous,
                                  uint8_t sample, bool running_average,
                                  charger_service_write_result_t *results,
                                  size_t result_count,
                                  size_t *written_count);
esp_err_t charger_service_set_minimal_system_voltage_mv(
    uint16_t mv, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count);
esp_err_t charger_service_set_charge_voltage_limit_mv(
    uint16_t mv, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count);
esp_err_t charger_service_set_charge_current_limit_ma(
    uint16_t ma, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count);
esp_err_t charger_service_set_input_voltage_limit_mv(
    uint16_t mv, charger_service_write_result_t *result);
esp_err_t charger_service_set_input_current_limit_ma(
    uint16_t ma, charger_service_write_result_t *results, size_t result_count,
    size_t *written_count);
void charger_service_request_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGER_SERVICE_H */
