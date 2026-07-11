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
    bool adc_enabled;
    int16_t ibus_ma;
    int16_t ibat_ma;
    uint16_t vbus_mv;
    uint16_t vac1_mv;
    uint16_t vac2_mv;
    uint16_t vbat_mv;
    uint16_t vsys_mv;
    double ts_percent;
    double tdie_c;
    uint16_t dp_mv;
    uint16_t dm_mv;
    uint8_t raw[CHARGER_SERVICE_REGISTER_MAP_SIZE];
} charger_service_snapshot_t;

esp_err_t charger_service_start(void);
void charger_service_get_snapshot(charger_service_snapshot_t *snapshot);
void charger_service_format_raw_hex(const charger_service_snapshot_t *snapshot,
                                    char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* CHARGER_SERVICE_H */
