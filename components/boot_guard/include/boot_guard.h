#ifndef BOOT_GUARD_H
#define BOOT_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

esp_err_t boot_guard_init(void);
esp_err_t boot_guard_mark_stable(void);
esp_err_t boot_guard_clear_recovery(void);

bool boot_guard_recovery_mode(void);
bool boot_guard_new_app_pending_verify(void);
bool boot_guard_new_app_validated(void);
bool boot_guard_rollback_possible(void);

uint32_t boot_guard_boot_count(void);
uint32_t boot_guard_failure_count(void);
uint32_t boot_guard_recovery_threshold(void);
uint32_t boot_guard_stable_delay_ms(void);
esp_reset_reason_t boot_guard_last_reset_reason(void);
const char *boot_guard_last_reset_reason_name(void);
esp_ota_img_states_t boot_guard_running_ota_state(void);
const char *boot_guard_ota_state_name(esp_ota_img_states_t state);

#endif /* BOOT_GUARD_H */
