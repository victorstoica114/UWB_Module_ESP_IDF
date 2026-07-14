#include "boot_guard.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

static const char *TAG = "boot_guard";

enum {
    BOOT_GUARD_MAGIC = 0x55424744U, /* UBGD */
    BOOT_GUARD_VERSION = 1,
};

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t boot_count;
    uint32_t consecutive_failures;
    uint32_t last_reset_reason;
    uint8_t boot_completed;
    uint8_t recovery_latched;
    uint16_t reserved;
    uint32_t checksum;
} boot_guard_rtc_state_t;

static RTC_NOINIT_ATTR boot_guard_rtc_state_t s_rtc_state;

static bool s_initialized;
static bool s_recovery_mode;
static bool s_pending_verify;
static bool s_validated;
static bool s_rollback_possible;
static esp_ota_img_states_t s_running_ota_state = ESP_OTA_IMG_UNDEFINED;
static esp_reset_reason_t s_last_reset_reason = ESP_RST_UNKNOWN;

static uint32_t boot_guard_checksum(const boot_guard_rtc_state_t *state)
{
    return state->magic ^ state->version ^ state->boot_count ^
           state->consecutive_failures ^ state->last_reset_reason ^
           ((uint32_t)state->boot_completed << 8U) ^
           ((uint32_t)state->recovery_latched << 16U);
}

static void boot_guard_store(void)
{
    s_rtc_state.checksum = boot_guard_checksum(&s_rtc_state);
}

static bool boot_guard_rtc_valid(void)
{
    return s_rtc_state.magic == BOOT_GUARD_MAGIC &&
           s_rtc_state.version == BOOT_GUARD_VERSION &&
           s_rtc_state.checksum == boot_guard_checksum(&s_rtc_state);
}

static void boot_guard_rtc_reset(void)
{
    memset(&s_rtc_state, 0, sizeof(s_rtc_state));
    s_rtc_state.magic = BOOT_GUARD_MAGIC;
    s_rtc_state.version = BOOT_GUARD_VERSION;
    s_rtc_state.boot_completed = 1U;
    boot_guard_store();
}

static uint32_t boot_guard_threshold(void)
{
    return APP_BOOT_GUARD_FAILURE_THRESHOLD > 0
               ? (uint32_t)APP_BOOT_GUARD_FAILURE_THRESHOLD
               : 1U;
}

static bool reset_reason_counts_as_failure(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_UNKNOWN:
    case ESP_RST_SW:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_CPU_LOCKUP:
        return true;
    default:
        return false;
    }
}

static void boot_guard_refresh_ota_state(void)
{
    s_running_ota_state = ESP_OTA_IMG_UNDEFINED;
    s_pending_verify = false;
    s_validated = false;
    s_rollback_possible = esp_ota_check_rollback_is_possible();

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return;
    }

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    const esp_err_t err = esp_ota_get_state_partition(running, &state);
    if (err != ESP_OK) {
        return;
    }

    s_running_ota_state = state;
    s_pending_verify = state == ESP_OTA_IMG_PENDING_VERIFY;
    s_validated = state == ESP_OTA_IMG_VALID;
}

const char *boot_guard_ota_state_name(esp_ota_img_states_t state)
{
    switch (state) {
    case ESP_OTA_IMG_NEW:
        return "new";
    case ESP_OTA_IMG_PENDING_VERIFY:
        return "pending_verify";
    case ESP_OTA_IMG_VALID:
        return "valid";
    case ESP_OTA_IMG_INVALID:
        return "invalid";
    case ESP_OTA_IMG_ABORTED:
        return "aborted";
    case ESP_OTA_IMG_UNDEFINED:
    default:
        return "undefined";
    }
}

const char *boot_guard_last_reset_reason_name(void)
{
    switch (s_last_reset_reason) {
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:
        return "task_watchdog";
    case ESP_RST_WDT:
        return "watchdog";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    case ESP_RST_USB:
        return "usb";
    case ESP_RST_JTAG:
        return "jtag";
    case ESP_RST_EFUSE:
        return "efuse";
    case ESP_RST_PWR_GLITCH:
        return "power_glitch";
    case ESP_RST_CPU_LOCKUP:
        return "cpu_lockup";
    case ESP_RST_UNKNOWN:
    default:
        return "unknown";
    }
}

esp_err_t boot_guard_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_last_reset_reason = esp_reset_reason();
    const bool reset_rtc_state =
        s_last_reset_reason == ESP_RST_POWERON ||
        s_last_reset_reason == ESP_RST_BROWNOUT || !boot_guard_rtc_valid();
    if (reset_rtc_state) {
        boot_guard_rtc_reset();
    }

    if (!s_rtc_state.boot_completed &&
        reset_reason_counts_as_failure(s_last_reset_reason)) {
        ++s_rtc_state.consecutive_failures;
    } else if (s_rtc_state.boot_completed) {
        s_rtc_state.consecutive_failures = 0;
    }

    ++s_rtc_state.boot_count;
    s_rtc_state.last_reset_reason = (uint32_t)s_last_reset_reason;
    if (s_rtc_state.consecutive_failures >= boot_guard_threshold()) {
        s_rtc_state.recovery_latched = 1U;
    }
    s_rtc_state.boot_completed = 0U;
    boot_guard_store();

    s_recovery_mode = s_rtc_state.recovery_latched != 0U;
    boot_guard_refresh_ota_state();
    s_initialized = true;

    ESP_LOGW(TAG,
             "boot guard: boot=%lu failures=%lu threshold=%lu recovery=%s reset=%s ota_state=%s rollback_possible=%s",
             (unsigned long)s_rtc_state.boot_count,
             (unsigned long)s_rtc_state.consecutive_failures,
             (unsigned long)boot_guard_threshold(),
             s_recovery_mode ? "true" : "false",
             boot_guard_last_reset_reason_name(),
             boot_guard_ota_state_name(s_running_ota_state),
             s_rollback_possible ? "true" : "false");

    return ESP_OK;
}

esp_err_t boot_guard_mark_stable(void)
{
    if (!s_initialized) {
        ESP_RETURN_ON_ERROR(boot_guard_init(), TAG, "boot guard init failed");
    }

    s_rtc_state.boot_completed = 1U;
    s_rtc_state.consecutive_failures = 0;
    if (!s_recovery_mode) {
        s_rtc_state.recovery_latched = 0U;
    }
    boot_guard_store();

    boot_guard_refresh_ota_state();
    if (!s_pending_verify) {
        return ESP_OK;
    }

    if (s_recovery_mode) {
        ESP_LOGW(TAG,
                 "running app is pending verify, but recovery mode is active; leaving OTA image unvalidated");
        return ESP_OK;
    }

    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to mark OTA image valid: %s",
                 esp_err_to_name(err));
        return err;
    }

    boot_guard_refresh_ota_state();
    ESP_LOGI(TAG, "OTA image marked valid after startup self-test");
    return ESP_OK;
}

esp_err_t boot_guard_clear_recovery(void)
{
    if (!s_initialized) {
        ESP_RETURN_ON_ERROR(boot_guard_init(), TAG, "boot guard init failed");
    }

    s_rtc_state.recovery_latched = 0U;
    s_rtc_state.consecutive_failures = 0;
    s_rtc_state.boot_completed = 1U;
    boot_guard_store();
    s_recovery_mode = false;
    ESP_LOGW(TAG, "boot recovery latch cleared");
    return ESP_OK;
}

bool boot_guard_recovery_mode(void)
{
    return s_recovery_mode;
}

bool boot_guard_new_app_pending_verify(void)
{
    return s_pending_verify;
}

bool boot_guard_new_app_validated(void)
{
    return s_validated;
}

bool boot_guard_rollback_possible(void)
{
    return s_rollback_possible;
}

uint32_t boot_guard_boot_count(void)
{
    return s_rtc_state.boot_count;
}

uint32_t boot_guard_failure_count(void)
{
    return s_rtc_state.consecutive_failures;
}

uint32_t boot_guard_recovery_threshold(void)
{
    return boot_guard_threshold();
}

uint32_t boot_guard_stable_delay_ms(void)
{
    return (uint32_t)APP_BOOT_GUARD_STABLE_DELAY_MS;
}

esp_reset_reason_t boot_guard_last_reset_reason(void)
{
    return s_last_reset_reason;
}

esp_ota_img_states_t boot_guard_running_ota_state(void)
{
    return s_running_ota_state;
}
