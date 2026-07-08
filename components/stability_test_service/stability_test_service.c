#include "stability_test_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uwb_dw3000.h"
#include "wireless_log_service.h"

static const char *TAG = "stability_test";

enum {
    STABILITY_TEST_TASK_STACK_WORDS = 3072,
    STABILITY_TEST_TASK_PRIORITY = 3,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define STABILITY_TEST_TASK_CORE 0
#else
#define STABILITY_TEST_TASK_CORE 0
#endif

static bool s_started;
static volatile enum stability_test_status s_status =
    STABILITY_TEST_STATUS_DISABLED;
static volatile uint32_t s_log_generated_count;
static volatile uint32_t s_log_enqueue_failed_count;

static TickType_t ms_to_ticks_min_1(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    return ticks;
}

static void make_payload(char *payload, size_t payload_size, uint32_t sequence)
{
    static const char pattern[] =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    if (payload_size == 0) {
        return;
    }

    const size_t requested_len = APP_STABILITY_LOG_STRESS_PAYLOAD_BYTES;
    const size_t max_len = payload_size - 1U;
    const size_t len = requested_len < max_len ? requested_len : max_len;

    for (size_t i = 0; i < len; ++i) {
        payload[i] = pattern[(sequence + i) % (sizeof(pattern) - 1U)];
    }
    payload[len] = '\0';
}

static void submit_stress_line(uint32_t sequence, uint32_t burst_index)
{
    char payload[128] = {0};
    make_payload(payload, sizeof(payload), sequence);

    const bool queued = wireless_log_service_submit(
        'I', TAG,
        "stress seq=%lu burst=%lu uwb_tx=%lu uwb_rx=%lu uwb_txerr=%lu uwb_rxerr=%lu dropped=%lu payload=%s",
        (unsigned long)sequence, (unsigned long)burst_index,
        (unsigned long)uwb_dw3000_get_tx_count(),
        (unsigned long)uwb_dw3000_get_rx_count(),
        (unsigned long)uwb_dw3000_get_tx_error_count(),
        (unsigned long)uwb_dw3000_get_rx_error_count(),
        (unsigned long)wireless_log_service_get_dropped_count(), payload);

    s_log_generated_count++;
    if (!queued) {
        s_log_enqueue_failed_count++;
    }
}

static void log_summary(void)
{
    ESP_LOGI(TAG,
             "Log stress summary: generated=%lu enqueue_failed=%lu wireless_dropped=%lu uwb_tx=%lu uwb_rx=%lu uwb_txerr=%lu uwb_rxerr=%lu",
             (unsigned long)s_log_generated_count,
             (unsigned long)s_log_enqueue_failed_count,
             (unsigned long)wireless_log_service_get_dropped_count(),
             (unsigned long)uwb_dw3000_get_tx_count(),
             (unsigned long)uwb_dw3000_get_rx_count(),
             (unsigned long)uwb_dw3000_get_tx_error_count(),
             (unsigned long)uwb_dw3000_get_rx_error_count());
}

static void stability_test_task(void *arg)
{
    (void)arg;

    s_status = STABILITY_TEST_STATUS_IDLE;
    ESP_LOGI(TAG,
             "Wireless log stress waiting for TCP log connection on core %d: burst=%u interval=%u ms payload=%u bytes",
             STABILITY_TEST_TASK_CORE,
             (unsigned)APP_STABILITY_LOG_STRESS_BURST_LINES,
             (unsigned)APP_STABILITY_LOG_STRESS_INTERVAL_MS,
             (unsigned)APP_STABILITY_LOG_STRESS_PAYLOAD_BYTES);

    uint32_t sequence = 0;
    TickType_t last_summary = xTaskGetTickCount();
    bool was_connected = false;

    while (true) {
        const bool connected = wireless_log_service_is_connected();
        if (!connected) {
            if (was_connected) {
                log_summary();
                ESP_LOGW(TAG, "Wireless log stress paused: TCP log disconnected");
            }
            s_status = STABILITY_TEST_STATUS_IDLE;
            was_connected = false;
            vTaskDelay(ms_to_ticks_min_1(500));
            continue;
        }

        if (!was_connected) {
            s_status = STABILITY_TEST_STATUS_RUNNING;
            was_connected = true;
            last_summary = xTaskGetTickCount();
            ESP_LOGI(TAG, "Wireless log stress running");
        }

        for (uint32_t i = 0; i < APP_STABILITY_LOG_STRESS_BURST_LINES; ++i) {
            submit_stress_line(sequence++, i);
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_summary) >=
            pdMS_TO_TICKS(APP_STABILITY_LOG_STRESS_SUMMARY_INTERVAL_MS)) {
            log_summary();
            last_summary = now;
        }

        vTaskDelay(ms_to_ticks_min_1(APP_STABILITY_LOG_STRESS_INTERVAL_MS));
    }
}

esp_err_t stability_test_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    if (!APP_STABILITY_LOG_STRESS_ENABLED) {
        s_status = STABILITY_TEST_STATUS_DISABLED;
        s_started = true;
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        stability_test_task, "stability_test", STABILITY_TEST_TASK_STACK_WORDS,
        NULL, STABILITY_TEST_TASK_PRIORITY, NULL, STABILITY_TEST_TASK_CORE);
    if (created != pdPASS) {
        s_status = STABILITY_TEST_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

bool stability_test_service_is_enabled(void)
{
    return APP_STABILITY_LOG_STRESS_ENABLED != 0;
}

enum stability_test_status stability_test_service_get_status(void)
{
    return s_status;
}

const char *stability_test_service_status_to_string(
    enum stability_test_status status)
{
    switch (status) {
    case STABILITY_TEST_STATUS_DISABLED:
        return "disabled";
    case STABILITY_TEST_STATUS_IDLE:
        return "idle";
    case STABILITY_TEST_STATUS_RUNNING:
        return "running";
    case STABILITY_TEST_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

uint32_t stability_test_service_get_log_generated_count(void)
{
    return s_log_generated_count;
}

uint32_t stability_test_service_get_log_enqueue_failed_count(void)
{
    return s_log_enqueue_failed_count;
}
