#ifndef STABILITY_TEST_SERVICE_H
#define STABILITY_TEST_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum stability_test_status {
    STABILITY_TEST_STATUS_DISABLED = 0,
    STABILITY_TEST_STATUS_IDLE,
    STABILITY_TEST_STATUS_RUNNING,
    STABILITY_TEST_STATUS_FAILED,
};

esp_err_t stability_test_service_start(void);
bool stability_test_service_is_enabled(void);
enum stability_test_status stability_test_service_get_status(void);
const char *stability_test_service_status_to_string(
    enum stability_test_status status);
uint32_t stability_test_service_get_log_generated_count(void);
uint32_t stability_test_service_get_log_enqueue_failed_count(void);

#ifdef __cplusplus
}
#endif

#endif /* STABILITY_TEST_SERVICE_H */
