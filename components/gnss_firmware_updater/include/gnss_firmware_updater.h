#ifndef GNSS_FIRMWARE_UPDATER_H
#define GNSS_FIRMWARE_UPDATER_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*gnss_firmware_reader_t)(void *context, uint8_t *buffer,
                                            size_t capacity,
                                            size_t *received);

typedef struct {
    size_t loader_bytes_written;
    size_t bytes_written;
    uint8_t calculated_sum;
    char stage[32];
    char feedback[128];
} gnss_firmware_update_result_t;

enum {
    GNSS_FIRMWARE_LZMA_HEADER_SIZE = 13,
};

esp_err_t gnss_firmware_update(
    size_t loader_size, gnss_firmware_reader_t loader_reader,
    void *loader_reader_context, size_t packed_image_size,
    const uint8_t lzma_header[GNSS_FIRMWARE_LZMA_HEADER_SIZE],
    gnss_firmware_reader_t reader, void *reader_context,
    gnss_firmware_update_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FIRMWARE_UPDATER_H */
