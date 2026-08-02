#include "gnss_firmware_updater.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_service.h"

static const char *TAG = "gnss_fw_update";

enum {
    GNSS_UPDATE_UART = UART_NUM_1,
    GNSS_UPDATE_RX_BUFFER_SIZE = 4096,
    GNSS_UPDATE_COMMAND_BAUD = 115200,
    GNSS_UPDATE_DOWNLOAD_BAUD = 115200,
    GNSS_UPDATE_DOWNLOAD_BAUD_INDEX = 5,
    GNSS_UPDATE_BUFFER_INDEX = 0,
    GNSS_UPDATE_BLOCK_SIZE = 8U * 1024U,
    GNSS_UPDATE_LOADER_SETTLE_MS = 200,
    GNSS_UPDATE_LOADER_TIMEOUT_MS = 15000,
    GNSS_UPDATE_ACK_TIMEOUT_MS = 6000,
    GNSS_UPDATE_BINSIZE_TIMEOUT_MS = 30000,
    GNSS_UPDATE_BLOCK_TIMEOUT_MS = 20000,
    GNSS_UPDATE_END_TIMEOUT_MS = 60000,
    GNSS_UPDATE_MAX_RESENDS = 5,
    GNSS_UPDATE_RAW_IMAGE_SUM8 = 123,
    GNSS_UPDATE_PACKED_PAYLOAD_SUM8 = 189,
    GNSS_UPDATE_PHOENIX_TAG_OFFSET = 1049152,
};

typedef enum {
    LOADER_FEEDBACK_TIMEOUT,
    LOADER_FEEDBACK_OK,
    LOADER_FEEDBACK_END,
    LOADER_FEEDBACK_RESEND,
    LOADER_FEEDBACK_RESEND_BIN,
    LOADER_FEEDBACK_RESET,
    LOADER_FEEDBACK_ERROR,
    LOADER_FEEDBACK_READY,
} loader_feedback_t;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void set_stage(gnss_firmware_update_result_t *result,
                      const char *stage)
{
    if (result != NULL) {
        snprintf(result->stage, sizeof(result->stage), "%s", stage);
    }
}

static void set_feedback(gnss_firmware_update_result_t *result,
                         const char *feedback)
{
    if (result != NULL) {
        snprintf(result->feedback, sizeof(result->feedback), "%s", feedback);
    }
}

static esp_err_t updater_uart_init_at_baud(int baud_rate)
{
    const uart_config_t config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(GNSS_UPDATE_UART,
                                        GNSS_UPDATE_RX_BUFFER_SIZE, 0, 0,
                                        NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(GNSS_UPDATE_UART, &config);
    if (err == ESP_OK) {
        err = uart_set_pin(GNSS_UPDATE_UART, BOARD_CONFIG_GPS_TX_GPIO,
                           BOARD_CONFIG_GPS_RX_GPIO, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        (void)uart_driver_delete(GNSS_UPDATE_UART);
        return err;
    }
    return uart_flush_input(GNSS_UPDATE_UART);
}

static esp_err_t updater_uart_reopen_at_download_baud(int command_baud,
                                                       int download_baud)
{
    /* Match GNSS Viewer closely. After ACK it closes and reopens the serial
     * port, waits 200 ms, changes to the selected loader baud, then waits
     * another 800 ms. Keep TX at the UART idle level while the ESP-IDF driver
     * is detached so the reopen cannot look like a byte or BREAK condition. */
    ESP_RETURN_ON_ERROR(
        uart_wait_tx_done(GNSS_UPDATE_UART, pdMS_TO_TICKS(2000)), TAG,
        "GNSS loader command drain failed");
    ESP_RETURN_ON_ERROR(uart_driver_delete(GNSS_UPDATE_UART), TAG,
                        "GNSS loader UART close failed");
    gpio_set_direction(BOARD_CONFIG_GPS_TX_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_CONFIG_GPS_TX_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_RETURN_ON_ERROR(updater_uart_init_at_baud(command_baud), TAG,
                        "GNSS loader UART reopen failed");
    ESP_RETURN_ON_ERROR(uart_set_baudrate(GNSS_UPDATE_UART, download_baud),
                        TAG, "GNSS loader baud switch failed");
    vTaskDelay(pdMS_TO_TICKS(800));
    return ESP_OK;
}

static esp_err_t uart_send_all(const uint8_t *data, size_t length)
{
    const int written = uart_write_bytes(GNSS_UPDATE_UART, data, length);
    if (written != (int)length) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(GNSS_UPDATE_UART, pdMS_TO_TICKS(2000));
}

static bool skytraq_ack_received(uint8_t command_id, uint32_t timeout_ms,
                                 bool *nack)
{
    uint8_t parser[64] = {0};
    size_t length = 0;
    const uint32_t started_ms = ticks_to_ms();
    *nack = false;
    while ((uint32_t)(ticks_to_ms() - started_ms) < timeout_ms) {
        uint8_t byte = 0;
        const int received = uart_read_bytes(GNSS_UPDATE_UART, &byte, 1,
                                             pdMS_TO_TICKS(20));
        if (received != 1) {
            continue;
        }
        if (length == 0 && byte != 0xA0) {
            continue;
        }
        if (length == 1 && byte != 0xA1) {
            length = byte == 0xA0 ? 1 : 0;
            parser[0] = 0xA0;
            continue;
        }
        if (length < sizeof(parser)) {
            parser[length++] = byte;
        } else {
            length = 0;
            continue;
        }
        if (length < 4) {
            continue;
        }
        const uint16_t payload_length =
            ((uint16_t)parser[2] << 8) | parser[3];
        const size_t frame_length = (size_t)payload_length + 7U;
        if (frame_length > sizeof(parser)) {
            length = 0;
            continue;
        }
        if (length != frame_length) {
            continue;
        }
        if (parser[frame_length - 2] == 0x0D &&
            parser[frame_length - 1] == 0x0A && payload_length >= 2 &&
            parser[5] == command_id) {
            if (parser[4] == 0x83) {
                return true;
            }
            if (parser[4] == 0x84) {
                *nack = true;
                return false;
            }
        }
        length = 0;
    }
    return false;
}

static loader_feedback_t classify_feedback(const char *buffer)
{
    if (strcmp(buffer, "READY") == 0 || strcmp(buffer, "READY1") == 0 ||
        strcmp(buffer, "READY2") == 0) {
        return LOADER_FEEDBACK_READY;
    }
    if (strstr(buffer, "Resendbin") != NULL) {
        return LOADER_FEEDBACK_RESEND_BIN;
    }
    if (strstr(buffer, "RESEND") != NULL) {
        return LOADER_FEEDBACK_RESEND;
    }
    if (strstr(buffer, "Reset") != NULL) {
        return LOADER_FEEDBACK_RESET;
    }
    if (strstr(buffer, "Error") != NULL) {
        return LOADER_FEEDBACK_ERROR;
    }
    if (strstr(buffer, "END") != NULL) {
        return LOADER_FEEDBACK_END;
    }
    if (strstr(buffer, "OK") != NULL) {
        return LOADER_FEEDBACK_OK;
    }
    return LOADER_FEEDBACK_TIMEOUT;
}

static void format_raw_feedback(const uint8_t *buffer, size_t length,
                                char *output, size_t output_size)
{
    if (output == NULL || output_size == 0) {
        return;
    }
    if (length == 0) {
        snprintf(output, output_size, "timeout:no-rx");
        return;
    }
    size_t used = (size_t)snprintf(output, output_size, "timeout:hex:");
    for (size_t index = 0; index < length && used + 2U < output_size;
         ++index) {
        const int written = snprintf(&output[used], output_size - used,
                                     "%02X", buffer[index]);
        if (written != 2) {
            break;
        }
        used += 2U;
    }
}

static loader_feedback_t wait_loader_feedback(uint32_t timeout_ms,
                                              char *last_feedback,
                                              size_t feedback_size)
{
    char token[64] = {0};
    size_t token_length = 0;
    uint8_t raw[64] = {0};
    size_t raw_length = 0;
    const uint32_t started_ms = ticks_to_ms();
    while ((uint32_t)(ticks_to_ms() - started_ms) < timeout_ms) {
        uint8_t byte = 0;
        const int received = uart_read_bytes(GNSS_UPDATE_UART, &byte, 1,
                                             pdMS_TO_TICKS(20));
        if (received != 1) {
            continue;
        }
        if (raw_length < sizeof(raw)) {
            raw[raw_length++] = byte;
        } else {
            memmove(raw, &raw[1], sizeof(raw) - 1U);
            raw[sizeof(raw) - 1U] = byte;
        }

        /* PX1105R's downloaded loader emits C-string records, for example
         * "$LOADER,...<NUL>END<NUL>".  Treat NUL and line endings as token
         * delimiters rather than allowing the first NUL to hide END/OK from
         * the normal C-string classifiers. */
        if (byte == '\0' || byte == '\r' || byte == '\n') {
            token_length = 0;
            token[0] = '\0';
            continue;
        }
        if (token_length < sizeof(token) - 1U) {
            token[token_length++] = (char)byte;
            token[token_length] = '\0';
        } else {
            memmove(token, &token[1], sizeof(token) - 2U);
            token[sizeof(token) - 2U] = (char)byte;
            token[sizeof(token) - 1U] = '\0';
        }
        const loader_feedback_t feedback = classify_feedback(token);
        if (feedback != LOADER_FEEDBACK_TIMEOUT) {
            if (last_feedback != NULL && feedback_size > 0) {
                snprintf(last_feedback, feedback_size, "%s", token);
            }
            return feedback;
        }
    }
    ESP_LOG_BUFFER_HEXDUMP(TAG, raw, raw_length, ESP_LOG_WARN);
    format_raw_feedback(raw, raw_length, last_feedback, feedback_size);
    return LOADER_FEEDBACK_TIMEOUT;
}

static esp_err_t read_exact(gnss_firmware_reader_t reader, void *context,
                            uint8_t *buffer, size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        size_t received = 0;
        const esp_err_t err = reader(context, &buffer[offset], length - offset,
                                     &received);
        if (err != ESP_OK) {
            return err;
        }
        if (received == 0 || received > length - offset) {
            return ESP_FAIL;
        }
        offset += received;
    }
    return ESP_OK;
}

static esp_err_t upload_srec_loader(
    size_t loader_size, gnss_firmware_reader_t reader, void *reader_context,
    gnss_firmware_update_result_t *result)
{
    uint8_t input[512] = {0};
    uint8_t line[160] = {0};
    size_t line_length = 0;
    size_t consumed = 0;

    while (consumed < loader_size) {
        const size_t remaining = loader_size - consumed;
        const size_t requested = remaining < sizeof(input) ? remaining
                                                            : sizeof(input);
        size_t received = 0;
        ESP_RETURN_ON_ERROR(
            reader(reader_context, input, requested, &received), TAG,
            "GNSS S-record read failed");
        if (received == 0 || received > requested) {
            return ESP_FAIL;
        }
        consumed += received;

        for (size_t index = 0; index < received; ++index) {
            if (line_length >= sizeof(line)) {
                ESP_LOGE(TAG, "GNSS S-record line exceeds %u bytes",
                         (unsigned)sizeof(line));
                return ESP_ERR_INVALID_SIZE;
            }
            line[line_length++] = input[index];
            if (input[index] != '\n') {
                continue;
            }
            /* GNSS Viewer does not forward the CRLF stored in its S-record
             * resource verbatim.  It copies the record without CRLF, then
             * writes LF followed by the zero byte left in its cleared line
             * buffer.  The PX1105R ROM monitor expects that exact wire
             * representation: "S...<LF><NUL>". */
            size_t record_length = line_length;
            while (record_length > 0 &&
                   (line[record_length - 1U] == '\r' ||
                    line[record_length - 1U] == '\n')) {
                --record_length;
            }
            if (record_length < 4 || line[0] != 'S' ||
                record_length + 2U > sizeof(line)) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            line[record_length] = '\n';
            line[record_length + 1U] = '\0';
            const size_t wire_length = record_length + 2U;
            ESP_RETURN_ON_ERROR(uart_send_all(line, wire_length), TAG,
                                "GNSS S-record write failed");
            result->loader_bytes_written += wire_length;
            line_length = 0;
        }
    }

    if (line_length != 0) {
        size_t record_length = line_length;
        while (record_length > 0 &&
               (line[record_length - 1U] == '\r' ||
                line[record_length - 1U] == '\n')) {
            --record_length;
        }
        if (record_length < 4 || line[0] != 'S' ||
            record_length + 2U > sizeof(line)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        line[record_length] = '\n';
        line[record_length + 1U] = '\0';
        const size_t wire_length = record_length + 2U;
        ESP_RETURN_ON_ERROR(uart_send_all(line, wire_length), TAG,
                            "GNSS final S-record write failed");
        result->loader_bytes_written += wire_length;
    }
    return consumed == loader_size ? ESP_OK : ESP_FAIL;
}

esp_err_t gnss_firmware_update(
    size_t loader_size, gnss_firmware_reader_t loader_reader,
    void *loader_reader_context, size_t packed_image_size,
    const uint8_t lzma_header[GNSS_FIRMWARE_LZMA_HEADER_SIZE],
    gnss_firmware_reader_t reader, void *reader_context,
    gnss_firmware_update_result_t *result)
{
    if (loader_size == 0 || loader_reader == NULL || packed_image_size == 0 ||
        lzma_header == NULL || reader == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));
    bool uart_installed = false;
    set_stage(result, "suspend_gps");
    esp_err_t err = gps_service_suspend_primary_uart();
    if (err != ESP_OK) {
        return err;
    }

    /* ENABLE is a real receiver power switch.  Start from a known normal-mode
     * boot instead of relying on whatever UART/parser state preceded the HTTP
     * request. */
    set_stage(result, "reset_gnss");
    err = gps_service_reset_receiver_for_update();
    if (err != ESP_OK) {
        goto done;
    }

    set_stage(result, "uart_init");
    err = updater_uart_init_at_baud(GNSS_UPDATE_COMMAND_BAUD);
    if (err != ESP_OK) {
        goto done;
    }
    uart_installed = true;

    /* Match the PX1105R path used by GNSS Viewer 2.1.147.  Sub-command 0x4f
     * selects the external serial loader that implements the ASCII BINSIZ8
     * protocol below.  Sub-command 0x4e selects the receiver's internal
     * loader, which ACKs this request but does not accept BINSIZ8.  Buffer
     * index 0 selects 8 KiB transfer blocks. */
    uint8_t enter_loader[] = {
        0xA0, 0xA1, 0x00, 0x07, 0x64, 0x4F,
        GNSS_UPDATE_DOWNLOAD_BAUD_INDEX,
        0x00, 0x00, 0x00, GNSS_UPDATE_BUFFER_INDEX, 0x00, 0x0D, 0x0A,
    };
    for (size_t index = 4; index < 11; ++index) {
        enter_loader[11] ^= enter_loader[index];
    }
    set_stage(result, "enter_loader");
    err = uart_send_all(enter_loader, sizeof(enter_loader));
    if (err != ESP_OK) {
        goto done;
    }
    bool nack = false;
    if (!skytraq_ack_received(0x64, GNSS_UPDATE_ACK_TIMEOUT_MS, &nack)) {
        if (nack) {
            set_feedback(result, "NACK");
            err = ESP_ERR_NOT_SUPPORTED;
            goto done;
        }
        set_feedback(result, "ACK timeout");
        err = ESP_ERR_TIMEOUT;
        goto done;
    } else {
        set_feedback(result, "ACK");
    }

    /* The ACK is returned at the normal receiver baud. The loader then uses
     * the baud selected in command 0x64/0x4f. Match GNSS Viewer: switch the
     * host UART immediately after ACK and allow 200 ms for the loader. */
    err = updater_uart_reopen_at_download_baud(GNSS_UPDATE_COMMAND_BAUD,
                                                GNSS_UPDATE_DOWNLOAD_BAUD);
    if (err != ESP_OK) {
        goto done;
    }
    uint32_t active_baud = 0;
    if (uart_get_baudrate(GNSS_UPDATE_UART, &active_baud) == ESP_OK) {
        ESP_LOGI(TAG, "GNSS loader UART ready at %u baud (validated profile)",
                 (unsigned)active_baud);
    }
    vTaskDelay(pdMS_TO_TICKS(GNSS_UPDATE_LOADER_SETTLE_MS));

    /* Command 0x64/0x4f only starts the receiver's S-record monitor. GNSS
     * Viewer then uploads its hardware-specific RAM loader line by line and
     * waits for that program to start before sending BINSIZ8. Skipping this
     * stage leaves no program running that understands BINSIZ8. */
    set_stage(result, "upload_loader");
    err = upload_srec_loader(loader_size, loader_reader,
                             loader_reader_context, result);
    if (err != ESP_OK) {
        goto done;
    }
    set_stage(result, "wait_loader");
    const loader_feedback_t loader_feedback = wait_loader_feedback(
        GNSS_UPDATE_LOADER_TIMEOUT_MS, result->feedback,
        sizeof(result->feedback));
    if (loader_feedback != LOADER_FEEDBACK_END &&
        loader_feedback != LOADER_FEEDBACK_OK &&
        loader_feedback != LOADER_FEEDBACK_READY) {
        err = loader_feedback == LOADER_FEEDBACK_TIMEOUT ? ESP_ERR_TIMEOUT
                                                          : ESP_FAIL;
        goto done;
    }
    unsigned check_code = (unsigned)packed_image_size +
                          GNSS_UPDATE_RAW_IMAGE_SUM8 +
                          GNSS_UPDATE_PHOENIX_TAG_OFFSET;
    for (size_t index = 0; index < GNSS_FIRMWARE_LZMA_HEADER_SIZE; ++index) {
        check_code += lzma_header[index];
    }
    char binsize[192] = {0};
    const int binsize_length =
        snprintf(binsize, sizeof(binsize),
                 "BINSIZ8 = %u %u %u 0 "
                 "%u %u %u %u %u %u %u %u %u %u %u %u %u %u ",
                 (unsigned)packed_image_size,
                 GNSS_UPDATE_RAW_IMAGE_SUM8,
                 GNSS_UPDATE_PHOENIX_TAG_OFFSET,
                 (unsigned)lzma_header[0], (unsigned)lzma_header[1],
                 (unsigned)lzma_header[2], (unsigned)lzma_header[3],
                 (unsigned)lzma_header[4], (unsigned)lzma_header[5],
                 (unsigned)lzma_header[6], (unsigned)lzma_header[7],
                 (unsigned)lzma_header[8], (unsigned)lzma_header[9],
                 (unsigned)lzma_header[10], (unsigned)lzma_header[11],
                 (unsigned)lzma_header[12],
                 check_code);
    if (binsize_length <= 0 || binsize_length >= (int)sizeof(binsize)) {
        err = ESP_ERR_INVALID_SIZE;
        goto done;
    }
    set_stage(result, "binsize");
    bool binsize_accepted = false;
    err = uart_send_all((const uint8_t *)binsize,
                        (size_t)binsize_length + 1U);
    if (err != ESP_OK) {
        goto done;
    }
    for (;;) {
        const loader_feedback_t feedback = wait_loader_feedback(
            GNSS_UPDATE_BINSIZE_TIMEOUT_MS, result->feedback,
            sizeof(result->feedback));
        if (feedback == LOADER_FEEDBACK_END) {
            continue;
        }
        binsize_accepted = feedback == LOADER_FEEDBACK_OK;
        break;
    }
    if (!binsize_accepted) {
        err = ESP_ERR_TIMEOUT;
        goto done;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t *block = malloc(GNSS_UPDATE_BLOCK_SIZE);
    if (block == NULL) {
        err = ESP_ERR_NO_MEM;
        goto done;
    }
    set_stage(result, "image_transfer");
    while (result->bytes_written < packed_image_size) {
        const size_t remaining = packed_image_size - result->bytes_written;
        const size_t block_length = remaining < GNSS_UPDATE_BLOCK_SIZE
                                        ? remaining
                                        : GNSS_UPDATE_BLOCK_SIZE;
        err = read_exact(reader, reader_context, block, block_length);
        if (err != ESP_OK) {
            free(block);
            goto done;
        }
        for (size_t index = 0; index < block_length; ++index) {
            result->calculated_sum += block[index];
        }

        unsigned resend_count = 0;
        for (;;) {
            err = uart_send_all(block, block_length);
            if (err != ESP_OK) {
                free(block);
                goto done;
            }
            const loader_feedback_t feedback = wait_loader_feedback(
                GNSS_UPDATE_BLOCK_TIMEOUT_MS, result->feedback,
                sizeof(result->feedback));
            if (feedback == LOADER_FEEDBACK_OK) {
                break;
            }
            if (feedback == LOADER_FEEDBACK_RESEND &&
                resend_count++ < GNSS_UPDATE_MAX_RESENDS) {
                ESP_LOGW(TAG, "Loader requested block resend at %u",
                         (unsigned)result->bytes_written);
                continue;
            }
            err = feedback == LOADER_FEEDBACK_TIMEOUT ? ESP_ERR_TIMEOUT
                                                       : ESP_FAIL;
            free(block);
            goto done;
        }
        result->bytes_written += block_length;
        if ((result->bytes_written % (128U * 1024U)) == 0 ||
            result->bytes_written == packed_image_size) {
            ESP_LOGI(TAG, "GNSS image %u/%u bytes",
                     (unsigned)result->bytes_written,
                     (unsigned)packed_image_size);
        }
    }
    free(block);

    if (result->calculated_sum != GNSS_UPDATE_PACKED_PAYLOAD_SUM8) {
        set_feedback(result, "sum mismatch");
        err = ESP_ERR_INVALID_CRC;
        goto done;
    }

    set_stage(result, "finalize");
    if (wait_loader_feedback(GNSS_UPDATE_END_TIMEOUT_MS, result->feedback,
                             sizeof(result->feedback)) !=
        LOADER_FEEDBACK_END) {
        err = ESP_ERR_TIMEOUT;
        goto done;
    }
    set_stage(result, "complete");
    err = ESP_OK;

done:
    if (uart_installed) {
        (void)uart_driver_delete(GNSS_UPDATE_UART);
    }
    /* ENABLE is a true GNSS power switch. A clean power cycle makes recovery
     * deterministic even when a host transfer stops inside the loader. */
    const esp_err_t reset_err = gps_service_reset_receiver_for_update();
    if (err == ESP_OK && reset_err != ESP_OK) {
        set_stage(result, "reboot_gnss");
        err = reset_err;
    }
    vTaskDelay(pdMS_TO_TICKS(err == ESP_OK ? 1500 : 200));
    const esp_err_t resume_err = gps_service_resume_primary_uart();
    if (err == ESP_OK && resume_err != ESP_OK) {
        set_stage(result, "resume_gps");
        err = resume_err;
    }
    ESP_LOGI(TAG, "GNSS firmware update result=%s stage=%s loader=%u "
                  "bytes=%u sum=%u",
             esp_err_to_name(err), result->stage,
             (unsigned)result->loader_bytes_written,
             (unsigned)result->bytes_written,
             (unsigned)result->calculated_sum);
    return err;
}
