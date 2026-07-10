#include "gps_service.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_runtime_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "gps_service";

enum {
    GPS_UART_NUM = UART_NUM_1,
    GPS_TASK_STACK_BYTES = 4096,
    GPS_TASK_PRIORITY = 4,
    GPS_UART_RX_BUFFER_SIZE = 4096,
    GPS_UART_TX_BUFFER_SIZE = 0,
    GPS_UART_READ_BUFFER_SIZE = 256,
    GPS_UART_READ_TIMEOUT_MS = 200,
    GPS_POWER_SETTLE_MS = 250,
    GPS_STOP_TIMEOUT_MS = 1000,
    GPS_STOP_POLL_MS = 20,
    GPS_NMEA_LINE_MAX = 160,
    GPS_NMEA_FIELD_MAX = 24,
    GPS_LOG_INTERVAL_MS = 1000,
    GPS_NO_DATA_LOG_INTERVAL_MS = 5000,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define GPS_TASK_CORE 0
#else
#define GPS_TASK_CORE 0
#endif

static bool s_service_initialized;
static volatile bool s_stop_requested;
static TaskHandle_t s_task_handle;
static SemaphoreHandle_t s_state_mutex;
static uint32_t s_last_log_ms;
static uint32_t s_last_no_data_log_ms;
static uint32_t s_last_rx_timestamp_ms;
static uint32_t s_last_fix_timestamp_ms;
static gps_service_snapshot_t s_snapshot;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t elapsed_since(uint32_t now_ms, uint32_t timestamp_ms)
{
    if (timestamp_ms == 0) {
        return UINT32_MAX;
    }
    return now_ms - timestamp_ms;
}

static bool gps_lock(TickType_t timeout)
{
    return s_state_mutex != NULL &&
           xSemaphoreTake(s_state_mutex, timeout) == pdTRUE;
}

static void gps_unlock(void)
{
    if (s_state_mutex != NULL) {
        xSemaphoreGive(s_state_mutex);
    }
}

static int gps_enable_level(bool enabled)
{
    const int active = BOARD_CONFIG_GPS_ENABLE_ACTIVE_LEVEL ? 1 : 0;
    return enabled ? active : !active;
}

static void gps_set_snapshot_runtime_enabled(bool enabled)
{
    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.runtime_enabled = enabled;
        gps_unlock();
    }
}

static void gps_set_snapshot_last_error(esp_err_t err)
{
    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.last_error = err;
        gps_unlock();
    }
}

static esp_err_t gps_set_enable_level(bool enabled)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_GPS_ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG,
                        "GPS enable GPIO init failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_GPS_ENABLE_GPIO, gps_enable_level(enabled)),
        TAG, "GPS enable GPIO set failed");

    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.powered = enabled;
        gps_unlock();
    }
    return ESP_OK;
}

static esp_err_t gps_configure_disabled_gpios(void)
{
    esp_err_t err = gps_set_enable_level(false);
    if (err != ESP_OK) {
        return err;
    }

    const uint64_t uart_pins = (1ULL << BOARD_CONFIG_GPS_RX_GPIO) |
                               (1ULL << BOARD_CONFIG_GPS_TX_GPIO);
    const gpio_config_t config = {
        .pin_bit_mask = uart_pins,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPS UART GPIO idle config failed: %s",
                 esp_err_to_name(err));
    }
    return err;
}

const char *gps_service_fix_quality_to_string(int quality)
{
    switch (quality) {
    case 0:
        return "no_fix";
    case 1:
        return "sps";
    case 2:
        return "dgps";
    case 3:
        return "pps";
    case 4:
        return "rtk_fixed";
    case 5:
        return "rtk_float";
    case 6:
        return "estimated";
    case 7:
        return "manual";
    case 8:
        return "simulator";
    default:
        return "unknown";
    }
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

static bool nmea_checksum_valid(const char *line)
{
    if (line == NULL || line[0] != '$') {
        return false;
    }

    const char *star = strchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') {
        return false;
    }

    const int hi = hex_value(star[1]);
    const int lo = hex_value(star[2]);
    if (hi < 0 || lo < 0) {
        return false;
    }

    uint8_t checksum = 0;
    for (const char *cursor = line + 1; cursor < star; ++cursor) {
        checksum ^= (uint8_t)(*cursor);
    }

    return checksum == (uint8_t)((hi << 4) | lo);
}

static size_t nmea_split_fields(char *line, char *fields[], size_t max_fields)
{
    if (line == NULL || fields == NULL || max_fields == 0) {
        return 0;
    }

    size_t count = 0;
    fields[count++] = line;
    for (char *cursor = line; *cursor != '\0' && count < max_fields;
         ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            fields[count++] = cursor + 1;
        }
    }
    return count;
}

static bool sentence_type_is(const char *id, const char *type)
{
    if (id == NULL || type == NULL) {
        return false;
    }
    if (id[0] == '$') {
        id++;
    }

    const size_t id_len = strlen(id);
    const size_t type_len = strlen(type);
    return id_len >= type_len &&
           strcmp(id + id_len - type_len, type) == 0;
}

static bool parse_double_field(const char *field, double *value)
{
    if (field == NULL || field[0] == '\0' || value == NULL) {
        return false;
    }

    char *end = NULL;
    const double parsed = strtod(field, &end);
    if (end == field || *end != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

static bool parse_u8_field(const char *field, uint8_t *value)
{
    if (field == NULL || field[0] == '\0' || value == NULL) {
        return false;
    }

    char *end = NULL;
    const unsigned long parsed = strtoul(field, &end, 10);
    if (end == field || *end != '\0' || parsed > 255UL) {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}

static bool parse_int_field(const char *field, int *value)
{
    if (field == NULL || field[0] == '\0' || value == NULL) {
        return false;
    }

    char *end = NULL;
    const long parsed = strtol(field, &end, 10);
    if (end == field || *end != '\0') {
        return false;
    }

    *value = (int)parsed;
    return true;
}

static bool parse_nmea_degrees(const char *field, char hemisphere,
                               double *value)
{
    double raw = 0.0;
    if (!parse_double_field(field, &raw) || value == NULL) {
        return false;
    }

    const int degrees = (int)(raw / 100.0);
    const double minutes = raw - ((double)degrees * 100.0);
    double decimal = (double)degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 's' || hemisphere == 'W' ||
        hemisphere == 'w') {
        decimal = -decimal;
    }

    *value = decimal;
    return true;
}

static void copy_field(char *destination, size_t destination_size,
                       const char *source)
{
    if (destination == NULL || destination_size == 0) {
        return;
    }
    if (source == NULL) {
        source = "";
    }

    snprintf(destination, destination_size, "%s", source);
}

static void copy_sentence_id(char *destination, size_t destination_size,
                             const char *source)
{
    if (source != NULL && source[0] == '$') {
        source++;
    }
    copy_field(destination, destination_size, source);
}

static void gps_note_bytes(size_t len)
{
    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.byte_count += (uint32_t)len;
        gps_unlock();
    }
}

static void gps_note_parse_error(void)
{
    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.parse_error_count++;
        gps_unlock();
    }
}

static void parse_gga(char *fields[], size_t count, uint32_t now_ms)
{
    if (count < 10) {
        gps_note_parse_error();
        return;
    }

    int quality = 0;
    uint8_t satellites = 0;
    double hdop = 0.0;
    double altitude = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    const bool have_quality = parse_int_field(fields[6], &quality);
    const bool have_sats = parse_u8_field(fields[7], &satellites);
    const bool have_hdop = parse_double_field(fields[8], &hdop);
    const bool have_alt = parse_double_field(fields[9], &altitude);
    const bool have_position =
        parse_nmea_degrees(fields[2], fields[3][0], &latitude) &&
        parse_nmea_degrees(fields[4], fields[5][0], &longitude);

    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.gga_count++;
        s_snapshot.last_rx_age_ms = 0;
        copy_field(s_snapshot.utc_time, sizeof(s_snapshot.utc_time), fields[1]);
        if (have_quality) {
            s_snapshot.fix_quality = quality;
        }
        if (have_sats) {
            s_snapshot.satellites = satellites;
        }
        if (have_hdop) {
            s_snapshot.hdop = hdop;
        }
        if (have_alt) {
            s_snapshot.altitude_m = altitude;
        }
        if (have_position && quality > 0) {
            s_snapshot.latitude_deg = latitude;
            s_snapshot.longitude_deg = longitude;
            s_snapshot.fix_valid = true;
            s_last_fix_timestamp_ms = now_ms;
            s_snapshot.last_fix_age_ms = 0;
        } else if (quality == 0) {
            s_snapshot.fix_valid = false;
        }
        gps_unlock();
    }

    (void)now_ms;
}

static char rmc_mode_from_fields(char *fields[], size_t count)
{
    if (count > 12 && fields[12] != NULL && fields[12][0] != '\0' &&
        fields[12][1] == '\0') {
        return fields[12][0];
    }
    if (count > 10 && fields[10] != NULL && fields[10][0] != '\0' &&
        fields[10][1] == '\0') {
        return fields[10][0];
    }

    for (size_t index = count; index > 10; --index) {
        const char *field = fields[index - 1U];
        if (field != NULL && field[0] != '\0' && field[1] == '\0') {
            return field[0];
        }
    }
    return '\0';
}

static void parse_rmc(char *fields[], size_t count, uint32_t now_ms)
{
    if (count < 10) {
        gps_note_parse_error();
        return;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    double speed_knots = 0.0;
    double course = 0.0;
    const bool have_position =
        parse_nmea_degrees(fields[3], fields[4][0], &latitude) &&
        parse_nmea_degrees(fields[5], fields[6][0], &longitude);
    const bool have_speed = parse_double_field(fields[7], &speed_knots);
    const bool have_course = parse_double_field(fields[8], &course);
    const char status = fields[2][0];
    const char mode = rmc_mode_from_fields(fields, count);

    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.rmc_count++;
        s_snapshot.last_rx_age_ms = 0;
        s_snapshot.rmc_status = status;
        s_snapshot.rmc_mode = mode;
        copy_field(s_snapshot.utc_time, sizeof(s_snapshot.utc_time), fields[1]);
        copy_field(s_snapshot.utc_date, sizeof(s_snapshot.utc_date), fields[9]);
        if (have_speed) {
            s_snapshot.speed_mps = speed_knots * 0.514444;
        }
        if (have_course) {
            s_snapshot.course_deg = course;
        }
        if (status == 'A' && have_position) {
            s_snapshot.latitude_deg = latitude;
            s_snapshot.longitude_deg = longitude;
            s_snapshot.fix_valid = true;
            s_last_fix_timestamp_ms = now_ms;
            s_snapshot.last_fix_age_ms = 0;
        }
        gps_unlock();
    }

    (void)now_ms;
}

static void parse_gsa(char *fields[], size_t count)
{
    if (count < 3) {
        gps_note_parse_error();
        return;
    }

    uint8_t fix_type = 0;
    if (!parse_u8_field(fields[2], &fix_type)) {
        gps_note_parse_error();
        return;
    }

    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.gsa_count++;
        s_snapshot.fix_type = fix_type;
        gps_unlock();
    }
}

static void parse_psti030(char *fields[], size_t count)
{
    if (count < 14 || strcmp(fields[1], "030") != 0) {
        return;
    }

    double altitude = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double rtk_age = 0.0;
    double rtk_ratio = 0.0;
    const bool have_alt = parse_double_field(fields[8], &altitude);
    const bool have_position =
        parse_nmea_degrees(fields[4], fields[5][0], &latitude) &&
        parse_nmea_degrees(fields[6], fields[7][0], &longitude);
    const bool have_age = count > 14 && parse_double_field(fields[14], &rtk_age);
    const bool have_ratio =
        count > 15 && parse_double_field(fields[15], &rtk_ratio);

    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.psti030_count++;
        s_snapshot.rmc_status = fields[3][0];
        s_snapshot.rmc_mode = fields[13][0];
        copy_field(s_snapshot.utc_time, sizeof(s_snapshot.utc_time), fields[2]);
        copy_field(s_snapshot.utc_date, sizeof(s_snapshot.utc_date), fields[12]);
        if (have_alt) {
            s_snapshot.altitude_m = altitude;
        }
        if (have_position && fields[3][0] == 'A') {
            s_snapshot.latitude_deg = latitude;
            s_snapshot.longitude_deg = longitude;
            s_snapshot.fix_valid = true;
            s_last_fix_timestamp_ms = ticks_to_ms();
            s_snapshot.last_fix_age_ms = 0;
        }
        if (have_age) {
            s_snapshot.rtk_age_s = rtk_age;
        }
        if (have_ratio) {
            s_snapshot.rtk_ratio = rtk_ratio;
        }
        gps_unlock();
    }
}

static void gps_handle_nmea_line(const char *line)
{
    const uint32_t now_ms = ticks_to_ms();

    if (!nmea_checksum_valid(line)) {
        if (gps_lock(pdMS_TO_TICKS(20))) {
            s_snapshot.checksum_error_count++;
            gps_unlock();
        }
        return;
    }

    char work[GPS_NMEA_LINE_MAX] = {0};
    snprintf(work, sizeof(work), "%s", line);

    char *star = strchr(work, '*');
    if (star != NULL) {
        *star = '\0';
    }

    char *fields[GPS_NMEA_FIELD_MAX] = {0};
    const size_t count =
        nmea_split_fields(work, fields, GPS_NMEA_FIELD_MAX);
    if (count == 0 || fields[0] == NULL) {
        gps_note_parse_error();
        return;
    }

    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_snapshot.sentence_count++;
        s_snapshot.last_rx_age_ms = 0;
        copy_sentence_id(s_snapshot.last_sentence_id,
                         sizeof(s_snapshot.last_sentence_id), fields[0]);
        gps_unlock();
    }

    if (sentence_type_is(fields[0], "GGA")) {
        parse_gga(fields, count, now_ms);
    } else if (sentence_type_is(fields[0], "RMC")) {
        parse_rmc(fields, count, now_ms);
    } else if (sentence_type_is(fields[0], "GSA")) {
        parse_gsa(fields, count);
    } else if (strcmp(fields[0], "$PSTI") == 0) {
        parse_psti030(fields, count);
    }
}

static void gps_copy_snapshot(gps_service_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (gps_lock(pdMS_TO_TICKS(50))) {
        *snapshot = s_snapshot;
        const uint32_t now_ms = ticks_to_ms();
        snapshot->last_rx_age_ms =
            elapsed_since(now_ms, s_last_rx_timestamp_ms);
        snapshot->last_fix_age_ms =
            elapsed_since(now_ms, s_last_fix_timestamp_ms);
        gps_unlock();
    } else {
        memset(snapshot, 0, sizeof(*snapshot));
    }
}

static void gps_set_last_rx_timestamp(uint32_t now_ms)
{
    if (gps_lock(pdMS_TO_TICKS(20))) {
        s_last_rx_timestamp_ms = now_ms;
        s_snapshot.last_rx_age_ms = 0;
        gps_unlock();
    }
}

static void gps_log_summary_if_needed(void)
{
    const uint32_t now_ms = ticks_to_ms();
    gps_service_snapshot_t snapshot = {0};
    gps_copy_snapshot(&snapshot);

    if (!snapshot.powered || !snapshot.task_running) {
        return;
    }

    if (snapshot.sentence_count == 0) {
        if ((uint32_t)(now_ms - s_last_no_data_log_ms) >=
            GPS_NO_DATA_LOG_INTERVAL_MS) {
            s_last_no_data_log_ms = now_ms;
            ESP_LOGW(TAG,
                     "GPS enabled but no NMEA yet: bytes=%lu err=%s uart=%s",
                     (unsigned long)snapshot.byte_count,
                     esp_err_to_name((esp_err_t)snapshot.last_error),
                     snapshot.uart_ready ? "ready" : "not_ready");
        }
        return;
    }

    if ((uint32_t)(now_ms - s_last_log_ms) < GPS_LOG_INTERVAL_MS) {
        return;
    }
    s_last_log_ms = now_ms;

    ESP_LOGI(TAG,
             "GPS summary fix=%s q=%d type=%u sats=%u hdop=%.2f lat=%.7f lon=%.7f alt=%.2f speed=%.2f mode=%c age_rx=%lu age_fix=%lu sent=%lu gga=%lu rmc=%lu psti030=%lu csum=%lu parse=%lu",
             snapshot.fix_valid ? "valid" : "invalid", snapshot.fix_quality,
             (unsigned)snapshot.fix_type, (unsigned)snapshot.satellites,
             snapshot.hdop, snapshot.latitude_deg, snapshot.longitude_deg,
             snapshot.altitude_m, snapshot.speed_mps,
             snapshot.rmc_mode != '\0' ? snapshot.rmc_mode : '-',
             (unsigned long)snapshot.last_rx_age_ms,
             (unsigned long)snapshot.last_fix_age_ms,
             (unsigned long)snapshot.sentence_count,
             (unsigned long)snapshot.gga_count,
             (unsigned long)snapshot.rmc_count,
             (unsigned long)snapshot.psti030_count,
             (unsigned long)snapshot.checksum_error_count,
             (unsigned long)snapshot.parse_error_count);
}

static esp_err_t gps_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BOARD_CONFIG_GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install((uart_port_t)GPS_UART_NUM,
                                            GPS_UART_RX_BUFFER_SIZE,
                                            GPS_UART_TX_BUFFER_SIZE, 0, NULL,
                                            0),
                        TAG, "GPS UART driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config((uart_port_t)GPS_UART_NUM,
                                          &uart_config),
                        TAG, "GPS UART param config failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin((uart_port_t)GPS_UART_NUM, BOARD_CONFIG_GPS_TX_GPIO,
                     BOARD_CONFIG_GPS_RX_GPIO, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE),
        TAG, "GPS UART pin config failed");
    ESP_RETURN_ON_ERROR(uart_flush_input((uart_port_t)GPS_UART_NUM), TAG,
                        "GPS UART flush failed");

    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.uart_ready = true;
        s_snapshot.last_error = ESP_OK;
        gps_unlock();
    }
    return ESP_OK;
}

static void gps_uart_deinit(void)
{
    (void)uart_driver_delete((uart_port_t)GPS_UART_NUM);
    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.uart_ready = false;
        gps_unlock();
    }
}

static void gps_task(void *arg)
{
    (void)arg;
    s_task_handle = xTaskGetCurrentTaskHandle();
    s_stop_requested = false;
    s_last_log_ms = 0;
    s_last_no_data_log_ms = ticks_to_ms();

    if (gps_lock(pdMS_TO_TICKS(50))) {
        memset(&s_snapshot, 0, sizeof(s_snapshot));
        s_last_rx_timestamp_ms = 0;
        s_last_fix_timestamp_ms = 0;
        s_snapshot.runtime_enabled = true;
        s_snapshot.task_running = true;
        s_snapshot.last_rx_age_ms = UINT32_MAX;
        s_snapshot.last_fix_age_ms = UINT32_MAX;
        gps_unlock();
    }

    ESP_LOGI(TAG, "GPS task starting: EN=%d RX=%d TX=%d baud=%d core=%d",
             BOARD_CONFIG_GPS_ENABLE_GPIO, BOARD_CONFIG_GPS_RX_GPIO,
             BOARD_CONFIG_GPS_TX_GPIO, BOARD_CONFIG_GPS_BAUD, GPS_TASK_CORE);

    esp_err_t err = gps_set_enable_level(true);
    if (err != ESP_OK) {
        gps_set_snapshot_last_error(err);
        goto done;
    }

    vTaskDelay(pdMS_TO_TICKS(GPS_POWER_SETTLE_MS));

    err = gps_uart_init();
    if (err != ESP_OK) {
        gps_set_snapshot_last_error(err);
        goto done;
    }

    uint8_t read_buffer[GPS_UART_READ_BUFFER_SIZE] = {0};
    char line[GPS_NMEA_LINE_MAX] = {0};
    size_t line_len = 0;

    while (!s_stop_requested) {
        const int len = uart_read_bytes(
            (uart_port_t)GPS_UART_NUM, read_buffer, sizeof(read_buffer),
            pdMS_TO_TICKS(GPS_UART_READ_TIMEOUT_MS));
        if (len > 0) {
            gps_note_bytes((size_t)len);
            gps_set_last_rx_timestamp(ticks_to_ms());
            for (int i = 0; i < len; ++i) {
                const char c = (char)read_buffer[i];
                if (c == '$') {
                    line_len = 0;
                    line[line_len++] = c;
                    continue;
                }
                if (c == '\r') {
                    continue;
                }
                if (c == '\n') {
                    if (line_len > 0) {
                        line[line_len] = '\0';
                        gps_handle_nmea_line(line);
                    }
                    line_len = 0;
                    continue;
                }
                if (line_len > 0 && line_len < sizeof(line) - 1U) {
                    line[line_len++] = c;
                } else if (line_len > 0) {
                    line_len = 0;
                    gps_note_parse_error();
                }
            }
        }

        gps_log_summary_if_needed();
    }

done:
    gps_uart_deinit();
    if (!app_runtime_config_get()->gps_enabled || err != ESP_OK) {
        (void)gps_configure_disabled_gpios();
    }
    if (gps_lock(pdMS_TO_TICKS(50))) {
        s_snapshot.runtime_enabled = app_runtime_config_get()->gps_enabled;
        s_snapshot.task_running = false;
        if (err != ESP_OK) {
            s_snapshot.last_error = err;
        }
        gps_unlock();
    }
    ESP_LOGI(TAG, "GPS task stopped");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t gps_start_task(void)
{
    if (s_task_handle != NULL) {
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        gps_task, "gps", GPS_TASK_STACK_BYTES, NULL, GPS_TASK_PRIORITY,
        &s_task_handle, GPS_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static esp_err_t gps_stop_task(void)
{
    if (s_task_handle == NULL) {
        return gps_configure_disabled_gpios();
    }

    s_stop_requested = true;

    const uint32_t start_ms = ticks_to_ms();
    while (s_task_handle != NULL &&
           (uint32_t)(ticks_to_ms() - start_ms) < GPS_STOP_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(GPS_STOP_POLL_MS));
    }

    if (s_task_handle != NULL) {
        ESP_LOGW(TAG, "GPS task did not stop within %u ms",
                 (unsigned)GPS_STOP_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    return gps_configure_disabled_gpios();
}

esp_err_t gps_service_apply_runtime_config(void)
{
    const app_runtime_config_t *runtime_config = app_runtime_config_get();
    const bool enabled = runtime_config->gps_enabled;
    gps_set_snapshot_runtime_enabled(enabled);

    if (enabled) {
        return gps_start_task();
    }

    return gps_stop_task();
}

esp_err_t gps_service_start(void)
{
    if (s_service_initialized) {
        return gps_service_apply_runtime_config();
    }

    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (gps_lock(pdMS_TO_TICKS(50))) {
        memset(&s_snapshot, 0, sizeof(s_snapshot));
        s_snapshot.runtime_enabled = app_runtime_config_get()->gps_enabled;
        s_snapshot.last_error = ESP_OK;
        s_snapshot.last_rx_age_ms = UINT32_MAX;
        s_snapshot.last_fix_age_ms = UINT32_MAX;
        gps_unlock();
    }

    s_service_initialized = true;
    const esp_err_t err = gps_service_apply_runtime_config();
    ESP_LOGI(TAG, "GPS %s on GPIO%d active-%s",
             app_runtime_config_get()->gps_enabled ? "enabled" : "disabled",
             BOARD_CONFIG_GPS_ENABLE_GPIO,
             BOARD_CONFIG_GPS_ENABLE_ACTIVE_LEVEL ? "high" : "low");
    return err;
}

void gps_service_get_snapshot(gps_service_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    gps_copy_snapshot(snapshot);
    snapshot->runtime_enabled = app_runtime_config_get()->gps_enabled;
}
