#include "gps_moving_base.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "app_config.h"
#include "board_config.h"
#include "esp_log.h"
#include "gps_ntrip_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"

static const char *TAG = "gps_moving_base";

enum {
    GPS_MB_CORRECTION_UART = UART_NUM_2,
    GPS_MB_RELAY_PORT = 21600,
    GPS_MB_MODULE_PORT = 21601,
    GPS_MB_HEADER_SIZE = 14,
    GPS_MB_MAX_PAYLOAD = 256,
    GPS_MB_RX_BUFFER_SIZE = 256,
    GPS_MB_TX_BUFFER_SIZE = 1024,
    GPS_MB_MAGIC = 0x474D4231UL, /* GMB1 */
    GPS_MB_VERSION = 1,
    GPS_MB_KIND_HEARTBEAT = 1,
    GPS_MB_KIND_STREAM = 2,
    GPS_MB_HEARTBEAT_INTERVAL_MS = 1000,
    GPS_MB_CONFIG_START_DELAY_MS = 800,
    /* RTK role/rate changes are acknowledged before the Phoenix engine has
     * fully transitioned.  Keep startup configuration deterministic; this
     * delay is paid once and does not affect the 8 Hz moving-base stream. */
    GPS_MB_CONFIG_STEP_DELAY_MS = 5000,
};

typedef struct {
    uint8_t buffer[1024];
    size_t length;
    size_t expected_length;
} skytraq_stream_parser_t;

static uart_port_t s_primary_uart = UART_NUM_MAX;
static uint8_t s_module_id;
static int s_socket = -1;
static struct sockaddr_in s_relay_address;
static gps_moving_base_snapshot_t s_snapshot;
static uint32_t s_started_ms;
static uint32_t s_last_heartbeat_ms;
static uint32_t s_last_uplink_ms;
static uint32_t s_last_downlink_ms;
static uint32_t s_uplink_sequence;
static uint32_t s_last_downlink_sequence;
static uint8_t s_config_step;
static skytraq_stream_parser_t s_binary_parser;

static uint32_t ticks_to_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t elapsed_since(uint32_t now_ms, uint32_t timestamp_ms)
{
    return timestamp_ms == 0 ? UINT32_MAX : now_ms - timestamp_ms;
}

static void put_be16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8);
    destination[1] = (uint8_t)value;
}

static void put_be32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value >> 24);
    destination[1] = (uint8_t)(value >> 16);
    destination[2] = (uint8_t)(value >> 8);
    destination[3] = (uint8_t)value;
}

static void put_be64(uint8_t *destination, uint64_t value)
{
    for (size_t index = 0; index < 8; ++index) {
        destination[index] = (uint8_t)(value >> (56U - (index * 8U)));
    }
}

static uint16_t get_be16(const uint8_t *source)
{
    return (uint16_t)(((uint16_t)source[0] << 8) | source[1]);
}

static uint32_t get_be32(const uint8_t *source)
{
    return ((uint32_t)source[0] << 24) | ((uint32_t)source[1] << 16) |
           ((uint32_t)source[2] << 8) | source[3];
}

static void put_be_float(uint8_t *destination, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    put_be32(destination, bits);
}

static void put_be_double(uint8_t *destination, double value)
{
    uint64_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    put_be64(destination, bits);
}

const char *gps_moving_base_role_to_string(gps_moving_base_role_t role)
{
    switch (role) {
    case GPS_MB_ROLE_PRECISE_BASE:
        return "precise_base";
    case GPS_MB_ROLE_MOVING_ROVER:
        return "moving_rover";
    case GPS_MB_ROLE_LOCAL_BASE:
        return "local_base";
    case GPS_MB_ROLE_NONE:
    default:
        return "none";
    }
}

static gps_moving_base_role_t role_for_module(uint8_t module_id)
{
    switch (module_id) {
    case 1:
        return GPS_MB_ROLE_PRECISE_BASE;
    case 2:
        return GPS_MB_ROLE_MOVING_ROVER;
    case 3:
        /* PointPerfect replaces the temporary local-base correction source.
         * Keeping M3 out of base mode prevents two independent correction
         * streams from reaching the precisely-kinematic base. */
        return gps_ntrip_client_is_enabled() ? GPS_MB_ROLE_NONE
                                             : GPS_MB_ROLE_LOCAL_BASE;
    default:
        return GPS_MB_ROLE_NONE;
    }
}

static bool role_forwards_primary(gps_moving_base_role_t role)
{
    return role == GPS_MB_ROLE_LOCAL_BASE ||
           role == GPS_MB_ROLE_PRECISE_BASE;
}

static bool role_receives_corrections(gps_moving_base_role_t role)
{
    return role == GPS_MB_ROLE_PRECISE_BASE ||
           role == GPS_MB_ROLE_MOVING_ROVER;
}

static esp_err_t send_skytraq_payload(const uint8_t *payload, size_t length)
{
    if (payload == NULL || length == 0 || length > UINT16_MAX ||
        s_primary_uart == UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[64] = {0};
    if (length + 7U > sizeof(packet)) {
        return ESP_ERR_INVALID_SIZE;
    }

    packet[0] = 0xA0;
    packet[1] = 0xA1;
    put_be16(&packet[2], (uint16_t)length);
    memcpy(&packet[4], payload, length);
    uint8_t checksum = 0;
    for (size_t index = 0; index < length; ++index) {
        checksum ^= payload[index];
    }
    packet[4 + length] = checksum;
    packet[5 + length] = 0x0D;
    packet[6 + length] = 0x0A;

    const int written = uart_write_bytes(
        s_primary_uart, packet, (size_t)(length + 7U));
    if (written != (int)(length + 7U)) {
        ESP_LOGW(TAG, "SkyTraq command write failed: wanted=%u wrote=%d",
                 (unsigned)(length + 7U), written);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t configure_rtk_role(void)
{
    uint8_t payload[37] = {0};
    payload[0] = 0x6A;
    payload[1] = 0x06;

    switch (s_snapshot.role) {
    case GPS_MB_ROLE_LOCAL_BASE:
        payload[2] = 1; /* RTK base */
        payload[3] = 1; /* survey */
        put_be32(&payload[4], 60); /* shortest documented survey */
        put_be32(&payload[8], 3);  /* strictest documented threshold */
        break;
    case GPS_MB_ROLE_PRECISE_BASE:
        payload[2] = 2; /* precisely kinematic base */
        payload[3] = 0; /* normal */
        break;
    case GPS_MB_ROLE_MOVING_ROVER:
        payload[2] = 0; /* rover */
        payload[3] = 2; /* moving base */
        put_be_float(&payload[32], 1.0f); /* measured T1-A2 baseline */
        break;
    case GPS_MB_ROLE_NONE:
    default:
        return ESP_ERR_INVALID_STATE;
    }

    /* Static coordinates are deliberately zero; A3 performs a local survey. */
    put_be_double(&payload[12], 0.0);
    put_be_double(&payload[20], 0.0);
    put_be_float(&payload[28], 0.0f);
    payload[36] = 0; /* SRAM only */
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t configure_advanced_moving_base_rate(void)
{
    const uint8_t payload[3] = {
        0x0E, /* Configure system position update rate */
        8,    /* 8 Hz: PX1105R Advanced Moving Base maximum */
        0,    /* SRAM only */
    };
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t configure_primary_binary_output(void)
{
    const uint8_t payload[3] = {
        0x09, /* Configure primary UART message type */
        2,    /* SkyTraq binary messages */
        0,    /* SRAM only */
    };
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t configure_local_base_rtcm(void)
{
    const uint8_t payload[17] = {
        0x20, /* Configure RTCM output */
        1,    /* enabled */
        6,    /* 8 Hz */
        1,    /* station ARP / 1005 */
        1,    /* GPS MSM4 */
        1,    /* GLONASS MSM4 */
        1,    /* Galileo MSM4 */
        0,    /* SBAS disabled */
        1,    /* QZSS MSM4 */
        1,    /* BeiDou MSM4 */
        10,   /* GPS ephemeris interval */
        10,   /* GLONASS ephemeris interval */
        10,   /* BeiDou ephemeris interval */
        10,   /* Galileo ephemeris interval */
        1,    /* MSM4 */
        2,    /* protocol version */
        0,    /* SRAM only */
    };
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t configure_precise_base_raw_output(void)
{
    const uint8_t payload[9] = {
        0x1E, /* Configure SkyTraq binary measurement output */
        0,    /* 1 Hz compatibility probe; raised after validation */
        0,    /* EXT_RAW_MEAS already carries the epoch time */
        0,    /* Legacy single-frequency RAW_MEAS */
        1,    /* Multi-frequency GNSS SV/channel status */
        1,    /* RCV_STATE at its documented 1 Hz rate */
        3,    /* GPS and GLONASS navigation subframes */
        1,    /* EXT_RAW_MEAS: PX1105R multi-frequency observations */
        0,    /* SRAM only */
    };
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t query_precise_base_raw_output(void)
{
    const uint8_t payload[1] = {0x1F};
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t query_software_version(void)
{
    /* SkyTraq QUERY SOFTWARE VERSION (0x02).  Software type 0 is the
     * protocol-defined compatibility query used by the vendor example. */
    const uint8_t payload[2] = {0x02, 0x00};
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t query_rtk_role(void)
{
    const uint8_t payload[2] = {0x6A, 0x07};
    return send_skytraq_payload(payload, sizeof(payload));
}

static esp_err_t correction_uart_init(void)
{
    if (!role_receives_corrections(s_snapshot.role)) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = BOARD_CONFIG_RTCM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* ESP-IDF requires an RX ring larger than the hardware FIFO even for a
     * TX-only UART.  No RX pin is assigned below, so these bytes are never
     * sourced from hardware; the buffer only satisfies the driver contract. */
    esp_err_t err = uart_driver_install(GPS_MB_CORRECTION_UART,
                                        GPS_MB_RX_BUFFER_SIZE,
                                        GPS_MB_TX_BUFFER_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(GPS_MB_CORRECTION_UART, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(GPS_MB_CORRECTION_UART,
                           BOARD_CONFIG_RTCM_TX_GPIO, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        (void)uart_driver_delete(GPS_MB_CORRECTION_UART);
        return err;
    }
    s_snapshot.correction_uart_ready = true;
    return ESP_OK;
}

static int ntrip_write_corrections(const uint8_t *data, size_t length,
                                   void *context)
{
    (void)context;
    if (!s_snapshot.correction_uart_ready || data == NULL || length == 0) {
        return -1;
    }
    return uart_write_bytes(GPS_MB_CORRECTION_UART, data, length);
}

static esp_err_t socket_init(void)
{
    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0) {
        return ESP_FAIL;
    }

    int flags = fcntl(s_socket, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(s_socket, F_SETFL, flags | O_NONBLOCK);
    }

    const struct sockaddr_in local_address = {
        .sin_family = AF_INET,
        .sin_port = htons(GPS_MB_MODULE_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_socket, (const struct sockaddr *)&local_address,
             sizeof(local_address)) < 0) {
        close(s_socket);
        s_socket = -1;
        return ESP_FAIL;
    }

    memset(&s_relay_address, 0, sizeof(s_relay_address));
    s_relay_address.sin_family = AF_INET;
    s_relay_address.sin_port = htons(GPS_MB_RELAY_PORT);
    if (inet_pton(AF_INET, APP_WIRELESS_LOG_TARGET,
                  &s_relay_address.sin_addr) != 1) {
        close(s_socket);
        s_socket = -1;
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t send_datagram(uint8_t kind, const uint8_t *payload,
                               size_t payload_length)
{
    if (s_socket < 0 || payload_length > GPS_MB_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t datagram[GPS_MB_HEADER_SIZE + GPS_MB_MAX_PAYLOAD] = {0};
    put_be32(&datagram[0], GPS_MB_MAGIC);
    datagram[4] = GPS_MB_VERSION;
    datagram[5] = kind;
    datagram[6] = s_module_id;
    datagram[7] = 0;
    const uint32_t sequence = kind == GPS_MB_KIND_STREAM
                                  ? ++s_uplink_sequence
                                  : 0;
    put_be32(&datagram[8], sequence);
    put_be16(&datagram[12], (uint16_t)payload_length);
    if (payload_length > 0 && payload != NULL) {
        memcpy(&datagram[GPS_MB_HEADER_SIZE], payload, payload_length);
    }

    const size_t total_length = GPS_MB_HEADER_SIZE + payload_length;
    const int sent = sendto(s_socket, datagram, total_length, MSG_DONTWAIT,
                            (const struct sockaddr *)&s_relay_address,
                            sizeof(s_relay_address));
    if (sent != (int)total_length) {
        s_snapshot.uplink_error_count++;
        return ESP_FAIL;
    }
    s_snapshot.uplink_packet_count++;
    s_snapshot.uplink_byte_count += (uint32_t)payload_length;
    s_last_uplink_ms = ticks_to_ms();
    return ESP_OK;
}

static bool downlink_source_allowed(uint8_t source_id)
{
    return (s_snapshot.role == GPS_MB_ROLE_PRECISE_BASE &&
            !gps_ntrip_client_is_enabled_for_module(s_module_id) &&
            source_id == 3) ||
           (s_snapshot.role == GPS_MB_ROLE_MOVING_ROVER && source_id == 1);
}

static void receive_datagrams(void)
{
    if (s_socket < 0 || !role_receives_corrections(s_snapshot.role) ||
        !s_snapshot.correction_uart_ready) {
        return;
    }

    uint8_t datagram[GPS_MB_HEADER_SIZE + GPS_MB_MAX_PAYLOAD] = {0};
    for (unsigned iteration = 0; iteration < 16; ++iteration) {
        const int received = recvfrom(s_socket, datagram, sizeof(datagram),
                                      MSG_DONTWAIT, NULL, NULL);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                s_snapshot.downlink_error_count++;
            }
            break;
        }
        if (received < GPS_MB_HEADER_SIZE ||
            get_be32(&datagram[0]) != GPS_MB_MAGIC ||
            datagram[4] != GPS_MB_VERSION ||
            datagram[5] != GPS_MB_KIND_STREAM ||
            !downlink_source_allowed(datagram[6])) {
            s_snapshot.downlink_error_count++;
            continue;
        }

        const uint16_t payload_length = get_be16(&datagram[12]);
        if ((size_t)received != GPS_MB_HEADER_SIZE + payload_length) {
            s_snapshot.downlink_error_count++;
            continue;
        }

        const uint32_t sequence = get_be32(&datagram[8]);
        if (s_last_downlink_sequence != 0 &&
            sequence != s_last_downlink_sequence + 1U) {
            s_snapshot.downlink_gap_count++;
        }
        s_last_downlink_sequence = sequence;

        const int written = uart_write_bytes(
            GPS_MB_CORRECTION_UART, &datagram[GPS_MB_HEADER_SIZE],
            payload_length);
        if (written != payload_length) {
            s_snapshot.downlink_error_count++;
            continue;
        }
        s_snapshot.downlink_packet_count++;
        s_snapshot.downlink_byte_count += payload_length;
        s_snapshot.last_downlink_source_id = datagram[6];
        s_last_downlink_ms = ticks_to_ms();
    }
}

static void process_skytraq_frame(const uint8_t *frame, size_t length)
{
    if (length < 8 || frame[0] != 0xA0 || frame[1] != 0xA1) {
        return;
    }
    const uint16_t payload_length = get_be16(&frame[2]);
    if ((size_t)payload_length + 7U != length || payload_length == 0) {
        return;
    }
    const uint8_t *payload = &frame[4];
    uint8_t checksum = 0;
    for (uint16_t index = 0; index < payload_length; ++index) {
        checksum ^= payload[index];
    }
    if (checksum != frame[4 + payload_length]) {
        return;
    }
    s_snapshot.skytraq_binary_frame_count++;
    if (payload[0] == 0x83) {
        s_snapshot.receiver_ack_count++;
        if (payload_length >= 2) {
            s_snapshot.receiver_last_ack_id = payload[1];
        }
    } else if (payload[0] == 0x84) {
        s_snapshot.receiver_nack_count++;
        if (payload_length >= 2) {
            s_snapshot.receiver_last_nack_id = payload[1];
        }
    } else if (payload[0] == 0x80 && payload_length >= 14) {
        s_snapshot.software_version_valid = true;
        s_snapshot.software_type = payload[1];
        s_snapshot.software_kernel_version = get_be32(&payload[2]);
        s_snapshot.software_odm_version = get_be32(&payload[6]);
        s_snapshot.software_revision = get_be32(&payload[10]);
    } else if (payload[0] == 0x89 && payload_length >= 8) {
        s_snapshot.binary_output_status_valid = true;
        s_snapshot.binary_output_rate_code = payload[1];
        s_snapshot.binary_meas_time_enabled = payload[2] == 1;
        /* Field 4 is legacy RAW_MEAS; field 8 is the extended raw format
         * used by multi-frequency receivers such as PX1105R. */
        s_snapshot.binary_raw_meas_enabled =
            payload[3] == 1 || payload[7] == 1;
    } else if (payload[0] == 0xDC) {
        s_snapshot.binary_meas_time_count++;
    } else if (payload[0] == 0xDD || payload[0] == 0xE5) {
        s_snapshot.binary_raw_meas_count++;
    }
}

static void feed_skytraq_parser(uint8_t byte)
{
    skytraq_stream_parser_t *parser = &s_binary_parser;
    if (parser->length == 0) {
        if (byte == 0xA0) {
            parser->buffer[parser->length++] = byte;
        }
        return;
    }
    if (parser->length == 1) {
        if (byte == 0xA1) {
            parser->buffer[parser->length++] = byte;
        } else if (byte != 0xA0) {
            parser->length = 0;
        }
        return;
    }
    if (parser->length >= sizeof(parser->buffer)) {
        parser->length = 0;
        parser->expected_length = 0;
        return;
    }
    parser->buffer[parser->length++] = byte;
    if (parser->length == 4) {
        const uint16_t payload_length = get_be16(&parser->buffer[2]);
        parser->expected_length = (size_t)payload_length + 7U;
        if (parser->expected_length > sizeof(parser->buffer) ||
            parser->expected_length < 8U) {
            parser->length = 0;
            parser->expected_length = 0;
        }
    }
    if (parser->expected_length > 0 &&
        parser->length == parser->expected_length) {
        process_skytraq_frame(parser->buffer, parser->length);
        parser->length = 0;
        parser->expected_length = 0;
    }
}

esp_err_t gps_moving_base_start(uart_port_t primary_uart, uint8_t module_id)
{
    gps_moving_base_stop();
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_binary_parser, 0, sizeof(s_binary_parser));
    s_primary_uart = primary_uart;
    s_module_id = module_id;
    s_snapshot.role = role_for_module(module_id);
    if (s_snapshot.role == GPS_MB_ROLE_NONE) {
        /* Software identity is useful on every physical module, including
         * anchors that do not participate in the moving-base transport.  The
         * normal GPS task still consumes the binary 0x80 response below. */
        const esp_err_t version_err = query_software_version();
        if (version_err != ESP_OK) {
            ESP_LOGW(TAG, "Software-version query failed: %s",
                     esp_err_to_name(version_err));
        }
        return ESP_OK;
    }

    esp_err_t err = socket_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Relay socket init failed: %s", esp_err_to_name(err));
        gps_moving_base_stop();
        return err;
    }
    err = correction_uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Correction UART init failed: %s", esp_err_to_name(err));
        gps_moving_base_stop();
        return err;
    }

    s_snapshot.active = true;
    s_started_ms = ticks_to_ms();
    err = gps_ntrip_client_start(s_module_id, ntrip_write_corrections, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NTRIP client start failed: %s", esp_err_to_name(err));
        gps_moving_base_stop();
        return err;
    }
    ESP_LOGI(TAG, "Local moving-base role=%s module=%u relay=%s:%u",
             gps_moving_base_role_to_string(s_snapshot.role),
             (unsigned)s_module_id, APP_WIRELESS_LOG_TARGET,
             (unsigned)GPS_MB_RELAY_PORT);
    return ESP_OK;
}

void gps_moving_base_stop(void)
{
    gps_ntrip_client_stop();
    if (s_snapshot.correction_uart_ready) {
        (void)uart_driver_delete(GPS_MB_CORRECTION_UART);
    }
    if (s_socket >= 0) {
        close(s_socket);
    }
    s_socket = -1;
    s_primary_uart = UART_NUM_MAX;
    s_snapshot.active = false;
    s_snapshot.correction_uart_ready = false;
    s_started_ms = 0;
    s_last_heartbeat_ms = 0;
    s_last_uplink_ms = 0;
    s_last_downlink_ms = 0;
    s_uplink_sequence = 0;
    s_last_downlink_sequence = 0;
    s_config_step = 0;
}

void gps_moving_base_set_gga(const char *sentence)
{
    gps_ntrip_client_set_gga(sentence);
}

void gps_moving_base_process_primary_bytes(const uint8_t *data, size_t length)
{
    if (s_primary_uart == UART_NUM_MAX || data == NULL || length == 0) {
        return;
    }
    for (size_t index = 0; index < length; ++index) {
        if (data[index] == 0xD3) {
            s_snapshot.rtcm_preamble_count++;
        }
        feed_skytraq_parser(data[index]);
    }
    if (!s_snapshot.active) {
        return;
    }
    if (!role_forwards_primary(s_snapshot.role)) {
        return;
    }
    size_t offset = 0;
    while (offset < length) {
        size_t chunk = length - offset;
        if (chunk > GPS_MB_MAX_PAYLOAD) {
            chunk = GPS_MB_MAX_PAYLOAD;
        }
        (void)send_datagram(GPS_MB_KIND_STREAM, data + offset, chunk);
        offset += chunk;
    }
}

static void configure_receiver_if_due(uint32_t now_ms)
{
    if (!s_snapshot.active || s_snapshot.receiver_config_sent ||
        (uint32_t)(now_ms - s_started_ms) < GPS_MB_CONFIG_START_DELAY_MS) {
        return;
    }
    const uint32_t due_ms = GPS_MB_CONFIG_START_DELAY_MS +
                            ((uint32_t)s_config_step *
                             GPS_MB_CONFIG_STEP_DELAY_MS);
    if ((uint32_t)(now_ms - s_started_ms) < due_ms) {
        return;
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    bool final_step = false;
    switch (s_snapshot.role) {
    case GPS_MB_ROLE_LOCAL_BASE:
        if (s_config_step == 0) {
            err = query_software_version();
        } else if (s_config_step == 1) {
            err = configure_rtk_role();
        } else if (s_config_step == 2) {
            err = configure_local_base_rtcm();
        } else {
            err = query_rtk_role();
            final_step = true;
        }
        break;
    case GPS_MB_ROLE_PRECISE_BASE:
        if (s_config_step == 0) {
            err = query_software_version();
        } else if (s_config_step == 1) {
            err = configure_advanced_moving_base_rate();
        } else if (s_config_step == 2) {
            /* The extended raw stream is an RTK-base output.  Enter the
             * final Advanced Moving Base role before configuring it. */
            err = configure_rtk_role();
        } else if (s_config_step == 3) {
            err = configure_precise_base_raw_output();
        } else if (s_config_step == 4) {
            /* Confirm that raw carrier-phase epochs are enabled. */
            err = query_precise_base_raw_output();
        } else if (s_config_step == 5) {
            err = configure_primary_binary_output();
        } else {
            err = query_rtk_role();
            final_step = true;
        }
        break;
    case GPS_MB_ROLE_MOVING_ROVER:
        if (s_config_step == 0) {
            err = query_software_version();
        } else if (s_config_step == 1) {
            err = configure_rtk_role();
        } else {
            err = query_rtk_role();
            final_step = true;
        }
        break;
    case GPS_MB_ROLE_NONE:
    default:
        return;
    }
    if (final_step) {
        s_snapshot.receiver_config_sent = err == ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Receiver configuration step %u failed: %s",
                 (unsigned)s_config_step, esp_err_to_name(err));
    }
    s_config_step++;
}

void gps_moving_base_poll(void)
{
    if (!s_snapshot.active) {
        return;
    }
    const uint32_t now_ms = ticks_to_ms();
    configure_receiver_if_due(now_ms);
    receive_datagrams();
    if (s_last_heartbeat_ms == 0 ||
        (uint32_t)(now_ms - s_last_heartbeat_ms) >=
            GPS_MB_HEARTBEAT_INTERVAL_MS) {
        (void)send_datagram(GPS_MB_KIND_HEARTBEAT, NULL, 0);
        s_last_heartbeat_ms = now_ms;
    }
}

void gps_moving_base_get_snapshot(gps_moving_base_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    *snapshot = s_snapshot;
    const uint32_t now_ms = ticks_to_ms();
    snapshot->last_uplink_age_ms = elapsed_since(now_ms, s_last_uplink_ms);
    snapshot->last_downlink_age_ms = elapsed_since(now_ms, s_last_downlink_ms);
    gps_ntrip_snapshot_t ntrip = {0};
    gps_ntrip_client_get_snapshot(&ntrip);
    snapshot->ntrip_configured = ntrip.configured;
    snapshot->ntrip_running = ntrip.running;
    snapshot->ntrip_tls_connected = ntrip.tls_connected;
    snapshot->ntrip_stream_active = ntrip.stream_active;
    snapshot->ntrip_http_status = ntrip.http_status;
    snapshot->ntrip_connect_count = ntrip.connect_count;
    snapshot->ntrip_reconnect_count = ntrip.reconnect_count;
    snapshot->ntrip_error_count = ntrip.error_count;
    snapshot->ntrip_rtcm_frame_count = ntrip.rtcm_frame_count;
    snapshot->ntrip_rtcm_byte_count = ntrip.rtcm_byte_count;
    snapshot->ntrip_last_data_age_ms = ntrip.last_data_age_ms;
    snprintf(snapshot->ntrip_state, sizeof(snapshot->ntrip_state), "%s",
             ntrip.state);
}
