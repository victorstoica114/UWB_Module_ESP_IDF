#include "uwb_dw3000.h"

#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "app_identity.h"
#include "uwb_config.h"

static const char *TAG = "uwb_dw3000";

enum {
    UWB_DW3000_TASK_STACK_WORDS = 8192,
    UWB_DW3000_TASK_PRIORITY = 8,
    UWB_DW3000_SPI_CLOCK_HZ = 4 * 1000 * 1000,
    UWB_DW3000_SPI_MAX_TRANSFER_BYTES = 96,
    UWB_DW3000_RESET_SETTLE_MS = 5,
    UWB_DW3000_RESET_PULSE_MS = 20,
    UWB_DW3000_WAKE_AFTER_RESET_MS = 300,
    UWB_DW3000_PROBE_ATTEMPTS = 16,
    UWB_DW3000_PROBE_INTERVAL_MS = 40,
    UWB_DW3000_IDLE_TIMEOUT_MS = 1000,
    UWB_DW3000_POLL_INTERVAL_MS = 5,
    UWB_DW3000_TX_TIMEOUT_MS = 120,
    UWB_DW3000_TX_POLL_MS = 2,
    UWB_DW3000_PAYLOAD_LEN = 48,
    UWB_DW3000_ANTENNA_DELAY = APP_UWB_ANTENNA_DELAY_DEFAULT,
};

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
#define UWB_DW3000_TASK_CORE 1
#else
#define UWB_DW3000_TASK_CORE 0
#endif

#define UWB_DW3000_SPI_HOST SPI2_HOST

#define DW3000_REG_GEN_CFG_AES_LOW 0x00
#define DW3000_REG_GEN_CFG_AES_HIGH 0x01
#define DW3000_REG_STS_CFG 0x02
#define DW3000_REG_RX_TUNE 0x03
#define DW3000_REG_EXT_SYNC 0x04
#define DW3000_REG_GPIO_CTRL 0x05
#define DW3000_REG_DRX 0x06
#define DW3000_REG_RF_CONF 0x07
#define DW3000_REG_FS_CTRL 0x09
#define DW3000_REG_AON 0x0A
#define DW3000_REG_OTP_IF 0x0B
#define DW3000_REG_CIA_1 0x0C
#define DW3000_REG_CIA_3 0x0E
#define DW3000_REG_DIG_DIAG 0x0F
#define DW3000_REG_PMSC 0x11
#define DW3000_REG_RX_BUFFER_0 0x12
#define DW3000_REG_TX_BUFFER 0x14

#define DW3000_SUB_NONE 0x00
#define DW3000_DEV_ID_DW3000 0xDECA0302UL
#define DW3000_DEV_ID_DW3120 0xDECA0312UL

#define DW3000_CMD_TXRXOFF 0x00
#define DW3000_CMD_TX 0x01
#define DW3000_CMD_RX 0x02
#define DW3000_CMD_DTX 0x03

#define DW3000_DX_TIME_SUB 0x2C
#define DW3000_RX_TIME_SUB 0x00
#define DW3000_TX_TIME_SUB 0x74

#define DW3000_GPIO_MODE_SUB 0x00
#define DW3000_GPIO_DIR_SUB 0x08
#define DW3000_PMSC_CLK_CTRL_SUB 0x04
#define DW3000_PMSC_LED_CTRL_SUB 0x16

#define DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK 0x7UL
#define DW3000_GPIO_MODE_MSGP1_MODE_BIT_MASK 0x38UL
#define DW3000_GPIO_MODE_MSGP2_MODE_BIT_MASK 0x1C0UL
#define DW3000_GPIO_MODE_MSGP3_MODE_BIT_MASK 0xE00UL
#define DW3000_GPIO_PIN0_RXOKLED 0x1UL
#define DW3000_GPIO_PIN1_SFDLED (1UL << (1U * 3U))
#define DW3000_GPIO_PIN2_RXLED (1UL << (2U * 3U))
#define DW3000_GPIO_PIN3_TXLED (1UL << (3U * 3U))

#define DW3000_CLK_CTRL_GPIO_DCLK_EN_BIT_MASK 0x40000UL
#define DW3000_CLK_CTRL_LP_CLK_EN_BIT_MASK 0x800000UL

#define DW3000_LED_CTRL_BLINK_EN_BIT_MASK 0x100UL
#define DW3000_LED_CTRL_FORCE_TRIGGER_BIT_MASK 0xF0000UL
#define DW3000_LED_CTRL_BLINK_TIME_MASK 0xFFUL

#define DW3000_STATUS_TXFRS 0x00000080UL
#define DW3000_STATUS_RXFR 0x00002000UL
#define DW3000_STATUS_RXFCG 0x00004000UL
#define DW3000_STATUS_RXPHE 0x00001000UL
#define DW3000_STATUS_RXFCE 0x00008000UL
#define DW3000_STATUS_RXFSL 0x00010000UL
#define DW3000_STATUS_RXFTO 0x00020000UL
#define DW3000_STATUS_CIAERR 0x00040000UL
#define DW3000_STATUS_RXPTO 0x00200000UL
#define DW3000_STATUS_RXSTO 0x04000000UL
#define DW3000_STATUS_HPDWARN 0x08000000UL
#define DW3000_STATUS_CPERR 0x10000000UL
#define DW3000_STATUS_ARFE 0x20000000UL
#define DW3000_STATUS_SPIRDY 0x00000080UL
#define DW3000_STATUS_RCINIT 0x00000100UL
#define DW3000_STATUS_CLEAR_MASK 0x3F7FFFFFUL

#define DW3000_RX_GOOD_MASK DW3000_STATUS_RXFCG
#define DW3000_RX_ERROR_MASK                                             \
    (DW3000_STATUS_RXPHE | DW3000_STATUS_RXFCE | DW3000_STATUS_RXFSL |   \
     DW3000_STATUS_CIAERR | DW3000_STATUS_ARFE)
#define DW3000_RX_TIMEOUT_MASK \
    (DW3000_STATUS_RXFTO | DW3000_STATUS_RXPTO | DW3000_STATUS_RXSTO | DW3000_STATUS_CPERR)
#define DW3000_RX_FINFO_RXFLEN_MASK 0x000003FFUL
#define DW3000_TX_FCTRL_TXB_OFFSET_MASK 0x03FF0000UL
#define DW3000_TX_FCTRL_TR_MASK 0x00000800UL
#define DW3000_TX_FCTRL_TXFLEN_MASK 0x000003FFUL

#define UWB_DW3000_TIMESTAMP_MASK ((1ULL << 40U) - 1ULL)
#define UWB_DW3000_DELAYED_TIME_MASK 0xFFFFFFFEUL
#define UWB_DW3000_TIME_UNIT_SECONDS 15.650040064102564e-12
#define UWB_DW3000_SPEED_OF_LIGHT_MPS 299702547.0

#define UWB_DISTANCE_FRAME_MAGIC_0 'U'
#define UWB_DISTANCE_FRAME_MAGIC_1 'W'
#define UWB_DISTANCE_FRAME_MAGIC_2 'B'
#define UWB_DISTANCE_FRAME_MAGIC_3 'R'
#define UWB_DISTANCE_FRAME_VERSION 1U
#define UWB_DISTANCE_FRAME_HEADER_LEN 10U
#define UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET 10U
#define UWB_DISTANCE_FRAME_POLL_RX_TS_OFFSET 15U
#define UWB_DISTANCE_FRAME_RESP_TX_TS_OFFSET 20U
#define UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET 25U
#define UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET 30U
#define UWB_DISTANCE_FRAME_FINAL_RX_TS_OFFSET 35U

enum uwb_distance_frame_type {
    UWB_DISTANCE_FRAME_POLL = 1,
    UWB_DISTANCE_FRAME_RESP = 2,
    UWB_DISTANCE_FRAME_FINAL = 3,
    UWB_DISTANCE_FRAME_REPORT = 4,
};

enum uwb_dw3000_runtime_mode {
    UWB_DW3000_RUNTIME_BEACON_SMOKE = 0,
    UWB_DW3000_RUNTIME_DISTANCE_TEST,
    UWB_DW3000_RUNTIME_CALIBRATION,
};

struct uwb_dw3000_rx_frame {
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN];
    uint16_t payload_len;
    uint64_t rx_timestamp;
};

struct uwb_distance_frame {
    uint8_t type;
    uint8_t source_id;
    uint8_t destination_id;
    uint16_t sequence;
    uint64_t rx_timestamp;
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN];
    uint16_t payload_len;
};

struct uwb_distance_measurement {
    uint8_t initiator_id;
    uint8_t responder_id;
    uint16_t sequence;
    double tof_dtu;
    double distance_m;
    uint64_t poll_tx_ts;
    uint64_t poll_rx_ts;
    uint64_t resp_tx_ts;
    uint64_t resp_rx_ts;
    uint64_t final_tx_ts;
    uint64_t final_rx_ts;
};

struct uwb_calibration_stats {
    uint32_t samples;
    double mean_m;
    double m2_m;
    double min_m;
    double max_m;
    double last_m;
};

static esp_err_t uwb_dw3000_send_payload(const uint8_t *payload,
                                         size_t payload_len,
                                         uint64_t *tx_timestamp);
static esp_err_t uwb_dw3000_send_payload_delayed(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp);

#define DW3000_PMSC_STATE_IDLE 0x03

static spi_device_handle_t s_spi;
static bool s_started;
static bool s_rx_armed;
static uint8_t s_source_id;
static uint32_t s_device_id;
static enum uwb_dw3000_runtime_mode s_runtime_mode =
    UWB_DW3000_RUNTIME_BEACON_SMOKE;
static volatile enum uwb_dw3000_status s_status = UWB_DW3000_STATUS_IDLE;
static volatile uint32_t s_tx_count;
static volatile uint32_t s_tx_error_count;
static volatile uint32_t s_rx_count;
static volatile uint32_t s_rx_error_count;
static volatile uint32_t s_rx_ignored_count;
static volatile uint8_t s_last_rx_source_id;
static volatile uint32_t s_last_rx_sequence;

static int gpio_level_active(int active_high)
{
    return active_high ? 1 : 0;
}

static int gpio_level_inactive(int active_high)
{
    return active_high ? 0 : 1;
}

static void uwb_dw3000_delay_ms(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

static bool uwb_dw3000_device_id_valid(uint32_t device_id)
{
    return device_id == DW3000_DEV_ID_DW3000 ||
           device_id == DW3000_DEV_ID_DW3120;
}

static uint8_t uwb_dw3000_pick_source_id(void)
{
    if (APP_UWB_SOURCE_ID > 0 && APP_UWB_SOURCE_ID <= 255) {
        return (uint8_t)APP_UWB_SOURCE_ID;
    }

    if (app_identity_init() == ESP_OK) {
        const uint8_t module_id = app_identity_get_module_id();
        if (module_id != 0) {
            return module_id;
        }
    }

    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK && mac[5] != 0) {
        return mac[5];
    }

    const uint8_t random_id = (uint8_t)(esp_random() & 0xFFU);
    return random_id != 0 ? random_id : 1;
}

static esp_err_t uwb_dw3000_configure_gpio(void)
{
    const uint64_t output_pin_mask =
        (1ULL << BOARD_CONFIG_UWB_CS_GPIO) |
        (1ULL << BOARD_CONFIG_UWB_WAKEUP_GPIO) |
        (1ULL << BOARD_CONFIG_UWB_RST_GPIO);

    const gpio_config_t output_config = {
        .pin_bit_mask = output_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG,
                        "configure UWB output GPIOs failed");

    const gpio_config_t irq_config = {
        .pin_bit_mask = 1ULL << BOARD_CONFIG_UWB_IRQ_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&irq_config), TAG,
                        "configure UWB IRQ GPIO failed");

    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_CONFIG_UWB_CS_GPIO, 1), TAG,
                        "set UWB CS high failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_WAKEUP_GPIO,
                       gpio_level_active(BOARD_CONFIG_UWB_WAKEUP_ACTIVE_HIGH)),
        TAG, "set UWB WAKEUP active failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_RST_GPIO,
                       gpio_level_inactive(BOARD_CONFIG_UWB_RST_ACTIVE_HIGH)),
        TAG, "set UWB RST inactive failed");

    return ESP_OK;
}

static esp_err_t uwb_dw3000_hardware_reset(void)
{
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_RST_GPIO,
                       gpio_level_inactive(BOARD_CONFIG_UWB_RST_ACTIVE_HIGH)),
        TAG, "set UWB RST inactive failed");
    uwb_dw3000_delay_ms(UWB_DW3000_RESET_SETTLE_MS);

    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_RST_GPIO,
                       gpio_level_active(BOARD_CONFIG_UWB_RST_ACTIVE_HIGH)),
        TAG, "set UWB RST active failed");
    uwb_dw3000_delay_ms(UWB_DW3000_RESET_PULSE_MS);

    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_RST_GPIO,
                       gpio_level_inactive(BOARD_CONFIG_UWB_RST_ACTIVE_HIGH)),
        TAG, "release UWB RST failed");
    return ESP_OK;
}

static esp_err_t uwb_dw3000_init_spi(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = BOARD_CONFIG_SPI_MOSI_GPIO,
        .miso_io_num = BOARD_CONFIG_SPI_MISO_GPIO,
        .sclk_io_num = BOARD_CONFIG_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = UWB_DW3000_SPI_MAX_TRANSFER_BYTES,
    };

    esp_err_t err =
        spi_bus_initialize(UWB_DW3000_SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = UWB_DW3000_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = BOARD_CONFIG_UWB_CS_GPIO,
        .queue_size = 1,
    };

    err = spi_bus_add_device(UWB_DW3000_SPI_HOST, &device_config, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static size_t uwb_dw3000_build_header(uint8_t *header, size_t header_size,
                                      uint8_t base, uint8_t sub,
                                      bool write)
{
    if (header_size < 1) {
        return 0;
    }

    const uint32_t reg_file_id =
        ((uint32_t)(base & 0x1FU) << 16) | (uint32_t)sub;
    const uint16_t reg_file =
        (uint16_t)(0x1FU & (reg_file_id >> 16));
    const uint16_t reg_offset = (uint16_t)(0x7FU & reg_file_id);
    const uint16_t address =
        (uint16_t)((reg_file << 9) | (reg_offset << 2));
    const uint16_t mode = write ? 0x8000U : 0x0000U;

    header[0] = (uint8_t)((mode | address) >> 8);
    if (reg_offset == 0) {
        return 1;
    }

    if (header_size < 2) {
        return 0;
    }

    header[0] |= 0x40U;
    header[1] = (uint8_t)address;
    return 2;
}

static esp_err_t uwb_dw3000_read_bytes(uint8_t base, uint8_t sub, uint8_t *data,
                                       size_t len)
{
    uint8_t tx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES] = {0};
    uint8_t rx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES] = {0};
    const size_t header_len =
        uwb_dw3000_build_header(tx, sizeof(tx), base, sub, false);
    const size_t total_len = header_len + len;

    if (data == NULL || header_len == 0 ||
        total_len > UWB_DW3000_SPI_MAX_TRANSFER_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t transaction = {
        .length = total_len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    ESP_RETURN_ON_ERROR(spi_device_transmit(s_spi, &transaction), TAG,
                        "SPI read failed");
    memcpy(data, rx + header_len, len);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_write_bytes(uint8_t base, uint8_t sub,
                                        const uint8_t *data, size_t len)
{
    uint8_t tx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES] = {0};
    const size_t header_len =
        uwb_dw3000_build_header(tx, sizeof(tx), base, sub, true);
    const size_t total_len = header_len + len;

    if (data == NULL || header_len == 0 ||
        total_len > UWB_DW3000_SPI_MAX_TRANSFER_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(tx + header_len, data, len);

    spi_transaction_t transaction = {
        .length = total_len * 8,
        .tx_buffer = tx,
    };

    ESP_RETURN_ON_ERROR(spi_device_transmit(s_spi, &transaction), TAG,
                        "SPI write failed");
    return ESP_OK;
}

static size_t uwb_dw3000_auto_len(uint32_t value)
{
    size_t len = 1;
    while (len < sizeof(uint32_t) && (value >> (len * 8U)) != 0U) {
        len++;
    }
    return len;
}

static esp_err_t uwb_dw3000_write_u32_len(uint8_t base, uint8_t sub,
                                          uint32_t value, size_t len)
{
    uint8_t bytes[sizeof(uint32_t)] = {0};
    if (len == 0 || len > sizeof(bytes)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < len; ++i) {
        bytes[i] = (uint8_t)((value >> (8U * i)) & 0xFFU);
    }

    return uwb_dw3000_write_bytes(base, sub, bytes, len);
}

static esp_err_t uwb_dw3000_write_u32_auto(uint8_t base, uint8_t sub,
                                           uint32_t value)
{
    return uwb_dw3000_write_u32_len(base, sub, value,
                                    uwb_dw3000_auto_len(value));
}

static esp_err_t uwb_dw3000_read32(uint8_t base, uint8_t sub, uint32_t *value)
{
    uint8_t bytes[sizeof(uint32_t)] = {0};
    ESP_RETURN_ON_ERROR(uwb_dw3000_read_bytes(base, sub, bytes, sizeof(bytes)),
                        TAG, "read32 failed");

    *value = ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_read_timestamp40(uint8_t base, uint8_t sub,
                                             uint64_t *timestamp)
{
    uint8_t bytes[5] = {0};
    if (timestamp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(uwb_dw3000_read_bytes(base, sub, bytes, sizeof(bytes)),
                        TAG, "timestamp read failed");
    *timestamp = ((uint64_t)bytes[0]) | ((uint64_t)bytes[1] << 8) |
                 ((uint64_t)bytes[2] << 16) |
                 ((uint64_t)bytes[3] << 24) |
                 ((uint64_t)bytes[4] << 32);
    *timestamp &= UWB_DW3000_TIMESTAMP_MASK;
    return ESP_OK;
}

static esp_err_t uwb_dw3000_read_rx_timestamp(uint64_t *timestamp)
{
    return uwb_dw3000_read_timestamp40(DW3000_REG_CIA_1, DW3000_RX_TIME_SUB,
                                       timestamp);
}

static esp_err_t uwb_dw3000_read_tx_timestamp(uint64_t *timestamp)
{
    return uwb_dw3000_read_timestamp40(DW3000_REG_GEN_CFG_AES_LOW,
                                       DW3000_TX_TIME_SUB, timestamp);
}

static uint64_t uwb_dw3000_add_timestamp_delta(uint64_t timestamp,
                                               uint64_t delta)
{
    return (timestamp + delta) & UWB_DW3000_TIMESTAMP_MASK;
}

static uint64_t uwb_dw3000_ms_to_dtu(uint32_t delay_ms)
{
    const double delay_seconds = (double)delay_ms / 1000.0;
    return (uint64_t)((delay_seconds / UWB_DW3000_TIME_UNIT_SECONDS) + 0.5);
}

static uint32_t uwb_dw3000_delayed_time_word(uint64_t tx_timestamp)
{
    return (uint32_t)(tx_timestamp >> 8U);
}

static uint64_t uwb_dw3000_programmed_tx_timestamp(uint32_t delayed_time_word)
{
    return ((((uint64_t)(delayed_time_word & UWB_DW3000_DELAYED_TIME_MASK))
             << 8U) +
            UWB_DW3000_ANTENNA_DELAY) &
           UWB_DW3000_TIMESTAMP_MASK;
}

static esp_err_t uwb_dw3000_set_delayed_trx_time(uint32_t delayed_time_word)
{
    return uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                    DW3000_DX_TIME_SUB, delayed_time_word, 4);
}

static esp_err_t uwb_dw3000_update_u32(uint8_t base, uint8_t sub,
                                       uint32_t clear_mask,
                                       uint32_t set_mask)
{
    uint32_t value = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(base, sub, &value), TAG,
                        "read before register update failed");
    value &= ~clear_mask;
    value |= set_mask;
    return uwb_dw3000_write_u32_len(base, sub, value, sizeof(value));
}

static esp_err_t uwb_dw3000_read_rx_payload(
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN], uint16_t *payload_len)
{
    if (payload == NULL || payload_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t rx_finfo = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x4C, &rx_finfo), TAG,
        "RX_FINFO read failed");

    const uint16_t frame_len =
        (uint16_t)(rx_finfo & DW3000_RX_FINFO_RXFLEN_MASK);
    if (frame_len <= 2U) {
        *payload_len = 0;
        return ESP_OK;
    }

    uint16_t read_len = (uint16_t)(frame_len - 2U);
    if (read_len > UWB_DW3000_PAYLOAD_LEN) {
        read_len = UWB_DW3000_PAYLOAD_LEN;
    }

    memset(payload, 0, UWB_DW3000_PAYLOAD_LEN);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read_bytes(DW3000_REG_RX_BUFFER_0, DW3000_SUB_NONE, payload,
                              read_len),
        TAG, "RX buffer read failed");
    *payload_len = read_len;
    return ESP_OK;
}

static esp_err_t uwb_dw3000_write_tx_payload(const uint8_t *payload,
                                             size_t payload_len)
{
    if (payload == NULL || payload_len == 0 ||
        payload_len > UWB_DW3000_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    return uwb_dw3000_write_bytes(DW3000_REG_TX_BUFFER, DW3000_SUB_NONE,
                                  payload, payload_len);
}

static esp_err_t uwb_dw3000_fast_command(uint8_t command)
{
    const uint8_t header = (uint8_t)(0x81U | ((command & 0x1FU) << 1));
    spi_transaction_t transaction = {
        .length = 8,
        .tx_buffer = &header,
    };
    return spi_device_transmit(s_spi, &transaction);
}

static esp_err_t uwb_dw3000_clear_status(void)
{
    return uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x44,
                                    DW3000_STATUS_CLEAR_MASK, 4);
}

static esp_err_t uwb_dw3000_read_otp(uint8_t address, uint32_t *value)
{
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_OTP_IF, 0x04, address), TAG,
        "OTP address write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_OTP_IF, 0x08, 0x02), TAG,
        "OTP command write failed");
    uwb_dw3000_delay_ms(1);
    return uwb_dw3000_read32(DW3000_REG_OTP_IF, 0x10, value);
}

static bool uwb_dw3000_is_idle(void)
{
    uint32_t diag = 0;
    uint32_t status = 0;

    if (uwb_dw3000_read32(DW3000_REG_DIG_DIAG, 0x30, &diag) != ESP_OK ||
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44, &status) !=
            ESP_OK) {
        return false;
    }

    const bool pmsc_idle =
        (((diag >> 16) & DW3000_PMSC_STATE_IDLE) == DW3000_PMSC_STATE_IDLE);
    const bool init_ready =
        (((status >> 16) & (DW3000_STATUS_SPIRDY | DW3000_STATUS_RCINIT)) ==
         (DW3000_STATUS_SPIRDY | DW3000_STATUS_RCINIT));
    return pmsc_idle || init_ready;
}

static esp_err_t uwb_dw3000_wait_idle(uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    while (!uwb_dw3000_is_idle()) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        uwb_dw3000_delay_ms(20);
    }
    return ESP_OK;
}

static esp_err_t uwb_dw3000_clear_aon_config(void)
{
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_AON, DW3000_SUB_NONE, 0x00, 2),
        TAG, "AON clear data failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_AON, 0x14, 0x00, 1),
                        TAG, "AON clear 0x14 failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_AON, 0x04, 0x00, 1),
                        TAG, "AON clear control failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_AON, 0x04, 0x02),
                        TAG, "AON trigger clear failed");
    uwb_dw3000_delay_ms(1);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_soft_reset(void)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_aon_config(), TAG,
                        "clear AON config failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x04, 0x01), TAG,
        "force FAST_RC failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_PMSC, 0x00, 0, 2),
                        TAG, "PMSC reset assert failed");
    uwb_dw3000_delay_ms(100);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x00, 0xFFFF), TAG,
        "PMSC reset release failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_PMSC, 0x04, 0, 1),
                        TAG, "PMSC clock auto failed");
    return ESP_OK;
}

static esp_err_t uwb_dw3000_write_sys_config(void)
{
    const uint8_t channel = 0x00;
    const uint8_t preamble_len = 0x05;
    const uint8_t preamble_code = 10;
    const uint8_t pac = 0x00;
    const uint8_t datarate = 0x01;
    const uint8_t phr_mode = 0x00;
    const uint8_t phr_rate = 0x01;

    const uint32_t usr_cfg = (0x188U & 0xFFFU) |
                             ((uint32_t)phr_mode << 3) |
                             ((uint32_t)phr_rate << 4);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_GEN_CFG_AES_LOW, 0x10, usr_cfg),
        TAG, "SYS_CFG write failed");

    uint32_t otp_write = 0x1400;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_OTP_IF, 0x08, otp_write), TAG,
        "OTP config write failed");

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_DRX, 0x00, 0, 1),
                        TAG, "DTUNE0 reset failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x00, pac),
                        TAG, "PAC write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_STS_CFG, 0x00, (64 / 8) - 1),
        TAG, "STS config write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x29, 0, 1),
        TAG, "AES config write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x0C, 0xAF5F584C), TAG,
        "DRX tune write failed");

    uint32_t chan_ctrl = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_HIGH, 0x14, &chan_ctrl),
        TAG, "CHAN_CTRL read failed");
    chan_ctrl &= ~0x1FFFUL;
    chan_ctrl |= channel;
    chan_ctrl |= (0x1F00UL & ((uint32_t)preamble_code << 8));
    chan_ctrl |= (0x00F8UL & ((uint32_t)preamble_code << 3));
    chan_ctrl |= 0x02UL;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_HIGH, 0x14, chan_ctrl,
                                 4),
        TAG, "CHAN_CTRL write failed");

    uint32_t tx_fctrl = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x24, &tx_fctrl), TAG,
        "TX_FCTRL read failed");
    tx_fctrl |= ((uint32_t)preamble_len << 12);
    tx_fctrl |= ((uint32_t)datarate << 10);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x24, tx_fctrl,
                                 4),
        TAG, "TX_FCTRL write failed");

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x02, 0x81),
                        TAG, "DRX 0x02 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C, 0x1C071134),
        TAG, "RF_TX_CTRL_2 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x00, 0x0F3C), TAG,
        "PLL_CFG write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x51, 0x14),
                        TAG, "RF 0x51 write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1A, 0x0E),
                        TAG, "RF_TX_CTRL_1 write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x08, 0x81),
                        TAG, "PLL_CAL write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_GEN_CFG_AES_LOW, 0x44, 0x02),
        TAG, "SYS_STATUS init clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x04, 0x300200), TAG,
        "PMSC auto clock write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x08, 0x0138), TAG,
        "PMSC 0x08 write failed");

    uint32_t otp_val = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_OTP_IF, 0x08, &otp_val),
                        TAG, "OTP config read failed");
    otp_val |= 0x40;
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_OTP_IF, 0x08, otp_val),
                        TAG, "OTP config update failed");

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x19, 0xF0),
                        TAG, "RX tune write failed");

    uint32_t ldo_ctrl = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_RF_CONF, 0x48, &ldo_ctrl),
                        TAG, "LDO control read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x48, 0x105 | 0x100 |
                                                               0x04 | 0x01),
        TAG, "temporary LDO write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_EXT_SYNC, 0x0C, 0x020000), TAG,
        "PGF calibration prepare failed");
    uwb_dw3000_delay_ms(20);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_EXT_SYNC, 0x0C, 0x11), TAG,
        "PGF calibration start failed");

    bool pgf_done = false;
    for (int i = 0; i < 100; ++i) {
        uint32_t pgf_status = 0;
        if (uwb_dw3000_read32(DW3000_REG_EXT_SYNC, 0x20, &pgf_status) ==
                ESP_OK &&
            pgf_status != 0) {
            pgf_done = true;
            break;
        }
        uwb_dw3000_delay_ms(10);
    }
    if (!pgf_done) {
        ESP_LOGW(TAG, "DW3000 PGF calibration did not report completion");
    }

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_EXT_SYNC, 0x0C, 0, 1),
                        TAG, "PGF calibration stop failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_EXT_SYNC, 0x20, 0x01),
                        TAG, "PGF status clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x48, ldo_ctrl), TAG,
        "LDO restore failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_CIA_3, 0x02, 0x01),
                        TAG, "CIA diagnostics enable failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_CIA_3, 0x00,
                                 UWB_DW3000_ANTENNA_DELAY, 2),
        TAG, "RX antenna delay write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_HIGH, 0x04,
                                 UWB_DW3000_ANTENNA_DELAY, 2),
        TAG, "TX antenna delay write failed");
    ESP_LOGI(TAG, "DW3000 antenna delay set: rx=0x%04x tx=0x%04x",
             (unsigned)UWB_DW3000_ANTENNA_DELAY,
             (unsigned)UWB_DW3000_ANTENNA_DELAY);

    return ESP_OK;
}

static esp_err_t uwb_dw3000_configure_hardware_leds(void)
{
    if (!APP_UWB_DW_LEDS_ENABLED) {
        ESP_LOGI(TAG, "DW3000 hardware LEDs disabled");
        return ESP_OK;
    }

    uint32_t gpio_led_mask = 0;
    uint32_t gpio_led_mode = 0;

    if (APP_UWB_DW_RXOK_LED_ENABLED) {
        if (BOARD_CONFIG_UWB_RXOK_DW_LED_INDEX != 0) {
            ESP_LOGE(TAG, "RXOKLED requires DW3000 GPIO0, got GPIO%d",
                     BOARD_CONFIG_UWB_RXOK_DW_LED_INDEX);
            return ESP_ERR_NOT_SUPPORTED;
        }
        gpio_led_mask |= DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK;
        gpio_led_mode |= DW3000_GPIO_PIN0_RXOKLED;
    }

    if (APP_UWB_DW_SFD_LED_ENABLED) {
        if (BOARD_CONFIG_UWB_SFD_DW_LED_INDEX != 1) {
            ESP_LOGE(TAG, "SFDLED requires DW3000 GPIO1, got GPIO%d",
                     BOARD_CONFIG_UWB_SFD_DW_LED_INDEX);
            return ESP_ERR_NOT_SUPPORTED;
        }
        gpio_led_mask |= DW3000_GPIO_MODE_MSGP1_MODE_BIT_MASK;
        gpio_led_mode |= DW3000_GPIO_PIN1_SFDLED;
    }

    if (APP_UWB_DW_RX_LED_ENABLED) {
        if (BOARD_CONFIG_UWB_RX_DW_LED_INDEX != 2) {
            ESP_LOGE(TAG, "RXLED requires DW3000 GPIO2, got GPIO%d",
                     BOARD_CONFIG_UWB_RX_DW_LED_INDEX);
            return ESP_ERR_NOT_SUPPORTED;
        }
        gpio_led_mask |= DW3000_GPIO_MODE_MSGP2_MODE_BIT_MASK;
        gpio_led_mode |= DW3000_GPIO_PIN2_RXLED;
    }

    if (APP_UWB_DW_TX_LED_ENABLED) {
        if (BOARD_CONFIG_UWB_TX_DW_LED_INDEX != 3) {
            ESP_LOGE(TAG, "TXLED requires DW3000 GPIO3, got GPIO%d",
                     BOARD_CONFIG_UWB_TX_DW_LED_INDEX);
            return ESP_ERR_NOT_SUPPORTED;
        }
        gpio_led_mask |= DW3000_GPIO_MODE_MSGP3_MODE_BIT_MASK;
        gpio_led_mode |= DW3000_GPIO_PIN3_TXLED;
    }

    if (gpio_led_mask == 0) {
        ESP_LOGI(TAG, "DW3000 hardware LED block enabled with no LED functions");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(DW3000_REG_GPIO_CTRL, DW3000_GPIO_MODE_SUB,
                              gpio_led_mask, gpio_led_mode),
        TAG, "configure DW3000 LED GPIO mode failed");

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(DW3000_REG_PMSC, DW3000_PMSC_CLK_CTRL_SUB, 0,
                              DW3000_CLK_CTRL_GPIO_DCLK_EN_BIT_MASK |
                                  DW3000_CLK_CTRL_LP_CLK_EN_BIT_MASK),
        TAG, "enable DW3000 LED clocks failed");

    uint32_t led_ctrl =
        DW3000_LED_CTRL_BLINK_EN_BIT_MASK |
        ((uint32_t)APP_UWB_DW_LEDS_BLINK_TIME &
         DW3000_LED_CTRL_BLINK_TIME_MASK);
    if (APP_UWB_DW_LEDS_INIT_BLINK) {
        led_ctrl |= DW3000_LED_CTRL_FORCE_TRIGGER_BIT_MASK;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_PMSC, DW3000_PMSC_LED_CTRL_SUB,
                                 led_ctrl, sizeof(led_ctrl)),
        TAG, "enable DW3000 hardware LED blink failed");

    if (APP_UWB_DW_LEDS_INIT_BLINK) {
        led_ctrl &= ~DW3000_LED_CTRL_FORCE_TRIGGER_BIT_MASK;
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_write_u32_len(DW3000_REG_PMSC,
                                     DW3000_PMSC_LED_CTRL_SUB, led_ctrl,
                                     sizeof(led_ctrl)),
            TAG, "clear DW3000 LED init blink trigger failed");
    }

    ESP_LOGI(TAG,
             "DW3000 hardware LEDs enabled: RXOK=%s(GPIO%d) SFD=%s(GPIO%d) "
             "RX=%s(GPIO%d) TX=%s(GPIO%d) blink_time=0x%02x",
             APP_UWB_DW_RXOK_LED_ENABLED ? "on" : "off",
             BOARD_CONFIG_UWB_RXOK_DW_LED_INDEX,
             APP_UWB_DW_SFD_LED_ENABLED ? "on" : "off",
             BOARD_CONFIG_UWB_SFD_DW_LED_INDEX,
             APP_UWB_DW_RX_LED_ENABLED ? "on" : "off",
             BOARD_CONFIG_UWB_RX_DW_LED_INDEX,
             APP_UWB_DW_TX_LED_ENABLED ? "on" : "off",
             BOARD_CONFIG_UWB_TX_DW_LED_INDEX,
             (unsigned)((uint32_t)APP_UWB_DW_LEDS_BLINK_TIME &
                        DW3000_LED_CTRL_BLINK_TIME_MASK));
    return ESP_OK;
}

static esp_err_t uwb_dw3000_radio_init(void)
{
    uint32_t sys_cfg = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x10,
                                          &sys_cfg),
                        TAG, "SYS_CFG read failed");
    sys_cfg |= (1UL << 4);
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                                 0x10, sys_cfg, 4),
                        TAG, "SYS_CFG bit update failed");

    const esp_err_t pre_reset_idle =
        uwb_dw3000_wait_idle(UWB_DW3000_IDLE_TIMEOUT_MS);
    if (pre_reset_idle != ESP_OK) {
        ESP_LOGW(TAG, "DW3000 not idle before soft reset; continuing");
    }
    ESP_RETURN_ON_ERROR(uwb_dw3000_soft_reset(), TAG, "soft reset failed");
    uwb_dw3000_delay_ms(200);
    const esp_err_t post_reset_idle =
        uwb_dw3000_wait_idle(UWB_DW3000_IDLE_TIMEOUT_MS);
    if (post_reset_idle != ESP_OK) {
        ESP_LOGW(TAG, "DW3000 not idle after soft reset; continuing");
    }

    uint32_t ldo_low = 0;
    uint32_t ldo_high = 0;
    uint32_t bias_tune = 0;
    if (uwb_dw3000_read_otp(0x04, &ldo_low) == ESP_OK &&
        uwb_dw3000_read_otp(0x05, &ldo_high) == ESP_OK &&
        uwb_dw3000_read_otp(0x0A, &bias_tune) == ESP_OK) {
        bias_tune = (bias_tune >> 16) & 0x1FU;
        if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
            ESP_RETURN_ON_ERROR(
                uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x1F, bias_tune),
                TAG, "bias tune write failed");
            ESP_RETURN_ON_ERROR(
                uwb_dw3000_write_u32_auto(DW3000_REG_OTP_IF, 0x08, 0x0100),
                TAG, "OTP LDO write failed");
        }
    }

    uint32_t xtrim = 0;
    if (uwb_dw3000_read_otp(0x1E, &xtrim) != ESP_OK || xtrim == 0) {
        xtrim = 0x2E;
    }
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x14,
                                                  xtrim),
                        TAG, "XTAL trim write failed");

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_sys_config(), TAG,
                        "radio profile write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x3C, 0xFFFFFFFF,
                                 4),
        TAG, "SYS_ENABLE low write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x40, 0xFFFF, 2),
        TAG, "SYS_ENABLE high write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_AON, DW3000_SUB_NONE, 0x000900, 3),
        TAG, "AON_DIG_CFG write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x1C, 0x10000240), TAG,
        "DGC_CFG0 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x20, 0x1B6DA489), TAG,
        "DGC_CFG1 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x38, 0x0001C0FD), TAG,
        "DGC_LUT_0 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x3C, 0x0001C43E), TAG,
        "DGC_LUT_1 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x40, 0x0001C6BE), TAG,
        "DGC_LUT_2 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x44, 0x0001C77E), TAG,
        "DGC_LUT_3 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x48, 0x0001CF36), TAG,
        "DGC_LUT_4 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x4C, 0x0001CFB5), TAG,
        "DGC_LUT_5 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x50, 0x0001CFF5), TAG,
        "DGC_LUT_6 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x18, 0xE5E5), TAG,
        "THR_64 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x00, 0x81101C), TAG,
        "DRX PAC write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x34, 0x04), TAG,
        "temperature sensor enable failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x48, 0x14), TAG,
        "LDO_RLOAD write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1A, 0x0E), TAG,
        "RF_TX_CTRL_1 final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C, 0x1C071134),
        TAG, "RF_TX_CTRL_2 final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x00, 0x1F3C), TAG,
        "PLL_CFG final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x08, 0x81), TAG,
        "PLL_CAL final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x04, 0xB40200), TAG,
        "PMSC final 0x04 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_PMSC, 0x08, 0x80030738), TAG,
        "PMSC final 0x08 write failed");

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_GPIO_CTRL, DW3000_GPIO_DIR_SUB,
                                  0xF0),
        TAG, "DW3000 GPIO direction setup failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_hardware_leds(), TAG,
                        "DW3000 hardware LED setup failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C, 0x34), TAG,
        "TX PG delay write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_GEN_CFG_AES_HIGH, 0x0C,
                                  0xFDFDFDFD),
        TAG, "TX power write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "initial status clear failed");
    return ESP_OK;
}

static esp_err_t uwb_dw3000_set_frame_length(size_t payload_len)
{
    uint32_t tx_fctrl = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x24,
                                          &tx_fctrl),
                        TAG, "TX_FCTRL read before length failed");
    const uint32_t frame_len = (uint32_t)payload_len + 2U;
    tx_fctrl &= ~(DW3000_TX_FCTRL_TXB_OFFSET_MASK | DW3000_TX_FCTRL_TR_MASK |
                  DW3000_TX_FCTRL_TXFLEN_MASK);
    tx_fctrl |= frame_len & DW3000_TX_FCTRL_TXFLEN_MASK;
    return uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x24,
                                    tx_fctrl, 4);
}

static void uwb_dw3000_build_payload(uint32_t sequence,
                                     uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    memset(payload, 0, UWB_DW3000_PAYLOAD_LEN);
    payload[0] = 'U';
    payload[1] = 'W';
    payload[2] = 'B';
    payload[3] = 'T';
    payload[4] = (uint8_t)((sequence >> 24) & 0xFFU);
    payload[5] = (uint8_t)((sequence >> 16) & 0xFFU);
    payload[6] = (uint8_t)((sequence >> 8) & 0xFFU);
    payload[7] = (uint8_t)(sequence & 0xFFU);
    payload[8] = s_source_id;
    payload[9] = (uint8_t)(esp_random() & 0xFFU);

    for (size_t i = 10; i < UWB_DW3000_PAYLOAD_LEN; ++i) {
        payload[i] = (uint8_t)(sequence + (uint32_t)i);
    }
}

static bool uwb_dw3000_payload_has_magic(
    const uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    return payload[0] == 'U' && payload[1] == 'W' && payload[2] == 'B' &&
           payload[3] == 'T';
}

static uint32_t uwb_dw3000_payload_sequence(
    const uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    return ((uint32_t)payload[4] << 24) | ((uint32_t)payload[5] << 16) |
           ((uint32_t)payload[6] << 8) | (uint32_t)payload[7];
}

static esp_err_t uwb_dw3000_arm_rx(void)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "clear before RX failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_RX), TAG,
                        "RX command failed");
    s_rx_armed = true;
    return ESP_OK;
}

static esp_err_t uwb_dw3000_poll_rx(void)
{
    uint32_t status = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44,
                                          &status),
                        TAG, "SYS_STATUS read failed");

    if ((status & DW3000_RX_GOOD_MASK) != 0) {
        uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
        uint16_t payload_len = 0;
        const esp_err_t read_err =
            uwb_dw3000_read_rx_payload(payload, &payload_len);
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear after RX good failed");
        s_rx_armed = false;

        if (read_err != ESP_OK) {
            s_rx_error_count++;
            return read_err;
        }

        if (payload_len >= UWB_DW3000_PAYLOAD_LEN &&
            uwb_dw3000_payload_has_magic(payload)) {
            const uint32_t sequence = uwb_dw3000_payload_sequence(payload);
            const uint8_t source_id = payload[8];
            s_rx_count++;
            s_last_rx_sequence = sequence;
            s_last_rx_source_id = source_id;
            ESP_LOGI(TAG, "UWB RX beacon src=%u seq=%lu total_rx=%lu",
                     (unsigned)source_id, (unsigned long)sequence,
                     (unsigned long)s_rx_count);
        } else {
            s_rx_ignored_count++;
            if ((s_rx_ignored_count % 1000UL) == 1UL) {
                ESP_LOGD(TAG,
                         "Ignoring non-UWBT RX frame len=%u bytes=%02x %02x %02x %02x ignored=%lu",
                         (unsigned)payload_len,
                         payload[0], payload[1], payload[2], payload[3],
                         (unsigned long)s_rx_ignored_count);
            }
        }
        return ESP_OK;
    }

    if ((status & DW3000_RX_ERROR_MASK) != 0) {
        s_rx_error_count++;
        ESP_LOGW(TAG, "UWB RX error SYS_STATUS=0x%08lx rx_errors=%lu",
                 (unsigned long)status, (unsigned long)s_rx_error_count);
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear after RX error failed");
        s_rx_armed = false;
        return ESP_OK;
    }

    if ((status & DW3000_RX_TIMEOUT_MASK) != 0) {
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear after RX timeout failed");
        s_rx_armed = false;
    }

    return ESP_OK;
}

static uint32_t uwb_dw3000_remaining_ms(TickType_t start_tick,
                                        uint32_t timeout_ms)
{
    const TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
    const uint32_t elapsed_ms = (uint32_t)elapsed_ticks * portTICK_PERIOD_MS;
    if (elapsed_ms >= timeout_ms) {
        return 0;
    }
    return timeout_ms - elapsed_ms;
}

static esp_err_t uwb_dw3000_receive_frame(struct uwb_dw3000_rx_frame *frame,
                                          uint32_t timeout_ms)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(frame, 0, sizeof(*frame));

    if (!s_rx_armed) {
        ESP_RETURN_ON_ERROR(uwb_dw3000_arm_rx(), TAG,
                            "distance RX arm failed");
    }

    const TickType_t start = xTaskGetTickCount();
    while (true) {
        uint32_t status = 0;
        ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44,
                                              &status),
                            TAG, "distance SYS_STATUS read failed");

        if ((status & DW3000_RX_GOOD_MASK) != 0) {
            const esp_err_t read_err =
                uwb_dw3000_read_rx_payload(frame->payload, &frame->payload_len);
            const esp_err_t ts_err =
                uwb_dw3000_read_rx_timestamp(&frame->rx_timestamp);
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after distance RX good failed");
            s_rx_armed = false;

            if (read_err != ESP_OK) {
                s_rx_error_count++;
                return read_err;
            }
            if (ts_err != ESP_OK) {
                s_rx_error_count++;
                return ts_err;
            }

            s_rx_count++;
            return ESP_OK;
        }

        if ((status & DW3000_RX_ERROR_MASK) != 0) {
            s_rx_error_count++;
            ESP_LOGW(TAG, "UWB distance RX error SYS_STATUS=0x%08lx",
                     (unsigned long)status);
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after distance RX error failed");
            s_rx_armed = false;
            return ESP_FAIL;
        }

        if ((status & DW3000_RX_TIMEOUT_MASK) != 0) {
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after distance RX timeout failed");
            s_rx_armed = false;
            return ESP_ERR_TIMEOUT;
        }

        if (uwb_dw3000_remaining_ms(start, timeout_ms) == 0) {
            (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
            (void)uwb_dw3000_clear_status();
            s_rx_armed = false;
            return ESP_ERR_TIMEOUT;
        }

        uwb_dw3000_delay_ms(UWB_DW3000_POLL_INTERVAL_MS);
    }
}

static void uwb_distance_put_u16(uint8_t *payload, size_t offset,
                                 uint16_t value)
{
    payload[offset] = (uint8_t)(value & 0xFFU);
    payload[offset + 1U] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t uwb_distance_get_u16(const uint8_t *payload, size_t offset)
{
    return (uint16_t)(((uint16_t)payload[offset]) |
                      ((uint16_t)payload[offset + 1U] << 8));
}

static void uwb_distance_put_ts40(uint8_t *payload, size_t offset,
                                  uint64_t timestamp)
{
    timestamp &= UWB_DW3000_TIMESTAMP_MASK;
    for (size_t i = 0; i < 5U; ++i) {
        payload[offset + i] = (uint8_t)((timestamp >> (8U * i)) & 0xFFU);
    }
}

static uint64_t uwb_distance_get_ts40(const uint8_t *payload, size_t offset)
{
    uint64_t timestamp = 0;
    for (size_t i = 0; i < 5U; ++i) {
        timestamp |= ((uint64_t)payload[offset + i]) << (8U * i);
    }
    return timestamp & UWB_DW3000_TIMESTAMP_MASK;
}

static void uwb_distance_build_frame(enum uwb_distance_frame_type type,
                                     uint8_t destination_id,
                                     uint16_t sequence,
                                     uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    memset(payload, 0, UWB_DW3000_PAYLOAD_LEN);
    payload[0] = UWB_DISTANCE_FRAME_MAGIC_0;
    payload[1] = UWB_DISTANCE_FRAME_MAGIC_1;
    payload[2] = UWB_DISTANCE_FRAME_MAGIC_2;
    payload[3] = UWB_DISTANCE_FRAME_MAGIC_3;
    payload[4] = UWB_DISTANCE_FRAME_VERSION;
    payload[5] = (uint8_t)type;
    payload[6] = s_source_id;
    payload[7] = destination_id;
    uwb_distance_put_u16(payload, 8, sequence);
}

static bool uwb_distance_parse_frame(const struct uwb_dw3000_rx_frame *rx_frame,
                                     struct uwb_distance_frame *frame)
{
    if (rx_frame == NULL || frame == NULL ||
        rx_frame->payload_len < UWB_DISTANCE_FRAME_HEADER_LEN) {
        return false;
    }

    const uint8_t *payload = rx_frame->payload;
    if (payload[0] != UWB_DISTANCE_FRAME_MAGIC_0 ||
        payload[1] != UWB_DISTANCE_FRAME_MAGIC_1 ||
        payload[2] != UWB_DISTANCE_FRAME_MAGIC_2 ||
        payload[3] != UWB_DISTANCE_FRAME_MAGIC_3 ||
        payload[4] != UWB_DISTANCE_FRAME_VERSION) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->type = payload[5];
    frame->source_id = payload[6];
    frame->destination_id = payload[7];
    frame->sequence = uwb_distance_get_u16(payload, 8);
    frame->rx_timestamp = rx_frame->rx_timestamp;
    frame->payload_len = rx_frame->payload_len;
    memcpy(frame->payload, payload, rx_frame->payload_len);
    return true;
}

static const char *uwb_distance_type_name(uint8_t type)
{
    switch (type) {
    case UWB_DISTANCE_FRAME_POLL:
        return "POLL";
    case UWB_DISTANCE_FRAME_RESP:
        return "RESP";
    case UWB_DISTANCE_FRAME_FINAL:
        return "FINAL";
    case UWB_DISTANCE_FRAME_REPORT:
        return "REPORT";
    default:
        return "UNKNOWN";
    }
}

static esp_err_t uwb_distance_receive_matching(
    uint8_t expected_type, uint8_t expected_source_id, bool match_sequence,
    uint16_t expected_sequence, struct uwb_distance_frame *frame,
    uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();

    while (true) {
        const uint32_t remaining_ms =
            uwb_dw3000_remaining_ms(start, timeout_ms);
        if (remaining_ms == 0) {
            return ESP_ERR_TIMEOUT;
        }

        struct uwb_dw3000_rx_frame rx_frame = {0};
        const esp_err_t err =
            uwb_dw3000_receive_frame(&rx_frame, remaining_ms);
        if (err != ESP_OK) {
            return err;
        }

        struct uwb_distance_frame parsed = {0};
        if (!uwb_distance_parse_frame(&rx_frame, &parsed)) {
            s_rx_ignored_count++;
            ESP_LOGD(TAG, "Ignoring non-ranging frame len=%u ignored=%lu",
                     (unsigned)rx_frame.payload_len,
                     (unsigned long)s_rx_ignored_count);
            continue;
        }

        const bool type_ok = parsed.type == expected_type;
        const bool source_ok =
            expected_source_id == 0 || parsed.source_id == expected_source_id;
        const bool destination_ok = parsed.destination_id == s_source_id;
        const bool sequence_ok =
            !match_sequence || parsed.sequence == expected_sequence;

        if (type_ok && source_ok && destination_ok && sequence_ok) {
            s_last_rx_source_id = parsed.source_id;
            s_last_rx_sequence = parsed.sequence;
            *frame = parsed;
            return ESP_OK;
        }

        s_rx_ignored_count++;
        ESP_LOGD(TAG,
                 "Ignoring ranging frame type=%s src=%u dst=%u seq=%u "
                 "while waiting for type=%s src=%u seq=%u ignored=%lu",
                 uwb_distance_type_name(parsed.type),
                 (unsigned)parsed.source_id, (unsigned)parsed.destination_id,
                 (unsigned)parsed.sequence,
                 uwb_distance_type_name(expected_type),
                 (unsigned)expected_source_id, (unsigned)expected_sequence,
                 (unsigned long)s_rx_ignored_count);
    }
}

static uint64_t uwb_distance_delta_ts(uint64_t later, uint64_t earlier)
{
    return (later - earlier) & UWB_DW3000_TIMESTAMP_MASK;
}

static double uwb_distance_tof_dtu(uint64_t poll_tx_ts, uint64_t poll_rx_ts,
                                   uint64_t resp_tx_ts, uint64_t resp_rx_ts,
                                   uint64_t final_tx_ts,
                                   uint64_t final_rx_ts)
{
    const double round_a =
        (double)uwb_distance_delta_ts(resp_rx_ts, poll_tx_ts);
    const double round_b =
        (double)uwb_distance_delta_ts(final_rx_ts, resp_tx_ts);
    const double reply_a =
        (double)uwb_distance_delta_ts(final_tx_ts, resp_rx_ts);
    const double reply_b =
        (double)uwb_distance_delta_ts(resp_tx_ts, poll_rx_ts);
    const double denominator = round_a + round_b + reply_a + reply_b;

    if (denominator == 0.0) {
        return 0.0;
    }

    return ((round_a * round_b) - (reply_a * reply_b)) / denominator;
}

static double uwb_distance_tof_to_meters(double tof_dtu)
{
    return tof_dtu * UWB_DW3000_TIME_UNIT_SECONDS *
           UWB_DW3000_SPEED_OF_LIGHT_MPS;
}

static uint8_t uwb_distance_peer_id(bool initiator)
{
    if (APP_UWB_DISTANCE_TEST_PEER_ID > 0 &&
        APP_UWB_DISTANCE_TEST_PEER_ID <= 255) {
        return (uint8_t)APP_UWB_DISTANCE_TEST_PEER_ID;
    }

    return initiator ? (uint8_t)APP_UWB_DISTANCE_TEST_RESPONDER_ID
                     : (uint8_t)APP_UWB_DISTANCE_TEST_INITIATOR_ID;
}

static esp_err_t uwb_distance_initiate_once(uint8_t peer_id, uint16_t sequence,
                                            bool log_success)
{
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uint64_t poll_tx_ts = 0;
    uint64_t resp_rx_ts = 0;
    uint64_t final_tx_ts = 0;

    uwb_distance_build_frame(UWB_DISTANCE_FRAME_POLL, peer_id, sequence,
                             payload);
    esp_err_t err =
        uwb_dw3000_send_payload(payload, sizeof(payload), &poll_tx_ts);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR POLL TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame response = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_RESP, peer_id, true,
                                        sequence, &response,
                                        APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR RESP wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }
    resp_rx_ts = response.rx_timestamp;

    uwb_distance_build_frame(UWB_DISTANCE_FRAME_FINAL, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_USE_DELAYED_TX
    const uint64_t final_tx_due = uwb_dw3000_add_timestamp_delta(
        resp_rx_ts, uwb_dw3000_ms_to_dtu(APP_UWB_DISTANCE_TEST_FINAL_DELAY_MS));
    uint64_t final_tx_actual_ts = 0;
    err = uwb_dw3000_send_payload_delayed(payload, sizeof(payload),
                                          final_tx_due, &final_tx_ts,
                                          &final_tx_actual_ts);
    ESP_LOGD(TAG,
             "DS-TWR FINAL delayed seq=%u peer=%u due=0x%010llx programmed=0x%010llx actual=0x%010llx",
             (unsigned)sequence, (unsigned)peer_id,
             (unsigned long long)final_tx_due,
             (unsigned long long)final_tx_ts,
             (unsigned long long)final_tx_actual_ts);
#else
    uwb_dw3000_delay_ms(APP_UWB_DISTANCE_TEST_FINAL_DELAY_MS);
    err = uwb_dw3000_send_payload(payload, sizeof(payload), &final_tx_ts);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR FINAL TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    uwb_dw3000_delay_ms(APP_UWB_DISTANCE_TEST_REPORT_DELAY_MS);
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_REPORT, peer_id, sequence,
                             payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          resp_rx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          final_tx_ts);
    err = uwb_dw3000_send_payload(payload, sizeof(payload), NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR REPORT TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    if (log_success) {
        ESP_LOGI(TAG,
                 "DS-TWR report sent seq=%u peer=%u poll_tx=0x%010llx resp_rx=0x%010llx final_tx=0x%010llx",
                 (unsigned)sequence, (unsigned)peer_id,
                 (unsigned long long)poll_tx_ts,
                 (unsigned long long)resp_rx_ts,
                 (unsigned long long)final_tx_ts);
    }

    return ESP_OK;
}

static void uwb_distance_fill_measurement(
    const struct uwb_distance_frame *poll, const struct uwb_distance_frame *final,
    const struct uwb_distance_frame *report, uint64_t resp_tx_ts,
    struct uwb_distance_measurement *measurement)
{
    const uint64_t poll_tx_ts = uwb_distance_get_ts40(
        report->payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET);
    const uint64_t resp_rx_ts = uwb_distance_get_ts40(
        report->payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET);
    const uint64_t final_tx_ts = uwb_distance_get_ts40(
        report->payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET);

    const double tof_dtu =
        uwb_distance_tof_dtu(poll_tx_ts, poll->rx_timestamp, resp_tx_ts,
                             resp_rx_ts, final_tx_ts, final->rx_timestamp);

    memset(measurement, 0, sizeof(*measurement));
    measurement->initiator_id = poll->source_id;
    measurement->responder_id = s_source_id;
    measurement->sequence = poll->sequence;
    measurement->tof_dtu = tof_dtu;
    measurement->distance_m = uwb_distance_tof_to_meters(tof_dtu);
    measurement->poll_tx_ts = poll_tx_ts;
    measurement->poll_rx_ts = poll->rx_timestamp;
    measurement->resp_tx_ts = resp_tx_ts;
    measurement->resp_rx_ts = resp_rx_ts;
    measurement->final_tx_ts = final_tx_ts;
    measurement->final_rx_ts = final->rx_timestamp;
}

static esp_err_t
uwb_distance_respond_to_poll(const struct uwb_distance_frame *poll,
                             struct uwb_distance_measurement *measurement)
{
    if (poll == NULL || measurement == NULL ||
        poll->type != UWB_DISTANCE_FRAME_POLL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t peer_id = poll->source_id;
    const uint16_t sequence = poll->sequence;
    uint64_t resp_tx_ts = 0;

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_RESP, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_USE_DELAYED_TX
    const uint64_t resp_tx_due = uwb_dw3000_add_timestamp_delta(
        poll->rx_timestamp,
        uwb_dw3000_ms_to_dtu(APP_UWB_DISTANCE_TEST_RESP_DELAY_MS));
    uint64_t resp_tx_actual_ts = 0;
    esp_err_t err = uwb_dw3000_send_payload_delayed(
        payload, sizeof(payload), resp_tx_due, &resp_tx_ts,
        &resp_tx_actual_ts);
    ESP_LOGD(TAG,
             "DS-TWR RESP delayed seq=%u peer=%u due=0x%010llx programmed=0x%010llx actual=0x%010llx",
             (unsigned)sequence, (unsigned)peer_id,
             (unsigned long long)resp_tx_due,
             (unsigned long long)resp_tx_ts,
             (unsigned long long)resp_tx_actual_ts);
#else
    uwb_dw3000_delay_ms(APP_UWB_DISTANCE_TEST_RESP_DELAY_MS);
    esp_err_t err =
        uwb_dw3000_send_payload(payload, sizeof(payload), &resp_tx_ts);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR RESP TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame final = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_FINAL, peer_id, true,
                                        sequence, &final,
                                        APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR FINAL wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame report = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_REPORT, peer_id, true,
                                        sequence, &report,
                                        APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR REPORT wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    uwb_distance_fill_measurement(poll, &final, &report, resp_tx_ts,
                                  measurement);
    return ESP_OK;
}

static void
uwb_distance_log_measurement(const struct uwb_distance_measurement *measurement)
{
    ESP_LOGI(TAG,
             "DS-TWR distance seq=%u peer=%u distance=%.3f m %.1f cm tof=%.2f dtu",
             (unsigned)measurement->sequence,
             (unsigned)measurement->initiator_id, measurement->distance_m,
             measurement->distance_m * 100.0, measurement->tof_dtu);
    ESP_LOGD(TAG,
             "DS-TWR timestamps seq=%u poll_tx=0x%010llx poll_rx=0x%010llx resp_tx=0x%010llx resp_rx=0x%010llx final_tx=0x%010llx final_rx=0x%010llx",
             (unsigned)measurement->sequence,
             (unsigned long long)measurement->poll_tx_ts,
             (unsigned long long)measurement->poll_rx_ts,
             (unsigned long long)measurement->resp_tx_ts,
             (unsigned long long)measurement->resp_rx_ts,
             (unsigned long long)measurement->final_tx_ts,
             (unsigned long long)measurement->final_rx_ts);
}

static bool uwb_distance_is_initiator(void)
{
    if (s_source_id == (uint8_t)APP_UWB_DISTANCE_TEST_INITIATOR_ID) {
        return true;
    }
    if (s_source_id == (uint8_t)APP_UWB_DISTANCE_TEST_RESPONDER_ID) {
        return false;
    }

#if APP_UWB_DISTANCE_TEST_PEER_ID > 0 && APP_UWB_DISTANCE_TEST_PEER_ID <= 255
    return s_source_id < (uint8_t)APP_UWB_DISTANCE_TEST_PEER_ID;
#endif

    return (s_source_id & 1U) != 0;
}

static void uwb_distance_initiator_loop(uint8_t peer_id)
{
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "DS-TWR distance test active as initiator: source_id=%u peer_id=%u interval=%u ms timeout=%u ms",
             (unsigned)s_source_id, (unsigned)peer_id,
             (unsigned)APP_UWB_DISTANCE_TEST_INTERVAL_MS,
             (unsigned)APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS);

    while (true) {
        (void)uwb_distance_initiate_once(peer_id, sequence, true);
        sequence++;
        uwb_dw3000_delay_ms(APP_UWB_DISTANCE_TEST_INTERVAL_MS);
    }
}

static void uwb_distance_responder_loop(uint8_t peer_id)
{
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "DS-TWR distance test active as responder: source_id=%u peer_id=%u timeout=%u ms",
             (unsigned)s_source_id, (unsigned)peer_id,
             (unsigned)APP_UWB_DISTANCE_TEST_RX_TIMEOUT_MS);

    while (true) {
        struct uwb_distance_frame poll = {0};
        esp_err_t err = uwb_distance_receive_matching(
            UWB_DISTANCE_FRAME_POLL, peer_id, false, 0, &poll, 1000);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "DS-TWR POLL wait failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
            continue;
        }

        struct uwb_distance_measurement measurement = {0};
        err = uwb_distance_respond_to_poll(&poll, &measurement);
        if (err == ESP_OK) {
            uwb_distance_log_measurement(&measurement);
        }
    }
}

static void uwb_dw3000_distance_test_loop(void)
{
    const bool initiator = uwb_distance_is_initiator();
    const uint8_t peer_id = uwb_distance_peer_id(initiator);

    if (peer_id == 0 || peer_id == s_source_id) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG,
                 "Invalid DS-TWR IDs: source_id=%u peer_id=%u initiator_id=%u responder_id=%u",
                 (unsigned)s_source_id, (unsigned)peer_id,
                 (unsigned)APP_UWB_DISTANCE_TEST_INITIATOR_ID,
                 (unsigned)APP_UWB_DISTANCE_TEST_RESPONDER_ID);
        vTaskDelete(NULL);
        return;
    }

    if (initiator) {
        uwb_distance_initiator_loop(peer_id);
    } else {
        uwb_distance_responder_loop(peer_id);
    }
}

static const char *uwb_calibration_method_name(void)
{
    switch (APP_UWB_CALIBRATION_METHOD) {
    case APP_UWB_CALIBRATION_METHOD_TWO_MODULE:
        return "two_module";
    case APP_UWB_CALIBRATION_METHOD_THREE_MODULE:
        return "three_module_edm";
    default:
        return "unknown";
    }
}

static double uwb_calibration_known_distance_for_pair_m(uint8_t first_id,
                                                       uint8_t second_id)
{
    const uint8_t id0 = (uint8_t)APP_UWB_CALIBRATION_THREE_ID_0;
    const uint8_t id1 = (uint8_t)APP_UWB_CALIBRATION_THREE_ID_1;
    const uint8_t id2 = (uint8_t)APP_UWB_CALIBRATION_THREE_ID_2;
    const uint32_t fallback_mm = APP_UWB_CALIBRATION_KNOWN_DISTANCE_MM;
    uint32_t distance_mm = fallback_mm;

    if ((first_id == id0 && second_id == id1) ||
        (first_id == id1 && second_id == id0)) {
        distance_mm = APP_UWB_CALIBRATION_THREE_DISTANCE_0_1_MM;
    } else if ((first_id == id0 && second_id == id2) ||
               (first_id == id2 && second_id == id0)) {
        distance_mm = APP_UWB_CALIBRATION_THREE_DISTANCE_0_2_MM;
    } else if ((first_id == id1 && second_id == id2) ||
               (first_id == id2 && second_id == id1)) {
        distance_mm = APP_UWB_CALIBRATION_THREE_DISTANCE_1_2_MM;
    }

    return (double)distance_mm / 1000.0;
}

static double uwb_calibration_meters_per_dtu(void)
{
    return UWB_DW3000_TIME_UNIT_SECONDS * UWB_DW3000_SPEED_OF_LIGHT_MPS;
}

static int32_t uwb_calibration_round_to_i32(double value)
{
    return value >= 0.0 ? (int32_t)(value + 0.5)
                        : (int32_t)(value - 0.5);
}

static uint32_t uwb_calibration_random_interval_ms(void)
{
    const uint32_t min_ms = APP_UWB_CALIBRATION_MIN_INTERVAL_MS;
    const uint32_t max_ms = APP_UWB_CALIBRATION_MAX_INTERVAL_MS;
    if (max_ms <= min_ms) {
        return min_ms;
    }

    return min_ms + (esp_random() % (max_ms - min_ms + 1U));
}

static void uwb_calibration_stats_update(struct uwb_calibration_stats *stats,
                                         double distance_m)
{
    if (stats->samples == 0) {
        stats->min_m = distance_m;
        stats->max_m = distance_m;
        stats->mean_m = distance_m;
        stats->m2_m = 0.0;
        stats->last_m = distance_m;
        stats->samples = 1;
        return;
    }

    stats->samples++;
    stats->last_m = distance_m;
    if (distance_m < stats->min_m) {
        stats->min_m = distance_m;
    }
    if (distance_m > stats->max_m) {
        stats->max_m = distance_m;
    }

    const double delta = distance_m - stats->mean_m;
    stats->mean_m += delta / (double)stats->samples;
    const double delta2 = distance_m - stats->mean_m;
    stats->m2_m += delta * delta2;
}

static double
uwb_calibration_stats_stddev_m(const struct uwb_calibration_stats *stats)
{
    if (stats->samples < 2) {
        return 0.0;
    }

    return sqrt(stats->m2_m / (double)(stats->samples - 1U));
}

static bool uwb_calibration_should_log_summary(uint32_t samples)
{
    if (samples == 0) {
        return false;
    }
    if (samples == 1 || samples == APP_UWB_CALIBRATION_SAMPLE_COUNT) {
        return true;
    }
    if (APP_UWB_CALIBRATION_SUMMARY_EVERY == 0) {
        return false;
    }
    return (samples % APP_UWB_CALIBRATION_SUMMARY_EVERY) == 0;
}

static void uwb_calibration_log_stats(
    const char *method, const struct uwb_distance_measurement *measurement,
    const struct uwb_calibration_stats *stats)
{
    const double known_m = uwb_calibration_known_distance_for_pair_m(
        measurement->initiator_id, measurement->responder_id);
    const double error_m = stats->mean_m - known_m;
    const double error_dtu = error_m / uwb_calibration_meters_per_dtu();
    const int32_t suggested_delta =
        uwb_calibration_round_to_i32(error_dtu);
    int32_t suggested_delay =
        (int32_t)APP_UWB_ANTENNA_DELAY_DEFAULT + suggested_delta;
    if (suggested_delay < 0) {
        suggested_delay = 0;
    } else if (suggested_delay > 0xFFFF) {
        suggested_delay = 0xFFFF;
    }

    ESP_LOGI(TAG,
             "UWB CAL %s pair=%u->%u samples=%lu last=%.3f m mean=%.3f m std=%.3f m min=%.3f m max=%.3f m known=%.3f m error=%+.3f m error_dtu=%+.1f suggested_delta=%ld suggested_delay=0x%04lx",
             method, (unsigned)measurement->initiator_id,
             (unsigned)measurement->responder_id,
             (unsigned long)stats->samples, stats->last_m, stats->mean_m,
             uwb_calibration_stats_stddev_m(stats), stats->min_m,
             stats->max_m, known_m, error_m, error_dtu,
             (long)suggested_delta, (unsigned long)suggested_delay);
}

static void uwb_calibration_record_measurement(
    const char *method, const struct uwb_distance_measurement *measurement,
    struct uwb_calibration_stats *stats)
{
    uwb_calibration_stats_update(stats, measurement->distance_m);
    ESP_LOGI(TAG,
             "UWB CAL sample %s pair=%u->%u sample=%lu seq=%u distance=%.3f m %.1f cm tof=%.2f dtu",
             method, (unsigned)measurement->initiator_id,
             (unsigned)measurement->responder_id,
             (unsigned long)stats->samples, (unsigned)measurement->sequence,
             measurement->distance_m, measurement->distance_m * 100.0,
             measurement->tof_dtu);

    if (uwb_calibration_should_log_summary(stats->samples)) {
        uwb_calibration_log_stats(method, measurement, stats);
    }
}

static void uwb_calibration_two_module_loop(void)
{
    const uint8_t reference_id = (uint8_t)APP_UWB_CALIBRATION_REFERENCE_ID;
    const uint8_t dut_id = (uint8_t)APP_UWB_CALIBRATION_DUT_ID;
    struct uwb_calibration_stats stats = {0};
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB CAL two-module active: source_id=%u reference_id=%u dut_id=%u known=%u mm delay=0x%04x",
             (unsigned)s_source_id, (unsigned)reference_id,
             (unsigned)dut_id, (unsigned)APP_UWB_CALIBRATION_KNOWN_DISTANCE_MM,
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT);

    if (reference_id == 0 || dut_id == 0 || reference_id == dut_id) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Invalid two-module calibration IDs: reference=%u dut=%u",
                 (unsigned)reference_id, (unsigned)dut_id);
        vTaskDelete(NULL);
        return;
    }

    if (s_source_id == reference_id) {
        ESP_LOGI(TAG,
                 "UWB CAL two-module role=reference initiator peer_dut=%u",
                 (unsigned)dut_id);
        while (true) {
            (void)uwb_distance_initiate_once(dut_id, sequence++, false);
            uwb_dw3000_delay_ms(uwb_calibration_random_interval_ms());
        }
    }

    if (s_source_id != dut_id) {
        ESP_LOGW(TAG,
                 "UWB CAL two-module idle: source_id=%u is neither reference nor DUT",
                 (unsigned)s_source_id);
        while (true) {
            uwb_dw3000_delay_ms(1000);
        }
    }

    ESP_LOGI(TAG,
             "UWB CAL two-module role=DUT responder reference_peer=%u",
             (unsigned)reference_id);
    while (true) {
        struct uwb_distance_frame poll = {0};
        esp_err_t err = uwb_distance_receive_matching(
            UWB_DISTANCE_FRAME_POLL, reference_id, false, 0, &poll, 1000);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "UWB CAL two-module POLL wait failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
            continue;
        }

        struct uwb_distance_measurement measurement = {0};
        err = uwb_distance_respond_to_poll(&poll, &measurement);
        if (err == ESP_OK) {
            uwb_calibration_record_measurement("two_module", &measurement,
                                               &stats);
        }
    }
}

static bool uwb_calibration_id_in_three_set(uint8_t id)
{
    return id == (uint8_t)APP_UWB_CALIBRATION_THREE_ID_0 ||
           id == (uint8_t)APP_UWB_CALIBRATION_THREE_ID_1 ||
           id == (uint8_t)APP_UWB_CALIBRATION_THREE_ID_2;
}

static int uwb_calibration_pair_index(uint8_t initiator_id,
                                      uint8_t responder_id)
{
    const uint8_t ids[3] = {
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_0,
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_1,
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_2,
    };
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == j) {
                continue;
            }
            if (ids[i] == initiator_id && ids[j] == responder_id) {
                return index;
            }
            index++;
        }
    }

    return -1;
}

static void uwb_calibration_three_module_loop(void)
{
    const uint8_t ids[3] = {
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_0,
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_1,
        (uint8_t)APP_UWB_CALIBRATION_THREE_ID_2,
    };
    uint8_t peers[2] = {0};
    size_t peer_count = 0;
    struct uwb_calibration_stats pair_stats[6] = {0};
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);
    size_t next_peer_index = 0;
    TickType_t next_initiate_tick =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(200U + (uint32_t)s_source_id * 73U);

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB CAL three-module EDM active: source_id=%u ids=[%u,%u,%u] known_edge=%u mm delay=0x%04x",
             (unsigned)s_source_id, (unsigned)ids[0], (unsigned)ids[1],
             (unsigned)ids[2],
             (unsigned)APP_UWB_CALIBRATION_KNOWN_DISTANCE_MM,
             (unsigned)APP_UWB_ANTENNA_DELAY_DEFAULT);

    if (ids[0] == 0 || ids[1] == 0 || ids[2] == 0 || ids[0] == ids[1] ||
        ids[0] == ids[2] || ids[1] == ids[2]) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Invalid three-module calibration IDs");
        vTaskDelete(NULL);
        return;
    }

    if (!uwb_calibration_id_in_three_set(s_source_id)) {
        ESP_LOGW(TAG,
                 "UWB CAL three-module idle: source_id=%u is not in the configured set",
                 (unsigned)s_source_id);
        while (true) {
            uwb_dw3000_delay_ms(1000);
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        if (ids[i] != s_source_id && peer_count < 2) {
            peers[peer_count++] = ids[i];
        }
    }

    while (true) {
        struct uwb_distance_frame poll = {0};
        esp_err_t err = uwb_distance_receive_matching(
            UWB_DISTANCE_FRAME_POLL, 0, false, 0, &poll,
            APP_UWB_CALIBRATION_RX_SLICE_MS);
        if (err == ESP_OK) {
            if (!uwb_calibration_id_in_three_set(poll.source_id)) {
                ESP_LOGD(TAG,
                         "Ignoring calibration poll from non-set source=%u",
                         (unsigned)poll.source_id);
                continue;
            }

            struct uwb_distance_measurement measurement = {0};
            err = uwb_distance_respond_to_poll(&poll, &measurement);
            if (err == ESP_OK) {
                const int pair_index = uwb_calibration_pair_index(
                    measurement.initiator_id, measurement.responder_id);
                if (pair_index >= 0 && pair_index < 6) {
                    uwb_calibration_record_measurement(
                        "three_module_edm", &measurement,
                        &pair_stats[pair_index]);
                }
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "UWB CAL three-module RX slice failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
        }

        if ((int32_t)(xTaskGetTickCount() - next_initiate_tick) >= 0) {
            const uint8_t peer_id = peers[next_peer_index];
            (void)uwb_distance_initiate_once(peer_id, sequence++, false);
            next_peer_index = (next_peer_index + 1U) % peer_count;
            next_initiate_tick =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(uwb_calibration_random_interval_ms());
        }
    }
}

static void uwb_dw3000_calibration_loop(void)
{
    ESP_LOGI(TAG, "UWB calibration runtime start: method=%s(%u)",
             uwb_calibration_method_name(),
             (unsigned)APP_UWB_CALIBRATION_METHOD);

    switch (APP_UWB_CALIBRATION_METHOD) {
    case APP_UWB_CALIBRATION_METHOD_TWO_MODULE:
        uwb_calibration_two_module_loop();
        break;
    case APP_UWB_CALIBRATION_METHOD_THREE_MODULE:
        uwb_calibration_three_module_loop();
        break;
    default:
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Unsupported UWB calibration method: %u",
                 (unsigned)APP_UWB_CALIBRATION_METHOD);
        vTaskDelete(NULL);
        break;
    }
}

static uint32_t uwb_dw3000_next_random_interval_ms(void)
{
    const uint32_t min_ms = APP_UWB_BEACON_MIN_INTERVAL_MS;
    const uint32_t max_ms = APP_UWB_BEACON_MAX_INTERVAL_MS;
    if (max_ms <= min_ms) {
        return min_ms;
    }

    return min_ms + (esp_random() % (max_ms - min_ms + 1U));
}

static esp_err_t uwb_dw3000_prepare_tx(const uint8_t *payload,
                                       size_t payload_len)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                        "TXRXOFF command failed");
    s_rx_armed = false;
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "clear before TX failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_tx_payload(payload, payload_len), TAG,
                        "TX buffer write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_frame_length(payload_len), TAG,
                        "TX frame length write failed");
    return ESP_OK;
}

static esp_err_t uwb_dw3000_wait_for_tx_complete(uint64_t *tx_timestamp)
{
    const TickType_t start = xTaskGetTickCount();
    while (true) {
        uint32_t status = 0;
        if (uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44, &status) !=
            ESP_OK) {
            s_tx_error_count++;
            return ESP_FAIL;
        }

        if ((status & DW3000_STATUS_TXFRS) != 0) {
            if (tx_timestamp != NULL) {
                ESP_RETURN_ON_ERROR(uwb_dw3000_read_tx_timestamp(tx_timestamp),
                                    TAG, "TX timestamp read failed");
            }
            s_tx_count++;
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX failed");
            return ESP_OK;
        }

        if ((xTaskGetTickCount() - start) >=
            pdMS_TO_TICKS(UWB_DW3000_TX_TIMEOUT_MS)) {
            s_tx_error_count++;
            ESP_LOGW(TAG, "UWB TX timeout SYS_STATUS=0x%08lx",
                     (unsigned long)status);
            ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF),
                                TAG, "TXRXOFF after TX timeout failed");
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX timeout failed");
            return ESP_ERR_TIMEOUT;
        }

        uwb_dw3000_delay_ms(UWB_DW3000_TX_POLL_MS);
    }
}

static esp_err_t uwb_dw3000_send_payload(const uint8_t *payload,
                                         size_t payload_len,
                                         uint64_t *tx_timestamp)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_prepare_tx(payload, payload_len), TAG,
                        "TX prepare failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TX), TAG,
                        "TX command failed");

    return uwb_dw3000_wait_for_tx_complete(tx_timestamp);
}

static esp_err_t uwb_dw3000_send_payload_delayed(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_prepare_tx(payload, payload_len), TAG,
                        "delayed TX prepare failed");

    const uint32_t delayed_time_word =
        uwb_dw3000_delayed_time_word(tx_timestamp);
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_delayed_trx_time(delayed_time_word),
                        TAG, "DX_TIME write failed");

    if (programmed_tx_timestamp != NULL) {
        *programmed_tx_timestamp =
            uwb_dw3000_programmed_tx_timestamp(delayed_time_word);
    }

    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_DTX), TAG,
                        "delayed TX command failed");

    uint32_t status = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44, &status), TAG,
        "SYS_STATUS after delayed TX failed");
    if ((status & DW3000_STATUS_HPDWARN) != 0) {
        s_tx_error_count++;
        ESP_LOGW(TAG,
                 "UWB delayed TX rejected timestamp=0x%010llx word=0x%08lx SYS_STATUS=0x%08lx",
                 (unsigned long long)tx_timestamp,
                 (unsigned long)delayed_time_word, (unsigned long)status);
        ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                            "TXRXOFF after delayed TX reject failed");
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear after delayed TX reject failed");
        return ESP_ERR_INVALID_STATE;
    }

    return uwb_dw3000_wait_for_tx_complete(actual_tx_timestamp);
}

static esp_err_t uwb_dw3000_send_beacon(uint32_t sequence)
{
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_dw3000_build_payload(sequence, payload);

    ESP_RETURN_ON_ERROR(uwb_dw3000_send_payload(payload, sizeof(payload), NULL),
                        TAG, "beacon TX failed");
    ESP_LOGI(TAG, "UWB TX beacon src=%u seq=%lu total_tx=%lu",
             (unsigned)s_source_id, (unsigned long)sequence,
             (unsigned long)s_tx_count);
    return ESP_OK;
}

static void uwb_dw3000_radio_loop(void)
{
    uint32_t sequence = 0;
    TickType_t next_tx_tick =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(uwb_dw3000_next_random_interval_ms());

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB beacon test active: source_id=%u random_tx=%u..%u ms payload=%u bytes",
             (unsigned)s_source_id, (unsigned)APP_UWB_BEACON_MIN_INTERVAL_MS,
             (unsigned)APP_UWB_BEACON_MAX_INTERVAL_MS,
             (unsigned)UWB_DW3000_PAYLOAD_LEN);

    while (true) {
        if (!s_rx_armed) {
            const esp_err_t rx_err = uwb_dw3000_arm_rx();
            if (rx_err != ESP_OK) {
                ESP_LOGW(TAG, "UWB RX arm failed: %s",
                         esp_err_to_name(rx_err));
                uwb_dw3000_delay_ms(50);
                continue;
            }
        }

        const esp_err_t poll_err = uwb_dw3000_poll_rx();
        if (poll_err != ESP_OK) {
            s_rx_error_count++;
            ESP_LOGW(TAG, "UWB RX poll failed: %s", esp_err_to_name(poll_err));
            s_rx_armed = false;
            uwb_dw3000_delay_ms(20);
            continue;
        }

        if (APP_UWB_BEACON_ENABLED &&
            (int32_t)(xTaskGetTickCount() - next_tx_tick) >= 0) {
            (void)uwb_dw3000_send_beacon(sequence++);
            next_tx_tick =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(uwb_dw3000_next_random_interval_ms());
        }

        uwb_dw3000_delay_ms(UWB_DW3000_POLL_INTERVAL_MS);
    }
}

static void uwb_dw3000_task(void *arg)
{
    (void)arg;

    s_status = UWB_DW3000_STATUS_INITIALIZING;
    s_device_id = 0;
    s_source_id = uwb_dw3000_pick_source_id();

    ESP_LOGI(TAG,
             "Starting DW3000 bring-up: CS=%d SCK=%d MISO=%d MOSI=%d RST=%d IRQ=%d WAKEUP=%d",
             BOARD_CONFIG_UWB_CS_GPIO, BOARD_CONFIG_SPI_SCK_GPIO,
             BOARD_CONFIG_SPI_MISO_GPIO, BOARD_CONFIG_SPI_MOSI_GPIO,
             BOARD_CONFIG_UWB_RST_GPIO, BOARD_CONFIG_UWB_IRQ_GPIO,
             BOARD_CONFIG_UWB_WAKEUP_GPIO);
    ESP_LOGI(TAG, "DW3000 GPIO polarity: reset active-%s, wakeup active-%s",
             BOARD_CONFIG_UWB_RST_ACTIVE_HIGH ? "HIGH" : "LOW",
             BOARD_CONFIG_UWB_WAKEUP_ACTIVE_HIGH ? "HIGH" : "LOW");

    esp_err_t err = uwb_dw3000_configure_gpio();
    if (err == ESP_OK) {
        err = uwb_dw3000_hardware_reset();
    }
    if (err == ESP_OK) {
        err = uwb_dw3000_init_spi();
    }

    if (err != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        vTaskDelete(NULL);
        return;
    }

    uwb_dw3000_delay_ms(UWB_DW3000_WAKE_AFTER_RESET_MS);
    s_status = UWB_DW3000_STATUS_PROBING;

    for (int attempt = 1; attempt <= UWB_DW3000_PROBE_ATTEMPTS; ++attempt) {
        err = uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_SUB_NONE,
                                &s_device_id);
        if (err == ESP_OK && uwb_dw3000_device_id_valid(s_device_id)) {
            ESP_LOGI(TAG, "DW3000 DEV_ID: 0x%08lx, SPI link ready",
                     (unsigned long)s_device_id);
            break;
        }

        ESP_LOGW(TAG, "DW3000 probe %d/%d failed: err=%s dev_id=0x%08lx",
                 attempt, UWB_DW3000_PROBE_ATTEMPTS, esp_err_to_name(err),
                 (unsigned long)s_device_id);
        uwb_dw3000_delay_ms(UWB_DW3000_PROBE_INTERVAL_MS);
    }

    if (!uwb_dw3000_device_id_valid(s_device_id)) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "DW3000 bring-up failed, last DEV_ID: 0x%08lx",
                 (unsigned long)s_device_id);
        vTaskDelete(NULL);
        return;
    }

    err = uwb_dw3000_radio_init();
    if (err != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "DW3000 radio init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    if (s_runtime_mode == UWB_DW3000_RUNTIME_DISTANCE_TEST) {
        uwb_dw3000_distance_test_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_CALIBRATION) {
        uwb_dw3000_calibration_loop();
    } else {
        uwb_dw3000_radio_loop();
    }
}

static esp_err_t uwb_dw3000_start_runtime(
    enum uwb_dw3000_runtime_mode runtime_mode)
{
    if (s_started) {
        if (s_runtime_mode != runtime_mode) {
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    s_runtime_mode = runtime_mode;

    const BaseType_t created = xTaskCreatePinnedToCore(
        uwb_dw3000_task, "uwb_dw3000", UWB_DW3000_TASK_STACK_WORDS, NULL,
        UWB_DW3000_TASK_PRIORITY, NULL, UWB_DW3000_TASK_CORE);
    if (created != pdPASS) {
        s_status = UWB_DW3000_STATUS_FAILED;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

esp_err_t uwb_dw3000_start(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_BEACON_SMOKE);
}

esp_err_t uwb_dw3000_start_distance_test(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_DISTANCE_TEST);
}

esp_err_t uwb_dw3000_start_calibration(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_CALIBRATION);
}

bool uwb_dw3000_is_ready(void)
{
    return s_status == UWB_DW3000_STATUS_READY;
}

enum uwb_dw3000_status uwb_dw3000_get_status(void)
{
    return s_status;
}

const char *uwb_dw3000_status_to_string(enum uwb_dw3000_status status)
{
    switch (status) {
    case UWB_DW3000_STATUS_IDLE:
        return "idle";
    case UWB_DW3000_STATUS_INITIALIZING:
        return "initializing";
    case UWB_DW3000_STATUS_PROBING:
        return "probing";
    case UWB_DW3000_STATUS_READY:
        return "ready";
    case UWB_DW3000_STATUS_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

uint32_t uwb_dw3000_get_device_id(void)
{
    return s_device_id;
}

uint8_t uwb_dw3000_get_source_id(void)
{
    return s_source_id;
}

uint32_t uwb_dw3000_get_tx_count(void)
{
    return s_tx_count;
}

uint32_t uwb_dw3000_get_tx_error_count(void)
{
    return s_tx_error_count;
}

uint32_t uwb_dw3000_get_rx_count(void)
{
    return s_rx_count;
}

uint32_t uwb_dw3000_get_rx_error_count(void)
{
    return s_rx_error_count;
}

uint32_t uwb_dw3000_get_rx_ignored_count(void)
{
    return s_rx_ignored_count;
}

uint8_t uwb_dw3000_get_last_rx_source_id(void)
{
    return s_last_rx_source_id;
}

uint32_t uwb_dw3000_get_last_rx_sequence(void)
{
    return s_last_rx_sequence;
}
