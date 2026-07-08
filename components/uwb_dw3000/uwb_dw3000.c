#include "uwb_dw3000.h"

#include <stdbool.h>
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

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#include "uwb_config.h"

#ifndef HOSTNAME
#define HOSTNAME "uwb-module"
#endif

static const char *TAG = "uwb_dw3000";

enum {
    UWB_DW3000_TASK_STACK_WORDS = 6144,
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
    UWB_DW3000_PAYLOAD_LEN = 32,
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

#define DW3000_GPIO_MODE_SUB 0x00
#define DW3000_GPIO_DIR_SUB 0x08
#define DW3000_PMSC_CLK_CTRL_SUB 0x04
#define DW3000_PMSC_LED_CTRL_SUB 0x16

#define DW3000_GPIO_MODE_MSGP2_MODE_BIT_MASK 0x1C0UL
#define DW3000_GPIO_MODE_MSGP3_MODE_BIT_MASK 0xE00UL
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

#define DW3000_PMSC_STATE_IDLE 0x03

static spi_device_handle_t s_spi;
static bool s_started;
static bool s_rx_armed;
static uint8_t s_source_id;
static uint32_t s_device_id;
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

static uint8_t uwb_dw3000_parse_hostname_id(const char *hostname)
{
    uint16_t value = 0;
    bool has_digit = false;

    if (hostname == NULL) {
        return 0;
    }

    for (size_t i = 0; hostname[i] != '\0'; ++i) {
        const char c = hostname[i];
        if (c >= '0' && c <= '9') {
            has_digit = true;
            value = (uint16_t)(value * 10U + (uint16_t)(c - '0'));
            if (value > 255U) {
                return 0;
            }
        } else {
            has_digit = false;
            value = 0;
        }
    }

    return has_digit && value > 0 ? (uint8_t)value : 0;
}

static uint8_t uwb_dw3000_pick_source_id(void)
{
    if (APP_UWB_SOURCE_ID > 0 && APP_UWB_SOURCE_ID <= 255) {
        return (uint8_t)APP_UWB_SOURCE_ID;
    }

    const uint8_t hostname_id = uwb_dw3000_parse_hostname_id(HOSTNAME);
    if (hostname_id != 0) {
        return hostname_id;
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

static esp_err_t uwb_dw3000_write_tx_payload(
    const uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return uwb_dw3000_write_bytes(DW3000_REG_TX_BUFFER, DW3000_SUB_NONE,
                                  payload, UWB_DW3000_PAYLOAD_LEN);
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
        uwb_dw3000_write_u32_auto(DW3000_REG_GEN_CFG_AES_HIGH, 0x04,
                                  UWB_DW3000_ANTENNA_DELAY),
        TAG, "TX antenna delay write failed");

    return ESP_OK;
}

static esp_err_t uwb_dw3000_configure_hardware_leds(void)
{
    if (!APP_UWB_DW_LEDS_ENABLED) {
        ESP_LOGI(TAG, "DW3000 hardware TX/RX LEDs disabled");
        return ESP_OK;
    }

    if (BOARD_CONFIG_UWB_RX_DW_LED_INDEX != 2 ||
        BOARD_CONFIG_UWB_TX_DW_LED_INDEX != 3) {
        ESP_LOGE(TAG,
                 "DW3000 hardware LED functions require RXLED=GPIO2 and "
                 "TXLED=GPIO3, got RX=%d TX=%d",
                 BOARD_CONFIG_UWB_RX_DW_LED_INDEX,
                 BOARD_CONFIG_UWB_TX_DW_LED_INDEX);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const uint32_t gpio_led_mask =
        DW3000_GPIO_MODE_MSGP2_MODE_BIT_MASK |
        DW3000_GPIO_MODE_MSGP3_MODE_BIT_MASK;
    const uint32_t gpio_led_mode =
        DW3000_GPIO_PIN2_RXLED | DW3000_GPIO_PIN3_TXLED;

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
             "DW3000 hardware LEDs enabled: RXLED=GPIO%d TXLED=GPIO%d "
             "blink_time=0x%02x",
             BOARD_CONFIG_UWB_RX_DW_LED_INDEX,
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

static uint32_t uwb_dw3000_next_random_interval_ms(void)
{
    const uint32_t min_ms = APP_UWB_BEACON_MIN_INTERVAL_MS;
    const uint32_t max_ms = APP_UWB_BEACON_MAX_INTERVAL_MS;
    if (max_ms <= min_ms) {
        return min_ms;
    }

    return min_ms + (esp_random() % (max_ms - min_ms + 1U));
}

static esp_err_t uwb_dw3000_send_beacon(uint32_t sequence)
{
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_dw3000_build_payload(sequence, payload);

    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                        "TXRXOFF command failed");
    s_rx_armed = false;
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "clear before TX failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_tx_payload(payload), TAG,
                        "TX buffer write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_frame_length(sizeof(payload)), TAG,
                        "TX frame length write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TX), TAG,
                        "TX command failed");

    const TickType_t start = xTaskGetTickCount();
    while (true) {
        uint32_t status = 0;
        if (uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44, &status) !=
            ESP_OK) {
            s_tx_error_count++;
            return ESP_FAIL;
        }

        if ((status & DW3000_STATUS_TXFRS) != 0) {
            s_tx_count++;
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX failed");
            ESP_LOGI(TAG, "UWB TX beacon src=%u seq=%lu total_tx=%lu",
                     (unsigned)s_source_id, (unsigned long)sequence,
                     (unsigned long)s_tx_count);
            return ESP_OK;
        }

        if ((xTaskGetTickCount() - start) >=
            pdMS_TO_TICKS(UWB_DW3000_TX_TIMEOUT_MS)) {
            s_tx_error_count++;
            ESP_LOGW(TAG, "UWB TX timeout seq=%lu SYS_STATUS=0x%08lx",
                     (unsigned long)sequence, (unsigned long)status);
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX timeout failed");
            return ESP_ERR_TIMEOUT;
        }

        uwb_dw3000_delay_ms(UWB_DW3000_TX_POLL_MS);
    }
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

    uwb_dw3000_radio_loop();
}

esp_err_t uwb_dw3000_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

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
