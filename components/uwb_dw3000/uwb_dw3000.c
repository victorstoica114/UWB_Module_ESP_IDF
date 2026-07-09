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
#include "app_runtime_config.h"
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
#define DW3000_CMD_TX_W4R 0x0C
#define DW3000_CMD_DTX_W4R 0x0D

#define DW3000_SYS_CFG_SUB 0x10
#define DW3000_TX_FCTRL_SUB 0x24
#define DW3000_DX_TIME_SUB 0x2C
#define DW3000_RX_FWTO_SUB 0x34
#define DW3000_SYS_ENABLE_LO_SUB 0x3C
#define DW3000_SYS_ENABLE_HI_SUB 0x40
#define DW3000_SYS_STATUS_SUB 0x44
#define DW3000_RX_FINFO_SUB 0x4C
#define DW3000_RX_TIME_SUB 0x00
#define DW3000_TX_ANTD_SUB 0x04
#define DW3000_ACK_RESP_SUB 0x08
#define DW3000_CHAN_CTRL_SUB 0x14
#define DW3000_DRX_CAR_INT_SUB 0x29
#define DW3000_TX_TIME_SUB 0x74
#define DW3000_STS_CONFIG_LO_SUB 0x12
#define DW3000_STS_CONFIG_HI_SUB 0x16

#define DW3000_CIA_DIAG_0_SUB 0x20
#define DW3000_IP_DIAG_0_SUB 0x28
#define DW3000_IP_DIAG_1_SUB 0x2C
#define DW3000_IP_DIAG_2_SUB 0x30
#define DW3000_IP_DIAG_3_SUB 0x34
#define DW3000_IP_DIAG_4_SUB 0x38
#define DW3000_IP_DIAG_8_SUB 0x48
#define DW3000_IP_DIAG_12_SUB 0x58
#define DW3000_RDB_DIAG_MODE_SUB 0x28

#define DW3000_EVC_CTRL_SUB 0x00
#define DW3000_EVC_COUNT0_SUB 0x04
#define DW3000_EVC_COUNT1_SUB 0x08
#define DW3000_EVC_COUNT2_SUB 0x0C
#define DW3000_EVC_COUNT3_SUB 0x10
#define DW3000_EVC_COUNT4_SUB 0x14
#define DW3000_EVC_COUNT5_SUB 0x18
#define DW3000_EVC_COUNT6_SUB 0x1C
#define DW3000_EVC_COUNT7_SUB 0x28

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
#define DW3000_IRQ_STATUS_MASK \
    (DW3000_STATUS_TXFRS | DW3000_RX_GOOD_MASK | DW3000_RX_ERROR_MASK | \
     DW3000_RX_TIMEOUT_MASK | DW3000_STATUS_HPDWARN)
#define DW3000_RX_FINFO_RXFLEN_MASK 0x000003FFUL
#define DW3000_RX_FINFO_RXPACC_MASK 0xFFF00000UL
#define DW3000_RX_FINFO_RXPACC_SHIFT 20U
#define DW3000_TX_FCTRL_TXB_OFFSET_MASK 0x03FF0000UL
#define DW3000_TX_FCTRL_TR_MASK 0x00000800UL
#define DW3000_TX_FCTRL_TXFLEN_MASK 0x000003FFUL
#define DW3000_TX_FCTRL_TXPSR_MASK 0x0000F000UL
#define DW3000_TX_FCTRL_TXPSR_SHIFT 12U
#define DW3000_TX_FCTRL_TXBR_MASK 0x00000400UL
#define DW3000_TX_FCTRL_TXBR_SHIFT 10U
#define DW3000_CHAN_CTRL_RX_PCODE_MASK 0x00001F00UL
#define DW3000_CHAN_CTRL_RX_PCODE_SHIFT 8U
#define DW3000_CHAN_CTRL_TX_PCODE_MASK 0x000000F8UL
#define DW3000_CHAN_CTRL_TX_PCODE_SHIFT 3U
#define DW3000_CHAN_CTRL_SFD_TYPE_MASK 0x00000006UL
#define DW3000_CHAN_CTRL_SFD_TYPE_SHIFT 1U
#define DW3000_CHAN_CTRL_RF_CHAN_MASK 0x00000001UL
#define DW3000_SYS_CFG_RXWTOE_BIT_MASK 0x00000200UL
#define DW3000_SYS_CFG_CP_SPC_BIT_MASK 0x00003000UL
#define DW3000_SYS_CFG_CP_SDC_BIT_MASK 0x00008000UL
#define DW3000_SYS_CFG_STS_MODE_MASK \
    (DW3000_SYS_CFG_CP_SPC_BIT_MASK | DW3000_SYS_CFG_CP_SDC_BIT_MASK)
#define DW3000_SYS_CFG_STS_MODE_SHIFT 12U
#define DW3000_ACK_RESP_W4R_TIM_BIT_MASK 0x000FFFFFUL

#define DW3000_STS_CONFIG_HI_RES 0x94UL
#define DW3000_STS_CONFIG_HI_CHECK_MASK \
    (0x80000000UL | 0x40000000UL | 0x000000F0UL)

#define DW3000_EVC_CTRL_CLR_BIT_MASK 0x02U
#define DW3000_EVC_CTRL_EN_BIT_MASK 0x01U
#define DW3000_EVC_12BIT_LOW_MASK 0x00000FFFUL
#define DW3000_EVC_12BIT_HIGH_MASK 0x0FFF0000UL
#define DW3000_EVC_8BIT_LOW_MASK 0x000000FFUL
#define DW3000_EVC_8BIT_HIGH_MASK 0x00FF0000UL
#define DW3000_EVC_HIGH_SHIFT 16U

#define DW3000_CIA_CONF_DIAGNOSTIC_OFF_MASK 0x00100000UL
#define DW3000_CIA_DIAG_LOG_ALL 0x01U
#define DW3000_CLOCK_OFFSET_RAW_MASK 0x001FFFFFUL
#define DW3000_CLOCK_OFFSET_RAW_BITS 21U
#define DW3000_CLOCK_OFFSET_CH5_FACTOR (-0.5731e-9)
#define DW3000_CLOCK_OFFSET_CH9_FACTOR (-0.1252e-9)
#define DW3000_IPATOV_PEAK_MASK 0x7FFFFFFFUL
#define DW3000_IPATOV_PEAK_AMP_MASK 0x001FFFFFUL
#define DW3000_IPATOV_PEAK_INDEX_SHIFT 21U
#define DW3000_IPATOV_POWER_MASK 0x0001FFFFUL
#define DW3000_IPATOV_F_MASK 0x003FFFFFUL
#define DW3000_IPATOV_FP_INDEX_MASK 0x0000FFFFUL
#define DW3000_IPATOV_ACCUM_COUNT_MASK 0x00000FFFUL
#define DW3000_CIA_XTAL_OFFSET_MASK 0x00001FFFUL

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
#define UWB_DISTANCE_FRAME_DISTANCE_MM_OFFSET 40U
#define UWB_DISTANCE_FRAME_RAW_DISTANCE_MM_OFFSET 44U
#define UWB_DISTANCE_FRAME_BROADCAST_ID 255U

#define UWB_ANCHOR_SURVEY_MAX_ANCHORS 4U
#define UWB_ANCHOR_SURVEY_MAX_PAIRS \
    ((UWB_ANCHOR_SURVEY_MAX_ANCHORS * (UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U)) / 2U)
#define UWB_ANCHOR_SURVEY_CMD_INITIATOR_OFFSET 10U
#define UWB_ANCHOR_SURVEY_CMD_RESPONDER_OFFSET 11U
#define UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET 12U

enum uwb_distance_frame_type {
    UWB_DISTANCE_FRAME_POLL = 1,
    UWB_DISTANCE_FRAME_RESP = 2,
    UWB_DISTANCE_FRAME_FINAL = 3,
    UWB_DISTANCE_FRAME_REPORT = 4,
    UWB_DISTANCE_FRAME_SURVEY_CMD = 5,
    UWB_DISTANCE_FRAME_REPORT2 = 6,
};

enum uwb_dw3000_runtime_mode {
    UWB_DW3000_RUNTIME_BEACON_SMOKE = 0,
    UWB_DW3000_RUNTIME_DISTANCE_TEST,
    UWB_DW3000_RUNTIME_CALIBRATION,
    UWB_DW3000_RUNTIME_ANCHOR_SURVEY,
    UWB_DW3000_RUNTIME_RANGING,
};

struct uwb_rx_diagnostics {
    bool valid;
    uint16_t rx_pacc;
    int16_t xtal_offset;
    uint32_t ipatov_peak_amp;
    uint16_t ipatov_peak_index;
    uint32_t ipatov_power;
    uint32_t ipatov_f1;
    uint32_t ipatov_f2;
    uint32_t ipatov_f3;
    uint16_t ipatov_fp_index;
    uint16_t ipatov_accum_count;
};

struct uwb_event_counters {
    uint16_t rse;
    uint16_t phe;
    uint16_t fce;
    uint16_t fcg;
    uint8_t ovr;
    uint8_t ffr;
    uint16_t pto;
    uint16_t sfdt;
    uint16_t txfs;
    uint8_t fwto;
    uint8_t swce;
    uint8_t hpw;
    uint16_t prej;
    uint8_t vwarn;
    uint8_t cpqe;
};

struct uwb_dw3000_rx_frame {
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN];
    uint16_t payload_len;
    uint64_t rx_timestamp;
    bool clock_offset_valid;
    int32_t clock_offset_raw;
    struct uwb_rx_diagnostics diagnostics;
};

struct uwb_distance_frame {
    uint8_t type;
    uint8_t source_id;
    uint8_t destination_id;
    uint16_t sequence;
    uint64_t rx_timestamp;
    bool clock_offset_valid;
    int32_t clock_offset_raw;
    struct uwb_rx_diagnostics diagnostics;
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN];
    uint16_t payload_len;
};

struct uwb_distance_measurement {
    uint8_t initiator_id;
    uint8_t responder_id;
    uint16_t sequence;
    double tof_dtu;
    double distance_m;
    double raw_tof_dtu;
    double raw_distance_m;
    double clock_offset_ratio;
    bool clock_offset_valid;
    int32_t clock_offset_raw;
    uint64_t poll_tx_ts;
    uint64_t poll_rx_ts;
    uint64_t resp_tx_ts;
    uint64_t resp_rx_ts;
    uint64_t final_tx_ts;
    uint64_t final_rx_ts;
    struct uwb_rx_diagnostics poll_rx_diagnostics;
    struct uwb_rx_diagnostics final_rx_diagnostics;
    struct uwb_rx_diagnostics report_rx_diagnostics;
};

struct uwb_calibration_stats {
    uint32_t samples;
    double mean_m;
    double m2_m;
    double min_m;
    double max_m;
    double last_m;
};

struct uwb_anchor_survey_pair {
    uint8_t initiator_id;
    uint8_t responder_id;
};

static uint32_t uwb_dw3000_remaining_ms(TickType_t start_tick,
                                        uint32_t timeout_ms);
static esp_err_t uwb_dw3000_send_payload(const uint8_t *payload,
                                         size_t payload_len,
                                         uint64_t *tx_timestamp);
static esp_err_t uwb_dw3000_send_payload_expect_rx(
    const uint8_t *payload, size_t payload_len, uint32_t rx_after_tx_delay_uus,
    uint32_t rx_timeout_ms, uint64_t *tx_timestamp);
static esp_err_t uwb_dw3000_send_payload_delayed(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp);
static esp_err_t uwb_dw3000_send_payload_delayed_expect_rx(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp);
static esp_err_t uwb_dw3000_update_u32(uint8_t base, uint8_t sub,
                                       uint32_t clear_mask,
                                       uint32_t set_mask);
static void uwb_distance_fill_tag_measurement(
    uint8_t peer_id, uint16_t sequence, uint64_t poll_tx_ts,
    const struct uwb_distance_frame *response, uint64_t final_tx_ts,
    const struct uwb_distance_frame *report2,
    struct uwb_distance_measurement *measurement);
static void
uwb_distance_log_tag_verification(const struct uwb_distance_measurement *measurement,
                                  const struct uwb_distance_frame *report2);

#define DW3000_PMSC_STATE_IDLE 0x03

static spi_device_handle_t s_spi;
static bool s_started;
static bool s_rx_armed;
static bool s_irq_enabled;
static TaskHandle_t s_task_handle;
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
static uint16_t s_antenna_delay = APP_UWB_ANTENNA_DELAY_DEFAULT;

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

static void IRAM_ATTR uwb_dw3000_irq_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void uwb_dw3000_wait_for_event_or_delay(TickType_t start_tick,
                                               uint32_t timeout_ms,
                                               uint32_t fallback_delay_ms)
{
#if APP_UWB_IRQ_ENABLED
    if (s_irq_enabled) {
        const uint32_t remaining_ms =
            uwb_dw3000_remaining_ms(start_tick, timeout_ms);
        if (remaining_ms == 0) {
            return;
        }

        TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
        if (ticks == 0) {
            ticks = 1;
        }
        (void)ulTaskNotifyTake(pdTRUE, ticks);
        return;
    }
#else
    (void)start_tick;
    (void)timeout_ms;
#endif

    uwb_dw3000_delay_ms(fallback_delay_ms);
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

static esp_err_t uwb_dw3000_configure_host_irq(void)
{
#if APP_UWB_IRQ_ENABLED
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "DW3000 host IRQ service unavailable: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(BOARD_CONFIG_UWB_IRQ_GPIO,
                               uwb_dw3000_irq_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DW3000 host IRQ handler unavailable: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_set_intr_type(BOARD_CONFIG_UWB_IRQ_GPIO,
                                           GPIO_INTR_POSEDGE),
                        TAG, "set UWB IRQ edge failed");
    ESP_RETURN_ON_ERROR(gpio_intr_enable(BOARD_CONFIG_UWB_IRQ_GPIO), TAG,
                        "enable UWB IRQ GPIO failed");

    s_irq_enabled = true;
    ESP_LOGI(TAG, "DW3000 host IRQ enabled on GPIO%d",
             BOARD_CONFIG_UWB_IRQ_GPIO);
    return ESP_OK;
#else
    s_irq_enabled = false;
    ESP_LOGI(TAG, "DW3000 host IRQ disabled by config");
    return ESP_OK;
#endif
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

static int32_t uwb_dw3000_sign_extend(uint32_t value, uint8_t bits)
{
    const uint32_t sign_bit = 1UL << (bits - 1U);
    const uint32_t mask = (1UL << bits) - 1UL;
    value &= mask;
    return (int32_t)((value ^ sign_bit) - sign_bit);
}

static esp_err_t uwb_dw3000_read_clock_offset_raw(int32_t *clock_offset_raw)
{
    if (clock_offset_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t raw = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_DRX, DW3000_DRX_CAR_INT_SUB, &raw), TAG,
        "clock offset read failed");
    *clock_offset_raw = uwb_dw3000_sign_extend(
        raw & DW3000_CLOCK_OFFSET_RAW_MASK, DW3000_CLOCK_OFFSET_RAW_BITS);
    return ESP_OK;
}

static double uwb_dw3000_clock_offset_ratio(int32_t clock_offset_raw)
{
    const double factor = APP_UWB_RADIO_RF_CHANNEL_BIT == 0
                              ? DW3000_CLOCK_OFFSET_CH5_FACTOR
                              : DW3000_CLOCK_OFFSET_CH9_FACTOR;
    return (double)clock_offset_raw * factor;
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
            s_antenna_delay) &
           UWB_DW3000_TIMESTAMP_MASK;
}

static esp_err_t uwb_dw3000_set_delayed_trx_time(uint32_t delayed_time_word)
{
    return uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                    DW3000_DX_TIME_SUB, delayed_time_word, 4);
}

static esp_err_t
uwb_dw3000_configure_cia_diagnostics(bool enable_all_registers)
{
    if (!enable_all_registers) {
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_update_u32(DW3000_REG_CIA_3, DW3000_SUB_NONE, 0,
                                  DW3000_CIA_CONF_DIAGNOSTIC_OFF_MASK),
            TAG, "CIA diagnostics disable failed");
        ESP_LOGI(TAG, "DW3000 CIA diagnostics disabled");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(DW3000_REG_CIA_3, DW3000_SUB_NONE,
                              DW3000_CIA_CONF_DIAGNOSTIC_OFF_MASK, 0),
        TAG, "CIA diagnostics enable failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_HIGH,
                                 DW3000_RDB_DIAG_MODE_SUB,
                                 DW3000_CIA_DIAG_LOG_ALL >> 1U, 1),
        TAG, "RDB_DIAG_MODE write failed");
    ESP_LOGI(TAG, "DW3000 CIA diagnostics enabled");
    return ESP_OK;
}

static esp_err_t
uwb_dw3000_read_rx_diagnostics(struct uwb_rx_diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(diagnostics, 0, sizeof(*diagnostics));

#if APP_UWB_DIAGNOSTICS_ENABLED
    uint32_t rx_finfo = 0;
    uint32_t cia_diag0 = 0;
    uint32_t ip_diag0 = 0;
    uint32_t ip_diag1 = 0;
    uint32_t ip_diag2 = 0;
    uint32_t ip_diag3 = 0;
    uint32_t ip_diag4 = 0;
    uint32_t ip_diag8 = 0;
    uint32_t ip_diag12 = 0;

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_RX_FINFO_SUB,
                          &rx_finfo),
        TAG,
        "RX diagnostics RX_FINFO read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_CIA_DIAG_0_SUB,
                          &cia_diag0),
        TAG, "RX diagnostics CIA_DIAG_0 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_0_SUB, &ip_diag0),
        TAG, "RX diagnostics IP_DIAG_0 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_1_SUB, &ip_diag1),
        TAG, "RX diagnostics IP_DIAG_1 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_2_SUB, &ip_diag2),
        TAG, "RX diagnostics IP_DIAG_2 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_3_SUB, &ip_diag3),
        TAG, "RX diagnostics IP_DIAG_3 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_4_SUB, &ip_diag4),
        TAG, "RX diagnostics IP_DIAG_4 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_8_SUB, &ip_diag8),
        TAG, "RX diagnostics IP_DIAG_8 read failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_CIA_1, DW3000_IP_DIAG_12_SUB,
                          &ip_diag12),
        TAG, "RX diagnostics IP_DIAG_12 read failed");

    diagnostics->valid = true;
    diagnostics->rx_pacc =
        (uint16_t)((rx_finfo & DW3000_RX_FINFO_RXPACC_MASK) >>
                   DW3000_RX_FINFO_RXPACC_SHIFT);
    diagnostics->xtal_offset = (int16_t)uwb_dw3000_sign_extend(
        cia_diag0 & DW3000_CIA_XTAL_OFFSET_MASK, 13);
    ip_diag0 &= DW3000_IPATOV_PEAK_MASK;
    diagnostics->ipatov_peak_amp =
        ip_diag0 & DW3000_IPATOV_PEAK_AMP_MASK;
    diagnostics->ipatov_peak_index =
        (uint16_t)(ip_diag0 >> DW3000_IPATOV_PEAK_INDEX_SHIFT);
    diagnostics->ipatov_power = ip_diag1 & DW3000_IPATOV_POWER_MASK;
    diagnostics->ipatov_f1 = ip_diag2 & DW3000_IPATOV_F_MASK;
    diagnostics->ipatov_f2 = ip_diag3 & DW3000_IPATOV_F_MASK;
    diagnostics->ipatov_f3 = ip_diag4 & DW3000_IPATOV_F_MASK;
    diagnostics->ipatov_fp_index =
        (uint16_t)(ip_diag8 & DW3000_IPATOV_FP_INDEX_MASK);
    diagnostics->ipatov_accum_count =
        (uint16_t)(ip_diag12 & DW3000_IPATOV_ACCUM_COUNT_MASK);
#endif

    return ESP_OK;
}

static bool uwb_dw3000_payload_is_distance_frame(const uint8_t *payload,
                                                 size_t payload_len)
{
    return payload != NULL && payload_len >= UWB_DISTANCE_FRAME_HEADER_LEN &&
           payload[0] == UWB_DISTANCE_FRAME_MAGIC_0 &&
           payload[1] == UWB_DISTANCE_FRAME_MAGIC_1 &&
           payload[2] == UWB_DISTANCE_FRAME_MAGIC_2 &&
           payload[3] == UWB_DISTANCE_FRAME_MAGIC_3 &&
           payload[4] == UWB_DISTANCE_FRAME_VERSION;
}

static bool uwb_dw3000_should_capture_rx_diagnostics(const uint8_t *payload,
                                                     size_t payload_len)
{
#if APP_UWB_DIAGNOSTICS_ENABLED
    if (!uwb_dw3000_payload_is_distance_frame(payload, payload_len) ||
        APP_UWB_DIAGNOSTICS_LOG_EVERY == 0) {
        return false;
    }

    const uint16_t sequence =
        (uint16_t)(((uint16_t)payload[8]) | ((uint16_t)payload[9] << 8));
    return (sequence % APP_UWB_DIAGNOSTICS_LOG_EVERY) == 0;
#else
    (void)payload;
    (void)payload_len;
    return false;
#endif
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

static esp_err_t uwb_dw3000_configure_device_interrupts(void)
{
#if APP_UWB_IRQ_ENABLED
    if (!s_irq_enabled) {
        ESP_LOGW(TAG, "DW3000 IRQ fallback: host IRQ is unavailable");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_ENABLE_LO_SUB, 0, 4),
        TAG, "SYS_ENABLE low clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_ENABLE_HI_SUB, 0, 2),
        TAG, "SYS_ENABLE high clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_ENABLE_LO_SUB,
                                 DW3000_IRQ_STATUS_MASK, 4),
        TAG, "SYS_ENABLE low write failed");

    ESP_LOGI(TAG, "DW3000 device IRQ mask enabled: 0x%08lx",
             (unsigned long)DW3000_IRQ_STATUS_MASK);
#else
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_ENABLE_LO_SUB, 0, 4),
        TAG, "SYS_ENABLE low disable failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_ENABLE_HI_SUB, 0, 2),
        TAG, "SYS_ENABLE high disable failed");
    ESP_LOGI(TAG, "DW3000 device IRQ mask disabled by config");
#endif
    return ESP_OK;
}

static esp_err_t
uwb_dw3000_set_rx_after_tx_delay(uint32_t rx_after_tx_delay_uus)
{
    if ((rx_after_tx_delay_uus & ~DW3000_ACK_RESP_W4R_TIM_BIT_MASK) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return uwb_dw3000_update_u32(DW3000_REG_GEN_CFG_AES_HIGH,
                                 DW3000_ACK_RESP_SUB,
                                 DW3000_ACK_RESP_W4R_TIM_BIT_MASK,
                                 rx_after_tx_delay_uus);
}

static esp_err_t uwb_dw3000_set_rx_timeout(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        return uwb_dw3000_update_u32(DW3000_REG_GEN_CFG_AES_LOW,
                                     DW3000_SYS_CFG_SUB,
                                     DW3000_SYS_CFG_RXWTOE_BIT_MASK, 0);
    }

    const uint32_t timeout_units = timeout_ms * 1000UL;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_RX_FWTO_SUB, timeout_units, 4),
        TAG, "RX_FWTO write failed");
    return uwb_dw3000_update_u32(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_CFG_SUB, 0,
                                 DW3000_SYS_CFG_RXWTOE_BIT_MASK);
}

static esp_err_t uwb_dw3000_prepare_rx_after_tx(uint32_t delay_uus,
                                                uint32_t timeout_ms)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_rx_after_tx_delay(delay_uus), TAG,
                        "RX-after-TX delay config failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_rx_timeout(timeout_ms), TAG,
                        "RX timeout config failed");
    return ESP_OK;
}

static bool uwb_dw3000_sts_mode_supported(uint32_t sts_mode)
{
    const uint32_t protocol = sts_mode & 0x3UL;
    const uint32_t allowed_bits = 0xBUL;

    if ((sts_mode & ~allowed_bits) != 0) {
        return false;
    }

    if (sts_mode == APP_UWB_STS_MODE_OFF) {
        return true;
    }

    return protocol == APP_UWB_STS_MODE_1 ||
           protocol == APP_UWB_STS_MODE_2;
}

static esp_err_t uwb_dw3000_sts_length_reg_value(uint32_t symbols,
                                                 uint8_t *reg_value)
{
    if (reg_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (symbols) {
    case 32:
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
    case 2048:
        *reg_value = (uint8_t)((symbols / 8U) - 1U);
        return ESP_OK;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static const char *uwb_dw3000_sts_mode_name(uint32_t sts_mode)
{
    switch (sts_mode) {
    case APP_UWB_STS_MODE_OFF:
        return "off";
    case APP_UWB_STS_MODE_1:
        return "mode1";
    case APP_UWB_STS_MODE_2:
        return "mode2";
    case APP_UWB_STS_MODE_1 | APP_UWB_STS_MODE_SDC:
        return "mode1_sdc";
    case APP_UWB_STS_MODE_2 | APP_UWB_STS_MODE_SDC:
        return "mode2_sdc";
    default:
        return "unsupported";
    }
}

static esp_err_t uwb_dw3000_configure_sts(void)
{
    const uint32_t sts_mode = APP_UWB_STS_MODE & 0xFUL;
    uint8_t sts_len_reg = 0;

    if (!uwb_dw3000_sts_mode_supported(sts_mode)) {
        ESP_LOGE(TAG, "Unsupported STS mode 0x%lx for data frames",
                 (unsigned long)sts_mode);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_sts_length_reg_value(APP_UWB_STS_LENGTH_SYMBOLS,
                                        &sts_len_reg),
        TAG, "invalid STS length");

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_STS_CFG, DW3000_SUB_NONE,
                                 sts_len_reg, 1),
        TAG, "STS length write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_CFG_SUB,
                              DW3000_SYS_CFG_STS_MODE_MASK,
                              sts_mode << DW3000_SYS_CFG_STS_MODE_SHIFT),
        TAG, "STS mode write failed");

    if (sts_mode != APP_UWB_STS_MODE_OFF) {
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_update_u32(DW3000_REG_CIA_3,
                                  DW3000_STS_CONFIG_HI_SUB,
                                  DW3000_STS_CONFIG_HI_CHECK_MASK,
                                  DW3000_STS_CONFIG_HI_RES),
            TAG, "STS quality config write failed");
    }

    ESP_LOGI(TAG, "DW3000 STS config: mode=%s(0x%lx) length=%u symbols",
             uwb_dw3000_sts_mode_name(sts_mode), (unsigned long)sts_mode,
             (unsigned)APP_UWB_STS_LENGTH_SYMBOLS);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_configure_event_counters(void)
{
#if APP_UWB_EVENT_COUNTERS_ENABLED
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_DIG_DIAG, DW3000_EVC_CTRL_SUB,
                                 DW3000_EVC_CTRL_CLR_BIT_MASK, 1),
        TAG, "event counter clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_DIG_DIAG, DW3000_EVC_CTRL_SUB,
                                 DW3000_EVC_CTRL_EN_BIT_MASK, 1),
        TAG, "event counter enable failed");
    ESP_LOGI(TAG, "DW3000 event counters enabled");
#else
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_DIG_DIAG, DW3000_EVC_CTRL_SUB, 0,
                                 1),
        TAG, "event counter disable failed");
    ESP_LOGI(TAG, "DW3000 event counters disabled");
#endif
    return ESP_OK;
}

static esp_err_t
uwb_dw3000_read_event_counters(struct uwb_event_counters *counters)
{
    if (counters == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(counters, 0, sizeof(*counters));

#if APP_UWB_EVENT_COUNTERS_ENABLED
    uint32_t count0 = 0;
    uint32_t count1 = 0;
    uint32_t count2 = 0;
    uint32_t count3 = 0;
    uint32_t count4 = 0;
    uint32_t count5 = 0;
    uint32_t count6 = 0;
    uint32_t count7 = 0;

    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT0_SUB, &count0),
                        TAG, "EVC_COUNT0 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT1_SUB, &count1),
                        TAG, "EVC_COUNT1 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT2_SUB, &count2),
                        TAG, "EVC_COUNT2 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT3_SUB, &count3),
                        TAG, "EVC_COUNT3 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT4_SUB, &count4),
                        TAG, "EVC_COUNT4 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT5_SUB, &count5),
                        TAG, "EVC_COUNT5 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT6_SUB, &count6),
                        TAG, "EVC_COUNT6 read failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                          DW3000_EVC_COUNT7_SUB, &count7),
                        TAG, "EVC_COUNT7 read failed");

    counters->phe = (uint16_t)(count0 & DW3000_EVC_12BIT_LOW_MASK);
    counters->rse = (uint16_t)((count0 & DW3000_EVC_12BIT_HIGH_MASK) >>
                               DW3000_EVC_HIGH_SHIFT);
    counters->fcg = (uint16_t)(count1 & DW3000_EVC_12BIT_LOW_MASK);
    counters->fce = (uint16_t)((count1 & DW3000_EVC_12BIT_HIGH_MASK) >>
                               DW3000_EVC_HIGH_SHIFT);
    counters->ffr = (uint8_t)(count2 & DW3000_EVC_8BIT_LOW_MASK);
    counters->ovr = (uint8_t)((count2 & DW3000_EVC_8BIT_HIGH_MASK) >>
                              DW3000_EVC_HIGH_SHIFT);
    counters->sfdt = (uint16_t)(count3 & DW3000_EVC_12BIT_LOW_MASK);
    counters->pto = (uint16_t)((count3 & DW3000_EVC_12BIT_HIGH_MASK) >>
                               DW3000_EVC_HIGH_SHIFT);
    counters->fwto = (uint8_t)(count4 & DW3000_EVC_8BIT_LOW_MASK);
    counters->txfs = (uint16_t)((count4 & DW3000_EVC_12BIT_HIGH_MASK) >>
                                DW3000_EVC_HIGH_SHIFT);
    counters->hpw = (uint8_t)(count5 & DW3000_EVC_8BIT_LOW_MASK);
    counters->swce = (uint8_t)((count5 & DW3000_EVC_8BIT_HIGH_MASK) >>
                               DW3000_EVC_HIGH_SHIFT);
    counters->prej = (uint16_t)(count6 & DW3000_EVC_12BIT_LOW_MASK);
    counters->cpqe = (uint8_t)(count7 & DW3000_EVC_8BIT_LOW_MASK);
    counters->vwarn = (uint8_t)((count7 & DW3000_EVC_8BIT_HIGH_MASK) >>
                                DW3000_EVC_HIGH_SHIFT);
#endif

    return ESP_OK;
}

static bool uwb_dw3000_should_log_event_counters(uint16_t sequence)
{
#if APP_UWB_EVENT_COUNTERS_ENABLED
    if (APP_UWB_EVENT_COUNTERS_LOG_EVERY == 0) {
        return false;
    }
    return (sequence % APP_UWB_EVENT_COUNTERS_LOG_EVERY) == 0;
#else
    (void)sequence;
    return false;
#endif
}

static void uwb_dw3000_maybe_log_event_counters(uint16_t sequence)
{
    if (!uwb_dw3000_should_log_event_counters(sequence)) {
        return;
    }

    struct uwb_event_counters counters = {0};
    const esp_err_t err = uwb_dw3000_read_event_counters(&counters);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DW3000 event counter read failed seq=%u: %s",
                 (unsigned)sequence, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG,
             "DW3000 counters seq=%u fcg=%u fce=%u phe=%u rse=%u sfdt=%u pto=%u fwto=%u txfs=%u hpw=%u prej=%u cpqe=%u vwarn=%u ffr=%u ovr=%u",
             (unsigned)sequence, (unsigned)counters.fcg,
             (unsigned)counters.fce, (unsigned)counters.phe,
             (unsigned)counters.rse, (unsigned)counters.sfdt,
             (unsigned)counters.pto, (unsigned)counters.fwto,
             (unsigned)counters.txfs, (unsigned)counters.hpw,
             (unsigned)counters.prej, (unsigned)counters.cpqe,
             (unsigned)counters.vwarn, (unsigned)counters.ffr,
             (unsigned)counters.ovr);
}

static esp_err_t uwb_dw3000_read_rx_payload(
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN], uint16_t *payload_len)
{
    if (payload == NULL || payload_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t rx_finfo = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_RX_FINFO_SUB,
                          &rx_finfo),
        TAG,
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

static uint16_t uwb_dw3000_preamble_len_symbols(uint8_t preamble_len_code)
{
    switch (preamble_len_code) {
    case APP_UWB_RADIO_PLEN_32:
        return 32;
    case APP_UWB_RADIO_PLEN_64:
        return 64;
    case APP_UWB_RADIO_PLEN_72:
        return 72;
    case APP_UWB_RADIO_PLEN_128:
        return 128;
    case APP_UWB_RADIO_PLEN_256:
        return 256;
    case APP_UWB_RADIO_PLEN_512:
        return 512;
    case APP_UWB_RADIO_PLEN_1024:
        return 1024;
    case APP_UWB_RADIO_PLEN_1536:
        return 1536;
    case APP_UWB_RADIO_PLEN_2048:
        return 2048;
    default:
        return 0;
    }
}

static esp_err_t uwb_dw3000_validate_radio_profile(void)
{
    if (APP_UWB_RADIO_RF_CHANNEL_BIT > 1 ||
        APP_UWB_RADIO_SFD_TYPE > 3 ||
        APP_UWB_RADIO_PREAMBLE_CODE > 31 ||
        APP_UWB_RADIO_PREAMBLE_LEN_CODE > 0x0F ||
        APP_UWB_RADIO_DATA_RATE > APP_UWB_RADIO_BR_6M8) {
        return ESP_ERR_INVALID_ARG;
    }

    if (uwb_dw3000_preamble_len_symbols(APP_UWB_RADIO_PREAMBLE_LEN_CODE) ==
        0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static const char *uwb_dw3000_radio_data_rate_name(uint8_t data_rate)
{
    switch (data_rate) {
    case APP_UWB_RADIO_BR_850K:
        return "850k";
    case APP_UWB_RADIO_BR_6M8:
        return "6m8";
    default:
        return "unknown";
    }
}

static esp_err_t uwb_dw3000_write_sys_config(void)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_validate_radio_profile(), TAG,
                        "invalid radio profile");

    const uint8_t channel = APP_UWB_RADIO_RF_CHANNEL_BIT;
    const uint8_t preamble_len = APP_UWB_RADIO_PREAMBLE_LEN_CODE;
    const uint8_t preamble_code = APP_UWB_RADIO_PREAMBLE_CODE;
    const uint8_t pac = APP_UWB_RADIO_PAC;
    const uint8_t datarate = APP_UWB_RADIO_DATA_RATE;
    const uint8_t phr_mode = APP_UWB_RADIO_PHR_MODE;
    const uint8_t phr_rate = APP_UWB_RADIO_PHR_RATE;
    const uint8_t sfd_type = APP_UWB_RADIO_SFD_TYPE;

    ESP_LOGI(TAG,
             "DW3000 radio profile: profile=%u channel=%u rf_bit=%u plen=%u(code=0x%02x) pcode=%u pac=%u br=%s phr_mode=%u phr_rate=%u sfd=%u",
             (unsigned)APP_UWB_RADIO_PROFILE,
             (unsigned)APP_UWB_RADIO_CHANNEL, (unsigned)channel,
             (unsigned)uwb_dw3000_preamble_len_symbols(preamble_len),
             (unsigned)preamble_len, (unsigned)preamble_code, (unsigned)pac,
             uwb_dw3000_radio_data_rate_name(datarate), (unsigned)phr_mode,
             (unsigned)phr_rate, (unsigned)sfd_type);
    ESP_LOGI(TAG,
             "DW3000 RF profile: pg=0x%02x power=0x%08lx rf_tx2=0x%08lx pll=0x%04x pll_final=0x%04x",
             (unsigned)APP_UWB_RADIO_TX_PG_DELAY,
             (unsigned long)APP_UWB_RADIO_TX_POWER,
             (unsigned long)APP_UWB_RADIO_RF_TX_CTRL_2,
             (unsigned)APP_UWB_RADIO_PLL_CFG,
             (unsigned)APP_UWB_RADIO_PLL_CFG_FINAL);

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
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_sts(), TAG,
                        "STS config failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x29, 0, 1),
        TAG, "AES config write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x0C, 0xAF5F584C), TAG,
        "DRX tune write failed");

    uint32_t chan_ctrl = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_HIGH, DW3000_CHAN_CTRL_SUB,
                          &chan_ctrl),
        TAG, "CHAN_CTRL read failed");
    chan_ctrl &= ~(DW3000_CHAN_CTRL_RF_CHAN_MASK |
                   DW3000_CHAN_CTRL_SFD_TYPE_MASK |
                   DW3000_CHAN_CTRL_RX_PCODE_MASK |
                   DW3000_CHAN_CTRL_TX_PCODE_MASK);
    chan_ctrl |= channel & DW3000_CHAN_CTRL_RF_CHAN_MASK;
    chan_ctrl |= DW3000_CHAN_CTRL_RX_PCODE_MASK &
                 ((uint32_t)preamble_code
                  << DW3000_CHAN_CTRL_RX_PCODE_SHIFT);
    chan_ctrl |= DW3000_CHAN_CTRL_TX_PCODE_MASK &
                 ((uint32_t)preamble_code
                  << DW3000_CHAN_CTRL_TX_PCODE_SHIFT);
    chan_ctrl |= DW3000_CHAN_CTRL_SFD_TYPE_MASK &
                 ((uint32_t)sfd_type << DW3000_CHAN_CTRL_SFD_TYPE_SHIFT);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_HIGH,
                                 DW3000_CHAN_CTRL_SUB, chan_ctrl, 4),
        TAG, "CHAN_CTRL write failed");

    uint32_t tx_fctrl = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_TX_FCTRL_SUB,
                          &tx_fctrl),
        TAG, "TX_FCTRL read failed");
    tx_fctrl &= ~(DW3000_TX_FCTRL_TXPSR_MASK |
                  DW3000_TX_FCTRL_TXBR_MASK);
    tx_fctrl |= DW3000_TX_FCTRL_TXPSR_MASK &
                ((uint32_t)preamble_len << DW3000_TX_FCTRL_TXPSR_SHIFT);
    tx_fctrl |= DW3000_TX_FCTRL_TXBR_MASK &
                ((uint32_t)datarate << DW3000_TX_FCTRL_TXBR_SHIFT);
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_TX_FCTRL_SUB, tx_fctrl, 4),
        TAG, "TX_FCTRL write failed");

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x02, 0x81),
                        TAG, "DRX 0x02 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C,
                                  APP_UWB_RADIO_RF_TX_CTRL_2),
        TAG, "RF_TX_CTRL_2 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x00,
                                  APP_UWB_RADIO_PLL_CFG),
        TAG, "PLL_CFG write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x51,
                                                  APP_UWB_RADIO_RF_0X51),
                        TAG, "RF 0x51 write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1A,
                                                  APP_UWB_RADIO_RF_TX_CTRL_1),
                        TAG, "RF_TX_CTRL_1 write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x08,
                                                  APP_UWB_RADIO_PLL_CAL),
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
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_CIA_3, 0x00,
                                 s_antenna_delay, 2),
        TAG, "RX antenna delay write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_HIGH, 0x04,
                                 s_antenna_delay, 2),
        TAG, "TX antenna delay write failed");
#if APP_UWB_DIAGNOSTICS_ENABLED
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_cia_diagnostics(true), TAG,
                        "CIA diagnostics config failed");
#else
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_cia_diagnostics(false), TAG,
                        "CIA diagnostics config failed");
#endif
    ESP_LOGI(TAG, "DW3000 antenna delay set: rx=0x%04x tx=0x%04x",
             (unsigned)s_antenna_delay, (unsigned)s_antenna_delay);

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
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x38,
                                  APP_UWB_DGC_LUT_0), TAG,
        "DGC_LUT_0 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x3C,
                                  APP_UWB_DGC_LUT_1), TAG,
        "DGC_LUT_1 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x40,
                                  APP_UWB_DGC_LUT_2), TAG,
        "DGC_LUT_2 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x44,
                                  APP_UWB_DGC_LUT_3), TAG,
        "DGC_LUT_3 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x48,
                                  APP_UWB_DGC_LUT_4), TAG,
        "DGC_LUT_4 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x4C,
                                  APP_UWB_DGC_LUT_5), TAG,
        "DGC_LUT_5 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x50,
                                  APP_UWB_DGC_LUT_6), TAG,
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
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1A,
                                  APP_UWB_RADIO_RF_TX_CTRL_1),
        TAG, "RF_TX_CTRL_1 final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C,
                                  APP_UWB_RADIO_RF_TX_CTRL_2),
        TAG, "RF_TX_CTRL_2 final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x00,
                                  APP_UWB_RADIO_PLL_CFG_FINAL),
        TAG,
        "PLL_CFG final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x08,
                                  APP_UWB_RADIO_PLL_CAL_FINAL),
        TAG,
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
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C,
                                  APP_UWB_RADIO_TX_PG_DELAY),
        TAG,
        "TX PG delay write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_GEN_CFG_AES_HIGH, 0x0C,
                                  APP_UWB_RADIO_TX_POWER),
        TAG, "TX power write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "initial status clear failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_event_counters(), TAG,
                        "event counter setup failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_configure_device_interrupts(), TAG,
                        "DW3000 IRQ setup failed");
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
            esp_err_t clock_err = ESP_OK;
            frame->clock_offset_valid = false;
            frame->clock_offset_raw = 0;
            if (read_err == ESP_OK &&
                uwb_dw3000_payload_is_distance_frame(frame->payload,
                                                     frame->payload_len)) {
                clock_err =
                    uwb_dw3000_read_clock_offset_raw(&frame->clock_offset_raw);
                frame->clock_offset_valid = clock_err == ESP_OK;
            }
            esp_err_t diag_err = ESP_OK;
            memset(&frame->diagnostics, 0, sizeof(frame->diagnostics));
            if (read_err == ESP_OK &&
                uwb_dw3000_should_capture_rx_diagnostics(
                    frame->payload, frame->payload_len)) {
                diag_err = uwb_dw3000_read_rx_diagnostics(&frame->diagnostics);
            }
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
            if (clock_err != ESP_OK) {
                ESP_LOGD(TAG, "RX clock offset read failed: %s",
                         esp_err_to_name(clock_err));
                frame->clock_offset_valid = false;
                frame->clock_offset_raw = 0;
            }
            if (diag_err != ESP_OK) {
                ESP_LOGD(TAG, "RX diagnostics read failed: %s",
                         esp_err_to_name(diag_err));
                memset(&frame->diagnostics, 0, sizeof(frame->diagnostics));
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

        uwb_dw3000_wait_for_event_or_delay(start, timeout_ms,
                                           UWB_DW3000_POLL_INTERVAL_MS);
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

static void uwb_distance_put_i32(uint8_t *payload, size_t offset, int32_t value)
{
    const uint32_t raw = (uint32_t)value;
    payload[offset] = (uint8_t)(raw & 0xFFU);
    payload[offset + 1U] = (uint8_t)((raw >> 8U) & 0xFFU);
    payload[offset + 2U] = (uint8_t)((raw >> 16U) & 0xFFU);
    payload[offset + 3U] = (uint8_t)((raw >> 24U) & 0xFFU);
}

static int32_t uwb_distance_get_i32(const uint8_t *payload, size_t offset)
{
    const uint32_t raw = ((uint32_t)payload[offset]) |
                         ((uint32_t)payload[offset + 1U] << 8U) |
                         ((uint32_t)payload[offset + 2U] << 16U) |
                         ((uint32_t)payload[offset + 3U] << 24U);
    return (int32_t)raw;
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
    frame->clock_offset_valid = rx_frame->clock_offset_valid;
    frame->clock_offset_raw = rx_frame->clock_offset_raw;
    frame->diagnostics = rx_frame->diagnostics;
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
    case UWB_DISTANCE_FRAME_REPORT2:
        return "REPORT2";
    case UWB_DISTANCE_FRAME_SURVEY_CMD:
        return "SURVEY_CMD";
    default:
        return "UNKNOWN";
    }
}

static bool uwb_distance_destination_matches(uint8_t destination_id)
{
    return destination_id == s_source_id ||
           destination_id == UWB_DISTANCE_FRAME_BROADCAST_ID;
}

static esp_err_t uwb_distance_receive_next(struct uwb_distance_frame *frame,
                                           uint32_t timeout_ms)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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

        s_last_rx_source_id = parsed.source_id;
        s_last_rx_sequence = parsed.sequence;
        *frame = parsed;
        return ESP_OK;
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
        const bool destination_ok =
            uwb_distance_destination_matches(parsed.destination_id);
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

static double uwb_distance_tof_dtu_clock_corrected(
    uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
    uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts,
    double clock_offset_ratio)
{
    const double round_a =
        (double)uwb_distance_delta_ts(resp_rx_ts, poll_tx_ts);
    const double round_b =
        (double)uwb_distance_delta_ts(final_rx_ts, resp_tx_ts);
    const double reply_a =
        (double)uwb_distance_delta_ts(final_tx_ts, resp_rx_ts);
    const double reply_b =
        (double)uwb_distance_delta_ts(resp_tx_ts, poll_rx_ts);

    const double reply_diff = reply_a - reply_b;
    const double clock_correction = reply_a > reply_b
                                        ? (1.0 + clock_offset_ratio)
                                        : (1.0 - clock_offset_ratio);
    const double first_round_trip = round_a - reply_b;
    const double second_round_trip = round_b - reply_a;
    const double combined_round_trip =
        (first_round_trip + second_round_trip -
         (reply_diff - (reply_diff * clock_correction))) /
        2.0;

    return combined_round_trip / 2.0;
}

static double uwb_distance_tof_to_meters(double tof_dtu)
{
    return tof_dtu * UWB_DW3000_TIME_UNIT_SECONDS *
           UWB_DW3000_SPEED_OF_LIGHT_MPS;
}

static int32_t uwb_distance_meters_to_mm(double distance_m)
{
    const double mm = distance_m * 1000.0;
    if (mm > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (mm < (double)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)(mm >= 0.0 ? mm + 0.5 : mm - 0.5);
}

static uint8_t uwb_distance_peer_id(bool initiator)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->distance_test_peer_id > 0) {
        return config->distance_test_peer_id;
    }

    return initiator ? config->distance_test_responder_id
                     : config->distance_test_initiator_id;
}

static bool uwb_distance_should_log_diagnostics(uint16_t sequence)
{
#if APP_UWB_DIAGNOSTICS_ENABLED
    if (APP_UWB_DIAGNOSTICS_LOG_EVERY == 0) {
        return false;
    }
    return (sequence % APP_UWB_DIAGNOSTICS_LOG_EVERY) == 0;
#else
    (void)sequence;
    return false;
#endif
}

static void
uwb_distance_log_rx_diagnostics(uint16_t sequence, const char *label,
                                const struct uwb_rx_diagnostics *diagnostics)
{
    if (label == NULL || diagnostics == NULL) {
        return;
    }

    if (!diagnostics->valid) {
        ESP_LOGI(TAG, "DS-TWR quality seq=%u %s unavailable",
                 (unsigned)sequence, label);
        return;
    }

    ESP_LOGI(TAG,
             "DS-TWR quality seq=%u %s rx_pacc=%u fp=%.2f peak_idx=%u peak_amp=%lu power=%lu f1=%lu f2=%lu f3=%lu acc=%u xtal=%d",
             (unsigned)sequence, label, (unsigned)diagnostics->rx_pacc,
             (double)diagnostics->ipatov_fp_index / 64.0,
             (unsigned)diagnostics->ipatov_peak_index,
             (unsigned long)diagnostics->ipatov_peak_amp,
             (unsigned long)diagnostics->ipatov_power,
             (unsigned long)diagnostics->ipatov_f1,
             (unsigned long)diagnostics->ipatov_f2,
             (unsigned long)diagnostics->ipatov_f3,
             (unsigned)diagnostics->ipatov_accum_count,
             (int)diagnostics->xtal_offset);
}

static esp_err_t uwb_distance_initiate_once(uint8_t peer_id, uint16_t sequence,
                                            bool log_success)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t rx_timeout_ms = config->distance_test_rx_timeout_ms;
    const uint32_t final_delay_ms = config->distance_test_final_delay_ms;
    const uint32_t report_delay_ms = config->distance_test_report_delay_ms;
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    const uint32_t auto_rx_delay_uus =
        config->distance_test_auto_rx_delay_uus;
#endif
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uint64_t poll_tx_ts = 0;
    uint64_t resp_rx_ts = 0;
    uint64_t final_tx_ts = 0;

    uwb_distance_build_frame(UWB_DISTANCE_FRAME_POLL, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    esp_err_t err = uwb_dw3000_send_payload_expect_rx(
        payload, sizeof(payload), auto_rx_delay_uus, rx_timeout_ms,
        &poll_tx_ts);
#else
    esp_err_t err =
        uwb_dw3000_send_payload(payload, sizeof(payload), &poll_tx_ts);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR POLL TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame response = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_RESP, peer_id, true,
                                        sequence, &response, rx_timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR RESP wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }
    resp_rx_ts = response.rx_timestamp;
    if (uwb_distance_should_log_diagnostics(sequence)) {
        uwb_distance_log_rx_diagnostics(sequence, "RESP_RX",
                                        &response.diagnostics);
    }

    uwb_distance_build_frame(UWB_DISTANCE_FRAME_FINAL, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_USE_DELAYED_TX
    const uint64_t final_tx_due = uwb_dw3000_add_timestamp_delta(
        resp_rx_ts, uwb_dw3000_ms_to_dtu(final_delay_ms));
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
    uwb_dw3000_delay_ms(final_delay_ms);
    err = uwb_dw3000_send_payload(payload, sizeof(payload), &final_tx_ts);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR FINAL TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    uwb_dw3000_delay_ms(report_delay_ms);
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_REPORT, peer_id, sequence,
                             payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          resp_rx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          final_tx_ts);
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    err = uwb_dw3000_send_payload_expect_rx(payload, sizeof(payload),
                                            auto_rx_delay_uus, rx_timeout_ms,
                                            NULL);
#else
    err = uwb_dw3000_send_payload(payload, sizeof(payload), NULL);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR REPORT TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame report2 = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_REPORT2, peer_id,
                                        true, sequence, &report2,
                                        rx_timeout_ms);
    if (err == ESP_OK) {
        struct uwb_distance_measurement tag_measurement = {0};
        uwb_distance_fill_tag_measurement(peer_id, sequence, poll_tx_ts,
                                          &response, final_tx_ts, &report2,
                                          &tag_measurement);
        uwb_distance_log_tag_verification(&tag_measurement, &report2);
    } else {
        ESP_LOGW(TAG, "DS-TWR REPORT2 wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
    }

    if (log_success) {
        ESP_LOGI(TAG,
                 "DS-TWR report sent seq=%u peer=%u poll_tx=0x%010llx resp_rx=0x%010llx final_tx=0x%010llx",
                 (unsigned)sequence, (unsigned)peer_id,
                 (unsigned long long)poll_tx_ts,
                 (unsigned long long)resp_rx_ts,
                 (unsigned long long)final_tx_ts);
    }
    uwb_dw3000_maybe_log_event_counters(sequence);

    return ESP_OK;
}

static void uwb_distance_fill_measurement_from_timestamps(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
    uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts,
    bool clock_offset_valid, int32_t clock_offset_raw,
    struct uwb_distance_measurement *measurement)
{
    const double raw_tof_dtu =
        uwb_distance_tof_dtu(poll_tx_ts, poll_rx_ts, resp_tx_ts, resp_rx_ts,
                             final_tx_ts, final_rx_ts);
    const double clock_offset_ratio =
        clock_offset_valid
            ? uwb_dw3000_clock_offset_ratio(clock_offset_raw)
            : 0.0;
#if APP_UWB_DISTANCE_TEST_CLOCK_OFFSET_CORRECTION
    const double tof_dtu =
        clock_offset_valid
            ? uwb_distance_tof_dtu_clock_corrected(
                  poll_tx_ts, poll_rx_ts, resp_tx_ts, resp_rx_ts, final_tx_ts,
                  final_rx_ts, clock_offset_ratio)
            : raw_tof_dtu;
#else
    const double tof_dtu = raw_tof_dtu;
#endif

    memset(measurement, 0, sizeof(*measurement));
    measurement->initiator_id = initiator_id;
    measurement->responder_id = responder_id;
    measurement->sequence = sequence;
    measurement->tof_dtu = tof_dtu;
    measurement->distance_m = uwb_distance_tof_to_meters(tof_dtu);
    measurement->raw_tof_dtu = raw_tof_dtu;
    measurement->raw_distance_m = uwb_distance_tof_to_meters(raw_tof_dtu);
    measurement->clock_offset_valid = clock_offset_valid;
    measurement->clock_offset_raw = clock_offset_valid ? clock_offset_raw : 0;
    measurement->clock_offset_ratio = clock_offset_ratio;
    measurement->poll_tx_ts = poll_tx_ts;
    measurement->poll_rx_ts = poll_rx_ts;
    measurement->resp_tx_ts = resp_tx_ts;
    measurement->resp_rx_ts = resp_rx_ts;
    measurement->final_tx_ts = final_tx_ts;
    measurement->final_rx_ts = final_rx_ts;
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

    uwb_distance_fill_measurement_from_timestamps(
        poll->source_id, s_source_id, poll->sequence, poll_tx_ts,
        poll->rx_timestamp, resp_tx_ts, resp_rx_ts, final_tx_ts,
        final->rx_timestamp, final->clock_offset_valid,
        final->clock_offset_raw, measurement);
    measurement->poll_rx_diagnostics = poll->diagnostics;
    measurement->final_rx_diagnostics = final->diagnostics;
    measurement->report_rx_diagnostics = report->diagnostics;
}

static void uwb_distance_build_report2(
    const struct uwb_distance_measurement *measurement,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_REPORT2,
                             measurement->initiator_id,
                             measurement->sequence, payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          measurement->poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_RX_TS_OFFSET,
                          measurement->poll_rx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_TX_TS_OFFSET,
                          measurement->resp_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          measurement->resp_rx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          measurement->final_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_RX_TS_OFFSET,
                          measurement->final_rx_ts);
    uwb_distance_put_i32(payload, UWB_DISTANCE_FRAME_DISTANCE_MM_OFFSET,
                         uwb_distance_meters_to_mm(measurement->distance_m));
    uwb_distance_put_i32(payload, UWB_DISTANCE_FRAME_RAW_DISTANCE_MM_OFFSET,
                         uwb_distance_meters_to_mm(measurement->raw_distance_m));
}

static void uwb_distance_fill_tag_measurement(
    uint8_t peer_id, uint16_t sequence, uint64_t poll_tx_ts,
    const struct uwb_distance_frame *response, uint64_t final_tx_ts,
    const struct uwb_distance_frame *report2,
    struct uwb_distance_measurement *measurement)
{
    const uint64_t poll_rx_ts = uwb_distance_get_ts40(
        report2->payload, UWB_DISTANCE_FRAME_POLL_RX_TS_OFFSET);
    const uint64_t resp_tx_ts = uwb_distance_get_ts40(
        report2->payload, UWB_DISTANCE_FRAME_RESP_TX_TS_OFFSET);
    const uint64_t final_rx_ts = uwb_distance_get_ts40(
        report2->payload, UWB_DISTANCE_FRAME_FINAL_RX_TS_OFFSET);

    uwb_distance_fill_measurement_from_timestamps(
        s_source_id, peer_id, sequence, poll_tx_ts, poll_rx_ts, resp_tx_ts,
        response->rx_timestamp, final_tx_ts, final_rx_ts,
        response->clock_offset_valid, response->clock_offset_raw, measurement);
    measurement->report_rx_diagnostics = report2->diagnostics;
}

static void
uwb_distance_log_tag_verification(const struct uwb_distance_measurement *measurement,
                                  const struct uwb_distance_frame *report2)
{
    const double anchor_distance_m =
        (double)uwb_distance_get_i32(report2->payload,
                                     UWB_DISTANCE_FRAME_DISTANCE_MM_OFFSET) /
        1000.0;
    const double anchor_raw_distance_m =
        (double)uwb_distance_get_i32(report2->payload,
                                     UWB_DISTANCE_FRAME_RAW_DISTANCE_MM_OFFSET) /
        1000.0;
    const double diff_cm =
        (measurement->distance_m - anchor_distance_m) * 100.0;
    const double raw_diff_cm =
        (measurement->raw_distance_m - anchor_raw_distance_m) * 100.0;

    ESP_LOGI(TAG,
             "DS-TWR tag verify seq=%u peer=%u tag=%.3f m %.1f cm anchor=%.3f m %.1f cm diff=%.1f cm raw_tag=%.3f m raw_anchor=%.3f m raw_diff=%.1f cm clk_valid=%u",
             (unsigned)measurement->sequence,
             (unsigned)measurement->responder_id, measurement->distance_m,
             measurement->distance_m * 100.0, anchor_distance_m,
             anchor_distance_m * 100.0, diff_cm,
             measurement->raw_distance_m, anchor_raw_distance_m, raw_diff_cm,
             measurement->clock_offset_valid ? 1U : 0U);
}

static esp_err_t
uwb_distance_send_report2(const struct uwb_distance_measurement *measurement)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};

    uwb_dw3000_delay_ms(config->distance_test_report_delay_ms);
    uwb_distance_build_report2(measurement, payload);
    return uwb_dw3000_send_payload(payload, sizeof(payload), NULL);
}

static esp_err_t
uwb_distance_respond_to_poll(const struct uwb_distance_frame *poll,
                             struct uwb_distance_measurement *measurement)
{
    if (poll == NULL || measurement == NULL ||
        poll->type != UWB_DISTANCE_FRAME_POLL) {
        return ESP_ERR_INVALID_ARG;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t rx_timeout_ms = config->distance_test_rx_timeout_ms;
    const uint32_t resp_delay_ms = config->distance_test_resp_delay_ms;
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    const uint32_t auto_rx_delay_uus =
        config->distance_test_auto_rx_delay_uus;
#endif
    const uint8_t peer_id = poll->source_id;
    const uint16_t sequence = poll->sequence;
    uint64_t resp_tx_ts = 0;

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_RESP, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_USE_DELAYED_TX
    const uint64_t resp_tx_due = uwb_dw3000_add_timestamp_delta(
        poll->rx_timestamp, uwb_dw3000_ms_to_dtu(resp_delay_ms));
    uint64_t resp_tx_actual_ts = 0;
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    esp_err_t err = uwb_dw3000_send_payload_delayed_expect_rx(
        payload, sizeof(payload), resp_tx_due, auto_rx_delay_uus,
        rx_timeout_ms, &resp_tx_ts, &resp_tx_actual_ts);
#else
    esp_err_t err = uwb_dw3000_send_payload_delayed(
        payload, sizeof(payload), resp_tx_due, &resp_tx_ts,
        &resp_tx_actual_ts);
#endif
    ESP_LOGD(TAG,
             "DS-TWR RESP delayed seq=%u peer=%u due=0x%010llx programmed=0x%010llx actual=0x%010llx",
             (unsigned)sequence, (unsigned)peer_id,
             (unsigned long long)resp_tx_due,
             (unsigned long long)resp_tx_ts,
             (unsigned long long)resp_tx_actual_ts);
#else
    uwb_dw3000_delay_ms(resp_delay_ms);
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
                                        sequence, &final, rx_timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR FINAL wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame report = {0};
    err = uwb_distance_receive_matching(UWB_DISTANCE_FRAME_REPORT, peer_id, true,
                                        sequence, &report, rx_timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR REPORT wait failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    uwb_distance_fill_measurement(poll, &final, &report, resp_tx_ts,
                                  measurement);
    const esp_err_t report2_err = uwb_distance_send_report2(measurement);
    if (report2_err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR REPORT2 TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id,
                 esp_err_to_name(report2_err));
    }
    return ESP_OK;
}

static void
uwb_distance_log_measurement(const struct uwb_distance_measurement *measurement)
{
    ESP_LOGI(TAG,
             "DS-TWR distance seq=%u peer=%u distance=%.3f m %.1f cm raw=%.3f m raw_tof=%.2f dtu tof=%.2f dtu clk_valid=%u clk_raw=%ld clk_ratio=%.3e clk_corr=%u",
             (unsigned)measurement->sequence,
             (unsigned)measurement->initiator_id, measurement->distance_m,
             measurement->distance_m * 100.0, measurement->raw_distance_m,
             measurement->raw_tof_dtu, measurement->tof_dtu,
             measurement->clock_offset_valid ? 1U : 0U,
             (long)measurement->clock_offset_raw,
             measurement->clock_offset_ratio,
             (unsigned)APP_UWB_DISTANCE_TEST_CLOCK_OFFSET_CORRECTION);
    ESP_LOGD(TAG,
             "DS-TWR timestamps seq=%u poll_tx=0x%010llx poll_rx=0x%010llx resp_tx=0x%010llx resp_rx=0x%010llx final_tx=0x%010llx final_rx=0x%010llx",
             (unsigned)measurement->sequence,
             (unsigned long long)measurement->poll_tx_ts,
             (unsigned long long)measurement->poll_rx_ts,
             (unsigned long long)measurement->resp_tx_ts,
             (unsigned long long)measurement->resp_rx_ts,
             (unsigned long long)measurement->final_tx_ts,
             (unsigned long long)measurement->final_rx_ts);
    if (uwb_distance_should_log_diagnostics(measurement->sequence)) {
        uwb_distance_log_rx_diagnostics(measurement->sequence, "POLL_RX",
                                        &measurement->poll_rx_diagnostics);
        uwb_distance_log_rx_diagnostics(measurement->sequence, "FINAL_RX",
                                        &measurement->final_rx_diagnostics);
    }
    uwb_dw3000_maybe_log_event_counters(measurement->sequence);
}

static bool uwb_distance_is_initiator(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (s_source_id == config->distance_test_initiator_id) {
        return true;
    }
    if (s_source_id == config->distance_test_responder_id) {
        return false;
    }

    if (config->distance_test_peer_id > 0) {
        return s_source_id < config->distance_test_peer_id;
    }

    return (s_source_id & 1U) != 0;
}

static void uwb_distance_initiator_loop(uint8_t peer_id)
{
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);
    const app_runtime_config_t *config = app_runtime_config_get();

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "DS-TWR distance test active as initiator: source_id=%u peer_id=%u interval=%u ms timeout=%u ms auto_rx=%u delay=%u uus",
             (unsigned)s_source_id, (unsigned)peer_id,
             (unsigned)config->distance_test_interval_ms,
             (unsigned)config->distance_test_rx_timeout_ms,
             (unsigned)APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX,
             (unsigned)config->distance_test_auto_rx_delay_uus);

    while (true) {
        (void)uwb_distance_initiate_once(peer_id, sequence, true);
        sequence++;
        uwb_dw3000_delay_ms(
            app_runtime_config_get()->distance_test_interval_ms);
    }
}

static void uwb_distance_responder_loop(uint8_t peer_id)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "DS-TWR distance test active as responder: source_id=%u peer_id=%u timeout=%u ms auto_rx=%u delay=%u uus",
             (unsigned)s_source_id, (unsigned)peer_id,
             (unsigned)config->distance_test_rx_timeout_ms,
             (unsigned)APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX,
             (unsigned)config->distance_test_auto_rx_delay_uus);

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
    const app_runtime_config_t *config = app_runtime_config_get();

    if (peer_id == 0 || peer_id == s_source_id) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG,
                 "Invalid DS-TWR IDs: source_id=%u peer_id=%u initiator_id=%u responder_id=%u",
                 (unsigned)s_source_id, (unsigned)peer_id,
                 (unsigned)config->distance_test_initiator_id,
                 (unsigned)config->distance_test_responder_id);
        vTaskDelete(NULL);
        return;
    }

    if (initiator) {
        uwb_distance_initiator_loop(peer_id);
    } else {
        uwb_distance_responder_loop(peer_id);
    }
}

static size_t uwb_anchor_survey_anchor_ids(
    uint8_t ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS])
{
    const size_t count = app_runtime_config_get_anchor_ids(ids);
    if (count > UWB_ANCHOR_SURVEY_MAX_ANCHORS) {
        return UWB_ANCHOR_SURVEY_MAX_ANCHORS;
    }
    return count;
}

static bool uwb_anchor_survey_id_in_set(const uint8_t *ids, size_t count,
                                        uint8_t id)
{
    for (size_t i = 0; i < count; ++i) {
        if (ids[i] == id) {
            return true;
        }
    }
    return false;
}

static bool uwb_anchor_survey_ids_valid(const uint8_t *ids, size_t count,
                                        uint8_t coordinator_id)
{
    if (count < 2 || count > UWB_ANCHOR_SURVEY_MAX_ANCHORS ||
        coordinator_id == 0) {
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        if (ids[i] == 0) {
            return false;
        }
        for (size_t j = i + 1U; j < count; ++j) {
            if (ids[i] == ids[j]) {
                return false;
            }
        }
    }

    return true;
}

static size_t uwb_anchor_survey_build_pairs(
    const uint8_t *ids, size_t count,
    struct uwb_anchor_survey_pair pairs[UWB_ANCHOR_SURVEY_MAX_PAIRS])
{
    size_t pair_count = 0;
    for (size_t i = 0; i < count; ++i) {
        for (size_t j = i + 1U; j < count; ++j) {
            if (pair_count < UWB_ANCHOR_SURVEY_MAX_PAIRS) {
                pairs[pair_count].initiator_id = ids[i];
                pairs[pair_count].responder_id = ids[j];
                pair_count++;
            }
        }
    }
    return pair_count;
}

static void uwb_anchor_survey_build_command(
    const struct uwb_anchor_survey_pair *pair, uint8_t slot_index,
    uint16_t sequence, uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_SURVEY_CMD,
                             pair->initiator_id, sequence, payload);
    payload[UWB_ANCHOR_SURVEY_CMD_INITIATOR_OFFSET] = pair->initiator_id;
    payload[UWB_ANCHOR_SURVEY_CMD_RESPONDER_OFFSET] = pair->responder_id;
    payload[UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET] = slot_index;
}

static bool uwb_anchor_survey_parse_command(
    const struct uwb_distance_frame *frame,
    struct uwb_anchor_survey_pair *pair, uint8_t *slot_index)
{
    if (frame == NULL || pair == NULL || slot_index == NULL ||
        frame->type != UWB_DISTANCE_FRAME_SURVEY_CMD ||
        frame->payload_len <= UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET) {
        return false;
    }

    pair->initiator_id =
        frame->payload[UWB_ANCHOR_SURVEY_CMD_INITIATOR_OFFSET];
    pair->responder_id =
        frame->payload[UWB_ANCHOR_SURVEY_CMD_RESPONDER_OFFSET];
    *slot_index = frame->payload[UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET];
    return pair->initiator_id != 0 && pair->responder_id != 0 &&
           pair->initiator_id != pair->responder_id;
}

static void uwb_anchor_survey_log_measurement(
    const struct uwb_distance_measurement *measurement)
{
    if (measurement == NULL) {
        return;
    }

    ESP_LOGI(TAG,
             "ANCHOR_SURVEY result pair=%u-%u seq=%u distance=%.3f m %.1f cm raw=%.3f m clk_valid=%u",
             (unsigned)measurement->initiator_id,
             (unsigned)measurement->responder_id,
             (unsigned)measurement->sequence,
             measurement->distance_m,
             measurement->distance_m * 100.0,
             measurement->raw_distance_m,
             measurement->clock_offset_valid ? 1U : 0U);
}

static esp_err_t uwb_anchor_survey_send_command(
    const struct uwb_anchor_survey_pair *pair, uint8_t slot_index,
    uint16_t sequence)
{
    if (pair == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_anchor_survey_build_command(pair, slot_index, sequence, payload);

    const esp_err_t err = uwb_dw3000_send_payload(payload, sizeof(payload),
                                                  NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "ANCHOR_SURVEY command TX failed slot=%u seq=%u pair=%u-%u: %s",
                 (unsigned)slot_index, (unsigned)sequence,
                 (unsigned)pair->initiator_id,
                 (unsigned)pair->responder_id,
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ANCHOR_SURVEY command slot=%u seq=%u initiator=%u responder=%u",
             (unsigned)slot_index, (unsigned)sequence,
             (unsigned)pair->initiator_id, (unsigned)pair->responder_id);
    return ESP_OK;
}

static void uwb_anchor_survey_handle_poll(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count)
{
    if (frame == NULL || frame->type != UWB_DISTANCE_FRAME_POLL ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return;
    }

    if (!uwb_anchor_survey_id_in_set(anchor_ids, anchor_count,
                                     frame->source_id)) {
        ESP_LOGD(TAG, "ANCHOR_SURVEY ignoring POLL from non-anchor src=%u",
                 (unsigned)frame->source_id);
        return;
    }

    struct uwb_distance_measurement measurement = {0};
    const esp_err_t err = uwb_distance_respond_to_poll(frame, &measurement);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ANCHOR_SURVEY respond failed src=%u seq=%u: %s",
                 (unsigned)frame->source_id, (unsigned)frame->sequence,
                 esp_err_to_name(err));
        return;
    }

    uwb_distance_log_measurement(&measurement);
    uwb_anchor_survey_log_measurement(&measurement);
}

static void uwb_anchor_survey_handle_command(
    const struct uwb_distance_frame *frame, uint8_t coordinator_id,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (frame == NULL || frame->type != UWB_DISTANCE_FRAME_SURVEY_CMD) {
        return;
    }

    if (frame->source_id != coordinator_id ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return;
    }

    struct uwb_anchor_survey_pair pair = {0};
    uint8_t slot_index = 0;
    if (!uwb_anchor_survey_parse_command(frame, &pair, &slot_index)) {
        ESP_LOGW(TAG, "ANCHOR_SURVEY invalid command seq=%u",
                 (unsigned)frame->sequence);
        return;
    }

    if (pair.initiator_id != s_source_id ||
        !uwb_anchor_survey_id_in_set(anchor_ids, anchor_count,
                                     pair.responder_id)) {
        return;
    }

    ESP_LOGI(TAG,
             "ANCHOR_SURVEY command accepted slot=%u seq=%u peer=%u delay=%u ms",
             (unsigned)slot_index, (unsigned)frame->sequence,
             (unsigned)pair.responder_id,
             (unsigned)app_runtime_config_get()
                 ->anchor_survey_command_delay_ms);
    uwb_dw3000_delay_ms(
        app_runtime_config_get()->anchor_survey_command_delay_ms);
    const esp_err_t err = uwb_distance_initiate_once(
        pair.responder_id, frame->sequence, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ANCHOR_SURVEY initiated pair=%u-%u seq=%u failed: %s",
                 (unsigned)pair.initiator_id, (unsigned)pair.responder_id,
                 (unsigned)frame->sequence, esp_err_to_name(err));
    }
}

static void uwb_anchor_survey_process_frame(
    const struct uwb_distance_frame *frame, uint8_t coordinator_id,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (frame == NULL) {
        return;
    }

    switch (frame->type) {
    case UWB_DISTANCE_FRAME_POLL:
        uwb_anchor_survey_handle_poll(frame, anchor_ids, anchor_count);
        break;
    case UWB_DISTANCE_FRAME_SURVEY_CMD:
        uwb_anchor_survey_handle_command(frame, coordinator_id, anchor_ids,
                                         anchor_count);
        break;
    default:
        break;
    }
}

static void uwb_anchor_survey_listen_until(TickType_t end_tick,
                                           uint8_t coordinator_id,
                                           const uint8_t *anchor_ids,
                                           size_t anchor_count)
{
    while ((int32_t)(xTaskGetTickCount() - end_tick) < 0) {
        const TickType_t now = xTaskGetTickCount();
        const uint32_t remaining_ms =
            (uint32_t)(end_tick - now) * portTICK_PERIOD_MS;
        uint32_t slice_ms =
            app_runtime_config_get()->anchor_survey_rx_slice_ms;
        if (remaining_ms < slice_ms) {
            slice_ms = remaining_ms;
        }
        if (slice_ms == 0) {
            break;
        }

        struct uwb_distance_frame frame = {0};
        const esp_err_t err = uwb_distance_receive_next(&frame, slice_ms);
        if (err == ESP_OK) {
            uwb_anchor_survey_process_frame(&frame, coordinator_id,
                                            anchor_ids, anchor_count);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ANCHOR_SURVEY listen failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
        }
    }
}

static void uwb_anchor_survey_passive_tag_loop(uint8_t coordinator_id)
{
    uint32_t frame_count = 0;
    const app_runtime_config_t *config = app_runtime_config_get();
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "ANCHOR_SURVEY passive tag active: source_id=%u coordinator=%u rx_slice=%u ms log_every=%u",
             (unsigned)s_source_id, (unsigned)coordinator_id,
             (unsigned)config->anchor_survey_rx_slice_ms,
             (unsigned)config->anchor_survey_passive_tag_log_every);

    while (true) {
        config = app_runtime_config_get();
        struct uwb_distance_frame frame = {0};
        const esp_err_t err =
            uwb_distance_receive_next(
                &frame, config->anchor_survey_rx_slice_ms);
        if (err == ESP_OK) {
            frame_count++;
            if (config->anchor_survey_passive_tag_log_every > 0 &&
                (frame_count %
                 config->anchor_survey_passive_tag_log_every) ==
                    1U) {
                ESP_LOGI(TAG,
                         "ANCHOR_SURVEY passive frame type=%s src=%u dst=%u seq=%u total=%lu",
                         uwb_distance_type_name(frame.type),
                         (unsigned)frame.source_id,
                         (unsigned)frame.destination_id,
                         (unsigned)frame.sequence,
                         (unsigned long)frame_count);
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ANCHOR_SURVEY passive RX failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
        }
    }
}

static void uwb_anchor_survey_anchor_loop(uint8_t coordinator_id,
                                          const uint8_t *anchor_ids,
                                          size_t anchor_count)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    s_status = UWB_DW3000_STATUS_READY;

    if (s_source_id != coordinator_id) {
        ESP_LOGI(TAG,
                 "ANCHOR_SURVEY anchor follower active: source_id=%u coordinator=%u rx_slice=%u ms",
                 (unsigned)s_source_id, (unsigned)coordinator_id,
                 (unsigned)config->anchor_survey_rx_slice_ms);
        while (true) {
            config = app_runtime_config_get();
            struct uwb_distance_frame frame = {0};
            const esp_err_t err = uwb_distance_receive_next(
                &frame, config->anchor_survey_rx_slice_ms);
            if (err == ESP_OK) {
                uwb_anchor_survey_process_frame(&frame, coordinator_id,
                                                anchor_ids, anchor_count);
            } else if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "ANCHOR_SURVEY follower RX failed: %s",
                         esp_err_to_name(err));
                uwb_dw3000_delay_ms(20);
            }
        }
    }

    struct uwb_anchor_survey_pair pairs[UWB_ANCHOR_SURVEY_MAX_PAIRS] = {0};
    const size_t pair_count =
        uwb_anchor_survey_build_pairs(anchor_ids, anchor_count, pairs);
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);
    uint32_t round = 0;

    ESP_LOGI(TAG,
             "ANCHOR_SURVEY coordinator active: source_id=%u pair_count=%u slot=%u ms round_gap=%u ms",
             (unsigned)s_source_id, (unsigned)pair_count,
             (unsigned)config->anchor_survey_slot_ms,
             (unsigned)config->anchor_survey_round_gap_ms);

    while (true) {
        ESP_LOGI(TAG, "ANCHOR_SURVEY round=%lu start",
                 (unsigned long)round);
        for (size_t i = 0; i < pair_count; ++i) {
            config = app_runtime_config_get();
            const struct uwb_anchor_survey_pair *pair = &pairs[i];
            const TickType_t slot_end =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(config->anchor_survey_slot_ms);

            if (pair->initiator_id == s_source_id) {
                ESP_LOGI(TAG,
                         "ANCHOR_SURVEY local slot=%u seq=%u pair=%u-%u",
                         (unsigned)i, (unsigned)sequence,
                         (unsigned)pair->initiator_id,
                         (unsigned)pair->responder_id);
                const esp_err_t err = uwb_distance_initiate_once(
                    pair->responder_id, sequence, false);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG,
                             "ANCHOR_SURVEY local pair=%u-%u seq=%u failed: %s",
                             (unsigned)pair->initiator_id,
                             (unsigned)pair->responder_id,
                             (unsigned)sequence, esp_err_to_name(err));
                }
            } else {
                (void)uwb_anchor_survey_send_command(pair, (uint8_t)i,
                                                     sequence);
            }

            sequence++;
            uwb_anchor_survey_listen_until(slot_end, coordinator_id,
                                           anchor_ids, anchor_count);
        }

        round++;
        ESP_LOGI(TAG, "ANCHOR_SURVEY round=%lu complete",
                 (unsigned long)(round - 1UL));
        uwb_dw3000_delay_ms(
            app_runtime_config_get()->anchor_survey_round_gap_ms);
    }
}

static void uwb_dw3000_anchor_survey_loop(void)
{
    uint8_t anchor_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS] = {0};
    const size_t anchor_count = uwb_anchor_survey_anchor_ids(anchor_ids);
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t coordinator_id = config->anchor_survey_coordinator_id;
    const uint8_t tag_id = config->tag_id;

    ESP_LOGI(TAG,
             "ANCHOR_SURVEY runtime start: source_id=%u tag_id=%u coordinator=%u anchors=[%u,%u,%u,%u]",
             (unsigned)s_source_id, (unsigned)tag_id, (unsigned)coordinator_id,
             (unsigned)anchor_ids[0], (unsigned)anchor_ids[1],
             (unsigned)anchor_ids[2], (unsigned)anchor_ids[3]);

    if (!uwb_anchor_survey_ids_valid(anchor_ids, anchor_count,
                                     coordinator_id)) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "ANCHOR_SURVEY invalid anchor configuration");
        vTaskDelete(NULL);
        return;
    }

    if (s_source_id == coordinator_id) {
        uwb_anchor_survey_anchor_loop(coordinator_id, anchor_ids, anchor_count);
        return;
    }

    if (s_source_id == tag_id) {
        uwb_anchor_survey_passive_tag_loop(coordinator_id);
        return;
    }

    if (!uwb_anchor_survey_id_in_set(anchor_ids, anchor_count, s_source_id)) {
        s_status = UWB_DW3000_STATUS_READY;
        ESP_LOGW(TAG,
                 "ANCHOR_SURVEY idle: source_id=%u is neither tag nor configured anchor",
                 (unsigned)s_source_id);
        while (true) {
            uwb_dw3000_delay_ms(1000);
        }
    }

    uwb_anchor_survey_anchor_loop(coordinator_id, anchor_ids, anchor_count);
}

static void uwb_ranging_tag_loop(const uint8_t *anchor_ids, size_t anchor_count)
{
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);
    uint32_t round = 0;
    const app_runtime_config_t *config = app_runtime_config_get();

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB_RANGING tag active: source_id=%u anchors=[%u,%u,%u,%u] slot=%u ms round_gap=%u ms",
             (unsigned)s_source_id, (unsigned)anchor_ids[0],
             (unsigned)anchor_ids[1], (unsigned)anchor_ids[2],
             (unsigned)anchor_ids[3], (unsigned)config->ranging_slot_ms,
             (unsigned)config->ranging_round_gap_ms);

    while (true) {
        ESP_LOGI(TAG, "UWB_RANGING round=%lu start", (unsigned long)round);
        for (size_t i = 0; i < anchor_count; ++i) {
            const uint8_t anchor_id = anchor_ids[i];
            ESP_LOGI(TAG, "UWB_RANGING tag poll anchor=%u seq=%u",
                     (unsigned)anchor_id, (unsigned)sequence);
            const esp_err_t err =
                uwb_distance_initiate_once(anchor_id, sequence, false);
            if (err != ESP_OK) {
                ESP_LOGW(TAG,
                         "UWB_RANGING tag anchor=%u seq=%u failed: %s",
                         (unsigned)anchor_id, (unsigned)sequence,
                         esp_err_to_name(err));
            }
            sequence++;
            uwb_dw3000_delay_ms(
                app_runtime_config_get()->ranging_slot_ms);
        }
        round++;
        ESP_LOGI(TAG, "UWB_RANGING round=%lu complete",
                 (unsigned long)(round - 1UL));
        uwb_dw3000_delay_ms(
            app_runtime_config_get()->ranging_round_gap_ms);
    }
}

static void uwb_ranging_anchor_loop(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB_RANGING anchor active: source_id=%u tag_id=%u rx_slice=%u ms",
             (unsigned)s_source_id, (unsigned)config->tag_id,
             (unsigned)config->ranging_rx_slice_ms);

    while (true) {
        config = app_runtime_config_get();
        struct uwb_distance_frame frame = {0};
        const esp_err_t rx_err =
            uwb_distance_receive_next(&frame, config->ranging_rx_slice_ms);
        if (rx_err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (rx_err != ESP_OK) {
            ESP_LOGW(TAG, "UWB_RANGING anchor RX failed: %s",
                     esp_err_to_name(rx_err));
            uwb_dw3000_delay_ms(20);
            continue;
        }
        if (frame.type != UWB_DISTANCE_FRAME_POLL ||
            frame.source_id != config->tag_id ||
            !uwb_distance_destination_matches(frame.destination_id)) {
            continue;
        }

        struct uwb_distance_measurement measurement = {0};
        const esp_err_t err =
            uwb_distance_respond_to_poll(&frame, &measurement);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "UWB_RANGING anchor respond failed tag=%u seq=%u: %s",
                     (unsigned)frame.source_id, (unsigned)frame.sequence,
                     esp_err_to_name(err));
            continue;
        }

        uwb_distance_log_measurement(&measurement);
        ESP_LOGI(TAG,
                 "UWB_RANGING result tag=%u anchor=%u seq=%u distance=%.3f m %.1f cm raw=%.3f m clk_valid=%u",
                 (unsigned)measurement.initiator_id, (unsigned)s_source_id,
                 (unsigned)measurement.sequence, measurement.distance_m,
                 measurement.distance_m * 100.0, measurement.raw_distance_m,
                 measurement.clock_offset_valid ? 1U : 0U);
    }
}

static void uwb_dw3000_ranging_loop(void)
{
    uint8_t anchor_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS] = {0};
    const size_t anchor_count = uwb_anchor_survey_anchor_ids(anchor_ids);
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t tag_id = config->tag_id;

    ESP_LOGI(TAG,
             "UWB_RANGING runtime start: source_id=%u tag_id=%u anchors=[%u,%u,%u,%u]",
             (unsigned)s_source_id, (unsigned)tag_id,
             (unsigned)anchor_ids[0], (unsigned)anchor_ids[1],
             (unsigned)anchor_ids[2], (unsigned)anchor_ids[3]);

    if (anchor_count == 0 || anchor_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "UWB_RANGING invalid anchor configuration");
        vTaskDelete(NULL);
        return;
    }

    if (s_source_id == tag_id) {
        uwb_ranging_tag_loop(anchor_ids, anchor_count);
        return;
    }

    if (uwb_anchor_survey_id_in_set(anchor_ids, anchor_count, s_source_id)) {
        uwb_ranging_anchor_loop();
        return;
    }

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGW(TAG,
             "UWB_RANGING idle: source_id=%u is neither tag nor configured anchor",
             (unsigned)s_source_id);
    while (true) {
        uwb_dw3000_delay_ms(1000);
    }
}

static const char *uwb_calibration_method_name(uint8_t method)
{
    switch (method) {
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
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t id0 = config->calibration_three_ids[0];
    const uint8_t id1 = config->calibration_three_ids[1];
    const uint8_t id2 = config->calibration_three_ids[2];
    const uint32_t fallback_mm = config->calibration_known_distance_mm;
    uint32_t distance_mm = fallback_mm;

    if ((first_id == id0 && second_id == id1) ||
        (first_id == id1 && second_id == id0)) {
        distance_mm = config->calibration_three_distance_0_1_mm;
    } else if ((first_id == id0 && second_id == id2) ||
               (first_id == id2 && second_id == id0)) {
        distance_mm = config->calibration_three_distance_0_2_mm;
    } else if ((first_id == id1 && second_id == id2) ||
               (first_id == id2 && second_id == id1)) {
        distance_mm = config->calibration_three_distance_1_2_mm;
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
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t min_ms = config->calibration_min_interval_ms;
    const uint32_t max_ms = config->calibration_max_interval_ms;
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
    const app_runtime_config_t *config = app_runtime_config_get();
    if (samples == 0) {
        return false;
    }
    if (samples == 1 || samples == config->calibration_sample_count) {
        return true;
    }
    if (config->calibration_summary_every == 0) {
        return false;
    }
    return (samples % config->calibration_summary_every) == 0;
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
        (int32_t)s_antenna_delay + suggested_delta;
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
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t reference_id = config->calibration_reference_id;
    const uint8_t dut_id = config->calibration_dut_id;
    struct uwb_calibration_stats stats = {0};
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB CAL two-module active: source_id=%u reference_id=%u dut_id=%u known=%u mm delay=0x%04x",
             (unsigned)s_source_id, (unsigned)reference_id,
             (unsigned)dut_id,
             (unsigned)config->calibration_known_distance_mm,
             (unsigned)s_antenna_delay);

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

static bool uwb_calibration_id_in_three_set(uint8_t id, const uint8_t ids[3])
{
    return ids != NULL &&
           (id == ids[0] || id == ids[1] || id == ids[2]);
}

static int uwb_calibration_pair_index(uint8_t initiator_id,
                                      uint8_t responder_id,
                                      const uint8_t ids[3])
{
    if (ids == NULL) {
        return -1;
    }

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
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t ids[3] = {
        config->calibration_three_ids[0],
        config->calibration_three_ids[1],
        config->calibration_three_ids[2],
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
             (unsigned)config->calibration_known_distance_mm,
             (unsigned)s_antenna_delay);

    if (ids[0] == 0 || ids[1] == 0 || ids[2] == 0 || ids[0] == ids[1] ||
        ids[0] == ids[2] || ids[1] == ids[2]) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Invalid three-module calibration IDs");
        vTaskDelete(NULL);
        return;
    }

    if (!uwb_calibration_id_in_three_set(s_source_id, ids)) {
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
            app_runtime_config_get()->calibration_rx_slice_ms);
        if (err == ESP_OK) {
            if (!uwb_calibration_id_in_three_set(poll.source_id, ids)) {
                ESP_LOGD(TAG,
                         "Ignoring calibration poll from non-set source=%u",
                         (unsigned)poll.source_id);
                continue;
            }

            struct uwb_distance_measurement measurement = {0};
            err = uwb_distance_respond_to_poll(&poll, &measurement);
            if (err == ESP_OK) {
                const int pair_index = uwb_calibration_pair_index(
                    measurement.initiator_id, measurement.responder_id, ids);
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
    const uint8_t method = app_runtime_config_get()->calibration_method;
    ESP_LOGI(TAG, "UWB calibration runtime start: method=%s(%u)",
             uwb_calibration_method_name(method), (unsigned)method);

    switch (method) {
    case APP_UWB_CALIBRATION_METHOD_TWO_MODULE:
        uwb_calibration_two_module_loop();
        break;
    case APP_UWB_CALIBRATION_METHOD_THREE_MODULE:
        uwb_calibration_three_module_loop();
        break;
    default:
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Unsupported UWB calibration method: %u",
                 (unsigned)method);
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

        uwb_dw3000_wait_for_event_or_delay(start, UWB_DW3000_TX_TIMEOUT_MS,
                                           UWB_DW3000_TX_POLL_MS);
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

static esp_err_t uwb_dw3000_send_payload_expect_rx(
    const uint8_t *payload, size_t payload_len, uint32_t rx_after_tx_delay_uus,
    uint32_t rx_timeout_ms, uint64_t *tx_timestamp)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_prepare_tx(payload, payload_len), TAG,
                        "TX/RX prepare failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_prepare_rx_after_tx(rx_after_tx_delay_uus, rx_timeout_ms),
        TAG, "TX/RX auto-RX config failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TX_W4R), TAG,
                        "TX_W4R command failed");

    const esp_err_t err = uwb_dw3000_wait_for_tx_complete(tx_timestamp);
    if (err == ESP_OK) {
        s_rx_armed = true;
    }
    return err;
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

static esp_err_t uwb_dw3000_send_payload_delayed_expect_rx(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_prepare_tx(payload, payload_len), TAG,
                        "delayed TX/RX prepare failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_prepare_rx_after_tx(rx_after_tx_delay_uus, rx_timeout_ms),
        TAG, "delayed TX/RX auto-RX config failed");

    const uint32_t delayed_time_word =
        uwb_dw3000_delayed_time_word(tx_timestamp);
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_delayed_trx_time(delayed_time_word),
                        TAG, "DX_TIME write failed");

    if (programmed_tx_timestamp != NULL) {
        *programmed_tx_timestamp =
            uwb_dw3000_programmed_tx_timestamp(delayed_time_word);
    }

    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_DTX_W4R), TAG,
                        "delayed TX_W4R command failed");

    uint32_t status = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_STATUS_SUB,
                          &status),
        TAG, "SYS_STATUS after delayed TX/RX failed");
    if ((status & DW3000_STATUS_HPDWARN) != 0) {
        s_tx_error_count++;
        ESP_LOGW(TAG,
                 "UWB delayed TX/RX rejected timestamp=0x%010llx word=0x%08lx SYS_STATUS=0x%08lx",
                 (unsigned long long)tx_timestamp,
                 (unsigned long)delayed_time_word, (unsigned long)status);
        ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                            "TXRXOFF after delayed TX/RX reject failed");
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear after delayed TX/RX reject failed");
        s_rx_armed = false;
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t err =
        uwb_dw3000_wait_for_tx_complete(actual_tx_timestamp);
    if (err == ESP_OK) {
        s_rx_armed = true;
    }
    return err;
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
    s_antenna_delay = app_identity_get_uwb_antenna_delay();
    s_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG,
             "Starting DW3000 bring-up: CS=%d SCK=%d MISO=%d MOSI=%d RST=%d IRQ=%d WAKEUP=%d",
             BOARD_CONFIG_UWB_CS_GPIO, BOARD_CONFIG_SPI_SCK_GPIO,
             BOARD_CONFIG_SPI_MISO_GPIO, BOARD_CONFIG_SPI_MOSI_GPIO,
             BOARD_CONFIG_UWB_RST_GPIO, BOARD_CONFIG_UWB_IRQ_GPIO,
             BOARD_CONFIG_UWB_WAKEUP_GPIO);
    ESP_LOGI(TAG, "DW3000 GPIO polarity: reset active-%s, wakeup active-%s",
             BOARD_CONFIG_UWB_RST_ACTIVE_HIGH ? "HIGH" : "LOW",
             BOARD_CONFIG_UWB_WAKEUP_ACTIVE_HIGH ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "DW3000 active antenna delay: 0x%04x (%s)",
             (unsigned)s_antenna_delay,
             app_identity_uwb_antenna_delay_from_nvs() ? "nvs" : "fallback");

    esp_err_t err = uwb_dw3000_configure_gpio();
    if (err == ESP_OK) {
        const esp_err_t irq_err = uwb_dw3000_configure_host_irq();
        if (irq_err != ESP_OK) {
            ESP_LOGW(TAG, "Continuing with DW3000 polling fallback");
            s_irq_enabled = false;
        }
    }
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
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_ANCHOR_SURVEY) {
        uwb_dw3000_anchor_survey_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_RANGING) {
        uwb_dw3000_ranging_loop();
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

esp_err_t uwb_dw3000_start_anchor_survey(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_ANCHOR_SURVEY);
}

esp_err_t uwb_dw3000_start_ranging(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_RANGING);
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

uint16_t uwb_dw3000_get_antenna_delay(void)
{
    if (!s_started) {
        return app_identity_get_uwb_antenna_delay();
    }

    return s_antenna_delay;
}
