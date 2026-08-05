#include "uwb_dw3000.h"

#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "app_config.h"
#include "app_identity.h"
#include "app_runtime_config.h"
#include "flextdoa_cfo_estimator.h"
#include "flextdoa_collector.h"
#include "flextdoa_protocol.h"
#include "uwb_config.h"
#include "uwb_anchor_range_cache.h"
#include "uwb_flex_tdoa_runtime.h"
#include "uwb_native_ds_runtime.h"
#include "uwb_native_ds_twr.h"
#include "uwb_passive_ds_runtime.h"
#include "uwb_passive_ds_tdoa.h"
#include "uwb_passive_ds_multi.h"
#include "wireless_log_service.h"
#include "wireless_telemetry_service.h"

static const char *TAG = "uwb_dw3000";

static StaticSemaphore_t s_native_ds_delay_semaphore_storage;
static SemaphoreHandle_t s_native_ds_delay_semaphore;
static esp_timer_handle_t s_native_ds_delay_timer;

enum {
    /* ESP-IDF expresses the task stack depth in bytes. The multipoint
     * receive-only path keeps several complete radio frames on this task's
     * stack, so the historical 8 KiB allocation is not sufficient. */
    UWB_DW3000_TASK_STACK_BYTES = 32768,
    // FlexTDOA delayed-TX deadlines are sub-millisecond. Keep this below the
    // ESP-IDF timer/Wi-Fi critical tasks, but above ordinary TCP/application
    // work; the task sleeps on the DW3000 interrupt between radio events.
    UWB_DW3000_TASK_PRIORITY = 20,
    UWB_DW3000_SPI_BOOT_CLOCK_HZ = 4 * 1000 * 1000,
    // Project default: ESP32-S3 GPSPI generates 40 MHz exactly from the
    // 80 MHz APB source. This intentionally exceeds the DW3000 datasheet
    // limit of 38 MHz; all field modules have passed repeated DEV_ID reads,
    // OTA boots and sustained FlexTDOA traffic at this clock. Keep the opt-in
    // and the explicit datasheet limit so the exception cannot be accidental.
    UWB_DW3000_SPI_ALLOW_OVERCLOCK = 1,
    UWB_DW3000_SPI_OPERATION_REQUEST_HZ = 40 * 1000 * 1000,
    UWB_DW3000_SPI_DATASHEET_MAX_HZ = 38 * 1000 * 1000,
    UWB_DW3000_SPI_VALIDATED_MAX_HZ = 40 * 1000 * 1000,
    UWB_DW3000_SPI_VERIFY_READS = 8,
    UWB_DW3000_SPI_MAX_TRANSFER_BYTES = 96,
    UWB_DW3000_RESET_SETTLE_MS = 5,
    UWB_DW3000_RESET_PULSE_MS = 20,
    UWB_DW3000_WAKE_AFTER_RESET_MS = 300,
    UWB_DW3000_PROBE_ATTEMPTS = 16,
    UWB_DW3000_PROBE_INTERVAL_MS = 40,
    UWB_DW3000_IDLE_TIMEOUT_MS = 1000,
    UWB_DW3000_POLL_INTERVAL_MS = 5,
    UWB_DW3000_TX_TIMEOUT_MS = 120,
    UWB_FLEX_TDOA_TX_TIMEOUT_MS = 10,
    UWB_DW3000_TX_POLL_MS = 2,
    UWB_DW3000_PAYLOAD_LEN = 64,
};

_Static_assert(UWB_DW3000_SPI_ALLOW_OVERCLOCK ||
                   UWB_DW3000_SPI_OPERATION_REQUEST_HZ <=
                       UWB_DW3000_SPI_DATASHEET_MAX_HZ,
               "DW3000 operational SPI request exceeds datasheet maximum");
_Static_assert(UWB_DW3000_SPI_ALLOW_OVERCLOCK ||
                   UWB_DW3000_SPI_VALIDATED_MAX_HZ <=
                       UWB_DW3000_SPI_DATASHEET_MAX_HZ,
               "DW3000 validated SPI maximum exceeds datasheet maximum");
_Static_assert(UWB_DW3000_SPI_OPERATION_REQUEST_HZ <=
                   UWB_DW3000_SPI_VALIDATED_MAX_HZ,
               "DW3000 operational SPI request exceeds validated maximum");

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
/* Qorvo ID 0x10024 encodes register file 0x01, offset 0x24. */
#define DW3000_REG_RDB_STATUS DW3000_REG_GEN_CFG_AES_HIGH
#define DW3000_REG_PMSC 0x11
#define DW3000_REG_RX_BUFFER_0 0x12
#define DW3000_REG_RX_BUFFER_1 0x13
#define DW3000_REG_TX_BUFFER 0x14
#define DW3000_REG_DOUBLE_BUFFER_DIAG 0x18
#define DW3000_REG_INDIRECT_POINTER_B 0x1E
#define DW3000_REG_INDIRECT_CTRL 0x1F

#define DW3000_SUB_NONE 0x00
#define DW3000_DEV_ID_DW3000 0xDECA0302UL
#define DW3000_DEV_ID_DW3120 0xDECA0312UL

#define DW3000_CMD_TXRXOFF 0x00
#define DW3000_CMD_TX 0x01
#define DW3000_CMD_RX 0x02
#define DW3000_CMD_DTX 0x03
#define DW3000_CMD_TX_W4R 0x0C
#define DW3000_CMD_DTX_W4R 0x0D
#define DW3000_CMD_DB_TOGGLE 0x13

#define DW3000_SYS_CFG_SUB 0x10
#define DW3000_SYS_TIME_SUB 0x1C
#define DW3000_TX_FCTRL_SUB 0x24
#define DW3000_DX_TIME_SUB 0x2C
#define DW3000_RX_FWTO_SUB 0x34
#define DW3000_SYS_ENABLE_LO_SUB 0x3C
#define DW3000_SYS_ENABLE_HI_SUB 0x40
#define DW3000_SYS_STATUS_SUB 0x44
#define DW3000_SYS_STATE_SUB 0x30
#define DW3000_RX_FINFO_SUB 0x4C
#define DW3000_RDB_STATUS_SUB 0x24
#define DW3000_INDIRECT_ADDR_B_SUB 0x0C
#define DW3000_ADDR_OFFSET_B_SUB 0x10
#define DW3000_RX_TIME_SUB 0x00
#define DW3000_ACK_RESP_SUB 0x08
#define DW3000_CHAN_CTRL_SUB 0x14
#define DW3000_DRX_CAR_INT_SUB 0x29
#define DW3000_TX_TIME_SUB 0x74
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
#define DW3000_STATUS_RX_GOOD_CLEAR_MASK 0x00006F00UL
#define DW3000_STATUS_RXFCG 0x00004000UL
#define DW3000_STATUS_RXPHE 0x00001000UL
#define DW3000_STATUS_RXFCE 0x00008000UL
#define DW3000_STATUS_RXFSL 0x00010000UL
#define DW3000_STATUS_RXFTO 0x00020000UL
#define DW3000_STATUS_CIAERR 0x00040000UL
#define DW3000_STATUS_RXPTO 0x00200000UL
#define DW3000_STATUS_RXSTO 0x04000000UL
#define DW3000_STATUS_HPDWARN 0x08000000UL
#define DW3000_SYS_STATE_DELAYED_TX_ERROR 0x000D0000UL
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
#define DW3000_SYS_CFG_RXAUTR_BIT_MASK 0x00000400UL
#define DW3000_SYS_CFG_DIS_DRXB_BIT_MASK 0x00000008UL
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
#define DW3000_CIA_DIAG_LOG_MIN 0x02U
#define DW3000_CLOCK_OFFSET_RAW_MASK 0x001FFFFFUL
#define DW3000_CLOCK_OFFSET_RAW_BITS 21U
#define DW3000_CLOCK_OFFSET_CH5_FACTOR (-0.5731e-9)
#define DW3000_CLOCK_OFFSET_CH9_FACTOR (-0.1252e-9)
#define DW3000_DOUBLE_BUFFER_0_RX_FINFO_SUB 0x00U
#define DW3000_DOUBLE_BUFFER_0_RX_TIME_SUB 0x04U
#define DW3000_DOUBLE_BUFFER_0_CIA_DIAG_0_SUB 0x0CU
#define DW3000_DOUBLE_BUFFER_1_RX_FINFO_OFFSET 0x00U
#define DW3000_DOUBLE_BUFFER_1_RX_TIME_OFFSET 0x04U
#define DW3000_DOUBLE_BUFFER_1_CIA_DIAG_0_OFFSET 0x0CU
#define DW3000_DOUBLE_BUFFER_FLEX_META_LEN 14U
#define DW3000_DOUBLE_BUFFER_1_DIAG_BASE 0x18U
#define DW3000_DOUBLE_BUFFER_1_DIAG_OFFSET 0x00E8U
#define DW3000_RDB_BUFFER_0_GOOD_MASK 0x01U
#define DW3000_RDB_BUFFER_1_GOOD_MASK 0x10U
#define DW3000_RDB_BUFFER_0_CIA_DONE_MASK 0x04U
#define DW3000_RDB_BUFFER_1_CIA_DONE_MASK 0x40U
#define DW3000_RDB_BUFFER_0_CLEAR_MASK 0x0FU
#define DW3000_RDB_BUFFER_1_CLEAR_MASK 0xF0U
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
#define UWB_DISTANCE_FRAME_FINAL_TIMESTAMPS_LEN \
    (UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET + 5U)
#define UWB_DISTANCE_FRAME_POLL_LEN UWB_DISTANCE_FRAME_HEADER_LEN
#define UWB_DISTANCE_FRAME_RESP_LEN UWB_DISTANCE_FRAME_HEADER_LEN
#define UWB_DISTANCE_FRAME_FINAL_LEN UWB_DISTANCE_FRAME_HEADER_LEN
#define UWB_DISTANCE_FRAME_REPORT_LEN UWB_DISTANCE_FRAME_FINAL_TIMESTAMPS_LEN
#define UWB_DISTANCE_FRAME_REPORT2_LEN \
    (UWB_DISTANCE_FRAME_RAW_DISTANCE_MM_OFFSET + sizeof(int32_t))
#define UWB_DISTANCE_FRAME_BROADCAST_ID 255U

/*
 * Passive DS-TWR keeps the native three-message exchange, but makes the
 * responder turnaround explicit so any receive-only tag can form a TDOA
 * observation from POLL_RX -> RESP_RX. FINAL still completes a genuine
 * anchor-to-anchor DS-TWR measurement.
 */
#define UWB_PASSIVE_DS_SLOT_ID_OFFSET UWB_DISTANCE_FRAME_HEADER_LEN
#define UWB_PASSIVE_DS_REPLY_DTU_OFFSET \
    (UWB_PASSIVE_DS_SLOT_ID_OFFSET + 4U)
#define UWB_PASSIVE_DS_POLL_PIGGYBACK_OFFSET \
    (UWB_PASSIVE_DS_SLOT_ID_OFFSET + 4U)
#define UWB_PASSIVE_DS_RESP_PIGGYBACK_OFFSET \
    (UWB_PASSIVE_DS_REPLY_DTU_OFFSET + 4U)
#define UWB_PASSIVE_DS_PIGGYBACK_PEER_OFFSET 0U
#define UWB_PASSIVE_DS_PIGGYBACK_DISTANCE_MM_OFFSET 1U
#define UWB_PASSIVE_DS_PIGGYBACK_RAW_DISTANCE_MM_OFFSET 3U
#define UWB_PASSIVE_DS_PIGGYBACK_SLOT_ID_OFFSET 5U
#define UWB_PASSIVE_DS_PIGGYBACK_SEQUENCE_OFFSET 9U
#define UWB_PASSIVE_DS_PIGGYBACK_EXCHANGE_DTU_OFFSET 11U
#define UWB_PASSIVE_DS_PIGGYBACK_LEN 15U
#define UWB_PASSIVE_DS_POLL_LEN \
    (UWB_PASSIVE_DS_POLL_PIGGYBACK_OFFSET + \
     UWB_PASSIVE_DS_PIGGYBACK_LEN)
#define UWB_PASSIVE_DS_RESP_LEN \
    (UWB_PASSIVE_DS_RESP_PIGGYBACK_OFFSET + \
     UWB_PASSIVE_DS_PIGGYBACK_LEN)
#define UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET \
    UWB_DISTANCE_FRAME_FINAL_TIMESTAMPS_LEN
#define UWB_PASSIVE_DS_FINAL_LEN \
    (UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET + 4U)
#define UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET \
    UWB_DISTANCE_FRAME_HEADER_LEN
#define UWB_PASSIVE_DS_MULTI_POLL_PIGGYBACK_OFFSET \
    (UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET + 4U)
#define UWB_PASSIVE_DS_MULTI_RESP_PIGGYBACK_OFFSET \
    (UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET + 9U)
#define UWB_PASSIVE_DS_MULTI_POLL_LEN \
    (UWB_PASSIVE_DS_MULTI_POLL_PIGGYBACK_OFFSET + \
     UWB_PASSIVE_DS_PIGGYBACK_LEN)
#define UWB_PASSIVE_DS_MULTI_RESP_LEN \
    (UWB_PASSIVE_DS_MULTI_RESP_PIGGYBACK_OFFSET + \
     UWB_PASSIVE_DS_PIGGYBACK_LEN)
#define UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US 2000000LL
#define UWB_PASSIVE_DS_FAST_GEOMETRY_FRAME_INTERVAL 4U
#define UWB_PASSIVE_DS_POLL_TX_LEAD_US 750LL
#define UWB_PASSIVE_DS_MULTI_POLL_TX_LEAD_US 2000LL
#define UWB_PASSIVE_DS_POLL_LATE_US 250LL
#define UWB_PASSIVE_DS_MULTI_RESPONSE_COLLECTION_SLACK_US 500LL
#define UWB_PASSIVE_DS_MULTI_RECOVERY_FRAMES 8LL
#define UWB_PASSIVE_DS_MULTI_RECOVERY_MIN_US 50000LL
#define UWB_ANCHOR_SURVEY_MAX_ANCHORS APP_RUNTIME_CONFIG_MAX_ANCHORS
#define UWB_ANCHOR_SURVEY_MAX_PAIRS \
    ((UWB_ANCHOR_SURVEY_MAX_ANCHORS * (UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U)) / 2U)
#define UWB_FLEX_TDOA_MAX_OBSERVATIONS \
    (UWB_ANCHOR_SURVEY_MAX_ANCHORS * (UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U))
#define UWB_ANCHOR_SURVEY_CMD_INITIATOR_OFFSET 10U
#define UWB_ANCHOR_SURVEY_CMD_RESPONDER_OFFSET 11U
#define UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET 12U
#define UWB_ANCHOR_SURVEY_CMD_LEN \
    (UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET + 1U)
#define UWB_FLEX_TDOA_CONFIG_GENERATION_OFFSET 10U
#define UWB_FLEX_TDOA_CONFIG_N_OFFSET 14U
#define UWB_FLEX_TDOA_CONFIG_K_OFFSET 15U
#define UWB_FLEX_TDOA_CONFIG_M_OFFSET 16U
#define UWB_FLEX_TDOA_CONFIG_ANCHORS_OFFSET 17U
#define UWB_FLEX_TDOA_CONFIG_SLOTS_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_ANCHORS_OFFSET + APP_RUNTIME_CONFIG_MAX_ANCHORS)
#define UWB_FLEX_TDOA_CONFIG_MASKS_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_SLOTS_OFFSET + \
     ((APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS + 1U) / 2U))
#define UWB_FLEX_TDOA_CONFIG_LEN \
    (UWB_FLEX_TDOA_CONFIG_MASKS_OFFSET + \
     ((APP_RUNTIME_CONFIG_MAX_ANCHORS * \
       APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS + 7U) / 8U) + 10U)
#define UWB_FLEX_TDOA_CONFIG_GUARD_US_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_LEN - 10U)
#define UWB_FLEX_TDOA_CONFIG_REQ_US_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_LEN - 8U)
#define UWB_FLEX_TDOA_CONFIG_REQ_PROCESS_US_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_LEN - 6U)
#define UWB_FLEX_TDOA_CONFIG_RESP_US_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_LEN - 4U)
#define UWB_FLEX_TDOA_CONFIG_RESP_PROCESS_US_OFFSET \
    (UWB_FLEX_TDOA_CONFIG_LEN - 2U)
#define UWB_FLEX_TDOA_GEOMETRY_GENERATION_OFFSET 10U
#define UWB_FLEX_TDOA_GEOMETRY_FIXED_OFFSET 14U
#define UWB_FLEX_TDOA_GEOMETRY_COUNT_OFFSET 15U
#define UWB_FLEX_TDOA_GEOMETRY_INDEX_OFFSET 16U
#define UWB_FLEX_TDOA_GEOMETRY_ANCHOR_ID_OFFSET 17U
#define UWB_FLEX_TDOA_GEOMETRY_X_MM_OFFSET 18U
#define UWB_FLEX_TDOA_GEOMETRY_Y_MM_OFFSET 22U
#define UWB_FLEX_TDOA_GEOMETRY_LEN 26U
#define UWB_FLEX_TDOA_CONFIG_PHASE_US 3000000LL
#define UWB_FLEX_TDOA_CONFIG_TX_SPACING_US 30000LL
#define UWB_FLEX_TDOA_CONFIG_FRAME_GAP_MS 10U
#define UWB_FLEX_TDOA_BOOTSTRAP_LISTEN_US 2000000LL
#define UWB_HOT_SWITCH_BOOTSTRAP_GUARD_US 150000LL
// Host-only preparation lead. It must be shorter than one 4.80 ms slot so an
// initiator keeps receiving the preceding slot while still programming its
// own DW3000 delayed TX with comfortable margin.
#define UWB_FLEX_TDOA_REQUEST_TX_LEAD_US 1750LL
#define UWB_FLEX_TDOA_REQUEST_LATE_US 500LL
#define UWB_FLEX_TDOA_RX_EPOCH_MAX_GAP_US 30000LL

enum uwb_distance_frame_type {
    UWB_DISTANCE_FRAME_POLL = 1,
    UWB_DISTANCE_FRAME_RESP = 2,
    UWB_DISTANCE_FRAME_FINAL = 3,
    UWB_DISTANCE_FRAME_REPORT = 4,
    UWB_DISTANCE_FRAME_SURVEY_CMD = 5,
    UWB_DISTANCE_FRAME_REPORT2 = 6,
    UWB_DISTANCE_FRAME_CAL_CMD = 7,
    UWB_DISTANCE_FRAME_CAL_SYNC = 8,
    UWB_DISTANCE_FRAME_FLEX_TDOA_REQ = 11,
    UWB_DISTANCE_FRAME_FLEX_TDOA_RESP = 12,
    UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG = 13,
    UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY = 14,
    UWB_DISTANCE_FRAME_PASSIVE_DS_POLL = 15,
    UWB_DISTANCE_FRAME_PASSIVE_DS_RESP = 16,
    UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL = 17,
    UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL = 18,
    UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP = 19,
    UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL = 20,
};

enum uwb_dw3000_runtime_mode {
    UWB_DW3000_RUNTIME_BEACON_SMOKE = 0,
    UWB_DW3000_RUNTIME_DISTANCE_TEST,
    UWB_DW3000_RUNTIME_CALIBRATION,
    UWB_DW3000_RUNTIME_ANCHOR_SURVEY,
    UWB_DW3000_RUNTIME_RANGING,
    UWB_DW3000_RUNTIME_FLEX_TDOA,
    UWB_DW3000_RUNTIME_PASSIVE_DS_TWR,
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
    int64_t rx_host_time_us;
    uint8_t rx_buffer_index;
    uint8_t rx_buffer_status;
    bool clock_offset_valid;
    bool clock_offset_from_cia;
    int32_t clock_offset_raw;
    struct uwb_rx_diagnostics diagnostics;
};

struct uwb_distance_frame {
    uint8_t type;
    uint8_t source_id;
    uint8_t destination_id;
    uint16_t sequence;
    uint64_t rx_timestamp;
    int64_t rx_host_time_us;
    uint8_t rx_buffer_index;
    uint8_t rx_buffer_status;
    bool clock_offset_valid;
    bool clock_offset_from_cia;
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

struct uwb_anchor_survey_pair {
    uint8_t initiator_id;
    uint8_t responder_id;
};

struct uwb_flex_tdoa_observation {
    bool in_use;
    uint16_t sequence;
    uint32_t slot_id;
    uint8_t initiator_id;
    uint8_t responder_id;
    uint8_t responder_index;
    uint64_t request_rx_tag_ts;
    uint64_t response_rx_tag_ts;
    uint64_t responder_reply_dtu;
    int32_t anchor_distance_mm;
    uint32_t anchor_distance_slot_id;
    bool have_request;
    bool have_response;
    bool resp_clock_offset_valid;
    int32_t resp_clock_offset_raw;
    double resp_clock_offset_ratio;
    TickType_t updated_tick;
};

struct uwb_flex_tdoa_message_metadata {
    uint32_t slot_id;
    uint8_t destination_count;
    uint8_t destinations[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U];
    uint32_t processing_dtu;
    uint8_t previous_responder_id;
    uint16_t previous_distance_mm;
    uint16_t previous_slot_id;
};

struct uwb_flex_tdoa_local_request {
    bool active;
    uint16_t sequence;
    uint32_t slot_id;
    uint8_t initiator_id;
    uint8_t responder_count;
    uint8_t responses_seen;
    uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U];
    uint64_t tx_timestamp;
    TickType_t updated_tick;
};

struct uwb_flex_tdoa_schedule {
    bool synced;
    bool reference_from_request;
    uint8_t own_slot_index;
    uint32_t reference_slot_id;
    uint32_t next_slot_id;
    uint64_t next_request_radio_ts;
    int64_t next_request_host_us;
    int64_t last_sync_host_us;
    uint32_t sync_count;
    uint32_t missed_slots;
};

struct uwb_flex_tdoa_geometry_staging {
    uint32_t generation;
    uint8_t anchor_count;
    uint16_t received_mask;
    int32_t anchor_x_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS];
    int32_t anchor_y_mm[APP_RUNTIME_CONFIG_MAX_ANCHORS];
};

struct uwb_passive_ds_schedule {
    bool synced;
    uint32_t next_owned_slot_id;
    uint64_t next_owned_poll_radio_ts;
    int64_t next_owned_poll_host_us;
    int64_t last_poll_host_us;
    uint32_t bootstrap_count;
    uint32_t late_count;
};

struct uwb_passive_ds_multi_schedule {
    bool synced;
    uint32_t next_frame_id;
    uint64_t next_poll_radio_ts;
    int64_t next_poll_host_us;
    int64_t last_poll_host_us;
    uint32_t bootstrap_count;
    uint32_t late_count;
};

struct uwb_passive_ds_multi_tag_stats {
    uint32_t poll_rx;
    uint32_t response_rx[UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS];
    uint32_t final_rx;
    uint32_t final_entries;
    uint32_t ready[UWB_PASSIVE_DS_MULTI_MAX_ANCHORS];
    uint32_t rejected[UWB_PASSIVE_DS_MULTI_MAX_ANCHORS];
    uint32_t piggyback_lag[8];
    uint32_t previous_ready_count;
    uint32_t previous_calculation_rejected_count;
    uint32_t previous_missing_final_context_count;
    uint32_t previous_missing_exchange_context_count;
    uint32_t previous_pending_replacement_count;
    int64_t summary_started_us;
};

static void uwb_passive_ds_multi_tag_record_piggyback_lag(
    struct uwb_passive_ds_multi_tag_stats *stats,
    uint32_t current_frame_id, uint8_t piggyback_initiator_id,
    uint32_t piggyback_frame_id)
{
    if (stats == NULL || piggyback_initiator_id == 0U) {
        return;
    }
    const uint32_t lag = current_frame_id - piggyback_frame_id;
    stats->piggyback_lag[lag < 7U ? lag : 7U]++;
}

enum uwb_passive_ds_exchange_state {
    UWB_PASSIVE_DS_EXCHANGE_IDLE = 0,
    UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE,
    UWB_PASSIVE_DS_EXCHANGE_WAIT_FINAL,
};

struct uwb_passive_ds_exchange {
    enum uwb_passive_ds_exchange_state state;
    uint16_t sequence;
    uint32_t slot_id;
    uint8_t peer_id;
    int64_t started_host_us;
    int64_t deadline_host_us;
    uint64_t poll_tx_ts;
    uint64_t poll_rx_ts;
    uint64_t response_tx_ts;
    struct uwb_rx_diagnostics poll_rx_diagnostics;
};

enum uwb_passive_ds_range_source {
    UWB_PASSIVE_DS_RANGE_NONE = 0,
    UWB_PASSIVE_DS_RANGE_PIGGYBACK_DS = 1,
};

static struct uwb_flex_tdoa_geometry_staging s_flex_tdoa_geometry_staging;

enum uwb_flex_tdoa_config_result {
    UWB_FLEX_TDOA_CONFIG_INVALID,
    UWB_FLEX_TDOA_CONFIG_CURRENT,
    UWB_FLEX_TDOA_CONFIG_UPDATED,
};

static void uwb_distance_put_u32(uint8_t *payload, size_t offset,
                                 uint32_t value);
static uint32_t uwb_distance_get_u32(const uint8_t *payload, size_t offset);
static void uwb_distance_put_u16(uint8_t *payload, size_t offset,
                                 uint16_t value);
static uint16_t uwb_distance_get_u16(const uint8_t *payload, size_t offset);
static void uwb_distance_put_i32(uint8_t *payload, size_t offset,
                                 int32_t value);
static int32_t uwb_distance_get_i32(const uint8_t *payload, size_t offset);
static void uwb_distance_build_frame(enum uwb_distance_frame_type type,
                                     uint8_t destination_id,
                                     uint16_t sequence,
                                     uint8_t payload[UWB_DW3000_PAYLOAD_LEN]);
static bool uwb_distance_destination_matches(uint8_t destination_id);
static esp_err_t uwb_dw3000_send_payload(const uint8_t *payload,
                                         size_t payload_len,
                                         uint64_t *tx_timestamp);

static uint8_t uwb_flex_tdoa_slot_initiator(uint32_t slot_id)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    return config->flex_tdoa_slot_initiator_ids[
        slot_id % config->flex_tdoa_slot_count];
}

static void uwb_flex_tdoa_build_config(
    const app_runtime_config_t *config,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG,
                             UWB_DISTANCE_FRAME_BROADCAST_ID,
                             (uint16_t)config->flex_tdoa_config_generation,
                             payload);
    uwb_distance_put_u32(payload, UWB_FLEX_TDOA_CONFIG_GENERATION_OFFSET,
                         config->flex_tdoa_config_generation);
    payload[UWB_FLEX_TDOA_CONFIG_N_OFFSET] = config->anchor_count;
    payload[UWB_FLEX_TDOA_CONFIG_K_OFFSET] =
        config->flex_tdoa_responder_count;
    payload[UWB_FLEX_TDOA_CONFIG_M_OFFSET] = config->flex_tdoa_slot_count;
    memcpy(&payload[UWB_FLEX_TDOA_CONFIG_ANCHORS_OFFSET], config->anchor_ids,
           sizeof(config->anchor_ids));
    for (size_t slot = 0; slot < config->flex_tdoa_slot_count; ++slot) {
        uint8_t initiator_index = 0;
        for (size_t anchor = 0; anchor < config->anchor_count; ++anchor) {
            if (config->anchor_ids[anchor] ==
                config->flex_tdoa_slot_initiator_ids[slot]) {
                initiator_index = (uint8_t)anchor;
                break;
            }
        }
        const size_t byte_index = UWB_FLEX_TDOA_CONFIG_SLOTS_OFFSET + slot / 2U;
        const uint8_t shift = (uint8_t)((slot % 2U) * 4U);
        payload[byte_index] |= (uint8_t)(initiator_index << shift);
        for (size_t anchor = 0; anchor < config->anchor_count; ++anchor) {
            if ((config->flex_tdoa_slot_responder_masks[slot] &
                 (uint16_t)(1U << anchor)) == 0U) {
                continue;
            }
            const size_t bit_index = slot * config->anchor_count + anchor;
            payload[UWB_FLEX_TDOA_CONFIG_MASKS_OFFSET + bit_index / 8U] |=
                (uint8_t)(1U << (bit_index % 8U));
        }
    }
    uwb_distance_put_u16(payload, UWB_FLEX_TDOA_CONFIG_GUARD_US_OFFSET,
                         (uint16_t)config->flex_tdoa_guard_us);
    uwb_distance_put_u16(payload, UWB_FLEX_TDOA_CONFIG_REQ_US_OFFSET,
                         (uint16_t)config->flex_tdoa_request_subslot_us);
    uwb_distance_put_u16(payload,
                         UWB_FLEX_TDOA_CONFIG_REQ_PROCESS_US_OFFSET,
                         (uint16_t)config->flex_tdoa_request_process_us);
    uwb_distance_put_u16(payload, UWB_FLEX_TDOA_CONFIG_RESP_US_OFFSET,
                         (uint16_t)config->flex_tdoa_response_subslot_us);
    uwb_distance_put_u16(payload,
                         UWB_FLEX_TDOA_CONFIG_RESP_PROCESS_US_OFFSET,
                         (uint16_t)config->flex_tdoa_response_process_us);
}

static bool uwb_flex_tdoa_parse_config(
    const struct uwb_distance_frame *frame, app_runtime_config_t *parsed)
{
    if (frame == NULL || parsed == NULL ||
        frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG ||
        frame->payload_len < UWB_FLEX_TDOA_CONFIG_LEN ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return false;
    }

    const app_runtime_config_t *current = app_runtime_config_get();
    *parsed = *current;
    parsed->flex_tdoa_config_generation = uwb_distance_get_u32(
        frame->payload, UWB_FLEX_TDOA_CONFIG_GENERATION_OFFSET);
    parsed->anchor_count = frame->payload[UWB_FLEX_TDOA_CONFIG_N_OFFSET];
    parsed->flex_tdoa_responder_count =
        frame->payload[UWB_FLEX_TDOA_CONFIG_K_OFFSET];
    parsed->flex_tdoa_slot_count =
        frame->payload[UWB_FLEX_TDOA_CONFIG_M_OFFSET];
    parsed->flex_tdoa_guard_us = uwb_distance_get_u16(
        frame->payload, UWB_FLEX_TDOA_CONFIG_GUARD_US_OFFSET);
    parsed->flex_tdoa_request_subslot_us = uwb_distance_get_u16(
        frame->payload, UWB_FLEX_TDOA_CONFIG_REQ_US_OFFSET);
    parsed->flex_tdoa_request_process_us = uwb_distance_get_u16(
        frame->payload, UWB_FLEX_TDOA_CONFIG_REQ_PROCESS_US_OFFSET);
    parsed->flex_tdoa_response_subslot_us = uwb_distance_get_u16(
        frame->payload, UWB_FLEX_TDOA_CONFIG_RESP_US_OFFSET);
    parsed->flex_tdoa_response_process_us = uwb_distance_get_u16(
        frame->payload, UWB_FLEX_TDOA_CONFIG_RESP_PROCESS_US_OFFSET);
    memcpy(parsed->anchor_ids,
           &frame->payload[UWB_FLEX_TDOA_CONFIG_ANCHORS_OFFSET],
           sizeof(parsed->anchor_ids));
    if (parsed->anchor_count != current->anchor_count ||
        memcmp(parsed->anchor_ids, current->anchor_ids,
               sizeof(parsed->anchor_ids)) != 0) {
        parsed->flex_tdoa_geometry_fixed = false;
        memset(parsed->flex_tdoa_anchor_x_mm, 0,
               sizeof(parsed->flex_tdoa_anchor_x_mm));
        memset(parsed->flex_tdoa_anchor_y_mm, 0,
               sizeof(parsed->flex_tdoa_anchor_y_mm));
    }
    memset(parsed->flex_tdoa_slot_initiator_ids, 0,
           sizeof(parsed->flex_tdoa_slot_initiator_ids));
    memset(parsed->flex_tdoa_slot_responder_masks, 0,
           sizeof(parsed->flex_tdoa_slot_responder_masks));
    if (parsed->anchor_count == 0U ||
        parsed->anchor_count > APP_RUNTIME_CONFIG_MAX_ANCHORS ||
        parsed->flex_tdoa_slot_count == 0U ||
        parsed->flex_tdoa_slot_count > APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS) {
        return false;
    }
    for (size_t slot = 0; slot < parsed->flex_tdoa_slot_count; ++slot) {
        const uint8_t packed = frame->payload[
            UWB_FLEX_TDOA_CONFIG_SLOTS_OFFSET + slot / 2U];
        const uint8_t initiator_index =
            (uint8_t)((packed >> ((slot % 2U) * 4U)) & 0x0FU);
        if (initiator_index >= parsed->anchor_count) {
            return false;
        }
        parsed->flex_tdoa_slot_initiator_ids[slot] =
            parsed->anchor_ids[initiator_index];
        for (size_t anchor = 0; anchor < parsed->anchor_count; ++anchor) {
            const size_t bit_index = slot * parsed->anchor_count + anchor;
            if ((frame->payload[UWB_FLEX_TDOA_CONFIG_MASKS_OFFSET +
                                bit_index / 8U] &
                 (uint8_t)(1U << (bit_index % 8U))) != 0U) {
                parsed->flex_tdoa_slot_responder_masks[slot] |=
                    (uint16_t)(1U << anchor);
            }
        }
    }
    return parsed->flex_tdoa_config_generation != 0U &&
           app_runtime_config_validate(parsed);
}

static esp_err_t uwb_flex_tdoa_send_config(void)
{
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_flex_tdoa_build_config(app_runtime_config_get(), payload);
    return uwb_dw3000_send_payload(payload, UWB_FLEX_TDOA_CONFIG_LEN, NULL);
}

static esp_err_t uwb_flex_tdoa_send_geometry(size_t anchor_index)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY,
                             UWB_DISTANCE_FRAME_BROADCAST_ID,
                             (uint16_t)config->flex_tdoa_geometry_generation,
                             payload);
    uwb_distance_put_u32(payload,
                         UWB_FLEX_TDOA_GEOMETRY_GENERATION_OFFSET,
                         config->flex_tdoa_geometry_generation);
    payload[UWB_FLEX_TDOA_GEOMETRY_FIXED_OFFSET] =
        config->flex_tdoa_geometry_fixed ? 1U : 0U;
    if (config->flex_tdoa_geometry_fixed) {
        if (anchor_index >= config->anchor_count) {
            return ESP_ERR_INVALID_ARG;
        }
        payload[UWB_FLEX_TDOA_GEOMETRY_COUNT_OFFSET] = config->anchor_count;
        payload[UWB_FLEX_TDOA_GEOMETRY_INDEX_OFFSET] = (uint8_t)anchor_index;
        payload[UWB_FLEX_TDOA_GEOMETRY_ANCHOR_ID_OFFSET] =
            config->anchor_ids[anchor_index];
        uwb_distance_put_i32(payload, UWB_FLEX_TDOA_GEOMETRY_X_MM_OFFSET,
                             config->flex_tdoa_anchor_x_mm[anchor_index]);
        uwb_distance_put_i32(payload, UWB_FLEX_TDOA_GEOMETRY_Y_MM_OFFSET,
                             config->flex_tdoa_anchor_y_mm[anchor_index]);
    }
    return uwb_dw3000_send_payload(payload, UWB_FLEX_TDOA_GEOMETRY_LEN,
                                   NULL);
}

static enum uwb_flex_tdoa_config_result
uwb_flex_tdoa_apply_geometry_frame(const struct uwb_distance_frame *frame)
{
    if (frame == NULL ||
        frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY ||
        frame->payload_len < UWB_FLEX_TDOA_GEOMETRY_LEN ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return UWB_FLEX_TDOA_CONFIG_INVALID;
    }
    const uint32_t generation = uwb_distance_get_u32(
        frame->payload, UWB_FLEX_TDOA_GEOMETRY_GENERATION_OFFSET);
    const app_runtime_config_t *current = app_runtime_config_get();
    if (generation == 0U ||
        generation <= current->flex_tdoa_geometry_generation) {
        return UWB_FLEX_TDOA_CONFIG_CURRENT;
    }

    const bool fixed =
        frame->payload[UWB_FLEX_TDOA_GEOMETRY_FIXED_OFFSET] != 0U;
    app_runtime_config_t updated = *current;
    if (!fixed) {
        updated.flex_tdoa_geometry_fixed = false;
        updated.flex_tdoa_geometry_generation = generation;
        memset(updated.flex_tdoa_anchor_x_mm, 0,
               sizeof(updated.flex_tdoa_anchor_x_mm));
        memset(updated.flex_tdoa_anchor_y_mm, 0,
               sizeof(updated.flex_tdoa_anchor_y_mm));
    } else {
        const uint8_t count =
            frame->payload[UWB_FLEX_TDOA_GEOMETRY_COUNT_OFFSET];
        const uint8_t index =
            frame->payload[UWB_FLEX_TDOA_GEOMETRY_INDEX_OFFSET];
        const uint8_t anchor_id =
            frame->payload[UWB_FLEX_TDOA_GEOMETRY_ANCHOR_ID_OFFSET];
        if (count != current->anchor_count || index >= count ||
            anchor_id != current->anchor_ids[index]) {
            return UWB_FLEX_TDOA_CONFIG_INVALID;
        }
        if (s_flex_tdoa_geometry_staging.generation != generation ||
            s_flex_tdoa_geometry_staging.anchor_count != count) {
            memset(&s_flex_tdoa_geometry_staging, 0,
                   sizeof(s_flex_tdoa_geometry_staging));
            s_flex_tdoa_geometry_staging.generation = generation;
            s_flex_tdoa_geometry_staging.anchor_count = count;
        }
        s_flex_tdoa_geometry_staging.anchor_x_mm[index] =
            uwb_distance_get_i32(frame->payload,
                                 UWB_FLEX_TDOA_GEOMETRY_X_MM_OFFSET);
        s_flex_tdoa_geometry_staging.anchor_y_mm[index] =
            uwb_distance_get_i32(frame->payload,
                                 UWB_FLEX_TDOA_GEOMETRY_Y_MM_OFFSET);
        s_flex_tdoa_geometry_staging.received_mask |=
            (uint16_t)(1U << index);
        const uint16_t complete_mask = (uint16_t)((1U << count) - 1U);
        if (s_flex_tdoa_geometry_staging.received_mask != complete_mask) {
            return UWB_FLEX_TDOA_CONFIG_CURRENT;
        }
        updated.flex_tdoa_geometry_fixed = true;
        updated.flex_tdoa_geometry_generation = generation;
        memcpy(updated.flex_tdoa_anchor_x_mm,
               s_flex_tdoa_geometry_staging.anchor_x_mm,
               sizeof(updated.flex_tdoa_anchor_x_mm));
        memcpy(updated.flex_tdoa_anchor_y_mm,
               s_flex_tdoa_geometry_staging.anchor_y_mm,
               sizeof(updated.flex_tdoa_anchor_y_mm));
    }

    const esp_err_t err = app_runtime_config_save(&updated);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FlexTDOA geometry save failed generation=%lu: %s",
                 (unsigned long)generation, esp_err_to_name(err));
        return UWB_FLEX_TDOA_CONFIG_INVALID;
    }
    (void)uwb_flex_tdoa_runtime_reload_geometry();
    ESP_LOGI(TAG,
             "FlexTDOA geometry applied source=%u generation=%lu fixed=%u",
             (unsigned)frame->source_id, (unsigned long)generation,
             fixed ? 1U : 0U);
    return UWB_FLEX_TDOA_CONFIG_UPDATED;
}

static enum uwb_flex_tdoa_config_result uwb_flex_tdoa_apply_config_frame(
    const struct uwb_distance_frame *frame)
{
    app_runtime_config_t received = {0};
    if (!uwb_flex_tdoa_parse_config(frame, &received)) {
        return UWB_FLEX_TDOA_CONFIG_INVALID;
    }

    const app_runtime_config_t *current = app_runtime_config_get();
    if (received.flex_tdoa_config_generation <=
        current->flex_tdoa_config_generation) {
        return UWB_FLEX_TDOA_CONFIG_CURRENT;
    }
    const esp_err_t err = app_runtime_config_save(&received);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FlexTDOA radio config save failed generation=%lu: %s",
                 (unsigned long)received.flex_tdoa_config_generation,
                 esp_err_to_name(err));
        return UWB_FLEX_TDOA_CONFIG_INVALID;
    }
    ESP_LOGI(TAG,
             "FlexTDOA radio config applied source=%u generation=%lu N=%u K=%u M=%u",
             (unsigned)frame->source_id,
             (unsigned long)received.flex_tdoa_config_generation,
             (unsigned)received.anchor_count,
             (unsigned)received.flex_tdoa_responder_count,
             (unsigned)received.flex_tdoa_slot_count);
    return UWB_FLEX_TDOA_CONFIG_UPDATED;
}

static bool uwb_flex_tdoa_handle_runtime_config(
    const struct uwb_distance_frame *frame)
{
    if (frame == NULL) {
        return false;
    }

    if (frame->type == UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY) {
        return uwb_flex_tdoa_apply_geometry_frame(frame) !=
               UWB_FLEX_TDOA_CONFIG_INVALID;
    }
    if (frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG) {
        return false;
    }

    const enum uwb_flex_tdoa_config_result result =
        uwb_flex_tdoa_apply_config_frame(frame);
    if (result == UWB_FLEX_TDOA_CONFIG_UPDATED) {
        ESP_LOGW(TAG,
                 "FlexTDOA schedule changed over UWB; restarting to rebuild local state");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    }
    return result != UWB_FLEX_TDOA_CONFIG_INVALID;
}

static uint32_t uwb_dw3000_remaining_ms(int64_t start_us,
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
static esp_err_t uwb_dw3000_send_payload_delayed_timeout(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint32_t tx_timeout_ms, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp);
static esp_err_t uwb_dw3000_send_payload_delayed_expect_rx(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp);
static esp_err_t uwb_dw3000_update_u32(uint8_t base, uint8_t sub,
                                       uint32_t clear_mask,
                                       uint32_t set_mask);
static esp_err_t uwb_dw3000_fast_command(uint8_t command);
static esp_err_t uwb_dw3000_clear_status(void);
static esp_err_t uwb_flex_tdoa_extend_rx_timestamp32(
    uint32_t low32, uint64_t *timestamp);
static void uwb_flex_tdoa_log_anchor_result(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, double distance_m, double raw_distance_m);
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
static uint32_t s_spi_clock_hz;
static uint32_t s_tx_fctrl_base;
static bool s_tx_fctrl_base_valid;
static size_t s_tx_fctrl_payload_len = SIZE_MAX;
static bool s_rx_double_buffer_enabled;
static uint8_t s_rx_double_buffer_index;
static bool s_flex_tdoa_anchor_request_buffer_pending;
static uint32_t s_rx_double_buffer_resync_count;
static bool s_flex_tdoa_rx_timestamp_reference_valid;
static uint64_t s_flex_tdoa_rx_timestamp_reference;
static int64_t s_flex_tdoa_rx_timestamp_reference_host_us;
static bool s_started;
static bool s_rx_armed;
static bool s_irq_enabled;
static TaskHandle_t s_task_handle;
static uint8_t s_source_id;
static uint32_t s_device_id;
static enum uwb_dw3000_runtime_mode s_runtime_mode =
    UWB_DW3000_RUNTIME_BEACON_SMOKE;
static volatile enum uwb_dw3000_runtime_mode s_requested_runtime_mode =
    UWB_DW3000_RUNTIME_BEACON_SMOKE;
static volatile uint32_t s_runtime_switch_request_generation;
static volatile uint32_t s_runtime_switch_applied_generation;
static volatile bool s_runtime_switch_active;
static volatile bool s_runtime_hot_entry;
static volatile uint32_t s_runtime_switch_count;
static volatile uint32_t s_last_runtime_switch_ms;
static volatile enum uwb_dw3000_status s_status = UWB_DW3000_STATUS_IDLE;
static volatile uint32_t s_tx_count;
static volatile uint32_t s_tx_error_count;
static volatile uint32_t s_rx_count;
static volatile uint32_t s_rx_error_count;
static volatile uint32_t s_rx_ignored_count;
static volatile uint8_t s_last_rx_source_id;
static volatile uint32_t s_last_rx_sequence;
static uint16_t s_antenna_delay = APP_UWB_ANTENNA_DELAY_DEFAULT;
static gptimer_handle_t s_calibration_timer;
static volatile bool s_calibration_timer_running;
static volatile bool s_calibration_timer_irq_armed;
static volatile bool s_calibration_timer_irq_started;
static volatile bool s_calibration_timer_start_on_tx_done;
static volatile TaskHandle_t s_calibration_timer_wait_task;
// These buffers live for the complete FlexTDOA runtime. Keeping them out of
// the UWB task stack avoids making the maximum N configuration consume nearly
// the entire 8 KiB real-time stack on entry to the tag/anchor loops.
static struct uwb_flex_tdoa_observation
    s_flex_tdoa_tag_observations[UWB_FLEX_TDOA_MAX_OBSERVATIONS];
static struct flextdoa_cfo_estimator s_flex_tdoa_cfo_estimator;
static struct flextdoa_slot_collection s_flex_tdoa_tag_collection;
static struct uwb_flex_tdoa_local_request s_flex_tdoa_local_request;
static esp_timer_handle_t s_flex_tdoa_schedule_timer;
static volatile bool s_flex_tdoa_schedule_alarm_fired;
static esp_timer_handle_t s_passive_ds_schedule_timer;
static volatile bool s_passive_ds_schedule_alarm_fired;
static uint32_t s_flex_tdoa_observations_since_summary;
static uint32_t s_flex_tdoa_observation_drops_since_summary;
static uint32_t s_flex_tdoa_observation_invalid_since_summary;
static uint32_t s_flex_tdoa_observation_invalid_age_since_summary;
static uint32_t s_flex_tdoa_observation_invalid_cfo_since_summary;
static uint32_t s_flex_tdoa_observation_invalid_order_since_summary;
static uint32_t s_flex_tdoa_complete_slots_since_summary;
static uint32_t s_flex_tdoa_incomplete_slots_since_summary;
static bool s_flex_tdoa_tag_request_slot_valid;
static uint32_t s_flex_tdoa_tag_last_request_slot_id;
static uint32_t s_flex_tdoa_missed_requests_since_summary;
static uint32_t s_flex_tdoa_observation_invalid_age_index_since_summary[
    UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U];
static TickType_t s_flex_tdoa_observation_summary_tick;
static uint32_t s_flex_tdoa_anchor_results_since_summary;
static uint32_t s_flex_tdoa_anchor_drops_since_summary;
static uint32_t s_flex_tdoa_anchor_incoherent_since_summary;
static uint32_t s_flex_tdoa_anchor_index_since_summary[
    UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U];
static uint32_t s_flex_tdoa_tag_requests_by_anchor[
    UWB_ANCHOR_SURVEY_MAX_ANCHORS];
static uint32_t s_flex_tdoa_tag_responses_by_anchor[
    UWB_ANCHOR_SURVEY_MAX_ANCHORS];
static uint32_t s_flex_tdoa_response_tx_by_initiator[
    UWB_ANCHOR_SURVEY_MAX_ANCHORS];
static TickType_t s_flex_tdoa_anchor_summary_tick;
static uint32_t s_flex_tdoa_rx_errors_since_summary;
static uint32_t s_flex_tdoa_rx_error_status_since_summary;
static uint32_t s_last_delayed_arm_us;

static bool uwb_dw3000_runtime_from_app_mode(
    uint8_t app_runtime_mode, enum uwb_dw3000_runtime_mode *runtime_mode)
{
    if (runtime_mode == NULL) {
        return false;
    }
    switch (app_runtime_mode) {
    case APP_RUNTIME_MODE_UWB_RANGING:
        *runtime_mode = UWB_DW3000_RUNTIME_RANGING;
        return true;
    case APP_RUNTIME_MODE_UWB_FLEX_TDOA:
        *runtime_mode = UWB_DW3000_RUNTIME_FLEX_TDOA;
        return true;
    case APP_RUNTIME_MODE_UWB_PASSIVE_DS_TWR:
        *runtime_mode = UWB_DW3000_RUNTIME_PASSIVE_DS_TWR;
        return true;
    default:
        return false;
    }
}

static bool uwb_dw3000_runtime_mode_hot_switchable(
    enum uwb_dw3000_runtime_mode runtime_mode)
{
    return runtime_mode == UWB_DW3000_RUNTIME_RANGING ||
           runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA ||
           runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR;
}

static bool uwb_dw3000_runtime_switch_pending(void)
{
    return s_runtime_switch_request_generation !=
           s_runtime_switch_applied_generation;
}

static bool uwb_local_is_configured_anchor(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    for (size_t i = 0; i < config->anchor_count; ++i) {
        if (config->anchor_ids[i] == s_source_id) {
            return true;
        }
    }
    return false;
}

static bool uwb_passive_ds_multi_local_is_receive_only_tag(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    return s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR &&
           config != NULL &&
           config->passive_ds_schedule ==
               APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS &&
           !uwb_local_is_configured_anchor();
}

static bool uwb_dw3000_runtime_uses_high_rate_rx_buffer(
    enum uwb_dw3000_runtime_mode runtime_mode)
{
    if (runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
        return true;
    }
    return runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR &&
           uwb_passive_ds_multi_local_is_receive_only_tag();
}

static void uwb_flex_tdoa_schedule_alarm_callback(void *arg)
{
    (void)arg;
    s_flex_tdoa_schedule_alarm_fired = true;
    if (s_task_handle != NULL) {
        xTaskNotifyGive(s_task_handle);
    }
}

static void uwb_passive_ds_schedule_alarm_callback(void *arg)
{
    (void)arg;
    s_passive_ds_schedule_alarm_fired = true;
    if (s_task_handle != NULL) {
        xTaskNotifyGive(s_task_handle);
    }
}

static void uwb_passive_ds_record_stage(
    enum uwb_passive_ds_runtime_stage stage, int64_t start_host_us,
    bool success)
{
    uwb_passive_ds_runtime_record_stage(
        stage, start_host_us, esp_timer_get_time(), success);
}

static bool IRAM_ATTR uwb_calibration_timer_alarm_callback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task = s_calibration_timer_wait_task;
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    }
    return higher_priority_task_woken == pdTRUE;
}

static int gpio_level_active(int active_high)
{
    return active_high ? 1 : 0;
}

static int gpio_level_inactive(int active_high)
{
    return active_high ? 0 : 1;
}

static uint8_t uwb_dw3000_runtime_radio_channel(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    return config->radio_channel == 9U ? 9U : 5U;
}

static uint8_t uwb_dw3000_runtime_radio_rf_channel_bit(void)
{
    return uwb_dw3000_runtime_radio_channel() == 9U ? 1U : 0U;
}

static uint8_t uwb_dw3000_runtime_radio_phy_mode(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    switch (config->radio_phy_mode) {
    case APP_UWB_RADIO_PHY_LONG_RANGE:
    case APP_UWB_RADIO_PHY_FAST_PLEN256:
    case APP_UWB_RADIO_PHY_FAST_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
        return config->radio_phy_mode;
    default:
        return APP_UWB_RADIO_PHY_FAST;
    }
}

static bool uwb_dw3000_runtime_radio_is_850k(void)
{
    const uint8_t mode = uwb_dw3000_runtime_radio_phy_mode();
    return mode == APP_UWB_RADIO_PHY_LONG_RANGE ||
           mode == APP_UWB_RADIO_PHY_850K_PLEN512 ||
           mode == APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD;
}

static uint8_t uwb_dw3000_runtime_radio_profile(void)
{
    const uint8_t mode = uwb_dw3000_runtime_radio_phy_mode();
    if (mode == APP_UWB_RADIO_PHY_LONG_RANGE) {
        return uwb_dw3000_runtime_radio_channel() == 9U
                   ? APP_UWB_RADIO_PROFILE_LONG_RANGE_CH9_850K_PLEN1024
                   : APP_UWB_RADIO_PROFILE_LONG_RANGE_CH5_850K_PLEN1024;
    }
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN256) {
        return uwb_dw3000_runtime_radio_channel() == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_6M8_PLEN256
                   : APP_UWB_RADIO_PROFILE_CH5_6M8_PLEN256;
    }
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN512) {
        return uwb_dw3000_runtime_radio_channel() == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_6M8_PLEN512
                   : APP_UWB_RADIO_PROFILE_CH5_6M8_PLEN512;
    }
    if (mode == APP_UWB_RADIO_PHY_850K_PLEN512) {
        return uwb_dw3000_runtime_radio_channel() == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_850K_PLEN512
                   : APP_UWB_RADIO_PROFILE_CH5_850K_PLEN512;
    }
    if (mode == APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD) {
        return uwb_dw3000_runtime_radio_channel() == 9U
                   ? APP_UWB_RADIO_PROFILE_CH9_850K_PLEN512_STD_SFD
                   : APP_UWB_RADIO_PROFILE_CH5_850K_PLEN512_STD_SFD;
    }
    return uwb_dw3000_runtime_radio_channel() == 9U
               ? APP_UWB_RADIO_PROFILE_LEGACY_CH9_6M8_PLEN128
               : APP_UWB_RADIO_PROFILE_LEGACY_CH5_6M8_PLEN128;
}

static uint8_t uwb_dw3000_runtime_radio_preamble_len_code(void)
{
    switch (uwb_dw3000_runtime_radio_phy_mode()) {
    case APP_UWB_RADIO_PHY_LONG_RANGE:
        return APP_UWB_RADIO_PLEN_1024;
    case APP_UWB_RADIO_PHY_FAST_PLEN256:
        return APP_UWB_RADIO_PLEN_256;
    case APP_UWB_RADIO_PHY_FAST_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
        return APP_UWB_RADIO_PLEN_512;
    default:
        return APP_UWB_RADIO_PREAMBLE_LEN_CODE;
    }
}

static uint8_t uwb_dw3000_runtime_radio_preamble_code(void)
{
    return uwb_dw3000_runtime_radio_phy_mode() ==
                   APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD
               ? 9U
               : APP_UWB_RADIO_PREAMBLE_CODE;
}

static uint8_t uwb_dw3000_runtime_radio_pac(void)
{
    const uint8_t mode = uwb_dw3000_runtime_radio_phy_mode();
    if (mode == APP_UWB_RADIO_PHY_FAST_PLEN256) {
        return 1U; /* DWT_PAC16 */
    }
    if (mode == APP_UWB_RADIO_PHY_LONG_RANGE ||
        mode == APP_UWB_RADIO_PHY_FAST_PLEN512 ||
        mode == APP_UWB_RADIO_PHY_850K_PLEN512 ||
        mode == APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD) {
        return 2U; /* DWT_PAC32 */
    }
    return APP_UWB_RADIO_PAC;
}

static uint8_t uwb_dw3000_runtime_radio_data_rate(void)
{
    return uwb_dw3000_runtime_radio_is_850k()
               ? APP_UWB_RADIO_BR_850K
               : APP_UWB_RADIO_DATA_RATE;
}

static uint8_t uwb_dw3000_runtime_radio_phr_rate(void)
{
    return uwb_dw3000_runtime_radio_is_850k()
               ? 0U
               : APP_UWB_RADIO_PHR_RATE;
}

static uint16_t uwb_dw3000_runtime_radio_sfd_timeout(void)
{
    /* plen + 1 + 8-symbol Qorvo SFD - PAC. */
    switch (uwb_dw3000_runtime_radio_phy_mode()) {
    case APP_UWB_RADIO_PHY_LONG_RANGE:
        return 1001U;
    case APP_UWB_RADIO_PHY_FAST_PLEN256:
        return 249U;
    case APP_UWB_RADIO_PHY_FAST_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512:
    case APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD:
        return 489U;
    default:
        return 129U;
    }
}

static uint8_t uwb_dw3000_runtime_radio_sfd_type(void)
{
    return uwb_dw3000_runtime_radio_phy_mode() ==
                   APP_UWB_RADIO_PHY_850K_PLEN512_STD_SFD
               ? 0U
               : APP_UWB_RADIO_SFD_TYPE;
}

static uint32_t uwb_dw3000_runtime_rf_tx_ctrl_2(void)
{
    return uwb_dw3000_runtime_radio_channel() == 9U
               ? APP_UWB_RADIO_RF_TX_CTRL_2_CH9
               : APP_UWB_RADIO_RF_TX_CTRL_2_CH5;
}

static uint32_t uwb_dw3000_runtime_pll_cfg_final(void)
{
    return uwb_dw3000_runtime_radio_channel() == 9U
               ? APP_UWB_RADIO_PLL_CFG_FINAL_CH9
               : APP_UWB_RADIO_PLL_CFG_FINAL_CH5;
}

static uint32_t uwb_dw3000_runtime_dgc_lut(size_t index)
{
    static const uint32_t ch5[] = {
        APP_UWB_DGC_LUT_CH5_0, APP_UWB_DGC_LUT_CH5_1,
        APP_UWB_DGC_LUT_CH5_2, APP_UWB_DGC_LUT_CH5_3,
        APP_UWB_DGC_LUT_CH5_4, APP_UWB_DGC_LUT_CH5_5,
        APP_UWB_DGC_LUT_CH5_6,
    };
    static const uint32_t ch9[] = {
        APP_UWB_DGC_LUT_CH9_0, APP_UWB_DGC_LUT_CH9_1,
        APP_UWB_DGC_LUT_CH9_2, APP_UWB_DGC_LUT_CH9_3,
        APP_UWB_DGC_LUT_CH9_4, APP_UWB_DGC_LUT_CH9_5,
        APP_UWB_DGC_LUT_CH9_6,
    };
    const uint32_t *lut =
        uwb_dw3000_runtime_radio_channel() == 9U ? ch9 : ch5;
    return index < 7U ? lut[index] : lut[0];
}

static void uwb_dw3000_delay_ms(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

static esp_err_t uwb_calibration_timer_init(void)
{
    if (s_calibration_timer != NULL) {
        return ESP_OK;
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000U,
    };

    esp_err_t err = gptimer_new_timer(&timer_config,
                                      &s_calibration_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration timer create failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    const gptimer_event_callbacks_t callbacks = {
        .on_alarm = uwb_calibration_timer_alarm_callback,
    };
    err = gptimer_register_event_callbacks(
        s_calibration_timer, &callbacks, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration timer callback register failed: %s",
                 esp_err_to_name(err));
        goto fail_delete;
    }
    err = gptimer_enable(s_calibration_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration timer enable failed: %s",
                 esp_err_to_name(err));
        goto fail_delete;
    }
    err = gptimer_set_raw_count(s_calibration_timer, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration timer initial clear failed: %s",
                 esp_err_to_name(err));
        (void)gptimer_disable(s_calibration_timer);
        goto fail_delete;
    }

    s_calibration_timer_running = false;
    s_calibration_timer_irq_armed = false;
    s_calibration_timer_irq_started = false;
    s_calibration_timer_start_on_tx_done = false;
    s_calibration_timer_wait_task = NULL;
    ESP_LOGI(TAG, "UWB calibration timer ready at 1 MHz");
    return ESP_OK;

fail_delete:
    (void)gptimer_del_timer(s_calibration_timer);
    s_calibration_timer = NULL;
    s_calibration_timer_running = false;
    s_calibration_timer_irq_armed = false;
    s_calibration_timer_irq_started = false;
    s_calibration_timer_start_on_tx_done = false;
    s_calibration_timer_wait_task = NULL;
    return err;
}

static esp_err_t uwb_calibration_timer_stop_if_running(void)
{
    if (s_calibration_timer == NULL || !s_calibration_timer_running) {
        return ESP_OK;
    }

    const esp_err_t err = gptimer_stop(s_calibration_timer);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    s_calibration_timer_running = false;
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_reset(void)
{
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_init(), TAG,
                        "calibration timer init failed");
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_stop_if_running(), TAG,
                        "calibration timer stop failed");

    s_calibration_timer_irq_armed = false;
    s_calibration_timer_irq_started = false;
    s_calibration_timer_start_on_tx_done = false;
    s_calibration_timer_wait_task = NULL;
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(s_calibration_timer, NULL),
                        TAG, "calibration timer alarm disable failed");
    ESP_RETURN_ON_ERROR(gptimer_set_raw_count(s_calibration_timer, 0), TAG,
                        "calibration timer count clear failed");
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_start(void)
{
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_init(), TAG,
                        "calibration timer init failed");
    if (s_calibration_timer_running) {
        return ESP_OK;
    }

    const esp_err_t err = gptimer_start(s_calibration_timer);
    if (err != ESP_OK) {
        return err;
    }
    s_calibration_timer_running = true;
    return ESP_OK;
}

static void IRAM_ATTR uwb_calibration_timer_start_from_isr(void)
{
    if (!s_calibration_timer_irq_armed || s_calibration_timer_irq_started ||
        s_calibration_timer == NULL) {
        return;
    }

    if (gptimer_start(s_calibration_timer) == ESP_OK) {
        s_calibration_timer_running = true;
        s_calibration_timer_irq_started = true;
        s_calibration_timer_irq_armed = false;
    }
}

static esp_err_t uwb_calibration_timer_arm_rx_irq_start(void)
{
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_reset(), TAG,
                        "calibration timer reset before RX sync failed");
    s_calibration_timer_irq_armed = s_irq_enabled;
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_arm_tx_irq_start(void)
{
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_reset(), TAG,
                        "calibration timer reset before TX sync failed");
    s_calibration_timer_start_on_tx_done = true;
    s_calibration_timer_irq_armed = s_irq_enabled;
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_accept_rx_sync(void)
{
    s_calibration_timer_irq_armed = false;
    if (!s_calibration_timer_irq_started) {
        ESP_RETURN_ON_ERROR(uwb_calibration_timer_start(), TAG,
                            "calibration timer RX fallback start failed");
    }
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_accept_tx_sync(void)
{
    if (!s_calibration_timer_start_on_tx_done) {
        return ESP_OK;
    }

    s_calibration_timer_start_on_tx_done = false;
    s_calibration_timer_irq_armed = false;
    if (!s_calibration_timer_irq_started) {
        ESP_RETURN_ON_ERROR(uwb_calibration_timer_start(), TAG,
                            "calibration timer TX fallback start failed");
    }
    return ESP_OK;
}

static esp_err_t uwb_calibration_timer_get_us(uint64_t *time_us)
{
    if (time_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(uwb_calibration_timer_init(), TAG,
                        "calibration timer init failed");
    return gptimer_get_raw_count(s_calibration_timer, time_us);
}

static void uwb_calibration_timer_wait_until_us(uint32_t target_us)
{
    while (true) {
        uint64_t now_us = 0;
        if (uwb_calibration_timer_get_us(&now_us) != ESP_OK ||
            now_us >= target_us) {
            s_calibration_timer_wait_task = NULL;
            return;
        }

        const uint32_t remaining_us = target_us - (uint32_t)now_us;
        if (!s_calibration_timer_running) {
            ESP_LOGW(TAG,
                     "calibration timer wait requested while timer is stopped");
            return;
        }

        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = target_us,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = false,
            },
        };
        s_calibration_timer_wait_task = xTaskGetCurrentTaskHandle();
        if (gptimer_set_alarm_action(s_calibration_timer, &alarm_config) !=
            ESP_OK) {
            s_calibration_timer_wait_task = NULL;
            vTaskDelay(1);
            continue;
        }

        if (uwb_calibration_timer_get_us(&now_us) != ESP_OK ||
            now_us >= target_us) {
            (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
            s_calibration_timer_wait_task = NULL;
            return;
        }

        TickType_t ticks =
            pdMS_TO_TICKS((remaining_us + 999U) / 1000U);
        if (ticks == 0) {
            ticks = 1;
        }
        (void)ulTaskNotifyTake(pdTRUE, ticks);
        (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
        s_calibration_timer_wait_task = NULL;
    }
}

static void IRAM_ATTR uwb_dw3000_irq_isr_handler(void *arg)
{
    (void)arg;
    uwb_calibration_timer_start_from_isr();
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_task_handle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void uwb_dw3000_wait_for_event_or_delay(int64_t start_us,
                                               uint32_t timeout_ms,
                                               uint32_t fallback_delay_ms)
{
#if APP_UWB_IRQ_ENABLED
    if (s_irq_enabled) {
        const uint32_t remaining_ms =
            uwb_dw3000_remaining_ms(start_us, timeout_ms);
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

esp_err_t uwb_dw3000_hold_in_reset(void)
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
                        "configure UWB reset GPIOs failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_CONFIG_UWB_CS_GPIO, 1), TAG,
                        "set UWB CS high failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_WAKEUP_GPIO,
                       gpio_level_inactive(BOARD_CONFIG_UWB_WAKEUP_ACTIVE_HIGH)),
        TAG, "set UWB WAKEUP inactive failed");
    ESP_RETURN_ON_ERROR(
        gpio_set_level(BOARD_CONFIG_UWB_RST_GPIO,
                       gpio_level_active(BOARD_CONFIG_UWB_RST_ACTIVE_HIGH)),
        TAG, "hold UWB RST active failed");

    s_status = UWB_DW3000_STATUS_IDLE;
    ESP_LOGW(TAG, "DW3000 held in reset for low-power/component-disable mode");
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

static esp_err_t uwb_dw3000_add_spi_device(uint32_t requested_clock_hz)
{
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = (int)requested_clock_hz,
        .mode = 0,
        .spics_io_num = BOARD_CONFIG_UWB_CS_GPIO,
        .queue_size = 1,
    };

    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(UWB_DW3000_SPI_HOST, &device_config, &s_spi), TAG,
        "spi_bus_add_device failed");

    int actual_clock_khz = 0;
    esp_err_t err = spi_device_get_actual_freq(s_spi, &actual_clock_khz);
    if (err != ESP_OK || actual_clock_khz <= 0) {
        (void)spi_bus_remove_device(s_spi);
        s_spi = NULL;
        return err == ESP_OK ? ESP_FAIL : err;
    }

    s_spi_clock_hz = (uint32_t)actual_clock_khz * 1000U;
    if (s_spi_clock_hz > UWB_DW3000_SPI_DATASHEET_MAX_HZ) {
        ESP_LOGW(TAG,
                 "DW3000 SPI actual clock %lu Hz exceeds datasheet maximum %u Hz",
                 (unsigned long)s_spi_clock_hz,
                 (unsigned)UWB_DW3000_SPI_DATASHEET_MAX_HZ);
    }
    if (s_spi_clock_hz > UWB_DW3000_SPI_VALIDATED_MAX_HZ) {
        ESP_LOGE(TAG,
                 "DW3000 SPI actual clock %lu Hz exceeds validated maximum %u Hz",
                 (unsigned long)s_spi_clock_hz,
                 (unsigned)UWB_DW3000_SPI_VALIDATED_MAX_HZ);
        (void)spi_bus_remove_device(s_spi);
        s_spi = NULL;
        s_spi_clock_hz = 0;
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "DW3000 SPI clock: requested=%lu Hz actual=%lu Hz",
             (unsigned long)requested_clock_hz,
             (unsigned long)s_spi_clock_hz);
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

    return uwb_dw3000_add_spi_device(UWB_DW3000_SPI_BOOT_CLOCK_HZ);
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
    uint8_t tx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES];
    uint8_t rx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES];
    const size_t header_len =
        uwb_dw3000_build_header(tx, sizeof(tx), base, sub, false);
    const size_t total_len = header_len + len;

    if (data == NULL || header_len == 0 ||
        total_len > UWB_DW3000_SPI_MAX_TRANSFER_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(tx + header_len, 0, len);

    spi_transaction_t transaction = {
        .length = total_len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &transaction), TAG,
                        "SPI read failed");
    memcpy(data, rx + header_len, len);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_write_bytes(uint8_t base, uint8_t sub,
                                        const uint8_t *data, size_t len)
{
    uint8_t tx[UWB_DW3000_SPI_MAX_TRANSFER_BYTES];
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

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &transaction), TAG,
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
    uint8_t tx[2U + sizeof(uint32_t)];
    const size_t header_len =
        uwb_dw3000_build_header(tx, sizeof(tx), base, sub, true);
    if (len == 0 || len > sizeof(uint32_t) || header_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < len; ++i) {
        tx[header_len + i] = (uint8_t)((value >> (8U * i)) & 0xFFU);
    }

    spi_transaction_t transaction = {
        .length = (header_len + len) * 8U,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_spi, &transaction);
}

static esp_err_t uwb_dw3000_write_u32_auto(uint8_t base, uint8_t sub,
                                           uint32_t value)
{
    return uwb_dw3000_write_u32_len(base, sub, value,
                                    uwb_dw3000_auto_len(value));
}

static esp_err_t uwb_dw3000_read32(uint8_t base, uint8_t sub, uint32_t *value)
{
    uint8_t tx[2U + sizeof(uint32_t)];
    uint8_t rx[2U + sizeof(uint32_t)];
    const size_t header_len =
        uwb_dw3000_build_header(tx, sizeof(tx), base, sub, false);
    if (value == NULL || header_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(tx + header_len, 0, sizeof(uint32_t));
    spi_transaction_t transaction = {
        .length = (header_len + sizeof(uint32_t)) * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &transaction), TAG,
                        "read32 failed");

    const uint8_t *bytes = rx + header_len;
    *value = ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_verify_spi_link(uint32_t expected_device_id,
                                            uint32_t read_count)
{
    for (uint32_t read_index = 0; read_index < read_count; ++read_index) {
        uint32_t device_id = 0;
        const esp_err_t err = uwb_dw3000_read32(
            DW3000_REG_GEN_CFG_AES_LOW, DW3000_SUB_NONE, &device_id);
        if (err != ESP_OK || device_id != expected_device_id) {
            ESP_LOGE(TAG,
                     "DW3000 SPI verify %lu/%lu failed: err=%s dev_id=0x%08lx expected=0x%08lx",
                     (unsigned long)(read_index + 1U),
                     (unsigned long)read_count, esp_err_to_name(err),
                     (unsigned long)device_id,
                     (unsigned long)expected_device_id);
            return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
        }
    }

    return ESP_OK;
}

static esp_err_t uwb_dw3000_restore_boot_spi(uint32_t expected_device_id)
{
    if (s_spi != NULL) {
        ESP_RETURN_ON_ERROR(spi_bus_remove_device(s_spi), TAG,
                            "remove operational SPI device failed");
        s_spi = NULL;
        s_spi_clock_hz = 0;
    }

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_add_spi_device(UWB_DW3000_SPI_BOOT_CLOCK_HZ), TAG,
        "restore boot SPI clock failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_verify_spi_link(expected_device_id, 1), TAG,
        "boot SPI fallback verification failed");
    ESP_LOGW(TAG, "DW3000 continues with boot SPI clock %lu Hz",
             (unsigned long)s_spi_clock_hz);
    return ESP_OK;
}

static esp_err_t uwb_dw3000_enable_operational_spi(uint32_t expected_device_id)
{
    ESP_RETURN_ON_ERROR(spi_bus_remove_device(s_spi), TAG,
                        "remove boot SPI device failed");
    s_spi = NULL;
    s_spi_clock_hz = 0;

    esp_err_t err =
        uwb_dw3000_add_spi_device(UWB_DW3000_SPI_OPERATION_REQUEST_HZ);
    if (err == ESP_OK) {
        err = uwb_dw3000_verify_spi_link(expected_device_id,
                                         UWB_DW3000_SPI_VERIFY_READS);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "DW3000 operational SPI verified: %lu DEV_ID reads at %lu Hz",
                 (unsigned long)UWB_DW3000_SPI_VERIFY_READS,
                 (unsigned long)s_spi_clock_hz);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "DW3000 operational SPI rejected, restoring boot clock");
    return uwb_dw3000_restore_boot_spi(expected_device_id);
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
    if (s_rx_double_buffer_enabled) {
        if (timestamp == NULL) {
            return ESP_ERR_INVALID_ARG;
        }

        const uint8_t base = s_rx_double_buffer_index == 0U
                                 ? DW3000_REG_DOUBLE_BUFFER_DIAG
                                 : DW3000_REG_INDIRECT_POINTER_B;
        const uint8_t time_sub = s_rx_double_buffer_index == 0U
                                     ? DW3000_DOUBLE_BUFFER_0_RX_TIME_SUB
                                     : DW3000_DOUBLE_BUFFER_1_RX_TIME_OFFSET;
        uint8_t low[4] = {0};
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_read_bytes(base, time_sub, low, sizeof(low)), TAG,
            "buffered RX timestamp low read failed");

        const uint32_t low32 = ((uint32_t)low[0]) |
                               ((uint32_t)low[1] << 8U) |
                               ((uint32_t)low[2] << 16U) |
                               ((uint32_t)low[3] << 24U);
        if (s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
            return uwb_flex_tdoa_extend_rx_timestamp32(low32, timestamp);
        }

        uint32_t system_time_word = 0;
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW,
                              DW3000_SYS_TIME_SUB, &system_time_word),
            TAG, "RX timestamp reference read failed");
        const uint64_t reference = (uint64_t)system_time_word << 8U;

        const int32_t delta = (int32_t)(low32 - (uint32_t)reference);
        *timestamp = (reference + (int64_t)delta) &
                     UWB_DW3000_TIMESTAMP_MASK;
        return ESP_OK;
    }

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

static esp_err_t uwb_dw3000_read_clock_offset_raw(int32_t *clock_offset_raw,
                                                   bool *from_cia)
{
    if (clock_offset_raw == NULL || from_cia == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_rx_double_buffer_enabled) {
        const uint8_t base = s_rx_double_buffer_index == 0U
                                 ? DW3000_REG_DOUBLE_BUFFER_DIAG
                                 : DW3000_REG_INDIRECT_POINTER_B;
        const uint8_t sub =
            s_rx_double_buffer_index == 0U
                ? DW3000_DOUBLE_BUFFER_0_CIA_DIAG_0_SUB
                : DW3000_DOUBLE_BUFFER_1_CIA_DIAG_0_OFFSET;
        uint8_t bytes[2] = {0};
        ESP_RETURN_ON_ERROR(uwb_dw3000_read_bytes(base, sub, bytes,
                                                   sizeof(bytes)),
                            TAG, "buffered clock offset read failed");
        const uint16_t raw =
            (uint16_t)(((uint16_t)bytes[0]) | ((uint16_t)bytes[1] << 8U));
        *clock_offset_raw = uwb_dw3000_sign_extend(raw & 0x1FFFU, 13U);
        *from_cia = true;
        return ESP_OK;
    }

    uint32_t raw = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_DRX, DW3000_DRX_CAR_INT_SUB, &raw), TAG,
        "clock offset read failed");
    *clock_offset_raw = uwb_dw3000_sign_extend(
        raw & DW3000_CLOCK_OFFSET_RAW_MASK, DW3000_CLOCK_OFFSET_RAW_BITS);
    *from_cia = false;
    return ESP_OK;
}

static double uwb_dw3000_clock_offset_ratio(int32_t clock_offset_raw)
{
    const double factor = uwb_dw3000_runtime_radio_rf_channel_bit() == 0
                              ? DW3000_CLOCK_OFFSET_CH5_FACTOR
                              : DW3000_CLOCK_OFFSET_CH9_FACTOR;
    return (double)clock_offset_raw * factor;
}

static double uwb_dw3000_cia_clock_offset_ratio(int32_t clock_offset_raw)
{
    return flextdoa_dw3000_cia_scale_delta((int16_t)clock_offset_raw);
}

static double uwb_dw3000_remote_interval_in_local_dtu(
    double remote_interval_dtu, double clock_offset_ratio)
{
    // uwb_dw3000_clock_offset_ratio() already applies the channel-specific
    // negative DRX_CAR_INT scale. Reciprocal anchor measurements on hardware
    // converge only when that converted value is applied with (1 + ratio);
    // applying the API-guide sign a second time makes the two directions
    // diverge symmetrically by metres for a 5 ms reply interval.
    return remote_interval_dtu * (1.0 + clock_offset_ratio);
}

static uint64_t uwb_dw3000_add_timestamp_delta(uint64_t timestamp,
                                               uint64_t delta)
{
    return (timestamp + delta) & UWB_DW3000_TIMESTAMP_MASK;
}

static uint64_t uwb_dw3000_sub_timestamp_delta(uint64_t timestamp,
                                               uint64_t delta)
{
    return (timestamp - delta) & UWB_DW3000_TIMESTAMP_MASK;
}

static uint64_t uwb_dw3000_ms_to_dtu(uint32_t delay_ms)
{
    const double delay_seconds = (double)delay_ms / 1000.0;
    return (uint64_t)((delay_seconds / UWB_DW3000_TIME_UNIT_SECONDS) + 0.5);
}

static uint64_t uwb_dw3000_us_to_dtu(uint32_t delay_us)
{
    const double delay_seconds = (double)delay_us / 1000000.0;
    return (uint64_t)((delay_seconds / UWB_DW3000_TIME_UNIT_SECONDS) + 0.5);
}

static int64_t uwb_dw3000_dtu_to_us(uint64_t interval_dtu)
{
    return (int64_t)(((double)interval_dtu *
                      UWB_DW3000_TIME_UNIT_SECONDS * 1000000.0) +
                     0.5);
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
        uwb_dw3000_write_u32_len(DW3000_REG_RDB_STATUS,
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

static bool uwb_dw3000_payload_is_flextdoa_localization(
    const uint8_t *payload, size_t payload_len,
    enum flextdoa_message_type expected_type)
{
    if (payload == NULL || payload_len < FLEXTDOA_PACKET_FIXED_SIZE ||
        (expected_type != FLEXTDOA_MESSAGE_REQUEST &&
         expected_type != FLEXTDOA_MESSAGE_RESPONSE)) {
        return false;
    }
    struct flextdoa_packet packet = {0};
    return flextdoa_decode_packet(payload, payload_len, &packet) &&
           packet.type == expected_type;
}

static bool uwb_dw3000_should_capture_rx_diagnostics(const uint8_t *payload,
                                                     size_t payload_len)
{
#if APP_UWB_DIAGNOSTICS_ENABLED
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL &&
        config->runtime_mode == APP_RUNTIME_MODE_UWB_RANGING) {
        /*
         * Native DS-TWR has a delayed TX deadline immediately after every
         * useful RX frame. Reading the full CIA diagnostic register set and
         * forwarding an ESP_LOG line here consumed several milliseconds on
         * the field modules, enough to make an otherwise valid 2-5 ms reply
         * late. Keep RF diagnostics for the dedicated distance-test modes;
         * Native DS exposes its timestamps, range and pipeline counters after
         * the exchange has completed instead.
         */
        return false;
    }

    if (!uwb_dw3000_payload_is_distance_frame(payload, payload_len) ||
        APP_UWB_DIAGNOSTICS_LOG_EVERY == 0) {
        return false;
    }

    const uint8_t frame_type = payload[5];
    if (frame_type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ ||
        frame_type == UWB_DISTANCE_FRAME_FLEX_TDOA_RESP ||
        frame_type == UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL ||
        frame_type == UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP ||
        frame_type == UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL) {
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

static esp_err_t uwb_dw3000_read_rdb_status(uint8_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return uwb_dw3000_read_bytes(DW3000_REG_RDB_STATUS,
                                 DW3000_RDB_STATUS_SUB, status, 1);
}

static uint8_t uwb_dw3000_current_rdb_good_mask(void)
{
    return s_rx_double_buffer_index == 0U
               ? DW3000_RDB_BUFFER_0_GOOD_MASK
               : DW3000_RDB_BUFFER_1_GOOD_MASK;
}

static uint8_t uwb_dw3000_current_rdb_cia_done_mask(void)
{
    return s_rx_double_buffer_index == 0U
               ? DW3000_RDB_BUFFER_0_CIA_DONE_MASK
               : DW3000_RDB_BUFFER_1_CIA_DONE_MASK;
}

static uint8_t uwb_dw3000_current_rdb_clear_mask(void)
{
    return s_rx_double_buffer_index == 0U
               ? DW3000_RDB_BUFFER_0_CLEAR_MASK
               : DW3000_RDB_BUFFER_1_CLEAR_MASK;
}

static esp_err_t uwb_dw3000_release_rx_double_buffer(void)
{
    // Qorvo's double-buffer sequence is read data, clear the processed RDB
    // status and the corresponding RX-good events, then toggle the host
    // buffer pointer. A global CMD_CLR_IRQS here can erase unrelated TX state.
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_RDB_STATUS,
                                 DW3000_RDB_STATUS_SUB,
                                 uwb_dw3000_current_rdb_clear_mask(), 1),
        TAG, "double-buffer RDB_STATUS clear failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW,
                                 DW3000_SYS_STATUS_SUB,
                                 DW3000_STATUS_RX_GOOD_CLEAR_MASK, 4),
        TAG, "double-buffer RX-good status clear failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_DB_TOGGLE), TAG,
                        "double-buffer release failed");
    s_rx_double_buffer_index ^= 1U;
    return ESP_OK;
}

static esp_err_t uwb_dw3000_release_pending_flex_request(void)
{
    if (!s_flex_tdoa_anchor_request_buffer_pending) {
        return ESP_OK;
    }
    const esp_err_t err = uwb_dw3000_release_rx_double_buffer();
    if (err == ESP_OK) {
        s_flex_tdoa_anchor_request_buffer_pending = false;
    }
    return err;
}

static esp_err_t uwb_dw3000_configure_high_rate_rx_double_buffer(void)
{
    ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                        "stop RX before double-buffer setup failed");
    s_rx_armed = false;

    // Seed on the first received frame, not here: the distributed bootstrap
    // listen interval is longer than the low-32 timestamp wrap.
    s_flex_tdoa_rx_timestamp_reference_valid = false;
    s_flex_tdoa_rx_timestamp_reference = 0;
    s_flex_tdoa_rx_timestamp_reference_host_us = 0;

    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_INDIRECT_CTRL,
                                 DW3000_INDIRECT_ADDR_B_SUB,
                                 DW3000_DOUBLE_BUFFER_1_DIAG_BASE, 4),
        TAG, "double-buffer indirect base setup failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_INDIRECT_CTRL,
                                 DW3000_ADDR_OFFSET_B_SUB,
                                 DW3000_DOUBLE_BUFFER_1_DIAG_OFFSET, 4),
        TAG, "double-buffer indirect offset setup failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(DW3000_REG_CIA_3, DW3000_SUB_NONE, 0,
                              DW3000_CIA_CONF_DIAGNOSTIC_OFF_MASK),
        TAG, "double-buffer full CIA diagnostics disable failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_RDB_STATUS,
                                 DW3000_RDB_DIAG_MODE_SUB,
                                 DW3000_CIA_DIAG_LOG_MIN >> 1U, 1),
        TAG, "double-buffer diagnostic set setup failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_update_u32(
            DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_CFG_SUB,
            DW3000_SYS_CFG_DIS_DRXB_BIT_MASK |
                DW3000_SYS_CFG_RXAUTR_BIT_MASK,
            0),
        TAG, "double-buffer SYS_CFG setup failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_len(DW3000_REG_RDB_STATUS,
                                 DW3000_RDB_STATUS_SUB, 0xFFU, 1),
        TAG, "double-buffer status reset failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                        "double-buffer initial SYS_STATUS clear failed");

    uint32_t sys_cfg = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_CFG_SUB,
                          &sys_cfg),
        TAG, "double-buffer SYS_CFG verification read failed");
    if ((sys_cfg & DW3000_SYS_CFG_DIS_DRXB_BIT_MASK) != 0U ||
        (sys_cfg & DW3000_SYS_CFG_RXAUTR_BIT_MASK) != 0U) {
        ESP_LOGE(TAG, "DW3000 double-buffer verification failed SYS_CFG=0x%08lx",
                 (unsigned long)sys_cfg);
        return ESP_ERR_INVALID_STATE;
    }

    s_rx_double_buffer_index = 0;
    s_flex_tdoa_anchor_request_buffer_pending = false;
    s_rx_double_buffer_resync_count = 0;
    s_rx_double_buffer_enabled = true;
    ESP_LOGI(TAG,
             "DW3000 high-rate RX double buffer enabled with manual early re-arm mode=%u",
             (unsigned)s_runtime_mode);
    return ESP_OK;
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
    /* COUNT0..COUNT7 occupy one register-file span (with reserved holes).
     * Reading the span in one SPI transaction keeps live diagnostics from
     * adding eight separate command/header latencies to the RX task. */
    uint8_t raw[(DW3000_EVC_COUNT7_SUB + sizeof(uint32_t)) -
                DW3000_EVC_COUNT0_SUB] = {0};
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read_bytes(DW3000_REG_DIG_DIAG,
                              DW3000_EVC_COUNT0_SUB, raw, sizeof(raw)),
        TAG, "EVC counter span read failed");
#define UWB_EVC_RAW32(sub)                                                \
    uwb_distance_get_u32(raw, (size_t)((sub) - DW3000_EVC_COUNT0_SUB))
    const uint32_t count0 = UWB_EVC_RAW32(DW3000_EVC_COUNT0_SUB);
    const uint32_t count1 = UWB_EVC_RAW32(DW3000_EVC_COUNT1_SUB);
    const uint32_t count2 = UWB_EVC_RAW32(DW3000_EVC_COUNT2_SUB);
    const uint32_t count3 = UWB_EVC_RAW32(DW3000_EVC_COUNT3_SUB);
    const uint32_t count4 = UWB_EVC_RAW32(DW3000_EVC_COUNT4_SUB);
    const uint32_t count5 = UWB_EVC_RAW32(DW3000_EVC_COUNT5_SUB);
    const uint32_t count6 = UWB_EVC_RAW32(DW3000_EVC_COUNT6_SUB);
    const uint32_t count7 = UWB_EVC_RAW32(DW3000_EVC_COUNT7_SUB);
#undef UWB_EVC_RAW32

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

    uint8_t finfo_base = DW3000_REG_GEN_CFG_AES_LOW;
    uint8_t finfo_sub = DW3000_RX_FINFO_SUB;
    uint8_t payload_base = DW3000_REG_RX_BUFFER_0;
    if (s_rx_double_buffer_enabled) {
        if (s_rx_double_buffer_index == 0U) {
            finfo_base = DW3000_REG_DOUBLE_BUFFER_DIAG;
            finfo_sub = DW3000_DOUBLE_BUFFER_0_RX_FINFO_SUB;
        } else {
            finfo_base = DW3000_REG_INDIRECT_POINTER_B;
            finfo_sub = DW3000_DOUBLE_BUFFER_1_RX_FINFO_OFFSET;
            payload_base = DW3000_REG_RX_BUFFER_1;
        }
    }

    uint32_t rx_finfo = 0;
    ESP_RETURN_ON_ERROR(uwb_dw3000_read32(finfo_base, finfo_sub, &rx_finfo),
                        TAG, "RX_FINFO read failed");

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
        uwb_dw3000_read_bytes(payload_base, DW3000_SUB_NONE, payload, read_len),
        TAG, "RX buffer read failed");
    *payload_len = read_len;
    return ESP_OK;
}

// Adjacent FlexTDOA frames are separated by much less than the 67 ms wrap of
// RX_TIME's low 32 bits. Extending every timestamp from the previous one is
// therefore unambiguous and avoids the occasionally corrupt upper byte in the
// double-buffer diagnostic set.
static esp_err_t uwb_flex_tdoa_extend_rx_timestamp32(
    uint32_t low32, uint64_t *timestamp)
{
    if (timestamp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t now_us = esp_timer_get_time();
    if (!s_flex_tdoa_rx_timestamp_reference_valid ||
        now_us - s_flex_tdoa_rx_timestamp_reference_host_us >
            UWB_FLEX_TDOA_RX_EPOCH_MAX_GAP_US) {
        uint32_t system_time_word = 0;
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW,
                              DW3000_SYS_TIME_SUB, &system_time_word),
            TAG, "FlexTDOA radio-time recovery seed failed");
        s_flex_tdoa_rx_timestamp_reference =
            ((uint64_t)system_time_word << 8U) &
            UWB_DW3000_TIMESTAMP_MASK;
        s_flex_tdoa_rx_timestamp_reference_valid = true;
    }

    const int32_t delta =
        (int32_t)(low32 - (uint32_t)s_flex_tdoa_rx_timestamp_reference);
    s_flex_tdoa_rx_timestamp_reference =
        (s_flex_tdoa_rx_timestamp_reference + (int64_t)delta) &
        UWB_DW3000_TIMESTAMP_MASK;
    s_flex_tdoa_rx_timestamp_reference_host_us = now_us;
    *timestamp = s_flex_tdoa_rx_timestamp_reference;
    return ESP_OK;
}

// The minimum double-buffer diagnostic set is contiguous: RX_FINFO starts at
// byte 0, RX_TIME at byte 4, and CIA_DIAG_0 at byte 12. Payload and metadata
// are read in two compact SPI bursts.

static esp_err_t uwb_dw3000_read_flex_buffered_metadata(
    struct uwb_dw3000_rx_frame *frame, uint16_t *frame_len)
{
    if (frame == NULL || frame_len == NULL || !s_rx_double_buffer_enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t diag_base = s_rx_double_buffer_index == 0U
                                  ? DW3000_REG_DOUBLE_BUFFER_DIAG
                                  : DW3000_REG_INDIRECT_POINTER_B;
    uint8_t metadata[DW3000_DOUBLE_BUFFER_FLEX_META_LEN] = {0};
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read_bytes(diag_base, 0, metadata, sizeof(metadata)), TAG,
        "FlexTDOA buffered metadata read failed");

    const uint32_t rx_finfo = ((uint32_t)metadata[0]) |
                              ((uint32_t)metadata[1] << 8U) |
                              ((uint32_t)metadata[2] << 16U) |
                              ((uint32_t)metadata[3] << 24U);
    *frame_len = (uint16_t)(rx_finfo & DW3000_RX_FINFO_RXFLEN_MASK);

    const uint32_t rx_time_low32 = ((uint32_t)metadata[4]) |
                                   ((uint32_t)metadata[5] << 8U) |
                                   ((uint32_t)metadata[6] << 16U) |
                                   ((uint32_t)metadata[7] << 24U);
    ESP_RETURN_ON_ERROR(
        uwb_flex_tdoa_extend_rx_timestamp32(rx_time_low32,
                                            &frame->rx_timestamp),
        TAG, "FlexTDOA buffered RX timestamp extension failed");

    const uint16_t raw =
        (uint16_t)(((uint16_t)metadata[12]) |
                   ((uint16_t)metadata[13] << 8U));
    frame->clock_offset_raw =
        uwb_dw3000_sign_extend(raw & 0x1FFFU, 13U);
    frame->clock_offset_from_cia = true;
    frame->clock_offset_valid = false;
    return ESP_OK;
}

static esp_err_t uwb_dw3000_read_flex_buffered_frame(
    struct uwb_dw3000_rx_frame *frame)
{
    if (frame == NULL || !s_rx_double_buffer_enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t frame_len = 0;
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read_flex_buffered_metadata(frame, &frame_len), TAG,
        "FlexTDOA buffered metadata unavailable");
    if (frame_len <= 2U) {
        frame->payload_len = 0;
        return ESP_OK;
    }

    uint16_t read_len = (uint16_t)(frame_len - 2U);
    if (read_len > UWB_DW3000_PAYLOAD_LEN) {
        read_len = UWB_DW3000_PAYLOAD_LEN;
    }
    const uint8_t payload_base = s_rx_double_buffer_index == 0U
                                     ? DW3000_REG_RX_BUFFER_0
                                     : DW3000_REG_RX_BUFFER_1;
    memset(frame->payload, 0, sizeof(frame->payload));
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_read_bytes(payload_base, DW3000_SUB_NONE, frame->payload,
                              read_len),
        TAG, "FlexTDOA RX buffer read failed");
    frame->payload_len = read_len;
    /*
     * RX-good does not imply CIADONE. The caller validates RDB_STATUS after
     * identifying a response and re-reads CIA_DIAG_0 if CIA completed while
     * the metadata and payload were being copied.
     */
    frame->clock_offset_valid = false;
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
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
    };
    transaction.tx_data[0] = header;
    return spi_device_polling_transmit(s_spi, &transaction);
}

static esp_err_t uwb_dw3000_clear_status(void)
{
    return uwb_dw3000_write_u32_len(DW3000_REG_GEN_CFG_AES_LOW, 0x44,
                                    DW3000_STATUS_CLEAR_MASK, 4);
}

static void uwb_micro_timer_wait_for_event_until(int64_t deadline_us)
{
    int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0) {
        return;
    }

#if APP_UWB_IRQ_ENABLED
    if (s_irq_enabled &&
        uwb_calibration_timer_start() == ESP_OK) {
        uint64_t timer_now_us = 0;
        if (uwb_calibration_timer_get_us(&timer_now_us) == ESP_OK) {
            remaining_us = deadline_us - esp_timer_get_time();
            if (remaining_us <= 0) {
                return;
            }

            const gptimer_alarm_config_t alarm_config = {
                .alarm_count = timer_now_us + (uint64_t)remaining_us,
                .reload_count = 0,
                .flags = {
                    .auto_reload_on_alarm = false,
                },
            };
            s_calibration_timer_wait_task = xTaskGetCurrentTaskHandle();
            if (gptimer_set_alarm_action(s_calibration_timer,
                                         &alarm_config) == ESP_OK) {
                uint64_t armed_timer_us = 0;
                if (uwb_calibration_timer_get_us(&armed_timer_us) != ESP_OK ||
                    armed_timer_us >= alarm_config.alarm_count ||
                    esp_timer_get_time() >= deadline_us) {
                    (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
                    s_calibration_timer_wait_task = NULL;
                    return;
                }
                /* Either the DW3000 IRQ or the 1 MHz GPTimer alarm wakes the
                 * same radio task. One RTOS tick is only a safety backstop;
                 * it is no longer the requested sub-millisecond deadline. */
                (void)ulTaskNotifyTake(pdTRUE, 1U);
                (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
                s_calibration_timer_wait_task = NULL;
                return;
            }
            s_calibration_timer_wait_task = NULL;
        }
    }
#endif

    /* IRQ-less/failure fallback: poll without sleeping past the deadline. */
    remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us > 0) {
        const uint32_t delay_us =
            (uint32_t)(remaining_us > 1000 ? 1000 : remaining_us);
        esp_rom_delay_us(delay_us);
    }
}

static void uwb_gptimer_delay_us(uint32_t delay_us)
{
    if (delay_us == 0U) {
        return;
    }

    if (uwb_calibration_timer_start() != ESP_OK) {
        esp_rom_delay_us(delay_us);
        return;
    }

    uint64_t start_us = 0;
    if (uwb_calibration_timer_get_us(&start_us) != ESP_OK) {
        esp_rom_delay_us(delay_us);
        return;
    }
    const uint64_t deadline_us = start_us + (uint64_t)delay_us;

    while (true) {
        uint64_t now_us = 0;
        if (uwb_calibration_timer_get_us(&now_us) != ESP_OK) {
            esp_rom_delay_us(delay_us);
            return;
        }
        if (now_us >= deadline_us) {
            return;
        }

        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = deadline_us,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = false,
            },
        };
        s_calibration_timer_wait_task = xTaskGetCurrentTaskHandle();
        if (gptimer_set_alarm_action(s_calibration_timer, &alarm_config) !=
            ESP_OK) {
            s_calibration_timer_wait_task = NULL;
            esp_rom_delay_us((uint32_t)(deadline_us - now_us));
            return;
        }

        uint64_t armed_us = 0;
        if (uwb_calibration_timer_get_us(&armed_us) != ESP_OK ||
            armed_us >= deadline_us) {
            (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
            s_calibration_timer_wait_task = NULL;
            return;
        }

        const uint64_t remaining_us = deadline_us - armed_us;
        TickType_t ticks = pdMS_TO_TICKS(
            (uint32_t)((remaining_us + 999U) / 1000U));
        if (ticks == 0U) {
            ticks = 1U;
        }
        (void)ulTaskNotifyTake(pdTRUE, ticks);
        (void)gptimer_set_alarm_action(s_calibration_timer, NULL);
        s_calibration_timer_wait_task = NULL;
    }
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
    const uint8_t preamble_len =
        uwb_dw3000_runtime_radio_preamble_len_code();
    const uint8_t preamble_code = uwb_dw3000_runtime_radio_preamble_code();
    const uint8_t data_rate = uwb_dw3000_runtime_radio_data_rate();
    if (uwb_dw3000_runtime_radio_rf_channel_bit() > 1 ||
        APP_UWB_RADIO_SFD_TYPE > 3 ||
        preamble_code > 31 || preamble_len > 0x0F ||
        data_rate > APP_UWB_RADIO_BR_6M8) {
        return ESP_ERR_INVALID_ARG;
    }

    if (uwb_dw3000_preamble_len_symbols(preamble_len) == 0) {
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

    const uint8_t channel = uwb_dw3000_runtime_radio_rf_channel_bit();
    const uint8_t channel_number = uwb_dw3000_runtime_radio_channel();
    const uint8_t preamble_len =
        uwb_dw3000_runtime_radio_preamble_len_code();
    const uint8_t preamble_code = uwb_dw3000_runtime_radio_preamble_code();
    const uint8_t pac = uwb_dw3000_runtime_radio_pac();
    const uint8_t datarate = uwb_dw3000_runtime_radio_data_rate();
    const uint8_t phr_mode = APP_UWB_RADIO_PHR_MODE;
    const uint8_t phr_rate = uwb_dw3000_runtime_radio_phr_rate();
    const uint8_t sfd_type = uwb_dw3000_runtime_radio_sfd_type();
    const uint16_t sfd_timeout =
        uwb_dw3000_runtime_radio_sfd_timeout();

    ESP_LOGI(TAG,
             "DW3000 radio profile: profile=%u phy=%u channel=%u rf_bit=%u plen=%u(code=0x%02x) pcode=%u pac=%u br=%s phr_mode=%u phr_rate=%u sfd=%u sfd_timeout=%u",
             (unsigned)uwb_dw3000_runtime_radio_profile(),
             (unsigned)uwb_dw3000_runtime_radio_phy_mode(),
             (unsigned)channel_number, (unsigned)channel,
             (unsigned)uwb_dw3000_preamble_len_symbols(preamble_len),
             (unsigned)preamble_len, (unsigned)preamble_code, (unsigned)pac,
             uwb_dw3000_radio_data_rate_name(datarate), (unsigned)phr_mode,
             (unsigned)phr_rate, (unsigned)sfd_type,
             (unsigned)sfd_timeout);
    ESP_LOGI(TAG,
             "DW3000 RF profile: pg=0x%02x power=0x%08lx rf_tx2=0x%08lx pll=0x%04x pll_final=0x%04x",
             (unsigned)APP_UWB_RADIO_TX_PG_DELAY,
             (unsigned long)APP_UWB_RADIO_TX_POWER,
             (unsigned long)uwb_dw3000_runtime_rf_tx_ctrl_2(),
             (unsigned)APP_UWB_RADIO_PLL_CFG,
             (unsigned)uwb_dw3000_runtime_pll_cfg_final());

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
    s_tx_fctrl_base =
        tx_fctrl & ~(DW3000_TX_FCTRL_TXB_OFFSET_MASK |
                     DW3000_TX_FCTRL_TR_MASK |
                     DW3000_TX_FCTRL_TXFLEN_MASK);
    s_tx_fctrl_base_valid = true;
    s_tx_fctrl_payload_len = SIZE_MAX;

    ESP_RETURN_ON_ERROR(uwb_dw3000_write_u32_auto(DW3000_REG_DRX, 0x02,
                                                  sfd_timeout),
                        TAG, "DRX 0x02 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RF_CONF, 0x1C,
                                  uwb_dw3000_runtime_rf_tx_ctrl_2()),
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
                                  uwb_dw3000_runtime_dgc_lut(0)), TAG,
        "DGC_LUT_0 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x3C,
                                  uwb_dw3000_runtime_dgc_lut(1)), TAG,
        "DGC_LUT_1 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x40,
                                  uwb_dw3000_runtime_dgc_lut(2)), TAG,
        "DGC_LUT_2 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x44,
                                  uwb_dw3000_runtime_dgc_lut(3)), TAG,
        "DGC_LUT_3 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x48,
                                  uwb_dw3000_runtime_dgc_lut(4)), TAG,
        "DGC_LUT_4 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x4C,
                                  uwb_dw3000_runtime_dgc_lut(5)), TAG,
        "DGC_LUT_5 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x50,
                                  uwb_dw3000_runtime_dgc_lut(6)), TAG,
        "DGC_LUT_6 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_RX_TUNE, 0x18, 0xE5E5), TAG,
        "THR_64 write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(
            DW3000_REG_DRX, 0x00,
            ((uint32_t)uwb_dw3000_runtime_radio_sfd_timeout() << 16U) |
                0x101CU | uwb_dw3000_runtime_radio_pac()),
        TAG,
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
                                  uwb_dw3000_runtime_rf_tx_ctrl_2()),
        TAG, "RF_TX_CTRL_2 final write failed");
    ESP_RETURN_ON_ERROR(
        uwb_dw3000_write_u32_auto(DW3000_REG_FS_CTRL, 0x00,
                                  uwb_dw3000_runtime_pll_cfg_final()),
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
    if (!s_tx_fctrl_base_valid) {
        uint32_t tx_fctrl = 0;
        ESP_RETURN_ON_ERROR(
            uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW,
                              DW3000_TX_FCTRL_SUB, &tx_fctrl),
            TAG, "TX_FCTRL read before length failed");
        s_tx_fctrl_base =
            tx_fctrl & ~(DW3000_TX_FCTRL_TXB_OFFSET_MASK |
                         DW3000_TX_FCTRL_TR_MASK |
                         DW3000_TX_FCTRL_TXFLEN_MASK);
        s_tx_fctrl_base_valid = true;
        s_tx_fctrl_payload_len = SIZE_MAX;
    }

    if (s_tx_fctrl_payload_len == payload_len) {
        return ESP_OK;
    }

    const uint32_t frame_len = (uint32_t)payload_len + 2U;
    const uint32_t tx_fctrl =
        s_tx_fctrl_base | (frame_len & DW3000_TX_FCTRL_TXFLEN_MASK);
    const esp_err_t err = uwb_dw3000_write_u32_len(
        DW3000_REG_GEN_CFG_AES_LOW, DW3000_TX_FCTRL_SUB, tx_fctrl, 4);
    if (err == ESP_OK) {
        s_tx_fctrl_payload_len = payload_len;
    }
    return err;
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
    const bool profile =
        s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR;
    const int64_t started_us = profile ? esp_timer_get_time() : 0;
    esp_err_t err = uwb_dw3000_clear_status();
    if (err == ESP_OK) {
        err = uwb_dw3000_fast_command(DW3000_CMD_RX);
    }
    if (profile) {
        uwb_passive_ds_record_stage(
            UWB_PASSIVE_DS_RUNTIME_STAGE_RX_REARM, started_us,
            err == ESP_OK);
        if (err != ESP_OK) {
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_RX_REARM_FAILURE);
        }
    }
    ESP_RETURN_ON_ERROR(err, TAG, "RX re-arm failed");
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

static uint32_t uwb_dw3000_remaining_ms(int64_t start_us,
                                        uint32_t timeout_ms)
{
    const int64_t elapsed_us = esp_timer_get_time() - start_us;
    const int64_t timeout_us = (int64_t)timeout_ms * 1000LL;
    if (elapsed_us >= timeout_us) {
        return 0;
    }
    const int64_t remaining_us = timeout_us - elapsed_us;
    return (uint32_t)((remaining_us + 999LL) / 1000LL);
}

static esp_err_t uwb_dw3000_receive_frame_until(
    struct uwb_dw3000_rx_frame *frame, uint32_t timeout_ms,
    int64_t precise_deadline_us)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(frame, 0, sizeof(*frame));

    if (!s_rx_armed) {
        ESP_RETURN_ON_ERROR(uwb_dw3000_arm_rx(), TAG,
                            "distance RX arm failed");
    }

    const int64_t start = esp_timer_get_time();
    while (true) {
        if (uwb_dw3000_runtime_switch_pending()) {
            (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
            (void)uwb_dw3000_clear_status();
            s_rx_armed = false;
            return ESP_ERR_INVALID_STATE;
        }
        uint32_t status = 0;
        uint8_t rdb_status = 0;
        bool rx_good = false;
        if (s_rx_double_buffer_enabled) {
            ESP_RETURN_ON_ERROR(uwb_dw3000_read_rdb_status(&rdb_status), TAG,
                                "distance RDB_STATUS read failed");
            const uint8_t current_good = uwb_dw3000_current_rdb_good_mask();
            const uint8_t other_good =
                s_rx_double_buffer_index == 0U
                    ? DW3000_RDB_BUFFER_1_GOOD_MASK
                    : DW3000_RDB_BUFFER_0_GOOD_MASK;
            rx_good = (rdb_status & current_good) != 0U;
            if (!rx_good && (rdb_status & other_good) != 0U) {
                // Keep the host and DW3000 buffer pointers aligned. A local
                // index flip alone would read the other buffer without
                // releasing the current hardware buffer.
                ESP_RETURN_ON_ERROR(
                    uwb_dw3000_fast_command(DW3000_CMD_DB_TOGGLE), TAG,
                    "double-buffer pointer recovery toggle failed");
                s_rx_double_buffer_index ^= 1U;
                s_rx_double_buffer_resync_count++;
                rx_good = true;
                if (s_rx_double_buffer_resync_count <= 4U ||
                    (s_rx_double_buffer_resync_count % 100U) == 0U) {
                    ESP_LOGW(TAG,
                             "DW3000 RX double-buffer pointer resync "
                             "status=0x%02x next=%u count=%lu",
                             rdb_status, s_rx_double_buffer_index,
                             (unsigned long)s_rx_double_buffer_resync_count);
                }
            }
            if (!rx_good) {
                ESP_RETURN_ON_ERROR(
                    uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44,
                                      &status),
                    TAG, "distance SYS_STATUS read failed");
            }
        } else {
            ESP_RETURN_ON_ERROR(
                uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW, 0x44, &status),
                TAG, "distance SYS_STATUS read failed");
            rx_good = (status & DW3000_RX_GOOD_MASK) != 0;
        }

        if (rx_good) {
            frame->rx_host_time_us = esp_timer_get_time();
            frame->rx_buffer_index = s_rx_double_buffer_index;
            frame->rx_buffer_status = rdb_status;
            bool spi_bus_acquired = false;
            bool double_buffer_early_rearmed = false;
            if (s_rx_double_buffer_enabled) {
                const esp_err_t acquire_err =
                    spi_device_acquire_bus(s_spi, portMAX_DELAY);
                if (acquire_err != ESP_OK) {
                    return acquire_err;
                }
                spi_bus_acquired = true;

                // DW3000 does not support RXAUTR in double-buffer mode. In
                // FlexTDOA, restart RX before copying the occupied buffer so
                // the radio can receive into the other buffer concurrently.
                // CMD_DB_TOGGLE still releases this buffer only after its
                // metadata and payload have been copied completely.
                s_rx_armed = false;
                if (s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
                    const esp_err_t rearm_err =
                        uwb_dw3000_fast_command(DW3000_CMD_RX);
                    if (rearm_err != ESP_OK) {
                        spi_device_release_bus(s_spi);
                        s_rx_error_count++;
                        return rearm_err;
                    }
                    s_rx_armed = true;
                    double_buffer_early_rearmed = true;
                }
            }
            // RX_FINFO, RX_TIME and the payload are valid as soon as the
            // double buffer reports a good frame. FlexTDOA requests do not
            // need CIA diagnostics, so they can be dispatched immediately.
            const bool buffered_fast_path = s_rx_double_buffer_enabled;
            esp_err_t read_err = ESP_OK;
            esp_err_t ts_err = ESP_OK;
            if (buffered_fast_path) {
                read_err = uwb_dw3000_read_flex_buffered_frame(frame);
                ts_err = read_err;
            } else {
                read_err = uwb_dw3000_read_rx_payload(frame->payload,
                                                      &frame->payload_len);
                ts_err =
                    uwb_dw3000_read_rx_timestamp(&frame->rx_timestamp);
            }
            const bool flex_fast_rearm =
                !s_rx_double_buffer_enabled && read_err == ESP_OK &&
                s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA &&
                (uwb_dw3000_payload_is_flextdoa_localization(
                     frame->payload, frame->payload_len,
                     FLEXTDOA_MESSAGE_REQUEST) ||
                 uwb_dw3000_payload_is_flextdoa_localization(
                     frame->payload, frame->payload_len,
                     FLEXTDOA_MESSAGE_RESPONSE));
            const bool passive_multi_fast_rearm =
                !s_rx_double_buffer_enabled && read_err == ESP_OK &&
                s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR &&
                uwb_dw3000_payload_is_distance_frame(
                    frame->payload, frame->payload_len) &&
                (frame->payload[5] ==
                     UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP ||
                 frame->payload[5] ==
                     UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL);
            const bool passive_ds_frame =
                read_err == ESP_OK &&
                s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR &&
                uwb_dw3000_payload_is_distance_frame(
                    frame->payload, frame->payload_len) &&
                (frame->payload[5] ==
                     UWB_DISTANCE_FRAME_PASSIVE_DS_POLL ||
                 frame->payload[5] ==
                     UWB_DISTANCE_FRAME_PASSIVE_DS_RESP ||
                 frame->payload[5] ==
                     UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL);
            const int64_t passive_ds_cia_started_us =
                passive_ds_frame ? esp_timer_get_time() : 0;
            esp_err_t clock_err = ESP_OK;
            const bool clean_flex_response =
                read_err == ESP_OK &&
                uwb_dw3000_payload_is_flextdoa_localization(
                    frame->payload, frame->payload_len,
                    FLEXTDOA_MESSAGE_RESPONSE);
            const bool common_frame_needs_cfo =
                read_err == ESP_OK &&
                uwb_dw3000_payload_is_distance_frame(
                    frame->payload, frame->payload_len) &&
                frame->payload[5] != UWB_DISTANCE_FRAME_FLEX_TDOA_REQ;
            if (buffered_fast_path && clean_flex_response) {
                const uint8_t cia_done_mask =
                    uwb_dw3000_current_rdb_cia_done_mask();
                bool cia_ready =
                    (frame->rx_buffer_status & cia_done_mask) != 0U;
                if (!cia_ready) {
                    uint8_t updated_rdb_status = 0U;
                    clock_err =
                        uwb_dw3000_read_rdb_status(&updated_rdb_status);
                    frame->rx_buffer_status |= updated_rdb_status;
                    cia_ready = clock_err == ESP_OK &&
                                (updated_rdb_status & cia_done_mask) != 0U;
                    if (cia_ready) {
                        /* The metadata copy preceded CIADONE and may be stale. */
                        clock_err = uwb_dw3000_read_clock_offset_raw(
                            &frame->clock_offset_raw,
                            &frame->clock_offset_from_cia);
                    }
                }
                frame->clock_offset_valid =
                    cia_ready && clock_err == ESP_OK;
                if (!frame->clock_offset_valid) {
                    if (clock_err == ESP_OK) {
                        clock_err = ESP_ERR_NOT_FINISHED;
                    }
                }
            }
            if (!buffered_fast_path && !passive_multi_fast_rearm &&
                (common_frame_needs_cfo || clean_flex_response)) {
                bool cia_ready = true;
                if (s_rx_double_buffer_enabled) {
                    const uint8_t cia_done_mask =
                        uwb_dw3000_current_rdb_cia_done_mask();
                    cia_ready = (rdb_status & cia_done_mask) != 0U;
                    if (!cia_ready) {
                        uint8_t updated_rdb_status = 0;
                        clock_err = uwb_dw3000_read_rdb_status(
                            &updated_rdb_status);
                        cia_ready = clock_err == ESP_OK &&
                                    (updated_rdb_status & cia_done_mask) != 0U;
                        frame->rx_buffer_status |= updated_rdb_status;
                    }
                    if (!cia_ready) {
                        clock_err = ESP_ERR_NOT_FINISHED;
                    }
                }
                if (cia_ready) {
                    clock_err = uwb_dw3000_read_clock_offset_raw(
                        &frame->clock_offset_raw,
                        &frame->clock_offset_from_cia);
                    frame->clock_offset_valid = clock_err == ESP_OK;
                }
            }
            if (passive_ds_frame) {
                uwb_passive_ds_record_stage(
                    UWB_PASSIVE_DS_RUNTIME_STAGE_CIA_READ,
                    passive_ds_cia_started_us, clock_err == ESP_OK);
            }
            const bool flex_anchor_request =
                buffered_fast_path && read_err == ESP_OK &&
                s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA &&
                uwb_local_is_configured_anchor() &&
                uwb_dw3000_payload_is_flextdoa_localization(
                    frame->payload, frame->payload_len,
                    FLEXTDOA_MESSAGE_REQUEST);
            if (buffered_fast_path && !flex_anchor_request &&
                !double_buffer_early_rearmed) {
                // Non-Flex high-rate users retain the historical late re-arm.
                // FlexTDOA already restarted RX before reading this buffer.
                const esp_err_t rearm_err =
                    uwb_dw3000_fast_command(DW3000_CMD_RX);
                if (rearm_err != ESP_OK) {
                    if (spi_bus_acquired) {
                        spi_device_release_bus(s_spi);
                    }
                    s_rx_error_count++;
                    return rearm_err;
                }
                s_rx_armed = true;
            }
            esp_err_t diag_err = ESP_OK;
            memset(&frame->diagnostics, 0, sizeof(frame->diagnostics));
            if (read_err == ESP_OK &&
                uwb_dw3000_should_capture_rx_diagnostics(
                    frame->payload, frame->payload_len)) {
                diag_err = uwb_dw3000_read_rx_diagnostics(&frame->diagnostics);
            }
            if (s_rx_double_buffer_enabled) {
                // A FlexTDOA anchor has no useful RX work between REQ and its
                // scheduled RESP. Defer the three-operation buffer release so
                // delayed TX is armed first; the copied frame remains valid.
                s_flex_tdoa_anchor_request_buffer_pending =
                    flex_anchor_request;
                const esp_err_t release_err =
                    flex_anchor_request
                        ? ESP_OK
                        : uwb_dw3000_release_rx_double_buffer();
                if (spi_bus_acquired) {
                    spi_device_release_bus(s_spi);
                    spi_bus_acquired = false;
                }
                if (release_err != ESP_OK) {
                    ESP_LOGE(TAG,
                             "release after distance RX good failed: %s",
                             esp_err_to_name(release_err));
                    return release_err;
                }
            } else {
                ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                    "clear after distance RX good failed");
                s_rx_armed = false;
            }

            if (spi_bus_acquired) {
                spi_device_release_bus(s_spi);
            }

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
                frame->clock_offset_from_cia = false;
                frame->clock_offset_raw = 0;
            }
            if (diag_err != ESP_OK) {
                ESP_LOGD(TAG, "RX diagnostics read failed: %s",
                         esp_err_to_name(diag_err));
                memset(&frame->diagnostics, 0, sizeof(frame->diagnostics));
            }

            // Keep the radio listening while the ESP32 parses and processes a
            // FlexTDOA frame. A responder cancels this RX state immediately
            // before programming its own delayed transmission.
            if (flex_fast_rearm || passive_multi_fast_rearm) {
                const esp_err_t rearm_err = uwb_dw3000_arm_rx();
                if (rearm_err != ESP_OK) {
                    s_rx_error_count++;
                    return rearm_err;
                }
            }

            s_rx_count++;
            return ESP_OK;
        }

        if ((status & DW3000_RX_ERROR_MASK) != 0) {
            s_rx_error_count++;
            if (s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
                s_flex_tdoa_rx_errors_since_summary++;
                s_flex_tdoa_rx_error_status_since_summary |= status;
            } else {
                ESP_LOGW(TAG, "UWB distance RX error SYS_STATUS=0x%08lx",
                         (unsigned long)status);
            }
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after distance RX error failed");
            s_rx_armed = false;
            return s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA
                       ? ESP_ERR_INVALID_RESPONSE
                       : ESP_FAIL;
        }

        if ((status & DW3000_RX_TIMEOUT_MASK) != 0) {
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after distance RX timeout failed");
            s_rx_armed = false;
            return ESP_ERR_TIMEOUT;
        }

        if (s_flex_tdoa_schedule_alarm_fired ||
            (s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR &&
             s_passive_ds_schedule_alarm_fired)) {
            (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
            (void)uwb_dw3000_clear_status();
            s_rx_armed = false;
            return ESP_ERR_NOT_FINISHED;
        }

        const bool receive_timed_out =
            precise_deadline_us > 0
                ? esp_timer_get_time() >= precise_deadline_us
                : uwb_dw3000_remaining_ms(start, timeout_ms) == 0;
        if (receive_timed_out) {
            if (s_rx_double_buffer_enabled) {
                // A passive tag has no TX path that would recover a silently
                // idle receiver. In a healthy FlexTDOA frame a 5 ms receive
                // interval cannot expire without traffic, so use that event
                // as a bounded radio-state recovery point.
                (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
                (void)uwb_dw3000_clear_status();
                s_rx_armed = false;
                return ESP_ERR_TIMEOUT;
            }
            (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
            (void)uwb_dw3000_clear_status();
            s_rx_armed = false;
            return ESP_ERR_TIMEOUT;
        }

        if (precise_deadline_us > 0) {
            uwb_micro_timer_wait_for_event_until(precise_deadline_us);
        } else {
            uwb_dw3000_wait_for_event_or_delay(
                start, timeout_ms, UWB_DW3000_POLL_INTERVAL_MS);
        }
    }
}

static esp_err_t uwb_dw3000_receive_frame(struct uwb_dw3000_rx_frame *frame,
                                          uint32_t timeout_ms)
{
    return uwb_dw3000_receive_frame_until(frame, timeout_ms, 0);
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

static void uwb_distance_put_u32(uint8_t *payload, size_t offset, uint32_t value)
{
    payload[offset] = (uint8_t)(value & 0xFFU);
    payload[offset + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
    payload[offset + 2U] = (uint8_t)((value >> 16U) & 0xFFU);
    payload[offset + 3U] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t uwb_distance_get_u32(const uint8_t *payload, size_t offset)
{
    return ((uint32_t)payload[offset]) |
           ((uint32_t)payload[offset + 1U] << 8U) |
           ((uint32_t)payload[offset + 2U] << 16U) |
           ((uint32_t)payload[offset + 3U] << 24U);
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
    const bool common_frame =
        payload[0] == UWB_DISTANCE_FRAME_MAGIC_0 &&
        payload[1] == UWB_DISTANCE_FRAME_MAGIC_1 &&
        payload[2] == UWB_DISTANCE_FRAME_MAGIC_2 &&
        payload[3] == UWB_DISTANCE_FRAME_MAGIC_3 &&
        payload[4] == UWB_DISTANCE_FRAME_VERSION;
    if (!common_frame && s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
        struct flextdoa_packet packet = {0};
        if (!flextdoa_decode_packet(payload, rx_frame->payload_len,
                                    &packet) ||
            packet.source_id > UINT8_MAX) {
            return false;
        }
        memset(frame, 0, sizeof(*frame));
        frame->type = packet.type == FLEXTDOA_MESSAGE_REQUEST
                          ? UWB_DISTANCE_FRAME_FLEX_TDOA_REQ
                          : UWB_DISTANCE_FRAME_FLEX_TDOA_RESP;
        frame->source_id = (uint8_t)packet.source_id;
        frame->destination_id = UWB_DISTANCE_FRAME_BROADCAST_ID;
        frame->sequence = (uint16_t)packet.slot_id;
        frame->rx_timestamp = rx_frame->rx_timestamp;
        frame->rx_host_time_us = rx_frame->rx_host_time_us;
        frame->rx_buffer_index = rx_frame->rx_buffer_index;
        frame->rx_buffer_status = rx_frame->rx_buffer_status;
        frame->clock_offset_valid = rx_frame->clock_offset_valid;
        frame->clock_offset_from_cia = rx_frame->clock_offset_from_cia;
        frame->clock_offset_raw = rx_frame->clock_offset_raw;
        frame->diagnostics = rx_frame->diagnostics;
        frame->payload_len = rx_frame->payload_len;
        memcpy(frame->payload, payload, rx_frame->payload_len);
        return true;
    }
    if (!common_frame ||
        payload[0] != UWB_DISTANCE_FRAME_MAGIC_0 ||
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
    frame->rx_host_time_us = rx_frame->rx_host_time_us;
    frame->rx_buffer_index = rx_frame->rx_buffer_index;
    frame->rx_buffer_status = rx_frame->rx_buffer_status;
    frame->clock_offset_valid = rx_frame->clock_offset_valid;
    frame->clock_offset_from_cia = rx_frame->clock_offset_from_cia;
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
    case UWB_DISTANCE_FRAME_CAL_CMD:
        return "CAL_CMD";
    case UWB_DISTANCE_FRAME_CAL_SYNC:
        return "CAL_SYNC";
    case UWB_DISTANCE_FRAME_FLEX_TDOA_REQ:
        return "FLEX_TDOA_REQ";
    case UWB_DISTANCE_FRAME_FLEX_TDOA_RESP:
        return "FLEX_TDOA_RESP";
    case UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG:
        return "FLEX_TDOA_CONFIG";
    case UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY:
        return "FLEX_TDOA_GEOMETRY";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_POLL:
        return "PASSIVE_DS_POLL";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_RESP:
        return "PASSIVE_DS_RESP";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL:
        return "PASSIVE_DS_FINAL";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL:
        return "PASSIVE_DS_MULTI_POLL";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP:
        return "PASSIVE_DS_MULTI_RESP";
    case UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL:
        return "PASSIVE_DS_MULTI_FINAL";
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

    const int64_t start = esp_timer_get_time();

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

        if (!uwb_distance_parse_frame(&rx_frame, frame)) {
            s_rx_ignored_count++;
            ESP_LOGD(TAG, "Ignoring non-ranging frame len=%u ignored=%lu",
                     (unsigned)rx_frame.payload_len,
                     (unsigned long)s_rx_ignored_count);
            continue;
        }

        s_last_rx_source_id = frame->source_id;
        s_last_rx_sequence = frame->sequence;
        return ESP_OK;
    }
}

static esp_err_t uwb_distance_receive_matching(
    uint8_t expected_type, uint8_t expected_source_id, bool match_sequence,
    uint16_t expected_sequence, struct uwb_distance_frame *frame,
    uint32_t timeout_ms)
{
    const int64_t start = esp_timer_get_time();

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
        payload, UWB_DISTANCE_FRAME_POLL_LEN, auto_rx_delay_uus, rx_timeout_ms,
        &poll_tx_ts);
#else
    esp_err_t err =
        uwb_dw3000_send_payload(payload, UWB_DISTANCE_FRAME_POLL_LEN,
                                &poll_tx_ts);
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
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_FINAL, peer_id, sequence,
                             payload);
#if APP_UWB_DISTANCE_TEST_USE_DELAYED_TX
    const uint64_t final_tx_due = uwb_dw3000_add_timestamp_delta(
        resp_rx_ts, uwb_dw3000_ms_to_dtu(final_delay_ms));
    uint64_t final_tx_actual_ts = 0;
    err = uwb_dw3000_send_payload_delayed(payload, UWB_DISTANCE_FRAME_FINAL_LEN,
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
    err = uwb_dw3000_send_payload(payload, UWB_DISTANCE_FRAME_FINAL_LEN,
                                  &final_tx_ts);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS-TWR FINAL TX failed seq=%u peer=%u: %s",
                 (unsigned)sequence, (unsigned)peer_id, esp_err_to_name(err));
        return err;
    }

    uwb_gptimer_delay_us(report_delay_ms * 1000U);
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_REPORT, peer_id, sequence,
                             payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          resp_rx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          final_tx_ts);
#if APP_UWB_DISTANCE_TEST_AUTO_RX_AFTER_TX
    err = uwb_dw3000_send_payload_expect_rx(payload,
                                             UWB_DISTANCE_FRAME_REPORT_LEN,
                                             auto_rx_delay_uus, rx_timeout_ms,
                                             NULL);
#else
    err = uwb_dw3000_send_payload(payload, UWB_DISTANCE_FRAME_REPORT_LEN, NULL);
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

    if (uwb_distance_should_log_diagnostics(sequence)) {
        uwb_distance_log_rx_diagnostics(sequence, "RESP_RX",
                                        &response.diagnostics);
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

    uwb_gptimer_delay_us(config->distance_test_report_delay_ms * 1000U);
    uwb_distance_build_report2(measurement, payload);
    return uwb_dw3000_send_payload(payload, UWB_DISTANCE_FRAME_REPORT2_LEN,
                                   NULL);
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
        payload, UWB_DISTANCE_FRAME_RESP_LEN, resp_tx_due, auto_rx_delay_uus,
        rx_timeout_ms, &resp_tx_ts, &resp_tx_actual_ts);
#else
    esp_err_t err = uwb_dw3000_send_payload_delayed(
        payload, UWB_DISTANCE_FRAME_RESP_LEN, resp_tx_due, &resp_tx_ts,
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
        uwb_dw3000_send_payload(payload, UWB_DISTANCE_FRAME_RESP_LEN,
                                &resp_tx_ts);
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
    const size_t count = app_runtime_config_get_anchor_ids(
        ids, UWB_ANCHOR_SURVEY_MAX_ANCHORS);
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

static size_t uwb_anchor_survey_id_index(const uint8_t *ids, size_t count,
                                         uint8_t id)
{
    for (size_t i = 0; i < count; ++i) {
        if (ids[i] == id) {
            return i;
        }
    }
    return SIZE_MAX;
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

static void uwb_anchor_survey_build_command_type(
    uint8_t type, const struct uwb_anchor_survey_pair *pair,
    uint8_t slot_index, uint16_t sequence,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_distance_build_frame(type, pair->initiator_id, sequence, payload);
    payload[UWB_ANCHOR_SURVEY_CMD_INITIATOR_OFFSET] = pair->initiator_id;
    payload[UWB_ANCHOR_SURVEY_CMD_RESPONDER_OFFSET] = pair->responder_id;
    payload[UWB_ANCHOR_SURVEY_CMD_SLOT_OFFSET] = slot_index;
}

static void uwb_anchor_survey_build_command(
    const struct uwb_anchor_survey_pair *pair, uint8_t slot_index,
    uint16_t sequence, uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_anchor_survey_build_command_type(UWB_DISTANCE_FRAME_SURVEY_CMD, pair,
                                         slot_index, sequence, payload);
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

    const esp_err_t err = uwb_dw3000_send_payload(
        payload, UWB_ANCHOR_SURVEY_CMD_LEN, NULL);
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

    while (!uwb_dw3000_runtime_switch_pending()) {
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

static struct uwb_flex_tdoa_observation *
uwb_flex_tdoa_find_observation(
    struct uwb_flex_tdoa_observation *observations, uint32_t slot_id,
    uint16_t sequence, uint8_t initiator_id, uint8_t responder_id, bool create)
{
    struct uwb_flex_tdoa_observation *oldest = NULL;
    for (size_t i = 0; i < UWB_FLEX_TDOA_MAX_OBSERVATIONS; ++i) {
        struct uwb_flex_tdoa_observation *observation = &observations[i];
        if (observation->in_use && observation->slot_id == slot_id &&
            observation->initiator_id == initiator_id &&
            observation->responder_id == responder_id) {
            return observation;
        }
        if (!create) {
            continue;
        }
        if (!observation->in_use) {
            oldest = observation;
            break;
        }
        if (oldest == NULL ||
            (int32_t)(observation->updated_tick - oldest->updated_tick) < 0) {
            oldest = observation;
        }
    }

    if (!create || oldest == NULL) {
        return NULL;
    }

    memset(oldest, 0, sizeof(*oldest));
    oldest->in_use = true;
    oldest->slot_id = slot_id;
    oldest->sequence = sequence;
    oldest->initiator_id = initiator_id;
    oldest->responder_id = responder_id;
    oldest->updated_tick = xTaskGetTickCount();
    return oldest;
}

static bool uwb_anchor_pair_valid(const uint8_t *anchor_ids,
                                  size_t anchor_count,
                                  uint8_t initiator_id,
                                  uint8_t responder_id)
{
    return initiator_id != 0 && responder_id != 0 &&
           initiator_id != responder_id &&
           uwb_anchor_survey_id_in_set(anchor_ids, anchor_count,
                                       initiator_id) &&
           uwb_anchor_survey_id_in_set(anchor_ids, anchor_count, responder_id);
}

static int32_t uwb_flex_tdoa_store_anchor_distance(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    return uwb_flex_tdoa_runtime_store_anchor_range(
        anchor_a_id, anchor_b_id, distance_mm, raw_distance_mm, slot_id,
        sequence);
}

static int32_t uwb_passive_ds_store_anchor_distance(
    uint8_t anchor_a_id, uint8_t anchor_b_id, uint16_t sequence,
    uint32_t slot_id, int32_t distance_mm, int32_t raw_distance_mm)
{
    return uwb_passive_ds_runtime_store_anchor_range(
        anchor_a_id, anchor_b_id, distance_mm, raw_distance_mm, slot_id,
        sequence);
}

static int32_t uwb_flex_tdoa_cached_anchor_distance_mm(uint8_t anchor_a_id,
                                                       uint8_t anchor_b_id)
{
    return uwb_flex_tdoa_runtime_anchor_range_mm(
        anchor_a_id, anchor_b_id);
}

static uint32_t uwb_flex_tdoa_cached_anchor_distance_slot_id(
    uint8_t anchor_a_id, uint8_t anchor_b_id)
{
    return uwb_flex_tdoa_runtime_anchor_range_slot(
        anchor_a_id, anchor_b_id);
}

static bool uwb_flex_tdoa_next_piggyback_distance(
    struct uwb_anchor_range_entry *range)
{
    return uwb_flex_tdoa_runtime_next_anchor_range(s_source_id, range);
}

static void uwb_passive_ds_write_piggyback(uint8_t *payload, size_t offset)
{
    if (payload == NULL ||
        offset + UWB_PASSIVE_DS_PIGGYBACK_LEN >
            UWB_DW3000_PAYLOAD_LEN) {
        return;
    }
    memset(&payload[offset], 0, UWB_PASSIVE_DS_PIGGYBACK_LEN);

    struct uwb_passive_ds_completed_exchange completed = {0};
    if (uwb_passive_ds_runtime_get_completed_exchange(&completed)) {
        payload[offset + UWB_PASSIVE_DS_PIGGYBACK_PEER_OFFSET] =
            completed.initiator_id;
        uwb_distance_put_u16(
            payload,
            offset + UWB_PASSIVE_DS_PIGGYBACK_DISTANCE_MM_OFFSET,
            (uint16_t)completed.distance_mm);
        uwb_distance_put_u16(
            payload,
            offset + UWB_PASSIVE_DS_PIGGYBACK_RAW_DISTANCE_MM_OFFSET,
            (uint16_t)completed.raw_distance_mm);
        uwb_distance_put_u32(
            payload, offset + UWB_PASSIVE_DS_PIGGYBACK_SLOT_ID_OFFSET,
            completed.slot_id);
        uwb_distance_put_u16(
            payload, offset + UWB_PASSIVE_DS_PIGGYBACK_SEQUENCE_OFFSET,
            completed.sequence);
        uwb_distance_put_u32(
            payload,
            offset + UWB_PASSIVE_DS_PIGGYBACK_EXCHANGE_DTU_OFFSET,
            completed.responder_exchange_dtu);
        return;
    }

    struct uwb_anchor_range_entry range = {0};
    if (!uwb_passive_ds_runtime_next_anchor_range(s_source_id, &range) ||
        range.distance_mm <= 0 || range.distance_mm > UINT16_MAX ||
        range.raw_distance_mm <= 0 ||
        range.raw_distance_mm > UINT16_MAX) {
        return;
    }
    payload[offset + UWB_PASSIVE_DS_PIGGYBACK_PEER_OFFSET] =
        range.responder_id;
    uwb_distance_put_u16(
        payload, offset + UWB_PASSIVE_DS_PIGGYBACK_DISTANCE_MM_OFFSET,
        (uint16_t)range.distance_mm);
    uwb_distance_put_u16(
        payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_RAW_DISTANCE_MM_OFFSET,
        (uint16_t)range.raw_distance_mm);
    uwb_distance_put_u32(
        payload, offset + UWB_PASSIVE_DS_PIGGYBACK_SLOT_ID_OFFSET,
        range.slot_id);
    uwb_distance_put_u16(
        payload, offset + UWB_PASSIVE_DS_PIGGYBACK_SEQUENCE_OFFSET,
        range.sequence);
}

static bool uwb_passive_ds_calibrated_anchor_range_mm(
    const uint8_t *anchor_ids, size_t anchor_count, uint8_t first_id,
    uint8_t second_id, int32_t measured_mm, int32_t *calibrated_mm)
{
    if (anchor_ids == NULL || calibrated_mm == NULL || measured_mm <= 0) {
        return false;
    }
    int32_t correction_mm = 0;
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->passive_ds_calibration_enabled) {
        const size_t first_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, first_id);
        const size_t second_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, second_id);
        const size_t pair_index = app_runtime_config_anchor_pair_index(
            first_index, second_index);
        if (first_index == SIZE_MAX || second_index == SIZE_MAX ||
            pair_index == SIZE_MAX) {
            return false;
        }
        correction_mm = config->passive_ds_range_bias_mm[pair_index];
    }
    *calibrated_mm = measured_mm - correction_mm;
    return *calibrated_mm > 0;
}

static enum uwb_passive_ds_tdoa_status uwb_passive_ds_accept_piggyback(
    const struct uwb_distance_frame *frame, size_t offset,
    const uint8_t *anchor_ids, size_t anchor_count,
    struct uwb_passive_ds_tdoa_context *context,
    struct uwb_passive_ds_tdoa_result *result,
    uint8_t *initiator_id, uint8_t *responder_id, uint16_t *sequence,
    uint32_t *slot_id)
{
    if (frame == NULL || frame->payload_len <
                             offset + UWB_PASSIVE_DS_PIGGYBACK_LEN) {
        return UWB_PASSIVE_DS_TDOA_INCOMPLETE;
    }
    const uint8_t peer_id =
        frame->payload[offset + UWB_PASSIVE_DS_PIGGYBACK_PEER_OFFSET];
    const int32_t distance_mm = (int32_t)uwb_distance_get_u16(
        frame->payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_DISTANCE_MM_OFFSET);
    const int32_t raw_distance_mm = (int32_t)uwb_distance_get_u16(
        frame->payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_RAW_DISTANCE_MM_OFFSET);
    const uint32_t range_slot_id = uwb_distance_get_u32(
        frame->payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_SLOT_ID_OFFSET);
    const uint16_t range_sequence = uwb_distance_get_u16(
        frame->payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_SEQUENCE_OFFSET);
    const uint32_t responder_exchange_dtu = uwb_distance_get_u32(
        frame->payload,
        offset + UWB_PASSIVE_DS_PIGGYBACK_EXCHANGE_DTU_OFFSET);
    if (distance_mm <= 0 || raw_distance_mm <= 0 ||
        !uwb_anchor_pair_valid(
            anchor_ids, anchor_count, frame->source_id, peer_id)) {
        return UWB_PASSIVE_DS_TDOA_INCOMPLETE;
    }
    int32_t calibrated_distance_mm = 0;
    if (!uwb_passive_ds_calibrated_anchor_range_mm(
            anchor_ids, anchor_count, frame->source_id, peer_id,
            distance_mm, &calibrated_distance_mm)) {
        return UWB_PASSIVE_DS_TDOA_INCOMPLETE;
    }
    const int32_t stored = uwb_passive_ds_store_anchor_distance(
        frame->source_id, peer_id, frame->sequence, range_slot_id,
        calibrated_distance_mm, raw_distance_mm);
    if (stored > 0) {
        (void)uwb_passive_ds_runtime_submit_anchor_range(
            frame->source_id, peer_id, range_slot_id, stored);
    }

    if (context == NULL || result == NULL ||
        responder_exchange_dtu == 0U) {
        return UWB_PASSIVE_DS_TDOA_INCOMPLETE;
    }
    if (initiator_id != NULL) {
        *initiator_id = peer_id;
    }
    if (responder_id != NULL) {
        *responder_id = frame->source_id;
    }
    if (sequence != NULL) {
        *sequence = range_sequence;
    }
    if (slot_id != NULL) {
        *slot_id = range_slot_id;
    }
    return uwb_passive_ds_tdoa_record_responder_exchange(
        context, peer_id, frame->source_id, range_sequence,
        range_slot_id, responder_exchange_dtu, result);
}

static uint32_t uwb_flex_tdoa_slot_duration_us(size_t responder_count)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const struct flextdoa_timing timing = {
        .guard_us = config->flex_tdoa_guard_us,
        .request_subslot_us = config->flex_tdoa_request_subslot_us,
        .request_process_us = config->flex_tdoa_request_process_us,
        .response_subslot_us = config->flex_tdoa_response_subslot_us,
        .response_process_us = config->flex_tdoa_response_process_us,
    };
    return flextdoa_slot_duration_us(&timing, (uint8_t)responder_count);
}

static int32_t uwb_flex_tdoa_reference_anchor_distance_mm(
    uint8_t anchor_a_id, uint8_t anchor_b_id)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config->flex_tdoa_geometry_fixed) {
        int first = -1;
        int second = -1;
        for (size_t index = 0U; index < config->anchor_count; ++index) {
            if (config->anchor_ids[index] == anchor_a_id) {
                first = (int)index;
            }
            if (config->anchor_ids[index] == anchor_b_id) {
                second = (int)index;
            }
        }
        if (first >= 0 && second >= 0 && first != second) {
            const double dx_m =
                (config->flex_tdoa_anchor_x_mm[first] -
                 config->flex_tdoa_anchor_x_mm[second]) /
                1000.0;
            const double dy_m =
                (config->flex_tdoa_anchor_y_mm[first] -
                 config->flex_tdoa_anchor_y_mm[second]) /
                1000.0;
            return uwb_distance_meters_to_mm(hypot(dx_m, dy_m));
        }
    }
    return uwb_flex_tdoa_cached_anchor_distance_mm(anchor_a_id,
                                                    anchor_b_id);
}

// FlexTDOA CI-CR: the initiator changes in round-robin order every slot, and
// the response order rotates independently through all remaining anchors.
static size_t uwb_flex_tdoa_build_responder_list(
    const uint8_t *anchor_ids, size_t anchor_count, uint8_t initiator_id,
    uint32_t slot_id,
    uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U])
{
    const app_runtime_config_t *config = app_runtime_config_get();
    uint16_t protocol_anchor_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS] = {0};
    uint16_t protocol_slot_initiators[APP_RUNTIME_CONFIG_FLEX_MAX_SLOTS] = {0};
    for (size_t i = 0; i < anchor_count; ++i) {
        protocol_anchor_ids[i] = anchor_ids[i];
    }
    for (size_t i = 0; i < config->flex_tdoa_slot_count; ++i) {
        protocol_slot_initiators[i] =
            config->flex_tdoa_slot_initiator_ids[i];
    }
    struct flextdoa_slot_plan plan = {0};
    if (!flextdoa_build_ci_cr_slot(
            protocol_anchor_ids, anchor_count, protocol_slot_initiators,
            config->flex_tdoa_slot_count,
            config->flex_tdoa_slot_responder_masks,
            config->flex_tdoa_responder_count, slot_id, &plan) ||
        plan.initiator_id != initiator_id ||
        plan.responder_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U) {
        return 0;
    }
    memcpy(responders, plan.responder_ids, plan.responder_count);
    return plan.responder_count;
}

static size_t uwb_flex_tdoa_responder_index(const uint8_t *responders,
                                            size_t responder_count,
                                            uint8_t responder_id)
{
    for (size_t i = 0; i < responder_count; ++i) {
        if (responders[i] == responder_id) {
            return i;
        }
    }
    return SIZE_MAX;
}

static size_t uwb_flex_tdoa_build_message(
    enum uwb_distance_frame_type type, const uint8_t *destinations,
    size_t destination_count, uint32_t slot_id, uint32_t processing_dtu,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    if (destination_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U ||
        (destination_count > 0U && destinations == NULL)) {
        return 0;
    }
    const enum flextdoa_message_type message_type =
        type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ
            ? FLEXTDOA_MESSAGE_REQUEST
            : type == UWB_DISTANCE_FRAME_FLEX_TDOA_RESP
                  ? FLEXTDOA_MESSAGE_RESPONSE
                  : 0;
    if (message_type == 0) {
        return 0;
    }

    struct flextdoa_packet packet = {
        .type = message_type,
        .slot_id = slot_id,
        .source_id = s_source_id,
        .destination_count = (uint8_t)destination_count,
        .processing_time_dtu = processing_dtu,
    };
    if (destination_count > 0U) {
        memcpy(packet.destination_ids, destinations, destination_count);
    }

    // Every localization packet carries one TWR result previously measured
    // by its source, as specified by Figure 3. Cycling prevents a fresh pair
    // from starving the other cached pairs.
    struct uwb_anchor_range_entry previous = {0};
    if (uwb_flex_tdoa_next_piggyback_distance(&previous)) {
        packet.previous_twr_responder_id = previous.responder_id;
        packet.previous_twr_mm = (uint16_t)previous.distance_mm;
        packet.previous_slot_id = (uint16_t)previous.slot_id;
    }
    memset(payload, 0, UWB_DW3000_PAYLOAD_LEN);
    return flextdoa_encode_packet(&packet, payload,
                                  UWB_DW3000_PAYLOAD_LEN);
}

static bool uwb_flex_tdoa_parse_message(
    const struct uwb_distance_frame *frame,
    struct uwb_flex_tdoa_message_metadata *metadata)
{
    if (frame == NULL || metadata == NULL ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return false;
    }
    struct flextdoa_packet packet = {0};
    if (!flextdoa_decode_packet(frame->payload, frame->payload_len,
                                &packet) ||
        packet.source_id != frame->source_id ||
        ((frame->type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ) !=
         (packet.type == FLEXTDOA_MESSAGE_REQUEST))) {
        return false;
    }
    memset(metadata, 0, sizeof(*metadata));
    metadata->slot_id = packet.slot_id;
    metadata->destination_count = packet.destination_count;
    if (metadata->destination_count >
        UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U) {
        return false;
    }
    memcpy(metadata->destinations, packet.destination_ids,
           metadata->destination_count);
    if (packet.previous_twr_responder_id > UINT8_MAX) {
        return false;
    }
    metadata->processing_dtu = packet.processing_time_dtu;
    metadata->previous_responder_id =
        (uint8_t)packet.previous_twr_responder_id;
    metadata->previous_distance_mm = packet.previous_twr_mm;
    metadata->previous_slot_id = packet.previous_slot_id;
    return true;
}

static size_t uwb_flex_tdoa_build_request(
    const uint8_t *responders, size_t responder_count, uint32_t slot_id,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    if (responders == NULL || responder_count == 0U) {
        return 0;
    }
    return uwb_flex_tdoa_build_message(
        UWB_DISTANCE_FRAME_FLEX_TDOA_REQ, responders, responder_count,
        slot_id, 0U, payload);
}

static bool uwb_flex_tdoa_parse_request(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count, uint32_t *slot_id, uint8_t *responder_count,
    uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U],
    struct uwb_flex_tdoa_message_metadata *metadata_out)
{
    if (frame == NULL || slot_id == NULL || responder_count == NULL ||
        responders == NULL || frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_REQ ||
        !uwb_anchor_survey_id_in_set(anchor_ids, anchor_count,
                                     frame->source_id)) {
        return false;
    }

    struct uwb_flex_tdoa_message_metadata metadata = {0};
    if (!uwb_flex_tdoa_parse_message(frame, &metadata) ||
        metadata.destination_count == 0U ||
        metadata.destination_count > anchor_count - 1U) {
        return false;
    }

    *slot_id = metadata.slot_id;
    *responder_count = metadata.destination_count;
    for (size_t i = 0; i < metadata.destination_count; ++i) {
        responders[i] = metadata.destinations[i];
        if (!uwb_anchor_pair_valid(anchor_ids, anchor_count,
                                             frame->source_id,
                                             responders[i])) {
            return false;
        }
    }
    if (metadata_out != NULL) {
        *metadata_out = metadata;
    }
    return true;
}

static bool uwb_flex_tdoa_parse_local_response_request(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count, uint32_t *slot_id, size_t *responder_index)
{
    if (frame == NULL || anchor_ids == NULL || anchor_count < 2U ||
        slot_id == NULL || responder_index == NULL ||
        frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_REQ ||
        !uwb_distance_destination_matches(frame->destination_id) ||
        !uwb_anchor_survey_id_in_set(anchor_ids, anchor_count,
                                     frame->source_id)) {
        return false;
    }

    struct uwb_flex_tdoa_message_metadata metadata = {0};
    if (!uwb_flex_tdoa_parse_message(frame, &metadata)) {
        return false;
    }
    const uint8_t responder_count = metadata.destination_count;
    if (responder_count == 0U || responder_count >= anchor_count ||
        responder_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U) {
        return false;
    }

    *slot_id = metadata.slot_id;
    *responder_index = SIZE_MAX;
    for (size_t i = 0; i < responder_count; ++i) {
        const uint8_t responder_id = metadata.destinations[i];
        if (!uwb_anchor_pair_valid(
                anchor_ids, anchor_count, frame->source_id, responder_id)) {
            return false;
        }
        if (responder_id == s_source_id) {
            if (*responder_index != SIZE_MAX) {
                return false;
            }
            *responder_index = i;
        }
    }
    return true;
}

static size_t uwb_flex_tdoa_build_response(
    uint32_t slot_id, uint32_t reply_dtu,
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    return uwb_flex_tdoa_build_message(
        UWB_DISTANCE_FRAME_FLEX_TDOA_RESP, NULL, 0U, slot_id,
        reply_dtu, payload);
}

static bool uwb_flex_tdoa_parse_response(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count, uint32_t *slot_id, uint8_t *initiator_id,
    uint8_t *responder_id, uint8_t *responder_index, uint64_t *reply_dtu,
    uint8_t *previous_responder_id, int32_t *previous_distance_mm,
    uint32_t *previous_slot_id)
{
    if (frame == NULL || slot_id == NULL || initiator_id == NULL ||
        responder_id == NULL || responder_index == NULL ||
        reply_dtu == NULL || previous_responder_id == NULL ||
        previous_distance_mm == NULL || previous_slot_id == NULL ||
        frame->type != UWB_DISTANCE_FRAME_FLEX_TDOA_RESP ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return false;
    }
    *responder_id = frame->source_id;

    struct uwb_flex_tdoa_message_metadata metadata = {0};
    if (uwb_flex_tdoa_parse_message(frame, &metadata) &&
        metadata.destination_count == 0U) {
        *slot_id = metadata.slot_id;
        *initiator_id = uwb_flex_tdoa_slot_initiator(*slot_id);
        *reply_dtu = metadata.processing_dtu;
        *previous_responder_id = metadata.previous_responder_id;
        *previous_distance_mm = metadata.previous_distance_mm;
        const uint16_t age =
            (uint16_t)((uint16_t)*slot_id - metadata.previous_slot_id);
        *previous_slot_id = *slot_id - age;
    } else {
        return false;
    }

    uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
    const size_t responder_count = uwb_flex_tdoa_build_responder_list(
        anchor_ids, anchor_count, *initiator_id, *slot_id, responders);
    const size_t parsed_index = uwb_flex_tdoa_responder_index(
        responders, responder_count, *responder_id);
    if (parsed_index == SIZE_MAX) {
        return false;
    }
    *responder_index = (uint8_t)parsed_index;

    return *reply_dtu != 0 &&
           uwb_anchor_pair_valid(anchor_ids, anchor_count,
                                           *initiator_id, *responder_id);
}

static void uwb_flex_tdoa_accept_piggyback(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count, uint8_t previous_responder_id,
    int32_t previous_distance_mm, uint32_t previous_slot_id)
{
    if (frame == NULL || previous_distance_mm <= 0 ||
        !uwb_anchor_pair_valid(
            anchor_ids, anchor_count, frame->source_id,
            previous_responder_id)) {
        return;
    }
    (void)uwb_flex_tdoa_store_anchor_distance(
        frame->source_id, previous_responder_id, frame->sequence,
        previous_slot_id, previous_distance_mm, previous_distance_mm);
}

static void uwb_flex_tdoa_log_paper_observation(
    uint8_t tag_id, struct uwb_flex_tdoa_observation *observation)
{
    if (observation == NULL || !observation->have_request ||
        !observation->have_response || observation->responder_reply_dtu == 0 ||
        observation->anchor_distance_mm <= 0) {
        return;
    }

    const double reply_dtu = (double)observation->responder_reply_dtu;
    const uint64_t rx_delta_tag_raw = uwb_distance_delta_ts(
        observation->response_rx_tag_ts, observation->request_rx_tag_ts);
    if (uwb_dw3000_dtu_to_us(rx_delta_tag_raw) > 20000) {
        s_flex_tdoa_observation_invalid_since_summary++;
        s_flex_tdoa_observation_invalid_age_since_summary++;
        if (observation->responder_index <
            UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U) {
            s_flex_tdoa_observation_invalid_age_index_since_summary[
                observation->responder_index]++;
        }
        memset(observation, 0, sizeof(*observation));
        return;
    }
    if (!observation->resp_clock_offset_valid) {
        // FlexTDOA equation (12) requires the responder-to-tag CFO for the
        // processing-time correction. Publishing the uncorrected value here
        // would create a plausible-looking but invalid TDOA observation.
        s_flex_tdoa_observation_invalid_since_summary++;
        s_flex_tdoa_observation_invalid_cfo_since_summary++;
        memset(observation, 0, sizeof(*observation));
        return;
    }
    const double anchor_distance_m =
        (double)observation->anchor_distance_mm / 1000.0;
    const double anchor_tof_dtu =
        anchor_distance_m /
        (UWB_DW3000_TIME_UNIT_SECONDS * UWB_DW3000_SPEED_OF_LIGHT_MPS);
    const double raw_diff_dtu =
        (double)rx_delta_tag_raw - reply_dtu - anchor_tof_dtu;

    const double raw_cfo_fraction =
        -observation->resp_clock_offset_ratio;
    const app_runtime_config_t *config = app_runtime_config_get();
    if (config != NULL) {
        flextdoa_cfo_estimator_configure(
            &s_flex_tdoa_cfo_estimator,
            config->flex_tdoa_config_generation);
    }
    flextdoa_cfo_estimator_note_slot(
        &s_flex_tdoa_cfo_estimator, observation->slot_id);
    struct flextdoa_cfo_result cfo_result = {0};
    if (!flextdoa_cfo_estimator_update(
            &s_flex_tdoa_cfo_estimator, observation->responder_id,
            raw_cfo_fraction, &cfo_result)) {
        s_flex_tdoa_observation_invalid_since_summary++;
        s_flex_tdoa_observation_invalid_cfo_since_summary++;
        memset(observation, 0, sizeof(*observation));
        return;
    }
    const double reply_corrected_dtu =
        reply_dtu * (1.0 - cfo_result.applied_fraction);
    const struct flextdoa_observation_input protocol_input = {
        .request_rx_tag_dtu = observation->request_rx_tag_ts,
        .response_rx_tag_dtu = observation->response_rx_tag_ts,
        .responder_processing_dtu =
            (uint32_t)observation->responder_reply_dtu,
        /* The driver ratio is applied as (1 + ratio); Eq. (12) names the
         * same DW3000 correction epsilon and applies (1 - epsilon). */
        .responder_to_tag_cfo_fraction = cfo_result.applied_fraction,
        .initiator_responder_tof_dtu = anchor_tof_dtu,
        .dtu_seconds = UWB_DW3000_TIME_UNIT_SECONDS,
        .speed_of_light_mps = UWB_DW3000_SPEED_OF_LIGHT_MPS,
    };
    double diff_m = 0.0;
    if (!flextdoa_compute_range_difference_m(&protocol_input, &diff_m)) {
        s_flex_tdoa_observation_invalid_since_summary++;
        memset(observation, 0, sizeof(*observation));
        return;
    }
    const double raw_diff_m = uwb_distance_tof_to_meters(raw_diff_dtu);
    const int32_t cfo_correction_mm =
        uwb_distance_meters_to_mm(diff_m - raw_diff_m);
    const int32_t raw_cfo_ppb =
        (int32_t)lround(cfo_result.raw_fraction * 1000000000.0);
    const int32_t estimated_cfo_ppb =
        (int32_t)lround(cfo_result.estimated_fraction * 1000000000.0);
    const int32_t applied_cfo_ppb =
        (int32_t)lround(cfo_result.applied_fraction * 1000000000.0);
    const bool queued =
        wireless_telemetry_service_submit_flex_tdoa_observation(
            tag_id, observation->initiator_id, observation->responder_id,
            observation->responder_index, observation->sequence,
            observation->slot_id, uwb_distance_meters_to_mm(diff_m),
            uwb_distance_meters_to_mm(raw_diff_m),
            observation->anchor_distance_mm, cfo_correction_mm,
            raw_cfo_ppb, estimated_cfo_ppb, applied_cfo_ppb,
            (uint32_t)observation->responder_reply_dtu,
            cfo_result.sample_count, cfo_result.flags);
    (void)uwb_flex_tdoa_runtime_submit_observation(
        tag_id, observation->initiator_id, observation->responder_id,
        observation->slot_id, uwb_distance_meters_to_mm(diff_m));
    s_flex_tdoa_observations_since_summary++;
    if (!queued) {
        s_flex_tdoa_observation_drops_since_summary++;
    }

    const TickType_t now = xTaskGetTickCount();
    if (s_flex_tdoa_observation_summary_tick == 0 ||
        now - s_flex_tdoa_observation_summary_tick >= pdMS_TO_TICKS(1000)) {
        (void)wireless_log_service_submit('I', TAG,
                 "FLEX_TDOA tag n=%lu drop=%lu slots=%lu/%lu "
                 "inv=%lu(age=%lu idx=%lu/%lu/%lu cfo=%lu ord=%lu) "
                 "req_gap=%lu "
                 "req=%lu/%lu/%lu/%lu resp=%lu/%lu/%lu/%lu "
                 "rxerr=%lu/0x%08lx",
                 (unsigned long)s_flex_tdoa_observations_since_summary,
                 (unsigned long)s_flex_tdoa_observation_drops_since_summary,
                 (unsigned long)s_flex_tdoa_complete_slots_since_summary,
                 (unsigned long)s_flex_tdoa_incomplete_slots_since_summary,
                 (unsigned long)s_flex_tdoa_observation_invalid_since_summary,
                 (unsigned long)s_flex_tdoa_observation_invalid_age_since_summary,
                 (unsigned long)s_flex_tdoa_observation_invalid_age_index_since_summary[0],
                 (unsigned long)s_flex_tdoa_observation_invalid_age_index_since_summary[1],
                 (unsigned long)s_flex_tdoa_observation_invalid_age_index_since_summary[2],
                 (unsigned long)s_flex_tdoa_observation_invalid_cfo_since_summary,
                 (unsigned long)s_flex_tdoa_observation_invalid_order_since_summary,
                 (unsigned long)s_flex_tdoa_missed_requests_since_summary,
                 (unsigned long)s_flex_tdoa_tag_requests_by_anchor[0],
                 (unsigned long)s_flex_tdoa_tag_requests_by_anchor[1],
                 (unsigned long)s_flex_tdoa_tag_requests_by_anchor[2],
                 (unsigned long)s_flex_tdoa_tag_requests_by_anchor[3],
                 (unsigned long)s_flex_tdoa_tag_responses_by_anchor[0],
                 (unsigned long)s_flex_tdoa_tag_responses_by_anchor[1],
                 (unsigned long)s_flex_tdoa_tag_responses_by_anchor[2],
                 (unsigned long)s_flex_tdoa_tag_responses_by_anchor[3],
                 (unsigned long)s_flex_tdoa_rx_errors_since_summary,
                 (unsigned long)s_flex_tdoa_rx_error_status_since_summary);
        s_flex_tdoa_observations_since_summary = 0;
        s_flex_tdoa_observation_drops_since_summary = 0;
        s_flex_tdoa_complete_slots_since_summary = 0;
        s_flex_tdoa_incomplete_slots_since_summary = 0;
        s_flex_tdoa_missed_requests_since_summary = 0;
        s_flex_tdoa_observation_invalid_since_summary = 0;
        s_flex_tdoa_observation_invalid_age_since_summary = 0;
        s_flex_tdoa_observation_invalid_cfo_since_summary = 0;
        s_flex_tdoa_observation_invalid_order_since_summary = 0;
        memset(s_flex_tdoa_observation_invalid_age_index_since_summary, 0,
               sizeof(s_flex_tdoa_observation_invalid_age_index_since_summary));
        memset(s_flex_tdoa_tag_requests_by_anchor, 0,
               sizeof(s_flex_tdoa_tag_requests_by_anchor));
        memset(s_flex_tdoa_tag_responses_by_anchor, 0,
               sizeof(s_flex_tdoa_tag_responses_by_anchor));
        s_flex_tdoa_rx_errors_since_summary = 0;
        s_flex_tdoa_rx_error_status_since_summary = 0;
        s_flex_tdoa_observation_summary_tick = now;
    }
    ESP_LOGD(TAG,
             "FLEX_TDOA paper obs timing tag=%u pair=%u-%u seq=%u rx_delta=%.2f dtu reply=%.2f dtu reply_corr=%.2f dtu anchor_tof=%.2f dtu clk_raw=%ld",
             (unsigned)tag_id, (unsigned)observation->initiator_id,
             (unsigned)observation->responder_id,
             (unsigned)observation->sequence,
             (double)rx_delta_tag_raw, reply_dtu,
             reply_corrected_dtu, anchor_tof_dtu,
             (long)observation->resp_clock_offset_raw);

    memset(observation, 0, sizeof(*observation));
}

static void uwb_flex_tdoa_record_incomplete_mask(uint16_t missing_mask)
{
    if (missing_mask != 0U) {
        s_flex_tdoa_incomplete_slots_since_summary++;
    }
}

static esp_err_t uwb_distance_receive_next_until(
    struct uwb_distance_frame *frame, int64_t deadline_us)
{
    if (frame == NULL || deadline_us <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    while (true) {
        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        const uint32_t remaining_ms =
            (uint32_t)((remaining_us + 999LL) / 1000LL);

        struct uwb_dw3000_rx_frame rx_frame = {0};
        const esp_err_t err = uwb_dw3000_receive_frame_until(
            &rx_frame, remaining_ms, deadline_us);
        if (err != ESP_OK) {
            return err;
        }

        if (!uwb_distance_parse_frame(&rx_frame, frame)) {
            s_rx_ignored_count++;
            ESP_LOGD(TAG, "Ignoring non-ranging frame len=%u ignored=%lu",
                     (unsigned)rx_frame.payload_len,
                     (unsigned long)s_rx_ignored_count);
            continue;
        }

        s_last_rx_source_id = frame->source_id;
        s_last_rx_sequence = frame->sequence;
        return ESP_OK;
    }
}

static void uwb_flex_tdoa_tag_note_request_slot(uint32_t slot_id)
{
    if (s_flex_tdoa_tag_request_slot_valid) {
        const int32_t delta =
            (int32_t)(slot_id - s_flex_tdoa_tag_last_request_slot_id);
        /* A forward gap is an exact count of requests not observed by the
         * passive tag. Ignore implausibly large jumps: distributed schedule
         * bootstrap intentionally restarts the slot sequence after resync. */
        if (delta > 1 && delta <= 4096) {
            s_flex_tdoa_missed_requests_since_summary +=
                (uint32_t)(delta - 1);
        }
    }
    s_flex_tdoa_tag_last_request_slot_id = slot_id;
    s_flex_tdoa_tag_request_slot_valid = true;
}

static void uwb_flex_tdoa_tag_process_frame(
    const struct uwb_distance_frame *frame,
    struct uwb_flex_tdoa_observation *observations,
    uint8_t tag_id, const uint8_t *anchor_ids, size_t anchor_count)
{
    if (frame == NULL) {
        return;
    }

    struct flextdoa_packet protocol_packet = {0};
    if (!flextdoa_decode_packet(frame->payload, frame->payload_len,
                                &protocol_packet)) {
        s_flex_tdoa_observation_invalid_since_summary++;
        return;
    }
    if (protocol_packet.type == FLEXTDOA_MESSAGE_REQUEST &&
        s_flex_tdoa_tag_collection.active &&
        !flextdoa_collector_complete(&s_flex_tdoa_tag_collection)) {
        uwb_flex_tdoa_record_incomplete_mask(
            flextdoa_collector_missing_mask(
                &s_flex_tdoa_tag_collection));
    }
    double protocol_cfo_fraction = 0.0;
    if (frame->clock_offset_valid) {
        const double driver_ratio =
            frame->clock_offset_from_cia
                ? uwb_dw3000_cia_clock_offset_ratio(
                      frame->clock_offset_raw)
                : uwb_dw3000_clock_offset_ratio(
                      frame->clock_offset_raw);
        protocol_cfo_fraction = -driver_ratio;
    }
    const enum flextdoa_collect_result collect_result =
        flextdoa_collector_ingest(
            &s_flex_tdoa_tag_collection, &protocol_packet,
            frame->rx_timestamp, frame->clock_offset_valid,
            protocol_cfo_fraction);
    if (collect_result == FLEXTDOA_COLLECT_REJECTED) {
        s_flex_tdoa_observation_invalid_since_summary++;
        s_flex_tdoa_observation_invalid_order_since_summary++;
        return;
    }
    if (collect_result == FLEXTDOA_COLLECT_COMPLETE) {
        s_flex_tdoa_complete_slots_since_summary++;
    }

    uint8_t initiator_id = 0;
    uint8_t responder_id = 0;
    switch (frame->type) {
    case UWB_DISTANCE_FRAME_FLEX_TDOA_REQ: {
        uint32_t slot_id = 0;
        uint8_t responder_count = 0;
        uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
        struct uwb_flex_tdoa_message_metadata metadata = {0};
        if (!uwb_flex_tdoa_parse_request(frame, anchor_ids, anchor_count,
                                         &slot_id, &responder_count,
                                         responders, &metadata)) {
            return;
        }
        uwb_flex_tdoa_accept_piggyback(
            frame, anchor_ids, anchor_count,
            metadata.previous_responder_id, metadata.previous_distance_mm,
            slot_id - (uint16_t)((uint16_t)slot_id -
                                 metadata.previous_slot_id));
        initiator_id = frame->source_id;
        const size_t initiator_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, initiator_id);
        if (initiator_index != SIZE_MAX) {
            s_flex_tdoa_tag_requests_by_anchor[initiator_index]++;
        }
        for (size_t i = 0; i < responder_count; ++i) {
            struct uwb_flex_tdoa_observation *observation =
                uwb_flex_tdoa_find_observation(
                    observations, slot_id, frame->sequence, initiator_id,
                    responders[i], true);
            if (observation == NULL) {
                continue;
            }
            observation->request_rx_tag_ts = frame->rx_timestamp;
            observation->responder_index = (uint8_t)i;
            observation->have_request = true;
            observation->updated_tick = xTaskGetTickCount();
        }
        break;
    }

    case UWB_DISTANCE_FRAME_FLEX_TDOA_RESP: {
        uint32_t slot_id = 0;
        uint8_t responder_index = 0;
        uint64_t reply_dtu = 0;
        uint8_t previous_responder_id = 0;
        int32_t previous_distance_mm = 0;
        uint32_t previous_slot_id = 0;
        if (!uwb_flex_tdoa_parse_response(
                frame, anchor_ids, anchor_count, &slot_id, &initiator_id,
                &responder_id, &responder_index, &reply_dtu,
                &previous_responder_id, &previous_distance_mm,
                &previous_slot_id)) {
            return;
        }
        const size_t responder_anchor_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, responder_id);
        if (responder_anchor_index != SIZE_MAX) {
            s_flex_tdoa_tag_responses_by_anchor[responder_anchor_index]++;
        }
        uwb_flex_tdoa_accept_piggyback(
            frame, anchor_ids, anchor_count, previous_responder_id,
            previous_distance_mm, previous_slot_id);
        struct uwb_flex_tdoa_observation *observation =
            uwb_flex_tdoa_find_observation(
                observations, slot_id, frame->sequence, initiator_id,
                responder_id, false);
        if (observation == NULL) {
            ESP_LOGD(TAG,
                     "FLEX_TDOA response without local request RX pair=%u-%u seq=%u",
                     (unsigned)initiator_id, (unsigned)responder_id,
                     (unsigned)frame->sequence);
            return;
        }
        if (observation->slot_id != slot_id ||
            observation->responder_index != responder_index) {
            s_flex_tdoa_observation_invalid_since_summary++;
            s_flex_tdoa_observation_invalid_order_since_summary++;
            memset(observation, 0, sizeof(*observation));
            return;
        }
        observation->response_rx_tag_ts = frame->rx_timestamp;
        observation->slot_id = slot_id;
        observation->responder_index = responder_index;
        observation->responder_reply_dtu = reply_dtu;
        observation->anchor_distance_mm =
            uwb_flex_tdoa_reference_anchor_distance_mm(initiator_id,
                                                       responder_id);
        observation->anchor_distance_slot_id =
            app_runtime_config_get()->flex_tdoa_geometry_fixed
                ? slot_id
                : uwb_flex_tdoa_cached_anchor_distance_slot_id(
                      initiator_id, responder_id);
        observation->have_response = true;
        observation->resp_clock_offset_valid = frame->clock_offset_valid;
        observation->resp_clock_offset_raw = frame->clock_offset_raw;
        if (frame->clock_offset_valid) {
            observation->resp_clock_offset_ratio =
                frame->clock_offset_from_cia
                    ? uwb_dw3000_cia_clock_offset_ratio(
                          frame->clock_offset_raw)
                    : uwb_dw3000_clock_offset_ratio(
                          frame->clock_offset_raw);
        } else {
            observation->resp_clock_offset_ratio = 0.0;
        }
        observation->updated_tick = xTaskGetTickCount();
        uwb_flex_tdoa_log_paper_observation(tag_id, observation);
        break;
    }

    default:
        break;
    }
}

static size_t uwb_flex_tdoa_tag_collect_response_burst(
    const struct uwb_distance_frame *request, const uint8_t *anchor_ids,
    size_t anchor_count,
    struct uwb_distance_frame
        responses[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U],
    bool present[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U],
    uint8_t *responder_count_out,
    struct uwb_distance_frame *deferred_request,
    bool *deferred_request_valid)
{
    if (request == NULL || responses == NULL || present == NULL ||
        responder_count_out == NULL || deferred_request == NULL ||
        deferred_request_valid == NULL) {
        return 0U;
    }
    *responder_count_out = 0U;
    *deferred_request_valid = false;
    uint32_t slot_id = 0U;
    uint8_t responder_count = 0U;
    uint8_t responder_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
    if (!uwb_flex_tdoa_parse_request(
            request, anchor_ids, anchor_count, &slot_id,
            &responder_count, responder_ids, NULL)) {
        return 0U;
    }
    uwb_flex_tdoa_tag_note_request_slot(slot_id);
    *responder_count_out = responder_count;

    memset(responses, 0,
           sizeof(*responses) *
               (UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U));
    memset(present, 0,
           sizeof(*present) *
               (UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U));
    const app_runtime_config_t *config = app_runtime_config_get();
    const int64_t request_host_us =
        request->rx_host_time_us > 0
            ? request->rx_host_time_us
            : esp_timer_get_time();
    const struct flextdoa_timing timing = {
        .guard_us = config->flex_tdoa_guard_us,
        .request_subslot_us = config->flex_tdoa_request_subslot_us,
        .request_process_us = config->flex_tdoa_request_process_us,
        .response_subslot_us = config->flex_tdoa_response_subslot_us,
        .response_process_us = config->flex_tdoa_response_process_us,
    };
    const uint32_t collection_us = flextdoa_response_collection_us(
        &timing, responder_count);
    if (collection_us == 0U) {
        return 0U;
    }
    /* Stop exactly at the end of the response train. The receive path uses
     * the existing 1 MHz GPTimer because a FreeRTOS tick is 10 ms in this
     * build. A deferred-request guard remains as a lossless safety net if a
     * radio IRQ and the deadline alarm arrive together. */
    const int64_t deadline_us = request_host_us + (int64_t)collection_us;
    size_t collected = 0U;
    while (collected < responder_count &&
           !uwb_dw3000_runtime_switch_pending()) {
        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            break;
        }
        struct uwb_distance_frame candidate = {0};
        const esp_err_t err =
            uwb_distance_receive_next_until(&candidate, deadline_us);
        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FINISHED) {
            break;
        }
        if (err != ESP_OK) {
            continue;
        }
        if (candidate.type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ) {
            /*
             * CONFIG_FREERTOS_HZ=100 means a nominal 1 ms receive timeout
             * blocks for one 10 ms tick. If the final response is absent,
             * the following request can therefore wake receive_next() while
             * this burst is still active. Preserve it for the outer loop;
             * discarding it creates a second, artificial lost slot.
             */
            *deferred_request = candidate;
            *deferred_request_valid = true;
            break;
        }
        if (candidate.type != UWB_DISTANCE_FRAME_FLEX_TDOA_RESP) {
            continue;
        }
        struct flextdoa_packet packet = {0};
        if (!flextdoa_decode_packet(
                candidate.payload, candidate.payload_len, &packet) ||
            packet.type != FLEXTDOA_MESSAGE_RESPONSE ||
            packet.slot_id != slot_id) {
            continue;
        }
        for (uint8_t index = 0U; index < responder_count; ++index) {
            if (responder_ids[index] == candidate.source_id &&
                !present[index]) {
                responses[index] = candidate;
                present[index] = true;
                collected++;
                break;
            }
        }
    }
    return collected;
}

static void uwb_flex_tdoa_tag_loop(const uint8_t *anchor_ids,
                                     size_t anchor_count)
{
    memset(s_flex_tdoa_tag_observations, 0,
           sizeof(s_flex_tdoa_tag_observations));
    flextdoa_collector_reset(&s_flex_tdoa_tag_collection);
    s_flex_tdoa_tag_request_slot_valid = false;
    s_flex_tdoa_tag_last_request_slot_id = 0U;
    s_flex_tdoa_missed_requests_since_summary = 0U;
    const app_runtime_config_t *config = app_runtime_config_get();
    flextdoa_cfo_estimator_reset(&s_flex_tdoa_cfo_estimator);
    if (config != NULL) {
        flextdoa_cfo_estimator_configure(
            &s_flex_tdoa_cfo_estimator,
            config->flex_tdoa_config_generation);
    }
    const uint8_t tag_id = s_source_id;
    const esp_err_t solver_err = uwb_flex_tdoa_runtime_start_solver();
    if (solver_err != ESP_OK) {
        ESP_LOGW(TAG, "FlexTDOA local solver unavailable: %s",
                 esp_err_to_name(solver_err));
    }
    const esp_err_t timer_err = uwb_calibration_timer_start();
    if (timer_err != ESP_OK) {
        ESP_LOGW(TAG, "FlexTDOA 1 MHz deadline timer unavailable: %s",
                 esp_err_to_name(timer_err));
    }
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "FlexTDOA radio-passive tag active: tag_id=%u anchors=[%u,%u,%u,%u] rx_slice=%u ms",
             (unsigned)tag_id, (unsigned)anchor_ids[0],
             (unsigned)anchor_ids[1], (unsigned)anchor_ids[2],
             (unsigned)anchor_ids[3],
             (unsigned)config->anchor_survey_rx_slice_ms);

    struct uwb_distance_frame deferred_request = {0};
    bool deferred_request_valid = false;
    while (!uwb_dw3000_runtime_switch_pending()) {
        config = app_runtime_config_get();
        struct uwb_distance_frame frame = {0};
        esp_err_t err = ESP_OK;
        if (deferred_request_valid) {
            frame = deferred_request;
            memset(&deferred_request, 0, sizeof(deferred_request));
            deferred_request_valid = false;
        } else {
            err = uwb_distance_receive_next(
                &frame, config->anchor_survey_rx_slice_ms);
        }
        if (err == ESP_OK) {
            if (uwb_flex_tdoa_handle_runtime_config(&frame)) {
                continue;
            }
            if (frame.type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ) {
                struct uwb_distance_frame responses[
                    UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
                bool present[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
                uint8_t responder_count = 0U;
                (void)uwb_flex_tdoa_tag_collect_response_burst(
                    &frame, anchor_ids, anchor_count, responses, present,
                    &responder_count, &deferred_request,
                    &deferred_request_valid);
                const uint16_t missing_mask =
                    flextdoa_missing_mask_from_presence(
                        present, responder_count);
                if (missing_mask != 0U) {
                    uwb_flex_tdoa_record_incomplete_mask(missing_mask);
                }
                uwb_flex_tdoa_tag_process_frame(
                    &frame, s_flex_tdoa_tag_observations, tag_id,
                    anchor_ids, anchor_count);
                for (size_t index = 0U;
                     index < UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U;
                     ++index) {
                    if (present[index]) {
                        uwb_flex_tdoa_tag_process_frame(
                            &responses[index],
                            s_flex_tdoa_tag_observations, tag_id,
                            anchor_ids, anchor_count);
                    }
                }
                if (missing_mask != 0U) {
                    /* Valid responses were already submitted to the frame
                     * solver. Drop only the unfinished slot-local collector
                     * state so the next request cannot count it twice. */
                    flextdoa_collector_reset(&s_flex_tdoa_tag_collection);
                    memset(s_flex_tdoa_tag_observations, 0,
                           sizeof(s_flex_tdoa_tag_observations));
                }
            } else {
                uwb_flex_tdoa_tag_process_frame(
                    &frame, s_flex_tdoa_tag_observations, tag_id,
                    anchor_ids, anchor_count);
            }
        } else if (err != ESP_ERR_TIMEOUT &&
                   err != ESP_ERR_INVALID_RESPONSE &&
                   !uwb_dw3000_runtime_switch_pending()) {
            ESP_LOGW(TAG, "FLEX_TDOA passive tag RX failed: %s",
                     esp_err_to_name(err));
        }
    }
}

static esp_err_t uwb_flex_tdoa_send_response_preparsed(
    const struct uwb_distance_frame *request, uint32_t slot_id,
    size_t responder_index)
{
    if (request == NULL || responder_index == SIZE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    // The DW3000 timestamp makes each ordered response independent of host
    // jitter after the configured request-processing window has elapsed.
    const uint32_t response_delay_us =
        config->flex_tdoa_request_subslot_us +
        config->flex_tdoa_request_process_us +
        ((uint32_t)responder_index *
         config->flex_tdoa_response_subslot_us);
    const uint64_t response_delay_dtu =
        uwb_dw3000_us_to_dtu(response_delay_us);
    const uint64_t response_due = uwb_dw3000_add_timestamp_delta(
        request->rx_timestamp, response_delay_dtu);
    const uint32_t absolute_delay_word =
        uwb_dw3000_delayed_time_word(response_due);
    const uint64_t absolute_programmed_tx_ts =
        uwb_dw3000_programmed_tx_timestamp(absolute_delay_word);
    const uint64_t reply_dtu = uwb_distance_delta_ts(
        absolute_programmed_tx_ts, request->rx_timestamp);
    if (reply_dtu == 0U || reply_dtu > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    const size_t payload_len = uwb_flex_tdoa_build_response(
        slot_id, (uint32_t)reply_dtu, payload);
    if (payload_len == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t err = uwb_dw3000_send_payload_delayed_timeout(
        payload, payload_len, response_due, UWB_FLEX_TDOA_TX_TIMEOUT_MS,
        NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "FLEX_TDOA response failed pair=%u-%u seq=%u slot=%lu index=%u delay=%lu us: %s",
                 (unsigned)request->source_id, (unsigned)s_source_id,
                 (unsigned)request->sequence, (unsigned long)slot_id,
                 (unsigned)responder_index, (unsigned long)response_delay_us,
                 esp_err_to_name(err));
        return err;
    }

    const size_t initiator_index = uwb_anchor_survey_id_index(
        config->anchor_ids, config->anchor_count, request->source_id);
    if (initiator_index != SIZE_MAX) {
        s_flex_tdoa_response_tx_by_initiator[initiator_index]++;
    }

    ESP_LOGD(TAG,
             "FLEX_TDOA response pair=%u-%u seq=%u slot=%lu index=%u reply=0x%010llx",
             (unsigned)request->source_id, (unsigned)s_source_id,
             (unsigned)request->sequence, (unsigned long)slot_id,
             (unsigned)responder_index, (unsigned long long)reply_dtu);
    return ESP_OK;
}

static esp_err_t uwb_flex_tdoa_send_response_for_request(
    const struct uwb_distance_frame *request, const uint8_t *anchor_ids,
    size_t anchor_count)
{
    uint32_t slot_id = 0;
    size_t responder_index = SIZE_MAX;
    if (!uwb_flex_tdoa_parse_local_response_request(
            request, anchor_ids, anchor_count, &slot_id, &responder_index)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (responder_index == SIZE_MAX) {
        return ESP_OK;
    }
    return uwb_flex_tdoa_send_response_preparsed(request, slot_id,
                                                 responder_index);
}

static void uwb_flex_tdoa_handle_response_measurement(
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count)
{
    uint32_t slot_id = 0;
    uint8_t initiator_id = 0;
    uint8_t responder_id = 0;
    uint8_t responder_index = 0;
    uint64_t reply_dtu = 0;
    uint8_t previous_responder_id = 0;
    int32_t previous_distance_mm = 0;
    uint32_t previous_slot_id = 0;
    if (!uwb_flex_tdoa_parse_response(
            frame, anchor_ids, anchor_count, &slot_id, &initiator_id,
            &responder_id, &responder_index, &reply_dtu,
            &previous_responder_id, &previous_distance_mm,
            &previous_slot_id)) {
        return;
    }
    uwb_flex_tdoa_accept_piggyback(
        frame, anchor_ids, anchor_count, previous_responder_id,
        previous_distance_mm, previous_slot_id);

    if (!s_flex_tdoa_local_request.active ||
        s_flex_tdoa_local_request.sequence != frame->sequence ||
        s_flex_tdoa_local_request.slot_id != slot_id ||
        s_flex_tdoa_local_request.initiator_id != s_source_id ||
        initiator_id != s_source_id ||
        uwb_flex_tdoa_responder_index(
            s_flex_tdoa_local_request.responders,
            s_flex_tdoa_local_request.responder_count, responder_id) ==
            SIZE_MAX) {
        return;
    }

    const uint64_t response_rx_timestamp = frame->rx_timestamp;
    const uint64_t round_dtu_raw = uwb_distance_delta_ts(
        response_rx_timestamp, s_flex_tdoa_local_request.tx_timestamp);
    const double round_dtu = (double)round_dtu_raw;
    const int64_t round_us = uwb_dw3000_dtu_to_us(round_dtu_raw);
    const int64_t reply_us = uwb_dw3000_dtu_to_us(reply_dtu);
    if (round_us < 0 || round_us > 20000 || reply_us < 0 ||
        reply_us > 20000) {
        s_flex_tdoa_anchor_incoherent_since_summary++;
        ESP_LOGD(TAG,
                 "FLEX_TDOA incoherent response pair=%u-%u seq=%u index=%u "
                 "buffer=%u rdb=0x%02x tx=0x%010llx rx=0x%010llx "
                 "round=%lld us reply=%lld us",
                 (unsigned)initiator_id, (unsigned)responder_id,
                 (unsigned)frame->sequence, (unsigned)responder_index,
                 (unsigned)frame->rx_buffer_index,
                 (unsigned)frame->rx_buffer_status,
                 (unsigned long long)s_flex_tdoa_local_request.tx_timestamp,
                 (unsigned long long)response_rx_timestamp,
                 (long long)round_us, (long long)reply_us);
        if (s_flex_tdoa_local_request.responses_seen < UINT8_MAX) {
            s_flex_tdoa_local_request.responses_seen++;
        }
        if (s_flex_tdoa_local_request.responses_seen >=
            s_flex_tdoa_local_request.responder_count) {
            s_flex_tdoa_local_request.active = false;
        }
        return;
    }
    if (!frame->clock_offset_valid) {
        // The paper's SS-TWR range uses the CFO-corrected responder delay.
        // Do not feed an uncorrected range into anchor self-localization.
        s_flex_tdoa_anchor_drops_since_summary++;
        return;
    }
    const double raw_tof_dtu = (round_dtu - (double)reply_dtu) / 2.0;
    const double clock_offset_ratio =
        frame->clock_offset_from_cia
            ? uwb_dw3000_cia_clock_offset_ratio(frame->clock_offset_raw)
            : uwb_dw3000_clock_offset_ratio(frame->clock_offset_raw);
    const double corrected_reply_dtu =
        uwb_dw3000_remote_interval_in_local_dtu((double)reply_dtu,
                                                clock_offset_ratio);
    const double tof_dtu = (round_dtu - corrected_reply_dtu) / 2.0;
    const double distance_m = uwb_distance_tof_to_meters(tof_dtu);
    const double raw_distance_m = uwb_distance_tof_to_meters(raw_tof_dtu);
    const int32_t distance_mm = uwb_distance_meters_to_mm(distance_m);
    const int32_t raw_distance_mm = uwb_distance_meters_to_mm(raw_distance_m);

    if (distance_mm <= 0) {
        ESP_LOGD(TAG,
                 "FLEX_TDOA anchor result ignored pair=%u-%u seq=%u round=%.2f dtu reply=%.2f dtu",
                 (unsigned)initiator_id, (unsigned)responder_id,
                 (unsigned)frame->sequence, round_dtu, (double)reply_dtu);
        return;
    }

    const int32_t stored_distance_mm = uwb_flex_tdoa_store_anchor_distance(
        initiator_id, responder_id, frame->sequence, slot_id, distance_mm,
        raw_distance_mm);
    if (stored_distance_mm > 0) {
        if (responder_index <
            (sizeof(s_flex_tdoa_anchor_index_since_summary) /
             sizeof(s_flex_tdoa_anchor_index_since_summary[0]))) {
            s_flex_tdoa_anchor_index_since_summary[responder_index]++;
        }
        uwb_flex_tdoa_log_anchor_result(
            initiator_id, responder_id, frame->sequence, slot_id,
            distance_m, raw_distance_m);
    }

    if (s_flex_tdoa_local_request.responses_seen < UINT8_MAX) {
        s_flex_tdoa_local_request.responses_seen++;
    }
    if (s_flex_tdoa_local_request.responses_seen >=
        s_flex_tdoa_local_request.responder_count) {
        s_flex_tdoa_local_request.active = false;
    }
}

static void uwb_flex_tdoa_log_anchor_result(
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, double distance_m, double raw_distance_m)
{
    const bool queued = wireless_telemetry_service_submit_flex_anchor_range(
        initiator_id, responder_id, sequence, slot_id,
        uwb_distance_meters_to_mm(distance_m),
        uwb_distance_meters_to_mm(raw_distance_m));
    s_flex_tdoa_anchor_results_since_summary++;
    if (!queued) {
        s_flex_tdoa_anchor_drops_since_summary++;
    }

    const TickType_t now = xTaskGetTickCount();
    if (s_flex_tdoa_anchor_summary_tick == 0 ||
        now - s_flex_tdoa_anchor_summary_tick >= pdMS_TO_TICKS(1000)) {
        (void)wireless_log_service_submit('I', TAG,
                 "FLEX_TDOA anchor n=%lu drop=%lu incoh=%lu idx=%lu/%lu/%lu "
                 "txresp=%lu/%lu/%lu/%lu "
                 "rxerr=%lu/0x%08lx",
                 (unsigned long)s_flex_tdoa_anchor_results_since_summary,
                 (unsigned long)s_flex_tdoa_anchor_drops_since_summary,
                 (unsigned long)s_flex_tdoa_anchor_incoherent_since_summary,
                 (unsigned long)s_flex_tdoa_anchor_index_since_summary[0],
                 (unsigned long)s_flex_tdoa_anchor_index_since_summary[1],
                 (unsigned long)s_flex_tdoa_anchor_index_since_summary[2],
                 (unsigned long)s_flex_tdoa_response_tx_by_initiator[0],
                 (unsigned long)s_flex_tdoa_response_tx_by_initiator[1],
                 (unsigned long)s_flex_tdoa_response_tx_by_initiator[2],
                 (unsigned long)s_flex_tdoa_response_tx_by_initiator[3],
                 (unsigned long)s_flex_tdoa_rx_errors_since_summary,
                 (unsigned long)s_flex_tdoa_rx_error_status_since_summary);
        s_flex_tdoa_anchor_results_since_summary = 0;
        s_flex_tdoa_anchor_drops_since_summary = 0;
        s_flex_tdoa_anchor_incoherent_since_summary = 0;
        memset(s_flex_tdoa_anchor_index_since_summary, 0,
               sizeof(s_flex_tdoa_anchor_index_since_summary));
        memset(s_flex_tdoa_response_tx_by_initiator, 0,
               sizeof(s_flex_tdoa_response_tx_by_initiator));
        s_flex_tdoa_rx_errors_since_summary = 0;
        s_flex_tdoa_rx_error_status_since_summary = 0;
        s_flex_tdoa_anchor_summary_tick = now;
    }
}

static esp_err_t uwb_flex_tdoa_send_request(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t slot_id,
    uint16_t sequence,
    uint64_t scheduled_tx_ts, uint64_t *actual_tx_ts)
{
    uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
    const size_t responder_count = uwb_flex_tdoa_build_responder_list(
        anchor_ids, anchor_count, s_source_id, slot_id, responders);
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    const size_t payload_len =
        uwb_flex_tdoa_build_request(responders, responder_count, slot_id,
                                    payload);
    if (payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t tx_timestamp = 0;
    esp_err_t err = ESP_OK;
    if (scheduled_tx_ts != 0) {
        err = uwb_dw3000_send_payload_delayed_timeout(
            payload, payload_len, scheduled_tx_ts,
            UWB_FLEX_TDOA_TX_TIMEOUT_MS, NULL, &tx_timestamp);
    } else {
        esp_rom_delay_us(app_runtime_config_get()->flex_tdoa_guard_us);
        err = uwb_dw3000_send_payload(payload, payload_len, &tx_timestamp);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "FLEX_TDOA request TX failed slot=%u slot_id=%lu seq=%u responders=%u: %s",
                 (unsigned)(slot_id % (uint32_t)anchor_count),
                 (unsigned long)slot_id,
                 (unsigned)sequence, (unsigned)responder_count,
                 esp_err_to_name(err));
        return err;
    }

    if (actual_tx_ts != NULL) {
        *actual_tx_ts = tx_timestamp;
    }

    memset(&s_flex_tdoa_local_request, 0, sizeof(s_flex_tdoa_local_request));
    s_flex_tdoa_local_request.active = true;
    s_flex_tdoa_local_request.sequence = sequence;
    s_flex_tdoa_local_request.slot_id = slot_id;
    s_flex_tdoa_local_request.initiator_id = s_source_id;
    s_flex_tdoa_local_request.responder_count = (uint8_t)responder_count;
    memcpy(s_flex_tdoa_local_request.responders, responders, responder_count);
    s_flex_tdoa_local_request.tx_timestamp = tx_timestamp;
    s_flex_tdoa_local_request.updated_tick = xTaskGetTickCount();

    ESP_LOGD(TAG,
             "FLEX_TDOA request slot=%u slot_id=%lu seq=%u initiator=%u responders=%u slot_body=%lu us scheduled=%u",
             (unsigned)(slot_id % (uint32_t)anchor_count),
             (unsigned long)slot_id,
             (unsigned)sequence, (unsigned)s_source_id,
             (unsigned)responder_count,
             (unsigned long)uwb_flex_tdoa_slot_duration_us(
                 responder_count),
             scheduled_tx_ts != 0 ? 1U : 0U);
    return ESP_OK;
}

static uint64_t uwb_flex_tdoa_slot_delta_us(
    uint32_t from_slot_id, uint32_t to_slot_id, size_t responder_count)
{
    const uint32_t slot_delta = to_slot_id - from_slot_id;
    const uint32_t slot_duration_us =
        uwb_flex_tdoa_slot_duration_us(responder_count);
    return (uint64_t)slot_delta * (uint64_t)slot_duration_us;
}

static uint32_t uwb_flex_tdoa_next_local_slot_id(
    uint32_t reference_slot_id, uint8_t source_id)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    for (uint32_t delta = 1U;
         delta <= (uint32_t)config->flex_tdoa_slot_count; ++delta) {
        const uint32_t candidate = reference_slot_id + delta;
        if (uwb_flex_tdoa_slot_initiator(candidate) == source_id) {
            return candidate;
        }
    }
    return UINT32_MAX;
}

static void uwb_flex_tdoa_schedule_from_phase(
    struct uwb_flex_tdoa_schedule *schedule, uint32_t slot_id,
    uint64_t request_phase_radio_ts, int64_t request_phase_host_us,
    bool from_request, size_t anchor_count)
{
    if (schedule->synced) {
        const int32_t order = (int32_t)(slot_id - schedule->reference_slot_id);
        if (order < 0 ||
            (order == 0 &&
             (schedule->reference_from_request || !from_request))) {
            return;
        }
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t next_slot_id =
        uwb_flex_tdoa_next_local_slot_id(slot_id, s_source_id);
    if (next_slot_id == UINT32_MAX) {
        schedule->synced = false;
        return;
    }
    const uint64_t delta_us = uwb_flex_tdoa_slot_delta_us(
        slot_id, next_slot_id, config->flex_tdoa_responder_count);

    schedule->synced = true;
    schedule->reference_from_request = from_request;
    schedule->reference_slot_id = slot_id;
    schedule->next_slot_id = next_slot_id;
    schedule->next_request_radio_ts = uwb_dw3000_add_timestamp_delta(
        request_phase_radio_ts, uwb_dw3000_us_to_dtu((uint32_t)delta_us));
    schedule->next_request_host_us = request_phase_host_us + (int64_t)delta_us;
    schedule->last_sync_host_us = esp_timer_get_time();
    schedule->sync_count++;
}

static void uwb_flex_tdoa_schedule_from_frame(
    struct uwb_flex_tdoa_schedule *schedule,
    const struct uwb_distance_frame *frame, const uint8_t *anchor_ids,
    size_t anchor_count)
{
    uint32_t slot_id = 0;
    uint8_t initiator_id = 0;
    uint64_t request_phase_radio_ts = 0;
    int64_t request_phase_host_us =
        frame->rx_host_time_us != 0 ? frame->rx_host_time_us
                                    : esp_timer_get_time();
    bool from_request = false;

    if (frame->type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ) {
        uint8_t responder_count = 0;
        uint8_t responders[UWB_ANCHOR_SURVEY_MAX_ANCHORS - 1U] = {0};
        if (!uwb_flex_tdoa_parse_request(frame, anchor_ids, anchor_count,
                                         &slot_id, &responder_count,
                                         responders, NULL)) {
            return;
        }
        initiator_id = frame->source_id;
        request_phase_radio_ts = frame->rx_timestamp;
        from_request = true;
    } else if (frame->type == UWB_DISTANCE_FRAME_FLEX_TDOA_RESP) {
        uint8_t responder_id = 0;
        uint8_t responder_index = 0;
        uint64_t reply_dtu = 0;
        uint8_t previous_responder_id = 0;
        int32_t previous_distance_mm = 0;
        uint32_t previous_slot_id = 0;
        if (!uwb_flex_tdoa_parse_response(
                frame, anchor_ids, anchor_count, &slot_id, &initiator_id,
                &responder_id, &responder_index, &reply_dtu,
                &previous_responder_id, &previous_distance_mm,
                &previous_slot_id)) {
            return;
        }
        double local_reply_dtu = (double)reply_dtu;
        if (frame->clock_offset_valid) {
            local_reply_dtu = uwb_dw3000_remote_interval_in_local_dtu(
                local_reply_dtu,
                frame->clock_offset_from_cia
                    ? uwb_dw3000_cia_clock_offset_ratio(
                          frame->clock_offset_raw)
                    : uwb_dw3000_clock_offset_ratio(
                          frame->clock_offset_raw));
        }
        const uint64_t local_reply = (uint64_t)(local_reply_dtu + 0.5);
        request_phase_radio_ts =
            uwb_dw3000_sub_timestamp_delta(frame->rx_timestamp, local_reply);
        request_phase_host_us -= uwb_dw3000_dtu_to_us(local_reply);
        (void)responder_id;
        (void)responder_index;
        (void)previous_responder_id;
        (void)previous_distance_mm;
        (void)previous_slot_id;
    } else {
        return;
    }

    const uint8_t expected_initiator =
        uwb_flex_tdoa_slot_initiator(slot_id);
    if (expected_initiator != initiator_id) {
        ESP_LOGD(TAG,
                 "FLEX_TDOA sync ignored slot=%lu source=%u expected=%u",
                 (unsigned long)slot_id, (unsigned)initiator_id,
                 (unsigned)expected_initiator);
        return;
    }

    uwb_flex_tdoa_schedule_from_phase(
        schedule, slot_id, request_phase_radio_ts, request_phase_host_us,
        from_request, anchor_count);
}

static void uwb_flex_tdoa_process_frame(
    const struct uwb_distance_frame *frame,
    struct uwb_flex_tdoa_schedule *schedule, const uint8_t *anchor_ids,
    size_t anchor_count)
{
    if (frame == NULL) {
        return;
    }

    switch (frame->type) {
    case UWB_DISTANCE_FRAME_FLEX_TDOA_REQ:
        (void)uwb_flex_tdoa_send_response_for_request(
            frame, anchor_ids, anchor_count);
        uwb_flex_tdoa_schedule_from_frame(schedule, frame, anchor_ids,
                                          anchor_count);
        break;
    case UWB_DISTANCE_FRAME_FLEX_TDOA_RESP:
        uwb_flex_tdoa_schedule_from_frame(schedule, frame, anchor_ids,
                                          anchor_count);
        uwb_flex_tdoa_handle_response_measurement(frame, anchor_ids,
                                                  anchor_count);
        break;
    default:
        break;
    }
}

static esp_err_t uwb_flex_tdoa_schedule_timer_init(void)
{
    if (s_flex_tdoa_schedule_timer != NULL) {
        return ESP_OK;
    }

    const esp_timer_create_args_t args = {
        .callback = uwb_flex_tdoa_schedule_alarm_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "flex_slot",
        .skip_unhandled_events = true,
    };
    return esp_timer_create(&args, &s_flex_tdoa_schedule_timer);
}

static void uwb_flex_tdoa_arm_schedule_alarm(int64_t request_due_host_us)
{
    (void)esp_timer_stop(s_flex_tdoa_schedule_timer);
    s_flex_tdoa_schedule_alarm_fired = false;
    const int64_t alarm_us = request_due_host_us -
                             UWB_FLEX_TDOA_REQUEST_TX_LEAD_US -
                             esp_timer_get_time();
    if (alarm_us > 0) {
        const esp_err_t err = esp_timer_start_once(
            s_flex_tdoa_schedule_timer, (uint64_t)alarm_us);
        if (err != ESP_OK) {
            s_flex_tdoa_schedule_alarm_fired = true;
            ESP_LOGW(TAG, "FLEX_TDOA schedule alarm failed: %s",
                     esp_err_to_name(err));
        }
    } else {
        s_flex_tdoa_schedule_alarm_fired = true;
    }
}

static void uwb_flex_tdoa_anchor_loop(uint8_t bootstrap_id,
                                       const uint8_t *anchor_ids,
                                       size_t anchor_count)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    struct uwb_flex_tdoa_schedule schedule = {0};
    bool owns_slot = false;
    for (size_t i = 0; i < config->flex_tdoa_slot_count; ++i) {
        if (config->flex_tdoa_slot_initiator_ids[i] == s_source_id) {
            schedule.own_slot_index = (uint8_t)i;
            owns_slot = true;
            break;
        }
    }

    if (uwb_flex_tdoa_schedule_timer_init() != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "FLEX_TDOA schedule timer init failed");
        return;
    }

    s_status = UWB_DW3000_STATUS_READY;
    int64_t bootstrap_deadline_us =
        esp_timer_get_time() +
        (s_runtime_hot_entry ? UWB_HOT_SWITCH_BOOTSTRAP_GUARD_US
                             : UWB_FLEX_TDOA_BOOTSTRAP_LISTEN_US);
    ESP_LOGI(TAG,
             "FLEX_TDOA CI-CR anchor active: source_id=%u bootstrap=%u slot_index=%u anchors=%u K=%u slot=%lu us frame=%lu us req_lead=%lu us",
             (unsigned)s_source_id, (unsigned)bootstrap_id,
             (unsigned)schedule.own_slot_index, (unsigned)anchor_count,
             (unsigned)config->flex_tdoa_responder_count,
             (unsigned long)uwb_flex_tdoa_slot_duration_us(
                 config->flex_tdoa_responder_count),
             (unsigned long)(config->flex_tdoa_slot_count *
                 uwb_flex_tdoa_slot_duration_us(
                     config->flex_tdoa_responder_count)),
             (unsigned long)UWB_FLEX_TDOA_REQUEST_TX_LEAD_US);

    while (!uwb_dw3000_runtime_switch_pending()) {
        config = app_runtime_config_get();
        const int64_t now_us = esp_timer_get_time();

        if (schedule.synced &&
            now_us >= schedule.next_request_host_us -
                          UWB_FLEX_TDOA_REQUEST_TX_LEAD_US) {
            const int64_t lateness_us =
                now_us - schedule.next_request_host_us;
            if (lateness_us > UWB_FLEX_TDOA_REQUEST_LATE_US) {
                schedule.synced = false;
                schedule.missed_slots++;
                s_flex_tdoa_local_request.active = false;
                bootstrap_deadline_us =
                    now_us + UWB_FLEX_TDOA_BOOTSTRAP_LISTEN_US;
                ESP_LOGW(TAG,
                         "FLEX_TDOA slot missed slot_id=%lu late=%lld us misses=%lu; listening for resync",
                         (unsigned long)schedule.next_slot_id,
                         (long long)lateness_us,
                         (unsigned long)schedule.missed_slots);
                continue;
            }

            const uint32_t slot_id = schedule.next_slot_id;
            uint64_t actual_tx_ts = 0;
            const esp_err_t err = uwb_flex_tdoa_send_request(
                anchor_ids, anchor_count, slot_id,
                (uint16_t)(slot_id & 0xFFFFU), schedule.next_request_radio_ts,
                &actual_tx_ts);
            if (err != ESP_OK) {
                schedule.synced = false;
                schedule.missed_slots++;
                bootstrap_deadline_us =
                    esp_timer_get_time() + UWB_FLEX_TDOA_BOOTSTRAP_LISTEN_US;
                ESP_LOGW(TAG,
                         "FLEX_TDOA scheduled request failed slot_id=%lu misses=%lu: %s",
                         (unsigned long)slot_id,
                         (unsigned long)schedule.missed_slots,
                         esp_err_to_name(err));
                continue;
            }

            int64_t actual_tx_host_us = esp_timer_get_time();
            uwb_flex_tdoa_schedule_from_phase(
                &schedule, slot_id, actual_tx_ts, actual_tx_host_us, true,
                anchor_count);
            continue;
        }

        if (owns_slot && !schedule.synced &&
            now_us >= bootstrap_deadline_us +
                (int64_t)schedule.own_slot_index *
                    (int64_t)uwb_flex_tdoa_slot_duration_us(
                        config->flex_tdoa_responder_count)) {
            const uint32_t bootstrap_slot_id = schedule.own_slot_index;
            uint64_t actual_tx_ts = 0;
            const esp_err_t err = uwb_flex_tdoa_send_request(
                anchor_ids, anchor_count, bootstrap_slot_id,
                (uint16_t)bootstrap_slot_id, 0, &actual_tx_ts);
            if (err == ESP_OK) {
                int64_t actual_tx_host_us = esp_timer_get_time();
                uwb_flex_tdoa_schedule_from_phase(
                    &schedule, bootstrap_slot_id, actual_tx_ts,
                    actual_tx_host_us, true, anchor_count);
                ESP_LOGI(TAG,
                         "FLEX_TDOA distributed schedule bootstrapped by anchor=%u slot=%lu",
                         (unsigned)s_source_id,
                         (unsigned long)bootstrap_slot_id);
            } else {
                bootstrap_deadline_us =
                    esp_timer_get_time() + UWB_FLEX_TDOA_BOOTSTRAP_LISTEN_US;
            }
            continue;
        }

        if (schedule.synced) {
            uwb_flex_tdoa_arm_schedule_alarm(schedule.next_request_host_us);
        } else {
            (void)esp_timer_stop(s_flex_tdoa_schedule_timer);
            s_flex_tdoa_schedule_alarm_fired = false;
        }

        struct uwb_distance_frame frame = {0};
        const esp_err_t err = uwb_distance_receive_next(
            &frame, config->anchor_survey_rx_slice_ms);
        const bool urgent_request =
            err == ESP_OK &&
            frame.type == UWB_DISTANCE_FRAME_FLEX_TDOA_REQ;
        if (urgent_request) {
            // Arm delayed TX before stopping the host schedule timer or
            // running generic frame work; RESP[0] uses the configured window.
            uwb_flex_tdoa_process_frame(&frame, &schedule, anchor_ids,
                                        anchor_count);
            const esp_err_t release_err =
                uwb_dw3000_release_pending_flex_request();
            if (release_err != ESP_OK) {
                ESP_LOGE(TAG, "FlexTDOA REQ buffer release failed: %s",
                         esp_err_to_name(release_err));
                s_rx_error_count++;
            }
        }
        bool config_frame = false;
        if (err == ESP_OK && !urgent_request) {
            config_frame = uwb_flex_tdoa_handle_runtime_config(&frame);
        }
        (void)esp_timer_stop(s_flex_tdoa_schedule_timer);
        const bool schedule_alarm = s_flex_tdoa_schedule_alarm_fired;
        s_flex_tdoa_schedule_alarm_fired = false;

        if (err == ESP_OK) {
            if (urgent_request) {
                continue;
            }
            if (config_frame) {
                continue;
            }
            if (schedule.synced &&
                esp_timer_get_time() >=
                    schedule.next_request_host_us -
                        UWB_FLEX_TDOA_REQUEST_TX_LEAD_US) {
                continue;
            }
            uwb_flex_tdoa_process_frame(&frame, &schedule, anchor_ids,
                                        anchor_count);
        } else if (err == ESP_ERR_NOT_FINISHED && schedule_alarm) {
            continue;
        } else if (err != ESP_ERR_TIMEOUT &&
                   err != ESP_ERR_INVALID_RESPONSE &&
                   !uwb_dw3000_runtime_switch_pending()) {
            ESP_LOGW(TAG, "FLEX_TDOA distributed RX failed: %s",
                     esp_err_to_name(err));
        }
    }
    (void)esp_timer_stop(s_flex_tdoa_schedule_timer);
    s_flex_tdoa_schedule_alarm_fired = false;
}

static void uwb_flex_tdoa_configuration_phase(void)
{
    const int64_t start_us = esp_timer_get_time();
    const int64_t end_us = start_us + UWB_FLEX_TDOA_CONFIG_PHASE_US;
    int64_t next_tx_us = start_us;
    uint32_t tx_count = 0;
    uint32_t rx_count = 0;

    while (!uwb_dw3000_runtime_switch_pending() &&
           esp_timer_get_time() < end_us) {
        const app_runtime_config_t *config = app_runtime_config_get();
        int own_index = -1;
        for (size_t i = 0; i < config->anchor_count; ++i) {
            if (config->anchor_ids[i] == s_source_id) {
                own_index = (int)i;
                break;
            }
        }

        // The tag and a node removed by the new topology must still be able to
        // introduce that topology. Configured anchors occupy [0, N), the tag
        // uses N, and any other local ID uses the final recovery position.
        size_t participant_count = config->anchor_count + 2U;
        if (own_index < 0) {
            own_index = config->tag_id == s_source_id
                            ? (int)config->anchor_count
                            : (int)config->anchor_count + 1;
        }

        const int64_t cycle_us =
            (int64_t)participant_count *
            UWB_FLEX_TDOA_CONFIG_TX_SPACING_US;
        if (esp_timer_get_time() >= next_tx_us) {
            const int64_t elapsed = esp_timer_get_time() - start_us;
            const int64_t cycle_start =
                start_us + (elapsed / cycle_us) * cycle_us;
            const int64_t own_due =
                cycle_start +
                (int64_t)own_index * UWB_FLEX_TDOA_CONFIG_TX_SPACING_US;
            if (esp_timer_get_time() >= own_due) {
                if (uwb_flex_tdoa_send_config() == ESP_OK) {
                    const app_runtime_config_t *active =
                        app_runtime_config_get();
                    const size_t geometry_index =
                        active->anchor_count > 0U
                            ? tx_count % active->anchor_count
                            : 0U;
                    // The receiver must process the topology frame and rearm
                    // the DW3000 before the following geometry frame arrives.
                    uwb_dw3000_delay_ms(
                        UWB_FLEX_TDOA_CONFIG_FRAME_GAP_MS);
                    (void)uwb_flex_tdoa_send_geometry(geometry_index);
                    tx_count++;
                }
                next_tx_us = cycle_start + cycle_us +
                             (int64_t)own_index *
                                 UWB_FLEX_TDOA_CONFIG_TX_SPACING_US;
            } else {
                next_tx_us = own_due;
            }
        }

        struct uwb_distance_frame frame = {0};
        const esp_err_t err = uwb_distance_receive_next(&frame, 20U);
        if (err == ESP_OK &&
            (frame.type == UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG ||
             frame.type == UWB_DISTANCE_FRAME_FLEX_TDOA_GEOMETRY)) {
            const enum uwb_flex_tdoa_config_result result =
                frame.type == UWB_DISTANCE_FRAME_FLEX_TDOA_CONFIG
                    ? uwb_flex_tdoa_apply_config_frame(&frame)
                    : uwb_flex_tdoa_apply_geometry_frame(&frame);
            if (result != UWB_FLEX_TDOA_CONFIG_INVALID) {
                rx_count++;
            }
        } else if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "FlexTDOA config RX: %s", esp_err_to_name(err));
        }
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    ESP_LOGI(TAG,
             "FlexTDOA radio configuration complete generation=%lu N=%u K=%u M=%u tx=%lu rx=%lu",
             (unsigned long)config->flex_tdoa_config_generation,
             (unsigned)config->anchor_count,
             (unsigned)config->flex_tdoa_responder_count,
             (unsigned)config->flex_tdoa_slot_count,
             (unsigned long)tx_count, (unsigned long)rx_count);
}

static void uwb_dw3000_flex_tdoa_loop(void)
{
    uwb_flex_tdoa_runtime_reset();
    if (s_runtime_hot_entry) {
        ESP_LOGI(TAG,
                 "FlexTDOA hot entry: unchanged topology, skipping 3 s radio configuration phase");
    } else {
        uwb_flex_tdoa_configuration_phase();
    }
    if (uwb_dw3000_runtime_switch_pending()) {
        return;
    }
    uint8_t anchor_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS] = {0};
    const app_runtime_config_t *config = app_runtime_config_get();
    const size_t anchor_count = app_runtime_config_get_anchor_ids(
        anchor_ids, UWB_ANCHOR_SURVEY_MAX_ANCHORS);
    const uint8_t tag_id = config->tag_id;
    const uint8_t coordinator_id = anchor_count > 0 ? anchor_ids[0] : 0;

    ESP_LOGI(TAG,
             "FLEX_TDOA runtime start: source_id=%u tag_id=%u coordinator=%u N=%u K=%u M=%u anchors=[%u,%u,%u,%u]",
             (unsigned)s_source_id, (unsigned)tag_id, (unsigned)coordinator_id,
             (unsigned)anchor_count,
             (unsigned)config->flex_tdoa_responder_count,
             (unsigned)config->flex_tdoa_slot_count,
             (unsigned)anchor_ids[0], (unsigned)anchor_ids[1],
             (unsigned)anchor_ids[2], (unsigned)anchor_ids[3]);

    if (anchor_count < 3 || anchor_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS ||
        !uwb_anchor_survey_ids_valid(anchor_ids, anchor_count,
                                     coordinator_id)) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "FLEX_TDOA invalid anchor configuration");
        vTaskDelete(NULL);
        return;
    }
    if (uwb_anchor_survey_id_in_set(anchor_ids, anchor_count, s_source_id)) {
        uwb_flex_tdoa_anchor_loop(coordinator_id, anchor_ids, anchor_count);
        return;
    }

    // FlexTDOA tags never transmit. Every node outside the anchor set can
    // listen and solve independently without consuming a radio slot.
    uwb_flex_tdoa_tag_loop(anchor_ids, anchor_count);
}

static bool uwb_passive_ds_slot_pair(
    uint32_t slot_id, const uint8_t *anchor_ids, size_t anchor_count,
    uint8_t schedule_mode, uint8_t *initiator_id, uint8_t *responder_id,
    uint8_t *responder_index)
{
    if (anchor_ids == NULL || anchor_count < 2U ||
        anchor_count > UWB_ANCHOR_SURVEY_MAX_ANCHORS ||
        initiator_id == NULL || responder_id == NULL) {
        return false;
    }

    const uint32_t slots_per_frame = (uint32_t)anchor_count - 1U;
    const uint32_t frame_id = slot_id / slots_per_frame;
    const uint32_t index = slot_id % slots_per_frame;
    size_t initiator_index = 0U;
    if (schedule_mode == APP_RUNTIME_PASSIVE_DS_ROBUST_ROTATING) {
        initiator_index = (size_t)(frame_id % (uint32_t)anchor_count);
    } else if ((frame_id + 1U) %
                   UWB_PASSIVE_DS_FAST_GEOMETRY_FRAME_INTERVAL ==
               0U) {
        /*
         * Fast Star keeps anchor[0] as the position reference for most
         * frames. One maintenance frame in four rotates through the other
         * anchors, making every pair observable without adding radio slots.
         */
        const uint32_t maintenance_frame =
            frame_id / UWB_PASSIVE_DS_FAST_GEOMETRY_FRAME_INTERVAL;
        initiator_index =
            1U + (size_t)(maintenance_frame %
                          ((uint32_t)anchor_count - 1U));
    }
    size_t candidate = 0U;
    for (size_t i = 0; i < anchor_count; ++i) {
        if (i == initiator_index) {
            continue;
        }
        if (candidate == index) {
            *initiator_id = anchor_ids[initiator_index];
            *responder_id = anchor_ids[i];
            if (responder_index != NULL) {
                *responder_index = (uint8_t)candidate;
            }
            return true;
        }
        candidate++;
    }
    return false;
}

static const char *uwb_passive_ds_solve_mode_name(uint8_t solve_mode)
{
    switch (solve_mode) {
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_ALL:
        return "rolling_ekf_all";
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_INDEPENDENT:
        return "rolling_ekf_independent";
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_SUPERFRAME:
        return "rolling_ekf_superframe";
    case APP_RUNTIME_PASSIVE_DS_SOLVE_ROLLING_MOTION:
        return "rolling_motion_compensated";
    case APP_RUNTIME_PASSIVE_DS_SOLVE_FRAME:
    default:
        return "frame";
    }
}

static int64_t uwb_passive_ds_next_slot_delta_us(
    uint32_t slot_id, const app_runtime_config_t *config,
    size_t anchor_count)
{
    const uint32_t slots_per_frame = (uint32_t)anchor_count - 1U;
    int64_t delta_us = (int64_t)config->passive_ds_slot_ms * 1000LL;
    if ((slot_id % slots_per_frame) == slots_per_frame - 1U) {
        delta_us += (int64_t)config->passive_ds_round_gap_ms * 1000LL;
    }
    return delta_us;
}

static void uwb_passive_ds_schedule_from_poll(
    struct uwb_passive_ds_schedule *schedule, uint32_t slot_id,
    uint64_t poll_radio_ts, int64_t poll_host_us,
    const uint8_t *anchor_ids, size_t anchor_count,
    const app_runtime_config_t *config)
{
    if (schedule == NULL || config == NULL || anchor_count < 2U) {
        return;
    }

    int64_t due_us = poll_host_us;
    uint64_t due_radio_ts = poll_radio_ts;
    uint32_t candidate_slot = slot_id;
    const uint32_t search_limit =
        (uint32_t)(anchor_count * (anchor_count - 1U)) + 1U;
    for (uint32_t i = 0; i < search_limit; ++i) {
        const int64_t slot_delta_us = uwb_passive_ds_next_slot_delta_us(
            candidate_slot, config, anchor_count);
        due_us += slot_delta_us;
        due_radio_ts = uwb_dw3000_add_timestamp_delta(
            due_radio_ts, uwb_dw3000_us_to_dtu((uint32_t)slot_delta_us));
        candidate_slot++;
        uint8_t initiator_id = 0;
        uint8_t responder_id = 0;
        if (uwb_passive_ds_slot_pair(
                candidate_slot, anchor_ids, anchor_count,
                config->passive_ds_schedule, &initiator_id, &responder_id,
                NULL) &&
            initiator_id == s_source_id) {
            schedule->synced = true;
            schedule->next_owned_slot_id = candidate_slot;
            schedule->next_owned_poll_radio_ts = due_radio_ts;
            schedule->next_owned_poll_host_us = due_us;
            schedule->last_poll_host_us = poll_host_us;
            return;
        }
    }

    schedule->synced = false;
}

static esp_err_t uwb_passive_ds_start_initiator(
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    uint64_t scheduled_poll_ts, struct uwb_passive_ds_exchange *exchange,
    uint64_t *actual_poll_tx_ts, int64_t *poll_host_us)
{
    if (exchange == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uint64_t poll_tx_ts = 0;
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_POLL,
                             responder_id, sequence, payload);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET, slot_id);
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_POLL_PIGGYBACK_OFFSET);

    const int64_t stage_started_us = esp_timer_get_time();
    esp_err_t err = ESP_OK;
    if (scheduled_poll_ts != 0U) {
        uint64_t programmed_poll_tx_ts = 0;
        err = uwb_dw3000_send_payload_delayed_expect_rx(
            payload, UWB_PASSIVE_DS_POLL_LEN, scheduled_poll_ts,
            config->passive_ds_auto_rx_delay_uus,
            config->passive_ds_rx_timeout_ms, &programmed_poll_tx_ts,
            &poll_tx_ts);
        const uint64_t expected_poll_tx_ts =
            uwb_dw3000_programmed_tx_timestamp(
                uwb_dw3000_delayed_time_word(scheduled_poll_ts));
        if (err == ESP_OK &&
            programmed_poll_tx_ts != expected_poll_tx_ts) {
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        err = uwb_dw3000_send_payload_expect_rx(
            payload, UWB_PASSIVE_DS_POLL_LEN,
            config->passive_ds_auto_rx_delay_uus,
            config->passive_ds_rx_timeout_ms, &poll_tx_ts);
    }
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_POLL_TX, stage_started_us,
        err == ESP_OK);

    const int64_t local_poll_host_us = esp_timer_get_time();
    if (poll_host_us != NULL) {
        *poll_host_us = local_poll_host_us;
    }
    if (err != ESP_OK) {
        return err;
    }

    memset(exchange, 0, sizeof(*exchange));
    exchange->state = UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE;
    exchange->sequence = sequence;
    exchange->slot_id = slot_id;
    exchange->peer_id = responder_id;
    exchange->started_host_us = local_poll_host_us;
    exchange->deadline_host_us =
        local_poll_host_us +
        (int64_t)config->passive_ds_rx_timeout_ms * 1000LL;
    exchange->poll_tx_ts = poll_tx_ts;
    if (actual_poll_tx_ts != NULL) {
        *actual_poll_tx_ts = poll_tx_ts;
    }
    return ESP_OK;
}

static esp_err_t uwb_passive_ds_complete_initiator_response(
    struct uwb_passive_ds_exchange *exchange,
    const struct uwb_distance_frame *response)
{
    if (exchange == NULL || response == NULL ||
        exchange->state != UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE ||
        response->type != UWB_DISTANCE_FRAME_PASSIVE_DS_RESP ||
        response->source_id != exchange->peer_id ||
        response->destination_id != s_source_id ||
        response->sequence != exchange->sequence) {
        return ESP_ERR_INVALID_ARG;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    if (response->payload_len < UWB_PASSIVE_DS_RESP_LEN ||
        uwb_distance_get_u32(response->payload,
                             UWB_PASSIVE_DS_SLOT_ID_OFFSET) !=
            exchange->slot_id ||
        uwb_distance_get_u32(response->payload,
                             UWB_PASSIVE_DS_REPLY_DTU_OFFSET) == 0U) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
        exchange->state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint64_t final_tx_due = uwb_dw3000_add_timestamp_delta(
        response->rx_timestamp,
        uwb_dw3000_us_to_dtu(config->passive_ds_final_delay_us));
    const uint64_t expected_final_tx_ts =
        uwb_dw3000_programmed_tx_timestamp(
            uwb_dw3000_delayed_time_word(final_tx_due));
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL,
                             exchange->peer_id, exchange->sequence,
                             payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          exchange->poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          response->rx_timestamp);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          expected_final_tx_ts);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET,
                         exchange->slot_id);
    uint64_t programmed_final_tx_ts = 0;
    uint64_t actual_final_tx_ts = 0;
    const int64_t stage_started_us = esp_timer_get_time();
    const esp_err_t err = uwb_dw3000_send_payload_delayed(
        payload, UWB_PASSIVE_DS_FINAL_LEN, final_tx_due,
        &programmed_final_tx_ts, &actual_final_tx_ts);
    const bool success =
        err == ESP_OK && programmed_final_tx_ts == expected_final_tx_ts;
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_TX, stage_started_us, success);
    exchange->state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
    (void)actual_final_tx_ts;
    return success ? ESP_OK
                   : err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
}

static esp_err_t uwb_passive_ds_start_responder(
    const struct uwb_distance_frame *poll, uint32_t slot_id,
    struct uwb_passive_ds_exchange *exchange)
{
    if (poll == NULL || exchange == NULL ||
        poll->type != UWB_DISTANCE_FRAME_PASSIVE_DS_POLL) {
        return ESP_ERR_INVALID_ARG;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint64_t resp_tx_due = uwb_dw3000_add_timestamp_delta(
        poll->rx_timestamp,
        uwb_dw3000_us_to_dtu(config->passive_ds_resp_delay_us));
    const uint64_t expected_resp_tx_ts =
        uwb_dw3000_programmed_tx_timestamp(
            uwb_dw3000_delayed_time_word(resp_tx_due));
    const uint64_t reply_dtu =
        uwb_distance_delta_ts(expected_resp_tx_ts, poll->rx_timestamp);
    if (reply_dtu == 0U || reply_dtu > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_RESP,
                             poll->source_id, poll->sequence, payload);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET, slot_id);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_REPLY_DTU_OFFSET,
                         (uint32_t)reply_dtu);
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_RESP_PIGGYBACK_OFFSET);
    uint64_t programmed_resp_tx_ts = 0;
    uint64_t actual_resp_tx_ts = 0;
    const int64_t stage_started_us = esp_timer_get_time();
    const esp_err_t err = uwb_dw3000_send_payload_delayed_expect_rx(
        payload, UWB_PASSIVE_DS_RESP_LEN, resp_tx_due,
        config->passive_ds_auto_rx_delay_uus,
        config->passive_ds_rx_timeout_ms, &programmed_resp_tx_ts,
        &actual_resp_tx_ts);
    const bool success =
        err == ESP_OK && programmed_resp_tx_ts == expected_resp_tx_ts;
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_RESPONSE_TX, stage_started_us,
        success);
    if (!success) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    memset(exchange, 0, sizeof(*exchange));
    exchange->state = UWB_PASSIVE_DS_EXCHANGE_WAIT_FINAL;
    exchange->sequence = poll->sequence;
    exchange->slot_id = slot_id;
    exchange->peer_id = poll->source_id;
    exchange->started_host_us = esp_timer_get_time();
    exchange->deadline_host_us =
        exchange->started_host_us +
        (int64_t)config->passive_ds_rx_timeout_ms * 1000LL;
    exchange->poll_rx_ts = poll->rx_timestamp;
    exchange->response_tx_ts = programmed_resp_tx_ts;
    exchange->poll_rx_diagnostics = poll->diagnostics;
    (void)actual_resp_tx_ts;
    return ESP_OK;
}

static void uwb_passive_ds_remember_responder_exchange(
    uint8_t initiator_id, uint16_t sequence, uint32_t slot_id,
    uint64_t poll_rx_timestamp, uint64_t final_rx_timestamp,
    const struct uwb_distance_measurement *measurement)
{
    if (measurement == NULL) {
        return;
    }
    const uint64_t exchange_dtu = uwb_distance_delta_ts(
        final_rx_timestamp, poll_rx_timestamp);
    const int32_t distance_mm =
        uwb_distance_meters_to_mm(measurement->distance_m);
    const int32_t raw_distance_mm =
        uwb_distance_meters_to_mm(measurement->raw_distance_m);
    if (initiator_id == 0U || exchange_dtu == 0U ||
        exchange_dtu > UINT32_MAX || distance_mm <= 0 ||
        distance_mm > UINT16_MAX || raw_distance_mm <= 0 ||
        raw_distance_mm > UINT16_MAX) {
        return;
    }

    const struct uwb_passive_ds_completed_exchange completed = {
        .valid = true,
        .initiator_id = initiator_id,
        .sequence = sequence,
        .slot_id = slot_id,
        .responder_exchange_dtu = (uint32_t)exchange_dtu,
        .distance_mm = distance_mm,
        .raw_distance_mm = raw_distance_mm,
    };
    uwb_passive_ds_runtime_store_completed_exchange(&completed);
}

static esp_err_t uwb_passive_ds_complete_responder_final(
    struct uwb_passive_ds_exchange *exchange,
    const struct uwb_distance_frame *final,
    struct uwb_distance_measurement *measurement)
{
    if (exchange == NULL || final == NULL || measurement == NULL ||
        exchange->state != UWB_PASSIVE_DS_EXCHANGE_WAIT_FINAL ||
        final->type != UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL ||
        final->source_id != exchange->peer_id ||
        final->destination_id != s_source_id ||
        final->sequence != exchange->sequence) {
        return ESP_ERR_INVALID_ARG;
    }
    bool success =
        final->payload_len >= UWB_PASSIVE_DS_FINAL_LEN &&
        uwb_distance_get_u32(
            final->payload, UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET) ==
            exchange->slot_id;
    if (!success) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
    } else {
        uwb_distance_fill_measurement_from_timestamps(
            exchange->peer_id, s_source_id, exchange->sequence,
            uwb_distance_get_ts40(
                final->payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET),
            exchange->poll_rx_ts, exchange->response_tx_ts,
            uwb_distance_get_ts40(
                final->payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET),
            uwb_distance_get_ts40(
                final->payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET),
            final->rx_timestamp, final->clock_offset_valid,
            final->clock_offset_raw, measurement);
        measurement->poll_rx_diagnostics =
            exchange->poll_rx_diagnostics;
        measurement->final_rx_diagnostics = final->diagnostics;
        uwb_passive_ds_remember_responder_exchange(
            exchange->peer_id, exchange->sequence, exchange->slot_id,
            exchange->poll_rx_ts, final->rx_timestamp, measurement);
        (void)uwb_passive_ds_store_anchor_distance(
            s_source_id, measurement->initiator_id,
            measurement->sequence, exchange->slot_id,
            uwb_distance_meters_to_mm(measurement->distance_m),
            uwb_distance_meters_to_mm(measurement->raw_distance_m));
    }
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
        exchange->started_host_us, success);
    exchange->state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
    return success ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t uwb_passive_ds_initiate_once(
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    int64_t *poll_host_us)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uint64_t poll_tx_ts = 0;
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_POLL,
                             responder_id, sequence, payload);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET, slot_id);
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_POLL_PIGGYBACK_OFFSET);
    const int64_t poll_stage_started_us = esp_timer_get_time();
    esp_err_t err = uwb_dw3000_send_payload_expect_rx(
        payload, UWB_PASSIVE_DS_POLL_LEN,
        config->passive_ds_auto_rx_delay_uus,
        config->passive_ds_rx_timeout_ms, &poll_tx_ts);
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_POLL_TX, poll_stage_started_us,
        err == ESP_OK);
    const int64_t local_poll_host_us = esp_timer_get_time();
    if (poll_host_us != NULL) {
        *poll_host_us = local_poll_host_us;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "PASSIVE_DS POLL TX failed slot=%lu seq=%u responder=%u: %s",
                 (unsigned long)slot_id, (unsigned)sequence,
                 (unsigned)responder_id, esp_err_to_name(err));
        return err;
    }

    struct uwb_distance_frame response = {0};
    err = uwb_distance_receive_matching(
        UWB_DISTANCE_FRAME_PASSIVE_DS_RESP, responder_id, true, sequence,
        &response, config->passive_ds_rx_timeout_ms);
    if (err != ESP_OK) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT);
        ESP_LOGW(TAG,
                 "PASSIVE_DS RESP wait failed slot=%lu seq=%u responder=%u: %s",
                 (unsigned long)slot_id, (unsigned)sequence,
                 (unsigned)responder_id, esp_err_to_name(err));
        return err;
    }
    if (response.payload_len < UWB_PASSIVE_DS_RESP_LEN ||
        uwb_distance_get_u32(response.payload,
                             UWB_PASSIVE_DS_SLOT_ID_OFFSET) != slot_id ||
        uwb_distance_get_u32(response.payload,
                             UWB_PASSIVE_DS_REPLY_DTU_OFFSET) == 0U) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint64_t final_tx_due = uwb_dw3000_add_timestamp_delta(
        response.rx_timestamp,
        uwb_dw3000_us_to_dtu(config->passive_ds_final_delay_us));
    const uint64_t expected_final_tx_ts = uwb_dw3000_programmed_tx_timestamp(
        uwb_dw3000_delayed_time_word(final_tx_due));
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL,
                             responder_id, sequence, payload);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET,
                          poll_tx_ts);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET,
                          response.rx_timestamp);
    uwb_distance_put_ts40(payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET,
                          expected_final_tx_ts);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET,
                         slot_id);
    uint64_t programmed_final_tx_ts = 0;
    uint64_t actual_final_tx_ts = 0;
    const int64_t final_stage_started_us = esp_timer_get_time();
    err = uwb_dw3000_send_payload_delayed(
        payload, UWB_PASSIVE_DS_FINAL_LEN, final_tx_due,
        &programmed_final_tx_ts, &actual_final_tx_ts);
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_TX, final_stage_started_us,
        err == ESP_OK && programmed_final_tx_ts == expected_final_tx_ts);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "PASSIVE_DS FINAL TX failed slot=%lu seq=%u responder=%u: %s",
                 (unsigned long)slot_id, (unsigned)sequence,
                 (unsigned)responder_id, esp_err_to_name(err));
        return err;
    }
    return programmed_final_tx_ts == expected_final_tx_ts
               ? ESP_OK
               : ESP_ERR_INVALID_STATE;
}

static esp_err_t uwb_passive_ds_respond_to_poll(
    const struct uwb_distance_frame *poll, uint32_t slot_id,
    struct uwb_distance_measurement *measurement)
{
    if (poll == NULL || measurement == NULL ||
        poll->type != UWB_DISTANCE_FRAME_PASSIVE_DS_POLL) {
        return ESP_ERR_INVALID_ARG;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint64_t resp_tx_due = uwb_dw3000_add_timestamp_delta(
        poll->rx_timestamp,
        uwb_dw3000_us_to_dtu(config->passive_ds_resp_delay_us));
    const uint64_t expected_resp_tx_ts = uwb_dw3000_programmed_tx_timestamp(
        uwb_dw3000_delayed_time_word(resp_tx_due));
    const uint64_t reply_dtu = uwb_distance_delta_ts(
        expected_resp_tx_ts, poll->rx_timestamp);
    if (reply_dtu == 0U || reply_dtu > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_RESP,
                             poll->source_id, poll->sequence, payload);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET, slot_id);
    uwb_distance_put_u32(payload, UWB_PASSIVE_DS_REPLY_DTU_OFFSET,
                         (uint32_t)reply_dtu);
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_RESP_PIGGYBACK_OFFSET);
    uint64_t programmed_resp_tx_ts = 0;
    uint64_t actual_resp_tx_ts = 0;
    const int64_t response_stage_started_us = esp_timer_get_time();
    esp_err_t err = uwb_dw3000_send_payload_delayed_expect_rx(
        payload, UWB_PASSIVE_DS_RESP_LEN, resp_tx_due,
        config->passive_ds_auto_rx_delay_uus,
        config->passive_ds_rx_timeout_ms, &programmed_resp_tx_ts,
        &actual_resp_tx_ts);
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_RESPONSE_TX,
        response_stage_started_us,
        err == ESP_OK && programmed_resp_tx_ts == expected_resp_tx_ts);
    if (err != ESP_OK || programmed_resp_tx_ts != expected_resp_tx_ts) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    struct uwb_distance_frame final = {0};
    const int64_t final_rx_started_us = esp_timer_get_time();
    err = uwb_distance_receive_matching(
        UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL, poll->source_id, true,
        poll->sequence, &final, config->passive_ds_rx_timeout_ms);
    if (err != ESP_OK) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT);
        uwb_passive_ds_record_stage(
            UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
            final_rx_started_us, false);
        return err;
    }
    if (final.payload_len < UWB_PASSIVE_DS_FINAL_LEN ||
        uwb_distance_get_u32(final.payload,
                             UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET) != slot_id) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
        uwb_passive_ds_record_stage(
            UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
            final_rx_started_us, false);
        return ESP_ERR_INVALID_RESPONSE;
    }
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX, final_rx_started_us, true);

    uwb_distance_fill_measurement_from_timestamps(
        poll->source_id, s_source_id, poll->sequence,
        uwb_distance_get_ts40(final.payload,
                              UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET),
        poll->rx_timestamp, programmed_resp_tx_ts,
        uwb_distance_get_ts40(final.payload,
                              UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET),
        uwb_distance_get_ts40(final.payload,
                              UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET),
        final.rx_timestamp, final.clock_offset_valid,
        final.clock_offset_raw, measurement);
    measurement->poll_rx_diagnostics = poll->diagnostics;
    measurement->final_rx_diagnostics = final.diagnostics;
    uwb_passive_ds_remember_responder_exchange(
        poll->source_id, poll->sequence, slot_id, poll->rx_timestamp,
        final.rx_timestamp, measurement);
    (void)uwb_passive_ds_store_anchor_distance(
        s_source_id, measurement->initiator_id, measurement->sequence,
        slot_id, uwb_distance_meters_to_mm(measurement->distance_m),
        uwb_distance_meters_to_mm(measurement->raw_distance_m));
    (void)actual_resp_tx_ts;
    return ESP_OK;
}


static void uwb_passive_ds_tag_submit_double_sided(
    uint8_t tag_id, uint8_t initiator_id, uint8_t responder_id,
    uint16_t sequence, uint32_t slot_id,
    const struct uwb_passive_ds_tdoa_result *result,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (result == NULL || initiator_id == 0U || responder_id == 0U ||
        initiator_id == responder_id) {
        return;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    int32_t difference_mm = uwb_distance_meters_to_mm(
        result->difference_m);
    if (config->passive_ds_calibration_enabled) {
        const size_t initiator_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, initiator_id);
        const size_t responder_index = uwb_anchor_survey_id_index(
            anchor_ids, anchor_count, responder_id);
        if (initiator_index == SIZE_MAX || responder_index == SIZE_MAX) {
            return;
        }
        difference_mm -=
            config->passive_ds_anchor_bias_mm[responder_index] -
            config->passive_ds_anchor_bias_mm[initiator_index];
    }

    uint8_t expected_initiator = 0U;
    uint8_t expected_responder = 0U;
    uint8_t responder_index = 0U;
    uint32_t solver_slot_id = slot_id;
    if (config->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS) {
        struct uwb_passive_ds_multi_plan plan = {0};
        if (!uwb_passive_ds_multi_build_plan(
                anchor_ids, anchor_count, slot_id, &plan)) {
            return;
        }
        const int index = uwb_passive_ds_multi_responder_index(
            &plan, responder_id);
        if (plan.initiator_id != initiator_id || index < 0) {
            return;
        }
        expected_initiator = plan.initiator_id;
        expected_responder = responder_id;
        responder_index = (uint8_t)index;
        solver_slot_id = slot_id * (uint32_t)(anchor_count - 1U) +
                         (uint32_t)responder_index;
    } else if (!uwb_passive_ds_slot_pair(
                   slot_id, anchor_ids, anchor_count,
                   config->passive_ds_schedule, &expected_initiator,
                   &expected_responder, &responder_index) ||
               expected_initiator != initiator_id ||
               expected_responder != responder_id) {
        return;
    }

    const int32_t anchor_distance_mm =
        uwb_passive_ds_runtime_anchor_range_mm(
            initiator_id, responder_id);
    const int32_t clock_ratio_ppb = (int32_t)lround(
        (result->listener_to_responder_clock_ratio - 1.0) *
        1000000000.0);
    const int64_t reply_delay_us_i64 =
        uwb_dw3000_dtu_to_us(result->responder_reply_dtu);
    const uint32_t reply_delay_us =
        reply_delay_us_i64 > 0 ? (uint32_t)reply_delay_us_i64 : 0U;
    const uint8_t range_source = anchor_distance_mm > 0
        ? (uint8_t)UWB_PASSIVE_DS_RANGE_PIGGYBACK_DS
        : (uint8_t)UWB_PASSIVE_DS_RANGE_NONE;

    /*
     * The N+2 runtime solves every observation locally on the receive-only
     * tag.  Forwarding the same raw observations to the dashboard would add
     * three telemetry packets per radio frame without contributing to the
     * position calculation.  Keep the legacy stream for the two reference
     * schedules, where it remains useful for offline comparison.
     */
    if (config->passive_ds_schedule !=
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS) {
        (void)wireless_telemetry_service_submit_passive_ds_observation(
            tag_id, initiator_id, responder_id, responder_index, sequence,
            solver_slot_id, difference_mm, difference_mm,
            anchor_distance_mm, 0, clock_ratio_ppb, reply_delay_us,
            range_source, 0U);
    }
    const uint16_t delay_ratio_q15 = (uint16_t)lround(
        fmin(32767.0,
             fmax(1.0, result->responder_delay_ratio * 32768.0)));
    (void)uwb_passive_ds_runtime_submit_observation(
        tag_id, initiator_id, responder_id, solver_slot_id, difference_mm,
        delay_ratio_q15);
}

static void uwb_passive_ds_tag_handle_status(
    enum uwb_passive_ds_tdoa_status status, uint8_t tag_id,
    uint8_t initiator_id, uint8_t responder_id, uint16_t sequence,
    uint32_t slot_id, const struct uwb_passive_ds_tdoa_result *result,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (status == UWB_PASSIVE_DS_TDOA_READY) {
        uwb_passive_ds_tag_submit_double_sided(
            tag_id, initiator_id, responder_id, sequence, slot_id,
            result, anchor_ids, anchor_count);
    }
}

static void uwb_passive_ds_multi_tag_record_status(
    struct uwb_passive_ds_multi_tag_stats *stats,
    enum uwb_passive_ds_tdoa_status status, uint8_t responder_id,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (stats == NULL || anchor_ids == NULL) {
        return;
    }
    const size_t index = uwb_anchor_survey_id_index(
        anchor_ids, anchor_count, responder_id);
    if (index >= UWB_PASSIVE_DS_MULTI_MAX_ANCHORS) {
        return;
    }
    if (status == UWB_PASSIVE_DS_TDOA_READY) {
        stats->ready[index]++;
    } else if (status == UWB_PASSIVE_DS_TDOA_REJECTED) {
        stats->rejected[index]++;
    }
}

static void uwb_passive_ds_tag_process_frame_double_sided(
    const struct uwb_distance_frame *frame,
    struct uwb_passive_ds_tdoa_context *context, uint8_t tag_id,
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (frame == NULL || context == NULL) {
        return;
    }

    const app_runtime_config_t *config = app_runtime_config_get();
    struct uwb_passive_ds_tdoa_result result = {0};
    uint8_t piggy_initiator = 0U;
    uint8_t piggy_responder = 0U;
    uint16_t piggy_sequence = 0U;
    uint32_t piggy_slot_id = 0U;

    if (frame->type == UWB_DISTANCE_FRAME_PASSIVE_DS_POLL) {
        if (frame->payload_len < UWB_PASSIVE_DS_POLL_LEN) {
            return;
        }
        const uint32_t slot_id = uwb_distance_get_u32(
            frame->payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET);
        uint8_t initiator_id = 0U;
        uint8_t responder_id = 0U;
        if (!uwb_passive_ds_slot_pair(
                slot_id, anchor_ids, anchor_count,
                config->passive_ds_schedule, &initiator_id,
                &responder_id, NULL) ||
            frame->source_id != initiator_id ||
            frame->destination_id != responder_id) {
            return;
        }

        const enum uwb_passive_ds_tdoa_status piggy_status =
            uwb_passive_ds_accept_piggyback(
                frame, UWB_PASSIVE_DS_POLL_PIGGYBACK_OFFSET,
                anchor_ids, anchor_count, context, &result,
                &piggy_initiator, &piggy_responder, &piggy_sequence,
                &piggy_slot_id);
        uwb_passive_ds_tag_handle_status(
            piggy_status, tag_id, piggy_initiator, piggy_responder,
            piggy_sequence, piggy_slot_id, &result, anchor_ids,
            anchor_count);

        memset(&result, 0, sizeof(result));
        const enum uwb_passive_ds_tdoa_status status =
            uwb_passive_ds_tdoa_record_poll(
                context, initiator_id, responder_id, frame->sequence,
                slot_id, frame->rx_timestamp, &result);
        uwb_passive_ds_tag_handle_status(
            status, tag_id, initiator_id, responder_id, frame->sequence,
            slot_id, &result, anchor_ids, anchor_count);
        return;
    }

    if (frame->type == UWB_DISTANCE_FRAME_PASSIVE_DS_RESP) {
        if (frame->payload_len < UWB_PASSIVE_DS_RESP_LEN) {
            return;
        }
        const uint32_t slot_id = uwb_distance_get_u32(
            frame->payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET);
        const uint32_t reply_dtu = uwb_distance_get_u32(
            frame->payload, UWB_PASSIVE_DS_REPLY_DTU_OFFSET);
        uint8_t initiator_id = 0U;
        uint8_t responder_id = 0U;
        if (reply_dtu == 0U ||
            !uwb_passive_ds_slot_pair(
                slot_id, anchor_ids, anchor_count,
                config->passive_ds_schedule, &initiator_id,
                &responder_id, NULL) ||
            frame->source_id != responder_id ||
            frame->destination_id != initiator_id) {
            return;
        }

        const enum uwb_passive_ds_tdoa_status piggy_status =
            uwb_passive_ds_accept_piggyback(
                frame, UWB_PASSIVE_DS_RESP_PIGGYBACK_OFFSET,
                anchor_ids, anchor_count, context, &result,
                &piggy_initiator, &piggy_responder, &piggy_sequence,
                &piggy_slot_id);
        uwb_passive_ds_tag_handle_status(
            piggy_status, tag_id, piggy_initiator, piggy_responder,
            piggy_sequence, piggy_slot_id, &result, anchor_ids,
            anchor_count);

        memset(&result, 0, sizeof(result));
        const enum uwb_passive_ds_tdoa_status status =
            uwb_passive_ds_tdoa_record_response(
                context, initiator_id, responder_id, frame->sequence,
                slot_id, frame->rx_timestamp, reply_dtu, &result);
        uwb_passive_ds_tag_handle_status(
            status, tag_id, initiator_id, responder_id, frame->sequence,
            slot_id, &result, anchor_ids, anchor_count);
        return;
    }

    if (frame->type != UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL ||
        frame->payload_len < UWB_PASSIVE_DS_FINAL_LEN) {
        return;
    }
    const uint32_t slot_id = uwb_distance_get_u32(
        frame->payload, UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET);
    uint8_t initiator_id = 0U;
    uint8_t responder_id = 0U;
    if (!uwb_passive_ds_slot_pair(
            slot_id, anchor_ids, anchor_count,
            config->passive_ds_schedule, &initiator_id, &responder_id,
            NULL) ||
        frame->source_id != initiator_id ||
        frame->destination_id != responder_id) {
        return;
    }
    const enum uwb_passive_ds_tdoa_status status =
        uwb_passive_ds_tdoa_record_final(
            context, initiator_id, responder_id, frame->sequence, slot_id,
            frame->rx_timestamp,
            uwb_distance_get_ts40(
                frame->payload, UWB_DISTANCE_FRAME_POLL_TX_TS_OFFSET),
            uwb_distance_get_ts40(
                frame->payload, UWB_DISTANCE_FRAME_RESP_RX_TS_OFFSET),
            uwb_distance_get_ts40(
                frame->payload, UWB_DISTANCE_FRAME_FINAL_TX_TS_OFFSET),
            &result);
    uwb_passive_ds_tag_handle_status(
        status, tag_id, initiator_id, responder_id, frame->sequence,
        slot_id, &result, anchor_ids, anchor_count);
}

static void uwb_passive_ds_tag_process_frame_multipoint(
    const struct uwb_distance_frame *frame,
    struct uwb_passive_ds_tdoa_context *context, uint8_t tag_id,
    const uint8_t *anchor_ids, size_t anchor_count,
    struct uwb_passive_ds_multi_tag_stats *stats)
{
    if (frame == NULL || context == NULL) {
        return;
    }

    struct uwb_passive_ds_tdoa_result result = {0};
    uint8_t piggy_initiator = 0U;
    uint8_t piggy_responder = 0U;
    uint16_t piggy_sequence = 0U;
    uint32_t piggy_frame_id = 0U;
    uint32_t frame_id = 0U;

    if (frame->type == UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL) {
        if (!uwb_passive_ds_multi_decode_poll(
                frame->payload, frame->payload_len,
                UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, &frame_id)) {
            return;
        }
        if (stats != NULL) {
            stats->poll_rx++;
        }
        struct uwb_passive_ds_multi_plan plan = {0};
        if (!uwb_passive_ds_multi_build_plan(
                anchor_ids, anchor_count, frame_id, &plan) ||
            frame->source_id != plan.initiator_id ||
            frame->destination_id != UWB_DISTANCE_FRAME_BROADCAST_ID) {
            return;
        }

        const enum uwb_passive_ds_tdoa_status piggy_status =
            uwb_passive_ds_accept_piggyback(
                frame, UWB_PASSIVE_DS_MULTI_POLL_PIGGYBACK_OFFSET,
                anchor_ids, anchor_count, context, &result,
                &piggy_initiator, &piggy_responder, &piggy_sequence,
                &piggy_frame_id);
        uwb_passive_ds_tag_handle_status(
            piggy_status, tag_id, piggy_initiator, piggy_responder,
            piggy_sequence, piggy_frame_id, &result, anchor_ids,
            anchor_count);
        uwb_passive_ds_multi_tag_record_status(
            stats, piggy_status, piggy_responder, anchor_ids,
            anchor_count);
        uwb_passive_ds_multi_tag_record_piggyback_lag(
            stats, frame_id, piggy_initiator, piggy_frame_id);

        for (uint8_t index = 0U; index < plan.responder_count; ++index) {
            memset(&result, 0, sizeof(result));
            const enum uwb_passive_ds_tdoa_status status =
                uwb_passive_ds_tdoa_record_poll(
                    context, plan.initiator_id,
                    plan.responder_ids[index], frame->sequence, frame_id,
                    frame->rx_timestamp, &result);
            uwb_passive_ds_tag_handle_status(
                status, tag_id, plan.initiator_id,
                plan.responder_ids[index], frame->sequence, frame_id,
                &result, anchor_ids, anchor_count);
        }
        return;
    }

    if (frame->type == UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP) {
        uint8_t responder_index = 0U;
        uint32_t reply_dtu = 0U;
        if (!uwb_passive_ds_multi_decode_response(
                frame->payload, frame->payload_len,
                UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, &frame_id,
                &responder_index, &reply_dtu)) {
            return;
        }
        struct uwb_passive_ds_multi_plan plan = {0};
        if (!uwb_passive_ds_multi_build_plan(
                anchor_ids, anchor_count, frame_id, &plan) ||
            responder_index >= plan.responder_count ||
            frame->source_id != plan.responder_ids[responder_index] ||
            frame->destination_id != plan.initiator_id) {
            return;
        }
        if (stats != NULL) {
            stats->response_rx[responder_index]++;
        }

        const enum uwb_passive_ds_tdoa_status piggy_status =
            uwb_passive_ds_accept_piggyback(
                frame, UWB_PASSIVE_DS_MULTI_RESP_PIGGYBACK_OFFSET,
                anchor_ids, anchor_count, context, &result,
                &piggy_initiator, &piggy_responder, &piggy_sequence,
                &piggy_frame_id);
        uwb_passive_ds_tag_handle_status(
            piggy_status, tag_id, piggy_initiator, piggy_responder,
            piggy_sequence, piggy_frame_id, &result, anchor_ids,
            anchor_count);
        uwb_passive_ds_multi_tag_record_status(
            stats, piggy_status, piggy_responder, anchor_ids,
            anchor_count);
        uwb_passive_ds_multi_tag_record_piggyback_lag(
            stats, frame_id, piggy_initiator, piggy_frame_id);

        memset(&result, 0, sizeof(result));
        const enum uwb_passive_ds_tdoa_status status =
            uwb_passive_ds_tdoa_record_response(
                context, plan.initiator_id, frame->source_id,
                frame->sequence, frame_id, frame->rx_timestamp, reply_dtu,
                &result);
        uwb_passive_ds_tag_handle_status(
            status, tag_id, plan.initiator_id, frame->source_id,
            frame->sequence, frame_id, &result, anchor_ids, anchor_count);
        return;
    }

    if (frame->type != UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL) {
        return;
    }
    struct uwb_passive_ds_multi_final final = {0};
    if (!uwb_passive_ds_multi_decode_final(
            frame->payload, frame->payload_len,
            UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, &final)) {
        return;
    }
    struct uwb_passive_ds_multi_plan plan = {0};
    if (!uwb_passive_ds_multi_build_plan(
            anchor_ids, anchor_count, final.frame_id, &plan) ||
        frame->source_id != plan.initiator_id ||
        frame->destination_id != UWB_DISTANCE_FRAME_BROADCAST_ID) {
        return;
    }
    if (stats != NULL) {
        stats->final_rx++;
        stats->final_entries += final.responder_count;
    }
    for (uint8_t index = 0U; index < final.responder_count; ++index) {
        const uint8_t responder_id =
            final.responders[index].responder_id;
        if (uwb_passive_ds_multi_responder_index(
                &plan, responder_id) < 0) {
            continue;
        }
        memset(&result, 0, sizeof(result));
        const enum uwb_passive_ds_tdoa_status status =
            uwb_passive_ds_tdoa_record_final(
                context, plan.initiator_id, responder_id,
                frame->sequence, final.frame_id, frame->rx_timestamp,
                final.initiator_poll_tx,
                final.responders[index].initiator_response_rx,
                final.initiator_final_tx, &result);
        uwb_passive_ds_tag_handle_status(
            status, tag_id, plan.initiator_id, responder_id,
            frame->sequence, final.frame_id, &result, anchor_ids,
            anchor_count);
        uwb_passive_ds_multi_tag_record_status(
            stats, status, responder_id, anchor_ids, anchor_count);
    }
}

static void uwb_passive_ds_tag_loop(const uint8_t *anchor_ids,
                                    size_t anchor_count)
{
    struct uwb_passive_ds_tdoa_context context;
    uwb_passive_ds_tdoa_init(&context);
    struct uwb_passive_ds_multi_tag_stats multi_stats = {
        .summary_started_us = esp_timer_get_time(),
    };
    const esp_err_t solver_err = uwb_passive_ds_runtime_start_solver();
    if (solver_err != ESP_OK) {
        ESP_LOGW(TAG, "PASSIVE_DS local solver unavailable: %s",
                 esp_err_to_name(solver_err));
    }
    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "PASSIVE_DS receive-only tag active: tag_id=%u "
             "estimator=three-packet-ds-tdoa geometry=live-anchor-ranges "
             "cfo=unused",
             (unsigned)s_source_id);
    while (!uwb_dw3000_runtime_switch_pending()) {
        const app_runtime_config_t *config = app_runtime_config_get();
        struct uwb_distance_frame frame = {0};
        const esp_err_t err = uwb_distance_receive_next(
            &frame, config->passive_ds_rx_slice_ms);
        if (err == ESP_OK) {
            if (config->passive_ds_schedule ==
                APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS) {
                uwb_passive_ds_tag_process_frame_multipoint(
                    &frame, &context, s_source_id, anchor_ids,
                    anchor_count, &multi_stats);
            } else {
                uwb_passive_ds_tag_process_frame_double_sided(
                    &frame, &context, s_source_id, anchor_ids,
                    anchor_count);
            }
        } else if (err != ESP_ERR_TIMEOUT &&
                   !uwb_dw3000_runtime_switch_pending()) {
            ESP_LOGW(TAG, "PASSIVE_DS tag RX failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(2);
        }
        if (config->passive_ds_schedule ==
                APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS &&
            esp_timer_get_time() - multi_stats.summary_started_us >=
                1000000LL) {
            ESP_LOGI(
                TAG,
                "PASSIVE_DS tag N+2 rx poll=%lu resp=%lu/%lu/%lu final=%lu entries=%lu ready=%lu/%lu/%lu/%lu reject=%lu/%lu/%lu/%lu",
                (unsigned long)multi_stats.poll_rx,
                (unsigned long)multi_stats.response_rx[0],
                (unsigned long)multi_stats.response_rx[1],
                (unsigned long)multi_stats.response_rx[2],
                (unsigned long)multi_stats.final_rx,
                (unsigned long)multi_stats.final_entries,
                (unsigned long)multi_stats.ready[0],
                (unsigned long)multi_stats.ready[1],
                (unsigned long)multi_stats.ready[2],
                (unsigned long)multi_stats.ready[3],
                (unsigned long)multi_stats.rejected[0],
                (unsigned long)multi_stats.rejected[1],
                (unsigned long)multi_stats.rejected[2],
                (unsigned long)multi_stats.rejected[3]);
            ESP_LOGI(
                TAG,
                "PASSIVE_DS tag N+2 assembly ready=%lu calc_reject=%lu missing_final=%lu missing_exchange=%lu evicted=%lu",
                (unsigned long)(context.ready_count -
                    multi_stats.previous_ready_count),
                (unsigned long)(context.calculation_rejected_count -
                    multi_stats.previous_calculation_rejected_count),
                (unsigned long)(context.missing_final_context_count -
                    multi_stats.previous_missing_final_context_count),
                (unsigned long)(context.missing_exchange_context_count -
                    multi_stats.previous_missing_exchange_context_count),
                (unsigned long)(context.pending_replacement_count -
                    multi_stats.previous_pending_replacement_count));
            ESP_LOGI(
                TAG,
                "PASSIVE_DS tag N+2 piggyback lag frames 0/1/2/3/4/5/6/7+=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu",
                (unsigned long)multi_stats.piggyback_lag[0],
                (unsigned long)multi_stats.piggyback_lag[1],
                (unsigned long)multi_stats.piggyback_lag[2],
                (unsigned long)multi_stats.piggyback_lag[3],
                (unsigned long)multi_stats.piggyback_lag[4],
                (unsigned long)multi_stats.piggyback_lag[5],
                (unsigned long)multi_stats.piggyback_lag[6],
                (unsigned long)multi_stats.piggyback_lag[7]);
            const uint32_t ready_count = context.ready_count;
            const uint32_t calculation_rejected_count =
                context.calculation_rejected_count;
            const uint32_t missing_final_context_count =
                context.missing_final_context_count;
            const uint32_t missing_exchange_context_count =
                context.missing_exchange_context_count;
            const uint32_t pending_replacement_count =
                context.pending_replacement_count;
            memset(&multi_stats, 0, sizeof(multi_stats));
            multi_stats.previous_ready_count = ready_count;
            multi_stats.previous_calculation_rejected_count =
                calculation_rejected_count;
            multi_stats.previous_missing_final_context_count =
                missing_final_context_count;
            multi_stats.previous_missing_exchange_context_count =
                missing_exchange_context_count;
            multi_stats.previous_pending_replacement_count =
                pending_replacement_count;
            multi_stats.summary_started_us = esp_timer_get_time();
        }
    }
}

static esp_err_t uwb_passive_ds_schedule_timer_init(void)
{
    if (s_passive_ds_schedule_timer != NULL) {
        return ESP_OK;
    }
    const esp_timer_create_args_t args = {
        .callback = uwb_passive_ds_schedule_alarm_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pds_slot",
        .skip_unhandled_events = true,
    };
    return esp_timer_create(&args, &s_passive_ds_schedule_timer);
}

static void uwb_passive_ds_arm_schedule_alarm(int64_t poll_due_host_us,
                                               int64_t lead_us)
{
    (void)esp_timer_stop(s_passive_ds_schedule_timer);
    s_passive_ds_schedule_alarm_fired = false;
    const int64_t alarm_us = poll_due_host_us - lead_us -
                             esp_timer_get_time();
    if (alarm_us > 0) {
        const esp_err_t err = esp_timer_start_once(
            s_passive_ds_schedule_timer, (uint64_t)alarm_us);
        if (err != ESP_OK) {
            s_passive_ds_schedule_alarm_fired = true;
        }
    } else {
        s_passive_ds_schedule_alarm_fired = true;
    }
}

static void uwb_passive_ds_anchor_loop_legacy(
    const uint8_t *anchor_ids, size_t anchor_count)
{
    struct uwb_passive_ds_schedule schedule = {
        .last_poll_host_us =
            esp_timer_get_time() -
            (s_runtime_hot_entry
                 ? UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US -
                       UWB_HOT_SWITCH_BOOTSTRAP_GUARD_US
                 : 0),
    };
    uint32_t completed = 0;
    uint32_t failed = 0;
    int64_t summary_started_us = esp_timer_get_time();
    s_status = UWB_DW3000_STATUS_READY;

    while (!uwb_dw3000_runtime_switch_pending()) {
        const app_runtime_config_t *config = app_runtime_config_get();
        int64_t now_us = esp_timer_get_time();
        bool initiate = schedule.synced &&
                        now_us >= schedule.next_owned_poll_host_us;
        uint32_t slot_id = schedule.next_owned_slot_id;
        if (!schedule.synced && s_source_id == anchor_ids[0] &&
            now_us - schedule.last_poll_host_us >=
                UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US) {
            initiate = true;
            slot_id = 0U;
            schedule.bootstrap_count++;
        }

        if (initiate) {
            uint8_t initiator_id = 0;
            uint8_t responder_id = 0;
            if (!uwb_passive_ds_slot_pair(
                    slot_id, anchor_ids, anchor_count,
                    config->passive_ds_schedule, &initiator_id,
                    &responder_id, NULL) ||
                initiator_id != s_source_id) {
                schedule.synced = false;
                schedule.last_poll_host_us = now_us;
                continue;
            }
            if (schedule.synced &&
                now_us - schedule.next_owned_poll_host_us >
                    (int64_t)config->passive_ds_slot_ms * 1000LL) {
                schedule.late_count++;
            }
            int64_t poll_host_us = now_us;
            const esp_err_t err = uwb_passive_ds_initiate_once(
                responder_id, (uint16_t)(slot_id & 0xFFFFU), slot_id,
                &poll_host_us);
            if (err == ESP_OK) {
                completed++;
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE);
            } else {
                failed++;
            }
            uwb_passive_ds_schedule_from_poll(
                &schedule, slot_id, 0, poll_host_us, anchor_ids,
                anchor_count, config);
            continue;
        }

        uint32_t rx_timeout_ms = config->passive_ds_rx_slice_ms;
        if (schedule.synced) {
            const int64_t until_due_us =
                schedule.next_owned_poll_host_us - now_us;
            if (until_due_us <= 1000LL) {
                rx_timeout_ms = 1U;
            } else {
                const uint32_t until_due_ms =
                    (uint32_t)(until_due_us / 1000LL);
                if (until_due_ms < rx_timeout_ms) {
                    rx_timeout_ms = until_due_ms;
                }
            }
        }
        struct uwb_distance_frame frame = {0};
        const esp_err_t rx_err =
            uwb_distance_receive_next(&frame, rx_timeout_ms);
        if (rx_err == ESP_OK &&
            frame.type == UWB_DISTANCE_FRAME_PASSIVE_DS_POLL &&
            frame.payload_len >= UWB_PASSIVE_DS_POLL_LEN) {
            const uint32_t received_slot_id = uwb_distance_get_u32(
                frame.payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET);
            uint8_t initiator_id = 0;
            uint8_t responder_id = 0;
            if (!uwb_passive_ds_slot_pair(
                    received_slot_id, anchor_ids, anchor_count,
                    config->passive_ds_schedule, &initiator_id,
                    &responder_id, NULL) ||
                frame.source_id != initiator_id ||
                frame.destination_id != responder_id) {
                continue;
            }
            uwb_passive_ds_schedule_from_poll(
                &schedule, received_slot_id, frame.rx_timestamp,
                frame.rx_host_time_us, anchor_ids, anchor_count, config);
            if (responder_id == s_source_id) {
                struct uwb_distance_measurement measurement = {0};
                const esp_err_t err = uwb_passive_ds_respond_to_poll(
                    &frame, received_slot_id, &measurement);
                if (err == ESP_OK) {
                    completed++;
                    uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE);
                    (void)wireless_telemetry_service_submit_passive_ds_anchor_range(
                        measurement.initiator_id,
                        measurement.responder_id, measurement.sequence,
                        received_slot_id,
                        uwb_distance_meters_to_mm(measurement.distance_m),
                        uwb_distance_meters_to_mm(
                            measurement.raw_distance_m));
                } else {
                    failed++;
                }
            }
        } else if (rx_err != ESP_OK && rx_err != ESP_ERR_TIMEOUT &&
                   !uwb_dw3000_runtime_switch_pending()) {
            ESP_LOGD(TAG, "PASSIVE_DS anchor RX: %s",
                     esp_err_to_name(rx_err));
        }

        now_us = esp_timer_get_time();
        if (now_us - summary_started_us >= 1000000LL) {
            ESP_LOGI(TAG,
                     "PASSIVE_DS anchor schedule=%s ok=%lu fail=%lu "
                     "bootstrap=%lu late=%lu next_slot=%lu",
                     config->passive_ds_schedule ==
                             APP_RUNTIME_PASSIVE_DS_ROBUST_ROTATING
                         ? "robust_rotating"
                         : "fast_star",
                     (unsigned long)completed, (unsigned long)failed,
                     (unsigned long)schedule.bootstrap_count,
                     (unsigned long)schedule.late_count,
                     (unsigned long)schedule.next_owned_slot_id);
            completed = 0;
            failed = 0;
            summary_started_us = now_us;
        }
    }
}

static void uwb_passive_ds_anchor_loop_deadline(
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (uwb_passive_ds_schedule_timer_init() != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "PASSIVE_DS schedule timer init failed");
        return;
    }

    struct uwb_passive_ds_schedule schedule = {
        .last_poll_host_us =
            esp_timer_get_time() -
            (s_runtime_hot_entry
                 ? UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US -
                       UWB_HOT_SWITCH_BOOTSTRAP_GUARD_US
                 : 0),
    };
    struct uwb_passive_ds_exchange exchange = {0};
    uint32_t completed_since_summary = 0;
    uint32_t failed_since_summary = 0;
    int64_t summary_started_us = esp_timer_get_time();
    int64_t armed_alarm_due_host_us = 0;
    s_status = UWB_DW3000_STATUS_READY;

    while (!uwb_dw3000_runtime_switch_pending()) {
        const app_runtime_config_t *config = app_runtime_config_get();
        int64_t now_us = esp_timer_get_time();

        if (exchange.state != UWB_PASSIVE_DS_EXCHANGE_IDLE &&
            now_us >= exchange.deadline_host_us) {
            if (exchange.state ==
                UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE) {
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT);
            } else {
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT);
                uwb_passive_ds_record_stage(
                    UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
                    exchange.started_host_us, false);
            }
            exchange.state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
            failed_since_summary++;
        }

        bool bootstrap = false;
        bool initiate =
            schedule.synced &&
            now_us >= schedule.next_owned_poll_host_us -
                          UWB_PASSIVE_DS_POLL_TX_LEAD_US;
        uint32_t slot_id = schedule.next_owned_slot_id;
        if (!schedule.synced && s_source_id == anchor_ids[0] &&
            now_us - schedule.last_poll_host_us >=
                UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US) {
            initiate = true;
            bootstrap = true;
            slot_id = 0U;
            schedule.bootstrap_count++;
        }

        if (initiate) {
            if (armed_alarm_due_host_us != 0) {
                (void)esp_timer_stop(s_passive_ds_schedule_timer);
                armed_alarm_due_host_us = 0;
                s_passive_ds_schedule_alarm_fired = false;
            }
            uint8_t initiator_id = 0;
            uint8_t responder_id = 0;
            if (!uwb_passive_ds_slot_pair(
                    slot_id, anchor_ids, anchor_count,
                    config->passive_ds_schedule, &initiator_id,
                    &responder_id, NULL) ||
                initiator_id != s_source_id) {
                schedule.synced = false;
                schedule.last_poll_host_us = now_us;
                continue;
            }

            const int64_t lateness_us =
                bootstrap ? 0 : now_us - schedule.next_owned_poll_host_us;
            if (!bootstrap && lateness_us > UWB_PASSIVE_DS_POLL_LATE_US) {
                schedule.synced = false;
                schedule.late_count++;
                schedule.last_poll_host_us = now_us;
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_OVERRUN);
                continue;
            }
            if (exchange.state != UWB_PASSIVE_DS_EXCHANGE_IDLE) {
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_STATE_COLLISION);
                exchange.state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
                (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
                (void)uwb_dw3000_clear_status();
                s_rx_armed = false;
            }

            const int64_t scheduled_poll_host_us =
                bootstrap ? now_us : schedule.next_owned_poll_host_us;
            uint64_t actual_poll_tx_ts = 0;
            int64_t actual_poll_host_us = now_us;
            const esp_err_t err = uwb_passive_ds_start_initiator(
                responder_id, (uint16_t)(slot_id & 0xFFFFU), slot_id,
                bootstrap ? 0 : schedule.next_owned_poll_radio_ts,
                &exchange, &actual_poll_tx_ts, &actual_poll_host_us);
            if (err == ESP_OK) {
                uwb_passive_ds_schedule_from_poll(
                    &schedule, slot_id, actual_poll_tx_ts,
                    scheduled_poll_host_us, anchor_ids, anchor_count,
                    config);
            } else {
                failed_since_summary++;
                schedule.synced = false;
                schedule.last_poll_host_us = actual_poll_host_us;
            }
            continue;
        }

        if (schedule.synced &&
            armed_alarm_due_host_us !=
                schedule.next_owned_poll_host_us) {
            uwb_passive_ds_arm_schedule_alarm(
                schedule.next_owned_poll_host_us,
                UWB_PASSIVE_DS_POLL_TX_LEAD_US);
            armed_alarm_due_host_us =
                schedule.next_owned_poll_host_us;
        } else if (!schedule.synced &&
                   armed_alarm_due_host_us != 0) {
            (void)esp_timer_stop(s_passive_ds_schedule_timer);
            s_passive_ds_schedule_alarm_fired = false;
            armed_alarm_due_host_us = 0;
        }

        struct uwb_distance_frame frame = {0};
        const esp_err_t rx_err = uwb_distance_receive_next(
            &frame, config->passive_ds_rx_slice_ms);
        const bool schedule_alarm =
            s_passive_ds_schedule_alarm_fired;
        s_passive_ds_schedule_alarm_fired = false;
        if (schedule_alarm) {
            armed_alarm_due_host_us = 0;
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_SCHEDULE_ALARM);
        }

        if (rx_err == ESP_ERR_TIMEOUT &&
            exchange.state != UWB_PASSIVE_DS_EXCHANGE_IDLE) {
            if (exchange.state ==
                UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE) {
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT);
            } else {
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT);
                uwb_passive_ds_record_stage(
                    UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_RX,
                    exchange.started_host_us, false);
            }
            exchange.state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
            failed_since_summary++;
            continue;
        }
        if (rx_err == ESP_ERR_NOT_FINISHED || rx_err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (rx_err != ESP_OK) {
            if (!uwb_dw3000_runtime_switch_pending()) {
                failed_since_summary++;
            }
            continue;
        }

        if (exchange.state ==
                UWB_PASSIVE_DS_EXCHANGE_WAIT_RESPONSE &&
            frame.type == UWB_DISTANCE_FRAME_PASSIVE_DS_RESP &&
            frame.source_id == exchange.peer_id &&
            frame.destination_id == s_source_id &&
            frame.sequence == exchange.sequence) {
            const esp_err_t err =
                uwb_passive_ds_complete_initiator_response(
                    &exchange, &frame);
            if (err == ESP_OK) {
                completed_since_summary++;
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE);
            } else {
                failed_since_summary++;
            }
            continue;
        }

        if (exchange.state == UWB_PASSIVE_DS_EXCHANGE_WAIT_FINAL &&
            frame.type == UWB_DISTANCE_FRAME_PASSIVE_DS_FINAL &&
            frame.source_id == exchange.peer_id &&
            frame.destination_id == s_source_id &&
            frame.sequence == exchange.sequence) {
            struct uwb_distance_measurement measurement = {0};
            const esp_err_t err =
                uwb_passive_ds_complete_responder_final(
                    &exchange, &frame, &measurement);
            if (err == ESP_OK) {
                completed_since_summary++;
                uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_COMPLETED_EXCHANGE);
                (void)wireless_telemetry_service_submit_passive_ds_anchor_range(
                    measurement.initiator_id,
                    measurement.responder_id, measurement.sequence,
                    uwb_distance_get_u32(
                        frame.payload,
                        UWB_PASSIVE_DS_FINAL_SLOT_ID_OFFSET),
                    uwb_distance_meters_to_mm(measurement.distance_m),
                    uwb_distance_meters_to_mm(
                        measurement.raw_distance_m));
            } else {
                failed_since_summary++;
            }
            continue;
        }

        if (frame.type != UWB_DISTANCE_FRAME_PASSIVE_DS_POLL) {
            // RESP and FINAL for other anchors are normal shared-medium
            // traffic, not pipeline failures on this module.
            continue;
        }
        if (frame.payload_len < UWB_PASSIVE_DS_POLL_LEN) {
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
            continue;
        }

        const uint32_t received_slot_id = uwb_distance_get_u32(
            frame.payload, UWB_PASSIVE_DS_SLOT_ID_OFFSET);
        uint8_t initiator_id = 0;
        uint8_t responder_id = 0;
        if (!uwb_passive_ds_slot_pair(
                received_slot_id, anchor_ids, anchor_count,
                config->passive_ds_schedule, &initiator_id,
                &responder_id, NULL) ||
            frame.source_id != initiator_id ||
            frame.destination_id != responder_id) {
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
            continue;
        }
        uwb_passive_ds_schedule_from_poll(
            &schedule, received_slot_id, frame.rx_timestamp,
            frame.rx_host_time_us, anchor_ids, anchor_count, config);
        if (responder_id != s_source_id) {
            continue;
        }

        if (exchange.state != UWB_PASSIVE_DS_EXCHANGE_IDLE) {
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_STATE_COLLISION);
            exchange.state = UWB_PASSIVE_DS_EXCHANGE_IDLE;
        }
        const esp_err_t response_err = uwb_passive_ds_start_responder(
            &frame, received_slot_id, &exchange);
        if (response_err != ESP_OK) {
            failed_since_summary++;
        }

        now_us = esp_timer_get_time();
        if (now_us - summary_started_us >= 1000000LL) {
            struct uwb_passive_ds_pipeline_stats stats = {0};
            uwb_passive_ds_runtime_get_stats(&stats);
            const struct uwb_passive_ds_stage_stats *poll_stats =
                &stats.poll_tx;
            const struct uwb_passive_ds_stage_stats *resp_stats =
                &stats.response_tx;
            const struct uwb_passive_ds_stage_stats *final_stats =
                &stats.final_tx;
            ESP_LOGI(
                TAG,
                "PASSIVE_DS pipeline=deadline ok=%lu fail=%lu "
                "poll_us=%llu/%lu resp_us=%llu/%lu final_us=%llu/%lu "
                "resp_to=%lu final_to=%lu invalid=%lu collision=%lu "
                "alarm=%lu overrun=%lu",
                (unsigned long)completed_since_summary,
                (unsigned long)failed_since_summary,
                (unsigned long long)(
                    poll_stats->count
                        ? poll_stats->total_duration_us /
                              poll_stats->count
                        : 0U),
                (unsigned long)poll_stats->max_duration_us,
                (unsigned long long)(
                    resp_stats->count
                        ? resp_stats->total_duration_us /
                              resp_stats->count
                        : 0U),
                (unsigned long)resp_stats->max_duration_us,
                (unsigned long long)(
                    final_stats->count
                        ? final_stats->total_duration_us /
                              final_stats->count
                        : 0U),
                (unsigned long)final_stats->max_duration_us,
                (unsigned long)stats.response_timeout_count,
                (unsigned long)stats.final_timeout_count,
                (unsigned long)stats.invalid_frame_count,
                (unsigned long)stats.state_collision_count,
                (unsigned long)stats.schedule_alarm_count,
                (unsigned long)stats.schedule_overrun_count);
            completed_since_summary = 0;
            failed_since_summary = 0;
            summary_started_us = now_us;
        }
    }
    (void)esp_timer_stop(s_passive_ds_schedule_timer);
    s_passive_ds_schedule_alarm_fired = false;
}

static void uwb_passive_ds_multi_schedule_from_poll(
    struct uwb_passive_ds_multi_schedule *schedule,
    uint32_t frame_id, uint64_t poll_radio_ts, int64_t poll_host_us,
    const uint8_t *anchor_ids, size_t anchor_count,
    const app_runtime_config_t *config)
{
    if (schedule == NULL || config == NULL) {
        return;
    }
    const uint32_t next_frame_id = frame_id + 1U;
    struct uwb_passive_ds_multi_plan next = {0};
    const uint32_t period_us =
        config->passive_ds_slot_ms * 1000U +
        (config->passive_ds_solve_mode ==
                 APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
             ? APP_UWB_PASSIVE_DS_DYNAMIC_GUARD_US
             : config->passive_ds_round_gap_ms * 1000U);
    schedule->last_poll_host_us = poll_host_us;
    schedule->synced =
        uwb_passive_ds_multi_build_plan(
            anchor_ids, anchor_count, next_frame_id, &next) &&
        next.initiator_id == s_source_id;
    if (!schedule->synced) {
        return;
    }
    schedule->next_frame_id = next_frame_id;
    schedule->next_poll_radio_ts = uwb_dw3000_add_timestamp_delta(
        poll_radio_ts, uwb_dw3000_us_to_dtu(period_us));
    schedule->next_poll_host_us = poll_host_us + (int64_t)period_us;
}

static esp_err_t uwb_passive_ds_multi_initiate_frame(
    const uint8_t *anchor_ids, size_t anchor_count, uint32_t frame_id,
    uint64_t scheduled_poll_ts, uint64_t *poll_tx_out,
    int64_t *poll_host_out, uint8_t *responses_out)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    struct uwb_passive_ds_multi_plan plan = {0};
    if (!uwb_passive_ds_multi_build_plan(
            anchor_ids, anchor_count, frame_id, &plan) ||
        plan.initiator_id != s_source_id) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t sequence = (uint16_t)(frame_id & 0xffffU);
    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL,
                             UWB_DISTANCE_FRAME_BROADCAST_ID, sequence,
                             payload);
    if (uwb_passive_ds_multi_encode_poll(
            payload, sizeof(payload),
            UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, frame_id) == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_MULTI_POLL_PIGGYBACK_OFFSET);

    uint64_t poll_tx_ts = 0U;
    const int64_t poll_stage_started_us = esp_timer_get_time();
    esp_err_t err = ESP_OK;
    if (scheduled_poll_ts != 0U) {
        uint64_t programmed_poll_ts = 0U;
        err = uwb_dw3000_send_payload_delayed_expect_rx(
            payload, UWB_PASSIVE_DS_MULTI_POLL_LEN,
            scheduled_poll_ts, config->passive_ds_auto_rx_delay_uus,
            config->passive_ds_rx_timeout_ms, &programmed_poll_ts,
            &poll_tx_ts);
        const uint64_t expected_poll_ts =
            uwb_dw3000_programmed_tx_timestamp(
                uwb_dw3000_delayed_time_word(scheduled_poll_ts));
        if (err == ESP_OK && programmed_poll_ts != expected_poll_ts) {
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        err = uwb_dw3000_send_payload_expect_rx(
            payload, UWB_PASSIVE_DS_MULTI_POLL_LEN,
            config->passive_ds_auto_rx_delay_uus,
            config->passive_ds_rx_timeout_ms, &poll_tx_ts);
    }
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_POLL_TX, poll_stage_started_us,
        err == ESP_OK);
    const int64_t poll_host_us = esp_timer_get_time();
    if (poll_tx_out != NULL) {
        *poll_tx_out = poll_tx_ts;
    }
    if (poll_host_out != NULL) {
        *poll_host_out = poll_host_us;
    }
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t final_from_poll_us =
        uwb_passive_ds_multi_final_delay_from_poll_us(
            config->passive_ds_resp_delay_us, plan.responder_count,
            config->passive_ds_final_delay_us);
    const uint64_t final_due = uwb_dw3000_add_timestamp_delta(
        poll_tx_ts, uwb_dw3000_us_to_dtu(final_from_poll_us));
    const uint64_t expected_final_tx =
        uwb_dw3000_programmed_tx_timestamp(
            uwb_dw3000_delayed_time_word(final_due));
    const uint32_t final_program_guard_us =
        config->passive_ds_final_delay_us / 2U > 500U
            ? config->passive_ds_final_delay_us / 2U
            : 500U;
    const bool single_star_dynamic =
        config->passive_ds_solve_mode ==
        APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR;
    const uint32_t last_response_from_poll_us =
        uwb_passive_ds_multi_response_delay_us(
            config->passive_ds_resp_delay_us,
            (uint8_t)(plan.responder_count - 1U));
    const int64_t collect_deadline_us = single_star_dynamic
        ? poll_host_us + (int64_t)last_response_from_poll_us +
              UWB_PASSIVE_DS_MULTI_RESPONSE_COLLECTION_SLACK_US
        : poll_host_us + (int64_t)final_from_poll_us -
              (int64_t)final_program_guard_us;
    uint64_t response_rx[UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS] = {0};
    bool received[UWB_PASSIVE_DS_MULTI_MAX_RESPONDERS] = {false};
    uint8_t response_count = 0U;

    /* A millisecond receive timeout can cross FINAL's delayed-TX deadline
     * when one response is absent.  The existing ESP timer wakes the same
     * radio task at a microsecond deadline; it neither creates another task
     * nor changes any UWB timestamp used by DS-TWR. */
    if (single_star_dynamic) {
        uwb_passive_ds_arm_schedule_alarm(collect_deadline_us, 0);
    }

    while (response_count < plan.responder_count &&
           esp_timer_get_time() < collect_deadline_us) {
        const int64_t remaining_us =
            collect_deadline_us - esp_timer_get_time();
        const uint32_t timeout_ms = remaining_us > 0
            ? (uint32_t)((remaining_us + 999LL) / 1000LL)
            : 1U;
        struct uwb_distance_frame response = {0};
        const esp_err_t rx_err = uwb_distance_receive_next(
            &response, timeout_ms > config->passive_ds_rx_timeout_ms
                           ? config->passive_ds_rx_timeout_ms
                           : timeout_ms);
        if (rx_err != ESP_OK) {
            if (rx_err == ESP_ERR_TIMEOUT ||
                rx_err == ESP_ERR_NOT_FINISHED) {
                continue;
            }
            break;
        }
        if (response.type !=
                UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP ||
            response.sequence != sequence ||
            response.destination_id != s_source_id) {
            continue;
        }
        uint32_t response_frame_id = 0U;
        uint8_t response_index = 0U;
        uint32_t reply_dtu = 0U;
        if (!uwb_passive_ds_multi_decode_response(
                response.payload, response.payload_len,
                UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET,
                &response_frame_id, &response_index, &reply_dtu) ||
            response_frame_id != frame_id ||
            response_index >= plan.responder_count ||
            response.source_id != plan.responder_ids[response_index] ||
            received[response_index]) {
            continue;
        }
        received[response_index] = true;
        response_rx[response_index] = response.rx_timestamp;
        response_count++;
    }
    if (single_star_dynamic) {
        (void)esp_timer_stop(s_passive_ds_schedule_timer);
        s_passive_ds_schedule_alarm_fired = false;
    }

    struct uwb_passive_ds_multi_final final = {
        .frame_id = frame_id,
        .initiator_poll_tx = poll_tx_ts,
        .initiator_final_tx = expected_final_tx,
    };
    for (uint8_t index = 0U; index < plan.responder_count; ++index) {
        if (!received[index]) {
            continue;
        }
        const uint8_t output = final.responder_count++;
        final.responders[output].responder_id = plan.responder_ids[index];
        final.responders[output].initiator_response_rx =
            response_rx[index];
    }
    if (responses_out != NULL) {
        *responses_out = response_count;
    }
    if (final.responder_count == 0U) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_RESPONSE_TIMEOUT);
    }

    memset(payload, 0, sizeof(payload));
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL,
                             UWB_DISTANCE_FRAME_BROADCAST_ID, sequence,
                             payload);
    const size_t final_length = uwb_passive_ds_multi_encode_final(
        payload, sizeof(payload), UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET,
        &final);
    if (final_length == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint64_t programmed_final_ts = 0U;
    uint64_t actual_final_ts = 0U;
    const int64_t final_started_us = esp_timer_get_time();
    err = uwb_dw3000_send_payload_delayed(
        payload, final_length, final_due, &programmed_final_ts,
        &actual_final_ts);
    const bool final_ok =
        err == ESP_OK && programmed_final_ts == expected_final_tx;
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_FINAL_TX, final_started_us, final_ok);
    (void)actual_final_ts;
    return final_ok ? ESP_OK
                    : (err == ESP_OK ? ESP_ERR_INVALID_STATE : err);
}

static esp_err_t uwb_passive_ds_multi_respond_to_poll(
    const struct uwb_distance_frame *poll,
    const struct uwb_passive_ds_multi_plan *plan,
    uint8_t responder_index)
{
    if (poll == NULL || plan == NULL ||
        responder_index >= plan->responder_count ||
        plan->responder_ids[responder_index] != s_source_id) {
        return ESP_ERR_INVALID_ARG;
    }
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t response_delay_us =
        uwb_passive_ds_multi_response_delay_us(
            config->passive_ds_resp_delay_us, responder_index);
    const uint32_t final_from_poll_us =
        uwb_passive_ds_multi_final_delay_from_poll_us(
            config->passive_ds_resp_delay_us, plan->responder_count,
            config->passive_ds_final_delay_us);
    const uint64_t response_due = uwb_dw3000_add_timestamp_delta(
        poll->rx_timestamp, uwb_dw3000_us_to_dtu(response_delay_us));
    const uint64_t expected_response_tx =
        uwb_dw3000_programmed_tx_timestamp(
            uwb_dw3000_delayed_time_word(response_due));
    const uint64_t reply_dtu = uwb_distance_delta_ts(
        expected_response_tx, poll->rx_timestamp);
    if (reply_dtu == 0U || reply_dtu > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_RESP,
                             plan->initiator_id, poll->sequence, payload);
    if (uwb_passive_ds_multi_encode_response(
            payload, sizeof(payload),
            UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, plan->frame_id,
            responder_index, (uint32_t)reply_dtu) == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }
    uwb_passive_ds_write_piggyback(
        payload, UWB_PASSIVE_DS_MULTI_RESP_PIGGYBACK_OFFSET);

    const uint32_t final_wait_us =
        final_from_poll_us - response_delay_us;
    const uint32_t final_wait_ms =
        (final_wait_us + 1999U) / 1000U;
    uint64_t programmed_response_tx = 0U;
    uint64_t actual_response_tx = 0U;
    const int64_t response_started_us = esp_timer_get_time();
    esp_err_t err = uwb_dw3000_send_payload_delayed_expect_rx(
        payload, UWB_PASSIVE_DS_MULTI_RESP_LEN, response_due,
        config->passive_ds_auto_rx_delay_uus, final_wait_ms,
        &programmed_response_tx, &actual_response_tx);
    const bool response_ok =
        err == ESP_OK && programmed_response_tx == expected_response_tx;
    uwb_passive_ds_record_stage(
        UWB_PASSIVE_DS_RUNTIME_STAGE_RESPONSE_TX, response_started_us,
        response_ok);
    if (!response_ok) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    struct uwb_distance_frame final_frame = {0};
    err = uwb_distance_receive_matching(
        UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_FINAL,
        plan->initiator_id, true, poll->sequence, &final_frame,
        final_wait_ms);
    if (err != ESP_OK) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_FINAL_TIMEOUT);
        return err;
    }
    struct uwb_passive_ds_multi_final final = {0};
    if (!uwb_passive_ds_multi_decode_final(
            final_frame.payload, final_frame.payload_len,
            UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET, &final) ||
        final.frame_id != plan->frame_id) {
        uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint64_t initiator_response_rx = 0U;
    for (uint8_t index = 0U; index < final.responder_count; ++index) {
        if (final.responders[index].responder_id == s_source_id) {
            initiator_response_rx =
                final.responders[index].initiator_response_rx;
            break;
        }
    }
    if (initiator_response_rx == 0U) {
        return ESP_ERR_NOT_FOUND;
    }

    struct uwb_distance_measurement measurement = {0};
    uwb_distance_fill_measurement_from_timestamps(
        plan->initiator_id, s_source_id, poll->sequence,
        final.initiator_poll_tx, poll->rx_timestamp,
        programmed_response_tx, initiator_response_rx,
        final.initiator_final_tx, final_frame.rx_timestamp,
        false, 0, &measurement);
    measurement.poll_rx_diagnostics = poll->diagnostics;
    measurement.final_rx_diagnostics = final_frame.diagnostics;
    uwb_passive_ds_remember_responder_exchange(
        plan->initiator_id, poll->sequence, plan->frame_id,
        poll->rx_timestamp, final_frame.rx_timestamp, &measurement);
    (void)uwb_passive_ds_store_anchor_distance(
        s_source_id, plan->initiator_id, poll->sequence, plan->frame_id,
        uwb_distance_meters_to_mm(measurement.distance_m),
        uwb_distance_meters_to_mm(measurement.raw_distance_m));
    (void)wireless_telemetry_service_submit_passive_ds_anchor_range(
        measurement.initiator_id, measurement.responder_id,
        measurement.sequence, plan->frame_id,
        uwb_distance_meters_to_mm(measurement.distance_m),
        uwb_distance_meters_to_mm(measurement.raw_distance_m));
    (void)actual_response_tx;
    return ESP_OK;
}

static void uwb_passive_ds_anchor_loop_multipoint(
    const uint8_t *anchor_ids, size_t anchor_count)
{
    if (uwb_passive_ds_schedule_timer_init() != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        return;
    }
    struct uwb_passive_ds_multi_schedule schedule = {
        .last_poll_host_us =
            esp_timer_get_time() -
            (s_runtime_hot_entry
                 ? UWB_PASSIVE_DS_BOOTSTRAP_LISTEN_US -
                       UWB_HOT_SWITCH_BOOTSTRAP_GUARD_US
                 : 0),
    };
    uint32_t good_frames = 0U;
    uint32_t partial_frames = 0U;
    uint32_t failed_frames = 0U;
    int64_t summary_started_us = esp_timer_get_time();
    int64_t armed_alarm_due_host_us = 0;
    s_status = UWB_DW3000_STATUS_READY;

    while (!uwb_dw3000_runtime_switch_pending()) {
        const app_runtime_config_t *config = app_runtime_config_get();
        const int64_t now_us = esp_timer_get_time();
        const int64_t frame_period_us =
            (int64_t)config->passive_ds_slot_ms * 1000LL +
            (config->passive_ds_solve_mode ==
                     APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
                 ? APP_UWB_PASSIVE_DS_DYNAMIC_GUARD_US
                 : (int64_t)config->passive_ds_round_gap_ms * 1000LL);
        const int64_t recovery_listen_us =
            frame_period_us * UWB_PASSIVE_DS_MULTI_RECOVERY_FRAMES >
                    UWB_PASSIVE_DS_MULTI_RECOVERY_MIN_US
                ? frame_period_us *
                      UWB_PASSIVE_DS_MULTI_RECOVERY_FRAMES
                : UWB_PASSIVE_DS_MULTI_RECOVERY_MIN_US;
        bool bootstrap = false;
        bool initiate = schedule.synced &&
            now_us >= schedule.next_poll_host_us -
                          UWB_PASSIVE_DS_MULTI_POLL_TX_LEAD_US;
        uint32_t frame_id = schedule.next_frame_id;
        if (!schedule.synced && s_source_id == anchor_ids[0] &&
            now_us - schedule.last_poll_host_us >=
                recovery_listen_us) {
            initiate = true;
            bootstrap = true;
            frame_id = 0U;
            schedule.bootstrap_count++;
        }

        if (initiate) {
            if (armed_alarm_due_host_us != 0) {
                (void)esp_timer_stop(s_passive_ds_schedule_timer);
                armed_alarm_due_host_us = 0;
                s_passive_ds_schedule_alarm_fired = false;
            }
            const int64_t lateness_us = bootstrap
                ? 0
                : now_us - schedule.next_poll_host_us;
            bool immediate_poll = bootstrap;
            if (!bootstrap && lateness_us > UWB_PASSIVE_DS_POLL_LATE_US) {
                /* Actual POLL timestamps are carried in the DS exchange, so
                 * an immediate recovery POLL preserves ranging accuracy. */
                schedule.late_count++;
                immediate_poll = true;
            }
            uint64_t poll_tx = 0U;
            int64_t poll_host = now_us;
            uint8_t responses = 0U;
            const esp_err_t err = uwb_passive_ds_multi_initiate_frame(
                anchor_ids, anchor_count, frame_id,
                immediate_poll ? 0U : schedule.next_poll_radio_ts,
                &poll_tx, &poll_host, &responses);
            schedule.synced = false;
            schedule.last_poll_host_us = poll_host;
            if (err == ESP_OK) {
                if (responses == anchor_count - 1U) {
                    good_frames++;
                } else {
                    partial_frames++;
                }
            } else {
                failed_frames++;
            }
            continue;
        }

        if (schedule.synced &&
            armed_alarm_due_host_us != schedule.next_poll_host_us) {
            uwb_passive_ds_arm_schedule_alarm(
                schedule.next_poll_host_us,
                UWB_PASSIVE_DS_MULTI_POLL_TX_LEAD_US);
            armed_alarm_due_host_us = schedule.next_poll_host_us;
        } else if (!schedule.synced && armed_alarm_due_host_us != 0) {
            (void)esp_timer_stop(s_passive_ds_schedule_timer);
            s_passive_ds_schedule_alarm_fired = false;
            armed_alarm_due_host_us = 0;
        }

        /* Do not let the blocking RX slice cross the point where this
         * anchor must arm the next delayed POLL.  The ESP timer alarm is a
         * useful wake hint, but it cannot interrupt an SPI-backed receive
         * already in progress.  Keeping the receive slice inside the radio
         * deadline preserves the DW3000 delayed-TX lead time without adding
         * another task or a host-clock synchronization layer. */
        uint32_t poll_rx_slice_ms = config->passive_ds_rx_slice_ms;
        if (schedule.synced) {
            const int64_t arm_due_us =
                schedule.next_poll_host_us -
                UWB_PASSIVE_DS_MULTI_POLL_TX_LEAD_US;
            const int64_t until_arm_us = arm_due_us - esp_timer_get_time();
            if (until_arm_us <= 0) {
                continue;
            }
            uint32_t deadline_slice_ms =
                (uint32_t)(until_arm_us / 1000LL);
            if (deadline_slice_ms == 0U) {
                deadline_slice_ms = 1U;
            }
            if (poll_rx_slice_ms > deadline_slice_ms) {
                poll_rx_slice_ms = deadline_slice_ms;
            }
        }

        struct uwb_distance_frame poll = {0};
        const esp_err_t rx_err = uwb_distance_receive_next(
            &poll, poll_rx_slice_ms);
        if (s_passive_ds_schedule_alarm_fired) {
            s_passive_ds_schedule_alarm_fired = false;
            armed_alarm_due_host_us = 0;
        }
        if (rx_err != ESP_OK) {
            if (rx_err != ESP_ERR_TIMEOUT &&
                rx_err != ESP_ERR_NOT_FINISHED) {
                failed_frames++;
            }
            continue;
        }
        if (poll.type != UWB_DISTANCE_FRAME_PASSIVE_DS_MULTI_POLL ||
            poll.destination_id != UWB_DISTANCE_FRAME_BROADCAST_ID) {
            continue;
        }
        uint32_t received_frame_id = 0U;
        struct uwb_passive_ds_multi_plan plan = {0};
        if (!uwb_passive_ds_multi_decode_poll(
                poll.payload, poll.payload_len,
                UWB_PASSIVE_DS_MULTI_APPLICATION_OFFSET,
                &received_frame_id) ||
            !uwb_passive_ds_multi_build_plan(
                anchor_ids, anchor_count, received_frame_id, &plan) ||
            poll.source_id != plan.initiator_id ||
            poll.sequence != (uint16_t)(received_frame_id & 0xffffU)) {
            uwb_passive_ds_runtime_increment(UWB_PASSIVE_DS_RUNTIME_COUNTER_INVALID_FRAME);
            continue;
        }

        uwb_passive_ds_multi_schedule_from_poll(
            &schedule, received_frame_id, poll.rx_timestamp,
            poll.rx_host_time_us, anchor_ids, anchor_count, config);
        const int responder_index =
            uwb_passive_ds_multi_responder_index(&plan, s_source_id);
        if (responder_index >= 0) {
            const esp_err_t response_err =
                uwb_passive_ds_multi_respond_to_poll(
                    &poll, &plan, (uint8_t)responder_index);
            if (response_err != ESP_OK) {
                failed_frames++;
            }
        }

        const int64_t summary_now_us = esp_timer_get_time();
        if (summary_now_us - summary_started_us >= 1000000LL) {
            ESP_LOGI(TAG,
                     "PASSIVE_DS multipoint N+2 full=%lu partial=%lu "
                     "fail=%lu late_recover=%lu period_us=%lld "
                     "guard_us=%lu packets=5",
                     (unsigned long)good_frames,
                     (unsigned long)partial_frames,
                     (unsigned long)failed_frames,
                     (unsigned long)schedule.late_count,
                     (long long)frame_period_us,
                     (unsigned long)(
                         config->passive_ds_solve_mode ==
                                 APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
                             ? APP_UWB_PASSIVE_DS_DYNAMIC_GUARD_US
                             : config->passive_ds_round_gap_ms * 1000U));
            good_frames = 0U;
            partial_frames = 0U;
            failed_frames = 0U;
            schedule.late_count = 0U;
            summary_started_us = summary_now_us;
        }
    }
    (void)esp_timer_stop(s_passive_ds_schedule_timer);
    s_passive_ds_schedule_alarm_fired = false;
}

static void uwb_passive_ds_anchor_loop(const uint8_t *anchor_ids,
                                       size_t anchor_count)
{
    if (app_runtime_config_get()->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS) {
        uwb_passive_ds_anchor_loop_multipoint(anchor_ids, anchor_count);
        return;
    }
    if (app_runtime_config_get()->passive_ds_pipeline_mode ==
        APP_RUNTIME_PASSIVE_DS_PIPELINE_DEADLINE) {
        uwb_passive_ds_anchor_loop_deadline(anchor_ids, anchor_count);
    } else {
        uwb_passive_ds_anchor_loop_legacy(anchor_ids, anchor_count);
    }
}

static void uwb_dw3000_passive_ds_twr_loop(void)
{
    uint8_t anchor_ids[UWB_ANCHOR_SURVEY_MAX_ANCHORS] = {0};
    const size_t anchor_count =
        uwb_anchor_survey_anchor_ids(anchor_ids);
    const app_runtime_config_t *config = app_runtime_config_get();
    uwb_passive_ds_runtime_reset(
        config->passive_ds_pipeline_mode ==
        APP_RUNTIME_PASSIVE_DS_PIPELINE_DEADLINE);
    if (anchor_count < 3U ||
        !uwb_anchor_survey_ids_valid(anchor_ids, anchor_count,
                                     anchor_ids[0])) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "PASSIVE_DS invalid anchor configuration");
        vTaskDelete(NULL);
        return;
    }

    const bool multipoint =
        config->passive_ds_schedule ==
        APP_RUNTIME_PASSIVE_DS_MULTIPOINT_FULL_DS;
    const uint32_t slots_per_frame = (uint32_t)anchor_count - 1U;
    const uint32_t frame_period_us = multipoint
        ? config->passive_ds_slot_ms * 1000U +
              (config->passive_ds_solve_mode ==
                       APP_RUNTIME_PASSIVE_DS_SOLVE_SINGLE_STAR
                   ? APP_UWB_PASSIVE_DS_DYNAMIC_GUARD_US
                   : config->passive_ds_round_gap_ms * 1000U)
        : (slots_per_frame * config->passive_ds_slot_ms +
           config->passive_ds_round_gap_ms) * 1000U;
    const char *schedule_name = multipoint
        ? "multipoint_full_ds_n_plus_2"
        : (config->passive_ds_schedule ==
                   APP_RUNTIME_PASSIVE_DS_ROBUST_ROTATING
               ? "robust_rotating"
               : "fast_star");
    ESP_LOGI(TAG,
             "PASSIVE_DS runtime source=%u schedule=%s anchors=%u "
             "period=%lu us slot=%lu ms pipeline=%s solve=%s; "
             "non-anchors are receive-only tags",
             (unsigned)s_source_id,
             schedule_name,
             (unsigned)anchor_count, (unsigned long)frame_period_us,
             (unsigned long)config->passive_ds_slot_ms,
             config->passive_ds_pipeline_mode ==
                     APP_RUNTIME_PASSIVE_DS_PIPELINE_DEADLINE
                 ? "deadline"
                 : "legacy",
             uwb_passive_ds_solve_mode_name(
                 config->passive_ds_solve_mode));
    if (uwb_anchor_survey_id_in_set(
            anchor_ids, anchor_count, s_source_id)) {
        uwb_passive_ds_anchor_loop(anchor_ids, anchor_count);
        return;
    }
    uwb_passive_ds_tag_loop(anchor_ids, anchor_count);
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

static esp_err_t native_ds_send_immediate_expect_rx(
    void *context, const uint8_t *payload, size_t payload_len,
    uint32_t rx_after_tx_delay_uus, uint32_t rx_timeout_ms,
    uint64_t *tx_timestamp)
{
    (void)context;
    return uwb_dw3000_send_payload_expect_rx(
        payload, payload_len, rx_after_tx_delay_uus, rx_timeout_ms,
        tx_timestamp);
}

static esp_err_t native_ds_send_delayed(
    void *context, const uint8_t *payload, size_t payload_len,
    uint64_t due_timestamp, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    (void)context;
    return uwb_dw3000_send_payload_delayed(
        payload, payload_len, due_timestamp, programmed_tx_timestamp,
        actual_tx_timestamp);
}

static esp_err_t native_ds_send_delayed_expect_rx(
    void *context, const uint8_t *payload, size_t payload_len,
    uint64_t due_timestamp, uint32_t rx_after_tx_delay_uus,
    uint32_t rx_timeout_ms, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    (void)context;
    return uwb_dw3000_send_payload_delayed_expect_rx(
        payload, payload_len, due_timestamp, rx_after_tx_delay_uus,
        rx_timeout_ms, programmed_tx_timestamp, actual_tx_timestamp);
}

static esp_err_t native_ds_receive(
    void *context, struct uwb_native_ds_rx_frame *frame,
    uint32_t timeout_ms)
{
    (void)context;
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    struct uwb_dw3000_rx_frame radio_frame = {0};
    const esp_err_t err = uwb_dw3000_receive_frame(
        &radio_frame, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (radio_frame.payload_len > sizeof(frame->payload)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memset(frame, 0, sizeof(*frame));
    frame->payload_len = radio_frame.payload_len;
    frame->rx_timestamp = radio_frame.rx_timestamp;
    memcpy(frame->payload, radio_frame.payload, radio_frame.payload_len);
    return ESP_OK;
}

static uint64_t native_ds_add_delay_ms(
    void *context, uint64_t timestamp, uint32_t delay_ms)
{
    (void)context;
    return uwb_dw3000_add_timestamp_delta(
        timestamp, uwb_dw3000_ms_to_dtu(delay_ms));
}

static uint64_t native_ds_programmed_tx_timestamp(
    void *context, uint64_t due_timestamp)
{
    (void)context;
    return uwb_dw3000_programmed_tx_timestamp(
        uwb_dw3000_delayed_time_word(due_timestamp));
}

static int64_t native_ds_now_us(void *context)
{
    (void)context;
    return esp_timer_get_time();
}

static void native_ds_delay_timer_callback(void *arg)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)arg;
    if (semaphore != NULL) {
        (void)xSemaphoreGive(semaphore);
    }
}

static esp_err_t native_ds_delay_timer_ensure_ready(void)
{
    if (s_native_ds_delay_timer != NULL &&
        s_native_ds_delay_semaphore != NULL) {
        return ESP_OK;
    }
    if (s_native_ds_delay_semaphore == NULL) {
        s_native_ds_delay_semaphore = xSemaphoreCreateBinaryStatic(
            &s_native_ds_delay_semaphore_storage);
        if (s_native_ds_delay_semaphore == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    const esp_timer_create_args_t timer_args = {
        .callback = native_ds_delay_timer_callback,
        .arg = s_native_ds_delay_semaphore,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "native_ds_pace",
        .skip_unhandled_events = true,
    };
    return esp_timer_create(&timer_args, &s_native_ds_delay_timer);
}

static void native_ds_delay_ms(void *context, uint32_t delay_ms)
{
    (void)context;
    if (delay_ms == 0U) {
        return;
    }
    if (native_ds_delay_timer_ensure_ready() != ESP_OK) {
        uwb_dw3000_delay_ms(delay_ms);
        return;
    }

    /* Drain a callback left by a stopped/restarted runtime before arming the
     * next one-shot. There is only one Native DS task, so this timer and
     * semaphore never have concurrent waiters. */
    while (xSemaphoreTake(s_native_ds_delay_semaphore, 0) == pdTRUE) {
    }
    (void)esp_timer_stop(s_native_ds_delay_timer);
    if (esp_timer_start_once(s_native_ds_delay_timer,
                             (uint64_t)delay_ms * 1000ULL) != ESP_OK) {
        uwb_dw3000_delay_ms(delay_ms);
        return;
    }
    (void)xSemaphoreTake(s_native_ds_delay_semaphore, portMAX_DELAY);
}

static bool native_ds_stop_requested(void *context)
{
    (void)context;
    return uwb_dw3000_runtime_switch_pending();
}

static void native_ds_set_ready(void *context)
{
    (void)context;
    s_status = UWB_DW3000_STATUS_READY;
}

static void uwb_dw3000_ranging_loop(void)
{
    uint8_t anchor_ids[UWB_NATIVE_DS_MAX_ANCHORS] = {0};
    const size_t anchor_count = uwb_anchor_survey_anchor_ids(anchor_ids);
    const app_runtime_config_t *runtime = app_runtime_config_get();
    if (anchor_count == 0U || anchor_count > UWB_NATIVE_DS_MAX_ANCHORS) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Native DS-TWR invalid anchor configuration");
        return;
    }

    struct uwb_native_ds_config config = {
        .source_id = s_source_id,
        .tag_id = runtime->tag_id,
        .anchor_count = (uint8_t)anchor_count,
        .slot_ms = runtime->ranging_slot_ms,
        .round_gap_ms = runtime->ranging_round_gap_ms,
        .rx_slice_ms = runtime->ranging_rx_slice_ms,
        .rx_timeout_ms = runtime->ranging_rx_timeout_ms,
        .response_delay_ms = runtime->ranging_resp_delay_ms,
        .final_delay_ms = runtime->ranging_final_delay_ms,
        .auto_rx_delay_uus = runtime->ranging_auto_rx_delay_uus,
        .maximum_distance_m = APP_UWB_RANGING_MAX_DISTANCE_M,
        .fixed_geometry = runtime->flex_tdoa_geometry_fixed,
        .geometry_version = runtime->flex_tdoa_geometry_generation,
        .range_calibration_enabled =
            runtime->native_ds_calibration_enabled,
        .range_calibration_generation =
            runtime->native_ds_calibration_generation,
    };
    memcpy(config.anchor_ids, anchor_ids, anchor_count);
    memcpy(config.anchor_x_mm, runtime->flex_tdoa_anchor_x_mm,
           anchor_count * sizeof(config.anchor_x_mm[0]));
    memcpy(config.anchor_y_mm, runtime->flex_tdoa_anchor_y_mm,
           anchor_count * sizeof(config.anchor_y_mm[0]));
    memcpy(config.anchor_range_bias_mm,
           runtime->native_ds_range_bias_mm,
           anchor_count * sizeof(config.anchor_range_bias_mm[0]));

    const struct uwb_native_ds_radio_ops radio = {
        .send_immediate_expect_rx = native_ds_send_immediate_expect_rx,
        .send_delayed = native_ds_send_delayed,
        .send_delayed_expect_rx = native_ds_send_delayed_expect_rx,
        .receive = native_ds_receive,
        .add_delay_ms = native_ds_add_delay_ms,
        .programmed_tx_timestamp = native_ds_programmed_tx_timestamp,
        .now_us = native_ds_now_us,
        .delay_ms = native_ds_delay_ms,
        .stop_requested = native_ds_stop_requested,
        .set_ready = native_ds_set_ready,
        .consume_report = NULL,
        .context = NULL,
    };

    ESP_LOGI(TAG,
             "Native DS-TWR clean runtime: source=%u tag=%u anchors=%u "
             "slot=%lu ms gap=%lu ms timeout=%lu ms resp=%lu ms "
             "final=%lu ms range_cal=%s generation=%lu; "
             "no clock correction",
             (unsigned)config.source_id, (unsigned)config.tag_id,
             (unsigned)config.anchor_count, (unsigned long)config.slot_ms,
             (unsigned long)config.round_gap_ms,
             (unsigned long)config.rx_timeout_ms,
             (unsigned long)config.response_delay_ms,
             (unsigned long)config.final_delay_ms,
             config.range_calibration_enabled ? "on" : "off",
             (unsigned long)config.range_calibration_generation);
    const esp_err_t err = uwb_native_ds_runtime_run(&config, &radio);
    if (err != ESP_OK && !uwb_dw3000_runtime_switch_pending()) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Native DS-TWR runtime failed: %s",
                 esp_err_to_name(err));
    }
}

#include "uwb_dw3000_calibration.inc"

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
    if (s_rx_armed) {
        ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF), TAG,
                            "TXRXOFF command failed");
        s_rx_armed = false;
    }
    if (!s_flex_tdoa_anchor_request_buffer_pending) {
        ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                            "clear before TX failed");
    }
    ESP_RETURN_ON_ERROR(uwb_dw3000_write_tx_payload(payload, payload_len), TAG,
                        "TX buffer write failed");
    ESP_RETURN_ON_ERROR(uwb_dw3000_set_frame_length(payload_len), TAG,
                        "TX frame length write failed");
    return ESP_OK;
}

static esp_err_t uwb_dw3000_wait_for_tx_complete_timeout(
    uint64_t *tx_timestamp, uint32_t timeout_ms, bool delayed_tx,
    uint32_t delayed_time_word)
{
    if (timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t start = esp_timer_get_time();
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
            ESP_RETURN_ON_ERROR(uwb_calibration_timer_accept_tx_sync(), TAG,
                                "calibration timer TX sync start failed");
            s_tx_count++;
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX failed");
            return ESP_OK;
        }

        if ((status & DW3000_STATUS_HPDWARN) != 0U) {
            s_tx_error_count++;
            ESP_LOGW(TAG, "UWB delayed TX missed SYS_STATUS=0x%08lx",
                     (unsigned long)status);
            ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF),
                                TAG, "TXRXOFF after delayed TX miss failed");
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after delayed TX miss failed");
            return ESP_ERR_INVALID_STATE;
        }

        if (uwb_dw3000_remaining_ms(start, timeout_ms) == 0U) {
            s_tx_error_count++;
            if (delayed_tx) {
                uint32_t system_state = 0;
                uint32_t system_time_word = 0;
                uint8_t rdb_status = 0;
                const esp_err_t state_err = uwb_dw3000_read32(
                    DW3000_REG_DIG_DIAG, DW3000_SYS_STATE_SUB,
                    &system_state);
                const esp_err_t time_err = uwb_dw3000_read32(
                    DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_TIME_SUB,
                    &system_time_word);
                const esp_err_t rdb_err =
                    uwb_dw3000_read_rdb_status(&rdb_status);
                const int32_t deadline_delta_word =
                    (int32_t)(delayed_time_word - system_time_word);
                const double deadline_delta_us =
                    (double)deadline_delta_word * 256.0 *
                    UWB_DW3000_TIME_UNIT_SECONDS * 1000000.0;
                ESP_LOGW(TAG,
                         "UWB delayed TX timeout SYS_STATUS=0x%08lx "
                         "SYS_STATE=0x%08lx RDB=0x%02x target=0x%08lx "
                         "now=0x%08lx delta=%.1f us "
                         "diag=%s/%s/%s",
                         (unsigned long)status,
                         (unsigned long)system_state, rdb_status,
                         (unsigned long)delayed_time_word,
                         (unsigned long)system_time_word, deadline_delta_us,
                         esp_err_to_name(state_err),
                         esp_err_to_name(time_err), esp_err_to_name(rdb_err));
            } else {
                ESP_LOGW(TAG, "UWB TX timeout SYS_STATUS=0x%08lx",
                         (unsigned long)status);
            }
            ESP_RETURN_ON_ERROR(uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF),
                                TAG, "TXRXOFF after TX timeout failed");
            ESP_RETURN_ON_ERROR(uwb_dw3000_clear_status(), TAG,
                                "clear after TX timeout failed");
            return ESP_ERR_TIMEOUT;
        }

        uwb_dw3000_wait_for_event_or_delay(start, timeout_ms,
                                           UWB_DW3000_TX_POLL_MS);
    }
}

static esp_err_t uwb_dw3000_wait_for_tx_complete(uint64_t *tx_timestamp)
{
    return uwb_dw3000_wait_for_tx_complete_timeout(
        tx_timestamp, UWB_DW3000_TX_TIMEOUT_MS, false, 0U);
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

static esp_err_t uwb_dw3000_send_payload_delayed_timeout(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint32_t tx_timeout_ms, uint64_t *programmed_tx_timestamp,
    uint64_t *actual_tx_timestamp)
{
    if (tx_timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t arm_start_us = esp_timer_get_time();
    s_last_delayed_arm_us = 0;
    const uint32_t delayed_time_word =
        uwb_dw3000_delayed_time_word(tx_timestamp);
    const char *failed_step = "SPI bus acquire for delayed TX";
    esp_err_t err = spi_device_acquire_bus(s_spi, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", failed_step, esp_err_to_name(err));
        return err;
    }

    // Keep TX_BUFFER, DX_TIME, deferred RDB release and CMD_DTX in one SPI
    // ownership interval. Individual transactions still control CS, but no
    // other caller can add host-side arbitration gaps to this critical path.
    failed_step = "delayed TX prepare";
    err = uwb_dw3000_prepare_tx(payload, payload_len);
    if (err != ESP_OK) {
        goto release_bus;
    }
    failed_step = "DX_TIME write";
    err = uwb_dw3000_set_delayed_trx_time(delayed_time_word);
    if (err != ESP_OK) {
        goto release_bus;
    }
    if (programmed_tx_timestamp != NULL) {
        *programmed_tx_timestamp =
            uwb_dw3000_programmed_tx_timestamp(delayed_time_word);
    }

    // The copied REQ remains valid in host memory. Return the occupied DW3000
    // double buffer before arming delayed TX; DB_TOGGLE while delayed TX is
    // pending can cancel the scheduled transmission on real hardware.
    failed_step = "FlexTDOA REQ buffer release before DTX";
    err = uwb_dw3000_release_pending_flex_request();
    if (err != ESP_OK) {
        goto release_bus;
    }
    failed_step = "delayed TX command";
    err = uwb_dw3000_fast_command(DW3000_CMD_DTX);
    if (err != ESP_OK) {
        goto release_bus;
    }
    s_last_delayed_arm_us = (uint32_t)(esp_timer_get_time() - arm_start_us);

    uint32_t status = 0;
    failed_step = "SYS_STATUS after delayed TX";
    err = uwb_dw3000_read32(DW3000_REG_GEN_CFG_AES_LOW,
                            DW3000_SYS_STATUS_SUB, &status);
    if (err != ESP_OK) {
        goto release_bus;
    }
    if ((status & DW3000_STATUS_HPDWARN) != 0) {
        s_tx_error_count++;
        uint32_t system_time_word = 0;
        const esp_err_t time_err = uwb_dw3000_read32(
            DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_TIME_SUB,
            &system_time_word);
        const int32_t lead_word =
            (int32_t)(delayed_time_word - system_time_word);
        const double lead_us =
            (double)lead_word * 256.0 * UWB_DW3000_TIME_UNIT_SECONDS *
            1000000.0;
        ESP_LOGW(TAG,
                 "UWB delayed TX rejected timestamp=0x%010llx "
                 "word=0x%08lx now=0x%08lx lead=%.1f us time_err=%s "
                 "SYS_STATUS=0x%08lx",
                 (unsigned long long)tx_timestamp,
                 (unsigned long)delayed_time_word,
                 (unsigned long)system_time_word, lead_us,
                 esp_err_to_name(time_err), (unsigned long)status);
        failed_step = "TXRXOFF after delayed TX reject";
        err = uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
        if (err != ESP_OK) {
            goto release_bus;
        }
        failed_step = "clear after delayed TX reject";
        err = uwb_dw3000_clear_status();
        if (err == ESP_OK) {
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        uint32_t system_state = 0;
        failed_step = "SYS_STATE after delayed TX";
        err = uwb_dw3000_read32(DW3000_REG_DIG_DIAG,
                                DW3000_SYS_STATE_SUB, &system_state);
        if (err != ESP_OK) {
            goto release_bus;
        }
        if (system_state == DW3000_SYS_STATE_DELAYED_TX_ERROR) {
            // DW3000 erratum: a late delayed TX can enter TSE=TX while its TX
            // state remains IDLE without asserting HPDWARN or ever setting
            // TXFRS. This is the same signature checked by Qorvo dwt_starttx().
            s_tx_error_count++;
            uint32_t system_time_word = 0;
            const esp_err_t time_err = uwb_dw3000_read32(
                DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_TIME_SUB,
                &system_time_word);
            const int32_t lead_word =
                (int32_t)(delayed_time_word - system_time_word);
            const double lead_us =
                (double)lead_word * 256.0 * UWB_DW3000_TIME_UNIT_SECONDS *
                1000000.0;
            ESP_LOGW(TAG,
                     "UWB delayed TX state error timestamp=0x%010llx "
                     "word=0x%08lx now=0x%08lx lead=%.1f us time_err=%s "
                     "SYS_STATE=0x%08lx arm=%lu us",
                     (unsigned long long)tx_timestamp,
                     (unsigned long)delayed_time_word,
                     (unsigned long)system_time_word, lead_us,
                     esp_err_to_name(time_err), (unsigned long)system_state,
                     (unsigned long)s_last_delayed_arm_us);
            failed_step = "TXRXOFF after delayed TX state error";
            err = uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
            if (err != ESP_OK) {
                goto release_bus;
            }
            failed_step = "clear after delayed TX state error";
            err = uwb_dw3000_clear_status();
            if (err == ESP_OK) {
                err = ESP_ERR_INVALID_STATE;
            }
        }
    }

release_bus:
    spi_device_release_bus(s_spi);
    if (err != ESP_OK) {
        if (err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "%s failed: %s", failed_step,
                     esp_err_to_name(err));
        }
        return err;
    }

    return uwb_dw3000_wait_for_tx_complete_timeout(
        actual_tx_timestamp, tx_timeout_ms, true, delayed_time_word);
}

static esp_err_t uwb_dw3000_send_payload_delayed(
    const uint8_t *payload, size_t payload_len, uint64_t tx_timestamp,
    uint64_t *programmed_tx_timestamp, uint64_t *actual_tx_timestamp)
{
    return uwb_dw3000_send_payload_delayed_timeout(
        payload, payload_len, tx_timestamp, UWB_DW3000_TX_TIMEOUT_MS,
        programmed_tx_timestamp, actual_tx_timestamp);
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
        uint32_t system_time_word = 0;
        const esp_err_t time_err = uwb_dw3000_read32(
            DW3000_REG_GEN_CFG_AES_LOW, DW3000_SYS_TIME_SUB,
            &system_time_word);
        const int32_t lead_word =
            (int32_t)(delayed_time_word - system_time_word);
        const double lead_us =
            (double)lead_word * 256.0 * UWB_DW3000_TIME_UNIT_SECONDS *
            1000000.0;
        ESP_LOGW(TAG,
                 "UWB delayed TX/RX rejected timestamp=0x%010llx "
                 "word=0x%08lx now=0x%08lx lead=%.1f us time_err=%s "
                 "SYS_STATUS=0x%08lx",
                 (unsigned long long)tx_timestamp,
                 (unsigned long)delayed_time_word,
                 (unsigned long)system_time_word, lead_us,
                 esp_err_to_name(time_err), (unsigned long)status);
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

static esp_err_t uwb_dw3000_reinitialize_runtime(
    enum uwb_dw3000_runtime_mode runtime_mode)
{
    const int64_t started_us = esp_timer_get_time();
    s_runtime_switch_active = true;
    s_status = UWB_DW3000_STATUS_INITIALIZING;

    if (s_flex_tdoa_schedule_timer != NULL) {
        (void)esp_timer_stop(s_flex_tdoa_schedule_timer);
    }
    if (s_passive_ds_schedule_timer != NULL) {
        (void)esp_timer_stop(s_passive_ds_schedule_timer);
    }
    s_flex_tdoa_schedule_alarm_fired = false;
    s_passive_ds_schedule_alarm_fired = false;
    s_flex_tdoa_local_request.active = false;
    s_flex_tdoa_anchor_request_buffer_pending = false;
    (void)uwb_dw3000_fast_command(DW3000_CMD_TXRXOFF);
    (void)uwb_dw3000_clear_status();
    s_rx_armed = false;
    s_rx_double_buffer_enabled = false;
    s_rx_double_buffer_index = 0;
    s_flex_tdoa_rx_timestamp_reference_valid = false;
    s_tx_fctrl_base_valid = false;
    s_tx_fctrl_payload_len = SIZE_MAX;
    (void)ulTaskNotifyTake(pdTRUE, 0);

    s_runtime_mode = runtime_mode;
    esp_err_t err = uwb_dw3000_restore_boot_spi(s_device_id);
    if (err == ESP_OK) {
        err = uwb_dw3000_radio_init();
    }
    if (err == ESP_OK) {
        err = uwb_dw3000_enable_operational_spi(s_device_id);
    }
    if (err == ESP_OK &&
        uwb_dw3000_runtime_uses_high_rate_rx_buffer(runtime_mode)) {
        err = uwb_dw3000_configure_high_rate_rx_double_buffer();
    }
    if (err != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        s_runtime_switch_active = false;
        ESP_LOGE(TAG, "DW3000 hot runtime reinitialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    s_runtime_hot_entry = true;
    s_runtime_switch_count++;
    s_last_runtime_switch_ms =
        (uint32_t)((esp_timer_get_time() - started_us + 999LL) / 1000LL);
    s_runtime_switch_active = false;
    ESP_LOGI(TAG,
             "DW3000 hot runtime reinitialized mode=%u in %lu ms; Wi-Fi and telemetry stayed online",
             (unsigned)runtime_mode,
             (unsigned long)s_last_runtime_switch_ms);
    return ESP_OK;
}

static void uwb_dw3000_run_selected_runtime(void)
{
    if (s_runtime_mode == UWB_DW3000_RUNTIME_DISTANCE_TEST) {
        uwb_dw3000_distance_test_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_CALIBRATION) {
        uwb_dw3000_calibration_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_ANCHOR_SURVEY) {
        uwb_dw3000_anchor_survey_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_RANGING) {
        uwb_dw3000_ranging_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_FLEX_TDOA) {
        uwb_dw3000_flex_tdoa_loop();
    } else if (s_runtime_mode == UWB_DW3000_RUNTIME_PASSIVE_DS_TWR) {
        uwb_dw3000_passive_ds_twr_loop();
    } else {
        uwb_dw3000_radio_loop();
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

    err = uwb_dw3000_enable_operational_spi(s_device_id);
    if (err != ESP_OK) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "DW3000 SPI clock setup failed: %s",
                 esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    if (uwb_dw3000_runtime_uses_high_rate_rx_buffer(s_runtime_mode)) {
        err = uwb_dw3000_configure_high_rate_rx_double_buffer();
        if (err != ESP_OK) {
            s_status = UWB_DW3000_STATUS_FAILED;
            ESP_LOGE(TAG, "DW3000 FlexTDOA RX buffer setup failed: %s",
                     esp_err_to_name(err));
            vTaskDelete(NULL);
            return;
        }
    }

    s_requested_runtime_mode = s_runtime_mode;
    s_runtime_switch_request_generation = 0;
    s_runtime_switch_applied_generation = 0;
    s_runtime_hot_entry = false;

    while (true) {
        uwb_dw3000_run_selected_runtime();
        if (!uwb_dw3000_runtime_switch_pending()) {
            s_status = UWB_DW3000_STATUS_FAILED;
            ESP_LOGE(TAG, "UWB runtime returned without a switch request");
            break;
        }

        const uint32_t request_generation =
            s_runtime_switch_request_generation;
        const enum uwb_dw3000_runtime_mode requested_runtime =
            s_requested_runtime_mode;
        const esp_err_t switch_err =
            uwb_dw3000_reinitialize_runtime(requested_runtime);
        if (switch_err != ESP_OK) {
            ESP_LOGE(TAG,
                     "Hot switch failed; rebooting ESP32 to recover persisted runtime");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
        s_runtime_switch_applied_generation = request_generation;
    }

    s_task_handle = NULL;
    s_started = false;
    vTaskDelete(NULL);
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
    s_requested_runtime_mode = runtime_mode;

    const BaseType_t created = xTaskCreatePinnedToCore(
        uwb_dw3000_task, "uwb_dw3000", UWB_DW3000_TASK_STACK_BYTES, NULL,
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

esp_err_t uwb_dw3000_start_flex_tdoa(void)
{
    return uwb_dw3000_start_runtime(UWB_DW3000_RUNTIME_FLEX_TDOA);
}

esp_err_t uwb_dw3000_start_passive_ds_twr(void)
{
    return uwb_dw3000_start_runtime(
        UWB_DW3000_RUNTIME_PASSIVE_DS_TWR);
}

bool uwb_dw3000_hot_switch_mode_supported(uint8_t app_runtime_mode)
{
    enum uwb_dw3000_runtime_mode runtime_mode =
        UWB_DW3000_RUNTIME_BEACON_SMOKE;
    return uwb_dw3000_runtime_from_app_mode(
               app_runtime_mode, &runtime_mode) &&
           uwb_dw3000_runtime_mode_hot_switchable(runtime_mode);
}

esp_err_t uwb_dw3000_request_runtime_switch(uint8_t app_runtime_mode)
{
    enum uwb_dw3000_runtime_mode runtime_mode =
        UWB_DW3000_RUNTIME_BEACON_SMOKE;
    if (!uwb_dw3000_runtime_from_app_mode(
            app_runtime_mode, &runtime_mode) ||
        !uwb_dw3000_runtime_mode_hot_switchable(runtime_mode)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!s_started || s_task_handle == NULL ||
        !uwb_dw3000_runtime_mode_hot_switchable(s_runtime_mode)) {
        return ESP_ERR_INVALID_STATE;
    }
    s_requested_runtime_mode = runtime_mode;
    s_runtime_switch_request_generation++;
    if (s_runtime_switch_request_generation == 0U) {
        s_runtime_switch_request_generation = 1U;
        s_runtime_switch_applied_generation = 0U;
    }
    xTaskNotifyGive(s_task_handle);
    ESP_LOGI(TAG,
             "DW3000 hot runtime switch requested current=%u target=%u generation=%lu",
             (unsigned)s_runtime_mode, (unsigned)runtime_mode,
             (unsigned long)s_runtime_switch_request_generation);
    return ESP_OK;
}

bool uwb_dw3000_runtime_switch_in_progress(void)
{
    return s_runtime_switch_active ||
           uwb_dw3000_runtime_switch_pending();
}

uint32_t uwb_dw3000_get_runtime_switch_count(void)
{
    return s_runtime_switch_count;
}

uint32_t uwb_dw3000_get_last_runtime_switch_ms(void)
{
    return s_last_runtime_switch_ms;
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

uint32_t uwb_dw3000_get_spi_clock_hz(void)
{
    return s_spi_clock_hz;
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

void uwb_dw3000_get_passive_ds_pipeline_stats(
    struct uwb_passive_ds_pipeline_stats *stats)
{
    if (stats == NULL) {
        return;
    }
    uwb_passive_ds_runtime_get_stats(stats);
}

void uwb_dw3000_get_native_ds_pipeline_stats(
    struct uwb_native_ds_pipeline_stats *stats)
{
    uwb_native_ds_twr_get_stats(stats);
}
