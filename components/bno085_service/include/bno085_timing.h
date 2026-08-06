#ifndef BNO085_TIMING_H
#define BNO085_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BNO085_FUSION_TIMER_HZ = 10000000U,
    BNO085_SH2_TICK_US = 100U,
};

int32_t bno085_timing_read_le_i32(const uint8_t *data);
uint16_t bno085_timing_report_delay_100us(const uint8_t *report);
uint64_t bno085_timing_reconstruct_ticks(uint64_t hint_ticks,
                                         int32_t base_delta_100us,
                                         int32_t rebase_delta_100us,
                                         uint16_t report_delay_100us);
bool bno085_timing_due(uint64_t previous_ticks, uint64_t current_ticks,
                       uint32_t rate_hz);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_TIMING_H */
