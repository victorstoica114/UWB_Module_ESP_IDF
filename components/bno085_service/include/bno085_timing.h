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
uint64_t bno085_timing_causal_monotonic(uint64_t candidate_ticks,
                                        uint64_t packet_upper_ticks,
                                        uint64_t previous_ticks,
                                        uint32_t period_ticks);
bool bno085_timing_due(uint64_t previous_ticks, uint64_t current_ticks,
                       uint32_t rate_hz);
uint64_t bno085_timing_advance(uint64_t previous_ticks,
                               uint64_t current_ticks, uint32_t rate_hz);
uint32_t bno085_timing_track_period(uint32_t previous_period_ticks,
                                    uint64_t hint_delta_ticks,
                                    uint32_t report_delta,
                                    uint32_t nominal_period_ticks);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_TIMING_H */
