#include "bno085_timing.h"

#include <limits.h>
#include <stddef.h>

int32_t bno085_timing_read_le_i32(const uint8_t *data)
{
    if (data == NULL) {
        return 0;
    }
    const uint32_t value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                           ((uint32_t)data[2] << 16) |
                           ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

uint16_t bno085_timing_report_delay_100us(const uint8_t *report)
{
    if (report == NULL) {
        return 0;
    }
    return (uint16_t)(((uint16_t)(report[2] & 0xFCU) << 6) | report[3]);
}

uint64_t bno085_timing_reconstruct_ticks(uint64_t hint_ticks,
                                         int32_t base_delta_100us,
                                         int32_t rebase_delta_100us,
                                         uint16_t report_delay_100us)
{
    const int64_t timer_ticks_per_sh2_tick =
        (int64_t)BNO085_FUSION_TIMER_HZ * BNO085_SH2_TICK_US / 1000000LL;
    const int64_t relative_100us = -(int64_t)base_delta_100us +
                                   (int64_t)rebase_delta_100us +
                                   (int64_t)report_delay_100us;
    const int64_t offset_ticks = relative_100us * timer_ticks_per_sh2_tick;

    if (offset_ticks < 0 && (uint64_t)(-offset_ticks) > hint_ticks) {
        return 0;
    }
    if (offset_ticks > 0 && hint_ticks > UINT64_MAX - (uint64_t)offset_ticks) {
        return UINT64_MAX;
    }
    return offset_ticks < 0 ? hint_ticks - (uint64_t)(-offset_ticks)
                            : hint_ticks + (uint64_t)offset_ticks;
}

bool bno085_timing_due(uint64_t previous_ticks, uint64_t current_ticks,
                       uint32_t rate_hz)
{
    if (rate_hz == 0 || rate_hz > BNO085_FUSION_TIMER_HZ) {
        return false;
    }
    if (previous_ticks == 0 || current_ticks < previous_ticks) {
        return true;
    }
    const uint64_t period_ticks = BNO085_FUSION_TIMER_HZ / rate_hz;
    return current_ticks - previous_ticks >= period_ticks;
}
