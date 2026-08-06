#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "bno085_timing.h"

int main(void)
{
    const uint8_t signed_delta[] = {0xF6, 0xFF, 0xFF, 0xFF};
    assert(bno085_timing_read_le_i32(signed_delta) == -10);

    const uint8_t report[] = {0x01, 0x00, 0xA9, 0x34};
    assert(bno085_timing_report_delay_100us(report) == 0x2A34);

    /* SH-2: basis=HINT-base delta; then add rebase and report delay. */
    assert(bno085_timing_reconstruct_ticks(1000000, 10, 5, 20) ==
           1015000);
    assert(bno085_timing_reconstruct_ticks(1000, 10, 0, 0) == 0);

    assert(bno085_timing_due(0, 1, 100));
    assert(!bno085_timing_due(100000, 150000, 100));
    assert(bno085_timing_due(100000, 200000, 100));
    assert(!bno085_timing_due(100000, 200000, 0));

    puts("bno085 timing tests passed");
    return 0;
}
