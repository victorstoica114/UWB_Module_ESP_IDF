# Native DS-TWR speed tuning — channel 9 RTK validation

- Date: 2026-08-05
- Branch: `ds-twr-speed-tuning`
- Position filtering: none
- Fixed geometry: GPS RTK ENU, A2 origin

## Selected profile

| Parameter | Value |
|---|---:|
| Per-anchor slot | 11 ms |
| Four-anchor frame gap | 2 ms |
| Nominal frame | 46 ms |
| RX timeout | 5 ms |
| RESPONSE delay | 2 ms |
| FINAL delay | 2 ms |
| RESULT delay | 2 ms |
| Auto-RX delay | 500 UUS |
| Channel | 9 |
| Range calibration A2..A5 | `[-13, +92, +39, -17]` mm |

The slot timing is enforced with a one-shot `esp_timer` and a blocking
semaphore. It does not busy-wait and does not change the global FreeRTOS tick
rate.

## Results

| Profile / implementation | End-to-end positions/s | 2D RMSE | P95 | Precision RMS | Solver residual mean |
|---|---:|---:|---:|---:|---:|
| 410 ms conservative baseline | 2.36 | **1.17 cm** | **1.98 cm** | 1.13 cm | 0.81 cm |
| 64 ms, critical-path diagnostics removed | sampled 9.08 | 1.45 cm | 2.39 cm | 1.35 cm | 3.35 cm |
| 46 ms, old tick pacing | sampled 7.73 | 1.32 cm | 2.32 cm | 1.32 cm | 3.40 cm |
| 46 ms, precise pacing, calibration generation 2 | firmware ~22 | 1.33 cm | 2.16 cm | 1.32 cm | 3.32 cm |
| **46 ms, precise pacing, generation 3, cursor capture** | **19.16** | **1.76 cm** | **2.85 cm** | **1.32 cm** | **0.80 cm** |

The selected profile is about 8.1 times faster end-to-end than the conservative
baseline. Its repeatability is almost unchanged; the difference in position
RMSE is dominated by a 1.17 cm capture bias relative to the live RTK frame.
All metrics are raw independent-frame solutions without temporal filtering.

During the final 90 s run, the firmware pipeline reported:

| Module | Initiated/received POLL | Complete RESULT | RX timeout | Delayed TX | Slot overrun |
|---|---:|---:|---:|---:|---:|
| M1 tag | 7815 | 7695 | 99 | 0 | 0 |
| M2 | 1950 | 1944 | 3 | 0 | 0 |
| M3 | 1890 | 1882 | 2 | 0 | 0 |
| M4 | 1899 | 1891 | 4 | 0 | 0 |
| M5 | 1898 | 1897 | 1 | 0 | 0 |

The tag completed 98.46% of initiated anchor exchanges. The slowest anchor
completed about 20.9 coherent ranges/s. The dashboard/collector retained 1724
positions in 90 s after the cursor fix, or 19.16 positions/s.

## Final OTA validation

The final ESP-IDF image was built locally and deployed to M1--M5:

- SHA-256: `43a0dc29ca7f4611002b58d6016d16612ee610dd2f606f4711a456b1f10781ec`;
- all five OTA uploads completed successfully;
- all five boot guards validated the new partition;
- every module reported channel 9, slot 11 ms, gap 2 ms, RX timeout 5 ms,
  RESPONSE/FINAL delay 2 ms and calibration generation 3;
- a clean 30 s post-boot window completed 2410 tag exchanges, with zero
  delayed-TX errors, invalid frames or slot overruns. Per-anchor completions
  were 605, 601, 602 and 601.

## Bottlenecks found

### RX diagnostics changed the protocol timing

The Native DS diagnostic trigger still decoded bytes 8–9 as a legacy sequence
number. In the clean `NDS4` header those bytes contain destination/reserved
data. A destination of M5 therefore looked like sequence 5 and triggered full
CIA register reads plus an `ESP_LOGI` on every M5 POLL. The delayed response was
then armed 2–3 ms after its deadline.

Native DS no longer reads or logs full CIA diagnostics on the RX critical path.
Range timestamps, pipeline counters, raw/corrected distances and the post-range
diagnostic remain available. Dedicated distance-test modes retain RF quality
diagnostics.

### Millisecond pacing was quantized to 10 ms

The firmware uses `CONFIG_FREERTOS_HZ=100`. The old delay helper converted every
remaining 1–9 ms interval to one 10 ms tick, once per anchor and once per frame.
Consequently the 46 ms and 64 ms profiles both ran at roughly 13 frames/s.

Native DS pacing now uses a high-resolution one-shot timer and blocks on a
semaphore. The 46 ms profile consequently produces approximately 21 firmware
frames/s without increasing CPU load through busy waiting.

### The comparison collector sampled only the newest Native DS position

Native DS was incorrectly excluded from the dashboard's cursor-based position
event endpoint. At high rate, repeated snapshots could retain only the latest
position and made the measured output rate appear to be 5–9 positions/s. The
collector now consumes `native_ds_position` events by cursor. A 30 s check
captured 568 positions, and the final 90 s run captured 1724.

## Artifacts

- `frame64_summary.json`: first stable accelerated profile;
- `frame46_summary.json`: 46 ms with coarse FreeRTOS tick pacing;
- `frame46_precise_timer_summary.json`: precise timer, calibration generation 2;
- `frame46_cal3_final_summary.json`: selected deployed profile;
- `frame46_cross_validated_calibration_summary.json`: calibration learned from
  the independent 64 ms capture and evaluated on the 46 ms capture;
- `data/*.jsonl` and matching `*.metadata.json`: raw events and pipeline status.
