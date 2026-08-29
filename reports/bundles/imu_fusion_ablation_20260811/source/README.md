# IMU fusion ablation and raw UWB regression audit — 2026-08-11

The complete 13-page dynamic/static field report is
[`../../../imu_fusion_ablation_20260811.pdf`](../../../imu_fusion_ablation_20260811.pdf).
It includes the six-cell
protocol matrix, continuity, static UWB and GPS RTK clouds, dynamic RTK audit,
radio-loss diagnostics, raw/fused comparison, controlled IMU ablation,
limitations and reproduction artifacts. The notes below are the compact audit
summary.

## Verdict

The apparent Native DS-TWR improvement from **17.91 cm raw RMSE** to
**16.33 cm fused RMSE** is not evidence that the accelerometer improved
positioning. A controlled replay ablation on the same capture produced:

| Native configuration | Acceleration used | RMSE cm | P95 cm | Max cm |
|---|---:|---:|---:|---:|
| Raw UWB | — | 17.911 | 26.822 | 260.873 |
| Fused, acceleration disabled | 4 / 108,563 | 16.323 | 25.395 | 105.327 |
| Fused, current gates | 20,449 / 108,563 | 16.326 | 25.395 | 103.494 |
| Fused, relaxed gates | 97,763 / 108,563 | 16.310 | 25.650 | 94.720 |

The current-versus-disabled RMSE difference is **+0.0031 cm**. Relaxing the
gates changes RMSE by only **-0.0128 cm**. The measurable gain comes from the
common UWB position smoother, not from acceleration integration.

Passive DS-TWR is even clearer: acceleration-disabled, current and relaxed
replays all produce the same RMSE (**14.867 cm**) and P95 (**29.389 cm**) to
the displayed precision.

## Static regression control

A new 30-second stationary capture was collected with the deployed
`bb23b07-dirty` firmware, 500 Hz sensor acquisition and the full telemetry
path active:

| Metric | Result |
|---|---:|
| Independent raw positions | 832 |
| Raw UWB RMSE versus RTK | **2.710 cm** |
| Raw UWB P95 versus RTK | **4.080 cm** |
| Raw UWB maximum versus RTK | 4.993 cm |
| Raw local-cloud RMS | **1.711 cm** |
| Raw local-cloud P95 | 2.915 cm |
| Fused RMSE versus RTK | 2.568 cm |
| Anchor rigid-fit RMS | 0.422 cm |
| RTK rate | 8.08 Hz |
| Tag IMU rate used by replay | 458.9 Hz |

This is close to the prior calibrated stationary baseline of 2.55 cm RMSE.
Therefore the raw radio/solver path has not suffered a general stationary
accuracy regression from enabling the IMU.

## Why the dynamic result remains poor

The current dynamic errors of 15–18 cm already exist in the **raw UWB**
stream. They are not created by the fusion output. Several independent
problems were identified:

1. UWB position packets contain only a 32-bit `uptime_ms` assigned at
   telemetry submission. They do not contain the radio measurement time.
   Replay consequently scores them using TCP/dashboard `received_at`.
2. The RTK measurement wall-time estimate is approximately 108–112 ms late
   relative to GNSS UTC. Correcting this explains roughly 5–6 cm of dynamic
   RMSE for FlexTDOA and Native DS-TWR, but does not explain the whole error.
3. The Native walk contained 4,254 complete and 1,208 incomplete candidate
   frames: **22.12% incomplete**, plus 658 slot overruns.
4. The telemetry consumer scans a PSRAM ring of up to 1,024 entries under a
   critical section for every dequeue. It runs on the UWB core and can delay
   the higher-priority UWB task while interrupts are masked.
5. The global GPIO ISR service is installed by the UWB task on core 1 before
   BNO085 starts. The 500 Hz BNO085 interrupt handler can therefore execute on
   the UWB core and reads GPTimer plus a critical section at every interrupt.

These scheduling risks can increase missed deadlines and packet loss. They do
not alter a DS-TWR distance whose DW3000 timestamps were already captured.

## Why acceleration was mostly unavailable

| Protocol | IMU-propagated / valid | Dominant exclusive gates |
|---|---:|---|
| Native DS-TWR | 20,449 / 108,563 = 18.84% | yaw stale 69.30%; yaw absent 9.28% |
| FlexTDOA | 0 / 98,805 | yaw absent 39.20%; yaw stale 34.87%; bias missing 25.93% |
| Passive DS-TWR | 0 / 78,893 | yaw stale 56.92%; yaw absent 25.53%; bias missing 17.55% |

The current mapping also assumes that body `+X` follows the direction of
travel. That is not guaranteed for a handheld or thrown tag. The bias is
estimated after rotation into the BNO reference frame even though physical
accelerometer bias belongs to the body frame.

FlexTDOA has an additional replay epoch defect: an IMU timer backstep is
translated into an artificial `2^32 ms` span and then causes a reset. Its 0%
figure therefore cannot be attributed only to the fusion gates.

## Acceptance decision

The current vector-acceleration fusion is **rejected** for production accuracy
claims. Raw UWB remains the authoritative audit stream. The smoother may be
evaluated separately, but its gain must not be described as an IMU gain.

The next validation sequence is:

1. add a versioned 64-bit UWB measurement timestamp derived from radio RX
   host time and use it end-to-end;
2. restore constant-time telemetry dequeue and perform an A/B test with IMU
   off, sensor acquisition without IMU streaming, and full telemetry;
3. use yaw-free IMU only for stationary detection, ZUPT and adaptive process
   noise until a physical mount calibration exists;
4. keep vector acceleration in a shadow predictor and enable it only if it
   beats the constant-velocity predictor on held-out captures;
5. repeat identical dynamic routes and require lower RMSE and P95 without
   additional overshoot.

## Source artifacts

- `../raw/data/native_ds_dynamic_quality_gate_02.native_ds.jsonl.xz`
- `../raw/data/passive_ds_dynamic_final_01.passive_ds.jsonl.xz`
- `../raw/data/flextdoa_dynamic_final_01.flextdoa.jsonl.xz`
- `../raw/data/passive_ds_static_raw_regression_01.passive_ds.jsonl.xz`
- `../imu_fusion_final_validation_20260811/passive_ds_static_raw_regression_01.replay.json`
- `analysis_summary.json`
- `ablation_metrics.csv`
