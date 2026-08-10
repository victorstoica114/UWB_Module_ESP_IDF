# Channel 9 Native DS-TWR RTK calibration — 2026-08-10

## Result

The existing per-module channel 9 antenna delays were left unchanged:

| Module | Antenna delay (DTU) | Hex |
|---|---:|---:|
| M1 | 16371 | `0x3ff3` |
| M2 | 16377 | `0x3ff9` |
| M3 | 16370 | `0x3ff2` |
| M4 | 16378 | `0x3ffa` |
| M5 | 16388 | `0x4004` |

A single M1 antenna-delay correction could not explain the observed error: the
four uncorrected M1-to-anchor range errors had different signs and magnitudes.
The least-squares common correction was approximately +12 DTU, but it still
predicted 5.14 cm range RMS and an 8.71 cm maximum residual. Applying it would
therefore have hidden link-dependent bias without solving it.

Native DS-TWR was instead calibrated with these measured-minus-RTK per-link
range biases, in anchor order A2, A3, A4, A5:

```text
native_ds_range_bias_mm = 78,-31,104,72
```

The values are stored persistently in NVS on M1-M5 as calibration generation
4. The firmware subtracts them from the measured ranges before position
solving.

## Independent validation

The calibration fit and validation used separate 60-second stationary
captures. The validation contained 2,002 range records, 1,360 raw positions
and 203 GNSS records, with no dashboard polling errors.

| Metric | Before calibration | Independent validation |
|---|---:|---:|
| Raw position RMSE vs RTK | 11.31 cm | **2.55 cm** |
| Raw position vector bias | 11.01 cm | **0.75 cm** |
| Raw position P95 vs RTK | 14.14 cm | **4.49 cm** |
| Raw static cloud RMS | 2.59 cm | **2.44 cm** |
| Fused position RMSE vs RTK | not evaluated | **1.46 cm** |
| Fused position P95 vs RTK | not evaluated | **1.98 cm** |

The validation's remaining mean range errors were +0.89 mm, +1.56 mm,
-1.11 mm and -0.89 mm for A2-A5 respectively. The 60-second IMU replay
accepted all 1,353 UWB positions, reported no outliers or filter resets, and
used 5,995 timestamp-aligned IMU samples.

Native DS-TWR radio continuity during the end-to-end capture was 1,327 complete
and 33 incomplete frames, or 2.43% incomplete. There were 21 recovered PHY
errors and no permanent loss of runtime connectivity.

## Scope and remaining checks

This is a strong independent confirmation for the current stationary tag
position and current anchor geometry. Per-link bias may still contain
position-, orientation- or propagation-dependent components. It must be
validated at several locations and during a walk before being treated as a
universal calibration. The raw and fused streams must remain separate in that
test.

M2 remained online with working Native DS-TWR, RTK Fixed GNSS and charger
telemetry after restart. Its BNO085 service still did not start and produced
zero IMU reports, so M2's separate I2C/device issue was not fixed by the
restart. It did not invalidate this test because M1 was the tag and provided
the IMU stream used by fusion.

## Files

- `native_ds_rtk_delay_dryrun.jsonl`: calibration fit capture.
- `native_ds_rtk_delay_analysis.json`: pre-calibration and offline fit metrics.
- `native_ds_rtk_delay_validation.jsonl`: independent raw validation capture.
- `native_ds_rtk_delay_validation_analysis.json`: independent RTK analysis.
- `native_ds_static_calibrated_validation.native_ds.jsonl`: end-to-end UWB,
  RTK and IMU capture.
- `native_ds_static_calibrated_validation.native_ds.replay.json`: deterministic
  fusion replay summary.
- `native_ds_static_calibrated_validation.native_ds.replay_samples.jsonl`:
  replayed raw/fused position samples.
