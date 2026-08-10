# Dynamic/static UWB comparison

Generated from immutable JSONL captures. This report compares published
position streams; it does not modify or invoke replay/dashboard code.

> **Interpretation warning:** RTK is an in-system reference, not independently
> surveyed ground truth. UWB coordinates are rigidly registered to robust
> medians of RTK-fixed anchor positions. The anchor-registration residual,
> GNSS fixed availability, sparse time association, antenna lever arms and any
> RTK-derived dynamic geometry limit every absolute-error statement below.

## Capture matrix

| capture | protocol | motion | duration s | position n | capture Hz | active Hz | independent Hz | coverage % | gap P95 ms | gap max ms |
|---|---|---|---|---|---|---|---|---|---|---|
| walk_flex_final | FlexTDOA | dynamic | 91.4 | 2312 | 25.31 | 25.72 | 25.31 | 98.3 | 80.0 | 520.0 |
| walk_native_final | Native DS-TWR | dynamic | 91.1 | 1981 | 21.74 | 21.86 | 21.74 | 99.4 | 60.0 | 200.0 |
| walk_passive_final | Passive DS-TWR | dynamic | 95.7 | 4664 | 48.73 | 49.35 | 16.01 | 98.7 | 70.0 | 1610.0 |
| rpi_static_flextdoa_rtkfixed_180s | FlexTDOA | static | 180.1 | 5138 | 28.54 | 28.54 | 28.54 | 100.0 | 70.0 | 170.0 |
| rpi_static_native_rtkfixed_180s | Native DS-TWR | static | 180.0 | 4040 | 22.44 | 22.44 | 22.44 | 100.0 | 60.0 | 100.0 |
| rpi_static_passive_rtkfixed_180s | Passive DS-TWR | static | 180.1 | 12557 | 69.74 | 69.83 | 23.24 | 99.9 | 30.0 | 460.0 |

Missing cells:

- None.

Passive DS publishes overlapping raw windows. Comparative position metrics use
only records marked `independent_frame`; the all-event rate remains visible as
an operational telemetry rate. Dashboard fusion records (`imu_fused=true` or
an `*_imu_fused_position` stream) are excluded from every raw/independent
capture metric and counted separately in `capture_metrics.csv`. `Capture Hz`
uses the full capture wall duration; `active Hz` uses the stream's own uptime
span. Coverage and end-lag fields expose captures whose telemetry stops early.

![Independent position rate](figures/01_position_rate.svg)

![Position gap](figures/02_position_gap.svg)

## Static precision

Static precision is radial displacement around each capture's own local-frame
median. It is **precision, not absolute accuracy**, and uses independent-frame
samples without deleting outliers.

| protocol | n | CEP50 m | RMS m | P95 m | 2DRMS m | first-last drift m |
|---|---|---|---|---|---|---|
| FlexTDOA | 5138 | 0.020 | 0.026 | 0.043 | 0.052 | 0.001 |
| Native DS-TWR | 4040 | 0.019 | 0.049 | 0.145 | 0.097 | 0.004 |
| Passive DS-TWR | 4185 | 0.027 | 0.040 | 0.082 | 0.079 | 0.065 |

![Static precision](figures/03_static_precision.svg)

## Static RTK cross-capture consistency gate

The tag was stationary, so its RTK center must agree between protocol blocks
before RTK can support an absolute ranking. The observed maximum pairwise
center separation is **1.583
m**, against a 0.250 m gate. Verdict:
**static absolute UWB-RTK errors are audit-only and invalid for cross-protocol ranking**.

This gate is independent of the per-capture anchor fit. A small anchor-fit RMS
can coexist with a shifted tag reference and cannot rescue static ranking.

## Replay continuity and fusion

The six replay summaries are loaded from the dedicated dynamic/static replay
directories. Position/IMU rates, gaps, accepted/outlier counts and reset counts
are direct event-stream diagnostics and are the most reliable comparison in
this campaign. Replay RTK RMSE is shown separately and remains indicative: its
many matched output samples do not create more independent RTK fixes, and an
anchor-fit RMS above 0.5 m is flagged as a large alignment uncertainty.

| protocol | motion | pos Hz | gap P95 ms | gap max ms | accepted % | resets | raw RTK RMSE m | fused RTK RMSE m | anchor fit RMS m | RTK grade |
|---|---|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | 25.72 | 80.0 | 520.0 | 98.6 | 2 | 1.095 | 1.097 | 0.764 | indicative_only_large_anchor_fit |
| Native DS-TWR | dynamic | 21.86 | 60.0 | 200.0 | 98.7 | 1 | 2.168 | 2.176 | 0.400 | indicative_in_system_reference |
| Passive DS-TWR | dynamic | 49.35 | 70.0 | 1610.0 | 88.0 | 27 | 3.436 | 3.437 | 0.003 | indicative_in_system_reference |
| FlexTDOA | static | 28.54 | 70.0 | 170.0 | 98.9 | 0 | 0.111 | 0.109 | 0.073 | audit_only_invalid_for_cross_protocol_ranking |
| Native DS-TWR | static | 22.44 | 60.0 | 100.0 | 98.6 | 0 | 0.535 | 0.534 | 0.684 | audit_only_invalid_for_cross_protocol_ranking |
| Passive DS-TWR | static | 69.83 | 30.0 | 460.0 | 87.6 | 0 | 1.490 | 1.490 | 0.004 | audit_only_invalid_for_cross_protocol_ranking |

## UWB versus RTK disagreement

Only RTK-fixed tag solutions are used. Each solution is matched to the nearest
independent UWB position by `estimated_measurement_wall_ns` versus the UWB
`received_at` timestamp, with an absolute limit of 100 ms.
The local UWB frame is mapped to RTK ENU with rotation and translation only;
scale is never fitted. `Debiased RMSE` removes the median tag residual after
anchor registration and is a shape/repeatability diagnostic, not accuracy.

| protocol | motion | pairs | time P95 ms | RMSE m | P95 m | debiased RMSE m | anchor-fit P95 RMSE m | alignment flag | RTK interpretation |
|---|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | 23 | 36.3 | 1.158 | 2.135 | 0.989 | 0.763 | poor / no accuracy claim | indicative_in_system_reference |
| Native DS-TWR | dynamic | 27 | 27.2 | 2.164 | 3.433 | 1.263 | 0.390 | poor / no accuracy claim | indicative_in_system_reference |
| Passive DS-TWR | dynamic | 19 | 63.6 | 3.298 | 4.756 | 1.903 | 0.268 | usable with limits | indicative_in_system_reference |
| FlexTDOA | static | 119 | 31.6 | 0.087 | 0.115 | 0.026 | 0.006 | usable with limits | audit_only_invalid_for_cross_protocol_ranking |
| Native DS-TWR | static | 119 | 28.9 | 0.092 | 0.122 | 0.055 | 0.110 | poor / no accuracy claim | audit_only_invalid_for_cross_protocol_ranking |
| Passive DS-TWR | static | 101 | 63.3 | 1.471 | 1.539 | 0.048 | 0.091 | usable with limits | audit_only_invalid_for_cross_protocol_ranking |

![Dynamic RTK error CDF](figures/04_dynamic_rtk_error_cdf.svg)

![Alignment quality](figures/05_alignment_quality.svg)

## Method and limitations

- GPS coordinates are converted from WGS84 ECEF to a common local ENU frame.
- Interleaved dashboard fusion positions are excluded before all raw position
  metrics; raw events may still carry `imu_fused_*` diagnostic fields.
- Anchor centers are per-capture component-wise medians of RTK-fixed samples.
- A 2-D proper rigid transform is fitted from the local anchor geometry to the
  RTK centers. At least three fixed anchors are required.
- Time-varying Passive DS geometries use the nearest captured geometry
  snapshot, preferring an exact `geometry_version`. A non-exact dynamic
  geometry older than 10.0 s is rejected.
- A Passive DS geometry marked dynamic is itself derived from GNSS. Its frame
  alignment is therefore not independent of the RTK reference; the report
  marks this circularity explicitly.
- RTK `fixed` status does not guarantee centimetre-level truth. Anchor P95
  spread and rigid-fit residual are exported, and poor cases are flagged.
- `estimated_measurement_wall_ns` subtracts the receiver-reported fix age from
  collector wall time. It is not hardware timestamp synchronization. Pair
  count, coverage and association error must accompany RMSE/P95.
- The tag and anchor GNSS antennas need not be collocated with their UWB
  antennas; unknown lever arms appear as bias.
- Static and dynamic blocks are not repeated randomized trials. Differences
  may include path, orientation, RF environment and geometry-state changes.
- Capture-boundary samples outside the common RTK/UWB interval are not paired.

## Audit artifacts

- `analysis_summary.json`: nested machine-readable results and policies;
- `capture_metrics.csv`: one flattened row per capture;
- `rtk_alignment_metrics.csv`: registration and RTK-pair metrics;
- `rtk_pairs.csv`: every accepted time association and residual;
- `alignment_snapshots.csv`: every geometry-to-RTK rigid fit;
- `replay_metrics.csv`: continuity, fusion and replay RTK diagnostics;
- `data/*.jsonl.xz`: the six complete RAW captures, compressed losslessly;
- `raw_data_manifest.csv` and `SHA256SUMS`: source/archive integrity;
- `figures/`: dependency-free SVG plots.

## Reproduce

```powershell
python tools/uwb_dynamic_static_report.py `
  --input-dir D:/Documente/UWB_ESP_IDF/.worktrees/passive-ds-rx-recovery/reports/uwb_dynamic_capture `
  --output-dir D:/Documente/UWB_ESP_IDF/.worktrees/passive-ds-rx-recovery/reports/uwb_dynamic_static_comparison_20260806 `
  --replay-dynamic-dir D:/Documente/UWB_ESP_IDF/.worktrees/passive-ds-rx-recovery/reports/imu_dynamic_20260806 `
  --replay-static-dir D:/Documente/UWB_ESP_IDF/.worktrees/passive-ds-rx-recovery/reports/imu_static_20260806 `
  --require-rpi-static
```

Restore an archived capture without an `xz` executable:

```powershell
python -c "import lzma,shutil; shutil.copyfileobj(lzma.open(r'data/CAPTURE.jsonl.xz','rb'), open(r'CAPTURE.jsonl','wb'))"
```
