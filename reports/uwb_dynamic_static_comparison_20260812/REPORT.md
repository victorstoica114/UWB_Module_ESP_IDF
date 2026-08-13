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
| flextdoa_final_dynamic_20260812 | FlexTDOA | dynamic | 84.6 | 2117 | 25.01 | 25.07 | 25.01 | 99.7 | 70.0 | 830.0 |
| native_ds_final_dynamic_20260812 | Native DS-TWR | dynamic | 113.5 | 2876 | 25.35 | 25.66 | 25.35 | 98.7 | 50.0 | 130.0 |
| passive_ds_final_dynamic_20260812 | Passive DS-TWR | dynamic | 106.5 | 7463 | 70.11 | 70.11 | 22.63 | 100.0 | 30.0 | 250.0 |
| flextdoa_static_final_01 | FlexTDOA | static | 30.3 | 798 | 26.30 | 26.61 | 26.30 | 98.8 | 70.0 | 120.0 |
| native_ds_static_final_02_rtk_fixed | Native DS-TWR | static | 30.1 | 780 | 25.89 | 25.95 | 25.89 | 99.6 | 50.0 | 80.0 |
| passive_ds_static_final_01 | Passive DS-TWR | static | 31.2 | 1906 | 61.06 | 63.44 | 19.45 | 96.2 | 40.0 | 130.0 |

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
| FlexTDOA | 798 | 0.019 | 0.024 | 0.043 | 0.048 | 0.008 |
| Native DS-TWR | 780 | 0.017 | 0.024 | 0.039 | 0.048 | 0.007 |
| Passive DS-TWR | 607 | 0.019 | 0.023 | 0.039 | 0.045 | 0.004 |

![Static precision](figures/03_static_precision.svg)

## Position-only adaptive EKF

The EKF consumes only UWB positions; IMU acceleration is disabled. It is
evaluated against the raw independent stream carried by the same records.
Lower step-speed P95 means less sample-to-sample jitter, while RTK columns are
only auditable where anchor registration is valid.

| protocol | motion | EKF coverage % | raw step P95 m/s | EKF step P95 m/s | raw static RMS cm | EKF static RMS cm | raw RTK RMSE m | EKF RTK RMSE m |
|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | 100.0 | 6.04 | 4.31 | n/a | n/a | 1.610 | 1.609 |
| Native DS-TWR | dynamic | 100.0 | 7.15 | 3.44 | n/a | n/a | 1.694 | 1.710 |
| Passive DS-TWR | dynamic | 100.0 | 4.66 | 4.50 | n/a | n/a | 1.397 | 1.395 |
| FlexTDOA | static | 100.0 | 1.97 | 0.36 | 2.41 | 1.07 | 0.599 | 0.599 |
| Native DS-TWR | static | 100.0 | 1.61 | 0.23 | 2.45 | 0.89 | n/a | n/a |
| Passive DS-TWR | static | 100.0 | 1.62 | 0.00 | 2.26 | 0.00 | n/a | n/a |

![Dynamic raw versus EKF](figures/09_dynamic_raw_vs_ekf.pdf)

![Static raw versus EKF](figures/10_static_raw_vs_ekf.pdf)

## Static RTK cross-capture consistency gate

The tag was stationary, so its RTK center must agree between protocol blocks
before RTK can support an absolute ranking. The observed maximum pairwise
center separation is **2.127
m**, against a 0.250 m gate. Verdict:
**static absolute UWB-RTK errors are audit-only and invalid for cross-protocol ranking**.

This gate is independent of the per-capture anchor fit. A small anchor-fit RMS
can coexist with a shifted tag reference and cannot rescue static ranking.

## UWB versus RTK disagreement

Only RTK-fixed tag solutions are used. Each solution is matched to the nearest
independent UWB position by `estimated_measurement_wall_ns` versus the UWB
`received_at` timestamp, with an absolute limit of 100 ms.
The local UWB frame is mapped to RTK ENU with rotation and translation only;
scale is never fitted. `Debiased RMSE` removes the median tag residual after
anchor registration and is a shape/repeatability diagnostic, not accuracy.

| protocol | motion | pairs | time P95 ms | RMSE m | P95 m | debiased RMSE m | anchor-fit P95 RMSE m | alignment flag | RTK interpretation |
|---|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | 656 | 34.9 | 1.610 | 1.828 | 0.376 | 0.859 | poor / no accuracy claim | indicative_in_system_reference |
| Native DS-TWR | dynamic | 633 | 18.5 | 1.694 | 2.391 | 1.202 | 0.675 | poor / no accuracy claim | indicative_in_system_reference |
| Passive DS-TWR | dynamic | 726 | 45.8 | 1.397 | 1.555 | 0.486 | 0.013 | usable with limits | indicative_in_system_reference |
| FlexTDOA | static | 137 | 33.2 | 0.599 | 0.629 | 0.025 | 0.930 | poor / no accuracy claim | audit_only_invalid_for_cross_protocol_ranking |
| Native DS-TWR | static | 0 | n/a | n/a | n/a | n/a | n/a | unavailable | audit_only_invalid_for_cross_protocol_ranking |
| Passive DS-TWR | static | 0 | n/a | n/a | n/a | n/a | n/a | unavailable | audit_only_invalid_for_cross_protocol_ranking |

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
- `replay_metrics.csv`: replay schema export; empty for this captured-stream
  comparison because the EKF counterpart is embedded in every raw record;
- `data/*.jsonl.xz`: the six complete RAW captures, compressed losslessly;
- `raw_data_manifest.csv` and `SHA256SUMS`: source/archive integrity;
- `figures/`: dependency-free SVG and PDF plots.

## Reproduce

```powershell
python tools/uwb_dynamic_static_report.py `
  --input-dir D:/Documente/UWB_ESP_IDF/reports/uwb_final_report_input_20260812 `
  --output-dir D:/Documente/UWB_ESP_IDF/reports/uwb_dynamic_static_comparison_20260812 `
  --replay-dynamic-dir D:/Documente/UWB_ESP_IDF/reports/uwb_final_report_input_20260812 `
  --replay-static-dir D:/Documente/UWB_ESP_IDF/reports/uwb_final_report_input_20260812

```

Restore an archived capture without an `xz` executable:

```powershell
python -c "import lzma,shutil; shutil.copyfileobj(lzma.open(r'data/CAPTURE.jsonl.xz','rb'), open(r'CAPTURE.jsonl','wb'))"
```
