# Dynamic/static UWB comparison

## Evidence boundary

- All UWB position, rate, gap, static-precision and EKF results come exclusively
  from the final August 12 captures.
- All GPS RTK availability and cloud results come exclusively from the August 6
  reference captures.
- No August 6 UWB position enters this report, and no August 12 GPS sample enters
  the RTK section.
- The sessions are not synchronized; cross-session UWB–RTK RMSE/P95 is therefore
  intentionally not calculated.

## August 12 UWB capture matrix

| protocol | mode | capture | duration s | events | independent | independent Hz | gap P95 ms | gap max ms |
|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | flextdoa_final_dynamic_20260812 | 84.6 | 2117 | 2117 | 25.01 | 70 | 830 |
| Native DS-TWR | dynamic | native_ds_final_dynamic_20260812 | 113.5 | 2876 | 2876 | 25.35 | 50 | 130 |
| Passive DS-TWR | dynamic | passive_ds_final_dynamic_20260812 | 106.5 | 7463 | 2409 | 22.63 | 30 | 250 |
| FlexTDOA | static | flextdoa_static_final_01 | 30.3 | 798 | 798 | 26.30 | 70 | 120 |
| Native DS-TWR | static | native_ds_static_final_02_rtk_fixed | 30.1 | 780 | 780 | 25.89 | 50 | 80 |
| Passive DS-TWR | static | passive_ds_static_final_01 | 31.2 | 1906 | 607 | 19.45 | 40 | 130 |

![August 12 raw UWB dynamic trajectories](figures/06_dynamic_trajectories.pdf)

## August 12 static UWB precision

| protocol | n | CEP50 cm | RMS cm | P95 cm | max cm |
|---|---|---|---|---|---|
| FlexTDOA | 798 | 1.91 | 2.41 | 4.30 | 7.38 |
| Native DS-TWR | 780 | 1.72 | 2.45 | 3.95 | 12.30 |
| Passive DS-TWR | 607 | 1.88 | 2.26 | 3.94 | 5.52 |

![August 12 static raw UWB clouds](figures/07_static_position_clouds.pdf)

## Position-only adaptive EKF

| protocol | mode | coverage % | raw step P95 m/s | EKF step P95 m/s | raw static RMS cm | EKF static RMS cm |
|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | 100.0 | 6.04 | 4.31 | n/a | n/a |
| Native DS-TWR | dynamic | 100.0 | 7.15 | 3.44 | n/a | n/a |
| Passive DS-TWR | dynamic | 100.0 | 4.66 | 4.50 | n/a | n/a |
| FlexTDOA | static | 100.0 | 1.97 | 0.36 | 2.41 | 1.07 |
| Native DS-TWR | static | 100.0 | 1.61 | 0.23 | 2.45 | 0.89 |
| Passive DS-TWR | static | 100.0 | 1.62 | 0.00 | 2.26 | 0.00 |

![August 12 dynamic raw versus EKF](figures/09_dynamic_raw_vs_ekf.pdf)

![August 12 static raw versus EKF](figures/10_static_raw_vs_ekf.pdf)

## August 6 GPS RTK reference

This is a GPS-only reference population. It characterizes RTK fix availability
and short-term scatter for the unchanged receiver path, but is not paired with
the August 12 UWB positions.

| protocol | mode | capture | tag total | tag fixed | tag fixed % | all modules fixed % | fix-age P95 ms | tag cloud RMS cm | tag cloud P95 cm |
|---|---|---|---|---|---|---|---|---|---|
| FlexTDOA | dynamic | walk_flex_final | 24 | 24 | 100.0 | 93.0 | 96 | 284.65 | 482.34 |
| Native DS-TWR | dynamic | walk_native_final | 30 | 28 | 93.3 | 55.9 | 96 | 334.51 | 487.56 |
| Passive DS-TWR | dynamic | walk_passive_final | 26 | 25 | 96.2 | 72.2 | 106 | 367.91 | 625.14 |
| FlexTDOA | static | rpi_static_flextdoa_rtkfixed_180s | 119 | 119 | 100.0 | 91.4 | 100 | 0.52 | 0.85 |
| Native DS-TWR | static | rpi_static_native_rtkfixed_180s | 119 | 119 | 100.0 | 86.3 | 100 | 0.74 | 1.15 |
| Passive DS-TWR | static | rpi_static_passive_rtkfixed_180s | 119 | 102 | 85.7 | 93.0 | 130 | 1.40 | 2.02 |

![August 6 GPS RTK Fixed dynamic reference](figures/11_dynamic_gps_rtk_reference.pdf)

![August 6 GPS RTK Fixed static clouds](figures/08_static_gps_rtk_clouds.pdf)

## Reproduction

```powershell
python tools/uwb_dynamic_static_report.py `
  --input-dir reports/uwb_final_report_input_20260812 `
  --output-dir reports/uwb_dynamic_static_comparison_20260812 `
  --rtk-reference-report-dir reports/uwb_dynamic_static_comparison_20260806
```

The August 12 raw UWB captures remain archived losslessly under `data/`. The RTK
reference manifest points to the already-versioned August 6 lossless archives;
the older UWB records in those archives are never loaded by the GPS-only reader.
