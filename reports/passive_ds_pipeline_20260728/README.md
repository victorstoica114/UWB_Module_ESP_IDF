# Passive DS-TWR pipeline field check — 2026-07-28

## Setup

- Firmware: `7226cca`
- Five modules, flashed once with one simultaneous OTA operation
- Passive DS-TWR, Robust Rotating, 1.0 ms RESP / 1.0 ms FINAL
- Physical reference: 3 m × 3 m anchor square, Tag 1 at `(1.5, 1.5)` m
- Dynamic anchor self-localization remained enabled
- Four 60 s captures, each preceded by a 10 s warm-up
- Position errors below use the dashboard EKF output; raw-solver results are
  shown separately to detect apparent gains caused only by filtering

## Position and throughput

| Capture | Independent frames/s | Rolling updates/s | Total solver updates/s | Filtered RMSE / P95 | Raw RMSE / P95 |
|---|---:|---:|---:|---:|---:|
| Legacy + Frame (A1) | 62.54 | — | 62.54 | 3.59 / 5.51 cm | 3.87 / 6.17 cm |
| Deadline + Frame (B) | 84.12 | — | 84.12 | 3.23 / 5.09 cm | 3.59 / 5.92 cm |
| Deadline + Rolling 100 Hz (C) | 84.38 | 84.76 | 169.14 | 3.01 / 4.72 cm* | 3.28 / 5.32 cm* |
| Legacy + Frame return (A2) | 63.24 | — | 63.24 | 5.12 / 7.29 cm | 5.35 / 8.04 cm |

`*` Accuracy for Deadline + Rolling is calculated only from its independent
frame solutions. The additional rolling solutions are correlated intermediate
updates and must not be counted as independent accuracy samples.

Deadline scheduling increased independent frame throughput by 34.5% over A1.
Enabling rolling solves did not reduce independent frame throughput and
approximately doubled the display update rate.

## Acceptance gates

The moving-block bootstrap validator uses the rule that the candidate RMSE and
P95 must not exceed the upper 95% confidence bound of the same-geometry
control.

- Deadline + Frame versus Legacy A1: accepted.
  - RMSE: 3.23 cm versus 3.59 cm control; control upper bound 3.71 cm.
  - P95: 5.09 cm versus 5.51 cm control; control upper bound 5.76 cm.
- Deadline + Rolling independent frames versus Deadline + Frame: accepted.
  - RMSE: 3.01 cm versus 3.23 cm control; control upper bound 3.36 cm.
  - P95: 4.72 cm versus 5.09 cm control; control upper bound 5.30 cm.

These results demonstrate no observed accuracy regression. They do not yet
prove that deadline scheduling improves accuracy because the captures were
sequential and the live geometry estimator continued to evolve.

## Pipeline diagnostics

Totals accumulated during each 60 s measurement interval:

| Mode | Completed exchanges | RESP timeouts | FINAL timeouts | State collisions | Schedule overruns | Invalid frames | RX re-arm failures |
|---|---:|---:|---:|---:|---:|---:|---:|
| Legacy + Frame A1 | 23,791 | 1,914 | 619 | 0 | 0 | 0 | 0 |
| Deadline + Frame | 32,128 | 408 | 277 | 672 | 39 | 0 | 0 |
| Deadline + Rolling | 32,585 | 305 | 275 | 620 | 41 | 0 | 0 |

Compared with Legacy A1, Deadline + Frame completed about 35% more exchanges
and reduced the combined RESP/FINAL timeout count by about 73%. The deadline
timer alarm count is a normal wake-up counter, not a failure counter.

The deadline runs reported no invalid frames and no RX re-arm failures. CIA
readout averaged roughly 25–42 us across modules and RX re-arm roughly
38–46 us. The rolling solver left radio/frame throughput effectively
unchanged, which is the most direct check that the added solves did not slow
the ranging pipeline.

## Geometry observations

Deadline + Frame kept the standard deviation of every raw anchor-pair range at
or below 1.78 cm in this capture. Both Legacy captures contained intermittent
bursts on some paths; in return control A2 the standard deviations reached
9.27 cm on A2–A3, 11.81 cm on A2–A5, and 7.62 cm on A4–A5. This explains the
worse A2 position result and is evidence that the legacy millisecond/blocking
schedule can still disturb the live geometry estimate.

The remaining mean anchor-pair biases are several centimetres and are a
separate calibration issue. They should not be attributed to the deadline
pipeline.

## Current runtime after the test

All five modules were left online in `Deadline + Rolling`, with the rolling cap
at 100 Hz and the 1.0/1.0 ms Robust Rotating timing unchanged.

