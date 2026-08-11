# Robust raw-localization candidate

This checkpoint implements the agreed raw-first localization foundation.  It
does **not** enable a new live solver and it does not use IMU data to alter the
default UWB position.

## What is implemented

- A common bounded Levenberg-Marquardt / IRLS core for direct ranges and
  range differences.
- Per-frame Huber, Cauchy, or ordinary least-squares loss.  There is no
  temporal position averaging in this core.
- Dense generalized least squares through an optional precision matrix.  The
  robust row weights are applied symmetrically around that matrix, preserving
  Passive DS-TWR's correlated same-initiator timing model.
- Protocol adapters for Native DS-TWR direct ranges, CFO-corrected FlexTDOA
  differences, and coherent Passive DS-TWR difference batches.
- Diagnostics for raw/weighted residual RMS, largest residual, downweighted
  observation count, HDOP proxy, normal-matrix condition, and iterations.
- A cursor-based dashboard stream that archives each raw UWB range,
  range-difference observation, anchor range, and geometry update.  The
  capture tool reports a cursor overrun instead of silently treating an
  incomplete archive as lossless.

The current live solvers are unchanged.  In particular, Passive DS-TWR keeps
its existing timing-covariance path and coherent batch policy; the common core
is a candidate that can consume the same precision model once a replayable
held-out capture exists.

## Baseline held-out position results

| Protocol | RTK pairs | raw RMSE cm | raw P95 cm | raw max cm |
|---|---:|---:|---:|---:|
| FlexTDOA | 1144 | 18.85 | 32.19 | 80.19 |
| Native DS-TWR | 1843 | 19.18 | 29.67 | 278.00 |
| Passive DS-TWR | 996 | 17.13 | 38.83 | 67.91 |

These archives contain raw positions but not every radio observation forming
each position.  They are therefore valid as the immutable baseline, but they
cannot be re-solved by the new numerical core.

## Channel-9 calibration decision

The current RTK range capture is a four-link star around M1.  Its additive
per-device delay matrix has rank 4 for 5 unknown devices, so changing antenna
delays from this capture would be arbitrary.  Keep the independently validated
full-graph delays `16371, 16377, 16370, 16378, 16388` and the currently applied
per-link corrections.  The independent stationary check leaves link medians
between 2.4 mm and 19.4 mm, but does not prove that those biases are universal
during motion.

The measured height correction in this geometry is below 0.3 mm median, so it
cannot explain the 17--19 cm dynamic RMSE.

## Evidence gate

Status: **offline-ready; live activation and OTA blocked**.

The next controlled capture must use the new raw-observation stream and keep
calibration and validation routes separate.  A candidate may replace a live
solver only if held-out raw replay improves RMSE, P95, maximum outlier, and
continuity without hiding the legacy raw solution.  Until then the dashboard
must continue to display the present raw UWB result as authoritative.

## Verification

- Full ESP-IDF 6.0.2 build: passed.
- Firmware image: 1,349,600 bytes; 68% of the smallest application partition
  remains free.
- Python suite: 109 tests passed.
- Strict host C build (`-Wall -Wextra -Werror`) and solver tests: passed.
- Source diff whitespace check: passed.
