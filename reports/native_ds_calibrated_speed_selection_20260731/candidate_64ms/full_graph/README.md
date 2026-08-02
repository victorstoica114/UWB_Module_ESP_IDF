# Full-graph Native DS-TWR antenna-delay calibration

This is a dry-run calibration. No antenna delay was written to a module.

- Geometry: 3.000 m square; T1 static at center; calibrated Native DS-TWR 64 ms position frame reserve; slot 15 ms, gap 4 ms, timeout 8 ms, RESP/FINAL 2+2 ms; full-graph antenna delays active.
- Samples: 14415 across all 10 links
- Estimator: median per link, equal-weight complete-graph least squares
- Matrix rank/condition: 5/1.633
- Raw link-error RMS: 4.25 cm
- Predicted RMS after rounded corrections: 2.45 cm
- Predicted maximum residual: 3.68 cm

## Proposed module corrections

| Module | Current | fitted DTU | rounded DTU | bootstrap 95% DTU | proposed |
|---|---:|---:|---:|---:|---:|
| M1 | `0x3ff3` | +3.22 | +3 | [+2.99, +3.44] | `0x3ff6` |
| M2 | `0x3ff9` | +2.89 | +3 | [+2.62, +3.18] | `0x3ffc` |
| M3 | `0x3ff2` | +3.39 | +3 | [+2.82, +3.78] | `0x3ff5` |
| M4 | `0x3ffa` | +4.81 | +5 | [+4.24, +5.34] | `0x3fff` |
| M5 | `0x4004` | +4.10 | +4 | [+3.73, +4.47] | `0x4008` |

Positive corrections increase the configured antenna-delay value. The confidence
intervals quantify capture noise only; the stated survey tolerance and link-specific
multipath are not included. Apply all five values as one set, reboot, and validate
with another unchanged capture before accepting them.

## Link diagnostics

| Link | n | known m | median m | raw error cm | std cm | residual after rounded cm |
|---|---:|---:|---:|---:|---:|---:|
| M1-M2 | 3429 | 2.121320 | 2.1750 | +5.37 | 1.16 | +2.55 |
| M1-M3 | 2997 | 2.121320 | 2.1650 | +4.37 | 1.78 | +1.55 |
| M1-M4 | 2564 | 2.121320 | 2.1530 | +3.17 | 4.79 | -0.58 |
| M1-M5 | 3325 | 2.121320 | 2.1240 | +0.27 | 1.93 | -3.02 |
| M2-M3 | 345 | 3.000000 | 3.0290 | +2.90 | 1.61 | +0.09 |
| M2-M4 | 333 | 3.000000 | 3.0410 | +4.10 | 10.05 | +0.35 |
| M2-M5 | 421 | 4.242641 | 4.2460 | +0.34 | 15.43 | -2.95 |
| M3-M4 | 111 | 4.242641 | 4.2450 | +0.24 | 41.31 | -3.52 |
| M3-M5 | 407 | 3.000000 | 3.0590 | +5.90 | 26.92 | +2.62 |
| M4-M5 | 483 | 3.000000 | 3.0790 | +7.90 | 2.85 | +3.68 |
