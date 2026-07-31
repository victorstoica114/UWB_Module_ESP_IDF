# Full-graph Native DS-TWR antenna-delay calibration

This is a dry-run calibration. No antenna delay was written to a module.

- Geometry: 3.000 m square; T1 static at center; calibrated Native DS-TWR 60 ms position frame candidate; slot 14 ms, gap 4 ms, timeout 8 ms, RESP/FINAL 2+2 ms; full-graph antenna delays active.
- Samples: 14837 across all 10 links
- Estimator: median per link, equal-weight complete-graph least squares
- Matrix rank/condition: 5/1.633
- Raw link-error RMS: 4.03 cm
- Predicted RMS after rounded corrections: 2.11 cm
- Predicted maximum residual: 3.15 cm

## Proposed module corrections

| Module | Current | fitted DTU | rounded DTU | bootstrap 95% DTU | proposed |
|---|---:|---:|---:|---:|---:|
| M1 | `0x3ff3` | +3.31 | +3 | [+3.10, +3.40] | `0x3ff6` |
| M2 | `0x3ff9` | +2.62 | +3 | [+2.51, +2.91] | `0x3ffc` |
| M3 | `0x3ff2` | +4.19 | +4 | [+3.92, +4.42] | `0x3ff6` |
| M4 | `0x3ffa` | +4.90 | +5 | [+4.61, +5.18] | `0x3fff` |
| M5 | `0x4004` | +3.12 | +3 | [+2.87, +3.30] | `0x4007` |

Positive corrections increase the configured antenna-delay value. The confidence
intervals quantify capture noise only; the stated survey tolerance and link-specific
multipath are not included. Apply all five values as one set, reboot, and validate
with another unchanged capture before accepting them.

## Link diagnostics

| Link | n | known m | median m | raw error cm | std cm | residual after rounded cm |
|---|---:|---:|---:|---:|---:|---:|
| M1-M2 | 3376 | 2.121320 | 2.1730 | +5.17 | 1.20 | +2.35 |
| M1-M3 | 3287 | 2.121320 | 2.1660 | +4.47 | 1.74 | +1.18 |
| M1-M4 | 2417 | 2.121320 | 2.1540 | +3.27 | 1.68 | -0.48 |
| M1-M5 | 3245 | 2.121320 | 2.1240 | +0.27 | 1.69 | -2.55 |
| M2-M3 | 251 | 3.000000 | 3.0300 | +3.00 | 1.56 | -0.28 |
| M2-M4 | 402 | 3.000000 | 3.0390 | +3.90 | 1.11 | +0.15 |
| M2-M5 | 427 | 4.242641 | 4.2440 | +0.14 | 4.49 | -2.68 |
| M3-M4 | 543 | 4.242641 | 4.2560 | +1.34 | 2.10 | -2.89 |
| M3-M5 | 550 | 3.000000 | 3.0560 | +5.60 | 1.26 | +2.32 |
| M4-M5 | 339 | 3.000000 | 3.0690 | +6.90 | 1.59 | +3.15 |
