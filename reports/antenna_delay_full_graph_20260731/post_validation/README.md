# Full-graph Native DS-TWR antenna-delay calibration

This is a dry-run calibration. No antenna delay was written to a module.

- Geometry: 3.000 m square; T1 static at center; independent post-application Native DS-TWR antenna-delay validation capture; applied M1=16371 M2=16377 M3=16370 M4=16378 M5=16388.
- Samples: 1465 across all 10 links
- Estimator: median per link, equal-weight complete-graph least squares
- Matrix rank/condition: 5/1.633
- Raw link-error RMS: 2.13 cm
- Predicted RMS after rounded corrections: 2.11 cm
- Predicted maximum residual: 3.53 cm

## Proposed module corrections

| Module | Current | fitted DTU | rounded DTU | bootstrap 95% DTU | proposed |
|---|---:|---:|---:|---:|---:|
| M1 | `0x3ff3` | +0.42 | +0 | [+0.10, +0.77] | `0x3ff3` |
| M2 | `0x3ff9` | +0.08 | +0 | [-0.36, +0.62] | `0x3ff9` |
| M3 | `0x3ff2` | -0.27 | +0 | [-0.88, +0.33] | `0x3ff2` |
| M4 | `0x3ffa` | +0.94 | +1 | [+0.24, +1.42] | `0x3ffb` |
| M5 | `0x4004` | +0.30 | +0 | [-0.22, +0.72] | `0x4004` |

Positive corrections increase the configured antenna-delay value. The confidence
intervals quantify capture noise only; the stated survey tolerance and link-specific
multipath are not included. Apply all five values as one set, reboot, and validate
with another unchanged capture before accepting them.

## Link diagnostics

| Link | n | known m | median m | raw error cm | std cm | residual after rounded cm |
|---|---:|---:|---:|---:|---:|---:|
| M1-M2 | 293 | 2.121320 | 2.1420 | +2.07 | 1.30 | +2.07 |
| M1-M3 | 292 | 2.121320 | 2.1340 | +1.27 | 1.50 | +1.27 |
| M1-M4 | 293 | 2.121320 | 2.1250 | +0.37 | 1.84 | -0.10 |
| M1-M5 | 293 | 2.121320 | 2.0970 | -2.43 | 1.77 | -2.43 |
| M2-M3 | 49 | 3.000000 | 2.9980 | -0.20 | 1.59 | -0.20 |
| M2-M4 | 49 | 3.000000 | 3.0120 | +1.20 | 1.00 | +0.73 |
| M2-M5 | 49 | 4.242641 | 4.2200 | -2.26 | 1.23 | -2.26 |
| M3-M4 | 49 | 4.242641 | 4.2120 | -3.06 | 1.50 | -3.53 |
| M3-M5 | 49 | 3.000000 | 3.0230 | +2.30 | 1.17 | +2.30 |
| M4-M5 | 49 | 3.000000 | 3.0350 | +3.50 | 1.79 | +3.03 |
