# Full-graph Native DS-TWR antenna-delay calibration

The first capture below is the calibration dry run. The proposed values were then
written to all five modules as one set and validated with a second independent
120-second capture. See `VALIDATION.md` for the measured before/after result.

- Geometry: 3.000 m square; T1 static at center; full 10-link raw Native DS-TWR antenna-delay calibration capture; no corrections applied.
- Samples: 1463 across all 10 links
- Estimator: median per link, equal-weight complete-graph least squares
- Matrix rank/condition: 5/1.633
- Raw link-error RMS: 9.61 cm
- Predicted RMS after rounded corrections: 1.96 cm
- Predicted maximum residual: 3.00 cm

## Proposed module corrections

| Module | Current | fitted DTU | rounded DTU | bootstrap 95% DTU | proposed |
|---|---:|---:|---:|---:|---:|
| M1 | `0x3feb` | +8.44 | +8 | [+7.92, +8.65] | `0x3ff3` |
| M2 | `0x3ff0` | +8.99 | +9 | [+8.51, +9.52] | `0x3ff9` |
| M3 | `0x3fea` | +7.75 | +8 | [+7.33, +8.76] | `0x3ff2` |
| M4 | `0x3ff3` | +7.22 | +7 | [+6.91, +8.22] | `0x3ffa` |
| M5 | `0x3ff3` | +16.63 | +17 | [+16.16, +17.16] | `0x4004` |

Positive corrections increase the configured antenna-delay value. The confidence
intervals quantify capture noise only; the stated survey tolerance and link-specific
multipath are not included. Apply all five values as one set, reboot, and validate
with another unchanged capture before accepting them.

## Independent validation

The calibrated values were applied and verified active after reboot:

| Module | Applied decimal | Applied hex |
|---|---:|---:|
| M1 | 16371 | `0x3ff3` |
| M2 | 16377 | `0x3ff9` |
| M3 | 16370 | `0x3ff2` |
| M4 | 16378 | `0x3ffa` |
| M5 | 16388 | `0x4004` |

The independent capture measured a 2.13 cm RMS link error, versus 9.61 cm before
calibration. Static coherent-position precision improved from 1.77 cm to 1.63 cm,
but absolute position RMSE at this single center point increased from 2.23 cm to
2.68 cm. The full interpretation and per-link result are in `VALIDATION.md`.

## Link diagnostics

| Link | n | known m | median m | raw error cm | std cm | residual after rounded cm |
|---|---:|---:|---:|---:|---:|---:|
| M1-M2 | 293 | 2.121320 | 2.2210 | +9.97 | 1.36 | +1.99 |
| M1-M3 | 293 | 2.121320 | 2.2090 | +8.77 | 1.61 | +1.26 |
| M1-M4 | 293 | 2.121320 | 2.1930 | +7.17 | 1.85 | +0.13 |
| M1-M5 | 292 | 2.121320 | 2.2110 | +8.97 | 1.74 | -2.76 |
| M2-M3 | 49 | 3.000000 | 3.0710 | +7.10 | 1.45 | -0.87 |
| M2-M4 | 48 | 3.000000 | 3.0855 | +8.55 | 0.81 | +1.05 |
| M2-M5 | 48 | 4.242641 | 4.3430 | +10.04 | 1.23 | -2.16 |
| M3-M4 | 49 | 4.242641 | 4.2830 | +4.04 | 1.67 | -3.00 |
| M3-M5 | 49 | 3.000000 | 3.1400 | +14.00 | 1.16 | +2.27 |
| M4-M5 | 49 | 3.000000 | 3.1340 | +13.40 | 1.59 | +2.14 |
