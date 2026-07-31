# Native DS-TWR antenna-delay validation

## Test conditions

- Two independent 120-second captures on the unchanged 3.000 m square.
- T1 remained at the surveyed center, 2.121320 m from each anchor.
- All ten module-to-module links were measured.
- Delivery was 100% in both captures: no missing frame sequences.
- No filtering was introduced for this comparison.

## Applied values

| Module | Before | Applied | Change |
|---|---:|---:|---:|
| M1 | 16363 | 16371 | +8 DTU |
| M2 | 16368 | 16377 | +9 DTU |
| M3 | 16362 | 16370 | +8 DTU |
| M4 | 16371 | 16378 | +7 DTU |
| M5 | 16371 | 16388 | +17 DTU |

## Independent measured result

| Metric | Before | After | Result |
|---|---:|---:|---:|
| All-link median-error RMS | 9.61 cm | 2.13 cm | 77.8% lower |
| Maximum absolute link bias | 14.00 cm | 3.50 cm | 75.0% lower |
| Tag-link median-error RMS | 8.78 cm | 1.73 cm | 80.3% lower |
| Coherent position precision RMS | 1.77 cm | 1.63 cm | 7.6% lower |
| Coherent position CEP95 | 3.02 cm | 2.83 cm | 6.4% lower |
| Coherent absolute bias at center | 1.36 cm | 2.12 cm | 0.76 cm higher |
| Coherent absolute RMSE at center | 2.23 cm | 2.68 cm | 20.1% higher |
| Coherent position rate | 2.447 Hz | 2.447 Hz | unchanged |
| Sequence delivery | 100% | 100% | unchanged |

The antenna-delay calibration clearly corrected the physical distance scale. It
also slightly improved static position dispersion. It did not improve absolute
position error at this one center point: the uncalibrated tag ranges all contained
a similar positive common-mode bias, which largely cancelled in multilateration.
After calibration, the much smaller but mixed-sign link-specific residuals shifted
the center solution by about 0.76 cm.

This means the corrected set is the better antenna-delay calibration, but one
center point is insufficient to optimize the complete positioning system. A
multi-position validation is required before adding any per-link residual
calibration.

## Per-link median errors

| Link | Before | After |
|---|---:|---:|
| M1-M2 | +9.97 cm | +2.07 cm |
| M1-M3 | +8.77 cm | +1.27 cm |
| M1-M4 | +7.17 cm | +0.37 cm |
| M1-M5 | +8.97 cm | -2.43 cm |
| M2-M3 | +7.10 cm | -0.20 cm |
| M2-M4 | +8.55 cm | +1.20 cm |
| M2-M5 | +10.04 cm | -2.26 cm |
| M3-M4 | +4.04 cm | -3.06 cm |
| M3-M5 | +14.00 cm | +2.30 cm |
| M4-M5 | +13.40 cm | +3.50 cm |

## Residual-fit decision

Fitting the independent post-calibration capture suggested only +1 DTU for M4
and zero rounded change for all other modules. That additional fit would reduce
the same-capture link RMS only from 2.13 cm to a predicted 2.11 cm. It was not
applied because the gain is negligible compared with the stated survey tolerance
and link-specific propagation effects.
