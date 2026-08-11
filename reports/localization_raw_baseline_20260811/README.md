# Raw UWB localization baseline

This report scores only raw UWB positions. IMU-derived and temporally filtered positions are excluded.

| Split | Protocol | RTK pairs | RMSE cm | P95 cm | Max cm |
|---|---|---:|---:|---:|---:|
| calibration | FlexTDOA | 23 | 115.79 | 213.53 | 258.07 |
| calibration | Native DS-TWR | 67 | 137.66 | 302.69 | 391.33 |
| calibration | Passive DS-TWR | 22 | 315.53 | 454.83 | 478.38 |
| validation | FlexTDOA | 1144 | 18.85 | 32.19 | 80.19 |
| validation | Native DS-TWR | 1843 | 19.18 | 29.67 | 278.00 |
| validation | Passive DS-TWR | 996 | 17.13 | 38.83 | 67.91 |

Validation matrix complete: **True**.
All validation captures have RTK accuracy evidence: **True**.

Absolute RTK error is reported separately from local cloud precision and debiased error. Geometry registration uses anchor coordinates only; the tag trajectory is never used to fit the transform.

`range_rtk_pairs.csv` and the per-capture range sections in `baseline_summary.json` may be incomplete when a capture did not record individual ranges. Missing evidence is reported, not inferred from positions.
