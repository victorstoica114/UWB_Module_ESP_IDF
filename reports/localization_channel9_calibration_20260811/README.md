# Channel 9 calibration audit

Current RTK link graph rank: **4/5**.
Per-device antenna-delay identifiable from this capture: **False**.

| Link | calibration median mm | applied mm | validation median mm | validation P95 abs mm |
|---|---:|---:|---:|---:|
| M1-M2 | +96.7 | 78 | +4.4 | 51.4 |
| M1-M3 | -115.0 | -31 | +15.8 | 48.9 |
| M1-M4 | +45.4 | 104 | -19.4 | 59.6 |
| M1-M5 | +91.2 | 72 | -2.4 | 31.4 |

The current star capture contains only M1-to-anchor links, so it cannot uniquely separate five device antenna delays. Existing antenna delays are retained from the earlier full 10-link calibration.

The independent validation confirms the link correction only at one stationary tag location. It is not evidence that the same link bias is universal during motion or at other positions.
