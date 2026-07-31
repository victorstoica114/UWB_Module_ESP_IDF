# Calibrated Native DS-TWR fast-profile selection

## Decision

Use the **60 ms Validated Fast Profile**:

- 14 ms per-anchor slot;
- 4 ms frame guard time;
- 8 ms RX timeout;
- 2 ms RESP delay;
- 2 ms FINAL delay;
- 100 ms anchor RX slice;
- 500 UUS automatic RX delay.

The 100 ms per-anchor slot / 410 ms complete frame remains the precision and
calibration reference.

## Current calibrated comparison

All captures used the same 3.000 m square, centered static tag, clean Native
DS-TWR firmware, Raspberry solver and calibrated per-module antenna delays.
Metrics are reconstructed offline without position filtering.

| Metric | 100 ms slot reference | 60 ms candidate | 64 ms candidate |
|---|---:|---:|---:|
| Capture duration | 120 s | 180 s | 180 s |
| Coherent frames/s | 2.45 | **12.21** | 11.41 |
| Rolling updates/s | 9.76 | **68.46** | 68.41 |
| Complete coherent frames | 292 | **2,195** | 2,054 |
| Coherent precision RMS | 1.63 cm | **1.64 cm** | 2.30 cm |
| Coherent CEP95 | 2.83 cm | **2.76 cm** | 2.87 cm |
| Tag-range deviations >10 cm | 0 | **0** | 3 |
| Anchor-range deviations >10 cm | 0 | 1 | 10 |
| Maximum position deviation around capture mean | 3.82 cm | 4.42 cm | 59.68 cm |

Relative to the calibrated reference, the 60 ms profile delivered 4.99 times
more complete coherent positions per second and 7.01 times more rolling updates,
while coherent precision RMS changed by only +0.7%. Its CEP95 was 2.4% better in
this capture. The single anchor-survey spike was rejected by the geometry path
and did not enter tag localization.

The 64 ms profile was rejected on the current runtime. Its failures were spread
throughout the capture rather than being a startup transient. The result suggests
a periodic scheduling interaction between the 15 ms tag slots and concurrent
100 ms geometry-survey traffic; the nominally slower frame did not produce more
usable coherent positions than the 60 ms profile.

## Historical boundary evidence

The earlier 2026-07-31 validation independently reached the same 60 ms boundary:
60 ms completed 180 seconds without impossible tag ranges, while 59 ms emitted a
-5.301 m range. Profiles below 60 ms therefore remain stress-test profiles, not
operational candidates.
