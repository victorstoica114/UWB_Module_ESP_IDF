# Native DS-TWR profile revalidation — 2026-07-31

## Outcome

The fastest profile accepted on the unchanged static setup is **60 ms**:

- 14 ms per anchor slot
- 4 ms frame gap
- 8 ms RX timeout
- 2 ms RESP delay
- 2 ms FINAL delay
- 100 ms anchor RX slice
- 500 UUS automatic RX delay

It completed a 180 s validation with 11,158 raw tag-to-anchor ranges and
**zero negative or >10 cm raw outliers**. The 59 ms candidate produced an
impossible -5.301 m range during the equivalent validation, so 60 ms is the
observed boundary rather than a rounded convenience value.

The 64 ms profile remains a validated reserve. The 410 ms profile remains the
slow precision control.

## Fair comparison against the fresh control

All three accepted captures used the same firmware, radio channel, antenna
delays, cached measured anchor geometry, tag reference and unchanged physical
setup. Every result below comes from an unfiltered offline reconstruction.
Precision is measured around each capture's own mean.

| Profile | Capture | Raw ranges | Delivery | Coherent frames/s | Rolling updates/s | Coherent RMS | Coherent CEP95 | Raw anomalies |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 410 ms control | 180 s | 1,647 | 94.11% | 1.72 | 9.16 | 1.36 cm | 2.30 cm | 0 |
| 64 ms reserve | 180 s | 10,468 | 94.03% | 10.92 | 58.14 | 1.38 cm | 2.41 cm | 0 |
| **60 ms limit** | **180 s** | **11,158** | **93.97%** | **11.63** | **61.96** | **1.38 cm** | **2.39 cm** | **0** |

Relative to 410 ms, the 60 ms profile provides:

- 6.75 times as many coherent independent frames per second;
- 6.77 times as many rolling display updates per second;
- +1.56% coherent precision RMS;
- +3.94% coherent CEP95;
- -0.14 percentage points of sequence delivery.

This is a substantial speed gain with no material static precision loss in
this capture.

## Boundary search

The central position distribution can remain deceptively tight when a timing
profile occasionally emits a physically impossible raw range. Therefore a
profile was rejected if any raw distance was negative or differed by more than
10 cm from that anchor's capture median. Solver rejection was not counted as a
successful radio result.

| Profile | Capture | Coherent frames/s | RMS | CEP95 | Negative ranges | >10 cm raw spikes | Minimum raw range | Verdict |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 60 ms | 180 s | 11.63 | 1.38 cm | 2.39 cm | 0 | 0 | 2.152 m | accepted |
| 59 ms | 180 s | 11.82 | 1.31 cm | 2.20 cm | 1 | 1 | -5.301 m | rejected |
| 58 ms | 180 s | 12.00 | 1.35 cm | 2.30 cm | 2 | 2 | -5.875 m | rejected |
| 57 ms | 60 s | 12.21 | 1.38 cm | 2.42 cm | 1 | 1 | -23.455 m | rejected |
| 49 ms | 60 s | 17.61 | 1.30 cm | 2.25 cm | 1 | 1 | -17.889 m | rejected |
| 33 ms | 180 s | 20.42 | 1.33 cm | 2.33 cm | 7 | 9 | -20.511 m | rejected |
| 29 ms | 180 s | 23.37 | 1.43 cm | 2.46 cm | 13 | 19 | -22.831 m | rejected |
| 25 ms | 60 s | 27.09 | 1.43 cm | 2.47 cm | 2 | 2 | -3.646 m | rejected |
| 21 ms | 60 s | 33.34 | 1.43 cm | 2.53 cm | 4 | 4 | -39.181 m | rejected |
| 17 ms | 60 s | 39.77 | 1.34 cm | 2.35 cm | 13 | 15 | -40.249 m | rejected |
| 13 ms | 60 s | 44.99 | 1.38 cm | 2.36 cm | 20 | 22 | -53.505 m | rejected |

The short 58 ms and 29 ms screens initially contained no anomaly. Both failed
their longer validation, demonstrating why a short visual test is not enough.

## Interpretation

The 59/60 ms transition points to recovery time between frames, not a gradual
precision-noise tradeoff. The 60 ms profile uses a 14 ms slot and a 4 ms frame
gap. Reducing the gap to 3 ms preserved the central RMS but allowed a rare
invalid timestamp/range path on A2.

This is an observed operational boundary for this firmware and five-module
setup, not a universal DW3000 limit. Dynamic tests, longer endurance runs and
other RF geometries are still required. Until then:

- use 60 ms for the best validated speed;
- use 64 ms when extra scheduling reserve matters;
- use 410 ms only as the precision/control reference;
- treat every profile below 60 ms as an experimental stress test.

## Reproducibility

Each profile directory retains:

- full initial/final module status metadata;
- unfiltered coherent and rolling position reconstructions;
- per-anchor range metrics;
- `summary.json`;
- `raw_quality.json`, including negative and >10 cm anomaly counts.

Under the aggressive retention policy, only the decisive 58/59/60 ms
validation event streams remain. Other event streams and all Native DS-TWR
raw log streams were removed after their derived artifacts were verified.
The per-capture `SHA256SUMS` files remain historical manifests and can name
RAW files that are no longer present.

The fresh control is documented in `README.md`. Exact profile results are in
the corresponding `profile_*` and `candidate_*` directories.
