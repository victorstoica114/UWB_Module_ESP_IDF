# Native DS-TWR clean rewrite — RTK field validation

- Date: 2026-08-05
- Branch: `ds-twr-clean-rewrite`
- Radio: DW3000 channel 9
- Position filtering: none

## Why the old dashboard reference was wrong

The old Native DS-TWR solver reconstructed an anchor map in an arbitrary local
gauge from anchor-to-anchor ranges. The dashboard then drew the GPS RTK tag in
the fixed RTK ENU frame. Both markers could be internally plausible while being
separated by metres because they did not share an origin, rotation, or scale
constraint. The screenshot was therefore primarily a coordinate-frame bug, not
evidence of a multi-metre UWB ranging error.

The clean solver uses the same persistent GPS RTK ENU anchor coordinates as
FlexTDOA, with A2 as `(0, 0)`. Anchor-to-anchor ranges are diagnostics only and
can no longer move, rotate, or rescale the position frame.

## Clean protocol

The implementation is an explicit four-message exchange:

1. tag sends `POLL`;
2. anchor sends delayed `RESPONSE`;
3. tag sends delayed `FINAL` containing the required timestamps;
4. anchor computes asymmetric DS-TWR and sends one-shot `RESULT`.

Packets use the new `NDS4` codec with version, exchange kind, source,
destination, 32-bit session ID, 32-bit frame ID, exact length validation, and
40-bit DW3000 timestamps. A result is accepted only for the matching
session/frame/anchor. The solver emits a position only after one coherent range
from every configured anchor and does not use the previous position as a seed.

Delayed TX timestamps remain strict inputs for `RESPONSE` and `FINAL`. The TX
timestamp of `RESULT` is deliberately diagnostic only: that packet carries an
already computed distance, so its TX timestamp is not part of the ADS-TWR
equation. A successfully completed `RESULT` transmission is therefore retained
even if the DW3000 reports a differently quantized timestamp.

## Field configuration

- anchors: A2 `(0.000, 0.000)`, A3 `(3.844, 2.923)`,
  A4 `(-3.120, 3.899)`, A5 `(1.055, 6.836)` metres;
- tag reference: approximately `(0.331, 3.390)` metres in the same RTK ENU
  frame;
- profile: 100 ms per anchor, 10 ms frame gap, 20 ms RESPONSE delay,
  20 ms FINAL delay, 20 ms RESULT delay, 30 ms RX timeout;
- persistent measured-minus-RTK range corrections for A2..A5:
  `[-49, +60, +13, -54]` mm, generation 2.

The per-anchor correction is hardware/range calibration, not a position filter.
Both raw and corrected distances remain available in telemetry.

## Results

| Capture | 2D RMSE | Bias | P95 | Precision RMS | Solver residual mean |
|---|---:|---:|---:|---:|---:|
| clean rewrite, calibration disabled | 2.75 cm | 2.46 cm | 3.97 cm | 1.24 cm | 4.41 cm |
| calibrated, first validation | 1.45 cm | 0.60 cm | 2.39 cm | 1.32 cm | 0.81 cm |
| calibrated, final reliability run | **1.17 cm** | **0.32 cm** | **1.98 cm** | **1.13 cm** | **0.81 cm** |

The final 120 s run captured 283 independent raw positions, or about 2.36
positions/s. 188 positions had a tag RTK Fixed sample within the strict 0.5 s
alignment window and were used for accuracy metrics. The maximum aligned error
was 3.08 cm. The fixed anchor geometry agreed with the live RTK observations to
0.27 cm RMS during this capture.

The range pipeline during the final run was:

| Module | POLL | RESPONSE | FINAL | RESULT / complete | RX timeout | delayed TX | overrun |
|---|---:|---:|---:|---:|---:|---:|---:|
| M1 tag | 1276 TX | 1266 RX | 1266 TX | 1263 RX | 8 | 0 | 0 |
| M2 | 319 RX | 319 TX | 318 RX | 318 TX | 1 | 0 | 0 |
| M3 | 314 RX | 314 TX | 313 RX | 313 TX | 1 | 0 | 0 |
| M4 | 318 RX | 317 TX | 315 RX | 315 TX | 2 | 0 | 0 |
| M5 | 319 RX | 317 TX | 315 RX | 314 TX | 1 | 3 | 0 |

The tag completed 1263 of 1276 initiated anchor exchanges (99.0%). Reducing
the timeout from 90 ms to 30 ms eliminated slot overruns in this run. The three
M5 delayed-TX failures are real radio/driver failures, not the removed
`RESULT` timestamp comparison; they remain exposed by telemetry.

## Reproducible artifacts

- `summary.json`: clean uncalibrated rewrite analysis;
- `calibrated_summary.json`: first calibrated analysis;
- `calibrated_reliability_summary.json`: final deployed image and 30 ms timeout;
- `cross_validated_bias_summary.json`: biases learned from the older capture
  applied out-of-sample to the clean raw capture;
- `old_native_ds_dryrun_summary.json`: legacy field capture analysis;
- `data/*.jsonl` and matching `*.metadata.json`: raw events and start/end
  device status.

## Verification

- Native DS packet codec/math unit test: PASS;
- independent-frame position solver unit test: PASS;
- protocol boundary checker: PASS;
- dashboard Python syntax and embedded JavaScript syntax: PASS;
- full ESP-IDF 6.0.2 build: PASS, 69% of the smallest app partition free;
- OTA: all five modules, boot guard validated;
- deployed image SHA-256:
  `7d1858b24d39f03ffeb443f5f94d1b3040585ef031c38bf42077daf4d968448d`.
