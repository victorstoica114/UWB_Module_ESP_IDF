# Passive DS-TDoA clean rewrite — 2026-07-31

## Scope

This dataset validates the clean receive-only Passive DS-TWR rewrite on the
surveyed 3 m square (A2, A3, A4, A5), with Tag 1 at the 1.5 m, 1.5 m centre.
The implementation listens to the native three-message DS-TWR exchange
(POLL, RESP, FINAL), uses DW3000 timestamps, and does not use carrier-frequency
offset as a substitute for the third timestamp.

The protocol math and coherent exchange assembler are isolated in:

- `components/uwb_dw3000/uwb_passive_ds_tdoa.c`
- `components/uwb_dw3000/uwb_passive_ds_tdoa.h`

The DW3000 radio adapter remains in `uwb_dw3000.c`; Native DS-TWR math is not
called by the passive estimator. The host regression test is
`tools/uwb_passive_ds_tdoa_host_test.c`.

## Active configuration

- schedule: robust rotating
- slot: 3 ms
- round gap: 1 ms
- responder delay: 1000 us
- final delay: 1000 us
- rolling maximum: 200 Hz
- tag/anchor bias list: `0,-27,18,-64` mm
- anchor-pair range bias list: `29,43,4,12,59,71` mm, ordered as
  A2-A3, A2-A4, A2-A5, A3-A4, A3-A5, A4-A5
- fixed module geometry: disabled

## Main result

The post-reset 30 s capture is
`ds_tdoa_3ms_dynamic_geometry_calibrated_30s.jsonl`.

| Metric | Live geometry | Surveyed 3 m geometry |
|---|---:|---:|
| Position rate | 99.77 Hz | 166.99 rolling solves/s |
| Mean position bias | 0.29 cm | 0.48 cm |
| RMSE | 2.55 cm | 1.64 cm |
| P95 position error | 4.53 cm | 2.84 cm |
| Maximum error | 11.17 cm | 9.41 cm |

The receiver produced 5024 coherent directed observations in 30.08 s, or
167.02 observations/s. The live-geometry independent-frame subset ran at
22.04 frames/s with 2.17 cm RMSE and 3.83 cm P95.

The preceding capture, before clearing the stale fixed geometry, used the
same passive observations but measured 6.42 cm live-geometry RMSE. This
separates the estimator performance from the geometry-state error.

## Interpretation

The clean three-packet estimator is not the former CFO-aided two-packet
approximation. It implements double-sided TDoA extraction from a normal
DS-TWR exchange. Symmetric responder and initiator delays are retained because
they minimise the propagated timestamp variance.

The GPS Compendium is used only for general measurement principles: receiver
delay and multipath remain physical error sources, while differential
measurements remove common clock state. It does not imply that GPS time or a
GPS clock correction belongs in this UWB estimator.

Scientific basis:

- Rathje and Landsiedel, *Time Difference of Arrival Extraction from Two-Way
  Ranging*, 2022: <https://arxiv.org/abs/2204.08996>
- Rathje and Landsiedel, *Precise Ranging: Modeling Bias and Variance of
  Double-Sided Two-Way Ranging with TDoA Extraction under Multipath and NLOS
  Effects*, 2024: <https://arxiv.org/abs/2410.12826>
- u-blox, *GPS Compendium*, GPS-X-02007, especially measurement-error and
  common-view differential-time discussions:
  `datasheets/GPS-Compendium_Book_(GPS-X-02007).pdf`

## Reproduction

```sh
gcc -std=c11 -Wall -Wextra -Werror \
  -Icomponents/uwb_dw3000 \
  tools/uwb_passive_ds_tdoa_host_test.c \
  components/uwb_dw3000/uwb_passive_ds_tdoa.c \
  -lm -o /tmp/uwb_passive_ds_tdoa_host_test
/tmp/uwb_passive_ds_tdoa_host_test

python3 tools/uwb_passive_ds_tdoa_analyze.py \
  reports/passive_ds_tdoa_rewrite_20260731/ds_tdoa_3ms_dynamic_geometry_calibrated_30s.jsonl \
  --output-directory \
  reports/passive_ds_tdoa_rewrite_20260731/dynamic_geometry_calibrated_analysis
```
