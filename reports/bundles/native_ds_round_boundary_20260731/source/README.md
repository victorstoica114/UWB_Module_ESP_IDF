# Native DS-TWR round-boundary investigation

Date: 2026-07-31

Branch: `experiment/native-ds-round-boundary`

Reference commit before instrumentation: `c727a64`

## Report

The illustrated interpretation report is available as
[`../../../native_ds_round_boundary_20260731.pdf`](../../../native_ds_round_boundary_20260731.pdf).
Its figures and machine-readable summary can be regenerated with:

```bash
tools/uwb_native_ds_round_boundary_report.py
latexmk -pdf -interaction=nonstopmode -halt-on-error \
  -cd reports/bundles/native_ds_round_boundary_20260731/source/report.tex
```

## Objective

Determine why shortening the Native DS-TWR frame occasionally reduced
position stability even though an individual POLL -> RESP -> FINAL exchange
fit inside its slot. The test specifically checked whether the failure was
tied to A2/the first configured anchor, to stale exchange state, or to the
periodic anchor-geometry exchange inserted between tag frames.

## Instrumentation

- Every POLL, RESP and FINAL carries the same random token, sequence, round,
  slot, first-anchor ID and boundary flags.
- Initiator and responder use two alternating exchange contexts.
- DW3000 timestamps and low-32 ESP timer values are recorded for every stage.
- Context and timestamp intervals are validated before the DS-TWR formula.
- A non-finite, non-positive or physically impossible ToF is rejected before
  telemetry and position solving.
- The first anchor rotates between rounds to separate an anchor-specific fault
  from a schedule-position fault.

## One-hour captures before the boundary guard

All modules were kept in the same static surveyed geometry. `valid ranges`
below is counted directly from the lossless log stream; the original event
counter used a 16-bit sequence-only deduplication key and under-counted after
the first sequence wrap.

| Normal frame | Slot + gap | Valid ranges | Source rejects | Geometry exchange | First post-geometry round | Ordinary round |
|---:|---:|---:|---:|---:|---:|---:|
| 59 ms | 14 + 3 ms | 220,710 | 34 | 29 | 5 | 0 |
| 58 ms | 14 + 2 ms | 223,776 | 35 | 25 | 10 | 0 |
| 57 ms | 14 + 1 ms | 226,315 | 98 | 81 | 17 | 0 |
| 53 ms | 13 + 1 ms | 242,069 | 100 | 78 | 22 | 0 |

The last two columns treat every slot in the first round after geometry as a
boundary event. In the original firmware only slot zero carried the explicit
flag; the analysis reconstructs the remainder from its round number.

## Finding

All 267 rejected ToFs are at the geometry transition. None occurs in an
ordinary tag ranging round. The errors move between A2, A3, A4 and A5 when the
first anchor rotates, disproving an A2-specific RF or antenna-delay problem.

The configured round gap was scheduled after the optional geometry exchange,
so it did not isolate the preceding tag FINAL from the SURVEY_CMD. The next
tag POLL also relied only on the nominal geometry-slot duration. Occasional
radio/host scheduling latency therefore allowed the two initiator regimes to
touch at either side of the boundary.

## Correction

- Place the configured frame gap immediately after every tag round.
- Add a 5 ms guard before SURVEY_CMD and another 5 ms after its DS-TWR slot.
- Mark every slot in the first following tag round as post-geometry.
- Keep detailed accepted traces sampled (first slot and every 64th sequence),
  while rejected exchanges always retain a complete trace.
- Deduplicate collector events with token + round + slot, not the wrapping
  16-bit sequence alone.

The guards affect only the periodic self-localization exchange; an ordinary
tag frame remains 53 ms in the fastest tested profile.

## One-hour validation after the guard

The corrected firmware was rebuilt, uploaded simultaneously to all five
modules, and run for another uninterrupted hour at the most aggressive 53 ms
profile.

| 53 ms profile | Valid ranges | Rate | RESP timeouts | FINAL timeouts | Source rejects | Context mismatches |
|---|---:|---:|---:|---:|---:|---:|
| Before guard | 242,069 | 67.24/s | 8,514 | 923 | 100 | 0 |
| Guarded | 235,249 | 65.35/s | 3,944 | 31 | 0 | 0 |

The 5 + 5 ms periodic guard costs 2.8% of delivered range throughput while
eliminating every physically impossible ToF in this capture. RESP timeouts
fall by 53.7% and FINAL timeouts by 96.6%. The collector also remains linear
through multiple 16-bit sequence wraps, confirming the token/round
deduplication fix.

This demonstrates that the original speed/precision trade-off was not caused
by the POLL -> RESP -> FINAL airtime of an ordinary frame. It was caused by
radio-state overlap around the separately inserted anchor-geometry exchange.
The normal 53 ms frame is therefore a viable operating point with the guarded
geometry schedule.

## Reproduction

```bash
tools/analyze_native_ds_round_boundary.py \
  reports/bundles/native_ds_round_boundary_20260731/raw/data/candidate_59ms_round_boundary_3600s.logs.jsonl \
  reports/bundles/native_ds_round_boundary_20260731/raw/data/candidate_58ms_round_boundary_3600s.logs.jsonl \
  reports/bundles/native_ds_round_boundary_20260731/raw/data/candidate_57ms_round_boundary_3600s.logs.jsonl \
  reports/bundles/native_ds_round_boundary_20260731/raw/data/candidate_53ms_round_boundary_3600s.logs.jsonl \
  reports/bundles/native_ds_round_boundary_20260731/raw/data/candidate_53ms_guarded_3600s.logs.jsonl
```

Raw event and timing-log streams are stored in
`../raw/data/` as individually
compressed `.jsonl.zst` archives, alongside their uncompressed metadata
sidecars. Restore them with
`zstd --decompress --keep reports/bundles/native_ds_round_boundary_20260731/raw/data/*.jsonl.zst`
before
running the reproduction command. The interrupted 130 s capture is retained
and explicitly named `candidate_53ms_guarded_aborted_130s` because it
documents the performance effect of over-instrumenting every post-geometry
slot.
