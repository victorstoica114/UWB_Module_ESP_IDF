# Native DS-TWR static precision reference — firmware 54a586c-dirty

This directory preserves the derived Native DS-TWR precision reference and the
minimum RAW set required by the current report generator.

## Test identity

- Firmware commit: `8244a3432f68caf23d604e475cbe718698c4ab2f`
- Dashboard source commit: `8244a3432f68caf23d604e475cbe718698c4ab2f` (worktree had uncommitted compatibility changes)
- Capture duration: 180.0 s
- Raw range events: 1647
- Sequence delivery: 94.11%
- Radio: channel 5 · slot 100 ms · round gap 10 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(0.000, 3.067), A4=(2.971, -0.071), A5=(3.196, 2.974)
- Reference geometry: Fresh static control on unchanged current setup; cached geometry generation 72 retained only for comparable precision reconstruction. Native DS-TWR Legacy Precision 100/10/90/20/20 ms, no filtering.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 1.72 | 1.01 | 0.90 | 1.14 | 2.30 | 2.93 |
| Rolling latest ranges | 9.16 | 1.02 | 0.90 | 1.16 | 2.34 | 3.71 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

> Retention note (2026-08-29): aggressive RAW retention keeps only the event
> streams for the decisive 58/59/60 ms validations. Other event streams and
> all Native DS-TWR raw log streams were removed. The file list and
> `SHA256SUMS` below are retained as a historical manifest of the original
> complete capture. The retained event streams and metadata are now under
> `../raw/native_ds_speed_revalidation_20260731/`.

- `data/baseline_410ms_legacy_precision_180s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/baseline_410ms_legacy_precision_180s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/baseline_410ms_legacy_precision_180s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference

## Updated speed-limit report

The fresh control was followed by a complete profile revalidation and a
59/60 ms boundary search. The current recommendation is the 60 ms profile:
14 ms slots, 4 ms frame gap, 8 ms RX timeout and 2+2 ms delayed
transmissions.

The full English field report is:

- [`../pdfs/native_ds_speed_revalidation_20260731.pdf`](../pdfs/native_ds_speed_revalidation_20260731.pdf)

Supporting outputs:

Only the decisive 58/59/60 ms validation directories in the central RAW
archive contain event streams. Other `candidate_*` and `profile_*` report
directories retain metadata,
unfiltered positions, summaries and per-anchor raw-integrity audits.

- `PROFILE_REVALIDATION.md` — concise decision and interpretation;
- `profile_comparison.csv` — machine-readable aggregate table;
- `figures/` — six plots in vector PDF and PNG;
- `candidate_*` and `profile_*` — raw events, timing logs, metadata,
  unfiltered positions and per-anchor raw-integrity audits.

Reproduce the figures and PDF:

```bash
cd /home/pi/Documents/UWB
python3 tools/uwb_native_ds_speed_revalidation_report.py
latexmk -cd -pdf -interaction=nonstopmode -halt-on-error \
  -jobname=DS-TWR_speed_limit_field_report_2026-07-31 \
  reports/native_ds_speed_revalidation_20260731/report.tex
```
