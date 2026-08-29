# Native DS-TWR static precision reference — firmware 54a586c-dirty

This directory retains the derived metrics and metadata for the original
unfiltered Native DS-TWR precision capture.

> Retention note (2026-08-29): only the decisive 58/59/60 ms validation
> event streams remain in this report bundle. Other event streams and all
> Native DS-TWR raw log streams listed below were intentionally removed.
> `SHA256SUMS` is retained as a historical manifest.

## Test identity

- Firmware commit: `8244a3432f68caf23d604e475cbe718698c4ab2f`
- Dashboard source commit: `8244a3432f68caf23d604e475cbe718698c4ab2f` (worktree had uncommitted compatibility changes)
- Capture duration: 60.0 s
- Raw range events: 3866
- Sequence delivery: 93.02%
- Radio: channel 5 · slot 12 ms · round gap 1 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(0.000, 3.067), A4=(2.971, -0.071), A5=(3.196, 2.974)
- Reference geometry: Unchanged physical setup from fresh 410 ms baseline; cached measured geometry; no position filtering.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 12.21 | 1.04 | 0.91 | 1.14 | 2.42 | 4.24 |
| Rolling latest ranges | 64.37 | 1.05 | 0.93 | 1.14 | 2.45 | 4.95 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/candidate_57ms_screen_60s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/candidate_57ms_screen_60s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/candidate_57ms_screen_60s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
