# Native DS-TWR static precision reference — firmware 54a586c-dirty

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `8244a3432f68caf23d604e475cbe718698c4ab2f`
- Dashboard source commit: `8244a3432f68caf23d604e475cbe718698c4ab2f` (worktree had uncommitted compatibility changes)
- Capture duration: 60.0 s
- Raw range events: 3719
- Sequence delivery: 93.91%
- Radio: channel 5 · slot 15 ms · round gap 1 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(0.000, 3.067), A4=(2.971, -0.071), A5=(3.196, 2.974)
- Reference geometry: Unchanged setup; isolate 14 ms slot with safe 4 ms frame gap.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 15.39 | 0.97 | 0.94 | 1.12 | 2.33 | 3.42 |
| Rolling latest ranges | 61.96 | 0.97 | 0.95 | 1.12 | 2.33 | 5.16 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/candidate_60ms_slot14_gap4_screen_60s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/candidate_60ms_slot14_gap4_screen_60s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/candidate_60ms_slot14_gap4_screen_60s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
