# Native DS-TWR static precision reference — firmware c6ee4a9-dirty

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `c6ee4a9-dirty`
- Dashboard source commit: `c6ee4a99e7a110b2a22b37f7675b4b9b82b735b8` (worktree had uncommitted compatibility changes)
- Capture duration: 180.1 s
- Raw range events: 12315
- Sequence delivery: 100.00%
- Radio: channel 5 · slot 15 ms · round gap 4 ms · RX slice 100 ms
- Antenna delays: M1=16371, M2=16377, M3=16370, M4=16378, M5=16388
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(3.000, 0.000), A4=(0.000, 3.000), A5=(3.000, 3.000)
- Reference geometry: 3.000 m square; T1 static at center; calibrated Native DS-TWR 64 ms position frame reserve; slot 15 ms, gap 4 ms, timeout 8 ms, RESP/FINAL 2+2 ms; full-graph antenna delays active.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 11.41 | 1.38 | 1.83 | 1.39 | 2.87 | 59.68 |
| Rolling latest ranges | 68.41 | 1.40 | 1.87 | 1.38 | 2.94 | 59.70 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/calibrated_64ms_validation_180s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/calibrated_64ms_validation_180s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/calibrated_64ms_validation_180s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
