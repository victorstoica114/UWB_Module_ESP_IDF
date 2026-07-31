# Native DS-TWR static precision reference — firmware c6ee4a9-dirty

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `c6ee4a9-dirty`
- Dashboard source commit: `c6ee4a99e7a110b2a22b37f7675b4b9b82b735b8` (worktree had uncommitted compatibility changes)
- Capture duration: 120.0 s
- Raw range events: 1171
- Sequence delivery: 100.00%
- Radio: channel 5 · slot 100 ms · round gap 10 ms · RX slice 100 ms
- Antenna delays: M1=16371, M2=16377, M3=16370, M4=16378, M5=16388
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(3.000, 0.000), A4=(0.000, 3.000), A5=(3.000, 3.000)
- Reference geometry: 3.000 m square; T1 static at center; independent post-application Native DS-TWR antenna-delay validation capture; applied M1=16371 M2=16377 M3=16370 M4=16378 M5=16388.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 2.45 | 1.08 | 1.23 | 1.38 | 2.83 | 3.82 |
| Rolling latest ranges | 9.76 | 1.09 | 1.21 | 1.37 | 2.82 | 3.89 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/antenna_delay_full_graph_corrected.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/antenna_delay_full_graph_corrected.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/antenna_delay_full_graph_corrected.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
