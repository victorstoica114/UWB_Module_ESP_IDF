# Native DS-TWR static precision reference — firmware c6ee4a9

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `c6ee4a99e7a110b2a22b37f7675b4b9b82b735b8`
- Dashboard source commit: `c6ee4a99e7a110b2a22b37f7675b4b9b82b735b8` (worktree had uncommitted compatibility changes)
- Capture duration: 60.0 s
- Raw range events: 586
- Sequence delivery: 100.00%
- Radio: channel 5 · slot 100 ms · round gap 10 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(3.000, 0.000), A4=(0.000, 3.000), A5=(3.000, 3.000)
- Reference geometry: Known 3 m square; static tag at center; clean c6 three-packet Native DS-TWR; 100/10/90/20/20 ms profile.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 2.46 | 1.09 | 1.29 | 1.53 | 2.62 | 4.29 |
| Rolling latest ranges | 9.77 | 1.07 | 1.23 | 1.36 | 2.75 | 4.28 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/clean_c6_reference_timing_retry_60s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/clean_c6_reference_timing_retry_60s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/clean_c6_reference_timing_retry_60s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
