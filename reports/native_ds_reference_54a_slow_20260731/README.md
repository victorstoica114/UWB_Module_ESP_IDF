# Native DS-TWR static precision reference — firmware 54a586c-dirty

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `54a586ca0355b1cb942e98ffd88e9cf0e221862d`
- Dashboard source commit: `54a586ca0355b1cb942e98ffd88e9cf0e221862d` (worktree had uncommitted compatibility changes)
- Capture duration: 180.0 s
- Raw range events: 1647
- Sequence delivery: 94.06%
- Radio: channel 5 · slot 100 ms · round gap 10 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(0.000, 3.000), A4=(3.000, 0.000), A5=(3.000, 3.000)
- Reference geometry: Same unchanged nominal 3 m square and centered tag as 04a2183 reference; 54a strict three-frame native DS-TWR using persisted legacy slow timing 100/10/90/20/20 ms.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 1.72 | 1.10 | 0.97 | 1.25 | 2.55 | 3.70 |
| Rolling latest ranges | 9.15 | 1.08 | 1.04 | 1.27 | 2.61 | 3.73 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/native_ds_54a_slow_timing_180s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/native_ds_54a_slow_timing_180s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/native_ds_54a_slow_timing_180s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
