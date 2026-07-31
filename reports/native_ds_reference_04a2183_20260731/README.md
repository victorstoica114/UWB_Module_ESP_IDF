# Native DS-TWR static precision reference — firmware 04a2183

This capture preserves a reproducible unfiltered Native DS-TWR precision reference.

## Test identity

- Firmware commit: `04a21839ddc0430991b77ee98f1d4e75bbcf8fa6`
- Dashboard source commit: `54a586ca0355b1cb942e98ffd88e9cf0e221862d` (worktree had uncommitted compatibility changes)
- Capture duration: 180.0 s
- Raw range events: 1488
- Sequence delivery: 84.74%
- Radio: channel 5 · slot 100 ms · round gap 10 ms · RX slice 100 ms
- Antenna delays: M1=16363, M2=16368, M3=16362, M4=16371, M5=16371
- Geometry used by the dashboard reconstruction: A2=(0.000, 0.000), A3=(0.000, 3.067), A4=(2.971, -0.071), A5=(3.196, 2.974)
- Reference geometry: Cached dynamic geometry generation 72 captured from the modules immediately before installing firmware 04a2183; tag reference is the cached anchor centroid; suitable for precision/jitter reference, not independent absolute-accuracy ground truth.
- Tag remained static. No filtering is applied by this offline reconstruction; it uses the dashboard's unweighted all-anchor linear trilateration.

## Precision reference

| Reconstruction | updates/s | σx [cm] | σy [cm] | precision CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |
|---|---:|---:|---:|---:|---:|---:|
| Coherent independent frames | 0.96 | 0.92 | 0.88 | 0.98 | 2.13 | 4.40 |
| Rolling latest ranges | 8.26 | 1.16 | 1.13 | 1.25 | 3.06 | 6.50 |

Precision is measured around each series' own mean and is the correct metric for the invisible/very short static trail. Values against the cached anchor centroid are retained in `summary.json`, but they are not independent absolute-accuracy ground truth.

## Files

- `data/native_ds_04a2183_static_reference_180s.jsonl` — deduplicated DS-TWR range events and status snapshots
- `data/native_ds_04a2183_static_reference_180s.logs.jsonl` — raw dashboard log records, including every `UWB_RANGING result` line
- `data/native_ds_04a2183_static_reference_180s.metadata.json` — full initial/final module status and capture settings
- `positions.csv` — coherent and rolling offline positions
- `range_metrics.csv` — per-anchor distance stability
- `summary.json` — complete machine-readable metrics and context
- `SHA256SUMS` — integrity hashes for the complete reference
