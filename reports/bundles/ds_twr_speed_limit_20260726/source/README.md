# Native DS-TWR speed-limit study

Static LOS speed sweep for the native `POLL -> RESP -> FINAL` DS-TWR
implementation. The immutable 64 ms field captures from
`reports/bundles/uwb_protocol_comparison_20260726/raw/data` are the reference.

Key validated results:

| Profile | Complete positions/s | Complete frames | 2D RMSE | P95 | Radio success |
|---|---:|---:|---:|---:|---:|
| 64 ms reference | 15.15 | 97.34% | 3.25 cm | 4.80 cm | not captured |
| 17 ms balanced | 58.02 | 99.85% | 3.35 cm | 4.65 cm | 99.966% |
| 13 ms maximum | 74.68 | 98.71% | 3.47 cm | 4.83 cm | 99.678% |

The recommended operating profile is 17 ms: 4 ms slots, 1 ms round gap,
4 ms RX timeout, 1 ms response delay, 1 ms final delay, and 500 UUS auto-RX
delay. The modules were left on this profile.

Two exploratory blocks are preserved but excluded from comparison:

- the first 33 ms block exposed periodic diagnostic-register/log overhead in
  the real-time anchor path;
- the first 21 ms block exposed the old HTTP snapshot collector's overwrite
  limit, while radio timing remained healthy.

Both exclusions are marked in metadata and are independent of ground-truth
position error.

Reproduce:

```bash
cd /home/pi/Documents/UWB
xz --decompress --keep reports/bundles/ds_twr_speed_limit_20260726/raw/data/*.jsonl.xz
python3 tools/uwb_ds_speed_analyze.py
cd reports/bundles/ds_twr_speed_limit_20260726/source
latexmk -pdf -interaction=nonstopmode -halt-on-error report.tex
```

Outputs:

- [`../../../ds_twr_speed_limit_20260726.pdf`](../../../ds_twr_speed_limit_20260726.pdf)
- `profile_metrics.csv`
- `block_metrics.csv`
- `positions.csv`
- `analysis_summary.json`
- `raw_data_manifest.csv`
- `figures/`
- `../raw/data/*.jsonl.xz` and adjacent metadata
