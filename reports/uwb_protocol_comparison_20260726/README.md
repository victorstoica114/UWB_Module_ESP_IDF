# DS-TWR versus FlexTDOA field comparison

Controlled LOS capture performed on 2026-07-26 with four anchors in a surveyed
3.000 m square and the tag fixed at the center.

The repository stores the six raw JSONL captures as lossless XZ archives so the
complete experiment remains below GitHub's normal file-size limits. Restore the
collector output before rerunning the analysis:

```bash
xz --decompress --keep data/*.jsonl.xz
cd ../..
python3 tools/uwb_compare_analyze.py
cd reports/uwb_protocol_comparison_20260726
latexmk -pdf -interaction=nonstopmode -halt-on-error report.tex
```

Primary artifacts:

- `DS-TWR_vs_FlexTDOA_raport_teren_2026-07-26.pdf`: final Romanian report;
- `analysis_summary.json`: machine-readable aggregate results;
- `measurement_metrics.csv`, `position_metrics.csv`, `block_metrics.csv`:
  tabular summaries;
- `figures/`: plots in vector PDF and PNG formats;
- `data/*.jsonl.xz`: complete official captures;
- `data/*.metadata.json`: runtime status and configuration for every block;
- `raw_data_manifest.csv`: SHA-256 hashes for raw and compressed files.

The short collector smoke tests remain local and are intentionally excluded
from version control.
