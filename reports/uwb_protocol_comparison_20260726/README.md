# DS-TWR versus FlexTDOA field comparison

Controlled LOS capture performed on 2026-07-26 with four anchors in a surveyed
3.000 m square and the tag fixed at the center.

Comparative metrics use ground-truth-independent structural validation:

- a FlexTDOA range difference must satisfy
  `abs(z_ij) <= anchor_separation + 0.01 m`;
- an embedded FlexTDOA position must report an internal solver RMS no greater
  than the 4.243 m anchor-array diagonal.

This excludes 4 of 289,716 observations and 1 of 9,075 embedded positions.
The raw captures remain immutable, and the complete embedded-position CSV
retains every row with `comparison_valid` and `exclusion_reason` fields.

The repository stores the six raw JSONL captures as lossless XZ archives so the
complete experiment remains below GitHub's normal file-size limits. Restore the
collector output before rerunning the analysis:

```bash
xz --decompress --keep reports/raw/uwb_protocol_comparison_20260726/data/*.jsonl.xz
cd ../..
python3 tools/uwb_compare_analyze.py
cd reports/uwb_protocol_comparison_20260726
latexmk -pdf -interaction=nonstopmode -halt-on-error report.tex
```

Primary artifacts:

- [`../pdfs/uwb_protocol_comparison_20260726.pdf`](../pdfs/uwb_protocol_comparison_20260726.pdf): final English report;
- `analysis_summary.json`: machine-readable aggregate results;
- `measurement_metrics.csv`, `position_metrics.csv`, `block_metrics.csv`:
  tabular summaries;
- `figures/`: plots in vector PDF and PNG formats;
- `../raw/uwb_protocol_comparison_20260726/data/*.jsonl.xz`: complete official captures;
- adjacent `*.metadata.json`: runtime status and configuration for every block;
- `raw_data_manifest.csv`: SHA-256 hashes for raw and compressed files.

The short collector smoke tests remain local and are intentionally excluded
from version control.
