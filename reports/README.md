# Report archive

This directory keeps the 13 final PDF reports directly at the top level. Each
PDF has one matching self-contained package under `bundles/<report>/`.

```text
reports/
  <report>.pdf
  PDF_SHA256SUMS
  bundles/
    RAW_SHA256SUMS
    <report>/
      README.md
      source/
      analysis/
      figures/
      raw/
```

Bundle contents:

- `source/`: TeX, Markdown, helper scripts and source-local configuration;
- `analysis/`: CSV/JSON results, manifests and historical derived data;
- `figures/`: retained PNG/SVG figures (intermediate plot PDFs were removed);
- `raw/`: retained RAW captures and adjacent metadata, with their original
  internal layout preserved.

`PDF_SHA256SUMS` verifies the final PDFs. `bundles/RAW_SHA256SUMS` verifies all
140 retained RAW and metadata files. Capture formats are stored through Git
LFS. Historical manifests inside `analysis/` may still record original paths
or captures removed by the aggressive retention pass.
