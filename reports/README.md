# Report archive layout

The report archive is split into three layers:

- `pdfs/` contains the 13 unique final reports, with normalized filenames;
- `raw/<report>/` contains retained RAW captures and adjacent capture metadata;
- the other report directories contain derived tables, figures, summaries,
  Markdown notes and report sources.

Plot PDFs remain next to their report sources. Only final report PDFs are
collected in `pdfs/`.

RAW files keep their original relative layout below the report name. Paths in
analysis tools point to this central archive. Large JSONL formats are stored
through Git LFS.

The central `raw/SHA256SUMS` and `pdfs/SHA256SUMS` files are authoritative for
the reorganized archive. Per-report manifests remain as historical provenance
and can contain the original capture paths or entries removed by the aggressive
retention pass.
