# Raw capture archive

The lossless event and firmware-log streams are committed as individual
Zstandard archives because the uncompressed five-hour dataset is 963 MB and
one source file exceeds GitHub's per-file size limit. The compressed archive
is 74 MB; every individual file remains below 11 MB.

Metadata sidecars remain uncompressed. To restore the exact files expected by
the analysis and report scripts, run from the repository root:

```bash
zstd --decompress --keep \
  reports/native_ds_round_boundary_20260731/data/*.jsonl.zst
```

Then regenerate the classification, figures, and PDF using the commands in
the parent directory's `README.md`. The local uncompressed files are ignored
by Git and are not removed by this workflow.
