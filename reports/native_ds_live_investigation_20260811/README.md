# Native DS-TWR live investigation — 2026-08-11

This directory archives the raw static baseline used during the Native DS-TWR
live investigation.

- `data/*.jsonl.xz` is the lossless XZ-compressed capture tracked through Git
  LFS.
- `raw_data_manifest.csv` records the original and archived sizes and SHA-256
  digests.
- `SHA256SUMS` contains the digest of the compressed artifact.
- The summary JSON remains directly readable without extracting the capture.

Restore the capture with:

```powershell
7z x data\static_baseline_01.native_ds.jsonl.xz
```

The uncompressed JSONL file remains available locally and is ignored by Git to
avoid storing duplicate data.
