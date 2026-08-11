# IMU fusion validation captures — 2026-08-11

This directory archives the field captures and replay results used for the final
IMU-fusion validation on FlexTDOA, Native DS-TWR, and Passive DS-TWR.

- `data/*.jsonl.xz` contains lossless XZ-compressed copies of the raw JSONL
  captures and replay streams. These files are tracked through Git LFS.
- `raw_data_manifest.csv` maps every archive to its original filename, byte
  count, and SHA-256 digest.
- `SHA256SUMS` contains SHA-256 digests for the compressed artifacts.
- The JSON files in this directory are derived summaries and sweep results.

Restore a capture with 7-Zip, for example:

```powershell
7z x data\flextdoa_dynamic_final_01.flextdoa.jsonl.xz
```

The uncompressed JSONL files remain available in the local workspace but are
ignored by Git to avoid adding several gigabytes of duplicate data.
