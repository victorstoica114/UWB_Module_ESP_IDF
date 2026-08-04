# UWB and GPS RTK comparison — initial static capture

This directory contains the first raw dataset for an extended comparison of:

- FlexTDOA;
- Native DS-TWR;
- Passive DS-TWR;
- GPS RTK.

## Capture conditions

- Date: 2026-08-03
- One static 300 s block per UWB protocol
- 15 s warm-up after every protocol change
- UWB dashboard polling: 50 Hz
- GPS sampling: at most 1 Hz per receiver, using the latest new ESP32 GGA event
- Modules: tag T1 and anchors A2, A3, A4, A5
- All five receivers had RTK FIX at the campaign preflight
- No filtering was added by the collector
- FlexTDOA was restored after the campaign

The physical setup was not independently surveyed for this capture. Therefore,
this dataset supports precision, repeatability, update-rate, delivery,
availability, geometry consistency, and cross-method agreement. It must not be
used to claim absolute accuracy. A surveyed point or another independent
reference is required for that claim; GPS RTK cannot be both the method under
test and its own ground truth.

## Blocks

| Block | Duration | Primary UWB observations | Positions published | GPS samples | Collector errors |
|---|---:|---:|---:|---:|---:|
| FlexTDOA | 300 s | 192,787 TDOA | 16,408 | 924 | 0 |
| Native DS-TWR | 300 s | 2,887 ranges | calculated later from coherent frames | 937 | 0 |
| Passive DS-TWR | 300 s | 37,320 passive observations | 7,839 | 932 | 0 |

Native DS-TWR ranges were collected through the lossless cursor-based dashboard
log stream. The Raspberry Pi position will be reconstructed from coherent
four-anchor frames during analysis, using the same raw range inputs as the live
solver.

## RTK availability observed in the sampled solutions

| Concurrent block | RTK FIX | Other valid fixes | No fix | RTK FIX share |
|---|---:|---:|---:|---:|
| FlexTDOA | 907 | 17 | 0 | 98.16% |
| Native DS-TWR | 902 | 34 | 1 | 96.26% |
| Passive DS-TWR | 925 | 7 | 0 | 99.25% |
| Total | 2,734 | 58 | 1 | 97.89% |

`Other valid fixes` includes transient RTK FLOAT and SPS samples. These samples
are intentionally retained so the final report can show both precision while
fixed and real-world solution availability.

## Files

- `campaign.json`: campaign order, conditions, preflight/final status, and block summaries;
- `data/*.jsonl.xz`: lossless XZ archives of the immutable raw events and
  sampled GNSS solutions (extract with `xz -dk <file>`);
- `data/*.metadata.json`: block configuration and exact event counts;
- `logs/*.jsonl`: protocol-specific diagnostic logs;
- `SHA256SUMS`: integrity hashes for the complete capture.

Every JSONL record contains `kind`, `protocol`, `block`, `captured_at`, and
`received_at`. GPS records additionally retain the ESP32 `gps_gga_count`, UTC,
coordinates, altitude, fix quality, satellites, HDOP, RTK age/ratio, NTRIP
state, and receiver diagnostics.

Verify the dataset from this directory with:

```bash
sha256sum -c SHA256SUMS
```

The checksum file validates the archived captures directly, so a fresh clone
does not need the much larger uncompressed JSONL files.

## Follow-up collection before the final report

Each additional test remains limited to five minutes per method:

1. surveyed static reference at the centre and at least one off-centre point;
2. repeatable dynamic route, with identical start/end marks and speed guidance;
3. optional LOS/range and controlled obstruction cases if field time permits.

The report should keep accuracy and precision separate. Its main outputs will
include bias/RMSE/CEP/P95, static dispersion, position and measurement rates,
latency, dropped/incomplete frames, RTK FIX availability, time-to-fix,
trajectory lag, cross-track error, and UWB/GNSS geometry agreement.
