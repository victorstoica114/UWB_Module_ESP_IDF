# IMU fusion static validation — 2026-08-10

M1 was held stationary at the same physical position for all three captures.
All five modules ran firmware `06e0551-dirty`, built from the
`imu-fusion-finalize` worktree, with the BNO085 configured at 200 Hz and
dashboard telemetry limited to approximately 100 Hz per working IMU.

Before the final captures, the persisted anchor geometry was found to be stale:
its rigid fit against the current RTK anchor coordinates was 5.45 m.  The fixed
ENU geometry was therefore refreshed from 12 RTK-fixed samples per anchor and
written to all modules as generation 99:

| Anchor | East (m) | North (m) |
|---|---:|---:|
| A2 | 0.000 | 0.000 |
| A3 | 1.693 | 9.378 |
| A4 | 5.409 | 3.622 |
| A5 | -4.130 | 5.852 |

This eliminated the Passive DS-TWR solver failure immediately: its field log
changed from 53–99 rejected batches/s and almost no positions to 58–88 raw
positions/s with zero solver rejections.  The final captures below all use the
new geometry.

## Results

The cloud metrics are radial distances from each stream's own capture median.
They measure stationary precision independently of absolute RTK alignment.

| Protocol | Raw positions | Raw rate (Hz) | Gaps >100 ms | Raw cloud RMS (cm) | Fused cloud RMS (cm) | Raw P95 (cm) | Fused P95 (cm) | Raw / fused RMSE vs RTK (cm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| FlexTDOA | 3,652 | 30.41 | 8 | 2.74 | 0.46 | 4.72 | 0.71 | 11.80 / 11.35 |
| Passive DS-TWR | 8,800 | 73.33 | 141 | 2.32 | 0.57 | 4.05 | 1.02 | invalid RTK tag fix |
| Native DS-TWR | 2,715 | 22.57 | 0 | 2.48 | 0.44 | 4.25 | 0.70 | 10.37 / 10.12 |

The replay used 11,998–12,018 M1 IMU samples per 120 s capture.  Stationary
detection and zero-velocity updates were active for more than 99% of those
samples.  Static fusion therefore gives a repeatable reduction in jitter while
the raw position stream remains preserved for audit and comparison.

The Passive DS-TWR absolute RTK error must not be attributed to UWB.  During
that capture M1 reported a stable but false `RTK Fixed` cluster displaced by
about 3.31 m from both UWB and its own Fixed cluster before and after the next
reboot.  The anchor alignment itself was valid (3.4 mm RMS).  Native DS-TWR and
FlexTDOA were captured after M1 returned to the coherent RTK cluster.

Native DS-TWR accumulated 3,784 complete and 101 incomplete frames during its
boot/capture interval, or 2.60% incomplete, with zero invalid frames, zero CRC
errors, and zero rejected ranges.

## Hardware/runtime notes

- All five modules completed OTA, booted the same image, and validated their
  OTA partitions.
- M1, M3, M4, and M5 stream BNO085 data normally.
- M2 runs the same firmware but does not detect either BNO085 or MAX77958 on
  I2C (`ESP_ERR_NOT_FOUND`).  This was already present before this OTA and is a
  module-specific I2C/hardware fault, not a firmware-image mismatch.
- The modules were left online in FlexTDOA mode with geometry generation 99.

## Files

For each protocol, the directory contains the complete raw capture (`*.jsonl`),
capture summary, deterministic fusion replay, and per-position replay samples.
The raw streams contain UWB positions, IMU samples, RTK fixes, periodic module
status, and radio diagnostics needed for later reports.
