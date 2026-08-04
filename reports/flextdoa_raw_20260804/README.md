# FlexTDOA raw field results — 2026-08-04

All measurements in this report are raw FlexTDOA/AlgMin results. No motion
filter, temporal averaging, decimation, or position rejection was applied.

## Test setup

- UWB channel: 9
- Tag: M1
- Anchors: M2, M3, M4, M5
- Fixed RTK ENU geometry, with M2 as the origin:
  - M2: `(0.000, 0.000)` m
  - M3: `(3.844, 2.923)` m
  - M4: `(-3.120, 3.899)` m
  - M5: `(1.055, 6.836)` m
- Static tag RTK reference: `(0.332, 3.392)` m
- The UWB and GNSS antenna centers coincide.
- Unless noted otherwise, each accuracy block is 60 seconds long.

The reported RTK RMSE is the horizontal position error:

`sqrt(mean((x_flex - x_rtk)^2 + (y_flex - y_rtk)^2))`

## Raw results

| Firmware / experiment | RESP / process (us) | Positions/s | Complete/s | Incomplete/s | Median error (cm) | RTK RMSE (cm) | P95 (cm) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Clean rewrite, paper reference | 250 / 600 | 56.9 | 56.8 | 70.2 | 10.7 | 13.7 | 23.9 |
| Clean rewrite, spacing sweep | 350 / 600 | 145.7 | 144.8 | 19.1 | 11.3 | 15.0 | 26.9 |
| Clean rewrite, spacing sweep | 400 / 600 | 158.7 | 159.0 | 9.5 | 11.3 | 15.0 | 27.0 |
| Clean rewrite, spacing sweep | 450 / 600 | 156.0 | 155.2 | 8.7 | 10.9 | 14.7 | 26.8 |
| Clean rewrite, spacing reference | 500 / 600 | 150.1 | 149.8 | 9.5 | 10.9 | 14.4 | 25.8 |
| Clean rewrite, equal-frame control | 250 / 850 | 62.3 | 61.9 | 54.7 | 10.8 | 13.7 | 24.2 |
| Early RX re-arm only | 250 / 600 | 121.9 | 121.7 | 37.4 | 10.8 | 14.5 | 25.8 |
| Strict CIADONE only | 250 / 600 | 56.4 | 56.5 | 69.6 | 10.4 | 13.5 | 24.1 |
| Strict CIADONE only | 500 / 600 | 155.1 | 154.4 | 7.6 | 11.1 | 14.4 | 25.5 |
| Combined RX re-arm + CIADONE | 250 / 600 | 122.9 | 122.7 | 36.6 | 10.7 | 14.1 | 25.0 |
| Combined RX re-arm + CIADONE | 500 / 600 | 155.2 | 154.4 | 7.4 | 10.8 | 14.4 | 25.7 |
| Combined RX re-arm + CIADONE | 1000 / 600 | 125.0 | 124.9 | 5.3 | 11.0 | 15.0 | 26.4 |
| Exact response deadline | 250 / 600 | 124.6 | 124.3 | 35.6 | 10.6 | 14.0 | 24.5 |
| Incomplete-slot fast-fail | 250 / 600 | 121.8 | 122.0 | 37.5 | 10.9 | 14.2 | 25.3 |
| SPI 26.666 MHz, in-spec | 500 / 600 | 153.9 | 153.5 | 8.0 | 10.9 | 14.5 | 26.4 |
| Qorvo RX status-order experiment (rejected) | 250 / 600 | 114.2 | 114.4 | 40.9 | 10.8 | 14.3 | 25.0 |
| Final integrated firmware `42325e3` | 500 / 600 | 157.35 | 157.50 | 6.39 | 11.07 | 14.49 | 25.95 |

Small differences between sequential 60-second captures must not be treated as
statistically significant without repeated/interleaved blocks. The large
availability changes are nevertheless unambiguous.

## Findings

1. Increasing only the total frame duration does not fix availability. The
   `250/850 us` control retained almost the same failure mode as `250/600 us`.
   Increasing the response-to-response spacing does fix it.
2. Early RX re-arm more than doubled complete slots at the paper timing, from
   about 57 to 122 positions/s, without a material accuracy regression.
3. At `250/600 us`, the remaining limit is the tag RX double-buffer processing
   path. `good -> CMD_RX` took at most 43 us, but the full metadata/payload,
   clear, release, and toggle path was approximately 240–293 us.
4. In the combined `250/600 us` capture, 1,893 of 2,196 incomplete slots
   (86.2%) missed only response index 2, the third response in the burst. CI-CR
   rotates the physical anchor occupying this index; this is primarily a
   subslot/pipeline symptom, not by itself evidence against one anchor.
5. Strict CIADONE validation is required for correctness, but it was neutral
   in these field tests. `cia_ready_after_retry` was zero, and only one final
   invalid CFO response occurred in the combined 250 us block.
6. Reducing the raw output rate by increasing spacing from 500 to 1000 us did
   not reduce RTK RMSE. It reduced incomplete slots but increased RMSE from
   approximately 14.4 to 15.0 cm. Simple rate relaxation is therefore not the
   next accuracy lever.
7. The current best raw operating compromise is response spacing 500 us and
   response processing 600 us. The paper timing remains useful as a stress and
   conformance profile.
8. Removing the old extra 1000 us response-collection grace was correct but
   only modestly improved the 250 us profile. Returning directly to RX after an
   incomplete burst did not improve it: complete throughput remained about
   122 positions/s and ordered-frame rejects increased to about 109/s. The
   remaining bottleneck is therefore in the DW3000 RX state/buffer service,
   before the partial observations reach AlgMin.
9. Replacing the out-of-spec 40 MHz SPI operation with the fastest ESP32-S3
   divider below the DWM3000 38 MHz limit (26.666 MHz actual) retained 153.9
   positions/s and 95.1% complete bursts at 500/600 us. Sequential 60-second
   accuracy differences were small and are not statistically resolved.
10. Reordering the RX-good clear, `CMD_RX`, payload copy, RDB release and
    double-buffer toggle to follow a generic Qorvo ISR sequence was tested on
    all five modules and rejected. It reduced complete throughput to 114.4/s,
    increased incomplete bursts to 40.9/s and increased ordered-frame rejects
    to 122.4/s at 250/600 us. The final firmware reverts this experiment.
11. The final 60-second validation of `42325e3` at 500/600 us produced 9,455
    raw positions: 157.35 positions/s, 96.10% complete bursts, 14.49 cm RTK
    RMSE and 25.95 cm P95. CFO-invalid responses were zero. Maximum measured
    early RX re-arm time was 41 us and maximum full RDB service time was 293 us.

## Final validated live state

- Firmware: `42325e3` (clean rewrite, early RX re-arm, strict CIADONE, exact
  deadline, incomplete-slot fast-fail and in-spec SPI clock; rejected RX
  status-order experiment reverted)
- Channel: 9
- Runtime FlexTDOA timing: `guard=250`, `request=2000`,
  `request_process=250`, `response=500`, `response_process=600` us
- UWB PHY data rate: 6.8 Mbit/s
- ESP32-S3 to DWM3000 SPI clock: 26.666 MHz actual (32 MHz requested), below
  the DWM3000 38 MHz maximum
- All five OTA boots validated successfully.
- No antenna-delay changes from the channel-9 RTK dry run were applied.

Protocol changes require a coordinated reboot. Timing changes also use a
coordinated reboot for now because the current live configuration update is not
atomic at a frame boundary.
