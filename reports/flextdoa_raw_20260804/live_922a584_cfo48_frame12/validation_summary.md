# FlexTDOA live RTK validation — firmware 922a584

Date: 2026-08-04  
Radio: DW3000 channel 9, 6.8 Mbps  
Timing: guard/request/request-process/response/response-process =
250/2000/250/500/600 us  
Geometry generation: 97, fixed GPS RTK ENU geometry  
Anchor order: 2,3,4,5  
Additive anchor corrections: 0,-112,-51,+9 mm

## Position result

The firmware publishes one independent AlgMin position from each complete
four-slot frame (12 directed observations). No position averaging, temporal
position filter, or residual-RMS rejection is active.

- Capture duration: 121.057 s after a 15 s warm-up
- Positions: 3,859
- Position rate: 31.878 positions/s
- RMSE versus median concurrent RTK reference: 1.939 cm
- RMSE versus nearest-in-time RTK fix: 1.942 cm
- Median radial error: 1.578 cm
- P95 radial error: 3.419 cm
- Mean bias: +0.493 cm east, -0.264 cm north
- Debiased RMSE: 1.856 cm

A second independent 30 s capture produced 1,009 positions and 1.930 cm
time-matched RTK RMSE (median 1.574 cm, P95 3.400 cm).

The previous firmware/capture reported 14.49 cm RMSE. The new live result is
an 86.6% reduction in radial RMSE.

## Frame completeness

The solver summaries inside the primary capture report:

- Complete frames: 3,858 (31.869/s)
- Incomplete frames: 1,343 (11.094/s)
- Complete-frame fraction: 74.18%
- Queue drops: 0

Only complete 12/12 frames publish a position. Incomplete frames are counted
but never solved or silently filled.

## CFO diagnostics

The CFO estimator is causal and separate per responder. It has an effective
window of 48 CFO samples per responder (about 16 frames / 0.4 s). It does not
average positions; every position uses only the current frame's observations.

The 30 s V2 diagnostic capture contains 11,991 observations. All were in the
ready/applied state, with no estimator reset during the capture.

| Responder | Raw CFO std (ppm) | Estimated CFO std (ppm) | Removed equivalent range std (cm) |
|---:|---:|---:|---:|
| 2 | 0.1102 | 0.0167 | 8.974 |
| 3 | 0.1206 | 0.0167 | 9.606 |
| 4 | 0.1282 | 0.0196 | 10.278 |
| 5 | 0.1258 | 0.0178 | 10.398 |

## Artifacts

- Primary events: `live_922a584_cfo48_frame12.jsonl`
- Primary metadata: `live_922a584_cfo48_frame12.metadata.json`
- CFO V2 events: `../live_922a584_cfo_diag30/live_922a584_cfo_diag30.jsonl`
- Firmware binary SHA-256:
  `777d98b6378c914a44123a10568a78a0f75cd54f083b8b0acb51bd3c5f7eae77`

The RTK tag was static. A later multi-point/motion validation is still needed
to demonstrate the same accuracy throughout the complete operating area.
