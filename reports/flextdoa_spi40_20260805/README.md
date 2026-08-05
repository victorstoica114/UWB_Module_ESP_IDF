# FlexTDOA SPI 40 MHz A/B/A field check - 2026-08-05

## Scope

This experiment checks whether running the ESP32-S3 to DW3000 SPI bus at an
actual 40 MHz improves the raw FlexTDOA pipeline compared with the fastest
datasheet-compliant ESP32-S3 divider, 26.666 MHz.

The 40 MHz setting is deliberately experimental: it is above the DW3000
datasheet maximum of 38 MHz. The code requires an explicit overclock opt-in,
keeps the 38 MHz limit visible, and still verifies the DW3000 device ID after
switching the bus clock.

## Fixed test conditions

- Five field modules, all RTK Fixed
- UWB channel 9
- FlexTDOA raw independent-frame solver; no position filter
- CFO correction: 48
- Frame timing: 250 / 2000 / 250 / 500 / 600 us
- Anchor IDs 2, 3, 4, 5; tag ID 1
- Same functional firmware source for the 26.666 and 40 MHz builds
- 15 s warm-up followed by 120 s acquisition for every run

## Accuracy and throughput

| Run | Actual SPI | Positions | Position rate | RMSE vs RTK | Bias | Precision RMS | P95 error |
|---|---:|---:|---:|---:|---:|---:|---:|
| A1 | 40 MHz | 3815 | 31.76/s | 3.29 cm | 2.65 cm | 1.95 cm | 4.96 cm |
| B | 26.666 MHz | 3847 | 32.07/s | 3.40 cm | 2.70 cm | 2.06 cm | 5.22 cm |
| A2 | 40 MHz | 3746 | 31.22/s | 3.43 cm | 2.82 cm | 1.95 cm | 5.26 cm |

The two 40 MHz runs average 31.49 positions/s and 3.36 cm RMSE. Within this
field session, that is equivalent to the 26.666 MHz control (32.07 positions/s
and 3.40 cm RMSE). The frame schedule, not SPI throughput, is the limiting
factor for the published position rate.

The approximately 3.3-3.4 cm RMSE shared by all three runs must not be
attributed to the SPI clock. It is dominated by a 2.65-2.82 cm session bias.
The older 1.94 cm capture was acquired in a different field session and is not
a valid clock-only control.

## Slot completeness and critical-path timing

The B and A2 acquisitions also captured the once-per-second FlexTDOA firmware
summaries.

| Metric | 26.666 MHz | 40 MHz |
|---|---:|---:|
| Complete slots | 18,902 | 18,768 |
| Incomplete slots | 851 | 902 |
| Complete fraction | 95.692% | 95.414% |
| RX good-to-rearm median | 34 us | 32 us |
| RX good-to-rearm P95 | 38 us | 35 us |
| Anchor arm maximum P95 | 205 us | 201 us |
| Request arm maximum P95 | 137 us | 134 us |

40 MHz reduced the internal receive/re-arm latency by roughly 2-4 us, as
expected. It did not reduce incomplete slots in this sample; the 0.278
percentage-point difference is small and is in the unfavorable direction.
Consequently, 40 MHz provides timing margin but no demonstrated FlexTDOA
throughput or completeness gain with the current frame profile.

## 400 us response-subslot follow-up

The timing margin was then used to shorten only the response subslot from 500
to 400 us. A same-session 500 us control separates the timing effect from the
slow RTK/UWB bias drift observed during the session. Both acquisitions used 40
MHz SPI, channel 9, raw independent frames, and no position filter.

| Response subslot | Positions | Position rate | Complete slots/s | Complete fraction | RMSE vs RTK | Bias | Precision RMS | P95 error |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 400 us | 3186 | 35.37/s | 167.39 | 96.676% | 4.20 cm | 3.68 cm | 2.03 cm | 6.06 cm |
| 500 us | 2867 | 31.64/s | 155.38 | 95.441% | 4.10 cm | 3.63 cm | 1.90 cm | 5.92 cm |

The 400 us profile raises the independent-frame position rate by 11.8% and the
complete-slot rate by 7.7%. Its extra 0.10 cm absolute RMSE is explained by
nearly identical session bias (3.68 versus 3.63 cm); the unfiltered precision
RMS changes by 0.13 cm. Solver residual RMS also remains effectively unchanged
(10.42 versus 10.27 cm). This is a useful throughput gain without a material
localization-precision regression in this field sample.

## Deployed result

All five modules were left on the 40 MHz experimental build with the validated
400 us response-subslot profile. The live dashboard reported for every module:

- `uwb_spi_clock_hz = 40000000`
- `uwb_status = ready`
- `uwb_device_id = 0xdeca0302`
- `runtime_mode_name = uwb_flex_tdoa`
- `runtime_radio_channel = 9`
- `runtime_flex_tdoa_response_subslot_us = 400`
- `runtime_flex_tdoa_response_process_us = 600`
- `boot_guard_validated = true`
- GPS fix quality `rtk_fixed`

Experimental binary:

- `build-spi40/uwb_esp_idf.bin`
- SHA-256 `A20B2CF2B72E42D3453ED2DF8006AF5F598578091B434F38A95AE96CEB116820`

26.666 MHz control binary:

- `build-ds-speed/uwb_esp_idf.bin`
- SHA-256 `43A0DC29CA7F4611002B58D6016D16612EE610DD2F606F4711A456B1F10781EC`

## Evidence

- `data/flextdoa_spi40_cfo48_frame12.jsonl` and matching metadata: A1
- `data/flextdoa_spi26666_control_cfo48_frame12.jsonl`: B
- `data/flextdoa_spi26666_control_logs.jsonl`: B timing summaries
- `data/flextdoa_spi40_confirm_cfo48_frame12.jsonl`: A2
- `data/flextdoa_spi40_confirm_logs.jsonl`: A2 timing summaries
- `flextdoa_spi40_summary.json`: A1 RTK analysis
- `flextdoa_spi26666_control_summary.json`: B RTK analysis
- `flextdoa_spi40_confirm_summary.json`: A2 RTK analysis
- `data/flextdoa_spi40_resp400.jsonl` and matching metadata: 400 us timing run
- `data/flextdoa_spi40_resp400_logs.jsonl`: 400 us timing summaries
- `flextdoa_spi40_resp400_summary.json`: 400 us RTK analysis
- `data/flextdoa_spi40_resp500_same_session.jsonl`: same-session 500 us control
- `data/flextdoa_spi40_resp500_same_session_logs.jsonl`: 500 us control timing summaries
- `flextdoa_spi40_resp500_same_session_summary.json`: 500 us control RTK analysis
