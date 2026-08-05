# FlexTDOA loss reduction - 2026-08-05

## Result

The deployed FlexTDOA profile remains raw and unfiltered. It uses channel 9,
40 MHz SPI, K=3, a relaxed 34 ms frame (1000 us response and 1000 us response
processing subslots), coordinated CI-CR responder rotation, conservative
10/12 partial-frame recovery, and the fast 6.8 Mb/s PHY with a 256-symbol
preamble and PAC16.

The final 120 s run produced 28.82 positions/s, 2.290% incomplete radio slots
and an estimated 0.580% loss per received packet. Its unfiltered position RMSE
against the static RTK reference was 2.289 cm, with 1.807 cm debiased RMS and
3.895 cm P95. The apparent slot loss is larger because one complete K=3 slot
requires one REQUEST and three RESPONSES; a single lost packet makes that slot
incomplete.

## Changes retained

- SPI requests 40 MHz by default. This deliberately exceeds the DW3000 data
  sheet maximum of 38 MHz; the exception, validated ceiling and boot-time
  device-ID verification remain explicit in the driver.
- CI-CR now rotates the physical responder order independently across frames.
  Missing response indices are consequently balanced instead of repeatedly
  assigning one anchor to the final response position.
- The campaign firmware reported PHE, FCE, FSL, CIAERR and ARFE separately,
  plus exact request gaps inferred from the FlexTDOA slot sequence. Those
  deep diagnostics were removed after the fault-isolation campaign; the
  compact production summary retains aggregate RX errors and request gaps.
- An incomplete slot publishes every valid raw observation it contains. The
  local solver accepts only 10/12 or 11/12 frames; 9/12 frames remain rejected
  because they can represent a completely missed request/initiator slot.
- Every partial solution must include all four initiators, connect the full
  anchor graph, have full-rank 2-D normal equations, condition number at most
  10 and residual RMS at most 0.20 m. No temporal position filter is used.
- The live NVS radio profile is PHY mode 2: channel 9, 6.8 Mb/s, preamble 256,
  PAC16, Qorvo 8-symbol SFD. The final diagnostic profile leaves a 1000 us
  response subslot and a separate 1000 us processing subslot.
- Sub-millisecond receive deadlines and diagnostic execution times use the
  existing 1 MHz GPTimer. A FreeRTOS tick is 10 ms in this build and is used
  only as a safety backstop, not as the requested deadline.
- A REQUEST received while the tag is closing an incomplete burst is deferred
  and processed as the next slot instead of being discarded as a RESP mismatch.

## Definitive relaxed-profile diagnosis

The final PHY-256 run covered 14,235 scheduled slots:

| Metric | Result |
|---|---:|
| Complete slots | 13,909 |
| Incomplete slots after a decoded REQUEST | 258 |
| Missed REQUEST slots | 68 |
| Raw incomplete-slot rate | 2.290% |
| Decoded RESPONSES / expected after decoded REQUESTS | 42,240 / 42,501 |
| Estimated lost radio packets | 329 / 56,736 (0.580%) |
| Positions | 3,470 (28.82/s) |
| RTK RMSE / debiased RMS / P95 | 2.289 / 1.807 / 3.895 cm |

The loss is not caused by a full DW3000 double buffer or by an anchor failing
to transmit:

- `OVR=0`, `rdb_resync=0` and `rdb_both_good=0` for the full run;
- 56,802 IRQ events had no polling fallback; only 44 exceeded 200 us;
- 190 tag PHY errors were 153 FSL/RSE, 27 PHE and 10 FCE;
- the 68 missed REQUESTS were followed by 203 orphan RESPONSES, or 2.99 per
  missed REQUEST. The anchors therefore decoded those REQUESTS and responded;
  the tag failed to acquire the REQUEST frame;
- missed REQUESTS were distributed A2/A3/A4/A5 = 12/22/20/14, and burst PHY
  errors by physical responder = 15/26/21/24. Rotation is active and no single
  anchor explains the failures.

The 0.580% packet-loss estimate predicts a complete four-packet slot rate of
`(1 - 0.00580)^4 = 97.70%`; the measured complete-slot rate was 97.71%. This
near-exact match shows that the remaining 2.29% incomplete slots are the
ordinary accumulation of small per-packet PHY losses, not an additional
frame-level software failure or loss cascade.

Physical response loss was A2 28/10,620 (0.264%), A3 73/10,630 (0.687%), A4
110/10,628 (1.035%) and A5 50/10,623 (0.471%). A4 is the weakest live response
link, but the isolated M1-A4 control below shows that it is not a failed anchor.

## Isolated M1-A4 control

The legacy distance-test frames initially failed before RF transmission. They
were always written as 64-byte payloads; with SPI DMA disabled, the one-byte
DW3000 header made a 65-byte ESP-IDF transaction and returned
`ESP_ERR_INVALID_ARG`. POLL/RESP/FINAL now send 10 bytes, REPORT 35 bytes and
REPORT2 48 bytes.

A second deterministic fault then appeared: RX-quality logging before arming
FINAL made every fifth delayed TX late. Moving that logging after the exchange
and replacing both nominal 1 ms REPORT gaps with a GPTimer alarm removed all
FINAL TX errors. With a 50 ms control interval, M1-A4 achieved:

| Metric | Result |
|---|---:|
| Complete exchanges | 787 / 798 (98.622%) |
| Approximate loss per five-packet exchange | 1.378% |
| Approximate loss per radio packet | 0.28% |
| Combined RX PHY errors | 6 in more than 4,200 receptions |
| Double-buffer overflow | 0 |
| Raw distance mean / standard deviation | 3.5425 m / 1.33 cm |

The 10 ms stress run also proved that synchronous diagnostic logging can
create its own loss: 76 of 160 RESP timeouts occurred immediately after the
three quality-log records emitted every fifth sequence.

## Relaxed PHY-256 versus PHY-512

Both runs used the same 34 ms frame, channel, geometry and solver. Only the
preamble changed.

| PHY | Preamble | Packet loss | Slot loss | Positions/s | RMSE | P95 |
|---|---:|---:|---:|---:|---:|---:|
| 2 | 256 | 0.580% | 2.290% | 28.82 | 2.289 cm | 3.895 cm |
| 3 | 512 | 0.808% | 3.098% | 28.43 | 2.314 cm | 4.223 cm |

PHY 3 also increased FSL/RSE from 153 to 221 and missed REQUESTS from 68 to
116. It was rejected and all modules were restored to PHY 2.

## Conservative partial-frame result

The first 120 s run used the 128-symbol PHY and compares strict full frames
with the exact same capture after 10/12 recovery.

| Solver policy | Positions | Rate | RMSE vs static RTK | Bias | Precision RMS | P95 |
|---|---:|---:|---:|---:|---:|---:|
| Strict 12/12 subset | 4,073 | 33.89/s | 3.995 cm | 3.522 cm | 1.885 cm | 5.708 cm |
| 10/12 or better | 4,301 | 35.78/s | 4.042 cm | 3.556 cm | 1.922 cm | 5.791 cm |

The partial path recovered 228 positions, a 5.6% throughput increase. The
changes of 0.047 cm RMSE and 0.037 cm precision RMS are negligible in this
field sample. Nine positions used 10 observations and 219 used 11; all gate
rejection counters remained zero.

## Preamble A/B/A

All runs used the same binary, channel, timing, geometry, corrections and
partial-frame policy. A radio-PHY change was applied to all five modules at
once and followed by a reboot and boot-guard validation.

| Run | Preamble | Complete | Incomplete | Request gaps | Real loss | PHE | FCE | FSL | Positions | Rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A1 | 128 | 19,977 | 792 | 1,230 | 9.191% | 361 | 22 | 562 | 4,301 | 35.78/s |
| B | 256 | 20,502 | 526 | 789 | 6.027% | 96 | 22 | 489 | 4,695 | 39.06/s |
| A2 | 128 | 19,964 | 794 | 1,242 | 9.255% | 348 | 25 | 688 | 4,294 | 35.72/s |

Relative to the mean of A1 and A2, preamble 256 reduced:

- real slot loss by 3.196 percentage points, or 34.7% relative;
- request gaps by 36.2%;
- PHE by 72.9%;
- total tag RX errors by 39.5%.

The time-matched RTK metrics were 4.045 cm RMSE / 1.962 cm precision RMS for
A1, 2.260 cm / 2.000 cm for B, and 3.978 cm / 3.069 cm for A2. Absolute bias
and spread drifted during the session, so they are secondary to the radio
counters for this PHY decision; importantly, B did not show an accuracy
regression.

## Experiments rejected

- Targeted RX-status clearing before every early re-arm did not reduce loss
  and increased critical-path latency, so it was removed.
- K=2 increased output to about 44.1 positions/s and slightly reduced loss,
  but precision RMS worsened from about 2.18 cm to 4.32 cm, so K=3 remains.
- Accepting all 9/12 frames recovered more positions but raised precision RMS.
  A 10/12 threshold preserves all four initiators and was retained instead.
- During the early compact-frame sweep, increasing the response subslot from
  400 to 500 us alone did not help. The final live profile intentionally uses
  a much more relaxed 1000/1000 us response/processing pair while diagnosing
  the residual PHY loss.

## Campaign build and current live state

The diagnostic binary used to produce this report remains on the RPi as:

`/home/pi/Documents/UWB/build-ds-rewrite/uwb_esp_idf_flex_loss_diag_gpt_v4.bin`

SHA-256:

`be360ae75507a3a8e3a551955fcb7c6ee3c5a55cf36628cfb5fb0c95ed7b895c`

After the campaign, the temporary IRQ-to-poll, RX-phase, subslot, EVC,
double-buffer timing and missing-mask instrumentation was removed. The
protocol timing, CFO correction, partial-frame recovery, precise GPTimer
deadlines, deferred REQUEST handling, RDB recovery and solver gates were kept.
The cleanup binary deployed on all five modules is:

`/home/pi/Documents/UWB/build-ds-rewrite/uwb_esp_idf_flextdoa_cleanup_canary.bin`

SHA-256:

`06497720d2d86a3c0ce4adaaf7794e7c3d1087828c33ba2f86f39abe1e44eb54`

Expected live state on all five modules:

- `runtime_mode_name = uwb_flex_tdoa`
- `runtime_radio_channel = 9`
- `runtime_radio_phy_mode = 2`
- `uwb_radio_profile = 6`
- `uwb_spi_clock_hz = 40000000`
- `runtime_flex_tdoa_responder_count = 3`
- `runtime_flex_tdoa_guard_us = 250`
- `runtime_flex_tdoa_request_subslot_us = 2000`
- `runtime_flex_tdoa_request_process_us = 250`
- `runtime_flex_tdoa_response_subslot_us = 1000`
- `runtime_flex_tdoa_response_process_us = 1000`
- `boot_guard_validated = true`

## Evidence

- `data/flextdoa_spi40_resp400_k3_partial10.*`: A1 and partial-frame control
- `data/flextdoa_spi40_resp400_k3_partial10_plen256.*`: B
- `data/flextdoa_spi40_resp400_k3_partial10_plen128_a2.*`: A2
- `data/flextdoa_spi40_resp400_cr_rotation_only.*`: rotation-only control
- `data/flextdoa_spi40_resp400_cr_rxclear_all.*`: coordinated RX-clear test
- `data/flextdoa_spi40_resp400_k2_rotation.*`: K=2 test
- `data/flextdoa_spi40_phy256_resp1000_proc1000_irq_rdb_diag_final.*`: final
  relaxed PHY-256 run and GPTimer/RDB/PHY diagnosis
- `data/flextdoa_spi40_phy512_resp1000_proc1000_irq_rdb_ab.*`: rejected
  same-timing PHY-512 A/B run
- `data/flextdoa_pair_m1_a4_ch9_phy256*`: isolated pair fault isolation,
  GPTimer correction and relaxed RF control
