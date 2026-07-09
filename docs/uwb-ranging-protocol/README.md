# UWB Ranging Protocol

This document explains how the current firmware performs UWB ranging with the
DW3000 radio. It focuses on the active `uwb_ranging` mode used in the lab:
module `1` is the tag/initiator and modules `2`, `3`, `4`, and `5` are anchors.

The implementation is based on Double-Sided Two-Way Ranging (DS-TWR). The
classic DS-TWR exchange uses three UWB frames: `POLL`, `RESP`, and `FINAL`.
This firmware adds two verification frames:

- `REPORT`: the tag sends its hardware timestamps to the anchor.
- `REPORT2`: the anchor sends its hardware timestamps and calculated distance
  back to the tag.

The anchor still calculates and logs the normal ranging result. The tag then
recalculates the same distance locally and logs a verification comparison.

## Current Runtime Shape

The same firmware image runs on every module. At boot, each board reads its
persistent module ID and runtime config from NVS.

| Module ID | Runtime behavior |
| --- | --- |
| `1` | Tag / initiator |
| `2` | Anchor / responder |
| `3` | Anchor / responder |
| `4` | Anchor / responder |
| `5` | Anchor / responder |

The tag ranges against the anchors sequentially:

```text
round N

module 1 -> anchor 2
module 1 -> anchor 3
module 1 -> anchor 4
module 1 -> anchor 5

wait round_gap_ms
round N + 1
...
```

The important runtime values are exposed by `/status` and can be changed with
`/config/runtime`. The firmware defaults are:

| Name | Current value | Meaning |
| --- | ---: | --- |
| `ranging_slot_ms` | `350 ms` | Delay inserted by the tag after each anchor attempt |
| `ranging_round_gap_ms` | `500 ms` | Delay after all anchors in a round have been attempted |
| `ranging_rx_slice_ms` | `100 ms` | Passive RX window used by each anchor while waiting for a `POLL` |
| `dt_rx_timeout_ms` | `250 ms` | Max wait for expected DS-TWR frames |
| `dt_resp_delay_ms` | `20 ms` | Scheduled delay from `POLL RX` to `RESP TX` on the anchor |
| `dt_final_delay_ms` | `20 ms` | Scheduled delay from `RESP RX` to `FINAL TX` on the tag |
| `dt_report_delay_ms` | `10 ms` | Software delay before `REPORT TX`, and before anchor `REPORT2 TX` |
| `dt_auto_rx_delay_uus` | `500 UUS` | Hardware delay after a TX before DW3000 opens RX |

The `dt_*` names come from the older distance-test runtime, but the current
multi-anchor ranging mode reuses the same DS-TWR implementation and therefore
uses the same parameters.

## Units

The DW3000 uses a few time units that are easy to confuse:

| Unit | Meaning |
| --- | --- |
| `ms` | Normal milliseconds used by FreeRTOS and runtime config |
| `UUS` | UWB microseconds; `1 UUS = 512 / 499.2 MHz = 1.025641 us` |
| `DTU` | DW3000 device time unit; about `15.650040064 ps` |

Useful conversions:

| Value | Approximate real time |
| --- | ---: |
| `500 UUS` | `512.82 us`, or `0.513 ms` |
| `20 ms` | `1,277,952,000 DTU` |
| `10 ms` | `638,976,000 DTU` |

`UUS` is used for the DW3000 automatic TX-to-RX wait. The larger `20 ms` DS-TWR
turnaround delays are converted to DTU and programmed as delayed TX timestamps.

## Message Sequence

One ranging attempt between tag `1` and one anchor looks like this:

```text
Tag / initiator, module 1                         Anchor / responder, module N

T1: POLL TX  ------------------------------------>
                                                   T2: POLL RX

                                                   schedule RESP at:
                                                   T3 = T2 + dt_resp_delay_ms

                                                   T3: RESP TX
       auto RX opens after 500 UUS  <-------------
T4: RESP RX

schedule FINAL at:
T5 = T4 + dt_final_delay_ms

T5: FINAL TX  ----------------------------------->
                                                   auto RX opens after 500 UUS
                                                   T6: FINAL RX

wait dt_report_delay_ms

REPORT TX  -------------------------------------->
                                                   REPORT RX
                                                   calculate distance
                                                   log UWB_RANGING result

                                                   wait dt_report_delay_ms

                                 <---------------- REPORT2 TX:
                                                   T2, T3, T6,
                                                   anchor distance

REPORT2 RX
calculate distance on tag
compare tag vs anchor result
```

There are two different kinds of delay in the diagram:

| Delay | Programmed where | Purpose |
| --- | --- | --- |
| `dt_resp_delay_ms = 20 ms` | Anchor delayed TX | Sets the exact `RESP` transmit timestamp after receiving `POLL` |
| `dt_final_delay_ms = 20 ms` | Tag delayed TX | Sets the exact `FINAL` transmit timestamp after receiving `RESP` |
| `dt_report_delay_ms = 10 ms` | Tag/anchor software delay | Gives a small gap before sending `REPORT` or `REPORT2` |
| `dt_auto_rx_delay_uus = 500 UUS` | DW3000 auto RX-after-TX | Prevents RX from opening immediately after TX |

The critical `RESP` and `FINAL` instants are owned by the DW3000 radio through
delayed TX. The firmware computes the target timestamp, writes `DX_TIME`, and
issues `DTX` or `DTX_W4R`. That keeps the timing stable even if FreeRTOS has
scheduler jitter.

## Frame Table

| Frame | Sent by | Received by | Purpose |
| --- | --- | --- | --- |
| `POLL` | Tag | Anchor | Starts one ranging exchange; tag captures `T1`, anchor captures `T2` |
| `RESP` | Anchor | Tag | Confirms the anchor received `POLL`; anchor captures `T3`, tag captures `T4` |
| `FINAL` | Tag | Anchor | Completes the DS-TWR timing triangle; tag captures `T5`, anchor captures `T6` |
| `REPORT` | Tag | Anchor | Carries tag timestamps so the anchor can compute distance |
| `REPORT2` | Anchor | Tag | Carries anchor timestamps and anchor distance so the tag can verify locally |

The anchor already knows its own receive/transmit timestamps:

| Timestamp | Side | Event |
| --- | --- | --- |
| `T1` | Tag | `POLL` transmitted |
| `T2` | Anchor | `POLL` received |
| `T3` | Anchor | `RESP` transmitted |
| `T4` | Tag | `RESP` received |
| `T5` | Tag | `FINAL` transmitted |
| `T6` | Anchor | `FINAL` received |

The tag includes `T1`, `T4`, and `T5` in `REPORT`. The anchor combines those
with `T2`, `T3`, and `T6`, calculates the distance, then sends `T2`, `T3`,
`T6`, and the anchor-calculated distance back in `REPORT2`.

The tag already still has `T1`, `T4`, and `T5` locally. After `REPORT2`, it has
all six timestamps too, so it recalculates and logs:

```text
DS-TWR tag verify ... tag=<distance> anchor=<distance> diff=<cm>
```

## Distance Calculation

The uncorrected DS-TWR calculation uses these intervals:

```text
round_a = T4 - T1   tag sees POLL TX -> RESP RX
reply_a = T5 - T4   tag waits RESP RX -> FINAL TX

reply_b = T3 - T2   anchor waits POLL RX -> RESP TX
round_b = T6 - T3   anchor sees RESP TX -> FINAL RX
```

Then:

```text
tof_dtu = (round_a * round_b - reply_a * reply_b)
          / (round_a + round_b + reply_a + reply_b)

distance_m = tof_dtu * 15.650040064 ps * 299702547 m/s
```

With `APP_UWB_DISTANCE_TEST_CLOCK_OFFSET_CORRECTION = 1`, the firmware uses the
DW3000 clock offset measurement from the received frame to compensate some
clock drift before converting time-of-flight to meters. The log line still
prints both values:

```text
DS-TWR distance ... distance=<corrected> raw=<uncorrected> clk_valid=1
```

## TX-to-RX and RX-to-TX Timing

The two most common timing questions are:

| Question | Current answer |
| --- | --- |
| How long after TX until RX opens? | `500 UUS`, about `0.513 ms` |
| How long after RX until the next TX? | `20 ms` for `RESP`, `20 ms` for `FINAL` |

The TX-to-RX delay is a DW3000 auto-receive setting. It applies after:

```text
POLL TX  -> tag opens RX for RESP
RESP TX  -> anchor opens RX for FINAL
```

The RX-to-TX delays are delayed-TX timestamps. They apply after:

```text
POLL RX  -> anchor schedules RESP TX at +20 ms
RESP RX  -> tag schedules FINAL TX at +20 ms
```

So the system is not trying to turn around instantly. The radio has about half
a millisecond before RX opens after a TX, and about 20 ms between receiving one
DS-TWR frame and transmitting the next scheduled DS-TWR frame.

## Antenna Delay

Antenna delay is separate from the protocol delays above. It is not a sleep,
slot, or turnaround time. It is a calibration value written to the DW3000 RX
and TX antenna delay registers during radio init.

The firmware reads the active antenna delay from persistent identity NVS. If no
calibrated value exists, it falls back to `APP_UWB_ANTENNA_DELAY_DEFAULT`.

Why it matters:

- DS-TWR depends on timestamp accuracy.
- The DW3000 timestamp is corrected by the programmed antenna delay.
- Wrong antenna delay shifts the absolute distance even if packet success is
  perfect.

Use `/status` to inspect the live values:

```text
uwb_active_antenna_delay
uwb_active_antenna_delay_hex
uwb_antenna_delay_from_nvs
```

## What Common Log Failures Mean

| Log pattern | Meaning |
| --- | --- |
| `DS-TWR RESP wait failed` | Tag sent `POLL` but did not receive a matching `RESP` before timeout |
| `DS-TWR FINAL wait failed` | Anchor sent `RESP` but did not receive the tag's `FINAL` before timeout |
| `DS-TWR REPORT wait failed` | Anchor received `FINAL` but did not receive the final timestamp report |
| `DS-TWR REPORT2 wait failed` | Anchor calculated the result, but the tag did not receive the verification report |
| `UWB distance RX error` | DW3000 reported a PHY/RX error instead of a valid frame |
| `UWB_RANGING result` | Full exchange completed; distance was calculated by the anchor |
| `DS-TWR tag verify` | Tag recalculated the same exchange and compared against anchor result |

## Code Map

| Area | File |
| --- | --- |
| Runtime mode selection | `components/app_manager/app_manager.c` |
| Runtime config defaults/NVS | `components/config/app_runtime_config.c` |
| Ranging service entry point | `components/uwb_ranging_service/uwb_ranging_service.c` |
| Tag loop and anchor loop | `components/uwb_dw3000/uwb_dw3000.c` |
| DS-TWR initiate/respond implementation | `components/uwb_dw3000/uwb_dw3000.c` |
| Firmware timing defaults | `components/config/include/uwb_config.h` |
| Antenna delay identity storage | `components/config/app_identity.c` |

The most useful functions when reading the code are:

```text
uwb_ranging_tag_loop()
uwb_ranging_anchor_loop()
uwb_distance_initiate_once()
uwb_distance_respond_to_poll()
uwb_dw3000_send_payload_delayed()
uwb_dw3000_send_payload_delayed_expect_rx()
uwb_distance_tof_dtu()
uwb_distance_tof_dtu_clock_corrected()
```
