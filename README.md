# UWB ESP-IDF

ESP-IDF project for ESP32-S3-WROOM-1-N16R8.

Current step:

- ESP-IDF v6.0.2 project skeleton
- custom 16 MB OTA partition table
- board, application, and UWB settings in `components/config`
- verified status LED blink on GPIO42 via `components/app_manager`
- Wi-Fi STA connection via `components/wifi_service`
- local HTTP OTA via `components/ota_service`
- bootloop recovery guard and ESP-IDF OTA rollback via `components/boot_guard`
- authenticated HTTP runtime configuration via `components/ota_service`
- DW3000 UWB SPI/reset bring-up and random TX/RX beacon smoke test via `components/uwb_dw3000`
- DW3000 hardware RXOK/SFD/RX/TX LED blink configured once at radio init
- first DS-TWR two-module distance test runtime
- antenna delay calibration runtime for two-module and three-module setups
- anchor-initiated 1-tag/4-anchor DS-TWR ranging runtime with dashboard
  position view
- optional BNO085 accelerometer hardware test via `components/bno085_service`

The board boot log confirms 16 MB QIO flash, 8 MB octal PSRAM at 80 MHz, and
the app running from the `ota_0` partition.

Wi-Fi SSID/password and tokens are read from `secrets.h`. Non-secret
application settings, including provisioning switches, live in `components/config`.
Use `secrets.example.h` as the template for local credentials.

## Hardware

This firmware targets the custom UWB Module V1.0 Rev.B hardware.

| Area | Details |
| --- | --- |
| MCU | ESP32-S3-WROOM-1-N16R8 |
| Flash / PSRAM | 16 MB flash, 8 MB PSRAM |
| USB | Native USB CDC on boot |
| UWB | DW3000/DWM3000 radio path |
| IMU | BNO085 over shared I2C |
| GPS | PX1105R/PX1125R-class GPS path |
| USB-C / PD | MAX77958 |
| Charger | BQ25792 |
| Status LED | GPIO42, active-high |

Active pin mapping lives in `components/config/include/board_config.h`.

| Function | GPIO |
| --- | --- |
| Status LED | `42` |
| I2C SDA / SCL | `9` / `10` |
| BNO085 reset / interrupt | `40` / `15` |
| UWB reset / IRQ / CS / wakeup | `8` / `6` / `48` / `7` |
| SPI MOSI / SCK / MISO | `11` / `12` / `13` |
| GPS enable / RX / TX | `47` / `18` / `17` |
| RTCM TX | `1` |

Project layout:

```text
main/                         app_main entry point
components/config/            board, app, and UWB configuration headers
components/app_manager/       app startup and status LED behavior
components/boot_guard/        OTA rollback confirmation and bootloop recovery
components/wifi_service/      Wi-Fi STA connection
components/ota_service/       local authenticated HTTP OTA
components/wireless_log_service/  TCP wireless mirror for ESP-IDF logs
components/bno085_service/    optional BNO085 accelerometer hardware test
components/uwb_dw3000/        DW3000 bring-up, beacon smoke test, DS-TWR loop
components/uwb_calibration_service/  antenna delay calibration entry point
components/uwb_distance_test_service/  two-module distance test entry point
components/uwb_ranging_service/  anchor/tag ranging entry point
docs/                         protocol notes and operator documentation
PCB/V1.REV.B/                 KiCad source, fabrication, BOM, and placement package
Schematic/                    exported schematic PDF
datasheets/                   local component/reference datasheets
reference/                    migration notes and legacy headers kept in-tree
reference/external/           optional local clones of third-party references
```

`components/config/include/board_config.h` is the active board pin map.
Old GPIO naming is kept only under `reference/legacy_headers`.
Old legacy project files and cloned third-party repositories are kept local for
inspiration/debugging and are intentionally ignored by Git.

`components/config/include/app_config.h` holds versioned non-secret application
settings: the default runtime mode, persistent identity provisioning, Wi-Fi
diagnostics/reconnect behavior, wireless-log defaults, OTA-adjacent
service defaults, and optional sensor-test parameters. `components/config/include/uwb_config.h`
holds UWB-only settings: role, source ID override, antenna delay, beacon
smoke-test parameters, and runtime defaults. Runtime overrides are stored in
NVS through `/config/runtime`. Keep Wi-Fi credentials and OTA tokens in
`secrets.h`.

Runtime ownership is intentionally narrow:

- `main/app_main.c` only boots the firmware and calls `app_manager_start()`.
- `components/app_manager/app_manager.c` owns what runs on the board and in
  what order.
- Long-lived workflows live in separate services/components, then are selected
  or sequenced by `app_manager`.

The application task/core split is kept simple so the UWB loop is isolated from
most Wi-Fi and TCP work:

| Task / service | Core | Notes |
| --- | --- | --- |
| `uwb_dw3000` | 1 | Owns DW3000 init, SPI access, RX/TX, ranging, survey, and calibration loops. |
| `status_led` | 1 | Lightweight GPIO blink task. |
| `boot_guard` | unpinned | Confirms stable boots after Wi-Fi/OTA are online and holds recovery state across resets. |
| `wifi_service` | 0 | Owns Wi-Fi STA connect/reconnect management. |
| `ota_service` / `httpd` | 0 | Starts authenticated OTA, `/status`, and runtime-config HTTP handling. |
| `bno085` | 0 | Optional BNO085 accelerometer test when enabled; owns realtime priority on the shared I2C bus. |
| `bq25792` | 0 | Low-rate charger monitor; uses background I2C access so BNO085 can win bus arbitration. |
| `max77958` | 0 | Low-rate USB-C PD monitor/config service; also uses background I2C access. |
| `wireless_log` | unpinned | Drains the log queue and mirrors logs over TCP; FreeRTOS may run it on either core. |
| `wireless_tel` | 1 | Drains high-rate telemetry into batched TCP writes. |
| short-lived reboot tasks | unpinned | Temporary restart helpers after OTA or runtime-config changes. |

ESP-IDF also creates internal Wi-Fi, TCP/IP, and event-loop tasks. The HTTP
server task is pinned to core 0 so `/status` and runtime config do not share the
UWB core. The timing-critical UWB transmit instants are still programmed into
the DW3000 with delayed TX, so the radio owns the sub-microsecond timing rather
than the FreeRTOS scheduler.

UWB waits are deliberately yielding. RX/TX waits use the DW3000 IRQ line and
FreeRTOS task notifications when IRQ is enabled, with `vTaskDelay` as the
fallback path. Calibration slot alignment uses the dedicated 1 MHz ESP GPTimer
alarm plus a task notification, so the firmware does not busy-loop while waiting
for the guard window or slot end.

## Boot Recovery

The firmware has two recovery layers so a bad OTA or a crash loop should not
make a module disappear from Wi-Fi permanently.

| Layer | What it handles | Mechanism |
| --- | --- | --- |
| ESP-IDF OTA rollback | A new OTA image crashes or resets before it proves it can boot. | `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` marks a fresh OTA image as `pending_verify`. The app calls `esp_ota_mark_app_valid_cancel_rollback()` only after Wi-Fi and OTA HTTP are online for `APP_BOOT_GUARD_STABLE_DELAY_MS`. If a reset happens first, the bootloader rolls back to the previous valid OTA slot. |
| Local recovery mode | A valid app, serial-flashed app, or bad runtime config keeps rebooting. | `components/boot_guard` stores a small state record in RTC no-init memory. After `APP_BOOT_GUARD_FAILURE_THRESHOLD` unstable resets, the next boot starts only identity/runtime config, Wi-Fi, wireless log, telemetry, and OTA. UWB, GPS, charger, USB-C PD, and BNO085 are skipped. |

Normal boot order is intentionally conservative: Wi-Fi, wireless log, telemetry,
and OTA start before UWB or I2C peripherals. Recovery mode uses the same path but
stops before the risky services and holds the DW3000 in reset.

`/status` exposes the recovery state:

| Field | Meaning |
| --- | --- |
| `boot_recovery_mode` | `true` when only the minimal recovery services are running. |
| `boot_guard_failure_count` | Consecutive unstable resets counted in RTC memory. |
| `boot_guard_recovery_threshold` | Failure count that latches recovery mode. |
| `boot_guard_ota_state` | Current OTA state, such as `valid` or `pending_verify`. |
| `boot_guard_pending_verify` | The running app still needs to be marked valid. |
| `boot_guard_last_reset_reason_name` | Last reset reason reported by ESP-IDF. |

Recovery can be cleared after uploading a known-good firmware or runtime config:

```sh
curl -X POST -H "X-OTA-Token: <APP_OTA_PASSWORD>" \
  "http://<module-ip>/config/recovery?clear=1&reboot=1"
```

Important: rollback lives in the bootloader. OTA updates replace only the app
partition, so each module needs one full serial flash after enabling rollback.
After that, future OTA images are protected by the rollback flow.

## PCB Package

The hardware package is versioned with the firmware so the board definition and
software assumptions stay together.

| Path | Purpose |
| --- | --- |
| `PCB/V1.REV.B/Kicad project/` | Editable KiCad project and local symbols/footprints |
| `PCB/V1.REV.B/Gerber/Gerber.zip` | Fabrication archive |
| `PCB/V1.REV.B/BOM/` | BOM exports |
| `PCB/V1.REV.B/PickAndPlace/` | Assembly placement files |
| `Schematic/UWB_V1.0_Rev.B.pdf` | Exported schematic PDF |

PCB preview:

![UWB PCB Preview](PCB/V1.REV.B/Kicad%20project/Preview/UWB.png)

## App Modes

The expected UWB workflow split is:

1. `APP_RUNTIME_MODE_UWB_BEACON_SMOKE`: current random beacon TX/RX smoke test
   in `components/uwb_dw3000`.
2. `APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION`: antenna delay
   calibration entry point in `components/uwb_calibration_service`.
3. `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`: current two-module DS-TWR ruler check
   entry point in `components/uwb_distance_test_service`.
4. `APP_RUNTIME_MODE_UWB_RANGING`: current 4-anchor plus 1-tag DS-TWR
   ranging runtime in `components/uwb_ranging_service`. The first configured
   anchor coordinates the slots, anchors initiate, and the tag responds/logs
   tag-anchor distances for the dashboard.
5. `APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY`: anchor-to-anchor survey/runtime
   diagnostics in `components/uwb_anchor_survey_service`.
6. `APP_RUNTIME_MODE_UWB_DS_TWR_TDOA`: experimental DS-TWR-TDOA runtime. Anchors
   keep running DS-TWR between themselves, while the tag only listens on UWB and
   sends range-difference observations to the dashboard over Wi-Fi logs.

The beacon smoke mode, distance-test mode, antenna-delay calibration workflows,
anchor survey, multi-anchor ranging, and experimental DS-TWR-TDOA runtimes are
implemented for the current lab workflow.

`APP_RUNTIME_MODE` remains the firmware default. The effective runtime mode and
common test parameters can be overridden at runtime through NVS using the
authenticated `/config/runtime` HTTP endpoint.

## UWB Ranging Protocol

The active ranging implementation is based on Double-Sided Two-Way Ranging
(DS-TWR). Classic DS-TWR needs three UWB frames: `POLL`, `RESP`, and `FINAL`.
This firmware adds two report frames:

| Frame | Direction | Purpose |
| --- | --- | --- |
| `POLL` | initiator -> responder | Starts one ranging exchange. |
| `RESP` | responder -> initiator | Confirms `POLL` and carries responder timing. |
| `FINAL` | initiator -> responder | Completes the DS-TWR timing triangle. |
| `REPORT` | initiator -> responder | Carries initiator timestamps so the responder can calculate distance. |
| `REPORT2` | responder -> initiator | Carries responder timestamps and responder-calculated distance so the initiator can verify locally. |

When looking only at the responder-calculated result, the useful exchange is
`POLL`, `RESP`, `FINAL`, and `REPORT`. `REPORT2` is a firmware verification
frame; it lets the initiator run the same calculation and compare the two
answers.

### Runtime Shape

The same firmware image runs on every module. At boot, each board reads its
persistent module ID and runtime config from NVS.

| Module ID | Normal ranging behavior |
| --- | --- |
| `1` | Tag / responder; waits for anchor-initiated DS-TWR and logs results |
| `2` | Anchor / coordinator by default when anchors are `2,3,4,5` |
| `3` | Anchor / follower |
| `4` | Anchor / follower |
| `5` | Anchor / follower |

`APP_RUNTIME_MODE_UWB_RANGING` is anchor-initiated. The tag no longer starts
each exchange. Instead, the first configured anchor ID is the ranging
coordinator. It walks the configured anchors in order and gives each anchor one
slot to initiate DS-TWR against the tag. The tag stays in RX, responds to
`POLL`, calculates the responder-side distance after `REPORT`, and emits the
dashboard-compatible log line:

```text
UWB_RANGING result tag=<tag_id> anchor=<anchor_id> seq=<seq> distance=<m> m ...
```

This keeps the PC/dashboard as the place where tag position is solved, while
the ESP32 tag only does the per-anchor DS-TWR calculation and reporting.

The anchor coordinator schedules anchors sequentially:

```text
round N

anchor 2 -> tag 1
anchor 3 -> tag 1
anchor 4 -> tag 1
anchor 5 -> tag 1

wait ranging_round_gap_ms
round N + 1
...
```

The important timing parameters are exposed by `/status`, can be changed with
`/config/runtime`, and are also available in the dashboard:

| Name | Default / lab value | Meaning |
| --- | ---: | --- |
| `ranging_slot_ms` | `350 ms` | Time budget for one anchor-initiated DS-TWR exchange. |
| `ranging_round_gap_ms` | `500 ms` | Delay after all anchors in one ranging round. |
| `ranging_rx_slice_ms` | `100 ms` | RX window used by the tag and follower anchors while waiting for UWB frames. |
| `dt_rx_timeout_ms` | `100 ms` | Max wait for expected DS-TWR frames. |
| `dt_resp_delay_ms` | `20 ms` | Scheduled delay from `POLL RX` to `RESP TX`. |
| `dt_final_delay_ms` | `20 ms` | Scheduled delay from `RESP RX` to `FINAL TX`. |
| `dt_report_delay_ms` | `10 ms` | Software delay before `REPORT` and `REPORT2`. |
| `dt_auto_rx_delay_uus` | `500 UUS` | DW3000 hardware delay after TX before auto-RX opens. |

Remote anchors receive a short `RANGING_CMD` from the coordinator and wait a
fixed `5 ms` guard before starting their DS-TWR exchange. The coordinator uses
the first anchor ID from the configured anchor list, so the order of `anchors`
matters.

The `dt_*` names come from the older distance-test runtime, but the current
multi-anchor ranging mode reuses the same DS-TWR implementation.

### DS-TWR-TDOA Runtime

`APP_RUNTIME_MODE_UWB_DS_TWR_TDOA` is the first experimental radio-passive tag
runtime. The configured anchors run the normal DS-TWR exchange pair-by-pair. The
tag does not call any UWB TX function in this mode; it only listens to the
anchor frames and logs one TDOA observation when it has heard `POLL`, `RESP`, and
`REPORT2` for the same anchor pair and sequence.

The coordinator is the first configured anchor ID. For anchors `2,3,4,5`, one
round walks all unordered anchor pairs. Successive rounds alternate the direction
of every pair, so each physical edge is measured in both radio directions:

```text
round 0:
2 -> 3
2 -> 4
2 -> 5
3 -> 4
3 -> 5
4 -> 5

round 1:
3 -> 2
4 -> 2
5 -> 2
4 -> 3
5 -> 3
5 -> 4
```

During the short-slot lab tests, the active runtime config was:

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `anchor_survey_slot_ms` | `100 ms` | Time budget for one directed anchor-anchor DS-TWR exchange. |
| `anchor_survey_round_gap_ms` | `10 ms` | Quiet gap after all anchor pairs in one round. |
| `anchor_survey_command_delay_ms` | `10 ms` | Delay after a coordinator command before a follower initiates DS-TWR. |
| `anchor_survey_rx_slice_ms` | `100 ms` | Listen window used by followers/tag while waiting for UWB frames. |
| `distance_test_rx_timeout_ms` | `90 ms` | Maximum wait for expected DS-TWR frames inside the slot. |
| `distance_test_resp_delay_ms` | `20 ms` | Delayed TX turnaround from `POLL RX` to `RESP TX`. |
| `distance_test_final_delay_ms` | `20 ms` | Delayed TX turnaround from `RESP RX` to `FINAL TX`. |
| `distance_test_report_delay_ms` | `10 ms` | Delay before `REPORT` and before `REPORT2`. |
| `distance_test_auto_rx_delay_uus` | `500 UUS` | DW3000 automatic TX-to-RX delay, about `513 us`. |

With four anchors this gives:

```text
one round = 6 slots * 100 ms + 10 ms round gap ~= 610 ms
```

For a slot where the coordinator is not the initiator, for example `A3 -> A4`
with `A2` as coordinator, the slot looks like this:

```text
slot A3 -> A4, total budget 100 ms

A2 coordinator        A3 initiator              A4 responder              Tag M1
     |                     |                         |                       |
t=0  |-- DS_TWR_TDOA_CMD ->|                         |                       |
     |                     |                         |                       |
     |                     | wait command_delay_ms   |                       |
     |                     | currently 10 ms         |                       |
     |                     |                         |                       |
t~10 |                     |-- POLL ---------------->|                       |
     |                     | \                       |                       |
     |                     |  \----------------------|---------------------> RX POLL
     |                     |                         |                       |
     |                     |                         | schedule RESP +20 ms |
     |                     |                         |                       |
t~30 |                     |<---------------- RESP --|                       |
     |                     |<------------------------|--------------------- RX RESP
     |                     |                         |                       |
     |                     | schedule FINAL +20 ms   |                       |
     |                     |                         |                       |
t~50 |                     |-- FINAL --------------->|                       |
     |                     |                         |                       |
     |                     | wait report_delay_ms    |                       |
     |                     | currently 10 ms         |                       |
     |                     |                         |                       |
t~60 |                     |-- REPORT -------------->|                       |
     |                     |                         | calculate distance   |
     |                     |                         | wait report_delay_ms |
     |                     |                         |                       |
t~70 |                     |<--------------- REPORT2-|                       |
     |                     |<------------------------|--------------------- RX REPORT2
     |                     |                         |                       |
t=100| slot end            | slot end                | slot end             | keeps listening
```

If the coordinator is also the initiator for the current pair, there is no
`DS_TWR_TDOA_CMD`: the coordinator starts the `POLL` directly at the beginning
of the slot. The passive tag never transmits in this runtime; it only records
its local RX timestamps for `POLL`, `RESP`, and `REPORT2`.

Unlike antenna-delay calibration, this runtime does not currently use
`CAL_SYNC` plus a 500 us guard at the start of each slot. The slots are paced by
the coordinator and by the received UWB commands/frames. If more timing margin
or determinism is needed, the calibration slot-sync mechanism is the natural
next upgrade path for `DS-TWR-TDOA`.

Short-slot profile testing on the live five-module setup gave these 25 s
snapshots:

| Profile | Slot | Timeout/error logs | Anchor distance std | TDOA diff std | Verdict |
| --- | ---: | ---: | --- | --- | --- |
| `Stable Baseline` | `100 ms` | `44` | typically `1.3-2.3 cm` | typically `3.9-5.9 cm` | Best current stable reference. |
| `Safe Fast` | `60 ms` | `148` | typically `1.0-2.6 cm` on successful pairs | typically `3.7-5.2 cm` | Too many missing slots in current geometry. |
| `Balanced` | `50 ms` | `144` | typically `1.5-2.3 cm` on successful pairs | typically `3.2-6.9 cm` | Faster, but still loses too many pair exchanges. |
| `Aggressive` | `40 ms` | `138` | typically `1.5-2.3 cm` on successful pairs | typically `3.0-5.5 cm` | Not catastrophic, but not reliable enough. |

The important observation is that the short profiles still produce good-looking
measurements when a pair succeeds; the problem is missing exchanges, especially
on weaker/currently awkward pairs involving module 5 and module 3. For now,
`Stable Baseline` should remain the fallback/default. The next useful candidate
is an intermediate profile rather than jumping straight to 50 ms:

```text
slot            = 80 ms
round gap       = 10 ms
RX slice        = 80 ms
command delay   = 8 ms
DS-TWR timeout  = 60 ms
RESP delay      = 15 ms
FINAL delay     = 15 ms
REPORT delay    = 8 ms
auto RX delay   = 500 UUS
```

This is inspired by the FlexTDOA idea of rotating radio roles. It does not
remove multipath, but it avoids always using the same antenna orientation and
same anchor role on a given edge. The dashboard stores the measured
anchor-anchor distance as an unordered pair, while each TDOA observation keeps
the directed `initiator -> responder` sign.

For one anchor pair `Ai -> Aj`, the tag hears:

```text
Ai POLL  -----> Aj
   \             \
    \             tag RX timestamp R_poll

Aj RESP  <----- Ai
   \             \
    \             tag RX timestamp R_resp

Aj REPORT2 ----> Ai
                 carries T2, T3, T6 and measured anchor distance Ai-Aj
```

`REPORT2` already contains the responder-side timestamps and the measured
anchor-anchor distance. The tag combines that with its local receive timestamps:

```text
reply_b      = T3 - T2                         // Aj clock
tof_ij       = distance(Ai, Aj) / c
rx_delta_tag = R_resp - R_poll                 // tag clock

range_diff_ij = c * (rx_delta_tag - reply_b_corrected - tof_ij)
              = distance(tag, Aj) - distance(tag, Ai)
```

The firmware logs:

```text
UWB_DS_TWR_TDOA obs tag=<tag> initiator=<Ai> responder=<Aj> seq=<seq> diff=<m> m ...
```

The passive tag keeps a small clock-offset filter per responder anchor. The raw
DW3000 carrier-integrator clock ratio is useful but noisy enough that applying a
single instantaneous value adds visible jitter to `diff`. The firmware therefore
uses a capped running average, currently up to 64 samples per responder, before
converting `reply_b` from the responder clock domain into the tag clock domain.
The log still includes both `raw` and corrected `diff` so the correction can be
audited later.

The dashboard `Position` tab reconstructs the relative anchor geometry from live
anchor-anchor DS-TWR measurements. The first selected anchor is placed at
`(0,0)`, the second defines the X axis, the third/fourth are trilaterated from
the measured edges, and a small least-squares refinement spreads any geometry
error across all fresh edges. The geometry table reports residual error in
centimeters, so the real setup can be a slightly skewed quadrilateral instead of
a perfect square.

With `DS-TWR-TDOA` selected, the dashboard takes fresh `diff` observations and
solves the tag position on the PC with a local least-squares range-difference
fit. When both directions of a pair are fresh, the dashboard uses the
antisymmetric median `(Ai->Aj - Aj->Ai) / 2` and reports the reverse sum as a
health check. A reverse sum near zero means the two directed observations agree;
a large reverse sum means the pair has common-mode bias even if each individual
line looks stable. `DS-TWR ranges` can use the same measured anchor geometry for
absolute tag-anchor ranges, but the blue distance circles only make sense for
absolute ranges, not TDOA range differences.

### Time Units

The DW3000 uses several time units:

| Unit | Meaning |
| --- | --- |
| `ms` | Normal milliseconds used by FreeRTOS and runtime config. |
| `UUS` | UWB microseconds; `1 UUS = 512 / 499.2 MHz = 1.025641 us`. |
| `DTU` | DW3000 device time unit; about `15.650040064 ps`. |

Useful conversions:

| Value | Approximate real time |
| --- | ---: |
| `500 UUS` | `512.82 us`, or `0.513 ms` |
| `20 ms` | `1,277,952,000 DTU` |
| `10 ms` | `638,976,000 DTU` |

`UUS` is used for DW3000 automatic TX-to-RX wait. The larger `20 ms` DS-TWR
turnaround delays are converted to DTU and programmed as delayed TX timestamps.

### Message Sequence

One ranging attempt between tag `1` and one anchor looks like this:

```text
Tag / initiator, module 1                         Anchor / responder, module N

T1: POLL TX  ------------------------------------>
                                                   T2: POLL RX

                                                   schedule RESP at:
                                                   T3 = T2 + dt_resp_delay_ms

                                                   T3: RESP TX
     auto RX opens after 500 UUS  <---------------
T4: RESP RX

schedule FINAL at:
T5 = T4 + dt_final_delay_ms

T5: FINAL TX  ----------------------------------->
                                                   auto RX opens after 500 UUS
                                                   T6: FINAL RX

wait dt_report_delay_ms

REPORT TX: T1, T4, T5 --------------------------->
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

The critical `RESP` and `FINAL` instants are owned by the DW3000 radio through
delayed TX. The firmware computes the target timestamp, writes `DX_TIME`, and
issues `DTX` or `DTX_W4R`. That keeps the timing stable even if FreeRTOS has
scheduler jitter.

### Calibration Slot Timing

The three-module antenna-delay calibration uses deterministic slots so only one
ordered pair is expected to talk at a time. For `1,2,3`, one complete round is:

```text
slot 0: 1 -> 2
slot 1: 1 -> 3
slot 2: 2 -> 1
slot 3: 2 -> 3
slot 4: 3 -> 1
slot 5: 3 -> 2
then wait calibration_round_gap_ms
round N + 1
```

The slot length is `calibration_min_interval_ms`. In the lab dashboard this is
normally `100 ms`. The round gap is `calibration_max_interval_ms`; the dashboard
labels it `Round gap ms` and defaults it to `10 ms`.

If the coordinator is also the initiator for the current pair, for example
`1 -> 2`, the slot looks like this:

```text
t = 0 us
M1 coordinator/init     M2 responder          M3 listener
     CAL_SYNC  ------->      receive               receive
     start timer             start timer           start timer
          |                       |                     |
          |<------ guard 500 us ------>|                |

t = 500 us
     POLL  ------------>      RX POLL
                              schedule RESP delayed TX
                              RESP at +20 ms
     <------------- RESP

     schedule FINAL delayed TX
     FINAL at +20 ms
     FINAL ------------>
                              RX FINAL

     wait 10 ms
     REPORT ----------->
                              RX REPORT
                              calculate distance
     <------------ REPORT2
     verify anchor result

t = slot end, normally 100 ms
all modules exit the slot and reset the calibration timer
```

If the initiator is a follower, for example `2 -> 3`, the coordinator assigns
the slot with `CAL_CMD`:

```text
t = 0 us
M1 coordinator          M2 initiator          M3 responder
     CAL_SYNC  ------->      receive               receive
     start timer             start timer           start timer

t = 500 us
     CAL_CMD ---------> M2
     "this slot is 2 -> 3"

M2 waits APP_UWB_CALIBRATION_COMMAND_DELAY_MS
currently 20 ms

     POLL -------------------------------> M3
                                            RESP delayed +20 ms
     <------------------------------- RESP

     FINAL delayed +20 ms ---------------->
     REPORT after 10 ms ------------------>
                                            calculate distance
     <------------------------------ REPORT2

t = slot end, normally 100 ms
all modules exit the slot and reset the calibration timer
```

`CAL_SYNC` is sent before every slot. The coordinator starts a dedicated 1 MHz
ESP GPTimer when the SYNC transmission completes. Followers arm the same timer
before RX and start it from the DW3000 IRQ when the SYNC frame arrives. The
first `cal_guard_us` microseconds of the slot are a guard window. The firmware
default is `APP_UWB_CALIBRATION_SLOT_GUARD_US = 500 us`, and the dashboard can
change it at runtime without OTA.

The guard and slot-end waits are implemented with a GPTimer alarm and task
notification. This preserves microsecond-level slot alignment without spinning
the UWB task in a CPU busy-wait.

This ESP-side timer does not enter the distance formula. It only aligns the
slot windows so the lab workflow is deterministic. Distance is still calculated
from DW3000 hardware timestamps with DTU resolution.

SYNC is intentionally not retried inside the same slot. If a follower misses
SYNC, if the SYNC sequence number jumps, or if the coordinator cannot transmit
SYNC, firmware logs:

```text
UWB CAL slot skipped due to sync fail
```

Dashboard-driven auto calibration treats any sync miss during the collection
window as invalid and blocks antenna-delay writes.

### Timestamp Ownership

The easiest way to reason about DS-TWR is to track who knows each timestamp.
Every timestamp below is a hardware timestamp captured or scheduled by DW3000.

After `POLL`:

| Side | What it knows |
| --- | --- |
| Initiator/tag | `T1`, timestamp when it transmitted `POLL`. |
| Responder/anchor | `T2`, timestamp when it received `POLL`. |

`T1` is in the tag clock domain. `T2` is in the anchor clock domain. The two
absolute values cannot be compared directly.

After `RESP`:

```text
T3 = T2 + dt_resp_delay_ms
```

| Side | What it knows |
| --- | --- |
| Initiator/tag | `T1`, `T4` |
| Responder/anchor | `T2`, `T3` |

The tag can compute:

```text
round_a = T4 - T1
```

The anchor can compute:

```text
reply_b = T3 - T2
```

The tag still cannot calculate a correct distance from only those two values,
because `reply_b` was measured in the anchor clock domain.

After `FINAL`:

```text
T5 = T4 + dt_final_delay_ms
```

| Side | What it knows |
| --- | --- |
| Initiator/tag | `T1`, `T4`, `T5` |
| Responder/anchor | `T2`, `T3`, `T6` |

The tag-side intervals are:

```text
round_a = T4 - T1
reply_a = T5 - T4
```

The anchor-side intervals are:

```text
reply_b = T3 - T2
round_b = T6 - T3
```

At this point all pieces needed for DS-TWR exist, but they are split between
the two modules. `REPORT` moves `T1`, `T4`, and `T5` to the anchor. The anchor
already has `T2`, `T3`, and `T6`, so after `REPORT` it can calculate distance.

`REPORT2` then moves `T2`, `T3`, `T6`, and the anchor-calculated distance back
to the tag. The tag still has its own `T1`, `T4`, and `T5`, so it can calculate
the same distance locally and log a verification line:

```text
DS-TWR tag verify ... tag=<distance> anchor=<distance> diff=<cm>
```

### Distance Formula

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

In the ideal model, `tof` is the real one-way flight time. If both sides shared
one perfect clock:

```text
round_a = 2 * tof + reply_b
round_b = 2 * tof + reply_a
```

The DS-TWR expression is chosen so the internal reply delays cancel out:

```text
round_a * round_b - reply_a * reply_b
```

Substitute the ideal expressions:

```text
(2 * tof + reply_b) * (2 * tof + reply_a) - reply_a * reply_b
```

Expand and cancel the `reply_a * reply_b` terms:

```text
4 * tof^2 + 2 * tof * reply_a + 2 * tof * reply_b
```

Factor:

```text
2 * tof * (2 * tof + reply_a + reply_b)
```

The denominator is:

```text
round_a + round_b + reply_a + reply_b
```

Substitute again:

```text
(2 * tof + reply_b) + (2 * tof + reply_a) + reply_a + reply_b
```

which factors to:

```text
2 * (2 * tof + reply_a + reply_b)
```

The full fraction therefore simplifies to `tof`:

```text
2 * tof * (2 * tof + reply_a + reply_b)
----------------------------------------
2       * (2 * tof + reply_a + reply_b)
```

This is why the formula has this shape:

```text
tof = (round_a * round_b - reply_a * reply_b)
      / (round_a + round_b + reply_a + reply_b)
```

A simpler expression such as this is tempting:

```text
tof = (round_a - reply_b) / 2
```

but `round_a` is measured by the tag clock while `reply_b` is measured by the
anchor clock. Those clocks are close, but not identical. The DS-TWR formula
uses both perspectives symmetrically and reduces clock mismatch error. With
`APP_UWB_DISTANCE_TEST_CLOCK_OFFSET_CORRECTION = 1`, the firmware also uses the
DW3000 clock-offset measurement from the received frame before converting time
of flight to meters.

### TX-to-RX and RX-to-TX Timing

The two most common timing questions are:

| Question | Current answer |
| --- | --- |
| How long after TX until RX opens? | `500 UUS`, about `0.513 ms` |
| How long after RX until the next TX? | `20 ms` for `RESP`, `20 ms` for `FINAL` |

The TX-to-RX delay is a DW3000 auto-receive setting. It applies after:

```text
POLL TX  -> tag opens RX for RESP
RESP TX  -> anchor opens RX for FINAL
REPORT TX -> tag opens RX for REPORT2
```

The RX-to-TX delays are delayed-TX timestamps. They apply after:

```text
POLL RX  -> anchor schedules RESP TX at +20 ms
RESP RX  -> tag schedules FINAL TX at +20 ms
```

So the system is not trying to turn around instantly. The radio has about half
a millisecond before RX opens after a TX, and about 20 ms between receiving one
DS-TWR frame and transmitting the next scheduled DS-TWR frame.

### Antenna Delay

Antenna delay is separate from protocol delays. It is not a sleep, slot, or
turnaround time. It is a calibration value written to the DW3000 RX and TX
antenna-delay registers during radio init.

The firmware reads the active antenna delay from persistent identity NVS. If no
calibrated value exists, it falls back to `APP_UWB_ANTENNA_DELAY_DEFAULT`.

Wrong antenna delay shifts absolute distance even if packet success is perfect.
Use `/status` to inspect the live values:

```text
uwb_active_antenna_delay
uwb_active_antenna_delay_hex
uwb_antenna_delay_from_nvs
```

### Common Log Patterns

| Log pattern | Meaning |
| --- | --- |
| `DS-TWR RESP wait failed` | Initiator sent `POLL` but did not receive matching `RESP` before timeout. |
| `DS-TWR FINAL wait failed` | Responder sent `RESP` but did not receive `FINAL` before timeout. |
| `DS-TWR REPORT wait failed` | Responder received `FINAL` but did not receive the timestamp report. |
| `DS-TWR REPORT2 wait failed` | Responder calculated distance, but initiator did not receive verification report. |
| `UWB distance RX error` | DW3000 reported PHY/RX error instead of a valid frame. |
| `UWB_RANGING result` | Full exchange completed; in ranging mode the tag calculated the responder-side distance for the initiating anchor. |
| `DS-TWR tag verify` | Initiator recalculated the same exchange and compared against anchor result. |
| `UWB CAL slot skipped due to sync fail` | Calibration slot synchronization failed; dashboard marks the run invalid. |

Useful code entry points:

| Area | File / function |
| --- | --- |
| Runtime mode selection | `components/app_manager/app_manager.c` |
| Runtime config defaults/NVS | `components/config/app_runtime_config.c` |
| Ranging service entry point | `components/uwb_ranging_service/uwb_ranging_service.c` |
| Tag/anchor loops | `components/uwb_dw3000/uwb_dw3000.c` |
| DS-TWR initiator | `uwb_distance_initiate_once()` |
| DS-TWR responder | `uwb_distance_respond_to_poll()` |
| Delayed TX helpers | `uwb_dw3000_send_payload_delayed*()` |
| Antenna-delay calibration | `components/uwb_dw3000/uwb_dw3000_calibration.inc` |

The board identity is stored in NVS, which plays the role of persistent EEPROM
storage on ESP32. Normal firmware reads `module_id` from NVS and builds the
hostname from `APP_IDENTITY_HOSTNAME_PREFIX`, for example `uwb-module-2`. To
change a board identity or UWB role, set these values in
`components/config/include/app_config.h` for one provisioning boot:

```c
#define APP_IDENTITY_PROVISION_ENABLED 1
#define APP_IDENTITY_PROVISION_MODULE_ID 2
#define APP_IDENTITY_PROVISION_UWB_ROLE_ENABLED 1
#define APP_IDENTITY_PROVISION_UWB_ROLE APP_UWB_ROLE_ANCHOR
```

For a mobile tag, use `APP_UWB_ROLE_TAG`; for fixed points, use
`APP_UWB_ROLE_ANCHOR`. After that boot, set the provisioning flags back to `0`
and flash the common firmware again. `/status` exposes `hostname`, `module_id`,
`module_id_from_nvs`, `module_id_provisioned_this_boot`, `uwb_role_name`,
`uwb_role_from_nvs`, and `uwb_role_provisioned_this_boot`.

The Wi-Fi service scans before connecting and logs each matching AP with BSSID,
channel, RSSI, auth mode, and cipher. If a network broadcasts the same SSID
from multiple radios and one behaves better, lock the board to that AP in
`components/config/include/app_config.h`:

```c
#define APP_WIFI_LOCK_BSSID 1
#define APP_WIFI_BSSID "aa:bb:cc:dd:ee:ff"
#define APP_WIFI_LOCK_CHANNEL 6
```

The firmware also keeps reconnecting after disconnects and exposes the last
disconnect reason plus scan/connected AP details in `/status`.

UWB bring-up follows the same first-step pattern used by the Zephyr DW3000
decadriver reference: initialize hardware, reset the chip, then read `DEV_ID`.
The valid IDs currently accepted are `0xDECA0302` and `0xDECA0312`.

After bring-up, the firmware runs a simple same-image UWB smoke test. Each
board sends a `UWBT` beacon at a random interval and spends the rest of the
time in RX. With the same firmware on two boards, wireless logs should show
`UWB TX beacon ...` and `UWB RX beacon ...` lines. The HTTP `/status` response
also exposes `uwb_tx_count`, `uwb_rx_count`, and the last received source/seq.

In `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`, the same firmware assigns roles from
the source ID: source `1` is the initiator and source `2` is the responder by
default. The UWB source ID is derived from the persistent module ID unless
`APP_UWB_SOURCE_ID` is explicitly overridden. The initiator sends `POLL`,
receives `RESP`, schedules `FINAL`, then sends a `REPORT` with its hardware
timestamps. The responder schedules `RESP`, receives `FINAL` and `REPORT`,
calculates DS-TWR distance, and logs lines like
`DS-TWR distance seq=... distance=...`.

`RESP` and `FINAL` use DW3000 delayed TX by default
(`APP_UWB_DISTANCE_TEST_USE_DELAYED_TX`). The delay windows are configured with
`APP_UWB_DISTANCE_TEST_RESP_DELAY_MS` and
`APP_UWB_DISTANCE_TEST_FINAL_DELAY_MS`; the firmware converts them to DW3000
device time units and programs `DX_TIME`, so the critical transmit instant is
owned by the radio instead of the FreeRTOS scheduler.

`APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION` reuses the same DS-TWR
exchange and supports two workflows selected in `components/config/include/uwb_config.h`:

```c
#define APP_UWB_CALIBRATION_METHOD APP_UWB_CALIBRATION_METHOD_TWO_MODULE
#define APP_UWB_CALIBRATION_KNOWN_DISTANCE_MM 2000
#define APP_UWB_CALIBRATION_REFERENCE_ID 1
#define APP_UWB_CALIBRATION_DUT_ID 2
```

For the two-module method, the reference module initiates and the DUT responds.
Each measurement slot starts with a short UWB `CAL_SYNC` frame from the
reference. The reference starts a dedicated 1 MHz ESP GPTimer when that SYNC
transmission completes; the DUT arms the same timer before RX and starts it from
the DW3000 IRQ when the SYNC frame arrives. Both sides then wait the fixed
`cal_guard_us` guard window before POLL/RESP/FINAL begins. The firmware default
comes from `APP_UWB_CALIBRATION_SLOT_GUARD_US`, currently `500 us`.
The DUT logs sample distance, mean, standard deviation, error against the known
distance, and a first-pass suggested antenna delay. Swap `REFERENCE_ID` and
`DUT_ID` to calibrate the other board.

```c
#define APP_UWB_CALIBRATION_METHOD APP_UWB_CALIBRATION_METHOD_THREE_MODULE
#define APP_UWB_CALIBRATION_THREE_ID_0 1
#define APP_UWB_CALIBRATION_THREE_ID_1 2
#define APP_UWB_CALIBRATION_THREE_ID_2 3
#define APP_UWB_CALIBRATION_THREE_DISTANCE_0_1_MM 2000
#define APP_UWB_CALIBRATION_THREE_DISTANCE_0_2_MM 2000
#define APP_UWB_CALIBRATION_THREE_DISTANCE_1_2_MM 2000
```

For the three-module method, the first ID in the configured set is the
calibration coordinator. It walks a deterministic six-slot schedule:
`id0->id1`, `id0->id2`, `id1->id0`, `id1->id2`, `id2->id0`, and `id2->id1`.
Each slot is `APP_UWB_CALIBRATION_MIN_INTERVAL_MS` long; in the lab this is
`100 ms`, which should fit the complete DS-TWR exchange when
`dt_rx_timeout_ms` is kept short enough for the experiment.
Before every slot, the coordinator broadcasts a short `CAL_SYNC` frame and all
participants restart the dedicated 1 MHz calibration timer. The first
`cal_guard_us` microseconds of the slot are a guard window (`500 us` by
default), then the assigned ordered pair performs DS-TWR. Because SYNC is
repeated before every slot, ESP timer drift can only accumulate inside one
slot. The firmware still attempts the full `POLL`, `RESP`, `FINAL`, `REPORT`,
and `REPORT2` sequence, so the logs show if the exchange does not fit.

The coordinator waits `APP_UWB_CALIBRATION_SYNC_PREPARE_MS` before each SYNC
frame (`20 ms` by default). This is not part of the measured slot; it simply
gives the followers time to finish the previous slot, clear DW3000 state, and
re-arm RX before the next SYNC is transmitted.

SYNC is intentionally not retried inside the same slot. Followers wait long
enough to cover the normal round gap, then use the `CAL_SYNC` sequence number to
detect skipped slots. If a sequence gap is detected, if the long SYNC wait
expires, or if the coordinator/reference cannot transmit SYNC, firmware logs
`UWB CAL slot skipped due to sync fail` and discards that slot. The same log line
includes aggregate sync health counters (`sync_ok`, `sync_timeout`,
`sync_invalid`, `sync_rx_error`, `sync_tx_fail`, `sync_sequence_gap`, and
`slot_skipped`). During dashboard-driven auto calibration, any such sync miss in
the collection window marks the run invalid and blocks antenna-delay writes, even
if enough diagnostic samples are later collected.

After all six directed pairs are measured, the firmware waits
`APP_UWB_CALIBRATION_MAX_INTERVAL_MS` before the next round. The firmware
default is `10 ms`; increase it only if short-gap testing shows sync misses or
radio cleanup problems between rounds. The dashboard labels these as `Slot ms`
and `Round gap ms`.

Calibration timing controls in the dashboard:

| Control | Meaning |
| --- | --- |
| `Summary every` | Log diagnostic calibration statistics every N accepted samples. It does not change the measurement sequence. |
| `Slot ms` | Fixed time budget for one directed pair, for example `M4 -> M2`. |
| `Guard us` | Quiet time at the beginning of each synchronized slot before the assigned pair starts talking. |
| `DS-TWR timeout ms` | Maximum wait for expected ranging frames inside the DS-TWR exchange. For the current short-slot test it is intentionally `100 ms`, the same as the slot width. |
| `Round gap ms` | Delay after all six directed pairs finish, before the next calibration round starts. |
| `RX slice ms` | Short listen window used by passive calibration nodes while they monitor the rest of the slot. Smaller slices wake more often; larger slices reduce loop churn. |

The logs contain directed pair statistics such as `1->2`, `2->1`, `1->3`,
etc. The dashboard keeps these directions separate and solves the antenna-delay
corrections by minimizing the full directed EDM residual, matching the spirit of
Qorvo APS014: collect all `nChips * (nChips - 1)` TWR measurements, compare them
against the measured physical EDM, and choose the delays that minimize the
matrix error. The three distance macros can all be equal for an equilateral
setup or can hold the measured distance of each triangle edge. Use the final
radio configuration and a known distance such as 2-5 m; 20 cm is useful for
smoke testing, not for calibration.

The dashboard uses a median center for calibration samples. The default
collection count is `39` samples per directed pair, so the center sample is
unambiguous after sorting. Mean, standard deviation, min, and max are still
reported as diagnostics, but antenna-delay corrections are based on medians to
make an occasional outlier less likely to move the result.
The dashboard uses exactly that many samples per directed pair for the solve:
once a direction reaches the requested count, later samples for that direction
are ignored so all six directed edges have equal weight.

For `39` samples over all six directed pairs, use at least a `240 s` dashboard
auto-calibration timeout. A `180 s` timeout is close enough to the real run time
that a clean calibration can finish one or two directed samples short.

When dashboard auto calibration finishes, it stops UWB on the calibration
participants by writing `uwb=0` and rebooting them. Antenna-delay writes made by
the same job are staged in NVS first, so this final reboot also applies the new
delay while leaving the DW3000 held in reset. If setup fails before collection
starts, the dashboard sends the same UWB stop command to all configured modules
so a partial calibration start cannot leave a transmitter running.
The dashboard Cancel Calibration button is an emergency stop: it marks the
running job as cancelled when one exists and always sends `uwb=0` plus reboot to
all configured modules.
The Settings tab keeps the last auto-calibration result in a table next to the
calibration controls, including summary, antenna-delay writes, pair errors,
directed samples, fit residuals, and UWB stop/cleanup status.

Lab calibration note, 2026-07-11:

- Modules `1`, `2`, and `3` were placed in printed alignment fixtures as a
  2.00 m equilateral triangle.
- Starting delays were `M1=0x3fd6`, `M2=0x4019`, `M3=0x3fdc`.
- The first pass showed symmetric pair errors of about `1-2=-0.05 dtu`,
  `1-3=+44.65 dtu`, and `2-3=-23.90 dtu`.
- Solving the three pair equations produced corrections of `M1=+34 dtu`,
  `M2=-34 dtu`, and `M3=+10 dtu`.
- The delays stored in NVS are now `M1=0x3ff8`, `M2=0x3ff7`,
  `M3=0x3fe6`.
- Verification after reboot measured `1-2=2.001 m`, `1-3=1.996 m`, and
  `2-3=2.000 m`, with per-link standard deviation around `1.0-1.4 cm`.

For additional modules, keep two already calibrated modules fixed as references
and place the uncalibrated module in the third fixture position. Run the
three-module calibration with all three sides set to the measured triangle
edges, but apply only the correction for the new module. If `K1` and `K2` are
known-good references and `U` is the new module, the dashboard fits the
new-module correction from all directed errors that include `U`, while using the
reference-only directions as a guard. Intuitively this is close to:

```text
err_K1_U = center(err_dtu K1->U, err_dtu U->K1)
err_K2_U = center(err_dtu K2->U, err_dtu U->K2)
correction_U = round(center(err_K1_U, err_K2_U))
new_delay_U = old_delay_U + correction_U
```

The reference-reference pair should remain close to zero. If it does not, check
the physical placement or RF stability before writing a new delay.

The local dashboard can automate this workflow from `Settings` ->
`Antenna Delay Calibration` with `Auto Calibrate + Apply`. The button:

1. writes the runtime calibration setup and reboots the selected modules,
2. collects the requested number of `UWB CAL sample` log entries for each
   required directed pair,
3. takes the median distance for each directed pair, computes the directed EDM
   fit and least-squares antenna-delay corrections, including residuals that
   show how well one delay per module explains the measurements, and
4. writes the corrected antenna delay values to NVS for the selected target
   modules.

For a new module, keep two calibrated references in the fixtures, select only
the new module in `Targets`, and use the three-module set that includes both
references. For example, select `module 4` and run `1,2,4` to adjust only M4.
The dashboard configures all three calibration participants automatically and
holds excluded live UWB modules in reset while the calibration runs.
`Min apply DTU` prevents rewriting NVS for tiny noise-level corrections; the lab
default is `2`.
`Reference guard cm` protects the calibrated references: when a reference-only
edge, such as M1-M2 while calibrating M4, exceeds this error threshold, the
dashboard refuses NVS writes even when `Auto apply` is enabled. The lab default
is `2.00 cm`.

`APP_RUNTIME_MODE_UWB_RANGING` is the current absolute-range 1-tag/4-anchor
runtime. `APP_RUNTIME_MODE_UWB_DS_TWR_TDOA` is the experimental radio-passive
tag runtime that reuses anchor-to-anchor DS-TWR frames and solves from
range-difference observations in the dashboard. The same firmware image runs on
all modules; the runtime config selects tag ID and anchor IDs, the first anchor
ID acts as slot coordinator, and persistent NVS identity keeps each module's
hostname, module ID, UWB role, and calibrated antenna delay.

If `APP_UWB_DW_LEDS_ENABLED` is set, the DW3000 configures GPIO0 as RXOKLED,
GPIO1 as SFDLED, GPIO2 as RXLED, and GPIO3 as TXLED once during radio init.
The chip then drives these LED indications in hardware, so the firmware does
not add SPI traffic for LED toggling. Each function can be disabled separately
with `APP_UWB_DW_RXOK_LED_ENABLED`, `APP_UWB_DW_SFD_LED_ENABLED`,
`APP_UWB_DW_RX_LED_ENABLED`, or `APP_UWB_DW_TX_LED_ENABLED`.

The configured antenna delay is applied to both DW3000 RX and TX antenna delay
registers during radio init.

The BNO085 accelerometer test is controlled from
`components/config/include/app_config.h`:

```c
#define APP_BNO085_ACCEL_TEST_ENABLED 0
#define APP_BNO085_I2C_ADDRESS 0x4A
#define APP_BNO085_I2C_CLOCK_HZ 400000
#define APP_BNO085_ACCEL_INTERVAL_MS 50
#define APP_BNO085_LOG_INTERVAL_MS 1000
#define APP_BNO085_INT_WAIT_TIMEOUT_MS 250
```

Keep `APP_BNO085_ACCEL_TEST_ENABLED` at `0` for normal ranging builds. Set it
to `1` only while verifying the accelerometer hardware. When enabled, the task
runs on core 0, pulses BNO085 reset on GPIO40, enables
the calibrated accelerometer report over I2C/SHTP, then waits on the BNO085
active-low interrupt on GPIO15. The GPIO interrupt is configured as active-level,
not edge-only: on I2C the BNO08X deasserts `H_INTN` as soon as the I2C address
is recognized, so the ISR masks the GPIO interrupt and the task rearms it after
draining the pending SHTP packet. The timeout is only a fallback if an interrupt
is missed. It logs lines like
`BNO085 accel x=... y=... z=... m/s^2 accuracy=... irqs=... wait_timeouts=...`.
Leaving it disabled avoids the extra I2C and CPU work.

Runtime enable/disable is applied live through `/config/runtime` and the
dashboard Settings tab. If BNO085 is disabled, firmware does not keep an
accelerometer task polling a variable; it holds GPIO40 reset active. When BNO085
is enabled, the runtime endpoint starts the task directly. While the task is
running, GPIO15 interrupts, rate changes, and stop requests all wake it with
FreeRTOS task notifications (`INT`, `CONFIG`, `STOP`).

The dashboard Graphs tab plots accelerometer samples as soon as their wireless
telemetry arrives. Its `Timebase` control only changes the visible time window,
oscilloscope-style. `Samples/s` is the actual BNO085/report export rate; the
dashboard applies it as `bno085_sample_hz`, which updates runtime config and
sets both `bno085_accel_interval_ms` and `bno085_log_interval_ms`. The running
BNO085 task picks up rate changes live by sending a new `Set Feature` command,
so no reboot is needed. The BNO08X datasheet lists
`Accelerometer` at a maximum configurable rate of 500 Hz, although I2C bandwidth
and wireless throughput still need to be considered in practice.

When BNO085 is enabled, the shared I2C service treats its configured sample
period as a realtime reservation. BNO085 packet reads and `Set Feature` writes
take the realtime lock. BQ25792 and MAX77958 use the background lock, which will
not start a new transaction when BNO085 is waiting or when the next expected BNO
sample is within `APP_I2C_BACKGROUND_GUARD_US` (`500 us` by default). If BNO stops
producing interrupts for several sample periods, the reservation is considered
stale so charger/PD status can still be read and recovery logic can run.
Background clients may wait for a free window, but each low-level BQ/MAX I2C
transaction has a short timeout (`10 ms`) so a stuck background transfer cannot
hold the bus for dozens or hundreds of milliseconds.
The one-time startup probe keeps a longer timeout (`200 ms`) because some
devices need more slack before they acknowledge reliably; it is not used for the
regular status-transfer path. Normal BQ/MAX reads retry twice before reporting an
error. BQ register writes are retried because they are single-register,
idempotent updates. MAX77958 single-register writes are retried too. Longer
MAX77958 AP command writes are attempted through the direct high-speed path in
bounded chunks and then fall back to the standard 1 MHz transaction if the HS
path fails, so a configuration command still has one clean recovery route.

The current board uses mixed I2C speeds on the same physical bus. BNO085 stays
at `400 kHz`, because the BNO08X datasheet only specifies standard/fast mode up
to 400 kHz. BQ25792 is configured at `1 MHz`. MAX77958 keeps a normal 1 MHz
ESP-IDF device handle for fallback and startup, then enables `HS_EXT_EN` and
uses a local direct-HS helper at about `2 MHz` for validated MAX register
traffic. ESP-IDF keeps `scl_speed_hz` in each `i2c_device_config_t`, so the
firmware does not reinitialize the bus between ordinary device transactions;
the MAX direct-HS helper temporarily programs the I2C peripheral only while it
holds the shared background lock. BQ25792 has one descriptive I2C section that
mentions fast mode, but its electrical table specifies `fSCL = 1000 kHz`; the
1 MHz setting was therefore validated empirically on the module.

The MAX77958 high-speed path is a measured ESP32-S3 board result, not a claim
that ESP-IDF exposes a turnkey high-speed switch for this target. The firmware
enters an HS-master-style transfer by sending the I2C HS master code (`0x08`) at
the entry clock, then reprogramming the peripheral timing for the direct
transaction. On M1 with 1k pull-ups, the validated timing profile is
`low/high/wait = 10/0/0`, which produced clean MAX77958 direct-HS traffic at
about `2 MHz`. That gives us a practical HS master mode for this board while
keeping the normal ESP-IDF 1 MHz handle as a safe fallback path.

At the maximum BNO085 accelerometer rate, the sample period is `2 ms`. A normal
accelerometer input report is small: the firmware reads the 4-byte SHTP header
and then the 14-byte SHTP accelerometer packet, so the I2C wire time is on the
order of `0.5-0.8 ms` at 400 kHz after protocol overhead. Long status maps are
read in adaptive chunks: at 500 Hz the chunk size shrinks to fit the tight
window, while at lower BNO sample rates the same code uses larger chunks and
refreshes BQ/MAX faster.

Worst-case planning at BNO085 500 Hz:

| Transfer | Wire estimate |
| --- | ---: |
| BNO header + normal accel packet at 400 kHz | about `0.8 ms` |
| BQ25792 16-byte read chunk at 1 MHz | about `0.27 ms` |
| BQ25792 single-register write at 1 MHz | about `0.13 ms` |
| MAX77958 single-byte direct HS read at about 2 MHz | about `0.04-0.05 ms` measured on M1 |
| MAX77958 AP-command write, direct HS `30 + 3` bytes | about `0.20 ms` wire time plus two HS entries |
| MAX77958 33-byte AP response read at 1 MHz | about `0.42 ms` |
| MAX77958 fallback 34-byte AP-command write at 1 MHz | about `0.42 ms` |

With a `2 ms` BNO period and `500 us` guard, a normal BNO packet leaves roughly
`700 us` for background work. That fits one MAX77958 full AP-data chunk or one
BQ25792 chunk. If BNO coalesces a larger input packet, the adaptive chunk logic
shrinks BQ/MAX transfers rather than delaying the next BNO read. The schematic
currently shows `10k` I2C pull-ups, but the live module tested cleanly at 1 MHz;
if a board revision is unstable, verify the actual pull-up values and reduce
them for Fast-Mode Plus before blaming firmware scheduling.

Validated M1 coexistence test:

| Condition | Result |
| --- | --- |
| Firmware / board | `3646a76` on ESP32-S3 M1, 1k I2C pull-ups |
| Duration | `180.96 s` |
| BNO085 accelerometer | `500 Hz` requested, `91,614` reports, `506.3 samples/s` observed |
| BNO085 errors | `0` read errors, `0` parse errors |
| BQ25792 load | `180` full refreshes, about `1 Hz`, `0` errors |
| MAX77958 load | `180` refreshes, about `1 Hz`, `0` public read errors, `0` operation errors |
| MAX77958 direct-HS recovery | `2` direct-HS attempts fell back cleanly; the higher-level MAX operation still completed |
| Control-path errors | `0` HTTP refresh errors, `0` `/status` errors |

That run proves the shared bus can carry the BNO085 at its maximum accelerometer
rate while still fitting charger and USB-C/PD refreshes at about 1 Hz. The
internal BQ25792 and MAX77958 monitor defaults are therefore set to
`APP_BQ25792_READ_INTERVAL_MS = 1000` and
`APP_MAX77958_READ_INTERVAL_MS = 1000`.

`/status` exposes `i2c_realtime_period_us`,
`i2c_realtime_time_to_next_us`, `i2c_background_window_us`, and the
realtime/background lock counters.

The firmware also has optional SCL pulse-count diagnostics for BNO085, BQ25792,
and the MAX77958 direct-HS path. They are disabled by default:

```c
#define APP_BNO085_I2C_MEASURE_SCL_ENABLED 0
#define APP_BQ25792_I2C_MEASURE_SCL_ENABLED 0
#define APP_MAX77958_I2C_HS_DIRECT_MEASURE_SCL_ENABLED 0
```

When enabled, the code uses the ESP32-S3 PCNT peripheral to count SCL rising
edges during selected transactions and exposes the result in `/status`. This is
useful for automated sanity checks and for confirming that the expected device
is actively clocked. It is not a replacement for an oscilloscope when measuring
absolute SCL frequency: the measured interval includes some software/driver
overhead around the I2C transaction, so very short transfers under-report the
actual wire frequency. For precise rise/fall time and duty-cycle checks, use
the oscilloscope; for continuous firmware health, use the PCNT counters.

The practical rule is: keep BNO085 as the realtime owner of the bus, keep BQ/MAX
refresh rates human-scale, and route charger/PD reads through the background
window model. The current validated human-scale refresh target is 1 Hz for both
BQ25792 and MAX77958 while BNO085 runs at 500 Hz.

High-rate accelerometer telemetry is intentionally handled like a small sensor
stream, not like human log text. The BNO085 task does not enqueue accelerometer
samples until the telemetry TCP connection is established, so startup transients
do not fill the queue. Once connected, samples are batched into binary frames:

```text
header: "UWT1", version=1, stream=1, module_id, sample_size=21,
        count_le16, payload_len_le16
sample: uptime_ms_le32, reports_le32,
        x_milli_le32, y_milli_le32, z_milli_le32, accuracy_u8
```

The dashboard also keeps support for the older compact text frame
`A,module,uptime_ms,x_milli,y_milli,z_milli,accuracy,reports` and the older
verbose `T,...,bno085.accel,...` frame, but normal high-rate data should use the
binary `UWT1` stream. `/status` exposes `wireless_telemetry_binary_frames`,
`wireless_telemetry_binary_samples`, and `wireless_telemetry_text_frames` so the
active transport is visible during tests.

The GPS/GNSS path is disabled by default and can be enabled live with runtime
config (`gps=1`) or from the dashboard Settings tab. When disabled, GPIO47 is
held inactive and the ESP32 GPS UART pins are returned to inputs so the receiver
does not waste current through idle-high UART lines. When enabled, the
`gps_service` task powers the receiver, opens UART1 on GPIO18/GPIO17 at 115200
8N1, validates NMEA checksums, and parses GGA, RMC, GSA, GSV, and SkyTraq
`$PSTI,030` summary sentences. `/status` exposes GPS power/UART state, fix
quality, mode, satellites used/in view, HDOP, position, altitude, RTK age/ratio
when present, and parser counters. The dashboard Info tab shows the same GPS
status per module. NTRIP/RTCM correction forwarding is intentionally not enabled yet; the
current implementation is a passive GNSS diagnostic suitable for testing
modules with antennas, currently modules 3 and 5 near the window.

The BQ25792 Li-Po charger monitor runs on the shared I2C bus
(`GPIO9/GPIO10`, address `0x6B`) at 1 MHz. A complete register-window dump
`0x00..0x48` runs at startup, on explicit refresh/configuration changes, and
then every `APP_BQ25792_READ_INTERVAL_MS` (`1s` by default). The full dump is
intentionally split into adaptive register chunks capped by
`APP_BQ25792_REGISTER_READ_CHUNK_BYTES` (`16` by default) with a short gap between
chunks so the charger monitor stays lower priority than the BNO085
accelerometer. At high BNO085 sample rates the chunk size is reduced to fit the
available background I2C window; at lower sample rates the monitor can use larger
chunks and finish refreshes faster. The shared I2C service has explicit
realtime/background locks:
BNO085 reads and configuration writes use the realtime lock, while BQ25792 reads
and writes use the background lock and are deferred during the BNO085 realtime
guard window. BQ25792 `INT` wakes the task for a shorter status/ADC refresh
split into the same small I2C chunks instead of a full raw-map dump, so charger
events can be handled while the BNO085 is running at high sample rates without a
long charger transfer sitting on the bus. `/status` exposes the raw register bytes
as `charger_raw_hex` plus decoded summary fields for part information,
status/fault bytes, ADC control, watchdog state, charge/input limits, `VBAT`,
`VSYS`, `VBUS`, `VAC1`, `VAC2`, `IBUS`, `IBAT`, `TS`, `TDIE`, `D+`, and `D-`.
Firmware also publishes a simple 1S Li-Po state-of-charge estimate derived from
`VBAT`; this is a
voltage-based dashboard aid, not a coulomb-counting fuel gauge. The dashboard
Info tab shows the decoded charger values in the Battery column, the Battery
Charger tab shows a full live table plus human controls for ADC, charge limits,
and safety timers. Raw registers are still available from a service checkbox, and
`tools/bq25792_dump.py --target-list tools/ota_targets.local.txt` prints every
register byte with names.

`TDIE` is the BQ25792 internal die-temperature ADC. The external/battery
temperature input is the `TS` pin, exposed as `charger_ts_percent` in percent of
`REGN`, plus decoded JEITA range bits (`cold`, `cool`, `normal`, `warm`,
`hot`). The PCB connects `TS` to the thermistor network around `R93`, `R94`,
`R96`, and optional connector `TH1`; the dashboard therefore shows the safe raw
quantity `%REGN` and the charger's own range decision. Converting `TS` to degrees
C requires confirming the actual assembled thermistor/resistor network.

Three charger/power side-band signals are wired to the ESP32 and exposed in
`/status`: `INT` on `GPIO4`, `QON_CMD` on `GPIO38`, and `PG` on `GPIO5`.
`INT` is an open-drain active-low pulse output; the firmware enables a pull-up
and uses the falling edge to wake the charger task for an immediate register
refresh. `PG` on `GPIO5` is the board power-good net from the power sheet
through a `100R` series resistor and a `100k` pull-up to the regulator output;
it is reported alongside, but is not the same signal as, the BQ25792 `PG_STAT`
register bit. `GPIO38` does not connect directly to the BQ25792 `~QON` pin: it
drives a BSS138 gate through the QON command net, which has a `100k` pulldown.
Idle `GPIO38=0` is normal; driving the command net high would pull the real
BQ25792 `~QON` pin low. That low pulse can wake the charger from ship mode or,
if held long enough, trigger a system power reset. Firmware currently leaves
`GPIO38` as high-impedance input and only reports its level.

The datasheet confirms that the BQ25792 is not read-only: many configuration
registers are `R/W`, and any I2C write moves the charger from default mode into
host mode and starts/resets the watchdog unless the watchdog is disabled. The
charger does not provide a user NVM profile for these host-side settings: after
POR it starts from defaults derived mainly from the `PROG` pin, and watchdog or
register reset can restore defaults. Firmware must therefore reapply any
required charger policy at startup if we want it guaranteed after reboot.
Firmware stores the dashboard charger policy in ESP32 NVS and reapplies it once
at BQ25792 startup. Refreshes only read status; they do not rewrite charger
configuration. Raw register writes remain live experiments and are not persisted
as policy.

The firmware exposes an authenticated live endpoint at `/config/charger` for
controlled writes. The endpoint disables the watchdog before charger
configuration changes, can enable/disable the ADC, can choose continuous or
one-shot ADC conversion and ADC sample speed, and can set `VSYSMIN`, charging
enable state, charge voltage/current, input voltage/current DPM, and charge
safety timers using human units (`mV`/`mA`/minutes/hours). It also has a guarded
raw register write path (`reg`, `value`, optional
`mask`/`bits`, and `confirm=1`) for datasheet-level experiments.

Dashboard charger ADC controls affect measurement behavior, not charging loops
directly:

| Control | BQ25792 field | Effect |
| --- | --- | --- |
| `ADC enabled` | `ADC_EN` in `REG2E` | Turns the internal ADC on/off. When off, `VBAT`, `VSYS`, `VBUS`, `IBAT`, `IBUS`, temperature, D+, and D- readings may be stale or unavailable. |
| `Rate` | `ADC_RATE` in `REG2E` | `continuous` keeps conversions running in the background. `one shot` performs a conversion sequence on request and then stops. |
| `Sample` | `ADC_SAMPLE[1:0]` in `REG2E` | Selects conversion time and effective resolution: `15 bit / 24 ms`, `14 bit / 12 ms`, `13 bit / 6 ms`, or `12 bit / 3 ms`. Slower settings are quieter; faster settings are noisier. |
| `Average` | `ADC_AVG` in `REG2E` | Enables the charger's running average. This stabilizes displayed values, but it makes fast transients less visible. |

When ADC is enabled from the dashboard, firmware also clears the ADC disable
registers `REG2F` and `REG30`, so all exposed ADC channels are measured. The
dashboard default is `13 bit / 6 ms`, which is a practical middle ground for
slow battery telemetry.

Dashboard charger limit controls affect the charger and NVDC power path:

| Control | BQ25792 field | Step | Effect |
| --- | --- | --- | --- |
| `Charging` | `EN_CHG` in `REG0F` | boolean | Enables or disables battery charging. Disabling it does not power down the board; the system rail can still be powered from `VBUS` through the NVDC power path. |
| `VSYSMIN mV` | `VSYSMIN[5:0]` in `REG00` | `250 mV` | Minimum target for the `SYS` rail when the battery is below the configured system minimum. For our 1S Li-Po modules the datasheet POR/default is `3500 mV`; keep it below `VREG`. |
| `Charge voltage mV` | `VREG[10:0]` in `REG01..REG02` | `10 mV` | Final battery regulation voltage. For a normal 1S Li-Po this is typically around `4200 mV`; setting this too high is unsafe for the cell. |
| `Charge current mA` | `ICHG[8:0]` in `REG03..REG04` | `10 mA` | Maximum battery charge current. The actual current can still be reduced by thermal regulation, input limits, or system-load priority. |
| `VINDPM mV` | `VINDPM[7:0]` in `REG05` | `100 mV` | Input voltage dynamic power management threshold. If `VBUS` droops below this threshold, the charger backs off to avoid collapsing the adapter/USB source. |
| `IINDPM mA` | `IINDPM[8:0]` in `REG06..REG07` | `10 mA` | Maximum current drawn from the input source. System load is served first; the remaining budget is available for battery charging. |
| `ILIM_HIZ clamp` | `EN_EXTILIM` in `REG14` | boolean | Enables the external `ILIM_HIZ` pin clamp. When enabled, the effective input-current limit is the lower of the `IINDPM` register and the analog `ILIM_HIZ` pin setting; disabling it lets software set `IINDPM` above that hardware clamp. |

In short: `Charge voltage` and `Charge current` define the battery charge target.
`Input voltage` and `Input current` define how aggressively the board may load
the external source. `VSYSMIN` protects the system rail when the battery is low.
TI notes that setting battery regulation voltage below the system minimum is not
recommended. In practical terms, `VREG=4200 mV` and `VSYSMIN=4500 mV` is a bad
1S combination: the charger reports `VSYS_STAT=1`, enters the NVDC/SYSMIN power
path loop, and charging behavior becomes harder to reason about. The dashboard
and `tools/bq25792_dump.py` warn when `VSYSMIN >= VREG` or when `VSYSMIN` looks
too high for a 1S pack.
All high-level charger policy values are stored in ESP32 NVS and reapplied once
at charger startup.

Dashboard charger safety-timer controls affect charge-cycle timeout behavior:

| Control | BQ25792 field | Effect |
| --- | --- | --- |
| `Fast timer` | `EN_CHG_TMR` in `REG0E` | Enables/disables the fast-charge safety timer used during CC/CV charging. If this expires, `CHG_TMR_STAT`/`CHG_TMR_FLAG` can explain an early `terminated` state. |
| `Fast duration` | `CHG_TMR[1:0]` in `REG0E` | Selects `5 h`, `8 h`, `12 h`, or `24 h`. Larger batteries or lower charge currents may need a longer timeout. |
| `Pre-charge timer` | `EN_PRECHG_TMR` in `REG0E` | Enables/disables the timer for the low-voltage pre-charge phase. |
| `Pre-charge duration` | `PRECHG_TMR` in `REG0D` | Selects `120 min` or `30 min`. |
| `Trickle timer` | `EN_TRICHG_TMR` in `REG0E` | Enables/disables the fixed 1 h trickle-charge timer for deeply discharged cells. |
| `Top-off timer` | `TOPOFF_TMR[1:0]` in `REG0E` | Optional extra `15/30/45 min` top-off after termination threshold; `0` disables it. |
| `TMR2X` | `TMR2X_EN` in `REG0E` | Doubles active safety timers while input-current/input-voltage DPM or thermal regulation slows charging. |

The dashboard `Reset Charge Cycle` button toggles `EN_CHG` off and back on for
the selected modules. Per the BQ25792 datasheet, stopping and restarting the
charge cycle resets the active fast/pre-charge/trickle safety timers and also
resets top-off timing.

## MAX77958 USB-C PD Controller

The MAX77958 USB-C/PD controller is monitored on the shared I2C bus
(`GPIO9/GPIO10`, address `0x25`). Firmware treats it like a
low-priority background device, the same way the charger monitor is treated:
MAX77958 reads and AP-command writes use the shared I2C background lock, so the
BNO085 realtime accelerometer path can win bus arbitration when it is active.
The AP-command handling follows `UG-7139: MAX77958 Customization Script and
OPCode Command Guide`, Rev. 5 / 7-2024: commands are written as one
`AP_DATAOUT0..32` packet, then confirmed via `APCmdRes` and `AP_DATAIN0..32`.
The regular raw-map refresh is split into adaptive
`APP_MAX77958_REGISTER_READ_CHUNK_BYTES` chunks (`33` bytes by default), so PD
status reads can use large transfers when the accelerometer rate is low, but
shrink to smaller transfers when the BNO085 realtime window is tight.
The monitor runs every `APP_MAX77958_READ_INTERVAL_MS` (`1s` by default), plus
on explicit dashboard refresh or after configuration operations.

MAX77958 has an optional high-speed path for boards with strong enough I2C
pull-ups. The clean implementation does not fork ESP-IDF: it keeps the standard
1 MHz MAX device handle as fallback, sets `HS_EXT_EN` in `I2C_CNFG`, and uses
small local direct helpers for MAX register reads and writes. On M1, after
changing the I2C pull-ups to `1k`, the validated direct profile is
`low/high/wait = 10/0/0`, with HS master code `0x08` plus STOP before each
direct transaction and about `2 MHz` observed on SCL. Direct writes are limited
to `30` data bytes per transaction because the ESP32-S3 I2C FIFO is `32` bytes
and must also hold the I2C address byte and starting register. AP commands are
therefore written as `0x21..0x3E` followed by `0x3F..0x41`; MAX77958 latches the
command when `AP_DATAOUT32` (`0x41`) is written. Runtime AP writes do not do
read-back verification, because that path was validated during M1 bring-up and
the normal firmware should keep the shared I2C bus free for the BNO085.
Direct-HS reads default to single-byte transactions because that is the read
form validated on M1 with zero direct errors; longer read bursts remain
configurable through `APP_MAX77958_I2C_HS_DIRECT_READ_CHUNK_BYTES` for future
oscilloscope-guided testing.

`/status` exposes both decoded fields and a raw register map:
`pd_raw_hex`, device/FW IDs, interrupt/status/mask registers, VBUS ADC range,
BC1.2 charger type, CC pin/orientation/status, PD role/status flags, CTRL1 USB2
switch state, source PDOs advertised by the connected supply, sink PDOs
configured in the MAX77958, PPS defaults, and the last AP-command result. Raw
AP command response bytes are also exposed as `pd_last_response_hex` for
datasheet-level debugging.
It also exposes HS diagnostics:
`pd_i2c_hs_ext_active`, `pd_i2c_hs_direct_read_count`,
`pd_i2c_hs_direct_error_count`, and
`pd_i2c_hs_direct_last_elapsed_us`.

The authenticated live endpoint is `/config/max77958`. Useful operations are:

| Operation | Example | Effect |
| --- | --- | --- |
| Refresh | `/config/max77958?refresh=1` | Requests an immediate status/AP discovery refresh. |
| BC detect | `/config/max77958?bc_trigger=1` | Triggers BC1.2 charger detection. |
| USB2 switch | `/config/max77958?usb2_closed=1` | Opens/closes the D+/D- pass-through switches through CTRL1. |
| Source PDO request | `/config/max77958?source_pdo_pos=2` | Requests one fixed supply profile from the source by advertised PDO position. |
| Sink PDO set | `/config/max77958?sink_pdos=5000:3000,9000:3000` | Writes fixed sink capabilities in `mV:mA` form to RAM by default. Add `sink_pdos_mtp=1` only when intentionally changing the non-volatile MAX77958 profile. |
| Sink PDO read | `/config/max77958?read_sink=1` or `read_sink_mtp=1` | Reads sink PDOs from RAM or MTP. |
| PPS default | `/config/max77958?pps_enabled=1&pps_voltage_mv=9000&pps_current_ma=2000` | Stores the default PPS policy in ESP32 NVS and applies it at MAX77958 startup. |
| APDO request | `/config/max77958?apdo_pos=1&apdo_voltage_mv=9000&apdo_current_ma=2000` | Requests a programmable PPS/APDO contract at runtime, if the source advertises one. |

Changing the module supply voltage is therefore done through USB-C PD
negotiation, not by directly forcing a rail voltage. For fixed adapters, first
read the advertised source PDOs, then request the desired PDO position. For PPS
adapters, use the APDO request with voltage/current in human units; internally
the MAX77958 opcode uses 20 mV voltage units and 50 mA current units.

The dashboard has a `USB-C PD` tab with a live table for all modules, human
controls for Source PDO, Sink PDOs, PPS/APDO, BC detect, USB2 switch, and a
hidden raw-register section. High-level policy settings are stored in ESP32 NVS
and applied once at MAX77958 startup; raw register writes are guarded with
`confirm=1` and are not persisted as policy.

The protocol reference used for the AP-command opcodes is stored locally as
`datasheets/max77958-customization-script-and-opcode-command-guide.pdf`, with
extracted text in `datasheets/extracted_text/max77958-customization-script-and-opcode-command-guide.txt`.

## Local Workflow and VS Code Tasks

Recommended workflow from this folder, in the ESP-IDF v6.0.2 terminal:

```bat
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

If using plain CMD instead of the ESP-IDF terminal, the local wrapper uses the
same IDF installation:

```bat
idf.bat set-target esp32s3
idf.bat build
idf.bat flash monitor
```

`idf.py monitor` needs a real interactive terminal because it handles keyboard
shortcuts and reset commands. Non-interactive runners can capture serial logs
with the local helper instead:

```bat
powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1 -Port COM55 -Seconds 30 -Reset
```

The current VS Code tasks are plain ESP-IDF/helper tasks. Run them from
`Terminal > Run Task...`:

| Task | Purpose |
| --- | --- |
| `Serial Log` | Runs `tools/serial_log.ps1` for a bounded serial capture. |
| `ESP-IDF OTA Upload` | Runs `tools/ota_upload.py --host-ip <ip>` for one module. |
| `ESP-IDF OTA Upload All` | Runs `tools/ota_upload.py --target-list tools/ota_targets.local.txt --parallel 5`. |
| `Wireless Logs` | Runs `tools/wireless_log_listener.py --port <port> --force-color`. |

After Wi-Fi connects, OTA status is available on the board IP:

```bat
curl http://192.168.140.143/status
curl.exe -H "X-OTA-Token: <APP_OTA_PASSWORD>" --data-binary "@build/uwb_esp_idf.bin" http://192.168.140.143/ota
```

For VS Code, use the `ESP-IDF OTA Upload` task. It reads `APP_OTA_PASSWORD`
from `secrets.h` and sends `build/uwb_esp_idf.bin` to the board.

For the shared firmware workflow, OTA can upload the same binary to multiple
boards at once. Create a local target file from `tools/ota_targets.example.txt`,
for example `tools/ota_targets.local.txt`, with one IP or hostname per line:

```bat
powershell -ExecutionPolicy Bypass -File tools\ota_upload.ps1 -TargetList tools\ota_targets.local.txt -Parallel 5
```

On Linux/macOS, use the Python helper:

```sh
python3 tools/ota_upload.py --target-list tools/ota_targets.local.txt --parallel 5
```

For VS Code, use the `ESP-IDF OTA Upload All` task.

Runtime configuration can change the common test settings without rebuilding or
uploading a new firmware image. The endpoint uses the same `X-OTA-Token` header
as OTA and stores values in NVS:

```sh
python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --parallel 5 --mode ranging --tag 1 --anchors 2,3,4,5 \
  --ranging-slot-ms 350 --ranging-gap-ms 500 --reboot
```

Useful mode names are `ranging`, `survey`, `calibration`, `distance`, and
`beacon`. Runtime role changes such as `mode`, `tag`, `anchors`, and survey
`coordinator` should be sent with `--reboot`; timing-only changes can be sent
without reboot and will be picked up by the long-running loops where supported.
The UWB radio channel is also runtime-configurable as `--radio-channel 5` or
`--radio-channel 9`. Send it to all active modules together and reboot so every
DW3000 is reinitialized with the same RF profile.

Calibration examples:

```sh
python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --mode calibration --cal-method three --cal-three 1,2,3 \
  --cal-d01-mm 2000 --cal-d02-mm 2000 --cal-d12-mm 2828 \
  --cal-slot-ms 100 --cal-guard-us 500 --reboot

python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --mode survey --tag 1 --anchors 2,3,4,5 --coord 1 --reboot
```

`GET /status` exposes the active runtime config as `runtime_*` fields. To clear
the override and return to firmware defaults:

```sh
python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt --clear --reboot
```

Wireless logs are part of the normal multi-module workflow. With five modules
spread out in a room, USB serial is only useful for bring-up. The wireless log
service mirrors ESP-IDF logs over a TCP connection from each board to your PC.
Set `APP_WIRELESS_LOG_TARGET` in `components/config/include/app_config.h`, then
run:

```bat
python -u tools/wireless_log_listener.py --port 6055
```

The listener uses ANSI colors when the terminal supports them. Use
`--force-color` to force colors or `--no-color` for plain text. Use
`--output logs/session.log --quiet` to write the full stream to disk while
printing only connection and progress summaries.

For VS Code, use the `Wireless Logs` task.

The local dashboard combines the wireless log stream, module status polling, and
runtime configuration controls in a browser UI:

```sh
python3 tools/uwb_dashboard.py --log-port 6055 --http-port 8780 --open
```

Open `http://127.0.0.1:8780/`. The dashboard has separate log tabs for module
pairs, a combined log view, accelerometer graphs, a status page, runtime and UWB
configuration controls, a Battery Charger tab for BQ25792 ADC/watchdog,
charge/input limit, and a USB-C PD tab for MAX77958 PDO/PPS experiments. Log filters and settings are
persisted in the browser. Calibration distances are entered in centimeters and
rounded to the nearest millimeter before being sent to `/config/runtime`.
Only one program can listen on TCP port 6055 at a time, so stop
`wireless_log_listener.py` before starting the dashboard.

On Linux systems with UFW enabled, open the high-rate telemetry port for the
module subnet. In the current lab network, logs use `6055/tcp` and high-rate
accelerometer telemetry uses `6060/tcp`:

```sh
sudo ufw allow in on wlp0s20f3 from 192.168.139.0/24 to any port 6060 proto tcp
```

The same wireless-log stream can drive a first live 2D view of the tag. The
viewer has no Python package dependencies; it listens on the wireless-log TCP
port and serves a browser UI locally:

```sh
python3 tools/uwb_live_view.py --log-port 6055 --http-port 8765
```

Then open `http://127.0.0.1:8765/`. The default geometry matches the current
2 m square lab setup:

```text
3 ----------- 2
|             |
|      1      |
|             |
4 ----------- 5
```

Use `--anchor ID=X,Y` to override anchor coordinates in meters.
