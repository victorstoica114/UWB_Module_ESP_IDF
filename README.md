# UWB ESP-IDF

ESP-IDF project for ESP32-S3-WROOM-1-N16R8.

Current step:

- ESP-IDF v6.0.2 project skeleton
- custom 16 MB OTA partition table
- board, application, and UWB settings in `components/config`
- verified status LED blink on GPIO42 via `components/app_manager`
- Wi-Fi STA connection via `components/wifi_service`
- local HTTP OTA via `components/ota_service`
- authenticated HTTP runtime configuration via `components/ota_service`
- DW3000 UWB SPI/reset bring-up and random TX/RX beacon smoke test via `components/uwb_dw3000`
- DW3000 hardware RXOK/SFD/RX/TX LED blink configured once at radio init
- first DS-TWR two-module distance test runtime
- antenna delay calibration runtime for two-module and three-module setups
- sequential 1-tag/4-anchor DS-TWR ranging runtime
- optional BNO085 accelerometer hardware test via `components/bno085_service`

The board boot log confirms 16 MB QIO flash, 8 MB octal PSRAM at 80 MHz, and
the app running from the `ota_0` partition.

Credentials and tokens are read from `secrets.h`. Non-secret application
settings, including provisioning switches, live in `components/config`.
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
The old PlatformIO project and cloned third-party repositories are kept local
for inspiration/debugging and are intentionally ignored by Git.

`components/config/include/app_config.h` holds versioned non-secret application
settings: the default runtime mode, persistent identity provisioning, Wi-Fi
SSID and diagnostics/reconnect behavior, wireless-log defaults, OTA-adjacent
service defaults, and optional sensor-test parameters. `components/config/include/uwb_config.h`
holds UWB-only settings: role, source ID override, antenna delay, beacon
smoke-test parameters, and runtime defaults. Runtime overrides are stored in
NVS through `/config/runtime`. Keep passwords and OTA tokens in `secrets.h`.

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
| `wifi_service` | 0 | Owns Wi-Fi STA connect/reconnect management. |
| `ota_service` | 0 | Starts authenticated OTA and runtime-config HTTP handling. |
| `bno085` | 0 | Optional BNO085 accelerometer test when enabled. |
| `wireless_log` | unpinned | Drains the log queue and mirrors logs over TCP; FreeRTOS may run it on either core. |
| short-lived reboot tasks | unpinned | Temporary restart helpers after OTA or runtime-config changes. |

ESP-IDF also creates internal Wi-Fi, TCP/IP, event-loop, and HTTP-server tasks.
Those are managed by the framework. The timing-critical UWB transmit instants
are still programmed into the DW3000 with delayed TX, so the radio owns the
sub-microsecond timing rather than the FreeRTOS scheduler.

The expected UWB workflow split is:

1. `APP_RUNTIME_MODE_UWB_BEACON_SMOKE`: current random beacon TX/RX smoke test
   in `components/uwb_dw3000`.
2. `APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION`: antenna delay
   calibration entry point in `components/uwb_calibration_service`.
3. `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`: current two-module DS-TWR ruler check
   entry point in `components/uwb_distance_test_service`.
4. `APP_RUNTIME_MODE_UWB_RANGING`: current sequential 4-anchor plus 1-tag
   DS-TWR ranging runtime in `components/uwb_ranging_service`.
5. `APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY`: anchor-to-anchor survey/runtime
   diagnostics in `components/uwb_anchor_survey_service`.

The beacon smoke mode, distance-test mode, antenna-delay calibration workflows,
anchor survey, and multi-anchor ranging runtime are implemented for the current
lab workflow.

`APP_RUNTIME_MODE` remains the firmware default. The effective runtime mode and
common test parameters can be overridden at runtime through NVS using the
authenticated `/config/runtime` HTTP endpoint.

For a step-by-step explanation of the current DS-TWR ranging protocol,
including the message diagram, timing table, and distance formula, see
[`docs/uwb-ranging-protocol/README.md`](docs/uwb-ranging-protocol/README.md).

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

For the three-module method, each configured module initiates to the other two
and responds when addressed. The logs contain directed pair statistics such as
`1->2`, `2->1`, `1->3`, etc., which can be compared against the measured EDM
geometry. The three distance macros can all be equal for an equilateral setup
or can hold the measured distance of each triangle edge. Use the final radio
configuration and a known distance such as 2-5 m; 20 cm is useful for smoke
testing, not for calibration.

`APP_RUNTIME_MODE_UWB_RANGING` is the current 1-tag/4-anchor runtime. The same
firmware image runs on all modules; the runtime config selects tag ID and anchor
IDs, while persistent NVS identity keeps each module's hostname, module ID, UWB
role, and calibrated antenna delay.

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
#define APP_BNO085_I2C_CLOCK_HZ 100000
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

The dashboard Graphs tab plots accelerometer samples as soon as their wireless
log lines arrive. Its `Timebase` control only changes the visible time window,
oscilloscope-style. `Samples/s` is the actual BNO085/report export rate; the
dashboard applies it as `bno085_sample_hz`, which updates runtime config and
sets both `bno085_accel_interval_ms` and `bno085_log_interval_ms`. The running
BNO085 task picks up rate changes live by sending a new `Set Feature` command,
so no reboot is needed for rate-only changes. The BNO08X datasheet lists
`Accelerometer` at a maximum configurable rate of 500 Hz, although I2C bandwidth
and wireless log throughput still need to be considered in practice.

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
(`GPIO9/GPIO10`, address `0x6B`) and performs a read-only dump of the complete
register window `0x00..0x48` every `APP_BQ25792_READ_INTERVAL_MS` (`10s` by
default). The dump is intentionally split into small register chunks
(`APP_BQ25792_REGISTER_READ_CHUNK_BYTES`, default `8`) with a short gap between
chunks so the charger monitor stays lower priority than the BNO085
accelerometer. `/status` exposes the raw register bytes as `charger_raw_hex`
plus decoded summary fields for part information, status/fault bytes, ADC
control, `VBAT`, `VSYS`, `VBUS`, `VAC1`, `VAC2`, `IBUS`, `IBAT`, `TS`, `TDIE`,
`D+`, and `D-`. The dashboard Info tab shows the decoded charger values in the
Battery column, and `tools/bq25792_dump.py --target-list
tools/ota_targets.local.txt` prints every register byte with names.

Three BQ25792 side-band signals are wired to the ESP32 and exposed in
`/status`: `INT` on `GPIO4`, `QON_CMD` on `GPIO38`, and `PG` on `GPIO5`.
`INT` is an open-drain active-low pulse output; the firmware enables a pull-up
and uses the falling edge to wake the charger task for an immediate register
refresh. `PG` is sampled as a power-good flag and reported alongside the
`PG_STAT` register bit. `GPIO38` does not connect directly to the BQ25792
`~QON` pin: it drives a BSS138 gate through the QON command net, which has a
`100k` pulldown. Idle `GPIO38=0` is normal; driving the command net high would
pull the real BQ25792 `~QON` pin low. That low pulse can wake the charger from
ship mode or, if held long enough, trigger a system power reset. Firmware
currently leaves `GPIO38` as high-impedance input and only reports its level.

The datasheet confirms that the BQ25792 is not read-only: many configuration
registers are `R/W`, and any I2C write moves the charger from default mode into
host mode and starts/resets the watchdog unless the watchdog is disabled.
Firmware currently leaves `charger_config_writes_enabled=false`; it reads and
reports the register map without changing charge limits, ADC enable, watchdog,
or masks. This is intentional for the first bring-up pass.

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

In VS Code, use `Terminal > Run Task... > Serial Log`.

After Wi-Fi connects, OTA status is available on the board IP:

```bat
curl http://192.168.140.143/status
curl.exe -H "X-OTA-Token: <APP_OTA_PASSWORD>" --data-binary "@build/uwb_esp_idf.bin" http://192.168.140.143/ota
```

In VS Code, use `Terminal > Run Task... > ESP-IDF OTA Upload` for OTA upload.
The task reads `APP_OTA_PASSWORD` from `secrets.h` and sends
`build/uwb_esp_idf.bin` to the board.

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

In VS Code, use `Terminal > Run Task... > ESP-IDF OTA Upload All`.

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
  --cal-d01-mm 2000 --cal-d02-mm 2000 --cal-d12-mm 2828 --reboot

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

In VS Code, use `Terminal > Run Task... > Wireless Logs`.

The local dashboard combines the wireless log stream, module status polling, and
runtime configuration controls in a browser UI:

```sh
python3 tools/uwb_dashboard.py --log-port 6055 --http-port 8780 --open
```

Open `http://127.0.0.1:8780/`. The dashboard has separate log tabs for module
pairs, a combined log view, a status page, and runtime/calibration controls.
Log filters and settings are persisted in the browser. Calibration distances are
entered in centimeters and rounded to the nearest millimeter before being sent
to `/config/runtime`.
Only one program can listen on TCP port 6055 at a time, so stop
`wireless_log_listener.py` before starting the dashboard.

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
