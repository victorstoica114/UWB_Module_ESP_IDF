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
- optional wireless-log stress test via `components/stability_test_service`

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
components/uwb_dw3000/        DW3000 bring-up, beacon smoke test, DS-TWR loop
components/uwb_calibration_service/  antenna delay calibration entry point
components/uwb_distance_test_service/  two-module distance test entry point
components/uwb_ranging_service/  anchor/tag ranging entry point
components/stability_test_service/  optional wireless-log stress generator
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
service defaults, and stability test parameters. `components/config/include/uwb_config.h`
holds UWB-only settings: role, source ID override, antenna delay, beacon
smoke-test parameters, and runtime defaults. Runtime overrides are stored in
NVS through `/config/runtime`. Keep passwords and OTA tokens in `secrets.h`.

Runtime ownership is intentionally narrow:

- `main/app_main.c` only boots the firmware and calls `app_manager_start()`.
- `components/app_manager/app_manager.c` owns what runs on the board and in
  what order.
- Long-lived workflows live in separate services/components, then are selected
  or sequenced by `app_manager`.

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
`--force-color` to force colors or `--no-color` for plain text.

In VS Code, use `Terminal > Run Task... > Wireless Logs`.

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

For a UWB plus wireless-log stability run, enable this in
`components/config/include/app_config.h`:

```c
#define APP_STABILITY_LOG_STRESS_ENABLED 1
```

The stress task runs on core 0 and injects wireless-log bursts while the UWB
task keeps running on core 1. Watch `/status` for
`stability_log_stress_generated`, `stability_log_stress_enqueue_failed`,
`wireless_log_dropped`, `uwb_tx_count`, and `uwb_rx_count`.
