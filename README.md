# UWB ESP-IDF

ESP-IDF project for ESP32-S3-WROOM-1-N16R8.

Current step:

- ESP-IDF v6.0.2 project skeleton
- custom 16 MB OTA partition table
- board, application, and UWB settings in `components/config`
- verified status LED blink on GPIO42 via `components/app_manager`
- Wi-Fi STA connection via `components/wifi_service`
- local HTTP OTA via `components/ota_service`
- DW3000 UWB SPI/reset bring-up and random TX/RX beacon smoke test via `components/uwb_dw3000`
- DW3000 hardware TX/RX LED blink configured once at radio init
- optional wireless-log stress test via `components/stability_test_service`

The board boot log confirms 16 MB QIO flash, 8 MB octal PSRAM at 80 MHz, and
the app running from the `ota_0` partition.

Local credentials are read from `secrets.h`. Use `secrets.example.h` as the
template for a new machine.

Project layout:

```text
main/                         app_main entry point
components/config/            board, app, and UWB configuration headers
components/app_manager/       app startup and status LED behavior
components/wifi_service/      Wi-Fi STA connection
components/ota_service/       local authenticated HTTP OTA
components/wireless_log_service/  TCP wireless mirror for ESP-IDF logs
components/uwb_dw3000/        DW3000 bring-up and current beacon smoke test
components/uwb_calibration_service/  antenna delay calibration workflow skeleton
components/uwb_distance_test_service/  two-module distance test workflow skeleton
components/uwb_ranging_service/  anchor/tag ranging workflow skeleton
components/stability_test_service/  optional wireless-log stress generator
reference/                    migration notes and legacy headers kept in-tree
reference/external/           optional local clones of third-party references
```

`components/config/include/board_config.h` is the active board pin map.
Old GPIO naming is kept only under `reference/legacy_headers`.
The old PlatformIO project and cloned third-party repositories are kept local
for inspiration/debugging and are intentionally ignored by Git.

`components/config/include/app_config.h` holds versioned non-secret application
settings: the selected runtime mode, Wi-Fi diagnostics/reconnect behavior,
wireless-log defaults, OTA-adjacent service defaults, and stability test
parameters. `components/config/include/uwb_config.h` holds UWB-only settings:
role, source ID, antenna delay, beacon smoke-test parameters, and future
ranging defaults. Keep local credentials, tokens, hostnames, and the PC log
target IP in `secrets.h`.

Runtime ownership is intentionally narrow:

- `main/app_main.c` only boots the firmware and calls `app_manager_start()`.
- `components/app_manager/app_manager.c` owns what runs on the board and in
  what order.
- Long-lived workflows live in separate services/components, then are selected
  or sequenced by `app_manager`.

The expected UWB workflow split is:

1. `APP_RUNTIME_MODE_UWB_BEACON_SMOKE`: current random beacon TX/RX smoke test
   in `components/uwb_dw3000`.
2. `APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION`: future antenna delay
   calibration entry point in `components/uwb_calibration_service`.
3. `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`: future two-module ruler check entry
   point in `components/uwb_distance_test_service`.
4. `APP_RUNTIME_MODE_UWB_RANGING`: future 4-anchor plus 1-tag positioning
   runtime in `components/uwb_ranging_service`.

Only the beacon smoke mode is implemented end to end right now. The other
workflow components intentionally log their selected configuration and return
success, so we can switch modes while the project structure is taking shape.

For now `APP_RUNTIME_MODE` is defined in `app_config.h`; later it can move to
NVS or an OTA/API setting if we want to switch runtime modes without rebuilding.

The Wi-Fi service scans before connecting and logs each matching AP with BSSID,
channel, RSSI, auth mode, and cipher. If a network broadcasts the same SSID
from multiple radios and one behaves better, lock the local board to that AP in
`secrets.h`:

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

If `APP_UWB_DW_LEDS_ENABLED` is set, the DW3000 configures GPIO2 as RXLED and
GPIO3 as TXLED once during radio init. The chip then drives TX/RX LED blink in
hardware, so the firmware does not add SPI traffic for LED toggling.

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

After Wi-Fi connects, OTA status is available on the board IP:

```bat
curl http://192.168.140.143/status
curl.exe -H "X-OTA-Token: <APP_OTA_PASSWORD>" --data-binary "@build/uwb_esp_idf.bin" http://192.168.140.143/ota
```

In VS Code, use `Terminal > Run Task... > ESP-IDF OTA Upload` for OTA upload.
The task reads `APP_OTA_PASSWORD` from `secrets.h` and sends
`build/uwb_esp_idf.bin` to the board.

Wireless logs use a TCP connection from the board to your PC. Set
`APP_WIRELESS_LOG_TARGET` in `secrets.h`, then run:

```bat
python -u tools/wireless_log_listener.py --port 6055
```

The listener uses ANSI colors when the terminal supports them. Use
`--force-color` to force colors or `--no-color` for plain text.

In VS Code, use `Terminal > Run Task... > Wireless Logs`.

For a UWB plus wireless-log stability run, enable this in
`components/config/include/app_config.h`:

```c
#define APP_STABILITY_LOG_STRESS_ENABLED 1
```

The stress task runs on core 0 and injects wireless-log bursts while the UWB
task keeps exchanging beacons on core 1. Watch `/status` for
`stability_log_stress_generated`, `stability_log_stress_enqueue_failed`,
`wireless_log_dropped`, `uwb_tx_count`, and `uwb_rx_count`.
