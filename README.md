# UWB ESP-IDF

ESP-IDF project for ESP32-S3-WROOM-1-N16R8.

Current step:

- ESP-IDF v6.0.2 project skeleton
- custom 16 MB OTA partition table
- board pin map in `components/board_config`
- verified status LED blink on GPIO42 via `components/app_manager`
- Wi-Fi STA connection via `components/wifi_service`
- local HTTP OTA via `components/ota_service`
- DW3000 UWB SPI/reset bring-up and random TX/RX beacon smoke test via `components/uwb_dw3000`

The board boot log confirms 16 MB QIO flash, 8 MB octal PSRAM at 80 MHz, and
the app running from the `ota_0` partition.

Local credentials are read from `secrets.h`. Use `secrets.example.h` as the
template for a new machine.

Project layout:

```text
main/                         app_main entry point
components/board_config/      board pins, active levels, UART/I2C/SPI mapping
components/app_manager/       app startup and status LED behavior
components/wifi_service/      Wi-Fi STA connection
components/ota_service/       local authenticated HTTP OTA
components/wireless_log_service/  TCP wireless mirror for ESP-IDF logs
components/uwb_dw3000/        DW3000 SPI/reset bring-up and DEV_ID probe
reference/                    migration notes and legacy headers kept in-tree
reference/external/           optional local clones of third-party references
```

`components/board_config/include/board_config.h` is the active board pin map.
Old GPIO naming is kept only under `reference/legacy_headers`.
The old PlatformIO project and cloned third-party repositories are kept local
for inspiration/debugging and are intentionally ignored by Git.

UWB bring-up follows the same first-step pattern used by the Zephyr DW3000
decadriver reference: initialize hardware, reset the chip, then read `DEV_ID`.
The valid IDs currently accepted are `0xDECA0302` and `0xDECA0312`.

After bring-up, the firmware runs a simple same-image UWB smoke test. Each
board sends a `UWBT` beacon at a random interval and spends the rest of the
time in RX. With the same firmware on two boards, wireless logs should show
`UWB TX beacon ...` and `UWB RX beacon ...` lines. The HTTP `/status` response
also exposes `uwb_tx_count`, `uwb_rx_count`, and the last received source/seq.

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
