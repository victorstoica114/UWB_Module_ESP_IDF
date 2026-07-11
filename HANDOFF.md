# Handoff Context

This file is the short restart context for continuing the ESP-IDF UWB work on
another PC or in a fresh Codex session.

## Current State

- Repository: `https://github.com/victorstoica114/UWB_Module_ESP_IDF`
- Last pushed commit before the runtime-config work: `5325d58`
- Target board: `ESP32-S3-WROOM-1-N16R8`
- ESP-IDF version used successfully: `v6.0.2`
- Current default runtime is `APP_RUNTIME_MODE_UWB_RANGING`.
- Runtime mode and common UWB test parameters can now be overridden over Wi-Fi
  with authenticated `/config/runtime` commands stored in NVS.
- GPS/GNSS support is implemented as a disabled-by-default runtime service.
  `gps=1` powers the PX1105R/PX1125R-class receiver, reads NMEA on UART1
  GPIO18/GPIO17 at 115200 8N1, parses GGA/RMC/GSA/GSV/`$PSTI,030`, and exposes
  fix status in `/status` and the dashboard Info tab. NTRIP/RTCM correction
  forwarding is not enabled yet.
- BQ25792 charger support is implemented as a read-only runtime monitor on the
  shared I2C bus. It reads registers `0x00..0x48` in small chunks every 10s or
  sooner after an `INT` pulse, exposes `charger_raw_hex` and decoded
  battery/charger fields through `/status`, and shows them in the dashboard
  Battery column. Datasheet review says config access is available through
  `R/W` registers, but firmware currently does not write them.
- The DS-TWR two-module flow works and is the chosen base for the project.
- TDoA was investigated conceptually, but we decided to stay on DS-TWR because
  precise anchor clock sync is the hard part.

## Important Local Files

- `secrets.h` is intentionally ignored by Git. Recreate it from
  `secrets.example.h` on the new PC.
- `components/config/include/app_config.h` owns runtime selection, Wi-Fi
  behavior defaults, provisioning flags, wireless log target, and stability-test
  knobs. Wi-Fi SSID/password stay in `secrets.h`.
- `components/config/include/uwb_config.h` owns UWB IDs, roles, antenna delay,
  DS-TWR timing, radio profile, diagnostics, and anchor survey settings.
- `components/config/include/app_runtime_config.h` defines the NVS-backed
  runtime override schema.
- `components/config/include/board_config.h` owns board wiring.
- `components/app_manager/app_manager.c` decides what runtime starts.
- `components/uwb_dw3000/uwb_dw3000.c` contains DW3000 bring-up, DS-TWR,
  calibration, and the anchor survey skeleton.
- `components/gps_service/gps_service.c` contains the optional passive GPS/NMEA
  reader. Modules `3` and `5` currently have GPS antennas and are the expected
  first live test targets. It reports satellites used and satellites in view
  separately, which helps diagnose a receiver that sees sky but does not have a
  fix yet.
- `components/i2c_bus_service/` owns the shared I2C master bus on GPIO9/GPIO10.
  BNO085 and BQ25792 both use it. BNO085 takes the bus through the realtime lock;
  BQ25792 uses the background lock and yields whenever a realtime waiter exists.
- `components/wireless_telemetry_service/` owns high-rate TCP telemetry. Keep
  human-readable diagnostics on the wireless log path, but send dense sensor
  samples as framed binary (`UWT1`) batches. The BNO085 path skips high-rate
  enqueueing until TCP telemetry is connected, then sends little-endian integer
  samples so the ESP32 does not spend CPU formatting text.
- `components/charger_service/` owns the BQ25792 monitor. Use
  `tools/bq25792_dump.py --target-list tools/ota_targets.local.txt` to inspect
  every charger register byte after OTA. Side-band signals are `INT=GPIO4`,
  `BOARD_PG=GPIO5`, and `QON_CMD=GPIO38`. `BOARD_PG` is the board/regulator
  power-good net and is distinct from the BQ25792 `PG_STAT` register bit.
  `GPIO38` drives a BSS138 gate; idle low is normal, and driving it high would
  pull the real BQ25792 `~QON` pin low. Keep QON_CMD high-Z/read-only until an
  explicit guarded pulse command is needed.

## Bring-Up On A New PC

From an ESP-IDF v6.0.2 terminal:

```powershell
git clone https://github.com/victorstoica114/UWB_Module_ESP_IDF.git
cd UWB_Module_ESP_IDF
Copy-Item secrets.example.h secrets.h
```

Edit `secrets.h` with the local Wi-Fi SSID/password / OTA credentials. Then
build:

```powershell
.\idf.bat set-target esp32s3
.\idf.bat build
```

Flash a board:

```powershell
.\idf.bat -p COM55 flash
```

For non-interactive serial logs, prefer the helper instead of `idf.py monitor`:

```powershell
powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1 -Port COM55 -Seconds 30 -Reset
```

`idf.py monitor` needs a real interactive TTY; Codex sessions often do not have
one.

## Known Hardware/Identity Plan

Final deployment target:

- Module `1`: tag / mobile point
- Modules `2`, `3`, `4`, `5`: fixed anchors
- Anchor survey coordinator: anchor `2`

Provision identities once per board through `app_config.h`:

```c
#define APP_IDENTITY_PROVISION_ENABLED 1
#define APP_IDENTITY_PROVISION_MODULE_ID 2
#define APP_IDENTITY_PROVISION_UWB_ROLE_ENABLED 1
#define APP_IDENTITY_PROVISION_UWB_ROLE APP_UWB_ROLE_ANCHOR
```

Flash once, confirm logs say it provisioned NVS, then set both provisioning
flags back to `0` and flash the common firmware again. For module `1`, use
`APP_UWB_ROLE_TAG`.

The normal firmware derives UWB source ID and hostname from the persistent NVS
module ID.

## Current Runtime Modes

Defined in `components/config/include/app_config.h`:

- `APP_RUNTIME_MODE_UWB_BEACON_SMOKE`
- `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`
- `APP_RUNTIME_MODE_UWB_ANTENNA_DELAY_CALIBRATION`
- `APP_RUNTIME_MODE_UWB_RANGING`
- `APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY`

To activate anchor survey later:

```sh
python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --mode survey --tag 1 --anchors 2,3,4,5 --coord 1 --reboot
```

Use `--reboot` when changing `mode`, `tag`, `anchors`, or survey
`coordinator`. Timing-only settings can be updated without reboot where the
running loop reads them dynamically.

Other useful examples:

```sh
python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --mode ranging --tag 1 --anchors 2,3,4,5 --reboot

python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --mode calibration --cal-method three --cal-three 1,2,3 \
  --cal-d01-mm 2000 --cal-d02-mm 2000 --cal-d12-mm 2828 --reboot

python3 tools/runtime_config.py --target-list tools/ota_targets.local.txt \
  --clear --reboot
```

## DS-TWR Status

Current DS-TWR features:

- Hardware delayed TX for `RESP` and `FINAL`.
- DW3000 IRQ-backed TX/RX wait path with polling fallback.
- Clock-offset compensation flag:
  `APP_UWB_DISTANCE_TEST_CLOCK_OFFSET_CORRECTION`.
- Diagnostics and DW3000 event counters.
- DW3000 hardware LED configuration for RXOK/SFD/RX/TX.

Last useful stability result around 1 m:

- `114` measurements in `120 s`
- `0` failed exchanges
- `0` timeout lines
- average around `1.317 m`
- stddev around `2.2 cm`
- p05-p95 around `7.3 cm`

Distance bias is expected before antenna delay calibration.

## Antenna Delay Calibration Snapshot

2026-07-11: modules `1`, `2`, and `3` were calibrated in printed alignment
fixtures as a 2.00 m equilateral triangle.

- Final NVS antenna delays:
  - `M1=0x3ff8`
  - `M2=0x3ff7`
  - `M3=0x3fe6`
- Verification after reboot:
  - `1-2=2.001 m`
  - `1-3=1.996 m`
  - `2-3=2.000 m`
- Per-link standard deviation was around `1.0-1.4 cm`.

Modules `4` and `5` still need antenna delay calibration. Suggested workflow:
keep two calibrated modules fixed as references, put the uncalibrated module in
the third fixture position, run three-module calibration, and apply only the
new module's correction. The reference-reference pair is the sanity check; if it
moves away from 2.00 m, fix the geometry/RF stability before writing NVS.

The dashboard now has `Auto Calibrate + Apply` in `Settings` ->
`Antenna Delay Calibration`. Use `Adjust modules` to restrict NVS writes to the
module being calibrated, for example `4` with calibration set `1,2,4`.

## Anchor Survey Skeleton

The new anchor survey skeleton is committed but not yet hardware-tested with
all five boards.

Default anchor survey settings in `uwb_config.h`:

- tag: `1`
- anchors: `[2, 3, 4, 5]`
- coordinator: `2`
- pair order:
  - `2 -> 3`
  - `2 -> 4`
  - `2 -> 5`
  - `3 -> 4`
  - `3 -> 5`
  - `4 -> 5`

Collision avoidance approach:

- Anchor `2` coordinates the round.
- If anchor `2` is the initiator, it starts DS-TWR directly.
- If another anchor must initiate, anchor `2` sends a `SURVEY_CMD`.
- Tag `1` only listens and logs passive frames.
- Responders answer normal DS-TWR `POLL` frames addressed to them.

Expected useful log prefix:

```text
ANCHOR_SURVEY result pair=2-3 seq=... distance=...
```

This is a first skeleton. The next session should expect to tune slot timing and
failure recovery after testing with actual boards.

## Next Intended Steps

1. Move to the new PC and pull `main`.
2. Update `secrets.h` and possibly `APP_WIFI_SSID` / wireless log target for
   the new AP.
3. Test OTA with the new AP.
4. Provision all five module IDs and roles if not already done.
5. Test OTA to all five boards at the same time.
6. Run antenna delay calibration.
7. Enable `APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY` and test inter-anchor distances.
8. Compare anchor-survey DS-TWR distances with tape-measure distances.
9. Build the real 4-anchor + 1-tag ranging runtime on top of the verified
   anchor layout.

## Notes For The Next Codex Instance

- Use `rg` for search.
- Use `apply_patch` for manual edits.
- Do not use destructive Git commands unless explicitly asked.
- Build with `.\idf.bat build`.
- Flash with `.\idf.bat -p COMxx flash`.
- Capture logs with `tools\serial_log.ps1`.
- The ESP-IDF build may print `dubious ownership` warnings for the ESP-IDF
  install when run from Codex sandbox; previous builds still completed.
- Current project has pushed references under `reference/external` and
  datasheets under `datasheets`; use them before browsing.
