# Handoff Context

This file is the short restart context for continuing the ESP-IDF UWB work on
another PC or in a fresh Codex session.

## Current State

- Repository: `https://github.com/victorstoica114/UWB_Module_ESP_IDF`
- Last pushed commit at handoff time: `5ba94d8 Add DS-TWR anchor survey skeleton`
- Target board: `ESP32-S3-WROOM-1-N16R8`
- ESP-IDF version used successfully: `v6.0.2`
- Current default runtime is still `APP_RUNTIME_MODE_UWB_DISTANCE_TEST`.
- The DS-TWR two-module flow works and is the chosen base for the project.
- TDoA was investigated conceptually, but we decided to stay on DS-TWR because
  precise anchor clock sync is the hard part.

## Important Local Files

- `secrets.h` is intentionally ignored by Git. Recreate it from
  `secrets.example.h` on the new PC.
- `components/config/include/app_config.h` owns runtime selection, Wi-Fi
  defaults, provisioning flags, wireless log target, and stability-test knobs.
- `components/config/include/uwb_config.h` owns UWB IDs, roles, antenna delay,
  DS-TWR timing, radio profile, diagnostics, and anchor survey settings.
- `components/config/include/board_config.h` owns board wiring.
- `components/app_manager/app_manager.c` decides what runtime starts.
- `components/uwb_dw3000/uwb_dw3000.c` contains DW3000 bring-up, DS-TWR,
  calibration, and the anchor survey skeleton.

## Bring-Up On A New PC

From an ESP-IDF v6.0.2 terminal:

```powershell
git clone https://github.com/victorstoica114/UWB_Module_ESP_IDF.git
cd UWB_Module_ESP_IDF
Copy-Item secrets.example.h secrets.h
```

Edit `secrets.h` with the local Wi-Fi password / OTA credentials. Then build:

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

```c
#define APP_RUNTIME_MODE APP_RUNTIME_MODE_UWB_ANCHOR_SURVEY
```

Do not switch the default casually before validating the new AP/OTA setup,
because `APP_RUNTIME_MODE_UWB_DISTANCE_TEST` is the known working runtime.

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
