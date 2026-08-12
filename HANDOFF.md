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
- NTRIP caster credentials also stay only in `secrets.h`. Set
  `NTRIP_USE_TLS=0` for plain HTTP casters on port 2101 and `1` for TLS
  casters; `ntrip.example.txt` contains a credential-free BUCU00ROU0 example.
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
`coordinator`. For reproducible measurements, timing changes currently also
require a coordinated reboot: the live runtime-config object is not published
atomically and radio loops can otherwise observe old and new frame fields in
the same transition.

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
`Antenna Delay Calibration`. For replacement-module calibration, select the
module being calibrated in `Targets` and use the three-module set that includes
the two calibrated references, for example `Targets = module 4` with calibration
set `1,2,4`. Auto calibration configures the three participants and holds other
live UWB modules in reset during the run. `Reference guard cm` defaults to
`2.00`; if a reference-only edge exceeds that error, NVS writes are blocked even
with auto-apply enabled.

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

## Historical Hot Protocol Switching (2026-07-27)

Current operational policy is to reboot all participating modules together
when changing positioning protocol. The dashboard enforces this policy; the
direct hot-switch endpoint remains diagnostic only. Timing changes also use a
coordinated reboot until a frame-boundary, atomic same-protocol reload exists.
The text below records the earlier hot-switch implementation and test, not the
recommended field workflow.

The Position tab now changes directly among Native DS-TWR, FlexTDOA, and Passive
DS-TWR without rebooting the ESP32. It sends an authenticated
`/config/runtime` request with `hot_switch=1`; the persistent UWB supervisor
cooperatively stops the old radio loop, clears stale solver observations,
reinitializes only the DW3000, and starts the new loop. Wi-Fi, HTTP, wireless
logs, and binary telemetry remain online throughout the transition.

The radio reset must use the DW3000 boot SPI rate: switch from the operational
rate back to 4 MHz, reset/configure the radio, then restore the operational
rate. Keeping the bus at the operational rate across reset causes an apparently
successful switch
followed by RX timeouts. FlexTDOA also restores its double-buffer configuration
when it becomes active. A radio reinitialization failure falls back to a full
ESP32 restart.

Hot switching is accepted only when the runtime mode is the sole effective
configuration change and both the old and new modes are one of the three
positioning protocols. Topology, channel, timing, calibration, diagnostics, and
survey changes still use the reboot path. `/status` now includes
`uwb_runtime_switching`, `uwb_runtime_switch_count`, and
`uwb_last_runtime_switch_ms`.

Live simultaneous testing on all five modules measured 0.623-0.773 s until all
modules were UWB-ready and 0.784-0.932 s until the first complete positioning
data. The final Passive-to-Native test acknowledged in 0.083 s, had all modules
ready in 0.764 s, produced tag ranges in 0.814 s, and refreshed all six
anchor-to-anchor geometry pairs in 2.115 s. Boot counters were unchanged. All
five modules were left in Native DS-TWR mode.

The runtime-config HTTP response is heap/PSRAM-backed. Do not move its large
response buffer back onto the 8192-byte HTTP server task stack: doing so caused
an ESP32 stack panic on every runtime-config POST and was the original reason
the dashboard waited for HTTP status. Native anchor-survey commands use the
13-byte wire length rather than the 64-byte local storage buffer.

## Dashboard Geometry Glitch Guard (2026-07-27)

Occasional 30-100 cm anchor-pair spikes were entering the browser TWR-EKF,
briefly turning the surveyed square into a trapezoid/parallelogram and
triggering repeated automatic relocation resets. A slow maintenance pair could
also age out for one refresh and temporarily cover the canvas with the dynamic
geometry overlay.

The Position tab now conditions every anchor edge before EKF assimilation:

- isolated jumps beyond the 10-16 cm adaptive gate are rejected;
- the last accepted edge remains usable for 15 seconds;
- a relocation needs three persistent samples on at least two changed edges
  sharing one anchor;
- the proposed complete geometry must fit within 12 cm RMS; and
- published anchor coordinates move by at most 5 cm per estimator update.

The Live Anchor Geometry panel shows raw/stable range, robust MAD-derived sigma,
cumulative rejected spikes, held edges, and the last rejected pair/delta.
Native DS-TWR tag positioning also uses a robust 3-of-4 fit and records omitted
tag-range spikes. This is browser/dashboard logic only; no firmware rollout is
needed.

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
- Shared firmware rollouts must use one simultaneous invocation:
  `python3 tools/ota_upload.py --target-list tools/ota_targets.local.txt --parallel 5`.
  Do not upload module-by-module unless an explicit single-device diagnostic
  requires it.
- Build with `.\idf.bat build`.
- Flash with `.\idf.bat -p COMxx flash`.
- Capture logs with `tools\serial_log.ps1`.
- The ESP-IDF build may print `dubious ownership` warnings for the ESP-IDF
  install when run from Codex sandbox; previous builds still completed.
- Current project has pushed references under `reference/external` and
  datasheets under `datasheets`; use them before browsing.

## Adaptive position-only EKF field validation (2026-08-12 evening)

- Branch/worktree remains `localization-raw-calibration` with a deliberately
  dirty tree. Do not discard or reset unrelated changes.
- The dashboard uses one position-only constant-velocity adaptive EKF per
  protocol and tag. It never consumes IMU data. Raw UWB remains authoritative
  and can be displayed beside the EKF trail.
- Controlled dynamic captures are in
  `reports/uwb_adaptive_ekf_field_20260812/`:
  - `flex_valid_rtk_ekf_walk_02.flextdoa.jsonl` (60 s);
  - `passive_valid_rtk_ekf_walk_02.passive_ds.jsonl` (60 s);
  - `passive_agile_ekf_validation_03.passive_ds.jsonl` (45 s).
- FlexTDOA result, 471 RTK-fixed pairs and 4.3 mm anchor-fit RMS: EKF changed
  raw RMSE/P95/P99/max from 14.78/25.03/33.76/53.07 cm to
  13.81/24.48/32.06/46.09 cm. Keep the current Flex profile.
- The first valid Passive capture had a 2.24 m rigid tag-reference offset even
  though anchor fit was 10.3 mm. Use its de-biased shape metrics only; do not
  interpret its absolute tag RMSE.
- Passive was retuned from Qmax=120, min std=0.03 m, measurement scale=0.6 to
  Qmax=480, min std=0.02 m, measurement scale=0.5. Qmin=20 and the 200 ms gap
  reset remain unchanged. The new profile is deployed on the Raspberry
  dashboard; no ESP OTA is required.
- Independent live validation of the Passive profile used 359 RTK pairs and a
  1.6 mm anchor-fit RMS. Relative to de-biased raw it improved RMSE by 1.24 mm
  and P99 by 2.41 mm, kept the maximum equal, shortened path length by 6.8%,
  but worsened P95 by 2.65 mm. Therefore Passive EKF remains optional, not a
  proven accuracy replacement for raw.
- A 168-profile cross-capture sweep found no Passive configuration that
  improved RMSE, P95, P99 and maximum on both RTK-valid walks. Do not continue
  blind scalar tuning without additional repeated tracks or a better motion
  model.
- After deployment the Raspberry reported 5 log clients, 5 telemetry clients,
  5/5 HTTP online and 5/5 RTK Fixed in `uwb_passive_ds_twr` mode.

## Final three-protocol dynamic capture (2026-08-12 night)

- Final 90 s walks are archived losslessly in
  `reports/uwb_final_dynamic_20260812/` together with capture summaries and
  position-only EKF replay summaries:
  - `passive_final_dynamic_01.passive_ds.jsonl`;
  - `flex_final_dynamic_01.flextdoa.jsonl`;
  - `native_final_dynamic_01.native_ds.jsonl`.
- Capture event totals were:
  - Passive: 6,451 position records, 3,601 GPS fixes, 53,311 UWB records;
  - Flex: 2,256 position records, 3,530 GPS fixes, 66,471 UWB records;
  - Native: 2,302 position records, 3,466 GPS fixes, 17,392 UWB records.
- Independent-position continuity from replay:
  - Passive 23.34 Hz, 84 gaps over 100 ms, maximum 500 ms;
  - Flex 25.03 Hz, 26 gaps over 120 ms, maximum 360 ms;
  - Native 25.53 Hz, one gap over 120 ms, maximum 130 ms.
- Native and Passive RTK anchor-registration fits are valid (0.69 cm and
  0.52 cm RMS). Flex is not yet valid for absolute accuracy ranking: its fixed
  runtime geometry generation 99 gives a 33.29 cm RTK anchor-fit RMS after the
  A4/GPS changes. Preserve the capture for continuity and raw-observation
  replay, but either re-solve it with the current RTK anchor geometry or apply
  and verify fresh Flex geometry before another live absolute-accuracy run.
- Default replay showed Native raw/EKF RMSE 19.76/19.18 cm and P95
  35.17/34.55 cm. Passive raw/EKF RMSE 13.96/13.97 cm and P95 26.25/26.53 cm,
  so the Passive EKF still does not pass the accuracy gate. Flex replay values
  must not be presented as absolute accuracy until the geometry mismatch is
  resolved.
- All modules were left in `uwb_ranging` (Native DS-TWR) after the campaign.
