# PX1105R firmware update on A2

Date: 2026-08-02
Target: A2 (`uwb-module-2`, `192.168.50.142`)
Result: successful

Only A2 was updated. A3 and the other UWB modules were not addressed.

## Installed receiver firmware

- Image: `STI_03.06.00-01.07.33_Phoenix_RTK_GPSL1L5_BDSB1B2a_GalileoE1E5a_GlonassG1_T_CRC_172b_115200_20240111.bin`
- Raw image size: 1,116,336 bytes
- Raw image SHA-256: `15e9384ed510f85045b76cff0e4756138e2dd2beeffcf11a1d5d0c0a2e044968`
- Exact GNSS Viewer LZMA transfer: 471,243 bytes including the 13-byte LZMA header
- Packed payload checksum (sum8): 189
- Phoenix tag offset: 1,049,152 (`0x100240`)

The receiver reports after reboot:

- kernel: `00030600`
- ODM: `00010721` (01.07.33)
- revision: `0018010b` (2024-01-11)

The previous revision was `0017081f` (2023-08-31).

## Loader protocol finding

GNSS Viewer 2.1.147 uses S-record resource 460 for this Phoenix/PX1105R path. The resource contains 68,492 bytes and loads 22,816 executable bytes at `0x50000000`, with entry point `0x50000000`.

The Viewer does not transmit its stored CRLF line endings verbatim. Each S-record is sent as:

`S-record text` + `LF` + `NUL`

After the final S7 record, the downloaded loader replied:

`$LOADER,LDR03,200303152641` + `NUL` + `END` + `NUL`

The full run then accepted all 471,230 packed payload bytes and returned `END`. The ESP32 power-cycled the receiver with the GNSS ENABLE pin and resumed normal UART service.

## Post-update health check

- GNSS powered: yes
- GPS task running: yes
- UART ready: yes
- last error: `ESP_OK`
- NMEA/PSTI byte and sentence counters increasing
- satellites in view: 11 at the verification instant
- checksum errors: 0
- parse errors: 0

No RTK fix was required to validate the firmware update.

## Reusable procedure

The validated artifacts and fixed one-command workflow now live in
`PX1105R-firmware/validated/`. For any module already running the ESP32 updater
firmware, only its address is needed:

```bash
./tools/gnss_firmware_update.py <ESP32-IP>
```

The loader and packed image are staged in ESP32 PSRAM, flashed with the fixed
profile above, released from PSRAM, and verified through `/status`.

## Five-module fleet validation

On 2026-08-03 the fixed workflow was exercised across all five modules. The
receivers on M1, M3, M4 and M5 were brought to the same revision as M2, which
was already current. A final simultaneous ESP32 OTA then left every module on
the same updater build.

All five receivers reported:

- kernel: `00030600`
- ODM: `00010721`
- revision: `0018010b`
- GPS UART ready and GPS task running
- increasing primary-UART byte counters

M3 was configured as `local_base`, so its active stream was RTCM/binary rather
than NMEA. Its increasing byte and uplink-packet counters confirmed normal data
flow; absence of NMEA sentences in that role is expected.
