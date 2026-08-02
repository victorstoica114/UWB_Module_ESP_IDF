# Validated PX1105R update artifacts

This directory contains the fixed artifacts used by
`tools/gnss_firmware_update.py`. They reproduce the successful A2 update from
2026-08-02; they are not tuning inputs.

## One-command update

The ESP32 firmware must include `components/gnss_firmware_updater`. Then run:

```bash
./tools/gnss_firmware_update.py 192.168.50.142
```

Only the exact ESP32 address changes. The tool performs all of the following:

1. verifies the loader, raw image and packed image against fixed SHA-256 values;
2. verifies every S-record checksum and decompresses the packed image locally;
3. uploads the loader and firmware image into ESP32 PSRAM;
4. starts the fixed PX1105R loader procedure;
5. waits for the GNSS service to report kernel `00030600`, ODM `00010721` and
   revision `0018010b`.

If that version is already installed, the tool exits successfully without
rewriting the receiver.

## Artifacts

- `PX1105R_GNSS_Viewer_2.1.147_loader.srec`
  - GNSS Viewer 2.1.147 resource 460
  - normalized from CRLF to LF for the repository
  - 67,064 source bytes, 1,428 records
  - the ESP32 emits each record as `text + LF + NUL`, totaling 68,492 wire bytes
  - SHA-256: `e236c1397b73a9cc28499513c3ac5e418ebf7ac389c374717a6210d48beef729`
- `PX1105R_01.07.33_viewer.lzma.b64`
  - base64 representation of the exact GNSS Viewer LZMA transfer
  - 471,243 decoded bytes, including the 13-byte LZMA-alone header
  - SHA-256: `44cb71aa9eb45a35fa97726a72ffe57835e7a7d2e33148cd5555260e1a6a4ce8`

The raw firmware remains in the parent directory and has SHA-256
`15e9384ed510f85045b76cff0e4756138e2dd2beeffcf11a1d5d0c0a2e044968`.

## Fixed embedded profile

- command/download UART: 115200 baud
- external loader command: `0x64/0x4f`
- Viewer baud index: 5
- loader buffer index: 0
- transfer block: 8 KiB
- BINSIZ8 terminator: NUL
- UART close/reopen sequence: enabled
- Phoenix tag offset: `0x100240`

The HTTP endpoints are deliberately internal plumbing. They no longer accept
headers that alter baud rates, block sizes, loader modes or timeouts.
