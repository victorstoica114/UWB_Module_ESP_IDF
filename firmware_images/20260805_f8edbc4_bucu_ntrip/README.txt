UWB ESP32-S3 firmware image

Source commit: f8edbc419398b75f9ff4a0e5a12b0a2230a21153
Source state:  f8edbc4
ESP-IDF:       v6.0.2
Built at:      2026-08-05T08:03:21+03:00

This image contains the validated BUCU00ROU0 NTRIP transport fix, including
incremental HTTP chunk decoding. The compiled binaries contain device and NTRIP
configuration and must be treated as sensitive. This directory is ignored by
Git.

Clean full restore over USB:
  python -m esptool --chip esp32s3 write-flash 0x0 uwb_full_flash.bin

Normal network update:
  python tools/ota_upload.py --firmware uwb_ota.bin --target-list tools/ota_targets.local.txt --parallel 5
