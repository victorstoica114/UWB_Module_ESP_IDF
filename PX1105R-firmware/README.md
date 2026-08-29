# PX1105R GNSS firmware

This directory is the permanent index for PX1105R firmware used by the UWB
modules. GNSS firmware is released separately from ESP32/UWB firmware so that
the same receiver image is not duplicated in every UWB release.

## Recommended release

- Version: `01.07.33`
- Release: [PX1105R GNSS firmware 01.07.33](https://github.com/victorstoica114/UWB_Module_ESP_IDF/releases/tag/gps-px1105r-01.07.33)
- Release asset: `px1105r-firmware-01.07.33.zip`

The release archive contains the original PX1105R images, the validated loader
and packed image used by the ESP32-assisted update path, documentation, and
`SHA256SUMS`.

## ESP32-assisted update

The ESP32 firmware must include `components/gnss_firmware_updater`. From the
repository root, run:

```bash
python tools/gnss_firmware_update.py <ESP32-IP>
```

Only the ESP32 address changes. For the validated artifacts, expected receiver
version, and detailed update procedure, see [validated/README.md](validated/README.md).

## Release policy

- Each PX1105R firmware version receives its own `gps-px1105r-*` release.
- UWB firmware releases link to this index instead of duplicating GPS assets.
- When a newer GPS version is validated, update the recommended release above
  and retain older GPS releases for recovery and traceability.
