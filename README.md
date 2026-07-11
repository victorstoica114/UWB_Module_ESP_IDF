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
- optional BNO085 accelerometer hardware test via `components/bno085_service`

The board boot log confirms 16 MB QIO flash, 8 MB octal PSRAM at 80 MHz, and
the app running from the `ota_0` partition.

Wi-Fi SSID/password and tokens are read from `secrets.h`. Non-secret
application settings, including provisioning switches, live in `components/config`.
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
| BNO085 reset / interrupt | `40` / `15` |
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
components/bno085_service/    optional BNO085 accelerometer hardware test
components/uwb_dw3000/        DW3000 bring-up, beacon smoke test, DS-TWR loop
components/uwb_calibration_service/  antenna delay calibration entry point
components/uwb_distance_test_service/  two-module distance test entry point
components/uwb_ranging_service/  anchor/tag ranging entry point
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
diagnostics/reconnect behavior, wireless-log defaults, OTA-adjacent
service defaults, and optional sensor-test parameters. `components/config/include/uwb_config.h`
holds UWB-only settings: role, source ID override, antenna delay, beacon
smoke-test parameters, and runtime defaults. Runtime overrides are stored in
NVS through `/config/runtime`. Keep Wi-Fi credentials and OTA tokens in
`secrets.h`.

Runtime ownership is intentionally narrow:

- `main/app_main.c` only boots the firmware and calls `app_manager_start()`.
- `components/app_manager/app_manager.c` owns what runs on the board and in
  what order.
- Long-lived workflows live in separate services/components, then are selected
  or sequenced by `app_manager`.

The application task/core split is kept simple so the UWB loop is isolated from
most Wi-Fi and TCP work:

| Task / service | Core | Notes |
| --- | --- | --- |
| `uwb_dw3000` | 1 | Owns DW3000 init, SPI access, RX/TX, ranging, survey, and calibration loops. |
| `status_led` | 1 | Lightweight GPIO blink task. |
| `wifi_service` | 0 | Owns Wi-Fi STA connect/reconnect management. |
| `ota_service` | 0 | Starts authenticated OTA and runtime-config HTTP handling. |
| `bno085` | 0 | Optional BNO085 accelerometer test when enabled. |
| `bq25792` | 0 | Low-rate charger monitor; uses background I2C access so BNO085 can win bus arbitration. |
| `wireless_log` | unpinned | Drains the log queue and mirrors logs over TCP; FreeRTOS may run it on either core. |
| `wireless_tel` | 1 | Drains high-rate telemetry into batched TCP writes. |
| short-lived reboot tasks | unpinned | Temporary restart helpers after OTA or runtime-config changes. |

ESP-IDF also creates internal Wi-Fi, TCP/IP, event-loop, and HTTP-server tasks.
Those are managed by the framework. The timing-critical UWB transmit instants
are still programmed into the DW3000 with delayed TX, so the radio owns the
sub-microsecond timing rather than the FreeRTOS scheduler.

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

Lab calibration note, 2026-07-11:

- Modules `1`, `2`, and `3` were placed in printed alignment fixtures as a
  2.00 m equilateral triangle.
- Starting delays were `M1=0x3fd6`, `M2=0x4019`, `M3=0x3fdc`.
- The first pass showed symmetric pair errors of about `1-2=-0.05 dtu`,
  `1-3=+44.65 dtu`, and `2-3=-23.90 dtu`.
- Solving the three pair equations produced corrections of `M1=+34 dtu`,
  `M2=-34 dtu`, and `M3=+10 dtu`.
- The delays stored in NVS are now `M1=0x3ff8`, `M2=0x3ff7`,
  `M3=0x3fe6`.
- Verification after reboot measured `1-2=2.001 m`, `1-3=1.996 m`, and
  `2-3=2.000 m`, with per-link standard deviation around `1.0-1.4 cm`.

For additional modules, keep two already calibrated modules fixed as references
and place the uncalibrated module in the third fixture position. Run the
three-module calibration with all three sides set to the measured triangle
edges, but apply only the correction for the new module. If `K1` and `K2` are
known-good references and `U` is the new module, compute the new-module
correction from the symmetric pair errors:

```text
err_K1_U = mean(err_dtu K1->U, err_dtu U->K1)
err_K2_U = mean(err_dtu K2->U, err_dtu U->K2)
correction_U = round(mean(err_K1_U, err_K2_U))
new_delay_U = old_delay_U + correction_U
```

The reference-reference pair should remain close to zero. If it does not, check
the physical placement or RF stability before writing a new delay.

The local dashboard can automate this workflow from `Settings` ->
`Antenna Delay Calibration` with `Auto Calibrate + Apply`. The button:

1. writes the runtime calibration setup and reboots the selected modules,
2. collects the requested number of `UWB CAL sample` log entries for each
   required directed pair,
3. computes symmetric pair errors and solves the antenna-delay corrections, and
4. writes the corrected antenna delay values to NVS for the selected target
   modules.

For a new module, keep two calibrated references in the fixtures, select only
the new module in `Targets`, and use the three-module set that includes both
references. For example, select `module 4` and run `1,2,4` to adjust only M4.
The dashboard configures all three calibration participants automatically and
holds excluded live UWB modules in reset while the calibration runs.
`Min apply DTU` prevents rewriting NVS for tiny noise-level corrections; the lab
default is `2`.
`Reference guard cm` protects the calibrated references: when a reference-only
edge, such as M1-M2 while calibrating M4, exceeds this error threshold, the
dashboard refuses NVS writes even when `Auto apply` is enabled. The lab default
is `2.00 cm`.

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

The BNO085 accelerometer test is controlled from
`components/config/include/app_config.h`:

```c
#define APP_BNO085_ACCEL_TEST_ENABLED 0
#define APP_BNO085_I2C_ADDRESS 0x4A
#define APP_BNO085_I2C_CLOCK_HZ 400000
#define APP_BNO085_ACCEL_INTERVAL_MS 50
#define APP_BNO085_LOG_INTERVAL_MS 1000
#define APP_BNO085_INT_WAIT_TIMEOUT_MS 250
```

Keep `APP_BNO085_ACCEL_TEST_ENABLED` at `0` for normal ranging builds. Set it
to `1` only while verifying the accelerometer hardware. When enabled, the task
runs on core 0, pulses BNO085 reset on GPIO40, enables
the calibrated accelerometer report over I2C/SHTP, then waits on the BNO085
active-low interrupt on GPIO15. The GPIO interrupt is configured as active-level,
not edge-only: on I2C the BNO08X deasserts `H_INTN` as soon as the I2C address
is recognized, so the ISR masks the GPIO interrupt and the task rearms it after
draining the pending SHTP packet. The timeout is only a fallback if an interrupt
is missed. It logs lines like
`BNO085 accel x=... y=... z=... m/s^2 accuracy=... irqs=... wait_timeouts=...`.
Leaving it disabled avoids the extra I2C and CPU work.

Runtime enable/disable is applied live through `/config/runtime` and the
dashboard Settings tab. If BNO085 is disabled, firmware does not keep an
accelerometer task polling a variable; it holds GPIO40 reset active. When BNO085
is enabled, the runtime endpoint starts the task directly. While the task is
running, GPIO15 interrupts, rate changes, and stop requests all wake it with
FreeRTOS task notifications (`INT`, `CONFIG`, `STOP`).

The dashboard Graphs tab plots accelerometer samples as soon as their wireless
telemetry arrives. Its `Timebase` control only changes the visible time window,
oscilloscope-style. `Samples/s` is the actual BNO085/report export rate; the
dashboard applies it as `bno085_sample_hz`, which updates runtime config and
sets both `bno085_accel_interval_ms` and `bno085_log_interval_ms`. The running
BNO085 task picks up rate changes live by sending a new `Set Feature` command,
so no reboot is needed. The BNO08X datasheet lists
`Accelerometer` at a maximum configurable rate of 500 Hz, although I2C bandwidth
and wireless throughput still need to be considered in practice.

High-rate accelerometer telemetry is intentionally handled like a small sensor
stream, not like human log text. The BNO085 task does not enqueue accelerometer
samples until the telemetry TCP connection is established, so startup transients
do not fill the queue. Once connected, samples are batched into binary frames:

```text
header: "UWT1", version=1, stream=1, module_id, sample_size=21,
        count_le16, payload_len_le16
sample: uptime_ms_le32, reports_le32,
        x_milli_le32, y_milli_le32, z_milli_le32, accuracy_u8
```

The dashboard also keeps support for the older compact text frame
`A,module,uptime_ms,x_milli,y_milli,z_milli,accuracy,reports` and the older
verbose `T,...,bno085.accel,...` frame, but normal high-rate data should use the
binary `UWT1` stream. `/status` exposes `wireless_telemetry_binary_frames`,
`wireless_telemetry_binary_samples`, and `wireless_telemetry_text_frames` so the
active transport is visible during tests.

The GPS/GNSS path is disabled by default and can be enabled live with runtime
config (`gps=1`) or from the dashboard Settings tab. When disabled, GPIO47 is
held inactive and the ESP32 GPS UART pins are returned to inputs so the receiver
does not waste current through idle-high UART lines. When enabled, the
`gps_service` task powers the receiver, opens UART1 on GPIO18/GPIO17 at 115200
8N1, validates NMEA checksums, and parses GGA, RMC, GSA, GSV, and SkyTraq
`$PSTI,030` summary sentences. `/status` exposes GPS power/UART state, fix
quality, mode, satellites used/in view, HDOP, position, altitude, RTK age/ratio
when present, and parser counters. The dashboard Info tab shows the same GPS
status per module. NTRIP/RTCM correction forwarding is intentionally not enabled yet; the
current implementation is a passive GNSS diagnostic suitable for testing
modules with antennas, currently modules 3 and 5 near the window.

The BQ25792 Li-Po charger monitor runs on the shared I2C bus
(`GPIO9/GPIO10`, address `0x6B`) at 400 kHz. A complete register-window dump
`0x00..0x48` runs at startup, on explicit refresh/configuration changes, and
then every `APP_BQ25792_READ_INTERVAL_MS` (`10s` by default). The full dump is
intentionally split into small register chunks
(`APP_BQ25792_REGISTER_READ_CHUNK_BYTES`, default `8`) with a short gap between
chunks so the charger monitor stays lower priority than the BNO085
accelerometer. The shared I2C service has explicit realtime/background locks:
BNO085 reads and configuration writes use the realtime lock, while BQ25792 reads
and writes use the background lock and do not start a transaction if a realtime
waiter is present. BQ25792 `INT` wakes the task for a shorter status/ADC refresh
split into two I2C transactions instead of a full raw-map dump, so charger
events can be handled while the BNO085 is running at high sample rates without a
long charger transfer sitting on the bus. `/status` exposes the raw register bytes
as `charger_raw_hex` plus decoded summary fields for part information,
status/fault bytes, ADC control, watchdog state, charge/input limits, `VBAT`,
`VSYS`, `VBUS`, `VAC1`, `VAC2`, `IBUS`, `IBAT`, `TS`, `TDIE`, `D+`, and `D-`.
Firmware also publishes a simple 1S Li-Po state-of-charge estimate derived from
`VBAT`; this is a
voltage-based dashboard aid, not a coulomb-counting fuel gauge. The dashboard
Info tab shows the decoded charger values in the Battery column, the Battery
Charger tab shows a full live table plus human controls for ADC, charge limits,
and safety timers. Raw registers are still available from a service checkbox, and
`tools/bq25792_dump.py --target-list tools/ota_targets.local.txt` prints every
register byte with names.

`TDIE` is the BQ25792 internal die-temperature ADC. The external/battery
temperature input is the `TS` pin, exposed as `charger_ts_percent` in percent of
`REGN`, plus decoded JEITA range bits (`cold`, `cool`, `normal`, `warm`,
`hot`). The PCB connects `TS` to the thermistor network around `R93`, `R94`,
`R96`, and optional connector `TH1`; the dashboard therefore shows the safe raw
quantity `%REGN` and the charger's own range decision. Converting `TS` to degrees
C requires confirming the actual assembled thermistor/resistor network.

Three charger/power side-band signals are wired to the ESP32 and exposed in
`/status`: `INT` on `GPIO4`, `QON_CMD` on `GPIO38`, and `PG` on `GPIO5`.
`INT` is an open-drain active-low pulse output; the firmware enables a pull-up
and uses the falling edge to wake the charger task for an immediate register
refresh. `PG` on `GPIO5` is the board power-good net from the power sheet
through a `100R` series resistor and a `100k` pull-up to the regulator output;
it is reported alongside, but is not the same signal as, the BQ25792 `PG_STAT`
register bit. `GPIO38` does not connect directly to the BQ25792 `~QON` pin: it
drives a BSS138 gate through the QON command net, which has a `100k` pulldown.
Idle `GPIO38=0` is normal; driving the command net high would pull the real
BQ25792 `~QON` pin low. That low pulse can wake the charger from ship mode or,
if held long enough, trigger a system power reset. Firmware currently leaves
`GPIO38` as high-impedance input and only reports its level.

The datasheet confirms that the BQ25792 is not read-only: many configuration
registers are `R/W`, and any I2C write moves the charger from default mode into
host mode and starts/resets the watchdog unless the watchdog is disabled. The
charger does not provide a user NVM profile for these host-side settings: after
POR it starts from defaults derived mainly from the `PROG` pin, and watchdog or
register reset can restore defaults. Firmware must therefore reapply any
required charger policy at startup if we want it guaranteed after reboot.
Firmware stores the dashboard charger policy in ESP32 NVS and reapplies it once
at BQ25792 startup. Refreshes only read status; they do not rewrite charger
configuration. Raw register writes remain live experiments and are not persisted
as policy.

The firmware exposes an authenticated live endpoint at `/config/charger` for
controlled writes. The endpoint disables the watchdog before charger
configuration changes, can enable/disable the ADC, can choose continuous or
one-shot ADC conversion and ADC sample speed, and can set `VSYSMIN`, charging
enable state, charge voltage/current, input voltage/current DPM, and charge
safety timers using human units (`mV`/`mA`/minutes/hours). It also has a guarded
raw register write path (`reg`, `value`, optional
`mask`/`bits`, and `confirm=1`) for datasheet-level experiments.

Dashboard charger ADC controls affect measurement behavior, not charging loops
directly:

| Control | BQ25792 field | Effect |
| --- | --- | --- |
| `ADC enabled` | `ADC_EN` in `REG2E` | Turns the internal ADC on/off. When off, `VBAT`, `VSYS`, `VBUS`, `IBAT`, `IBUS`, temperature, D+, and D- readings may be stale or unavailable. |
| `Rate` | `ADC_RATE` in `REG2E` | `continuous` keeps conversions running in the background. `one shot` performs a conversion sequence on request and then stops. |
| `Sample` | `ADC_SAMPLE[1:0]` in `REG2E` | Selects conversion time and effective resolution: `15 bit / 24 ms`, `14 bit / 12 ms`, `13 bit / 6 ms`, or `12 bit / 3 ms`. Slower settings are quieter; faster settings are noisier. |
| `Average` | `ADC_AVG` in `REG2E` | Enables the charger's running average. This stabilizes displayed values, but it makes fast transients less visible. |

When ADC is enabled from the dashboard, firmware also clears the ADC disable
registers `REG2F` and `REG30`, so all exposed ADC channels are measured. The
dashboard default is `13 bit / 6 ms`, which is a practical middle ground for
slow battery telemetry.

Dashboard charger limit controls affect the charger and NVDC power path:

| Control | BQ25792 field | Step | Effect |
| --- | --- | --- | --- |
| `Charging` | `EN_CHG` in `REG0F` | boolean | Enables or disables battery charging. Disabling it does not power down the board; the system rail can still be powered from `VBUS` through the NVDC power path. |
| `VSYSMIN mV` | `VSYSMIN[5:0]` in `REG00` | `250 mV` | Minimum target for the `SYS` rail when the battery is below the configured system minimum. For our 1S Li-Po modules the datasheet POR/default is `3500 mV`; keep it below `VREG`. |
| `Charge voltage mV` | `VREG[10:0]` in `REG01..REG02` | `10 mV` | Final battery regulation voltage. For a normal 1S Li-Po this is typically around `4200 mV`; setting this too high is unsafe for the cell. |
| `Charge current mA` | `ICHG[8:0]` in `REG03..REG04` | `10 mA` | Maximum battery charge current. The actual current can still be reduced by thermal regulation, input limits, or system-load priority. |
| `VINDPM mV` | `VINDPM[7:0]` in `REG05` | `100 mV` | Input voltage dynamic power management threshold. If `VBUS` droops below this threshold, the charger backs off to avoid collapsing the adapter/USB source. |
| `IINDPM mA` | `IINDPM[8:0]` in `REG06..REG07` | `10 mA` | Maximum current drawn from the input source. System load is served first; the remaining budget is available for battery charging. |
| `ILIM_HIZ clamp` | `EN_EXTILIM` in `REG14` | boolean | Enables the external `ILIM_HIZ` pin clamp. When enabled, the effective input-current limit is the lower of the `IINDPM` register and the analog `ILIM_HIZ` pin setting; disabling it lets software set `IINDPM` above that hardware clamp. |

In short: `Charge voltage` and `Charge current` define the battery charge target.
`Input voltage` and `Input current` define how aggressively the board may load
the external source. `VSYSMIN` protects the system rail when the battery is low.
TI notes that setting battery regulation voltage below the system minimum is not
recommended. In practical terms, `VREG=4200 mV` and `VSYSMIN=4500 mV` is a bad
1S combination: the charger reports `VSYS_STAT=1`, enters the NVDC/SYSMIN power
path loop, and charging behavior becomes harder to reason about. The dashboard
and `tools/bq25792_dump.py` warn when `VSYSMIN >= VREG` or when `VSYSMIN` looks
too high for a 1S pack.
All high-level charger policy values are stored in ESP32 NVS and reapplied once
at charger startup.

Dashboard charger safety-timer controls affect charge-cycle timeout behavior:

| Control | BQ25792 field | Effect |
| --- | --- | --- |
| `Fast timer` | `EN_CHG_TMR` in `REG0E` | Enables/disables the fast-charge safety timer used during CC/CV charging. If this expires, `CHG_TMR_STAT`/`CHG_TMR_FLAG` can explain an early `terminated` state. |
| `Fast duration` | `CHG_TMR[1:0]` in `REG0E` | Selects `5 h`, `8 h`, `12 h`, or `24 h`. Larger batteries or lower charge currents may need a longer timeout. |
| `Pre-charge timer` | `EN_PRECHG_TMR` in `REG0E` | Enables/disables the timer for the low-voltage pre-charge phase. |
| `Pre-charge duration` | `PRECHG_TMR` in `REG0D` | Selects `120 min` or `30 min`. |
| `Trickle timer` | `EN_TRICHG_TMR` in `REG0E` | Enables/disables the fixed 1 h trickle-charge timer for deeply discharged cells. |
| `Top-off timer` | `TOPOFF_TMR[1:0]` in `REG0E` | Optional extra `15/30/45 min` top-off after termination threshold; `0` disables it. |
| `TMR2X` | `TMR2X_EN` in `REG0E` | Doubles active safety timers while input-current/input-voltage DPM or thermal regulation slows charging. |

The dashboard `Reset Charge Cycle` button toggles `EN_CHG` off and back on for
the selected modules. Per the BQ25792 datasheet, stopping and restarting the
charge cycle resets the active fast/pre-charge/trickle safety timers and also
resets top-off timing.

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
The UWB radio channel is also runtime-configurable as `--radio-channel 5` or
`--radio-channel 9`. Send it to all active modules together and reboot so every
DW3000 is reinitialized with the same RF profile.

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
`--force-color` to force colors or `--no-color` for plain text. Use
`--output logs/session.log --quiet` to write the full stream to disk while
printing only connection and progress summaries.

In VS Code, use `Terminal > Run Task... > Wireless Logs`.

The local dashboard combines the wireless log stream, module status polling, and
runtime configuration controls in a browser UI:

```sh
python3 tools/uwb_dashboard.py --log-port 6055 --http-port 8780 --open
```

Open `http://127.0.0.1:8780/`. The dashboard has separate log tabs for module
pairs, a combined log view, accelerometer graphs, a status page, runtime and UWB
configuration controls, and a Battery Charger tab for BQ25792 ADC/watchdog,
charge/input limit, and raw register experiments. Log filters and settings are
persisted in the browser. Calibration distances are entered in centimeters and
rounded to the nearest millimeter before being sent to `/config/runtime`.
Only one program can listen on TCP port 6055 at a time, so stop
`wireless_log_listener.py` before starting the dashboard.

On Linux systems with UFW enabled, open the high-rate telemetry port for the
module subnet. In the current lab network, logs use `6055/tcp` and high-rate
accelerometer telemetry uses `6060/tcp`:

```sh
sudo ufw allow in on wlp0s20f3 from 192.168.139.0/24 to any port 6060 proto tcp
```

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
