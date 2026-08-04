# FlexTDOA clean reimplementation

This document is the executable design contract for replacing the existing
FlexTDOA runtime. The reference is *FlexTDOA: Robust and Scalable
Time-Difference of Arrival Localization Using Ultra-Wideband Devices*, IEEE
Access, 2023. The implementation targets the paper's CI-CR downlink TDOA
variant, with a passive tag.

## Protocol invariants

The first version of the new runtime must preserve these invariants:

| Area | Required behavior |
| --- | --- |
| Participation | Anchors transmit; the tag remains passive. |
| Schedule | TDMA with `M` slots, one request followed by `K` ordered responses. |
| Diversity | Both initiator and response order change round-robin (CI-CR). |
| Synchronization | Every localization packet carries the continuously increasing 32-bit slot ID. |
| Clock correction | Correct each responder processing interval from its received-packet DW3000 CFO, using Eq. (12), before Eq. (8). |
| Observation | `d_j - d_i = c * (T_j - T_i - corrected_processing_j - tof_ij)`. |
| Collection | An observation belongs to exactly one slot. Never combine timestamps from different slots. |
| Geometry | Anchor positions are fixed during a localization run. |
| Solver | AlgMin-style least-squares over raw range differences; no motion filter in the reference path. |

The clean runtime will not depend on the old FlexTDOA solver or its dynamic
range-derived geometry. The dashboard configuration transport and ranging
settings UI remain reusable infrastructure.

## Paper timing profile

Equation (20) defines the slot duration as:

```text
t_slot = t_guard + t_request + t_request_process
       + K * t_response + K * t_response_process
```

The paper's evaluation parameters are:

- guard: 250 us
- request subslot: 2000 us
- request processing: 250 us
- response subslot: 250 us
- response processing: 600 us per responder

For the current `K=3`, this is 5050 us per slot and 20200 us for an `M=4`
frame. This profile is the correctness baseline. Faster profiles may remain in
the dashboard as explicitly experimental profiles, but are not defaults.

## Packet contract

The localization payload follows Figure 3 in field order and width:

```text
message type                 1 byte
slot ID                      4 bytes
source ID                    2 bytes
number of destinations (K)  1 byte
K destination IDs           K bytes
processing time              4 bytes
previous TWR responder ID    2 bytes
previous TWR measurement     2 bytes
previous slot ID             2 bytes
```

All multi-byte project fields are encoded little-endian. Destination IDs stay
8-bit, as in the paper's implementation. A previous TWR measurement is encoded
in millimeters; zero means unavailable. Management/configuration messages are
separate from this localization payload.

## Runtime layers

1. `flextdoa_protocol`: pure C packet codec, CI-CR schedule, paper timing, and
   Eq. (8)/(12) observation math. It has no ESP-IDF or DW3000 dependencies.
2. radio runtime: delayed TX/RX state machine and a slot-local collector. It
   owns DW3000 timestamps and CFO samples and must release/re-arm RX early
   enough to receive every response.
3. solver: a small AlgMin-compatible 2D least-squares implementation. It only
   accepts complete, internally consistent observations and fixed geometry.
4. service/dashboard adapters: configuration, telemetry, OTA, and comparison
   against GPS RTK.

The radio collector is designed around one RX record per packet. Copying a
record to RAM must not block re-arming the DW3000 for the next response. Solver
and telemetry work run after the response train, outside the radio hot path.

## GPS RTK geometry and ground truth

GPS RTK replaces manual tape measurements for the field deployment:

1. Accept only valid RTK-fixed samples (`fix_quality == 4`) from every anchor.
2. Use robust median latitude, longitude, and altitude per anchor.
3. Convert WGS84 coordinates through ECEF into one local ENU frame, using
   module 2 as the origin. The existing conversion in
   `tools/uwb_gps_rtk_report.py` is the reference implementation.
4. Store the resulting ENU anchor coordinates as fixed FlexTDOA geometry for
   the complete run. Do not continuously move anchors with new GPS samples.
5. Preserve ENU `z` and report vertical spread. The first solver is 2D and uses
   ENU `x/y`; a significant height spread must be flagged rather than hidden.
6. Convert the tag's RTK-fixed samples into the same ENU frame only for
   evaluation. Tag GPS coordinates must never initialize, constrain, or update
   the UWB reference solver.

Comparison is time-aligned and reports horizontal error, UWB residual RMS,
observation count, GPS fix quality, and GPS correction age. Antenna lever-arm
offsets between the GNSS antenna and the UWB antenna must be measured or at
least documented before interpreting centimeter-level bias.

## Migration gates

The old runtime remains selectable until the following gates pass:

1. Pure unit tests pass for timing, CI-CR schedule, packet round-trip, 40-bit
   timestamp wrap, and Eq. (8)/(12).
2. ESP-IDF firmware builds with the new pure component linked.
3. Recorded or synthetic slot replay receives all `K=3` responses without
   cross-slot mixing.
4. One tag and four anchors run the 20.20 ms reference frame with complete-slot
   telemetry.
5. UWB coordinates are compared with GPS RTK in the same ENU frame.

Only after gate 3 is the legacy FlexTDOA runtime removed from the production
path. Its Git history remains the rollback mechanism.
