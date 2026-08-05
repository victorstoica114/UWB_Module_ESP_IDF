# Passive DS-TWR v2

Passive DS-TWR combines full double-sided ranging between anchors with
receive-only tags. It is a clean protocol, independent of Native DS-TWR and
FlexTDOA runtime state.

## Radio star

For `N` configured anchors, one star contains `N + 1` broadcasts:

1. The rotating initiator broadcasts `POLL`.
2. The other `N - 1` anchors broadcast delayed `RESPONSE` packets in fixed,
   collision-free subslots.
3. The initiator broadcasts one aggregate `FINAL` after receiving the response
   train.

The initiator is `anchor_ids[frame_id % anchor_count]`. All packets carry the
`PDS2` magic, protocol version, session ID, frame ID, initiator ID and CRC-16.
The session ID separates scheduler epochs after a reboot.

Each response contains its responder index and exact programmed reply interval
in DW3000 time units. The final contains the initiator's 40-bit POLL and FINAL
TX timestamps plus one responder ID and 40-bit response RX timestamp for every
received response. After receiving FINAL, every responder retains its complete
`POLL_RX..FINAL_RX` interval. Its next POLL or RESPONSE carries the two most
recent completed-exchange references. This delayed transport adds no radio
packet and gives every reference a natural retransmission opportunity.

## Anchor ranging

The initiator and each responder use their six POLL/RESPONSE/FINAL timestamps
to calculate a genuine full DS-TWR distance. These ranges are exported as
diagnostics and can be compared with the fixed RTK baselines. They are not
fed back into tag positioning. Only the responder's complete exchange interval
is piggybacked for the passive timing equation; measured anchor distance is
never used as tag geometry.

## Passive observation

A tag never transmits. For an initiator `I`, responder `R` and tag `T`, define:

```text
M_T = T(RESPONSE_RX - POLL_RX)
R_I = I(RESPONSE_RX - POLL_TX)
D_R = R(RESPONSE_TX - POLL_RX)
E_T = T(FINAL_RX - POLL_RX)
E_I = I(FINAL_TX - POLL_TX)
E_R = R(FINAL_RX - POLL_RX)
```

After the delayed `E_R` reference arrives, the tag emits the genuine
three-packet passive DS observation (Rathje/Landsiedel Eq. 19, with the sign
used by our solver):

```text
d(T,R) - d(T,I)
  = c * [M_T - 0.5 * (E_T/E_I) * R_I
             - 0.5 * (E_T/E_R) * D_R]
```

The full intervals estimate the two clock ratios in the tag clock domain.
Neither a measured anchor range nor the DW3000 carrier-integrator CFO enters
the production observation. CFO is retained only as an A/B diagnostic.
An observation is rejected when its packet, session/frame identity, responder
subslot, FINAL timestamps or delayed exchange reference is invalid.

FINAL is mandatory for both the anchor DS range and the passive tag equation.
A complete position is consequently published after the responder intervals
are carried by following stars, not immediately after the current response
train.

## Position solve

A complete star contains exactly one observation for each of the `N - 1`
responders. The ESP32 tag exposes two deliberately raw solve policies:

- mode `0` solves one coherent star and is retained as a radio and equation
  diagnostic;
- mode `2` combines three recent complete coherent stars with distinct
  initiators in one spatial GLS solve. Independent RTK positions never reuse
  a star and are limited to a three-frame span. Additional display positions
  may reuse stars but are limited to a six-frame span and at most one result
  for each newest complete star.

The three-star policy reduces the rotating single-star geometry anisotropy and
avoids multiplying isolated radio losses into three consecutive-window losses.
It does not average previous positions: all nine observations in a four-anchor
solve are raw radio measurements from the bounded three-star window. There is
no EKF, prediction or temporal low-pass filter. A partial radio star expires
and produces no position. The previous solution may be used only as a
numerical initial seed; it is never blended into the new result.

Before publication, equation residuals are recomputed without temporal state.
A batch is rejected when raw equation RMS exceeds `0.25 m` or the largest
absolute residual exceeds `0.50 m`. The same observations are retried once
without the previous numerical seed before rejection. This is a coherence
check on the current UWB measurements, not a motion or jump filter.

## Field baseline

The initial conservative four-anchor profile is:

```text
exchange budget       8.000 ms
inter-star gap         1.000 ms
first RESPONSE         1.500 ms after POLL
RESPONSE spacing       0.750 ms
FINAL                  1.500 ms after last RESPONSE
RX timeout             5 ms
```

The response train plus FINAL consumes 4.500 ms, leaving 3.500 ms scheduler
slack inside the exchange budget. Including the gap, the nominal radio upper
bound is 111.11 complete stars per second. Single-star mode can publish one
position per complete star. Three-star mode can publish one additional raw
display position for each newly completed star once three distinct initiators
are available; independent positions consume three complete stars and never
reuse them. The profile is intended as a safe field baseline; it should be
compacted only after packet loss and RTK RMSE are measured together.

## Calibration and telemetry

Existing signed millimetre Passive DS calibration records remain compatible.
The per-anchor observation correction is applied to the directed TDOA value;
the per-pair range correction applies only to reported anchor DS-TWR
diagnostics.

Observation telemetry identifies fixed geometry, frame/session identity,
three-packet DS result, interpolation ratio and applied calibration. The old
CFO result is exposed only as a same-frame comparison. Position telemetry
labels independent and overlapping raw windows separately and reports their
residuals and solver status. RTK error statistics use independent positions.
The dashboard labels anchor DS-TWR ranges as diagnostics and never presents
them as live geometry.

## Verification order

1. Host tests verify packet round trips, CRC rejection, real 40-bit wrap on all
   three clocks, independent clock drift, the three-packet passive equation,
   diagnostic CFO comparison and full anchor DS-TWR arithmetic.
2. Firmware must compile with the production ESP-IDF toolchain.
3. OTA starts with all modules on the conservative profile.
4. A field capture records complete/incomplete stars, rejection reasons,
   anchor-range error and raw position RMSE/P95 against time-aligned RTK.
5. Only then may the exchange budget, response spacing or gap be reduced.
