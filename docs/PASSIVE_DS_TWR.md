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
received response.

## Anchor ranging

The initiator and each responder use their six POLL/RESPONSE/FINAL timestamps
to calculate a genuine full DS-TWR distance. These ranges are exported as
diagnostics and can be compared with the fixed RTK baselines. They are not
fed back into tag positioning and are not piggybacked into later stars.

## Passive observation

A tag never transmits. For an initiator `I`, responder `R` and tag `T`, it
timestamps only POLL and RESPONSE and emits:

```text
d(T,R) - d(T,I)
  = c * [tag(RESPONSE_RX - POLL_RX)
         - responder_reply_dtu * (1 - CFO)]
    - d_RTK(I,R)
```

`d_RTK(I,R)` comes from the same fixed GPS RTK ENU anchor geometry used by
FlexTDOA. CFO is read for every response and corrects the responder interval
into the tag clock domain. An observation is rejected when its packet,
session/frame identity, responder subslot, CFO or interval is invalid.

The tag does not wait for FINAL. FINAL exists only to close full DS-TWR at the
anchors, so a missed FINAL cannot invalidate an otherwise complete tag star.

## Position solve

A complete star contains exactly one observation for each of the `N - 1`
responders. The ESP32 tag solves that coherent set once with raw AlgMin and
publishes the result directly.

There are deliberately no rolling windows, EKF updates, prediction, temporal
averaging or mixed-age observations. A partial star expires and produces no
position. The previous solution may be used only as a numerical initial seed;
it is never blended into the new result.

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
slack inside the exchange budget. Including the gap, the nominal upper bound
is 111.11 complete stars and raw positions per second. The profile is intended
as a safe field baseline; it should be compacted only after packet loss and
RTK RMSE are measured together.

## Calibration and telemetry

Existing signed millimetre Passive DS calibration records remain compatible.
The per-anchor observation correction is applied to the directed TDOA value;
the per-pair range correction applies only to reported anchor DS-TWR
diagnostics.

Observation telemetry identifies fixed geometry, frame/session identity, CFO,
reply interval and applied correction. Position telemetry reports one
independent raw frame, its residuals and solver status. The dashboard labels
the anchor DS-TWR ranges as diagnostics and never presents them as live
geometry.

## Verification order

1. Host tests verify packet round trips, CRC rejection, 40-bit wrap handling,
   CFO correction and full DS-TWR arithmetic.
2. Firmware must compile with the production ESP-IDF toolchain.
3. OTA starts with all modules on the conservative profile.
4. A field capture records complete/incomplete stars, rejection reasons,
   anchor-range error and raw position RMSE/P95 against time-aligned RTK.
5. Only then may the exchange budget, response spacing or gap be reduced.
