# Passive DS-TWR

Passive DS-TWR is a project-specific three-frame anchor protocol that preserves
receive-only tags. It is intentionally separate from both native DS-TWR and
FlexTDOA.

## Goals

- Exactly three radio frames per anchor pair: `POLL`, `RESP`, `FINAL`.
- Anchor-to-anchor DS-TWR remains available for path and geometry diagnostics.
- Every module outside the configured anchor set is a passive tag.
- Adding a tag consumes no UWB airtime and requires no scheduler slot.
- Native DS-TWR and FlexTDOA settings remain unchanged.

## Wire exchange

1. The initiator sends `PASSIVE_DS_POLL` with a 32-bit slot ID.
2. The responder schedules `PASSIVE_DS_RESP` and includes both the slot ID and
   its exact 32-bit programmed reply interval in DW3000 time units.
3. The initiator sends `PASSIVE_DS_FINAL` with its native DS-TWR timestamps and
   the slot ID. The responder uses the six timestamps to calculate the
   anchor-to-anchor range.

A passive tag timestamps the received POLL and RESP. It does not transmit and
does not need to receive FINAL to form its observation.

For initiator `I`, responder `R`, and tag `T`, the emitted range difference is:

```text
d(T,R) - d(T,I)
  = c * (RESP_RX_T - POLL_RX_T - corrected_reply_R)
    - d(I,R)
```

The fixed anchor geometry supplies `d(I,R)`. The responder-to-tag carrier
frequency offset corrects `reply_R` into tag clock units.

## Clock requirements

Absolute clock phase synchronization is not required. Adding an arbitrary
offset to the tag clock cancels when `RESP_RX_T - POLL_RX_T` is formed.

Clock frequency error does not cancel. The reply interval is measured by the
responder but subtracted from an interval measured by the tag, so the tag must
apply the DW3000 carrier-frequency-offset correction. Observations without a
valid CFO estimate are rejected.

## Schedules

### Fast Star

The first configured anchor is always the initiator. A frame contains one
exchange from it to every other anchor. For four anchors this is three slots
and three independent TDOA observations. It has the smallest coordination
overhead, but the fixed reference makes its error and NLOS sensitivity more
dependent on that anchor.

### Robust Rotating

Each frame is still a three-slot star for four anchors, but the initiator
rotates after the frame. Four frames form a superframe in which every anchor
has served as the reference. This adds directed-path diversity without adding
tag transmissions or protocol control frames.

All anchors derive the next slot from the received slot ID and POLL arrival
time. The first anchor bootstraps slot zero after a silent startup interval.

## Geometry and current validation status

Passive positioning requires fixed anchor coordinates. The implementation
shares the existing fixed FlexTDOA geometry record so the same surveyed
coordinate frame is used by both passive protocols.

Fast Star alone measures only ranges from the fixed reference to the other
anchors, so it cannot fully self-localize an unconstrained 2D anchor geometry.
Robust Rotating eventually measures every anchor pair and can provide the
complete range set needed by the existing geometry workflow.

The firmware and dashboard are build-validated. Radio timing, packet loss,
position rate, accuracy, and the practical minimum slot size still require
field validation with a known anchor geometry.
