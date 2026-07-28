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

A POLL or RESP also carries one previously measured DS-TWR anchor range.
Measurements are rotated through the available cache, following the
piggyback principle from the FlexTDOA paper. This adds no radio frame and lets
receive-only tags self-localize an unknown anchor geometry.

A passive tag timestamps the received POLL and RESP. It does not transmit and
does not need to receive FINAL to form its observation.

For initiator `I`, responder `R`, and tag `T`, the emitted range difference is:

```text
d(T,R) - d(T,I)
  = c * (RESP_RX_T - POLL_RX_T - corrected_reply_R)
    - d(I,R)
```

The freshest piggybacked DS-TWR range supplies `d(I,R)`. Fixed anchor geometry
is used only as a bootstrap fallback. The responder-to-tag carrier frequency
offset corrects `reply_R` into tag clock units.

## Clock requirements

Absolute clock phase synchronization is not required. Adding an arbitrary
offset to the tag clock cancels when `RESP_RX_T - POLL_RX_T` is formed.

Clock frequency error does not cancel. The reply interval is measured by the
responder but subtracted from an interval measured by the tag, so the tag must
apply the DW3000 carrier-frequency-offset correction. Observations without a
valid CFO estimate are rejected.

RESP and FINAL delays are configured in microseconds. Shorter responder delays
reduce the distance-equivalent CFO correction and its residual noise. Hardware
validation selected 1000 us as the default: 750 us was measurably less reliable
on the current five-module setup.

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

## Experimental low-latency modes

The validated control remains Robust Rotating with 1000 us RESP and FINAL
delays. Two independent runtime switches allow the scheduling and display
experiments to be evaluated without changing that radio profile:

- `passive_ds_pipeline_mode=0` keeps the original blocking control;
  `passive_ds_pipeline_mode=1` uses a non-blocking exchange state machine,
  DW3000 delayed POLL/RESP/FINAL transmission, hardware RX-after-TX, a
  microsecond host alarm for the next owned slot, and explicit response/final
  deadlines.
- `passive_ds_solve_mode=0` emits one independent solution when a complete
  frame closes; `passive_ds_solve_mode=1` additionally solves after each new
  usable observation, capped by `passive_ds_rolling_max_hz`.

Rolling solutions reuse observations inside the freshness window and are
therefore low-latency updates, not additional independent radio frames. The
position telemetry and dashboard report `solver updates/s` separately from
`independent frames/s`.

The dashboard also keeps the two roles visually separate. Every rolling
solution updates the live marker, while the retained EKF and raw-solver trails
contain independent frames only. Their point counts and retained time span are
shown alongside the measured browser `event -> render` latency. Drawing is
downsampled when necessary, without removing points from the retained trail or
its statistics.

The deadline pipeline accumulates stage counters and host execution time for
POLL TX, delayed RESP TX, FINAL TX/RX, CIA readout, and RX re-arm. Counters are
held in RAM and returned by the existing low-rate `/status` request; the radio
path does not produce a log for every packet.

For A/B validation, change only one switch at a time. A candidate is accepted
only when position RMSE and P95 remain inside the confidence band of a new
control capture made in the same surveyed geometry. Code verification alone
does not establish that result. The gate uses only V3 positions marked as
independent frames and a moving-block bootstrap:

```bash
python3 tools/uwb_passive_ab_validate.py \
  --control control.jsonl --candidate candidate.jsonl \
  --truth-x 1.5 --truth-y 1.5 --output ab_gate.json
```

The command exits with status 2 when either candidate metric exceeds the
upper 95% confidence limit of the control.

## Geometry and current validation status

Passive positioning requires anchor coordinates, but they do not have to be
surveyed manually. With fixed geometry cleared, piggybacked DS-TWR ranges feed
the existing local anchor solver. The coordinate frame fixes the first anchor
at the origin and the second anchor on the positive Y axis. A fixed FlexTDOA
geometry record remains available as a surveyed bootstrap and fallback.

Fast Star alone measures only ranges from the fixed reference to the other
anchors, so it cannot fully self-localize an unconstrained 2D anchor geometry.
Robust Rotating measures every directed anchor pair over one superframe and
provides the complete range set needed by the autonomous geometry workflow.

Passive observation telemetry includes the CFO in ppm, the applied
distance-equivalent correction, reply delay, anchor-range source and range age.
Position telemetry already carries the frame RMS and solver sigma.

## Unsurveyed-geometry smoke test

The first hardware test on 2026-07-27 deliberately used the modules in their
existing, unsurveyed positions. Robust Rotating used 5 ms slots, a 1 ms frame
gap, and 1000 us RESP and FINAL delays. No fixed geometry was available.

The receive-only tag reconstructed all six anchor ranges from piggybacked
DS-TWR measurements and produced 49.3 position results/s during a 30 s sample.
The four anchors reported 96.4% to 97.7% successful protocol operations.
The six fitted distances were consistent with a planar geometry to 1.61 cm RMS
(2.01 cm maximum residual).

One equivalent coordinate realization, fixing A2 at the origin and A3 on the
positive Y axis, was:

```text
A2 = (0.000, 0.000) m
A3 = (0.000, 5.619) m
A4 = (4.193, 2.122) m
A5 = (3.704, 4.315) m
```

Across 1492 consecutive estimates, the solver RMS median was 8.1 cm and the
95th percentile was 10.6 cm. Position standard deviation was 2.35 cm on X and
1.59 cm on Y, with no estimate above 25 cm solver RMS in that window. These are
repeatability and internal-consistency results, not absolute-accuracy results:
the anchor and tag coordinates were not surveyed.

An earlier 750 us trial reached only about 72% successful protocol operations
and produced four consecutive bad estimates in a 4000-result window. This is
why 1000 us, rather than 750 us, is the validated default.

## Hardware-bias calibration

Passive DS-TWR keeps two calibration layers separate:

- one observation bias per configured anchor, with the first anchor fixed to
  zero; a directed `I -> R` observation is corrected by subtracting
  `bias(R) - bias(I)`;
- one DS range bias per unordered anchor pair; the piggybacked range is
  corrected before it updates the autonomous geometry or enters the passive
  observation equation.

This is compact and scalable. `N` anchors require `N - 1` independent
observation biases and `N(N - 1)/2` range biases, rather than two values for
every directed path. Values are signed integer millimeters.

The dashboard exposes both lists under **Ranging Settings → Passive DS-TWR
Frame Profiles**. The calibration target is intentionally separate from the
frame-profile target and defaults to module 1, the current passive tag.
Calibration belongs on every passive tag that calculates a position; anchors
do not need it to perform the radio exchange.

The runtime API accepts:

```text
passive_ds_anchor_bias_mm=0,34,-16,-55
passive_ds_range_bias_mm=71,56,-49,67,66,-29
```

Range pairs follow compact upper-triangle order: A1-A2, A1-A3, ..., A2-A3,
... . `passive_ds_calibration_clear=1` disables and clears both lists. A
successful update increments the calibration generation and reboots the
selected tag.

New firmware stores only the configured anchor count and pair count in NVS,
while remaining backward-compatible with the original fixed-size blobs. For
four anchors this reduces the two calibration blobs from 220 bytes to 40
bytes, avoiding unnecessary NVS fragmentation.

## Known-geometry validation

On 2026-07-27, A2-A5 were restored to a surveyed 3.000 m square and tag 1 was
placed at `(1.500, 1.500)` m. Two independent 120 s Robust Rotating captures
were recorded before and after calibration.

The embedded autonomous-geometry result improved from 4.12 cm to 3.33 cm
2-D RMSE, while its bias magnitude fell from 3.51 cm to 0.96 cm. Re-solving
the calibrated observations with the surveyed square produced 1.89 cm RMSE
and 3.11 cm P95 at 49.17 complete frames/s.

A direct before/after measurement separates the calibration layers. Mean
absolute effective-range bias across the six anchor pairs fell from 5.63 cm
to 0.17 cm, so the pair-range correction is effective. Mean absolute
directed-observation bias across the twelve paths increased from 4.65 cm to
5.33 cm, however. Correcting the range also shifts the observation formed with
that range, so the independently fitted per-anchor observation term
double-counts part of the error. The next calibration iteration must fit
observation residuals after range correction, using both an antisymmetric
per-anchor term and, if required, a symmetric per-pair term. Autonomous
geometry filtering and this calibration interaction are the two remaining
accuracy tasks.

All raw captures, metrics, figures, limitations, and reproduction instructions
are in
[`reports/passive_ds_known_geometry_20260727`](../reports/passive_ds_known_geometry_20260727/README.md).
