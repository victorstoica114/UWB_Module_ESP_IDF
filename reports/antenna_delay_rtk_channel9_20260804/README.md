# FlexTDOA raw channel-9 timing and RTK calibration study

Date: 2026-08-04

All localization metrics in this report are unfiltered. GPS RTK is used only as
independent ground truth and as the source of the fixed anchor geometry; it is
never fed into the FlexTDOA solver.

## Fixed RTK ENU geometry

M2 is the ENU origin. Coordinates are in metres.

| Module | Role | East | North | Up |
|---|---|---:|---:|---:|
| M1 | tag / ground truth | +0.332 | +3.392 | +0.018 |
| M2 | anchor | 0.000 | 0.000 | 0.000 |
| M3 | anchor | +3.844 | +2.923 | -0.009 |
| M4 | anchor | -3.120 | +3.899 | -0.080 |
| M5 | anchor | +1.055 | +6.836 | -0.131 |

All captures used DW3000 channel 9 and firmware `048359a`.

## Raw FlexTDOA timing A/B

Profiles B and C have the same 23.2 ms frame duration. They differ only in
whether the added 250 us is placed between responses or in the end-of-slot
processing tail.

| Profile | response subslot | response process | positions/s | complete slots/s | incomplete slots/s | rejected responses/s | RTK RMSE |
|---|---:|---:|---:|---:|---:|---:|---:|
| A paper reference | 250 us | 600 us | 56.9 | 56.8 | 70.2 | 140.8 | 13.7 cm |
| B spaced responses | 500 us | 600 us | 150.1 | 149.8 | 9.5 | 38.3 | 14.4 cm |
| C longer tail control | 250 us | 850 us | 62.3 | 61.9 | 54.7 | 109.6 | 13.7 cm |

The control establishes that response spacing, not total frame length, is the
dominant availability limitation. The observed double-buffer read/release/rearm
path peaks at 240--280 us, which exceeds the paper's 250 us response spacing.

## Channel-9 antenna-delay dry run

The active values were not changed:

| Module | Active delay |
|---|---:|
| M1 | `0x3ff3` |
| M2 | `0x3ff9` |
| M3 | `0x3ff2` |
| M4 | `0x3ffa` |
| M5 | `0x4004` |

A 120 s Native DS-TWR calibration-only capture supplied all ten pair links.
The complete-graph per-module antenna-delay model reduced the predicted link
RMS from 7.99 cm to only 5.99 cm and left an 11.77 cm maximum residual. This is
not a sufficiently explanatory fit to justify writing new antenna delays.

The fitted dry-run values were M1 `+0`, M2 `+5`, M3 `+10`, M4 `-17`, and M5
`-3` DTU. They were deliberately not applied. Additional evidence from the 12
directed FlexTDOA observations shows strong direction/order-dependent residuals,
which a single constant antenna delay per module cannot remove.

## Data sets

- `flex_before/data`: paper-reference baseline, 60 s.
- `flex_resp500/data`: 500 us response-spacing candidate, 60 s.
- `flex_control250/data`: same-frame-duration control, 60 s.
- `native_ds_dryrun/data`: calibration-only full-graph capture, 120 s.
