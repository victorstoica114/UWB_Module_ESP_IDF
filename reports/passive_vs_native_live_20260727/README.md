# Native vs Passive DS-TWR live audit

## Scope

This is a controlled A/B/A capture made on 2026-07-27 with the four anchors
in the surveyed 3 m square and Tag 1 at its center:

- 60 s Native DS-TWR;
- 90 s calibrated Passive DS-TWR, Robust Rotating;
- 60 s Native DS-TWR.

All five modules were hot-switched together. No reboot or OTA was used, and
the system was returned to Native DS-TWR after the capture. The nominal anchor
coordinates are A2=(0,0), A3=(0,3), A4=(3,0), A5=(3,3) m and the tag reference
is (1.5,1.5) m.

The comparison separates:

- **accuracy**: error from the surveyed center;
- **precision**: spread around each capture's own mean;
- protocol observations from anchor-geometry and solver effects.

## Main result

The screenshots correctly show that the embedded Passive position cloud is
wider. They do not show that its absolute accuracy is worse.

| Capture / solver | Rate | Bias norm | 2-D RMSE | P95 error | CEP95 precision | 2DRMS precision |
|---|---:|---:|---:|---:|---:|---:|
| Native before, surveyed geometry, robust 3-of-4 | 47.85 Hz | 4.05 cm | 4.41 cm | 6.41 cm | 3.34 cm | 3.49 cm |
| Native after, surveyed geometry, robust 3-of-4 | 47.62 Hz | 3.55 cm | 3.89 cm | 5.49 cm | 2.86 cm | 3.22 cm |
| Passive, embedded autonomous geometry | 26.88 Hz | 1.26 cm | 3.54 cm | 6.20 cm | 5.86 cm | 6.62 cm |
| Passive, surveyed geometry and rolling 12-path AlgMin | 49.20 Hz | 2.18 cm | 2.98 cm | 4.62 cm | 3.12 cm | 4.05 cm |
| Passive, same solver plus per-path CFO median | 49.20 Hz | 2.19 cm | 2.93 cm | 4.54 cm | 2.95 cm | 3.89 cm |

Therefore:

1. The current embedded Passive result is more accurate on average than both
   Native blocks, but its unfiltered scatter is about twice as large.
2. Recomputing Passive with the surveyed geometry and the same rolling
   observation set removes most of the apparent gap. Its P95 and RMSE become
   better than Native, while its precision remains approximately 12--21%
   worse depending on which Native block is used.
3. The largest avoidable loss is in the current embedded geometry/solver path,
   not in the passive packet equation.

See [position scatter](figures/position_scatter.png) and
[position-error CDF](figures/position_error_cdf.png).

## Formula and CFO audit

The implemented observation has the correct orientation:

```text
d(T,R) - d(T,I)
  = c * ((RESP_RX_tag - POLL_RX_tag) - corrected_reply_interval)
    - d(I,R)
```

Absolute tag clock phase cancels. Relative clock frequency does not cancel and
must correct the responder's programmed reply interval into tag clock units.
The captured data confirms both the sign and the necessity of the correction:

- mean absolute directed-path bias before CFO correction: **87.05 cm**;
- after CFO correction: **6.75 cm**;
- after the current hardware calibration: **5.23 cm**.

There is no evidence of a missing factor or reversed CFO sign. A 31-sample
per-directed-path CFO median only reduces within-path standard deviation from
5.45 cm to 5.22 cm and position 2DRMS from 4.05 cm to 3.89 cm. Per-packet CFO
noise is real, but it is not the dominant remaining error.

The 1000 us reply delay remains justified. A prior 750 us hardware trial
reached only about 72% successful protocol operations. Shorter delays would
reduce CFO sensitivity, but should be evaluated separately at 850--950 us
rather than assumed safe.

## Differences from the paper that matter

The FlexTDOA paper explicitly states that:

- TDOA is more sensitive to noise than range/TOA localization;
- CFO correction is crucial and remains imperfect;
- live tracking benefits from an EKF/static or motion model;
- anchor self-localization is run with TWR for several minutes, then the
  resulting anchor coordinates are kept fixed for the experiment.

The current embedded Passive implementation instead:

- continuously updates anchor geometry from the newest raw pair ranges;
- has no robust gate or confirmation state in the embedded geometry solver;
- uses unweighted raw AlgMin observations from a 500 ms rolling window;
- has no Huber/MAD observation rejection and no temporal position filter.

The browser-side Native path already uses a robust 3-of-4 fit and the new
robust geometry gate. The dashboard currently prefers the embedded ESP32
position for Passive, so the screenshot compares different solver and
robustness paths in addition to different protocols.

## Outliers

The radio links occasionally produce isolated corrupt data in both modes:

- Native: 21/14,290 and 28/14,228 ranges outside 0--5 m;
- Passive: 6/12,625 directed observations detected as path-local robust
  spikes (0.0475%);
- Passive anchor geometry: 5 pair ranges more than 12 cm from surveyed
  geometry;
- Passive embedded position: 12/2,421 results had internal RMS above 15 cm.

Native's browser solver rejects or survives its bad samples. The embedded
Passive solver currently consumes them. Merely rejecting the 12 Passive
positions above 15 cm internal RMS reduces RMSE from 3.54 cm to 3.43 cm and
maximum error from 17.15 cm to 12.81 cm.

## Calibration opportunity

The existing calibration still helps, but its per-anchor component was fitted
before the latest session and no longer removes the current antisymmetric
hardware bias.

An honest train/validation check was performed: the first third of the Passive
capture fitted an incremental compact model, and the remaining two thirds were
evaluated without refitting.

| Passive validation variant | Bias norm | 2-D RMSE | P95 | 2DRMS precision |
|---|---:|---:|---:|---:|
| Current calibration | 2.24 cm | 2.95 cm | 4.61 cm | 3.85 cm |
| Incremental per-anchor correction | 0.29 cm | 1.94 cm | 3.12 cm | 3.84 cm |
| Joint per-anchor + symmetric-pair correction | 0.24 cm | 1.93 cm | 3.12 cm | 3.83 cm |

The fitted incremental per-anchor values, relative to the currently programmed
calibration, are A2=0, A3=-37, A4=-41, A5=-40 mm. They demonstrate that the
bias is calibratable; they must not be deployed as final constants from this
single center point. A final calibration needs several known tag positions so
that tag-position error cannot be absorbed into hardware bias.

The symmetric pair terms greatly reduce individual directed-path means but add
almost no center-position benefit beyond the compact per-anchor correction.
This is useful evidence for retaining a scalable compact model unless
multi-position validation proves a repeatable pair-specific residual.

## Recommended implementation order

1. Make the embedded anchor geometry robust: gate isolated pair spikes, require
   multi-edge relocation confirmation, limit coordinate step size, and hold a
   converged geometry unless movement is confirmed.
2. Add path-local robust observation rejection and reject/hold positions with
   implausible internal RMS.
3. Replace raw embedded AlgMin output with an EKF or equivalent position
   tracker; expose raw and filtered positions separately so dynamic tests stay
   interpretable.
4. Low-pass CFO per directed hardware pair with a slow tracker, not a fixed
   value.
5. Refit the compact per-anchor calibration over multiple surveyed positions,
   then validate on held-out positions and later on motion trajectories.
6. Only after these changes, test 950, 900 and 850 us reply delays for the
   accuracy/reliability frontier.

## Reproduction

Run:

```bash
python3 tools/uwb_passive_native_live_analyze.py
```

Machine-readable results are in
[analysis_summary.json](analysis_summary.json),
[position_metrics.csv](position_metrics.csv), and
[directed_path_metrics.csv](directed_path_metrics.csv). Raw JSONL captures and
their runtime metadata are under `data/`.
