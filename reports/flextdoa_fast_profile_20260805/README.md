# FlexTDOA field timing sweep — 2026-08-05

## Decision

Use `250/2000/250/850/1000 us` for guard, request, request processing,
response subslot, and per-response processing. With four initiator slots and
three responders this is an `8.05 ms` slot, a `32.20 ms` frame, and a
theoretical maximum of `31.06` complete positions/s.

The final clean 120 s channel-9 capture produced:

- `3618` independent unfiltered positions in `120.516 s` (`30.02/s`)
- `14473` complete slots, `316` incomplete slots, and `120` request gaps
- `2.924%` effective slot loss: `(incomplete + request gaps) / all slots`
- zero new DW3000 delayed-TX errors on all five modules
- `2.03 cm` position RMSE, `1.90 cm` debiased RMSE, and `3.58 cm` P95
  against the static RTK survey reference
- zero wireless log or telemetry drops

No position filter was enabled.

## Sweep

| RESP / process | Frame | Positions/s | Effective loss | New TX errors | RTK RMSE | P95 | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `1000/1000 us` | `34.0 ms` | `28.61` | `3.003%` | `0` | `1.95 cm` | `3.36 cm` | Stable control |
| `500/600 us` | `23.2 ms` | `40.98` | `4.412%` | `2` | `2.15 cm` | `3.78 cm` | Rejected |
| `650/800 us` | `27.4 ms` | `34.93` | `4.430%` | `6` | `2.11 cm` | `3.69 cm` | Rejected |
| `750/1000 us` | `31.0 ms` | `31.07` | `3.113%` | `1` | `2.27 cm` | `3.93 cm` | Rejected |
| `800/1000 us` | `31.6 ms` | `30.75` | `2.494%` | `4` | `2.24 cm` | `4.03 cm` | Rejected: insufficient TX margin |
| `850/1000 us` | `32.2 ms` | `30.02` | `2.924%` | `0` | `2.03 cm` | `3.58 cm` | Selected |

The selected profile gains about `4.9%` complete position throughput over the
stable control. The slightly faster `800/1000 us` profile had lower aggregate
loss but produced four delayed-TX errors in 120 s and a wider position tail, so
it was not adopted.

A follow-up read after another `1701 s` of uninterrupted operation found `25`
new delayed-TX errors across `843920` anchor transmissions. That is a
`0.00296%` TX-error rate, about three orders of magnitude below the aggregate
slot-loss rate. It is recorded as real long-run behavior but is too small to
justify relaxing the selected profile to `900/1000 us`.

## Reference handling

All position metrics use the known static survey truth `(0.332, 3.392) m`.
After the simultaneous reboot used to activate the first candidate, the live
M1 GNSS solution shifted about `0.69 m` east while remaining marked RTK fixed;
the UWB position and surveyed geometry did not move. Time-matched post-reboot
GNSS was therefore not used as truth for this timing comparison.

## Capture integrity

The authoritative capture is:

`build-flex-fast/final_resp850_proc1000_clean/fast_final_resp850_proc1000_clean.jsonl`

Its diagnostic log is stored separately as
`fast_final_resp850_proc1000_clean.timing.jsonl`. The collector now rejects a
`--log-output` path equal to its automatically generated event path, preventing
two writers from corrupting a JSONL capture.

`FLEXTDOA_PAPER_TIMING` remains unchanged at the exact paper values
`250/2000/250/250/600 us`; only the deployed firmware default changes to the
field-validated profile.
