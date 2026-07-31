# Native DS-TWR regression check: 04a2183 vs 54a586c

Both captures were recorded for 180 seconds without position or range
filtering, with the same static modules, antenna delays, cached anchor
geometry, and tag reference. The 54a586c run used the strict three-message
`POLL -> RESP -> FINAL` exchange with the complete legacy timing:

- slot: 100 ms
- frame gap: 10 ms
- RX timeout: 90 ms
- RESP delay: 20 ms
- FINAL delay: 20 ms
- auto-RX delay: 500 UUS

## Position precision

| Reconstruction | Metric | 04a2183 | 54a586c | Change |
|---|---|---:|---:|---:|
| Coherent 4/4 frames | updates/s | 0.96 | 1.72 | +79.7% |
| Coherent 4/4 frames | sigma X / Y [cm] | 0.92 / 0.88 | 1.05 / 0.98 | +14.3% / +11.3% |
| Coherent 4/4 frames | CEP50 [cm] | 0.98 | 1.14 | +16.2% |
| Coherent 4/4 frames | CEP95 [cm] | 2.13 | 2.50 | +17.4% |
| Coherent 4/4 frames | maximum [cm] | 4.40 | 3.56 | -19.3% |
| Rolling latest ranges | updates/s | 8.26 | 9.15 | +10.8% |
| Rolling latest ranges | sigma X / Y [cm] | 1.16 / 1.13 | 1.08 / 0.97 | -7.0% / -13.8% |
| Rolling latest ranges | CEP50 [cm] | 1.25 | 1.20 | -3.9% |
| Rolling latest ranges | CEP95 [cm] | 3.06 | 2.50 | -18.5% |
| Rolling latest ranges | maximum [cm] | 6.50 | 4.07 | -37.4% |

## Raw range stability

| Anchor | 04a2183 sigma [cm] | 54a586c sigma [cm] |
|---|---:|---:|
| A2 | 1.28 | 1.50 |
| A3 | 1.12 | 1.09 |
| A4 | 0.99 | 1.23 |
| A5 | 1.75 | 1.82 |

Sequence delivery improved from 84.74% to 94.12%.

## Conclusion

The Native DS-TWR distance formula in 54a586c is not the regression. With
equivalent radio timing, its low-latency rolling position is more stable than
the functional 04a2183 reference while updating 10.8% faster. The apparent
regression came from comparing 04a2183's slow `100/10/90/20/20 ms` runtime
against the card labelled `64 ms Reference`, which actually applied the much
faster `15/4/8/2/2 ms` speed-study timing.

The dashboard now exposes the recovered settings as
`410 ms Legacy Precision Reference` and explicitly names the other card
`64 ms Speed-study Reference`.
