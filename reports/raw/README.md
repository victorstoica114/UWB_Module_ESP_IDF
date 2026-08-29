# Retained RAW data

Each subdirectory matches one final PDF in `../pdfs/`. Capture paths preserve
their original internal layout. Adjacent `*.metadata.json` files travel with
their captures so analysis inputs remain self-contained.

| Report bundle | Captures | Capture bytes | Metadata |
|---|---:|---:|---:|
| `ds_twr_speed_limit_20260726` | 24 | 3,557,608 | 12 |
| `imu_fusion_ablation_20260811` | 7 | 119,170,664 | 0 |
| `imu_fusion_validation_20260811` | 6 | 25,947,544 | 0 |
| `native_ds_round_boundary_20260731` | 14 | 77,427,409 | 6 |
| `native_ds_speed_revalidation_20260731` | 3 | 17,111,423 | 3 |
| `passive_ds_known_geometry_20260727` | 4 | 2,211,060 | 2 |
| `passive_ds_pipeline_20260728` | 4 | 59,376,048 | 4 |
| `passive_ds_speed_limit_20260727` | 10 | 33,963,912 | 5 |
| `uwb_dynamic_static_comparison_20260806` | 6 | 19,912,116 | 0 |
| `uwb_dynamic_static_comparison_20260812` | 6 | 6,659,032 | 0 |
| `uwb_dynamic_static_comparison_20260813` | 6 | 12,655,316 | 0 |
| `uwb_gps_rtk_comparison_20260803` | 3 | 10,788,160 | 3 |
| `uwb_protocol_comparison_20260726` | 6 | 10,873,184 | 6 |

There are 99 retained captures and 41 adjacent metadata files. `SHA256SUMS`
verifies every file in this archive except this README and the checksum file
itself.
