"""Hash the immutable raw captures referenced by the 2026-08-11 report."""

from __future__ import annotations

import csv
import hashlib
import pathlib


REPORT_DIR = pathlib.Path(__file__).resolve().parent
ROOT = REPORT_DIR.parents[1]
CAPTURES = [
    ("flextdoa_dynamic_final_01", "flextdoa", "dynamic", True,
     ROOT / "reports/imu_fusion_final_validation_20260811/flextdoa_dynamic_final_01.flextdoa.jsonl"),
    ("native_ds_dynamic_quality_gate_02", "native_ds", "dynamic", True,
     ROOT / "reports/imu_fusion_final_validation_20260811/native_ds_dynamic_quality_gate_02.native_ds.jsonl"),
    ("passive_ds_dynamic_final_01", "passive_ds", "dynamic", False,
     ROOT / "reports/imu_fusion_final_validation_20260811/passive_ds_dynamic_final_01.passive_ds.jsonl"),
    ("flextdoa_static_final", "flextdoa", "static", True,
     ROOT / "reports/imu_fusion_static_20260811/flextdoa_static_final.flextdoa.jsonl"),
    ("native_ds_static_final", "native_ds", "static", True,
     ROOT / "reports/imu_fusion_static_20260811/native_ds_static_final.native_ds.jsonl"),
    ("passive_ds_static_final", "passive_ds", "static", True,
     ROOT / "reports/imu_fusion_static_20260811/passive_ds_static_final.passive_ds.jsonl"),
    ("passive_ds_static_raw_regression_01", "passive_ds", "static_control", True,
     ROOT / "reports/imu_fusion_final_validation_20260811/passive_ds_static_raw_regression_01.passive_ds.jsonl"),
]


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    rows = []
    sums = []
    for capture_id, protocol, motion, complete, path in CAPTURES:
        digest = sha256(path)
        relative = path.relative_to(ROOT).as_posix()
        rows.append(
            {
                "capture_id": capture_id,
                "protocol": protocol,
                "motion": motion,
                "container_complete": complete,
                "source_file": relative,
                "source_bytes": path.stat().st_size,
                "source_sha256": digest,
            }
        )
        sums.append(f"{digest}  ../../{relative}")

    with (REPORT_DIR / "raw_data_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (REPORT_DIR / "SHA256SUMS").write_text("\n".join(sums) + "\n", encoding="ascii")


if __name__ == "__main__":
    main()
