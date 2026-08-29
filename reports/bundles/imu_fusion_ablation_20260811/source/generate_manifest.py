"""Verify the archived captures and regenerate their integrity manifest."""

from __future__ import annotations

import csv
import hashlib
import lzma
import pathlib


SOURCE_DIR = pathlib.Path(__file__).resolve().parent
BUNDLE_DIR = SOURCE_DIR.parent
ANALYSIS_DIR = BUNDLE_DIR / "analysis"
RAW_DIR = BUNDLE_DIR / "raw" / "data"
CAPTURES = [
    ("flextdoa_dynamic_final_01", "flextdoa", "dynamic", True,
     "reports/imu_fusion_final_validation_20260811/flextdoa_dynamic_final_01.flextdoa.jsonl",
     825791787, "decd89b3fcb28e2d9afad151514b7e8b4496ac01878faed30d0bfd6e9e751bc0",
     "flextdoa_dynamic_final_01.flextdoa.jsonl.xz"),
    ("native_ds_dynamic_quality_gate_02", "native_ds", "dynamic", True,
     "reports/imu_fusion_final_validation_20260811/native_ds_dynamic_quality_gate_02.native_ds.jsonl",
     925052704, "0b339899ba133a68a16b46dff9be3d396fe87c3b7e5a49b7c6ac052423692bab",
     "native_ds_dynamic_quality_gate_02.native_ds.jsonl.xz"),
    ("passive_ds_dynamic_final_01", "passive_ds", "dynamic", False,
     "reports/imu_fusion_final_validation_20260811/passive_ds_dynamic_final_01.passive_ds.jsonl",
     683623213, "b06485af937c5aff3c7e5af41bc77c13357bca43367d500630c8f21a282b102a",
     "passive_ds_dynamic_final_01.passive_ds.jsonl.xz"),
    ("flextdoa_static_final", "flextdoa", "static", True,
     "reports/imu_fusion_static_20260811/flextdoa_static_final.flextdoa.jsonl",
     113112326, "7e814842dffac507f9777541fd547ab440ad899efbe820cdeed1ceca9ee914c9",
     "flextdoa_static_final.flextdoa.jsonl.xz"),
    ("native_ds_static_final", "native_ds", "static", True,
     "reports/imu_fusion_static_20260811/native_ds_static_final.native_ds.jsonl",
     112312663, "968d87a3cae8c59ba20977b49928fd2613db4d1e0f083e1544666dae50f92871",
     "native_ds_static_final.native_ds.jsonl.xz"),
    ("passive_ds_static_final", "passive_ds", "static", True,
     "reports/imu_fusion_static_20260811/passive_ds_static_final.passive_ds.jsonl",
     122477187, "709db38970fddd078cc0980e8cb2300378431d82e443de938c766655a52c7f2a",
     "passive_ds_static_final.passive_ds.jsonl.xz"),
    ("passive_ds_static_raw_regression_01", "passive_ds", "static_control", True,
     "reports/imu_fusion_final_validation_20260811/passive_ds_static_raw_regression_01.passive_ds.jsonl",
     118450496, "d06800f718ba100e268f63293e71b79bdd5541c6efaf06a266da1a1a95154c9d",
     "passive_ds_static_raw_regression_01.passive_ds.jsonl.xz"),
]


def sha256_stream(handle) -> tuple[int, str]:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
        size += len(block)
        digest.update(block)
    return size, digest.hexdigest()


def sha256(path: pathlib.Path) -> tuple[int, str]:
    with path.open("rb") as handle:
        return sha256_stream(handle)


def main() -> None:
    rows = []
    sums = []
    for (capture_id, protocol, motion, complete, source_file,
         expected_bytes, expected_digest, archive_name) in CAPTURES:
        archive = RAW_DIR / archive_name
        archive_bytes, archive_digest = sha256(archive)
        with lzma.open(archive, "rb") as handle:
            source_bytes, source_digest = sha256_stream(handle)
        if (source_bytes, source_digest) != (expected_bytes, expected_digest):
            raise ValueError(f"RAW round-trip verification failed: {archive}")
        rows.append(
            {
                "capture_id": capture_id,
                "protocol": protocol,
                "motion": motion,
                "container_complete": complete,
                "source_file": source_file,
                "source_bytes": source_bytes,
                "source_sha256": source_digest,
                "archive_file": f"../raw/data/{archive_name}",
                "archive_bytes": archive_bytes,
                "archive_sha256": archive_digest,
                "compression": "XZ/LZMA2 preset 3 (lossless)",
                "roundtrip_sha256_matches": True,
            }
        )
        sums.append(
            f"{archive_digest}  ../raw/data/{archive_name}"
        )

    with (ANALYSIS_DIR / "raw_data_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (ANALYSIS_DIR / "SHA256SUMS").write_text("\n".join(sums) + "\n", encoding="ascii")


if __name__ == "__main__":
    main()
