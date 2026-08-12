#!/usr/bin/env python3

import json
import lzma
import math
import pathlib
import sys
import tempfile
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dynamic_static_report import Capture, load_capture
from uwb_localization_baseline import (
    geometry_proxy_at_position,
    range_error_rows,
    summarize_range_errors,
)


class LocalizationBaselineTest(unittest.TestCase):
    def test_load_capture_reads_xz_and_excludes_fused_position(self) -> None:
        records = [
            {
                "kind": "capture_start",
                "capture_id": "xz_baseline",
                "protocol": "native_ds",
                "collector_wall_ns": 1_000_000_000,
            },
            {
                "kind": "position",
                "protocol": "native_ds",
                "tag_id": 1,
                "module_id": 1,
                "uptime_ms": 100,
                "collector_wall_ns": 1_100_000_000,
                "x_m": 1.0,
                "y_m": 2.0,
            },
            {
                "kind": "position",
                "protocol": "native_ds",
                "tag_id": 1,
                "module_id": 1,
                "uptime_ms": 101,
                "collector_wall_ns": 1_101_000_000,
                "x_m": 9.0,
                "y_m": 9.0,
                "imu_fused": True,
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "capture.jsonl.xz"
            with lzma.open(path, "wt", encoding="utf-8") as handle:
                for record in records:
                    handle.write(json.dumps(record) + "\n")
            capture = load_capture(path, 1)

        self.assertEqual(capture.protocol, "native_ds")
        self.assertEqual(len(capture.positions), 1)
        self.assertEqual(capture.fused_positions_excluded, 1)

    def test_load_capture_reads_structured_uwb_measurement_ranges(self) -> None:
        records = [
            {
                "kind": "capture_start",
                "capture_id": "measurement_ranges",
                "protocol": "native_ds",
                "collector_wall_ns": 1_000_000_000,
            },
            {
                "kind": "uwb_measurement",
                "measurement_kind": "native_ds_range",
                "protocol": "native_ds",
                "tag_id": 1,
                "anchor_id": 3,
                "frame_id": 17,
                "distance_m": 5.125,
                "raw_distance_m": 5.094,
                "collector_wall_ns": 1_100_000_000,
            },
            {
                "kind": "uwb_measurement",
                "measurement_kind": "range_difference",
                "protocol": "native_ds",
                "tag_id": 1,
                "initiator_id": 2,
                "responder_id": 3,
                "diff_m": 0.25,
                "collector_wall_ns": 1_200_000_000,
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "capture.jsonl"
            with path.open("w", encoding="utf-8") as handle:
                for record in records:
                    handle.write(json.dumps(record) + "\n")
            capture = load_capture(path, 1)

        self.assertEqual(len(capture.ranges), 1)
        self.assertEqual(capture.ranges[0]["kind"], "ds_range")
        self.assertEqual(capture.ranges[0]["first_id"], 1)
        self.assertEqual(capture.ranges[0]["second_id"], 3)
        self.assertAlmostEqual(capture.ranges[0]["distance_m"], 5.125)
        self.assertAlmostEqual(capture.ranges[0]["raw_distance_m"], 5.094)

    def test_geometry_proxy_is_finite_for_range_and_tdoa_models(self) -> None:
        point = {"x_m": 1.0, "y_m": 1.0}
        anchors = {2: (0.0, 0.0), 3: (2.0, 0.0), 4: (2.0, 2.0), 5: (0.0, 2.0)}
        native = geometry_proxy_at_position("native_ds", point, anchors)
        tdoa = geometry_proxy_at_position("flextdoa", point, anchors)
        self.assertIsNotNone(native)
        self.assertIsNotNone(tdoa)
        self.assertAlmostEqual(native[0], 1.0, places=6)
        self.assertAlmostEqual(native[1], 1.0, places=6)
        self.assertAlmostEqual(tdoa[1], 1.0, places=6)

    def test_range_error_uses_3d_rtk_distance_and_keeps_height_correction(self) -> None:
        capture = Capture(path=pathlib.Path("synthetic.jsonl"), capture_id="synthetic")
        capture.protocol = "native_ds"
        capture.gps = [
            {
                "time": 10.0,
                "module_id": 1,
                "valid": True,
                "quality": 4,
                "east_m": 0.0,
                "north_m": 0.0,
                "up_m": 0.0,
            },
            {
                "time": 10.0,
                "module_id": 2,
                "valid": True,
                "quality": 4,
                "east_m": 3.0,
                "north_m": 4.0,
                "up_m": 1.0,
            },
        ]
        truth = math.sqrt(26.0)
        capture.ranges = [
            {
                "time": 10.0,
                "kind": "ds_range",
                "first_id": 1,
                "second_id": 2,
                "distance_m": truth + 0.02,
                "frame_id": 1,
            }
        ]

        rows = range_error_rows(capture, 100.0)
        metrics = summarize_range_errors(rows)

        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]["error_m"], 0.02, places=9)
        self.assertAlmostEqual(rows[0]["rtk_horizontal_m"], 5.0, places=9)
        self.assertAlmostEqual(
            rows[0]["height_correction_m"], truth - 5.0, places=9
        )
        self.assertAlmostEqual(metrics["links"]["M1-M2"]["mean_bias_m"], 0.02)


if __name__ == "__main__":
    unittest.main()
