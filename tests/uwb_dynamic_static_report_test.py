import json
import pathlib
import tempfile
import unittest

from tools.uwb_dynamic_static_report import (
    cdf_axis_max,
    cdf_sample_indices,
    fit_rigid,
    jump_metrics,
    load_capture,
    stream_metrics,
    transform_positions,
)


class UptimeEpochMetricsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.points = [
            {"time": 1.00, "uptime_ms": 51_000, "x_m": 0.00, "y_m": 0.0},
            {"time": 1.05, "uptime_ms": 51_050, "x_m": 0.05, "y_m": 0.0},
            {"time": 1.10, "uptime_ms": 51_100, "x_m": 0.10, "y_m": 0.0},
            {"time": 2.00, "uptime_ms": 100, "x_m": 1.00, "y_m": 0.0},
            {"time": 2.05, "uptime_ms": 150, "x_m": 1.05, "y_m": 0.0},
            {"time": 2.10, "uptime_ms": 200, "x_m": 1.10, "y_m": 0.0},
        ]

    def test_stream_gaps_do_not_bridge_uptime_reboot(self) -> None:
        metrics = stream_metrics(self.points, 1.1, 1.0, 2.1)
        self.assertEqual(metrics["uptime_epoch_count"], 2)
        self.assertEqual(metrics["unique_uptime_events"], 6)
        self.assertEqual(metrics["gap_max_ms"], 50.0)
        self.assertAlmostEqual(metrics["uptime_span_s"], 0.2)

    def test_jump_speed_does_not_bridge_uptime_reboot(self) -> None:
        metrics = jump_metrics(self.points)
        self.assertEqual(metrics["step_speed_n"], 4)
        self.assertAlmostEqual(metrics["step_speed_max_mps"], 1.0)


class RigidAlignmentTest(unittest.TestCase):
    source = {
        2: (0.0, 0.0),
        3: (0.0, 2.0),
        4: (3.0, 2.0),
        5: (3.0, 0.0),
    }

    @staticmethod
    def target(*, reflected: bool) -> dict[int, tuple[float, float, float]]:
        # Rotate by 90 degrees and translate by (10, -4).  Reflection, when
        # requested, mirrors the source y coordinate before that transform.
        result = {}
        for anchor_id, (x_m, y_m) in RigidAlignmentTest.source.items():
            local_y = -y_m if reflected else y_m
            result[anchor_id] = (10.0 - local_y, -4.0 + x_m, 0.0)
        return result

    def assert_transformed_probe(self, fit: dict, expected: tuple[float, float]) -> None:
        alignment = {
            **fit,
            "geometry_version": 7,
            "time": 10.0,
            "dynamic": False,
        }
        points, diagnostics = transform_positions(
            [
                {
                    "time": 10.1,
                    "geometry_version": 7,
                    "x_m": 1.0,
                    "y_m": 0.5,
                }
            ],
            [alignment],
            10.0,
        )
        self.assertEqual(diagnostics["transformed_positions"], 1)
        self.assertAlmostEqual(points[0]["east_m"], expected[0])
        self.assertAlmostEqual(points[0]["north_m"], expected[1])

    def test_direct_frame_is_preserved(self) -> None:
        fit = fit_rigid(self.source, self.target(reflected=False))
        self.assertIsNotNone(fit)
        assert fit is not None
        self.assertFalse(fit["reflected"])
        self.assertAlmostEqual(fit["scale"], 1.0)
        self.assertAlmostEqual(fit["fit_rmse_m"], 0.0)
        self.assert_transformed_probe(fit, (9.5, -3.0))

    def test_mirrored_frame_is_selected_and_applied(self) -> None:
        fit = fit_rigid(self.source, self.target(reflected=True))
        self.assertIsNotNone(fit)
        assert fit is not None
        self.assertTrue(fit["reflected"])
        self.assertAlmostEqual(fit["scale"], 1.0)
        self.assertAlmostEqual(fit["fit_rmse_m"], 0.0)
        self.assert_transformed_probe(fit, (10.5, -3.0))


class CdfRenderingTest(unittest.TestCase):
    def test_axis_includes_full_tail(self) -> None:
        values = [0.1] * 99 + [1.25]
        self.assertGreater(cdf_axis_max(values), max(values))

    def test_downsampling_retains_both_endpoints(self) -> None:
        indices = cdf_sample_indices(3_635, 600)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 3_634)


class CaptureProtocolFilterTest(unittest.TestCase):
    def test_mismatched_position_protocol_is_excluded(self) -> None:
        records = [
            {
                "kind": "capture_start",
                "capture_id": "static_passive_ds_test",
                "protocol": "passive_ds",
                "received_at": 1.0,
            },
            {
                "kind": "position",
                "protocol": "passive_ds",
                "tdoa_protocol": "passive_ds",
                "tag_id": 1,
                "received_at": 1.1,
                "uptime_ms": 100,
                "x_m": 1.0,
                "y_m": 2.0,
            },
            {
                "kind": "position",
                "protocol": "flextdoa",
                "tdoa_protocol": "flextdoa",
                "tag_id": 1,
                "received_at": 1.2,
                "uptime_ms": 200,
                "x_m": 9.0,
                "y_m": 9.0,
            },
            {
                "kind": "capture_end",
                "capture_id": "static_passive_ds_test",
                "protocol": "passive_ds",
                "received_at": 1.3,
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "mixed.passive_ds.jsonl"
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            capture = load_capture(path, tag_id=1)
        self.assertEqual(len(capture.positions), 1)
        self.assertEqual(capture.protocol_mismatch_positions_excluded, 1)


if __name__ == "__main__":
    unittest.main()
