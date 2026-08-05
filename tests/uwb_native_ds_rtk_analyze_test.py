import math
import unittest

from tools.uwb_native_ds_rtk_analyze import (
    fit_rigid_2d,
    geometry_difference_cm,
    runtime_anchors,
    transform_2d,
)


class RigidAlignmentTest(unittest.TestCase):
    def test_recovers_rotation_and_translation(self):
        source = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
        expected_rotation = math.radians(-7.5)
        expected_translation = (1.25, -0.75)
        target = [
            transform_2d(point, expected_rotation, *expected_translation)
            for point in source
        ]

        rotation, translation_x, translation_y = fit_rigid_2d(source, target)

        self.assertAlmostEqual(rotation, expected_rotation, places=12)
        self.assertAlmostEqual(translation_x, expected_translation[0], places=12)
        self.assertAlmostEqual(translation_y, expected_translation[1], places=12)

    def test_rejects_degenerate_source(self):
        with self.assertRaises(ValueError):
            fit_rigid_2d([(1.0, 1.0), (1.0, 1.0)], [(0.0, 0.0), (2.0, 2.0)])

    def test_reads_runtime_anchor_geometry_in_meters(self):
        events = [
            {"kind": "gps_fix"},
            {
                "kind": "status",
                "modules": [{
                    "runtime_anchor_ids": [2, 3, 4, 5],
                    "runtime_flex_tdoa_anchor_x_mm": [0, 3844, -3120, 1055],
                    "runtime_flex_tdoa_anchor_y_mm": [0, 2923, 3899, 6836],
                }],
            },
        ]

        self.assertEqual(
            runtime_anchors(events),
            {
                2: (0.0, 0.0),
                3: (3.844, 2.923),
                4: (-3.12, 3.899),
                5: (1.055, 6.836),
            },
        )

    def test_reports_external_geometry_mismatch(self):
        difference = geometry_difference_cm(
            {2: (0.0, 0.0), 3: (0.0, 4.85)},
            {2: (0.0, 0.0), 3: (3.844, 2.923)},
        )

        self.assertIsNotNone(difference)
        self.assertEqual(difference["anchors_compared"], 2)
        self.assertGreater(difference["max_cm"], 300.0)


if __name__ == "__main__":
    unittest.main()
