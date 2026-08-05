import math
import unittest

from tools.uwb_native_ds_rtk_analyze import fit_rigid_2d, transform_2d


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


if __name__ == "__main__":
    unittest.main()
