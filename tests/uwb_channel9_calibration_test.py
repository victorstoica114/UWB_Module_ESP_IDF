#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_channel9_calibration import fit_device_terms, summarize_links


def range_row(first: int, second: int, error_m: float) -> dict:
    return {
        "first_id": first,
        "second_id": second,
        "link": f"M{min(first, second)}-M{max(first, second)}",
        "error_m": error_m,
    }


class Channel9CalibrationTest(unittest.TestCase):
    def test_star_graph_refuses_arbitrary_per_device_delay(self) -> None:
        rows = [range_row(1, anchor, 0.01 * anchor) for anchor in (2, 3, 4, 5)]
        fit = fit_device_terms(rows)
        self.assertEqual(fit["matrix_rank"], 4)
        self.assertEqual(fit["unknown_count"], 5)
        self.assertFalse(fit["identifiable"])
        self.assertIsNone(fit["antenna_delay_corrections"])

    def test_complete_graph_recovers_device_additive_terms(self) -> None:
        biases = {1: 0.01, 2: -0.02, 3: 0.03, 4: -0.01, 5: 0.02}
        rows = []
        for first in biases:
            for second in biases:
                if second <= first:
                    continue
                rows.append(
                    range_row(first, second, biases[first] + biases[second])
                )
        fit = fit_device_terms(rows)
        self.assertTrue(fit["identifiable"])
        for module, expected in biases.items():
            self.assertAlmostEqual(fit["device_biases"][str(module)], expected)
        self.assertAlmostEqual(fit["link_fit_residual_rms_m"], 0.0, places=12)

    def test_link_validation_is_kept_separate_from_calibration(self) -> None:
        calibration = [range_row(1, 2, 0.08), range_row(1, 2, 0.10)]
        validation = [range_row(1, 2, 0.002), range_row(1, 2, -0.004)]
        links = summarize_links(calibration, validation, {"M1-M2": 78})
        self.assertEqual(len(links), 1)
        self.assertEqual(links[0]["applied_bias_mm"], 78)
        self.assertEqual(links[0]["direct_rtk_recommended_bias_mm"], 90)
        self.assertAlmostEqual(links[0]["validation"]["median_bias_m"], -0.001)


if __name__ == "__main__":
    unittest.main()
