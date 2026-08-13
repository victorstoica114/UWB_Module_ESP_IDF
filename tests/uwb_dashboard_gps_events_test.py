#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import DashboardState  # noqa: E402


class DashboardGpsEventsTests(unittest.TestCase):
    def test_cursor_returns_only_new_tail_in_chronological_order(self) -> None:
        state = DashboardState(max_logs=16)
        for event_id in range(1, 9):
            state.gps_samples.append({"gps_event_id": event_id})
        state.next_gps_id = 9

        result = state.gps_after(5, 16)

        self.assertEqual(
            [sample["gps_event_id"] for sample in result["samples"]],
            [6, 7, 8],
        )
        self.assertEqual(result["next_id"], 9)

    def test_cursor_limit_keeps_newest_events(self) -> None:
        state = DashboardState(max_logs=16)
        for event_id in range(1, 9):
            state.gps_samples.append({"gps_event_id": event_id})

        result = state.gps_after(0, 3)

        self.assertEqual(
            [sample["gps_event_id"] for sample in result["samples"]],
            [6, 7, 8],
        )


if __name__ == "__main__":
    unittest.main()
