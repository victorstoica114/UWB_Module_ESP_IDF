#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_compare_collect import ds_range_key, status_summary


class DsRangeKeyTest(unittest.TestCase):
    def test_slot_id_prevents_low_16_sequence_collision(self) -> None:
        first = {"tag_id": 1, "anchor_id": 2, "seq": 17, "slot_id": 17}
        wrapped = {
            "tag_id": 1,
            "anchor_id": 2,
            "seq": 17,
            "slot_id": 65553,
        }

        self.assertNotEqual(ds_range_key(first), ds_range_key(wrapped))

    def test_slot_id_is_authoritative_when_available(self) -> None:
        first = {"tag_id": 1, "anchor_id": 2, "seq": 17, "slot_id": 42}
        duplicate = {
            "tag_id": 1,
            "anchor_id": 2,
            "seq": 18,
            "slot_id": 42,
        }

        self.assertEqual(ds_range_key(first), ds_range_key(duplicate))

    def test_legacy_log_ranges_still_use_sequence_and_context(self) -> None:
        first = {
            "tag_id": 1,
            "anchor_id": 2,
            "seq": 17,
            "context_token": 3,
            "round_index": 4,
        }
        next_round = {**first, "round_index": 5}

        self.assertNotEqual(ds_range_key(first), ds_range_key(next_round))


class StatusSummaryTest(unittest.TestCase):
    def test_keeps_native_ds_telemetry_drop_counters(self) -> None:
        summary = status_summary(
            {
                "statuses": [
                    {
                        "module_id": 1,
                        "wireless_telemetry_dropped": 7,
                        "wireless_telemetry_drop_full": 5,
                        "wireless_telemetry_send_failures": 2,
                        "wireless_telemetry_queue_high_water": 11,
                    }
                ]
            }
        )

        self.assertEqual(summary[0]["wireless_telemetry_dropped"], 7)
        self.assertEqual(summary[0]["wireless_telemetry_drop_full"], 5)
        self.assertEqual(summary[0]["wireless_telemetry_send_failures"], 2)
        self.assertEqual(
            summary[0]["wireless_telemetry_queue_high_water"], 11
        )


if __name__ == "__main__":
    unittest.main()
