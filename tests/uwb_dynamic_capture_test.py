#!/usr/bin/env python3

import collections
import io
import json
import pathlib
import sys
import unittest
import urllib.error


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dynamic_capture import (
    accel_cursor,
    fetch_capture_snapshot,
    gps_records,
    gps_cursor,
    gps_telemetry_record,
    measurement_cursor,
    position_cursor,
    position_event_kind,
    write_position_event,
)


class CaptureSnapshotTest(unittest.TestCase):
    def test_prefers_compact_capture_snapshot(self) -> None:
        calls: list[str] = []

        def fetcher(url: str, timeout_sec: float) -> dict:
            calls.append(url)
            self.assertEqual(timeout_sec, 1.5)
            return {"statuses": []}

        payload, path = fetch_capture_snapshot(
            "http://dashboard/", 1.5, fetcher=fetcher
        )

        self.assertEqual(payload, {"statuses": []})
        self.assertEqual(path, "/api/capture-snapshot")
        self.assertEqual(calls, ["http://dashboard/api/capture-snapshot"])

    def test_falls_back_once_to_legacy_snapshot_on_404(self) -> None:
        calls: list[str] = []

        def fetcher(url: str, timeout_sec: float) -> dict:
            calls.append(url)
            if url.endswith("/api/capture-snapshot"):
                raise urllib.error.HTTPError(url, 404, "not found", None, None)
            return {"statuses": [{"module_id": 1}]}

        payload, path = fetch_capture_snapshot(
            "http://dashboard", fetcher=fetcher
        )
        cached_payload, cached_path = fetch_capture_snapshot(
            "http://dashboard", preferred_path=path, fetcher=fetcher
        )

        self.assertEqual(payload, {"statuses": [{"module_id": 1}]})
        self.assertEqual(cached_payload, payload)
        self.assertEqual(path, "/api/snapshot")
        self.assertEqual(cached_path, "/api/snapshot")
        self.assertEqual(
            calls,
            [
                "http://dashboard/api/capture-snapshot",
                "http://dashboard/api/snapshot",
                "http://dashboard/api/snapshot",
            ],
        )

    def test_does_not_mask_non_404_compact_endpoint_errors(self) -> None:
        calls: list[str] = []

        def fetcher(url: str, timeout_sec: float) -> dict:
            calls.append(url)
            raise urllib.error.HTTPError(url, 500, "failed", None, None)

        with self.assertRaises(urllib.error.HTTPError):
            fetch_capture_snapshot("http://dashboard", fetcher=fetcher)

        self.assertEqual(calls, ["http://dashboard/api/capture-snapshot"])


class CursorTest(unittest.TestCase):
    def test_accel_next_id_is_next_unassigned_id(self) -> None:
        payload = {"samples": [{"sample_id": 10}], "next_id": 11}
        self.assertEqual(accel_cursor(payload, 9), 10)

    def test_accel_cursor_recovers_after_dashboard_restart(self) -> None:
        self.assertEqual(accel_cursor({"samples": [], "next_id": 3}, 900), 2)

    def test_position_cursor_uses_stream_event_id(self) -> None:
        payload = {
            "events": [{"position_stream_event_id": 42}],
            "next_event_id": 42,
        }
        self.assertEqual(position_cursor(payload, 41), 42)

    def test_measurement_cursor_tracks_last_assigned_id(self) -> None:
        payload = {
            "events": [{"uwb_measurement_event_id": 77}],
            "next_id": 78,
        }
        self.assertEqual(measurement_cursor(payload, 76), 77)
        self.assertEqual(
            measurement_cursor({"events": [], "next_id": 3}, 900), 2
        )


class PositionEventTest(unittest.TestCase):
    def test_classifies_dashboard_fused_positions_separately(self) -> None:
        self.assertEqual(position_event_kind({"imu_fused": True}), "position_fused")
        self.assertEqual(
            position_event_kind({"kind": "position_fused"}), "position_fused"
        )
        self.assertEqual(position_event_kind({"imu_fused": False}), "position")
        self.assertEqual(position_event_kind({}), "position")

    def test_writer_owns_kind_and_counts_fused_stream_separately(self) -> None:
        handle = io.StringIO()
        counts: collections.Counter[str] = collections.Counter()

        write_position_event(
            handle,
            "passive_ds",
            "capture-1",
            {"kind": "position", "imu_fused": True, "x_m": 1.0, "y_m": 2.0},
            counts,
        )

        record = json.loads(handle.getvalue())
        self.assertEqual(record["kind"], "position_fused")
        self.assertTrue(record["imu_fused"])
        self.assertEqual(counts["position_fused"], 1)
        self.assertEqual(counts["position"], 0)


class GpsRecordTest(unittest.TestCase):
    def test_high_rate_gps_cursor_and_record_mapping(self) -> None:
        payload = {
            "next_id": 10,
            "samples": [
                {
                    "gps_event_id": 9,
                    "module_id": 1,
                    "gga_sequence": 101,
                    "fix_valid": True,
                    "fix_quality": 4,
                    "latitude_deg": 44.4,
                    "longitude_deg": 26.1,
                    "estimated_measurement_wall_ns": 123,
                }
            ],
        }
        self.assertEqual(gps_cursor(payload), 9)
        record = gps_telemetry_record(payload["samples"][0])
        self.assertEqual(record["gps_gga_count"], 101)
        self.assertEqual(record["gps_latitude_deg"], 44.4)
        self.assertTrue(record["gps_fix_valid"])

    def test_deduplicates_gga_and_preserves_rtk_fields(self) -> None:
        snapshot = {
            "statuses": [
                {
                    "module_id": 1,
                    "gps_gga_count": 7,
                    "gps_latitude_deg": 44.4,
                    "gps_longitude_deg": 26.1,
                    "gps_fix_valid": True,
                    "gps_fix_quality": 4,
                    "gps_last_fix_age_ms": 25,
                }
            ]
        }
        seen: set[tuple[int, int]] = set()
        first = gps_records(snapshot, seen, 1_000_000_000)
        second = gps_records(snapshot, seen, 1_000_000_000)
        self.assertEqual(len(first), 1)
        self.assertEqual(second, [])
        self.assertEqual(first[0]["gps_fix_quality"], 4)
        self.assertEqual(
            first[0]["estimated_measurement_wall_ns"], 975_000_000
        )


if __name__ == "__main__":
    unittest.main()
