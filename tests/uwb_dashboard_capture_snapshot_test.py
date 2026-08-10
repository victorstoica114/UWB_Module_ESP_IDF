#!/usr/bin/env python3

import json
import pathlib
import sys
import threading
import time
import unittest
import urllib.request
from unittest import mock


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import DashboardHttpServer, DashboardState  # noqa: E402
from uwb_dynamic_capture import compact_snapshot, status_summary  # noqa: E402


CLIENT = ("127.0.0.1", 31337)


def populated_state() -> DashboardState:
    state = DashboardState(max_logs=200)
    state.set_client_count(3, "logs")
    state.set_telemetry_client_count(4, "telemetry")
    state.set_status(
        1,
        {
            "module_id": 1,
            "hostname": "uwb-module-1",
            "target": "192.0.2.1",
            "wifi_connected": True,
            "runtime_mode_name": "uwb_flex_tdoa",
            "runtime_tag_id": 4,
            "runtime_anchor_ids": [1, 2, 3],
            "gps_fix_valid": True,
            "gps_fix_quality": 4,
            "gps_gga_count": 10,
            "gps_latitude_deg": 44.4,
            "gps_longitude_deg": 26.1,
            "native_ds_pipeline_stats": {"completed_ranges": 7},
            "diagnostic_field_not_filtered": "kept in full status",
        },
    )

    observations = []
    for sequence in range(120):
        observations.append(
            {
                "topic": "uwb.flex_tdoa.observation",
                "module_id": 4,
                "tag_id": 4,
                "initiator_id": 1,
                "responder_id": 2,
                "seq": sequence,
                "slot_id": sequence,
                "responder_index": 0,
                "diff_m": 0.25 + sequence / 10000.0,
                "raw_diff_m": 0.26 + sequence / 10000.0,
                "anchor_distance_m": 2.5,
                "tdoa_protocol": "flextdoa",
            }
        )
    state.add_telemetry_samples(observations, CLIENT)
    state.add_telemetry_samples(
        [
            {
                "topic": "uwb.native_ds.tag_range",
                "module_id": 4,
                "initiator_id": 4,
                "responder_id": 1,
                "seq": 12,
                "slot_id": 12,
                "distance_m": 1.75,
                "raw_distance_m": 1.76,
            },
            {
                "topic": "uwb.flex_tdoa.position",
                "module_id": 4,
                "tag_id": 4,
                "uptime_ms": 1234,
                "tdoa_protocol": "flextdoa",
                "x_m": 1.0,
                "y_m": 2.0,
                "sigma_m": 0.1,
                "rms_m": 0.2,
            },
            *[
                {
                    "topic": "uwb.flextdoa.geometry",
                    "module_id": 4,
                    "tag_id": 4,
                    "anchor_id": anchor_id,
                    "anchor_count": 3,
                    "geometry_version": 2,
                    "x_m": float(anchor_id),
                    "y_m": float(anchor_id) / 2.0,
                    "fit_rms_m": 0.03,
                    "tdoa_protocol": "flextdoa",
                }
                for anchor_id in (1, 2, 3)
            ],
        ],
        CLIENT,
    )
    return state


class CaptureSnapshotPayloadTest(unittest.TestCase):
    def test_matches_collector_inputs_without_recent_histories(self) -> None:
        state = populated_state()
        frozen_now = time.time()
        with mock.patch("uwb_dashboard.time.time", return_value=frozen_now):
            full = state.snapshot()
            capture = state.capture_snapshot()

        self.assertEqual(
            set(capture),
            {
                "client_count",
                "telemetry_client_count",
                "statuses",
                "ranging",
                "tdoa",
            },
        )
        self.assertEqual(capture["client_count"], full["client_count"])
        self.assertEqual(
            capture["telemetry_client_count"],
            full["telemetry_client_count"],
        )
        self.assertEqual(status_summary(capture), status_summary(full))
        self.assertEqual(
            capture["ranging"],
            {"distances": full["ranging"]["distances"]},
        )
        for key in (
            "observations",
            "anchor_distances",
            "local_positions",
            "local_geometries",
        ):
            self.assertEqual(capture["tdoa"][key], full["tdoa"][key])
        self.assertEqual(compact_snapshot(capture), compact_snapshot(full))
        self.assertNotIn("recent_observations", capture["tdoa"])
        self.assertNotIn("recent_anchor_ranges", capture["tdoa"])
        self.assertNotIn("accel_history", capture)
        self.assertNotIn("log_count", capture)

    def test_payload_is_materially_smaller_than_full_snapshot(self) -> None:
        state = populated_state()
        full_raw = json.dumps(state.snapshot(), separators=(",", ":")).encode()
        capture_raw = json.dumps(
            state.capture_snapshot(), separators=(",", ":")
        ).encode()

        self.assertLess(len(capture_raw), len(full_raw) // 2)


class CaptureSnapshotEndpointTest(unittest.TestCase):
    def test_serves_compact_read_only_payload(self) -> None:
        state = populated_state()
        server = DashboardHttpServer(
            ("127.0.0.1", 0),
            state,
            targets=[],
            token="",
            quiet=True,
            timeout_sec=1.0,
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            with urllib.request.urlopen(
                base + "/api/capture-snapshot", timeout=2.0
            ) as response:
                capture_raw = response.read()
                self.assertEqual(response.status, 200)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
            with urllib.request.urlopen(
                base + "/api/snapshot", timeout=2.0
            ) as response:
                full_raw = response.read()
            capture = json.loads(capture_raw)
            self.assertEqual(capture["client_count"], 3)
            self.assertIn("distances", capture["ranging"])
            self.assertNotIn("recent_observations", capture["tdoa"])
            self.assertLess(len(capture_raw), len(full_raw) // 2)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
