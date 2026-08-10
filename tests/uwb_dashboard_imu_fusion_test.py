#!/usr/bin/env python3

import pathlib
import sys
import threading
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import DashboardState  # noqa: E402


GRAVITY = 9.80665
CLIENT = ("127.0.0.1", 31337)


def imu_sample(module_id: int, uptime_ms: int) -> dict[str, object]:
    return {
        "topic": "bno085.accel",
        "module_id": module_id,
        "uptime_ms": uptime_ms,
        "x": 0.0,
        "y": 0.0,
        "z": GRAVITY,
        "quat_i": 0.0,
        "quat_j": 0.0,
        "quat_k": 0.0,
        "quat_real": 1.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "imu_valid": True,
    }


def position_sample(
    publisher_id: int,
    tag_id: int,
    uptime_ms: int,
    *,
    protocol: str = "flextdoa",
    x_m: float = 1.25,
    y_m: float = -0.5,
) -> dict[str, object]:
    topic = {
        "flextdoa": "uwb.flex_tdoa.position",
        "passive_ds": "uwb.passive_ds.position",
        "native_ds": "uwb.native_ds.position",
    }[protocol]
    return {
        "topic": topic,
        "module_id": publisher_id,
        "tag_id": tag_id,
        "uptime_ms": uptime_ms,
        "tdoa_protocol": protocol,
        "x_m": x_m,
        "y_m": y_m,
        "sigma_m": 0.2,
        "rms_m": 0.15,
    }


class DashboardImuFusionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.state = DashboardState(max_logs=20)

    def add(self, *samples: dict[str, object]) -> None:
        self.state.add_telemetry_samples(list(samples), CLIENT)

    def test_routes_imu_by_tag_and_keeps_raw_and_fused_events_separate(
        self,
    ) -> None:
        self.add(imu_sample(1, 90))
        self.add(position_sample(5, 1, 100))

        self.assertIn((1, "flextdoa", 1), self.state.imu_fusions)
        self.assertNotIn((5, "flextdoa", 1), self.state.imu_fusions)
        raw = self.state.tdoa_local_positions[1]
        self.assertEqual(raw["module_id"], 5)
        self.assertEqual(raw["imu_fusion_module_id"], 1)
        self.assertEqual(raw["x_m"], 1.25)
        self.assertTrue(raw["imu_fused_valid"])

        events = list(self.state.tdoa_position_events)
        self.assertEqual(len(events), 2)
        self.assertNotIn("imu_fused", events[0])
        self.assertTrue(events[1]["imu_fused"])
        self.assertEqual(events[1]["module_id"], 1)
        self.assertEqual(events[1]["tag_id"], 1)
        self.assertEqual(events[1]["tdoa_protocol"], "flextdoa")
        self.assertEqual(
            [item["position_stream_event_id"] for item in events],
            [1, 2],
        )

        fused = self.state.snapshot()["imu_fusion"]["1:flextdoa:1"]
        self.assertTrue(fused["ready"])
        self.assertEqual(fused["module_id"], 1)
        self.assertEqual(fused["tag_id"], 1)
        self.assertEqual(fused["tdoa_protocol"], "flextdoa")
        self.assertIsInstance(fused["received_at"], float)

    def test_protocol_switch_retires_the_previous_track(self) -> None:
        self.add(imu_sample(1, 90))
        self.add(position_sample(7, 1, 100, protocol="flextdoa"))
        self.add(position_sample(7, 1, 200, protocol="passive_ds"))

        self.assertEqual(
            set(self.state.imu_fusions), {(1, "passive_ds", 1)}
        )
        self.assertNotIn("1:flextdoa:1", self.state.imu_fusion_latest)
        self.assertIn("1:passive_ds:1", self.state.imu_fusion_latest)
        self.assertEqual(
            self.state.tdoa_local_positions[1]["tdoa_protocol"],
            "passive_ds",
        )

    def test_imu_gap_emits_reset_and_clears_snapshot_readiness(self) -> None:
        self.add(imu_sample(1, 10))
        self.add(position_sample(5, 1, 20))
        self.add(imu_sample(1, 400))

        event = self.state.tdoa_position_events[-1]
        self.assertTrue(event["imu_fused"])
        self.assertTrue(event["imu_fusion_reset"])
        self.assertEqual(event["solution_kind"], "imu_reset")
        self.assertIn("reset", event["imu_fusion_flags"])
        self.assertIn("reset:imu_gap", event["imu_fusion_flags"])
        self.assertNotIn("x_m", event)
        self.assertNotIn("y_m", event)

        fused = self.state.snapshot()["imu_fusion"]["1:flextdoa:1"]
        self.assertFalse(fused["ready"])
        self.assertEqual(fused["tag_id"], 1)
        self.assertEqual(fused["tdoa_protocol"], "flextdoa")

    def test_duplicate_position_does_not_emit_a_stale_fused_event(self) -> None:
        self.add(imu_sample(1, 10))
        self.add(position_sample(5, 1, 20))
        event_count = len(self.state.tdoa_position_events)

        self.add(position_sample(5, 1, 20, x_m=9.0, y_m=9.0))

        self.assertEqual(len(self.state.tdoa_position_events), event_count + 1)
        duplicate = self.state.tdoa_position_events[-1]
        self.assertNotIn("imu_fused", duplicate)
        self.assertIn(
            "position_duplicate_uptime", duplicate["imu_fusion_flags"]
        )

    def test_invalid_full_imu_reaches_fusion_but_is_not_bootstrap_state(
        self,
    ) -> None:
        self.add(imu_sample(1, 10))
        self.add(position_sample(5, 1, 20))
        invalid = imu_sample(1, 40)
        invalid["imu_valid"] = False

        self.add(invalid)

        self.assertEqual(self.state.latest_imu_by_module[1]["uptime_ms"], 10)
        fused = self.state.snapshot()["imu_fusion"]["1:flextdoa:1"]
        self.assertIn("imu_invalid", fused["flags"])
        self.assertEqual(fused["uptime_ms"], 40)

    def test_snapshot_and_imu_updates_are_thread_safe(self) -> None:
        self.add(imu_sample(1, 10))
        self.add(position_sample(5, 1, 20))
        errors: list[BaseException] = []
        start = threading.Barrier(2)

        def update_imu() -> None:
            try:
                start.wait()
                for uptime_ms in range(30, 1030, 10):
                    self.add(imu_sample(1, uptime_ms))
            except BaseException as exc:  # pragma: no cover - failure capture
                errors.append(exc)

        def read_snapshots() -> None:
            try:
                start.wait()
                for _ in range(100):
                    snapshot = self.state.snapshot()
                    fused = snapshot["imu_fusion"].get("1:flextdoa:1")
                    if fused is not None:
                        self.assertEqual(fused["tag_id"], 1)
            except BaseException as exc:  # pragma: no cover - failure capture
                errors.append(exc)

        threads = [
            threading.Thread(target=update_imu),
            threading.Thread(target=read_snapshots),
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=5)

        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
