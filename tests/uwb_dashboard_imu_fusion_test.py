#!/usr/bin/env python3

import pathlib
import math
import sys
import threading
import time
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import DashboardState, rtk_course_to_uwb_heading  # noqa: E402


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

    def test_gptimer_is_aligned_once_without_losing_sub_ms_time(self) -> None:
        first = imu_sample(1, 1000)
        first["fusion_time_ticks"] = 2_000_000
        second = imu_sample(1, 1005)
        second["fusion_time_ticks"] = 2_050_120

        self.add(first, second)

        self.assertEqual(first["sample_time_us"], 1_000_000)
        self.assertEqual(second["sample_time_us"], 1_005_012)
        self.assertEqual(
            second["sample_time_source"], "gptimer64_aligned"
        )
        self.assertEqual(first["fusion_clock_generation"], 1)
        self.assertEqual(second["fusion_clock_generation"], 1)

    def test_gptimer_alignment_resets_after_module_reboot(self) -> None:
        before = imu_sample(1, 50_000)
        before["fusion_time_ticks"] = 400_000_000
        after = imu_sample(1, 1200)
        after["fusion_time_ticks"] = 2_000_000

        self.add(before, after)

        self.assertEqual(after["sample_time_us"], 1_200_000)
        self.assertEqual(after["fusion_clock_generation"], 2)
        self.assertEqual(len(self.state.accel_history[1]), 1)
        self.assertIs(self.state.accel_history[1][0], after)

    def test_compact_accel_uses_separate_recent_orientation(self) -> None:
        orientation = {
            "topic": "bno085.orientation",
            "module_id": 1,
            "uptime_ms": 1000,
            "fusion_time_ticks": 10_000_000,
            "quat_i": 0.0,
            "quat_j": 0.0,
            "quat_k": 0.0,
            "quat_real": 1.0,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "gyro_reports": 1,
            "gyro_time_flags": 0,
        }
        accel = {
            "topic": "bno085.accel",
            "module_id": 1,
            "uptime_ms": 1002,
            "fusion_time_ticks": 10_020_000,
            "x": 0.0,
            "y": 0.0,
            "z": GRAVITY,
            "reports": 1,
            "accuracy": 3,
            "imu_compact": True,
        }
        self.add(orientation, accel)
        self.assertTrue(accel["imu_valid"])
        self.assertEqual(accel["quat_real"], 1.0)
        self.assertEqual(accel["orientation_age_us"], 2000)

    def test_clock_anchor_removes_batch_arrival_bias(self) -> None:
        clock = {
            "topic": "bno085.clock",
            "module_id": 1,
            "uptime_ms": 1100,
            "fusion_time_ticks": 10_000_000,
            "fusion_timer_hz": 10_000_000,
            "esp_timer_us": 1_000_007,
        }
        accel = imu_sample(1, 1110)
        accel["fusion_time_ticks"] = 10_020_000
        self.add(clock, accel)
        self.assertEqual(accel["sample_time_us"], 1_002_007)
        self.assertEqual(accel["sample_time_source"], "gptimer64_aligned")

    def test_gps_telemetry_updates_status_and_cursor(self) -> None:
        gps = {
            "topic": "gps.gga",
            "module_id": 1,
            "uptime_ms": 1000,
            "sample_monotonic_us": 999_500,
            "gga_sequence": 17,
            "fix_valid": True,
            "fix_quality": 4,
            "latitude_deg": 44.4,
            "longitude_deg": 26.1,
            "altitude_m": 80.0,
            "speed_mps": 1.2,
            "course_deg": 90.0,
            "hdop": 0.7,
            "satellites": 20,
        }
        self.add(gps)
        payload = self.state.gps_after(0, 10)
        self.assertEqual(len(payload["samples"]), 1)
        self.assertEqual(payload["samples"][0]["gps_event_id"], 1)
        self.assertEqual(self.state.status_by_module[1]["gps_gga_count"], 17)
        self.assertEqual(
            self.state.status_by_module[1]["gps_fix_quality_text"],
            "rtk_fixed",
        )

    def test_sequence_stats_count_real_missing_samples(self) -> None:
        first = imu_sample(1, 1000)
        first["reports"] = 10
        second = imu_sample(1, 1004)
        second["reports"] = 13
        self.add(first, second)
        stats = self.state.telemetry_stats()["sequence"]["1:bno085.accel"]
        self.assertEqual(stats["received"], 2)
        self.assertEqual(stats["missing"], 2)
        self.assertEqual(second["sequence_gap"], 2)

    def test_raw_500hz_is_retained_while_ui_is_limited_to_100hz(self) -> None:
        samples = []
        for index in range(10):
            sample = imu_sample(1, 1000 + index * 2)
            sample["fusion_time_ticks"] = 10_000_000 + index * 20_000
            sample["reports"] = index + 1
            samples.append(sample)
        self.add(*samples)
        raw = self.state.raw_accel_after(0, 100)["samples"]
        ui = self.state.accel_after(0, 100)["samples"]
        self.assertEqual(len(raw), 10)
        self.assertEqual(len(ui), 2)

    @staticmethod
    def rtk_geometry() -> tuple[dict[str, dict[str, float]], dict[int, dict[str, object]]]:
        latitude = 44.0
        longitude = 26.0
        latitude_scale = 6378137.0 * math.pi / 180.0
        longitude_scale = latitude_scale * math.cos(math.radians(latitude))
        geometry = {
            "2": {"x": 0.0, "y": 0.0},
            "3": {"x": 0.0, "y": 10.0},
            "4": {"x": -10.0, "y": 0.0},
        }
        statuses = {
            2: {
                "gps_fix_valid": True,
                "gps_fix_quality": 4,
                "gps_latitude_deg": latitude,
                "gps_longitude_deg": longitude,
            },
            3: {
                "gps_fix_valid": True,
                "gps_fix_quality": 4,
                "gps_latitude_deg": latitude,
                "gps_longitude_deg": longitude + 10.0 / longitude_scale,
            },
            4: {
                "gps_fix_valid": True,
                "gps_fix_quality": 4,
                "gps_latitude_deg": latitude + 10.0 / latitude_scale,
                "gps_longitude_deg": longitude,
            },
        }
        return geometry, statuses

    def test_rtk_course_is_rotated_into_uwb_frame(self) -> None:
        geometry, statuses = self.rtk_geometry()

        result = rtk_course_to_uwb_heading(geometry, statuses, 90.0)

        self.assertIsNotNone(result)
        heading, fit_rms = result  # type: ignore[misc]
        self.assertAlmostEqual(heading, math.pi / 2.0, places=5)
        self.assertLess(fit_rms, 0.001)

    def test_fixed_rtk_course_initializes_dashboard_fusion_yaw(self) -> None:
        geometry, statuses = self.rtk_geometry()
        for module_id, status in statuses.items():
            self.state.set_status(module_id, status)
        self.state.set_status(
            1,
            {
                "gps_fix_valid": True,
                "gps_fix_quality": 4,
                "gps_last_fix_age_ms": 20,
                "gps_speed_mps": 1.0,
                "gps_course_deg": 90.0,
            },
        )
        self.state.tdoa_local_geometries["flextdoa:1"] = {
            "complete": True,
            "anchors": geometry,
            "received_at": time.time(),
        }
        self.add(imu_sample(1, 90))

        self.add(position_sample(5, 1, 100))

        diagnostics = self.state.snapshot()["imu_fusion"][
            "1:flextdoa:1"
        ]["diagnostics"]
        self.assertTrue(diagnostics["yaw_alignment_valid"])
        self.assertEqual(diagnostics["yaw_alignment_source"], "rtk_course")

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
        common_ekf = self.state.imu_fusions[(1, "flextdoa", 1)]
        self.add(position_sample(7, 1, 200, protocol="passive_ds"))

        self.assertEqual(
            set(self.state.imu_fusions), {(1, "passive_ds", 1)}
        )
        self.assertNotIn("1:flextdoa:1", self.state.imu_fusion_latest)
        self.assertIn("1:passive_ds:1", self.state.imu_fusion_latest)
        self.assertIs(
            self.state.imu_fusions[(1, "passive_ds", 1)], common_ekf
        )
        self.assertEqual(
            common_ekf.snapshot()["diagnostics"]["last_reset_reason"],
            "protocol_changed",
        )
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
            "position_duplicate_time", duplicate["imu_fusion_flags"]
        )

    def test_fused_event_keeps_64_bit_gptimer_timestamp(self) -> None:
        imu = imu_sample(1, 10)
        imu["fusion_time_ticks"] = (1 << 40) + 17
        imu["fusion_timer_hz"] = 10_000_000
        self.add(imu)
        self.add(position_sample(5, 1, 20))

        fused_event = self.state.tdoa_position_events[-1]
        self.assertEqual(
            fused_event["fusion_time_ticks"], (1 << 40) + 17
        )
        self.assertEqual(fused_event["fusion_timer_hz"], 10_000_000)
        self.assertIsNotNone(fused_event["sample_time_us"])

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
