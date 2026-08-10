#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_imu_fusion import FusionConfig
from uwb_imu_replay import (
    DEG_TO_RAD,
    EARTH_RADIUS_M,
    TrackKey,
    ordered_track_events,
    raw_position_xy,
    replay_capture,
)


LAT0 = 44.0
LON0 = 26.0
GRAVITY = 9.80665


def gps_from_enu(east_m: float, north_m: float) -> tuple[float, float]:
    latitude = LAT0 + north_m / EARTH_RADIUS_M / DEG_TO_RAD
    longitude = (
        LON0
        + east_m
        / (EARTH_RADIUS_M * math.cos(LAT0 * DEG_TO_RAD))
        / DEG_TO_RAD
    )
    return latitude, longitude


def gps_record(
    module_id: int,
    wall_ns: int,
    east: float,
    north: float,
    *,
    measurement_wall_ns: int | None = None,
) -> dict:
    latitude, longitude = gps_from_enu(east, north)
    return {
        "kind": "gps_fix",
        "protocol": "flextdoa",
        "module_id": module_id,
        "collector_wall_ns": wall_ns,
        "estimated_measurement_wall_ns": (
            wall_ns if measurement_wall_ns is None else measurement_wall_ns
        ),
        "gps_fix_valid": True,
        "gps_fix_quality": 4,
        "gps_latitude_deg": latitude,
        "gps_longitude_deg": longitude,
    }


def imu_record(module_id: int, uptime_ms: int, wall_ns: int) -> dict:
    return {
        "kind": "accel",
        "protocol": "flextdoa",
        "module_id": module_id,
        "uptime_ms": uptime_ms,
        "collector_wall_ns": wall_ns,
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


def position_record(
    uptime_ms: int,
    wall_ns: int,
    x_m: float,
    y_m: float,
    *,
    received_at: float | None = None,
    sigma_m: float | None = None,
    rms_m: float | None = None,
) -> dict:
    record = {
        "kind": "position",
        "protocol": "flextdoa",
        "tdoa_protocol": "flextdoa",
        "module_id": 1,
        "tag_id": 1,
        "uptime_ms": uptime_ms,
        "collector_wall_ns": wall_ns,
        "x_m": x_m,
        "y_m": y_m,
    }
    if received_at is not None:
        record["received_at"] = received_at
    if sigma_m is not None:
        record["sigma_m"] = sigma_m
    if rms_m is not None:
        record["rms_m"] = rms_m
    return record


class ReplayTest(unittest.TestCase):
    def test_uses_raw_passive_coordinates(self) -> None:
        self.assertEqual(
            raw_position_xy(
                {
                    "raw_x_m": 1.0,
                    "raw_y_m": 2.0,
                    "x_m": 90.0,
                    "y_m": 91.0,
                }
            ),
            (1.0, 2.0),
        )

    def test_excludes_live_fused_positions_from_raw_replay_input(self) -> None:
        raw = position_record(100, 100_000_000, 1.0, 2.0)
        legacy_fused = position_record(110, 110_000_000, 90.0, 91.0)
        legacy_fused["imu_fused"] = True
        separate_fused = position_record(120, 120_000_000, 92.0, 93.0)
        separate_fused["kind"] = "position_fused"
        records = [raw, legacy_fused, separate_fused]
        for index, record in enumerate(records):
            record["_input_index"] = index

        report, samples = replay_capture(records, protocol="flextdoa")

        self.assertEqual(report["track_count"], 1)
        self.assertEqual(report["tracks"][0]["input_counts"]["positions"], 1)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["raw_x_m"], 1.0)
        self.assertEqual(samples[0]["raw_y_m"], 2.0)

    def test_routes_imu_by_module_and_scores_against_anchor_aligned_rtk(self) -> None:
        geometry_wall = 1_000_000_000
        records = [
            {
                "kind": "dashboard_snapshot",
                "protocol": "flextdoa",
                "collector_wall_ns": geometry_wall,
                "snapshot": {
                    "tdoa": {
                        "local_geometries": {
                            "flextdoa:1": {
                                "tdoa_protocol": "flextdoa",
                                "tag_id": 1,
                                "all_rtk_fixed": True,
                                "anchors": {
                                    "2": {"id": 2, "x": 0.0, "y": 0.0},
                                    "3": {"id": 3, "x": 10.0, "y": 0.0},
                                    "4": {"id": 4, "x": 0.0, "y": 10.0},
                                },
                            }
                        }
                    }
                },
            },
            gps_record(2, geometry_wall, 0.0, 0.0),
            gps_record(3, geometry_wall, 10.0, 0.0),
            gps_record(4, geometry_wall, 0.0, 10.0),
            # IMU from module 2 must never feed the tag-1/module-1 track.
            imu_record(2, 1900, 1_900_000_000),
            imu_record(1, 1900, 1_900_000_000),
            gps_record(1, 2_000_000_000, 1.0, 2.0),
            position_record(2000, 2_000_000_000, 1.0, 2.0),
            imu_record(1, 2500, 2_500_000_000),
            gps_record(1, 3_000_000_000, 2.0, 2.0),
            position_record(3000, 3_000_000_000, 2.0, 2.0),
        ]
        for index, record in enumerate(records):
            record["_input_index"] = index

        report, samples = replay_capture(
            records,
            protocol="flextdoa",
            config=FusionConfig(
                alpha=1.0,
                beta=0.0,
                imu_gap_reset_ms=2000,
                position_gap_reset_ms=5000,
            ),
            rtk_max_age_ms=10.0,
            gap_ms=900.0,
        )

        self.assertEqual(report["track_count"], 1)
        track = report["tracks"][0]
        self.assertEqual(track["module_id"], 1)
        self.assertEqual(track["input_counts"]["imu_used"], 2)
        self.assertEqual(track["input_counts"]["rtk_matches"], 2)
        self.assertTrue(track["rtk_alignment"]["available"])
        self.assertLess(track["metrics"]["raw_vs_rtk"]["rmse_m"], 1e-5)
        self.assertLess(track["metrics"]["fused_vs_rtk"]["rmse_m"], 1e-5)
        self.assertEqual(track["timing"]["raw_position"]["gap_count"], 1)
        self.assertEqual(
            track["timing"]["fused_position"]["rate_hz"],
            track["timing"]["raw_position"]["rate_hz"],
        )
        self.assertEqual(len(samples), 2)

    def test_reboot_epochs_are_detected_independently_per_event_kind(self) -> None:
        records = [
            imu_record(1, 5000, 1_000_000_000),
            position_record(1000, 1_010_000_000, 1.0, 2.0),
            imu_record(1, 5010, 1_020_000_000),
            position_record(1010, 1_030_000_000, 1.1, 2.0),
            imu_record(1, 100, 1_040_000_000),
            position_record(1020, 1_050_000_000, 1.2, 2.0),
        ]
        for index, record in enumerate(records):
            record["_input_index"] = index

        events = ordered_track_events(records, TrackKey("flextdoa", 1, 1))
        by_input = {event["input_index"]: event for event in events}

        # A backlog offset between endpoints is not a reboot.
        self.assertLess(by_input[1]["sort_ms"], 1 << 32)
        self.assertLess(by_input[3]["sort_ms"], 1 << 32)
        self.assertLess(by_input[5]["sort_ms"], 1 << 32)
        # A real backstep in the IMU stream increments only its own epoch.
        self.assertEqual(by_input[4]["sort_ms"], (1 << 32) + 100)

    def test_passes_module_and_position_quality_to_fusion(self) -> None:
        records = [
            position_record(
                100,
                100_000_000,
                3.0,
                4.0,
                sigma_m=0.08,
                rms_m=0.12,
            ),
        ]
        records[0]["_input_index"] = 0

        report, samples = replay_capture(records, protocol="flextdoa")
        diagnostics = report["tracks"][0]["fusion_final"]["diagnostics"]

        self.assertEqual(diagnostics["module_id"], 1)
        self.assertEqual(diagnostics["module_mismatches"], 0)
        self.assertAlmostEqual(diagnostics["last_position_sigma_m"], 0.08)
        self.assertAlmostEqual(diagnostics["last_position_rms_m"], 0.12)
        self.assertAlmostEqual(samples[0]["sigma_m"], 0.08)
        self.assertAlmostEqual(samples[0]["rms_m"], 0.12)

    def test_rtk_uses_position_receive_time_not_late_batch_drain_time(self) -> None:
        geometry_wall = 1_000_000_000
        tag_measurement_wall = 1_500_000_000
        records = [
            {
                "kind": "dashboard_snapshot",
                "protocol": "flextdoa",
                "collector_wall_ns": geometry_wall,
                "snapshot": {
                    "tdoa": {
                        "local_geometries": {
                            "flextdoa:1": {
                                "tdoa_protocol": "flextdoa",
                                "tag_id": 1,
                                "all_rtk_fixed": True,
                                "anchors": {
                                    "2": {"id": 2, "x": 0.0, "y": 0.0},
                                    "3": {"id": 3, "x": 10.0, "y": 0.0},
                                    "4": {"id": 4, "x": 0.0, "y": 10.0},
                                },
                            }
                        }
                    }
                },
            },
            gps_record(2, geometry_wall, 0.0, 0.0),
            gps_record(3, geometry_wall, 10.0, 0.0),
            gps_record(4, geometry_wall, 0.0, 10.0),
            gps_record(
                1,
                2_000_000_000,
                1.0,
                2.0,
                measurement_wall_ns=tag_measurement_wall,
            ),
            # The collector drains this event 500 ms after it reached the
            # dashboard.  Only received_at can associate it within 10 ms.
            position_record(
                2000,
                2_000_000_000,
                1.0,
                2.0,
                received_at=1.501,
            ),
        ]
        for index, record in enumerate(records):
            record["_input_index"] = index

        report, samples = replay_capture(
            records,
            protocol="flextdoa",
            config=FusionConfig(alpha=1.0, beta=0.0),
            rtk_max_age_ms=10.0,
        )

        self.assertEqual(report["tracks"][0]["input_counts"]["rtk_matches"], 1)
        self.assertAlmostEqual(samples[0]["rtk_time_delta_ms"], 1.0)
        self.assertEqual(samples[0]["position_wall_ns"], 1_501_000_000)
        self.assertEqual(samples[0]["collector_wall_ns"], 2_000_000_000)
        self.assertLess(samples[0]["raw_error_m"], 1e-5)

    def test_without_anchor_alignment_keeps_fusion_but_marks_rtk_unavailable(self) -> None:
        records = [
            imu_record(1, 90, 90_000_000),
            position_record(100, 100_000_000, 3.0, 4.0),
        ]
        for index, record in enumerate(records):
            record["_input_index"] = index

        report, samples = replay_capture(records, protocol="flextdoa")

        self.assertEqual(len(samples), 1)
        self.assertIsNone(samples[0]["raw_error_m"])
        self.assertFalse(report["tracks"][0]["rtk_alignment"]["available"])


if __name__ == "__main__":
    unittest.main()
