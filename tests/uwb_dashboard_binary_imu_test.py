#!/usr/bin/env python3

import pathlib
import struct
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import (  # noqa: E402
    TELEMETRY_IMU_SAMPLE_LEN,
    TELEMETRY_IMU_STRUCT,
    TELEMETRY_IMU_V2_SAMPLE_LEN,
    TELEMETRY_IMU_V2_STRUCT,
    TELEMETRY_GPS_GGA_STRUCT,
    TELEMETRY_IMU_ACCEL_COMPACT_STRUCT,
    TELEMETRY_IMU_ORIENTATION_STRUCT,
    TELEMETRY_IMU_CLOCK_STRUCT,
    TELEMETRY_STREAM_BNO085_IMU,
    TELEMETRY_STREAM_BNO085_IMU_V2,
    TELEMETRY_STREAM_GPS_GGA,
    TELEMETRY_STREAM_BNO085_ACCEL_COMPACT,
    TELEMETRY_STREAM_BNO085_ORIENTATION,
    TELEMETRY_STREAM_BNO085_CLOCK,
    binary_telemetry_frame_len,
    parse_binary_telemetry_frame,
)


def frame(stream_type: int, payload: bytes) -> bytes:
    return (
        b"UWT1"
        + bytes((1, stream_type, 1, len(payload)))
        + (1).to_bytes(2, "little")
        + len(payload).to_bytes(2, "little")
        + payload
    )


def imu_values() -> tuple[int, ...]:
    return (
        1234,
        10,
        1000,
        -2000,
        9807,
        20,
        0,
        0,
        0,
        16384,
        0,
        0,
        0,
        3,
        1,
        2,
        1,
    )


class BinaryImuTelemetryTest(unittest.TestCase):
    def test_v2_preserves_gptimer_timestamp(self) -> None:
        values = imu_values()
        fusion_ticks = 12_345_678_900
        payload = TELEMETRY_IMU_V2_STRUCT.pack(
            values[0], fusion_ticks, *values[1:]
        )
        self.assertEqual(len(payload), TELEMETRY_IMU_V2_SAMPLE_LEN)
        packet = frame(TELEMETRY_STREAM_BNO085_IMU_V2, payload)

        self.assertEqual(binary_telemetry_frame_len(packet), len(packet))
        sample = parse_binary_telemetry_frame(packet)[0]

        self.assertEqual(sample["fusion_time_ticks"], fusion_ticks)
        self.assertEqual(sample["fusion_timer_hz"], 10_000_000)
        self.assertEqual(sample["fusion_time_us"], fusion_ticks // 10)
        self.assertIsNone(sample["sample_time_us"])
        self.assertEqual(sample["uptime_ms"], values[0])
        self.assertTrue(sample["imu_valid"])

    def test_legacy_imu_frame_keeps_millisecond_fallback(self) -> None:
        values = imu_values()
        payload = TELEMETRY_IMU_STRUCT.pack(*values)
        self.assertEqual(len(payload), TELEMETRY_IMU_SAMPLE_LEN)
        sample = parse_binary_telemetry_frame(
            frame(TELEMETRY_STREAM_BNO085_IMU, payload)
        )[0]

        self.assertIsNone(sample["fusion_time_ticks"])
        self.assertEqual(sample["sample_time_us"], values[0] * 1000)

    def test_compact_accel_preserves_native_q8_and_gptimer(self) -> None:
        fusion_ticks = 98_765_432_100
        payload = TELEMETRY_IMU_ACCEL_COMPACT_STRUCT.pack(
            1234, fusion_ticks, 77, 256, -512, 2511, 17, 3, 5
        )
        sample = parse_binary_telemetry_frame(
            frame(TELEMETRY_STREAM_BNO085_ACCEL_COMPACT, payload)
        )[0]
        self.assertEqual(sample["fusion_time_ticks"], fusion_ticks)
        self.assertEqual((sample["x_q8"], sample["y_q8"]), (256, -512))
        self.assertEqual(sample["x"], 1.0)
        self.assertEqual(sample["y"], -2.0)
        self.assertTrue(sample["imu_compact"])

    def test_orientation_is_a_separate_lossless_fixed_point_stream(self) -> None:
        payload = TELEMETRY_IMU_ORIENTATION_STRUCT.pack(
            1234, 50_000, 8, 1, 2, 3, 16384, 10, -20, 30, 4, 1
        )
        sample = parse_binary_telemetry_frame(
            frame(TELEMETRY_STREAM_BNO085_ORIENTATION, payload)
        )[0]
        self.assertEqual(sample["topic"], "bno085.orientation")
        self.assertEqual(sample["quat_real"], 1.0)
        self.assertEqual(sample["gyro_y"], -20 / 1024.0)

    def test_gps_gga_preserves_nanodegrees_and_measurement_clock(self) -> None:
        flags = sum(1 << bit for bit in range(11))
        payload = TELEMETRY_GPS_GGA_STRUCT.pack(
            1234,
            1_233_900,
            88,
            43_210_125,
            100826,
            44_426_800_123,
            26_102_500_987,
            92_345,
            1_250,
            91_125,
            65,
            21,
            4,
            ord("A"),
            ord("R"),
            flags,
        )
        sample = parse_binary_telemetry_frame(
            frame(TELEMETRY_STREAM_GPS_GGA, payload)
        )[0]
        self.assertEqual(sample["sample_monotonic_us"], 1_233_900)
        self.assertEqual(sample["gga_sequence"], 88)
        self.assertAlmostEqual(sample["latitude_deg"], 44.426800123)
        self.assertAlmostEqual(sample["longitude_deg"], 26.102500987)
        self.assertEqual(sample["fix_quality"], 4)
        self.assertTrue(sample["fix_valid"])

    def test_clock_anchor_carries_exact_timer_pair(self) -> None:
        payload = TELEMETRY_IMU_CLOCK_STRUCT.pack(
            1234, 12_340_005, 1_234_003
        )
        sample = parse_binary_telemetry_frame(
            frame(TELEMETRY_STREAM_BNO085_CLOCK, payload)
        )[0]
        self.assertEqual(sample["topic"], "bno085.clock")
        self.assertEqual(sample["fusion_time_ticks"], 12_340_005)
        self.assertEqual(sample["esp_timer_us"], 1_234_003)


if __name__ == "__main__":
    unittest.main()
