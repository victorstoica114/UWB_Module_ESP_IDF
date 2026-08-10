#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_imu_fusion import (  # noqa: E402
    FusionConfig,
    ImuSample,
    RawPosition,
    UwbImuFusion,
    body_accel_to_reference_linear,
    normalize_quaternion,
    rotate_body_to_reference,
)


GRAVITY = 9.80665


def imu_mapping(
    uptime_ms: int,
    *,
    accel: tuple[float, float, float] = (0.0, 0.0, GRAVITY),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    valid: bool = True,
    module_id: int | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "uptime_ms": uptime_ms,
        "x": accel[0],
        "y": accel[1],
        "z": accel[2],
        "quat_i": quaternion[0],
        "quat_j": quaternion[1],
        "quat_k": quaternion[2],
        "quat_real": quaternion[3],
        "gyro_x": gyro[0],
        "gyro_y": gyro[1],
        "gyro_z": gyro[2],
        "imu_valid": valid,
    }
    if module_id is not None:
        result["module_id"] = module_id
    return result


def raw_position(
    uptime_ms: int,
    x: float,
    y: float,
    *,
    protocol: str = "passive_ds",
    tag_id: int = 1,
    module_id: int | None = None,
    sigma_m: float | None = None,
    rms_m: float | None = None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "uptime_ms": uptime_ms,
        "protocol": protocol,
        "tag_id": tag_id,
        "raw_x": x,
        "raw_y": y,
    }
    if module_id is not None:
        result["module_id"] = module_id
    if sigma_m is not None:
        result["sigma_m"] = sigma_m
    if rms_m is not None:
        result["rms_m"] = rms_m
    return result


def align_east(
    fusion: UwbImuFusion,
    *,
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    gyro_z: float = 0.0,
    module_id: int | None = None,
) -> dict[str, object]:
    fusion.update_position(raw_position(0, 0.0, 0.0, module_id=module_id))
    fusion.update_position(raw_position(500, 0.3, 0.0, module_id=module_id))
    fusion.update_position(raw_position(1000, 0.6, 0.0, module_id=module_id))
    fusion.update_position(raw_position(1500, 0.9, 0.0, module_id=module_id))
    fusion.update_imu(
        imu_mapping(
            1990,
            quaternion=quaternion,
            gyro=(0.0, 0.0, gyro_z),
            module_id=module_id,
        )
    )
    return fusion.update_position(
        raw_position(2000, 1.2, 0.0, module_id=module_id)
    )


class QuaternionTest(unittest.TestCase):
    def test_normalizes_and_rotates_body_vector(self) -> None:
        half_sqrt = math.sqrt(0.5)
        quaternion = (0.0, 0.0, 2.0 * half_sqrt, 2.0 * half_sqrt)

        normalized = normalize_quaternion(quaternion)
        rotated = rotate_body_to_reference(normalized, (1.0, 0.0, 0.0))

        self.assertAlmostEqual(sum(value * value for value in normalized), 1.0)
        self.assertAlmostEqual(rotated[0], 0.0, places=12)
        self.assertAlmostEqual(rotated[1], 1.0, places=12)
        self.assertAlmostEqual(rotated[2], 0.0, places=12)

    def test_rotates_accel_then_removes_reference_gravity(self) -> None:
        half_sqrt = math.sqrt(0.5)
        linear = body_accel_to_reference_linear(
            (1.0, 0.0, GRAVITY),
            (0.0, 0.0, half_sqrt, half_sqrt),
        )

        self.assertAlmostEqual(linear[0], 0.0, places=12)
        self.assertAlmostEqual(linear[1], 1.0, places=12)
        self.assertAlmostEqual(linear[2], 0.0, places=12)

    def test_rejects_zero_quaternion(self) -> None:
        with self.assertRaises(ValueError):
            normalize_quaternion((0.0, 0.0, 0.0, 0.0))


class FusionTest(unittest.TestCase):
    def test_combined_mapping_preserves_body_values(self) -> None:
        sample = ImuSample.from_mapping(
            imu_mapping(
                123,
                accel=(1.0, 2.0, 3.0),
                gyro=(0.1, -0.2, 0.3),
            )
        )

        self.assertEqual(sample.uptime_ms, 123)
        self.assertEqual(sample.accel_body_mps2, (1.0, 2.0, 3.0))
        self.assertEqual(sample.gyro_body_rps, (0.1, -0.2, 0.3))
        self.assertTrue(sample.valid)

    def test_alpha_beta_position_correction_is_separate_from_raw(self) -> None:
        config = FusionConfig(alpha=0.5, beta=0.25)
        fusion = UwbImuFusion(config)
        first_raw = raw_position(0, 0.0, 0.0)
        second_raw = raw_position(1000, 2.0, 0.0)

        initialized = fusion.update_position(first_raw)
        corrected = fusion.update_position(second_raw)

        self.assertEqual(initialized["stream"], "fused")
        self.assertAlmostEqual(corrected["x"], 1.0)
        self.assertAlmostEqual(corrected["y"], 0.0)
        self.assertAlmostEqual(corrected["vx"], 0.5)
        self.assertAlmostEqual(corrected["vy"], 0.0)
        self.assertIn("position_corrected", corrected["flags"])
        self.assertEqual(second_raw["raw_x"], 2.0)
        self.assertNotIn("raw_x", corrected)

    def test_yaw_alignment_maps_body_forward_accel_to_uwb_motion(self) -> None:
        half_sqrt = math.sqrt(0.5)
        yaw_90 = (0.0, 0.0, half_sqrt, half_sqrt)
        fusion = UwbImuFusion(
            FusionConfig(
                alpha=1.0,
                beta=0.0,
                yaw_alignment_gain=1.0,
                imu_gap_reset_ms=2000,
            )
        )

        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_position(raw_position(500, 0.3, 0.0))
        fusion.update_position(raw_position(1000, 0.6, 0.0))
        fusion.update_position(raw_position(1500, 0.9, 0.0))
        fusion.update_imu(imu_mapping(1990, quaternion=yaw_90))
        aligned = fusion.update_position(raw_position(2000, 1.2, 0.0))
        propagated = fusion.update_imu(
            imu_mapping(
                2040,
                accel=(1.0, 0.0, GRAVITY),
                quaternion=yaw_90,
            )
        )

        self.assertTrue(aligned["diagnostics"]["yaw_alignment_valid"])
        self.assertAlmostEqual(
            aligned["diagnostics"]["yaw_alignment_deg"], -90.0, places=8
        )
        self.assertIn("imu_propagated", propagated["flags"])
        self.assertGreater(propagated["vx"], 0.03)
        self.assertAlmostEqual(propagated["vy"], 0.0, places=8)
        accel_uwb = propagated["diagnostics"]["last_linear_accel_uwb_mps2"]
        self.assertAlmostEqual(accel_uwb[0], 1.0, places=8)
        self.assertAlmostEqual(accel_uwb[1], 0.0, places=8)

    def test_accel_gap_over_50ms_uses_constant_velocity_only(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=1000))
        aligned = align_east(fusion)
        self.assertTrue(aligned["diagnostics"]["yaw_alignment_valid"])
        integrated = fusion.update_imu(
            imu_mapping(2040, accel=(1.0, 0.0, GRAVITY))
        )
        velocity_before_gap = integrated["vx"]

        after_gap = fusion.update_imu(
            imu_mapping(2140, accel=(10.0, 0.0, GRAVITY))
        )

        self.assertIn("imu_accel_gap_cv_only", after_gap["flags"])
        self.assertNotIn("imu_propagated", after_gap["flags"])
        self.assertAlmostEqual(after_gap["vx"], velocity_before_gap, places=12)

    def test_yaw_requires_two_seconds_and_one_meter(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=2000))
        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_imu(imu_mapping(990))

        output = fusion.update_position(raw_position(1000, 1.2, 0.0))

        self.assertFalse(output["diagnostics"]["yaw_alignment_valid"])
        self.assertEqual(output["diagnostics"]["yaw_alignment_updates"], 0)

    def test_yaw_rejects_motion_at_or_below_half_meter_per_second(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=4000))
        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_position(raw_position(750, 0.3, 0.0))
        fusion.update_position(raw_position(1500, 0.6, 0.0))
        fusion.update_position(raw_position(2250, 0.9, 0.0))
        fusion.update_imu(imu_mapping(2990))

        output = fusion.update_position(raw_position(3000, 1.2, 0.0))

        self.assertIn("yaw_reject:speed", output["flags"])
        self.assertFalse(output["diagnostics"]["yaw_alignment_valid"])
        self.assertAlmostEqual(output["diagnostics"]["last_yaw_speed_mps"], 0.4)

    def test_yaw_rejects_turning_gyro(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=3000))

        output = align_east(fusion, gyro_z=0.35)

        self.assertIn("yaw_reject:gyro_z", output["flags"])
        self.assertFalse(output["diagnostics"]["yaw_alignment_valid"])

    def test_yaw_rejects_imu_older_than_50ms(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=3000))
        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_position(raw_position(500, 0.3, 0.0))
        fusion.update_position(raw_position(1000, 0.6, 0.0))
        fusion.update_position(raw_position(1500, 0.9, 0.0))
        fusion.update_imu(imu_mapping(1949))

        output = fusion.update_position(raw_position(2000, 1.2, 0.0))

        self.assertIn("yaw_reject:stale_imu", output["flags"])
        self.assertFalse(output["diagnostics"]["yaw_alignment_valid"])

    def test_yaw_rejects_nonstraight_path(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=3000))
        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_position(raw_position(500, 0.3, 0.4))
        fusion.update_position(raw_position(1000, 0.6, -0.4))
        fusion.update_position(raw_position(1500, 0.9, 0.4))
        fusion.update_imu(imu_mapping(1990))

        output = fusion.update_position(raw_position(2000, 1.2, 0.0))

        self.assertIn("yaw_reject:not_straight", output["flags"])

    def test_yaw_rejects_uncertain_course(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                imu_gap_reset_ms=3000,
                yaw_min_straightness=0.1,
            )
        )
        fusion.update_position(raw_position(0, 0.0, 0.0))
        fusion.update_position(raw_position(500, 0.3, 0.2))
        fusion.update_position(raw_position(1000, 0.6, -0.2))
        fusion.update_position(raw_position(1500, 0.9, 0.2))
        fusion.update_imu(imu_mapping(1990))

        output = fusion.update_position(raw_position(2000, 1.2, 0.0))

        self.assertIn("yaw_reject:course_uncertainty", output["flags"])
        self.assertGreater(
            output["diagnostics"]["last_yaw_course_uncertainty_deg"], 10.0
        )

    def test_yaw_rejects_large_alignment_innovation(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                alpha=1.0,
                beta=0.0,
                imu_gap_reset_ms=3000,
                uwb_default_innovation_gate_m=20.0,
            )
        )
        first_alignment = align_east(fusion)
        self.assertTrue(first_alignment["diagnostics"]["yaw_alignment_valid"])
        cosine = math.cos(math.radians(30.0))
        sine = math.sin(math.radians(30.0))
        for index in range(1, 4):
            fraction = 0.3 * index
            fusion.update_position(
                raw_position(
                    2000 + 500 * index,
                    1.2 + fraction * cosine,
                    fraction * sine,
                )
            )
        fusion.update_imu(imu_mapping(3990))

        output = fusion.update_position(
            raw_position(4000, 1.2 + 1.2 * cosine, 1.2 * sine)
        )

        self.assertIn("yaw_reject:innovation", output["flags"])
        self.assertAlmostEqual(
            output["diagnostics"]["last_yaw_innovation_deg"], 30.0, places=6
        )
        self.assertAlmostEqual(
            output["diagnostics"]["yaw_alignment_deg"], 0.0, places=6
        )

    def test_quaternion_soft_norm_warns_but_hard_norm_rejects(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(0, 0.0, 0.0))

        soft = fusion.update_imu(
            imu_mapping(10, quaternion=(0.0, 0.0, 0.0, 0.9))
        )
        hard = fusion.update_imu(
            imu_mapping(20, quaternion=(0.0, 0.0, 0.0, 0.79))
        )

        self.assertIn("quaternion_norm_soft_warning", soft["flags"])
        self.assertIn("imu_valid", soft["flags"])
        self.assertIn("quaternion_norm_hard_reject", hard["flags"])
        self.assertIn("imu_invalid", hard["flags"])

    def test_uwb_quality_gate_rejects_position_outlier(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(
            raw_position(0, 0.0, 0.0, sigma_m=0.05, rms_m=0.02)
        )

        output = fusion.update_position(
            raw_position(1000, 1.0, 0.0, sigma_m=0.05, rms_m=0.02)
        )

        self.assertIn("uwb_outlier_rejected", output["flags"])
        self.assertAlmostEqual(output["x"], 0.0)
        self.assertAlmostEqual(
            output["diagnostics"]["last_uwb_innovation_gate_m"], 0.2
        )
        self.assertEqual(output["diagnostics"]["position_accepted"], 1)
        self.assertEqual(output["diagnostics"]["position_outliers"], 1)

    def test_uwb_rms_expands_quality_innovation_gate(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(0, 0.0, 0.0, rms_m=0.5))

        output = fusion.update_position(
            raw_position(1000, 1.0, 0.0, rms_m=0.5)
        )

        self.assertNotIn("uwb_outlier_rejected", output["flags"])
        self.assertAlmostEqual(
            output["diagnostics"]["last_uwb_innovation_gate_m"], 1.5
        )

    def test_three_consecutive_outliers_reacquire_with_zero_velocity(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                beta=0.0,
                position_reacquire_after_rejects=3,
                position_reacquire_gap_ms=10_000,
            )
        )
        fusion.update_position(raw_position(0, 0.0, 0.0, sigma_m=0.05))

        first = fusion.update_position(
            raw_position(100, 1.0, 0.0, sigma_m=0.05)
        )
        second = fusion.update_position(
            raw_position(200, 1.0, 0.0, sigma_m=0.05)
        )
        third = fusion.update_position(
            raw_position(300, 1.0, 0.0, sigma_m=0.05)
        )

        self.assertIn("uwb_outlier_rejected", first["flags"])
        self.assertNotIn("position_reacquired", first["flags"])
        self.assertNotIn("position_reacquired", second["flags"])
        self.assertIn("position_reacquired", third["flags"])
        self.assertIn("reset:position_outlier_streak", third["flags"])
        self.assertAlmostEqual(third["x"], 1.0)
        self.assertAlmostEqual(third["y"], 0.0)
        self.assertAlmostEqual(third["vx"], 0.0)
        self.assertAlmostEqual(third["vy"], 0.0)
        self.assertEqual(third["diagnostics"]["position_outliers"], 3)
        self.assertEqual(third["diagnostics"]["position_reacquisitions"], 1)
        self.assertEqual(third["diagnostics"]["consecutive_position_outliers"], 0)

    def test_outlier_reacquires_after_gap_since_last_accepted_position(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                beta=0.0,
                position_gap_reset_ms=1000,
                position_reacquire_after_rejects=100,
                position_reacquire_gap_ms=150,
            )
        )
        fusion.update_position(raw_position(0, 0.0, 0.0, sigma_m=0.05))
        first = fusion.update_position(
            raw_position(100, 1.0, 0.0, sigma_m=0.05)
        )
        reacquired = fusion.update_position(
            raw_position(150, 1.1, 0.0, sigma_m=0.05)
        )

        self.assertNotIn("position_reacquired", first["flags"])
        self.assertIn("position_reacquired", reacquired["flags"])
        self.assertIn("reset:position_acceptance_gap", reacquired["flags"])
        self.assertAlmostEqual(reacquired["x"], 1.1)
        self.assertAlmostEqual(reacquired["vx"], 0.0)

    def test_reacquisition_bounds_long_prediction_only_run(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                alpha=1.0,
                beta=1.0,
                max_velocity_mps=5.0,
                position_reacquire_after_rejects=3,
                position_reacquire_gap_ms=500,
            )
        )
        fusion.update_position(raw_position(0, 0.0, 0.0, rms_m=1.0))
        accelerated = fusion.update_position(
            raw_position(10, 0.2, 0.0, rms_m=1.0)
        )
        self.assertAlmostEqual(accelerated["vx"], 5.0)

        maximum_divergence = 0.0
        reacquisitions = 0
        for uptime_ms in range(20, 2020, 10):
            output = fusion.update_position(
                raw_position(uptime_ms, 0.0, 0.0, sigma_m=0.05)
            )
            maximum_divergence = max(maximum_divergence, abs(output["x"]))
            reacquisitions = output["diagnostics"]["position_reacquisitions"]

        self.assertGreaterEqual(reacquisitions, 1)
        self.assertLessEqual(maximum_divergence, 0.35)
        self.assertAlmostEqual(output["x"], 0.0)
        self.assertAlmostEqual(output["vx"], 0.0)

    def test_velocity_correction_is_clamped(self) -> None:
        fusion = UwbImuFusion(
            FusionConfig(
                alpha=1.0,
                beta=1.0,
                max_velocity_mps=1.0,
                uwb_default_innovation_gate_m=20.0,
            )
        )
        fusion.update_position(raw_position(0, 0.0, 0.0))

        output = fusion.update_position(raw_position(1000, 2.0, 0.0))

        self.assertIn("velocity_clamped", output["flags"])
        self.assertAlmostEqual(math.hypot(output["vx"], output["vy"]), 1.0)

    def test_module_mismatch_and_cross_stream_order_are_rejected(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(1000, 1.0, 2.0, module_id=1))

        mismatch = fusion.update_imu(imu_mapping(1010, module_id=2))
        out_of_order = fusion.update_imu(imu_mapping(900, module_id=1))

        self.assertIn("module_mismatch", mismatch["flags"])
        self.assertIn("event_out_of_order", out_of_order["flags"])
        self.assertEqual(out_of_order["module_id"], 1)
        self.assertEqual(out_of_order["diagnostics"]["module_mismatches"], 1)
        self.assertEqual(out_of_order["diagnostics"]["ordering_rejects"], 1)

    def test_duplicate_uptime_is_rejected_before_tag_switch(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(100, 1.0, 1.0, tag_id=1))

        output = fusion.update_position(raw_position(100, 9.0, 9.0, tag_id=2))

        self.assertIn("position_duplicate_uptime", output["flags"])
        self.assertNotIn("reset:tag_changed", output["flags"])
        self.assertEqual(output["tag_id"], 1)

    def test_invalid_imu_does_not_apply_acceleration(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(0, 2.0, 3.0))

        output = fusion.update_imu(
            imu_mapping(10, accel=(20.0, 0.0, GRAVITY), valid=False)
        )

        self.assertIn("imu_invalid", output["flags"])
        self.assertAlmostEqual(output["x"], 2.0)
        self.assertAlmostEqual(output["y"], 3.0)
        self.assertEqual(output["diagnostics"]["imu_invalid_samples"], 1)

    def test_protocol_change_resets_and_reinitializes(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(100, 1.0, 2.0, protocol="passive_ds"))

        output = fusion.update_position(
            raw_position(200, 8.0, 9.0, protocol="native_ds")
        )

        self.assertIn("reset:protocol_changed", output["flags"])
        self.assertIn("position_initialized", output["flags"])
        self.assertEqual(output["protocol"], "native_ds")
        self.assertAlmostEqual(output["x"], 8.0)
        self.assertAlmostEqual(output["y"], 9.0)
        self.assertEqual(output["diagnostics"]["last_reset_reason"], "protocol_changed")

    def test_tag_change_resets_track_identity(self) -> None:
        fusion = UwbImuFusion()
        fusion.update_position(raw_position(100, 1.0, 2.0, tag_id=1))

        output = fusion.update_position(raw_position(200, 3.0, 4.0, tag_id=2))

        self.assertIn("reset:tag_changed", output["flags"])
        self.assertEqual(output["tag_id"], 2)
        self.assertAlmostEqual(output["x"], 3.0)

    def test_imu_gap_resets_dynamic_state(self) -> None:
        fusion = UwbImuFusion(FusionConfig(imu_gap_reset_ms=100))
        fusion.update_position(raw_position(0, 1.0, 1.0))
        fusion.update_imu(imu_mapping(10))

        output = fusion.update_imu(imu_mapping(500))

        self.assertIn("reset:imu_gap", output["flags"])
        self.assertFalse(output["ready"])
        self.assertIsNone(output["x"])

    def test_position_gap_resets_and_uses_new_raw_origin(self) -> None:
        fusion = UwbImuFusion(FusionConfig(position_gap_reset_ms=100))
        fusion.update_position(raw_position(0, 1.0, 1.0))

        output = fusion.update_position(raw_position(500, 5.0, 6.0))

        self.assertIn("reset:position_gap", output["flags"])
        self.assertIn("position_initialized", output["flags"])
        self.assertAlmostEqual(output["x"], 5.0)
        self.assertAlmostEqual(output["y"], 6.0)

    def test_uptime_backstep_is_reported_as_reboot(self) -> None:
        fusion = UwbImuFusion(FusionConfig(reboot_backstep_ms=20))
        fusion.update_position(raw_position(1000, 1.0, 1.0))

        output = fusion.update_position(raw_position(100, 2.0, 3.0))

        self.assertIn("reset:position_uptime_reboot", output["flags"])
        self.assertIn("position_initialized", output["flags"])
        self.assertAlmostEqual(output["x"], 2.0)

    def test_dataclass_inputs_and_snapshot(self) -> None:
        fusion = UwbImuFusion()
        position = RawPosition(10, "flex_tdoa", 5, -1.0, 4.0)
        imu = ImuSample(
            uptime_ms=20,
            accel_body_mps2=(0.0, 0.0, GRAVITY),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

        fusion.update_position(position)
        fusion.update_imu(imu)
        snapshot = fusion.snapshot()

        self.assertEqual(snapshot["stream"], "fused")
        self.assertEqual(snapshot["protocol"], "flex_tdoa")
        self.assertEqual(snapshot["tag_id"], 5)
        self.assertIn("snapshot", snapshot["flags"])
        self.assertIn("not ENU-calibrated", snapshot["diagnostics"]["frame"])


if __name__ == "__main__":
    unittest.main()
