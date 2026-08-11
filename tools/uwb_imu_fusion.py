#!/usr/bin/env python3
"""Pure UWB + BNO085 IMU fusion primitives.

The module deliberately has no network, dashboard or file I/O.  One
``UwbImuFusion`` instance represents one physical IMU/tag at a time.  Raw UWB
positions are measurements only; every return value is a separate ``fused``
record.

GyroRV quaternions are interpreted as body-to-BNO-reference rotations.  The
BNO reference frame is *not* assumed to be ENU.  Its horizontal yaw alignment
to the UWB frame is learned from UWB displacement while assuming that body +X
points along the direction of travel.  That assumption is exposed in the
diagnostics rather than hidden.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _mapping_value(
    values: Mapping[str, Any], names: Sequence[str], *, required: bool = True
) -> Any:
    for name in names:
        if name in values:
            return values[name]
    if required:
        raise ValueError(f"missing required field: {names[0]}")
    return None


def wrap_angle_radians(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    """Return a finite unit quaternion in (x, y, z, w) order."""

    if len(quaternion) != 4:
        raise ValueError("quaternion must contain exactly four values")
    x, y, z, w = (_finite(value, "quaternion") for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("quaternion norm is too small")
    return x / norm, y / norm, z / norm, w / norm


def rotate_body_to_reference(
    quaternion: Sequence[float], vector_body: Sequence[float]
) -> Vector3:
    """Rotate a body-frame vector into the BNO reference frame."""

    qx, qy, qz, qw = normalize_quaternion(quaternion)
    if len(vector_body) != 3:
        raise ValueError("vector must contain exactly three values")
    vx, vy, vz = (_finite(value, "vector") for value in vector_body)

    # Expanded q * [v, 0] * conjugate(q), avoiding temporary quaternion
    # allocations in the high-rate path.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def body_accel_to_reference_linear(
    accel_body_mps2: Sequence[float],
    quaternion: Sequence[float],
    gravity_mps2: float = 9.80665,
) -> Vector3:
    """Rotate calibrated acceleration to BNO reference and remove +Z gravity."""

    gravity = _finite(gravity_mps2, "gravity_mps2")
    rotated = rotate_body_to_reference(quaternion, accel_body_mps2)
    return rotated[0], rotated[1], rotated[2] - gravity


def quaternion_yaw_radians(quaternion: Sequence[float]) -> float:
    """Return body yaw in the BNO reference frame."""

    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


@dataclass(frozen=True)
class ImuSample:
    uptime_ms: int
    accel_body_mps2: Vector3
    quaternion_xyzw: Quaternion
    gyro_body_rps: Vector3 = (0.0, 0.0, 0.0)
    valid: bool = True
    module_id: int | None = None
    sample_time_us: int | None = None
    fusion_time_ticks: int | None = None
    fusion_timer_hz: int | None = None
    fusion_uptime_offset_us: float | None = None

    def __post_init__(self) -> None:
        if self.uptime_ms < 0:
            raise ValueError("uptime_ms must be non-negative")
        if self.module_id is not None and self.module_id < 0:
            raise ValueError("module_id must be non-negative")
        if self.sample_time_us is not None and self.sample_time_us < 0:
            raise ValueError("sample_time_us must be non-negative")
        if self.fusion_time_ticks is not None and self.fusion_time_ticks < 0:
            raise ValueError("fusion_time_ticks must be non-negative")
        if self.fusion_timer_hz is not None and self.fusion_timer_hz <= 0:
            raise ValueError("fusion_timer_hz must be positive")
        if self.fusion_uptime_offset_us is not None:
            _finite(self.fusion_uptime_offset_us, "fusion_uptime_offset_us")

    @property
    def time_us(self) -> float:
        """Hardware-derived sample time, with legacy millisecond fallback."""

        if (
            self.fusion_time_ticks is not None
            and self.fusion_timer_hz is not None
            and self.fusion_uptime_offset_us is not None
        ):
            # Do not quantize the 10 MHz GPTimer to integer microseconds.
            # Reports repaired one timer tick apart are 0.1 us apart and must
            # remain distinct all the way into chronological fusion.
            return (
                self.fusion_time_ticks * 1_000_000.0 / self.fusion_timer_hz
                + self.fusion_uptime_offset_us
            )
        if self.sample_time_us is not None:
            return float(self.sample_time_us)
        if self.fusion_time_ticks is not None and self.fusion_timer_hz is not None:
            return self.fusion_time_ticks * 1_000_000.0 / self.fusion_timer_hz
        return float(self.uptime_ms * 1000)

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "ImuSample":
        uptime_ms = int(_mapping_value(values, ("uptime_ms",)))
        accel = (
            _finite(_mapping_value(values, ("x", "accel_x")), "accel_x"),
            _finite(_mapping_value(values, ("y", "accel_y")), "accel_y"),
            _finite(_mapping_value(values, ("z", "accel_z")), "accel_z"),
        )
        quaternion = (
            _finite(_mapping_value(values, ("quat_i", "quat_x")), "quat_i"),
            _finite(_mapping_value(values, ("quat_j", "quat_y")), "quat_j"),
            _finite(_mapping_value(values, ("quat_k", "quat_z")), "quat_k"),
            _finite(
                _mapping_value(values, ("quat_real", "quat_w")),
                "quat_real",
            ),
        )
        gyro = (
            _finite(
                _mapping_value(values, ("gyro_x",), required=False) or 0.0,
                "gyro_x",
            ),
            _finite(
                _mapping_value(values, ("gyro_y",), required=False) or 0.0,
                "gyro_y",
            ),
            _finite(
                _mapping_value(values, ("gyro_z",), required=False) or 0.0,
                "gyro_z",
            ),
        )
        valid_value = _mapping_value(
            values, ("imu_valid", "valid"), required=False
        )
        module_value = _mapping_value(values, ("module_id",), required=False)
        sample_time_value = _mapping_value(
            values, ("sample_time_us",), required=False
        )
        fusion_ticks_value = _mapping_value(
            values, ("fusion_time_ticks",), required=False
        )
        fusion_timer_hz_value = _mapping_value(
            values, ("fusion_timer_hz",), required=False
        )
        fusion_uptime_offset_value = _mapping_value(
            values, ("fusion_uptime_offset_us",), required=False
        )
        return cls(
            uptime_ms=uptime_ms,
            accel_body_mps2=accel,
            quaternion_xyzw=quaternion,
            gyro_body_rps=gyro,
            valid=bool(valid_value) if valid_value is not None else True,
            module_id=int(module_value) if module_value is not None else None,
            sample_time_us=(
                int(sample_time_value) if sample_time_value is not None else None
            ),
            fusion_time_ticks=(
                int(fusion_ticks_value) if fusion_ticks_value is not None else None
            ),
            fusion_timer_hz=(
                int(fusion_timer_hz_value)
                if fusion_timer_hz_value is not None
                else None
            ),
            fusion_uptime_offset_us=(
                float(fusion_uptime_offset_value)
                if fusion_uptime_offset_value is not None
                else None
            ),
        )


@dataclass(frozen=True)
class RawPosition:
    uptime_ms: int
    protocol: str
    tag_id: int
    x_m: float
    y_m: float
    module_id: int | None = None
    sigma_m: float | None = None
    rms_m: float | None = None
    sample_time_us: int | None = None

    def __post_init__(self) -> None:
        if self.uptime_ms < 0:
            raise ValueError("uptime_ms must be non-negative")
        protocol = str(self.protocol).strip().lower()
        if not protocol:
            raise ValueError("protocol must not be empty")
        object.__setattr__(self, "protocol", protocol)
        if self.tag_id < 0:
            raise ValueError("tag_id must be non-negative")
        if self.module_id is not None and self.module_id < 0:
            raise ValueError("module_id must be non-negative")
        if self.sample_time_us is not None and self.sample_time_us < 0:
            raise ValueError("sample_time_us must be non-negative")
        _finite(self.x_m, "raw_x")
        _finite(self.y_m, "raw_y")
        for name, value in (("sigma_m", self.sigma_m), ("rms_m", self.rms_m)):
            if value is not None and _finite(value, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")

    @property
    def time_us(self) -> int:
        return (
            self.sample_time_us
            if self.sample_time_us is not None
            else self.uptime_ms * 1000
        )

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "RawPosition":
        protocol = str(_mapping_value(values, ("protocol", "tdoa_protocol")))
        if not protocol.strip():
            raise ValueError("protocol must not be empty")
        module_value = _mapping_value(values, ("module_id",), required=False)
        sigma_value = _mapping_value(
            values, ("sigma_m", "position_sigma_m"), required=False
        )
        rms_value = _mapping_value(
            values, ("rms_m", "equation_rms_m"), required=False
        )
        sample_time_value = _mapping_value(
            values, ("sample_time_us",), required=False
        )
        sigma_m = _finite(sigma_value, "sigma_m") if sigma_value is not None else None
        rms_m = _finite(rms_value, "rms_m") if rms_value is not None else None
        if sigma_m is not None and sigma_m < 0.0:
            raise ValueError("sigma_m must be non-negative")
        if rms_m is not None and rms_m < 0.0:
            raise ValueError("rms_m must be non-negative")
        return cls(
            uptime_ms=int(_mapping_value(values, ("uptime_ms",))),
            protocol=protocol.strip().lower(),
            tag_id=int(_mapping_value(values, ("tag_id", "tag"))),
            x_m=_finite(
                _mapping_value(values, ("raw_x_m", "raw_x", "x_m", "x")),
                "raw_x",
            ),
            y_m=_finite(
                _mapping_value(values, ("raw_y_m", "raw_y", "y_m", "y")),
                "raw_y",
            ),
            module_id=int(module_value) if module_value is not None else None,
            sigma_m=sigma_m,
            rms_m=rms_m,
            sample_time_us=(
                int(sample_time_value) if sample_time_value is not None else None
            ),
        )


@dataclass(frozen=True)
class FusionConfig:
    gravity_mps2: float = 9.80665
    alpha: float = 0.35
    beta: float = 0.08
    yaw_alignment_gain: float = 0.25
    yaw_min_displacement_m: float = 1.0
    yaw_min_window_ms: int = 2000
    yaw_max_window_ms: int = 5000
    yaw_min_speed_mps: float = 0.5
    yaw_max_imu_age_ms: int = 50
    yaw_max_abs_gyro_z_rps: float = 0.35
    yaw_min_straightness: float = 0.95
    yaw_max_course_uncertainty_deg: float = 10.0
    yaw_max_innovation_deg: float = 20.0
    yaw_alignment_max_age_ms: int = 10000
    yaw_min_segment_m: float = 0.02
    max_accel_integration_gap_ms: int = 50
    imu_gap_reset_ms: int = 300
    position_gap_reset_ms: int = 2500
    reboot_backstep_ms: int = 50
    cross_stream_reorder_tolerance_ms: int = 50
    min_velocity_correction_dt_s: float = 0.005
    max_horizontal_accel_mps2: float = 15.0
    high_dynamic_gyro_threshold_rps: float = 2.0
    high_dynamic_cooldown_ms: int = 500
    accel_prediction_max_uwb_age_ms: int = 150
    quaternion_soft_norm_min: float = 0.95
    quaternion_soft_norm_max: float = 1.05
    quaternion_hard_norm_min: float = 0.8
    quaternion_hard_norm_max: float = 1.2
    uwb_default_innovation_gate_m: float = 2.0
    uwb_min_innovation_gate_m: float = 0.20
    uwb_sigma_gate_multiplier: float = 4.0
    uwb_rms_gate_multiplier: float = 3.0
    max_velocity_mps: float = 5.0
    position_reacquire_after_rejects: int = 2
    position_reacquire_gap_ms: int = 500
    position_reacquire_max_speed_mps: float = 8.0
    position_reacquire_min_step_gate_m: float = 0.12
    position_reacquire_quality_multiplier: float = 2.0
    position_reacquire_probation_positions: int = 3
    process_accel_noise_mps2: float = 2.5
    initial_velocity_std_mps: float = 1.0
    uwb_default_std_m: float = 0.25
    uwb_min_std_m: float = 0.02
    uwb_measurement_std_scale: float = 0.6
    uwb_nis_gate: float = 13.8155
    stationary_accel_threshold_mps2: float = 0.35
    stationary_gyro_threshold_rps: float = 0.12
    stationary_max_speed_mps: float = 0.20
    stationary_min_duration_ms: int = 400
    stationary_uwb_window_ms: int = 600
    stationary_uwb_extent_m: float = 0.12
    stationary_bias_gain: float = 0.02
    zupt_velocity_std_mps: float = 0.03

    def __post_init__(self) -> None:
        if not (0.0 < self.alpha <= 1.0):
            raise ValueError("alpha must be in (0, 1]")
        if not (0.0 <= self.beta <= 1.0):
            raise ValueError("beta must be in [0, 1]")
        if not (0.0 < self.yaw_alignment_gain <= 1.0):
            raise ValueError("yaw_alignment_gain must be in (0, 1]")
        if self.yaw_min_displacement_m <= 0.0:
            raise ValueError("yaw_min_displacement_m must be positive")
        if self.yaw_min_window_ms <= 0 or self.yaw_max_window_ms < self.yaw_min_window_ms:
            raise ValueError("yaw window thresholds are invalid")
        if self.yaw_min_speed_mps <= 0.0:
            raise ValueError("yaw_min_speed_mps must be positive")
        if self.yaw_max_imu_age_ms < 0 or self.yaw_max_abs_gyro_z_rps <= 0.0:
            raise ValueError("yaw IMU gates are invalid")
        if not (0.0 < self.yaw_min_straightness <= 1.0):
            raise ValueError("yaw_min_straightness must be in (0, 1]")
        if self.yaw_max_course_uncertainty_deg <= 0.0:
            raise ValueError("yaw_max_course_uncertainty_deg must be positive")
        if not (0.0 < self.yaw_max_innovation_deg < 180.0):
            raise ValueError("yaw_max_innovation_deg must be in (0, 180)")
        if self.yaw_alignment_max_age_ms <= 0:
            raise ValueError("yaw_alignment_max_age_ms must be positive")
        if self.max_accel_integration_gap_ms <= 0:
            raise ValueError("max_accel_integration_gap_ms must be positive")
        if self.imu_gap_reset_ms <= 0 or self.position_gap_reset_ms <= 0:
            raise ValueError("gap reset thresholds must be positive")
        if self.imu_gap_reset_ms < self.max_accel_integration_gap_ms:
            raise ValueError("imu_gap_reset_ms must cover the acceleration gap")
        if self.reboot_backstep_ms < 0:
            raise ValueError("reboot_backstep_ms must be non-negative")
        if self.cross_stream_reorder_tolerance_ms < 0:
            raise ValueError("cross-stream reorder tolerance must be non-negative")
        if self.max_horizontal_accel_mps2 <= 0.0:
            raise ValueError("max_horizontal_accel_mps2 must be positive")
        if (
            self.high_dynamic_gyro_threshold_rps <= 0.0
            or self.high_dynamic_cooldown_ms <= 0
            or self.accel_prediction_max_uwb_age_ms <= 0
        ):
            raise ValueError("high-dynamic thresholds are invalid")
        if not (
            0.0 < self.quaternion_hard_norm_min
            <= self.quaternion_soft_norm_min
            <= self.quaternion_soft_norm_max
            <= self.quaternion_hard_norm_max
        ):
            raise ValueError("quaternion norm thresholds are invalid")
        if self.uwb_default_innovation_gate_m <= 0.0:
            raise ValueError("default UWB innovation gate must be positive")
        if self.uwb_min_innovation_gate_m <= 0.0:
            raise ValueError("minimum UWB innovation gate must be positive")
        if self.uwb_sigma_gate_multiplier <= 0.0 or self.uwb_rms_gate_multiplier <= 0.0:
            raise ValueError("UWB quality multipliers must be positive")
        if self.max_velocity_mps <= 0.0:
            raise ValueError("max_velocity_mps must be positive")
        if self.position_reacquire_after_rejects <= 0:
            raise ValueError("position_reacquire_after_rejects must be positive")
        if self.position_reacquire_gap_ms <= 0:
            raise ValueError("position_reacquire_gap_ms must be positive")
        if (
            self.position_reacquire_max_speed_mps <= 0.0
            or self.position_reacquire_min_step_gate_m <= 0.0
            or self.position_reacquire_quality_multiplier <= 0.0
            or self.position_reacquire_probation_positions < 0
        ):
            raise ValueError("position reacquisition thresholds are invalid")
        if self.process_accel_noise_mps2 <= 0.0:
            raise ValueError("process_accel_noise_mps2 must be positive")
        if self.initial_velocity_std_mps <= 0.0:
            raise ValueError("initial_velocity_std_mps must be positive")
        if not (0.0 < self.uwb_min_std_m <= self.uwb_default_std_m):
            raise ValueError("UWB standard deviations are invalid")
        if self.uwb_measurement_std_scale <= 0.0:
            raise ValueError("uwb_measurement_std_scale must be positive")
        if self.uwb_nis_gate <= 0.0:
            raise ValueError("uwb_nis_gate must be positive")
        if (
            self.stationary_accel_threshold_mps2 <= 0.0
            or self.stationary_gyro_threshold_rps <= 0.0
            or self.stationary_max_speed_mps <= 0.0
            or self.stationary_min_duration_ms <= 0
            or self.stationary_uwb_window_ms < self.stationary_min_duration_ms
            or self.stationary_uwb_extent_m <= 0.0
        ):
            raise ValueError("stationary thresholds are invalid")
        if not (0.0 < self.stationary_bias_gain <= 1.0):
            raise ValueError("stationary_bias_gain must be in (0, 1]")
        if self.zupt_velocity_std_mps <= 0.0:
            raise ValueError("zupt_velocity_std_mps must be positive")


class UwbImuFusion:
    """Small alpha-beta position tracker with IMU prediction.

    ``update_imu`` accepts a combined telemetry mapping or ``ImuSample``.
    ``update_position`` accepts only a raw UWB position mapping or
    ``RawPosition``.  Both methods return a new JSON-compatible record whose
    ``stream`` is ``"fused"``; the input mapping is never changed.
    """

    def __init__(self, config: FusionConfig | None = None) -> None:
        self.config = config or FusionConfig()

        self._ready = False
        self._module_id: int | None = None
        self._protocol: str | None = None
        self._tag_id: int | None = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._vx_mps = 0.0
        self._vy_mps = 0.0
        self._covariance = [[0.0 for _ in range(4)] for _ in range(4)]
        self._state_time_us: int | None = None

        self._last_imu_time_us: int | None = None
        self._last_position_time_us: int | None = None
        self._last_accepted_position_time_us: int | None = None
        self._last_event_time_us: int | None = None
        self._last_output_uptime_ms: int | None = None
        self._last_fusion_time_ticks: int | None = None
        self._last_fusion_timer_hz: int | None = None
        self._last_imu_yaw_ref_rad: float | None = None
        self._last_quaternion_norm: float | None = None
        self._last_linear_ref_mps2: Vector3 | None = None
        self._last_linear_uwb_mps2: Vector3 | None = None
        self._last_gyro_body_rps: Vector3 | None = None
        self._last_imu_dt_s: float | None = None
        self._last_position_dt_s: float | None = None

        self._yaw_alignment_valid = False
        self._yaw_alignment_rad = 0.0
        self._yaw_alignment_updates = 0
        self._yaw_alignment_source: str | None = None
        self._last_yaw_alignment_time_us: int | None = None
        self._uwb_to_enu_yaw_valid = False
        self._uwb_to_enu_yaw_rad = 0.0
        self._uwb_to_enu_yaw_source: str | None = None
        self._yaw_window: list[tuple[int, float, float]] = []
        self._last_yaw_window_duration_s: float | None = None
        self._last_yaw_displacement_m: float | None = None
        self._last_yaw_speed_mps: float | None = None
        self._last_yaw_straightness: float | None = None
        self._last_yaw_course_uncertainty_deg: float | None = None
        self._last_yaw_innovation_deg: float | None = None
        self._last_yaw_reject_reason: str | None = None

        self._last_position_residual_m: float | None = None
        self._last_uwb_innovation_gate_m: float | None = None
        self._last_position_sigma_m: float | None = None
        self._last_position_rms_m: float | None = None
        self._last_position_measurement_std_m: float | None = None
        self._last_position_nis: float | None = None
        self._accel_bias_ref_mps2: Vector3 = (0.0, 0.0, 0.0)
        self._accel_bias_valid = False
        self._stationary_candidate_since_us: int | None = None
        self._stationary_candidate_sum: list[float] = [0.0, 0.0, 0.0]
        self._stationary_candidate_count = 0
        self._stationary = False
        self._stationary_sample_count = 0
        self._raw_position_window: list[tuple[int, float, float]] = []
        self._last_raw_position: RawPosition | None = None
        self._rejected_position_cluster: list[RawPosition] = []
        self._reacquire_probation_remaining = 0
        self._high_dynamic_until_us: int | None = None
        self._bias_update_count = 0
        self._zupt_count = 0
        self._last_reset_reason: str | None = None
        self._reset_count = 0
        self._imu_sample_count = 0
        self._imu_invalid_count = 0
        self._position_sample_count = 0
        self._position_accepted_count = 0
        self._position_correction_count = 0
        self._position_outlier_count = 0
        self._consecutive_position_outliers = 0
        self._position_reacquisition_count = 0
        self._position_soft_reacquisition_count = 0
        self._position_same_time_count = 0
        self._velocity_clamp_count = 0
        self._ordering_reject_count = 0
        self._ordering_reject_reasons: dict[str, int] = {}
        self._last_ordering_reject_reason: str | None = None
        self._last_ordering_lag_us: int | None = None
        self._max_ordering_lag_us = 0
        self._module_mismatch_count = 0

    def reset(self, reason: str = "manual") -> dict[str, Any]:
        """Reset dynamic state while preserving cumulative diagnostics."""

        flags = self._reset_state(reason)
        return self._output(None, flags)

    def _reset_state(self, reason: str) -> list[str]:
        self._ready = False
        self._protocol = None
        self._tag_id = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._vx_mps = 0.0
        self._vy_mps = 0.0
        self._covariance = [[0.0 for _ in range(4)] for _ in range(4)]
        self._state_time_us = None
        self._last_imu_time_us = None
        self._last_position_time_us = None
        self._last_accepted_position_time_us = None
        self._last_event_time_us = None
        self._last_output_uptime_ms = None
        self._last_fusion_time_ticks = None
        self._last_fusion_timer_hz = None
        self._last_imu_yaw_ref_rad = None
        self._last_quaternion_norm = None
        self._last_linear_ref_mps2 = None
        self._last_linear_uwb_mps2 = None
        self._last_gyro_body_rps = None
        self._last_imu_dt_s = None
        self._last_position_dt_s = None
        self._yaw_alignment_valid = False
        self._yaw_alignment_rad = 0.0
        self._yaw_alignment_updates = 0
        self._yaw_alignment_source = None
        self._last_yaw_alignment_time_us = None
        self._uwb_to_enu_yaw_valid = False
        self._uwb_to_enu_yaw_rad = 0.0
        self._uwb_to_enu_yaw_source = None
        self._yaw_window = []
        self._last_yaw_window_duration_s = None
        self._last_yaw_displacement_m = None
        self._last_yaw_speed_mps = None
        self._last_yaw_straightness = None
        self._last_yaw_course_uncertainty_deg = None
        self._last_yaw_innovation_deg = None
        self._last_yaw_reject_reason = None
        self._last_position_residual_m = None
        self._last_uwb_innovation_gate_m = None
        self._last_position_sigma_m = None
        self._last_position_rms_m = None
        self._last_position_measurement_std_m = None
        self._last_position_nis = None
        self._accel_bias_ref_mps2 = (0.0, 0.0, 0.0)
        self._accel_bias_valid = False
        self._stationary_candidate_since_us = None
        self._stationary_candidate_sum = [0.0, 0.0, 0.0]
        self._stationary_candidate_count = 0
        self._stationary = False
        self._stationary_sample_count = 0
        self._raw_position_window = []
        self._last_raw_position = None
        self._rejected_position_cluster = []
        self._reacquire_probation_remaining = 0
        self._high_dynamic_until_us = None
        self._consecutive_position_outliers = 0
        self._last_reset_reason = str(reason)
        self._reset_count += 1
        return ["reset", f"reset:{reason}"]

    def _verify_module(
        self, module_id: int | None, flags: list[str]
    ) -> bool:
        if module_id is None:
            flags.append("module_id_missing")
            return True
        if module_id < 0:
            self._module_mismatch_count += 1
            flags.append("module_id_invalid")
            return False
        if self._module_id is None:
            self._module_id = module_id
            flags.append("module_bound")
            return True
        if module_id != self._module_id:
            self._module_mismatch_count += 1
            flags.append("module_mismatch")
            return False
        return True

    def _record_ordering_reject(self, reason: str, lag_us: int) -> None:
        self._ordering_reject_count += 1
        self._ordering_reject_reasons[reason] = (
            self._ordering_reject_reasons.get(reason, 0) + 1
        )
        self._last_ordering_reject_reason = reason
        self._last_ordering_lag_us = max(0, int(lag_us))
        self._max_ordering_lag_us = max(
            self._max_ordering_lag_us, self._last_ordering_lag_us
        )

    def _verify_cross_stream_order(
        self, time_us: int, flags: list[str], source: str
    ) -> bool:
        tolerance_us = self.config.cross_stream_reorder_tolerance_ms * 1000
        if (
            self._last_event_time_us is not None
            and time_us < self._last_event_time_us - tolerance_us
        ):
            self._record_ordering_reject(
                f"{source}_cross_stream",
                self._last_event_time_us - time_us,
            )
            flags.append("event_out_of_order")
            return False
        if self._last_event_time_us is None or time_us > self._last_event_time_us:
            self._last_event_time_us = time_us
        return True

    def _clamp_velocity(self, flags: list[str]) -> None:
        speed = math.hypot(self._vx_mps, self._vy_mps)
        if speed <= self.config.max_velocity_mps:
            return
        scale = self.config.max_velocity_mps / speed
        self._vx_mps *= scale
        self._vy_mps *= scale
        self._velocity_clamp_count += 1
        flags.append("velocity_clamped")

    def _uwb_innovation_gate(self, position: RawPosition) -> float:
        quality_gates: list[float] = []
        if position.sigma_m is not None and position.sigma_m > 0.0:
            quality_gates.append(
                self.config.uwb_sigma_gate_multiplier * position.sigma_m
            )
        if position.rms_m is not None and position.rms_m > 0.0:
            quality_gates.append(
                self.config.uwb_rms_gate_multiplier * position.rms_m
            )
        if quality_gates:
            return max(self.config.uwb_min_innovation_gate_m, *quality_gates)
        return max(
            self.config.uwb_min_innovation_gate_m,
            self.config.uwb_default_innovation_gate_m,
        )

    def _uwb_measurement_std(self, position: RawPosition) -> float:
        candidates = [
            value
            for value in (position.sigma_m, position.rms_m)
            if value is not None and value > 0.0
        ]
        estimate = max(candidates) if candidates else self.config.uwb_default_std_m
        estimate *= self.config.uwb_measurement_std_scale
        return max(self.config.uwb_min_std_m, estimate)

    def _state_vector(self) -> list[float]:
        return [self._x_m, self._y_m, self._vx_mps, self._vy_mps]

    def _set_state_vector(self, state: Sequence[float]) -> None:
        self._x_m, self._y_m, self._vx_mps, self._vy_mps = (
            float(state[0]),
            float(state[1]),
            float(state[2]),
            float(state[3]),
        )

    def _kalman_update_pair(
        self,
        indices: tuple[int, int],
        residual: tuple[float, float],
        variance: float,
        *,
        apply: bool,
    ) -> float:
        first, second = indices
        covariance = self._covariance
        s00 = covariance[first][first] + variance
        s01 = covariance[first][second]
        s10 = covariance[second][first]
        s11 = covariance[second][second] + variance
        determinant = s00 * s11 - s01 * s10
        if determinant <= 1e-18:
            return math.inf
        inv00 = s11 / determinant
        inv01 = -s01 / determinant
        inv10 = -s10 / determinant
        inv11 = s00 / determinant
        rx, ry = residual
        nis = rx * (inv00 * rx + inv01 * ry) + ry * (
            inv10 * rx + inv11 * ry
        )
        if not apply:
            return nis

        gain = [[0.0, 0.0] for _ in range(4)]
        for row in range(4):
            p0 = covariance[row][first]
            p1 = covariance[row][second]
            gain[row][0] = p0 * inv00 + p1 * inv10
            gain[row][1] = p0 * inv01 + p1 * inv11

        state = self._state_vector()
        for row in range(4):
            state[row] += gain[row][0] * rx + gain[row][1] * ry
        self._set_state_vector(state)

        identity_minus_kh = [
            [1.0 if row == column else 0.0 for column in range(4)]
            for row in range(4)
        ]
        for row in range(4):
            identity_minus_kh[row][first] -= gain[row][0]
            identity_minus_kh[row][second] -= gain[row][1]
        left = [
            [
                sum(identity_minus_kh[row][k] * covariance[k][column] for k in range(4))
                for column in range(4)
            ]
            for row in range(4)
        ]
        joseph = [
            [
                sum(left[row][k] * identity_minus_kh[column][k] for k in range(4))
                + variance
                * (
                    gain[row][0] * gain[column][0]
                    + gain[row][1] * gain[column][1]
                )
                for column in range(4)
            ]
            for row in range(4)
        ]
        self._covariance = [
            [
                max(0.0, joseph[row][column])
                if row == column
                else 0.5 * (joseph[row][column] + joseph[column][row])
                for column in range(4)
            ]
            for row in range(4)
        ]
        return nis

    def _zero_velocity_update(self, flags: list[str]) -> None:
        variance = self.config.zupt_velocity_std_mps**2
        self._kalman_update_pair(
            (2, 3), (-self._vx_mps, -self._vy_mps), variance, apply=True
        )
        self._zupt_count += 1
        flags.append("zero_velocity_update")

    def _append_raw_position(self, position: RawPosition) -> None:
        point = (position.time_us, position.x_m, position.y_m)
        self._raw_position_window.append(point)
        cutoff_us = position.time_us - self.config.stationary_uwb_window_ms * 1000
        while (
            len(self._raw_position_window) > 1
            and self._raw_position_window[0][0] < cutoff_us
        ):
            self._raw_position_window.pop(0)

    def _uwb_confirms_stationary(self, time_us: int) -> bool:
        if len(self._raw_position_window) < 2:
            return False
        first_time_us = self._raw_position_window[0][0]
        latest_time_us = self._raw_position_window[-1][0]
        if time_us - latest_time_us > self.config.stationary_uwb_window_ms * 1000:
            return False
        if (
            latest_time_us - first_time_us
            < self.config.stationary_min_duration_ms * 1000
        ):
            return False
        xs = [point[1] for point in self._raw_position_window]
        ys = [point[2] for point in self._raw_position_window]
        return (
            max(xs) - min(xs) <= self.config.stationary_uwb_extent_m
            and max(ys) - min(ys) <= self.config.stationary_uwb_extent_m
        )

    @staticmethod
    def _position_quality_m(position: RawPosition) -> float:
        candidates = [
            value
            for value in (position.sigma_m, position.rms_m)
            if value is not None and value > 0.0
        ]
        return max(candidates) if candidates else 0.0

    def _rejected_cluster_is_coherent(self) -> bool:
        required = self.config.position_reacquire_after_rejects
        if len(self._rejected_position_cluster) < required:
            return False
        previous = self._rejected_position_cluster[-2]
        current = self._rejected_position_cluster[-1]
        delta_us = current.time_us - previous.time_us
        if delta_us <= 0:
            return False
        step_m = math.hypot(
            current.x_m - previous.x_m,
            current.y_m - previous.y_m,
        )
        quality_gate_m = self.config.position_reacquire_quality_multiplier * max(
            self._position_quality_m(previous),
            self._position_quality_m(current),
        )
        step_gate_m = max(
            self.config.position_reacquire_min_step_gate_m,
            quality_gate_m,
        )
        implied_speed_mps = step_m / (delta_us / 1_000_000.0)
        return (
            step_m <= step_gate_m
            and implied_speed_mps <= self.config.position_reacquire_max_speed_mps
        )

    def _soft_reacquire_position(
        self, position: RawPosition, flags: list[str]
    ) -> None:
        """Re-anchor kinematics without discarding IMU calibration state."""

        measurement_std_m = self._uwb_measurement_std(position)
        position_variance = measurement_std_m**2
        velocity_variance = self.config.initial_velocity_std_mps**2
        self._x_m = position.x_m
        self._y_m = position.y_m
        # Two samples are enough to establish a coherent displaced cluster,
        # but not a low-noise velocity at 20--100 Hz.  Re-anchor position and
        # let subsequent accepted measurements rebuild velocity safely.
        self._vx_mps = 0.0
        self._vy_mps = 0.0
        self._covariance = [
            [position_variance, 0.0, 0.0, 0.0],
            [0.0, position_variance, 0.0, 0.0],
            [0.0, 0.0, velocity_variance, 0.0],
            [0.0, 0.0, 0.0, velocity_variance],
        ]
        self._state_time_us = position.time_us
        self._last_accepted_position_time_us = position.time_us
        self._last_position_measurement_std_m = measurement_std_m
        self._last_position_residual_m = 0.0
        self._last_position_nis = 0.0
        self._consecutive_position_outliers = 0
        self._rejected_position_cluster = []
        self._reacquire_probation_remaining = (
            self.config.position_reacquire_probation_positions
        )
        # Preserve the learned yaw offset and accelerometer bias, but restart
        # the motion-derived yaw window at the new coherent location.
        self._yaw_window = [(position.time_us, position.x_m, position.y_m)]
        self._position_accepted_count += 1
        self._position_reacquisition_count += 1
        self._position_soft_reacquisition_count += 1
        flags.extend(("position_reacquired", "position_soft_reacquired"))

    def _stationary_corrected_accel(
        self,
        time_us: int,
        linear_ref: Vector3,
        gyro: Vector3,
        flags: list[str],
    ) -> Vector3:
        corrected_before = tuple(
            linear_ref[index] - self._accel_bias_ref_mps2[index]
            for index in range(3)
        )
        accel_magnitude = math.sqrt(sum(value * value for value in corrected_before))
        gyro_magnitude = math.sqrt(sum(value * value for value in gyro))
        stationary_candidate = (
            accel_magnitude <= self.config.stationary_accel_threshold_mps2
            and gyro_magnitude <= self.config.stationary_gyro_threshold_rps
            and math.hypot(self._vx_mps, self._vy_mps)
            <= self.config.stationary_max_speed_mps
            and self._uwb_confirms_stationary(time_us)
        )
        if not stationary_candidate:
            self._stationary_candidate_since_us = None
            self._stationary_candidate_sum = [0.0, 0.0, 0.0]
            self._stationary_candidate_count = 0
            self._stationary = False
            flags.append("motion_detected")
            return corrected_before  # type: ignore[return-value]

        if self._stationary_candidate_since_us is None:
            self._stationary_candidate_since_us = time_us
            self._stationary_candidate_sum = list(linear_ref)
            self._stationary_candidate_count = 1
        else:
            for index in range(3):
                self._stationary_candidate_sum[index] += linear_ref[index]
            self._stationary_candidate_count += 1
        duration_us = time_us - self._stationary_candidate_since_us
        if duration_us < self.config.stationary_min_duration_ms * 1000:
            flags.append("stationary_pending")
            return corrected_before  # type: ignore[return-value]

        if not self._stationary:
            count = max(1, self._stationary_candidate_count)
            self._accel_bias_ref_mps2 = tuple(
                value / count for value in self._stationary_candidate_sum
            )  # type: ignore[assignment]
            self._accel_bias_valid = True
            self._stationary = True
            flags.append("stationary_initialized")
        else:
            gain = self.config.stationary_bias_gain
            self._accel_bias_ref_mps2 = tuple(
                (1.0 - gain) * self._accel_bias_ref_mps2[index]
                + gain * linear_ref[index]
                for index in range(3)
            )  # type: ignore[assignment]
        self._stationary_sample_count += 1
        self._bias_update_count += 1
        flags.extend(("stationary", "accel_bias_updated"))
        return (0.0, 0.0, 0.0)

    def _initialize_position(
        self, position: RawPosition, flags: list[str]
    ) -> None:
        """Initialize a bounded state from an authoritative raw position."""

        self._protocol = position.protocol
        self._tag_id = position.tag_id
        self._last_position_time_us = position.time_us
        self._last_position_sigma_m = position.sigma_m
        self._last_position_rms_m = position.rms_m
        self._last_position_measurement_std_m = self._uwb_measurement_std(position)
        self._last_uwb_innovation_gate_m = self._uwb_innovation_gate(position)
        self._x_m = position.x_m
        self._y_m = position.y_m
        self._vx_mps = 0.0
        self._vy_mps = 0.0
        position_variance = self._last_position_measurement_std_m**2
        velocity_variance = self.config.initial_velocity_std_mps**2
        self._covariance = [
            [position_variance, 0.0, 0.0, 0.0],
            [0.0, position_variance, 0.0, 0.0],
            [0.0, 0.0, velocity_variance, 0.0],
            [0.0, 0.0, 0.0, velocity_variance],
        ]
        self._state_time_us = position.time_us
        self._last_event_time_us = position.time_us
        self._last_output_uptime_ms = position.uptime_ms
        self._ready = True
        self._last_accepted_position_time_us = position.time_us
        self._last_position_dt_s = None
        self._last_position_residual_m = 0.0
        self._last_position_nis = 0.0
        self._consecutive_position_outliers = 0
        self._rejected_position_cluster = []
        self._reacquire_probation_remaining = 0
        self._position_accepted_count += 1
        self._update_yaw_alignment(position, flags)
        flags.append("position_initialized")

    def _predict_to(
        self,
        time_us: int,
        acceleration_uwb_mps2: tuple[float, float] | None,
    ) -> bool:
        if not self._ready:
            return False
        if self._state_time_us is None:
            self._state_time_us = time_us
            return False
        delta_us = time_us - self._state_time_us
        if delta_us <= 0:
            return False

        dt = delta_us / 1_000_000.0
        ax, ay = acceleration_uwb_mps2 or (0.0, 0.0)
        self._x_m += self._vx_mps * dt + 0.5 * ax * dt * dt
        self._y_m += self._vy_mps * dt + 0.5 * ay * dt * dt
        self._vx_mps += ax * dt
        self._vy_mps += ay * dt
        transition = [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        covariance = self._covariance
        left = [
            [
                sum(transition[row][k] * covariance[k][column] for k in range(4))
                for column in range(4)
            ]
            for row in range(4)
        ]
        predicted = [
            [
                sum(left[row][k] * transition[column][k] for k in range(4))
                for column in range(4)
            ]
            for row in range(4)
        ]
        accel_variance = self.config.process_accel_noise_mps2**2
        half_dt_squared = 0.5 * dt * dt
        noise_vectors = (
            (half_dt_squared, 0.0, dt, 0.0),
            (0.0, half_dt_squared, 0.0, dt),
        )
        for noise in noise_vectors:
            for row in range(4):
                for column in range(4):
                    predicted[row][column] += (
                        accel_variance * noise[row] * noise[column]
                    )
        self._covariance = [
            [
                max(0.0, predicted[row][column])
                if row == column
                else 0.5 * (predicted[row][column] + predicted[column][row])
                for column in range(4)
            ]
            for row in range(4)
        ]
        self._state_time_us = time_us
        return acceleration_uwb_mps2 is not None

    def _aligned_horizontal_accel(self, linear_ref: Vector3) -> Vector3:
        cosine = math.cos(self._yaw_alignment_rad)
        sine = math.sin(self._yaw_alignment_rad)
        ax = cosine * linear_ref[0] - sine * linear_ref[1]
        ay = sine * linear_ref[0] + cosine * linear_ref[1]
        return ax, ay, linear_ref[2]

    def update_imu(
        self, sample: ImuSample | Mapping[str, Any]
    ) -> dict[str, Any]:
        imu = sample if isinstance(sample, ImuSample) else ImuSample.from_mapping(sample)
        flags: list[str] = []
        self._imu_sample_count += 1
        imu_time_us = imu.time_us
        self._last_output_uptime_ms = imu.uptime_ms

        if not self._verify_module(imu.module_id, flags):
            return self._output(imu.uptime_ms, flags)

        if self._last_imu_time_us is not None:
            backstep_us = self._last_imu_time_us - imu_time_us
            if backstep_us > self.config.reboot_backstep_ms * 1000:
                flags.extend(self._reset_state("imu_time_reboot"))
                self._last_output_uptime_ms = imu.uptime_ms
            elif backstep_us > 0:
                self._record_ordering_reject("imu_out_of_order", backstep_us)
                flags.append("imu_out_of_order")
                return self._output(imu.uptime_ms, flags)
            elif backstep_us == 0:
                self._record_ordering_reject("imu_duplicate_time", 0)
                flags.append("imu_duplicate_time")
                return self._output(imu.uptime_ms, flags)

        if self._last_imu_time_us is not None:
            gap_us = imu_time_us - self._last_imu_time_us
            if gap_us > self.config.imu_gap_reset_ms * 1000:
                flags.extend(self._reset_state("imu_gap"))
                self._last_output_uptime_ms = imu.uptime_ms

        if not self._verify_cross_stream_order(imu_time_us, flags, "imu"):
            return self._output(imu.uptime_ms, flags)

        previous_imu_us = self._last_imu_time_us
        self._last_imu_time_us = imu_time_us
        self._last_fusion_time_ticks = imu.fusion_time_ticks
        self._last_fusion_timer_hz = imu.fusion_timer_hz
        self._last_imu_dt_s = (
            (imu_time_us - previous_imu_us) / 1_000_000.0
            if previous_imu_us is not None
            else None
        )

        if not imu.valid:
            self._imu_invalid_count += 1
            self._predict_to(imu_time_us, None)
            self._last_linear_ref_mps2 = None
            self._last_linear_uwb_mps2 = None
            flags.append("imu_invalid")
            return self._output(imu.uptime_ms, flags)

        try:
            quaternion_norm = math.sqrt(
                sum(_finite(value, "quaternion") ** 2 for value in imu.quaternion_xyzw)
            )
            self._last_quaternion_norm = quaternion_norm
            if not (
                self.config.quaternion_hard_norm_min
                <= quaternion_norm
                <= self.config.quaternion_hard_norm_max
            ):
                raise ValueError("quaternion norm outside hard gate")
            quaternion = normalize_quaternion(imu.quaternion_xyzw)
            linear_ref = body_accel_to_reference_linear(
                imu.accel_body_mps2,
                quaternion,
                self.config.gravity_mps2,
            )
            gyro = tuple(
                _finite(value, "gyro") for value in imu.gyro_body_rps
            )
        except ValueError:
            self._imu_invalid_count += 1
            self._predict_to(imu_time_us, None)
            self._last_linear_ref_mps2 = None
            self._last_linear_uwb_mps2 = None
            flags.extend(
                ("imu_invalid", "quaternion_invalid", "quaternion_norm_hard_reject")
            )
            return self._output(imu.uptime_ms, flags)

        if not (
            self.config.quaternion_soft_norm_min
            <= quaternion_norm
            <= self.config.quaternion_soft_norm_max
        ):
            flags.append("quaternion_norm_soft_warning")
        self._last_gyro_body_rps = gyro  # type: ignore[assignment]
        self._last_imu_yaw_ref_rad = quaternion_yaw_radians(quaternion)
        flags.extend(("imu_valid", "orientation_valid"))
        corrected_linear_ref = self._stationary_corrected_accel(
            imu_time_us,
            linear_ref,
            gyro,  # type: ignore[arg-type]
            flags,
        )
        self._last_linear_ref_mps2 = corrected_linear_ref

        linear_accel_magnitude = math.sqrt(
            sum(value * value for value in corrected_linear_ref)
        )
        gyro_magnitude = math.sqrt(sum(value * value for value in gyro))
        if (
            linear_accel_magnitude > self.config.max_horizontal_accel_mps2
            or gyro_magnitude > self.config.high_dynamic_gyro_threshold_rps
        ):
            self._high_dynamic_until_us = max(
                self._high_dynamic_until_us or 0,
                imu_time_us + self.config.high_dynamic_cooldown_ms * 1000,
            )
            flags.append("high_dynamic_detected")
        high_dynamic = (
            self._high_dynamic_until_us is not None
            and imu_time_us < self._high_dynamic_until_us
        )

        yaw_recent = (
            self._last_yaw_alignment_time_us is not None
            and imu_time_us - self._last_yaw_alignment_time_us
            <= self.config.yaw_alignment_max_age_ms * 1000
        )
        uwb_recent = (
            self._last_accepted_position_time_us is not None
            and 0
            <= imu_time_us - self._last_accepted_position_time_us
            <= self.config.accel_prediction_max_uwb_age_ms * 1000
        )
        accel_confident = (
            self._yaw_alignment_valid
            and yaw_recent
            and self._accel_bias_valid
            and uwb_recent
            and self._reacquire_probation_remaining == 0
            and not high_dynamic
        )

        aligned: Vector3 | None = None
        if accel_confident:
            candidate = self._aligned_horizontal_accel(corrected_linear_ref)
            horizontal_magnitude = math.hypot(candidate[0], candidate[1])
            if horizontal_magnitude <= self.config.max_horizontal_accel_mps2:
                aligned = candidate
                self._last_linear_uwb_mps2 = candidate
                flags.append("yaw_aligned")
            else:
                self._last_linear_uwb_mps2 = None
                flags.append("accel_rejected_limit")
        else:
            self._last_linear_uwb_mps2 = None
            if not self._yaw_alignment_valid:
                flags.append("yaw_unaligned")
            elif not yaw_recent:
                flags.append("accel_blocked_yaw_stale")
            elif not self._accel_bias_valid:
                flags.append("accel_blocked_bias_uncalibrated")
            elif not uwb_recent:
                flags.append("accel_blocked_uwb_stale")
            elif self._reacquire_probation_remaining > 0:
                flags.append("accel_blocked_reacquire_probation")
            elif high_dynamic:
                flags.append("accel_blocked_high_dynamic")

        imu_delta_us = (
            imu_time_us - previous_imu_us if previous_imu_us is not None else None
        )
        accel_gap_valid = (
            imu_delta_us is not None
            and 0 < imu_delta_us <= self.config.max_accel_integration_gap_ms * 1000
        )
        acceleration_xy = (
            (aligned[0], aligned[1])
            if aligned is not None and accel_gap_valid
            else None
        )
        if self._predict_to(imu_time_us, acceleration_xy):
            flags.append("imu_propagated")
        elif self._ready:
            flags.append("constant_velocity_propagated")
            if previous_imu_us is None:
                flags.append("imu_accel_warmup_cv_only")
            elif not accel_gap_valid:
                flags.append("imu_accel_gap_cv_only")
        if self._stationary and self._ready:
            self._zero_velocity_update(flags)
        self._clamp_velocity(flags)
        return self._output(imu.uptime_ms, flags)

    def _update_yaw_alignment(
        self, position: RawPosition, flags: list[str]
    ) -> None:
        point = (position.time_us, position.x_m, position.y_m)
        self._yaw_window.append(point)
        cutoff_us = position.time_us - self.config.yaw_max_window_ms * 1000
        while len(self._yaw_window) > 1 and self._yaw_window[0][0] < cutoff_us:
            self._yaw_window.pop(0)
        if len(self._yaw_window) < 2:
            flags.append("yaw_anchor_initialized")
            return

        first = self._yaw_window[0]
        duration_s = (position.time_us - first[0]) / 1_000_000.0
        dx = position.x_m - first[1]
        dy = position.y_m - first[2]
        displacement = math.hypot(dx, dy)
        self._last_yaw_window_duration_s = duration_s
        self._last_yaw_displacement_m = displacement
        if (
            duration_s < self.config.yaw_min_window_ms / 1000.0
            or displacement < self.config.yaw_min_displacement_m
        ):
            return

        speed_mps = displacement / duration_s if duration_s > 0.0 else 0.0
        uwb_heading = math.atan2(dy, dx)
        path_length = 0.0
        weighted_heading_error = 0.0
        for previous, current in zip(self._yaw_window, self._yaw_window[1:]):
            segment_dx = current[1] - previous[1]
            segment_dy = current[2] - previous[2]
            segment_length = math.hypot(segment_dx, segment_dy)
            if segment_length < self.config.yaw_min_segment_m:
                continue
            segment_heading = math.atan2(segment_dy, segment_dx)
            heading_error = wrap_angle_radians(segment_heading - uwb_heading)
            path_length += segment_length
            weighted_heading_error += segment_length * heading_error * heading_error

        straightness = (
            min(1.0, displacement / path_length) if path_length > 0.0 else 0.0
        )
        course_uncertainty_deg = (
            math.degrees(math.sqrt(weighted_heading_error / path_length))
            if path_length > 0.0
            else math.inf
        )
        self._last_yaw_speed_mps = speed_mps
        self._last_yaw_straightness = straightness
        self._last_yaw_course_uncertainty_deg = course_uncertainty_deg

        def reject(reason: str) -> None:
            self._last_yaw_reject_reason = reason
            flags.extend(("yaw_alignment_rejected", f"yaw_reject:{reason}"))
            self._yaw_window = [point]

        if speed_mps <= self.config.yaw_min_speed_mps:
            reject("speed")
            return
        if straightness < self.config.yaw_min_straightness:
            reject("not_straight")
            return
        if course_uncertainty_deg >= self.config.yaw_max_course_uncertainty_deg:
            reject("course_uncertainty")
            return

        imu_fresh = (
            self._last_imu_time_us is not None
            and self._last_imu_yaw_ref_rad is not None
            and abs(position.time_us - self._last_imu_time_us)
            <= self.config.yaw_max_imu_age_ms * 1000
        )
        if not imu_fresh:
            reject("stale_imu")
            return
        if (
            self._last_gyro_body_rps is None
            or abs(self._last_gyro_body_rps[2])
            >= self.config.yaw_max_abs_gyro_z_rps
        ):
            reject("gyro_z")
            return

        candidate = wrap_angle_radians(
            uwb_heading - float(self._last_imu_yaw_ref_rad)
        )
        if self._yaw_alignment_valid:
            error = wrap_angle_radians(candidate - self._yaw_alignment_rad)
            self._last_yaw_innovation_deg = abs(math.degrees(error))
            if self._last_yaw_innovation_deg > self.config.yaw_max_innovation_deg:
                reject("innovation")
                return
            self._yaw_alignment_rad = wrap_angle_radians(
                self._yaw_alignment_rad + self.config.yaw_alignment_gain * error
            )
            flags.append("yaw_alignment_updated")
        else:
            self._last_yaw_innovation_deg = 0.0
            self._yaw_alignment_rad = candidate
            self._yaw_alignment_valid = True
            flags.append("yaw_alignment_initialized")
        self._yaw_alignment_updates += 1
        self._yaw_alignment_source = "uwb_motion"
        self._last_yaw_alignment_time_us = position.time_us
        self._last_yaw_reject_reason = None
        self._yaw_window = [point]
        flags.extend(("yaw_aligned", "yaw_motion_assumes_body_forward_x"))

    def align_yaw_from_heading(
        self,
        uwb_heading_rad: float,
        *,
        source: str = "external_heading",
        uwb_to_enu_yaw_rad: float | None = None,
    ) -> dict[str, Any]:
        """Align BNO reference yaw to an absolute heading in the UWB frame."""

        flags: list[str] = []
        heading = _finite(uwb_heading_rad, "uwb_heading_rad")
        if self._last_imu_yaw_ref_rad is None:
            flags.append("yaw_reference_rejected:no_imu")
            return self._output(self._last_output_uptime_ms, flags)
        if (
            self._last_gyro_body_rps is None
            or abs(self._last_gyro_body_rps[2])
            >= self.config.yaw_max_abs_gyro_z_rps
        ):
            flags.append("yaw_reference_rejected:gyro_z")
            return self._output(self._last_output_uptime_ms, flags)

        candidate = wrap_angle_radians(
            heading - self._last_imu_yaw_ref_rad
        )
        if self._yaw_alignment_valid:
            error = wrap_angle_radians(candidate - self._yaw_alignment_rad)
            self._last_yaw_innovation_deg = abs(math.degrees(error))
            if self._last_yaw_innovation_deg > self.config.yaw_max_innovation_deg:
                flags.append("yaw_reference_rejected:innovation")
                return self._output(self._last_output_uptime_ms, flags)
            self._yaw_alignment_rad = wrap_angle_radians(
                self._yaw_alignment_rad
                + self.config.yaw_alignment_gain * error
            )
            flags.append("yaw_alignment_updated")
        else:
            self._yaw_alignment_rad = candidate
            self._yaw_alignment_valid = True
            self._last_yaw_innovation_deg = 0.0
            flags.append("yaw_alignment_initialized")
        self._yaw_alignment_updates += 1
        self._yaw_alignment_source = str(source)
        self._last_yaw_alignment_time_us = self._last_imu_time_us
        self._last_yaw_reject_reason = None
        if uwb_to_enu_yaw_rad is not None:
            self._uwb_to_enu_yaw_rad = wrap_angle_radians(
                _finite(uwb_to_enu_yaw_rad, "uwb_to_enu_yaw_rad")
            )
            self._uwb_to_enu_yaw_valid = True
            self._uwb_to_enu_yaw_source = str(source)
            flags.append("enu_frame_aligned")
        flags.extend(("yaw_aligned", f"yaw_source:{source}"))
        return self._output(self._last_output_uptime_ms, flags)

    def update_position(
        self, sample: RawPosition | Mapping[str, Any]
    ) -> dict[str, Any]:
        position = (
            sample
            if isinstance(sample, RawPosition)
            else RawPosition.from_mapping(sample)
        )
        flags: list[str] = []
        self._position_sample_count += 1
        position_time_us = position.time_us
        self._last_output_uptime_ms = position.uptime_ms

        if not self._verify_module(position.module_id, flags):
            return self._output(position.uptime_ms, flags)

        if self._last_position_time_us is not None:
            backstep_us = self._last_position_time_us - position_time_us
            if backstep_us > self.config.reboot_backstep_ms * 1000:
                flags.extend(self._reset_state("position_time_reboot"))
                self._last_output_uptime_ms = position.uptime_ms
            elif backstep_us > 0:
                self._record_ordering_reject(
                    "position_out_of_order", backstep_us
                )
                flags.append("position_out_of_order")
                return self._output(position.uptime_ms, flags)
            elif backstep_us == 0:
                same_track = (
                    self._protocol == position.protocol
                    and self._tag_id == position.tag_id
                )
                if not same_track:
                    self._record_ordering_reject("position_duplicate_time", 0)
                    flags.append("position_duplicate_time")
                    return self._output(position.uptime_ms, flags)
                # The ESP32 solver can publish more than one distinct raw
                # correction inside one millisecond. Preserve TCP/queue
                # sequence as a stable tie-break and apply them at zero dt;
                # no synthetic physical time is introduced.
                self._position_same_time_count += 1
                flags.append("position_same_time_update")

        if self._protocol is not None and position.protocol != self._protocol:
            flags.extend(self._reset_state("protocol_changed"))
        elif self._tag_id is not None and position.tag_id != self._tag_id:
            flags.extend(self._reset_state("tag_changed"))

        if self._last_position_time_us is not None:
            gap_us = position_time_us - self._last_position_time_us
            if gap_us > self.config.position_gap_reset_ms * 1000:
                flags.extend(self._reset_state("position_gap"))
                self._last_output_uptime_ms = position.uptime_ms

        if not self._verify_cross_stream_order(
            position_time_us, flags, "position"
        ):
            return self._output(position.uptime_ms, flags)

        self._protocol = position.protocol
        self._tag_id = position.tag_id
        previous_raw_position = self._last_raw_position
        self._last_raw_position = position
        self._append_raw_position(position)
        self._last_position_time_us = position_time_us
        self._last_position_sigma_m = position.sigma_m
        self._last_position_rms_m = position.rms_m
        self._last_position_measurement_std_m = self._uwb_measurement_std(position)
        self._last_uwb_innovation_gate_m = self._uwb_innovation_gate(position)

        if not self._ready:
            self._initialize_position(position, flags)
            return self._output(position.uptime_ms, flags)

        self._predict_to(position_time_us, None)
        residual_x = position.x_m - self._x_m
        residual_y = position.y_m - self._y_m
        self._last_position_residual_m = math.hypot(residual_x, residual_y)
        measurement_variance = self._last_position_measurement_std_m**2
        self._last_position_nis = self._kalman_update_pair(
            (0, 1),
            (residual_x, residual_y),
            measurement_variance,
            apply=False,
        )
        if (
            self._last_position_residual_m > self._last_uwb_innovation_gate_m
            or self._last_position_nis > self.config.uwb_nis_gate
        ):
            self._position_outlier_count += 1
            self._consecutive_position_outliers += 1
            if (
                not self._rejected_position_cluster
                and previous_raw_position is not None
            ):
                self._rejected_position_cluster.append(previous_raw_position)
            self._rejected_position_cluster.append(position)
            keep = max(2, self.config.position_reacquire_after_rejects)
            if len(self._rejected_position_cluster) > keep:
                self._rejected_position_cluster = self._rejected_position_cluster[-keep:]
            flags.extend(("uwb_outlier_rejected", "position_prediction_only"))
            accepted_gap_us = (
                position_time_us - self._last_accepted_position_time_us
                if self._last_accepted_position_time_us is not None
                else None
            )
            cluster_ready = (
                self._consecutive_position_outliers
                >= self.config.position_reacquire_after_rejects
                and self._rejected_cluster_is_coherent()
            )
            acceptance_gap_ready = (
                accepted_gap_us is not None
                and accepted_gap_us >= self.config.position_reacquire_gap_ms * 1000
            )
            if cluster_ready or acceptance_gap_ready:
                reason = (
                    "position_acceptance_gap"
                    if acceptance_gap_ready
                    else "coherent_outlier_cluster"
                )
                self._soft_reacquire_position(position, flags)
                flags.append(f"reacquire:{reason}")
                return self._output(position.uptime_ms, flags)
            self._clamp_velocity(flags)
            if not self._stationary:
                # Keep the public correction sample equal to the raw dynamic
                # measurement while retaining the rejected prediction only as
                # internal state.  This guarantees that fusion cannot amplify
                # a disputed sample; prediction still fills the radio gap.
                flags.append("moving_uwb_position_authoritative")
                output = self._output(position.uptime_ms, flags)
                output["x"] = position.x_m
                output["y"] = position.y_m
                return output
            return self._output(position.uptime_ms, flags)

        self._consecutive_position_outliers = 0
        self._rejected_position_cluster = []
        if self._reacquire_probation_remaining > 0:
            self._reacquire_probation_remaining -= 1
        previous_accepted_us = self._last_accepted_position_time_us
        self._last_accepted_position_time_us = position_time_us
        self._last_position_dt_s = (
            (position_time_us - previous_accepted_us) / 1_000_000.0
            if previous_accepted_us is not None
            else None
        )
        self._position_accepted_count += 1
        self._update_yaw_alignment(position, flags)
        previous_velocity = (self._vx_mps, self._vy_mps)
        self._kalman_update_pair(
            (0, 1),
            (residual_x, residual_y),
            measurement_variance,
            apply=True,
        )
        if (
            abs(self._vx_mps - previous_velocity[0]) > 1e-12
            or abs(self._vy_mps - previous_velocity[1]) > 1e-12
        ):
            flags.append("velocity_corrected")
        else:
            flags.append("velocity_correction_negligible")
        self._clamp_velocity(flags)
        if not self._stationary:
            # A causal low-pass position state necessarily lags a moving tag.
            # Keep every accepted raw UWB correction authoritative while the
            # IMU/CV state bridges only the intervals between corrections.
            # Once UWB independently confirms rest, retain the EKF correction
            # so the static cloud still benefits from noise reduction.
            self._x_m = position.x_m
            self._y_m = position.y_m
            flags.append("moving_uwb_position_authoritative")
        self._position_correction_count += 1
        flags.extend(("position_corrected", "ekf_corrected"))
        if self._yaw_alignment_valid:
            flags.append("yaw_aligned")
        return self._output(position.uptime_ms, flags)

    def snapshot(self) -> dict[str, Any]:
        """Return the current fused state without advancing it."""

        return self._output(self._last_output_uptime_ms, ["snapshot"])

    def _diagnostics(self) -> dict[str, Any]:
        yaw_bno_rad = self._last_imu_yaw_ref_rad
        yaw_uwb_rad = (
            wrap_angle_radians(yaw_bno_rad + self._yaw_alignment_rad)
            if yaw_bno_rad is not None and self._yaw_alignment_valid
            else None
        )
        yaw_enu_rad = (
            wrap_angle_radians(yaw_uwb_rad + self._uwb_to_enu_yaw_rad)
            if yaw_uwb_rad is not None and self._uwb_to_enu_yaw_valid
            else None
        )
        return {
            "ready": self._ready,
            "filter": "ekf_cv_accel_zupt",
            "frame": "body to BNO; yaw-calibrated to UWB; not ENU-calibrated until RTK course",
            "yaw_alignment_assumption": "body +X follows UWB displacement",
            "imu_to_uwb_calibration": "yaw_only",
            "module_id": self._module_id,
            "protocol": self._protocol,
            "tag_id": self._tag_id,
            "imu_samples": self._imu_sample_count,
            "imu_invalid_samples": self._imu_invalid_count,
            "position_samples": self._position_sample_count,
            "position_accepted": self._position_accepted_count,
            "position_corrections": self._position_correction_count,
            "position_outliers": self._position_outlier_count,
            "consecutive_position_outliers": self._consecutive_position_outliers,
            "position_reacquisitions": self._position_reacquisition_count,
            "position_soft_reacquisitions": (
                self._position_soft_reacquisition_count
            ),
            "position_same_time_updates": self._position_same_time_count,
            "reacquire_probation_remaining": (
                self._reacquire_probation_remaining
            ),
            "velocity_clamps": self._velocity_clamp_count,
            "ordering_rejects": self._ordering_reject_count,
            "ordering_reject_reasons": dict(self._ordering_reject_reasons),
            "last_ordering_reject_reason": self._last_ordering_reject_reason,
            "last_ordering_lag_ms": (
                self._last_ordering_lag_us / 1000.0
                if self._last_ordering_lag_us is not None
                else None
            ),
            "max_ordering_lag_ms": self._max_ordering_lag_us / 1000.0,
            "module_mismatches": self._module_mismatch_count,
            "reset_count": self._reset_count,
            "last_reset_reason": self._last_reset_reason,
            "yaw_alignment_valid": self._yaw_alignment_valid,
            "yaw_alignment_rad": (
                self._yaw_alignment_rad if self._yaw_alignment_valid else None
            ),
            "yaw_alignment_deg": (
                math.degrees(self._yaw_alignment_rad)
                if self._yaw_alignment_valid
                else None
            ),
            "yaw_alignment_updates": self._yaw_alignment_updates,
            "yaw_alignment_source": self._yaw_alignment_source,
            "last_imu_yaw_bno_deg": (
                math.degrees(yaw_bno_rad) if yaw_bno_rad is not None else None
            ),
            "last_imu_yaw_uwb_deg": (
                math.degrees(yaw_uwb_rad) if yaw_uwb_rad is not None else None
            ),
            "enu_yaw_valid": yaw_enu_rad is not None,
            "last_imu_yaw_enu_deg": (
                math.degrees(yaw_enu_rad) if yaw_enu_rad is not None else None
            ),
            "uwb_to_enu_yaw_deg": (
                math.degrees(self._uwb_to_enu_yaw_rad)
                if self._uwb_to_enu_yaw_valid
                else None
            ),
            "uwb_to_enu_yaw_source": self._uwb_to_enu_yaw_source,
            "last_yaw_alignment_time_us": self._last_yaw_alignment_time_us,
            "last_yaw_window_duration_s": self._last_yaw_window_duration_s,
            "last_yaw_displacement_m": self._last_yaw_displacement_m,
            "last_yaw_speed_mps": self._last_yaw_speed_mps,
            "last_yaw_straightness": self._last_yaw_straightness,
            "last_yaw_course_uncertainty_deg": (
                self._last_yaw_course_uncertainty_deg
            ),
            "last_yaw_innovation_deg": self._last_yaw_innovation_deg,
            "last_yaw_reject_reason": self._last_yaw_reject_reason,
            "last_quaternion_norm": self._last_quaternion_norm,
            "last_linear_accel_bno_ref_mps2": self._last_linear_ref_mps2,
            "last_linear_accel_uwb_mps2": self._last_linear_uwb_mps2,
            "last_gyro_body_rps": self._last_gyro_body_rps,
            "last_imu_dt_s": self._last_imu_dt_s,
            "last_position_dt_s": self._last_position_dt_s,
            "last_position_residual_m": self._last_position_residual_m,
            "last_uwb_innovation_gate_m": self._last_uwb_innovation_gate_m,
            "last_position_sigma_m": self._last_position_sigma_m,
            "last_position_rms_m": self._last_position_rms_m,
            "last_position_measurement_std_m": (
                self._last_position_measurement_std_m
            ),
            "last_position_nis": self._last_position_nis,
            "covariance_diagonal": [
                self._covariance[index][index] for index in range(4)
            ],
            "accel_bias_ref_mps2": list(self._accel_bias_ref_mps2),
            "accel_bias_valid": self._accel_bias_valid,
            "stationary": self._stationary,
            "stationary_samples": self._stationary_sample_count,
            "accel_bias_updates": self._bias_update_count,
            "zero_velocity_updates": self._zupt_count,
            "high_dynamic_until_us": self._high_dynamic_until_us,
            "state_time_us": self._state_time_us,
            "last_fusion_time_ticks": self._last_fusion_time_ticks,
            "fusion_timer_hz": self._last_fusion_timer_hz,
        }

    def _output(
        self, uptime_ms: int | None, flags: Sequence[str]
    ) -> dict[str, Any]:
        return {
            "stream": "fused",
            "uptime_ms": uptime_ms,
            "sample_time_us": self._state_time_us,
            "fusion_time_ticks": self._last_fusion_time_ticks,
            "fusion_timer_hz": self._last_fusion_timer_hz,
            "module_id": self._module_id,
            "protocol": self._protocol,
            "tag_id": self._tag_id,
            "ready": self._ready,
            "x": self._x_m if self._ready else None,
            "y": self._y_m if self._ready else None,
            "vx": self._vx_mps if self._ready else None,
            "vy": self._vy_mps if self._ready else None,
            "flags": list(dict.fromkeys(flags)),
            "diagnostics": self._diagnostics(),
        }


__all__ = [
    "FusionConfig",
    "ImuSample",
    "RawPosition",
    "UwbImuFusion",
    "body_accel_to_reference_linear",
    "normalize_quaternion",
    "quaternion_yaw_radians",
    "rotate_body_to_reference",
    "wrap_angle_radians",
]
