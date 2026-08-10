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

    def __post_init__(self) -> None:
        if self.uptime_ms < 0:
            raise ValueError("uptime_ms must be non-negative")
        if self.module_id is not None and self.module_id < 0:
            raise ValueError("module_id must be non-negative")

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
        return cls(
            uptime_ms=uptime_ms,
            accel_body_mps2=accel,
            quaternion_xyzw=quaternion,
            gyro_body_rps=gyro,
            valid=bool(valid_value) if valid_value is not None else True,
            module_id=int(module_value) if module_value is not None else None,
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
        _finite(self.x_m, "raw_x")
        _finite(self.y_m, "raw_y")
        for name, value in (("sigma_m", self.sigma_m), ("rms_m", self.rms_m)):
            if value is not None and _finite(value, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")

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
    yaw_min_segment_m: float = 0.02
    max_accel_integration_gap_ms: int = 50
    imu_gap_reset_ms: int = 300
    position_gap_reset_ms: int = 2500
    reboot_backstep_ms: int = 50
    cross_stream_reorder_tolerance_ms: int = 50
    min_velocity_correction_dt_s: float = 0.005
    max_horizontal_accel_mps2: float = 40.0
    quaternion_soft_norm_min: float = 0.95
    quaternion_soft_norm_max: float = 1.05
    quaternion_hard_norm_min: float = 0.8
    quaternion_hard_norm_max: float = 1.2
    uwb_default_innovation_gate_m: float = 2.0
    uwb_min_innovation_gate_m: float = 0.20
    uwb_sigma_gate_multiplier: float = 4.0
    uwb_rms_gate_multiplier: float = 3.0
    max_velocity_mps: float = 5.0
    position_reacquire_after_rejects: int = 3
    position_reacquire_gap_ms: int = 2500

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
        self._state_uptime_ms: int | None = None

        self._last_imu_uptime_ms: int | None = None
        self._last_position_uptime_ms: int | None = None
        self._last_accepted_position_uptime_ms: int | None = None
        self._last_event_uptime_ms: int | None = None
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
        self._velocity_clamp_count = 0
        self._ordering_reject_count = 0
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
        self._state_uptime_ms = None
        self._last_imu_uptime_ms = None
        self._last_position_uptime_ms = None
        self._last_accepted_position_uptime_ms = None
        self._last_event_uptime_ms = None
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

    def _verify_cross_stream_order(
        self, uptime_ms: int, flags: list[str]
    ) -> bool:
        if (
            self._last_event_uptime_ms is not None
            and uptime_ms
            < self._last_event_uptime_ms
            - self.config.cross_stream_reorder_tolerance_ms
        ):
            self._ordering_reject_count += 1
            flags.append("event_out_of_order")
            return False
        if self._last_event_uptime_ms is None or uptime_ms > self._last_event_uptime_ms:
            self._last_event_uptime_ms = uptime_ms
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

    def _initialize_position(
        self, position: RawPosition, flags: list[str]
    ) -> None:
        """Initialize a bounded state from an authoritative raw position."""

        self._protocol = position.protocol
        self._tag_id = position.tag_id
        self._last_position_uptime_ms = position.uptime_ms
        self._last_position_sigma_m = position.sigma_m
        self._last_position_rms_m = position.rms_m
        self._last_uwb_innovation_gate_m = self._uwb_innovation_gate(position)
        self._x_m = position.x_m
        self._y_m = position.y_m
        self._vx_mps = 0.0
        self._vy_mps = 0.0
        self._state_uptime_ms = position.uptime_ms
        self._last_event_uptime_ms = position.uptime_ms
        self._ready = True
        self._last_accepted_position_uptime_ms = position.uptime_ms
        self._last_position_dt_s = None
        self._last_position_residual_m = 0.0
        self._consecutive_position_outliers = 0
        self._position_accepted_count += 1
        self._update_yaw_alignment(position, flags)
        flags.append("position_initialized")

    def _predict_to(
        self,
        uptime_ms: int,
        acceleration_uwb_mps2: tuple[float, float] | None,
    ) -> bool:
        if not self._ready:
            return False
        if self._state_uptime_ms is None:
            self._state_uptime_ms = uptime_ms
            return False
        delta_ms = uptime_ms - self._state_uptime_ms
        if delta_ms <= 0:
            return False

        dt = delta_ms / 1000.0
        ax, ay = acceleration_uwb_mps2 or (0.0, 0.0)
        self._x_m += self._vx_mps * dt + 0.5 * ax * dt * dt
        self._y_m += self._vy_mps * dt + 0.5 * ay * dt * dt
        self._vx_mps += ax * dt
        self._vy_mps += ay * dt
        self._state_uptime_ms = uptime_ms
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

        if not self._verify_module(imu.module_id, flags):
            return self._output(imu.uptime_ms, flags)

        if self._last_imu_uptime_ms is not None:
            backstep_ms = self._last_imu_uptime_ms - imu.uptime_ms
            if backstep_ms > self.config.reboot_backstep_ms:
                flags.extend(self._reset_state("imu_uptime_reboot"))
            elif backstep_ms > 0:
                self._ordering_reject_count += 1
                flags.append("imu_out_of_order")
                return self._output(imu.uptime_ms, flags)
            elif backstep_ms == 0:
                self._ordering_reject_count += 1
                flags.append("imu_duplicate_uptime")
                return self._output(imu.uptime_ms, flags)

        if self._last_imu_uptime_ms is not None:
            gap_ms = imu.uptime_ms - self._last_imu_uptime_ms
            if gap_ms > self.config.imu_gap_reset_ms:
                flags.extend(self._reset_state("imu_gap"))

        if not self._verify_cross_stream_order(imu.uptime_ms, flags):
            return self._output(imu.uptime_ms, flags)

        previous_imu_ms = self._last_imu_uptime_ms
        self._last_imu_uptime_ms = imu.uptime_ms
        self._last_imu_dt_s = (
            (imu.uptime_ms - previous_imu_ms) / 1000.0
            if previous_imu_ms is not None
            else None
        )

        if not imu.valid:
            self._imu_invalid_count += 1
            self._predict_to(imu.uptime_ms, None)
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
            self._predict_to(imu.uptime_ms, None)
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
        self._last_linear_ref_mps2 = linear_ref
        self._last_gyro_body_rps = gyro  # type: ignore[assignment]
        self._last_imu_yaw_ref_rad = quaternion_yaw_radians(quaternion)
        flags.extend(("imu_valid", "orientation_valid"))

        aligned: Vector3 | None = None
        if self._yaw_alignment_valid:
            candidate = self._aligned_horizontal_accel(linear_ref)
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
            flags.append("yaw_unaligned")

        imu_delta_ms = (
            imu.uptime_ms - previous_imu_ms if previous_imu_ms is not None else None
        )
        accel_gap_valid = (
            imu_delta_ms is not None
            and 0 < imu_delta_ms <= self.config.max_accel_integration_gap_ms
        )
        acceleration_xy = (
            (aligned[0], aligned[1])
            if aligned is not None and accel_gap_valid
            else None
        )
        if self._predict_to(imu.uptime_ms, acceleration_xy):
            flags.append("imu_propagated")
        elif self._ready:
            flags.append("constant_velocity_propagated")
            if previous_imu_ms is None:
                flags.append("imu_accel_warmup_cv_only")
            elif not accel_gap_valid:
                flags.append("imu_accel_gap_cv_only")
        self._clamp_velocity(flags)
        return self._output(imu.uptime_ms, flags)

    def _update_yaw_alignment(
        self, position: RawPosition, flags: list[str]
    ) -> None:
        point = (position.uptime_ms, position.x_m, position.y_m)
        self._yaw_window.append(point)
        cutoff_ms = position.uptime_ms - self.config.yaw_max_window_ms
        while len(self._yaw_window) > 1 and self._yaw_window[0][0] < cutoff_ms:
            self._yaw_window.pop(0)
        if len(self._yaw_window) < 2:
            flags.append("yaw_anchor_initialized")
            return

        first = self._yaw_window[0]
        duration_s = (position.uptime_ms - first[0]) / 1000.0
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
            self._last_imu_uptime_ms is not None
            and self._last_imu_yaw_ref_rad is not None
            and abs(position.uptime_ms - self._last_imu_uptime_ms)
            <= self.config.yaw_max_imu_age_ms
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
        self._last_yaw_reject_reason = None
        self._yaw_window = [point]
        flags.extend(("yaw_aligned", "yaw_motion_assumes_body_forward_x"))

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

        if not self._verify_module(position.module_id, flags):
            return self._output(position.uptime_ms, flags)

        if self._last_position_uptime_ms is not None:
            backstep_ms = self._last_position_uptime_ms - position.uptime_ms
            if backstep_ms > self.config.reboot_backstep_ms:
                flags.extend(self._reset_state("position_uptime_reboot"))
            elif backstep_ms > 0:
                self._ordering_reject_count += 1
                flags.append("position_out_of_order")
                return self._output(position.uptime_ms, flags)
            elif backstep_ms == 0:
                self._ordering_reject_count += 1
                flags.append("position_duplicate_uptime")
                return self._output(position.uptime_ms, flags)

        if self._protocol is not None and position.protocol != self._protocol:
            flags.extend(self._reset_state("protocol_changed"))
        elif self._tag_id is not None and position.tag_id != self._tag_id:
            flags.extend(self._reset_state("tag_changed"))

        if self._last_position_uptime_ms is not None:
            gap_ms = position.uptime_ms - self._last_position_uptime_ms
            if gap_ms > self.config.position_gap_reset_ms:
                flags.extend(self._reset_state("position_gap"))

        if not self._verify_cross_stream_order(position.uptime_ms, flags):
            return self._output(position.uptime_ms, flags)

        self._protocol = position.protocol
        self._tag_id = position.tag_id
        self._last_position_uptime_ms = position.uptime_ms
        self._last_position_sigma_m = position.sigma_m
        self._last_position_rms_m = position.rms_m
        self._last_uwb_innovation_gate_m = self._uwb_innovation_gate(position)

        if not self._ready:
            self._initialize_position(position, flags)
            return self._output(position.uptime_ms, flags)

        self._predict_to(position.uptime_ms, None)
        residual_x = position.x_m - self._x_m
        residual_y = position.y_m - self._y_m
        self._last_position_residual_m = math.hypot(residual_x, residual_y)
        if self._last_position_residual_m > self._last_uwb_innovation_gate_m:
            self._position_outlier_count += 1
            self._consecutive_position_outliers += 1
            flags.extend(("uwb_outlier_rejected", "position_prediction_only"))
            accepted_gap_ms = (
                position.uptime_ms - self._last_accepted_position_uptime_ms
                if self._last_accepted_position_uptime_ms is not None
                else None
            )
            reacquire_reason = None
            if (
                self._consecutive_position_outliers
                >= self.config.position_reacquire_after_rejects
            ):
                reacquire_reason = "position_outlier_streak"
            elif (
                accepted_gap_ms is not None
                and accepted_gap_ms >= self.config.position_reacquire_gap_ms
            ):
                reacquire_reason = "position_acceptance_gap"
            if reacquire_reason is not None:
                # The gate still identifies and counts the incompatible
                # measurement.  Once the prediction has repeatedly disagreed
                # with UWB, however, retaining it indefinitely is less safe
                # than re-acquiring the measured track with zero velocity.
                self._position_reacquisition_count += 1
                flags.extend(self._reset_state(reacquire_reason))
                self._initialize_position(position, flags)
                flags.extend(("position_reacquired", f"reacquire:{reacquire_reason}"))
                return self._output(position.uptime_ms, flags)
            self._clamp_velocity(flags)
            return self._output(position.uptime_ms, flags)

        self._consecutive_position_outliers = 0
        previous_accepted_ms = self._last_accepted_position_uptime_ms
        self._last_accepted_position_uptime_ms = position.uptime_ms
        self._last_position_dt_s = (
            (position.uptime_ms - previous_accepted_ms) / 1000.0
            if previous_accepted_ms is not None
            else None
        )
        self._position_accepted_count += 1
        self._update_yaw_alignment(position, flags)
        self._x_m += self.config.alpha * residual_x
        self._y_m += self.config.alpha * residual_y
        correction_dt = self._last_position_dt_s
        if (
            correction_dt is not None
            and correction_dt >= self.config.min_velocity_correction_dt_s
        ):
            self._vx_mps += self.config.beta * residual_x / correction_dt
            self._vy_mps += self.config.beta * residual_y / correction_dt
            flags.append("velocity_corrected")
        else:
            flags.append("velocity_correction_skipped_dt")
        self._clamp_velocity(flags)
        self._position_correction_count += 1
        flags.append("position_corrected")
        if self._yaw_alignment_valid:
            flags.append("yaw_aligned")
        return self._output(position.uptime_ms, flags)

    def snapshot(self) -> dict[str, Any]:
        """Return the current fused state without advancing it."""

        return self._output(self._state_uptime_ms, ["snapshot"])

    def _diagnostics(self) -> dict[str, Any]:
        return {
            "ready": self._ready,
            "frame": "UWB horizontal; BNO reference yaw-aligned, not ENU-calibrated",
            "yaw_alignment_assumption": "body +X follows UWB displacement",
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
            "velocity_clamps": self._velocity_clamp_count,
            "ordering_rejects": self._ordering_reject_count,
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
        }

    def _output(
        self, uptime_ms: int | None, flags: Sequence[str]
    ) -> dict[str, Any]:
        return {
            "stream": "fused",
            "uptime_ms": uptime_ms,
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
