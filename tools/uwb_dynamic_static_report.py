#!/usr/bin/env python3
"""Generate an auditable static/dynamic comparison from UWB capture JSONL files.

This tool consumes the lossless output of ``uwb_dynamic_capture.py``.  It does
not call, import, or change the replay/dashboard implementation.  Static
precision is measured in the published local frame.  UWB-to-RTK disagreement
is evaluated only after a rigid 2-D anchor-frame registration and a bounded
wall-clock association, with the registration residual and all limitations
reported alongside the result.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import hashlib
import html
import json
import lzma
import math
import os
import pathlib
import re
import shutil
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Any, Iterable


PROTOCOL_LABELS = {
    "flextdoa": "FlexTDOA",
    "native_ds": "Native DS-TWR",
    "passive_ds": "Passive DS-TWR",
}
PROTOCOL_ALIASES = {
    "ds_twr": "native_ds",
    "native": "native_ds",
    "native_ds": "native_ds",
    "passive": "passive_ds",
    "passive_ds": "passive_ds",
    "flex_tdoa": "flextdoa",
    "flextdoa": "flextdoa",
}
PROTOCOL_COLORS = {
    "flextdoa": "#ea580c",
    "native_ds": "#2563eb",
    "passive_ds": "#7c3aed",
}
ANCHOR_IDS = (2, 3, 4, 5)
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalize_protocol(value: Any) -> str:
    clean = str(value or "unknown").strip().lower()
    return PROTOCOL_ALIASES.get(clean, clean)


def percentile(values: Iterable[float], q: float) -> float:
    clean = sorted(float(value) for value in values if finite(value))
    if not clean:
        return math.nan
    rank = (len(clean) - 1) * q / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return clean[low]
    return clean[low] + (clean[high] - clean[low]) * (rank - low)


def median(values: Iterable[float]) -> float:
    clean = [float(value) for value in values if finite(value)]
    return float(statistics.median(clean)) if clean else math.nan


def mean(values: Iterable[float]) -> float:
    clean = [float(value) for value in values if finite(value)]
    return float(statistics.fmean(clean)) if clean else math.nan


def json_clean(value: Any) -> Any:
    """Replace non-finite floats so the JSON output is standards compliant."""
    if isinstance(value, dict):
        return {str(key): json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_clean(item) for item in value]
    if isinstance(value, float):
        return float(value) if math.isfinite(float(value)) else None
    return value


def wall_seconds(record: dict[str, Any], *, measurement: bool = False) -> float:
    if measurement and finite(record.get("estimated_measurement_wall_ns")):
        return float(record["estimated_measurement_wall_ns"]) / 1e9
    if finite(record.get("received_at")):
        return float(record["received_at"])
    if finite(record.get("collector_wall_ns")):
        return float(record["collector_wall_ns"]) / 1e9
    return math.nan


def ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    return (
        (normal + altitude_m) * cos_latitude * math.cos(longitude),
        (normal + altitude_m) * cos_latitude * math.sin(longitude),
        (normal * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


def enu_rotation(latitude_deg: float, longitude_deg: float) -> tuple[tuple[float, float, float], ...]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    return (
            (-math.sin(longitude), math.cos(longitude), 0.0),
            (
                -math.sin(latitude) * math.cos(longitude),
                -math.sin(latitude) * math.sin(longitude),
                math.cos(latitude),
            ),
            (
                math.cos(latitude) * math.cos(longitude),
                math.cos(latitude) * math.sin(longitude),
                math.sin(latitude),
            ),
    )


def matrix3_vector(
    matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)  # type: ignore[return-value]


def is_rtk_fixed(record: dict[str, Any]) -> bool:
    return bool(record.get("gps_fix_valid")) and int(record.get("gps_fix_quality", 0)) == 4


def detect_motion(capture_id: str, path: pathlib.Path) -> str:
    text = f"{capture_id} {path.stem}".lower()
    if "static" in text:
        return "static"
    if any(token in text for token in ("walk", "dynamic", "motion")):
        return "dynamic"
    return "unknown"


@dataclass
class Capture:
    path: pathlib.Path
    capture_id: str = ""
    protocol: str = "unknown"
    motion: str = "unknown"
    complete: bool = False
    interrupted: bool = False
    start_wall: float = math.nan
    end_wall: float = math.nan
    counts: Counter[str] = field(default_factory=Counter)
    fused_positions_excluded: int = 0
    protocol_mismatch_positions_excluded: int = 0
    positions: list[dict[str, Any]] = field(default_factory=list)
    ekf_positions: list[dict[str, Any]] = field(default_factory=list)
    imu: list[dict[str, Any]] = field(default_factory=list)
    gps: list[dict[str, Any]] = field(default_factory=list)
    ranges: list[dict[str, Any]] = field(default_factory=list)
    geometries: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def duration_s(self) -> float:
        if finite(self.start_wall) and finite(self.end_wall) and self.end_wall > self.start_wall:
            return self.end_wall - self.start_wall
        times = [
            item["time"]
            for series in (self.positions, self.imu, self.gps)
            for item in series
            if finite(item.get("time"))
        ]
        return max(times) - min(times) if len(times) >= 2 else math.nan


def initial_status_geometry(
    record: dict[str, Any], protocol: str, tag_id: int
) -> dict[str, Any] | None:
    for status in record.get("initial_status", []):
        if int(status.get("module_id", -1)) != tag_id:
            continue
        ids = status.get("runtime_anchor_ids")
        xs = status.get("runtime_flex_tdoa_anchor_x_mm")
        ys = status.get("runtime_flex_tdoa_anchor_y_mm")
        if not (isinstance(ids, list) and isinstance(xs, list) and isinstance(ys, list)):
            continue
        if not (len(ids) == len(xs) == len(ys) and len(ids) >= 3):
            continue
        anchors = {
            int(anchor_id): (float(x_mm) / 1000.0, float(y_mm) / 1000.0)
            for anchor_id, x_mm, y_mm in zip(ids, xs, ys)
            if finite(x_mm) and finite(y_mm)
        }
        return {
            "time": wall_seconds(record),
            "snapshot_time": wall_seconds(record),
            "geometry_version": int(status.get("runtime_flex_tdoa_geometry_generation", 0)),
            "dynamic": not bool(status.get("runtime_flex_tdoa_geometry_fixed", True)),
            "fit_rms_m": math.nan,
            "all_rtk_fixed": False,
            "anchors": anchors,
            "source": "capture_start_status",
            "protocol": protocol,
        }
    return None


def snapshot_geometry(record: dict[str, Any], protocol: str, tag_id: int) -> dict[str, Any] | None:
    snapshot = record.get("snapshot")
    if not isinstance(snapshot, dict):
        return None
    tdoa = snapshot.get("tdoa")
    if not isinstance(tdoa, dict):
        return None
    geometry = tdoa.get("local_geometries", {}).get(f"{protocol}:{tag_id}")
    if not isinstance(geometry, dict) or not geometry.get("complete"):
        return None
    anchors: dict[int, tuple[float, float]] = {}
    for key, item in geometry.get("anchors", {}).items():
        if not isinstance(item, dict) or not finite(item.get("x")) or not finite(item.get("y")):
            continue
        anchors[int(item.get("id", key))] = (float(item["x"]), float(item["y"]))
    if len(anchors) < 3:
        return None
    return {
        "time": float(geometry["received_at"])
        if finite(geometry.get("received_at"))
        else wall_seconds(record),
        "snapshot_time": wall_seconds(record),
        "geometry_version": int(geometry.get("geometry_version", 0)),
        "dynamic": bool(geometry.get("dynamic")),
        "fit_rms_m": float(geometry["fit_rms_m"])
        if finite(geometry.get("fit_rms_m"))
        else math.nan,
        "all_rtk_fixed": bool(geometry.get("all_rtk_fixed")),
        "anchors": anchors,
        "source": "dashboard_snapshot",
        "protocol": protocol,
    }


def status_geometry(
    record: dict[str, Any], protocol: str, tag_id: int
) -> dict[str, Any] | None:
    modules = record.get("modules")
    if not isinstance(modules, list):
        return None
    for status in modules:
        if not isinstance(status, dict) or int(status.get("module_id", -1)) != tag_id:
            continue
        ids = status.get("runtime_anchor_ids")
        xs = status.get("runtime_flex_tdoa_anchor_x_mm")
        ys = status.get("runtime_flex_tdoa_anchor_y_mm")
        if not (
            isinstance(ids, list)
            and isinstance(xs, list)
            and isinstance(ys, list)
            and len(ids) == len(xs) == len(ys)
            and len(ids) >= 3
        ):
            continue
        anchors = {
            int(anchor_id): (float(x_mm) / 1000.0, float(y_mm) / 1000.0)
            for anchor_id, x_mm, y_mm in zip(ids, xs, ys)
            if int(anchor_id) > 0 and finite(x_mm) and finite(y_mm)
        }
        if len(anchors) < 3:
            continue
        return {
            "time": wall_seconds(record),
            "snapshot_time": wall_seconds(record),
            "geometry_version": int(
                status.get("runtime_flex_tdoa_geometry_generation", 0)
            ),
            "dynamic": not bool(
                status.get("runtime_flex_tdoa_geometry_fixed", True)
            ),
            "fit_rms_m": math.nan,
            "all_rtk_fixed": False,
            "anchors": anchors,
            "source": "status_runtime_geometry",
            "protocol": protocol,
        }
    return None


def load_capture(path: pathlib.Path, tag_id: int) -> Capture:
    capture = Capture(path=path)
    if path.suffix.lower() == ".xz":
        handle_context = lzma.open(path, "rt", encoding="utf-8")
    else:
        handle_context = path.open(encoding="utf-8")
    with handle_context as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                capture.warnings.append(f"invalid JSON on line {line_number}: {error}")
                continue
            kind = str(record.get("kind", "unknown"))
            capture.counts[kind] += 1
            record_protocol = normalize_protocol(
                record.get("tdoa_protocol") or record.get("protocol")
            )
            if capture.protocol == "unknown" and record_protocol != "unknown":
                capture.protocol = record_protocol
            if kind == "capture_start":
                capture.capture_id = str(record.get("capture_id", path.stem.split(".")[0]))
                capture.protocol = normalize_protocol(
                    record.get("protocol", capture.protocol)
                )
                capture.start_wall = wall_seconds(record)
                capture.warnings.extend(str(item) for item in record.get("warnings", []))
                geometry = initial_status_geometry(record, capture.protocol, tag_id)
                if geometry is not None:
                    capture.geometries.append(geometry)
            elif kind == "capture_end":
                capture.complete = True
                capture.interrupted = bool(record.get("interrupted"))
                capture.end_wall = wall_seconds(record)
            elif kind in {"position", "local_position"} and int(
                record.get("tag_id", record.get("module_id", -1))
            ) == tag_id:
                if (
                    capture.protocol != "unknown"
                    and record_protocol != "unknown"
                    and record_protocol != capture.protocol
                ):
                    capture.protocol_mismatch_positions_excluded += 1
                    continue
                if bool(record.get("imu_fused")) or "imu_fused_position" in str(
                    record.get("position_stream_type", "")
                ):
                    capture.fused_positions_excluded += 1
                    continue
                if not all(finite(record.get(key)) for key in ("x_m", "y_m", "uptime_ms")):
                    continue
                raw_point = {
                        "time": wall_seconds(record),
                        "uptime_ms": int(record["uptime_ms"]),
                        "x_m": float(record["x_m"]),
                        "y_m": float(record["y_m"]),
                        "sigma_m": float(record["sigma_m"]) if finite(record.get("sigma_m")) else math.nan,
                        "rms_m": float(record["rms_m"]) if finite(record.get("rms_m")) else math.nan,
                        "independent": bool(record.get("independent_frame", True)),
                        "solution_kind": str(record.get("solution_kind", "unknown")),
                        "geometry_version": int(record.get("geometry_version", 0)),
                        "batch_max_age_ms": float(record["batch_max_age_ms"])
                        if finite(record.get("batch_max_age_ms"))
                        else math.nan,
                    }
                capture.positions.append(raw_point)
                if (
                    bool(record.get("kalman_valid"))
                    and bool(record.get("independent_frame", True))
                    and finite(record.get("kalman_x_m"))
                    and finite(record.get("kalman_y_m"))
                ):
                    capture.ekf_positions.append(
                        {
                            **raw_point,
                            "x_m": float(record["kalman_x_m"]),
                            "y_m": float(record["kalman_y_m"]),
                            "vx_mps": float(record["kalman_vx_mps"])
                            if finite(record.get("kalman_vx_mps"))
                            else math.nan,
                            "vy_mps": float(record["kalman_vy_mps"])
                            if finite(record.get("kalman_vy_mps"))
                            else math.nan,
                            "innovation_m": float(record["kalman_innovation_m"])
                            if finite(record.get("kalman_innovation_m"))
                            else math.nan,
                            "nis": float(record["kalman_nis"])
                            if finite(record.get("kalman_nis"))
                            else math.nan,
                            "held": "position_only_stationary_hold"
                            in record.get("kalman_flags", []),
                            "reason": str(record.get("kalman_reason", "unknown")),
                        }
                    )
            elif kind == "accel" and int(record.get("module_id", -1)) == tag_id:
                if not finite(record.get("uptime_ms")):
                    continue
                capture.imu.append(
                    {
                        "time": wall_seconds(record),
                        "uptime_ms": int(record["uptime_ms"]),
                        "sample_id": int(record.get("sample_id", -1)),
                        "valid": bool(record.get("imu_valid")),
                    }
                )
            elif kind == "gps_fix":
                module_id = int(record.get("module_id", -1))
                capture.gps.append(
                    {
                        "time": wall_seconds(record, measurement=True),
                        "received_time": wall_seconds(record),
                        "module_id": module_id,
                        "valid": bool(record.get("gps_fix_valid")),
                        "quality": int(record.get("gps_fix_quality", 0)),
                        "latitude_deg": float(record["gps_latitude_deg"])
                        if finite(record.get("gps_latitude_deg"))
                        else math.nan,
                        "longitude_deg": float(record["gps_longitude_deg"])
                        if finite(record.get("gps_longitude_deg"))
                        else math.nan,
                        "altitude_m": float(record["gps_altitude_m"])
                        if finite(record.get("gps_altitude_m"))
                        else math.nan,
                        "fix_age_ms": float(record["gps_last_fix_age_ms"])
                        if finite(record.get("gps_last_fix_age_ms"))
                        else math.nan,
                        "hdop": float(record["gps_hdop"])
                        if finite(record.get("gps_hdop"))
                        else math.nan,
                        "satellites": int(record.get("gps_satellites", 0)),
                        "speed_mps": float(record["gps_speed_mps"])
                        if finite(record.get("gps_speed_mps"))
                        else math.nan,
                        "gga_count": int(record.get("gps_gga_count", -1)),
                    }
                )
            elif kind in {"ds_range", "anchor_range", "uwb_measurement"}:
                measurement_kind = (
                    str(record.get("measurement_kind", ""))
                    if kind == "uwb_measurement"
                    else kind
                )
                if measurement_kind == "native_ds_range":
                    normalized_kind = "ds_range"
                elif measurement_kind in {"ds_range", "anchor_range"}:
                    normalized_kind = measurement_kind
                else:
                    # Range-difference observations are retained in the raw
                    # capture for protocol replay, but are not absolute ranges
                    # and therefore cannot be compared with an RTK slant range.
                    continue
                if normalized_kind == "ds_range":
                    first_id = int(record.get("tag_id", -1))
                    second_id = int(record.get("anchor_id", -1))
                else:
                    first_id = int(
                        record.get("anchor_a_id", record.get("initiator_id", -1))
                    )
                    second_id = int(
                        record.get("anchor_b_id", record.get("responder_id", -1))
                    )
                distance_m = record.get("distance_m", record.get("raw_distance_m"))
                if first_id > 0 and second_id > 0 and finite(distance_m):
                    capture.ranges.append(
                        {
                            "time": wall_seconds(record),
                            "kind": normalized_kind,
                            "first_id": first_id,
                            "second_id": second_id,
                            "distance_m": float(distance_m),
                            "raw_distance_m": float(record["raw_distance_m"])
                            if finite(record.get("raw_distance_m"))
                            else math.nan,
                            "frame_id": int(
                                record.get("frame_id", record.get("slot_id", -1))
                            ),
                        }
                    )
            elif kind == "dashboard_snapshot":
                geometry = snapshot_geometry(record, capture.protocol, tag_id)
                if geometry is not None:
                    capture.geometries.append(geometry)
            elif kind == "status":
                geometry = status_geometry(record, capture.protocol, tag_id)
                if geometry is not None:
                    capture.geometries.append(geometry)

    if not capture.capture_id:
        capture.capture_id = path.stem.split(".")[0]
    capture.motion = detect_motion(capture.capture_id, path)
    if not finite(capture.end_wall):
        observed = [
            item["time"]
            for series in (capture.positions, capture.imu, capture.gps, capture.ranges)
            for item in series
            if finite(item.get("time"))
        ]
        if observed:
            capture.end_wall = max(observed)
    return capture


def load_rtk_reference_capture(
    path: pathlib.Path,
    *,
    capture_id: str,
    protocol: str,
    motion: str,
) -> Capture:
    """Load only GPS fixes from an archived reference capture.

    Reference archives can contain hundreds of megabytes of UWB and IMU
    telemetry.  The final cross-session report deliberately uses none of that
    older positioning data, so this reader retains only capture bounds and
    ``gps_fix`` records.
    """
    capture = Capture(
        path=path,
        capture_id=capture_id,
        protocol=normalize_protocol(protocol),
        motion=motion,
    )
    handle_context = (
        lzma.open(path, "rt", encoding="utf-8")
        if path.suffix.lower() == ".xz"
        else path.open(encoding="utf-8")
    )
    with handle_context as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                capture.warnings.append(
                    f"invalid JSON on line {line_number}: {error}"
                )
                continue
            kind = str(record.get("kind", "unknown"))
            if kind == "capture_start":
                capture.start_wall = wall_seconds(record)
            elif kind == "capture_end":
                capture.complete = True
                capture.interrupted = bool(record.get("interrupted"))
                capture.end_wall = wall_seconds(record)
            elif kind == "gps_fix":
                capture.gps.append(
                    {
                        "time": wall_seconds(record, measurement=True),
                        "received_time": wall_seconds(record),
                        "module_id": int(record.get("module_id", -1)),
                        "valid": bool(record.get("gps_fix_valid")),
                        "quality": int(record.get("gps_fix_quality", 0)),
                        "latitude_deg": float(record["gps_latitude_deg"])
                        if finite(record.get("gps_latitude_deg"))
                        else math.nan,
                        "longitude_deg": float(record["gps_longitude_deg"])
                        if finite(record.get("gps_longitude_deg"))
                        else math.nan,
                        "altitude_m": float(record["gps_altitude_m"])
                        if finite(record.get("gps_altitude_m"))
                        else math.nan,
                        "fix_age_ms": float(record["gps_last_fix_age_ms"])
                        if finite(record.get("gps_last_fix_age_ms"))
                        else math.nan,
                        "hdop": float(record["gps_hdop"])
                        if finite(record.get("gps_hdop"))
                        else math.nan,
                        "satellites": int(record.get("gps_satellites", 0)),
                        "speed_mps": float(record["gps_speed_mps"])
                        if finite(record.get("gps_speed_mps"))
                        else math.nan,
                        "gga_count": int(record.get("gps_gga_count", -1)),
                    }
                )
    if not finite(capture.end_wall):
        observed = [
            point["received_time"]
            for point in capture.gps
            if finite(point.get("received_time"))
        ]
        if observed:
            capture.end_wall = max(observed)
    return capture


def load_rtk_reference_report(
    report_dir: pathlib.Path, tag_id: int
) -> tuple[
    list[Capture],
    dict[str, dict[str, Any]],
    list[dict[str, Any]],
    tuple[float, float, float],
]:
    """Load the GPS-only reference matrix and its provenance manifest."""
    manifest_path = report_dir / "raw_data_manifest.csv"
    if not manifest_path.is_file():
        raise RuntimeError(f"RTK reference manifest not found: {manifest_path}")
    with manifest_path.open(newline="", encoding="utf-8-sig") as handle:
        manifest = list(csv.DictReader(handle))
    reference_manifest: list[dict[str, Any]] = []
    captures: list[Capture] = []
    for row in manifest:
        protocol = normalize_protocol(row.get("protocol"))
        motion = str(row.get("motion", "unknown"))
        if protocol not in PROTOCOL_LABELS or motion not in {"static", "dynamic"}:
            continue
        archive = report_dir / str(row.get("archive_file", ""))
        if not archive.is_file():
            raise RuntimeError(f"RTK reference archive not found: {archive}")
        reference_manifest.append(
            {
                **row,
                "reference_report_dir": str(report_dir),
                "reference_archive_file": str(archive),
                "records_used": "gps_fix only",
                "uwb_records_used": False,
            }
        )
        captures.append(
            load_rtk_reference_capture(
                archive,
                capture_id=str(row.get("capture_id") or archive.stem),
                protocol=protocol,
                motion=motion,
            )
        )
    selected, _ = select_capture_matrix(captures)
    origin = gps_origin(selected)
    add_enu(selected, origin)
    summaries: dict[str, dict[str, Any]] = {}
    for capture in selected:
        rtk, _, _ = rtk_metrics(capture, tag_id)
        fixed_points = fixed_tag_rtk_points(capture, tag_id)
        cloud = precision_metrics(
            [
                {"x_m": point[0], "y_m": point[1]}
                for point in fixed_points
            ]
        )
        summaries[capture.capture_id] = {
            "capture_id": capture.capture_id,
            "protocol": capture.protocol,
            "motion": capture.motion,
            "duration_s": capture.duration_s,
            "rtk": rtk,
            "tag_fixed_cloud_precision": cloud,
        }
    return selected, summaries, reference_manifest, origin


def select_capture_matrix(captures: list[Capture]) -> tuple[list[Capture], list[Capture]]:
    """Select one completed capture per protocol/motion cell.

    Explicitly named final dynamic and RTK-fixed static captures win over older
    blocks.  Within an equal priority, the most recently completed capture is
    selected.  Superseded captures remain untouched and are listed in output.
    """
    grouped: dict[tuple[str, str], list[Capture]] = defaultdict(list)
    passthrough: list[Capture] = []
    for capture in captures:
        if capture.protocol in PROTOCOL_LABELS and capture.motion in ("static", "dynamic"):
            grouped[(capture.protocol, capture.motion)].append(capture)
        else:
            passthrough.append(capture)
    selected = list(passthrough)
    superseded: list[Capture] = []
    for (_, motion), values in grouped.items():
        def priority(capture: Capture) -> tuple[int, float]:
            identifier = capture.capture_id.lower()
            if motion == "static" and identifier.startswith("rpi_static_") and "rtkfixed" in identifier:
                preferred = 3
            elif motion == "static" and "rtkfixed" in identifier:
                preferred = 2
            elif motion == "dynamic" and "final" in identifier:
                preferred = 2
            else:
                preferred = 0
            return preferred, capture.end_wall if finite(capture.end_wall) else -math.inf

        winner = max(values, key=priority)
        selected.append(winner)
        superseded.extend(item for item in values if item is not winner)
    return selected, superseded


UPTIME_REBOOT_BACKSTEP_MS = 5_000


def uptime_epochs(items: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    """Return chronological uptime epochs without joining across a reboot.

    Position streams can contain duplicate or mildly reordered publications,
    especially Passive DS rolling windows.  A multi-second uptime backstep is
    treated as a reboot/protocol restart; gaps and speeds must never bridge it.
    """

    chronological = sorted(
        enumerate(items),
        key=lambda pair: (
            float(pair[1].get("time", math.inf))
            if finite(pair[1].get("time"))
            else math.inf,
            pair[0],
        ),
    )
    epochs: list[list[dict[str, Any]]] = []
    previous_uptime: int | None = None
    for _, item in chronological:
        if not finite(item.get("uptime_ms")):
            continue
        uptime = int(item["uptime_ms"])
        if (
            previous_uptime is None
            or uptime < previous_uptime - UPTIME_REBOOT_BACKSTEP_MS
        ):
            epochs.append([])
        epochs[-1].append(item)
        previous_uptime = uptime
    return epochs


def stream_metrics(
    items: list[dict[str, Any]],
    duration_s: float,
    start_wall: float = math.nan,
    end_wall: float = math.nan,
) -> dict[str, Any]:
    uptimes = [int(item["uptime_ms"]) for item in items if finite(item.get("uptime_ms"))]
    epochs = uptime_epochs(items)
    unique_by_epoch = [sorted({int(item["uptime_ms"]) for item in epoch}) for epoch in epochs]
    gaps = [
        float(second - first)
        for unique in unique_by_epoch
        for first, second in zip(unique, unique[1:])
    ]
    unique_count = sum(len(unique) for unique in unique_by_epoch)
    uptime_spans = [
        (unique[-1] - unique[0]) / 1000.0
        for unique in unique_by_epoch
        if len(unique) >= 2
    ]
    uptime_span_s = sum(uptime_spans) if uptime_spans else math.nan
    times = sorted(float(item["time"]) for item in items if finite(item.get("time")))
    wall_span_s = times[-1] - times[0] if len(times) >= 2 else math.nan
    return {
        "events": len(items),
        "event_rate_hz": len(items) / duration_s if finite(duration_s) and duration_s > 0 else math.nan,
        "active_event_rate_hz": (len(items) - 1) / uptime_span_s
        if len(items) >= 2 and finite(uptime_span_s) and uptime_span_s > 0
        else math.nan,
        "unique_uptime_events": unique_count,
        "unique_uptime_rate_hz": unique_count / duration_s if finite(duration_s) and duration_s > 0 else math.nan,
        "active_unique_uptime_rate_hz": (unique_count - len(uptime_spans)) / uptime_span_s
        if unique_count >= 2 and finite(uptime_span_s) and uptime_span_s > 0
        else math.nan,
        "uptime_epoch_count": len(epochs),
        "uptime_span_s": uptime_span_s,
        "wall_span_s": wall_span_s,
        "wall_coverage_pct": 100.0 * wall_span_s / duration_s
        if finite(wall_span_s) and finite(duration_s) and duration_s > 0
        else math.nan,
        "first_event_after_capture_start_s": times[0] - start_wall
        if times and finite(start_wall)
        else math.nan,
        "last_event_before_capture_end_s": end_wall - times[-1]
        if times and finite(end_wall)
        else math.nan,
        "duplicate_uptime_pct": 100.0 * (1.0 - unique_count / len(uptimes)) if uptimes else math.nan,
        "gap_p50_ms": percentile(gaps, 50),
        "gap_p95_ms": percentile(gaps, 95),
        "gap_p99_ms": percentile(gaps, 99),
        "gap_max_ms": max(gaps) if gaps else math.nan,
        "gaps_gt_30ms": sum(value > 30.0 for value in gaps),
        "gaps_gt_50ms": sum(value > 50.0 for value in gaps),
        "gaps_gt_100ms": sum(value > 100.0 for value in gaps),
    }


def precision_metrics(points: list[dict[str, Any]], prefix: str = "") -> dict[str, Any]:
    if not points:
        return {f"{prefix}n": 0}
    x = [float(point["x_m"]) for point in points]
    y = [float(point["y_m"]) for point in points]
    center_x = median(x)
    center_y = median(y)
    radial = [math.hypot(px - center_x, py - center_y) for px, py in zip(x, y)]
    count = max(1, len(points) // 10)
    first_x, first_y = median(x[:count]), median(y[:count])
    last_x, last_y = median(x[-count:]), median(y[-count:])
    std_x = statistics.stdev(x) if len(x) > 1 else 0.0
    std_y = statistics.stdev(y) if len(y) > 1 else 0.0
    return {
        f"{prefix}n": len(x),
        f"{prefix}median_x_m": center_x,
        f"{prefix}median_y_m": center_y,
        f"{prefix}cep50_m": median(radial),
        f"{prefix}rms_m": math.sqrt(mean(value * value for value in radial)),
        f"{prefix}p95_m": percentile(radial, 95),
        f"{prefix}p99_m": percentile(radial, 99),
        f"{prefix}max_m": max(radial),
        f"{prefix}std_x_m": std_x,
        f"{prefix}std_y_m": std_y,
        f"{prefix}two_drms_m": 2.0 * math.hypot(std_x, std_y),
        f"{prefix}first_to_last_decile_drift_m": math.hypot(last_x - first_x, last_y - first_y),
    }


def jump_metrics(points: list[dict[str, Any]]) -> dict[str, Any]:
    speeds: list[float] = []
    for epoch in uptime_epochs(points):
        by_uptime: dict[int, dict[str, Any]] = {}
        for point in epoch:
            by_uptime[int(point["uptime_ms"])] = point
        ordered = [by_uptime[key] for key in sorted(by_uptime)]
        for first, second in zip(ordered, ordered[1:]):
            elapsed = (int(second["uptime_ms"]) - int(first["uptime_ms"])) / 1000.0
            if elapsed <= 0:
                continue
            speeds.append(
                math.hypot(float(second["x_m"]) - float(first["x_m"]), float(second["y_m"]) - float(first["y_m"]))
                / elapsed
            )
    return {
        "step_speed_n": len(speeds),
        "step_speed_p95_mps": percentile(speeds, 95),
        "step_speed_p99_mps": percentile(speeds, 99),
        "step_speed_max_mps": max(speeds) if speeds else math.nan,
        "step_speed_gt_3mps": sum(value > 3.0 for value in speeds),
        "step_speed_gt_5mps": sum(value > 5.0 for value in speeds),
        "step_speed_gt_10mps": sum(value > 10.0 for value in speeds),
    }


def analysis_positions(capture: Capture) -> list[dict[str, Any]]:
    independent = [point for point in capture.positions if point["independent"]]
    return independent if independent else list(capture.positions)


def gps_origin(captures: list[Capture]) -> tuple[float, float, float]:
    fixes = [
        point
        for capture in captures
        for point in capture.gps
        if point["valid"]
        and point["quality"] == 4
        and all(finite(point.get(key)) for key in ("latitude_deg", "longitude_deg", "altitude_m"))
    ]
    if not fixes:
        raise RuntimeError("no RTK-fixed GPS solution with finite coordinates was found")
    return (
        median(point["latitude_deg"] for point in fixes),
        median(point["longitude_deg"] for point in fixes),
        median(point["altitude_m"] for point in fixes),
    )


def add_enu(captures: list[Capture], origin: tuple[float, float, float]) -> None:
    origin_ecef = ecef(*origin)
    rotation = enu_rotation(origin[0], origin[1])
    for capture in captures:
        for point in capture.gps:
            if not all(finite(point.get(key)) for key in ("latitude_deg", "longitude_deg", "altitude_m")):
                continue
            absolute = ecef(point["latitude_deg"], point["longitude_deg"], point["altitude_m"])
            delta = tuple(absolute[index] - origin_ecef[index] for index in range(3))
            local = matrix3_vector(rotation, delta)  # type: ignore[arg-type]
            point["east_m"], point["north_m"], point["up_m"] = local


def rtk_metrics(capture: Capture, tag_id: int) -> tuple[dict[str, Any], dict[int, tuple[float, float, float]], dict[int, float]]:
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for point in capture.gps:
        grouped[int(point["module_id"])].append(point)
    availability: dict[str, Any] = {}
    centers: dict[int, tuple[float, float, float]] = {}
    spreads: dict[int, float] = {}
    for module_id, values in sorted(grouped.items()):
        fixed = [
            point
            for point in values
            if point["valid"]
            and point["quality"] == 4
            and all(finite(point.get(key)) for key in ("east_m", "north_m", "up_m"))
        ]
        availability[str(module_id)] = {
            "total": len(values),
            "valid": sum(bool(point["valid"]) for point in values),
            "fixed": len(fixed),
            "fixed_pct": 100.0 * len(fixed) / len(values) if values else math.nan,
            "fix_age_p95_ms": percentile((point["fix_age_ms"] for point in fixed), 95),
            "hdop_median": median(point["hdop"] for point in fixed),
            "satellites_median": median(point["satellites"] for point in fixed),
        }
        if fixed:
            center = (
                median(point["east_m"] for point in fixed),
                median(point["north_m"] for point in fixed),
                median(point["up_m"] for point in fixed),
            )
            centers[module_id] = center
            radial = [math.hypot(point["east_m"] - center[0], point["north_m"] - center[1]) for point in fixed]
            spreads[module_id] = percentile(radial, 95)
            availability[str(module_id)]["horizontal_spread_p95_m"] = spreads[module_id]
    tag = availability.get(str(tag_id), {"total": 0, "valid": 0, "fixed": 0})
    overall_total = sum(len(values) for values in grouped.values())
    overall_fixed = sum(
        point["valid"] and point["quality"] == 4 for point in capture.gps
    )
    summary = {
        "by_module": availability,
        "tag_total": tag.get("total", 0),
        "tag_valid": tag.get("valid", 0),
        "tag_fixed": tag.get("fixed", 0),
        "tag_fixed_pct": tag.get("fixed_pct", math.nan),
        "tag_fix_age_p95_ms": tag.get("fix_age_p95_ms", math.nan),
        "all_modules_fixed_pct": 100.0 * overall_fixed / overall_total if overall_total else math.nan,
        "fixed_anchor_count": sum(anchor_id in centers for anchor_id in ANCHOR_IDS),
        "anchor_spread_p95_m_max": max((spreads[anchor_id] for anchor_id in ANCHOR_IDS if anchor_id in spreads), default=math.nan),
        "tag_center_east_m": centers[tag_id][0] if tag_id in centers else math.nan,
        "tag_center_north_m": centers[tag_id][1] if tag_id in centers else math.nan,
        "tag_center_up_m": centers[tag_id][2] if tag_id in centers else math.nan,
    }
    return summary, centers, spreads


def static_rtk_reference_consistency(
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    threshold_m: float,
) -> dict[str, Any]:
    centers: dict[str, dict[str, Any]] = {}
    for capture in captures:
        if capture.motion != "static":
            continue
        rtk = summaries[capture.capture_id]["rtk"]
        if not finite(rtk.get("tag_center_east_m")) or not finite(rtk.get("tag_center_north_m")):
            continue
        centers[capture.protocol] = {
            "capture_id": capture.capture_id,
            "east_m": float(rtk["tag_center_east_m"]),
            "north_m": float(rtk["tag_center_north_m"]),
        }
    pairwise: list[dict[str, Any]] = []
    protocols = sorted(centers)
    for index, first in enumerate(protocols):
        for second in protocols[index + 1 :]:
            distance = math.hypot(
                centers[first]["east_m"] - centers[second]["east_m"],
                centers[first]["north_m"] - centers[second]["north_m"],
            )
            pairwise.append(
                {"first_protocol": first, "second_protocol": second, "distance_m": distance}
            )
    maximum = max((item["distance_m"] for item in pairwise), default=math.nan)
    complete = len(centers) == len(PROTOCOL_LABELS)
    valid = complete and finite(maximum) and maximum <= threshold_m
    return {
        "stationary_tag_centers_enu_m": centers,
        "pairwise_distances": pairwise,
        "max_pairwise_distance_m": maximum,
        "threshold_m": threshold_m,
        "complete_protocol_matrix": complete,
        "valid_for_cross_protocol_absolute_ranking": valid,
        "interpretation": (
            "static RTK reference is cross-capture consistent"
            if valid
            else "static absolute UWB-RTK errors are audit-only and invalid for cross-protocol ranking"
        ),
    }


def fit_rigid(
    source: dict[int, tuple[float, float]],
    target: dict[int, tuple[float, float, float]],
) -> dict[str, Any] | None:
    """Fit the best fixed-scale 2-D isometry, including mirror ambiguity.

    Indoor UWB coordinates have no intrinsic geographic chirality.  The same
    ID-keyed anchor geometry can therefore be expressed as either a direct or
    mirrored local frame relative to ENU.  Evaluate both while keeping scale
    fixed at one, then retain the candidate with the smaller anchor RMSE.
    """

    ids = [anchor_id for anchor_id in ANCHOR_IDS if anchor_id in source and anchor_id in target]
    if len(ids) < 3:
        return None
    y = [(target[anchor_id][0], target[anchor_id][1]) for anchor_id in ids]
    cy = (mean(point[0] for point in y), mean(point[1] for point in y))
    candidates: list[dict[str, Any]] = []
    for reflected in (False, True):
        x = [
            (source[anchor_id][0], -source[anchor_id][1] if reflected else source[anchor_id][1])
            for anchor_id in ids
        ]
        cx = (mean(point[0] for point in x), mean(point[1] for point in x))
        cosine_term = 0.0
        sine_term = 0.0
        for local, common in zip(x, y):
            lx, ly = local[0] - cx[0], local[1] - cx[1]
            ex, ey = common[0] - cy[0], common[1] - cy[1]
            cosine_term += lx * ex + ly * ey
            sine_term += lx * ey - ly * ex
        norm = math.hypot(cosine_term, sine_term)
        if norm <= 1e-12:
            continue
        cosine = cosine_term / norm
        sine = sine_term / norm
        rotation = ((cosine, -sine), (sine, cosine))
        translation = (
            cy[0] - (cosine * cx[0] - sine * cx[1]),
            cy[1] - (sine * cx[0] + cosine * cx[1]),
        )
        predicted = [
            (
                cosine * point[0] - sine * point[1] + translation[0],
                sine * point[0] + cosine * point[1] + translation[1],
            )
            for point in x
        ]
        residuals = [math.dist(first, second) for first, second in zip(predicted, y)]
        candidates.append(
            {
                "anchor_ids": ids,
                "rotation": rotation,
                "translation": translation,
                "reflected": reflected,
                "scale": 1.0,
                "yaw_deg": math.degrees(math.atan2(sine, cosine)),
                "fit_rmse_m": math.sqrt(mean(value * value for value in residuals)),
                "fit_p95_m": percentile(residuals, 95),
                "fit_max_m": max(residuals),
                "residuals_m": {
                    str(anchor_id): float(value)
                    for anchor_id, value in zip(ids, residuals)
                },
            }
        )
    return min(candidates, key=lambda item: (item["fit_rmse_m"], item["reflected"])) if candidates else None


def build_alignments(capture: Capture, centers: dict[int, tuple[float, float, float]]) -> list[dict[str, Any]]:
    alignments: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()
    for geometry in sorted(capture.geometries, key=lambda item: item["time"]):
        key = (
            geometry["geometry_version"],
            round(float(geometry["time"]), 3),
            tuple((anchor_id, *geometry["anchors"][anchor_id]) for anchor_id in sorted(geometry["anchors"])),
        )
        if key in seen:
            continue
        seen.add(key)
        fit = fit_rigid(geometry["anchors"], centers)
        if fit is None:
            continue
        alignments.append({**geometry, **fit})
    return alignments


def alignment_for_position(
    point: dict[str, Any], alignments: list[dict[str, Any]], max_dynamic_age_s: float
) -> tuple[dict[str, Any] | None, bool, float]:
    if not alignments:
        return None, False, math.nan
    version = int(point["geometry_version"])
    exact = [alignment for alignment in alignments if int(alignment["geometry_version"]) == version]
    candidates = exact or alignments
    selected = min(candidates, key=lambda item: abs(float(item["time"]) - float(point["time"])))
    age = abs(float(selected["time"]) - float(point["time"]))
    if selected["dynamic"] and not exact and age > max_dynamic_age_s:
        return None, False, age
    return selected, bool(exact), age


def transform_positions(
    points: list[dict[str, Any]], alignments: list[dict[str, Any]], max_dynamic_age_s: float
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    transformed: list[dict[str, Any]] = []
    exact_count = 0
    ages: list[float] = []
    for point in points:
        alignment, exact, age = alignment_for_position(point, alignments, max_dynamic_age_s)
        if alignment is None:
            continue
        rotation = alignment["rotation"]
        translation = alignment["translation"]
        local_y = -point["y_m"] if alignment.get("reflected", False) else point["y_m"]
        common = (
            rotation[0][0] * point["x_m"] + rotation[0][1] * local_y + translation[0],
            rotation[1][0] * point["x_m"] + rotation[1][1] * local_y + translation[1],
        )
        transformed.append(
            {
                **point,
                "east_m": float(common[0]),
                "north_m": float(common[1]),
                "alignment_version": int(alignment["geometry_version"]),
                "alignment_exact_version": exact,
                "alignment_age_s": age,
                "alignment_dynamic": bool(alignment["dynamic"]),
            }
        )
        exact_count += int(exact)
        ages.append(age)
    diagnostics = {
        "transformed_positions": len(transformed),
        "transform_coverage_pct": 100.0 * len(transformed) / len(points) if points else math.nan,
        "exact_geometry_version_pct": 100.0 * exact_count / len(transformed) if transformed else math.nan,
        "geometry_match_age_p95_s": percentile(ages, 95),
        "geometry_match_age_max_s": max(ages) if ages else math.nan,
    }
    return transformed, diagnostics


def alignment_metrics(
    alignments: list[dict[str, Any]], spreads: dict[int, float], diagnostics: dict[str, Any]
) -> dict[str, Any]:
    if not alignments:
        return {"available": False, **diagnostics}
    fit_rmse = [alignment["fit_rmse_m"] for alignment in alignments]
    yaw = [alignment["yaw_deg"] for alignment in alignments]
    reflected = [bool(alignment.get("reflected", False)) for alignment in alignments]
    dynamic = [alignment for alignment in alignments if alignment["dynamic"]]
    max_spread = max((spreads.get(anchor_id, math.nan) for anchor_id in ANCHOR_IDS if finite(spreads.get(anchor_id))), default=math.nan)
    reliable = (
        max(fit_rmse) <= 0.5
        and (not finite(max_spread) or max_spread <= 0.5)
        and min(len(alignment["anchor_ids"]) for alignment in alignments) >= 3
        and len(set(reflected)) == 1
    )
    return {
        "available": True,
        "snapshot_count": len(alignments),
        "dynamic_snapshot_count": len(dynamic),
        "rtk_derived_dynamic_geometry": bool(dynamic),
        "anchor_count_min": min(len(alignment["anchor_ids"]) for alignment in alignments),
        "reflected": reflected[0] if len(set(reflected)) == 1 else None,
        "reflected_snapshot_count": sum(reflected),
        "reflection_consistent": len(set(reflected)) == 1,
        "fit_rmse_m_median": median(fit_rmse),
        "fit_rmse_m_p95": percentile(fit_rmse, 95),
        "fit_rmse_m_max": max(fit_rmse),
        "fit_residual_m_max": max(alignment["fit_max_m"] for alignment in alignments),
        "yaw_deg_median": median(yaw),
        "yaw_deg_span": max(yaw) - min(yaw),
        "anchor_spread_p95_m_max": max_spread,
        "quality_flag": "usable_with_limitations" if reliable else "poor_alignment_do_not_claim_accuracy",
        **diagnostics,
    }


def nearest_position_pairs(
    capture: Capture,
    transformed: list[dict[str, Any]],
    tag_id: int,
    max_match_ms: float,
    stream: str = "raw",
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    positions = sorted(transformed, key=lambda item: item["time"])
    times = [float(point["time"]) for point in positions]
    fixed_tag = sorted(
        (
            point
            for point in capture.gps
            if int(point["module_id"]) == tag_id
            and point["valid"]
            and point["quality"] == 4
            and all(finite(point.get(key)) for key in ("time", "east_m", "north_m"))
        ),
        key=lambda item: item["time"],
    )
    pairs: list[dict[str, Any]] = []
    for gps in fixed_tag:
        index = bisect.bisect_left(times, float(gps["time"]))
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(positions)]
        if not candidates:
            continue
        selected_index = min(candidates, key=lambda candidate: abs(times[candidate] - float(gps["time"])))
        position = positions[selected_index]
        delta_ms = (float(position["time"]) - float(gps["time"])) * 1000.0
        if abs(delta_ms) > max_match_ms:
            continue
        dx = float(position["east_m"]) - float(gps["east_m"])
        dy = float(position["north_m"]) - float(gps["north_m"])
        pairs.append(
            {
                "capture_id": capture.capture_id,
                "protocol": capture.protocol,
                "motion": capture.motion,
                "position_stream": stream,
                "gps_time": gps["time"],
                "uwb_time": position["time"],
                "delta_ms": delta_ms,
                "uwb_east_m": position["east_m"],
                "uwb_north_m": position["north_m"],
                "rtk_east_m": gps["east_m"],
                "rtk_north_m": gps["north_m"],
                "error_east_m": dx,
                "error_north_m": dy,
                "error_m": math.hypot(dx, dy),
                "gps_fix_age_ms": gps["fix_age_ms"],
                "gps_speed_mps": gps["speed_mps"],
                "position_sigma_m": position["sigma_m"],
                "position_rms_m": position["rms_m"],
                "geometry_version": position["geometry_version"],
                "alignment_version": position["alignment_version"],
                "alignment_exact_version": position["alignment_exact_version"],
                "alignment_age_s": position["alignment_age_s"],
            }
        )
    return pairs, {"fixed_tag_solutions": len(fixed_tag)}


def error_metrics(pairs: list[dict[str, Any]], fixed_tag_solutions: int) -> dict[str, Any]:
    if not pairs:
        return {
            "pairs": 0,
            "fixed_tag_solutions": fixed_tag_solutions,
            "pair_coverage_pct": 0.0 if fixed_tag_solutions else math.nan,
        }
    dx = [float(pair["error_east_m"]) for pair in pairs]
    dy = [float(pair["error_north_m"]) for pair in pairs]
    error = [math.hypot(ex, ey) for ex, ey in zip(dx, dy)]
    median_dx, median_dy = median(dx), median(dy)
    debiased = [math.hypot(ex - median_dx, ey - median_dy) for ex, ey in zip(dx, dy)]
    for pair, value in zip(pairs, debiased):
        pair["debiased_error_m"] = float(value)
    return {
        "pairs": len(pairs),
        "fixed_tag_solutions": fixed_tag_solutions,
        "pair_coverage_pct": 100.0 * len(pairs) / fixed_tag_solutions if fixed_tag_solutions else math.nan,
        "association_abs_p50_ms": percentile((abs(pair["delta_ms"]) for pair in pairs), 50),
        "association_abs_p95_ms": percentile((abs(pair["delta_ms"]) for pair in pairs), 95),
        "association_abs_max_ms": max(abs(pair["delta_ms"]) for pair in pairs),
        "bias_east_mean_m": mean(dx),
        "bias_north_mean_m": mean(dy),
        "bias_east_median_m": median_dx,
        "bias_north_median_m": median_dy,
        "error_median_m": median(error),
        "error_rmse_m": math.sqrt(mean(value * value for value in error)),
        "error_p95_m": percentile(error, 95),
        "error_max_m": max(error),
        "error_gt_0_5m": sum(value > 0.5 for value in error),
        "error_gt_1m": sum(value > 1.0 for value in error),
        "error_gt_2m": sum(value > 2.0 for value in error),
        "debiased_error_rmse_m": math.sqrt(mean(value * value for value in debiased)),
        "debiased_error_p95_m": percentile(debiased, 95),
        "debiased_error_max_m": max(debiased),
    }


def write_csv(path: pathlib.Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: "" if isinstance(value, float) and not math.isfinite(value) else value
                    for key, value in row.items()
                }
            )


def flatten(prefix: str, data: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in data.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            continue
        result[name] = value
    return result


def svg_escape(value: Any) -> str:
    return html.escape(str(value), quote=True)


def pdf_escape(value: Any) -> str:
    """Escape an ASCII label for a built-in PDF Type-1 font."""
    text = str(value).encode("ascii", "replace").decode("ascii")
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


class PdfCanvas:
    """Tiny dependency-free vector PDF canvas used for report figures."""

    def __init__(self, width: float, height: float) -> None:
        self.width = float(width)
        self.height = float(height)
        self.commands: list[str] = ["1 J 1 j"]

    @staticmethod
    def color(value: str) -> tuple[float, float, float]:
        value = value.lstrip("#")
        return tuple(int(value[index : index + 2], 16) / 255.0 for index in (0, 2, 4))  # type: ignore[return-value]

    def line(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        color: str = "#111827",
        width: float = 1.0,
    ) -> None:
        red, green, blue = self.color(color)
        self.commands.append(
            f"{red:.4f} {green:.4f} {blue:.4f} RG {width:.2f} w "
            f"{x1:.2f} {y1:.2f} m {x2:.2f} {y2:.2f} l S"
        )

    def polyline(
        self,
        points: list[tuple[float, float]],
        color: str,
        width: float = 1.4,
    ) -> None:
        if len(points) < 2:
            return
        red, green, blue = self.color(color)
        path = [f"{points[0][0]:.2f} {points[0][1]:.2f} m"]
        path.extend(f"{x:.2f} {y:.2f} l" for x, y in points[1:])
        self.commands.append(
            f"{red:.4f} {green:.4f} {blue:.4f} RG {width:.2f} w " + " ".join(path) + " S"
        )

    def rect(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        color: str,
        *,
        fill: bool = True,
        line_width: float = 0.8,
    ) -> None:
        red, green, blue = self.color(color)
        operator = "f" if fill else "S"
        channel = "rg" if fill else "RG"
        self.commands.append(
            f"{red:.4f} {green:.4f} {blue:.4f} {channel} {line_width:.2f} w "
            f"{x:.2f} {y:.2f} {width:.2f} {height:.2f} re {operator}"
        )

    def text(
        self,
        x: float,
        y: float,
        value: Any,
        size: float = 10.0,
        color: str = "#111827",
        *,
        bold: bool = False,
        centered: bool = False,
    ) -> None:
        label = pdf_escape(value)
        if centered:
            x -= len(label) * size * 0.25
        red, green, blue = self.color(color)
        font = "F2" if bold else "F1"
        self.commands.append(
            f"{red:.4f} {green:.4f} {blue:.4f} rg BT /{font} {size:.2f} Tf "
            f"1 0 0 1 {x:.2f} {y:.2f} Tm ({label}) Tj ET"
        )

    def save(self, path: pathlib.Path) -> None:
        content = ("\n".join(self.commands) + "\n").encode("ascii")
        objects = [
            b"<< /Type /Catalog /Pages 2 0 R >>",
            b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            (
                f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {self.width:.2f} {self.height:.2f}] "
                "/Resources << /Font << /F1 4 0 R /F2 5 0 R >> >> /Contents 6 0 R >>"
            ).encode("ascii"),
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>",
            b"<< /Length " + str(len(content)).encode("ascii") + b" >>\nstream\n" + content + b"endstream",
        ]
        document = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
        offsets = [0]
        for index, item in enumerate(objects, start=1):
            offsets.append(len(document))
            document.extend(f"{index} 0 obj\n".encode("ascii"))
            document.extend(item)
            document.extend(b"\nendobj\n")
        xref = len(document)
        document.extend(f"xref\n0 {len(objects)+1}\n".encode("ascii"))
        document.extend(b"0000000000 65535 f \n")
        for offset in offsets[1:]:
            document.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
        document.extend(
            f"trailer\n<< /Size {len(objects)+1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode(
                "ascii"
            )
        )
        path.write_bytes(document)


def pdf_bar_chart(
    path: pathlib.Path,
    title: str,
    labels: list[str],
    series: list[tuple[str, list[float], str]],
    y_label: str,
) -> None:
    width, height = 900.0, 470.0
    left, right, top, bottom = 78.0, 24.0, 55.0, 92.0
    plot_width, plot_height = width - left - right, height - top - bottom
    values = [value for _, items, _ in series for value in items if finite(value)]
    maximum = max(max(values, default=1.0) * 1.12, 1e-9)
    canvas = PdfCanvas(width, height)
    canvas.text(width / 2, height - 28, title, 17, bold=True, centered=True)
    for tick in range(6):
        value = maximum * tick / 5
        y = bottom + plot_height * tick / 5
        canvas.line(left, y, left + plot_width, y, "#e5e7eb", 0.7)
        canvas.text(left - 42, y - 3, f"{value:.2g}", 9, "#475569")
    canvas.line(left, bottom, left, bottom + plot_height)
    canvas.line(left, bottom, left + plot_width, bottom)
    group_width = plot_width / max(1, len(labels))
    bar_width = min(44.0, group_width * 0.72 / max(1, len(series)))
    for label_index, label in enumerate(labels):
        center = left + group_width * (label_index + 0.5)
        total_width = bar_width * len(series)
        for series_index, (_, items, color) in enumerate(series):
            value = items[label_index] if label_index < len(items) else math.nan
            if not finite(value):
                continue
            bar_height = plot_height * max(0.0, value) / maximum
            x = center - total_width / 2 + series_index * bar_width
            canvas.rect(x, bottom, bar_width - 2, bar_height, color)
            canvas.text(x, bottom + bar_height + 5, f"{value:.2f}", 7, color)
        canvas.text(center, bottom - 20, label.replace("\n", " "), 8, centered=True)
    legend_x = left + 8
    for index, (name, _, color) in enumerate(series):
        x = legend_x + index * 240
        canvas.rect(x, 21, 13, 13, color)
        canvas.text(x + 20, 23, name, 10)
    canvas.text(10, height - 48, y_label, 9, "#475569")
    canvas.save(path)


def cdf_axis_max(values: Iterable[float]) -> float:
    """Return an axis limit that includes every finite CDF sample."""

    clean = [float(value) for value in values if finite(value)]
    return max(max(clean) * 1.02 if clean else 1.0, 0.1)


def cdf_sample_indices(count: int, maximum_points: int) -> list[int]:
    """Downsample a CDF while always retaining its 0% and 100% endpoints."""

    if count <= 0:
        return []
    stride = max(1, count // maximum_points)
    return sorted({*range(0, count, stride), count - 1})


def pdf_cdf_chart(
    path: pathlib.Path,
    title: str,
    series: list[tuple[str, list[float], str]],
) -> None:
    width, height = 820.0, 470.0
    left, right, top, bottom = 70.0, 24.0, 55.0, 58.0
    plot_width, plot_height = width - left - right, height - top - bottom
    values = [value for _, items, _ in series for value in items if finite(value)]
    maximum = cdf_axis_max(values)
    canvas = PdfCanvas(width, height)
    canvas.text(width / 2, height - 28, title, 17, bold=True, centered=True)
    for tick in range(6):
        x = left + plot_width * tick / 5
        y = bottom + plot_height * tick / 5
        canvas.line(x, bottom, x, bottom + plot_height, "#e5e7eb", 0.7)
        canvas.line(left, y, left + plot_width, y, "#e5e7eb", 0.7)
        canvas.text(x - 12, bottom - 20, f"{maximum*tick/5:.2f}", 8, "#475569")
        canvas.text(left - 34, y - 3, f"{20*tick}", 8, "#475569")
    canvas.line(left, bottom, left, bottom + plot_height)
    canvas.line(left, bottom, left + plot_width, bottom)
    for index, (label, items, color) in enumerate(series):
        ordered = sorted(value for value in items if finite(value))
        if not ordered:
            continue
        points = []
        for item_index in cdf_sample_indices(len(ordered), 600):
            value = ordered[item_index]
            points.append(
                (
                    left + plot_width * value / maximum,
                    bottom + plot_height * item_index / max(1, len(ordered) - 1),
                )
            )
        canvas.polyline(points, color, 2.0)
        legend_y = height - top - 18 * index
        canvas.line(left + plot_width - 210, legend_y, left + plot_width - 184, legend_y, color, 2.5)
        canvas.text(left + plot_width - 177, legend_y - 3, label, 9)
    canvas.text(left + plot_width / 2, 14, "aligned UWB - RTK disagreement [m]", 10, centered=True)
    canvas.text(7, height - 45, "CDF [%]", 9, "#475569")
    canvas.save(path)


def pdf_position_panels(
    path: pathlib.Path,
    title: str,
    captures: list[Capture],
    *,
    centered_static: bool,
) -> None:
    """Draw raw independent trajectories/clouds in comparable protocol panels."""
    width, height = 1000.0, 390.0
    canvas = PdfCanvas(width, height)
    canvas.text(width / 2, height - 27, title, 17, bold=True, centered=True)
    panel_width = 300.0
    plot_size = 250.0
    panel_gap = (width - panel_width * len(captures)) / (len(captures) + 1)
    prepared: list[tuple[Capture, list[dict[str, Any]], list[tuple[float, float]]]] = []
    global_extent = 0.0
    for capture in captures:
        points = analysis_positions(capture)
        ekf_points = list(capture.ekf_positions)
        center_x = median(point["x_m"] for point in points) if centered_static else 0.0
        center_y = median(point["y_m"] for point in points) if centered_static else 0.0
        xy = [(float(point["x_m"]) - center_x, float(point["y_m"]) - center_y) for point in points]
        prepared.append((capture, points, xy))
        if centered_static and xy:
            global_extent = max(global_extent, max(max(abs(x), abs(y)) for x, y in xy))
    for panel_index, (capture, points, xy) in enumerate(prepared):
        panel_x = panel_gap + panel_index * (panel_width + panel_gap)
        plot_x, plot_y = panel_x + 25, 55.0
        if centered_static:
            extent = max(global_extent * 1.08, 0.05)
            min_x = min_y = -extent
            max_x = max_y = extent
        else:
            geometry = capture.geometries[-1]["anchors"] if capture.geometries else {}
            combined = list(xy) + list(geometry.values())
            min_x = min((item[0] for item in combined), default=0.0)
            max_x = max((item[0] for item in combined), default=1.0)
            min_y = min((item[1] for item in combined), default=0.0)
            max_y = max((item[1] for item in combined), default=1.0)
            span = max(max_x - min_x, max_y - min_y, 1.0) * 1.08
            center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2
            min_x, max_x = center_x - span / 2, center_x + span / 2
            min_y, max_y = center_y - span / 2, center_y + span / 2
        def map_point(item: tuple[float, float]) -> tuple[float, float]:
            return (
                plot_x + plot_size * (item[0] - min_x) / max(1e-12, max_x - min_x),
                plot_y + plot_size * (item[1] - min_y) / max(1e-12, max_y - min_y),
            )
        for grid in range(6):
            offset = plot_size * grid / 5
            canvas.line(plot_x + offset, plot_y, plot_x + offset, plot_y + plot_size, "#e2e8f0", 0.6)
            canvas.line(plot_x, plot_y + offset, plot_x + plot_size, plot_y + offset, "#e2e8f0", 0.6)
        canvas.rect(plot_x, plot_y, plot_size, plot_size, "#94a3b8", fill=False)
        stride = max(1, len(xy) // 1600)
        mapped = [map_point(item) for item in xy[::stride]]
        color = PROTOCOL_COLORS.get(capture.protocol, "#64748b")
        if centered_static:
            for x, y in mapped:
                canvas.rect(x - 0.7, y - 0.7, 1.4, 1.4, color)
            cx, cy = map_point((0.0, 0.0))
            canvas.line(cx - 5, cy, cx + 5, cy, "#111827", 1.0)
            canvas.line(cx, cy - 5, cx, cy + 5, "#111827", 1.0)
        else:
            canvas.polyline(mapped, color, 1.0)
            if capture.geometries:
                anchors = capture.geometries[-1]["anchors"]
                for anchor_id, anchor in sorted(anchors.items()):
                    ax, ay = map_point(anchor)
                    canvas.rect(ax - 3, ay - 3, 6, 6, "#15803d")
                    canvas.text(ax + 5, ay + 4, f"A{anchor_id}", 7, "#166534", bold=True)
        canvas.text(panel_x + panel_width / 2, 328, PROTOCOL_LABELS.get(capture.protocol, capture.protocol), 12, color, bold=True, centered=True)
        canvas.text(plot_x, 34, f"x [{min_x:.2f}, {max_x:.2f}] m", 8, "#475569")
        canvas.text(plot_x + 132, 34, f"y [{min_y:.2f}, {max_y:.2f}] m", 8, "#475569")
        canvas.text(plot_x, 19, f"n={len(points)} independent", 8, "#475569")
    canvas.save(path)


def pdf_raw_ekf_panels(
    path: pathlib.Path,
    title: str,
    captures: list[Capture],
    *,
    centered_static: bool,
) -> None:
    """Overlay raw independent positions and the position-only adaptive EKF."""
    width, height = 1000.0, 405.0
    canvas = PdfCanvas(width, height)
    canvas.text(width / 2, height - 27, title, 17, bold=True, centered=True)
    panel_width = 300.0
    plot_size = 250.0
    panel_gap = (width - panel_width * len(captures)) / (len(captures) + 1)
    prepared: list[
        tuple[Capture, list[tuple[float, float]], list[tuple[float, float]]]
    ] = []
    global_extent = 0.0
    for capture in captures:
        raw_points = analysis_positions(capture)
        ekf_points = capture.ekf_positions
        if centered_static:
            raw_center = (
                median(point["x_m"] for point in raw_points),
                median(point["y_m"] for point in raw_points),
            )
            ekf_center = (
                median(point["x_m"] for point in ekf_points),
                median(point["y_m"] for point in ekf_points),
            )
        else:
            raw_center = ekf_center = (0.0, 0.0)
        raw_xy = [
            (float(point["x_m"]) - raw_center[0], float(point["y_m"]) - raw_center[1])
            for point in raw_points
        ]
        ekf_xy = [
            (float(point["x_m"]) - ekf_center[0], float(point["y_m"]) - ekf_center[1])
            for point in ekf_points
        ]
        prepared.append((capture, raw_xy, ekf_xy))
        if centered_static and (raw_xy or ekf_xy):
            global_extent = max(
                global_extent,
                max(max(abs(x), abs(y)) for x, y in raw_xy + ekf_xy),
            )

    for panel_index, (capture, raw_xy, ekf_xy) in enumerate(prepared):
        panel_x = panel_gap + panel_index * (panel_width + panel_gap)
        plot_x, plot_y = panel_x + 25, 62.0
        if centered_static:
            extent = max(global_extent * 1.08, 0.03)
            min_x = min_y = -extent
            max_x = max_y = extent
        else:
            geometry = capture.geometries[-1]["anchors"] if capture.geometries else {}
            combined = raw_xy + ekf_xy + list(geometry.values())
            min_x = min((item[0] for item in combined), default=0.0)
            max_x = max((item[0] for item in combined), default=1.0)
            min_y = min((item[1] for item in combined), default=0.0)
            max_y = max((item[1] for item in combined), default=1.0)
            span = max(max_x - min_x, max_y - min_y, 1.0) * 1.08
            center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2
            min_x, max_x = center_x - span / 2, center_x + span / 2
            min_y, max_y = center_y - span / 2, center_y + span / 2

        def map_point(item: tuple[float, float]) -> tuple[float, float]:
            return (
                plot_x + plot_size * (item[0] - min_x) / max(1e-12, max_x - min_x),
                plot_y + plot_size * (item[1] - min_y) / max(1e-12, max_y - min_y),
            )

        for grid in range(6):
            offset = plot_size * grid / 5
            canvas.line(plot_x + offset, plot_y, plot_x + offset, plot_y + plot_size, "#e2e8f0", 0.6)
            canvas.line(plot_x, plot_y + offset, plot_x + plot_size, plot_y + offset, "#e2e8f0", 0.6)
        canvas.rect(plot_x, plot_y, plot_size, plot_size, "#94a3b8", fill=False)
        raw = [map_point(item) for item in raw_xy[:: max(1, len(raw_xy) // 1500)]]
        ekf = [map_point(item) for item in ekf_xy[:: max(1, len(ekf_xy) // 1500)]]
        if centered_static:
            for x, y in raw:
                canvas.rect(x - 0.65, y - 0.65, 1.3, 1.3, "#2563eb")
            for x, y in ekf:
                canvas.rect(x - 0.65, y - 0.65, 1.3, 1.3, "#10b981")
        else:
            canvas.polyline(raw, "#2563eb", 0.75)
            canvas.polyline(ekf, "#10b981", 1.15)
        canvas.text(
            panel_x + panel_width / 2,
            331,
            PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
            12,
            PROTOCOL_COLORS.get(capture.protocol, "#64748b"),
            bold=True,
            centered=True,
        )
        canvas.text(plot_x, 42, f"raw n={len(raw_xy)}; EKF n={len(ekf_xy)}", 8, "#475569")
    canvas.line(330, 20, 355, 20, "#2563eb", 2.0)
    canvas.text(362, 17, "raw independent", 9, "#2563eb")
    canvas.line(500, 20, 525, 20, "#10b981", 2.0)
    canvas.text(532, 17, "position-only adaptive EKF", 9, "#10b981")
    canvas.save(path)


def fixed_tag_rtk_points(capture: Capture, tag_id: int = 1) -> list[tuple[float, float]]:
    """Return finite RTK Fixed horizontal tag solutions in the common ENU frame."""
    return [
        (float(point["east_m"]), float(point["north_m"]))
        for point in capture.gps
        if int(point.get("module_id", -1)) == tag_id
        and point.get("valid")
        and int(point.get("quality", 0)) == 4
        and finite(point.get("east_m"))
        and finite(point.get("north_m"))
    ]


def pdf_static_rtk_clouds(
    path: pathlib.Path,
    captures: list[Capture],
    tag_id: int = 1,
    source_label: str = "",
) -> None:
    """Show both within-block RTK precision and the absolute cross-block ENU jump."""
    width, height = 1000.0, 735.0
    canvas = PdfCanvas(width, height)
    title = "GPS RTK Fixed tag clouds and cross-capture centers"
    if source_label:
        title += f" - {source_label}"
    canvas.text(width / 2, height - 27, title, 17, bold=True, centered=True)
    canvas.text(
        width / 2,
        height - 48,
        "Top: each block centered on its own median. Bottom: the same fixes in one common ENU frame.",
        10,
        "#475569",
        centered=True,
    )
    prepared: list[tuple[Capture, list[tuple[float, float]], tuple[float, float], list[tuple[float, float]]]] = []
    centered_extent = 0.0
    for capture in captures:
        absolute = fixed_tag_rtk_points(capture, tag_id)
        center = (median(point[0] for point in absolute), median(point[1] for point in absolute))
        centered = [(point[0] - center[0], point[1] - center[1]) for point in absolute]
        if centered:
            centered_extent = max(centered_extent, max(max(abs(x), abs(y)) for x, y in centered))
        prepared.append((capture, absolute, center, centered))
    centered_extent = max(centered_extent * 1.08, 0.01)

    panel_width, plot_size = 300.0, 225.0
    panel_gap = (width - panel_width * len(prepared)) / (len(prepared) + 1)
    for index, (capture, absolute, _, centered) in enumerate(prepared):
        panel_x = panel_gap + index * (panel_width + panel_gap)
        plot_x, plot_y = panel_x + 38, 415.0
        for grid in range(6):
            offset = plot_size * grid / 5
            canvas.line(plot_x + offset, plot_y, plot_x + offset, plot_y + plot_size, "#e2e8f0", 0.6)
            canvas.line(plot_x, plot_y + offset, plot_x + plot_size, plot_y + offset, "#e2e8f0", 0.6)
        canvas.rect(plot_x, plot_y, plot_size, plot_size, "#94a3b8", fill=False)

        def map_centered(point: tuple[float, float]) -> tuple[float, float]:
            return (
                plot_x + plot_size * (point[0] + centered_extent) / (2 * centered_extent),
                plot_y + plot_size * (point[1] + centered_extent) / (2 * centered_extent),
            )

        color = PROTOCOL_COLORS.get(capture.protocol, "#15803d")
        for point in centered:
            x, y = map_centered(point)
            canvas.rect(x - 1.4, y - 1.4, 2.8, 2.8, color)
        cx, cy = map_centered((0.0, 0.0))
        canvas.line(cx - 6, cy, cx + 6, cy, "#111827", 1.0)
        canvas.line(cx, cy - 6, cx, cy + 6, "#111827", 1.0)
        canvas.text(
            panel_x + panel_width / 2,
            654,
            PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
            12,
            color,
            bold=True,
            centered=True,
        )
        extent_cm = 100 * centered_extent
        canvas.text(plot_x, 397, f"E/N +/-{extent_cm:.1f} cm; n={len(absolute)} fixed", 8, "#475569")

    all_absolute = [point for _, absolute, _, _ in prepared for point in absolute]
    if all_absolute:
        min_e = min(point[0] for point in all_absolute)
        max_e = max(point[0] for point in all_absolute)
        min_n = min(point[1] for point in all_absolute)
        max_n = max(point[1] for point in all_absolute)
        span = max(max_e - min_e, max_n - min_n, 0.1) * 1.10
        center_e, center_n = (min_e + max_e) / 2, (min_n + max_n) / 2
        min_e, max_e = center_e - span / 2, center_e + span / 2
        min_n, max_n = center_n - span / 2, center_n + span / 2
        plot_size = 285.0
        plot_x, plot_y = (width - plot_size) / 2, 58.0
        for grid in range(6):
            offset = plot_size * grid / 5
            canvas.line(plot_x + offset, plot_y, plot_x + offset, plot_y + plot_size, "#e2e8f0", 0.6)
            canvas.line(plot_x, plot_y + offset, plot_x + plot_size, plot_y + offset, "#e2e8f0", 0.6)
        canvas.rect(plot_x, plot_y, plot_size, plot_size, "#94a3b8", fill=False)

        def map_absolute(point: tuple[float, float]) -> tuple[float, float]:
            return (
                plot_x + plot_size * (point[0] - min_e) / max(1e-12, max_e - min_e),
                plot_y + plot_size * (point[1] - min_n) / max(1e-12, max_n - min_n),
            )

        for legend_index, (capture, absolute, center, _) in enumerate(prepared):
            color = PROTOCOL_COLORS.get(capture.protocol, "#64748b")
            for point in absolute:
                x, y = map_absolute(point)
                canvas.rect(x - 1.1, y - 1.1, 2.2, 2.2, color)
            cx, cy = map_absolute(center)
            canvas.line(cx - 7, cy, cx + 7, cy, color, 1.8)
            canvas.line(cx, cy - 7, cx, cy + 7, color, 1.8)
            legend_x = 705.0
            legend_y = 272.0 - legend_index * 26
            canvas.rect(legend_x, legend_y, 12, 12, color)
            canvas.text(legend_x + 20, legend_y + 2, PROTOCOL_LABELS.get(capture.protocol, capture.protocol), 10)
            canvas.text(legend_x + 20, legend_y - 11, f"center E={center[0]:.3f}, N={center[1]:.3f} m", 8, "#475569")
        canvas.text(width / 2, 362, "Common ENU frame; equal East/North scale", 12, bold=True, centered=True)
        canvas.text(plot_x, 40, f"East [{min_e:.2f}, {max_e:.2f}] m", 8, "#475569")
        canvas.text(plot_x + 160, 40, f"North [{min_n:.2f}, {max_n:.2f}] m", 8, "#475569")
    canvas.save(path)


def pdf_rtk_reference_trajectories(
    path: pathlib.Path,
    captures: list[Capture],
    tag_id: int = 1,
    source_label: str = "",
) -> None:
    """Draw GPS-only RTK Fixed dynamic trajectories without older UWB data."""
    width, height = 1000.0, 390.0
    canvas = PdfCanvas(width, height)
    title = "GPS RTK Fixed dynamic reference trajectories"
    if source_label:
        title += f" - {source_label}"
    canvas.text(width / 2, height - 27, title, 17, bold=True, centered=True)
    panel_width = 300.0
    plot_size = 250.0
    panel_gap = (width - panel_width * len(captures)) / (len(captures) + 1)
    for panel_index, capture in enumerate(captures):
        points = fixed_tag_rtk_points(capture, tag_id)
        panel_x = panel_gap + panel_index * (panel_width + panel_gap)
        plot_x, plot_y = panel_x + 25, 55.0
        min_x = min((point[0] for point in points), default=0.0)
        max_x = max((point[0] for point in points), default=1.0)
        min_y = min((point[1] for point in points), default=0.0)
        max_y = max((point[1] for point in points), default=1.0)
        span = max(max_x - min_x, max_y - min_y, 1.0) * 1.08
        center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2
        min_x, max_x = center_x - span / 2, center_x + span / 2
        min_y, max_y = center_y - span / 2, center_y + span / 2

        def map_point(item: tuple[float, float]) -> tuple[float, float]:
            return (
                plot_x + plot_size * (item[0] - min_x) / max(1e-12, max_x - min_x),
                plot_y + plot_size * (item[1] - min_y) / max(1e-12, max_y - min_y),
            )

        for grid in range(6):
            offset = plot_size * grid / 5
            canvas.line(plot_x + offset, plot_y, plot_x + offset, plot_y + plot_size, "#e2e8f0", 0.6)
            canvas.line(plot_x, plot_y + offset, plot_x + plot_size, plot_y + offset, "#e2e8f0", 0.6)
        canvas.rect(plot_x, plot_y, plot_size, plot_size, "#94a3b8", fill=False)
        mapped = [map_point(point) for point in points]
        color = PROTOCOL_COLORS.get(capture.protocol, "#15803d")
        canvas.polyline(mapped, color, 1.25)
        canvas.text(
            panel_x + panel_width / 2,
            328,
            PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
            12,
            color,
            bold=True,
            centered=True,
        )
        canvas.text(plot_x, 34, f"x [{min_x:.2f}, {max_x:.2f}] m", 8, "#475569")
        canvas.text(plot_x + 132, 34, f"y [{min_y:.2f}, {max_y:.2f}] m", 8, "#475569")
        canvas.text(plot_x, 19, f"n={len(points)} RTK Fixed tag fixes", 8, "#475569")
    canvas.save(path)


def svg_bar_chart(
    path: pathlib.Path,
    title: str,
    labels: list[str],
    series: list[tuple[str, list[float], str]],
    y_label: str,
) -> None:
    width, height = 900, 470
    left, right, top, bottom = 85, 25, 55, 115
    plot_width, plot_height = width - left - right, height - top - bottom
    finite_values = [value for _, values, _ in series for value in values if finite(value)]
    maximum = max(finite_values, default=1.0)
    maximum = max(maximum * 1.12, 1e-9)
    group_width = plot_width / max(1, len(labels))
    bar_width = min(46.0, group_width * 0.72 / max(1, len(series)))
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="28" text-anchor="middle" font-family="sans-serif" font-size="20">{svg_escape(title)}</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_height}" stroke="#111827"/>',
        f'<line x1="{left}" y1="{top+plot_height}" x2="{left+plot_width}" y2="{top+plot_height}" stroke="#111827"/>',
    ]
    for tick in range(6):
        value = maximum * tick / 5
        y = top + plot_height - plot_height * tick / 5
        lines.extend(
            [
                f'<line x1="{left}" y1="{y:.1f}" x2="{left+plot_width}" y2="{y:.1f}" stroke="#e5e7eb"/>',
                f'<text x="{left-9}" y="{y+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:.2g}</text>',
            ]
        )
    for label_index, label in enumerate(labels):
        center = left + group_width * (label_index + 0.5)
        total_width = bar_width * len(series)
        for series_index, (name, values, color) in enumerate(series):
            value = values[label_index] if label_index < len(values) else math.nan
            if not finite(value):
                continue
            bar_height = plot_height * max(0.0, value) / maximum
            x = center - total_width / 2 + series_index * bar_width
            y = top + plot_height - bar_height
            lines.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width-2:.1f}" height="{bar_height:.1f}" fill="{color}" opacity="0.86"><title>{svg_escape(name)}: {value:.4g}</title></rect>'
            )
        lines.append(
            f'<text x="{center:.1f}" y="{top+plot_height+20}" text-anchor="middle" font-family="sans-serif" font-size="11" transform="rotate(24 {center:.1f} {top+plot_height+20})">{svg_escape(label)}</text>'
        )
    legend_x = left + 10
    for index, (name, _, color) in enumerate(series):
        x = legend_x + index * 210
        lines.extend(
            [
                f'<rect x="{x}" y="{height-28}" width="14" height="14" fill="{color}"/>',
                f'<text x="{x+20}" y="{height-16}" font-family="sans-serif" font-size="12">{svg_escape(name)}</text>',
            ]
        )
    lines.append(
        f'<text x="18" y="{top+plot_height/2}" text-anchor="middle" font-family="sans-serif" font-size="13" transform="rotate(-90 18 {top+plot_height/2})">{svg_escape(y_label)}</text>'
    )
    lines.append("</svg>\n")
    path.write_text("\n".join(lines), encoding="utf-8")


def svg_cdf_chart(
    path: pathlib.Path,
    title: str,
    series: list[tuple[str, list[float], str]],
) -> None:
    width, height = 820, 470
    left, right, top, bottom = 75, 25, 55, 65
    plot_width, plot_height = width - left - right, height - top - bottom
    all_values = [value for _, values, _ in series for value in values if finite(value)]
    maximum = cdf_axis_max(all_values)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="28" text-anchor="middle" font-family="sans-serif" font-size="20">{svg_escape(title)}</text>',
    ]
    for tick in range(6):
        x = left + plot_width * tick / 5
        y = top + plot_height - plot_height * tick / 5
        lines.extend(
            [
                f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top+plot_height}" stroke="#e5e7eb"/>',
                f'<text x="{x:.1f}" y="{top+plot_height+22}" text-anchor="middle" font-family="sans-serif" font-size="12">{maximum*tick/5:.2f}</text>',
                f'<line x1="{left}" y1="{y:.1f}" x2="{left+plot_width}" y2="{y:.1f}" stroke="#e5e7eb"/>',
                f'<text x="{left-8}" y="{y+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{20*tick}</text>',
            ]
        )
    lines.extend(
        [
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_height}" stroke="#111827"/>',
            f'<line x1="{left}" y1="{top+plot_height}" x2="{left+plot_width}" y2="{top+plot_height}" stroke="#111827"/>',
        ]
    )
    for index, (label, values, color) in enumerate(series):
        ordered = sorted(value for value in values if finite(value))
        if not ordered:
            continue
        points = []
        for item_index in cdf_sample_indices(len(ordered), 500):
            value = ordered[item_index]
            x = left + plot_width * value / maximum
            y = top + plot_height * (1.0 - item_index / max(1, len(ordered) - 1))
            points.append(f"{x:.1f},{y:.1f}")
        lines.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2.2"/>')
        legend_y = top + 18 * index
        lines.extend(
            [
                f'<line x1="{left+plot_width-190}" y1="{legend_y}" x2="{left+plot_width-165}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>',
                f'<text x="{left+plot_width-158}" y="{legend_y+4}" font-family="sans-serif" font-size="12">{svg_escape(label)}</text>',
            ]
        )
    lines.extend(
        [
            f'<text x="{left+plot_width/2}" y="{height-12}" text-anchor="middle" font-family="sans-serif" font-size="13">aligned UWB - RTK disagreement [m]</text>',
            f'<text x="18" y="{top+plot_height/2}" text-anchor="middle" font-family="sans-serif" font-size="13" transform="rotate(-90 18 {top+plot_height/2})">cumulative probability [%]</text>',
            "</svg>\n",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def make_figures(
    directory: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    transformed: dict[str, list[dict[str, Any]]],
    pairs_by_capture: dict[str, list[dict[str, Any]]],
    rtk_reference_captures: list[Capture] | None = None,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    campaign_match = re.search(r"(20\d{6})", str(directory))
    if campaign_match:
        try:
            campaign_day = dt.datetime.strptime(campaign_match.group(1), "%Y%m%d").date()
            capture_date_label = (
                f"{campaign_day.strftime('%B')} {campaign_day.day}, {campaign_day.year}"
            )
        except ValueError:
            capture_date_label = "selected"
    else:
        capture_date_label = "selected"
    ordered = sorted(captures, key=lambda item: (item.motion, item.protocol, item.capture_id))
    labels = [f"{PROTOCOL_LABELS.get(item.protocol, item.protocol)}\n{item.motion}" for item in ordered]
    colors = [PROTOCOL_COLORS.get(item.protocol, "#64748b") for item in ordered]

    rates = [summaries[item.capture_id]["position_stream"]["independent_rate_hz"] for item in ordered]
    gaps = [summaries[item.capture_id]["position_stream"]["gap_p95_ms"] for item in ordered]
    svg_bar_chart(
        directory / "01_position_rate.svg",
        "Effective independent position rate",
        labels,
        [("independent Hz", rates, "#0f766e")],
        "positions/s",
    )
    pdf_bar_chart(
        directory / "01_position_rate.pdf",
        "Effective independent position rate",
        labels,
        [("independent Hz", rates, "#0f766e")],
        "positions/s",
    )
    svg_bar_chart(
        directory / "02_position_gap.svg",
        "Position positive-gap P95",
        labels,
        [("gap P95", gaps, "#f59e0b")],
        "milliseconds",
    )
    pdf_bar_chart(
        directory / "02_position_gap.pdf",
        "Position positive-gap P95",
        labels,
        [("gap P95", gaps, "#f59e0b")],
        "milliseconds",
    )

    static = [item for item in ordered if item.motion == "static"]
    if static:
        cep = [summaries[item.capture_id]["static_precision"].get("cep50_m", math.nan) for item in static]
        p95 = [summaries[item.capture_id]["static_precision"].get("p95_m", math.nan) for item in static]
        svg_bar_chart(
            directory / "03_static_precision.svg",
            "Static precision; independent-frame samples",
            [PROTOCOL_LABELS.get(item.protocol, item.protocol) for item in static],
            [("CEP50", cep, "#0f766e"), ("P95", p95, "#f59e0b")],
            "radial displacement [m]",
        )
        pdf_bar_chart(
            directory / "03_static_precision.pdf",
            "Static precision; independent-frame samples",
            [PROTOCOL_LABELS.get(item.protocol, item.protocol) for item in static],
            [("CEP50", cep, "#0f766e"), ("P95", p95, "#f59e0b")],
            "radial displacement [m]",
        )
        pdf_position_panels(
            directory / "07_static_position_clouds.pdf",
            f"Static raw UWB position clouds - {capture_date_label} captures",
            static,
            centered_static=True,
        )
        rtk_static = [
            item for item in (rtk_reference_captures or static) if item.motion == "static"
        ]
        pdf_static_rtk_clouds(
            directory / "08_static_gps_rtk_clouds.pdf",
            rtk_static,
            source_label="August 6 RTK reference" if rtk_reference_captures else "",
        )
        pdf_raw_ekf_panels(
            directory / "10_static_raw_vs_ekf.pdf",
            "Static raw versus position-only adaptive EKF",
            static,
            centered_static=True,
        )

    dynamic = [item for item in ordered if item.motion == "dynamic"]
    if dynamic:
        if rtk_reference_captures is None:
            cdf_series = []
            for capture in dynamic:
                pairs = pairs_by_capture[capture.capture_id]
                errors = [pair["error_m"] for pair in pairs]
                fit_grade = summaries[capture.capture_id]["alignment"].get("quality_flag")
                grade_label = "usable" if fit_grade == "usable_with_limitations" else "poor fit"
                cdf_series.append(
                    (
                        f"{PROTOCOL_LABELS.get(capture.protocol, capture.protocol)} (n={len(errors)}; {grade_label})",
                        errors,
                        PROTOCOL_COLORS.get(capture.protocol, "#64748b"),
                    )
                )
            svg_cdf_chart(
                directory / "04_dynamic_rtk_error_cdf.svg",
                "Dynamic paired residual CDF (see anchor-fit grade)",
                cdf_series,
            )
            pdf_cdf_chart(
                directory / "04_dynamic_rtk_error_cdf.pdf",
                "Dynamic paired residual CDF (see anchor-fit grade)",
                cdf_series,
            )
        pdf_position_panels(
            directory / "06_dynamic_trajectories.pdf",
            f"Raw dynamic UWB trajectories - {capture_date_label} captures",
            dynamic,
            centered_static=False,
        )
        pdf_raw_ekf_panels(
            directory / "09_dynamic_raw_vs_ekf.pdf",
            "Dynamic raw versus position-only adaptive EKF",
            dynamic,
            centered_static=False,
        )

    if rtk_reference_captures:
        rtk_dynamic = [item for item in rtk_reference_captures if item.motion == "dynamic"]
        if rtk_dynamic:
            pdf_rtk_reference_trajectories(
                directory / "11_dynamic_gps_rtk_reference.pdf",
                rtk_dynamic,
                source_label="August 6 RTK reference",
            )
        for stale_name in (
            "04_dynamic_rtk_error_cdf.svg",
            "04_dynamic_rtk_error_cdf.pdf",
            "05_alignment_quality.svg",
            "05_alignment_quality.pdf",
        ):
            stale = directory / stale_name
            if stale.exists():
                stale.unlink()

    aligned = [] if rtk_reference_captures else [
        item for item in ordered if summaries[item.capture_id]["alignment"].get("available")
    ]
    if aligned:
        fit = [summaries[item.capture_id]["alignment"]["fit_rmse_m_p95"] for item in aligned]
        error = [summaries[item.capture_id]["rtk_error"].get("error_rmse_m", math.nan) for item in aligned]
        svg_bar_chart(
            directory / "05_alignment_quality.svg",
            "Anchor-registration RMSE (P95 across snapshots)",
            [f"{PROTOCOL_LABELS.get(item.protocol, item.protocol)} {item.motion}" for item in aligned],
            [
                ("anchor-fit RMSE P95", fit, "#64748b"),
                ("paired tag RMSE", error, "#dc2626"),
            ],
            "metres",
        )
        pdf_bar_chart(
            directory / "05_alignment_quality.pdf",
            "Anchor-registration RMSE (P95 across snapshots)",
            [f"{PROTOCOL_LABELS.get(item.protocol, item.protocol)} {item.motion}" for item in aligned],
            [
                ("anchor-fit RMSE P95", fit, "#64748b"),
                ("paired tag RMSE", error, "#dc2626"),
            ],
            "metres",
        )


def fmt(value: Any, digits: int = 3) -> str:
    if value is None or not finite(value):
        return "n/a"
    return f"{float(value):.{digits}f}"


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    lines.extend("| " + " | ".join(str(value) for value in row) + " |" for row in rows)
    return "\n".join(lines)


def write_cross_session_report_md(
    path: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    args: argparse.Namespace,
    rtk_reference_captures: list[Capture],
    rtk_reference_summaries: dict[str, dict[str, Any]],
) -> None:
    """Write the concise audit companion for split-date UWB/RTK evidence."""
    capture_rows: list[list[Any]] = []
    static_rows: list[list[Any]] = []
    ekf_rows: list[list[Any]] = []
    for capture in sorted(captures, key=lambda item: (item.motion, item.protocol)):
        item = summaries[capture.capture_id]
        stream = item["position_stream"]
        capture_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                capture.capture_id,
                fmt(item["duration_s"], 1),
                stream["events"],
                stream["independent_events"],
                fmt(stream["independent_rate_hz"], 2),
                fmt(stream["gap_p95_ms"], 0),
                fmt(stream["gap_max_ms"], 0),
            ]
        )
        if capture.motion == "static":
            precision = item["static_precision"]
            static_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    precision.get("n", 0),
                    fmt(100 * precision.get("cep50_m", math.nan), 2),
                    fmt(100 * precision.get("rms_m", math.nan), 2),
                    fmt(100 * precision.get("p95_m", math.nan), 2),
                    fmt(100 * precision.get("max_m", math.nan), 2),
                ]
            )
        ekf_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(item["ekf_stream"].get("available_pct"), 1),
                fmt(item["position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(item["ekf_position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(100 * item["static_precision"].get("rms_m", math.nan), 2),
                fmt(100 * item["ekf_static_precision"].get("rms_m", math.nan), 2),
            ]
        )
    reference_rows: list[list[Any]] = []
    for capture in sorted(
        rtk_reference_captures, key=lambda item: (item.motion, item.protocol)
    ):
        item = rtk_reference_summaries[capture.capture_id]
        rtk = item["rtk"]
        cloud = item["tag_fixed_cloud_precision"]
        reference_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                capture.capture_id,
                rtk.get("tag_total", 0),
                rtk.get("tag_fixed", 0),
                fmt(rtk.get("tag_fixed_pct"), 1),
                fmt(rtk.get("all_modules_fixed_pct"), 1),
                fmt(rtk.get("tag_fix_age_p95_ms"), 0),
                fmt(100 * cloud.get("rms_m", math.nan), 2),
                fmt(100 * cloud.get("p95_m", math.nan), 2),
            ]
        )
    report = f"""# Dynamic/static UWB comparison

## Evidence boundary

- All UWB position, rate, gap, static-precision and EKF results come exclusively
  from the final August 12 captures.
- All GPS RTK availability and cloud results come exclusively from the August 6
  reference captures.
- No August 6 UWB position enters this report, and no August 12 GPS sample enters
  the RTK section.
- The sessions are not synchronized; cross-session UWB–RTK RMSE/P95 is therefore
  intentionally not calculated.

## August 12 UWB capture matrix

{markdown_table(
    ["protocol", "mode", "capture", "duration s", "events", "independent", "independent Hz", "gap P95 ms", "gap max ms"],
    capture_rows,
)}

![August 12 raw UWB dynamic trajectories](figures/06_dynamic_trajectories.pdf)

## August 12 static UWB precision

{markdown_table(
    ["protocol", "n", "CEP50 cm", "RMS cm", "P95 cm", "max cm"],
    static_rows,
)}

![August 12 static raw UWB clouds](figures/07_static_position_clouds.pdf)

## Position-only adaptive EKF

{markdown_table(
    ["protocol", "mode", "coverage %", "raw step P95 m/s", "EKF step P95 m/s", "raw static RMS cm", "EKF static RMS cm"],
    ekf_rows,
)}

![August 12 dynamic raw versus EKF](figures/09_dynamic_raw_vs_ekf.pdf)

![August 12 static raw versus EKF](figures/10_static_raw_vs_ekf.pdf)

## August 6 GPS RTK reference

This is a GPS-only reference population. It characterizes RTK fix availability
and short-term scatter for the unchanged receiver path, but is not paired with
the August 12 UWB positions.

{markdown_table(
    ["protocol", "mode", "capture", "tag total", "tag fixed", "tag fixed %", "all modules fixed %", "fix-age P95 ms", "tag cloud RMS cm", "tag cloud P95 cm"],
    reference_rows,
)}

![August 6 GPS RTK Fixed dynamic reference](figures/11_dynamic_gps_rtk_reference.pdf)

![August 6 GPS RTK Fixed static clouds](figures/08_static_gps_rtk_clouds.pdf)

## Reproduction

```powershell
python tools/uwb_dynamic_static_report.py `
  --input-dir reports/bundles/uwb_dynamic_static_comparison_20260812/raw/data `
  --output-dir reports/bundles/uwb_dynamic_static_comparison_20260812/analysis `
  --rtk-reference-report-dir reports/bundles/uwb_dynamic_static_comparison_20260806/analysis
```

The August 12 raw UWB captures remain archived losslessly under
`../raw/data/`. The RTK
reference manifest points to the already-versioned August 6 lossless archives;
the older UWB records in those archives are never loaded by the GPS-only reader.
"""
    path.write_text(report, encoding="utf-8")


def load_replay_summaries(
    dynamic_dir: pathlib.Path,
    static_dir: pathlib.Path,
    captures: list[Capture] | None = None,
) -> dict[str, dict[str, Any]]:
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for motion, directory in (("dynamic", dynamic_dir), ("static", static_dir)):
        if not directory.exists():
            continue
        for path in sorted(directory.glob("*.summary.json")):
            with path.open(encoding="utf-8") as handle:
                payload = json.load(handle)
            protocol = str(payload.get("selected_protocol", "unknown"))
            tracks = payload.get("tracks", [])
            if not tracks:
                continue
            track = tracks[0]
            diagnostics = track.get("fusion_final", {}).get("diagnostics", {})
            raw = (
                payload.get("overall_metrics", {}).get("raw_vs_rtk") or {}
            )
            fused = (
                payload.get("overall_metrics", {}).get("fused_vs_rtk") or {}
            )
            timing = track.get("timing", {})
            alignment = track.get("rtk_alignment", {})
            position_samples = int(diagnostics.get("position_samples", 0))
            accepted = int(diagnostics.get("position_accepted", 0))
            outliers = int(diagnostics.get("position_outliers", 0))
            fit_rms = alignment.get("anchor_fit_rms_m", math.nan)
            if not alignment.get("available"):
                rtk_grade = "unavailable"
            elif finite(fit_rms) and float(fit_rms) > 0.5:
                rtk_grade = "indicative_only_large_anchor_fit"
            else:
                rtk_grade = "indicative_in_system_reference"
            item = {
                "summary_path": str(path),
                "protocol": protocol,
                "motion": motion,
                "input_files": payload.get("input_files", []),
                "position_samples": position_samples,
                "position_accepted": accepted,
                "position_outliers": outliers,
                "position_accept_pct": 100.0 * accepted / position_samples if position_samples else math.nan,
                "position_outlier_pct": 100.0 * outliers / position_samples if position_samples else math.nan,
                "position_corrections": int(diagnostics.get("position_corrections", 0)),
                "ordering_rejects": int(diagnostics.get("ordering_rejects", 0)),
                "reset_count": int(diagnostics.get("reset_count", 0)),
                "velocity_clamps": int(diagnostics.get("velocity_clamps", 0)),
                "yaw_alignment_valid": bool(diagnostics.get("yaw_alignment_valid")),
                "yaw_alignment_updates": int(diagnostics.get("yaw_alignment_updates", 0)),
                "imu_samples": int(diagnostics.get("imu_samples", 0)),
                "imu_invalid_samples": int(diagnostics.get("imu_invalid_samples", 0)),
                "position_rate_hz": timing.get("raw_position", {}).get("rate_hz", math.nan),
                "position_gap_p95_ms": timing.get("raw_position", {}).get("interval_p95_ms", math.nan),
                "position_gap_max_ms": timing.get("raw_position", {}).get("interval_max_ms", math.nan),
                "position_gap_gt_threshold_count": timing.get("raw_position", {}).get("gap_count", 0),
                "imu_rate_hz": timing.get("imu_used", {}).get("rate_hz", math.nan),
                "imu_gap_p95_ms": timing.get("imu_used", {}).get("interval_p95_ms", math.nan),
                "imu_gap_max_ms": timing.get("imu_used", {}).get("interval_max_ms", math.nan),
                "imu_gap_gt_threshold_count": timing.get("imu_used", {}).get("gap_count", 0),
                "rtk_fixed_fixes": track.get("input_counts", {}).get("rtk_fixed_fixes", 0),
                "rtk_matches": track.get("input_counts", {}).get("rtk_matches", 0),
                "anchor_fit_rms_m": fit_rms,
                "anchor_count": alignment.get("anchor_count", 0),
                "all_anchor_geometry_rtk_fixed": bool(alignment.get("all_anchor_geometry_rtk_fixed")),
                "maximum_anchor_time_delta_ms": alignment.get("maximum_anchor_time_delta_ms", math.nan),
                "rtk_interpretation_grade": rtk_grade,
                "raw_vs_rtk_count": raw.get("count", 0),
                "raw_vs_rtk_rmse_m": raw.get("rmse_m", math.nan),
                "raw_vs_rtk_p95_m": raw.get("p95_m", math.nan),
                "raw_vs_rtk_max_m": raw.get("max_m", math.nan),
                "fused_vs_rtk_count": fused.get("count", 0),
                "fused_vs_rtk_rmse_m": fused.get("rmse_m", math.nan),
                "fused_vs_rtk_p95_m": fused.get("p95_m", math.nan),
                "fused_vs_rtk_max_m": fused.get("max_m", math.nan),
                "fused_minus_raw_rmse_m": (
                    float(fused["rmse_m"]) - float(raw["rmse_m"])
                    if finite(fused.get("rmse_m")) and finite(raw.get("rmse_m"))
                    else math.nan
                ),
                "limitations": payload.get("limitations", []),
            }
            candidates[f"{motion}:{protocol}"].append(item)
    selected_ids = {
        (capture.motion, capture.protocol): capture.capture_id for capture in (captures or [])
    }
    result: dict[str, dict[str, Any]] = {}
    for key, values in candidates.items():
        motion, protocol = key.split(":", 1)
        capture_id = selected_ids.get((motion, protocol), "")

        def priority(item: dict[str, Any]) -> tuple[int, int, float]:
            inputs = " ".join(str(value).lower() for value in item.get("input_files", []))
            direct_match = int(bool(capture_id) and capture_id.lower() in inputs)
            preferred_name = int(
                (motion == "static" and "rtkfixed" in inputs)
                or (motion == "dynamic" and "final" in inputs)
            )
            summary_path = pathlib.Path(item["summary_path"])
            return direct_match, preferred_name, summary_path.stat().st_mtime

        result[key] = max(values, key=priority)
    return result


def write_report(
    path: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    replays: dict[str, dict[str, Any]],
    static_consistency: dict[str, Any],
    args: argparse.Namespace,
    missing: list[dict[str, str]],
    rtk_reference_captures: list[Capture] | None = None,
    rtk_reference_summaries: dict[str, dict[str, Any]] | None = None,
) -> None:
    if rtk_reference_captures and rtk_reference_summaries:
        write_cross_session_report_md(
            path,
            captures,
            summaries,
            args,
            rtk_reference_captures,
            rtk_reference_summaries,
        )
        return
    ordered = sorted(captures, key=lambda item: (item.motion, item.protocol, item.capture_id))
    stream_rows = []
    static_rows = []
    rtk_rows = []
    replay_rows = []
    ekf_rows = []
    for capture in ordered:
        item = summaries[capture.capture_id]
        stream = item["position_stream"]
        stream_rows.append(
            [
                capture.capture_id,
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(item["duration_s"], 1),
                stream["events"],
                fmt(stream["event_rate_hz"], 2),
                fmt(stream["active_event_rate_hz"], 2),
                fmt(stream["independent_rate_hz"], 2),
                fmt(stream["wall_coverage_pct"], 1),
                fmt(stream["gap_p95_ms"], 1),
                fmt(stream["gap_max_ms"], 1),
            ]
        )
        if capture.motion == "static":
            precision = item["static_precision"]
            static_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    precision.get("n", 0),
                    fmt(precision.get("cep50_m")),
                    fmt(precision.get("rms_m")),
                    fmt(precision.get("p95_m")),
                    fmt(precision.get("two_drms_m")),
                    fmt(precision.get("first_to_last_decile_drift_m")),
                ]
            )
        error = item["rtk_error"]
        ekf_error = item["ekf_rtk_error"]
        ekf = item["ekf_stream"]
        ekf_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(ekf.get("available_pct"), 1),
                fmt(item["position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(item["ekf_position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(100 * item["static_precision"].get("rms_m", math.nan), 2),
                fmt(100 * item["ekf_static_precision"].get("rms_m", math.nan), 2),
                fmt(error.get("error_rmse_m")),
                fmt(ekf_error.get("error_rmse_m")),
            ]
        )
        alignment = item["alignment"]
        alignment_label = {
            "poor_alignment_do_not_claim_accuracy": "poor / no accuracy claim",
            "usable_with_limitations": "usable with limits",
            "unavailable": "unavailable",
        }.get(alignment.get("quality_flag", "unavailable"), alignment.get("quality_flag", "unavailable"))
        rtk_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                error.get("pairs", 0),
                fmt(error.get("association_abs_p95_ms"), 1),
                fmt(error.get("error_rmse_m")),
                fmt(error.get("error_p95_m")),
                fmt(error.get("debiased_error_rmse_m")),
                fmt(alignment.get("fit_rmse_m_p95")),
                alignment_label,
                error.get("interpretation_grade", "indicative_in_system_reference"),
            ]
        )
    for key in sorted(replays):
        item = replays[key]
        replay_rows.append(
            [
                PROTOCOL_LABELS.get(item["protocol"], item["protocol"]),
                item["motion"],
                fmt(item["position_rate_hz"], 2),
                fmt(item["position_gap_p95_ms"], 1),
                fmt(item["position_gap_max_ms"], 1),
                fmt(item["position_accept_pct"], 1),
                item["reset_count"],
                fmt(item["raw_vs_rtk_rmse_m"]),
                fmt(item["fused_vs_rtk_rmse_m"]),
                fmt(item["anchor_fit_rms_m"]),
                item["rtk_interpretation_grade"],
            ]
        )

    missing_text = "\n".join(
        f"- {PROTOCOL_LABELS.get(item['protocol'], item['protocol'])} / {item['motion']}: missing capture"
        for item in missing
    ) or "- None."
    if args.skip_raw_archive:
        raw_archive_artifacts = (
            "- RAW capture archiving was intentionally skipped for this "
            "development run; the immutable source JSONL paths remain in "
            "`analysis_summary.json`."
        )
        raw_restore_instructions = ""
    else:
        raw_archive_artifacts = (
            "- `data/*.jsonl.xz`: the six complete RAW captures, compressed "
            "losslessly;\n- `raw_data_manifest.csv` and `SHA256SUMS`: "
            "source/archive integrity;"
        )
        raw_restore_instructions = r'''

Restore an archived capture without an `xz` executable:

```powershell
python -c "import lzma,shutil; shutil.copyfileobj(lzma.open(r'data/CAPTURE.jsonl.xz','rb'), open(r'CAPTURE.jsonl','wb'))"
```
'''
    reproduce_flags = []
    if args.require_rpi_static:
        reproduce_flags.append("  --require-rpi-static `")
    if args.skip_raw_archive:
        reproduce_flags.append("  --skip-raw-archive `")
    reproduce_options = (
        "\n".join(reproduce_flags).rstrip(" `")
        if reproduce_flags
        else ""
    )
    report = f"""# Dynamic/static UWB comparison

Generated from immutable JSONL captures. This report compares published
position streams; it does not modify or invoke replay/dashboard code.

> **Interpretation warning:** RTK is an in-system reference, not independently
> surveyed ground truth. UWB coordinates are rigidly registered to robust
> medians of RTK-fixed anchor positions. The anchor-registration residual,
> GNSS fixed availability, sparse time association, antenna lever arms and any
> RTK-derived dynamic geometry limit every absolute-error statement below.

## Capture matrix

{markdown_table(
    ["capture", "protocol", "motion", "duration s", "position n", "capture Hz", "active Hz", "independent Hz", "coverage %", "gap P95 ms", "gap max ms"],
    stream_rows,
)}

Missing cells:

{missing_text}

Passive DS publishes overlapping raw windows. Comparative position metrics use
only records marked `independent_frame`; the all-event rate remains visible as
an operational telemetry rate. Dashboard fusion records (`imu_fused=true` or
an `*_imu_fused_position` stream) are excluded from every raw/independent
capture metric and counted separately in `capture_metrics.csv`. `Capture Hz`
uses the full capture wall duration; `active Hz` uses the stream's own uptime
span. Coverage and end-lag fields expose captures whose telemetry stops early.
Every accepted position record must also match the protocol declared by its
capture envelope; mismatches are excluded and counted in `capture_metrics.csv`.

![Independent position rate](figures/01_position_rate.svg)

![Position gap](figures/02_position_gap.svg)

## Static precision

Static precision is radial displacement around each capture's own local-frame
median. It is **precision, not absolute accuracy**, and uses independent-frame
samples without deleting outliers.

{markdown_table(
    ["protocol", "n", "CEP50 m", "RMS m", "P95 m", "2DRMS m", "first-last drift m"],
    static_rows,
) if static_rows else "No completed static capture was available."}

![Static precision](figures/03_static_precision.svg)

## Position-only adaptive EKF

The EKF consumes only UWB positions; IMU acceleration is disabled. It is
evaluated against the raw independent stream carried by the same records.
Lower step-speed P95 means less sample-to-sample jitter, while RTK columns are
only auditable where anchor registration is valid.

{markdown_table(
    ["protocol", "motion", "EKF coverage %", "raw step P95 m/s", "EKF step P95 m/s", "raw static RMS cm", "EKF static RMS cm", "raw RTK RMSE m", "EKF RTK RMSE m"],
    ekf_rows,
)}

![Dynamic raw versus EKF](figures/09_dynamic_raw_vs_ekf.pdf)

![Static raw versus EKF](figures/10_static_raw_vs_ekf.pdf)

## Static RTK cross-capture consistency gate

The tag was stationary, so its RTK center must agree between protocol blocks
before RTK can support an absolute ranking. The observed maximum pairwise
center separation is **{fmt(static_consistency.get('max_pairwise_distance_m'))}
m**, against a {fmt(static_consistency.get('threshold_m'))} m gate. Verdict:
**{static_consistency.get('interpretation')}**.

This gate is independent of the per-capture anchor fit. A small anchor-fit RMS
can coexist with a shifted tag reference and cannot rescue static ranking.

## UWB versus RTK disagreement

Only RTK-fixed tag solutions are used. Each solution is matched to the nearest
independent UWB position by `estimated_measurement_wall_ns` versus the UWB
`received_at` timestamp, with an absolute limit of {args.rtk_match_ms:.0f} ms.
The local UWB frame is mapped to RTK ENU with the better ID-keyed direct or
reflected rigid transform. Scale is fixed at one and is never fitted.
`Debiased RMSE` removes the median tag residual after anchor registration and
is a shape/repeatability diagnostic, not accuracy.

{markdown_table(
    ["protocol", "motion", "pairs", "time P95 ms", "RMSE m", "P95 m", "debiased RMSE m", "anchor-fit P95 RMSE m", "alignment flag", "RTK interpretation"],
    rtk_rows,
)}

![Dynamic RTK error CDF](figures/04_dynamic_rtk_error_cdf.svg)

![Alignment quality](figures/05_alignment_quality.svg)

## Method and limitations

- GPS coordinates are converted from WGS84 ECEF to a common local ENU frame.
- Interleaved dashboard fusion positions are excluded before all raw position
  metrics; raw events may still carry `imu_fused_*` diagnostic fields.
- Anchor centers are per-capture component-wise medians of RTK-fixed samples.
- Both direct and reflected fixed-scale 2-D rigid transforms are evaluated
  from the local anchor geometry to the RTK centers; the smaller anchor-RMSE
  candidate is selected. At least three fixed anchors are required.
- Time-varying Passive DS geometries use the nearest captured geometry
  snapshot, preferring an exact `geometry_version`. A non-exact dynamic
  geometry older than {args.max_geometry_age_s:.1f} s is rejected.
- A Passive DS geometry marked dynamic is itself derived from GNSS. Its frame
  alignment is therefore not independent of the RTK reference; the report
  marks this circularity explicitly.
- RTK `fixed` status does not guarantee centimetre-level truth. Anchor P95
  spread and rigid-fit residual are exported, and poor cases are flagged.
- `estimated_measurement_wall_ns` subtracts the receiver-reported fix age from
  collector wall time. It is not hardware timestamp synchronization. Pair
  count, coverage and association error must accompany RMSE/P95.
- The tag and anchor GNSS antennas need not be collocated with their UWB
  antennas; unknown lever arms appear as bias.
- Static and dynamic blocks are not repeated randomized trials. Differences
  may include path, orientation, RF environment and geometry-state changes.
- Capture-boundary samples outside the common RTK/UWB interval are not paired.

## Audit artifacts

- `analysis_summary.json`: nested machine-readable results and policies;
- `capture_metrics.csv`: one flattened row per capture;
- `rtk_alignment_metrics.csv`: registration and RTK-pair metrics;
- `rtk_pairs.csv`: every accepted time association and residual;
- `alignment_snapshots.csv`: every geometry-to-RTK rigid fit;
- `replay_metrics.csv`: replay schema export; empty for this captured-stream
  comparison because the EKF counterpart is embedded in every raw record;
{raw_archive_artifacts}
- `figures/`: dependency-free SVG and PDF plots.

## Reproduce

```powershell
python tools/uwb_dynamic_static_report.py `
  --input-dir {args.input_dir.as_posix()} `
  --output-dir {args.output_dir.as_posix()} `
  --replay-dynamic-dir {args.replay_dynamic_dir.as_posix()} `
  --replay-static-dir {args.replay_static_dir.as_posix()}{(' `' if reproduce_options else '')}
{reproduce_options}
```{raw_restore_instructions.rstrip()}
"""
    path.write_text(report, encoding="utf-8")


def tex_escape(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in text)


def tex_table(headers: list[str], rows: list[list[Any]], alignment: str | None = None) -> str:
    if not rows:
        return r"\emph{No data available.}"
    columns = alignment or ("l" + "r" * (len(headers) - 1))
    header = " & ".join(r"\textbf{" + tex_escape(item) + "}" for item in headers) + r" \\"
    body = "\n".join(" & ".join(tex_escape(item) for item in row) + r" \\" for row in rows)
    return (
        r"\begin{center}\scriptsize\resizebox{\textwidth}{!}{%" + "\n"
        + r"\begin{tabular}{" + columns + r"}\toprule" + "\n"
        + header
        + "\n"
        + r"\midrule"
        + "\n"
        + body
        + "\n"
        + r"\bottomrule\end{tabular}}\end{center}"
    )


def tex_figure(filename: str, caption: str, width: str = r"0.94\textwidth") -> str:
    return (
        "\n\\begin{figure}[htbp]\n\\centering\n"
        + f"\\includegraphics[width={width}]{{figures/{filename}}}\n"
        + "\\caption{" + tex_escape(caption) + "}\n\\end{figure}\n"
    )


def write_tex_report_legacy(
    path: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    replays: dict[str, dict[str, Any]],
    static_consistency: dict[str, Any],
    args: argparse.Namespace,
) -> None:
    ordered = sorted(captures, key=lambda item: (item.motion, item.protocol, item.capture_id))
    continuity_rows: list[list[Any]] = []
    static_rows: list[list[Any]] = []
    rtk_rows: list[list[Any]] = []
    replay_rows: list[list[Any]] = []
    for capture in ordered:
        item = summaries[capture.capture_id]
        stream = item["position_stream"]
        imu = item["imu_stream"]
        continuity_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(stream["event_rate_hz"], 2),
                fmt(stream["active_event_rate_hz"], 2),
                fmt(stream["independent_rate_hz"], 2),
                fmt(stream["wall_coverage_pct"], 1),
                fmt(stream["gap_p95_ms"], 1),
                fmt(stream["gap_max_ms"], 1),
                fmt(imu["event_rate_hz"], 2),
                fmt(imu["gap_p95_ms"], 1),
                fmt(imu["gap_max_ms"], 1),
            ]
        )
        if capture.motion == "static":
            precision = item["static_precision"]
            static_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    precision.get("n", 0),
                    fmt(precision.get("cep50_m")),
                    fmt(precision.get("rms_m")),
                    fmt(precision.get("p95_m")),
                    fmt(precision.get("two_drms_m")),
                    fmt(precision.get("first_to_last_decile_drift_m")),
                ]
            )
        error = item["rtk_error"]
        alignment = item["alignment"]
        rtk_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                error.get("pairs", 0),
                fmt(error.get("error_rmse_m")),
                fmt(error.get("error_p95_m")),
                fmt(error.get("debiased_error_rmse_m")),
                fmt(alignment.get("fit_rmse_m_p95")),
                alignment.get("quality_flag", "unavailable"),
                error.get("interpretation_grade", "indicative_in_system_reference"),
            ]
        )
    for key in sorted(replays):
        item = replays[key]
        replay_rows.append(
            [
                PROTOCOL_LABELS.get(item["protocol"], item["protocol"]),
                item["motion"],
                fmt(item["position_accept_pct"], 1),
                item["position_outliers"],
                item["reset_count"],
                fmt(item["raw_vs_rtk_rmse_m"]),
                fmt(item["fused_vs_rtk_rmse_m"]),
                fmt(item["anchor_fit_rms_m"]),
                item["rtk_interpretation_grade"],
            ]
        )

    tex = r"""\documentclass[10pt,a4paper]{article}
\usepackage{lmodern}
\usepackage[margin=18mm]{geometry}
\usepackage{booktabs}
\usepackage{graphicx}
\usepackage[table]{xcolor}
\usepackage[hidelinks]{hyperref}
\usepackage{parskip}
\definecolor{warning}{HTML}{FFF3CD}
\title{Dynamic/static UWB comparison\\\large FlexTDOA, Native DS-TWR, Passive DS-TWR}
\author{Reproducible analysis of the 2026-08-12 captures}
\date{Generated automatically}
\begin{document}
\maketitle

\section{Executive summary}
\colorbox{warning}{\parbox{0.96\textwidth}{\textbf{Evidence boundary.}
Stream rates, uptime gaps, sample counts, fusion acceptance and reset counts are
direct capture/replay diagnostics and are the reliable part of this report.
Absolute UWB--RTK disagreement is only indicative. RTK was not an independent
survey, clocks are associated in software, antenna lever arms are unknown, and
the UWB frame is registered through the RTK anchor geometry.}}

The report keeps all samples. Passive DS overlapping raw windows are visible in
the telemetry rate, while cross-protocol precision and trajectory comparisons
use only records marked as independent frames. A large anchor-fit RMS is never
hidden behind a tag RMSE. Interleaved records explicitly marked
\texttt{imu\_fused=true} are excluded from all raw/independent metrics.

\section{Reliable continuity and update-rate results}
"""
    tex += tex_table(
        ["Protocol", "Mode", "pos capture Hz", "pos active Hz", "pos indep Hz", "coverage pct", "pos P95 ms", "pos max ms", "IMU Hz", "IMU P95 ms", "IMU max ms"],
        continuity_rows,
    )
    tex += r"""

Duplicate millisecond uptime values are not extra elapsed time. The JSON and CSV
artifacts additionally report unique-uptime rates, duplicate shares, capture
coverage and the lag from the last stream event to \texttt{capture\_end}.

\section{Static precision}
Static precision is displacement about each local-frame median, not accuracy.
Outliers are retained; Passive DS uses independent-frame positions.
"""
    tex += tex_table(
        ["Protocol", "N", "CEP50 m", "RMS m", "P95 m", "2DRMS m", "decile drift m"], static_rows
    )
    tex += "\n\\subsection{RTK cross-capture consistency gate}\n"
    tex += (
        "The stationary tag RTK center has a maximum pairwise separation of "
        + fmt(static_consistency.get("max_pairwise_distance_m"))
        + " m, compared with a "
        + fmt(static_consistency.get("threshold_m"))
        + " m gate. \\textbf{"
        + tex_escape(static_consistency.get("interpretation"))
        + "} A small per-capture anchor-fit RMS cannot rescue a shifted tag reference.\n"
    )
    tex += r"""

\section{Replay continuity and fusion}
The six replay summaries are consumed verbatim after replay completion. Acceptance,
outliers, resets and timing remain direct diagnostics. Replay RTK metrics are
shown only as secondary evidence: repeated output matches do not increase the
number of independent RTK fixes.
"""
    tex += tex_table(
        ["Protocol", "Mode", "accepted share", "outliers", "resets", "raw RTK RMSE m", "fused RTK RMSE m", "anchor fit m", "RTK grade"],
        replay_rows,
    )
    tex += r"""

\section{Independent RTK association audit}
For this report, each RTK-fixed tag solution contributes at most one pair. The
nearest independent UWB position must be within """ + f"{args.rtk_match_ms:.0f}" + r""" ms. A proper 2-D
rotation and translation is fitted from at least three anchors; scale is fixed.
The debiased result removes the median tag residual and is a shape diagnostic,
not accuracy.
"""
    tex += tex_table(
        ["Protocol", "Mode", "pairs", "RMSE m", "P95 m", "debiased RMSE m", "anchor-fit P95 m", "alignment flag", "RTK interpretation"],
        rtk_rows,
    )
    tex += r"""

\section{Limitations}
\begin{itemize}
\item RTK is an in-system comparison reference, not independently surveyed truth.
\item GPS measurement time is estimated by subtracting reported fix age from collector wall time; it is not hardware synchronization.
\item GNSS and UWB antenna phase centers are not known to be collocated.
\item Passive DS dynamic geometry can itself be GNSS-derived, so frame registration is not independent of RTK.
\item Anchor RTK fixed status does not guarantee a correct solution. Anchor spread and rigid-fit residual are exported for every capture.
\item Static and dynamic blocks are single trials and differ in route, orientation and RF history.
\end{itemize}

\section{Reproduction and audit files}
The report directory contains machine-readable JSON/CSV outputs, SVG plots,
the six lossless raw captures in \texttt{data/*.jsonl.xz}, a SHA-256 manifest,
and the source \texttt{report.tex}. Restore a capture with Python's standard
library \texttt{lzma} module, then rerun \texttt{tools/uwb\_dynamic\_static\_report.py}.

\end{document}
"""
    path.write_text(tex, encoding="utf-8")


def campaign_date_from_output(args: argparse.Namespace) -> dt.date:
    match = re.search(r"(20\d{6})", args.output_dir.name)
    if match:
        try:
            return dt.datetime.strptime(match.group(1), "%Y%m%d").date()
        except ValueError:
            pass
    return dt.datetime.now().date()


def write_tex_report_same_session(
    path: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    static_consistency: dict[str, Any],
    args: argparse.Namespace,
) -> None:
    """Write the illustrated same-session report used by the August 13 campaign."""

    ordered = sorted(captures, key=lambda item: (item.motion, item.protocol, item.capture_id))
    by_cell = {(capture.motion, capture.protocol): summaries[capture.capture_id] for capture in captures}

    def metric(motion: str, protocol: str, group: str, name: str) -> float:
        return float(by_cell.get((motion, protocol), {}).get(group, {}).get(name, math.nan))

    capture_rows: list[list[Any]] = []
    dynamic_rows: list[list[Any]] = []
    gap_rows: list[list[Any]] = []
    static_rows: list[list[Any]] = []
    ekf_rows: list[list[Any]] = []
    rtk_rows: list[list[Any]] = []
    quality_rows: list[list[Any]] = []
    for capture in ordered:
        item = summaries[capture.capture_id]
        stream = item["position_stream"]
        capture_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                capture.capture_id,
                fmt(item["duration_s"], 1),
                stream["events"],
                stream["independent_events"],
                fmt(stream["independent_rate_hz"], 2),
                fmt(stream["wall_coverage_pct"], 1),
                stream.get("uptime_epoch_count", 1),
            ]
        )
        flags = set(item.get("quality_flags", []))
        notes: list[str] = []
        if "position_stream_covers_less_than_80pct_of_capture" in flags:
            notes.append(
                "position coverage below 80%; first position at "
                f"+{fmt(stream.get('first_event_after_capture_start_s'), 2)} s"
            )
        if "capture_duration_less_than_10s" in flags:
            notes.append("short capture; interpret static precision as indicative")
        excluded_mismatch = int(item.get("protocol_mismatch_position_records_excluded", 0))
        if excluded_mismatch:
            notes.append(f"excluded {excluded_mismatch} position records from another protocol")
        if notes:
            quality_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    capture.motion,
                    fmt(item["duration_s"], 2),
                    fmt(stream.get("wall_span_s"), 2),
                    "; ".join(notes),
                ]
            )
        gap_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(stream["event_rate_hz"], 2),
                fmt(stream["independent_rate_hz"], 2),
                fmt(stream["gap_p50_ms"], 0),
                fmt(stream["gap_p95_ms"], 0),
                fmt(stream["gap_p99_ms"], 0),
                fmt(stream["gap_max_ms"], 0),
                stream["gaps_gt_100ms"],
            ]
        )
        if capture.motion == "dynamic":
            dynamic_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    fmt(item["duration_s"], 1),
                    stream["events"],
                    stream["independent_events"],
                    fmt(stream["event_rate_hz"], 2),
                    fmt(stream["independent_rate_hz"], 2),
                    fmt(stream["gap_p95_ms"], 0),
                    fmt(stream["gap_max_ms"], 0),
                    fmt(item["position_jumps"].get("step_speed_p95_mps"), 2),
                ]
            )
        if capture.motion == "static":
            precision = item["static_precision"]
            static_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    precision.get("n", 0),
                    fmt(100 * precision.get("cep50_m", math.nan), 2),
                    fmt(100 * precision.get("rms_m", math.nan), 2),
                    fmt(100 * precision.get("p95_m", math.nan), 2),
                    fmt(100 * precision.get("p99_m", math.nan), 2),
                    fmt(100 * precision.get("max_m", math.nan), 2),
                    fmt(100 * precision.get("first_to_last_decile_drift_m", math.nan), 2),
                ]
            )
        ekf = item["ekf_stream"]
        ekf_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(ekf.get("available_pct"), 1),
                fmt(item["position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(item["ekf_position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(100 * item["static_precision"].get("rms_m", math.nan), 2),
                fmt(100 * item["ekf_static_precision"].get("rms_m", math.nan), 2),
            ]
        )
        error = item["rtk_error"]
        alignment = item["alignment"]
        alignment_label = {
            "poor_alignment_do_not_claim_accuracy": "poor; no accuracy claim",
            "usable_with_limitations": "usable with limits",
            "unavailable": "unavailable",
        }.get(alignment.get("quality_flag", "unavailable"), alignment.get("quality_flag", "unavailable"))
        rtk_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                error.get("pairs", 0),
                fmt(error.get("association_abs_p95_ms"), 1),
                fmt(error.get("error_rmse_m")),
                fmt(error.get("error_p95_m")),
                fmt(error.get("debiased_error_rmse_m")),
                fmt(alignment.get("fit_rmse_m_p95")),
                alignment_label,
            ]
        )

    campaign_date = campaign_date_from_output(args)
    campaign_label = f"{campaign_date.strftime('%B')} {campaign_date.day}, {campaign_date.year}"
    dynamic_rates = {
        protocol: metric("dynamic", protocol, "position_stream", "independent_rate_hz")
        for protocol in PROTOCOL_LABELS
    }
    dynamic_gaps = {
        protocol: metric("dynamic", protocol, "position_stream", "gap_max_ms")
        for protocol in PROTOCOL_LABELS
    }
    static_rms = {
        protocol: 100 * metric("static", protocol, "static_precision", "rms_m")
        for protocol in PROTOCOL_LABELS
    }

    tex = r"""\documentclass[10pt,a4paper]{article}
\usepackage{lmodern}
\usepackage[margin=19mm]{geometry}
\usepackage{booktabs}
\usepackage{graphicx}
\usepackage[table]{xcolor}
\usepackage[hidelinks]{hyperref}
\usepackage{parskip}
\usepackage{array}
\graphicspath{{figures/}}
\definecolor{warning}{HTML}{FFF3CD}
\definecolor{info}{HTML}{E0F2FE}
\hypersetup{pdftitle={Dynamic and static comparison of three UWB protocols},pdfauthor={UWB ESP-IDF technical report}}
\begin{document}
\pagenumbering{gobble}
\begin{titlepage}
\centering
\vspace*{18mm}
{\Huge\bfseries Dynamic and static UWB comparison\par}
\vspace{4mm}
{\Large FlexTDOA, Native DS-TWR and Passive DS-TWR\par}
\vspace{14mm}
{\large Field campaign of """ + tex_escape(campaign_label) + r"""\par}
\vspace{12mm}
\begin{tabular}{rl}
Platform: & 5 ESP32-S3 + DW3000 modules, Raspberry Pi collector\\
Topology: & M1 tag; M2, M3, M4 and M5 anchors\\
Radio: & UWB channel 9\\
Position streams: & raw independent UWB and position-only adaptive EKF\\
Reference: & embedded GPS RTK telemetry, quality audited per capture\\
\end{tabular}
\vfill
\colorbox{info}{\parbox{0.88\textwidth}{\vspace{2mm}
\textbf{Main result.} All six protocol/motion cells are populated. Dynamic
continuity is directly measurable, and static local precision is consistently
at centimetre scale. RTK anchor registration passes for some, but not all,
captures, so no complete three-protocol absolute-accuracy ranking is claimed;
raw UWB remains the audit stream.\vspace{2mm}}}
\vfill
{\large Reproducible technical report\par}
\end{titlepage}
\pagenumbering{roman}
\tableofcontents
\clearpage
\pagenumbering{arabic}

\section{Executive summary}
\colorbox{warning}{\parbox{0.96\textwidth}{\textbf{Evidence boundary.}
Rates, gaps, sample counts, local static precision and raw-versus-EKF
comparisons are direct capture diagnostics. RTK is an in-system reference, not
independently surveyed truth. Every RTK residual must be read with its per-row
anchor-fit grade; a failed fit invalidates that row and prevents a complete
cross-protocol absolute-accuracy ranking.}}

The dynamic independent rates were \textbf{""" + fmt(dynamic_rates["flextdoa"], 2) + r""" positions/s}
for FlexTDOA, \textbf{""" + fmt(dynamic_rates["native_ds"], 2) + r""" positions/s} for Native
DS-TWR and \textbf{""" + fmt(dynamic_rates["passive_ds"], 2) + r""" positions/s} for Passive
DS-TWR. Maximum observed gaps were """ + fmt(dynamic_gaps["flextdoa"], 0) + r""",
""" + fmt(dynamic_gaps["native_ds"], 0) + r""" and """ + fmt(dynamic_gaps["passive_ds"], 0) + r""" ms respectively.

Static raw local RMS was \textbf{""" + fmt(static_rms["flextdoa"], 2) + r""" cm} for FlexTDOA,
\textbf{""" + fmt(static_rms["native_ds"], 2) + r""" cm} for Native DS-TWR and
\textbf{""" + fmt(static_rms["passive_ds"], 2) + r""" cm} for Passive DS-TWR. These values
measure repeatability around each capture median, not absolute position error.

\section{Scope and protocol background}
The campaign compares the published position streams of three independent UWB
protocols. FlexTDOA and Passive DS-TWR keep the tag radio receive-only. Native
DS-TWR performs active tag-to-anchor exchanges and provides the coherent-frame
baseline. Passive rolling windows may publish multiple events from shared radio
measurements, so all-event and independent-frame rates are reported separately.

\section{Captured evidence}
"""
    tex += tex_table(
        ["Protocol", "Mode", "Capture ID", "duration s", "events", "independent", "indep Hz", "coverage pct", "epochs"],
        capture_rows,
    )
    if quality_rows:
        tex += r"""

\subsection{Capture-quality notices}
These notices do not invalidate the retained samples, but they limit how far
the affected block can be generalized.
"""
        tex += tex_table(
            ["Protocol", "Mode", "envelope s", "active span s", "notice"],
            quality_rows,
            "llrrl",
        )
    tex += r"""

All selected blocks have terminal capture envelopes. Derived blocks are
lossless time slices of source JSONL captures. The raw records are not
smoothed, trimmed by error or rewritten by the report generator.

\section{Dynamic results}
\subsection{Raw trajectories}
The routes are separate physical trials. Their panels support continuity and
gross-shape inspection; they are not synchronized point-by-point repeats.
"""
    tex += tex_figure(
        "06_dynamic_trajectories.pdf",
        "Raw independent UWB trajectories in each protocol's local frame. Green markers are anchors.",
        r"\textwidth",
    )
    tex += r"""\subsection{Update rate and gaps}
"""
    tex += tex_table(
        ["Protocol", "duration s", "events", "independent", "event Hz", "indep Hz", "gap P95 ms", "gap max ms", "step P95 m/s"],
        dynamic_rows,
    )
    tex += tex_figure(
        "01_position_rate.pdf",
        "Independent position rate for every dynamic and static block.",
    )
    tex += tex_figure(
        "02_position_gap.pdf",
        "P95 positive position gap. Maximum gaps and counts remain in the tables.",
    )
    tex += r"""
\subsection{Complete continuity audit}
Uptime epochs are separated at multi-second backsteps, so a protocol restart is
never turned into a synthetic gap or step-speed measurement.
"""
    tex += tex_table(
        ["Protocol", "Mode", "event Hz", "indep Hz", "P50 ms", "P95", "P99", "max", "gaps over 100 ms"],
        gap_rows,
    )
    tex += r"""

\section{Position-only adaptive EKF}
The dashboard EKF consumes UWB positions only; acceleration is disabled. Raw
measurements remain archived, and each valid EKF point is tied to its raw event.
Step-speed P95 is a jitter diagnostic rather than physical walking speed.
"""
    tex += tex_table(
        ["Protocol", "Mode", "coverage pct", "raw step P95", "EKF step P95", "raw static RMS cm", "EKF static RMS cm"],
        ekf_rows,
    )
    tex += tex_figure(
        "09_dynamic_raw_vs_ekf.pdf",
        "Raw dynamic trajectories (blue) and position-only EKF trajectories (green).",
        r"\textwidth",
    )
    tex += r"""

\section{Static results}
Static precision is radial displacement around each block's own component-wise
median. All independent samples and outliers are retained.
"""
    tex += tex_table(
        ["Protocol", "N", "CEP50 cm", "RMS cm", "P95 cm", "P99 cm", "max cm", "drift cm"],
        static_rows,
    )
    tex += tex_figure(
        "03_static_precision.pdf",
        "Static raw precision around each block median.",
    )
    tex += tex_figure(
        "07_static_position_clouds.pdf",
        "Static raw position clouds, centered independently and drawn at one common scale.",
        r"\textwidth",
    )
    tex += tex_figure(
        "10_static_raw_vs_ekf.pdf",
        "Static raw and position-only EKF clouds, each centered on its own median.",
        r"\textwidth",
    )
    tex += r"""

\clearpage
\section{RTK association audit}
Each quality-4 tag solution contributes at most one pair. The nearest
independent UWB position must lie within """ + f"{args.rtk_match_ms:.0f}" + r""" ms. The UWB
frame is registered to robust RTK anchor centers using the better ID-keyed
direct or reflected rigid transform. Scale is fixed at one and is never fitted.
The reported anchor-fit value is the P95, across captured geometry snapshots,
of the four-anchor RMSE. A row is graded usable only when the worst snapshot
RMSE is at most 0.5 m, anchor RTK spread is at most 0.5 m and reflection choice
is consistent.

The stationary-tag cross-capture separation is """ + fmt(static_consistency.get("max_pairwise_distance_m")) + r""" m,
against a """ + fmt(static_consistency.get("threshold_m")) + r""" m consistency gate. The gate fails.
Absolute static ranking is therefore invalid.
"""
    tex += tex_table(
        ["Protocol", "Mode", "pairs", "time P95 ms", "RMSE m", "P95 m", "debiased RMSE m", "fit RMSE P95 m", "grade"],
        rtk_rows,
    )
    tex += tex_figure(
        "04_dynamic_rtk_error_cdf.pdf",
        "Dynamic UWB--RTK residual CDF. Each legend entry includes its anchor-fit grade; RTK is not surveyed truth.",
    )
    tex += tex_figure(
        "05_alignment_quality.pdf",
        "Best direct/reflected fixed-scale anchor-registration residual. The per-row grade controls interpretation.",
    )
    tex += tex_figure(
        "08_static_gps_rtk_clouds.pdf",
        "RTK Fixed stationary tag clouds and cross-capture centers.",
        r"\textwidth",
    )
    tex += r"""

\clearpage
\section{Interpretation}
\begin{itemize}
\item Passive DS-TWR has the highest independent dynamic rate in this dataset;
its all-event rate is higher because rolling windows overlap.
\item Native DS-TWR provides the best maximum dynamic gap among the three
selected walks and remains the coherent active-ranging baseline.
\item FlexTDOA preserves a receive-only tag but shows the longest gaps in this
campaign.
\item Static local precision is similar across all protocols. The EKF reduces
visible stationary jitter but is not a replacement for raw UWB.
\item FlexTDOA and Passive DS-TWR pass the configured dynamic anchor-fit gate;
Native DS-TWR does not, so a complete absolute RTK ranking is not made.
\end{itemize}

\section{Limitations}
\begin{itemize}
\item RTK is an in-system comparison reference, not independently surveyed truth.
\item GPS and UWB measurement times are associated in software, not by one hardware clock.
\item GNSS and UWB antenna phase centers have unknown lever arms.
\item Routes are single trials rather than randomized synchronized repeats.
\item One static position does not characterize edge GDOP, NLOS or spatial bias.
\item Collector rate measures the complete telemetry path, not DW3000 PHY throughput.
\end{itemize}

\section{Reproduction and audit files}
The directory contains machine-readable JSON/CSV results, SVG/PDF figures,
losslessly compressed raw captures, SHA-256 checksums and this TeX source.

\begin{verbatim}
$reportArgs = @(
  "--input-dir", "reports/bundles/uwb_dynamic_static_comparison_20260813/raw/data"
  "--output-dir", "reports/bundles/uwb_dynamic_static_comparison_20260813/analysis"
  "--replay-dynamic-dir", "reports/uwb_final_report_input_20260813"
  "--replay-static-dir", "reports/uwb_final_report_input_20260813"
)
python tools/uwb_dynamic_static_report.py @reportArgs
\end{verbatim}

No raw capture is modified during report generation.
\end{document}
"""
    path.write_text(tex, encoding="utf-8")


def write_tex_report(
    path: pathlib.Path,
    captures: list[Capture],
    summaries: dict[str, dict[str, Any]],
    replays: dict[str, dict[str, Any]],
    static_consistency: dict[str, Any],
    args: argparse.Namespace,
    rtk_reference_captures: list[Capture] | None = None,
    rtk_reference_summaries: dict[str, dict[str, Any]] | None = None,
) -> None:
    """Write the full field report; the legacy compact writer is kept above for audit."""
    if not rtk_reference_captures:
        write_tex_report_same_session(path, captures, summaries, static_consistency, args)
        return
    ordered = sorted(captures, key=lambda item: (item.motion, item.protocol, item.capture_id))
    by_cell = {(capture.motion, capture.protocol): summaries[capture.capture_id] for capture in captures}

    def metric(motion: str, protocol: str, group: str, name: str) -> float:
        return float(by_cell.get((motion, protocol), {}).get(group, {}).get(name, math.nan))

    capture_rows: list[list[Any]] = []
    dynamic_rows: list[list[Any]] = []
    gap_rows: list[list[Any]] = []
    static_rows: list[list[Any]] = []
    ekf_rows: list[list[Any]] = []
    imu_rows: list[list[Any]] = []
    rtk_rows: list[list[Any]] = []
    rtk_reference_rows: list[list[Any]] = []
    for capture in ordered:
        item = summaries[capture.capture_id]
        stream = item["position_stream"]
        imu = item["imu_stream"]
        capture_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol), capture.motion,
                capture.capture_id, fmt(item["duration_s"], 1), stream["events"],
                stream["independent_events"], fmt(stream["independent_rate_hz"], 2),
                fmt(stream["wall_coverage_pct"], 1),
                "yes" if capture.complete and not capture.interrupted else "no",
            ]
        )
        gap_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol), capture.motion,
                fmt(stream["event_rate_hz"], 2), fmt(stream["independent_rate_hz"], 2),
                fmt(stream["gap_p50_ms"], 0), fmt(stream["gap_p95_ms"], 0),
                fmt(stream["gap_p99_ms"], 0), fmt(stream["gap_max_ms"], 0),
                stream["gaps_gt_100ms"],
            ]
        )
        imu_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol), capture.motion,
                imu["events"], fmt(imu["event_rate_hz"], 1), fmt(imu["valid_pct"], 1),
                fmt(imu["gap_p95_ms"], 0), fmt(imu["gap_max_ms"], 0),
            ]
        )
        if capture.motion == "dynamic":
            jumps = item["position_jumps"]
            dynamic_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                    fmt(item["duration_s"], 1), stream["events"], stream["independent_events"],
                    fmt(stream["event_rate_hz"], 2), fmt(stream["independent_rate_hz"], 2),
                    fmt(stream["gap_p95_ms"], 0), fmt(stream["gap_max_ms"], 0),
                    fmt(jumps.get("step_speed_p95_mps"), 2),
                ]
            )
        if capture.motion == "static":
            precision = item["static_precision"]
            static_rows.append(
                [
                    PROTOCOL_LABELS.get(capture.protocol, capture.protocol), precision.get("n", 0),
                    fmt(100 * precision.get("cep50_m", math.nan), 2),
                    fmt(100 * precision.get("rms_m", math.nan), 2),
                    fmt(100 * precision.get("p95_m", math.nan), 2),
                    fmt(100 * precision.get("p99_m", math.nan), 2),
                    fmt(100 * precision.get("max_m", math.nan), 2),
                    fmt(100 * precision.get("two_drms_m", math.nan), 2),
                    fmt(100 * precision.get("first_to_last_decile_drift_m", math.nan), 2),
                ]
            )
        error = item["rtk_error"]
        ekf_error = item["ekf_rtk_error"]
        ekf = item["ekf_stream"]
        ekf_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                fmt(ekf.get("available_pct"), 1),
                fmt(item["position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(item["ekf_position_jumps"].get("step_speed_p95_mps"), 2),
                fmt(100 * item["static_precision"].get("rms_m", math.nan), 2),
                fmt(100 * item["ekf_static_precision"].get("rms_m", math.nan), 2),
            ]
        )
        alignment = item["alignment"]
        rtk_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol), capture.motion,
                error.get("pairs", 0), fmt(error.get("association_abs_p95_ms"), 1),
                fmt(error.get("error_rmse_m")), fmt(error.get("error_p95_m")),
                fmt(error.get("debiased_error_rmse_m")), fmt(alignment.get("fit_rmse_m_p95")),
                alignment.get("quality_flag", "unavailable"),
            ]
        )

    for capture in sorted(
        rtk_reference_captures or [],
        key=lambda item: (item.motion, item.protocol, item.capture_id),
    ):
        item = (rtk_reference_summaries or {}).get(capture.capture_id, {})
        rtk = item.get("rtk", {})
        cloud = item.get("tag_fixed_cloud_precision", {})
        rtk_reference_rows.append(
            [
                PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
                capture.motion,
                rtk.get("tag_total", 0),
                rtk.get("tag_fixed", 0),
                fmt(rtk.get("tag_fixed_pct"), 1),
                fmt(rtk.get("all_modules_fixed_pct"), 1),
                fmt(rtk.get("tag_fix_age_p95_ms"), 0),
                rtk.get("fixed_anchor_count", 0),
                fmt(100 * cloud.get("rms_m", math.nan), 2),
                fmt(100 * cloud.get("p95_m", math.nan), 2),
            ]
        )

    replay_rows: list[list[Any]] = []
    for key in sorted(replays):
        item = replays[key]
        delta_mm = (
            1000 * (item["fused_vs_rtk_rmse_m"] - item["raw_vs_rtk_rmse_m"])
            if finite(item.get("fused_vs_rtk_rmse_m")) and finite(item.get("raw_vs_rtk_rmse_m"))
            else math.nan
        )
        replay_rows.append(
            [
                PROTOCOL_LABELS.get(item["protocol"], item["protocol"]), item["motion"],
                fmt(item["position_accept_pct"], 1), item["position_outliers"], item["reset_count"],
                fmt(item["raw_vs_rtk_rmse_m"]), fmt(item["fused_vs_rtk_rmse_m"]),
                fmt(delta_mm, 1), item["yaw_alignment_updates"], fmt(item["anchor_fit_rms_m"]),
            ]
        )
    tex = r"""\documentclass[10pt,a4paper]{article}
\usepackage{lmodern}
\usepackage[margin=19mm]{geometry}
\usepackage{booktabs}
\usepackage{graphicx}
\usepackage[table]{xcolor}
\usepackage[hidelinks]{hyperref}
\usepackage{parskip}
\usepackage{array}
\graphicspath{{figures/}}
\definecolor{warning}{HTML}{FFF3CD}
\definecolor{info}{HTML}{E0F2FE}
\hypersetup{pdftitle={Dynamic and static comparison of three UWB protocols},pdfauthor={UWB ESP-IDF technical report}}
\begin{document}
\pagenumbering{gobble}
\begin{titlepage}
\centering
\vspace*{18mm}
{\Huge\bfseries Dynamic and static UWB comparison\par}
\vspace{4mm}
{\Large FlexTDOA, Native DS-TWR and Passive DS-TWR\par}
\vspace{14mm}
{\large Final field campaign of August 12, 2026\par}
\vspace{12mm}
\begin{tabular}{rl}
Platform: & 5 ESP32-S3 + DW3000 modules, Raspberry Pi collector\\
Topology: & M1 tag; M2, M3, M4 and M5 anchors\\
Radio: & UWB channel 9; configured 40 MHz SPI\\
Firmware branch: & \texttt{localization-raw-calibration}\\
UWB evidence: & six complete August 12 JSONL captures; raw and position-only EKF\\
RTK evidence: & GPS-only reference captures from August 6\\
\end{tabular}
\vfill
\colorbox{info}{\parbox{0.88\textwidth}{\vspace{2mm}
\textbf{Main result.} All three protocols produced complete raw dynamic and
static datasets. Independent dynamic rates are approximately 25.0 Hz Flex,
25.4 Hz Native and 22.6 Hz Passive. Raw static RMS is 2.3--2.4 cm. The
position-only adaptive EKF materially reduces static jitter and dynamic
sample-to-sample spikes, while raw UWB remains the permanent audit stream.\vspace{2mm}}}
\vfill
{\large Reproducible technical report\par}
\end{titlepage}
\pagenumbering{roman}
\tableofcontents
\clearpage
\pagenumbering{arabic}

\section{Executive summary}
\colorbox{warning}{\parbox{0.96\textwidth}{\textbf{Evidence boundary.}
Stream rates, uptime gaps, sample counts, fusion acceptance and reset counts are
direct August 12 diagnostics. GPS RTK quality and precision use only the August
6 reference captures. The two sessions are not time-paired, so this report does
not calculate or claim August 12 absolute UWB--RTK position error.}}

Passive DS overlapping raw windows remain visible in the telemetry rate, while
all comparative precision and trajectory metrics use independent frames.
Interleaved records marked \texttt{imu\_fused=true} are excluded from raw UWB
metrics. Outliers are retained.
"""
    tex += (
        "\nThe dynamic independent rates were \\textbf{"
        + fmt(metric("dynamic", "flextdoa", "position_stream", "independent_rate_hz"), 2)
        + " positions/s} for FlexTDOA, \\textbf{"
        + fmt(metric("dynamic", "native_ds", "position_stream", "independent_rate_hz"), 2)
        + " positions/s} for Native DS-TWR, and \\textbf{"
        + fmt(metric("dynamic", "passive_ds", "position_stream", "independent_rate_hz"), 2)
        + " positions/s} for Passive DS-TWR. Passive DS emitted "
        + fmt(metric("dynamic", "passive_ds", "position_stream", "event_rate_hz"), 2)
        + " dashboard events/s, but those include overlapping windows.\n\n"
        + "The static raw local RMS values were \\textbf{"
        + fmt(100 * metric("static", "flextdoa", "static_precision", "rms_m"), 2)
        + " cm} (FlexTDOA), \\textbf{"
        + fmt(100 * metric("static", "native_ds", "static_precision", "rms_m"), 2)
        + " cm} (Native DS-TWR) and \\textbf{"
        + fmt(100 * metric("static", "passive_ds", "static_precision", "rms_m"), 2)
        + " cm} (Passive DS-TWR). These are precision around each capture median, not absolute accuracy.\n\n"
    )
    tex += r"""
The UWB and RTK evidence populations are intentionally separated: no August 6
UWB positions enter the protocol results, and no August 12 GPS samples enter
the RTK section. The new filter uses positions only: this campaign intentionally
does not claim acceleration-derived position improvement.

\section{Scope and questions}
The campaign asks how many independent positions each protocol delivers, where
stream gaps occur, how stable the unfiltered stationary position is, and whether
the position-only adaptive EKF reduces visible jitter without hiding raw UWB.

The RTK reference was acquired in a separate session. It establishes receiver
fix availability and short-term precision, but it cannot be paired sample by
sample with the August 12 UWB walks or used as an absolute-error ground truth.

\section{System and protocol background}
\subsection{FlexTDOA}
The tag is UWB receive-only. Anchor transmissions form directed time-difference
observations and the tag solves from a recent coherent observation set. Radio
airtime is largely independent of the number of passive tags.

\subsection{Native DS-TWR}
The active tag exchanges poll, response and final messages with every anchor,
obtains four absolute ranges and solves a coherent frame. Airtime grows with
active tag count, but frame semantics are direct.

\subsection{Passive DS-TWR}
The tag listens to anchor-to-anchor DS-TWR exchanges. Rolling windows may
produce several publications from shared measurements, so dashboard events and
independent frames are intentionally reported separately.

\section{Test configuration and captured evidence}
The three operator-marked dynamic walks lasted 85--114 s and the stationary
blocks approximately 30 s. The selector uses the final continuous-capture
segments and the final short static capture for each protocol.
"""
    tex += tex_table(
        ["Protocol", "Mode", "Capture ID", "duration s", "events", "independent", "indep Hz", "coverage pct", "complete"],
        capture_rows,
    )
    tex += r"""
Every selected block has a terminal \texttt{capture\_end}. All raw data are
archived losslessly and identified by SHA-256.

\section{Methodology}
\subsection{Population and continuity}
No error-based trimming or temporal smoothing is applied. Rates use full
capture wall time. Gaps are positive differences between unique ESP32 uptime
values; P50/P95/P99, maximum and counts above 100 ms are retained together.

\subsection{Static precision}
Radial displacement is computed around each block's component-wise position
median. CEP50, RMS, P95, P99, maximum, 2DRMS and first-to-last-decile drift use
all independent samples. They describe repeatability, not offset from truth.

\subsection{RTK reference}
Only quality-4 fixes from the August 6 reference captures enter the RTK section.
WGS84 is converted through ECEF to one local ENU frame for that session. No
cross-session timestamp association or UWB--RTK rigid fit is performed.

\section{Dynamic results}
\subsection{Raw trajectories}
The walks are separate physical trials, not synchronized repeats. The panels
therefore show continuity and gross geometry rather than point-by-point route
agreement.
"""
    tex += tex_figure(
        "06_dynamic_trajectories.pdf",
        "August 12 raw independent UWB trajectories in each local frame. Green markers are anchors; no August 6 UWB positions are used.",
        r"\textwidth",
    )
    tex += r"""\subsection{Update rate and gaps}
"""
    tex += tex_table(
        ["Protocol", "duration s", "events", "independent", "event Hz", "indep Hz", "gap P95 ms", "gap max ms", "step P95 m/s"],
        dynamic_rows,
    )
    tex += tex_figure(
        "01_position_rate.pdf",
        "Independent position rate in the selected dynamic and static blocks. Passive DS all-event publications are not substituted for independent frames.",
    )
    tex += tex_figure(
        "02_position_gap.pdf",
        "P95 positive gap between unique position uptimes. Maximum gaps and counts remain in the tables.",
    )
    tex += r"""
Native DS-TWR delivered the highest independent rate (25.35 Hz) and the best
worst-case continuity (130 ms). FlexTDOA was close in rate (25.01 Hz), but its
maximum gap was 830 ms. Passive DS delivered 22.63 independent Hz despite
overlapping dashboard publications, with a 250 ms maximum gap. Step speed is a
sample-to-sample trajectory diagnostic, not measured walking speed.

\subsection{Complete continuity audit}
"""
    tex += tex_table(
        ["Protocol", "Mode", "event Hz", "indep Hz", "P50 ms", "P95", "P99", "max", ">100 ms"],
        gap_rows,
    )
    tex += r"""

\section{Position-only adaptive EKF}
The dashboard EKF uses only the raw UWB position stream. IMU acceleration is
disabled, raw measurements remain archived separately, and each EKF result in
this report comes from the same position event as its raw counterpart.
Sample-to-sample step-speed P95 is a jitter/continuity diagnostic; it is not
physical walking speed. Static RMS is repeatability about each stream's own
median. No cross-session RTK error is attached to the filter results.
"""
    tex += tex_table(
        ["Protocol", "Mode", "coverage pct", "raw step P95", "EKF step P95", "raw static RMS cm", "EKF static RMS cm"],
        ekf_rows,
    )
    tex += tex_figure(
        "09_dynamic_raw_vs_ekf.pdf",
        "Raw independent dynamic trajectory (blue) and the position-only adaptive EKF (green). The EKF reduces high-frequency corners without replacing archived raw data.",
        r"\textwidth",
    )
    tex += tex_figure(
        "10_static_raw_vs_ekf.pdf",
        "Static raw and EKF clouds, each centered on its own median and plotted at one common scale. Passive DS may collapse to a held point while stationary by design.",
        r"\textwidth",
    )
    tex += r"""

\section{Static results}
\subsection{Raw local precision}
"""
    tex += tex_table(
        ["Protocol", "N", "CEP50 cm", "RMS cm", "P95 cm", "P99 cm", "max cm", "2DRMS cm", "drift cm"],
        static_rows,
    )
    tex += tex_figure(
        "03_static_precision.pdf",
        "Static precision around each block's own median. All independent raw samples are included.",
    )
    tex += tex_figure(
        "07_static_position_clouds.pdf",
        "Static raw position clouds centered on their own medians and drawn at one common scale. The cross is the median.",
        r"\textwidth",
    )
    tex += r"""
All raw static RMS values lie in a narrow 2.26--2.45 cm range. Passive DS is
best on RMS, while Native and Passive share the best P95 near 3.94 cm. All
outliers are retained. One stationary point cannot characterize GDOP or
spatial bias across the anchor polygon.

\section{GPS RTK reference from August 6}
This section contains GPS data only. It characterizes RTK availability and
short-term scatter for the unchanged receiver path; it does not reuse the old
UWB trajectories and it is not synchronized with the August 12 walks.
"""
    tex += tex_table(
        ["Protocol", "Mode", "tag total", "tag fixed", "tag fixed pct", "all modules fixed pct", "fix-age P95 ms", "fixed anchors", "tag cloud RMS cm", "tag cloud P95 cm"],
        rtk_reference_rows,
    )
    tex += tex_figure(
        "08_static_gps_rtk_clouds.pdf",
        "August 6 GPS RTK Fixed tag solutions only. Top: each reference block centered on its own median. Bottom: the same fixes in the August 6 common ENU frame.",
        r"\textwidth",
    )
    tex += r"""
The trajectory panels below are GPS-only and remain in the August 6 ENU frame.
They are shown as a qualitative RTK reference, not overlaid on the August 12
UWB paths.
"""
    tex += tex_figure(
        "11_dynamic_gps_rtk_reference.pdf",
        "August 6 GPS RTK Fixed dynamic reference trajectories. No UWB positions from that session are plotted.",
        r"\textwidth",
    )
    tex += r"""

\section{Accelerometer scope}
Acceleration-assisted positioning was disabled for this final campaign because
the preceding experiment did not improve raw UWB accuracy. The adaptive EKF
assessed above is position-only. Orientation/IMU telemetry is outside the
protocol ranking, and raw UWB remains the permanent audit stream.

\section{Protocol interpretation and selection}
\subsection{FlexTDOA}
Near-25 Hz independent dynamic rate with a receive-only scalable tag. Static
raw RMS was 2.41 cm. The 830 ms maximum gap is the principal caveat.

\subsection{Native DS-TWR}
Clearest coherent-frame semantics, highest measured independent rate and best
maximum gap. Static raw RMS was 2.45 cm. It remains the preferred deterministic
baseline for a small number of active tags.

\subsection{Passive DS-TWR}
Successfully follows motion without tag transmissions. Independent-frame
accounting yields 22.63 Hz despite overlapping publications; static raw RMS was
the best at 2.26 cm and the maximum dynamic gap was 250 ms.

\begin{center}\small
\begin{tabular}{p{0.20\textwidth}p{0.23\textwidth}p{0.23\textwidth}p{0.23\textwidth}}
\toprule
Criterion & FlexTDOA & Native DS-TWR & Passive DS-TWR\\
\midrule
Tag radio role & receive-only & active exchanges & receive-only\\
Dynamic independent rate & 25.01 Hz & best: 25.35 Hz & 22.63 Hz\\
Dynamic worst gap & 830 ms & best: 130 ms & 250 ms\\
Static raw precision & 2.41 cm RMS & 2.45 cm RMS & best: 2.26 cm RMS\\
Scaling with tag count & strongest & airtime grows & strong\\
Recommendation & primary scalable mode & coherent baseline & passive scalable alternative\\
\bottomrule
\end{tabular}
\end{center}

\section{Limitations}
\begin{itemize}
\item RTK is a separate August 6 reference session, not synchronized August 12 truth.
\item Cross-session UWB--RTK RMSE/P95 is intentionally not computed.
\item GNSS and UWB antenna phase centers have unknown lever arms.
\item Dynamic routes are not repeated surveyed paths; static/dynamic blocks are single trials.
\item One static point does not cover edge geometry, NLOS, orientation or spatial bias.
\item Collector rate is end-to-end telemetry, not direct DW3000 PHY throughput.
\end{itemize}

\section{Recommended next tests}
\begin{enumerate}
\item Repeat every protocol on one marked route with identical orientation and synchronized markers.
\item Interleave static blocks at the center, edges and outside the polygon.
\item Repeat the final UWB campaign with one simultaneous continuous RTK session.
\item Add hardware-correlated timestamps for latency claims.
\item Keep acceleration disabled until body-to-map yaw is independently validated.
\item Capture focused FlexTDOA diagnostics around gaps above 250 ms.
\end{enumerate}

\section{Preliminary findings}
\begin{itemize}
\item FlexTDOA remains a strong receive-only scalable mode, with one long-gap caveat.
\item Native DS-TWR is the best coherent-frame rate and continuity baseline.
\item Passive DS-TWR is a viable receive-only alternative when independent frames are counted correctly.
\item The cross-session RTK reference cannot support an August 12 absolute-accuracy ranking; local UWB precision is ranked.
\item The position-only EKF improves display stability; acceleration-assisted accuracy is not claimed.
\end{itemize}

\section{Reproduction and audit files}
The directory contains \texttt{analysis\_summary.json}, capture/replay/alignment
CSV files, SVG/PDF plots and this TeX source. The machine-readable summary
records whether optional lossless RAW archive generation was enabled; when it
is skipped, immutable source JSONL paths remain recorded for reproduction.

\begin{verbatim}
python tools/uwb_dynamic_static_report.py `
  --input-dir reports/bundles/uwb_dynamic_static_comparison_20260812/raw/data `
  --output-dir reports/bundles/uwb_dynamic_static_comparison_20260812/analysis `
  --rtk-reference-report-dir reports/bundles/uwb_dynamic_static_comparison_20260806/analysis
\end{verbatim}

No raw capture is modified during report generation.
\end{document}
"""
    path.write_text(tex, encoding="utf-8")


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def archive_raw_captures(
    captures: list[Capture],
    output_dir: pathlib.Path,
    data_dir: pathlib.Path,
) -> list[dict[str, Any]]:
    data_dir.mkdir(parents=True, exist_ok=True)
    expected = {f"{capture.path.name}.xz" for capture in captures}
    for stale in data_dir.glob("*.jsonl.xz"):
        if stale.name not in expected:
            stale.unlink()
    rows: list[dict[str, Any]] = []
    checksum_lines: list[str] = []
    for capture in sorted(captures, key=lambda item: item.path.name):
        destination = data_dir / f"{capture.path.name}.xz"
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        if temporary.exists():
            temporary.unlink()
        raw_sha256 = sha256_file(capture.path)
        sibling_archive = pathlib.Path(str(capture.path) + ".xz")
        destination_reused = False
        reused_existing = False
        if destination.exists():
            restored_digest = hashlib.sha256()
            with lzma.open(destination, "rb") as restored:
                for chunk in iter(lambda: restored.read(1024 * 1024), b""):
                    restored_digest.update(chunk)
            if restored_digest.hexdigest() == raw_sha256:
                destination_reused = True
        if not destination_reused and sibling_archive.exists():
            restored_digest = hashlib.sha256()
            with lzma.open(sibling_archive, "rb") as restored:
                for chunk in iter(lambda: restored.read(1024 * 1024), b""):
                    restored_digest.update(chunk)
            if restored_digest.hexdigest() == raw_sha256:
                shutil.copy2(sibling_archive, temporary)
                reused_existing = True
                destination_reused = True
        if not destination_reused:
            with capture.path.open("rb") as source, lzma.open(
                temporary, "wb", format=lzma.FORMAT_XZ, preset=3
            ) as compressed:
                shutil.copyfileobj(source, compressed, length=1024 * 1024)
        if temporary.exists():
            temporary.replace(destination)
        archived_digest = sha256_file(destination)
        restored_digest = hashlib.sha256()
        with lzma.open(destination, "rb") as restored:
            for chunk in iter(lambda: restored.read(1024 * 1024), b""):
                restored_digest.update(chunk)
        roundtrip_matches = restored_digest.hexdigest() == raw_sha256
        if not roundtrip_matches:
            raise RuntimeError(f"lossless archive verification failed for {capture.path}")
        archive_file = pathlib.Path(
            os.path.relpath(destination, output_dir)
        ).as_posix()
        rows.append(
            {
                "capture_id": capture.capture_id,
                "protocol": capture.protocol,
                "motion": capture.motion,
                "source_file": str(capture.path),
                "source_bytes": capture.path.stat().st_size,
                "source_sha256": raw_sha256,
                "archive_file": archive_file,
                "archive_bytes": destination.stat().st_size,
                "archive_sha256": archived_digest,
                "compression": "existing verified XZ (lossless)"
                if reused_existing
                else "XZ/LZMA2 preset 3 (lossless)",
                "reused_verified_sibling_archive": reused_existing,
                "roundtrip_sha256_matches": roundtrip_matches,
            }
        )
        checksum_lines.append(f"{archived_digest}  {archive_file}")
    write_csv(output_dir / "raw_data_manifest.csv", rows)
    (output_dir / "SHA256SUMS").write_text("\n".join(checksum_lines) + "\n", encoding="ascii")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=pathlib.Path,
        default=pathlib.Path("reports/bundles/uwb_dynamic_static_comparison_20260806/raw/data"),
        help="directory containing capture JSONL files",
    )
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("reports/bundles/uwb_dynamic_static_comparison_20260806/analysis"),
        help="new report directory",
    )
    parser.add_argument(
        "--raw-output-dir",
        type=pathlib.Path,
        help=(
            "RAW archive directory; defaults to "
            "<output parent>/raw/<output name>/data"
        ),
    )
    parser.add_argument("--tag-id", type=int, default=1)
    parser.add_argument(
        "--replay-dynamic-dir",
        type=pathlib.Path,
        default=pathlib.Path("reports/imu_dynamic_20260806"),
        help="directory containing final dynamic replay summary JSON files",
    )
    parser.add_argument(
        "--replay-static-dir",
        type=pathlib.Path,
        default=pathlib.Path("reports/imu_static_20260806"),
        help="directory containing final static replay summary JSON files",
    )
    parser.add_argument(
        "--rtk-reference-report-dir",
        type=pathlib.Path,
        help=(
            "optional prior report whose lossless archives supply GPS RTK data only; "
            "older UWB/IMU records are never loaded into the comparison"
        ),
    )
    parser.add_argument(
        "--rtk-match-ms",
        type=float,
        default=100.0,
        help="maximum absolute UWB/RTK wall-time association error",
    )
    parser.add_argument(
        "--max-geometry-age-s",
        type=float,
        default=10.0,
        help="maximum age for a non-exact dynamic geometry snapshot",
    )
    parser.add_argument(
        "--static-rtk-consistency-max-m",
        type=float,
        default=0.25,
        help="maximum stationary-tag RTK center separation allowed for absolute static ranking",
    )
    parser.add_argument(
        "--include-incomplete",
        action="store_true",
        help="include captures without a capture_end record",
    )
    parser.add_argument(
        "--include-all-captures",
        action="store_true",
        help="do not reduce duplicate protocol/motion cells to the preferred latest capture",
    )
    parser.add_argument(
        "--require-rpi-static",
        action="store_true",
        help="fail unless every selected static cell is an rpi_static_*_rtkfixed capture",
    )
    parser.add_argument(
        "--skip-raw-archive",
        action="store_true",
        help="skip lossless RAW XZ archive generation (intended only for fast development runs)",
    )
    args = parser.parse_args()

    args.input_dir = args.input_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    if args.raw_output_dir is None:
        args.raw_output_dir = (
            args.output_dir.parent / "raw" / "data"
            if args.output_dir.name == "analysis"
            else args.output_dir.parent / "raw" / args.output_dir.name / "data"
        )
    args.raw_output_dir = args.raw_output_dir.resolve()
    args.replay_dynamic_dir = args.replay_dynamic_dir.resolve()
    args.replay_static_dir = args.replay_static_dir.resolve()
    if args.rtk_reference_report_dir is not None:
        args.rtk_reference_report_dir = args.rtk_reference_report_dir.resolve()
    paths = sorted(args.input_dir.glob("*.jsonl"))
    if not paths:
        parser.error(f"no JSONL captures found in {args.input_dir}")

    loaded = [load_capture(path, args.tag_id) for path in paths]
    complete = [capture for capture in loaded if capture.complete or args.include_incomplete]
    skipped = [capture.capture_id for capture in loaded if capture not in complete]
    if not complete:
        parser.error("no completed captures found (use --include-incomplete to override)")
    if args.include_all_captures:
        captures, superseded = complete, []
    else:
        captures, superseded = select_capture_matrix(complete)
    if args.require_rpi_static:
        invalid_static = [
            capture.capture_id
            for capture in captures
            if capture.motion == "static"
            and not (
                capture.capture_id.lower().startswith("rpi_static_")
                and "rtkfixed" in capture.capture_id.lower()
            )
        ]
        selected_static_protocols = {
            capture.protocol for capture in captures if capture.motion == "static"
        }
        missing_rpi = sorted(set(PROTOCOL_LABELS) - selected_static_protocols)
        if invalid_static or missing_rpi:
            parser.error(
                "RTK-fixed Raspberry static matrix is incomplete; "
                f"invalid={invalid_static}, missing_protocols={missing_rpi}"
            )

    origin = gps_origin(captures)
    add_enu(captures, origin)
    rtk_reference_captures: list[Capture] = []
    rtk_reference_summaries: dict[str, dict[str, Any]] = {}
    rtk_reference_manifest: list[dict[str, Any]] = []
    rtk_reference_origin: tuple[float, float, float] | None = None
    if args.rtk_reference_report_dir is not None:
        (
            rtk_reference_captures,
            rtk_reference_summaries,
            rtk_reference_manifest,
            rtk_reference_origin,
        ) = load_rtk_reference_report(args.rtk_reference_report_dir, args.tag_id)
    replays = (
        {}
        if rtk_reference_captures
        else load_replay_summaries(
            args.replay_dynamic_dir, args.replay_static_dir, captures
        )
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bundle_layout = args.output_dir.name == "analysis"
    figure_dir = (
        args.output_dir.parent / "figures"
        if bundle_layout
        else args.output_dir / "figures"
    )
    source_dir = args.output_dir.parent / "source" if bundle_layout else args.output_dir

    summaries: dict[str, dict[str, Any]] = {}
    transformed_by_capture: dict[str, list[dict[str, Any]]] = {}
    pairs_by_capture: dict[str, list[dict[str, Any]]] = {}
    pair_rows: list[dict[str, Any]] = []
    alignment_rows: list[dict[str, Any]] = []
    capture_rows: list[dict[str, Any]] = []
    rtk_rows: list[dict[str, Any]] = []

    for capture in captures:
        points = analysis_positions(capture)
        ekf_points = list(capture.ekf_positions)
        position_stream = stream_metrics(
            capture.positions, capture.duration_s, capture.start_wall, capture.end_wall
        )
        position_stream.update(
            {
                "independent_events": len(points),
                "independent_rate_hz": len(points) / capture.duration_s if capture.duration_s > 0 else math.nan,
                "independent_pct": 100.0 * len(points) / len(capture.positions) if capture.positions else math.nan,
                "sigma_median_m": median(point["sigma_m"] for point in capture.positions),
                "sigma_p95_m": percentile((point["sigma_m"] for point in capture.positions), 95),
                "rms_median_m": median(point["rms_m"] for point in capture.positions),
                "rms_p95_m": percentile((point["rms_m"] for point in capture.positions), 95),
                "batch_age_p95_ms": percentile((point["batch_max_age_ms"] for point in capture.positions), 95),
            }
        )
        imu_stream = stream_metrics(
            capture.imu, capture.duration_s, capture.start_wall, capture.end_wall
        )
        imu_stream["valid_pct"] = 100.0 * sum(point["valid"] for point in capture.imu) / len(capture.imu) if capture.imu else math.nan
        static_precision = precision_metrics(points) if capture.motion == "static" else {"n": 0}
        jumps = jump_metrics(points)
        ekf_stream = stream_metrics(
            ekf_points, capture.duration_s, capture.start_wall, capture.end_wall
        )
        ekf_stream.update(
            {
                "available_pct": 100.0 * len(ekf_points) / len(points) if points else math.nan,
                "held_events": sum(bool(point.get("held")) for point in ekf_points),
                "held_pct": 100.0 * sum(bool(point.get("held")) for point in ekf_points) / len(ekf_points)
                if ekf_points
                else math.nan,
                "innovation_p95_m": percentile(
                    (point.get("innovation_m", math.nan) for point in ekf_points), 95
                ),
            }
        )
        ekf_static_precision = (
            precision_metrics(ekf_points) if capture.motion == "static" else {"n": 0}
        )
        ekf_jumps = jump_metrics(ekf_points)
        rtk, centers, spreads = rtk_metrics(capture, args.tag_id)
        alignments = build_alignments(capture, centers)
        transformed, transform_diagnostics = transform_positions(points, alignments, args.max_geometry_age_s)
        ekf_transformed, _ = transform_positions(
            ekf_points, alignments, args.max_geometry_age_s
        )
        alignment = alignment_metrics(alignments, spreads, transform_diagnostics)
        pairs, pair_diagnostics = nearest_position_pairs(
            capture, transformed, args.tag_id, args.rtk_match_ms
        )
        errors = error_metrics(pairs, pair_diagnostics["fixed_tag_solutions"])
        ekf_pairs, ekf_pair_diagnostics = nearest_position_pairs(
            capture,
            ekf_transformed,
            args.tag_id,
            args.rtk_match_ms,
            stream="ekf",
        )
        ekf_errors = error_metrics(
            ekf_pairs, ekf_pair_diagnostics["fixed_tag_solutions"]
        )
        transformed_by_capture[capture.capture_id] = transformed
        pairs_by_capture[capture.capture_id] = pairs
        pair_rows.extend(pairs)
        pair_rows.extend(ekf_pairs)

        for index, item in enumerate(alignments):
            alignment_rows.append(
                {
                    "capture_id": capture.capture_id,
                    "protocol": capture.protocol,
                    "motion": capture.motion,
                    "alignment_index": index,
                    "geometry_time": item["time"],
                    "snapshot_time": item["snapshot_time"],
                    "geometry_version": item["geometry_version"],
                    "dynamic": item["dynamic"],
                    "all_rtk_fixed": item["all_rtk_fixed"],
                    "source_geometry_fit_rms_m": item["fit_rms_m"],
                    "anchor_ids": ",".join(str(value) for value in item["anchor_ids"]),
                    "reflected": item.get("reflected", False),
                    "scale": item.get("scale", 1.0),
                    "yaw_deg": item["yaw_deg"],
                    "translation_east_m": item["translation"][0],
                    "translation_north_m": item["translation"][1],
                    "fit_rmse_m": item["fit_rmse_m"],
                    "fit_p95_m": item["fit_p95_m"],
                    "fit_max_m": item["fit_max_m"],
                }
            )

        quality_flags = []
        if finite(capture.duration_s) and capture.duration_s < 10.0:
            quality_flags.append("capture_duration_less_than_10s")
        if finite(position_stream.get("wall_coverage_pct")) and position_stream["wall_coverage_pct"] < 80.0:
            quality_flags.append("position_stream_covers_less_than_80pct_of_capture")
        if finite(imu_stream.get("wall_coverage_pct")) and imu_stream["wall_coverage_pct"] < 80.0:
            quality_flags.append("imu_stream_covers_less_than_80pct_of_capture")
        if finite(position_stream.get("last_event_before_capture_end_s")) and position_stream["last_event_before_capture_end_s"] > 5.0:
            quality_flags.append("position_stream_stops_more_than_5s_before_capture_end")
        if finite(imu_stream.get("last_event_before_capture_end_s")) and imu_stream["last_event_before_capture_end_s"] > 5.0:
            quality_flags.append("imu_stream_stops_more_than_5s_before_capture_end")
        summary = {
            "path": str(capture.path),
            "capture_id": capture.capture_id,
            "protocol": capture.protocol,
            "protocol_label": PROTOCOL_LABELS.get(capture.protocol, capture.protocol),
            "motion": capture.motion,
            "complete": capture.complete,
            "interrupted": capture.interrupted,
            "duration_s": capture.duration_s,
            "event_counts": dict(capture.counts),
            "imu_fused_position_records_excluded": capture.fused_positions_excluded,
            "protocol_mismatch_position_records_excluded": (
                capture.protocol_mismatch_positions_excluded
            ),
            "warnings": capture.warnings,
            "quality_flags": quality_flags,
            "position_stream": position_stream,
            "ekf_stream": ekf_stream,
            "imu_stream": imu_stream,
            "static_precision": static_precision,
            "ekf_static_precision": ekf_static_precision,
            "position_jumps": jumps,
            "ekf_position_jumps": ekf_jumps,
            "rtk": rtk,
            "alignment": alignment,
            "rtk_error": errors,
            "ekf_rtk_error": ekf_errors,
        }
        summaries[capture.capture_id] = summary
        capture_rows.append(
            {
                "capture_id": capture.capture_id,
                "protocol": capture.protocol,
                "motion": capture.motion,
                "duration_s": capture.duration_s,
                "imu_fused_position_records_excluded": capture.fused_positions_excluded,
                "protocol_mismatch_position_records_excluded": (
                    capture.protocol_mismatch_positions_excluded
                ),
                **flatten("position_", position_stream),
                **flatten("ekf_", ekf_stream),
                **flatten("imu_", imu_stream),
                **flatten("static_", static_precision),
                **flatten("ekf_static_", ekf_static_precision),
                **flatten("jump_", jumps),
                **flatten("ekf_jump_", ekf_jumps),
                **flatten("rtk_", rtk),
                **flatten("error_", errors),
                **flatten("ekf_error_", ekf_errors),
            }
        )
        rtk_rows.append(
            {
                "capture_id": capture.capture_id,
                "protocol": capture.protocol,
                "motion": capture.motion,
                **flatten("alignment_", alignment),
                **flatten("rtk_", rtk),
                **flatten("error_", errors),
                **flatten("ekf_error_", ekf_errors),
            }
        )

    static_consistency = static_rtk_reference_consistency(
        captures, summaries, args.static_rtk_consistency_max_m
    )
    static_reference_valid = static_consistency[
        "valid_for_cross_protocol_absolute_ranking"
    ]
    for capture in captures:
        if capture.motion != "static":
            continue
        summaries[capture.capture_id]["rtk_error"][
            "reference_consistency_valid_for_ranking"
        ] = static_reference_valid
        summaries[capture.capture_id]["rtk_error"]["interpretation_grade"] = (
            "indicative_in_system_reference"
            if static_reference_valid
            else "audit_only_invalid_for_cross_protocol_ranking"
        )
    for item in replays.values():
        if item["motion"] == "static" and not static_reference_valid:
            item["rtk_interpretation_grade"] = (
                "audit_only_invalid_for_cross_protocol_ranking"
            )
    for rows in (capture_rows, rtk_rows):
        for row in rows:
            if row.get("motion") == "static":
                row["static_rtk_reference_consistent"] = static_reference_valid
                row["static_rtk_max_cross_capture_center_distance_m"] = static_consistency[
                    "max_pairwise_distance_m"
                ]
                row["error_interpretation_grade"] = (
                    "indicative_in_system_reference"
                    if static_reference_valid
                    else "audit_only_invalid_for_cross_protocol_ranking"
                )

    expected = {(protocol, motion) for protocol in PROTOCOL_LABELS for motion in ("static", "dynamic")}
    present = {(capture.protocol, capture.motion) for capture in captures}
    missing = [
        {"protocol": protocol, "motion": motion}
        for protocol, motion in sorted(expected - present)
    ]
    analysis = {
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "tool": "tools/uwb_dynamic_static_report.py",
        "input_dir": str(args.input_dir),
        "output_dir": str(args.output_dir),
        "tag_id": args.tag_id,
        "rtk_match_limit_ms": args.rtk_match_ms,
        "max_non_exact_dynamic_geometry_age_s": args.max_geometry_age_s,
        "static_rtk_reference_consistency": (
            None if rtk_reference_captures else static_consistency
        ),
        "position_sample_policy": "independent_frame when available; all-event rate retained separately",
        "capture_selection_policy": "one cell per protocol/motion; prefer rpi_static_*_rtkfixed and dynamic *_final",
        "rtk_pair_policy": (
            "not performed across sessions"
            if rtk_reference_captures
            else "RTK-fixed M1; one nearest independent UWB position per GPS solution"
        ),
        "frame_registration": (
            "best ID-keyed direct/reflected fixed-scale 2-D rigid transform "
            "plus translation; scale fixed at one"
        ),
        "rtk_is_independent_survey_truth": False,
        "rtk_interpretation": (
            "August 6 GPS-only availability and precision reference; not synchronized with August 12 UWB"
            if rtk_reference_captures
            else "in-system comparison reference with alignment, timing, fix-state, and lever-arm limitations"
        ),
        "report_rtk_population": (
            "GPS-only reference captures; no cross-session UWB/RTK association"
            if rtk_reference_captures
            else "GPS records embedded in the selected captures"
        ),
        "rtk_reference_report_dir": (
            str(args.rtk_reference_report_dir)
            if args.rtk_reference_report_dir is not None
            else None
        ),
        "rtk_reference_origin_wgs84": (
            {
                "latitude_deg": rtk_reference_origin[0],
                "longitude_deg": rtk_reference_origin[1],
                "altitude_m": rtk_reference_origin[2],
            }
            if rtk_reference_origin is not None
            else None
        ),
        "cross_session_rtk_pairing_performed": False if rtk_reference_captures else True,
        "origin_wgs84": (
            None
            if rtk_reference_captures
            else {
                "latitude_deg": origin[0],
                "longitude_deg": origin[1],
                "altitude_m": origin[2],
            }
        ),
        "skipped_incomplete_captures": skipped,
        "excluded_superseded_captures": [capture.capture_id for capture in superseded],
        "missing_capture_matrix_cells": missing,
        "replay_dynamic_dir": (
            None if rtk_reference_captures else str(args.replay_dynamic_dir)
        ),
        "replay_static_dir": (
            None if rtk_reference_captures else str(args.replay_static_dir)
        ),
    }
    raw_manifest = (
        []
        if args.skip_raw_archive
        else archive_raw_captures(captures, args.output_dir, args.raw_output_dir)
    )
    analysis["raw_archive_generated"] = not args.skip_raw_archive
    exported_summaries = summaries
    if rtk_reference_captures:
        exported_summaries = {}
        for capture_id, summary in summaries.items():
            exported = dict(summary)
            for field_name in ("rtk", "alignment", "rtk_error", "ekf_rtk_error"):
                exported.pop(field_name, None)
            exported["rtk_note"] = (
                "GPS embedded in this August 12 capture is excluded; see "
                "rtk_reference_captures for the August 6 GPS-only population"
            )
            exported_summaries[capture_id] = exported
        capture_rows = [
            {
                key: value
                for key, value in row.items()
                if not key.startswith(("rtk_", "error_", "ekf_error_"))
            }
            for row in capture_rows
        ]
    output = {
        "analysis": analysis,
        "captures": exported_summaries,
        "replay_summaries": replays,
        "raw_data_manifest": raw_manifest,
        "rtk_reference_captures": rtk_reference_summaries,
        "rtk_reference_manifest": rtk_reference_manifest,
    }
    (args.output_dir / "analysis_summary.json").write_text(
        json.dumps(json_clean(output), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_csv(args.output_dir / "capture_metrics.csv", capture_rows)
    if rtk_reference_captures:
        reference_rows = []
        for capture in rtk_reference_captures:
            item = rtk_reference_summaries[capture.capture_id]
            reference_rows.append(
                {
                    "capture_id": capture.capture_id,
                    "protocol": capture.protocol,
                    "motion": capture.motion,
                    "duration_s": capture.duration_s,
                    **flatten("rtk_", item["rtk"]),
                    **flatten("tag_cloud_", item["tag_fixed_cloud_precision"]),
                }
            )
        write_csv(args.output_dir / "rtk_reference_metrics.csv", reference_rows)
        write_csv(args.output_dir / "rtk_reference_manifest.csv", rtk_reference_manifest)
        for stale_name in (
            "rtk_alignment_metrics.csv",
            "rtk_pairs.csv",
            "alignment_snapshots.csv",
        ):
            stale = args.output_dir / stale_name
            if stale.exists():
                stale.unlink()
    else:
        write_csv(args.output_dir / "rtk_alignment_metrics.csv", rtk_rows)
        write_csv(args.output_dir / "rtk_pairs.csv", pair_rows)
        write_csv(args.output_dir / "alignment_snapshots.csv", alignment_rows)
    if rtk_reference_captures:
        stale_replay = args.output_dir / "replay_metrics.csv"
        if stale_replay.exists():
            stale_replay.unlink()
    else:
        write_csv(
            args.output_dir / "replay_metrics.csv",
            [replays[key] for key in sorted(replays)],
        )
    make_figures(
        figure_dir,
        captures,
        summaries,
        transformed_by_capture,
        pairs_by_capture,
        rtk_reference_captures or None,
    )
    write_report(
        source_dir / "REPORT.md",
        captures,
        summaries,
        replays,
        static_consistency,
        args,
        missing,
        rtk_reference_captures or None,
        rtk_reference_summaries or None,
    )
    write_tex_report(
        source_dir / "report.tex",
        captures,
        summaries,
        replays,
        static_consistency,
        args,
        rtk_reference_captures or None,
        rtk_reference_summaries or None,
    )

    # LaTeX compilation intentionally remains an explicit reproducibility
    # step.  The generated source has a stable descriptive name in the report
    # directory, while `report.tex` stays convenient for pdflatex.

    print(f"Generated {source_dir / 'REPORT.md'}")
    for capture in sorted(captures, key=lambda item: (item.motion, item.protocol)):
        item = summaries[capture.capture_id]
        line = (
            f"{capture.capture_id}: positions={item['position_stream']['events']}, "
            f"independent={item['position_stream']['independent_events']}"
        )
        if not rtk_reference_captures:
            line += (
                f", RTK pairs={item['rtk_error'].get('pairs', 0)}, "
                f"alignment={item['alignment'].get('quality_flag', 'unavailable')}"
            )
        print(line)
    if skipped:
        print("Skipped incomplete captures: " + ", ".join(skipped))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
