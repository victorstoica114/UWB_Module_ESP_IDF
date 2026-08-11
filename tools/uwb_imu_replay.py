#!/usr/bin/env python3
"""Offline replay and RTK scoring for dynamic UWB + BNO085 captures."""

from __future__ import annotations

import argparse
import bisect
import collections
import json
import lzma
import math
import pathlib
import statistics
import sys
from dataclasses import asdict, dataclass
from typing import Any, Iterable, Mapping, Sequence

from uwb_imu_fusion import FusionConfig, UwbImuFusion


EARTH_RADIUS_M = 6378137.0
DEG_TO_RAD = math.pi / 180.0
PROTOCOL_ALIASES = {
    "flex_tdoa": "flextdoa",
    "flextdoa": "flextdoa",
    "passive": "passive_ds",
    "passive_ds": "passive_ds",
    "native": "native_ds",
    "native_ds": "native_ds",
    "ds_twr": "native_ds",
}


@dataclass(frozen=True, order=True)
class TrackKey:
    protocol: str
    tag_id: int
    module_id: int


@dataclass(frozen=True)
class RigidTransform2D:
    origin_latitude_deg: float
    origin_longitude_deg: float
    cosine: float
    sine: float
    translate_x_m: float
    translate_y_m: float

    def gps_to_uwb(self, latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
        east_m, north_m = gps_to_enu(
            latitude_deg,
            longitude_deg,
            self.origin_latitude_deg,
            self.origin_longitude_deg,
        )
        return (
            self.cosine * east_m - self.sine * north_m + self.translate_x_m,
            self.sine * east_m + self.cosine * north_m + self.translate_y_m,
        )

    def course_to_uwb_heading(self, course_deg: float) -> float:
        """Rotate an NMEA course (north, clockwise) into the UWB frame."""

        course_rad = math.radians(course_deg)
        east = math.sin(course_rad)
        north = math.cos(course_rad)
        direction_x = self.cosine * east - self.sine * north
        direction_y = self.sine * east + self.cosine * north
        return math.atan2(direction_y, direction_x)


def normalize_protocol(value: Any) -> str:
    clean = str(value or "").strip().lower()
    return PROTOCOL_ALIASES.get(clean, clean)


def finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _record_relevant_for_track(
    record: Mapping[str, Any], protocol: str, module_id: int
) -> bool:
    """Keep only records needed to replay one high-rate tag track.

    Field captures contain 500 Hz IMU telemetry from every module. Loading all
    of it can turn a one-minute replay into several gigabytes of Python objects,
    although a track needs IMU data only from its tag. Anchor GPS and geometry
    snapshots remain global because they define the RTK alignment.
    """

    kind = str(record.get("kind") or "")
    if kind in {"capture_start", "capture_end", "dashboard_snapshot", "gps_fix"}:
        return True
    if kind == "accel":
        return (
            int(record.get("module_id") or 0) == module_id
            and normalize_protocol(record.get("protocol")) == protocol
        )
    if kind == "position":
        record_module_id = int(
            record.get("module_id")
            or record.get("tag_id")
            or record.get("tag")
            or 0
        )
        return (
            record_module_id == module_id
            and normalize_protocol(
                record.get("tdoa_protocol") or record.get("protocol")
            )
            == protocol
            and record.get("imu_fused") is not True
        )
    return False


def load_jsonl(
    paths: Sequence[pathlib.Path],
    *,
    protocol: str | None = None,
    module_id: int | None = None,
) -> list[dict[str, Any]]:
    selected_protocol = normalize_protocol(protocol) if protocol else None
    if (selected_protocol is None) != (module_id is None):
        raise ValueError("protocol and module_id must be provided together")
    records: list[dict[str, Any]] = []
    index = 0
    for path in paths:
        if path.suffix.lower() == ".xz":
            handle_context = lzma.open(path, "rt", encoding="utf-8")
        else:
            handle_context = path.open("r", encoding="utf-8")
        with handle_context as handle:
            for line_number, line in enumerate(handle, 1):
                clean = line.strip()
                if not clean:
                    continue
                try:
                    record = json.loads(clean)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_number}: {exc}") from exc
                if not isinstance(record, dict):
                    raise ValueError(f"{path}:{line_number}: expected JSON object")
                if (
                    selected_protocol is not None
                    and module_id is not None
                    and not _record_relevant_for_track(
                        record, selected_protocol, module_id
                    )
                ):
                    continue
                record = dict(record)
                record["_input_index"] = index
                record["_input_file"] = str(path)
                records.append(record)
                index += 1
    return records


def record_protocol(record: dict[str, Any]) -> str:
    return normalize_protocol(
        record.get("tdoa_protocol") or record.get("protocol")
    )


def raw_position_xy(record: dict[str, Any]) -> tuple[float, float] | None:
    x_m = finite_float(
        record.get("raw_x_m")
        if record.get("raw_x_m") is not None
        else record.get("raw_x")
        if record.get("raw_x") is not None
        else record.get("x_m")
        if record.get("x_m") is not None
        else record.get("x")
    )
    y_m = finite_float(
        record.get("raw_y_m")
        if record.get("raw_y_m") is not None
        else record.get("raw_y")
        if record.get("raw_y") is not None
        else record.get("y_m")
        if record.get("y_m") is not None
        else record.get("y")
    )
    return (x_m, y_m) if x_m is not None and y_m is not None else None


def position_track(record: dict[str, Any]) -> TrackKey | None:
    # New captures label dashboard-fused output separately.  The explicit
    # imu_fused check is required for legacy/in-progress captures where those
    # events were still written as kind=position.
    if (
        record.get("kind") != "position"
        or record.get("imu_fused") is True
        or raw_position_xy(record) is None
    ):
        return None
    protocol = record_protocol(record)
    # Passive DS-TWR can emit overlapping rolling-window solutions between
    # complete coherent stars.  Those rows are useful live continuity
    # diagnostics, but they are strongly correlated and must not be counted as
    # independent position measurements by the offline accuracy benchmark.
    solution_kind = str(record.get("solution_kind") or "")
    if (
        protocol == "passive_ds"
        and solution_kind
        and solution_kind != "independent_frame"
    ):
        return None
    tag_id = int(record.get("tag_id") or record.get("tag") or 0)
    module_id = int(record.get("module_id") or tag_id)
    if protocol not in PROTOCOL_ALIASES.values() or tag_id <= 0 or module_id <= 0:
        return None
    if record.get("uptime_ms") is None:
        return None
    return TrackKey(protocol, tag_id, module_id)


def collector_wall_ns(record: dict[str, Any]) -> int:
    value = record.get("collector_wall_ns")
    if value is not None:
        return int(value)
    received_at = finite_float(record.get("received_at"))
    return int(received_at * 1_000_000_000) if received_at is not None else 0


def gps_wall_ns(record: dict[str, Any]) -> int:
    return int(
        record.get("estimated_measurement_wall_ns")
        or record.get("collector_wall_ns")
        or 0
    )


def position_wall_ns(record: dict[str, Any]) -> int:
    """Best available wall clock for when a position reached the dashboard.

    Position events can wait in the dashboard queue before the offline collector
    drains them.  ``received_at`` is stamped when the dashboard receives the
    telemetry event, whereas ``collector_wall_ns`` is stamped later for the
    entire drained batch.  RTK measurement timestamps therefore need the former
    when it is available.
    """

    received_at = finite_float(record.get("received_at"))
    if received_at is not None and received_at > 0.0:
        return int(round(received_at * 1_000_000_000.0))
    return collector_wall_ns(record)


def valid_gps(record: dict[str, Any], *, require_fixed: bool = True) -> bool:
    if record.get("kind") != "gps_fix":
        return False
    if require_fixed and int(record.get("gps_fix_quality") or 0) != 4:
        return False
    if record.get("gps_fix_valid") is False:
        return False
    return (
        int(record.get("module_id") or 0) > 0
        and finite_float(record.get("gps_latitude_deg")) is not None
        and finite_float(record.get("gps_longitude_deg")) is not None
    )


def gps_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
) -> tuple[float, float]:
    latitude = latitude_deg * DEG_TO_RAD
    longitude = longitude_deg * DEG_TO_RAD
    origin_latitude = origin_latitude_deg * DEG_TO_RAD
    origin_longitude = origin_longitude_deg * DEG_TO_RAD
    east = (
        (longitude - origin_longitude)
        * EARTH_RADIUS_M
        * math.cos(0.5 * (latitude + origin_latitude))
    )
    north = (latitude - origin_latitude) * EARTH_RADIUS_M
    return east, north


def rigid_fit(
    gps_rows: Sequence[dict[str, Any]],
    uwb_by_module: dict[int, tuple[float, float]],
) -> tuple[RigidTransform2D, float, list[int]] | None:
    usable = [
        row
        for row in gps_rows
        if int(row.get("module_id") or 0) in uwb_by_module
        and valid_gps(row)
    ]
    by_module: dict[int, dict[str, Any]] = {}
    for row in usable:
        by_module[int(row["module_id"])] = row
    if len(by_module) < 3:
        return None
    module_ids = sorted(by_module)
    origin_lat = float(by_module[module_ids[0]]["gps_latitude_deg"])
    origin_lon = float(by_module[module_ids[0]]["gps_longitude_deg"])
    gps_points = [
        gps_to_enu(
            float(by_module[module_id]["gps_latitude_deg"]),
            float(by_module[module_id]["gps_longitude_deg"]),
            origin_lat,
            origin_lon,
        )
        for module_id in module_ids
    ]
    uwb_points = [uwb_by_module[module_id] for module_id in module_ids]
    gps_center = (
        statistics.fmean(point[0] for point in gps_points),
        statistics.fmean(point[1] for point in gps_points),
    )
    uwb_center = (
        statistics.fmean(point[0] for point in uwb_points),
        statistics.fmean(point[1] for point in uwb_points),
    )
    dot = 0.0
    cross = 0.0
    for gps_point, uwb_point in zip(gps_points, uwb_points):
        gx = gps_point[0] - gps_center[0]
        gy = gps_point[1] - gps_center[1]
        ux = uwb_point[0] - uwb_center[0]
        uy = uwb_point[1] - uwb_center[1]
        dot += gx * ux + gy * uy
        cross += gx * uy - gy * ux
    if math.hypot(dot, cross) < 1e-9:
        return None
    angle = math.atan2(cross, dot)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    translate_x = uwb_center[0] - (
        cosine * gps_center[0] - sine * gps_center[1]
    )
    translate_y = uwb_center[1] - (
        sine * gps_center[0] + cosine * gps_center[1]
    )
    transform = RigidTransform2D(
        origin_lat,
        origin_lon,
        cosine,
        sine,
        translate_x,
        translate_y,
    )
    squared = 0.0
    for module_id, gps_point in zip(module_ids, gps_points):
        predicted = (
            cosine * gps_point[0] - sine * gps_point[1] + translate_x,
            sine * gps_point[0] + cosine * gps_point[1] + translate_y,
        )
        target = uwb_by_module[module_id]
        squared += (predicted[0] - target[0]) ** 2 + (
            predicted[1] - target[1]
        ) ** 2
    return transform, math.sqrt(squared / len(module_ids)), module_ids


def geometry_candidates(
    records: Sequence[dict[str, Any]], protocol: str, tag_id: int
) -> list[tuple[int, dict[int, tuple[float, float]], str, bool]]:
    candidates: list[tuple[int, dict[int, tuple[float, float]], str, bool]] = []
    for record in records:
        wall_ns = collector_wall_ns(record)
        if record.get("kind") == "dashboard_snapshot":
            snapshot = record.get("snapshot") or {}
            geometries = ((snapshot.get("tdoa") or {}).get("local_geometries") or {})
            for geometry in geometries.values():
                if not isinstance(geometry, dict):
                    continue
                if record_protocol(geometry) != protocol:
                    continue
                if int(geometry.get("tag_id") or tag_id) != tag_id:
                    continue
                anchors: dict[int, tuple[float, float]] = {}
                for anchor in (geometry.get("anchors") or {}).values():
                    if not isinstance(anchor, dict):
                        continue
                    anchor_id = int(anchor.get("id") or 0)
                    x_m = finite_float(anchor.get("x"))
                    y_m = finite_float(anchor.get("y"))
                    if anchor_id > 0 and x_m is not None and y_m is not None:
                        anchors[anchor_id] = x_m, y_m
                if len(anchors) >= 3:
                    candidates.append(
                        (
                            wall_ns,
                            anchors,
                            "telemetry_geometry",
                            bool(geometry.get("all_rtk_fixed")),
                        )
                    )
        statuses: Iterable[dict[str, Any]] = []
        if record.get("kind") == "capture_start":
            statuses = record.get("initial_status") or []
        elif record.get("kind") == "dashboard_snapshot":
            statuses = (record.get("snapshot") or {}).get("statuses") or []
        for status in statuses:
            ids = status.get("runtime_anchor_ids") or []
            xs = status.get("runtime_flex_tdoa_anchor_x_mm") or []
            ys = status.get("runtime_flex_tdoa_anchor_y_mm") or []
            if len(ids) < 3 or len(xs) != len(ids) or len(ys) != len(ids):
                continue
            anchors = {
                int(anchor_id): (float(x_mm) / 1000.0, float(y_mm) / 1000.0)
                for anchor_id, x_mm, y_mm in zip(ids, xs, ys)
                if int(anchor_id) > 0
            }
            if len(anchors) >= 3:
                candidates.append(
                    (
                        wall_ns,
                        anchors,
                        "runtime_fixed_geometry",
                        bool(status.get("runtime_flex_tdoa_geometry_fixed")),
                    )
                )
                break
    return candidates


def nearest_gps_rows(
    gps_by_module: dict[int, list[dict[str, Any]]],
    module_ids: Iterable[int],
    wall_ns: int,
) -> tuple[list[dict[str, Any]], int]:
    rows = []
    maximum_age_ns = 0
    for module_id in module_ids:
        module_rows = gps_by_module.get(module_id) or []
        if not module_rows:
            continue
        times = [gps_wall_ns(row) for row in module_rows]
        index = bisect.bisect_left(times, wall_ns)
        choices = module_rows[max(0, index - 1) : min(len(module_rows), index + 1)]
        if not choices:
            continue
        nearest = min(choices, key=lambda row: abs(gps_wall_ns(row) - wall_ns))
        rows.append(nearest)
        maximum_age_ns = max(maximum_age_ns, abs(gps_wall_ns(nearest) - wall_ns))
    return rows, maximum_age_ns


def build_rtk_alignment(
    records: Sequence[dict[str, Any]], protocol: str, tag_id: int
) -> tuple[RigidTransform2D | None, dict[str, Any]]:
    gps_by_module: dict[int, list[dict[str, Any]]] = collections.defaultdict(list)
    for record in records:
        if valid_gps(record):
            gps_by_module[int(record["module_id"])].append(record)
    for rows in gps_by_module.values():
        rows.sort(key=gps_wall_ns)
    fitted = []
    for wall_ns, anchors, source, all_fixed in geometry_candidates(
        records, protocol, tag_id
    ):
        gps_rows, maximum_age_ns = nearest_gps_rows(
            gps_by_module, anchors, wall_ns
        )
        fit = rigid_fit(gps_rows, anchors)
        if fit is None:
            continue
        transform, fit_rms_m, module_ids = fit
        fitted.append(
            (
                (0 if all_fixed else 1, maximum_age_ns, fit_rms_m),
                transform,
                {
                    "available": True,
                    "source": source,
                    "all_anchor_geometry_rtk_fixed": all_fixed,
                    "anchor_ids": module_ids,
                    "anchor_count": len(module_ids),
                    "anchor_fit_rms_m": fit_rms_m,
                    "maximum_anchor_time_delta_ms": maximum_age_ns / 1_000_000.0,
                    "geometry_wall_ns": wall_ns,
                    "transform": asdict(transform),
                },
            )
        )
    if not fitted:
        return None, {
            "available": False,
            "reason": "need >=3 matching RTK-fixed anchors and UWB geometry",
        }
    fitted.sort(key=lambda item: item[0])
    return fitted[0][1], fitted[0][2]


def percentile(values: Sequence[float], percentage: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = percentage / 100.0 * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def error_metrics(values: Sequence[float]) -> dict[str, Any] | None:
    if not values:
        return None
    return {
        "count": len(values),
        "rmse_m": math.sqrt(statistics.fmean(value * value for value in values)),
        "mean_m": statistics.fmean(values),
        "median_m": statistics.median(values),
        "p95_m": percentile(values, 95.0),
        "p99_m": percentile(values, 99.0),
        "max_m": max(values),
    }


def timing_metrics(values_ms: Sequence[float], gap_ms: float | None) -> dict[str, Any]:
    ordered = sorted(values_ms)
    intervals = [
        float(later - earlier)
        for earlier, later in zip(ordered, ordered[1:])
        if later > earlier
    ]
    span_s = (ordered[-1] - ordered[0]) / 1000.0 if len(ordered) >= 2 else 0.0
    median_ms = statistics.median(intervals) if intervals else None
    threshold_ms = (
        gap_ms
        if gap_ms is not None
        else max(100.0, 3.0 * median_ms)
        if median_ms is not None
        else 100.0
    )
    return {
        "samples": len(ordered),
        "span_s": span_s,
        "rate_hz": (len(ordered) - 1) / span_s if span_s > 0 else None,
        "interval_median_ms": median_ms,
        "interval_p95_ms": percentile(intervals, 95.0),
        "interval_max_ms": max(intervals) if intervals else None,
        "gap_threshold_ms": threshold_ms,
        "gap_count": sum(interval > threshold_ms for interval in intervals),
    }


def fusion_acceptance(
    raw_metrics: Mapping[str, Any] | None,
    fused_metrics: Mapping[str, Any] | None,
    raw_timing: Mapping[str, Any],
    fused_timing: Mapping[str, Any],
    overshoot_tolerance_m: float,
) -> dict[str, Any]:
    """Apply the field acceptance contract without hiding missing evidence."""

    accuracy_available = raw_metrics is not None and fused_metrics is not None
    raw_gap_count = int(raw_timing.get("gap_count") or 0)
    fused_gap_count = int(fused_timing.get("gap_count") or 0)
    checks: dict[str, bool | None] = {
        "rmse_reduced": (
            float(fused_metrics["rmse_m"]) < float(raw_metrics["rmse_m"])
            if accuracy_available
            else None
        ),
        "p95_reduced": (
            float(fused_metrics["p95_m"]) < float(raw_metrics["p95_m"])
            if accuracy_available
            else None
        ),
        # If raw already has no detected dead zone, fusion must keep it at zero.
        "dead_zones_reduced_or_absent": (
            fused_gap_count < raw_gap_count
            if raw_gap_count > 0
            else fused_gap_count == 0
        ),
        "no_overshoot": (
            float(fused_metrics["max_m"])
            <= float(raw_metrics["max_m"]) + overshoot_tolerance_m
            and float(fused_metrics["p99_m"])
            <= float(raw_metrics["p99_m"]) + overshoot_tolerance_m
            if accuracy_available
            else None
        ),
    }
    accepted = accuracy_available and all(value is True for value in checks.values())
    return {
        "accepted": accepted,
        "accuracy_evidence_available": accuracy_available,
        "overshoot_tolerance_m": overshoot_tolerance_m,
        "checks": checks,
        "deltas": {
            "rmse_m": (
                float(fused_metrics["rmse_m"]) - float(raw_metrics["rmse_m"])
                if accuracy_available
                else None
            ),
            "p95_m": (
                float(fused_metrics["p95_m"]) - float(raw_metrics["p95_m"])
                if accuracy_available
                else None
            ),
            "p99_m": (
                float(fused_metrics["p99_m"]) - float(raw_metrics["p99_m"])
                if accuracy_available
                else None
            ),
            "max_m": (
                float(fused_metrics["max_m"]) - float(raw_metrics["max_m"])
                if accuracy_available
                else None
            ),
            "gap_count": fused_gap_count - raw_gap_count,
        },
    }


def ordered_track_events(
    records: Sequence[dict[str, Any]], key: TrackKey
) -> list[dict[str, Any]]:
    selected = []
    for record in records:
        if record.get("kind") == "accel":
            if int(record.get("module_id") or 0) != key.module_id:
                continue
            event_kind = "imu"
        elif position_track(record) == key:
            event_kind = "position"
        else:
            continue
        try:
            uptime_ms = int(record["uptime_ms"])
            sample_time_us = int(
                record.get("sample_time_us") or uptime_ms * 1000
            )
        except (KeyError, TypeError, ValueError):
            continue
        selected.append(
            {
                "record": record,
                "event_kind": event_kind,
                "uptime_ms": uptime_ms,
                "sample_time_us": sample_time_us,
                "input_index": int(record.get("_input_index") or 0),
            }
        )
    # The two endpoints are drained independently and can contain different
    # amounts of backlog.  Detect a reboot within each stream; comparing an IMU
    # timestamp with a position timestamp can otherwise invent alternating
    # reboot epochs and add repeated 2^32-ms jumps.
    epochs = {"imu": 0, "position": 0}
    previous_us: dict[str, int | None] = {"imu": None, "position": None}
    for event in sorted(selected, key=lambda item: item["input_index"]):
        event_kind = str(event["event_kind"])
        sample_time_us = int(event["sample_time_us"])
        previous = previous_us[event_kind]
        if previous is not None and sample_time_us + 1_000_000 < previous:
            epochs[event_kind] += 1
            previous_us[event_kind] = sample_time_us
        elif previous is None or sample_time_us > previous:
            # Ignore small backwards steps caused by polling order; retaining
            # the high-water mark keeps reboot detection stable.
            previous_us[event_kind] = sample_time_us
        event["sort_us"] = (
            epochs[event_kind] * (1 << 32) * 1000 + sample_time_us
        )
        event["sort_ms"] = event["sort_us"] / 1000.0
    return sorted(
        selected,
        key=lambda item: (
            item["sort_us"],
            0 if item["event_kind"] == "imu" else 1,
            item["input_index"],
        ),
    )


def nearest_tag_rtk(
    gps_rows: Sequence[dict[str, Any]], wall_ns: int, max_age_ms: float
) -> tuple[dict[str, Any] | None, float | None]:
    if not gps_rows or wall_ns <= 0:
        return None, None
    times = [gps_wall_ns(row) for row in gps_rows]
    index = bisect.bisect_left(times, wall_ns)
    choices = gps_rows[max(0, index - 1) : min(len(gps_rows), index + 1)]
    if not choices:
        return None, None
    nearest = min(choices, key=lambda row: abs(gps_wall_ns(row) - wall_ns))
    age_ms = abs(gps_wall_ns(nearest) - wall_ns) / 1_000_000.0
    return (nearest, age_ms) if age_ms <= max_age_ms else (None, age_ms)


def interpolate_track_at_rtk_fixes(
    position_samples: Sequence[dict[str, Any]],
    gps_rows: Sequence[dict[str, Any]],
    transform: RigidTransform2D | None,
    max_bracket_age_ms: float,
) -> list[dict[str, Any]]:
    """Interpolate raw and fused UWB at each unique RTK measurement time."""

    if transform is None:
        return []
    positions = sorted(
        [
            sample
            for sample in position_samples
            if int(sample.get("position_wall_ns") or 0) > 0
            and finite_float(sample.get("raw_x_m")) is not None
            and finite_float(sample.get("raw_y_m")) is not None
            and finite_float(sample.get("fused_x_m")) is not None
            and finite_float(sample.get("fused_y_m")) is not None
        ],
        key=lambda sample: int(sample["position_wall_ns"]),
    )
    if not positions:
        return []
    position_times = [int(sample["position_wall_ns"]) for sample in positions]
    comparisons = []
    seen_fixes: set[tuple[int, int]] = set()
    max_age_ns = int(max_bracket_age_ms * 1_000_000.0)
    for gps in gps_rows:
        fix_time_ns = gps_wall_ns(gps)
        fix_key = (fix_time_ns, int(gps.get("gps_gga_count") or 0))
        if fix_time_ns <= 0 or fix_key in seen_fixes:
            continue
        seen_fixes.add(fix_key)
        after_index = bisect.bisect_left(position_times, fix_time_ns)
        exact = (
            after_index < len(positions)
            and position_times[after_index] == fix_time_ns
        )
        if exact:
            before = positions[after_index]
            after = before
        else:
            if after_index <= 0:
                before = positions[0]
                after = before
            elif after_index >= len(positions):
                before = positions[-1]
                after = before
            else:
                before = positions[after_index - 1]
                after = positions[after_index]
        before_time_ns = int(before["position_wall_ns"])
        after_time_ns = int(after["position_wall_ns"])
        single_point = before is after
        if single_point:
            before_age_ns = abs(fix_time_ns - before_time_ns)
            after_age_ns = before_age_ns
        else:
            before_age_ns = fix_time_ns - before_time_ns
            after_age_ns = after_time_ns - fix_time_ns
        span_ns = after_time_ns - before_time_ns
        if (
            before_age_ns < 0
            or after_age_ns < 0
            or before_age_ns > max_age_ns
            or after_age_ns > max_age_ns
        ):
            continue
        weight = 0.0 if span_ns <= 0 else before_age_ns / span_ns

        def interpolate(name: str) -> float:
            start = float(before[name])
            return start + weight * (float(after[name]) - start)

        raw_x = interpolate("raw_x_m")
        raw_y = interpolate("raw_y_m")
        fused_x = interpolate("fused_x_m")
        fused_y = interpolate("fused_y_m")
        rtk_x, rtk_y = transform.gps_to_uwb(
            float(gps["gps_latitude_deg"]),
            float(gps["gps_longitude_deg"]),
        )
        raw_error = math.hypot(raw_x - rtk_x, raw_y - rtk_y)
        fused_error = math.hypot(fused_x - rtk_x, fused_y - rtk_y)
        comparisons.append(
            {
                "schema_version": 1,
                "kind": "replay_rtk_comparison",
                "protocol": before.get("protocol"),
                "tag_id": before.get("tag_id"),
                "module_id": before.get("module_id"),
                "rtk_wall_ns": fix_time_ns,
                "rtk_gga_count": fix_key[1],
                "rtk_x_m": rtk_x,
                "rtk_y_m": rtk_y,
                "raw_x_m": raw_x,
                "raw_y_m": raw_y,
                "fused_x_m": fused_x,
                "fused_y_m": fused_y,
                "raw_error_m": raw_error,
                "fused_error_m": fused_error,
                "before_position_wall_ns": before_time_ns,
                "after_position_wall_ns": after_time_ns,
                "before_age_ms": before_age_ns / 1_000_000.0,
                "after_age_ms": after_age_ns / 1_000_000.0,
                "interpolation_weight": weight,
            }
        )
    return comparisons


def replay_track(
    records: Sequence[dict[str, Any]],
    key: TrackKey,
    transform: RigidTransform2D | None,
    config: FusionConfig,
    rtk_max_age_ms: float,
    gap_ms: float | None,
    overshoot_tolerance_m: float,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    fusion = UwbImuFusion(config)
    events = ordered_track_events(records, key)
    tag_gps = sorted(
        [
            record
            for record in records
            if int(record.get("module_id") or 0) == key.module_id
            and valid_gps(record)
        ],
        key=gps_wall_ns,
    )
    samples = []
    imu_times = []
    position_times = []
    fused_stream_times = []
    fused_stream_time_keys: set[int] = set()
    gps_times = [gps_wall_ns(record) // 1_000_000 for record in tag_gps]
    imu_missing_orientation = 0
    imu_used = 0
    rtk_yaw_alignment_attempts = 0
    rtk_yaw_alignment_updates = 0
    last_yaw_gps_wall_ns = 0
    fusion_flag_counts: collections.Counter[str] = collections.Counter()

    def record_fused_stream_time(
        output: Mapping[str, Any], event: Mapping[str, Any]
    ) -> None:
        flags = {str(value) for value in output.get("flags", [])}
        if not bool(output.get("ready")) or flags & {
            "module_mismatch",
            "event_out_of_order",
            "imu_out_of_order",
            "imu_duplicate_time",
            "position_out_of_order",
            "position_duplicate_time",
        }:
            return
        sort_us = int(event["sort_us"])
        if sort_us in fused_stream_time_keys:
            return
        fused_stream_time_keys.add(sort_us)
        fused_stream_times.append(sort_us / 1000.0)

    for event in events:
        record = event["record"]
        if event["event_kind"] == "imu":
            if not all(
                record.get(name) is not None
                for name in ("quat_i", "quat_j", "quat_k", "quat_real")
            ):
                imu_missing_orientation += 1
                continue
            try:
                output = fusion.update_imu(record)
            except (TypeError, ValueError):
                imu_missing_orientation += 1
                continue
            imu_used += 1
            fusion_flag_counts.update(str(value) for value in output.get("flags", ()))
            imu_times.append(float(event["sort_ms"]))
            record_fused_stream_time(output, event)
            imu_wall_ns = collector_wall_ns(record)
            yaw_gps, _ = nearest_tag_rtk(
                tag_gps, imu_wall_ns, rtk_max_age_ms
            )
            if transform is not None and yaw_gps is not None:
                yaw_gps_wall_ns = gps_wall_ns(yaw_gps)
                speed_mps = finite_float(yaw_gps.get("gps_speed_mps"))
                course_deg = finite_float(yaw_gps.get("gps_course_deg"))
                is_new_fix = yaw_gps_wall_ns > last_yaw_gps_wall_ns
                if is_new_fix:
                    # A rejected RTK fix is still consumed once.  Retrying the
                    # same course at every 100 Hz IMU sample overweights it and
                    # hides the actual yaw rejection reasons.
                    last_yaw_gps_wall_ns = yaw_gps_wall_ns
                if (
                    is_new_fix
                    and speed_mps is not None
                    and speed_mps >= config.yaw_min_speed_mps
                    and course_deg is not None
                ):
                    rtk_yaw_alignment_attempts += 1
                    yaw_output = fusion.align_yaw_from_heading(
                        transform.course_to_uwb_heading(course_deg),
                        source="rtk_course",
                    )
                    fusion_flag_counts.update(
                        str(value) for value in yaw_output.get("flags", ())
                    )
                    if "yaw_aligned" in yaw_output.get("flags", ()):
                        rtk_yaw_alignment_updates += 1
            continue
        xy = raw_position_xy(record)
        if xy is None:
            continue
        position_times.append(float(event["sort_ms"]))
        output = fusion.update_position(
            {
                "uptime_ms": int(record["uptime_ms"]),
                "protocol": key.protocol,
                "tag_id": key.tag_id,
                "module_id": key.module_id,
                "sample_time_us": record.get("sample_time_us"),
                "raw_x_m": xy[0],
                "raw_y_m": xy[1],
                "sigma_m": record.get("sigma_m"),
                "rms_m": record.get("rms_m"),
            }
        )
        fused_x = finite_float(output.get("x"))
        fused_y = finite_float(output.get("y"))
        fusion_flag_counts.update(str(value) for value in output.get("flags", ()))
        record_fused_stream_time(output, event)
        collector_ns = collector_wall_ns(record)
        wall_ns = position_wall_ns(record)
        rtk_row, rtk_age_ms = nearest_tag_rtk(tag_gps, wall_ns, rtk_max_age_ms)
        rtk_xy = None
        if transform is not None and rtk_row is not None:
            rtk_xy = transform.gps_to_uwb(
                float(rtk_row["gps_latitude_deg"]),
                float(rtk_row["gps_longitude_deg"]),
            )
        raw_error = None
        fused_error = None
        rtk_fix_wall_ns = None
        rtk_gga_count = None
        if rtk_xy is not None:
            raw_error = math.hypot(xy[0] - rtk_xy[0], xy[1] - rtk_xy[1])
            if fused_x is not None and fused_y is not None:
                fused_error = math.hypot(fused_x - rtk_xy[0], fused_y - rtk_xy[1])
            rtk_fix_wall_ns = gps_wall_ns(rtk_row)
            rtk_gga_count = int(rtk_row.get("gps_gga_count") or 0)
        samples.append(
            {
                "schema_version": 1,
                "kind": "replay_position",
                "protocol": key.protocol,
                "tag_id": key.tag_id,
                "module_id": key.module_id,
                "uptime_ms": int(record["uptime_ms"]),
                "collector_wall_ns": collector_ns,
                "position_wall_ns": wall_ns,
                "raw_x_m": xy[0],
                "raw_y_m": xy[1],
                "sigma_m": finite_float(record.get("sigma_m")),
                "rms_m": finite_float(record.get("rms_m")),
                "fused_x_m": fused_x,
                "fused_y_m": fused_y,
                "rtk_x_m": rtk_xy[0] if rtk_xy is not None else None,
                "rtk_y_m": rtk_xy[1] if rtk_xy is not None else None,
                "rtk_wall_ns": rtk_fix_wall_ns,
                "rtk_gga_count": rtk_gga_count,
                "rtk_time_delta_ms": rtk_age_ms,
                # Filled after the nearest UWB correction for each unique RTK
                # fix is known.  Other rows retain RTK coordinates for plots,
                # but do not multiply one slow GNSS fix into many samples.
                "rtk_match_selected": False,
                "raw_error_m": raw_error,
                "fused_error_m": fused_error,
                "fusion_flags": output.get("flags") or [],
            }
        )
    rtk_comparisons = interpolate_track_at_rtk_fixes(
        samples, tag_gps, transform, rtk_max_age_ms
    )
    raw_errors = [float(sample["raw_error_m"]) for sample in rtk_comparisons]
    fused_errors = [
        float(sample["fused_error_m"]) for sample in rtk_comparisons
    ]
    raw_metrics = error_metrics(raw_errors)
    fused_metrics = error_metrics(fused_errors)
    delta_rmse_m = None
    if raw_metrics is not None and fused_metrics is not None:
        delta_rmse_m = fused_metrics["rmse_m"] - raw_metrics["rmse_m"]
    raw_timing = timing_metrics(position_times, gap_ms)
    # Dead-zone counts must use the exact same physical threshold.  Let the
    # raw UWB cadence select it once when the caller did not provide one.
    shared_gap_ms = float(raw_timing["gap_threshold_ms"])
    fused_stream_timing = timing_metrics(fused_stream_times, shared_gap_ms)
    report = {
        "protocol": key.protocol,
        "tag_id": key.tag_id,
        "module_id": key.module_id,
        "input_counts": {
            "imu_candidates": sum(event["event_kind"] == "imu" for event in events),
            "imu_used": imu_used,
            "imu_missing_or_invalid_orientation": imu_missing_orientation,
            "positions": len(position_times),
            "rtk_fixed_fixes": len(tag_gps),
            "rtk_matches": len(raw_errors),
            "rtk_yaw_alignment_attempts": rtk_yaw_alignment_attempts,
            "rtk_yaw_alignment_updates": rtk_yaw_alignment_updates,
        },
        "timing": {
            "imu_used": timing_metrics(imu_times, gap_ms),
            "raw_position": raw_timing,
            # Correction-only cadence is retained for backward compatibility.
            "fused_position": timing_metrics(position_times, gap_ms),
            # Actual EKF output includes high-rate IMU predictions between UWB
            # corrections and is the relevant signal for dead-zone coverage.
            "fused_stream": fused_stream_timing,
            "rtk_fixed": timing_metrics(gps_times, gap_ms),
        },
        "metrics": {
            "raw_vs_rtk": raw_metrics,
            "fused_vs_rtk": fused_metrics,
            "fused_minus_raw_rmse_m": delta_rmse_m,
        },
        "rtk_comparisons": rtk_comparisons,
        "acceptance": fusion_acceptance(
            raw_metrics,
            fused_metrics,
            raw_timing,
            fused_stream_timing,
            overshoot_tolerance_m,
        ),
        "fusion_flag_counts": dict(sorted(fusion_flag_counts.items())),
        "fusion_final": fusion.snapshot(),
    }
    return report, samples


def replay_capture(
    records: Sequence[dict[str, Any]],
    *,
    protocol: str | None = None,
    config: FusionConfig | None = None,
    rtk_max_age_ms: float = 250.0,
    gap_ms: float | None = None,
    overshoot_tolerance_m: float = 0.05,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    selected_protocol = normalize_protocol(protocol) if protocol else None
    tracks = sorted(
        {
            key
            for record in records
            if (key := position_track(record)) is not None
            and (selected_protocol is None or key.protocol == selected_protocol)
        }
    )
    reports = []
    all_samples = []
    alignment_reports = {}
    for key in tracks:
        alignment_key = f"{key.protocol}:{key.tag_id}"
        if alignment_key not in alignment_reports:
            transform, alignment = build_rtk_alignment(
                records, key.protocol, key.tag_id
            )
            alignment_reports[alignment_key] = (transform, alignment)
        transform, alignment = alignment_reports[alignment_key]
        report, samples = replay_track(
            records,
            key,
            transform,
            config or FusionConfig(),
            rtk_max_age_ms,
            gap_ms,
            overshoot_tolerance_m,
        )
        report["rtk_alignment"] = alignment
        reports.append(report)
        all_samples.extend(samples)
    raw_errors = [
        float(sample["raw_error_m"])
        for sample in all_samples
        if sample.get("raw_error_m") is not None
    ]
    fused_errors = [
        float(sample["fused_error_m"])
        for sample in all_samples
        if sample.get("fused_error_m") is not None
    ]
    return (
        {
            "schema_version": 1,
            "selected_protocol": selected_protocol,
            "track_count": len(reports),
            "tracks": reports,
            "overall_metrics": {
                "raw_vs_rtk": error_metrics(raw_errors),
                "fused_vs_rtk": error_metrics(fused_errors),
            },
            "fusion_acceptance": {
                "accepted": bool(reports)
                and all(track["acceptance"]["accepted"] for track in reports),
                "accepted_tracks": sum(
                    bool(track["acceptance"]["accepted"]) for track in reports
                ),
                "total_tracks": len(reports),
            },
            "limitations": [
                "RTK association compares dashboard telemetry receive time with GPS measurement time estimated from the reported fix age.",
                "RTK-to-UWB alignment uses anchor geometry, never the tag trajectory.",
                "Only RTK-fixed quality 4 fixes enter accuracy metrics.",
                "RMSE/P95/P99 compare raw and fused states at identical UWB correction timestamps; fused_stream timing separately measures IMU dead-zone coverage.",
                "RTK course initializes IMU-to-UWB yaw while moving; UWB straight-line motion can refine it and assumes body +X follows travel.",
            ],
        },
        all_samples,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay dynamic capture JSONL through UwbImuFusion."
    )
    parser.add_argument("inputs", nargs="+", type=pathlib.Path)
    parser.add_argument("--protocol", choices=sorted(set(PROTOCOL_ALIASES.values())))
    parser.add_argument(
        "--module-id",
        type=int,
        help="Replay one tag module and stream-filter other 500 Hz IMU records.",
    )
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--samples-output", type=pathlib.Path)
    parser.add_argument("--rtk-max-age-ms", type=float, default=250.0)
    parser.add_argument("--gap-ms", type=float)
    parser.add_argument("--overshoot-tolerance-m", type=float, default=0.05)
    parser.add_argument("--alpha", type=float, default=FusionConfig.alpha)
    parser.add_argument("--beta", type=float, default=FusionConfig.beta)
    parser.add_argument(
        "--process-accel-noise",
        type=float,
        default=FusionConfig.process_accel_noise_mps2,
    )
    parser.add_argument(
        "--uwb-std-scale",
        type=float,
        default=FusionConfig.uwb_measurement_std_scale,
    )
    parser.add_argument(
        "--yaw-min-displacement-m",
        type=float,
        default=FusionConfig.yaw_min_displacement_m,
    )
    parser.add_argument(
        "--position-smoothing-blend",
        type=float,
        default=FusionConfig.position_smoothing_blend,
    )
    parser.add_argument(
        "--high-confidence-sigma-m",
        type=float,
        default=FusionConfig.uwb_high_confidence_sigma_m,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = [path.expanduser().resolve() for path in args.inputs]
    missing = [path for path in paths if not path.is_file()]
    if missing:
        print(f"missing input: {missing[0]}", file=sys.stderr)
        return 2
    if (
        args.rtk_max_age_ms <= 0
        or (args.gap_ms is not None and args.gap_ms <= 0)
        or args.overshoot_tolerance_m < 0
        or (args.module_id is not None and args.module_id <= 0)
        or ((args.protocol is None) != (args.module_id is None))
    ):
        print(
            "RTK age/gap/module thresholds are invalid; --protocol and "
            "--module-id must be used together",
            file=sys.stderr,
        )
        return 2
    try:
        records = load_jsonl(
            paths, protocol=args.protocol, module_id=args.module_id
        )
        config = FusionConfig(
            alpha=args.alpha,
            beta=args.beta,
            process_accel_noise_mps2=args.process_accel_noise,
            uwb_measurement_std_scale=args.uwb_std_scale,
            yaw_min_displacement_m=args.yaw_min_displacement_m,
            position_smoothing_blend=args.position_smoothing_blend,
            uwb_high_confidence_sigma_m=args.high_confidence_sigma_m,
        )
        report, samples = replay_capture(
            records,
            protocol=args.protocol,
            config=config,
            rtk_max_age_ms=args.rtk_max_age_ms,
            gap_ms=args.gap_ms,
            overshoot_tolerance_m=args.overshoot_tolerance_m,
        )
    except (OSError, ValueError) as exc:
        print(f"replay failed: {exc}", file=sys.stderr)
        return 1
    report["input_files"] = [str(path) for path in paths]
    report["fusion_config"] = asdict(config)
    output = (
        args.output.expanduser().resolve()
        if args.output
        else paths[0].with_suffix(".imu_replay.json")
    )
    samples_output = (
        args.samples_output.expanduser().resolve()
        if args.samples_output
        else paths[0].with_suffix(".imu_replay.jsonl")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    samples_output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with samples_output.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(json.dumps(sample, separators=(",", ":"), sort_keys=True) + "\n")
    print(
        f"replayed {len(records)} records, tracks={report['track_count']}, "
        f"samples={len(samples)} -> {output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
