#!/usr/bin/env python3
"""Compare raw embedded Native DS-TWR positions with time-aligned RTK fixes."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys
from typing import Any, Iterable


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = min(len(ordered) - 1, lower + 1)
    blend = index - lower
    return ordered[lower] * (1.0 - blend) + ordered[upper] * blend


def ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    normal = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    return (
        (normal + altitude_m) * cos_latitude * math.cos(longitude),
        (normal + altitude_m) * cos_latitude * math.sin(longitude),
        (normal * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


def ecef_to_enu(
    point: tuple[float, float, float],
    origin: tuple[float, float, float],
    latitude_deg: float,
    longitude_deg: float,
) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    dz = point[2] - origin[2]
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz
    )
    up = (
        math.cos(latitude) * math.cos(longitude) * dx
        + math.cos(latitude) * math.sin(longitude) * dy
        + math.sin(latitude) * dz
    )
    return east, north, up


def fit_rigid_2d(
    source: list[tuple[float, float]],
    target: list[tuple[float, float]],
) -> tuple[float, float, float]:
    """Return the rotation and translation mapping source points to target points."""
    if len(source) != len(target) or len(source) < 2:
        raise ValueError("rigid 2D alignment requires matching point sets of size >= 2")
    source_x = statistics.fmean(point[0] for point in source)
    source_y = statistics.fmean(point[1] for point in source)
    target_x = statistics.fmean(point[0] for point in target)
    target_y = statistics.fmean(point[1] for point in target)
    dot = cross = source_spread = 0.0
    for measured, fixed in zip(source, target):
        measured_x = measured[0] - source_x
        measured_y = measured[1] - source_y
        fixed_x = fixed[0] - target_x
        fixed_y = fixed[1] - target_y
        dot += measured_x * fixed_x + measured_y * fixed_y
        cross += measured_x * fixed_y - measured_y * fixed_x
        source_spread += measured_x * measured_x + measured_y * measured_y
    if source_spread < 1.0e-12:
        raise ValueError("rigid 2D alignment requires distinct source points")
    rotation_rad = math.atan2(cross, dot)
    cosine = math.cos(rotation_rad)
    sine = math.sin(rotation_rad)
    translation_x_m = target_x - (cosine * source_x - sine * source_y)
    translation_y_m = target_y - (sine * source_x + cosine * source_y)
    return rotation_rad, translation_x_m, translation_y_m


def transform_2d(
    point: tuple[float, float],
    rotation_rad: float,
    translation_x_m: float,
    translation_y_m: float,
) -> tuple[float, float]:
    cosine = math.cos(rotation_rad)
    sine = math.sin(rotation_rad)
    return (
        cosine * point[0] - sine * point[1] + translation_x_m,
        sine * point[0] + cosine * point[1] + translation_y_m,
    )


def load_events(path: pathlib.Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                events.append(json.loads(line))
    return events


def valid_rtk_fix(event: dict[str, Any]) -> bool:
    return (
        event.get("kind") == "gps_fix"
        and event.get("gps_fix_valid") is True
        and int(event.get("gps_fix_quality") or 0) == 4
        and all(
            math.isfinite(float(event.get(key)))
            for key in ("gps_latitude_deg", "gps_longitude_deg", "gps_altitude_m")
        )
    )


def median_coordinate(events: list[dict[str, Any]]) -> tuple[float, float, float]:
    return tuple(
        statistics.median(float(event[key]) for event in events)
        for key in ("gps_latitude_deg", "gps_longitude_deg", "gps_altitude_m")
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze raw Native DS-TWR positions against RTK Fixed."
    )
    parser.add_argument("--events", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--origin-module", type=int, default=2)
    parser.add_argument("--tag-module", type=int, default=1)
    parser.add_argument("--max-alignment-age-sec", type=float, default=2.0)
    parser.add_argument(
        "--anchor",
        action="append",
        default=[],
        metavar="ID:X_M:Y_M",
        help=(
            "Override the runtime firmware anchor coordinate; repeat for every "
            "anchor. By default the geometry is read from captured status events."
        ),
    )
    parser.add_argument(
        "--range-bias-cm",
        action="append",
        default=[],
        metavar="ID:BIAS_CM",
        help="Measured-minus-RTK range bias to subtract before offline solving.",
    )
    return parser.parse_args()


def parse_anchors(entries: list[str]) -> dict[int, tuple[float, float]]:
    anchors: dict[int, tuple[float, float]] = {}
    for entry in entries:
        anchor_id_text, x_text, y_text = entry.split(":", 2)
        anchors[int(anchor_id_text)] = (float(x_text), float(y_text))
    return anchors


def runtime_anchors(events: list[dict[str, Any]]) -> dict[int, tuple[float, float]]:
    """Return the first complete runtime anchor geometry captured from a module."""
    for event in events:
        modules = event.get("modules")
        candidates = modules if isinstance(modules, list) else [event]
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            anchor_ids = candidate.get("runtime_anchor_ids")
            anchor_x_mm = candidate.get("runtime_flex_tdoa_anchor_x_mm")
            anchor_y_mm = candidate.get("runtime_flex_tdoa_anchor_y_mm")
            if not isinstance(anchor_ids, list):
                continue
            if not isinstance(anchor_x_mm, list) or not isinstance(anchor_y_mm, list):
                continue
            if len(anchor_ids) < 3 or not (
                len(anchor_ids) == len(anchor_x_mm) == len(anchor_y_mm)
            ):
                continue
            return {
                int(anchor_id): (float(x_mm) / 1000.0, float(y_mm) / 1000.0)
                for anchor_id, x_mm, y_mm in zip(anchor_ids, anchor_x_mm, anchor_y_mm)
            }
    return {}


def geometry_difference_cm(
    first: dict[int, tuple[float, float]],
    second: dict[int, tuple[float, float]],
) -> dict[str, float | int] | None:
    common = sorted(set(first) & set(second))
    if not common:
        return None
    errors_m = [
        math.hypot(first[anchor_id][0] - second[anchor_id][0],
                   first[anchor_id][1] - second[anchor_id][1])
        for anchor_id in common
    ]
    return {
        "anchors_compared": len(common),
        "rms_cm": math.sqrt(statistics.fmean(value * value for value in errors_m)) * 100.0,
        "max_cm": max(errors_m) * 100.0,
    }


def parse_range_biases(entries: list[str]) -> dict[int, float]:
    biases: dict[int, float] = {}
    for entry in entries:
        anchor_id_text, bias_cm_text = entry.split(":", 1)
        biases[int(anchor_id_text)] = float(bias_cm_text) / 100.0
    return biases


def solve_position(
    anchors: dict[int, tuple[float, float]],
    distances: dict[int, float],
) -> tuple[float, float] | None:
    anchor_ids = sorted(anchors)
    if len(anchor_ids) < 3 or set(anchor_ids) != set(distances):
        return None
    x0, y0 = anchors[anchor_ids[0]]
    r0 = distances[anchor_ids[0]]
    h00 = h01 = h11 = g0 = g1 = 0.0
    for anchor_id in anchor_ids[1:]:
        xi, yi = anchors[anchor_id]
        ri = distances[anchor_id]
        ax = 2.0 * (xi - x0)
        ay = 2.0 * (yi - y0)
        right = xi * xi + yi * yi - x0 * x0 - y0 * y0 - ri * ri + r0 * r0
        h00 += ax * ax
        h01 += ax * ay
        h11 += ay * ay
        g0 += ax * right
        g1 += ay * right
    determinant = h00 * h11 - h01 * h01
    if abs(determinant) < 1.0e-8:
        return None
    x_m = (h11 * g0 - h01 * g1) / determinant
    y_m = (h00 * g1 - h01 * g0) / determinant
    for _ in range(8):
        h00 = h11 = 1.0e-6
        h01 = g0 = g1 = 0.0
        for anchor_id in anchor_ids:
            anchor_x, anchor_y = anchors[anchor_id]
            dx = x_m - anchor_x
            dy = y_m - anchor_y
            predicted = math.hypot(dx, dy)
            if predicted < 1.0e-5:
                return None
            residual = distances[anchor_id] - predicted
            jx = dx / predicted
            jy = dy / predicted
            h00 += jx * jx
            h01 += jx * jy
            h11 += jy * jy
            g0 += jx * residual
            g1 += jy * residual
        determinant = h00 * h11 - h01 * h01
        if abs(determinant) < 1.0e-8:
            return None
        step_x = (h11 * g0 - h01 * g1) / determinant
        step_y = (h00 * g1 - h01 * g0) / determinant
        x_m += step_x
        y_m += step_y
        if math.hypot(step_x, step_y) < 0.0001:
            break
    return x_m, y_m


def error_metrics(rows: list[dict[str, float]]) -> dict[str, float | int]:
    mean_dx = statistics.fmean(row["dx_m"] for row in rows)
    mean_dy = statistics.fmean(row["dy_m"] for row in rows)
    errors = [row["error_m"] for row in rows]
    centered = [
        math.hypot(row["dx_m"] - mean_dx, row["dy_m"] - mean_dy)
        for row in rows
    ]
    span_sec = max(0.0, rows[-1]["received_at"] - rows[0]["received_at"])
    return {
        "positions": len(rows),
        "span_sec": span_sec,
        "position_rate_hz": len(rows) / span_sec if span_sec > 0 else math.nan,
        "mean_dx_cm": mean_dx * 100.0,
        "mean_dy_cm": mean_dy * 100.0,
        "bias_2d_cm": math.hypot(mean_dx, mean_dy) * 100.0,
        "rmse_2d_cm": math.sqrt(statistics.fmean(value * value for value in errors)) * 100.0,
        "p50_2d_cm": percentile(errors, 0.50) * 100.0,
        "p95_2d_cm": percentile(errors, 0.95) * 100.0,
        "max_2d_cm": max(errors) * 100.0,
        "precision_rms_cm": math.sqrt(statistics.fmean(value * value for value in centered)) * 100.0,
        "precision_p95_cm": percentile(centered, 0.95) * 100.0,
    }


def main() -> int:
    args = parse_args()
    events_path = pathlib.Path(args.events).resolve()
    events = load_events(events_path)
    rtk_by_module: dict[int, list[dict[str, Any]]] = {}
    for event in events:
        if valid_rtk_fix(event):
            rtk_by_module.setdefault(int(event["module_id"]), []).append(event)

    origin_fixes = rtk_by_module.get(args.origin_module, [])
    tag_fixes = rtk_by_module.get(args.tag_module, [])
    if not origin_fixes:
        raise SystemExit(f"no RTK Fixed samples for origin M{args.origin_module}")
    if not tag_fixes:
        raise SystemExit(f"no RTK Fixed samples for tag M{args.tag_module}")

    origin_latitude, origin_longitude, origin_altitude = median_coordinate(origin_fixes)
    origin_ecef = ecef(origin_latitude, origin_longitude, origin_altitude)

    def event_enu(event: dict[str, Any]) -> tuple[float, float, float]:
        return ecef_to_enu(
            ecef(
                float(event["gps_latitude_deg"]),
                float(event["gps_longitude_deg"]),
                float(event["gps_altitude_m"]),
            ),
            origin_ecef,
            origin_latitude,
            origin_longitude,
        )

    tag_track = sorted(
        (
            float(event["received_at"]),
            *event_enu(event),
        )
        for event in tag_fixes
    )
    positions = sorted(
        (
            event
            for event in events
            if event.get("kind") == "local_position"
            and int(event.get("module_id") or 0) == args.tag_module
            and event.get("position_filter", "none") == "none"
            and event.get("independent_frame", True) is True
        ),
        key=lambda event: float(event["received_at"]),
    )
    aligned: list[dict[str, float]] = []
    for position in positions:
        received_at = float(position["received_at"])
        nearest = min(tag_track, key=lambda item: abs(item[0] - received_at))
        age = abs(nearest[0] - received_at)
        if age > args.max_alignment_age_sec:
            continue
        dx = float(position["x_m"]) - nearest[1]
        dy = float(position["y_m"]) - nearest[2]
        aligned.append(
            {
                "received_at": received_at,
                "uwb_x_m": float(position["x_m"]),
                "uwb_y_m": float(position["y_m"]),
                "rtk_x_m": nearest[1],
                "rtk_y_m": nearest[2],
                "alignment_age_sec": age,
                "dx_m": dx,
                "dy_m": dy,
                "error_m": math.hypot(dx, dy),
                "solver_rms_m": float(position.get("rms_m") or 0.0),
            }
        )
    if not aligned:
        raise SystemExit("no time-aligned raw Native DS-TWR positions")

    embedded_metrics = error_metrics(aligned)

    captured_anchors = runtime_anchors(events)
    explicit_anchors = parse_anchors(args.anchor)
    anchors = explicit_anchors or captured_anchors
    anchor_geometry_source = "command_line" if explicit_anchors else "runtime_status"
    geometry_override_difference = (
        geometry_difference_cm(explicit_anchors, captured_anchors)
        if explicit_anchors and captured_anchors else None
    )
    if (
        geometry_override_difference is not None
        and geometry_override_difference["max_cm"] > 5.0
    ):
        print(
            "warning: command-line anchor geometry differs from captured runtime "
            f"geometry by up to {geometry_override_difference['max_cm']:.1f} cm",
            file=sys.stderr,
        )
    anchor_fit: dict[str, Any] = {}
    anchor_errors: list[float] = []
    anchor_pairs: list[tuple[int, tuple[float, float], tuple[float, float]]] = []
    for anchor_id, fixed in sorted(anchors.items()):
        fixes = rtk_by_module.get(anchor_id, [])
        if not fixes:
            continue
        median_latitude, median_longitude, median_altitude = median_coordinate(fixes)
        measured = ecef_to_enu(
            ecef(median_latitude, median_longitude, median_altitude),
            origin_ecef,
            origin_latitude,
            origin_longitude,
        )
        dx = fixed[0] - measured[0]
        dy = fixed[1] - measured[1]
        error = math.hypot(dx, dy)
        anchor_errors.append(error)
        anchor_pairs.append((anchor_id, (measured[0], measured[1]), fixed))
        anchor_fit[str(anchor_id)] = {
            "fixed_x_m": fixed[0],
            "fixed_y_m": fixed[1],
            "rtk_x_m": measured[0],
            "rtk_y_m": measured[1],
            "dx_cm": dx * 100.0,
            "dy_cm": dy * 100.0,
            "error_cm": error * 100.0,
            "rtk_samples": len(fixes),
        }

    rigid_alignment = None
    if len(anchor_pairs) >= 2:
        rotation_rad, translation_x_m, translation_y_m = fit_rigid_2d(
            [pair[1] for pair in anchor_pairs],
            [pair[2] for pair in anchor_pairs],
        )
        rigid_anchor_fit: dict[str, Any] = {}
        rigid_anchor_errors: list[float] = []
        for anchor_id, measured, fixed in anchor_pairs:
            aligned_anchor = transform_2d(
                measured,
                rotation_rad,
                translation_x_m,
                translation_y_m,
            )
            dx = fixed[0] - aligned_anchor[0]
            dy = fixed[1] - aligned_anchor[1]
            error = math.hypot(dx, dy)
            rigid_anchor_errors.append(error)
            rigid_anchor_fit[str(anchor_id)] = {
                "aligned_rtk_x_m": aligned_anchor[0],
                "aligned_rtk_y_m": aligned_anchor[1],
                "dx_cm": dx * 100.0,
                "dy_cm": dy * 100.0,
                "error_cm": error * 100.0,
            }
        rigid_rows: list[dict[str, float]] = []
        for row in aligned:
            aligned_rtk = transform_2d(
                (row["rtk_x_m"], row["rtk_y_m"]),
                rotation_rad,
                translation_x_m,
                translation_y_m,
            )
            dx = row["uwb_x_m"] - aligned_rtk[0]
            dy = row["uwb_y_m"] - aligned_rtk[1]
            rigid_rows.append({
                **row,
                "rtk_x_m": aligned_rtk[0],
                "rtk_y_m": aligned_rtk[1],
                "dx_m": dx,
                "dy_m": dy,
                "error_m": math.hypot(dx, dy),
            })
        rigid_alignment = {
            "method": "least-squares rigid 2D, RTK anchors to fixed UWB anchors",
            "rotation_deg": math.degrees(rotation_rad),
            "translation_x_cm": translation_x_m * 100.0,
            "translation_y_cm": translation_y_m * 100.0,
            "anchor_fit_rms_cm": (
                math.sqrt(statistics.fmean(value * value for value in rigid_anchor_errors))
                * 100.0
            ),
            "anchors": rigid_anchor_fit,
            "tag_position_error": error_metrics(rigid_rows),
        }

    ranges_by_anchor: dict[int, list[float]] = {}
    for event in events:
        if event.get("kind") != "ds_range":
            continue
        anchor_id = int(event.get("anchor_id") or 0)
        if anchor_id not in anchors:
            continue
        received_at = float(event["received_at"])
        nearest = min(tag_track, key=lambda item: abs(item[0] - received_at))
        if abs(nearest[0] - received_at) > args.max_alignment_age_sec:
            continue
        expected = math.hypot(
            nearest[1] - anchors[anchor_id][0],
            nearest[2] - anchors[anchor_id][1],
        )
        ranges_by_anchor.setdefault(anchor_id, []).append(
            float(event["distance_m"]) - expected
        )
    range_bias: dict[str, Any] = {}
    for anchor_id, errors_for_anchor in sorted(ranges_by_anchor.items()):
        mean_error = statistics.fmean(errors_for_anchor)
        range_bias[str(anchor_id)] = {
            "samples": len(errors_for_anchor),
            "mean_bias_cm": mean_error * 100.0,
            "std_cm": (
                statistics.stdev(errors_for_anchor) * 100.0
                if len(errors_for_anchor) > 1 else 0.0
            ),
            "rmse_cm": math.sqrt(statistics.fmean(
                value * value for value in errors_for_anchor
            )) * 100.0,
            "p95_abs_cm": percentile(
                (abs(value) for value in errors_for_anchor), 0.95
            ) * 100.0,
        }

    range_biases = parse_range_biases(args.range_bias_cm)
    offline_metrics = None
    if anchors and range_biases:
        frames: dict[int, dict[int, dict[str, Any]]] = {}
        for event in events:
            if event.get("kind") != "ds_range":
                continue
            anchor_id = int(event.get("anchor_id") or 0)
            if anchor_id not in anchors:
                continue
            frame_id = int(event.get("frame_id") or event.get("seq") or 0)
            frames.setdefault(frame_id, {})[anchor_id] = event
        offline_aligned: list[dict[str, float]] = []
        for items in frames.values():
            if set(items) != set(anchors):
                continue
            distances = {
                anchor_id: float(items[anchor_id]["distance_m"]) - range_biases.get(anchor_id, 0.0)
                for anchor_id in anchors
            }
            solution = solve_position(anchors, distances)
            if solution is None:
                continue
            received_at = max(float(item["received_at"]) for item in items.values())
            nearest = min(tag_track, key=lambda item: abs(item[0] - received_at))
            if abs(nearest[0] - received_at) > args.max_alignment_age_sec:
                continue
            dx = solution[0] - nearest[1]
            dy = solution[1] - nearest[2]
            offline_aligned.append({
                "received_at": received_at,
                "dx_m": dx,
                "dy_m": dy,
                "error_m": math.hypot(dx, dy),
            })
        offline_aligned.sort(key=lambda row: row["received_at"])
        if offline_aligned:
            offline_metrics = {
                "calibration_source": "command-line measured-minus-RTK biases",
                "range_bias_cm": {
                    str(anchor_id): value * 100.0
                    for anchor_id, value in sorted(range_biases.items())
                },
                **error_metrics(offline_aligned),
            }

    summary = {
        "schema_version": 3,
        "events_file": str(events_path),
        "coordinate_frame": f"WGS84 ENU, median RTK Fixed M{args.origin_module} origin",
        "position_filter": "none",
        "solution_kind": "independent_frame",
        "positions_captured": len(positions),
        "positions_aligned": len(aligned),
        "rtk_tag_samples": len(tag_fixes),
        **embedded_metrics,
        "alignment_age_p95_ms": percentile(
            (row["alignment_age_sec"] for row in aligned), 0.95
        ) * 1000.0,
        "solver_residual_rms_mean_cm": statistics.fmean(
            row["solver_rms_m"] for row in aligned
        ) * 100.0,
        "anchor_geometry_source": anchor_geometry_source,
        "anchor_geometry_override_difference": geometry_override_difference,
        "fixed_anchor_geometry": anchor_fit,
        "anchor_geometry_fit_rms_cm": (
            math.sqrt(statistics.fmean(value * value for value in anchor_errors)) * 100.0
            if anchor_errors else None
        ),
        "rigid_anchor_alignment": rigid_alignment,
        "range_error_vs_rtk": range_bias,
        "offline_range_corrected_position": offline_metrics,
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        output_path = pathlib.Path(args.output).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
