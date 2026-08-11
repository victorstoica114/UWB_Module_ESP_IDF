#!/usr/bin/env python3
"""Build an auditable raw-UWB localization baseline against RTK.

The evaluator intentionally ignores every temporally filtered or IMU-derived
position.  It accepts both plain JSONL and lossless ``.jsonl.xz`` captures,
registers UWB anchor geometry to RTK with a rigid 2-D transform, and keeps
absolute error, local precision, solver residuals, continuity, geometry, and
per-link range evidence separate.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import pathlib
import statistics
from collections import defaultdict
from typing import Any, Iterable, Sequence

from uwb_dynamic_static_report import (
    PROTOCOL_LABELS,
    Capture,
    add_enu,
    alignment_for_position,
    alignment_metrics,
    analysis_positions,
    build_alignments,
    error_metrics as position_error_metrics,
    finite,
    gps_origin,
    json_clean,
    jump_metrics,
    load_capture,
    mean,
    median,
    nearest_position_pairs,
    percentile,
    precision_metrics,
    rtk_metrics,
    stream_metrics,
    transform_positions,
)


ROLES = {"calibration", "validation"}


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_values(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values if finite(value)]


def distribution(values: Iterable[Any]) -> dict[str, Any]:
    clean = finite_values(values)
    if not clean:
        return {"available": False, "samples": 0}
    return {
        "available": True,
        "samples": len(clean),
        "mean": mean(clean),
        "median": median(clean),
        "rmse": math.sqrt(mean(value * value for value in clean)),
        "p95": percentile(clean, 95),
        "p99": percentile(clean, 99),
        "max": max(clean),
    }


def raw_position_residual_metrics(points: Sequence[dict[str, Any]]) -> dict[str, Any]:
    rms = distribution(point.get("rms_m") for point in points)
    sigma = distribution(point.get("sigma_m") for point in points)
    return {
        "reported_solver_rms_m": rms,
        "reported_solver_sigma_m": sigma,
        "warning": (
            "reported solver residual is an equation residual, not position error"
        ),
    }


def invert_geometry_2x2(
    h00: float, h01: float, h11: float
) -> tuple[float, float] | None:
    determinant = h00 * h11 - h01 * h01
    if determinant <= 1e-12:
        return None
    trace_inverse = (h00 + h11) / determinant
    trace = h00 + h11
    discriminant = max(0.0, trace * trace - 4.0 * determinant)
    eigen_max = 0.5 * (trace + math.sqrt(discriminant))
    eigen_min = 0.5 * (trace - math.sqrt(discriminant))
    if eigen_min <= 1e-12:
        return None
    return math.sqrt(trace_inverse), math.sqrt(eigen_max / eigen_min)


def geometry_proxy_at_position(
    protocol: str,
    point: dict[str, Any],
    anchors: dict[int, tuple[float, float]],
) -> tuple[float, float] | None:
    gradients: list[tuple[float, float]] = []
    for anchor_id in sorted(anchors):
        anchor_x, anchor_y = anchors[anchor_id]
        dx = float(point["x_m"]) - anchor_x
        dy = float(point["y_m"]) - anchor_y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return None
        gradients.append((dx / distance, dy / distance))
    rows: list[tuple[float, float]] = []
    if protocol == "native_ds":
        rows = gradients
    else:
        for first in range(len(gradients)):
            for second in range(first + 1, len(gradients)):
                rows.append(
                    (
                        gradients[first][0] - gradients[second][0],
                        gradients[first][1] - gradients[second][1],
                    )
                )
    h00 = sum(row[0] * row[0] for row in rows)
    h01 = sum(row[0] * row[1] for row in rows)
    h11 = sum(row[1] * row[1] for row in rows)
    return invert_geometry_2x2(h00, h01, h11)


def geometry_proxy_metrics(
    capture: Capture,
    points: Sequence[dict[str, Any]],
    alignments: Sequence[dict[str, Any]],
    max_dynamic_age_s: float,
) -> dict[str, Any]:
    hdop: list[float] = []
    condition: list[float] = []
    missing = 0
    for point in points:
        alignment, _, _ = alignment_for_position(
            point, list(alignments), max_dynamic_age_s
        )
        if alignment is None:
            missing += 1
            continue
        proxy = geometry_proxy_at_position(
            capture.protocol, point, alignment["anchors"]
        )
        if proxy is None:
            missing += 1
            continue
        hdop.append(proxy[0])
        condition.append(proxy[1])
    return {
        "available": bool(hdop),
        "model": (
            "direct-range unit Jacobian"
            if capture.protocol == "native_ds"
            else "all-pairs range-difference Jacobian proxy"
        ),
        "samples": len(hdop),
        "missing": missing,
        "hdop_median": median(hdop),
        "hdop_p95": percentile(hdop, 95),
        "hdop_max": max(hdop) if hdop else math.nan,
        "condition_median": median(condition),
        "condition_p95": percentile(condition, 95),
        "condition_max": max(condition) if condition else math.nan,
        "warning": (
            "geometry-only proxy; it does not include packet loss, link noise, "
            "or the actual observation covariance"
        ),
    }


def nearest_fixed_gps(
    rows: Sequence[dict[str, Any]], wall_time: float, maximum_age_s: float
) -> dict[str, Any] | None:
    if not rows:
        return None
    times = [float(row["time"]) for row in rows]
    index = bisect.bisect_left(times, wall_time)
    candidates = [
        rows[candidate]
        for candidate in (index - 1, index)
        if 0 <= candidate < len(rows)
    ]
    if not candidates:
        return None
    selected = min(candidates, key=lambda row: abs(float(row["time"]) - wall_time))
    if abs(float(selected["time"]) - wall_time) > maximum_age_s:
        return None
    return selected


def range_error_rows(
    capture: Capture, maximum_age_ms: float
) -> list[dict[str, Any]]:
    gps_by_module: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for fix in capture.gps:
        if (
            fix.get("valid")
            and int(fix.get("quality", 0)) == 4
            and all(
                finite(fix.get(key))
                for key in ("time", "east_m", "north_m", "up_m")
            )
        ):
            gps_by_module[int(fix["module_id"])].append(fix)
    for rows in gps_by_module.values():
        rows.sort(key=lambda row: float(row["time"]))
    result: list[dict[str, Any]] = []
    maximum_age_s = maximum_age_ms / 1000.0
    for item in capture.ranges:
        wall_time = float(item["time"])
        first = nearest_fixed_gps(
            gps_by_module.get(int(item["first_id"]), []), wall_time, maximum_age_s
        )
        second = nearest_fixed_gps(
            gps_by_module.get(int(item["second_id"]), []), wall_time, maximum_age_s
        )
        if first is None or second is None:
            continue
        horizontal_m = math.hypot(
            float(first["east_m"]) - float(second["east_m"]),
            float(first["north_m"]) - float(second["north_m"]),
        )
        vertical_m = float(first["up_m"]) - float(second["up_m"])
        slant_m = math.hypot(horizontal_m, vertical_m)
        measured_m = float(item["distance_m"])
        result.append(
            {
                "capture_id": capture.capture_id,
                "protocol": capture.protocol,
                "kind": item["kind"],
                "first_id": int(item["first_id"]),
                "second_id": int(item["second_id"]),
                "link": f"M{min(int(item['first_id']), int(item['second_id']))}-M{max(int(item['first_id']), int(item['second_id']))}",
                "time": wall_time,
                "measured_m": measured_m,
                "rtk_horizontal_m": horizontal_m,
                "rtk_vertical_m": vertical_m,
                "rtk_slant_m": slant_m,
                "height_correction_m": slant_m - horizontal_m,
                "error_m": measured_m - slant_m,
                "first_rtk_age_ms": abs(float(first["time"]) - wall_time) * 1000.0,
                "second_rtk_age_ms": abs(float(second["time"]) - wall_time) * 1000.0,
            }
        )
    return result


def summarize_range_errors(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["link"])].append(row)
    links: dict[str, Any] = {}
    for link, values in sorted(grouped.items()):
        errors = [float(value["error_m"]) for value in values]
        absolute = [abs(value) for value in errors]
        links[link] = {
            "samples": len(errors),
            "mean_bias_m": mean(errors),
            "median_bias_m": median(errors),
            "std_m": statistics.stdev(errors) if len(errors) > 1 else 0.0,
            "rmse_m": math.sqrt(mean(value * value for value in errors)),
            "p95_abs_m": percentile(absolute, 95),
            "max_abs_m": max(absolute),
            "height_correction_median_m": median(
                value["height_correction_m"] for value in values
            ),
            "rtk_age_p95_ms": percentile(
                (
                    max(value["first_rtk_age_ms"], value["second_rtk_age_ms"])
                    for value in values
                ),
                95,
            ),
        }
    return {
        "available": bool(rows),
        "matched_samples": len(rows),
        "links": links,
        "warning": (
            "a link bias is not automatically a universal antenna-delay correction; "
            "it may include propagation, orientation, and timestamp errors"
        ),
    }


def flatten(prefix: str, value: Any, output: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            flatten(f"{prefix}.{key}" if prefix else str(key), item, output)
    elif isinstance(value, (str, int, float, bool)) or value is None:
        output[prefix] = value


def write_csv(path: pathlib.Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_capture(value: str) -> tuple[str, pathlib.Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("capture must be ROLE=PATH")
    role, raw_path = value.split("=", 1)
    role = role.strip().lower()
    if role not in ROLES:
        raise argparse.ArgumentTypeError(
            f"capture role must be one of {', '.join(sorted(ROLES))}"
        )
    path = pathlib.Path(raw_path).expanduser().resolve()
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"capture does not exist: {path}")
    return role, path


def aggregate_position_metrics(
    summaries: Sequence[dict[str, Any]], role: str, protocol: str
) -> dict[str, Any]:
    pairs = [
        pair
        for summary in summaries
        if summary["role"] == role and summary["protocol"] == protocol
        for pair in summary["_position_pairs"]
    ]
    fixed = sum(
        int(summary["position_vs_rtk"].get("fixed_tag_solutions", 0))
        for summary in summaries
        if summary["role"] == role and summary["protocol"] == protocol
    )
    return position_error_metrics(pairs, fixed)


def build_readme(summary: dict[str, Any]) -> str:
    rows = []
    for role in sorted(ROLES):
        for protocol in PROTOCOL_LABELS:
            metrics = summary["aggregates"].get(role, {}).get(protocol)
            if not metrics:
                continue
            rows.append(
                "| {role} | {protocol} | {pairs} | {rmse} | {p95} | {maximum} |".format(
                    role=role,
                    protocol=PROTOCOL_LABELS[protocol],
                    pairs=metrics.get("pairs", 0),
                    rmse=(
                        f"{100.0 * metrics['error_rmse_m']:.2f}"
                        if finite(metrics.get("error_rmse_m"))
                        else "n/a"
                    ),
                    p95=(
                        f"{100.0 * metrics['error_p95_m']:.2f}"
                        if finite(metrics.get("error_p95_m"))
                        else "n/a"
                    ),
                    maximum=(
                        f"{100.0 * metrics['error_max_m']:.2f}"
                        if finite(metrics.get("error_max_m"))
                        else "n/a"
                    ),
                )
            )
    evidence = summary["evidence_gate"]
    return "\n".join(
        [
            "# Raw UWB localization baseline",
            "",
            "This report scores only raw UWB positions. IMU-derived and temporally filtered positions are excluded.",
            "",
            "| Split | Protocol | RTK pairs | RMSE cm | P95 cm | Max cm |",
            "|---|---|---:|---:|---:|---:|",
            *rows,
            "",
            f"Validation matrix complete: **{evidence['validation_protocol_matrix_complete']}**.",
            f"All validation captures have RTK accuracy evidence: **{evidence['validation_accuracy_complete']}**.",
            "",
            "Absolute RTK error is reported separately from local cloud precision and debiased error. Geometry registration uses anchor coordinates only; the tag trajectory is never used to fit the transform.",
            "",
            "`range_rtk_pairs.csv` and the per-capture range sections in `baseline_summary.json` may be incomplete when a capture did not record individual ranges. Missing evidence is reported, not inferred from positions.",
            "",
        ]
    )


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    assignments = list(args.capture)
    captures = [load_capture(path, args.tag_id) for _, path in assignments]
    add_enu(captures, gps_origin(captures))
    summaries: list[dict[str, Any]] = []
    all_range_rows: list[dict[str, Any]] = []
    all_position_pairs: list[dict[str, Any]] = []
    for (role, path), capture in zip(assignments, captures):
        points = analysis_positions(capture)
        rtk, centers, spreads = rtk_metrics(capture, args.tag_id)
        alignments = build_alignments(capture, centers)
        transformed, transform_diagnostics = transform_positions(
            points, alignments, args.max_dynamic_geometry_age_s
        )
        pairs, pair_diagnostics = nearest_position_pairs(
            capture, transformed, args.tag_id, args.rtk_max_age_ms
        )
        accuracy = position_error_metrics(
            pairs, int(pair_diagnostics["fixed_tag_solutions"])
        )
        ranges = range_error_rows(capture, args.range_rtk_max_age_ms)
        all_range_rows.extend({"role": role, **row} for row in ranges)
        all_position_pairs.extend({"role": role, **row} for row in pairs)
        alignment = alignment_metrics(alignments, spreads, transform_diagnostics)
        capture_summary = {
            "role": role,
            "path": str(path),
            "sha256": sha256(path),
            "capture_id": capture.capture_id,
            "protocol": capture.protocol,
            "motion": capture.motion,
            "complete": capture.complete,
            "interrupted": capture.interrupted,
            "duration_s": capture.duration_s,
            "event_counts": dict(capture.counts),
            "raw_positions_excluding_fused": len(points),
            "excluded_fused_positions": capture.fused_positions_excluded,
            "stream": stream_metrics(
                points, capture.duration_s, capture.start_wall, capture.end_wall
            ),
            "local_precision": precision_metrics(points),
            "jump_diagnostics": jump_metrics(points),
            "reported_solver_residual": raw_position_residual_metrics(points),
            "rtk": rtk,
            "alignment": alignment,
            "position_vs_rtk": accuracy,
            "geometry_proxy": geometry_proxy_metrics(
                capture,
                points,
                alignments,
                args.max_dynamic_geometry_age_s,
            ),
            "range_vs_rtk": summarize_range_errors(ranges),
            "_position_pairs": pairs,
        }
        summaries.append(capture_summary)

    aggregates: dict[str, dict[str, Any]] = {}
    for role in sorted(ROLES):
        role_metrics: dict[str, Any] = {}
        for protocol in PROTOCOL_LABELS:
            selected = [
                summary
                for summary in summaries
                if summary["role"] == role and summary["protocol"] == protocol
            ]
            if selected:
                role_metrics[protocol] = aggregate_position_metrics(
                    summaries, role, protocol
                )
        aggregates[role] = role_metrics
    validation_protocols = set(aggregates.get("validation", {}))
    validation_summaries = [
        summary for summary in summaries if summary["role"] == "validation"
    ]
    evidence_gate = {
        "validation_protocol_matrix_complete": validation_protocols
        == set(PROTOCOL_LABELS),
        "validation_protocols": sorted(validation_protocols),
        "validation_accuracy_complete": bool(validation_summaries)
        and all(
            int(summary["position_vs_rtk"].get("pairs", 0))
            >= args.minimum_rtk_pairs
            for summary in validation_summaries
        ),
        "minimum_rtk_pairs_per_capture": args.minimum_rtk_pairs,
        "range_evidence_protocols": sorted(
            {
                summary["protocol"]
                for summary in summaries
                if summary["range_vs_rtk"]["available"]
            }
        ),
        "candidate_solver_evaluated": False,
        "ota_justified": False,
        "reason": "baseline only; no candidate solver has beaten held-out raw metrics",
    }
    public_summaries = []
    for summary in summaries:
        public = dict(summary)
        public.pop("_position_pairs", None)
        public_summaries.append(public)
    return {
        "schema_version": 1,
        "policy": {
            "position_stream": "raw only",
            "imu_positions_excluded": True,
            "temporal_filtering": "none",
            "rtk_alignment": "rigid 2-D anchor-only registration",
            "rtk_max_age_ms": args.rtk_max_age_ms,
            "range_rtk_max_age_ms": args.range_rtk_max_age_ms,
            "calibration_and_validation_separated": True,
        },
        "captures": public_summaries,
        "aggregates": aggregates,
        "evidence_gate": evidence_gate,
        "_range_rows": all_range_rows,
        "_position_pairs": all_position_pairs,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capture",
        action="append",
        type=parse_capture,
        required=True,
        metavar="ROLE=PATH",
        help="Add a calibration or held-out validation capture.",
    )
    parser.add_argument("--tag-id", type=int, default=1)
    parser.add_argument("--rtk-max-age-ms", type=float, default=250.0)
    parser.add_argument("--range-rtk-max-age-ms", type=float, default=750.0)
    parser.add_argument("--max-dynamic-geometry-age-s", type=float, default=2.0)
    parser.add_argument("--minimum-rtk-pairs", type=int, default=20)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = evaluate(args)
    range_rows = summary.pop("_range_rows")
    position_pairs = summary.pop("_position_pairs")
    (args.output_dir / "baseline_summary.json").write_text(
        json.dumps(json_clean(summary), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(args.output_dir / "position_rtk_pairs.csv", position_pairs)
    write_csv(args.output_dir / "range_rtk_pairs.csv", range_rows)
    flattened = []
    for capture in summary["captures"]:
        row: dict[str, Any] = {}
        flatten("", capture, row)
        flattened.append(row)
    write_csv(args.output_dir / "capture_metrics.csv", flattened)
    (args.output_dir / "README.md").write_text(
        build_readme(summary), encoding="utf-8"
    )
    print(json.dumps(summary["evidence_gate"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
