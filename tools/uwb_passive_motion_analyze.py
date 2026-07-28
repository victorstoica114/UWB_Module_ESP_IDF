#!/usr/bin/env python3
"""Compare Passive DS-TWR motion continuity across EKF policies."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def event_time(row: dict[str, Any]) -> float:
    for key in ("event_received_at", "received_at", "captured_at"):
        value = row.get(key)
        if value is not None:
            return float(value)
    return 0.0


def load_positions(path: pathlib.Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seen: set[tuple[int, int]] = set()
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if (
                row.get("kind") != "local_position"
                or row.get("tdoa_protocol") != "passive_ds"
            ):
                continue
            key = (
                int(row.get("module_id") or 0),
                int(
                    row.get("position_event_id")
                    or row.get("position_stream_event_id")
                    or 0
                ),
            )
            if key in seen:
                continue
            seen.add(key)
            rows.append(row)
    rows.sort(key=event_time)
    return rows


def trajectory_metrics(
    rows: list[dict[str, Any]], x_key: str, y_key: str
) -> dict[str, float | int]:
    points = [
        (event_time(row), float(row[x_key]), float(row[y_key]))
        for row in rows
        if row.get(x_key) is not None and row.get(y_key) is not None
    ]
    if len(points) < 3:
        return {"count": len(points)}

    duration = max(1e-9, points[-1][0] - points[0][0])
    steps_m: list[float] = []
    speeds_m_s: list[float] = []
    changing = 0
    for previous, current in zip(points, points[1:]):
        dt = current[0] - previous[0]
        step = math.hypot(current[1] - previous[1], current[2] - previous[2])
        steps_m.append(step)
        changing += int(step >= 0.002)
        if dt > 0.0005:
            speeds_m_s.append(step / dt)

    turn_angles_deg: list[float] = []
    for first, middle, final in zip(points, points[1:], points[2:]):
        first_vector = (middle[1] - first[1], middle[2] - first[2])
        second_vector = (final[1] - middle[1], final[2] - middle[2])
        first_length = math.hypot(*first_vector)
        second_length = math.hypot(*second_vector)
        if first_length < 0.003 or second_length < 0.003:
            continue
        cosine = (
            first_vector[0] * second_vector[0]
            + first_vector[1] * second_vector[1]
        ) / (first_length * second_length)
        turn_angles_deg.append(
            math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        )

    xs = [point[1] for point in points]
    ys = [point[2] for point in points]
    return {
        "count": len(points),
        "duration_s": duration,
        "rate_hz": (len(points) - 1) / duration,
        "effective_2mm_change_rate_hz": changing / duration,
        "path_length_m": sum(steps_m),
        "step_p50_cm": percentile(steps_m, 0.50) * 100.0,
        "step_p95_cm": percentile(steps_m, 0.95) * 100.0,
        "step_p99_cm": percentile(steps_m, 0.99) * 100.0,
        "step_max_cm": max(steps_m) * 100.0,
        "step_over_5cm_per_s": sum(step > 0.05 for step in steps_m) / duration,
        "speed_p95_m_s": percentile(speeds_m_s, 0.95),
        "speed_max_m_s": max(speeds_m_s, default=float("nan")),
        "turn_p50_deg": percentile(turn_angles_deg, 0.50),
        "turn_p95_deg": percentile(turn_angles_deg, 0.95),
        "turn_over_45deg_per_s": (
            sum(angle > 45.0 for angle in turn_angles_deg) / duration
        ),
        "turn_over_90deg_per_s": (
            sum(angle > 90.0 for angle in turn_angles_deg) / duration
        ),
        "std_x_cm": statistics.pstdev(xs) * 100.0,
        "std_y_cm": statistics.pstdev(ys) * 100.0,
    }


def telemetry_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"count": 0}
    ordered = sorted(
        rows,
        key=lambda row: int(
            row.get("position_event_id")
            or row.get("position_stream_event_id")
            or 0
        ),
    )
    gaps_ms = [
        (int(current.get("uptime_ms") or 0) -
         int(previous.get("uptime_ms") or 0)) & 0xFFFFFFFF
        for previous, current in zip(ordered, ordered[1:])
    ]
    coherent = [
        row
        for row in ordered
        if row.get("independent_frame") is True
        and row.get("coherent_batch_telemetry") is True
    ]
    spans = [
        float(row["batch_span_ms"])
        for row in coherent
        if row.get("batch_span_ms") is not None
    ]
    ages = [
        float(row["batch_max_age_ms"])
        for row in coherent
        if row.get("batch_max_age_ms") is not None
    ]
    reason_labels = {
        1: "frame_incomplete",
        2: "frame_mismatch",
        3: "too_few",
        4: "singular",
        5: "bounds",
        6: "rms",
        7: "out_of_order",
    }
    reason_events = {
        label: sum(
            bool(int(row.get("rejection_reason_mask") or 0) & (1 << bit))
            for row in ordered
        )
        for bit, label in reason_labels.items()
    }
    return {
        "count": len(ordered),
        "same_tick_pct": (
            100.0 * sum(gap == 0 for gap in gaps_ms) / len(gaps_ms)
            if gaps_ms else 0.0
        ),
        "gap_p50_ms": percentile(gaps_ms, 0.50),
        "gap_p95_ms": percentile(gaps_ms, 0.95),
        "gap_p99_ms": percentile(gaps_ms, 0.99),
        "gap_max_ms": max(gaps_ms, default=0),
        "gaps_over_100ms": sum(gap > 100 for gap in gaps_ms),
        "coherent_count": len(coherent),
        "batch_span_p50_ms": percentile(spans, 0.50),
        "batch_span_p95_ms": percentile(spans, 0.95),
        "batch_span_max_ms": max(spans, default=float("nan")),
        "batch_age_p50_ms": percentile(ages, 0.50),
        "batch_age_p95_ms": percentile(ages, 0.95),
        "batch_age_max_ms": max(ages, default=float("nan")),
        "rejected_since_last_sum": sum(
            int(row.get("rejected_since_last") or 0)
            for row in ordered
        ),
        "position_rejected_total": max(
            (
                int(row.get("position_rejected_count") or 0)
                for row in ordered
            ),
            default=0,
        ),
        "rejection_reason_events": reason_events,
    }


def capture_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"count": 0}
    duration = max(1e-9, event_time(rows[-1]) - event_time(rows[0]))
    independent = [
        row for row in rows if row.get("independent_frame") is True
    ]
    superframes = [
        row for row in rows if row.get("complete_superframe") is True
    ]
    corrections = [
        row
        for row in rows
        if row.get("filter_correction", True) is True
    ]
    geometry_versions = {
        int(row["geometry_version"])
        for row in rows
        if row.get("geometry_version") is not None
    }
    return {
        "count": len(rows),
        "duration_s": duration,
        "solver_rate_hz": len(rows) / duration,
        "independent_rate_hz": len(independent) / duration,
        "superframe_rate_hz": len(superframes) / duration,
        "filter_correction_rate_hz": len(corrections) / duration,
        "geometry_version_rate_hz": max(
            0, len(geometry_versions) - 1
        ) / duration,
        "telemetry": telemetry_metrics(rows),
        "all_filtered": trajectory_metrics(rows, "x_m", "y_m"),
        "all_raw": trajectory_metrics(rows, "raw_x_m", "raw_y_m"),
        "independent_filtered": trajectory_metrics(
            independent, "x_m", "y_m"
        ),
        "independent_raw": trajectory_metrics(
            independent, "raw_x_m", "raw_y_m"
        ),
    }


def parse_capture(value: str) -> tuple[str, pathlib.Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("use LABEL=PATH")
    label, raw_path = value.split("=", 1)
    label = label.strip()
    path = pathlib.Path(raw_path).expanduser().resolve()
    if not label or not path.is_file():
        raise argparse.ArgumentTypeError(f"invalid capture {value!r}")
    return label, path


def save_trajectories(
    captures: list[tuple[str, list[dict[str, Any]]]], output_dir: pathlib.Path
) -> None:
    figure, axes = plt.subplots(
        1, len(captures), figsize=(5.0 * len(captures), 4.8), squeeze=False
    )
    for axis, (label, rows) in zip(axes[0], captures):
        independent = [
            row for row in rows if row.get("independent_frame") is True
        ]
        raw_x = [float(row["raw_x_m"]) for row in independent]
        raw_y = [float(row["raw_y_m"]) for row in independent]
        ekf_x = [float(row["x_m"]) for row in independent]
        ekf_y = [float(row["y_m"]) for row in independent]
        axis.plot(
            raw_x, raw_y, color="#8b95a3", linewidth=0.8,
            linestyle="--", alpha=0.7, label="raw independent"
        )
        axis.plot(
            ekf_x, ekf_y, color="#2b64d8", linewidth=1.1,
            alpha=0.85, label="EKF independent"
        )
        axis.set_title(label)
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.grid(alpha=0.22)
        axis.legend()
    figure.suptitle("Passive DS-TWR motion continuity")
    figure.tight_layout()
    figure.savefig(output_dir / "motion_trajectories.pdf", bbox_inches="tight")
    figure.savefig(output_dir / "motion_trajectories.png", bbox_inches="tight")
    plt.close(figure)


def save_rates(
    labels: list[str], metrics: list[dict[str, Any]], output_dir: pathlib.Path
) -> None:
    series = (
        ("solver", "solver_rate_hz", "#77b5e8"),
        ("independent", "independent_rate_hz", "#2f6fbb"),
        ("superframe", "superframe_rate_hz", "#7d69b4"),
        ("EKF correction", "filter_correction_rate_hz", "#168f5b"),
    )
    x = np.arange(len(labels))
    width = 0.18
    figure, axis = plt.subplots(figsize=(max(7.0, 2.3 * len(labels)), 4.4))
    for index, (name, key, color) in enumerate(series):
        values = [float(item.get(key) or 0.0) for item in metrics]
        axis.bar(
            x + (index - 1.5) * width, values, width,
            label=name, color=color
        )
    axis.set_xticks(x, labels)
    axis.set_ylabel("events/s")
    axis.set_title("Solver information and EKF correction rates")
    axis.grid(axis="y", alpha=0.22)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "motion_rates.pdf", bbox_inches="tight")
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze Passive DS-TWR motion continuity without truth."
    )
    parser.add_argument(
        "--capture", action="append", type=parse_capture, required=True,
        help="Capture in LABEL=PATH form; repeat for each EKF policy.",
    )
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    loaded = [(label, load_positions(path)) for label, path in args.capture]
    metrics = [capture_metrics(rows) for _, rows in loaded]
    result = {
        "captures": [
            {
                "label": label,
                "source": str(path),
                "metrics": item,
            }
            for (label, path), item in zip(args.capture, metrics)
        ]
    }
    (output_dir / "motion_metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    save_trajectories(loaded, output_dir)
    save_rates([label for label, _ in loaded], metrics, output_dir)
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"wrote motion analysis to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
