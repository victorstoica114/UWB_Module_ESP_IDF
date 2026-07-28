#!/usr/bin/env python3
"""Generate figures and metrics for the Passive DS-TWR pipeline field check."""

from __future__ import annotations

import argparse
import collections
import json
import math
import pathlib
import statistics
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TRUTH_X_M = 1.5
TRUTH_Y_M = 1.5
PIPELINE_COUNTERS = (
    "completed",
    "response_timeouts",
    "final_timeouts",
    "invalid_frames",
    "state_collisions",
    "schedule_overruns",
    "rx_rearm_failures",
)
CAPTURES = (
    ("legacy_frame_1000us", "Legacy A1", "all"),
    ("deadline_frame_1000us", "Deadline + Frame", "all"),
    (
        "deadline_rolling_100hz_1000us",
        "Deadline + Rolling",
        "independent",
    ),
    ("legacy_frame_return_1000us", "Legacy A2", "all"),
)


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def position_metrics(
    rows: list[dict[str, Any]], x_key: str, y_key: str, duration_s: float
) -> dict[str, float | int]:
    points = [
        (float(row[x_key]), float(row[y_key]))
        for row in rows
        if row.get(x_key) is not None and row.get(y_key) is not None
    ]
    errors_cm = [
        math.hypot(x - TRUTH_X_M, y - TRUTH_Y_M) * 100.0
        for x, y in points
    ]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return {
        "count": len(points),
        "rate_hz": len(points) / duration_s,
        "bias_x_cm": (statistics.fmean(xs) - TRUTH_X_M) * 100.0,
        "bias_y_cm": (statistics.fmean(ys) - TRUTH_Y_M) * 100.0,
        "rmse_cm": math.sqrt(statistics.fmean(error * error for error in errors_cm)),
        "median_cm": percentile(errors_cm, 0.50),
        "p95_cm": percentile(errors_cm, 0.95),
        "p99_cm": percentile(errors_cm, 0.99),
        "max_cm": max(errors_cm),
        "std_x_cm": statistics.pstdev(xs) * 100.0,
        "std_y_cm": statistics.pstdev(ys) * 100.0,
    }


def pipeline_delta(
    initial: list[dict[str, Any]], final: list[dict[str, Any]]
) -> dict[str, int]:
    before = {int(item["module_id"]): item for item in initial}
    after = {int(item["module_id"]): item for item in final}
    result = {key: 0 for key in PIPELINE_COUNTERS}
    for module_id, final_status in after.items():
        final_stats = final_status.get("passive_ds_pipeline_stats") or {}
        initial_stats = (
            before.get(module_id, {}).get("passive_ds_pipeline_stats") or {}
        )
        for key in PIPELINE_COUNTERS:
            result[key] += int(final_stats.get(key) or 0) - int(
                initial_stats.get(key) or 0
            )
    return result


def load_capture(data_dir: pathlib.Path, block: str, subset: str) -> dict[str, Any]:
    metadata = json.loads((data_dir / f"{block}.metadata.json").read_text())
    all_positions: list[dict[str, Any]] = []
    anchor_ranges: dict[tuple[int, int], list[float]] = collections.defaultdict(list)
    with (data_dir / f"{block}.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("kind") == "local_position":
                all_positions.append(row)
            elif row.get("kind") == "anchor_range":
                pair = tuple(
                    sorted(
                        (
                            int(row["anchor_a_id"]),
                            int(row["anchor_b_id"]),
                        )
                    )
                )
                anchor_ranges[pair].append(float(row["distance_m"]))

    positions = all_positions
    if subset == "independent":
        positions = [
            row for row in all_positions if row.get("independent_frame") is True
        ]
    rolling_positions = [
        row for row in all_positions if row.get("independent_frame") is False
    ]
    duration_s = float(metadata["duration_sec"])
    return {
        "block": block,
        "duration_s": duration_s,
        "positions": positions,
        "all_positions": all_positions,
        "filtered": position_metrics(positions, "x_m", "y_m", duration_s),
        "raw": position_metrics(positions, "raw_x_m", "raw_y_m", duration_s),
        "independent_rate_hz": len(positions) / duration_s,
        "rolling_rate_hz": len(rolling_positions) / duration_s,
        "total_solver_rate_hz": len(all_positions) / duration_s,
        "pipeline": pipeline_delta(
            metadata["initial_status"], metadata["final_status"]
        ),
        "anchor_ranges": anchor_ranges,
    }


def configure_plot_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 140,
            "savefig.dpi": 180,
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "legend.frameon": False,
        }
    )


def save_update_rate(captures: list[dict[str, Any]], figures_dir: pathlib.Path) -> None:
    labels = [capture["label"] for capture in captures]
    independent = np.array(
        [capture["independent_rate_hz"] for capture in captures]
    )
    rolling = np.array([capture["rolling_rate_hz"] for capture in captures])
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8.0, 4.3))
    ax.bar(x, independent, color="#2f6fbb", label="independent frames/s")
    ax.bar(
        x,
        rolling,
        bottom=independent,
        color="#77b5e8",
        label="additional rolling updates/s",
    )
    ax.axhline(85.0, color="#c23b22", linestyle="--", linewidth=1.4, label="85/s target")
    for index, total in enumerate(independent + rolling):
        ax.text(index, total + 3.0, f"{total:.1f}", ha="center", fontweight="bold")
    ax.set_xticks(x, labels)
    ax.set_ylabel("updates per second")
    ax.set_ylim(0, 190)
    ax.set_title("Measured position throughput")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_update_rate.pdf", bbox_inches="tight")
    plt.close(fig)


def save_accuracy(captures: list[dict[str, Any]], figures_dir: pathlib.Path) -> None:
    labels = [capture["label"] for capture in captures]
    filtered_rmse = [capture["filtered"]["rmse_cm"] for capture in captures]
    filtered_p95 = [capture["filtered"]["p95_cm"] for capture in captures]
    raw_rmse = [capture["raw"]["rmse_cm"] for capture in captures]
    raw_p95 = [capture["raw"]["p95_cm"] for capture in captures]
    x = np.arange(len(labels))
    width = 0.19
    fig, ax = plt.subplots(figsize=(8.3, 4.5))
    ax.bar(x - 1.5 * width, filtered_rmse, width, label="EKF RMSE", color="#2f6fbb")
    ax.bar(x - 0.5 * width, filtered_p95, width, label="EKF P95", color="#77b5e8")
    ax.bar(x + 0.5 * width, raw_rmse, width, label="raw RMSE", color="#d2823c")
    ax.bar(x + 1.5 * width, raw_p95, width, label="raw P95", color="#e8b06f")
    ax.set_xticks(x, labels)
    ax.set_ylabel("2-D position error [cm]")
    ax.set_title("Accuracy check against the surveyed centre")
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_accuracy.pdf", bbox_inches="tight")
    plt.close(fig)


def save_error_cdf(captures: list[dict[str, Any]], figures_dir: pathlib.Path) -> None:
    colors = ("#626c78", "#2f6fbb", "#168f5b", "#b04d45")
    fig, ax = plt.subplots(figsize=(7.7, 4.6))
    for capture, color in zip(captures, colors):
        errors = sorted(
            math.hypot(
                float(row["x_m"]) - TRUTH_X_M,
                float(row["y_m"]) - TRUTH_Y_M,
            )
            * 100.0
            for row in capture["positions"]
        )
        cumulative = np.arange(1, len(errors) + 1) / len(errors)
        ax.plot(errors, cumulative, linewidth=1.7, color=color, label=capture["label"])
    ax.axhline(0.95, color="#777777", linestyle=":", linewidth=1)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.01)
    ax.set_xlabel("2-D filtered position error [cm]")
    ax.set_ylabel("cumulative probability")
    ax.set_title("Position-error empirical CDF")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_error_cdf.pdf", bbox_inches="tight")
    plt.close(fig)


def save_position_clouds(
    captures: list[dict[str, Any]], figures_dir: pathlib.Path
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(8.0, 7.4), sharex=True, sharey=True)
    for ax, capture in zip(axes.flat, captures):
        rows = capture["positions"]
        stride = max(1, len(rows) // 1800)
        dx = [
            (float(row["x_m"]) - TRUTH_X_M) * 100.0
            for row in rows[::stride]
        ]
        dy = [
            (float(row["y_m"]) - TRUTH_Y_M) * 100.0
            for row in rows[::stride]
        ]
        ax.scatter(dx, dy, s=5, alpha=0.18, color="#2f6fbb", edgecolors="none")
        ax.scatter([0], [0], marker="*", s=100, color="#c23b22", label="truth")
        ax.set_title(capture["label"])
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
    for ax in axes[-1, :]:
        ax.set_xlabel("x error [cm]")
    for ax in axes[:, 0]:
        ax.set_ylabel("y error [cm]")
    fig.suptitle("Filtered static position clouds", y=0.995)
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_position_clouds.pdf", bbox_inches="tight")
    plt.close(fig)


def save_pipeline_diagnostics(
    captures: list[dict[str, Any]], figures_dir: pathlib.Path
) -> None:
    labels = [capture["label"] for capture in captures[:3]]
    complete = np.array([capture["pipeline"]["completed"] for capture in captures[:3]])
    response = np.array(
        [capture["pipeline"]["response_timeouts"] for capture in captures[:3]]
    )
    final = np.array(
        [capture["pipeline"]["final_timeouts"] for capture in captures[:3]]
    )
    x = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 4.0))
    axes[0].bar(x, complete, color=("#626c78", "#2f6fbb", "#168f5b"))
    axes[0].set_xticks(x, labels, rotation=12)
    axes[0].set_ylabel("completed exchanges / 60 s")
    axes[0].set_title("Completed radio exchanges")
    axes[1].bar(x, response, color="#d2823c", label="RESP timeout")
    axes[1].bar(x, final, bottom=response, color="#b04d45", label="FINAL timeout")
    axes[1].set_xticks(x, labels, rotation=12)
    axes[1].set_ylabel("timeouts / 60 s")
    axes[1].set_title("Pipeline timeouts")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_diagnostics.pdf", bbox_inches="tight")
    plt.close(fig)


def save_anchor_stability(
    captures: list[dict[str, Any]], figures_dir: pathlib.Path
) -> None:
    pairs = ((2, 3), (2, 4), (2, 5), (3, 4), (3, 5), (4, 5))
    labels = [f"A{a}–A{b}" for a, b in pairs]
    x = np.arange(len(pairs))
    width = 0.19
    colors = ("#626c78", "#2f6fbb", "#168f5b", "#b04d45")
    fig, ax = plt.subplots(figsize=(8.4, 4.5))
    for index, (capture, color) in enumerate(zip(captures, colors)):
        stds = [
            statistics.pstdev(capture["anchor_ranges"][pair]) * 100.0
            for pair in pairs
        ]
        ax.bar(
            x + (index - 1.5) * width,
            stds,
            width,
            color=color,
            label=capture["label"],
        )
    ax.set_xticks(x, labels)
    ax.set_ylabel("raw anchor-pair range standard deviation [cm]")
    ax.set_title("Anchor-range stability")
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "pipeline_anchor_stability.pdf", bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report-dir",
        default="reports/passive_ds_pipeline_20260728",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_dir = pathlib.Path(args.report_dir).expanduser().resolve()
    data_dir = report_dir / "data"
    figures_dir = report_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    configure_plot_style()

    captures: list[dict[str, Any]] = []
    for block, label, subset in CAPTURES:
        capture = load_capture(data_dir, block, subset)
        capture["label"] = label
        capture["subset"] = subset
        captures.append(capture)

    save_update_rate(captures, figures_dir)
    save_accuracy(captures, figures_dir)
    save_error_cdf(captures, figures_dir)
    save_position_clouds(captures, figures_dir)
    save_pipeline_diagnostics(captures, figures_dir)
    save_anchor_stability(captures, figures_dir)

    serializable = {
        "firmware": "7226cca",
        "truth_m": [TRUTH_X_M, TRUTH_Y_M],
        "captures": [
            {
                key: value
                for key, value in capture.items()
                if key
                not in {
                    "positions",
                    "all_positions",
                    "anchor_ranges",
                }
            }
            for capture in captures
        ],
    }
    (report_dir / "metrics.generated.json").write_text(
        json.dumps(serializable, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"wrote figures and metrics to {report_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
