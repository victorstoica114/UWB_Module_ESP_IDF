#!/usr/bin/env python3
"""Generate figures for the 2026-07-31 Native DS-TWR speed revalidation."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PROFILE_DIRS = {
    410: "",
    64: "profile_64ms",
    60: "candidate_60ms_validation",
    59: "candidate_59ms_validation",
    58: "candidate_58ms_validation",
    57: "candidate_57ms_screen",
    49: "candidate_49ms_screen",
    33: "profile_33ms",
    29: "profile_29ms_validation",
    25: "profile_25ms_screen",
    21: "profile_21ms_screen",
    17: "profile_17ms_screen",
    13: "profile_13ms_screen",
}

MAIN_PROFILES = [410, 64, 60, 59, 58, 33, 29, 25, 21, 17, 13]
BOUNDARY_PROFILES = [64, 60, 59, 58, 57, 49]
ACCEPTED = {410, 64, 60}
TIMINGS = {
    410: (100, 10),
    64: (15, 4),
    60: (14, 4),
    59: (14, 3),
    58: (14, 2),
    57: (14, 1),
    49: (12, 1),
    33: (8, 1),
    29: (7, 1),
    25: (6, 1),
    21: (5, 1),
    17: (4, 1),
    13: (3, 1),
}

BLUE = "#2563eb"
GREEN = "#16a34a"
RED = "#dc2626"
AMBER = "#d97706"
SLATE = "#475569"
GRID = "#cbd5e1"


def profile_path(root: Path, period_ms: int) -> Path:
    suffix = PROFILE_DIRS[period_ms]
    return root / suffix if suffix else root


def read_profile(root: Path, period_ms: int) -> dict:
    directory = profile_path(root, period_ms)
    summary = json.loads((directory / "summary.json").read_text())
    quality = json.loads((directory / "raw_quality.json").read_text())
    position = summary["position_metrics"]
    return {
        "period_ms": period_ms,
        "directory": directory,
        "summary": summary,
        "quality": quality,
        "coherent": position["coherent"],
        "rolling": position["rolling"],
        "delivery_pct": summary["sequence_metrics"]["delivery_pct"],
        "raw_ranges": sum(row["n"] for row in quality.values()),
        "negative": sum(row["negative"] for row in quality.values()),
        "spikes": sum(row["deviation_gt_10cm"] for row in quality.values()),
        "min_raw_m": min(row["min_m"] for row in quality.values()),
    }


def style_axis(axis) -> None:
    axis.grid(True, axis="y", color=GRID, alpha=0.55, linewidth=0.7)
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def save_figure(fig, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_rate_and_delivery(profiles: dict[int, dict], output_dir: Path) -> None:
    rows = [profiles[p] for p in MAIN_PROFILES]
    labels = [
        f"{row['period_ms']} ms" +
        ("" if row["period_ms"] in ACCEPTED else "*")
        for row in rows
    ]
    x = np.arange(len(rows))

    fig, (ax_rate, ax_delivery) = plt.subplots(
        2, 1, figsize=(10.5, 7.0), sharex=True,
        gridspec_kw={"height_ratios": [2.1, 1.0], "hspace": 0.10},
    )
    width = 0.36
    ax_rate.bar(
        x - width / 2,
        [row["coherent"]["rate_hz"] for row in rows],
        width,
        color=GREEN,
        alpha=0.90,
        label="coherent independent frames",
    )
    ax_rate.bar(
        x + width / 2,
        [row["rolling"]["rate_hz"] for row in rows],
        width,
        color=BLUE,
        alpha=0.72,
        label="rolling display updates",
    )
    ax_rate.set_ylabel("updates per second")
    ax_rate.set_title("Measured Native DS-TWR update rate and scheduler yield")
    ax_rate.legend(frameon=False, ncol=2, loc="upper left")
    style_axis(ax_rate)

    coherent_yield = []
    rolling_yield = []
    for row in rows:
        period = row["period_ms"]
        slot_ms, gap_ms = TIMINGS[period]
        average_period_ms = (
            4 * slot_ms + gap_ms + (slot_ms + 1.0) / 4.0
        )
        ideal_frame_hz = 1000.0 / average_period_ms
        coherent_yield.append(
            100.0 * row["coherent"]["rate_hz"] / ideal_frame_hz
        )
        rolling_yield.append(
            100.0 * row["rolling"]["rate_hz"] / (4.0 * ideal_frame_hz)
        )
    ax_delivery.bar(
        x - width / 2,
        coherent_yield,
        width,
        color=GREEN,
        alpha=0.90,
        label="coherent independent frames",
    )
    ax_delivery.bar(
        x + width / 2,
        rolling_yield,
        width,
        color=BLUE,
        alpha=0.72,
        label="rolling display updates",
    )
    ax_delivery.axhline(100.0, color=SLATE, linestyle="--", linewidth=1.0)
    ax_delivery.set_ylim(55, 102)
    ax_delivery.set_ylabel("scheduler yield [%]")
    ax_delivery.set_xticks(x, labels, rotation=36, ha="right")
    style_axis(ax_delivery)
    fig.text(
        0.5,
        0.012,
        "* Raw-integrity validation failed: at least one impossible "
        "distance was observed.",
        ha="center",
        va="bottom",
        fontsize=9,
        color=SLATE,
    )
    fig.subplots_adjust(bottom=0.18)
    save_figure(fig, output_dir, "01_rate_delivery")


def plot_central_precision(profiles: dict[int, dict], output_dir: Path) -> None:
    rows = [profiles[p] for p in MAIN_PROFILES]
    labels = [
        f"{row['period_ms']} ms" +
        ("" if row["period_ms"] in ACCEPTED else "*")
        for row in rows
    ]
    x = np.arange(len(rows))
    width = 0.36

    fig, ax = plt.subplots(figsize=(10.5, 4.7))
    ax.bar(
        x - width / 2,
        [row["coherent"]["precision_rms_cm"] for row in rows],
        width,
        color=GREEN,
        alpha=0.88,
        label="precision RMS",
    )
    ax.bar(
        x + width / 2,
        [row["coherent"]["precision_cep95_cm"] for row in rows],
        width,
        color=BLUE,
        alpha=0.70,
        label="precision CEP95",
    )
    ax.axhline(
        profiles[410]["coherent"]["precision_rms_cm"],
        color=SLATE,
        linestyle="--",
        linewidth=1.0,
    )
    ax.set_ylabel("position dispersion [cm]")
    ax.set_title(
        "Central static precision stays tight even when raw timing integrity fails"
    )
    ax.set_xticks(x, labels, rotation=36, ha="right")
    ax.legend(frameon=False, ncol=2)
    style_axis(ax)
    fig.text(
        0.5,
        0.012,
        "* Raw-integrity validation failed; central precision alone did "
        "not expose the fault.",
        ha="center",
        va="bottom",
        fontsize=9,
        color=SLATE,
    )
    fig.subplots_adjust(bottom=0.25)
    save_figure(fig, output_dir, "02_central_precision")


def plot_raw_anomalies(profiles: dict[int, dict], output_dir: Path) -> None:
    rows = [profiles[p] for p in MAIN_PROFILES]
    labels = [f"{row['period_ms']} ms" for row in rows]
    x = np.arange(len(rows))
    negative_rates = [
        10000.0 * row["negative"] / row["raw_ranges"] for row in rows
    ]
    spike_rates = [
        10000.0 * row["spikes"] / row["raw_ranges"] for row in rows
    ]

    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    width = 0.36
    ax.bar(
        x - width / 2,
        negative_rates,
        width,
        color=RED,
        alpha=0.9,
        label="negative ranges",
    )
    ax.bar(
        x + width / 2,
        spike_rates,
        width,
        color=AMBER,
        alpha=0.8,
        label=">10 cm deviations from anchor median",
    )
    ax.set_ylabel("events per 10,000 raw ranges")
    ax.set_title("Raw-range integrity, before solver rejection")
    ax.set_xticks(x, labels, rotation=36, ha="right")
    ax.legend(frameon=False, ncol=2)
    style_axis(ax)
    save_figure(fig, output_dir, "03_raw_anomalies")


def find_event_file(directory: Path) -> Path:
    candidates = sorted((directory / "data").glob("*.jsonl"))
    return next(path for path in candidates if ".logs." not in path.name)


def load_anchor_ranges(directory: Path, anchor_id: int) -> tuple[np.ndarray, np.ndarray]:
    timestamps: list[float] = []
    distances: list[float] = []
    with find_event_file(directory).open() as handle:
        for line in handle:
            event = json.loads(line)
            if event.get("kind") != "ds_range":
                continue
            if int(event.get("anchor_id", -1)) != anchor_id:
                continue
            distance = event.get("distance_m")
            timestamp = event.get("received_at")
            if not isinstance(distance, (int, float)):
                continue
            if not isinstance(timestamp, (int, float)):
                continue
            timestamps.append(float(timestamp))
            distances.append(float(distance))
    time = np.asarray(timestamps)
    time -= time.min()
    return time, np.asarray(distances)


def plot_boundary_integrity(
    profiles: dict[int, dict], raw_root: Path, output_dir: Path
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 7.2), sharex=True)
    for axis, period in zip(axes, [58, 59, 60], strict=True):
        time, distance = load_anchor_ranges(profile_path(raw_root, period), 2)
        median = float(np.median(distance))
        error_cm = (distance - median) * 100.0
        normal = np.abs(error_cm) <= 10.0
        axis.scatter(
            time[normal],
            error_cm[normal],
            s=5,
            alpha=0.35,
            color=BLUE,
            linewidths=0,
            label="normal raw A2 ranges",
        )
        axis.scatter(
            time[~normal],
            error_cm[~normal],
            s=34,
            marker="x",
            color=RED,
            linewidths=1.5,
            label="impossible raw range",
            zorder=4,
        )
        axis.set_yscale("symlog", linthresh=10, linscale=0.8)
        axis.set_ylim(-4000, 25)
        axis.set_ylabel(f"{period} ms\nA2 error [cm]")
        axis.axhspan(-10, 10, color=GREEN, alpha=0.07)
        axis.grid(True, color=GRID, alpha=0.5, linewidth=0.7)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        axis.text(
            0.99,
            0.88,
            f"{profiles[period]['negative']} negative / "
            f"{profiles[period]['raw_ranges']} total ranges",
            transform=axis.transAxes,
            ha="right",
            va="top",
            fontsize=9,
        )
    axes[0].set_title(
        "Boundary validation: rare A2 failures emerge below a 4 ms frame gap"
    )
    axes[-1].set_xlabel("capture time [s]")
    handles, labels = axes[0].get_legend_handles_labels()
    if len(handles) == 1:
        extra = axes[1].get_legend_handles_labels()
        handles += extra[0]
        labels += extra[1]
    fig.legend(handles, labels, frameon=False, ncol=2, loc="lower center")
    fig.subplots_adjust(bottom=0.11, hspace=0.12)
    save_figure(fig, output_dir, "04_boundary_integrity")


def load_coherent_positions(directory: Path) -> np.ndarray:
    points: list[tuple[float, float]] = []
    with (directory / "positions.csv").open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row["mode"] != "coherent":
                continue
            points.append((float(row["x_m"]), float(row["y_m"])))
    return np.asarray(points)


def ecdf(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    values = np.sort(values)
    return values, np.arange(1, len(values) + 1) / len(values)


def plot_precision_cdf(profiles: dict[int, dict], output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(9.6, 5.2))
    specs = [
        (410, "410 ms control", SLATE, "--"),
        (64, "64 ms reserve", BLUE, "-."),
        (60, "60 ms recommended", GREEN, "-"),
    ]
    for period, label, color, linestyle in specs:
        points = load_coherent_positions(profiles[period]["directory"])
        centered = points - points.mean(axis=0)
        radius_cm = np.linalg.norm(centered, axis=1) * 100.0
        x, y = ecdf(radius_cm)
        ax.plot(x, y, label=label, color=color, linestyle=linestyle, linewidth=2)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.005)
    ax.set_xlabel("radial deviation from each capture mean [cm]")
    ax.set_ylabel("empirical cumulative probability")
    ax.set_title("Unfiltered coherent-position precision")
    ax.legend(frameon=False, loc="lower right")
    ax.grid(True, color=GRID, alpha=0.55, linewidth=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    save_figure(fig, output_dir, "05_precision_cdf")


def plot_anchor_stability(profiles: dict[int, dict], output_dir: Path) -> None:
    anchors = ["2", "3", "4", "5"]
    x = np.arange(len(anchors))
    fig, ax = plt.subplots(figsize=(9.6, 4.8))
    width = 0.24
    for offset, period, label, color in [
        (-width, 410, "410 ms control", SLATE),
        (0.0, 64, "64 ms reserve", BLUE),
        (width, 60, "60 ms recommended", GREEN),
    ]:
        ax.bar(
            x + offset,
            [profiles[period]["quality"][anchor]["std_cm"] for anchor in anchors],
            width,
            color=color,
            alpha=0.85,
            label=label,
        )
    ax.set_xticks(x, [f"A{anchor}" for anchor in anchors])
    ax.set_ylabel("raw range standard deviation [cm]")
    ax.set_title("Per-anchor raw-range stability for accepted profiles")
    ax.legend(frameon=False, ncol=3)
    style_axis(ax)
    save_figure(fig, output_dir, "06_anchor_stability")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("reports/bundles/native_ds_speed_revalidation_20260731/analysis"),
    )
    parser.add_argument(
        "--raw-root",
        type=Path,
        default=Path("reports/bundles/native_ds_speed_revalidation_20260731/raw"),
    )
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    raw_root = args.raw_root.resolve()
    output_dir = (
        args.output_dir.resolve() if args.output_dir else root.parent / "figures"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    profiles = {
        period: read_profile(root, period)
        for period in sorted(PROFILE_DIRS, reverse=True)
    }
    plot_rate_and_delivery(profiles, output_dir)
    plot_central_precision(profiles, output_dir)
    plot_raw_anomalies(profiles, output_dir)
    plot_boundary_integrity(profiles, raw_root, output_dir)
    plot_precision_cdf(profiles, output_dir)
    plot_anchor_stability(profiles, output_dir)
    print(f"generated six report figures in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
