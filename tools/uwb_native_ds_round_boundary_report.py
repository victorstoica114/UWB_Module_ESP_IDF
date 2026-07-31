#!/usr/bin/env python3
"""Generate figures and a machine-readable summary for the round-boundary report."""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_native_ds_round_boundary import capture_summary


BLUE = "#2563eb"
GREEN = "#16a34a"
RED = "#dc2626"
AMBER = "#d97706"
SLATE = "#475569"
LIGHT = "#e2e8f0"
GRID = "#cbd5e1"

CAPTURES = (
    ("59 ms", "candidate_59ms_round_boundary_3600s", False),
    ("58 ms", "candidate_58ms_round_boundary_3600s", False),
    ("57 ms", "candidate_57ms_round_boundary_3600s", False),
    ("53 ms", "candidate_53ms_round_boundary_3600s", False),
    ("53 ms guarded", "candidate_53ms_guarded_3600s", True),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report-dir",
        type=Path,
        default=Path("reports/native_ds_round_boundary_20260731"),
    )
    return parser.parse_args()


def style_axis(axis) -> None:
    axis.grid(True, axis="y", color=GRID, alpha=0.6, linewidth=0.7)
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def save(fig, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def load_summaries(report_dir: Path) -> list[dict]:
    rows = []
    for label, stem, guarded in CAPTURES:
        path = report_dir / "data" / f"{stem}.logs.jsonl"
        summary = capture_summary(path)
        rejects = summary["source_rejects"]
        duration = float(summary["duration_sec"])
        rows.append(
            {
                "label": label,
                "stem": stem,
                "guarded": guarded,
                "duration_sec": duration,
                "valid_results": rejects["valid_results"],
                "valid_rate_hz": rejects["valid_results"] / duration,
                "response_timeouts": rejects["response_timeouts"],
                "final_timeouts": rejects["final_timeouts"],
                "context_mismatches": rejects["context_mismatches"],
                "source_rejects": rejects["total"],
                "classes": rejects["classes"],
                "anchors": rejects["anchors"],
                "reasons": rejects["reasons"],
            }
        )
    return rows


def plot_throughput(rows: list[dict], output_dir: Path) -> None:
    labels = [row["label"] for row in rows]
    x = np.arange(len(rows))
    colors = [GREEN if row["guarded"] else BLUE for row in rows]

    fig, ax = plt.subplots(figsize=(9.2, 4.5))
    bars = ax.bar(x, [row["valid_rate_hz"] for row in rows], color=colors, alpha=0.88)
    for bar, row in zip(bars, rows):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.35,
            f"{row['valid_rate_hz']:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_ylim(0, 73)
    ax.set_ylabel("valid ranges per second")
    ax.set_title("Delivered Native DS-TWR range throughput")
    ax.set_xticks(x, labels)
    ax.legend(
        handles=[
            plt.Rectangle((0, 0), 1, 1, color=BLUE, alpha=0.88, label="before boundary guard"),
            plt.Rectangle((0, 0), 1, 1, color=GREEN, alpha=0.88, label="guarded validation"),
        ],
        frameon=False,
        ncol=2,
    )
    style_axis(ax)
    save(fig, output_dir, "01_valid_throughput")


def plot_boundary_rejects(rows: list[dict], output_dir: Path) -> None:
    labels = [row["label"] for row in rows]
    x = np.arange(len(rows))
    geometry = [row["classes"].get("geometry_exchange", 0) for row in rows]
    post = [
        row["classes"].get("first_slot_after_geometry", 0)
        + row["classes"].get("later_slot_same_post_geometry_round", 0)
        for row in rows
    ]
    ordinary = [row["classes"].get("ordinary_round", 0) for row in rows]

    fig, ax = plt.subplots(figsize=(9.2, 4.7))
    ax.bar(x, geometry, color=RED, alpha=0.88, label="geometry exchange")
    ax.bar(x, post, bottom=geometry, color=AMBER, alpha=0.88, label="first post-geometry round")
    ax.bar(
        x,
        ordinary,
        bottom=np.asarray(geometry) + np.asarray(post),
        color=SLATE,
        alpha=0.8,
        label="ordinary ranging round",
    )
    for index, row in enumerate(rows):
        ax.text(index, row["source_rejects"] + 2.0, str(row["source_rejects"]), ha="center", fontsize=9)
    ax.set_ylim(0, 112)
    ax.set_ylabel("physically impossible ToFs rejected")
    ax.set_title("Every source rejection is localized to the geometry boundary")
    ax.set_xticks(x, labels)
    ax.legend(frameon=False, ncol=3, loc="upper left")
    style_axis(ax)
    save(fig, output_dir, "02_boundary_rejects")


def plot_failure_rates(rows: list[dict], output_dir: Path) -> None:
    labels = [row["label"] for row in rows]
    x = np.arange(len(rows))
    width = 0.25

    def per_thousand(row: dict, key: str) -> float:
        return 1000.0 * row[key] / row["valid_results"]

    fig, ax = plt.subplots(figsize=(9.2, 4.7))
    ax.bar(
        x - width,
        [per_thousand(row, "response_timeouts") for row in rows],
        width,
        color=BLUE,
        alpha=0.8,
        label="RESP timeouts",
    )
    ax.bar(
        x,
        [per_thousand(row, "final_timeouts") for row in rows],
        width,
        color=AMBER,
        alpha=0.88,
        label="FINAL timeouts",
    )
    ax.bar(
        x + width,
        [per_thousand(row, "source_rejects") for row in rows],
        width,
        color=RED,
        alpha=0.88,
        label="source ToF rejects",
    )
    ax.set_ylabel("events per 1,000 delivered ranges")
    ax.set_title("Failure burden falls sharply after boundary isolation")
    ax.set_xticks(x, labels)
    ax.legend(frameon=False, ncol=3)
    style_axis(ax)
    save(fig, output_dir, "03_failure_rates")


def minute_series(path: Path, duration_sec: float) -> tuple[np.ndarray, np.ndarray]:
    bins = max(1, int(np.ceil(duration_sec / 60.0)))
    valid = np.zeros(bins, dtype=int)
    rejects = np.zeros(bins, dtype=int)
    start: float | None = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            stamp = item.get("received_at")
            if stamp is None:
                continue
            stamp = float(stamp)
            if start is None:
                start = stamp
            index = min(bins - 1, max(0, int((stamp - start) // 60.0)))
            message = str(item.get("message") or "")
            if "UWB_RANGING result" in message:
                valid[index] += 1
            if "UWB_RANGING rejected reason=" in message:
                rejects[index] += 1
    return valid / 60.0, np.cumsum(rejects)


def plot_before_after_timeline(rows: list[dict], report_dir: Path, output_dir: Path) -> None:
    before = next(row for row in rows if row["stem"] == "candidate_53ms_round_boundary_3600s")
    after = next(row for row in rows if row["stem"] == "candidate_53ms_guarded_3600s")
    before_rate, before_reject = minute_series(
        report_dir / "data" / f"{before['stem']}.logs.jsonl", before["duration_sec"]
    )
    after_rate, after_reject = minute_series(
        report_dir / "data" / f"{after['stem']}.logs.jsonl", after["duration_sec"]
    )
    minutes_before = np.arange(len(before_rate)) + 1
    minutes_after = np.arange(len(after_rate)) + 1

    fig, (ax_rate, ax_reject) = plt.subplots(
        2,
        1,
        figsize=(9.5, 6.2),
        sharex=True,
        gridspec_kw={"height_ratios": [1.15, 1.0], "hspace": 0.12},
    )
    ax_rate.plot(minutes_before, before_rate, color=BLUE, linewidth=1.6, label="53 ms before guard")
    ax_rate.plot(minutes_after, after_rate, color=GREEN, linewidth=1.6, label="53 ms guarded")
    ax_rate.axhline(before["valid_rate_hz"], color=BLUE, linewidth=0.9, linestyle="--", alpha=0.7)
    ax_rate.axhline(after["valid_rate_hz"], color=GREEN, linewidth=0.9, linestyle="--", alpha=0.7)
    ax_rate.set_ylabel("valid ranges/s\n(60 s bins)")
    ax_rate.set_title("One-hour 53 ms comparison: stable delivery and cumulative integrity failures")
    ax_rate.legend(frameon=False, ncol=2)
    style_axis(ax_rate)

    ax_reject.plot(minutes_before, before_reject, color=RED, linewidth=2.0, label="before: 100 rejects")
    ax_reject.plot(minutes_after, after_reject, color=GREEN, linewidth=2.0, label="guarded: 0 rejects")
    ax_reject.set_xlabel("elapsed time [min]")
    ax_reject.set_ylabel("cumulative source rejects")
    ax_reject.set_ylim(-2, 105)
    ax_reject.legend(frameon=False, ncol=2, loc="upper left")
    style_axis(ax_reject)
    save(fig, output_dir, "04_before_after_timeline")


def plot_anchor_distribution(rows: list[dict], output_dir: Path) -> None:
    pre_guard = [row for row in rows if not row["guarded"]]
    labels = [row["label"] for row in pre_guard]
    anchors = (2, 3, 4, 5)
    x = np.arange(len(labels))
    width = 0.19
    colors = (BLUE, GREEN, AMBER, RED)

    fig, ax = plt.subplots(figsize=(9.2, 4.7))
    for offset, (anchor, color) in enumerate(zip(anchors, colors)):
        ax.bar(
            x + (offset - 1.5) * width,
            [row["anchors"].get(str(anchor), 0) for row in pre_guard],
            width,
            color=color,
            alpha=0.84,
            label=f"A{anchor}",
        )
    ax.set_ylabel("source rejects")
    ax.set_title("Rotating the first anchor moves the fault across all anchors")
    ax.set_xticks(x, labels)
    ax.legend(frameon=False, ncol=4)
    style_axis(ax)
    save(fig, output_dir, "05_anchor_distribution")


def plot_schedule(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(10.0, 3.9))
    y_before, y_after = 1.0, 0.0

    before_segments = [
        (0, 13, BLUE, "tag FINAL / round end"),
        (13, 1, RED, "SURVEY CMD"),
        (14, 13, AMBER, "geometry DS-TWR"),
        (27, 1, LIGHT, "gap"),
        (28, 13, BLUE, "next POLL / tag slot"),
    ]
    after_segments = [
        (0, 13, BLUE, "tag FINAL / round end"),
        (13, 1, LIGHT, "gap"),
        (14, 5, GREEN, "5 ms guard"),
        (19, 1, RED, "SURVEY CMD"),
        (20, 13, AMBER, "geometry DS-TWR"),
        (33, 5, GREEN, "5 ms guard"),
        (38, 13, BLUE, "next POLL / tag slot"),
    ]
    for y, segments in ((y_before, before_segments), (y_after, after_segments)):
        for start, duration, color, label in segments:
            ax.broken_barh([(start, duration)], (y - 0.28, 0.56), facecolors=color, alpha=0.86)
            if duration >= 5:
                ax.text(start + duration / 2, y, label, ha="center", va="center", fontsize=8)
    ax.text(13.5, y_before + 0.38, "no isolation", color=RED, ha="center", fontsize=9, fontweight="bold")
    ax.set_yticks([y_before, y_after], ["before guard", "guarded"])
    ax.set_xlabel("conceptual time around the periodic geometry boundary [ms]")
    ax.set_title("The fix isolates both sides of the separately inserted geometry exchange")
    ax.set_xlim(0, 52)
    ax.set_ylim(-0.6, 1.6)
    ax.grid(True, axis="x", color=GRID, alpha=0.5, linewidth=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    save(fig, output_dir, "06_schedule_guard")


def main() -> int:
    args = parse_args()
    report_dir = args.report_dir.resolve()
    output_dir = report_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_summaries(report_dir)
    (report_dir / "report_summary.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    plot_throughput(rows, output_dir)
    plot_boundary_rejects(rows, output_dir)
    plot_failure_rates(rows, output_dir)
    plot_before_after_timeline(rows, report_dir, output_dir)
    plot_anchor_distribution(rows, output_dir)
    plot_schedule(output_dir)
    print(f"Generated report data and figures in {report_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
