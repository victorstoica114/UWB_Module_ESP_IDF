#!/usr/bin/env python3
"""Analyze the native DS-TWR speed-limit experiment.

The established 64 ms field captures are used as an immutable baseline. New
blocks are discovered from the dedicated study directory and grouped by their
runtime timing profile.
"""

from __future__ import annotations

import csv
import json
import math
import pathlib
import re
import statistics
from collections import Counter, defaultdict
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uwb_compare_analyze import (
    ANCHOR_IDS,
    ANCHOR_INDEX,
    SURVEYED_ANCHORS,
    SURVEYED_TAG,
    TRUE_RANGE_M,
    position_metrics,
    trilaterate_linear,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]
BASELINE_DIR = (
    ROOT / "reports" / "uwb_protocol_comparison_20260726" / "data"
)
STUDY_DIR = ROOT / "reports" / "ds_twr_speed_limit_20260726"
DATA_DIR = STUDY_DIR / "data"
FIGURE_DIR = STUDY_DIR / "figures"

BASELINE_BLOCKS = ("ds_1", "ds_2", "ds_3")
TIMING_PATTERN = re.compile(
    r"ok=(?P<ok>\d+) fail=(?P<fail>\d+) "
    r"rate=(?P<rate>[0-9.]+)/s overrun=(?P<overrun>\d+) "
    r"max=(?P<max_us>\d+) us"
)


def read_jsonl(path: pathlib.Path) -> Iterable[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def write_csv(path: pathlib.Path, rows: list[dict[str, Any]]) -> None:
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def runtime_profile(metadata: dict[str, Any]) -> dict[str, int]:
    # Collection starts only after the configured warmup. A module may still
    # expose one stale pre-reboot status in the snapshot taken before that
    # warmup, so use the post-collection snapshot as the authoritative
    # profile and still require every module to agree there.
    statuses = metadata.get("final_status") or metadata.get(
        "initial_status", []
    )
    if not statuses:
        raise ValueError(f"{metadata.get('block')}: missing initial status")
    first = statuses[0]
    fields = {
        "slot_ms": "runtime_ranging_slot_ms",
        "gap_ms": "runtime_ranging_round_gap_ms",
        "timeout_ms": "runtime_ranging_rx_timeout_ms",
        "resp_ms": "runtime_ranging_resp_delay_ms",
        "final_ms": "runtime_ranging_final_delay_ms",
        "auto_rx_uus": "runtime_ranging_auto_rx_delay_uus",
    }
    profile = {
        output: int(first[source])
        for output, source in fields.items()
    }
    for status in statuses[1:]:
        for output, source in fields.items():
            if int(status[source]) != profile[output]:
                raise ValueError(
                    f"{metadata.get('block')}: inconsistent {source}"
                )
    return profile


def profile_id(profile: dict[str, int]) -> str:
    return (
        f"s{profile['slot_ms']:02d}_g{profile['gap_ms']:02d}_"
        f"t{profile['timeout_ms']:02d}_r{profile['resp_ms']:02d}_"
        f"f{profile['final_ms']:02d}"
    )


def frame_positions(
    ranges: list[dict[str, Any]],
    block: str,
) -> tuple[list[dict[str, Any]], int]:
    groups: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
    for event in ranges:
        anchor_id = int(event["anchor_id"])
        frame_key = (
            int(event["seq"]) - ANCHOR_INDEX[anchor_id]
        ) % 65536
        groups[frame_key][anchor_id] = event

    positions: list[dict[str, Any]] = []
    for frame_key, items in groups.items():
        if set(items) != set(ANCHOR_IDS):
            continue
        distances = {
            anchor_id: float(items[anchor_id]["distance_m"])
            for anchor_id in ANCHOR_IDS
        }
        solution = trilaterate_linear(SURVEYED_ANCHORS, distances)
        if solution is None:
            continue
        positions.append(
            {
                "block": block,
                "frame_key": frame_key,
                "received_at": statistics.fmean(
                    float(items[anchor_id]["received_at"])
                    for anchor_id in ANCHOR_IDS
                ),
                "x_m": solution[0],
                "y_m": solution[1],
                "error_m": math.hypot(
                    solution[0] - SURVEYED_TAG[0],
                    solution[1] - SURVEYED_TAG[1],
                ),
            }
        )
    return positions, len(groups)


def timing_metrics(path: pathlib.Path | None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "timing_windows": 0,
        "timing_attempts": 0,
        "timing_ok": 0,
        "timing_fail": 0,
        "timing_success_pct": math.nan,
        "timing_overruns": 0,
        "timing_max_us": math.nan,
        "timing_healthy_window_max_us": math.nan,
        "timing_healthy_window_p50_us": math.nan,
        "timing_healthy_window_p95_us": math.nan,
        "timing_healthy_window_p99_us": math.nan,
        "resp_timeout_logs": 0,
        "resp_tx_failure_logs": 0,
        "final_failure_logs": 0,
        "timestamp_mismatch_logs": 0,
    }
    if path is None or not path.exists():
        return result

    maxima: list[int] = []
    healthy_maxima: list[int] = []
    counts = Counter()
    for item in read_jsonl(path):
        message = str(item.get("message") or "")
        match = TIMING_PATTERN.search(message)
        if match:
            values = {key: int(value) if key != "rate" else float(value)
                      for key, value in match.groupdict().items()}
            result["timing_windows"] += 1
            result["timing_ok"] += values["ok"]
            result["timing_fail"] += values["fail"]
            result["timing_overruns"] += values["overrun"]
            maxima.append(values["max_us"])
            if values["fail"] == 0:
                healthy_maxima.append(values["max_us"])
        if "RESP wait failed" in message:
            counts["resp_timeout_logs"] += 1
        if "RESP TX failed" in message:
            counts["resp_tx_failure_logs"] += 1
        if "FINAL" in message and "failed" in message:
            counts["final_failure_logs"] += 1
        if "timestamp mismatch" in message:
            counts["timestamp_mismatch_logs"] += 1

    attempts = int(result["timing_ok"]) + int(result["timing_fail"])
    result["timing_attempts"] = attempts
    if attempts:
        result["timing_success_pct"] = (
            100.0 * float(result["timing_ok"]) / attempts
        )
    if maxima:
        result["timing_max_us"] = max(maxima)
    if healthy_maxima:
        result["timing_healthy_window_max_us"] = max(healthy_maxima)
        result["timing_healthy_window_p50_us"] = float(
            np.percentile(healthy_maxima, 50)
        )
        result["timing_healthy_window_p95_us"] = float(
            np.percentile(healthy_maxima, 95)
        )
        result["timing_healthy_window_p99_us"] = float(
            np.percentile(healthy_maxima, 99)
        )
    result.update(counts)
    return result


def analyze_block(
    events_path: pathlib.Path,
    metadata_path: pathlib.Path,
    *,
    baseline: bool,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    profile = runtime_profile(metadata)
    block = str(metadata["block"])
    ranges = [
        item
        for item in read_jsonl(events_path)
        if item.get("kind") == "ds_range"
    ]
    positions, candidate_frames = frame_positions(ranges, block)
    duration = float(metadata["duration_sec"])
    metrics = position_metrics(positions, SURVEYED_TAG)
    phase = (
        "baseline"
        if baseline
        else "validation"
        if "validation" in block
        else "screening"
    )
    row: dict[str, Any] = {
        "block": block,
        "baseline": int(baseline),
        "phase": phase,
        "comparison_eligible": int(
            baseline or metadata.get("comparison_eligible", True)
        ),
        "study_note": str(metadata.get("study_note", "")),
        "profile": profile_id(profile),
        **profile,
        "frame_ms": 4 * profile["slot_ms"] + profile["gap_ms"],
        "theoretical_position_hz": (
            1000.0 / (4 * profile["slot_ms"] + profile["gap_ms"])
        ),
        "duration_s": duration,
        "ranges": len(ranges),
        "range_rate_hz": len(ranges) / duration,
        "candidate_frames": candidate_frames,
        "complete_frames": len(positions),
        "complete_frame_pct": (
            100.0 * len(positions) / candidate_frames
            if candidate_frames
            else math.nan
        ),
        "position_rate_hz": len(positions) / duration,
        **{
            f"pos_{key}": value
            for key, value in metrics.items()
        },
    }
    for position in positions:
        position["profile"] = row["profile"]
        position["phase"] = phase
        position["comparison_eligible"] = row["comparison_eligible"]
    for anchor_id in ANCHOR_IDS:
        errors = np.asarray(
            [
                float(item["distance_m"]) - TRUE_RANGE_M
                for item in ranges
                if int(item["anchor_id"]) == anchor_id
            ],
            dtype=float,
        )
        row[f"a{anchor_id}_n"] = int(errors.size)
        row[f"a{anchor_id}_bias_cm"] = (
            float(np.mean(errors) * 100.0) if errors.size else math.nan
        )
        row[f"a{anchor_id}_std_cm"] = (
            float(np.std(errors, ddof=1) * 100.0)
            if errors.size > 1
            else math.nan
        )

    timing_path = events_path.with_name(
        events_path.stem + ".timing.jsonl"
    )
    row.update(timing_metrics(timing_path if not baseline else None))
    return row, positions


def mean_or_nan(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.fmean(finite) if finite else math.nan


def aggregate_profiles(
    block_rows: list[dict[str, Any]],
    all_positions: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in block_rows:
        if not int(row.get("comparison_eligible", 1)):
            continue
        grouped[str(row["profile"])].append(row)

    result: list[dict[str, Any]] = []
    metric_keys = (
        "range_rate_hz",
        "complete_frame_pct",
        "position_rate_hz",
        "pos_rmse_2d_cm",
        "pos_median_error_cm",
        "pos_p95_error_cm",
        "pos_p99_error_cm",
        "pos_max_error_cm",
        "timing_success_pct",
        "timing_overruns",
        "timing_max_us",
        "timing_healthy_window_max_us",
        "timing_healthy_window_p50_us",
        "timing_healthy_window_p95_us",
        "timing_healthy_window_p99_us",
        "a2_bias_cm",
        "a3_bias_cm",
        "a4_bias_cm",
        "a5_bias_cm",
        "a2_std_cm",
        "a3_std_cm",
        "a4_std_cm",
        "a5_std_cm",
    )
    for name, rows in grouped.items():
        validation_rows = [
            row for row in rows if row.get("phase") == "validation"
        ]
        selected = validation_rows or rows
        first = selected[0]
        selected_blocks = {str(row["block"]) for row in selected}
        selected_positions = [
            item
            for item in all_positions
            if str(item.get("block")) in selected_blocks
        ]
        pooled_position = position_metrics(
            selected_positions,
            SURVEYED_TAG,
        )
        duration_s = sum(float(row["duration_s"]) for row in selected)
        ranges = sum(int(row["ranges"]) for row in selected)
        candidate_frames = sum(
            int(row["candidate_frames"]) for row in selected
        )
        complete_frames = sum(
            int(row["complete_frames"]) for row in selected
        )
        timing_attempts = sum(
            int(row["timing_attempts"]) for row in selected
        )
        timing_ok = sum(int(row["timing_ok"]) for row in selected)
        timing_fail = sum(int(row["timing_fail"]) for row in selected)
        timing_overruns = sum(
            int(row["timing_overruns"]) for row in selected
        )
        aggregate: dict[str, Any] = {
            "profile": name,
            "baseline": max(int(row["baseline"]) for row in selected),
            "aggregation_phase": (
                "validation" if validation_rows else first["phase"]
            ),
            "blocks": len(selected),
            "screening_blocks": sum(
                row.get("phase") == "screening" for row in rows
            ),
            "slot_ms": first["slot_ms"],
            "gap_ms": first["gap_ms"],
            "timeout_ms": first["timeout_ms"],
            "resp_ms": first["resp_ms"],
            "final_ms": first["final_ms"],
            "frame_ms": first["frame_ms"],
            "theoretical_position_hz": first["theoretical_position_hz"],
            "duration_s": duration_s,
            "ranges": ranges,
            "candidate_frames": candidate_frames,
            "complete_frames": complete_frames,
            "timing_attempts": timing_attempts,
            "timing_ok": timing_ok,
            "timing_fail": timing_fail,
            "range_rate_hz": ranges / duration_s,
            "complete_frame_pct": (
                100.0 * complete_frames / candidate_frames
                if candidate_frames
                else math.nan
            ),
            "position_rate_hz": complete_frames / duration_s,
            "timing_success_pct": (
                100.0 * timing_ok / timing_attempts
                if timing_attempts
                else math.nan
            ),
            "timing_failure_per_1000": (
                1000.0 * timing_fail / timing_attempts
                if timing_attempts
                else math.nan
            ),
            "timing_overruns": timing_overruns,
            "timing_overruns_per_min": (
                60.0 * timing_overruns / duration_s
            ),
            **{
                f"pos_{key}": value
                for key, value in pooled_position.items()
            },
            "resp_timeout_logs": sum(
                int(row["resp_timeout_logs"]) for row in selected
            ),
            "resp_tx_failure_logs": sum(
                int(row["resp_tx_failure_logs"]) for row in selected
            ),
            "final_failure_logs": sum(
                int(row["final_failure_logs"]) for row in selected
            ),
        }
        for key in metric_keys:
            if key in aggregate:
                continue
            if key in {"timing_max_us", "timing_healthy_window_max_us"}:
                finite = [
                    float(row[key])
                    for row in selected
                    if math.isfinite(float(row[key]))
                ]
                aggregate[key] = max(finite) if finite else math.nan
                continue
            if key.startswith("a") and key.endswith("_bias_cm"):
                anchor = key.split("_", 1)[0]
                total_n = sum(int(row[f"{anchor}_n"]) for row in selected)
                aggregate[key] = (
                    sum(
                        int(row[f"{anchor}_n"]) * float(row[key])
                        for row in selected
                    )
                    / total_n
                    if total_n
                    else math.nan
                )
                continue
            aggregate[key] = mean_or_nan(
                [float(row[key]) for row in selected]
            )
        result.append(aggregate)
    return sorted(result, key=lambda row: int(row["frame_ms"]), reverse=True)


def save_figures(
    profile_rows: list[dict[str, Any]],
    block_rows: list[dict[str, Any]],
    all_positions: list[dict[str, Any]],
) -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    labels = [f"{row['frame_ms']} ms" for row in profile_rows]
    colors = [
        "#2563eb" if row["baseline"] else "#ea580c"
        for row in profile_rows
    ]

    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8))
    axes[0].bar(
        labels,
        [float(row["position_rate_hz"]) for row in profile_rows],
        color=colors,
        alpha=0.8,
    )
    axes[0].plot(
        labels,
        [float(row["theoretical_position_hz"]) for row in profile_rows],
        "o--",
        color="#0f172a",
        label="theoretical",
    )
    axes[0].set_ylabel("complete positions/s")
    axes[0].set_title("Effective position rate")
    axes[0].legend()
    axes[1].bar(
        labels,
        [float(row["complete_frame_pct"]) for row in profile_rows],
        color=colors,
        alpha=0.8,
    )
    axes[1].set_ylim(0, 102)
    axes[1].set_ylabel("complete frames [%]")
    axes[1].set_title("Frame completeness")
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "01_rate_completeness.pdf")
    figure.savefig(FIGURE_DIR / "01_rate_completeness.png", dpi=220)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.8, 4.0))
    x = np.arange(len(labels))
    width = 0.25
    axis.bar(
        x - width,
        [float(row["pos_rmse_2d_cm"]) for row in profile_rows],
        width,
        label="RMSE",
        color="#475569",
    )
    axis.bar(
        x,
        [float(row["pos_median_error_cm"]) for row in profile_rows],
        width,
        label="Median",
        color="#64748b",
    )
    axis.bar(
        x + width,
        [float(row["pos_p95_error_cm"]) for row in profile_rows],
        width,
        label="P95",
        color="#94a3b8",
    )
    axis.set_xticks(x, labels)
    axis.set_ylabel("position error [cm]")
    axis.set_title("Accuracy versus DS-TWR frame period")
    axis.legend()
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "02_accuracy.pdf")
    figure.savefig(FIGURE_DIR / "02_accuracy.png", dpi=220)
    plt.close(figure)

    timed_rows = [
        row
        for row in profile_rows
        if math.isfinite(float(row["timing_success_pct"]))
    ]
    timed_labels = [f"{row['frame_ms']} ms" for row in timed_rows]
    figure, axis = plt.subplots(figsize=(8.2, 4.2))
    x = np.arange(len(timed_rows))
    failures = [
        float(row["timing_failure_per_1000"]) for row in timed_rows
    ]
    overruns = [
        float(row["timing_overruns_per_min"]) for row in timed_rows
    ]
    bars = axis.bar(
        x,
        failures,
        color="#dc2626",
        alpha=0.78,
        label="failed exchanges / 1,000",
    )
    axis.set_xticks(x, timed_labels)
    axis.set_ylabel("failed exchanges / 1,000")
    axis.set_title("Reliability cost near the scheduling limit")
    second = axis.twinx()
    second.plot(
        x,
        overruns,
        "o-",
        color="#7c3aed",
        linewidth=2,
        label="slot overruns / min",
    )
    second.set_ylabel("slot overruns / min")
    handles = [bars, second.lines[0]]
    axis.legend(
        handles,
        ["failed exchanges / 1,000", "slot overruns / min"],
        loc="upper left",
    )
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "03_reliability.pdf")
    figure.savefig(FIGURE_DIR / "03_reliability.png", dpi=220)
    plt.close(figure)

    selected_series = [
        ("64 ms baseline", 64, "#2563eb"),
        ("17 ms balanced", 17, "#16a34a"),
        ("13 ms maximum", 13, "#dc2626"),
    ]
    profile_by_frame = {
        int(row["frame_ms"]): row for row in profile_rows
    }
    figure, axis = plt.subplots(figsize=(7.8, 4.3))
    for label, frame_ms, color in selected_series:
        profile = profile_by_frame[frame_ms]
        phase = str(profile["aggregation_phase"])
        errors_cm = sorted(
            100.0 * float(item["error_m"])
            for item in all_positions
            if int(item.get("comparison_eligible", 1))
            and str(item.get("profile")) == str(profile["profile"])
            and str(item.get("phase")) == phase
        )
        if not errors_cm:
            continue
        cdf = np.arange(1, len(errors_cm) + 1) / len(errors_cm)
        axis.plot(
            errors_cm,
            cdf,
            linewidth=2,
            color=color,
            label=f"{label} (n={len(errors_cm):,})",
        )
    axis.set_xlim(left=0)
    axis.set_ylim(0, 1.005)
    axis.set_xlabel("2D position error [cm]")
    axis.set_ylabel("empirical CDF")
    axis.set_title("Unfiltered static position-error distribution")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "04_position_error_cdf.pdf")
    figure.savefig(FIGURE_DIR / "04_position_error_cdf.png", dpi=220)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.4, 4.3))
    frames = [int(row["frame_ms"]) for row in profile_rows]
    for anchor_id, color in zip(
        ANCHOR_IDS,
        ("#2563eb", "#ea580c", "#16a34a", "#7c3aed"),
    ):
        axis.plot(
            frames,
            [float(row[f"a{anchor_id}_bias_cm"]) for row in profile_rows],
            "o-",
            linewidth=1.8,
            color=color,
            label=f"A{anchor_id}",
        )
    axis.axhline(0.0, color="#0f172a", linewidth=1)
    axis.invert_xaxis()
    axis.set_xlabel("frame period [ms] (faster to the right)")
    axis.set_ylabel("mean range bias [cm]")
    axis.set_title("Per-anchor range bias remains profile-independent")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "05_anchor_bias.pdf")
    figure.savefig(FIGURE_DIR / "05_anchor_bias.png", dpi=220)
    plt.close(figure)

    validation_rows = sorted(
        [
            row
            for row in block_rows
            if row.get("phase") == "validation"
            and int(row.get("comparison_eligible", 1))
        ],
        key=lambda row: (str(row["block"])[-1], int(row["frame_ms"])),
    )
    figure, axes = plt.subplots(1, 2, figsize=(9.2, 4.0))
    validation_labels = [
        f"{row['frame_ms']} ms V{str(row['block'])[-1]}"
        for row in validation_rows
    ]
    validation_colors = [
        "#16a34a" if int(row["frame_ms"]) == 17 else "#dc2626"
        for row in validation_rows
    ]
    axes[0].bar(
        validation_labels,
        [float(row["position_rate_hz"]) for row in validation_rows],
        color=validation_colors,
        alpha=0.8,
    )
    axes[0].set_ylabel("complete positions/s")
    axes[0].set_title("Throughput repeatability")
    axes[0].tick_params(axis="x", rotation=25)
    axes[1].bar(
        validation_labels,
        [float(row["pos_rmse_2d_cm"]) for row in validation_rows],
        color=validation_colors,
        alpha=0.8,
        label="RMSE",
    )
    axes[1].plot(
        validation_labels,
        [float(row["pos_p95_error_cm"]) for row in validation_rows],
        "ko--",
        label="P95",
    )
    axes[1].set_ylabel("2D error [cm]")
    axes[1].set_title("Accuracy repeatability")
    axes[1].tick_params(axis="x", rotation=25)
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "06_validation_repeatability.pdf")
    figure.savefig(FIGURE_DIR / "06_validation_repeatability.png", dpi=220)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.2, 4.2))
    p50_ms = [
        float(row["timing_healthy_window_p50_us"]) / 1000.0
        for row in timed_rows
    ]
    p95_ms = [
        float(row["timing_healthy_window_p95_us"]) / 1000.0
        for row in timed_rows
    ]
    p99_ms = [
        float(row["timing_healthy_window_p99_us"]) / 1000.0
        for row in timed_rows
    ]
    slots_ms = [float(row["slot_ms"]) for row in timed_rows]
    x = np.arange(len(timed_rows))
    width = 0.34
    axis.bar(
        x - width / 2,
        p50_ms,
        width,
        color="#94a3b8",
        alpha=0.78,
        label="median 1 s maximum",
    )
    axis.bar(
        x + width / 2,
        p95_ms,
        width,
        color="#475569",
        alpha=0.82,
        label="P95 of 1 s maxima",
    )
    axis.plot(
        x,
        p99_ms,
        "d:",
        color="#dc2626",
        linewidth=1.5,
        label="P99 of 1 s maxima",
    )
    axis.plot(
        x,
        slots_ms,
        "o--",
        color="#0f172a",
        linewidth=1.8,
        label="configured slot",
    )
    axis.set_xticks(x, timed_labels)
    axis.set_ylabel("time [ms]")
    axis.set_title("Observed exchange envelope versus slot budget")
    axis.legend()
    figure.tight_layout()
    figure.savefig(FIGURE_DIR / "07_timing_margin.pdf")
    figure.savefig(FIGURE_DIR / "07_timing_margin.png", dpi=220)
    plt.close(figure)


def main() -> int:
    STUDY_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    block_rows: list[dict[str, Any]] = []
    all_positions: list[dict[str, Any]] = []

    for block in BASELINE_BLOCKS:
        row, positions = analyze_block(
            BASELINE_DIR / f"{block}.jsonl",
            BASELINE_DIR / f"{block}.metadata.json",
            baseline=True,
        )
        block_rows.append(row)
        all_positions.extend(positions)

    for metadata_path in sorted(DATA_DIR.glob("*.metadata.json")):
        block = metadata_path.name.removesuffix(".metadata.json")
        events_path = DATA_DIR / f"{block}.jsonl"
        if not events_path.exists():
            continue
        row, positions = analyze_block(
            events_path,
            metadata_path,
            baseline=False,
        )
        block_rows.append(row)
        all_positions.extend(positions)

    profile_rows = aggregate_profiles(block_rows, all_positions)
    write_csv(STUDY_DIR / "block_metrics.csv", block_rows)
    write_csv(STUDY_DIR / "profile_metrics.csv", profile_rows)
    write_csv(STUDY_DIR / "positions.csv", all_positions)
    (STUDY_DIR / "analysis_summary.json").write_text(
        json.dumps(
            {
                "baseline_source": str(BASELINE_DIR.relative_to(ROOT)),
                "block_metrics": block_rows,
                "profile_metrics": profile_rows,
            },
            indent=2,
            sort_keys=True,
            allow_nan=True,
        )
        + "\n",
        encoding="utf-8",
    )
    save_figures(profile_rows, block_rows, all_positions)

    print(
        "profile frame  pos/s complete%  RMSEcm  P95cm success% overrun"
    )
    for row in profile_rows:
        print(
            f"{row['profile']:20s} {row['frame_ms']:5.0f} "
            f"{row['position_rate_hz']:6.2f} "
            f"{row['complete_frame_pct']:9.2f} "
            f"{row['pos_rmse_2d_cm']:7.3f} "
            f"{row['pos_p95_error_cm']:7.3f} "
            f"{row['timing_success_pct']:8.3f} "
            f"{row['timing_overruns']:7.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
