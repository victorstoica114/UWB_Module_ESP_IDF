#!/usr/bin/env python3
"""Analyze the Passive DS-TWR known-geometry calibration experiment."""

from __future__ import annotations

import csv
import hashlib
import json
import lzma
import math
import pathlib
import re
import statistics
from collections import defaultdict
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from uwb_compare_analyze import position_metrics, solve_tdoa


ROOT = pathlib.Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports" / "passive_ds_known_geometry_20260727"
DATA_DIR = REPORT_DIR / "data"
FIGURE_DIR = REPORT_DIR / "figures"
BLOCKS = (
    ("passive_ds_known_geometry_1", "Uncalibrated"),
    ("passive_ds_calibrated_1", "Calibrated"),
)
ANCHOR_IDS = (2, 3, 4, 5)
ANCHORS = {
    2: (0.0, 0.0),
    3: (0.0, 3.0),
    4: (3.0, 0.0),
    5: (3.0, 3.0),
}
TAG = (1.5, 1.5)
PAIR_ORDER = tuple(
    (first, second)
    for index, first in enumerate(ANCHOR_IDS)
    for second in ANCHOR_IDS[index + 1 :]
)
TIMING_RE = re.compile(
    r"PASSIVE_DS anchor .*? ok=(?P<ok>\d+) fail=(?P<fail>\d+)"
)
COLORS = {
    "Uncalibrated": "#dc2626",
    "Calibrated": "#2563eb",
    "Calibrated, surveyed geometry": "#16a34a",
}


def data_path(block: str, suffix: str = ".jsonl") -> pathlib.Path:
    plain = DATA_DIR / f"{block}{suffix}"
    compressed = plain.with_suffix(plain.suffix + ".xz")
    if plain.exists():
        return plain
    if compressed.exists():
        return compressed
    raise FileNotFoundError(plain)


def read_jsonl(path: pathlib.Path) -> Iterable[dict[str, Any]]:
    opener = lzma.open if path.suffix == ".xz" else open
    with opener(path, "rt", encoding="utf-8") as handle:
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


def save_figure(figure: plt.Figure, name: str) -> None:
    figure.tight_layout()
    for suffix in ("png", "pdf"):
        figure.savefig(
            FIGURE_DIR / f"{name}.{suffix}",
            dpi=220 if suffix == "png" else None,
            bbox_inches="tight",
        )
    plt.close(figure)


def expected_difference(initiator: int, responder: int) -> float:
    return (
        math.dist(TAG, ANCHORS[responder])
        - math.dist(TAG, ANCHORS[initiator])
    )


def metric_row(
    label: str,
    source: str,
    positions: list[dict[str, Any]],
    duration_sec: float,
) -> dict[str, Any]:
    metrics = position_metrics(positions, TAG)
    errors = [
        math.hypot(float(item["x_m"]) - TAG[0], float(item["y_m"]) - TAG[1])
        for item in positions
    ]
    return {
        "capture": label,
        "position_source": source,
        "duration_sec": duration_sec,
        "position_count": len(positions),
        "captured_position_rate_hz": len(positions) / duration_sec,
        "outliers_gt_10cm": sum(value > 0.10 for value in errors),
        "outliers_gt_15cm": sum(value > 0.15 for value in errors),
        **metrics,
    }


def offline_frame_positions(
    observations: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Solve once per 3-slot star frame using the latest directed paths.

    Robust Rotating emits three responder observations per frame. Keeping the
    latest observation for every directed path matches the embedded solver's
    diversity window while replacing only its autonomous geometry with the
    surveyed square. The final incomplete capture frame is intentionally not
    solved.
    """
    latest: dict[tuple[int, int], dict[str, Any]] = {}
    positions: list[dict[str, Any]] = []
    current_frame: int | None = None

    for event in observations:
        frame = int(event["slot_id"]) // 3
        if current_frame is not None and frame != current_frame:
            solution = solve_tdoa(ANCHORS, list(latest.values()), TAG)
            if solution is not None:
                positions.append(
                    {
                        "x_m": solution[0],
                        "y_m": solution[1],
                        "frame": current_frame,
                    }
                )
        current_frame = frame
        latest[(int(event["initiator_id"]), int(event["responder_id"]))] = event
    return positions


def observation_rows(
    label: str,
    observations: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in observations:
        initiator = int(item["initiator_id"])
        responder = int(item["responder_id"])
        grouped[(initiator, responder)].append(
            float(item["diff_m"])
            - expected_difference(initiator, responder)
        )

    rows = []
    for pair in sorted(grouped):
        values = np.asarray(grouped[pair], dtype=float)
        rows.append(
            {
                "capture": label,
                "initiator_id": pair[0],
                "responder_id": pair[1],
                "n": int(values.size),
                "mean_error_cm": float(np.mean(values) * 100.0),
                "std_cm": float(np.std(values, ddof=1) * 100.0),
                "p95_abs_error_cm": float(
                    np.percentile(np.abs(values), 95) * 100.0
                ),
                "rmse_cm": float(np.sqrt(np.mean(values**2)) * 100.0),
            }
        )
    return rows


def anchor_range_rows(
    label: str,
    ranges: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in ranges:
        pair = tuple(
            sorted((int(item["anchor_a_id"]), int(item["anchor_b_id"])))
        )
        if pair not in PAIR_ORDER:
            continue
        grouped[pair].append(
            float(item["distance_m"])
            - math.dist(ANCHORS[pair[0]], ANCHORS[pair[1]])
        )

    rows = []
    for pair in PAIR_ORDER:
        values = np.asarray(grouped[pair], dtype=float)
        rows.append(
            {
                "capture": label,
                "anchor_a_id": pair[0],
                "anchor_b_id": pair[1],
                "n": int(values.size),
                "mean_error_cm": float(np.mean(values) * 100.0),
                "std_cm": float(np.std(values, ddof=1) * 100.0),
                "p95_abs_error_cm": float(
                    np.percentile(np.abs(values), 95) * 100.0
                ),
            }
        )
    return rows


def effective_anchor_range_rows(
    label: str,
    observations: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Summarize the range actually used in each passive observation."""
    grouped: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in observations:
        pair = tuple(
            sorted((int(item["initiator_id"]), int(item["responder_id"])))
        )
        if pair not in PAIR_ORDER:
            continue
        grouped[pair].append(
            float(item["anchor_distance_m"])
            - math.dist(ANCHORS[pair[0]], ANCHORS[pair[1]])
        )

    rows = []
    for pair in PAIR_ORDER:
        values = np.asarray(grouped[pair], dtype=float)
        rows.append(
            {
                "capture": label,
                "anchor_a_id": pair[0],
                "anchor_b_id": pair[1],
                "n": int(values.size),
                "mean_error_cm": float(np.mean(values) * 100.0),
                "std_cm": float(np.std(values, ddof=1) * 100.0),
                "p95_abs_error_cm": float(
                    np.percentile(np.abs(values), 95) * 100.0
                ),
            }
        )
    return rows


def fit_anchor_biases(rows: list[dict[str, Any]]) -> dict[int, float]:
    """Fit error(I,R) = bias(R) - bias(I), fixing A2 at zero."""
    matrix = []
    values = []
    variable_ids = ANCHOR_IDS[1:]
    for row in rows:
        vector = [0.0] * len(variable_ids)
        initiator = int(row["initiator_id"])
        responder = int(row["responder_id"])
        if responder != ANCHOR_IDS[0]:
            vector[variable_ids.index(responder)] += 1.0
        if initiator != ANCHOR_IDS[0]:
            vector[variable_ids.index(initiator)] -= 1.0
        matrix.append(vector)
        values.append(float(row["mean_error_cm"]) * 10.0)
    fitted, *_ = np.linalg.lstsq(
        np.asarray(matrix), np.asarray(values), rcond=None
    )
    return {
        ANCHOR_IDS[0]: 0.0,
        **{
            anchor_id: float(fitted[index])
            for index, anchor_id in enumerate(variable_ids)
        },
    }


def fitted_geometry(
    range_rows: list[dict[str, Any]],
) -> tuple[dict[int, tuple[float, float]], float]:
    measured = {
        (int(row["anchor_a_id"]), int(row["anchor_b_id"])): (
            math.dist(
                ANCHORS[int(row["anchor_a_id"])],
                ANCHORS[int(row["anchor_b_id"])],
            )
            + float(row["mean_error_cm"]) / 100.0
        )
        for row in range_rows
    }

    def coordinates(parameters: np.ndarray) -> dict[int, tuple[float, float]]:
        return {
            2: (0.0, 0.0),
            3: (0.0, float(parameters[0])),
            4: (float(parameters[1]), float(parameters[2])),
            5: (float(parameters[3]), float(parameters[4])),
        }

    def residuals(parameters: np.ndarray) -> np.ndarray:
        points = coordinates(parameters)
        return np.asarray(
            [
                math.dist(points[first], points[second])
                - measured[(first, second)]
                for first, second in PAIR_ORDER
            ]
        )

    result = least_squares(
        residuals,
        np.asarray([3.0, 3.0, 0.0, 3.0, 3.0]),
        method="trf",
    )
    points = coordinates(result.x)
    rms_cm = float(np.sqrt(np.mean(residuals(result.x) ** 2)) * 100.0)
    return points, rms_cm


def protocol_metrics(log_path: pathlib.Path) -> list[dict[str, Any]]:
    totals: dict[int, list[int]] = defaultdict(lambda: [0, 0, 0])
    for item in read_jsonl(log_path):
        match = TIMING_RE.search(str(item.get("message") or ""))
        if not match:
            continue
        module_id = int(item["module_id"])
        totals[module_id][0] += int(match.group("ok"))
        totals[module_id][1] += int(match.group("fail"))
        totals[module_id][2] += 1
    return [
        {
            "module_id": module_id,
            "ok": values[0],
            "fail": values[1],
            "windows": values[2],
            "success_pct": 100.0 * values[0] / (values[0] + values[1]),
        }
        for module_id, values in sorted(totals.items())
    ]


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "figure.facecolor": "white",
        }
    )

    captures: dict[str, dict[str, Any]] = {}
    position_rows: list[dict[str, Any]] = []
    all_observation_rows: list[dict[str, Any]] = []
    all_range_rows: list[dict[str, Any]] = []
    all_effective_range_rows: list[dict[str, Any]] = []
    protocol_rows: list[dict[str, Any]] = []
    manifest_rows: list[dict[str, Any]] = []

    for block, label in BLOCKS:
        events_path = data_path(block)
        metadata_path = DATA_DIR / f"{block}.metadata.json"
        logs_path = data_path(block, ".logs.jsonl")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        events = list(read_jsonl(events_path))
        observations = [
            item for item in events if item.get("kind") == "tdoa_observation"
        ]
        ranges = [
            item for item in events if item.get("kind") == "anchor_range"
        ]
        embedded = [
            item for item in events if item.get("kind") == "local_position"
        ]
        offline = offline_frame_positions(observations)
        duration = float(metadata["duration_sec"])
        captures[label] = {
            "block": block,
            "metadata": metadata,
            "observations": observations,
            "ranges": ranges,
            "embedded": embedded,
            "offline": offline,
        }
        position_rows.append(
            metric_row(label, "embedded autonomous geometry", embedded, duration)
        )
        position_rows.append(
            metric_row(label, "offline surveyed geometry", offline, duration)
        )
        all_observation_rows.extend(observation_rows(label, observations))
        all_range_rows.extend(anchor_range_rows(label, ranges))
        all_effective_range_rows.extend(
            effective_anchor_range_rows(label, observations)
        )
        for row in protocol_metrics(logs_path):
            protocol_rows.append({"capture": label, **row})
        for path in (events_path, logs_path, metadata_path):
            manifest_rows.append(
                {
                    "capture": label,
                    "file": str(path.relative_to(REPORT_DIR)),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )

    baseline_events = captures["Uncalibrated"]["observations"]
    baseline_midpoint = (
        float(baseline_events[0]["received_at"])
        + float(baseline_events[-1]["received_at"])
    ) / 2.0
    baseline_training_observations = [
        item
        for item in baseline_events
        if float(item["received_at"]) < baseline_midpoint
    ]
    baseline_observations = observation_rows(
        "Uncalibrated training half", baseline_training_observations
    )
    baseline_ranges = [
        row for row in all_range_rows if row["capture"] == "Uncalibrated"
    ]
    anchor_biases = fit_anchor_biases(baseline_observations)
    geometry, geometry_rms_cm = fitted_geometry(baseline_ranges)
    range_biases_mm = {
        f"A{row['anchor_a_id']}-A{row['anchor_b_id']}": int(
            round(float(row["mean_error_cm"]) * 10.0)
        )
        for row in baseline_ranges
    }

    write_csv(REPORT_DIR / "position_metrics.csv", position_rows)
    write_csv(REPORT_DIR / "observation_metrics.csv", all_observation_rows)
    write_csv(REPORT_DIR / "anchor_range_metrics.csv", all_range_rows)
    write_csv(
        REPORT_DIR / "effective_anchor_range_metrics.csv",
        all_effective_range_rows,
    )
    write_csv(REPORT_DIR / "protocol_metrics.csv", protocol_rows)
    write_csv(REPORT_DIR / "raw_data_manifest.csv", manifest_rows)

    embedded_before = next(
        row
        for row in position_rows
        if row["capture"] == "Uncalibrated"
        and row["position_source"] == "embedded autonomous geometry"
    )
    embedded_after = next(
        row
        for row in position_rows
        if row["capture"] == "Calibrated"
        and row["position_source"] == "embedded autonomous geometry"
    )
    surveyed_after = next(
        row
        for row in position_rows
        if row["capture"] == "Calibrated"
        and row["position_source"] == "offline surveyed geometry"
    )
    observation_bias_before = statistics.fmean(
        abs(float(row["mean_error_cm"]))
        for row in all_observation_rows
        if row["capture"] == "Uncalibrated"
    )
    observation_bias_after = statistics.fmean(
        abs(float(row["mean_error_cm"]))
        for row in all_observation_rows
        if row["capture"] == "Calibrated"
    )
    effective_range_bias_before = statistics.fmean(
        abs(float(row["mean_error_cm"]))
        for row in all_effective_range_rows
        if row["capture"] == "Uncalibrated"
    )
    effective_range_bias_after = statistics.fmean(
        abs(float(row["mean_error_cm"]))
        for row in all_effective_range_rows
        if row["capture"] == "Calibrated"
    )
    summary = {
        "experiment": "Passive DS-TWR known 3 m square, static center tag",
        "surveyed_anchors_m": {
            str(key): list(value) for key, value in ANCHORS.items()
        },
        "surveyed_tag_m": list(TAG),
        "position_metrics": position_rows,
        "protocol_metrics": protocol_rows,
        "derived_calibration": {
            "anchor_observation_bias_mm": {
                str(key): int(round(value))
                for key, value in anchor_biases.items()
            },
            "anchor_pair_range_bias_mm": range_biases_mm,
        },
        "uncalibrated_autonomous_geometry_fit": {
            "anchors_m": {
                str(key): list(value) for key, value in geometry.items()
            },
            "range_residual_rms_cm": geometry_rms_cm,
        },
        "key_results": {
            "embedded_rmse_improvement_pct": 100.0
            * (
                float(embedded_before["rmse_2d_cm"])
                - float(embedded_after["rmse_2d_cm"])
            )
            / float(embedded_before["rmse_2d_cm"]),
            "embedded_bias_norm_before_cm": math.hypot(
                float(embedded_before["bias_x_cm"]),
                float(embedded_before["bias_y_cm"]),
            ),
            "embedded_bias_norm_after_cm": math.hypot(
                float(embedded_after["bias_x_cm"]),
                float(embedded_after["bias_y_cm"]),
            ),
            "calibrated_surveyed_geometry_rmse_cm": surveyed_after[
                "rmse_2d_cm"
            ],
            "calibrated_surveyed_geometry_p95_cm": surveyed_after[
                "p95_error_cm"
            ],
            "directed_path_mean_abs_bias_before_cm":
                observation_bias_before,
            "directed_path_mean_abs_bias_after_cm":
                observation_bias_after,
            "effective_range_mean_abs_bias_before_cm":
                effective_range_bias_before,
            "effective_range_mean_abs_bias_after_cm":
                effective_range_bias_after,
        },
    }
    (REPORT_DIR / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.6), sharex=True, sharey=True)
    for axis, label in zip(axes, ("Uncalibrated", "Calibrated")):
        points = captures[label]["embedded"]
        sample = points[:: max(1, len(points) // 2500)]
        axis.scatter(
            [float(item["x_m"]) for item in sample],
            [float(item["y_m"]) for item in sample],
            s=5,
            alpha=0.3,
            color=COLORS[label],
            label="embedded position",
        )
        axis.scatter([TAG[0]], [TAG[1]], marker="*", s=130, color="black", label="truth")
        axis.set_title(label)
        axis.set_xlabel("x [m]")
        axis.set_aspect("equal", adjustable="box")
        axis.legend(loc="upper right")
    axes[0].set_ylabel("y [m]")
    figure.suptitle("Passive DS-TWR static position cloud")
    save_figure(figure, "position_scatter")

    figure, axis = plt.subplots(figsize=(7.2, 4.4))
    series = (
        ("Uncalibrated", captures["Uncalibrated"]["embedded"]),
        ("Calibrated", captures["Calibrated"]["embedded"]),
        (
            "Calibrated, surveyed geometry",
            captures["Calibrated"]["offline"],
        ),
    )
    for label, positions in series:
        errors = np.sort(
            [
                math.hypot(
                    float(item["x_m"]) - TAG[0],
                    float(item["y_m"]) - TAG[1],
                )
                * 100.0
                for item in positions
            ]
        )
        axis.plot(
            errors,
            np.arange(1, len(errors) + 1) / len(errors),
            label=label,
            color=COLORS[label],
        )
    axis.set_xlim(left=0)
    axis.set_ylim(0, 1.005)
    axis.set_xlabel("2-D position error [cm]")
    axis.set_ylabel("Empirical CDF")
    axis.set_title("Position error distribution")
    axis.legend()
    save_figure(figure, "position_error_cdf")

    figure, axes = plt.subplots(
        1, 2, figsize=(11.5, 4.5), sharex=True, sharey=True
    )
    labels = [
        f"{initiator}→{responder}"
        for initiator in ANCHOR_IDS
        for responder in ANCHOR_IDS
        if initiator != responder
    ]
    for axis, capture_label in zip(axes, ("Uncalibrated", "Calibrated")):
        values = []
        for initiator in ANCHOR_IDS:
            for responder in ANCHOR_IDS:
                if initiator == responder:
                    continue
                values.append(
                    [
                        (
                            float(item["diff_m"])
                            - expected_difference(initiator, responder)
                        )
                        * 100.0
                        for item in captures[capture_label]["observations"]
                        if int(item["initiator_id"]) == initiator
                        and int(item["responder_id"]) == responder
                    ]
                )
        box = axis.boxplot(
            values,
            tick_labels=labels,
            showfliers=False,
            patch_artist=True,
        )
        color = COLORS[capture_label]
        for patch in box["boxes"]:
            patch.set_facecolor(color)
            patch.set_alpha(0.28)
        axis.axhline(0, color="black", linewidth=0.9)
        axis.set_xlabel("directed anchor path")
        axis.set_title(
            "Before calibration"
            if capture_label == "Uncalibrated"
            else "After current calibration"
        )
        axis.tick_params(axis="x", labelrotation=45)
    axes[0].set_ylabel("directed observation error [cm]")
    figure.suptitle("Directed observation error: before vs after")
    save_figure(figure, "directed_observation_bias")

    figure, axis = plt.subplots(figsize=(7.5, 4.2))
    pair_labels = [f"{first}-{second}" for first, second in PAIR_ORDER]
    x = np.arange(len(PAIR_ORDER))
    width = 0.36
    for offset, capture_label in (
        (-width / 2, "Uncalibrated"),
        (width / 2, "Calibrated"),
    ):
        rows = [
            row
            for row in all_effective_range_rows
            if row["capture"] == capture_label
        ]
        axis.bar(
            x + offset,
            [float(row["mean_error_cm"]) for row in rows],
            width,
            yerr=[float(row["std_cm"]) for row in rows],
            capsize=3,
            color=COLORS[capture_label],
            alpha=0.72,
            label=(
                "Before calibration"
                if capture_label == "Uncalibrated"
                else "After calibration"
            ),
        )
    axis.axhline(0, color="black", linewidth=0.9)
    axis.set_xticks(x, pair_labels)
    axis.set_xlabel("anchor pair")
    axis.set_ylabel("effective range error at tag [cm]")
    axis.set_title("Effective anchor range: before vs after (mean ± 1σ)")
    axis.legend()
    save_figure(figure, "anchor_pair_range_bias")

    print(
        "Generated Passive DS-TWR known-geometry report data: "
        f"{embedded_before['rmse_2d_cm']:.3f} -> "
        f"{embedded_after['rmse_2d_cm']:.3f} cm embedded RMSE; "
        f"{surveyed_after['rmse_2d_cm']:.3f} cm with surveyed geometry."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
