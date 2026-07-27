#!/usr/bin/env python3
"""Analyze the 2026-07-27 Native/Passive DS-TWR A/B/A capture.

The analysis keeps accuracy (error relative to the surveyed center) separate
from precision (spread around the sample mean).  Passive positions are also
recomputed with the surveyed geometry so that protocol noise is not confused
with the continuously moving embedded anchor-geometry estimate.
"""

from __future__ import annotations

import csv
import json
import math
import pathlib
import statistics
from collections import defaultdict, deque
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from uwb_compare_analyze import (
    position_metrics,
    solve_tdoa,
    trilaterate_linear,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports" / "passive_vs_native_live_20260727"
DATA_DIR = REPORT_DIR / "data"
FIGURE_DIR = REPORT_DIR / "figures"

ANCHOR_IDS = (2, 3, 4, 5)
ANCHOR_INDEX = {anchor_id: index for index, anchor_id in enumerate(ANCHOR_IDS)}
ANCHORS = {
    2: (0.0, 0.0),
    3: (0.0, 3.0),
    4: (3.0, 0.0),
    5: (3.0, 3.0),
}
TAG = (1.5, 1.5)
TRUE_TAG_RANGE_M = math.sqrt(1.5**2 + 1.5**2)
PAIR_ORDER = tuple(
    (first, second)
    for index, first in enumerate(ANCHOR_IDS)
    for second in ANCHOR_IDS[index + 1 :]
)
CURRENT_ANCHOR_BIAS_M = {
    2: 0.0,
    3: 0.034,
    4: -0.016,
    5: -0.055,
}


def read_jsonl(path: pathlib.Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


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


def metric_row(
    label: str,
    source: str,
    rows: list[dict[str, Any]],
    duration_sec: float,
) -> dict[str, Any]:
    return {
        "capture": label,
        "source": source,
        "duration_sec": duration_sec,
        "position_rate_hz": len(rows) / duration_sec,
        **position_metrics(rows, TAG),
    }


def position_residuals(
    position: tuple[float, float],
    distances: dict[int, float],
) -> dict[int, float]:
    return {
        anchor_id: math.dist(position, ANCHORS[anchor_id]) - distance
        for anchor_id, distance in distances.items()
    }


def robust_trilaterate(
    distances: dict[int, float],
    seed: tuple[float, float] | None,
) -> tuple[float, float] | None:
    """Python port of the dashboard's robust 3-of-4 Native fit."""
    ids = sorted(
        anchor_id
        for anchor_id, distance in distances.items()
        if anchor_id in ANCHORS and math.isfinite(distance) and distance > 0
    )
    if len(ids) < 3:
        return None
    subsets = [ids]
    if len(ids) > 3:
        subsets.extend(
            [anchor_id for anchor_id in ids if anchor_id != omitted]
            for omitted in ids
        )
    candidates = []
    for subset in subsets:
        solution = trilaterate_linear(
            ANCHORS,
            {anchor_id: distances[anchor_id] for anchor_id in subset},
        )
        if solution is None:
            continue
        absolute = sorted(
            (abs(residual), anchor_id)
            for anchor_id, residual in position_residuals(
                solution, distances
            ).items()
        )
        inlier_ids = [
            anchor_id for residual, anchor_id in absolute
            if residual <= 0.12
        ]
        trimmed = absolute[: min(3, len(absolute))]
        trimmed_rms = math.sqrt(
            statistics.fmean(residual**2 for residual, _ in trimmed)
        )
        seed_distance = math.dist(solution, seed) if seed is not None else 0.0
        candidates.append(
            (
                -len(inlier_ids),
                trimmed_rms,
                seed_distance,
                solution,
                inlier_ids,
            )
        )
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[:3])
    _, trimmed_rms, _, solution, inlier_ids = candidates[0]
    if len(inlier_ids) < 3 or trimmed_rms > 0.12:
        return None
    return (
        trilaterate_linear(
            ANCHORS,
            {anchor_id: distances[anchor_id] for anchor_id in inlier_ids},
        )
        or solution
    )


def native_positions(
    events: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
    for event in events:
        if event.get("kind") != "ds_range":
            continue
        anchor_id = int(event["anchor_id"])
        frame_id = (int(event["seq"]) - ANCHOR_INDEX[anchor_id]) % 65536
        grouped[frame_id][anchor_id] = event

    rows: list[dict[str, Any]] = []
    seed: tuple[float, float] | None = None
    for frame_id, frame in grouped.items():
        if set(frame) != set(ANCHOR_IDS):
            continue
        distances = {
            anchor_id: float(frame[anchor_id]["distance_m"])
            for anchor_id in ANCHOR_IDS
        }
        solution = robust_trilaterate(distances, seed)
        if solution is None:
            continue
        seed = solution
        rows.append(
            {
                "frame_id": frame_id,
                "received_at": statistics.fmean(
                    float(item["received_at"]) for item in frame.values()
                ),
                "x_m": solution[0],
                "y_m": solution[1],
            }
        )
    rows.sort(key=lambda item: float(item["received_at"]))
    return rows


def rolling_passive_positions(
    observations: list[dict[str, Any]],
    field: str,
) -> list[dict[str, Any]]:
    frames: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for event in observations:
        frames[int(event["slot_id"]) // 3].append(event)
    latest: dict[tuple[int, int], dict[str, Any]] = {}
    rows: list[dict[str, Any]] = []
    seed = TAG
    for frame_id in sorted(frames):
        received_at = 0.0
        for event in frames[frame_id]:
            copy = dict(event)
            copy["diff_m"] = float(event[field])
            key = (int(event["initiator_id"]), int(event["responder_id"]))
            latest[key] = copy
            received_at = max(received_at, float(event["received_at"]))
        solution = solve_tdoa(ANCHORS, list(latest.values()), seed)
        if solution is None:
            continue
        seed = solution
        rows.append(
            {
                "frame_id": frame_id,
                "received_at": received_at,
                "x_m": solution[0],
                "y_m": solution[1],
            }
        )
    return rows


def incremental_bias_design(
    event: dict[str, Any],
    include_pair_terms: bool,
) -> list[float]:
    initiator = int(event["initiator_id"])
    responder = int(event["responder_id"])
    variable_anchors = ANCHOR_IDS[1:]
    row = [0.0] * len(variable_anchors)
    if responder != ANCHOR_IDS[0]:
        row[variable_anchors.index(responder)] += 1.0
    if initiator != ANCHOR_IDS[0]:
        row[variable_anchors.index(initiator)] -= 1.0
    if include_pair_terms:
        pair = tuple(sorted((initiator, responder)))
        row.extend(1.0 if pair == candidate else 0.0 for candidate in PAIR_ORDER)
    return row


def fit_incremental_bias(
    observations: list[dict[str, Any]],
    include_pair_terms: bool,
) -> np.ndarray:
    matrix = np.asarray(
        [
            incremental_bias_design(event, include_pair_terms)
            for event in observations
        ],
        dtype=float,
    )
    values = np.asarray(
        [float(event["diff_m"]) for event in observations],
        dtype=float,
    )
    fitted, *_ = np.linalg.lstsq(matrix, values, rcond=None)
    return fitted


def apply_incremental_bias(
    observations: list[dict[str, Any]],
    parameters: np.ndarray,
    *,
    include_pair_terms: bool,
    output_field: str,
) -> None:
    for event in observations:
        event[output_field] = float(event["diff_m"]) - float(
            np.dot(
                incremental_bias_design(event, include_pair_terms),
                parameters,
            )
        )


def smooth_cfo(
    observations: list[dict[str, Any]],
    window: int = 31,
) -> None:
    history: dict[tuple[int, int], deque[float]] = defaultdict(
        lambda: deque(maxlen=window)
    )
    for event in observations:
        initiator = int(event["initiator_id"])
        responder = int(event["responder_id"])
        key = (initiator, responder)
        history[key].append(float(event["cfo_correction_m"]))
        correction = float(np.median(history[key]))
        anchor_bias = (
            CURRENT_ANCHOR_BIAS_M[responder]
            - CURRENT_ANCHOR_BIAS_M[initiator]
        )
        event["cfo_only_diff_m"] = (
            float(event["raw_diff_m"])
            - float(event["cfo_correction_m"])
        )
        event["smoothed_cfo_diff_m"] = (
            float(event["raw_diff_m"]) - correction - anchor_bias
        )


def path_metrics(
    observations: list[dict[str, Any]],
    field: str,
    label: str,
) -> list[dict[str, Any]]:
    rows = []
    for initiator in ANCHOR_IDS:
        for responder in ANCHOR_IDS:
            if initiator == responder:
                continue
            values = np.asarray(
                [
                    float(event[field])
                    for event in observations
                    if int(event["initiator_id"]) == initiator
                    and int(event["responder_id"]) == responder
                ],
                dtype=float,
            )
            rows.append(
                {
                    "variant": label,
                    "initiator_id": initiator,
                    "responder_id": responder,
                    "n": int(values.size),
                    "mean_error_cm": float(np.mean(values) * 100.0),
                    "std_cm": float(np.std(values, ddof=1) * 100.0),
                    "p95_abs_error_cm": float(
                        np.percentile(np.abs(values), 95) * 100.0
                    ),
                    "max_abs_error_cm": float(
                        np.max(np.abs(values)) * 100.0
                    ),
                }
            )
    return rows


def pooled_within_path_std_cm(
    observations: list[dict[str, Any]],
    field: str,
) -> float:
    centered: list[float] = []
    for initiator in ANCHOR_IDS:
        for responder in ANCHOR_IDS:
            if initiator == responder:
                continue
            values = np.asarray(
                [
                    float(event[field])
                    for event in observations
                    if int(event["initiator_id"]) == initiator
                    and int(event["responder_id"]) == responder
                ]
            )
            centered.extend(values - np.mean(values))
    return float(np.std(np.asarray(centered), ddof=1) * 100.0)


def mean_absolute_path_bias_cm(
    observations: list[dict[str, Any]],
    field: str,
) -> float:
    biases = []
    for initiator in ANCHOR_IDS:
        for responder in ANCHOR_IDS:
            if initiator == responder:
                continue
            values = [
                float(event[field])
                for event in observations
                if int(event["initiator_id"]) == initiator
                and int(event["responder_id"]) == responder
            ]
            biases.append(abs(statistics.fmean(values)))
    return statistics.fmean(biases) * 100.0


def passive_path_spike_count(
    observations: list[dict[str, Any]],
    field: str = "diff_m",
) -> int:
    count = 0
    for initiator in ANCHOR_IDS:
        for responder in ANCHOR_IDS:
            if initiator == responder:
                continue
            values = np.asarray(
                [
                    float(event[field])
                    for event in observations
                    if int(event["initiator_id"]) == initiator
                    and int(event["responder_id"]) == responder
                ],
                dtype=float,
            )
            median = float(np.median(values))
            robust_sigma = 1.4826 * float(np.median(np.abs(values - median)))
            gate = max(0.10, 6.0 * robust_sigma)
            count += int(np.sum(np.abs(values - median) > gate))
    return count


def save_figure(figure: plt.Figure, name: str) -> None:
    figure.tight_layout()
    for suffix in ("png", "pdf"):
        figure.savefig(
            FIGURE_DIR / f"{name}.{suffix}",
            dpi=220 if suffix == "png" else None,
            bbox_inches="tight",
        )
    plt.close(figure)


def position_errors_cm(rows: Iterable[dict[str, Any]]) -> np.ndarray:
    return np.asarray(
        [
            math.hypot(float(row["x_m"]) - TAG[0], float(row["y_m"]) - TAG[1])
            * 100.0
            for row in rows
        ],
        dtype=float,
    )


def main() -> int:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    native: dict[str, list[dict[str, Any]]] = {}
    native_measurement_quality: dict[str, dict[str, Any]] = {}
    position_rows: list[dict[str, Any]] = []
    for block, label in (
        ("native_before", "Native before"),
        ("native_after", "Native after"),
    ):
        events = read_jsonl(DATA_DIR / f"{block}.jsonl")
        metadata = json.loads(
            (DATA_DIR / f"{block}.metadata.json").read_text(encoding="utf-8")
        )
        ranges = [
            event for event in events if event.get("kind") == "ds_range"
        ]
        invalid_ranges = [
            event for event in ranges
            if float(event["distance_m"]) <= 0.0
            or float(event["distance_m"]) >= 5.0
        ]
        native_measurement_quality[label] = {
            "range_count": len(ranges),
            "invalid_outside_0_to_5m": len(invalid_ranges),
            "invalid_pct": 100.0 * len(invalid_ranges) / len(ranges),
        }
        native[label] = native_positions(events)
        position_rows.append(
            metric_row(
                label,
                "surveyed geometry, robust 3-of-4",
                native[label],
                float(metadata["duration_sec"]),
            )
        )

    passive_events = read_jsonl(DATA_DIR / "passive_calibrated.jsonl")
    passive_metadata = json.loads(
        (DATA_DIR / "passive_calibrated.metadata.json").read_text(
            encoding="utf-8"
        )
    )
    duration = float(passive_metadata["duration_sec"])
    observations = sorted(
        [
            event for event in passive_events
            if event.get("kind") == "tdoa_observation"
        ],
        key=lambda item: float(item["received_at"]),
    )
    embedded = [
        event for event in passive_events
        if event.get("kind") == "local_position"
    ]
    anchor_ranges = [
        event for event in passive_events
        if event.get("kind") == "anchor_range"
    ]
    smooth_cfo(observations)
    baseline_offline = rolling_passive_positions(observations, "diff_m")
    smoothed_cfo_offline = rolling_passive_positions(
        observations, "smoothed_cfo_diff_m"
    )
    position_rows.extend(
        [
            metric_row(
                "Passive",
                "embedded autonomous geometry",
                embedded,
                duration,
            ),
            metric_row(
                "Passive",
                "surveyed geometry, rolling 12 directed paths",
                baseline_offline,
                duration,
            ),
            metric_row(
                "Passive",
                "surveyed geometry, 31-sample path CFO median",
                smoothed_cfo_offline,
                duration,
            ),
        ]
    )

    split_index = len(observations) // 3
    split_time = float(observations[split_index]["received_at"])
    training = [
        event for event in observations
        if float(event["received_at"]) < split_time
    ]
    validation = [
        event for event in observations
        if float(event["received_at"]) >= split_time
    ]
    anchor_parameters = fit_incremental_bias(training, False)
    joint_parameters = fit_incremental_bias(training, True)
    apply_incremental_bias(
        observations,
        anchor_parameters,
        include_pair_terms=False,
        output_field="refit_anchor_diff_m",
    )
    apply_incremental_bias(
        observations,
        joint_parameters,
        include_pair_terms=True,
        output_field="refit_joint_diff_m",
    )
    validation_baseline = rolling_passive_positions(validation, "diff_m")
    validation_anchor = rolling_passive_positions(
        validation, "refit_anchor_diff_m"
    )
    validation_joint = rolling_passive_positions(
        validation, "refit_joint_diff_m"
    )
    validation_duration = (
        max(float(event["received_at"]) for event in validation)
        - min(float(event["received_at"]) for event in validation)
    )
    position_rows.extend(
        [
            metric_row(
                "Passive validation",
                "current calibration",
                validation_baseline,
                validation_duration,
            ),
            metric_row(
                "Passive validation",
                "incremental per-anchor calibration",
                validation_anchor,
                validation_duration,
            ),
            metric_row(
                "Passive validation",
                "joint per-anchor + symmetric pair calibration",
                validation_joint,
                validation_duration,
            ),
        ]
    )

    path_rows = []
    path_rows.extend(path_metrics(observations, "diff_m", "current"))
    path_rows.extend(
        path_metrics(
            observations,
            "smoothed_cfo_diff_m",
            "31-sample path CFO median",
        )
    )
    path_rows.extend(
        path_metrics(validation, "refit_anchor_diff_m", "refit anchor validation")
    )
    path_rows.extend(
        path_metrics(validation, "refit_joint_diff_m", "refit joint validation")
    )
    write_csv(REPORT_DIR / "position_metrics.csv", position_rows)
    write_csv(REPORT_DIR / "directed_path_metrics.csv", path_rows)

    embedded_accepted = [
        row for row in embedded if float(row.get("rms_m") or math.inf) <= 0.15
    ]
    summary = {
        "experiment": "Native / calibrated Passive / Native A-B-A",
        "geometry": {
            "anchors_m": {
                str(anchor_id): list(position)
                for anchor_id, position in ANCHORS.items()
            },
            "tag_m": list(TAG),
        },
        "position_metrics": position_rows,
        "native_measurement_quality": native_measurement_quality,
        "passive_observation_count": len(observations),
        "passive_embedded_position_count": len(embedded),
        "passive_measurement_quality": {
            "raw_mean_absolute_directed_path_bias_cm":
                mean_absolute_path_bias_cm(observations, "raw_diff_m"),
            "cfo_corrected_mean_absolute_directed_path_bias_cm":
                mean_absolute_path_bias_cm(observations, "cfo_only_diff_m"),
            "current_calibrated_mean_absolute_directed_path_bias_cm":
                mean_absolute_path_bias_cm(observations, "diff_m"),
            "raw_within_path_std_cm": pooled_within_path_std_cm(
                observations, "raw_diff_m"
            ),
            "cfo_corrected_within_path_std_cm": pooled_within_path_std_cm(
                observations, "cfo_only_diff_m"
            ),
            "current_calibrated_within_path_std_cm":
                pooled_within_path_std_cm(observations, "diff_m"),
            "robust_path_spikes": passive_path_spike_count(observations),
            "robust_path_spike_pct":
                100.0
                * passive_path_spike_count(observations)
                / len(observations),
            "anchor_range_errors_over_12cm": sum(
                abs(
                    float(event["distance_m"])
                    - math.dist(
                        ANCHORS[int(event["anchor_a_id"])],
                        ANCHORS[int(event["anchor_b_id"])],
                    )
                )
                > 0.12
                for event in anchor_ranges
            ),
        },
        "passive_current_within_path_std_cm": pooled_within_path_std_cm(
            observations, "diff_m"
        ),
        "passive_smoothed_cfo_within_path_std_cm": pooled_within_path_std_cm(
            observations, "smoothed_cfo_diff_m"
        ),
        "passive_embedded_rms_gate_15cm": {
            "accepted": len(embedded_accepted),
            "rejected": len(embedded) - len(embedded_accepted),
            "metrics": position_metrics(embedded_accepted, TAG),
        },
        "calibration_validation": {
            "training_fraction": 1.0 / 3.0,
            "validation_fraction": 2.0 / 3.0,
            "incremental_anchor_bias_mm": {
                "2": 0,
                **{
                    str(anchor_id): int(round(anchor_parameters[index] * 1000.0))
                    for index, anchor_id in enumerate(ANCHOR_IDS[1:])
                },
            },
            "joint_incremental_anchor_bias_mm": {
                "2": 0,
                **{
                    str(anchor_id): int(round(joint_parameters[index] * 1000.0))
                    for index, anchor_id in enumerate(ANCHOR_IDS[1:])
                },
            },
            "joint_symmetric_pair_bias_mm": {
                f"{first}-{second}": int(
                    round(joint_parameters[3 + index] * 1000.0)
                )
                for index, (first, second) in enumerate(PAIR_ORDER)
            },
        },
    }
    (REPORT_DIR / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "figure.facecolor": "white",
        }
    )
    figure, axis = plt.subplots(figsize=(7.5, 4.6))
    series = (
        ("Native before", native["Native before"], "#2563eb"),
        ("Passive ESP32", embedded, "#dc2626"),
        ("Passive same geometry/solver", baseline_offline, "#16a34a"),
        ("Native after", native["Native after"], "#7c3aed"),
    )
    for label, rows, color in series:
        errors = np.sort(position_errors_cm(rows))
        axis.plot(
            errors,
            np.arange(1, len(errors) + 1) / len(errors),
            label=label,
            color=color,
        )
    axis.set_xlim(left=0)
    axis.set_ylim(0, 1.005)
    axis.set_xlabel("2-D error from surveyed center [cm]")
    axis.set_ylabel("Empirical CDF")
    axis.set_title("Native vs Passive DS-TWR: absolute position error")
    axis.legend()
    save_figure(figure, "position_error_cdf")

    figure, axes = plt.subplots(1, 3, figsize=(12.2, 4.2), sharex=True, sharey=True)
    scatter_series = (
        ("Native before", native["Native before"], "#2563eb"),
        ("Passive ESP32", embedded, "#dc2626"),
        ("Passive surveyed geometry", baseline_offline, "#16a34a"),
    )
    for axis, (label, rows, color) in zip(axes, scatter_series):
        sample = rows[:: max(1, len(rows) // 2500)]
        axis.scatter(
            [float(row["x_m"]) for row in sample],
            [float(row["y_m"]) for row in sample],
            s=5,
            alpha=0.28,
            color=color,
        )
        axis.scatter([TAG[0]], [TAG[1]], marker="*", s=110, color="black")
        axis.set_title(label)
        axis.set_xlabel("x [m]")
        axis.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("y [m]")
    save_figure(figure, "position_scatter")

    labels = [
        f"{initiator}→{responder}"
        for initiator in ANCHOR_IDS
        for responder in ANCHOR_IDS
        if initiator != responder
    ]
    current_means = [
        next(
            row["mean_error_cm"] for row in path_rows
            if row["variant"] == "current"
            and row["initiator_id"] == initiator
            and row["responder_id"] == responder
        )
        for initiator in ANCHOR_IDS
        for responder in ANCHOR_IDS
        if initiator != responder
    ]
    refit_means = [
        next(
            row["mean_error_cm"] for row in path_rows
            if row["variant"] == "refit joint validation"
            and row["initiator_id"] == initiator
            and row["responder_id"] == responder
        )
        for initiator in ANCHOR_IDS
        for responder in ANCHOR_IDS
        if initiator != responder
    ]
    x = np.arange(len(labels))
    figure, axis = plt.subplots(figsize=(9.5, 4.5))
    axis.bar(x - 0.2, current_means, width=0.4, label="current calibration")
    axis.bar(x + 0.2, refit_means, width=0.4, label="joint refit validation")
    axis.axhline(0, color="black", linewidth=0.8)
    axis.set_xticks(x, labels, rotation=45, ha="right")
    axis.set_ylabel("Mean directed observation error [cm]")
    axis.set_title("Passive DS-TWR residual bias by directed path")
    axis.legend()
    save_figure(figure, "directed_path_bias")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
