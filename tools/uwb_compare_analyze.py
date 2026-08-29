#!/usr/bin/env python3
"""Analyze controlled native DS-TWR versus FlexTDOA field captures.

The script intentionally keeps four layers separate:
  * raw measurement quality;
  * positions recomputed with the surveyed 3 m square;
  * positions published by the embedded FlexTDOA solver;
  * radio/telemetry throughput and completeness.
"""

from __future__ import annotations

import csv
import hashlib
import json
import lzma
import math
import pathlib
import statistics
from collections import Counter, defaultdict
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
BUNDLE_DIR = ROOT / "reports" / "bundles" / "uwb_protocol_comparison_20260726"
REPORT_DIR = BUNDLE_DIR / "analysis"
DATA_DIR = BUNDLE_DIR / "raw" / "data"
FIGURE_DIR = BUNDLE_DIR / "figures"

ANCHOR_IDS = (2, 3, 4, 5)
ANCHOR_INDEX = {anchor_id: index for index, anchor_id in enumerate(ANCHOR_IDS)}
SURVEYED_ANCHORS = {
    2: (0.0, 0.0),
    3: (0.0, 3.0),
    4: (3.0, 0.0),
    5: (3.0, 3.0),
}
NVS_ANCHORS = {
    2: (0.0, 0.0),
    3: (0.0, 3.045),
    4: (3.030, -0.058),
    5: (3.040, 2.880),
}
SURVEYED_TAG = (1.5, 1.5)
NVS_TAG_REFERENCE = tuple(
    sum(NVS_ANCHORS[anchor_id][axis] for anchor_id in ANCHOR_IDS)
    / len(ANCHOR_IDS)
    for axis in (0, 1)
)
TRUE_RANGE_M = math.sqrt(1.5**2 + 1.5**2)
FLEX_PHYSICAL_BOUND_MARGIN_M = 0.01
FLEX_POSITION_RMS_LIMIT_M = max(
    math.dist(first, second)
    for first in SURVEYED_ANCHORS.values()
    for second in SURVEYED_ANCHORS.values()
)
BLOCKS = ("flex_1", "ds_1", "flex_2", "ds_2", "flex_3", "ds_3")
PROTOCOL_COLORS = {
    "DS-TWR": "#2563eb",
    "FlexTDOA offline": "#ea580c",
    "FlexTDOA ESP32": "#16a34a",
}


def percentile(values: Iterable[float], q: float) -> float:
    array = np.asarray(list(values), dtype=float)
    return float(np.percentile(array, q)) if array.size else math.nan


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def flex_observation_is_structurally_valid(event: dict[str, Any]) -> bool:
    initiator_id = int(event["initiator_id"])
    responder_id = int(event["responder_id"])
    if (
        initiator_id not in SURVEYED_ANCHORS
        or responder_id not in SURVEYED_ANCHORS
        or not finite(event.get("diff_m"))
    ):
        return False
    separation = math.dist(
        SURVEYED_ANCHORS[initiator_id],
        SURVEYED_ANCHORS[responder_id],
    )
    return (
        abs(float(event["diff_m"]))
        <= separation + FLEX_PHYSICAL_BOUND_MARGIN_M
    )


def flex_position_is_structurally_valid(event: dict[str, Any]) -> bool:
    """Use only solver self-consistency, never ground-truth position error."""
    return (
        finite(event.get("rms_m"))
        and float(event["rms_m"]) <= FLEX_POSITION_RMS_LIMIT_M
    )


def load_events(block: str) -> Iterable[dict[str, Any]]:
    path = DATA_DIR / f"{block}.jsonl"
    if not path.exists():
        path = path.with_suffix(path.suffix + ".xz")
    opener = lzma.open if path.suffix == ".xz" else open
    with opener(path, "rt", encoding="utf-8") as handle:
        for line in handle:
            yield json.loads(line)


def load_metadata(block: str) -> dict[str, Any]:
    with (DATA_DIR / f"{block}.metadata.json").open(encoding="utf-8") as handle:
        return json.load(handle)


def solve_2x2(
    a00: float,
    a01: float,
    a10: float,
    a11: float,
    b0: float,
    b1: float,
) -> tuple[float, float] | None:
    determinant = a00 * a11 - a01 * a10
    if abs(determinant) < 1e-12:
        return None
    return (
        (b0 * a11 - a01 * b1) / determinant,
        (a00 * b1 - b0 * a10) / determinant,
    )


def trilaterate_linear(
    anchors: dict[int, tuple[float, float]],
    distances: dict[int, float],
) -> tuple[float, float] | None:
    """Match the dashboard's unweighted linear trilateration."""
    usable = [
        (anchor_id, anchors[anchor_id], float(distances[anchor_id]))
        for anchor_id in sorted(distances)
        if anchor_id in anchors
        and finite(distances[anchor_id])
        and float(distances[anchor_id]) > 0
    ]
    if len(usable) < 3:
        return None
    _, base, r0 = usable[0]
    nxx = nxy = nyy = rhs_x = rhs_y = 0.0
    for _, anchor, distance in usable[1:]:
        ax = 2.0 * (anchor[0] - base[0])
        ay = 2.0 * (anchor[1] - base[1])
        b = (
            r0 * r0
            - distance * distance
            + anchor[0] * anchor[0]
            - base[0] * base[0]
            + anchor[1] * anchor[1]
            - base[1] * base[1]
        )
        nxx += ax * ax
        nxy += ax * ay
        nyy += ay * ay
        rhs_x += ax * b
        rhs_y += ay * b
    return solve_2x2(nxx, nxy, nxy, nyy, rhs_x, rhs_y)


def solve_tdoa(
    anchors: dict[int, tuple[float, float]],
    observations: list[dict[str, Any]],
    initial: tuple[float, float] | None = None,
) -> tuple[float, float] | None:
    """Match the dashboard's raw 2-D Gauss-Newton AlgMin solver."""
    usable = []
    for item in observations:
        initiator_id = int(item["initiator_id"])
        responder_id = int(item["responder_id"])
        if (
            initiator_id in anchors
            and responder_id in anchors
            and finite(item.get("diff_m"))
        ):
            usable.append(
                (
                    anchors[initiator_id],
                    anchors[responder_id],
                    float(item["diff_m"]),
                )
            )
    if len(usable) < 2:
        return None

    if initial is None:
        x = statistics.fmean(anchor[0] for anchor in anchors.values())
        y = statistics.fmean(anchor[1] for anchor in anchors.values())
    else:
        x, y = initial

    for _ in range(24):
        nxx = 1e-6
        nxy = 0.0
        nyy = 1e-6
        rhs_x = 0.0
        rhs_y = 0.0
        used = 0
        for initiator, responder, difference in usable:
            di = max(1e-6, math.hypot(x - initiator[0], y - initiator[1]))
            dr = max(1e-6, math.hypot(x - responder[0], y - responder[1]))
            residual = (dr - di) - difference
            gx = (x - responder[0]) / dr - (x - initiator[0]) / di
            gy = (y - responder[1]) / dr - (y - initiator[1]) / di
            nxx += gx * gx
            nxy += gx * gy
            nyy += gy * gy
            rhs_x += -gx * residual
            rhs_y += -gy * residual
            used += 1
        if used < 2:
            return None
        step = solve_2x2(nxx, nxy, nxy, nyy, rhs_x, rhs_y)
        if step is None or not all(finite(value) for value in step):
            return None
        dx = max(-0.5, min(0.5, step[0]))
        dy = max(-0.5, min(0.5, step[1]))
        x += dx
        y += dy
        if math.hypot(dx, dy) < 0.0005:
            break
    return x, y


def position_metrics(
    rows: list[dict[str, Any]],
    truth: tuple[float, float],
) -> dict[str, float | int]:
    x = np.asarray([float(row["x_m"]) for row in rows], dtype=float)
    y = np.asarray([float(row["y_m"]) for row in rows], dtype=float)
    if not x.size:
        return {"n": 0}
    dx = x - truth[0]
    dy = y - truth[1]
    radial = np.hypot(dx, dy)
    mean_x = float(np.mean(x))
    mean_y = float(np.mean(y))
    centered = np.hypot(x - mean_x, y - mean_y)
    sx = float(np.std(x, ddof=1)) if x.size > 1 else 0.0
    sy = float(np.std(y, ddof=1)) if y.size > 1 else 0.0
    within_50_cm = radial <= 0.5
    outlier_count = int(np.sum(~within_50_cm))
    return {
        "n": int(x.size),
        "mean_x_m": mean_x,
        "mean_y_m": mean_y,
        "bias_x_cm": float(np.mean(dx) * 100.0),
        "bias_y_cm": float(np.mean(dy) * 100.0),
        "mean_error_cm": float(np.mean(radial) * 100.0),
        "rmse_2d_cm": float(np.sqrt(np.mean(radial**2)) * 100.0),
        "median_error_cm": float(np.median(radial) * 100.0),
        "p95_error_cm": float(np.percentile(radial, 95) * 100.0),
        "p99_error_cm": float(np.percentile(radial, 99) * 100.0),
        "max_error_cm": float(np.max(radial) * 100.0),
        "outliers_gt_50cm": outlier_count,
        "outliers_gt_50cm_pct": 100.0 * outlier_count / x.size,
        "rmse_within_50cm_cm": float(
            np.sqrt(np.mean(radial[within_50_cm] ** 2)) * 100.0
        )
        if np.any(within_50_cm)
        else math.nan,
        "std_x_cm": sx * 100.0,
        "std_y_cm": sy * 100.0,
        "cep50_precision_cm": float(np.median(centered) * 100.0),
        "cep95_precision_cm": float(np.percentile(centered, 95) * 100.0),
        "two_drms_cm": 2.0 * math.sqrt(sx * sx + sy * sy) * 100.0,
    }


def measurement_metrics(values_m: list[float]) -> dict[str, float | int]:
    array = np.asarray(values_m, dtype=float)
    if not array.size:
        return {"n": 0}
    median = float(np.median(array))
    robust_sigma = 1.4826 * float(np.median(np.abs(array - median)))
    return {
        "n": int(array.size),
        "bias_cm": float(np.mean(array) * 100.0),
        "std_cm": float(np.std(array, ddof=1) * 100.0),
        "mae_cm": float(np.mean(np.abs(array)) * 100.0),
        "median_cm": median * 100.0,
        "robust_sigma_mad_cm": robust_sigma * 100.0,
        "p05_cm": float(np.percentile(array, 5) * 100.0),
        "p95_cm": float(np.percentile(array, 95) * 100.0),
        "p95_abs_cm": float(np.percentile(np.abs(array), 95) * 100.0),
        "p99_abs_cm": float(np.percentile(np.abs(array), 99) * 100.0),
        "max_abs_cm": float(np.max(np.abs(array)) * 100.0),
        "rmse_cm": float(np.sqrt(np.mean(array**2)) * 100.0),
    }


def interval_metrics(timestamps: list[float]) -> dict[str, float]:
    ordered = np.sort(np.asarray(timestamps, dtype=float))
    if ordered.size < 2:
        return {"median_interval_ms": math.nan, "p95_interval_ms": math.nan}
    delta = np.diff(ordered)
    delta = delta[delta > 1e-6]
    if not delta.size:
        return {"median_interval_ms": math.nan, "p95_interval_ms": math.nan}
    return {
        "median_interval_ms": float(np.median(delta) * 1000.0),
        "p95_interval_ms": float(np.percentile(delta, 95) * 1000.0),
    }


def write_csv(path: pathlib.Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=keys,
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def save_figure(figure: plt.Figure, name: str) -> None:
    figure.tight_layout()
    for suffix in ("pdf", "png"):
        figure.savefig(
            FIGURE_DIR / f"{name}.{suffix}",
            dpi=220 if suffix == "png" else None,
            bbox_inches="tight",
        )
    plt.close(figure)


def subsample(rows: list[dict[str, Any]], maximum: int = 6000) -> list[dict[str, Any]]:
    if len(rows) <= maximum:
        return rows
    indices = np.linspace(0, len(rows) - 1, maximum, dtype=int)
    return [rows[index] for index in indices]


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
            "figure.titlesize": 12,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "figure.facecolor": "white",
        }
    )

    ds_ranges: list[dict[str, Any]] = []
    ds_positions: list[dict[str, Any]] = []
    flex_observations: list[dict[str, Any]] = []
    flex_valid_observations: list[dict[str, Any]] = []
    flex_slot_positions: list[dict[str, Any]] = []
    flex_embedded_positions: list[dict[str, Any]] = []
    flex_valid_embedded_positions: list[dict[str, Any]] = []
    block_metrics: list[dict[str, Any]] = []
    measurement_rows: list[dict[str, Any]] = []
    metadata = {block: load_metadata(block) for block in BLOCKS}

    for block in BLOCKS:
        protocol = "FlexTDOA" if block.startswith("flex") else "DS-TWR"
        duration = float(metadata[block]["duration_sec"])
        block_ds: list[dict[str, Any]] = []
        block_flex: list[dict[str, Any]] = []
        block_flex_valid: list[dict[str, Any]] = []
        block_positions: list[dict[str, Any]] = []
        block_positions_valid: list[dict[str, Any]] = []
        for event in load_events(block):
            kind = event["kind"]
            if kind == "ds_range":
                event["error_m"] = float(event["distance_m"]) - TRUE_RANGE_M
                block_ds.append(event)
                ds_ranges.append(event)
            elif kind == "tdoa_observation":
                event["error_m"] = float(event["diff_m"])
                event["raw_error_m"] = float(event["raw_diff_m"])
                event["comparison_valid"] = int(
                    flex_observation_is_structurally_valid(event)
                )
                event["exclusion_reason"] = (
                    ""
                    if event["comparison_valid"]
                    else "physical_range_difference_violation"
                )
                block_flex.append(event)
                flex_observations.append(event)
                if event["comparison_valid"]:
                    block_flex_valid.append(event)
                    flex_valid_observations.append(event)
            elif kind == "local_position":
                event["truth_x_m"] = NVS_TAG_REFERENCE[0]
                event["truth_y_m"] = NVS_TAG_REFERENCE[1]
                event["error_m"] = math.hypot(
                    float(event["x_m"]) - NVS_TAG_REFERENCE[0],
                    float(event["y_m"]) - NVS_TAG_REFERENCE[1],
                )
                event["comparison_valid"] = int(
                    flex_position_is_structurally_valid(event)
                )
                event["exclusion_reason"] = (
                    ""
                    if event["comparison_valid"]
                    else "internal_rms_exceeds_anchor_array_diagonal"
                )
                block_positions.append(event)
                flex_embedded_positions.append(event)
                if event["comparison_valid"]:
                    block_positions_valid.append(event)
                    flex_valid_embedded_positions.append(event)

        if protocol == "DS-TWR":
            groups: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
            for event in block_ds:
                anchor_id = int(event["anchor_id"])
                frame_key = (
                    int(event["seq"]) - ANCHOR_INDEX[anchor_id]
                ) % 65536
                groups[frame_key][anchor_id] = event
            complete = 0
            frame_timestamps: list[float] = []
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
                timestamp = statistics.fmean(
                    float(items[anchor_id]["received_at"])
                    for anchor_id in ANCHOR_IDS
                )
                row = {
                    "block": block,
                    "frame_key": frame_key,
                    "received_at": timestamp,
                    "x_m": solution[0],
                    "y_m": solution[1],
                    "truth_x_m": SURVEYED_TAG[0],
                    "truth_y_m": SURVEYED_TAG[1],
                    "error_m": math.hypot(
                        solution[0] - SURVEYED_TAG[0],
                        solution[1] - SURVEYED_TAG[1],
                    ),
                    **{
                        f"range_a{anchor_id}_m": distances[anchor_id]
                        for anchor_id in ANCHOR_IDS
                    },
                }
                ds_positions.append(row)
                frame_timestamps.append(timestamp)
                complete += 1
            expected_ranges = duration * 4.0 / 0.064
            expected_frames = duration / 0.064
            block_metrics.append(
                {
                    "block": block,
                    "protocol": protocol,
                    "duration_s": duration,
                    "measurements": len(block_ds),
                    "measurement_rate_hz": len(block_ds) / duration,
                    "valid_measurements": len(block_ds),
                    "excluded_measurements": 0,
                    "valid_measurement_rate_hz": len(block_ds) / duration,
                    "theoretical_measurement_rate_hz": 62.5,
                    "measurement_delivery_pct": 100.0
                    * len(block_ds)
                    / expected_ranges,
                    "position_updates": complete,
                    "position_rate_hz": complete / duration,
                    "theoretical_position_rate_hz": 15.625,
                    "position_delivery_pct": 100.0
                    * complete
                    / expected_frames,
                    "complete_units": complete,
                    "candidate_units": len(groups),
                    "complete_unit_pct": 100.0 * complete / len(groups),
                    **interval_metrics(frame_timestamps),
                    **{
                        f"pos_{key}": value
                        for key, value in position_metrics(
                            [
                                row
                                for row in ds_positions
                                if row["block"] == block
                            ],
                            SURVEYED_TAG,
                        ).items()
                    },
                }
            )
        else:
            groups: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
            slot_times: dict[int, float] = {}
            observed_slot_ids = {
                int(event["slot_id"]) for event in block_flex
            }
            for event in block_flex_valid:
                slot_id = int(event["slot_id"])
                groups[slot_id][int(event["responder_id"])] = event
                slot_times[slot_id] = max(
                    slot_times.get(slot_id, -math.inf),
                    float(event["received_at"]),
                )
            complete = 0
            offline_timestamps: list[float] = []
            for slot_id, items in groups.items():
                if len(items) < 3:
                    continue
                observations = sorted(
                    items.values(),
                    key=lambda item: int(item.get("responder_index", 99)),
                )
                solution = solve_tdoa(
                    SURVEYED_ANCHORS,
                    observations,
                    SURVEYED_TAG,
                )
                if solution is None:
                    continue
                timestamp = slot_times[slot_id]
                row = {
                    "block": block,
                    "slot_id": slot_id,
                    "initiator_id": int(observations[0]["initiator_id"]),
                    "received_at": timestamp,
                    "x_m": solution[0],
                    "y_m": solution[1],
                    "truth_x_m": SURVEYED_TAG[0],
                    "truth_y_m": SURVEYED_TAG[1],
                    "error_m": math.hypot(
                        solution[0] - SURVEYED_TAG[0],
                        solution[1] - SURVEYED_TAG[1],
                    ),
                }
                flex_slot_positions.append(row)
                offline_timestamps.append(timestamp)
                complete += 1
            expected_observations = duration * 1000.0
            expected_slots = duration / 0.003
            embedded_times = [
                float(row["received_at"]) for row in block_positions_valid
            ]
            block_metrics.append(
                {
                    "block": block,
                    "protocol": protocol,
                    "duration_s": duration,
                    "measurements": len(block_flex),
                    "measurement_rate_hz": len(block_flex) / duration,
                    "valid_measurements": len(block_flex_valid),
                    "excluded_measurements": (
                        len(block_flex) - len(block_flex_valid)
                    ),
                    "valid_measurement_rate_hz": (
                        len(block_flex_valid) / duration
                    ),
                    "theoretical_measurement_rate_hz": 1000.0,
                    "measurement_delivery_pct": 100.0
                    * len(block_flex)
                    / expected_observations,
                    "observed_slots": len(observed_slot_ids),
                    "slot_rate_hz": len(observed_slot_ids) / duration,
                    "theoretical_slot_rate_hz": 333.333333,
                    "slot_delivery_pct": (
                        100.0 * len(observed_slot_ids) / expected_slots
                    ),
                    "complete_units": complete,
                    "candidate_units": len(observed_slot_ids),
                    "complete_unit_pct": (
                        100.0 * complete / len(observed_slot_ids)
                    ),
                    "offline_position_rate_hz": complete / duration,
                    **{
                        f"offline_pos_{key}": value
                        for key, value in position_metrics(
                            [
                                row
                                for row in flex_slot_positions
                                if row["block"] == block
                            ],
                            SURVEYED_TAG,
                        ).items()
                    },
                    "position_updates": len(block_positions_valid),
                    "excluded_position_updates": (
                        len(block_positions) - len(block_positions_valid)
                    ),
                    "position_rate_hz": len(block_positions_valid) / duration,
                    "theoretical_position_rate_hz": math.nan,
                    "position_delivery_pct": math.nan,
                    **interval_metrics(embedded_times),
                    **{
                        f"pos_{key}": value
                        for key, value in position_metrics(
                            block_positions_valid,
                            NVS_TAG_REFERENCE,
                        ).items()
                    },
                }
            )

    for anchor_id in ANCHOR_IDS:
        errors = [
            float(row["error_m"])
            for row in ds_ranges
            if int(row["anchor_id"]) == anchor_id
        ]
        measurement_rows.append(
            {
                "protocol": "DS-TWR",
                "series": f"A{anchor_id}",
                "reference": f"{TRUE_RANGE_M:.9f} m",
                **measurement_metrics(errors),
            }
        )
    for initiator_id in ANCHOR_IDS:
        for responder_id in ANCHOR_IDS:
            if initiator_id == responder_id:
                continue
            all_selected = [
                row
                for row in flex_observations
                if int(row["initiator_id"]) == initiator_id
                and int(row["responder_id"]) == responder_id
            ]
            selected = [
                row for row in all_selected if row["comparison_valid"]
            ]
            pair_separation = math.dist(
                SURVEYED_ANCHORS[initiator_id],
                SURVEYED_ANCHORS[responder_id],
            )
            impossible_corrected = sum(
                not bool(row["comparison_valid"]) for row in all_selected
            )
            impossible_raw = sum(
                abs(float(row["raw_error_m"]))
                > pair_separation + FLEX_PHYSICAL_BOUND_MARGIN_M
                for row in all_selected
            )
            measurement_rows.append(
                {
                    "protocol": "FlexTDOA corrected",
                    "series": f"A{initiator_id}->A{responder_id}",
                    "reference": "0 m",
                    "physical_bound_m": pair_separation,
                    "physically_impossible_count": impossible_corrected,
                    **measurement_metrics(
                        [float(row["error_m"]) for row in selected]
                    ),
                }
            )
            measurement_rows.append(
                {
                    "protocol": "FlexTDOA uncorrected",
                    "series": f"A{initiator_id}->A{responder_id}",
                    "reference": "0 m",
                    "physical_bound_m": pair_separation,
                    "physically_impossible_count": impossible_raw,
                    **measurement_metrics(
                        [float(row["raw_error_m"]) for row in selected]
                    ),
                }
            )

    position_summary = {
        "DS-TWR surveyed offline": position_metrics(
            ds_positions,
            SURVEYED_TAG,
        ),
        "FlexTDOA surveyed complete-slot offline": position_metrics(
            flex_slot_positions,
            SURVEYED_TAG,
        ),
        "FlexTDOA embedded operational geometry (structurally valid)": position_metrics(
            flex_valid_embedded_positions,
            NVS_TAG_REFERENCE,
        ),
    }

    summary = {
        "experiment": {
            "date": "2026-07-26",
            "blocks": list(BLOCKS),
            "duration_per_protocol_s": 360.0,
            "surveyed_anchor_coordinates_m": SURVEYED_ANCHORS,
            "surveyed_tag_coordinates_m": SURVEYED_TAG,
            "survey_uncertainty_mm": 2.0,
            "true_tag_anchor_range_m": TRUE_RANGE_M,
            "nvs_anchor_coordinates_m": NVS_ANCHORS,
            "nvs_center_reference_m": NVS_TAG_REFERENCE,
        },
        "position_metrics": position_summary,
        "aggregate_counts": {
            "ds_ranges": len(ds_ranges),
            "ds_complete_positions": len(ds_positions),
            "flex_observations": len(flex_observations),
            "flex_valid_observations": len(flex_valid_observations),
            "flex_excluded_observations": (
                len(flex_observations) - len(flex_valid_observations)
            ),
            "flex_complete_slot_positions": len(flex_slot_positions),
            "flex_embedded_positions": len(flex_embedded_positions),
            "flex_valid_embedded_positions": len(
                flex_valid_embedded_positions
            ),
            "flex_excluded_embedded_positions": (
                len(flex_embedded_positions)
                - len(flex_valid_embedded_positions)
            ),
            "flex_position_internal_rms_limit_m": (
                FLEX_POSITION_RMS_LIMIT_M
            ),
            "flex_corrected_physically_impossible": sum(
                int(row.get("physically_impossible_count", 0))
                for row in measurement_rows
                if row["protocol"] == "FlexTDOA corrected"
            ),
        },
        "aggregate_measurement_metrics": {
            "DS-TWR range error": measurement_metrics(
                [float(row["error_m"]) for row in ds_ranges]
            ),
            "FlexTDOA corrected valid range-difference error": measurement_metrics(
                [float(row["error_m"]) for row in flex_valid_observations]
            ),
            "FlexTDOA uncorrected valid range-difference error": measurement_metrics(
                [float(row["raw_error_m"]) for row in flex_valid_observations]
            ),
            "FlexTDOA corrected all observations (audit)": measurement_metrics(
                [float(row["error_m"]) for row in flex_observations]
            ),
        },
        "block_metrics": block_metrics,
    }
    with (REPORT_DIR / "analysis_summary.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=True)

    write_csv(REPORT_DIR / "block_metrics.csv", block_metrics)
    write_csv(REPORT_DIR / "measurement_metrics.csv", measurement_rows)
    write_csv(
        REPORT_DIR / "position_metrics.csv",
        [
            {"series": series, **metrics}
            for series, metrics in position_summary.items()
        ],
    )
    write_csv(REPORT_DIR / "ds_positions.csv", ds_positions)
    write_csv(REPORT_DIR / "flex_slot_positions.csv", flex_slot_positions)
    write_csv(
        REPORT_DIR / "flex_embedded_positions.csv",
        flex_embedded_positions,
    )

    # Figure 1: surveyed geometry and the NVS geometry used operationally.
    figure, axes = plt.subplots(1, 2, figsize=(8.0, 3.5))
    for axis, anchors, tag, title in (
        (
            axes[0],
            SURVEYED_ANCHORS,
            SURVEYED_TAG,
            "Surveyed geometry (common analysis)",
        ),
        (
            axes[1],
            NVS_ANCHORS,
            NVS_TAG_REFERENCE,
            "NVS geometry (operational solver)",
        ),
    ):
        polygon = [anchors[key] for key in (2, 3, 5, 4, 2)]
        axis.plot(
            [item[0] for item in polygon],
            [item[1] for item in polygon],
            "-",
            color="#64748b",
            linewidth=1.2,
        )
        for anchor_id, (x, y) in anchors.items():
            axis.scatter(x, y, s=50, marker="s", color="#0f172a")
            axis.annotate(
                f"A{anchor_id}",
                (x, y),
                xytext=(5, 5),
                textcoords="offset points",
            )
        axis.scatter(
            tag[0],
            tag[1],
            s=70,
            marker="*",
            color="#dc2626",
            label="tag reference",
        )
        axis.set_title(title)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(-0.25, 3.35)
        axis.set_ylim(-0.3, 3.3)
        axis.legend(loc="lower right")
    save_figure(figure, "01_geometry")

    # Figure 2: DS-TWR range errors.
    figure, axis = plt.subplots(figsize=(7.2, 3.8))
    ds_by_anchor = [
        [
            100.0 * float(row["error_m"])
            for row in ds_ranges
            if int(row["anchor_id"]) == anchor_id
        ]
        for anchor_id in ANCHOR_IDS
    ]
    box = axis.boxplot(
        ds_by_anchor,
        tick_labels=[f"A{anchor_id}" for anchor_id in ANCHOR_IDS],
        showfliers=False,
        patch_artist=True,
    )
    for patch in box["boxes"]:
        patch.set_facecolor(PROTOCOL_COLORS["DS-TWR"])
        patch.set_alpha(0.55)
    axis.axhline(0.0, color="#111827", linewidth=1.0)
    axis.set_title("DS-TWR: range error relative to 2.12132 m")
    axis.set_xlabel("anchor")
    axis.set_ylabel("error [cm]")
    save_figure(figure, "02_ds_range_error")

    # Figure 3: corrected and raw FlexTDOA directed-pair bias/std heatmaps.
    def pair_matrix(field: str, statistic: str) -> np.ndarray:
        result = np.full((4, 4), np.nan)
        for i, initiator_id in enumerate(ANCHOR_IDS):
            for j, responder_id in enumerate(ANCHOR_IDS):
                values = [
                    100.0 * float(row[field])
                    for row in flex_valid_observations
                    if int(row["initiator_id"]) == initiator_id
                    and int(row["responder_id"]) == responder_id
                ]
                if not values:
                    continue
                if statistic == "mean":
                    result[i, j] = float(np.mean(values))
                elif statistic == "std":
                    result[i, j] = float(np.std(values, ddof=1))
                else:
                    median = float(np.median(values))
                    result[i, j] = 1.4826 * float(
                        np.median(np.abs(np.asarray(values) - median))
                    )
        return result

    heatmaps = (
        (pair_matrix("error_m", "mean"), "Corrected: bias [cm]", "coolwarm"),
        (
            pair_matrix("error_m", "robust"),
            "Corrected: robust σ (MAD) [cm]",
            "viridis",
        ),
        (pair_matrix("raw_error_m", "mean"), "Uncorrected: bias [cm]", "coolwarm"),
    )
    figure, axes = plt.subplots(1, 3, figsize=(10.0, 3.5))
    for axis, (matrix, title, color_map) in zip(axes, heatmaps):
        vmax = float(np.nanmax(np.abs(matrix)))
        if color_map == "coolwarm":
            image = axis.imshow(matrix, cmap=color_map, vmin=-vmax, vmax=vmax)
        else:
            image = axis.imshow(matrix, cmap=color_map, vmin=0.0, vmax=vmax)
        for i in range(4):
            for j in range(4):
                if finite(matrix[i, j]):
                    axis.text(
                        j,
                        i,
                        f"{matrix[i, j]:.1f}",
                        ha="center",
                        va="center",
                        color="white" if abs(matrix[i, j]) > 0.55 * vmax else "black",
                        fontsize=8,
                    )
        axis.set_xticks(range(4), [f"A{x}" for x in ANCHOR_IDS])
        axis.set_yticks(range(4), [f"A{x}" for x in ANCHOR_IDS])
        axis.set_xlabel("responder")
        axis.set_ylabel("initiator")
        axis.set_title(title)
        figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
    save_figure(figure, "03_flex_pair_heatmaps")

    # Figure 4: position clouds on the appropriate common/reference geometry.
    position_sets = (
        (
            "DS-TWR",
            subsample(ds_positions),
            SURVEYED_TAG,
            PROTOCOL_COLORS["DS-TWR"],
        ),
        (
            "FlexTDOA offline",
            subsample(flex_slot_positions),
            SURVEYED_TAG,
            PROTOCOL_COLORS["FlexTDOA offline"],
        ),
        (
            "FlexTDOA ESP32",
            subsample(flex_valid_embedded_positions),
            NVS_TAG_REFERENCE,
            PROTOCOL_COLORS["FlexTDOA ESP32"],
        ),
    )
    figure, axes = plt.subplots(1, 3, figsize=(10.2, 3.5))
    all_extent = []
    for _, rows, truth, _ in position_sets:
        all_extent.extend(
            max(
                abs(float(row["x_m"]) - truth[0]),
                abs(float(row["y_m"]) - truth[1]),
            )
            for row in rows
        )
    limit = min(1.0, max(0.15, percentile(all_extent, 99.5) * 1.15))
    for axis, (title, rows, truth, color) in zip(axes, position_sets):
        x = [float(row["x_m"]) for row in rows]
        y = [float(row["y_m"]) for row in rows]
        axis.scatter(x, y, s=4, alpha=0.18, color=color, rasterized=True)
        axis.scatter(
            [truth[0]],
            [truth[1]],
            marker="+",
            s=120,
            linewidths=2.0,
            color="#111827",
        )
        axis.set_xlim(truth[0] - limit, truth[0] + limit)
        axis.set_ylim(truth[1] - limit, truth[1] + limit)
        axis.set_aspect("equal", adjustable="box")
        axis.set_title(title)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
    save_figure(figure, "04_position_clouds")

    # Figure 5: radial error empirical CDF.
    figure, axis = plt.subplots(figsize=(7.2, 4.1))
    cdf_sets = (
        (
            "DS-TWR",
            ds_positions,
            SURVEYED_TAG,
            PROTOCOL_COLORS["DS-TWR"],
        ),
        (
            "FlexTDOA offline",
            flex_slot_positions,
            SURVEYED_TAG,
            PROTOCOL_COLORS["FlexTDOA offline"],
        ),
        (
            "FlexTDOA ESP32",
            flex_valid_embedded_positions,
            NVS_TAG_REFERENCE,
            PROTOCOL_COLORS["FlexTDOA ESP32"],
        ),
    )
    cdf_limits = []
    cdf_maxima = []
    for title, rows, truth, color in cdf_sets:
        errors_cm = np.sort(
            np.asarray(
                [
                    100.0
                    * math.hypot(
                        float(row["x_m"]) - truth[0],
                        float(row["y_m"]) - truth[1],
                    )
                    for row in rows
                ]
            )
        )
        cdf_limits.append(float(np.percentile(errors_cm, 99.9)))
        cdf_maxima.append(float(np.max(errors_cm)))
        probability = np.arange(1, len(errors_cm) + 1) / len(errors_cm)
        axis.plot(errors_cm, probability, label=title, color=color, linewidth=1.7)
    axis.axhline(0.95, color="#64748b", linestyle="--", linewidth=1.0)
    display_limit = max(cdf_limits) * 1.08
    axis.set_xlim(0.0, display_limit)
    axis.set_ylim(0.0, 1.01)
    axis.set_xlabel("radial error [cm]")
    axis.set_ylabel("cumulative probability")
    axis.set_title("Position-error CDF")
    axis.legend()
    if max(cdf_maxima) > display_limit:
        axis.text(
            0.99,
            0.04,
            f"maximum outside axis: {max(cdf_maxima):.1f} cm",
            transform=axis.transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            color="#475569",
        )
    save_figure(figure, "05_position_error_cdf")

    # Figure 6: one-second median position error across alternating blocks.
    figure, axis = plt.subplots(figsize=(8.2, 4.1))
    offset = 0.0
    block_centers: list[tuple[float, str]] = []
    for block in BLOCKS:
        if block.startswith("ds"):
            rows = [row for row in ds_positions if row["block"] == block]
            truth = SURVEYED_TAG
            color = PROTOCOL_COLORS["DS-TWR"]
        else:
            rows = [
                row
                for row in flex_valid_embedded_positions
                if row["block"] == block
            ]
            truth = NVS_TAG_REFERENCE
            color = PROTOCOL_COLORS["FlexTDOA ESP32"]
        if not rows:
            continue
        start = min(float(row["received_at"]) for row in rows)
        bins: dict[int, list[float]] = defaultdict(list)
        for row in rows:
            second = int(float(row["received_at"]) - start)
            bins[second].append(
                100.0
                * math.hypot(
                    float(row["x_m"]) - truth[0],
                    float(row["y_m"]) - truth[1],
                )
            )
        x = [offset + second for second in sorted(bins)]
        y = [statistics.median(bins[second]) for second in sorted(bins)]
        axis.plot(x, y, color=color, linewidth=1.0)
        block_centers.append((offset + 60.0, block))
        offset += 120.0
        axis.axvline(offset, color="#cbd5e1", linewidth=0.8)
    for center, block in block_centers:
        axis.text(
            center,
            0.965,
            block,
            transform=axis.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8,
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.65},
        )
    axis.set_xlabel("cumulative block time [s]")
    axis.set_ylabel("1 s median error [cm]")
    axis.set_title("Temporal stability over the alternating Flex/DS sequence")
    save_figure(figure, "06_position_error_time")

    # Figure 7: measured throughput and completeness.
    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8))
    labels = [row["block"] for row in block_metrics]
    rates = [
        float(row["valid_measurement_rate_hz"]) for row in block_metrics
    ]
    colors = [
        PROTOCOL_COLORS["FlexTDOA offline"]
        if row["protocol"] == "FlexTDOA"
        else PROTOCOL_COLORS["DS-TWR"]
        for row in block_metrics
    ]
    axes[0].bar(labels, rates, color=colors, alpha=0.78)
    axes[0].set_ylabel("valid observations/s")
    axes[0].set_title("Observed throughput")
    axes[0].tick_params(axis="x", rotation=30)
    completeness = [
        float(row["complete_unit_pct"]) for row in block_metrics
    ]
    axes[1].bar(labels, completeness, color=colors, alpha=0.78)
    axes[1].set_ylabel("complete units [%]")
    axes[1].set_title("DS frames / Flex slots with all 4 / 3 measurements")
    axes[1].set_ylim(0.0, 105.0)
    axes[1].tick_params(axis="x", rotation=30)
    save_figure(figure, "07_throughput_completeness")

    # Figure 8: block-level RMSE and p95 repeatability.
    figure, axis = plt.subplots(figsize=(8.0, 4.0))
    rmse = [float(row["pos_rmse_2d_cm"]) for row in block_metrics]
    p95 = [float(row["pos_p95_error_cm"]) for row in block_metrics]
    x = np.arange(len(labels))
    width = 0.36
    axis.bar(x - width / 2, rmse, width, label="RMSE 2D", color="#475569")
    axis.bar(x + width / 2, p95, width, label="P95", color="#94a3b8")
    axis.set_xticks(x, labels, rotation=30)
    axis.set_ylabel("error [cm]")
    axis.set_title("Between-block repeatability (published/operational position)")
    axis.legend()
    save_figure(figure, "08_block_repeatability")

    # Figure 9: protocol transaction/timing summary.
    figure, axes = plt.subplots(2, 1, figsize=(9.0, 4.3))
    axes[0].set_xlim(0, 64)
    axes[0].set_ylim(-0.7, 1.2)
    for index, anchor_id in enumerate(ANCHOR_IDS):
        start = index * 15
        axes[0].broken_barh(
            [(start, 15)],
            (-0.1, 0.5),
            facecolors=PROTOCOL_COLORS["DS-TWR"],
            alpha=0.65,
        )
        axes[0].text(start + 7.5, 0.15, f"A{anchor_id}\\nP-R-F", ha="center", va="center")
    axes[0].broken_barh([(60, 4)], (-0.1, 0.5), facecolors="#94a3b8")
    axes[0].text(62, 0.15, "gap", ha="center", va="center")
    axes[0].set_yticks([])
    axes[0].set_xlabel("time [ms]")
    axes[0].set_title("DS-TWR: 4 × 15 ms + 4 ms = 64 ms")

    axes[1].set_xlim(0, 12)
    axes[1].set_ylim(-0.7, 1.2)
    for index, anchor_id in enumerate(ANCHOR_IDS):
        start = index * 3
        axes[1].broken_barh(
            [(start, 3)],
            (-0.1, 0.5),
            facecolors=PROTOCOL_COLORS["FlexTDOA offline"],
            alpha=0.65,
        )
        axes[1].text(start + 1.5, 0.15, f"A{anchor_id}\\nREQ+3 RESP", ha="center", va="center")
    axes[1].set_yticks([])
    axes[1].set_xlabel("time [ms]")
    axes[1].set_title("FlexTDOA: 4 × 3 ms = 12 ms; passive tag")
    save_figure(figure, "09_protocol_timing")

    manifest_rows = []
    for path in sorted(DATA_DIR.glob("*")):
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        manifest_rows.append(
            {
                "file": str(path.relative_to(ROOT)),
                "bytes": path.stat().st_size,
                "sha256": digest.hexdigest(),
            }
        )
    write_csv(REPORT_DIR / "raw_data_manifest.csv", manifest_rows)

    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
