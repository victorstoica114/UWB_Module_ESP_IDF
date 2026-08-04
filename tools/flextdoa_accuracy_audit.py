#!/usr/bin/env python3
"""Audit raw FlexTDOA accuracy without third-party Python packages.

The script separates absolute position bias from random spread and evaluates
whether TDOA residuals look like per-anchor, response-index, or directed-link
offsets.  Bias models are learned on the first half of a capture and evaluated
on the second half so the reported improvement is not an in-sample solver fit.
"""

from __future__ import annotations

import argparse
import json
import lzma
import math
import statistics
from collections import defaultdict, deque
from pathlib import Path
from typing import Iterable


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def open_text(path: Path):
    if path.suffix.lower() == ".xz":
        return lzma.open(path, mode="rt", encoding="utf-8")
    return path.open(encoding="utf-8")


def ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    return (
        (normal + altitude_m) * cos_latitude * math.cos(longitude),
        (normal + altitude_m) * cos_latitude * math.sin(longitude),
        (normal * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


def derive_rtk_geometry(path: Path) -> tuple[dict[int, tuple[float, float]], tuple[float, float]]:
    fixes: dict[int, list[tuple[float, float, float]]] = defaultdict(list)
    with open_text(path) as handle:
        for line in handle:
            event = json.loads(line)
            if (
                event.get("kind") == "gps_fix"
                and int(event.get("gps_fix_quality") or 0) == 4
                and bool(event.get("gps_fix_valid"))
            ):
                fixes[int(event["module_id"])].append(
                    (
                        float(event["gps_latitude_deg"]),
                        float(event["gps_longitude_deg"]),
                        float(event["gps_altitude_m"]),
                    )
                )
    if any(module_id not in fixes for module_id in (1, 2, 3, 4, 5)):
        raise ValueError("RTK capture lacks fixed samples for one or more modules")
    origin_latitude = statistics.median(row[0] for row in fixes[2])
    origin_longitude = statistics.median(row[1] for row in fixes[2])
    origin_altitude = statistics.median(row[2] for row in fixes[2])
    origin = ecef(origin_latitude, origin_longitude, origin_altitude)
    latitude = math.radians(origin_latitude)
    longitude = math.radians(origin_longitude)

    def to_enu(row: tuple[float, float, float]) -> tuple[float, float, float]:
        point = ecef(*row)
        dx, dy, dz = (point[index] - origin[index] for index in range(3))
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

    centers = {
        module_id: tuple(
            statistics.median(point[axis] for point in map(to_enu, rows))
            for axis in range(3)
        )
        for module_id, rows in fixes.items()
    }
    anchors = {module_id: centers[module_id][:2] for module_id in (2, 3, 4, 5)}
    return anchors, centers[1][:2]


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * q / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def metrics(points: Iterable[tuple[float, float]], truth: tuple[float, float]) -> dict:
    rows = list(points)
    dx = [point[0] - truth[0] for point in rows]
    dy = [point[1] - truth[1] for point in rows]
    radial = [math.hypot(x, y) for x, y in zip(dx, dy)]
    if not rows:
        return {"n": 0}
    bias_x = statistics.fmean(dx)
    bias_y = statistics.fmean(dy)
    centered = [
        math.hypot(x - bias_x, y - bias_y) for x, y in zip(dx, dy)
    ]
    return {
        "n": len(rows),
        "bias_x_cm": bias_x * 100.0,
        "bias_y_cm": bias_y * 100.0,
        "rmse_cm": math.sqrt(statistics.fmean(value * value for value in radial)) * 100.0,
        "debiased_rmse_cm": math.sqrt(
            statistics.fmean(value * value for value in centered)
        )
        * 100.0,
        "median_cm": statistics.median(radial) * 100.0,
        "p95_cm": percentile(radial, 95.0) * 100.0,
    }


def solve_2x2(
    a00: float, a01: float, a11: float, b0: float, b1: float
) -> tuple[float, float] | None:
    determinant = a00 * a11 - a01 * a01
    if abs(determinant) < 1e-12:
        return None
    return (
        (b0 * a11 - a01 * b1) / determinant,
        (a00 * b1 - a01 * b0) / determinant,
    )


def solve_tdoa(
    anchors: dict[int, tuple[float, float]],
    observations: list[dict],
    initial: tuple[float, float],
) -> tuple[float, float] | None:
    x, y = initial
    for _ in range(24):
        nxx = 1e-6
        nxy = 0.0
        nyy = 1e-6
        rhs_x = 0.0
        rhs_y = 0.0
        used = 0
        for item in observations:
            initiator = anchors[int(item["initiator_id"])]
            responder = anchors[int(item["responder_id"])]
            difference = float(item["diff_m"])
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
        step = solve_2x2(nxx, nxy, nyy, rhs_x, rhs_y)
        if step is None:
            return None
        dx = max(-0.5, min(0.5, step[0]))
        dy = max(-0.5, min(0.5, step[1]))
        x += dx
        y += dy
        if math.hypot(dx, dy) < 0.0005:
            break
    return x, y


def solve_linear(normal: list[list[float]], rhs: list[float]) -> list[float]:
    matrix = [row[:] + [value] for row, value in zip(normal, rhs)]
    size = len(rhs)
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(matrix[row][column]))
        if abs(matrix[pivot][column]) < 1e-12:
            raise ValueError("singular normal matrix")
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        divisor = matrix[column][column]
        matrix[column] = [value / divisor for value in matrix[column]]
        for row in range(size):
            if row == column:
                continue
            factor = matrix[row][column]
            matrix[row] = [
                left - factor * right
                for left, right in zip(matrix[row], matrix[column])
            ]
    return [matrix[row][-1] for row in range(size)]


def fit_anchor_offsets(rows: list[dict], anchor_ids: list[int]) -> dict[int, float]:
    # Anchor 2 is the reference.  residual ~= offset[responder]-offset[initiator].
    reference = anchor_ids[0]
    variables = [anchor_id for anchor_id in anchor_ids if anchor_id != reference]
    index = {anchor_id: position for position, anchor_id in enumerate(variables)}
    normal = [[0.0 for _ in variables] for _ in variables]
    rhs = [0.0 for _ in variables]
    for item in rows:
        design = [0.0 for _ in variables]
        initiator_id = int(item["initiator_id"])
        responder_id = int(item["responder_id"])
        if initiator_id != reference:
            design[index[initiator_id]] -= 1.0
        if responder_id != reference:
            design[index[responder_id]] += 1.0
        residual = float(item["residual_m"])
        for row_index, left in enumerate(design):
            rhs[row_index] += left * residual
            for column_index, right in enumerate(design):
                normal[row_index][column_index] += left * right
    fitted = solve_linear(normal, rhs)
    return {reference: 0.0, **dict(zip(variables, fitted))}


def group_means(rows: list[dict], key, field: str = "residual_m") -> dict:
    groups: dict[object, list[float]] = defaultdict(list)
    for item in rows:
        groups[key(item)].append(float(item[field]))
    return {name: statistics.fmean(values) for name, values in groups.items()}


def residual_summary(rows: list[dict], field: str = "residual_m") -> dict:
    values = [float(item[field]) for item in rows]
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "bias_cm": statistics.fmean(values) * 100.0,
        "std_cm": statistics.stdev(values) * 100.0 if len(values) > 1 else 0.0,
        "rmse_cm": math.sqrt(statistics.fmean(value * value for value in values))
        * 100.0,
        "p95_abs_cm": percentile([abs(value) for value in values], 95.0) * 100.0,
    }


def corrected_groups(
    groups: dict[int, list[dict]],
    anchors: dict[int, tuple[float, float]],
    truth: tuple[float, float],
    correction,
    minimum_group: int,
    minimum_observations: int,
    exact_observations: int | None = None,
    source_field: str = "diff_m",
) -> list[tuple[float, float]]:
    positions = []
    initial = truth
    for group_id in sorted(groups):
        observations_in_group = groups[group_id]
        if (
            group_id < minimum_group
            or len(observations_in_group) < minimum_observations
            or (
                exact_observations is not None
                and len(observations_in_group) != exact_observations
            )
        ):
            continue
        observations = []
        for item in observations_in_group:
            copy = dict(item)
            copy["diff_m"] = float(copy[source_field]) - correction(copy)
            observations.append(copy)
        solution = solve_tdoa(anchors, observations, initial)
        if solution is not None:
            positions.append(solution)
            initial = solution
    return positions


def frame_average_cfo_positions(
    frames: dict[int, list[dict]],
    anchors: dict[int, tuple[float, float]],
    truth: tuple[float, float],
    minimum_frame: int,
    request_us: float,
    request_process_us: float,
    response_us: float,
) -> list[tuple[float, float]]:
    speed_of_light_mps = 299702547.0
    positions = []
    initial = truth
    for frame_id in sorted(frames):
        rows = frames[frame_id]
        if frame_id < minimum_frame or len(rows) != 12:
            continue
        ratios: dict[int, list[float]] = defaultdict(list)
        for item in rows:
            reply_seconds = (
                request_us
                + request_process_us
                + int(item["responder_index"]) * response_us
            ) * 1e-6
            ratio = (
                float(item["raw_diff_m"]) - float(item["diff_m"])
            ) / (reply_seconds * speed_of_light_mps)
            ratios[int(item["responder_id"])].append(ratio)
        if any(len(ratios[anchor_id]) != 3 for anchor_id in anchors):
            continue
        mean_ratio = {
            anchor_id: statistics.fmean(values)
            for anchor_id, values in ratios.items()
        }
        observations = []
        for item in rows:
            copy = dict(item)
            reply_seconds = (
                request_us
                + request_process_us
                + int(item["responder_index"]) * response_us
            ) * 1e-6
            copy["diff_m"] = float(item["raw_diff_m"]) - (
                mean_ratio[int(item["responder_id"])]
                * reply_seconds
                * speed_of_light_mps
            )
            observations.append(copy)
        solution = solve_tdoa(anchors, observations, initial)
        if solution is not None:
            positions.append(solution)
            initial = solution
    return positions


def rolling_cfo_positions(
    frames: dict[int, list[dict]],
    anchors: dict[int, tuple[float, float]],
    truth: tuple[float, float],
    minimum_frame: int,
    request_us: float,
    request_process_us: float,
    response_us: float,
    window_frames: int,
    anchor_offsets: dict[int, float],
) -> list[tuple[float, float]]:
    speed_of_light_mps = 299702547.0
    history = {
        anchor_id: deque(maxlen=window_frames * 3) for anchor_id in anchors
    }
    positions = []
    initial = truth
    for frame_id in sorted(frames):
        rows = frames[frame_id]
        if len(rows) != 12:
            continue
        for item in rows:
            reply_seconds = (
                request_us
                + request_process_us
                + int(item["responder_index"]) * response_us
            ) * 1e-6
            history[int(item["responder_id"])].append(
                (float(item["raw_diff_m"]) - float(item["diff_m"]))
                / (reply_seconds * speed_of_light_mps)
            )
        if frame_id < minimum_frame or any(not values for values in history.values()):
            continue
        mean_ratio = {
            anchor_id: statistics.fmean(values)
            for anchor_id, values in history.items()
        }
        observations = []
        for item in rows:
            copy = dict(item)
            reply_seconds = (
                request_us
                + request_process_us
                + int(item["responder_index"]) * response_us
            ) * 1e-6
            copy["diff_m"] = (
                float(item["raw_diff_m"])
                - mean_ratio[int(item["responder_id"])]
                * reply_seconds
                * speed_of_light_mps
                - anchor_offsets[int(item["responder_id"])]
                + anchor_offsets[int(item["initiator_id"])]
            )
            observations.append(copy)
        solution = solve_tdoa(anchors, observations, initial)
        if solution is not None:
            positions.append(solution)
            initial = solution
    return positions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--anchors", default="2:0,0;3:3.844,2.923;4:-3.120,3.899;5:1.055,6.836")
    parser.add_argument("--truth", default="0.332,3.392")
    parser.add_argument(
        "--rtk-capture",
        type=Path,
        help="derive anchor geometry and tag truth from RTK-fixed samples",
    )
    parser.add_argument("--request-us", type=float, default=2000.0)
    parser.add_argument("--request-process-us", type=float, default=250.0)
    parser.add_argument("--response-us", type=float, default=500.0)
    parser.add_argument(
        "--anchor-biases-cm",
        help="override RTK-derived per-anchor TDOA biases, e.g. 2:0;3:11.2;4:5.1;5:-0.9",
    )
    args = parser.parse_args()

    anchors = {
        int(identifier): tuple(float(value) for value in coordinate.split(","))
        for identifier, coordinate in (
            item.split(":", 1) for item in args.anchors.split(";")
        )
    }
    truth = tuple(float(value) for value in args.truth.split(","))
    if args.rtk_capture is not None:
        anchors, truth = derive_rtk_geometry(args.rtk_capture)
    positions = []
    observations = []
    anchor_ranges = []
    slots: dict[int, list[dict]] = defaultdict(list)
    frames: dict[int, list[dict]] = defaultdict(list)
    with open_text(args.capture) as handle:
        for line in handle:
            event = json.loads(line)
            if event.get("kind") == "local_position":
                positions.append((float(event["x_m"]), float(event["y_m"])))
            elif event.get("kind") == "tdoa_observation":
                initiator_id = int(event["initiator_id"])
                responder_id = int(event["responder_id"])
                di = math.dist(truth, anchors[initiator_id])
                dr = math.dist(truth, anchors[responder_id])
                event["residual_m"] = float(event["diff_m"]) - (dr - di)
                event["raw_residual_m"] = float(event["raw_diff_m"]) - (dr - di)
                observations.append(event)
                slots[int(event["slot_id"])].append(event)
                frames[int(event["slot_id"]) - int(event["slot_id"]) % 4].append(event)
            elif event.get("kind") == "anchor_range":
                anchor_a = int(event["anchor_a_id"])
                anchor_b = int(event["anchor_b_id"])
                if anchor_a in anchors and anchor_b in anchors:
                    event["pair"] = tuple(sorted((anchor_a, anchor_b)))
                    event["residual_m"] = float(event["distance_m"]) - math.dist(
                        anchors[anchor_a], anchors[anchor_b]
                    )
                    anchor_ranges.append(event)

    slot_ids = sorted(slots)
    split_slot = slot_ids[len(slot_ids) // 2]
    training = [item for item in observations if int(item["slot_id"]) < split_slot]
    evaluation = [item for item in observations if int(item["slot_id"]) >= split_slot]

    directed = group_means(
        training, lambda item: (int(item["initiator_id"]), int(item["responder_id"]))
    )
    raw_directed = group_means(
        training,
        lambda item: (int(item["initiator_id"]), int(item["responder_id"])),
        "raw_residual_m",
    )
    index_offsets = group_means(training, lambda item: int(item["responder_index"]))
    anchor_offsets = fit_anchor_offsets(training, sorted(anchors))
    anchor_range_training = [
        item
        for item in anchor_ranges
        if int(item.get("slot_id") or item.get("seq") or 0) < split_slot
    ]
    pair_biases = group_means(anchor_range_training, lambda item: item["pair"])
    pair_corrected_training = []
    for item in training:
        copy = dict(item)
        pair = tuple(sorted((int(item["initiator_id"]), int(item["responder_id"]))))
        copy["residual_m"] = float(item["residual_m"]) - pair_biases[pair]
        pair_corrected_training.append(copy)
    pair_anchor_offsets = fit_anchor_offsets(pair_corrected_training, sorted(anchors))
    rolling_anchor_offsets = pair_anchor_offsets
    if args.anchor_biases_cm:
        rolling_anchor_offsets = {
            int(identifier): float(value) / 100.0
            for identifier, value in (
                item.split(":", 1) for item in args.anchor_biases_cm.split(";")
            )
        }
        if set(rolling_anchor_offsets) != set(anchors):
            raise ValueError("--anchor-biases-cm must define every anchor")
    mean_cfo_correction = group_means(
        [
            {**item, "cfo_correction_m": float(item["raw_diff_m"]) - float(item["diff_m"])}
            for item in training
        ],
        lambda item: (int(item["initiator_id"]), int(item["responder_id"])),
        "cfo_correction_m",
    )

    output = {
        "capture": str(args.capture),
        "anchors_m": anchors,
        "truth_m": truth,
        "position_metrics_all": metrics(positions, truth),
        "split_slot": split_slot,
        "tdoa_training": residual_summary(training),
        "tdoa_evaluation": residual_summary(evaluation),
        "anchor_offsets_cm": {
            str(key): value * 100.0 for key, value in anchor_offsets.items()
        },
        "anchor_range_pair_biases_cm": {
            f"{key[0]}-{key[1]}": value * 100.0 for key, value in pair_biases.items()
        },
        "pair_corrected_anchor_offsets_cm": {
            str(key): value * 100.0 for key, value in pair_anchor_offsets.items()
        },
        "index_offsets_cm": {
            str(key): value * 100.0 for key, value in index_offsets.items()
        },
        "directed_offsets_cm": {
            f"{key[0]}->{key[1]}": value * 100.0 for key, value in directed.items()
        },
        "raw_directed_offsets_cm": {
            f"{key[0]}->{key[1]}": value * 100.0
            for key, value in raw_directed.items()
        },
        "evaluation_by_index": {
            str(index): residual_summary(
                [item for item in evaluation if int(item["responder_index"]) == index]
            )
            for index in range(3)
        },
        "evaluation_by_direction": {
            f"{initiator}->{responder}": residual_summary(
                [
                    item
                    for item in evaluation
                    if int(item["initiator_id"]) == initiator
                    and int(item["responder_id"]) == responder
                ]
            )
            for initiator in sorted(anchors)
            for responder in sorted(anchors)
            if initiator != responder
        },
        "raw_evaluation_by_direction": {
            f"{initiator}->{responder}": residual_summary(
                [
                    item
                    for item in evaluation
                    if int(item["initiator_id"]) == initiator
                    and int(item["responder_id"]) == responder
                ],
                "raw_residual_m",
            )
            for initiator in sorted(anchors)
            for responder in sorted(anchors)
            if initiator != responder
        },
    }

    corrections = {
        "raw_test_half": lambda item: 0.0,
        "anchor_offset_test_half": lambda item: (
            anchor_offsets[int(item["responder_id"])]
            - anchor_offsets[int(item["initiator_id"])]
        ),
        "response_index_test_half": lambda item: index_offsets[
            int(item["responder_index"])
        ],
        "directed_link_test_half": lambda item: directed[
            (int(item["initiator_id"]), int(item["responder_id"]))
        ],
        "anchor_range_pair_test_half": lambda item: pair_biases[
            tuple(sorted((int(item["initiator_id"]), int(item["responder_id"]))))
        ],
        "anchor_range_pair_plus_anchor_test_half": lambda item: (
            pair_biases[
                tuple(sorted((int(item["initiator_id"]), int(item["responder_id"]))))
            ]
            + pair_anchor_offsets[int(item["responder_id"])]
            - pair_anchor_offsets[int(item["initiator_id"])]
        ),
    }
    for name, correction in corrections.items():
        output[name] = metrics(
            corrected_groups(slots, anchors, truth, correction, split_slot, 3), truth
        )
        output[f"frame_{name}"] = metrics(
            corrected_groups(
                frames,
                anchors,
                truth,
                correction,
                split_slot - split_slot % 4,
                6,
            ),
            truth,
        )
        output[f"frame12_{name}"] = metrics(
            corrected_groups(
                frames,
                anchors,
                truth,
                correction,
                split_slot - split_slot % 4,
                12,
                exact_observations=12,
            ),
            truth,
        )

    raw_directed_correction = lambda item: raw_directed[
        (int(item["initiator_id"]), int(item["responder_id"]))
    ]
    output["raw_directed_slot_test_half"] = metrics(
        corrected_groups(
            slots,
            anchors,
            truth,
            raw_directed_correction,
            split_slot,
            3,
            source_field="raw_diff_m",
        ),
        truth,
    )
    output["raw_directed_frame12_test_half"] = metrics(
        corrected_groups(
            frames,
            anchors,
            truth,
            raw_directed_correction,
            split_slot - split_slot % 4,
            12,
            exact_observations=12,
            source_field="raw_diff_m",
        ),
        truth,
    )
    output["frame12_in_frame_cfo_mean_test_half"] = metrics(
        frame_average_cfo_positions(
            frames,
            anchors,
            truth,
            split_slot - split_slot % 4,
            args.request_us,
            args.request_process_us,
            args.response_us,
        ),
        truth,
    )
    mean_cfo_pair_correction = lambda item: (
        mean_cfo_correction[(int(item["initiator_id"]), int(item["responder_id"]))]
        + pair_biases[
            tuple(sorted((int(item["initiator_id"]), int(item["responder_id"]))))
        ]
    )
    output["mean_cfo_plus_anchor_range_frame12_test_half"] = metrics(
        corrected_groups(
            frames,
            anchors,
            truth,
            mean_cfo_pair_correction,
            split_slot - split_slot % 4,
            12,
            exact_observations=12,
            source_field="raw_diff_m",
        ),
        truth,
    )
    mean_cfo_pair_anchor_correction = lambda item: (
        mean_cfo_pair_correction(item)
        + pair_anchor_offsets[int(item["responder_id"])]
        - pair_anchor_offsets[int(item["initiator_id"])]
    )
    output["mean_cfo_plus_pair_anchor_frame12_test_half"] = metrics(
        corrected_groups(
            frames,
            anchors,
            truth,
            mean_cfo_pair_anchor_correction,
            split_slot - split_slot % 4,
            12,
            exact_observations=12,
            source_field="raw_diff_m",
        ),
        truth,
    )
    output["rolling_cfo_frame12_test_half"] = {
        str(window): metrics(
            rolling_cfo_positions(
                frames,
                anchors,
                truth,
                split_slot - split_slot % 4,
                args.request_us,
                args.request_process_us,
                args.response_us,
                window,
                rolling_anchor_offsets,
            ),
            truth,
        )
        for window in (1, 2, 4, 8, 16, 32, 64, 128)
    }

    print(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
