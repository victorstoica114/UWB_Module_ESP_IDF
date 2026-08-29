#!/usr/bin/env python3
"""Build a common-frame UWB versus RTK field comparison.

The three UWB protocols are deliberately solved offline with the same raw,
unweighted 2-D solver and the same RTK-derived anchor coordinates.  This keeps
the main comparison about the ranging observations, not about dashboard or
embedded-solver differences.  Embedded position streams are retained as a
separate end-to-end precision result.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
from collections import Counter, defaultdict
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ANCHOR_IDS = (2, 3, 4, 5)
ANCHOR_INDEX = {anchor_id: index for index, anchor_id in enumerate(ANCHOR_IDS)}
PROTOCOLS = {
    "flextdoa": "FlexTDOA",
    "ds_twr": "Native DS-TWR",
    "passive_ds": "Passive DS-TWR",
}
COLORS = {
    "FlexTDOA": "#ea580c",
    "Native DS-TWR": "#2563eb",
    "Passive DS-TWR": "#7c3aed",
    "GPS RTK": "#16a34a",
}
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def percentile(values: Iterable[float], q: float) -> float:
    array = np.asarray(list(values), dtype=float)
    return float(np.percentile(array, q)) if array.size else math.nan


def median(values: Iterable[float]) -> float:
    clean = [float(value) for value in values if finite(value)]
    return float(statistics.median(clean)) if clean else math.nan


def ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    return np.asarray(
        [
            (normal + altitude_m) * cos_latitude * math.cos(longitude),
            (normal + altitude_m) * cos_latitude * math.sin(longitude),
            (normal * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
        ],
        dtype=float,
    )


def enu_rotation(latitude_deg: float, longitude_deg: float) -> np.ndarray:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    return np.asarray(
        [
            [-math.sin(longitude), math.cos(longitude), 0.0],
            [
                -math.sin(latitude) * math.cos(longitude),
                -math.sin(latitude) * math.sin(longitude),
                math.cos(latitude),
            ],
            [
                math.cos(latitude) * math.cos(longitude),
                math.cos(latitude) * math.sin(longitude),
                math.sin(latitude),
            ],
        ],
        dtype=float,
    )


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
    usable = [
        (anchor_id, anchors[anchor_id], float(distances[anchor_id]))
        for anchor_id in sorted(distances)
        if anchor_id in anchors
        and finite(distances[anchor_id])
        and float(distances[anchor_id]) > 0.0
    ]
    if len(usable) < 3:
        return None
    _, base, radius0 = usable[0]
    nxx = nxy = nyy = rhs_x = rhs_y = 0.0
    for _, anchor, radius in usable[1:]:
        ax = 2.0 * (anchor[0] - base[0])
        ay = 2.0 * (anchor[1] - base[1])
        b = (
            radius0**2
            - radius**2
            + anchor[0] ** 2
            - base[0] ** 2
            + anchor[1] ** 2
            - base[1] ** 2
        )
        nxx += ax * ax
        nxy += ax * ay
        nyy += ay * ay
        rhs_x += ax * b
        rhs_y += ay * b
    return solve_2x2(nxx, nxy, nxy, nyy, rhs_x, rhs_y)


def solve_tdoa(
    anchors: dict[int, tuple[float, float]],
    observations: list[dict[str, float | int]],
) -> tuple[float, float] | None:
    usable: list[tuple[tuple[float, float], tuple[float, float], float]] = []
    for item in observations:
        initiator_id = int(item["initiator_id"])
        responder_id = int(item["responder_id"])
        difference = item.get("diff_m")
        if (
            initiator_id in anchors
            and responder_id in anchors
            and finite(difference)
        ):
            usable.append(
                (
                    anchors[initiator_id],
                    anchors[responder_id],
                    float(difference),
                )
            )
    if len(usable) < 2:
        return None

    x = statistics.fmean(point[0] for point in anchors.values())
    y = statistics.fmean(point[1] for point in anchors.values())
    for _ in range(32):
        nxx = nyy = 1e-6
        nxy = rhs_x = rhs_y = 0.0
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
        step = solve_2x2(nxx, nxy, nxy, nyy, rhs_x, rhs_y)
        if step is None or not all(finite(value) for value in step):
            return None
        dx = max(-1.0, min(1.0, step[0]))
        dy = max(-1.0, min(1.0, step[1]))
        x += dx
        y += dy
        if math.hypot(dx, dy) < 0.0002:
            break
    return (x, y) if finite(x) and finite(y) else None


def position_metrics(
    points: list[tuple[float, float, float]],
    reference: tuple[float, float] | None,
) -> dict[str, float | int]:
    if not points:
        return {"n": 0}
    x = np.asarray([point[1] for point in points], dtype=float)
    y = np.asarray([point[2] for point in points], dtype=float)
    center_x = float(np.median(x))
    center_y = float(np.median(y))
    centered = np.hypot(x - center_x, y - center_y)
    result: dict[str, float | int] = {
        "n": int(x.size),
        "median_x_m": center_x,
        "median_y_m": center_y,
        "mean_x_m": float(np.mean(x)),
        "mean_y_m": float(np.mean(y)),
        "std_x_cm": float(np.std(x, ddof=1) * 100.0) if x.size > 1 else 0.0,
        "std_y_cm": float(np.std(y, ddof=1) * 100.0) if y.size > 1 else 0.0,
        "precision_cep50_cm": float(np.median(centered) * 100.0),
        "precision_p95_cm": float(np.percentile(centered, 95) * 100.0),
        "precision_max_cm": float(np.max(centered) * 100.0),
        "two_drms_cm": float(
            2.0
            * math.hypot(
                np.std(x, ddof=1) if x.size > 1 else 0.0,
                np.std(y, ddof=1) if y.size > 1 else 0.0,
            )
            * 100.0
        ),
    }
    if reference is not None:
        dx = x - reference[0]
        dy = y - reference[1]
        radial = np.hypot(dx, dy)
        result.update(
            {
                "bias_east_cm": float(np.mean(dx) * 100.0),
                "bias_north_cm": float(np.mean(dy) * 100.0),
                "accuracy_mean_cm": float(np.mean(radial) * 100.0),
                "accuracy_rmse_cm": float(np.sqrt(np.mean(radial**2)) * 100.0),
                "accuracy_median_cm": float(np.median(radial) * 100.0),
                "accuracy_p95_cm": float(np.percentile(radial, 95) * 100.0),
                "accuracy_max_cm": float(np.max(radial) * 100.0),
            }
        )
    return result


def interval_metrics(points: list[tuple[float, float, float]]) -> dict[str, float]:
    times = np.asarray(sorted({point[0] for point in points}), dtype=float)
    if times.size < 2:
        return {
            "median_interval_ms": math.nan,
            "p95_interval_ms": math.nan,
            "max_interval_ms": math.nan,
        }
    intervals = np.diff(times) * 1000.0
    return {
        "median_interval_ms": float(np.median(intervals)),
        "p95_interval_ms": float(np.percentile(intervals, 95)),
        "max_interval_ms": float(np.max(intervals)),
    }


def json_lines(path: pathlib.Path) -> Iterable[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def block_protocol(metadata: dict[str, Any], path: pathlib.Path) -> str:
    value = str(metadata.get("protocol") or "")
    if value in PROTOCOLS:
        return value
    name = path.name
    if "passive" in name:
        return "passive_ds"
    if "ds_twr" in name:
        return "ds_twr"
    return "flextdoa"


def load_capture(data_dir: pathlib.Path) -> dict[str, Any]:
    blocks: dict[str, dict[str, Any]] = {}
    gps: dict[int, dict[tuple[int, int], dict[str, Any]]] = defaultdict(dict)
    for data_path in sorted(data_dir.glob("*.jsonl")):
        metadata_path = data_path.with_suffix(".metadata.json")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        protocol = block_protocol(metadata, data_path)
        block = data_path.stem
        duration = float(metadata.get("duration_sec") or 0.0)
        item: dict[str, Any] = {
            "block": block,
            "protocol": protocol,
            "duration_s": duration,
            "metadata": metadata,
            "tdoa": {},
            "ds": {},
            "anchor_ranges": defaultdict(dict),
            "local_positions": {},
            "counters": Counter(),
        }
        for event in json_lines(data_path):
            kind = str(event.get("kind") or "")
            item["counters"][kind] += 1
            if kind == "gps_fix":
                module_id = int(event.get("module_id") or 0)
                gga_count = int(event.get("gps_gga_count") or 0)
                key = (module_id, gga_count)
                if module_id > 0 and key not in gps[module_id]:
                    gps[module_id][key] = event
            elif kind == "local_position":
                event_id = int(
                    event.get("position_stream_event_id")
                    or event.get("position_event_id")
                    or 0
                )
                key = (event_id, int(event.get("slot_id") or 0))
                item["local_positions"].setdefault(key, event)
            elif kind == "tdoa_observation":
                key = (
                    int(event.get("slot_id") or event.get("seq") or 0),
                    int(event.get("initiator_id") or 0),
                    int(event.get("responder_id") or 0),
                    int(event.get("tag_id") or 0),
                )
                item["tdoa"].setdefault(key, event)
            elif kind == "ds_range":
                anchor_id = int(event.get("anchor_id") or 0)
                key = (
                    int(event.get("seq") or 0),
                    anchor_id,
                    int(event.get("tag_id") or 0),
                )
                item["ds"].setdefault(key, event)
            elif kind == "anchor_range":
                a = int(event.get("anchor_a_id") or 0)
                b = int(event.get("anchor_b_id") or 0)
                pair = tuple(sorted((a, b)))
                key = (
                    int(event.get("seq") or 0),
                    int(event.get("initiator_id") or 0),
                    int(event.get("responder_id") or 0),
                )
                if a and b:
                    item["anchor_ranges"][pair].setdefault(key, event)
        blocks[protocol] = item
    return {"blocks": blocks, "gps": gps}


def rtk_reference(
    gps_events: dict[int, dict[tuple[int, int], dict[str, Any]]]
) -> dict[str, Any]:
    fixed_by_module: dict[int, list[dict[str, Any]]] = {}
    all_by_module: dict[int, list[dict[str, Any]]] = {}
    for module_id, keyed in gps_events.items():
        rows = list(keyed.values())
        all_by_module[module_id] = rows
        fixed_by_module[module_id] = [
            row
            for row in rows
            if int(row.get("gps_fix_quality") or 0) == 4
            and bool(row.get("gps_fix_valid"))
            and finite(row.get("gps_latitude_deg"))
            and finite(row.get("gps_longitude_deg"))
            and finite(row.get("gps_altitude_m"))
        ]
    if 2 not in fixed_by_module or not fixed_by_module[2]:
        raise RuntimeError("A2 has no RTK-fixed samples for the ENU origin")
    origin_rows = fixed_by_module[2]
    origin_latitude = median(row["gps_latitude_deg"] for row in origin_rows)
    origin_longitude = median(row["gps_longitude_deg"] for row in origin_rows)
    origin_altitude = median(row["gps_altitude_m"] for row in origin_rows)
    origin_ecef = ecef(origin_latitude, origin_longitude, origin_altitude)
    rotation = enu_rotation(origin_latitude, origin_longitude)

    points: dict[int, list[tuple[float, float, float, float]]] = {}
    centers: dict[int, tuple[float, float, float]] = {}
    block_points: dict[str, dict[int, list[tuple[float, float, float, float]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for module_id, rows in fixed_by_module.items():
        module_points = []
        for row in rows:
            vector = ecef(
                float(row["gps_latitude_deg"]),
                float(row["gps_longitude_deg"]),
                float(row["gps_altitude_m"]),
            )
            east, north, up = rotation @ (vector - origin_ecef)
            module_points.append(
                (
                    float(row.get("received_at") or row.get("captured_at") or 0.0),
                    float(east),
                    float(north),
                    float(up),
                )
            )
            block_points[str(row.get("block") or "unknown")][module_id].append(
                module_points[-1]
            )
        points[module_id] = module_points
        centers[module_id] = (
            median(point[1] for point in module_points),
            median(point[2] for point in module_points),
            median(point[3] for point in module_points),
        )

    availability: dict[int, dict[str, Any]] = {}
    for module_id, rows in all_by_module.items():
        qualities = Counter(int(row.get("gps_fix_quality") or 0) for row in rows)
        valid = sum(bool(row.get("gps_fix_valid")) for row in rows)
        availability[module_id] = {
            "samples": len(rows),
            "valid": valid,
            "rtk_fixed": qualities.get(4, 0),
            "rtk_float": qualities.get(5, 0),
            "other": len(rows) - qualities.get(4, 0) - qualities.get(5, 0),
            "rtk_fixed_pct": 100.0 * qualities.get(4, 0) / len(rows) if rows else 0.0,
            "median_hdop": median(row.get("gps_hdop") for row in rows),
            "median_satellites": median(row.get("gps_satellites") for row in rows),
        }
    block_centers: dict[str, dict[int, tuple[float, float, float]]] = {}
    for block, module_map in block_points.items():
        block_centers[block] = {}
        for module_id, module_points in module_map.items():
            block_centers[block][module_id] = (
                median(point[1] for point in module_points),
                median(point[2] for point in module_points),
                median(point[3] for point in module_points),
            )

    cross_block_span: dict[int, float] = {}
    for module_id in sorted(centers):
        module_centers = [
            center_map[module_id]
            for center_map in block_centers.values()
            if module_id in center_map
        ]
        cross_block_span[module_id] = max(
            (math.dist(first, second) for first in module_centers for second in module_centers),
            default=0.0,
        )

    return {
        "origin": {
            "latitude_deg": origin_latitude,
            "longitude_deg": origin_longitude,
            "altitude_m": origin_altitude,
        },
        "points": points,
        "centers": centers,
        "block_centers": block_centers,
        "cross_block_span_m": cross_block_span,
        "availability": availability,
    }


def select_coherent_rtk_geometry(
    blocks: dict[str, dict[str, Any]],
    rtk: dict[str, Any],
) -> tuple[str, dict[int, tuple[float, float, float]], list[dict[str, Any]]]:
    """Select the RTK epoch that passes an independent UWB-distance check.

    All three radio modes measured the same static layout.  Comparing their
    robust pair medians with each consecutive RTK block catches a fixed-status
    false solution without assuming a surveyed shape or tag position.
    """
    uwb_pair_values: dict[tuple[int, int], list[float]] = defaultdict(list)
    for block in blocks.values():
        for pair, keyed in block["anchor_ranges"].items():
            uwb_pair_values[pair].extend(
                float(event["distance_m"])
                for event in keyed.values()
                if finite(event.get("distance_m"))
            )
    uwb_pair_medians = {
        pair: float(np.median(values))
        for pair, values in uwb_pair_values.items()
        if values
    }
    integrity_rows = []
    for block_name, centers in sorted(rtk["block_centers"].items()):
        residuals = []
        for pair, uwb_distance in uwb_pair_medians.items():
            a, b = pair
            if a in centers and b in centers:
                residuals.append(math.dist(centers[a], centers[b]) - uwb_distance)
        rms = math.sqrt(statistics.fmean(value * value for value in residuals)) if residuals else math.inf
        maximum = max((abs(value) for value in residuals), default=math.inf)
        integrity_rows.append(
            {
                "block": block_name,
                "pairs": len(residuals),
                "rtk_vs_uwb_pair_rmse_m": rms,
                "rtk_vs_uwb_pair_max_m": maximum,
            }
        )
    usable = [
        row
        for row in integrity_rows
        if row["pairs"] == 6
        and all(module_id in rtk["block_centers"][row["block"]] for module_id in ANCHOR_IDS)
    ]
    if not usable:
        raise RuntimeError("No RTK block has a complete four-anchor geometry")
    selected = min(usable, key=lambda row: row["rtk_vs_uwb_pair_rmse_m"])
    return selected["block"], rtk["block_centers"][selected["block"]], integrity_rows


def solve_protocols(
    blocks: dict[str, dict[str, Any]],
    anchors: dict[int, tuple[float, float]],
) -> tuple[dict[str, list[tuple[float, float, float]]], dict[str, dict[str, Any]]]:
    positions: dict[str, list[tuple[float, float, float]]] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    for protocol, block in blocks.items():
        label = PROTOCOLS[protocol]
        solved: list[tuple[float, float, float]] = []
        candidate_units = 0
        complete_units = 0
        impossible = 0
        if protocol == "ds_twr":
            groups: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
            for event in block["ds"].values():
                anchor_id = int(event["anchor_id"])
                if anchor_id not in ANCHOR_INDEX:
                    continue
                frame = (int(event["seq"]) - ANCHOR_INDEX[anchor_id]) % 65536
                groups[frame][anchor_id] = event
            candidate_units = len(groups)
            for items in groups.values():
                if not all(anchor_id in items for anchor_id in ANCHOR_IDS):
                    continue
                distances = {
                    anchor_id: float(items[anchor_id]["distance_m"])
                    for anchor_id in ANCHOR_IDS
                }
                solution = trilaterate_linear(anchors, distances)
                if solution is None:
                    continue
                timestamp = statistics.fmean(
                    float(items[anchor_id]["received_at"])
                    for anchor_id in ANCHOR_IDS
                )
                solved.append((timestamp, solution[0], solution[1]))
                complete_units += 1
        else:
            groups: dict[tuple[int, int, int], dict[int, dict[str, Any]]] = defaultdict(dict)
            for event in block["tdoa"].values():
                initiator = int(event["initiator_id"])
                responder = int(event["responder_id"])
                slot = int(event.get("slot_id") or event.get("seq") or 0)
                tag = int(event.get("tag_id") or 0)
                # FlexTDOA carries all responder observations under one slot.
                # Passive DS-TWR emits the three responder exchanges as three
                # consecutive slots; responder_index recovers their common
                # independent-frame key without any PC clock synchronization.
                frame = (
                    slot - int(event.get("responder_index") or 0)
                    if protocol == "passive_ds"
                    else slot
                )
                groups[(frame, initiator, tag)][responder] = event
                if initiator in anchors and responder in anchors:
                    physical_bound = math.dist(anchors[initiator], anchors[responder])
                    if abs(float(event["diff_m"])) > physical_bound + 0.02:
                        impossible += 1
            candidate_units = len(groups)
            for items in groups.values():
                if len(items) < 3:
                    continue
                observations = sorted(
                    items.values(),
                    key=lambda event: int(event.get("responder_index") or 99),
                )
                solution = solve_tdoa(anchors, observations)
                if solution is None:
                    continue
                timestamp = max(float(event["received_at"]) for event in observations)
                solved.append((timestamp, solution[0], solution[1]))
                complete_units += 1
        positions[label] = sorted(solved)
        duration = float(block["duration_s"])
        diagnostics[label] = {
            "protocol": protocol,
            "duration_s": duration,
            "candidate_units": candidate_units,
            "complete_units": complete_units,
            "complete_pct": 100.0 * complete_units / candidate_units if candidate_units else 0.0,
            "position_rate_hz": complete_units / duration if duration else 0.0,
            "physically_impossible_observations": impossible,
            "unique_tdoa_observations": len(block["tdoa"]),
            "unique_ds_ranges": len(block["ds"]),
            **interval_metrics(solved),
        }
    return positions, diagnostics


def embedded_positions(blocks: dict[str, dict[str, Any]]) -> dict[str, list[tuple[float, float, float]]]:
    result: dict[str, list[tuple[float, float, float]]] = {}
    for protocol, block in blocks.items():
        rows = []
        for event in block["local_positions"].values():
            if finite(event.get("x_m")) and finite(event.get("y_m")):
                rows.append(
                    (
                        float(event.get("received_at") or 0.0),
                        float(event["x_m"]),
                        float(event["y_m"]),
                    )
                )
        result[PROTOCOLS[protocol]] = sorted(rows)
    return result


def anchor_pair_metrics(
    blocks: dict[str, dict[str, Any]],
    rtk_centers: dict[int, tuple[float, float, float]],
) -> list[dict[str, Any]]:
    rows = []
    for protocol, block in blocks.items():
        label = PROTOCOLS[protocol]
        for pair in sorted(block["anchor_ranges"]):
            a, b = pair
            if a not in rtk_centers or b not in rtk_centers:
                continue
            values = [
                float(event["distance_m"])
                for event in block["anchor_ranges"][pair].values()
                if finite(event.get("distance_m"))
            ]
            if not values:
                continue
            ca = rtk_centers[a]
            cb = rtk_centers[b]
            horizontal = math.hypot(ca[0] - cb[0], ca[1] - cb[1])
            distance_3d = math.dist(ca, cb)
            med = float(np.median(values))
            rows.append(
                {
                    "protocol": label,
                    "pair": f"A{a}-A{b}",
                    "n": len(values),
                    "median_m": med,
                    "std_cm": float(np.std(values, ddof=1) * 100.0) if len(values) > 1 else 0.0,
                    "rtk_horizontal_m": horizontal,
                    "rtk_3d_m": distance_3d,
                    "bias_vs_horizontal_cm": (med - horizontal) * 100.0,
                    "bias_vs_3d_cm": (med - distance_3d) * 100.0,
                }
            )
    return rows


def write_csv(path: pathlib.Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_figure(figure: plt.Figure, figure_dir: pathlib.Path, name: str) -> None:
    figure.tight_layout()
    figure.savefig(figure_dir / f"{name}.pdf", bbox_inches="tight")
    figure.savefig(figure_dir / f"{name}.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def make_figures(
    figure_dir: pathlib.Path,
    positions: dict[str, list[tuple[float, float, float]]],
    gps_points: list[tuple[float, float, float, float]],
    rtk_centers: dict[int, tuple[float, float, float]],
    truth: tuple[float, float],
    metrics: dict[str, dict[str, Any]],
    diagnostics: dict[str, dict[str, Any]],
    pair_rows: list[dict[str, Any]],
    availability: dict[int, dict[str, Any]],
) -> None:
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

    figure, axis = plt.subplots(figsize=(7.2, 5.4))
    order = [2, 3, 5, 4, 2]
    axis.plot(
        [rtk_centers[module][0] for module in order],
        [rtk_centers[module][1] for module in order],
        color="#64748b",
        linewidth=1.2,
    )
    for module_id in ANCHOR_IDS:
        east, north, _ = rtk_centers[module_id]
        axis.scatter(east, north, marker="^", s=80, color="#15803d", zorder=3)
        axis.annotate(f"A{module_id}", (east, north), xytext=(5, 5), textcoords="offset points")
    axis.scatter(truth[0], truth[1], s=55, color="#dc2626", zorder=4)
    axis.annotate("T1 RTK reference", truth, xytext=(6, 5), textcoords="offset points")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("east [m]")
    axis.set_ylabel("north [m]")
    axis.set_title("RTK-derived common analysis geometry")
    save_figure(figure, figure_dir, "01_rtk_geometry")

    figure, axes = plt.subplots(2, 2, figsize=(9.0, 7.2), sharex=True, sharey=True)
    series = list(PROTOCOLS.values()) + ["GPS RTK"]
    clouds: dict[str, list[tuple[float, float]]] = {
        label: [(point[1], point[2]) for point in positions.get(label, [])]
        for label in PROTOCOLS.values()
    }
    clouds["GPS RTK"] = [(point[1], point[2]) for point in gps_points]
    all_x = [point[0] for values in clouds.values() for point in values]
    all_y = [point[1] for values in clouds.values() for point in values]
    low_x, high_x = np.percentile(all_x, [0.5, 99.5]) if all_x else (truth[0] - 1, truth[0] + 1)
    low_y, high_y = np.percentile(all_y, [0.5, 99.5]) if all_y else (truth[1] - 1, truth[1] + 1)
    span = max(high_x - low_x, high_y - low_y, 0.5)
    center_x = (low_x + high_x) / 2.0
    center_y = (low_y + high_y) / 2.0
    for axis, label in zip(axes.flat, series):
        values = clouds[label]
        step = max(1, len(values) // 5000)
        shown = values[::step]
        axis.scatter(
            [point[0] for point in shown],
            [point[1] for point in shown],
            s=4,
            alpha=0.30,
            color=COLORS[label],
            rasterized=True,
        )
        axis.scatter(truth[0], truth[1], marker="+", s=90, linewidth=1.3, color="black")
        axis.set_title(f"{label} (n={len(values):,})")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(center_x - 0.58 * span, center_x + 0.58 * span)
        axis.set_ylim(center_y - 0.58 * span, center_y + 0.58 * span)
    for axis in axes[-1]:
        axis.set_xlabel("east [m]")
    for axis in axes[:, 0]:
        axis.set_ylabel("north [m]")
    figure.suptitle("Unfiltered static position clouds in the common RTK frame")
    save_figure(figure, figure_dir, "02_position_clouds")

    figure, axis = plt.subplots(figsize=(7.4, 4.6))
    for label, values in positions.items():
        errors = sorted(
            math.hypot(point[1] - truth[0], point[2] - truth[1]) * 100.0
            for point in values
        )
        if not errors:
            continue
        axis.plot(errors, np.linspace(0.0, 100.0, len(errors)), label=label, color=COLORS[label])
    gps_errors = sorted(
        math.hypot(point[1] - truth[0], point[2] - truth[1]) * 100.0
        for point in gps_points
    )
    axis.plot(
        gps_errors,
        np.linspace(0.0, 100.0, len(gps_errors)),
        label="GPS RTK self-dispersion",
        color=COLORS["GPS RTK"],
    )
    p99 = max(
        percentile(
            [math.hypot(point[1] - truth[0], point[2] - truth[1]) * 100.0 for point in values],
            99,
        )
        for values in list(positions.values()) + [[(p[0], p[1], p[2]) for p in gps_points]]
        if values
    )
    axis.set_xlim(0.0, max(10.0, p99 * 1.05))
    axis.set_ylim(0.0, 100.0)
    axis.set_xlabel("horizontal error / radial displacement [cm]")
    axis.set_ylabel("cumulative probability [%]")
    axis.set_title("Horizontal error CDF; RTK is the shared position reference")
    axis.legend(loc="lower right")
    save_figure(figure, figure_dir, "03_error_cdf")

    figure, axis = plt.subplots(figsize=(7.5, 4.6))
    labels = list(PROTOCOLS.values()) + ["GPS RTK"]
    values = []
    for label in labels:
        source = positions[label] if label in positions else [(p[0], p[1], p[2]) for p in gps_points]
        cx = median(point[1] for point in source)
        cy = median(point[2] for point in source)
        values.append([math.hypot(point[1] - cx, point[2] - cy) * 100.0 for point in source])
    box = axis.boxplot(
        values, tick_labels=labels, showfliers=False, patch_artist=True
    )
    for patch, label in zip(box["boxes"], labels):
        patch.set_facecolor(COLORS[label])
        patch.set_alpha(0.72)
    axis.set_ylabel("radial displacement from own median [cm]")
    axis.set_title("Static precision (outliers retained in metrics, hidden in boxplot)")
    axis.tick_params(axis="x", rotation=15)
    save_figure(figure, figure_dir, "04_precision_boxplot")

    figure, axes = plt.subplots(3, 1, figsize=(8.2, 7.2), sharex=False)
    for axis, label in zip(axes, PROTOCOLS.values()):
        source = positions[label]
        if source:
            t0 = source[0][0]
            step = max(1, len(source) // 5000)
            shown = source[::step]
            axis.plot(
                [point[0] - t0 for point in shown],
                [math.hypot(point[1] - truth[0], point[2] - truth[1]) * 100.0 for point in shown],
                linewidth=0.55,
                color=COLORS[label],
            )
        axis.set_ylabel("error [cm]")
        axis.set_title(label, loc="left")
    axes[-1].set_xlabel("time within protocol block [s]")
    figure.suptitle("Unfiltered horizontal error versus time")
    save_figure(figure, figure_dir, "05_error_time")

    figure, axes = plt.subplots(1, 2, figsize=(8.6, 4.1))
    labels = list(PROTOCOLS.values())
    rates = [diagnostics[label]["position_rate_hz"] for label in labels]
    completion = [diagnostics[label]["complete_pct"] for label in labels]
    axes[0].bar(labels, rates, color=[COLORS[label] for label in labels])
    axes[0].set_ylabel("common-solver positions/s")
    axes[0].set_title("Measured update rate")
    axes[1].bar(labels, completion, color=[COLORS[label] for label in labels])
    axes[1].set_ylabel("complete solvable units [%]")
    axes[1].set_ylim(0, 105)
    axes[1].set_title("Frame/slot completeness")
    for axis in axes:
        axis.tick_params(axis="x", rotation=18)
    save_figure(figure, figure_dir, "06_rate_delivery")

    figure, axis = plt.subplots(figsize=(9.0, 4.8))
    pairs = sorted({row["pair"] for row in pair_rows})
    width = 0.24
    x = np.arange(len(pairs))
    for index, label in enumerate(PROTOCOLS.values()):
        lookup = {row["pair"]: row for row in pair_rows if row["protocol"] == label}
        bias = [lookup.get(pair, {}).get("bias_vs_3d_cm", math.nan) for pair in pairs]
        axis.bar(x + (index - 1) * width, bias, width, label=label, color=COLORS[label])
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xticks(x, pairs)
    axis.set_ylabel("median UWB range - RTK 3-D separation [cm]")
    axis.set_title("Anchor-to-anchor range bias against RTK geometry")
    axis.legend()
    save_figure(figure, figure_dir, "07_anchor_range_bias")

    figure, axes = plt.subplots(1, 2, figsize=(8.4, 4.0))
    modules = sorted(availability)
    fixed = [availability[module]["rtk_fixed_pct"] for module in modules]
    axes[0].bar([f"T1" if module == 1 else f"A{module}" for module in modules], fixed, color=COLORS["GPS RTK"])
    axes[0].set_ylim(90, 100.3)
    axes[0].set_ylabel("RTK-fixed samples [%]")
    axes[0].set_title("RTK availability")
    gps_precision = []
    for module in modules:
        center = rtk_centers[module]
        module_points = []
        if module == 1:
            module_points = gps_points
            center = (truth[0], truth[1], center[2])
        if module_points:
            gps_precision.append(percentile([math.hypot(p[1] - center[0], p[2] - center[1]) * 100.0 for p in module_points], 95))
        else:
            gps_precision.append(math.nan)
    axes[1].axis("off")
    lines = [
        f"T1 RTK-fixed samples: {availability.get(1, {}).get('rtk_fixed', 0):,}",
        f"T1 RTK-fixed: {availability.get(1, {}).get('rtk_fixed_pct', 0):.2f}%",
        f"T1 median HDOP: {availability.get(1, {}).get('median_hdop', math.nan):.2f}",
        f"T1 median satellites: {availability.get(1, {}).get('median_satellites', math.nan):.0f}",
        f"T1 RTK-fixed P95 precision: {gps_precision[modules.index(1)]:.2f} cm",
    ]
    axes[1].text(0.05, 0.85, "\n".join(lines), va="top", family="monospace")
    axes[1].set_title("Reference stream summary")
    save_figure(figure, figure_dir, "08_rtk_quality")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "report_dir",
        type=pathlib.Path,
        nargs="?",
        default=pathlib.Path("reports/bundles/uwb_gps_rtk_comparison_20260803/analysis"),
    )
    parser.add_argument(
        "--data-dir",
        type=pathlib.Path,
        default=pathlib.Path("reports/bundles/uwb_gps_rtk_comparison_20260803/raw/data"),
    )
    args = parser.parse_args()
    report_dir = args.report_dir.resolve()
    data_dir = args.data_dir.resolve()
    figure_dir = report_dir.parent / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    capture = load_capture(data_dir)
    rtk = rtk_reference(capture["gps"])
    geometry_block, rtk_centers, rtk_integrity = select_coherent_rtk_geometry(
        capture["blocks"], rtk
    )
    anchors = {
        anchor_id: (rtk_centers[anchor_id][0], rtk_centers[anchor_id][1])
        for anchor_id in ANCHOR_IDS
    }
    # T1 remained stable across all three captures, so use all of its RTK-fixed
    # samples for the temporal reference.  Anchor coordinates deliberately use
    # the integrity-selected block because A3 entered false fixed solutions.
    truth = (rtk["centers"][1][0], rtk["centers"][1][1])
    positions, diagnostics = solve_protocols(capture["blocks"], anchors)
    embedded = embedded_positions(capture["blocks"])
    pair_rows = anchor_pair_metrics(capture["blocks"], rtk_centers)

    gps_points = rtk["points"][1]
    gps_as_positions = [(point[0], point[1], point[2]) for point in gps_points]
    metrics = {
        label: {
            **position_metrics(points, truth),
            **diagnostics[label],
        }
        for label, points in positions.items()
    }
    metrics["GPS RTK"] = {
        **position_metrics(gps_as_positions, None),
        "position_rate_hz": len(gps_as_positions)
        / sum(float(block["duration_s"]) for block in capture["blocks"].values()),
        **interval_metrics(gps_as_positions),
    }
    embedded_metrics = {}
    for label, points in embedded.items():
        duration = float(capture["blocks"][next(key for key, value in PROTOCOLS.items() if value == label)]["duration_s"])
        embedded_metrics[label] = {
            **position_metrics(points, None),
            "position_rate_hz": len(points) / duration if duration else 0.0,
            **interval_metrics(points),
        }

    geometry_rows = []
    for a_index, a in enumerate(ANCHOR_IDS):
        for b in ANCHOR_IDS[a_index + 1 :]:
            ca, cb = rtk_centers[a], rtk_centers[b]
            geometry_rows.append(
                {
                    "pair": f"A{a}-A{b}",
                    "horizontal_m": math.hypot(ca[0] - cb[0], ca[1] - cb[1]),
                    "delta_up_m": cb[2] - ca[2],
                    "distance_3d_m": math.dist(ca, cb),
                }
            )

    summary = {
        "analysis": {
            "reference": "RTK-fixed robust median; common WGS84 ECEF-to-ENU frame",
            "reference_geometry_block": geometry_block,
            "reference_geometry_selection": "minimum six-pair residual against the static UWB anchor-range consensus",
            "uwb_solver": "unfiltered, unweighted 2-D common offline solver",
            "gps_is_independent_ground_truth": False,
            "gps_accuracy_interpretation": "RTK is an in-system shared reference; its self-dispersion is precision, not absolute accuracy",
        },
        "rtk_origin": rtk["origin"],
        "rtk_centers_enu_m": {
            str(module): list(center) for module, center in rtk_centers.items()
        },
        "rtk_overall_centers_enu_m": {
            str(module): list(center) for module, center in rtk["centers"].items()
        },
        "rtk_block_centers_enu_m": {
            block: {str(module): list(center) for module, center in centers.items()}
            for block, centers in rtk["block_centers"].items()
        },
        "rtk_cross_block_span_m": {
            str(module): span for module, span in rtk["cross_block_span_m"].items()
        },
        "rtk_geometry_integrity": rtk_integrity,
        "rtk_availability": {str(key): value for key, value in rtk["availability"].items()},
        "position_metrics": metrics,
        "embedded_position_metrics": embedded_metrics,
        "protocol_diagnostics": diagnostics,
        "rtk_geometry": geometry_rows,
        "anchor_pair_metrics": pair_rows,
    }
    (report_dir / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    metric_rows = []
    for label, values in metrics.items():
        metric_rows.append({"method": label, **values})
    write_csv(report_dir / "position_metrics.csv", metric_rows)
    write_csv(report_dir / "rtk_geometry.csv", geometry_rows)
    write_csv(report_dir / "rtk_geometry_integrity.csv", rtk_integrity)
    write_csv(report_dir / "anchor_pair_metrics.csv", pair_rows)

    point_rows = []
    for label, values in positions.items():
        for timestamp, east, north in values:
            point_rows.append(
                {
                    "method": label,
                    "received_at": f"{timestamp:.6f}",
                    "east_m": f"{east:.6f}",
                    "north_m": f"{north:.6f}",
                    "error_to_rtk_m": f"{math.hypot(east - truth[0], north - truth[1]):.6f}",
                }
            )
    write_csv(report_dir / "common_solver_positions.csv", point_rows)

    make_figures(
        figure_dir,
        positions,
        gps_points,
        rtk_centers,
        truth,
        metrics,
        diagnostics,
        pair_rows,
        rtk["availability"],
    )
    print(json.dumps(summary["position_metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
