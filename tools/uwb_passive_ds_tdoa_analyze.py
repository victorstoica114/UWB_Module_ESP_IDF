#!/usr/bin/env python3
"""Analyze isolated three-packet Passive DS-TDoA captures.

The embedded position stream uses the live, radio-derived anchor geometry.
This analyzer also solves the same observations against the surveyed 3 m x 3 m
geometry.  Keeping those two results separate prevents anchor self-localization
error from being attributed to the passive ranging method.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
from collections import defaultdict
from typing import Any, Iterable

from uwb_compare_analyze import solve_tdoa


ANCHOR_IDS = (2, 3, 4, 5)
ANCHORS = {
    2: (0.0, 0.0),
    3: (0.0, 3.0),
    4: (3.0, 0.0),
    5: (3.0, 3.0),
}
REFERENCE = (1.5, 1.5)
def read_events(path: pathlib.Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def duration_seconds(events: list[dict[str, Any]]) -> float:
    times = [float(event["received_at"]) for event in events]
    return max(times) - min(times) if len(times) >= 2 else 0.0


def solve_rolling_surveyed(
    observations: list[dict[str, Any]],
    freshness_seconds: float,
) -> list[dict[str, Any]]:
    latest: dict[tuple[int, int], dict[str, Any]] = {}
    positions: list[dict[str, Any]] = []
    seed = REFERENCE
    for event in sorted(observations, key=lambda item: float(item["received_at"])):
        now = float(event["received_at"])
        key = (int(event["initiator_id"]), int(event["responder_id"]))
        latest[key] = event
        fresh = [
            item
            for item in latest.values()
            if now - float(item["received_at"]) <= freshness_seconds
        ]
        solution = solve_tdoa(ANCHORS, fresh, seed)
        if solution is None:
            continue
        seed = solution
        positions.append(
            {
                "received_at": now,
                "slot_id": int(event["slot_id"]),
                "x_m": solution[0],
                "y_m": solution[1],
                "observation_count": len(fresh),
            }
        )
    return positions


def solve_three_slot_batches_surveyed(
    observations: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Solve disjoint three-radio-slot batches without mixing later data."""
    frames: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for event in observations:
        slot_id = int(event["slot_id"])
        frames[slot_id // 3].append(event)

    positions: list[dict[str, Any]] = []
    seed = REFERENCE
    for frame_id in sorted(frames):
        events = frames[frame_id]
        solution = solve_tdoa(ANCHORS, events, seed)
        if solution is None:
            continue
        seed = solution
        positions.append(
            {
                "received_at": max(float(item["received_at"]) for item in events),
                "frame_id": frame_id,
                "x_m": solution[0],
                "y_m": solution[1],
                "observation_count": len(events),
            }
        )
    return positions


def position_metrics(
    rows: list[dict[str, Any]], duration: float
) -> dict[str, float | int]:
    if not rows:
        return {"count": 0, "rate_hz": 0.0}
    xs = [float(row["x_m"]) for row in rows]
    ys = [float(row["y_m"]) for row in rows]
    errors_cm = [
        math.hypot(x - REFERENCE[0], y - REFERENCE[1]) * 100.0
        for x, y in zip(xs, ys)
    ]
    return {
        "count": len(rows),
        "rate_hz": len(rows) / duration if duration > 0.0 else 0.0,
        "mean_x_m": statistics.fmean(xs),
        "mean_y_m": statistics.fmean(ys),
        "std_x_cm": statistics.stdev(xs) * 100.0 if len(xs) > 1 else 0.0,
        "std_y_cm": statistics.stdev(ys) * 100.0 if len(ys) > 1 else 0.0,
        "bias_cm": math.hypot(
            statistics.fmean(xs) - REFERENCE[0],
            statistics.fmean(ys) - REFERENCE[1],
        )
        * 100.0,
        "rmse_cm": math.sqrt(statistics.fmean(value**2 for value in errors_cm)),
        "p95_cm": percentile(errors_cm, 0.95),
        "max_cm": max(errors_cm),
    }


def path_metrics(observations: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for initiator_id in ANCHOR_IDS:
        for responder_id in ANCHOR_IDS:
            if initiator_id == responder_id:
                continue
            values = [
                float(event["diff_m"]) * 100.0
                for event in observations
                if int(event["initiator_id"]) == initiator_id
                and int(event["responder_id"]) == responder_id
            ]
            if not values:
                continue
            result.append(
                {
                    "initiator_id": initiator_id,
                    "responder_id": responder_id,
                    "count": len(values),
                    "mean_cm": statistics.fmean(values),
                    "std_cm": statistics.stdev(values) if len(values) > 1 else 0.0,
                    "p95_abs_cm": percentile([abs(value) for value in values], 0.95),
                }
            )
    return result


def write_csv(path: pathlib.Path, rows: Iterable[dict[str, Any]]) -> None:
    materialized = list(rows)
    if not materialized:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(materialized[0]))
        writer.writeheader()
        writer.writerows(materialized)


def analyze_capture(
    capture: pathlib.Path,
    output_directory: pathlib.Path,
    freshness_seconds: float,
) -> dict[str, Any]:
    events = read_events(capture)
    observations = [
        event for event in events if event.get("kind") == "tdoa_observation"
    ]
    embedded = [event for event in events if event.get("kind") == "local_position"]
    embedded_independent = [
        event for event in embedded if bool(event.get("independent_frame"))
    ]
    duration = duration_seconds(events)
    surveyed_rolling = solve_rolling_surveyed(observations, freshness_seconds)
    surveyed_three_slot = solve_three_slot_batches_surveyed(observations)

    stem = capture.stem
    write_csv(output_directory / f"{stem}_surveyed_rolling.csv", surveyed_rolling)
    write_csv(
        output_directory / f"{stem}_surveyed_three_slot.csv",
        surveyed_three_slot,
    )
    write_csv(output_directory / f"{stem}_directed_paths.csv", path_metrics(observations))

    return {
        "capture": str(capture),
        "duration_sec": duration,
        "reference_geometry": {
            "anchors_m": {str(key): value for key, value in ANCHORS.items()},
            "tag_m": REFERENCE,
        },
        "observation_count": len(observations),
        "observation_rate_hz": len(observations) / duration if duration > 0.0 else 0.0,
        "embedded_live_geometry_all": position_metrics(embedded, duration),
        "embedded_live_geometry_independent": position_metrics(
            embedded_independent, duration
        ),
        "surveyed_geometry_rolling": position_metrics(surveyed_rolling, duration),
        "surveyed_geometry_disjoint_three_slot_batches": position_metrics(
            surveyed_three_slot, duration
        ),
        "directed_paths": path_metrics(observations),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("captures", nargs="+", type=pathlib.Path)
    parser.add_argument("--output-directory", type=pathlib.Path)
    parser.add_argument("--freshness-seconds", type=float, default=0.2)
    args = parser.parse_args()

    output_directory = args.output_directory or args.captures[0].parent
    output_directory.mkdir(parents=True, exist_ok=True)
    summaries = [
        analyze_capture(capture, output_directory, args.freshness_seconds)
        for capture in args.captures
    ]
    output_path = output_directory / "passive_ds_tdoa_surveyed_summary.json"
    output_path.write_text(
        json.dumps(summaries, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for summary in summaries:
        rolling = summary["surveyed_geometry_rolling"]
        embedded = summary["embedded_live_geometry_all"]
        print(
            f"{pathlib.Path(summary['capture']).name}: "
            f"surveyed RMSE {rolling.get('rmse_cm', math.nan):.2f} cm, "
            f"P95 {rolling.get('p95_cm', math.nan):.2f} cm; "
            f"live-geometry RMSE {embedded.get('rmse_cm', math.nan):.2f} cm"
        )
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
