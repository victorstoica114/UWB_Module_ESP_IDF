#!/usr/bin/env python3
"""Audit channel-9 antenna delay and per-link Native DS-TWR bias.

This tool deliberately refuses to turn a rank-deficient star capture into five
per-device antenna-delay values.  It fits device terms only when the observed
link graph is identifiable, reports link-specific residuals separately, and
uses an independent capture only for validation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
from collections import defaultdict
from typing import Any, Sequence

from uwb_dynamic_static_report import add_enu, gps_origin, json_clean, load_capture
from uwb_localization_baseline import (
    mean,
    median,
    percentile,
    range_error_rows,
)


METERS_PER_DTU = 15.650040064102564e-12 * 299702547.0


def matrix_rank(rows: Sequence[Sequence[float]], tolerance: float = 1e-12) -> int:
    matrix = [list(map(float, row)) for row in rows]
    if not matrix:
        return 0
    row_count = len(matrix)
    column_count = len(matrix[0])
    rank = 0
    for column in range(column_count):
        pivot = max(
            range(rank, row_count), key=lambda row: abs(matrix[row][column])
        )
        if abs(matrix[pivot][column]) <= tolerance:
            continue
        matrix[rank], matrix[pivot] = matrix[pivot], matrix[rank]
        pivot_value = matrix[rank][column]
        for item in range(column, column_count):
            matrix[rank][item] /= pivot_value
        for row in range(row_count):
            if row == rank:
                continue
            factor = matrix[row][column]
            for item in range(column, column_count):
                matrix[row][item] -= factor * matrix[rank][item]
        rank += 1
        if rank == row_count:
            break
    return rank


def solve_square_system(
    matrix: Sequence[Sequence[float]], vector: Sequence[float], tolerance: float = 1e-12
) -> list[float]:
    size = len(vector)
    augmented = [list(map(float, matrix[row])) + [float(vector[row])] for row in range(size)]
    for column in range(size):
        pivot = max(
            range(column, size), key=lambda row: abs(augmented[row][column])
        )
        if abs(augmented[pivot][column]) <= tolerance:
            raise ValueError("singular calibration system")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        pivot_value = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= pivot_value
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][size] for row in range(size)]


def least_squares(
    matrix: Sequence[Sequence[float]], observed: Sequence[float]
) -> list[float]:
    columns = len(matrix[0])
    normal = [
        [
            sum(float(row[first]) * float(row[second]) for row in matrix)
            for second in range(columns)
        ]
        for first in range(columns)
    ]
    right = [
        sum(float(row[column]) * float(value) for row, value in zip(matrix, observed))
        for column in range(columns)
    ]
    return solve_square_system(normal, right)


def parse_link_bias(value: str) -> tuple[str, int]:
    try:
        link, bias = value.split("=", 1)
        endpoints = sorted(
            int(item.lstrip("Mm")) for item in link.replace("-", ":").split(":")
        )
        if len(endpoints) != 2 or endpoints[0] == endpoints[1]:
            raise ValueError
        return f"M{endpoints[0]}-M{endpoints[1]}", int(bias)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "link bias must look like M1-M2=78 (millimetres)"
        ) from error


def grouped_errors(rows: Sequence[dict[str, Any]]) -> dict[str, list[float]]:
    result: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        result[str(row["link"])].append(float(row["error_m"]))
    return dict(result)


def error_summary(values: Sequence[float]) -> dict[str, Any]:
    absolute = [abs(value) for value in values]
    return {
        "samples": len(values),
        "mean_bias_m": mean(values),
        "median_bias_m": median(values),
        "std_m": statistics.stdev(values) if len(values) > 1 else 0.0,
        "rmse_m": math.sqrt(mean(value * value for value in values)),
        "p95_abs_m": percentile(absolute, 95),
        "max_abs_m": max(absolute),
    }


def link_endpoints(link: str) -> tuple[int, int]:
    first, second = link.replace("M", "").split("-", 1)
    return int(first), int(second)


def fit_device_terms(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped = grouped_errors(rows)
    modules = sorted(
        {
            module
            for link in grouped
            for module in link_endpoints(link)
        }
    )
    matrix_rows = []
    observed = []
    link_order = sorted(grouped)
    for link in link_order:
        endpoints = link_endpoints(link)
        matrix_rows.append(
            [1.0 if module in endpoints else 0.0 for module in modules]
        )
        observed.append(median(grouped[link]))
    rank = matrix_rank(matrix_rows)
    identifiable = bool(modules) and rank == len(modules)
    output: dict[str, Any] = {
        "model": "link_error_ij = device_bias_i + device_bias_j",
        "module_ids": modules,
        "observed_links": link_order,
        "link_count": len(link_order),
        "matrix_rank": rank,
        "unknown_count": len(modules),
        "identifiable": identifiable,
    }
    if not identifiable:
        output.update(
            {
                "device_biases": None,
                "antenna_delay_corrections": None,
                "reason": (
                    "observed link graph is rank deficient; per-device antenna "
                    "delay would be arbitrary"
                ),
            }
        )
        return output
    solution = least_squares(matrix_rows, observed)
    predicted = [
        sum(coefficient * value for coefficient, value in zip(row, solution))
        for row in matrix_rows
    ]
    residual = [value - estimate for value, estimate in zip(observed, predicted)]
    output.update(
        {
            "device_biases": {
                str(module): solution[index]
                for index, module in enumerate(modules)
            },
            "antenna_delay_corrections": {
                str(module): int(round(solution[index] / METERS_PER_DTU))
                for index, module in enumerate(modules)
            },
            "link_fit_residual_rms_m": float(
                math.sqrt(mean(value * value for value in residual))
            ),
            "link_fit_residual_max_m": max(abs(value) for value in residual),
        }
    )
    return output


def summarize_links(
    calibration_rows: Sequence[dict[str, Any]],
    validation_rows: Sequence[dict[str, Any]],
    applied_bias_mm: dict[str, int],
) -> list[dict[str, Any]]:
    calibration = grouped_errors(calibration_rows)
    validation = grouped_errors(validation_rows)
    links = sorted(set(calibration) | set(validation) | set(applied_bias_mm))
    result = []
    for link in links:
        calibration_metrics = (
            error_summary(calibration[link]) if calibration.get(link) else None
        )
        validation_metrics = (
            error_summary(validation[link]) if validation.get(link) else None
        )
        recommended_mm = (
            int(round(1000.0 * calibration_metrics["median_bias_m"]))
            if calibration_metrics is not None
            else None
        )
        result.append(
            {
                "link": link,
                "calibration": calibration_metrics,
                "direct_rtk_recommended_bias_mm": recommended_mm,
                "applied_bias_mm": applied_bias_mm.get(link),
                "validation": validation_metrics,
                "validation_median_abs_mm": (
                    abs(1000.0 * validation_metrics["median_bias_m"])
                    if validation_metrics is not None
                    else None
                ),
            }
        )
    return result


def historical_delay_evidence(
    fit_path: pathlib.Path | None, validation_path: pathlib.Path | None
) -> dict[str, Any]:
    if fit_path is None:
        return {"available": False}
    fit = json.loads(fit_path.read_text(encoding="utf-8"))
    validation = (
        json.loads(validation_path.read_text(encoding="utf-8"))
        if validation_path is not None
        else None
    )
    return {
        "available": True,
        "fit_path": str(fit_path),
        "validation_path": str(validation_path) if validation_path else None,
        "fit_link_count": fit.get("link_count"),
        "fit_matrix_rank": fit.get("matrix_rank"),
        "fit_matrix_condition": fit.get("matrix_condition"),
        "proposed_delays": {
            str(item["module_id"]): item.get("proposed_delay")
            for item in fit.get("modules", [])
        },
        "fit_predicted_link_rms_cm": fit.get(
            "predicted_rounded_link_error_rms_cm"
        ),
        "independent_validation_link_rms_cm": (
            validation.get("raw_link_error_rms_cm") if validation else None
        ),
        "independent_validation_link_max_cm": (
            max(
                abs(float(link.get("raw_error_cm", 0.0)))
                for link in validation.get("links", [])
            )
            if validation and validation.get("links")
            else None
        ),
    }


def write_csv(path: pathlib.Path, links: Sequence[dict[str, Any]]) -> None:
    rows = []
    for item in links:
        calibration = item.get("calibration") or {}
        validation = item.get("validation") or {}
        rows.append(
            {
                "link": item["link"],
                "calibration_samples": calibration.get("samples"),
                "calibration_median_bias_mm": (
                    1000.0 * calibration["median_bias_m"]
                    if calibration
                    else None
                ),
                "calibration_p95_abs_mm": (
                    1000.0 * calibration["p95_abs_m"] if calibration else None
                ),
                "direct_rtk_recommended_bias_mm": item.get(
                    "direct_rtk_recommended_bias_mm"
                ),
                "applied_bias_mm": item.get("applied_bias_mm"),
                "validation_samples": validation.get("samples"),
                "validation_mean_bias_mm": (
                    1000.0 * validation["mean_bias_m"] if validation else None
                ),
                "validation_median_bias_mm": (
                    1000.0 * validation["median_bias_m"] if validation else None
                ),
                "validation_rmse_mm": (
                    1000.0 * validation["rmse_m"] if validation else None
                ),
                "validation_p95_abs_mm": (
                    1000.0 * validation["p95_abs_m"] if validation else None
                ),
            }
        )
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def build_readme(summary: dict[str, Any]) -> str:
    device = summary["current_capture_device_fit"]
    rows = []
    for item in summary["links"]:
        calibration = item.get("calibration") or {}
        validation = item.get("validation") or {}
        rows.append(
            "| {link} | {calibration:+.1f} | {applied} | {validation:+.1f} | {p95:.1f} |".format(
                link=item["link"],
                calibration=1000.0 * calibration.get("median_bias_m", math.nan),
                applied=(
                    str(item["applied_bias_mm"])
                    if item.get("applied_bias_mm") is not None
                    else "-"
                ),
                validation=1000.0 * validation.get("median_bias_m", math.nan),
                p95=1000.0 * validation.get("p95_abs_m", math.nan),
            )
        )
    return "\n".join(
        [
            "# Channel 9 calibration audit",
            "",
            f"Current RTK link graph rank: **{device['matrix_rank']}/{device['unknown_count']}**.",
            f"Per-device antenna-delay identifiable from this capture: **{device['identifiable']}**.",
            "",
            "| Link | calibration median mm | applied mm | validation median mm | validation P95 abs mm |",
            "|---|---:|---:|---:|---:|",
            *rows,
            "",
            "The current star capture contains only M1-to-anchor links, so it cannot uniquely separate five device antenna delays. Existing antenna delays are retained from the earlier full 10-link calibration.",
            "",
            "The independent validation confirms the link correction only at one stationary tag location. It is not evidence that the same link bias is universal during motion or at other positions.",
            "",
        ]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=pathlib.Path, required=True)
    parser.add_argument("--validation", type=pathlib.Path, required=True)
    parser.add_argument("--tag-id", type=int, default=1)
    parser.add_argument("--rtk-max-age-ms", type=float, default=750.0)
    parser.add_argument(
        "--applied-link-bias",
        action="append",
        type=parse_link_bias,
        default=[],
        metavar="M1-M2=MM",
    )
    parser.add_argument("--historical-full-graph", type=pathlib.Path)
    parser.add_argument("--historical-full-graph-validation", type=pathlib.Path)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    calibration = load_capture(args.calibration.resolve(), args.tag_id)
    validation = load_capture(args.validation.resolve(), args.tag_id)
    add_enu([calibration, validation], gps_origin([calibration, validation]))
    calibration_rows = range_error_rows(calibration, args.rtk_max_age_ms)
    validation_rows = range_error_rows(validation, args.rtk_max_age_ms)
    applied_bias = dict(args.applied_link_bias)
    links = summarize_links(calibration_rows, validation_rows, applied_bias)
    device_fit = fit_device_terms(calibration_rows)
    history = historical_delay_evidence(
        args.historical_full_graph.resolve()
        if args.historical_full_graph
        else None,
        args.historical_full_graph_validation.resolve()
        if args.historical_full_graph_validation
        else None,
    )
    validation_medians = [
        float(item["validation_median_abs_mm"])
        for item in links
        if item.get("validation_median_abs_mm") is not None
    ]
    summary = {
        "schema_version": 1,
        "channel": 9,
        "calibration_capture": str(args.calibration.resolve()),
        "validation_capture": str(args.validation.resolve()),
        "rtk_max_age_ms": args.rtk_max_age_ms,
        "calibration_range_samples": len(calibration_rows),
        "validation_range_samples": len(validation_rows),
        "current_capture_device_fit": device_fit,
        "historical_full_graph_evidence": history,
        "links": links,
        "decision": {
            "change_antenna_delays": False,
            "reason": (
                "current RTK star graph is not identifiable per device; keep the "
                "independently validated full-graph delays"
            ),
            "static_link_bias_validation_pass": bool(validation_medians)
            and max(validation_medians) <= 20.0,
            "static_link_bias_validation_limit_mm": 20.0,
            "universal_dynamic_link_bias_validated": False,
            "next_required_capture": (
                "multi-position channel-9 range capture with high-rate RTK; include "
                "anchor-to-anchor links when estimating per-device delays"
            ),
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "calibration_summary.json").write_text(
        json.dumps(json_clean(summary), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(args.output_dir / "link_metrics.csv", links)
    (args.output_dir / "README.md").write_text(
        build_readme(summary), encoding="utf-8"
    )
    print(json.dumps(summary["decision"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
