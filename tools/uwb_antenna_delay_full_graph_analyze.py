#!/usr/bin/env python3
"""Estimate per-module DW3000 antenna-delay corrections from a known layout."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np


METERS_PER_DTU = 15.650040064102564e-12 * 299702547.0


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_samples(path: Path) -> dict[tuple[int, int], list[float]]:
    samples: dict[tuple[int, int], list[float]] = defaultdict(list)
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            item = json.loads(line)
            if item.get("kind") == "ds_range":
                endpoints = (int(item["tag_id"]), int(item["anchor_id"]))
            elif item.get("kind") == "anchor_range":
                endpoints = (
                    int(item["anchor_a_id"]), int(item["anchor_b_id"])
                )
            else:
                continue
            distance_m = float(item["distance_m"])
            if math.isfinite(distance_m) and distance_m > 0:
                samples[tuple(sorted(endpoints))].append(distance_m)
    return dict(samples)


def module_delays(metadata: dict[str, Any]) -> dict[int, int]:
    delays: dict[int, int] = {}
    for status in metadata.get("initial_status", []):
        module_id = int(status.get("module_id") or 0)
        delay = status.get("uwb_active_antenna_delay")
        if module_id > 0 and delay is not None:
            delays[module_id] = int(delay)
    return delays


def robust_sigma(values: list[float]) -> float:
    center = median(values)
    return 1.4826 * median([abs(value - center) for value in values])


def rounded_i32(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def fmt_cm(value_m: float) -> str:
    return f"{value_m * 100.0:.2f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--bootstrap", type=int, default=5000)
    parser.add_argument("--seed", type=int, default=20260731)
    args = parser.parse_args()

    metadata = load_json(args.metadata)
    ground_truth = metadata.get("ground_truth") or {}
    coordinates = {
        int(module_id): tuple(map(float, point))
        for module_id, point in (ground_truth.get("anchors_m") or {}).items()
    }
    tag_point = ground_truth.get("tag_m")
    if tag_point is None:
        raise SystemExit("metadata has no tag_m ground truth")
    tag_ids = sorted(
        {
            int(json.loads(line).get("tag_id"))
            for line in args.events.open("r", encoding="utf-8")
            if '"kind":"ds_range"' in line
        }
    )
    if len(tag_ids) != 1:
        raise SystemExit(f"expected exactly one tag ID, got {tag_ids}")
    coordinates[tag_ids[0]] = tuple(map(float, tag_point))

    samples = load_samples(args.events)
    module_ids = sorted(coordinates)
    expected_pairs = [
        (module_ids[i], module_ids[j])
        for i in range(len(module_ids))
        for j in range(i + 1, len(module_ids))
    ]
    missing = [pair for pair in expected_pairs if not samples.get(pair)]
    if missing:
        raise SystemExit(f"missing calibration links: {missing}")

    rows: list[list[float]] = []
    errors_dtu: list[float] = []
    links: list[dict[str, Any]] = []
    for pair in expected_pairs:
        values = samples[pair]
        known_m = math.dist(coordinates[pair[0]], coordinates[pair[1]])
        center_m = median(values)
        error_m = center_m - known_m
        rows.append([1.0 if module_id in pair else 0.0 for module_id in module_ids])
        errors_dtu.append(error_m / METERS_PER_DTU)
        links.append(
            {
                "pair": f"M{pair[0]}-M{pair[1]}",
                "module_a": pair[0],
                "module_b": pair[1],
                "samples": len(values),
                "known_m": known_m,
                "median_m": center_m,
                "mean_m": mean(values),
                "std_cm": pstdev(values) * 100.0,
                "robust_sigma_cm": robust_sigma(values) * 100.0,
                "raw_error_cm": error_m * 100.0,
                "raw_error_dtu": error_m / METERS_PER_DTU,
            }
        )

    matrix = np.asarray(rows, dtype=float)
    observed = np.asarray(errors_dtu, dtype=float)
    correction_float, _, rank, singular = np.linalg.lstsq(
        matrix, observed, rcond=None
    )
    if rank != len(module_ids):
        raise SystemExit(f"calibration matrix rank {rank}, expected {len(module_ids)}")
    correction_rounded = np.asarray(
        [rounded_i32(value) for value in correction_float], dtype=int
    )

    rng = np.random.default_rng(args.seed)
    bootstrap_errors = np.empty((args.bootstrap, len(expected_pairs)), dtype=float)
    for column, pair in enumerate(expected_pairs):
        values = np.asarray(samples[pair], dtype=float)
        indexes = rng.integers(0, len(values), size=(args.bootstrap, len(values)))
        medians = np.median(values[indexes], axis=1)
        known_m = math.dist(coordinates[pair[0]], coordinates[pair[1]])
        bootstrap_errors[:, column] = (medians - known_m) / METERS_PER_DTU
    pseudo_inverse = np.linalg.pinv(matrix)
    bootstrap_corrections = bootstrap_errors @ pseudo_inverse.T

    predicted = matrix @ correction_float
    rounded_predicted = matrix @ correction_rounded
    fit_residual_dtu = observed - predicted
    rounded_residual_dtu = observed - rounded_predicted
    for index, link in enumerate(links):
        link["fit_residual_cm"] = fit_residual_dtu[index] * METERS_PER_DTU * 100.0
        link["rounded_residual_cm"] = (
            rounded_residual_dtu[index] * METERS_PER_DTU * 100.0
        )

    delays = module_delays(metadata)
    modules: list[dict[str, Any]] = []
    for index, module_id in enumerate(module_ids):
        old_delay = delays.get(module_id)
        rounded = int(correction_rounded[index])
        low, high = np.percentile(bootstrap_corrections[:, index], [2.5, 97.5])
        modules.append(
            {
                "module_id": module_id,
                "old_delay": old_delay,
                "old_delay_hex": f"0x{old_delay:04x}" if old_delay is not None else None,
                "correction_float_dtu": float(correction_float[index]),
                "correction_dtu": rounded,
                "correction_cm_equivalent": rounded * METERS_PER_DTU * 100.0,
                "bootstrap_ci95_dtu": [float(low), float(high)],
                "proposed_delay": old_delay + rounded if old_delay is not None else None,
                "proposed_delay_hex": (
                    f"0x{old_delay + rounded:04x}" if old_delay is not None else None
                ),
            }
        )

    raw_errors_cm = observed * METERS_PER_DTU * 100.0
    corrected_errors_cm = rounded_residual_dtu * METERS_PER_DTU * 100.0
    summary = {
        "schema_version": 1,
        "events_file": str(args.events),
        "metadata_file": str(args.metadata),
        "coordinate_convention": ground_truth.get("coordinate_convention"),
        "survey_tolerance_m": ground_truth.get("survey_tolerance_m"),
        "meters_per_dtu": METERS_PER_DTU,
        "module_ids": module_ids,
        "link_count": len(links),
        "sample_count": sum(len(samples[pair]) for pair in expected_pairs),
        "matrix_rank": int(rank),
        "matrix_condition": float(singular[0] / singular[-1]),
        "center_method": "median per link; equal link weight",
        "bootstrap_replicates": args.bootstrap,
        "raw_link_error_rms_cm": float(np.sqrt(np.mean(raw_errors_cm**2))),
        "predicted_rounded_link_error_rms_cm": float(
            np.sqrt(np.mean(corrected_errors_cm**2))
        ),
        "predicted_rounded_link_error_max_cm": float(
            np.max(np.abs(corrected_errors_cm))
        ),
        "modules": modules,
        "links": links,
    }

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (output_dir / "link_metrics.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(links[0]))
        writer.writeheader()
        writer.writerows(links)

    module_rows = "\n".join(
        "| M{module_id} | `{old_delay_hex}` | {correction_float_dtu:+.2f} | "
        "{correction_dtu:+d} | [{low:+.2f}, {high:+.2f}] | `{proposed_delay_hex}` |".format(
            **item,
            low=item["bootstrap_ci95_dtu"][0],
            high=item["bootstrap_ci95_dtu"][1],
        )
        for item in modules
    )
    link_rows = "\n".join(
        f"| {item['pair']} | {item['samples']} | {item['known_m']:.6f} | "
        f"{item['median_m']:.4f} | {item['raw_error_cm']:+.2f} | "
        f"{item['std_cm']:.2f} | {item['rounded_residual_cm']:+.2f} |"
        for item in links
    )
    readme = f"""# Full-graph Native DS-TWR antenna-delay calibration

This is a dry-run calibration. No antenna delay was written to a module.

- Geometry: {ground_truth.get('coordinate_convention', '-')}
- Samples: {summary['sample_count']} across all {summary['link_count']} links
- Estimator: median per link, equal-weight complete-graph least squares
- Matrix rank/condition: {rank}/{singular[0] / singular[-1]:.3f}
- Raw link-error RMS: {summary['raw_link_error_rms_cm']:.2f} cm
- Predicted RMS after rounded corrections: {summary['predicted_rounded_link_error_rms_cm']:.2f} cm
- Predicted maximum residual: {summary['predicted_rounded_link_error_max_cm']:.2f} cm

## Proposed module corrections

| Module | Current | fitted DTU | rounded DTU | bootstrap 95% DTU | proposed |
|---|---:|---:|---:|---:|---:|
{module_rows}

Positive corrections increase the configured antenna-delay value. The confidence
intervals quantify capture noise only; the stated survey tolerance and link-specific
multipath are not included. Apply all five values as one set, reboot, and validate
with another unchanged capture before accepting them.

## Link diagnostics

| Link | n | known m | median m | raw error cm | std cm | residual after rounded cm |
|---|---:|---:|---:|---:|---:|---:|
{link_rows}
"""
    (output_dir / "README.md").write_text(readme, encoding="utf-8")

    checksum_paths = [summary_path, output_dir / "link_metrics.csv", output_dir / "README.md"]
    checksum_lines = []
    for path in checksum_paths:
        checksum_lines.append(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}"
        )
    (output_dir / "SHA256SUMS").write_text(
        "\n".join(checksum_lines) + "\n", encoding="utf-8"
    )

    print(json.dumps({
        "modules": modules,
        "raw_link_error_rms_cm": summary["raw_link_error_rms_cm"],
        "predicted_rounded_link_error_rms_cm": summary[
            "predicted_rounded_link_error_rms_cm"
        ],
        "predicted_rounded_link_error_max_cm": summary[
            "predicted_rounded_link_error_max_cm"
        ],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
