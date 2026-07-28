#!/usr/bin/env python3
"""Gate a Passive DS-TWR candidate against a same-geometry control capture.

Only independent-frame position events are used when the V3 telemetry flag is
available. A moving-block bootstrap preserves short-range temporal correlation
better than resampling individual positions.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from typing import Any

import numpy as np


def load_errors(
    path: pathlib.Path, truth_x: float, truth_y: float, raw: bool
) -> np.ndarray:
    x_key = "raw_x_m" if raw else "x_m"
    y_key = "raw_y_m" if raw else "y_m"
    errors: list[float] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            item: dict[str, Any] = json.loads(line)
            if item.get("kind") != "local_position":
                continue
            if item.get("independent_frame") is False:
                continue
            x = item.get(x_key)
            y = item.get(y_key)
            if not isinstance(x, (int, float)) or not isinstance(
                y, (int, float)
            ):
                continue
            if not math.isfinite(float(x)) or not math.isfinite(float(y)):
                continue
            errors.append(
                100.0 * math.hypot(float(x) - truth_x, float(y) - truth_y)
            )
    if not errors:
        raise ValueError(f"no usable independent positions in {path}")
    return np.asarray(errors, dtype=float)


def metrics(errors_cm: np.ndarray) -> dict[str, float]:
    return {
        "rmse_cm": float(np.sqrt(np.mean(np.square(errors_cm)))),
        "p95_cm": float(np.percentile(errors_cm, 95)),
    }


def moving_block_sample(
    values: np.ndarray, block_len: int, rng: np.random.Generator
) -> np.ndarray:
    count = values.size
    block_len = max(1, min(block_len, count))
    block_count = math.ceil(count / block_len)
    starts = rng.integers(0, count, size=block_count)
    offsets = np.arange(block_len)
    indices = (starts[:, None] + offsets[None, :]) % count
    return values[indices.ravel()[:count]]


def bootstrap_metrics(
    errors_cm: np.ndarray, iterations: int, block_len: int, seed: int
) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    rmse = np.empty(iterations, dtype=float)
    p95 = np.empty(iterations, dtype=float)
    for index in range(iterations):
        sample = moving_block_sample(errors_cm, block_len, rng)
        result = metrics(sample)
        rmse[index] = result["rmse_cm"]
        p95[index] = result["p95_cm"]
    return {"rmse_cm": rmse, "p95_cm": p95}


def confidence_interval(
    samples: np.ndarray, confidence: float
) -> tuple[float, float]:
    tail = 50.0 * (1.0 - confidence)
    return (
        float(np.percentile(samples, tail)),
        float(np.percentile(samples, 100.0 - tail)),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", required=True, type=pathlib.Path)
    parser.add_argument("--candidate", required=True, type=pathlib.Path)
    parser.add_argument("--truth-x", required=True, type=float)
    parser.add_argument("--truth-y", required=True, type=float)
    parser.add_argument("--raw", action="store_true")
    parser.add_argument("--confidence", type=float, default=0.95)
    parser.add_argument("--bootstrap", type=int, default=5000)
    parser.add_argument("--block-length", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    if not 0.5 < args.confidence < 1.0:
        parser.error("--confidence must be between 0.5 and 1")
    if args.bootstrap < 100:
        parser.error("--bootstrap must be at least 100")
    if args.block_length < 1:
        parser.error("--block-length must be positive")

    control_errors = load_errors(
        args.control, args.truth_x, args.truth_y, args.raw
    )
    candidate_errors = load_errors(
        args.candidate, args.truth_x, args.truth_y, args.raw
    )
    control_point = metrics(control_errors)
    candidate_point = metrics(candidate_errors)
    control_bootstrap = bootstrap_metrics(
        control_errors, args.bootstrap, args.block_length, args.seed
    )
    candidate_bootstrap = bootstrap_metrics(
        candidate_errors, args.bootstrap, args.block_length, args.seed + 1
    )

    gates: dict[str, dict[str, float | bool | list[float]]] = {}
    accepted = True
    for metric in ("rmse_cm", "p95_cm"):
        control_ci = confidence_interval(
            control_bootstrap[metric], args.confidence
        )
        candidate_ci = confidence_interval(
            candidate_bootstrap[metric], args.confidence
        )
        metric_accepted = candidate_point[metric] <= control_ci[1]
        accepted = accepted and metric_accepted
        gates[metric] = {
            "control": control_point[metric],
            "control_ci": [control_ci[0], control_ci[1]],
            "candidate": candidate_point[metric],
            "candidate_ci": [candidate_ci[0], candidate_ci[1]],
            "control_upper_limit": control_ci[1],
            "accepted": metric_accepted,
        }

    result = {
        "accepted": accepted,
        "rule": (
            "candidate point RMSE and P95 must each be no greater than the "
            "upper confidence bound of the same-geometry control"
        ),
        "confidence": args.confidence,
        "bootstrap_iterations": args.bootstrap,
        "moving_block_length": args.block_length,
        "position_source": "raw" if args.raw else "filtered",
        "control_count": int(control_errors.size),
        "candidate_count": int(candidate_errors.size),
        "gates": gates,
    }
    payload = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
