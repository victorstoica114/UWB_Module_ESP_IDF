#!/usr/bin/env python3
"""Build a reproducible static Native DS-TWR precision reference."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import pathlib
import statistics
import subprocess
from collections import Counter, defaultdict
from typing import Any, Iterable


DEFAULT_FIRMWARE_COMMIT = "04a21839ddc0430991b77ee98f1d4e75bbcf8fa6"


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = min(len(ordered) - 1, lower + 1)
    blend = index - lower
    return ordered[lower] * (1.0 - blend) + ordered[upper] * blend


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
    """Match the dashboard's unweighted all-anchor linear fit."""
    usable = [
        (anchor_id, anchors[anchor_id], float(distance))
        for anchor_id, distance in sorted(distances.items())
        if anchor_id in anchors
        and math.isfinite(float(distance))
        and float(distance) > 0
    ]
    if len(usable) < 3:
        return None
    _, base, base_distance = usable[0]
    nxx = nxy = nyy = rhs_x = rhs_y = 0.0
    for _, anchor, distance in usable[1:]:
        ax = 2.0 * (anchor[0] - base[0])
        ay = 2.0 * (anchor[1] - base[1])
        rhs = (
            base_distance * base_distance
            - distance * distance
            + anchor[0] * anchor[0]
            - base[0] * base[0]
            + anchor[1] * anchor[1]
            - base[1] * base[1]
        )
        nxx += ax * ax
        nxy += ax * ay
        nyy += ay * ay
        rhs_x += ax * rhs
        rhs_y += ay * rhs
    return solve_2x2(nxx, nxy, nxy, nyy, rhs_x, rhs_y)


def load_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def load_ranges(path: pathlib.Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seen: set[tuple[int, int, int]] = set()
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            event = json.loads(line)
            if event.get("kind") != "ds_range":
                continue
            key = (
                int(event.get("tag_id") or 0),
                int(event.get("anchor_id") or 0),
                int(event.get("seq") or 0),
            )
            if key in seen:
                continue
            seen.add(key)
            rows.append(event)
    rows.sort(
        key=lambda row: (
            float(row.get("received_at") or 0),
            int(row.get("log_id") or 0),
        )
    )
    return rows


def unwrap_sequences(rows: list[dict[str, Any]]) -> None:
    wrap_base = 0
    previous: int | None = None
    for row in rows:
        raw = int(row["seq"])
        if previous is not None and raw - previous < -32768:
            wrap_base += 65536
        elif previous is not None and raw - previous > 32768:
            # A delayed pre-wrap record; retain the previous epoch for it.
            row["_unwrapped_seq"] = raw + max(0, wrap_base - 65536)
            continue
        row["_unwrapped_seq"] = raw + wrap_base
        previous = raw


def infer_sequence_phase(
    rows: list[dict[str, Any]], anchor_ids: tuple[int, ...]
) -> tuple[int, Counter[int]]:
    anchor_index = {
        anchor_id: index for index, anchor_id in enumerate(anchor_ids)
    }
    modulus = len(anchor_ids)
    counts = Counter(
        (
            int(row["seq"]) % modulus
            - anchor_index[int(row["anchor_id"])]
        )
        % modulus
        for row in rows
        if int(row["anchor_id"]) in anchor_index
    )
    if not counts:
        raise ValueError("no valid anchor/sequence observations")
    return counts.most_common(1)[0][0], counts


def coherent_positions(
    rows: list[dict[str, Any]],
    anchors: dict[int, tuple[float, float]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    anchor_ids = tuple(sorted(anchors))
    anchor_index = {
        anchor_id: index for index, anchor_id in enumerate(anchor_ids)
    }
    phase, phase_counts = infer_sequence_phase(rows, anchor_ids)
    modulus = len(anchor_ids)
    buckets: dict[int, dict[int, dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        anchor_id = int(row["anchor_id"])
        if anchor_id not in anchor_index:
            continue
        phase_index = (anchor_index[anchor_id] + phase) % modulus
        frame_start = int(row["_unwrapped_seq"]) - phase_index
        buckets[frame_start][anchor_id] = row

    positions: list[dict[str, Any]] = []
    for frame_start, items in sorted(buckets.items()):
        if set(items) != set(anchor_ids):
            continue
        ordered = [items[anchor_id] for anchor_id in anchor_ids]
        timestamps = [float(item["received_at"]) for item in ordered]
        solution = trilaterate_linear(
            anchors,
            {
                anchor_id: float(items[anchor_id]["distance_m"])
                for anchor_id in anchor_ids
            },
        )
        if solution is None:
            continue
        positions.append(
            {
                "mode": "coherent",
                "received_at": max(timestamps),
                "frame_start_seq": frame_start,
                "frame_span_ms": (max(timestamps) - min(timestamps)) * 1000.0,
                "anchors_used": len(anchor_ids),
                "x_m": solution[0],
                "y_m": solution[1],
            }
        )
    diagnostics = {
        "inferred_sequence_phase": phase,
        "sequence_phase_counts": {
            str(key): value for key, value in sorted(phase_counts.items())
        },
        "candidate_frames": len(buckets),
        "complete_frames": len(positions),
        "incomplete_frames": len(buckets) - len(positions),
    }
    return positions, diagnostics


def rolling_positions(
    rows: list[dict[str, Any]],
    anchors: dict[int, tuple[float, float]],
    max_age_sec: float,
) -> list[dict[str, Any]]:
    anchor_ids = set(anchors)
    latest: dict[int, dict[str, Any]] = {}
    positions: list[dict[str, Any]] = []
    for row in rows:
        anchor_id = int(row["anchor_id"])
        if anchor_id not in anchor_ids:
            continue
        latest[anchor_id] = row
        now = float(row["received_at"])
        fresh = {
            item: sample
            for item, sample in latest.items()
            if now - float(sample["received_at"]) <= max_age_sec
        }
        if len(fresh) < 3:
            continue
        timestamps = [
            float(sample["received_at"]) for sample in fresh.values()
        ]
        span = max(timestamps) - min(timestamps)
        solution = trilaterate_linear(
            anchors,
            {
                item: float(sample["distance_m"])
                for item, sample in fresh.items()
            },
        )
        if solution is None:
            continue
        positions.append(
            {
                "mode": "rolling",
                "received_at": float(row["received_at"]),
                "frame_start_seq": "",
                "frame_span_ms": span * 1000.0,
                "anchors_used": len(fresh),
                "x_m": solution[0],
                "y_m": solution[1],
            }
        )
    return positions


def position_metrics(
    rows: list[dict[str, Any]],
    reference: tuple[float, float],
) -> dict[str, Any]:
    if not rows:
        return {"samples": 0}
    x_values = [float(row["x_m"]) for row in rows]
    y_values = [float(row["y_m"]) for row in rows]
    mean_x = statistics.fmean(x_values)
    mean_y = statistics.fmean(y_values)
    median_x = statistics.median(x_values)
    median_y = statistics.median(y_values)
    reference_errors = [
        math.hypot(x - reference[0], y - reference[1])
        for x, y in zip(x_values, y_values)
    ]
    centered_errors = [
        math.hypot(x - mean_x, y - mean_y)
        for x, y in zip(x_values, y_values)
    ]
    elapsed = max(0.0, float(rows[-1]["received_at"]) -
                  float(rows[0]["received_at"]))
    return {
        "samples": len(rows),
        "span_sec": elapsed,
        "rate_hz": len(rows) / elapsed if elapsed > 0 else math.nan,
        "mean_x_m": mean_x,
        "mean_y_m": mean_y,
        "median_x_m": median_x,
        "median_y_m": median_y,
        "std_x_cm": (
            statistics.stdev(x_values) * 100.0
            if len(x_values) > 1 else 0.0
        ),
        "std_y_cm": (
            statistics.stdev(y_values) * 100.0
            if len(y_values) > 1 else 0.0
        ),
        "peak_to_peak_x_cm": (max(x_values) - min(x_values)) * 100.0,
        "peak_to_peak_y_cm": (max(y_values) - min(y_values)) * 100.0,
        "precision_rms_cm": (
            math.sqrt(statistics.fmean(value * value
                                      for value in centered_errors)) * 100.0
        ),
        "precision_cep50_cm": percentile(centered_errors, 0.50) * 100.0,
        "precision_cep95_cm": percentile(centered_errors, 0.95) * 100.0,
        "precision_max_cm": max(centered_errors) * 100.0,
        "reference_bias_x_cm": (mean_x - reference[0]) * 100.0,
        "reference_bias_y_cm": (mean_y - reference[1]) * 100.0,
        "reference_bias_2d_cm": math.hypot(
            mean_x - reference[0], mean_y - reference[1]) * 100.0,
        "reference_rmse_cm": (
            math.sqrt(statistics.fmean(value * value
                                      for value in reference_errors)) * 100.0
        ),
        "reference_p95_cm": percentile(reference_errors, 0.95) * 100.0,
        "reference_max_cm": max(reference_errors) * 100.0,
        "median_frame_span_ms": percentile(
            (float(row["frame_span_ms"]) for row in rows), 0.50),
        "p95_frame_span_ms": percentile(
            (float(row["frame_span_ms"]) for row in rows), 0.95),
    }


def range_metrics(
    rows: list[dict[str, Any]],
    anchors: dict[int, tuple[float, float]],
    reference: tuple[float, float],
) -> list[dict[str, Any]]:
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[int(row["anchor_id"])].append(row)
    output = []
    for anchor_id in sorted(grouped):
        samples = grouped[anchor_id]
        values = [float(row["distance_m"]) for row in samples]
        expected = math.dist(anchors[anchor_id], reference)
        errors = [value - expected for value in values]
        elapsed = max(
            0.0,
            float(samples[-1]["received_at"]) -
            float(samples[0]["received_at"]),
        )
        median = statistics.median(values)
        mad = statistics.median(abs(value - median) for value in values)
        output.append(
            {
                "anchor_id": anchor_id,
                "samples": len(values),
                "rate_hz": len(values) / elapsed if elapsed > 0 else math.nan,
                "mean_m": statistics.fmean(values),
                "median_m": median,
                "std_cm": (
                    statistics.stdev(values) * 100.0
                    if len(values) > 1 else 0.0
                ),
                "robust_sigma_cm": mad * 1.4826 * 100.0,
                "min_m": min(values),
                "max_m": max(values),
                "peak_to_peak_cm": (max(values) - min(values)) * 100.0,
                "reference_distance_m": expected,
                "reference_bias_cm": statistics.fmean(errors) * 100.0,
                "reference_rmse_cm": math.sqrt(
                    statistics.fmean(value * value for value in errors)
                ) * 100.0,
                "reference_p95_abs_cm": percentile(
                    (abs(value) for value in errors), 0.95) * 100.0,
            }
        )
    return output


def sequence_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    sequences = sorted({int(row["_unwrapped_seq"]) for row in rows})
    if not sequences:
        return {"received": 0}
    expected = sequences[-1] - sequences[0] + 1
    missing = max(0, expected - len(sequences))
    return {
        "received": len(sequences),
        "first_unwrapped_seq": sequences[0],
        "last_unwrapped_seq": sequences[-1],
        "expected_inclusive": expected,
        "missing": missing,
        "delivery_pct": len(sequences) / expected * 100.0,
    }


def write_csv(
    path: pathlib.Path, rows: list[dict[str, Any]]
) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def write_hash_manifest(output_dir: pathlib.Path) -> None:
    entries = []
    for path in sorted(output_dir.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS":
            continue
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        entries.append(
            f"{digest.hexdigest()}  {path.relative_to(output_dir)}"
        )
    (output_dir / "SHA256SUMS").write_text(
        "\n".join(entries) + "\n", encoding="utf-8"
    )


def git_context(root: pathlib.Path) -> dict[str, Any]:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=root, text=True
            ).strip()
        )
        return {"dashboard_repo_commit": commit, "worktree_dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"dashboard_repo_commit": None, "worktree_dirty": None}


def fmt(value: Any, digits: int = 2) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "-"
    return f"{number:.{digits}f}" if math.isfinite(number) else "-"


def write_readme(
    path: pathlib.Path,
    summary: dict[str, Any],
    events_name: str,
    logs_name: str | None,
) -> None:
    coherent = summary["position_metrics"]["coherent"]
    rolling = summary["position_metrics"]["rolling"]
    sequence = summary["sequence_metrics"]
    statuses = summary.get("initial_status") or []
    first_status = statuses[0] if statuses else {}
    firmware_label = (
        str(first_status.get("version") or "").strip()
        or str(summary.get("firmware_commit") or "unknown")[:7]
    )
    antenna_delays = ", ".join(
        f"M{status.get('module_id')}={status.get('uwb_active_antenna_delay')}"
        for status in statuses
        if status.get("uwb_active_antenna_delay") is not None
    )
    anchor_text = ", ".join(
        f"A{anchor_id}=({fmt(point[0], 3)}, {fmt(point[1], 3)})"
        for anchor_id, point in sorted(
            summary["anchors_m"].items(), key=lambda item: int(item[0])
        )
    )
    lines = [
        f"# Native DS-TWR static precision reference — firmware {firmware_label}",
        "",
        "This capture preserves a reproducible unfiltered Native DS-TWR "
        "precision reference.",
        "",
        "## Test identity",
        "",
        f"- Firmware commit: `{summary['firmware_commit']}`",
        f"- Dashboard source commit: "
        f"`{summary.get('dashboard_repo_commit') or 'unknown'}`"
        f"{' (worktree had uncommitted compatibility changes)' if summary.get('worktree_dirty') else ''}",
        f"- Capture duration: {fmt(summary['capture_duration_sec'], 1)} s",
        f"- Raw range events: {summary['range_events']}",
        f"- Sequence delivery: {fmt(sequence.get('delivery_pct'))}%",
        f"- Radio: channel {first_status.get('runtime_radio_channel', '-')} · "
        f"slot {first_status.get('runtime_ranging_slot_ms', '-')} ms · "
        f"round gap {first_status.get('runtime_ranging_round_gap_ms', '-')} ms · "
        f"RX slice {first_status.get('runtime_ranging_rx_slice_ms', '-')} ms",
        f"- Antenna delays: {antenna_delays or '-'}",
        f"- Geometry used by the dashboard reconstruction: {anchor_text}",
        f"- Reference geometry: {summary['reference_note']}",
        "- Tag remained static. No filtering is applied by this offline "
        "reconstruction; it uses the dashboard's unweighted all-anchor "
        "linear trilateration.",
        "",
        "## Precision reference",
        "",
        "| Reconstruction | updates/s | σx [cm] | σy [cm] | precision "
        "CEP50 [cm] | precision CEP95 [cm] | precision max [cm] |",
        "|---|---:|---:|---:|---:|---:|---:|",
        f"| Coherent independent frames | {fmt(coherent.get('rate_hz'))} | "
        f"{fmt(coherent.get('std_x_cm'))} | {fmt(coherent.get('std_y_cm'))} | "
        f"{fmt(coherent.get('precision_cep50_cm'))} | "
        f"{fmt(coherent.get('precision_cep95_cm'))} | "
        f"{fmt(coherent.get('precision_max_cm'))} |",
        f"| Rolling latest ranges | {fmt(rolling.get('rate_hz'))} | "
        f"{fmt(rolling.get('std_x_cm'))} | {fmt(rolling.get('std_y_cm'))} | "
        f"{fmt(rolling.get('precision_cep50_cm'))} | "
        f"{fmt(rolling.get('precision_cep95_cm'))} | "
        f"{fmt(rolling.get('precision_max_cm'))} |",
        "",
        "Precision is measured around each series' own mean and is the correct "
        "metric for the invisible/very short static trail. Values against the "
        "cached anchor centroid are retained in `summary.json`, but they are "
        "not independent absolute-accuracy ground truth.",
        "",
        "## Files",
        "",
        f"- `{events_name}` — deduplicated DS-TWR range events and status snapshots",
    ]
    if logs_name:
        lines.append(
            f"- `{logs_name}` — raw dashboard log records, including every "
            "`UWB_RANGING result` line"
        )
    lines.extend(
        [
            f"- `{summary['metadata_file']}` — full initial/final module status "
            "and capture settings",
            "- `positions.csv` — coherent and rolling offline positions",
            "- `range_metrics.csv` — per-anchor distance stability",
            "- `summary.json` — complete machine-readable metrics and context",
            "- `SHA256SUMS` — integrity hashes for the complete reference",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--events", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rolling-max-age-sec", type=float, default=0.5)
    parser.add_argument(
        "--firmware-commit", default=DEFAULT_FIRMWARE_COMMIT
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    events_path = pathlib.Path(args.events).resolve()
    metadata_path = pathlib.Path(args.metadata).resolve()
    output_dir = pathlib.Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = load_json(metadata_path)
    ground_truth = metadata["ground_truth"]
    anchors = {
        int(anchor_id): (float(point[0]), float(point[1]))
        for anchor_id, point in ground_truth["anchors_m"].items()
    }
    reference = tuple(float(value) for value in ground_truth["tag_m"])
    ranges = [
        row for row in load_ranges(events_path)
        if int(row["anchor_id"]) in anchors
    ]
    if not ranges:
        raise SystemExit("capture contains no DS-TWR ranges")
    unwrap_sequences(ranges)
    coherent, coherent_diagnostics = coherent_positions(ranges, anchors)
    rolling = rolling_positions(
        ranges, anchors, max(0.1, args.rolling_max_age_sec)
    )
    positions = coherent + rolling
    for row in positions:
        row["reference_error_cm"] = math.hypot(
            float(row["x_m"]) - reference[0],
            float(row["y_m"]) - reference[1],
        ) * 100.0
    positions.sort(key=lambda row: (float(row["received_at"]), row["mode"]))

    root = pathlib.Path(__file__).resolve().parents[1]
    summary = {
        "schema_version": 1,
        "firmware_commit": args.firmware_commit,
        **git_context(root),
        "events_file": str(events_path.relative_to(output_dir)),
        "metadata_file": str(metadata_path.relative_to(output_dir)),
        "raw_log_file": (
            f"data/{metadata['timing_log_file']}"
            if metadata.get("timing_log_file") else None
        ),
        "capture_duration_sec": float(metadata["duration_sec"]),
        "range_events": len(ranges),
        "reference_note": ground_truth["coordinate_convention"],
        "anchors_m": {
            str(anchor_id): list(point)
            for anchor_id, point in anchors.items()
        },
        "tag_reference_m": list(reference),
        "rolling_max_age_sec": args.rolling_max_age_sec,
        "sequence_metrics": sequence_metrics(ranges),
        "coherent_diagnostics": coherent_diagnostics,
        "range_metrics": range_metrics(ranges, anchors, reference),
        "position_metrics": {
            "coherent": position_metrics(coherent, reference),
            "rolling": position_metrics(rolling, reference),
        },
        "initial_status": metadata.get("initial_status"),
        "final_status": metadata.get("final_status"),
    }
    with (output_dir / "summary.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    write_csv(output_dir / "positions.csv", positions)
    write_csv(output_dir / "range_metrics.csv", summary["range_metrics"])
    write_readme(
        output_dir / "README.md",
        summary,
        summary["events_file"],
        summary["raw_log_file"],
    )
    write_hash_manifest(output_dir)
    print(json.dumps(summary["position_metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
