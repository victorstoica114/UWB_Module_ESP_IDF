#!/usr/bin/env python3
"""Analyze the Passive DS-TWR speed-limit captures.

The report deliberately separates update rate, absolute accuracy, precision,
and radio schedule health.  A high position counter is not treated as useful
throughput unless each update contains newly received UWB information.
"""

from __future__ import annotations

import csv
import json
import math
import pathlib
import re
import statistics
from collections import defaultdict
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports" / "passive_ds_speed_limit_20260727"
DATA_DIR = ROOT / "reports" / "raw" / "passive_ds_speed_limit_20260727" / "data"
FIGURE_DIR = REPORT_DIR / "figures"
TRUTH = (1.5, 1.5)

BLOCKS = (
    ("robust_16ms", "Robust 16 ms"),
    ("robust_13ms", "Robust 13 ms"),
    ("robust_10ms", "Robust 10 ms"),
    ("robust_10ms_900us", "Robust 10 ms, 0.9/0.9 ms"),
    ("fast_10ms", "Fast Star 10 ms"),
)

ANCHOR_SUMMARY_RE = re.compile(
    r"PASSIVE_DS anchor schedule=(?P<schedule>\w+) "
    r"ok=(?P<ok>\d+) fail=(?P<fail>\d+) "
    r"bootstrap=(?P<bootstrap>\d+) late=(?P<late>\d+) "
    r"next_slot=(?P<next_slot>\d+)"
)
SOLVER_SUMMARY_RE = re.compile(
    r"PASSIVE_DS solver pos=(?P<rate>\d+(?:\.\d+)?)/s "
    r"accept=(?P<accept>\d+)"
    r"(?: frames=(?P<frames>\d+) rolling=(?P<rolling>\d+))? "
    r"reject=(?P<reject>\d+) "
    r"obs=(?P<obs_accept>\d+)/(?P<obs_reject>\d+) "
    r"range=(?P<range_accept>\d+)/(?P<range_reject>\d+) "
    r"reloc=(?P<reloc>\d+)"
)


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q))


def position_metrics(
    rows: list[dict[str, Any]], x_key: str, y_key: str
) -> dict[str, float | int]:
    usable = [
        row
        for row in rows
        if isinstance(row.get(x_key), (int, float))
        and isinstance(row.get(y_key), (int, float))
        and math.isfinite(float(row[x_key]))
        and math.isfinite(float(row[y_key]))
    ]
    x = np.asarray([float(row[x_key]) for row in usable], dtype=float)
    y = np.asarray([float(row[y_key]) for row in usable], dtype=float)
    dx = (x - TRUTH[0]) * 100.0
    dy = (y - TRUTH[1]) * 100.0
    radial = np.hypot(dx, dy)
    sx = float(np.std(x, ddof=1) * 100.0) if x.size > 1 else 0.0
    sy = float(np.std(y, ddof=1) * 100.0) if y.size > 1 else 0.0
    return {
        "n": int(x.size),
        "mean_x_m": float(np.mean(x)),
        "mean_y_m": float(np.mean(y)),
        "bias_x_cm": float(np.mean(dx)),
        "bias_y_cm": float(np.mean(dy)),
        "bias_2d_cm": float(math.hypot(np.mean(dx), np.mean(dy))),
        "rmse_2d_cm": float(math.sqrt(np.mean(radial**2))),
        "p95_error_cm": percentile(radial.tolist(), 95),
        "max_error_cm": float(np.max(radial)),
        "std_x_cm": sx,
        "std_y_cm": sy,
        "two_drms_cm": float(2.0 * math.sqrt(sx * sx + sy * sy)),
    }


def load_jsonl(path: pathlib.Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def status_value(metadata: dict[str, Any], key: str) -> Any:
    values = {
        json.dumps(item.get(key), sort_keys=True)
        for item in metadata.get("final_status", [])
        if item.get(key) is not None
    }
    if not values:
        return None
    parsed = [json.loads(value) for value in sorted(values)]
    return parsed[0] if len(parsed) == 1 else parsed


def timing_metrics(path: pathlib.Path) -> dict[str, float | int | str]:
    if not path.exists():
        return {}
    anchor_rows: list[dict[str, Any]] = []
    solver_rows: list[dict[str, Any]] = []
    for item in load_jsonl(path):
        message = str(item.get("message") or "")
        match = ANCHOR_SUMMARY_RE.search(message)
        if match:
            anchor_rows.append(
                {
                    "module_id": int(item.get("module_id") or 0),
                    "received_at": float(item.get("received_at") or 0.0),
                    **{
                        key: (
                            value
                            if key == "schedule"
                            else int(value)
                        )
                        for key, value in match.groupdict().items()
                    },
                }
            )
        solver_match = SOLVER_SUMMARY_RE.search(message)
        if solver_match:
            solver_rows.append(
                {
                    key: (
                        float(value)
                        if key == "rate"
                        else int(value) if value is not None else 0
                    )
                    for key, value in solver_match.groupdict().items()
                }
            )

    result: dict[str, float | int | str] = {}
    if anchor_rows:
        total_ok = sum(int(row["ok"]) for row in anchor_rows)
        total_fail = sum(int(row["fail"]) for row in anchor_rows)
        result.update(
            {
                "schedule": str(anchor_rows[-1]["schedule"]),
                "anchor_ok": total_ok,
                "anchor_fail": total_fail,
                "anchor_success_pct": (
                    100.0 * total_ok / (total_ok + total_fail)
                    if total_ok + total_fail
                    else math.nan
                ),
            }
        )
        by_module: dict[int, list[dict[str, Any]]] = defaultdict(list)
        for row in anchor_rows:
            by_module[int(row["module_id"])].append(row)
        slot_rates = []
        late_deltas = []
        for rows in by_module.values():
            rows.sort(key=lambda row: float(row["received_at"]))
            if len(rows) < 2:
                continue
            elapsed = float(rows[-1]["received_at"]) - float(
                rows[0]["received_at"]
            )
            if elapsed > 0:
                slot_rates.append(
                    (
                        int(rows[-1]["next_slot"])
                        - int(rows[0]["next_slot"])
                    )
                    / elapsed
                )
            late_deltas.append(
                max(0, int(rows[-1]["late"]) - int(rows[0]["late"]))
            )
        if slot_rates:
            result["observed_slot_rate_hz"] = statistics.median(slot_rates)
            result["observed_slot_period_ms"] = (
                1000.0 / float(result["observed_slot_rate_hz"])
            )
        result["late_slot_delta"] = sum(late_deltas)

    if solver_rows:
        accepted = sum(int(row["accept"]) for row in solver_rows)
        rejected = sum(int(row["reject"]) for row in solver_rows)
        result.update(
            {
                "solver_logged_rate_hz": statistics.fmean(
                    float(row["rate"]) for row in solver_rows
                ),
                "solver_accept": accepted,
                "independent_frame_accept": sum(
                    int(row["frames"]) for row in solver_rows
                ),
                "rolling_accept": sum(
                    int(row["rolling"]) for row in solver_rows
                ),
                "solver_reject": rejected,
                "solver_accept_pct": (
                    100.0 * accepted / (accepted + rejected)
                    if accepted + rejected
                    else math.nan
                ),
                "observation_accept": sum(
                    int(row["obs_accept"]) for row in solver_rows
                ),
                "observation_reject": sum(
                    int(row["obs_reject"]) for row in solver_rows
                ),
                "range_accept": sum(
                    int(row["range_accept"]) for row in solver_rows
                ),
                "range_reject": sum(
                    int(row["range_reject"]) for row in solver_rows
                ),
                "relocation_count_last": int(solver_rows[-1]["reloc"]),
            }
        )
    return result


def analyze_block(name: str, label: str) -> dict[str, Any]:
    metadata = json.loads(
        (DATA_DIR / f"{name}.metadata.json").read_text(encoding="utf-8")
    )
    events = load_jsonl(DATA_DIR / f"{name}.jsonl")
    positions = [row for row in events if row.get("kind") == "local_position"]
    observations = [
        row for row in events if row.get("kind") == "tdoa_observation"
    ]
    ranges = [row for row in events if row.get("kind") == "anchor_range"]
    duration = float(metadata["duration_sec"])
    slot_ms = int(status_value(metadata, "runtime_passive_ds_slot_ms"))
    gap_ms = int(
        status_value(metadata, "runtime_passive_ds_round_gap_ms")
    )
    frame_ms = 3 * slot_ms + gap_ms
    rate = len(positions) / duration
    internal_rms_cm = [
        float(row["rms_m"]) * 100.0
        for row in positions
        if isinstance(row.get("rms_m"), (int, float))
    ]
    result = {
        "block": name,
        "label": label,
        "duration_sec": duration,
        "schedule_id": int(
            status_value(metadata, "runtime_passive_ds_schedule")
        ),
        "slot_ms": slot_ms,
        "gap_ms": gap_ms,
        "configured_frame_ms": frame_ms,
        "nominal_position_rate_hz": 1000.0 / frame_ms,
        "position_count": len(positions),
        "position_rate_hz": rate,
        "effective_frame_ms": 1000.0 / rate,
        "nominal_rate_achieved_pct": 100.0 * rate / (1000.0 / frame_ms),
        "observation_count": len(observations),
        "observation_rate_hz": len(observations) / duration,
        "anchor_range_count": len(ranges),
        "anchor_range_rate_hz": len(ranges) / duration,
        "resp_delay_us": int(
            status_value(metadata, "runtime_passive_ds_resp_delay_us")
        ),
        "final_delay_us": int(
            status_value(metadata, "runtime_passive_ds_final_delay_us")
        ),
        "internal_rms_mean_cm": statistics.fmean(internal_rms_cm),
        "internal_rms_p95_cm": percentile(internal_rms_cm, 95),
        "filtered": position_metrics(positions, "x_m", "y_m"),
        "raw": position_metrics(positions, "raw_x_m", "raw_y_m"),
        **timing_metrics(DATA_DIR / f"{name}.timing.jsonl"),
    }
    return result


def flattened_rows(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for result in results:
        row = {
            key: value
            for key, value in result.items()
            if key not in {"filtered", "raw"}
        }
        row.update(
            {
                f"filtered_{key}": value
                for key, value in result["filtered"].items()
            }
        )
        row.update(
            {
                f"raw_{key}": value
                for key, value in result["raw"].items()
            }
        )
        rows.append(row)
    return rows


def make_figures(results: list[dict[str, Any]]) -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    labels = [str(result["label"]) for result in results]
    x = np.arange(len(results))

    fig, ax_rate = plt.subplots(figsize=(11.5, 5.5))
    rates = [float(result["position_rate_hz"]) for result in results]
    bars = ax_rate.bar(x, rates, color="#2563eb", alpha=0.82)
    ax_rate.axhline(85.0, color="#dc2626", linestyle="--", label="85 Hz target")
    ax_rate.set_ylabel("Measured positions / s")
    ax_rate.set_xticks(x, labels, rotation=13, ha="right")
    ax_rate.set_ylim(0, 105)
    ax_rate.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, rates):
        ax_rate.text(
            bar.get_x() + bar.get_width() / 2,
            value + 1.2,
            f"{value:.1f}",
            ha="center",
            fontsize=9,
        )
    ax_rate.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "measured_update_rate.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11.5, 5.5))
    width = 0.36
    raw = [float(result["raw"]["rmse_2d_cm"]) for result in results]
    filtered = [
        float(result["filtered"]["rmse_2d_cm"]) for result in results
    ]
    ax.bar(x - width / 2, raw, width, label="Raw", color="#f97316")
    ax.bar(
        x + width / 2, filtered, width, label="CV-EKF", color="#16a34a"
    )
    ax.set_ylabel("2-D RMSE [cm]")
    ax.set_xticks(x, labels, rotation=13, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "raw_vs_filtered_rmse.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.2, 5.7))
    colors = ["#16a34a", "#0ea5e9", "#2563eb", "#7c3aed", "#f97316"]
    for result, color in zip(results, colors):
        ax.scatter(
            float(result["position_rate_hz"]),
            float(result["filtered"]["rmse_2d_cm"]),
            s=85,
            color=color,
            label=str(result["label"]),
        )
    ax.axvline(85.0, color="#dc2626", linestyle="--", alpha=0.8)
    ax.set_xlabel("Measured positions / s")
    ax.set_ylabel("Filtered 2-D RMSE [cm]")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "speed_accuracy_tradeoff.png", dpi=180)
    plt.close(fig)


def write_summary(results: list[dict[str, Any]]) -> None:
    fast = next(result for result in results if result["block"] == "fast_10ms")
    robust = next(
        result for result in results if result["block"] == "robust_10ms"
    )
    short_delay = next(
        result
        for result in results
        if result["block"] == "robust_10ms_900us"
    )
    safest = next(
        result for result in results if result["block"] == "robust_16ms"
    )
    needed_slot_rate = 85.0 * 3.0
    measured_slot_rate = float(robust.get("observed_slot_rate_hz", math.nan))
    required_gain = 100.0 * (85.0 / float(robust["position_rate_hz"]) - 1.0)

    lines = [
        "# Passive DS-TWR speed-limit study",
        "",
        "Date: 2026-07-27",
        "",
        "Geometry: surveyed 3 m × 3 m square (±2 mm), stationary tag at "
        "(1.5 m, 1.5 m)",
        "Firmware: `137c2b5`",
        "",
        "## Result",
        "",
        "The best fast operating point is **Robust Rotating, configured "
        "10 ms frame, 1.0/1.0 ms RESP/FINAL delays**. It delivered "
        f"**{robust['position_rate_hz']:.1f} positions/s**, "
        f"**{robust['filtered']['rmse_2d_cm']:.2f} cm 2-D RMSE**, and "
        f"**{robust['filtered']['p95_error_cm']:.2f} cm P95**.",
        "",
        "The most accurate point was Robust 16 ms: "
        f"{safest['position_rate_hz']:.1f} positions/s and "
        f"{safest['filtered']['rmse_2d_cm']:.2f} cm RMSE. "
        "The 10 ms setting therefore adds useful speed for only a small "
        "accuracy cost.",
        "",
        "Fast Star did not improve speed in this geometry. It reached only "
        f"{fast['position_rate_hz']:.1f} positions/s and "
        f"{fast['filtered']['rmse_2d_cm']:.2f} cm RMSE. Its asymmetric "
        "initiator load and the observed failures make Robust Rotating the "
        "better basis for further optimization.",
        "",
        "Reducing both reply delays to 0.9 ms was rejected: rate changed from "
        f"{robust['position_rate_hz']:.1f} to "
        f"{short_delay['position_rate_hz']:.1f} positions/s, while RMSE "
        f"worsened from {robust['filtered']['rmse_2d_cm']:.2f} to "
        f"{short_delay['filtered']['rmse_2d_cm']:.2f} cm and P95 from "
        f"{robust['filtered']['p95_error_cm']:.2f} to "
        f"{short_delay['filtered']['p95_error_cm']:.2f} cm. The live setup "
        "was restored to 1.0/1.0 ms.",
        "",
        "## Controlled measurements",
        "",
        "| Profile | position/s | nominal/s | filtered RMSE cm | "
        "filtered P95 cm | raw RMSE cm | anchor success |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        success = result.get("anchor_success_pct")
        success_text = (
            f"{float(success):.1f}%"
            if isinstance(success, (int, float)) and math.isfinite(success)
            else "n/a"
        )
        lines.append(
            f"| {result['label']} | {result['position_rate_hz']:.1f} | "
            f"{result['nominal_position_rate_hz']:.1f} | "
            f"{result['filtered']['rmse_2d_cm']:.2f} | "
            f"{result['filtered']['p95_error_cm']:.2f} | "
            f"{result['raw']['rmse_2d_cm']:.2f} | {success_text} |"
        )
    lines.extend(
        [
            "",
            "![Measured update rate](figures/measured_update_rate.png)",
            "",
            "![Raw versus filtered RMSE](figures/raw_vs_filtered_rmse.png)",
            "",
            "![Speed/accuracy trade-off](figures/speed_accuracy_tradeoff.png)",
            "",
            "## Why the result is below 85 positions/s",
            "",
            f"The 10 ms profile nominally permits 100 frames/s, but measured "
            f"only {robust['position_rate_hz']:.1f}. Anchor logs show about "
            f"{measured_slot_rate:.1f} slots/s "
            f"({1000.0 / measured_slot_rate:.2f} ms effective slot) rather "
            "than the 300 slots/s implied by the nominal 10 ms frame. "
            "Because one independent position frame needs three directed "
            "anchor dialogues, 85 "
            f"positions/s needs at least {needed_slot_rate:.0f} successful "
            "slots/s, before rejection margin.",
            "",
            f"The remaining gap is approximately {required_gain:.1f}% in "
            "accepted position throughput. The limiting layer is the radio "
            "transaction/scheduling path, not the dashboard or EKF. Publishing "
            "a new solution after every observation could display more than "
            "85 updates/s, but adjacent solutions would reuse almost all of "
            "the same measurements; this report does not count that as 85 "
            "independent updates/s.",
            "",
            "## Recommended next engineering step",
            "",
            "1. Add per-stage timestamps and failure counters for POLL TX, "
            "RESP delayed TX, FINAL TX/RX, CIA readout, and RX re-arm.",
            "2. Replace millisecond receive-loop scheduling with DW3000 "
            "delayed-TX/delayed-RX deadlines and a non-blocking slot state "
            "machine.",
            "3. Keep Robust Rotating and 1.0/1.0 ms as the control profile; "
            "change one timing stage at a time and accept a variant only if "
            "RMSE and P95 remain inside the control confidence band.",
            "4. Separately evaluate a rolling solution on every new "
            "observation as a low-latency display mode, clearly reporting "
            "both solver updates/s and independent frames/s.",
            "",
            "## Reproducibility",
            "",
            "Raw event streams, status snapshots, and timing logs are in "
            "[`data/`](data/). Recreate the tables and figures with:",
            "",
            "```bash",
            "python3 tools/uwb_passive_speed_analyze.py",
            "```",
            "",
        ]
    )
    (REPORT_DIR / "README.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def main() -> None:
    results = [
        analyze_block(name, label)
        for name, label in BLOCKS
        if (DATA_DIR / f"{name}.metadata.json").exists()
    ]
    if len(results) != len(BLOCKS):
        raise SystemExit("one or more expected capture blocks are missing")
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    make_figures(results)
    write_summary(results)
    (REPORT_DIR / "metrics.json").write_text(
        json.dumps(results, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    flat = flattened_rows(results)
    with (REPORT_DIR / "metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=sorted({key for row in flat for key in row}),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(flat)
    print(f"wrote {REPORT_DIR}")


if __name__ == "__main__":
    main()
