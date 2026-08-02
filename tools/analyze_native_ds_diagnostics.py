#!/usr/bin/env python3
"""Capture and correlate Native DS-TWR range and DW3000 CIA diagnostics."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import time
import urllib.parse
import urllib.request
from collections import defaultdict
from pathlib import Path
from typing import Any


RX_RE = re.compile(
    r"NATIVE_DS_RX_DIAG local=(?P<local>\d+) frame=(?P<frame>\d+) "
    r"type=(?P<type>\d+) src=(?P<src>\d+) dst=(?P<dst>\d+) "
    r"rx_ts=0x(?P<rx_ts>[0-9a-fA-F]+) pacc=(?P<pacc>\d+) "
    r"fp=(?P<fp>[0-9.]+) peak_idx=(?P<peak_idx>\d+) "
    r"peak_amp=(?P<peak_amp>\d+) power=(?P<power>\d+) "
    r"f1=(?P<f1>\d+) f2=(?P<f2>\d+) f3=(?P<f3>\d+) "
    r"acc=(?P<acc>\d+) xtal=(?P<xtal>-?\d+)"
)
RANGE_RE = re.compile(
    r"NATIVE_DS_RANGE_DIAG local=(?P<local>\d+) frame=(?P<frame>\d+) "
    r"exchange=(?P<exchange>tag|survey) src=(?P<src>\d+) .* "
    r"distance=(?P<distance>[0-9.]+)"
)


def fetch_json(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=5) as response:
        return json.load(response)


def pearson(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3 or len(xs) != len(ys):
        return None
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    dx = [value - mean_x for value in xs]
    dy = [value - mean_y for value in ys]
    denominator = math.sqrt(sum(value * value for value in dx) *
                            sum(value * value for value in dy))
    if denominator == 0.0:
        return None
    return sum(x * y for x, y in zip(dx, dy)) / denominator


def capture(base_url: str, duration_s: float) -> list[dict[str, Any]]:
    base_url = base_url.rstrip("/")
    snapshot = fetch_json(f"{base_url}/api/snapshot")
    after = max(0, int(snapshot.get("next_log_id") or 1) - 1)
    deadline = time.monotonic() + duration_s
    captured: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        query = urllib.parse.urlencode({"after": after, "limit": 8000})
        payload = fetch_json(f"{base_url}/api/logs?{query}")
        logs = payload.get("logs") or []
        if logs:
            captured.extend(logs)
            after = max(after, max(int(item.get("id") or 0) for item in logs))
        time.sleep(0.25)
    return captured


def parse_rx(match: re.Match[str]) -> dict[str, float | int]:
    values: dict[str, float | int] = {}
    for name, text in match.groupdict().items():
        if name == "fp":
            values[name] = float(text)
        elif name == "rx_ts":
            values[name] = int(text, 16)
        else:
            values[name] = int(text)
    values["fp_peak_gap"] = float(values["peak_idx"]) - float(values["fp"])
    return values


def analyze(logs: list[dict[str, Any]]) -> dict[str, Any]:
    rx: dict[tuple[int, int, str], dict[str, float | int]] = {}
    ranges: list[dict[str, float | int | str]] = []
    timestamp_mismatches = 0

    for item in logs:
        message = str(item.get("message") or item.get("raw") or "")
        if "TX timestamp mismatch" in message:
            timestamp_mismatches += 1
        rx_match = RX_RE.search(message)
        if rx_match:
            diag = parse_rx(rx_match)
            frame = int(diag["frame"])
            local = int(diag["local"])
            frame_type = int(diag["type"])
            if frame_type == 1:
                anchor = local
                stage = "poll"
            elif frame_type == 2:
                anchor = int(diag["src"])
                stage = "response"
            else:
                anchor = local
                stage = "final"
            rx[(frame, anchor, stage)] = diag
            continue
        range_match = RANGE_RE.search(message)
        if range_match and range_match.group("exchange") == "tag":
            ranges.append({
                "anchor": int(range_match.group("local")),
                "frame": int(range_match.group("frame")),
                "distance": float(range_match.group("distance")),
            })

    joined: dict[int, list[dict[str, float]]] = defaultdict(list)
    for sample in ranges:
        anchor = int(sample["anchor"])
        frame = int(sample["frame"])
        row = {"distance": float(sample["distance"])}
        complete = True
        for stage in ("poll", "response", "final"):
            diag = rx.get((frame, anchor, stage))
            if diag is None:
                complete = False
                break
            for field in ("pacc", "fp", "peak_idx", "peak_amp", "power",
                          "f1", "f2", "f3", "acc", "xtal", "fp_peak_gap"):
                row[f"{stage}_{field}"] = float(diag[field])
        if complete:
            joined[anchor].append(row)

    result: dict[str, Any] = {
        "captured_logs": len(logs),
        "range_diagnostics": len(ranges),
        "rx_diagnostics": len(rx),
        "timestamp_mismatches": timestamp_mismatches,
        "anchors": {},
    }
    for anchor, rows in sorted(joined.items()):
        distances = [row["distance"] for row in rows]
        median = statistics.median(distances)
        mad = statistics.median(abs(value - median) for value in distances)
        correlations = []
        for field in rows[0]:
            if field == "distance":
                continue
            correlation = pearson(distances, [row[field] for row in rows])
            if correlation is not None:
                correlations.append({"field": field, "r": correlation})
        correlations.sort(key=lambda item: abs(float(item["r"])), reverse=True)
        result["anchors"][str(anchor)] = {
            "samples": len(rows),
            "median_m": median,
            "robust_sigma_cm": 1.4826 * mad * 100.0,
            "min_m": min(distances),
            "max_m": max(distances),
            "strongest_correlations": correlations[:8],
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8780")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    logs = capture(args.url, max(1.0, args.duration))
    result = analyze(logs)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps({"analysis": result, "logs": logs}, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
