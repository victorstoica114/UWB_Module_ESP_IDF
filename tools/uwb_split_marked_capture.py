#!/usr/bin/env python3
"""Split a continuous dashboard JSONL capture using explicit wall-clock markers.

The splitter is intentionally lossless for records inside each interval, apart
from capture envelope replacement and protocol/capture-id normalization.  It is
used when an operator keeps one capture running while changing UWB protocols.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import pathlib
from typing import Any


PROTOCOL_ALIASES = {
    "flex": "flextdoa",
    "flex_tdoa": "flextdoa",
    "flextdoa": "flextdoa",
    "native": "native_ds",
    "native_ds": "native_ds",
    "native_ds_twr": "native_ds",
    "passive": "passive_ds",
    "passive_ds": "passive_ds",
    "passive_ds_twr": "passive_ds",
}


def normalize_protocol(value: Any) -> str:
    return PROTOCOL_ALIASES.get(str(value or "").strip().lower(), "unknown")


def wall_ms(record: dict[str, Any]) -> int | None:
    value = record.get("collector_wall_ns")
    if isinstance(value, (int, float)):
        return int(value) // 1_000_000
    value = record.get("received_at")
    if isinstance(value, (int, float)):
        return int(float(value) * 1000.0)
    return None


def position_protocol(record: dict[str, Any]) -> str:
    return normalize_protocol(record.get("tdoa_protocol") or record.get("protocol"))


def split_capture(source: pathlib.Path, markers: pathlib.Path, output_dir: pathlib.Path) -> list[pathlib.Path]:
    marker_payload = json.loads(markers.read_text(encoding="utf-8"))
    segments: list[dict[str, Any]] = []
    for item in marker_payload.get("segments", []):
        protocol = normalize_protocol(item.get("protocol"))
        start_ms = int(item["start_unix_ms"])
        stop_ms = int(item["stop_unix_ms"])
        if protocol == "unknown" or stop_ms <= start_ms:
            raise ValueError(f"invalid segment: {item!r}")
        capture_id = f"{protocol}_final_dynamic_20260812"
        segments.append(
            {
                "protocol": protocol,
                "start_ms": start_ms,
                "stop_ms": stop_ms,
                "capture_id": capture_id,
                "counts": Counter(),
            }
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    handles: dict[str, Any] = {}
    outputs: list[pathlib.Path] = []
    try:
        for segment in segments:
            path = output_dir / f"{segment['capture_id']}.{segment['protocol']}.jsonl"
            outputs.append(path)
            handle = path.open("w", encoding="utf-8", newline="\n")
            handles[segment["capture_id"]] = handle
            start = {
                "kind": "capture_start",
                "schema_version": 1,
                "capture_id": segment["capture_id"],
                "protocol": segment["protocol"],
                "collector_wall_ns": segment["start_ms"] * 1_000_000,
                "received_at": segment["start_ms"] / 1000.0,
                "initial_status": [],
                "warnings": [
                    "derived losslessly from a continuous capture using explicit operator markers",
                    f"source={source.as_posix()}",
                ],
            }
            handle.write(json.dumps(start, separators=(",", ":"), sort_keys=True) + "\n")

        protocol_bound_kinds = {"position", "local_position", "ds_range", "anchor_range", "uwb_measurement"}
        with source.open(encoding="utf-8") as input_handle:
            for line_number, line in enumerate(input_handle, start=1):
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(f"invalid JSON on line {line_number}: {error}") from error
                if record.get("kind") in {"capture_start", "capture_end"}:
                    continue
                timestamp_ms = wall_ms(record)
                if timestamp_ms is None:
                    continue
                for segment in segments:
                    if not (segment["start_ms"] <= timestamp_ms <= segment["stop_ms"]):
                        continue
                    kind = str(record.get("kind", "unknown"))
                    if kind in protocol_bound_kinds:
                        actual = position_protocol(record)
                        if actual != segment["protocol"]:
                            continue
                    record["capture_id"] = segment["capture_id"]
                    record["protocol"] = segment["protocol"]
                    handles[segment["capture_id"]].write(
                        json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
                    )
                    segment["counts"][kind] += 1
                    break

        for segment in segments:
            end = {
                "kind": "capture_end",
                "schema_version": 1,
                "capture_id": segment["capture_id"],
                "protocol": segment["protocol"],
                "collector_wall_ns": segment["stop_ms"] * 1_000_000,
                "received_at": segment["stop_ms"] / 1000.0,
                "interrupted": False,
                "counts": dict(sorted(segment["counts"].items())),
                "source_capture": source.as_posix(),
                "marker_file": markers.as_posix(),
            }
            handles[segment["capture_id"]].write(
                json.dumps(end, separators=(",", ":"), sort_keys=True) + "\n"
            )
    finally:
        for handle in handles.values():
            handle.close()
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--markers", required=True, type=pathlib.Path)
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    args = parser.parse_args()
    for path in split_capture(args.input, args.markers, args.output_dir):
        print(path)


if __name__ == "__main__":
    main()
