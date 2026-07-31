#!/usr/bin/env python3
"""Summarize Native DS-TWR round-boundary captures.

The collector stores firmware logs as JSON Lines and the initial/final status
snapshots in a sidecar metadata file.  This tool keeps the boundary analysis
reproducible without loading the multi-gigabyte capture set into memory.
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import re
from typing import Any


REJECT_RE = re.compile(
    r"UWB_RANGING rejected reason=(?P<reason>\S+) "
    r"tag=(?P<tag>\d+) anchor=(?P<anchor>\d+) seq=(?P<seq>\d+) "
    r"token=\S+ round=(?P<round>\d+) slot=(?P<slot>\d+) "
    r"first=(?P<first>\d+) flags=0x(?P<flags>[0-9a-fA-F]+)"
)
CONTEXT_RE = re.compile(
    r"UWB_RANGING (?:result|rejected\s+reason=\S+) "
    r"tag=(?P<tag>\d+) anchor=(?P<anchor>\d+) seq=(?P<seq>\d+).*?"
    r"round=(?P<round>\d+) slot=(?P<slot>\d+) "
    r"first=(?P<first>\d+) flags=0x(?P<flags>[0-9a-fA-F]+)"
)

COUNTER_KEYS = (
    "initiated",
    "completed",
    "responded",
    "response_timeouts",
    "final_timeouts",
    "context_mismatches",
    "timestamp_rejects",
    "negative_tof_rejects",
    "impossible_range_rejects",
    "slot_overruns",
    "round_boundaries",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("captures", nargs="+", type=pathlib.Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def load_status(path: pathlib.Path) -> dict[int, dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return {
        int(item["module_id"]): item
        for item in data.get("final_status", [])
        if item.get("module_id") is not None
    }


def counter_totals(path: pathlib.Path) -> dict[str, int]:
    metadata = json.loads(path.read_text(encoding="utf-8"))
    initial = {
        int(item["module_id"]): item
        for item in metadata.get("initial_status", [])
        if item.get("module_id") is not None
    }
    final = load_status(path)
    totals = {key: 0 for key in COUNTER_KEYS}
    for module_id, final_item in final.items():
        before = initial.get(module_id, {}).get("native_ds_pipeline_stats", {})
        after = final_item.get("native_ds_pipeline_stats", {})
        for key in COUNTER_KEYS:
            delta = int(after.get(key) or 0) - int(before.get(key) or 0)
            # A module reboot resets its monotonic firmware counters.
            totals[key] += delta if delta >= 0 else int(after.get(key) or 0)
    return totals


def classify_rejects(path: pathlib.Path) -> dict[str, Any]:
    records: list[dict[str, int | str]] = []
    first_after_rounds: set[tuple[int, int]] = set()
    valid_results = 0
    response_timeouts = 0
    final_timeouts = 0
    context_mismatches = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                message = str(json.loads(line).get("message") or "")
            except json.JSONDecodeError:
                continue
            if "UWB_RANGING result" in message:
                valid_results += 1
            if "Native DS-TWR RESP wait failed" in message:
                response_timeouts += 1
            if "Native DS-TWR FINAL wait failed" in message:
                final_timeouts += 1
            if "context mismatch" in message:
                context_mismatches += 1
            context_match = CONTEXT_RE.search(message)
            if context_match is not None and (
                int(context_match.group("flags"), 16) & 0x01
            ):
                first_after_rounds.add(
                    (
                        int(context_match.group("tag")),
                        int(context_match.group("round")),
                    )
                )
            match = REJECT_RE.search(message)
            if match is None:
                continue
            record: dict[str, int | str] = {"reason": match.group("reason")}
            for key in ("tag", "anchor", "seq", "round", "slot", "first"):
                record[key] = int(match.group(key))
            record["flags"] = int(match.group("flags"), 16)
            records.append(record)

    classes: collections.Counter[str] = collections.Counter()
    anchors: collections.Counter[str] = collections.Counter()
    first_anchors: collections.Counter[str] = collections.Counter()
    reasons: collections.Counter[str] = collections.Counter()
    slots: collections.Counter[str] = collections.Counter()
    for item in records:
        flags = int(item["flags"])
        key = (int(item["tag"]), int(item["round"]))
        if flags & 0x02:
            category = "geometry_exchange"
        elif flags & 0x01:
            category = "first_slot_after_geometry"
        elif key in first_after_rounds:
            category = "later_slot_same_post_geometry_round"
        else:
            category = "ordinary_round"
        classes[category] += 1
        anchors[str(item["anchor"])] += 1
        first_anchors[str(item["first"])] += 1
        reasons[str(item["reason"])] += 1
        slots[str(item["slot"])] += 1
    return {
        "valid_results": valid_results,
        "response_timeouts": response_timeouts,
        "final_timeouts": final_timeouts,
        "context_mismatches": context_mismatches,
        "total": len(records),
        "classes": dict(sorted(classes.items())),
        "anchors": dict(sorted(anchors.items())),
        "first_anchors": dict(sorted(first_anchors.items())),
        "slots": dict(sorted(slots.items(), key=lambda pair: int(pair[0]))),
        "reasons": dict(sorted(reasons.items())),
    }


def capture_summary(capture: pathlib.Path) -> dict[str, Any]:
    if capture.name.endswith(".logs.jsonl"):
        log_path = capture
        stem = capture.name.removesuffix(".logs.jsonl")
    else:
        stem = capture.name.removesuffix(".metadata.json")
        log_path = capture.with_name(stem + ".logs.jsonl")
    metadata_path = log_path.with_name(stem + ".metadata.json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    return {
        "capture": stem,
        "duration_sec": metadata.get("duration_sec"),
        "event_counts": metadata.get("event_counts", {}),
        "firmware_counter_deltas": counter_totals(metadata_path),
        "source_rejects": classify_rejects(log_path),
    }


def main() -> int:
    args = parse_args()
    summaries = [capture_summary(path.resolve()) for path in args.captures]
    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
        return 0
    for item in summaries:
        rejects = item["source_rejects"]
        print(item["capture"])
        print(
            f"  duration={item['duration_sec']:.1f}s "
            f"valid_results={rejects['valid_results']} "
            f"response_timeouts={rejects['response_timeouts']} "
            f"final_timeouts={rejects['final_timeouts']} "
            f"source_rejects={rejects['total']}"
        )
        print(f"  reject classes={rejects['classes']}")
        print(f"  reject anchors={rejects['anchors']}")
        print(f"  first anchors={rejects['first_anchors']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
