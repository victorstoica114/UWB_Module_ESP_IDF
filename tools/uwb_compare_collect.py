#!/usr/bin/env python3
"""Collect deduplicated field data for DS-TWR, FlexTDOA, or Passive DS-TWR."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request
from typing import Any


PROTOCOL_MODES = {
    "ds_twr": "uwb_ranging",
    "flextdoa": "uwb_flex_tdoa",
    "passive_ds": "uwb_passive_ds_twr",
}

RANGING_RESULT_RE = re.compile(
    r"\bUWB_RANGING result\s+tag=(?P<tag>\d+)\s+"
    r"anchor=(?P<anchor>\d+)\s+seq=(?P<seq>\d+)\s+"
    r"distance=(?P<distance>[+-]?(?:\d+(?:\.\d*)?|\.\d+))\s+m\b"
)

STATUS_KEYS = (
    "module_id",
    "hostname",
    "version",
    "http_status_online",
    "wifi_connected",
    "wifi_connected_rssi",
    "runtime_mode_name",
    "runtime_tag_id",
    "runtime_anchor_ids",
    "runtime_flex_tdoa_geometry_fixed",
    "runtime_flex_tdoa_geometry_generation",
    "runtime_flex_tdoa_anchor_x_mm",
    "runtime_flex_tdoa_anchor_y_mm",
    "runtime_ranging_slot_ms",
    "runtime_ranging_round_gap_ms",
    "runtime_ranging_rx_slice_ms",
    "runtime_ranging_rx_timeout_ms",
    "runtime_ranging_resp_delay_ms",
    "runtime_ranging_final_delay_ms",
    "runtime_ranging_auto_rx_delay_uus",
    "runtime_flex_tdoa_guard_us",
    "runtime_flex_tdoa_request_subslot_us",
    "runtime_flex_tdoa_request_process_us",
    "runtime_flex_tdoa_response_subslot_us",
    "runtime_flex_tdoa_response_process_us",
    "runtime_passive_ds_schedule",
    "runtime_passive_ds_slot_ms",
    "runtime_passive_ds_round_gap_ms",
    "runtime_passive_ds_rx_slice_ms",
    "runtime_passive_ds_rx_timeout_ms",
    "runtime_passive_ds_resp_delay_us",
    "runtime_passive_ds_final_delay_us",
    "runtime_passive_ds_auto_rx_delay_uus",
    "runtime_passive_ds_calibration_enabled",
    "runtime_passive_ds_calibration_generation",
    "runtime_passive_ds_anchor_bias_mm",
    "runtime_passive_ds_range_bias_mm",
    "runtime_radio_channel",
    "uwb_active_antenna_delay",
    "resource_temperature_c",
    "boot_guard_boot_count",
    "boot_guard_validated",
)


def fetch_json(url: str, timeout: float = 3.0) -> dict[str, Any]:
    request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def received_at(item: dict[str, Any], captured_at: float) -> float:
    explicit = finite_float(item.get("received_at"))
    if explicit is not None:
        return explicit
    age_sec = finite_float(item.get("age_sec"))
    return captured_at - max(0.0, age_sec or 0.0)


def clean_item(item: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in item.items()
        if key not in {"raw", "stats"} and value is not None
    }


def status_summary(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        {key: item.get(key) for key in STATUS_KEYS if key in item}
        for item in sorted(
            snapshot.get("statuses", []),
            key=lambda status: int(status.get("module_id") or 0),
        )
    ]


def ready_modules(snapshot: dict[str, Any], runtime_mode: str) -> int:
    return sum(
        1
        for item in snapshot.get("statuses", [])
        if item.get("http_status_online")
        and item.get("wifi_connected")
        and item.get("runtime_mode_name") == runtime_mode
    )


def wait_for_runtime(
    snapshot_url: str,
    runtime_mode: str,
    expected_modules: int,
    timeout_sec: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    last_error = ""
    while time.monotonic() < deadline:
        try:
            snapshot = fetch_json(snapshot_url)
            ready = ready_modules(snapshot, runtime_mode)
            if (
                ready >= expected_modules
                and int(snapshot.get("telemetry_client_count") or 0)
                >= expected_modules
            ):
                return snapshot
            last_error = (
                f"{ready}/{expected_modules} modules in {runtime_mode}, "
                f"{snapshot.get('telemetry_client_count', 0)} telemetry clients"
            )
        except (OSError, ValueError, urllib.error.URLError) as exc:
            last_error = str(exc)
        time.sleep(1.0)
    raise RuntimeError(f"runtime did not become ready: {last_error}")


def write_event(
    handle: Any,
    *,
    kind: str,
    protocol: str,
    block: str,
    captured_at: float,
    event_received_at: float,
    payload: dict[str, Any],
) -> None:
    record = {
        "kind": kind,
        "protocol": protocol,
        "block": block,
        "captured_at": captured_at,
        "received_at": event_received_at,
        **payload,
    }
    handle.write(json.dumps(record, separators=(",", ":"), sort_keys=True))
    handle.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect one controlled UWB comparison block."
    )
    parser.add_argument("--protocol", choices=sorted(PROTOCOL_MODES), required=True)
    parser.add_argument("--block", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--dashboard", default="http://127.0.0.1:8780")
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--warmup-sec", type=float, default=15.0)
    parser.add_argument("--poll-hz", type=float, default=50.0)
    parser.add_argument("--expected-modules", type=int, default=5)
    parser.add_argument("--ready-timeout-sec", type=float, default=90.0)
    parser.add_argument(
        "--log-output",
        default="",
        help=(
            "Optional JSONL file for DS-TWR timing summaries and failures "
            "captured concurrently from the dashboard log API."
        ),
    )
    return parser.parse_args()


def relevant_timing_log(item: dict[str, Any]) -> bool:
    message = str(item.get("message") or "")
    return (
        "UWB_RANGING native summary" in message
        or "UWB_RANGING native tag initiator active" in message
        or "UWB_RANGING native anchor responder active" in message
        or (
            "Native DS-TWR" in message
            and (
                "failed" in message
                or "mismatch" in message
            )
        )
        or "PASSIVE_DS anchor schedule=" in message
        or "PASSIVE_DS solver pos=" in message
        or "PASSIVE_DS runtime source=" in message
        or "PASSIVE_DS receive-only tag active" in message
        or (
            "PASSIVE_DS" in message
            and (
                "failed" in message
                or "timeout" in message
            )
        )
    )


def capture_timing_logs(
    log_url: str,
    cursor: int,
    handle: Any,
) -> tuple[int, int, list[dict[str, Any]]]:
    captured = 0
    ranging_items: list[dict[str, Any]] = []
    while True:
        response = fetch_json(
            f"{log_url}?after={max(0, cursor)}&limit=8000",
            timeout=3.0,
        )
        logs = response.get("logs", [])
        if not logs:
            break
        for item in logs:
            cursor = max(cursor, int(item.get("id") or 0))
            match = RANGING_RESULT_RE.search(str(item.get("message") or ""))
            if match is not None:
                ranging_items.append(
                    {
                        "tag_id": int(match.group("tag")),
                        "anchor_id": int(match.group("anchor")),
                        "seq": int(match.group("seq")),
                        "distance_m": float(match.group("distance")),
                        "log_id": item.get("id"),
                        "source_module_id": item.get("module_id"),
                        "received_at": item.get("received_at"),
                    }
                )
            if relevant_timing_log(item):
                handle.write(
                    json.dumps(
                        clean_item(item),
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                )
                handle.write("\n")
                captured += 1
        if len(logs) < 8000:
            break
    return cursor, captured, ranging_items


def main() -> int:
    args = parse_args()
    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    events_path = output_dir / f"{args.block}.jsonl"
    metadata_path = output_dir / f"{args.block}.metadata.json"
    snapshot_url = args.dashboard.rstrip("/") + "/api/snapshot"
    log_url = args.dashboard.rstrip("/") + "/api/logs"
    position_events_url = (
        args.dashboard.rstrip("/") + "/api/position-events"
    )
    log_path = (
        pathlib.Path(args.log_output).expanduser().resolve()
        if args.log_output
        else None
    )
    runtime_mode = PROTOCOL_MODES[args.protocol]

    initial = wait_for_runtime(
        snapshot_url,
        runtime_mode,
        max(1, args.expected_modules),
        max(1.0, args.ready_timeout_sec),
    )
    print(
        f"{args.block}: runtime ready, warmup {args.warmup_sec:.1f}s",
        flush=True,
    )
    warmup_deadline = time.monotonic() + max(0.0, args.warmup_sec)
    while time.monotonic() < warmup_deadline:
        time.sleep(min(0.25, warmup_deadline - time.monotonic()))

    last_snapshot = fetch_json(snapshot_url)
    initial_position_events = fetch_json(
        f"{position_events_url}?after=0&limit=1"
    )
    position_cursor = int(
        initial_position_events.get("next_event_id") or 0
    )
    log_cursor = max(
        0,
        int(last_snapshot.get("next_log_id") or 0) - 1,
    )
    started_at = time.time()
    started_monotonic = time.monotonic()
    deadline = started_monotonic + max(1.0, args.duration_sec)
    interval = 1.0 / max(1.0, args.poll_hz)
    next_poll = started_monotonic
    next_status = started_monotonic
    next_log_capture = started_monotonic
    next_position_capture = started_monotonic
    seen_ds: set[tuple[int, int, int]] = set()
    seen_tdoa: set[tuple[int, int, int, int, int]] = set()
    seen_anchor: set[tuple[int, int, int, int]] = set()
    seen_position: set[tuple[int, int]] = set()
    counters = {
        "ds_range": 0,
        "tdoa_observation": 0,
        "anchor_range": 0,
        "local_position": 0,
        "status": 0,
        "poll_error": 0,
        "timing_log": 0,
        "timing_log_error": 0,
    }

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
    with (
        events_path.open("w", encoding="utf-8", buffering=1) as handle,
        (
            log_path.open("w", encoding="utf-8", buffering=1)
            if log_path is not None
            else open("/dev/null", "w", encoding="utf-8")
        ) as log_handle,
    ):
        while time.monotonic() < deadline:
            now_monotonic = time.monotonic()
            if now_monotonic < next_poll:
                time.sleep(next_poll - now_monotonic)
            captured_at = time.time()
            next_poll += interval
            if next_poll < time.monotonic() - interval:
                next_poll = time.monotonic()

            try:
                snapshot = fetch_json(snapshot_url)
                last_snapshot = snapshot
            except (OSError, ValueError, urllib.error.URLError) as exc:
                counters["poll_error"] += 1
                if counters["poll_error"] <= 5:
                    print(f"{args.block}: snapshot error: {exc}", file=sys.stderr)
                continue

            if log_path is not None and time.monotonic() >= next_log_capture:
                try:
                    log_cursor, captured, log_ranges = capture_timing_logs(
                        log_url,
                        log_cursor,
                        log_handle,
                    )
                    counters["timing_log"] += captured
                    if args.protocol == "ds_twr":
                        for item in log_ranges:
                            key = (
                                int(item["tag_id"]),
                                int(item["anchor_id"]),
                                int(item["seq"]),
                            )
                            if key in seen_ds:
                                continue
                            seen_ds.add(key)
                            event_time = received_at(item, captured_at)
                            if event_time + 0.05 < started_at:
                                continue
                            write_event(
                                handle,
                                kind="ds_range",
                                protocol=args.protocol,
                                block=args.block,
                                captured_at=captured_at,
                                event_received_at=event_time,
                                payload=clean_item(item),
                            )
                            counters["ds_range"] += 1
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    counters["timing_log_error"] += 1
                    if counters["timing_log_error"] <= 5:
                        print(
                            f"{args.block}: timing log error: {exc}",
                            file=sys.stderr,
                        )
                next_log_capture += 1.0

            if time.monotonic() >= next_position_capture:
                try:
                    while True:
                        response = fetch_json(
                            f"{position_events_url}?after={position_cursor}"
                            "&limit=512",
                            timeout=3.0,
                        )
                        position_events = response.get("events", [])
                        if not position_events:
                            break
                        for item in position_events:
                            position_cursor = max(
                                position_cursor,
                                int(
                                    item.get(
                                        "position_stream_event_id"
                                    )
                                    or 0
                                ),
                            )
                            if item.get("position_stream_type") not in (
                                "flextdoa_position",
                                "passive_ds_position",
                            ):
                                continue
                            event_time = received_at(
                                item, captured_at
                            )
                            if event_time + 0.05 < started_at:
                                continue
                            key = (
                                int(item.get("module_id") or 0),
                                int(
                                    item.get("position_event_id")
                                    or item.get("slot_id")
                                    or 0
                                ),
                            )
                            if key in seen_position:
                                continue
                            seen_position.add(key)
                            write_event(
                                handle,
                                kind="local_position",
                                protocol=args.protocol,
                                block=args.block,
                                captured_at=captured_at,
                                event_received_at=event_time,
                                payload=clean_item(item),
                            )
                            counters["local_position"] += 1
                        if len(position_events) < 512:
                            break
                except (
                    OSError,
                    ValueError,
                    urllib.error.URLError,
                ) as exc:
                    counters["poll_error"] += 1
                    if counters["poll_error"] <= 5:
                        print(
                            f"{args.block}: position stream error: {exc}",
                            file=sys.stderr,
                        )
                next_position_capture += 0.1

            for item in snapshot.get("ranging", {}).get("distances", {}).values():
                event_time = received_at(item, captured_at)
                if event_time + 0.05 < started_at:
                    continue
                key = (
                    int(item.get("tag_id") or 0),
                    int(item.get("anchor_id") or 0),
                    int(item.get("seq") or 0),
                )
                if key in seen_ds:
                    continue
                seen_ds.add(key)
                write_event(
                    handle,
                    kind="ds_range",
                    protocol=args.protocol,
                    block=args.block,
                    captured_at=captured_at,
                    event_received_at=event_time,
                    payload=clean_item(item),
                )
                counters["ds_range"] += 1

            tdoa = snapshot.get("tdoa", {})
            for item in tdoa.get("recent_observations", []):
                event_time = received_at(item, captured_at)
                if event_time + 0.05 < started_at:
                    continue
                key = (
                    int(item.get("tag_id") or 0),
                    int(item.get("initiator_id") or 0),
                    int(item.get("responder_id") or 0),
                    int(item.get("slot_id") or 0),
                    int(item.get("responder_index") or 0),
                )
                if key in seen_tdoa:
                    continue
                seen_tdoa.add(key)
                write_event(
                    handle,
                    kind="tdoa_observation",
                    protocol=args.protocol,
                    block=args.block,
                    captured_at=captured_at,
                    event_received_at=event_time,
                    payload=clean_item(item),
                )
                counters["tdoa_observation"] += 1

            for item in tdoa.get("recent_anchor_ranges", []):
                event_time = received_at(item, captured_at)
                if event_time + 0.05 < started_at:
                    continue
                key = (
                    int(item.get("initiator_id") or 0),
                    int(item.get("responder_id") or 0),
                    int(item.get("slot_id") or 0),
                    int(item.get("seq") or 0),
                )
                if key in seen_anchor:
                    continue
                seen_anchor.add(key)
                write_event(
                    handle,
                    kind="anchor_range",
                    protocol=args.protocol,
                    block=args.block,
                    captured_at=captured_at,
                    event_received_at=event_time,
                    payload=clean_item(item),
                )
                counters["anchor_range"] += 1

            for item in tdoa.get("local_positions", {}).values():
                event_time = received_at(item, captured_at)
                if event_time + 0.05 < started_at:
                    continue
                key = (
                    int(item.get("module_id") or 0),
                    int(
                        item.get("position_event_id")
                        or item.get("slot_id")
                        or item.get("uptime_ms")
                        or 0
                    ),
                )
                if key in seen_position:
                    continue
                seen_position.add(key)
                write_event(
                    handle,
                    kind="local_position",
                    protocol=args.protocol,
                    block=args.block,
                    captured_at=captured_at,
                    event_received_at=event_time,
                    payload=clean_item(item),
                )
                counters["local_position"] += 1

            if time.monotonic() >= next_status:
                write_event(
                    handle,
                    kind="status",
                    protocol=args.protocol,
                    block=args.block,
                    captured_at=captured_at,
                    event_received_at=captured_at,
                    payload={
                        "client_count": snapshot.get("client_count"),
                        "telemetry_client_count": snapshot.get(
                            "telemetry_client_count"
                        ),
                        "modules": status_summary(snapshot),
                    },
                )
                counters["status"] += 1
                next_status += 1.0

            elapsed = time.monotonic() - started_monotonic
            if int(elapsed) > 0 and int(elapsed) % 10 == 0:
                marker = int(elapsed)
                if counters.get("_last_marker") != marker:
                    counters["_last_marker"] = marker
                    print(
                        f"{args.block}: {elapsed:5.1f}/{args.duration_sec:.1f}s "
                        f"ds={counters['ds_range']} "
                        f"tdoa={counters['tdoa_observation']} "
                        f"pos={counters['local_position']}",
                        flush=True,
                    )

        if log_path is not None:
            try:
                final_captured_at = time.time()
                log_cursor, captured, log_ranges = capture_timing_logs(
                    log_url,
                    log_cursor,
                    log_handle,
                )
                counters["timing_log"] += captured
                if args.protocol == "ds_twr":
                    for item in log_ranges:
                        key = (
                            int(item["tag_id"]),
                            int(item["anchor_id"]),
                            int(item["seq"]),
                        )
                        if key in seen_ds:
                            continue
                        seen_ds.add(key)
                        event_time = received_at(item, final_captured_at)
                        if event_time + 0.05 < started_at:
                            continue
                        write_event(
                            handle,
                            kind="ds_range",
                            protocol=args.protocol,
                            block=args.block,
                            captured_at=final_captured_at,
                            event_received_at=event_time,
                            payload=clean_item(item),
                        )
                        counters["ds_range"] += 1
            except (OSError, ValueError, urllib.error.URLError) as exc:
                counters["timing_log_error"] += 1
                print(
                    f"{args.block}: final timing log error: {exc}",
                    file=sys.stderr,
                )

    ended_at = time.time()
    counters.pop("_last_marker", None)
    metadata = {
        "schema_version": 1,
        "block": args.block,
        "protocol": args.protocol,
        "runtime_mode": runtime_mode,
        "started_at": started_at,
        "ended_at": ended_at,
        "duration_sec": ended_at - started_at,
        "warmup_sec": args.warmup_sec,
        "poll_hz": args.poll_hz,
        "expected_modules": args.expected_modules,
        "events_file": events_path.name,
        "timing_log_file": log_path.name if log_path is not None else None,
        "event_counts": counters,
        "ground_truth": {
            "coordinate_convention": "A2=(0,0), A3=(0,3), A4=(3,0), A5=(3,3)",
            "anchors_m": {
                "2": [0.0, 0.0],
                "3": [0.0, 3.0],
                "4": [3.0, 0.0],
                "5": [3.0, 3.0],
            },
            "tag_m": [1.5, 1.5],
            "survey_tolerance_m": 0.002,
        },
        "initial_status": status_summary(initial),
        "final_status": status_summary(last_snapshot),
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{args.block}: complete, events={events_path}, counts={counters}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
