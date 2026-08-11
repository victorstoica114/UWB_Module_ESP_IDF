#!/usr/bin/env python3
"""Read-only field capture for dynamic UWB + IMU + RTK experiments."""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import math
import pathlib
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


PROTOCOL_RUNTIME = {
    "flextdoa": "uwb_flex_tdoa",
    "passive_ds": "uwb_passive_ds_twr",
    "native_ds": "uwb_ranging",
}
CAPTURE_SNAPSHOT_PATH = "/api/capture-snapshot"
LEGACY_SNAPSHOT_PATH = "/api/snapshot"


def fetch_json(url: str, timeout_sec: float = 3.0) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json", "Cache-Control": "no-cache"},
    )
    with urllib.request.urlopen(request, timeout=timeout_sec) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object from {url}")
    return payload


def endpoint(base: str, path: str, **query: int) -> str:
    result = base.rstrip("/") + path
    if query:
        result += "?" + urllib.parse.urlencode(query)
    return result


def fetch_capture_snapshot(
    base: str,
    timeout_sec: float = 3.0,
    *,
    preferred_path: str | None = None,
    fetcher: Any = fetch_json,
) -> tuple[dict[str, Any], str]:
    """Fetch the compact capture snapshot, falling back on legacy 404.

    The returned path should be passed back as ``preferred_path`` on the next
    poll.  This remembers a legacy dashboard choice instead of probing the
    unsupported compact route during every capture cycle.
    """

    selected = preferred_path or CAPTURE_SNAPSHOT_PATH
    candidates = [selected]
    if selected != LEGACY_SNAPSHOT_PATH:
        candidates.append(LEGACY_SNAPSHOT_PATH)
    for index, path in enumerate(candidates):
        try:
            return fetcher(endpoint(base, path), timeout_sec), path
        except urllib.error.HTTPError as exc:
            if exc.code != 404 or index + 1 >= len(candidates):
                raise
    raise RuntimeError("snapshot endpoint selection exhausted")


def accel_cursor(payload: dict[str, Any], previous: int = 0) -> int:
    sample_ids = [
        int(item.get("sample_id") or 0)
        for item in payload.get("samples") or []
    ]
    next_id = int(payload.get("next_id") or 0)
    candidate = max(sample_ids + [max(0, next_id - 1)])
    # The dashboard starts IDs again at one after a restart.
    return candidate if next_id and next_id <= previous else max(previous, candidate)


def position_cursor(payload: dict[str, Any], previous: int = 0) -> int:
    event_ids = [
        int(item.get("position_stream_event_id") or 0)
        for item in payload.get("events") or []
    ]
    candidate = max(event_ids + [int(payload.get("next_event_id") or 0)])
    return candidate if candidate and candidate < previous else max(previous, candidate)


def gps_cursor(payload: dict[str, Any], previous: int = 0) -> int:
    event_ids = [
        int(item.get("gps_event_id") or 0)
        for item in payload.get("samples") or []
    ]
    candidate = max(event_ids + [max(0, int(payload.get("next_id") or 0) - 1)])
    return candidate if candidate and candidate < previous else max(previous, candidate)


def measurement_cursor(payload: dict[str, Any], previous: int = 0) -> int:
    event_ids = [
        int(item.get("uwb_measurement_event_id") or 0)
        for item in payload.get("events") or []
    ]
    candidate = max(event_ids + [max(0, int(payload.get("next_id") or 0) - 1)])
    return candidate if candidate and candidate < previous else max(previous, candidate)


def gps_telemetry_record(sample: dict[str, Any]) -> dict[str, Any]:
    return {
        "module_id": int(sample.get("module_id") or 0),
        "estimated_measurement_wall_ns": int(
            sample.get("estimated_measurement_wall_ns") or 0
        ),
        "measurement_time_source": sample.get("measurement_time_source"),
        "sample_monotonic_us": sample.get("sample_monotonic_us"),
        "gps_gga_count": int(sample.get("gga_sequence") or 0),
        "gps_fix_valid": bool(sample.get("fix_valid")),
        "gps_fix_quality": int(sample.get("fix_quality") or 0),
        "gps_latitude_deg": sample.get("latitude_deg"),
        "gps_longitude_deg": sample.get("longitude_deg"),
        "gps_altitude_m": sample.get("altitude_m"),
        "gps_speed_mps": sample.get("speed_mps"),
        "gps_course_deg": sample.get("course_deg"),
        "gps_hdop": sample.get("hdop"),
        "gps_satellites": sample.get("satellites"),
        "gps_utc_ms_of_day": sample.get("utc_ms_of_day"),
        "gps_utc_date_ddmmyy": sample.get("utc_date_ddmmyy"),
    }


def status_summary(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    keep = {
        "module_id", "hostname", "version", "http_status_online",
        "wifi_connected", "runtime_mode_name", "runtime_tag_id",
        "runtime_anchor_ids", "gps_fix_valid", "gps_fix_quality",
        "runtime_flex_tdoa_geometry_fixed",
        "runtime_flex_tdoa_geometry_generation",
        "runtime_flex_tdoa_anchor_x_mm", "runtime_flex_tdoa_anchor_y_mm",
        "runtime_flex_tdoa_slot_count",
        "runtime_flex_tdoa_responder_count",
        "runtime_passive_ds_solve_mode",
        "gps_fix_quality_text", "gps_gga_count", "gps_latitude_deg",
        "gps_longitude_deg", "gps_altitude_m", "gps_speed_mps",
        "gps_course_deg", "gps_last_fix_age_ms", "gps_hdop",
        "gps_satellites", "gps_utc_time", "gps_utc_date",
        "native_ds_pipeline_stats", "passive_ds_pipeline_stats",
        "wireless_telemetry_dropped", "wireless_telemetry_drop_full",
        "wireless_telemetry_send_failures",
    }
    rows = []
    for status in snapshot.get("statuses") or []:
        if isinstance(status, dict):
            rows.append({key: status[key] for key in keep if key in status})
    return sorted(rows, key=lambda item: int(item.get("module_id") or 9999))


def compact_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    ranging = snapshot.get("ranging") or {}
    tdoa = snapshot.get("tdoa") or {}
    return {
        "client_count": snapshot.get("client_count"),
        "telemetry_client_count": snapshot.get("telemetry_client_count"),
        "statuses": status_summary(snapshot),
        "ranging": {"distances": ranging.get("distances") or {}},
        "tdoa": {
            "observations": tdoa.get("observations") or {},
            "anchor_distances": tdoa.get("anchor_distances") or {},
            "local_positions": tdoa.get("local_positions") or {},
            "local_geometries": tdoa.get("local_geometries") or {},
        },
    }


def gps_records(
    snapshot: dict[str, Any], seen: set[tuple[int, int]], wall_ns: int
) -> list[dict[str, Any]]:
    records = []
    for status in snapshot.get("statuses") or []:
        if not isinstance(status, dict):
            continue
        module_id = int(status.get("module_id") or 0)
        gga_count = int(status.get("gps_gga_count") or 0)
        latitude = status.get("gps_latitude_deg")
        longitude = status.get("gps_longitude_deg")
        try:
            coordinate_ok = math.isfinite(float(latitude)) and math.isfinite(
                float(longitude)
            )
        except (TypeError, ValueError):
            coordinate_ok = False
        key = module_id, gga_count
        if module_id <= 0 or gga_count <= 0 or not coordinate_ok or key in seen:
            continue
        seen.add(key)
        try:
            age_ns = max(
                0,
                int(float(status.get("gps_last_fix_age_ms") or 0) * 1_000_000),
            )
        except (TypeError, ValueError, OverflowError):
            age_ns = 0
        payload = {
            name: value
            for name, value in status.items()
            if name.startswith("gps_") and value is not None
        }
        records.append(
            {
                "module_id": module_id,
                "estimated_measurement_wall_ns": wall_ns - age_ns,
                "measurement_time_source": "collector_wall_minus_gps_fix_age",
                **payload,
            }
        )
    return records


def runtime_warnings(snapshot: dict[str, Any], protocol: str) -> list[str]:
    expected = PROTOCOL_RUNTIME[protocol]
    statuses = [
        item
        for item in snapshot.get("statuses") or []
        if isinstance(item, dict) and item.get("module_id")
    ]
    modes = collections.Counter(
        str(item.get("runtime_mode_name") or "") for item in statuses
    )
    warnings = []
    if not statuses:
        warnings.append("dashboard has no module statuses")
    if modes.get(expected, 0) == 0:
        warnings.append(
            f"no module reports expected runtime {expected}; modes={dict(modes)}"
        )
    offline = [
        int(item["module_id"])
        for item in statuses
        if not item.get("http_status_online")
    ]
    if offline:
        warnings.append(f"HTTP-offline modules: {offline}")
    return warnings


def write_record(
    handle: Any,
    protocol: str,
    capture_id: str,
    kind: str,
    payload: dict[str, Any],
    counts: collections.Counter[str],
) -> None:
    record = {
        "schema_version": 1,
        "capture_id": capture_id,
        "protocol": protocol,
        "kind": kind,
        "collector_wall_ns": time.time_ns(),
        "collector_monotonic_ns": time.monotonic_ns(),
        **payload,
    }
    handle.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    counts[kind] += 1


def position_event_kind(event: dict[str, Any]) -> str:
    """Keep dashboard-fused positions separate from raw UWB measurements."""

    if event.get("kind") == "position_fused" or event.get("imu_fused") is True:
        return "position_fused"
    return "position"


def write_position_event(
    handle: Any,
    protocol: str,
    capture_id: str,
    event: dict[str, Any],
    counts: collections.Counter[str],
) -> None:
    kind = position_event_kind(event)
    payload = dict(event)
    # The capture envelope owns kind even if the dashboard event contains a
    # legacy kind field of its own.
    payload.pop("kind", None)
    write_record(handle, protocol, capture_id, kind, payload, counts)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture accel + UWB positions + RTK during a field walk."
    )
    parser.add_argument("--protocol", choices=sorted(PROTOCOL_RUNTIME), required=True)
    parser.add_argument("--dashboard", default="http://192.168.123.6:8780")
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--poll-hz", type=float, default=20.0)
    parser.add_argument("--snapshot-hz", type=float, default=8.0)
    parser.add_argument("--status-hz", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--output-dir", default="reports/uwb_dynamic_capture")
    parser.add_argument("--name", default="")
    parser.add_argument(
        "--strict-runtime",
        action="store_true",
        help="Abort instead of warning when the selected runtime is not online.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0 or args.poll_hz <= 0 or args.snapshot_hz <= 0:
        print("duration and polling rates must be positive", file=sys.stderr)
        return 2
    capture_id = args.name.strip() or dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{capture_id}.{args.protocol}"
    events_path = output_dir / f"{stem}.jsonl"
    summary_path = output_dir / f"{stem}.summary.json"
    base = args.dashboard.rstrip("/")
    try:
        initial, snapshot_path = fetch_capture_snapshot(base, args.timeout_sec)
        accel_initial = fetch_json(
            endpoint(base, "/api/accel-raw", after=0, limit=1), args.timeout_sec
        )
        position_initial = fetch_json(
            endpoint(base, "/api/position-events", after=0, limit=1),
            args.timeout_sec,
        )
        gps_initial = fetch_json(
            endpoint(base, "/api/gps-events", after=0, limit=1),
            args.timeout_sec,
        )
        measurement_initial = fetch_json(
            endpoint(
                base, "/api/uwb-measurement-events", after=0, limit=1
            ),
            args.timeout_sec,
        )
    except (OSError, ValueError, urllib.error.URLError) as exc:
        print(f"dashboard preflight failed: {exc}", file=sys.stderr)
        return 1
    warnings = runtime_warnings(initial, args.protocol)
    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)
    if warnings and args.strict_runtime:
        return 1

    accel_after = accel_cursor(accel_initial)
    position_after = position_cursor(position_initial)
    gps_after = gps_cursor(gps_initial)
    measurement_after = measurement_cursor(measurement_initial)
    counts: collections.Counter[str] = collections.Counter()
    errors: collections.Counter[str] = collections.Counter()
    gps_quality: dict[int, collections.Counter[str]] = collections.defaultdict(
        collections.Counter
    )
    started_wall_ns = time.time_ns()
    deadline = time.monotonic() + args.duration_sec
    next_snapshot = time.monotonic()
    next_status = time.monotonic()
    last_snapshot = initial
    interrupted = False
    print(
        f"capture {capture_id}: {args.protocol} for {args.duration_sec:.1f}s "
        f"-> {events_path}"
    )
    with events_path.open("w", encoding="utf-8", buffering=1) as handle:
        write_record(
            handle,
            args.protocol,
            capture_id,
            "capture_start",
            {
                "dashboard": base,
                "warnings": warnings,
                "initial_status": status_summary(initial),
            },
            counts,
        )
        try:
            while time.monotonic() < deadline:
                cycle_started = time.monotonic()
                try:
                    payload = fetch_json(
                        endpoint(
                            base,
                            "/api/accel-raw",
                            after=accel_after,
                            limit=20000,
                        ),
                        args.timeout_sec,
                    )
                    for sample in payload.get("samples") or []:
                        write_record(
                            handle,
                            args.protocol,
                            capture_id,
                            "accel",
                            dict(sample),
                            counts,
                        )
                    accel_after = accel_cursor(payload, accel_after)
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    errors["accel"] += 1
                    if errors["accel"] <= 3:
                        print(f"accel poll failed: {exc}", file=sys.stderr)
                try:
                    payload = fetch_json(
                        endpoint(
                            base,
                            "/api/position-events",
                            after=position_after,
                            limit=4096,
                        ),
                        args.timeout_sec,
                    )
                    for event in payload.get("events") or []:
                        write_position_event(
                            handle,
                            args.protocol,
                            capture_id,
                            dict(event),
                            counts,
                        )
                    position_after = position_cursor(payload, position_after)
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    errors["position"] += 1
                    if errors["position"] <= 3:
                        print(f"position poll failed: {exc}", file=sys.stderr)
                try:
                    payload = fetch_json(
                        endpoint(
                            base,
                            "/api/gps-events",
                            after=gps_after,
                            limit=10000,
                        ),
                        args.timeout_sec,
                    )
                    for sample in payload.get("samples") or []:
                        record = gps_telemetry_record(dict(sample))
                        module_id = int(record.get("module_id") or 0)
                        gps_quality[module_id]["samples"] += 1
                        if record.get("gps_fix_valid"):
                            gps_quality[module_id]["valid"] += 1
                        if int(record.get("gps_fix_quality") or 0) == 4:
                            gps_quality[module_id]["rtk_fixed"] += 1
                        write_record(
                            handle,
                            args.protocol,
                            capture_id,
                            "gps_fix",
                            record,
                            counts,
                        )
                    gps_after = gps_cursor(payload, gps_after)
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    errors["gps"] += 1
                    if errors["gps"] <= 3:
                        print(f"gps poll failed: {exc}", file=sys.stderr)
                try:
                    payload = fetch_json(
                        endpoint(
                            base,
                            "/api/uwb-measurement-events",
                            after=measurement_after,
                            limit=65536,
                        ),
                        args.timeout_sec,
                    )
                    if payload.get("cursor_gap"):
                        errors["uwb_measurement_cursor_gap"] += 1
                    for event in payload.get("events") or []:
                        write_record(
                            handle,
                            args.protocol,
                            capture_id,
                            "uwb_measurement",
                            dict(event),
                            counts,
                        )
                    measurement_after = measurement_cursor(
                        payload, measurement_after
                    )
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    errors["uwb_measurement"] += 1
                    if errors["uwb_measurement"] <= 3:
                        print(
                            f"UWB measurement poll failed: {exc}",
                            file=sys.stderr,
                        )
                now = time.monotonic()
                if now >= next_snapshot:
                    try:
                        last_snapshot, snapshot_path = fetch_capture_snapshot(
                            base,
                            args.timeout_sec,
                            preferred_path=snapshot_path,
                        )
                        if now >= next_status:
                            write_record(
                                handle,
                                args.protocol,
                                capture_id,
                                "dashboard_snapshot",
                                {"snapshot": compact_snapshot(last_snapshot)},
                                counts,
                            )
                            next_status = now + 1.0 / max(0.1, args.status_hz)
                    except (OSError, ValueError, urllib.error.URLError) as exc:
                        errors["snapshot"] += 1
                        if errors["snapshot"] <= 3:
                            print(f"snapshot poll failed: {exc}", file=sys.stderr)
                    next_snapshot = now + 1.0 / args.snapshot_hz
                sleep_sec = 1.0 / args.poll_hz - (time.monotonic() - cycle_started)
                if sleep_sec > 0:
                    time.sleep(sleep_sec)
        except KeyboardInterrupt:
            interrupted = True
        write_record(
            handle,
            args.protocol,
            capture_id,
            "capture_end",
            {"interrupted": interrupted},
            counts,
        )

    ended_wall_ns = time.time_ns()
    duration_sec = (ended_wall_ns - started_wall_ns) / 1_000_000_000.0
    summary = {
        "schema_version": 1,
        "capture_id": capture_id,
        "protocol": args.protocol,
        "expected_runtime": PROTOCOL_RUNTIME[args.protocol],
        "dashboard": base,
        "started_wall_ns": started_wall_ns,
        "ended_wall_ns": ended_wall_ns,
        "duration_sec": duration_sec,
        "interrupted": interrupted,
        "events_file": events_path.name,
        "event_counts": dict(counts),
        "event_rates_hz": {
            key: value / duration_sec
            for key, value in counts.items()
            if key not in {"capture_start", "capture_end"} and duration_sec > 0
        },
        "poll_errors": dict(errors),
        "gps_quality_by_module": {
            str(module_id): dict(values)
            for module_id, values in gps_quality.items()
        },
        "warnings": warnings,
        "initial_status": status_summary(initial),
        "final_status": status_summary(last_snapshot),
    }
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"done: counts={dict(counts)} summary={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
