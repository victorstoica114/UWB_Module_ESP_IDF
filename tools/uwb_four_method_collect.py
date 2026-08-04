#!/usr/bin/env python3
"""Run a balanced Native/Passive/FlexTDOA campaign with concurrent RTK."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import pathlib
import subprocess
import sys
import time
import urllib.request
from typing import Any


PROTOCOL_MODE = {
    "flextdoa": "flex_tdoa",
    "ds_twr": "ranging",
    "passive_ds": "passive_ds_twr",
}

RUNTIME_PROTOCOL = {
    "uwb_flex_tdoa": "flextdoa",
    "uwb_ranging": "ds_twr",
    "uwb_passive_ds_twr": "passive_ds",
}

BALANCED_ORDERS = (
    ("flextdoa", "ds_twr", "passive_ds"),
    ("ds_twr", "passive_ds", "flextdoa"),
    ("passive_ds", "flextdoa", "ds_twr"),
)


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def fetch_json(url: str, timeout: float = 5.0) -> dict[str, Any]:
    request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def post_json(url: str, payload: dict[str, Any], timeout: float = 30.0) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect balanced static blocks for all three UWB protocols while "
            "recording every new RTK solution from all modules."
        )
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--dashboard", default="http://127.0.0.1:8780")
    parser.add_argument("--duration-sec", type=float, default=300.0)
    parser.add_argument("--warmup-sec", type=float, default=15.0)
    parser.add_argument("--poll-hz", type=float, default=50.0)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--expected-modules", type=int, default=5)
    parser.add_argument("--tag", type=int, default=1)
    parser.add_argument("--anchors", default="2,3,4,5")
    parser.add_argument(
        "--allow-non-fixed-rtk",
        action="store_true",
        help="Do not fail the preflight when one or more modules lack RTK FIX.",
    )
    parser.add_argument(
        "--surveyed",
        action="store_true",
        help=(
            "Use the reference coordinates passed through --anchor and "
            "--tag-reference. Without this flag the dataset is explicitly "
            "precision-only."
        ),
    )
    parser.add_argument("--anchor", action="append", default=[])
    parser.add_argument("--tag-reference", default="1.5:1.5")
    parser.add_argument("--reference-note", default="")
    parser.add_argument("--survey-tolerance-m", type=float, default=0.002)
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="Leave the final protocol active instead of restoring the initial one.",
    )
    return parser.parse_args()


def compact_status(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    keys = (
        "module_id",
        "hostname",
        "http_status_online",
        "wifi_connected",
        "runtime_mode_name",
        "gps_fix_valid",
        "gps_fix_quality",
        "gps_fix_quality_text",
        "gps_satellites",
        "gps_hdop",
        "gps_rtk_age_s",
        "gps_ntrip_stream_active",
        "gps_ntrip_state",
    )
    return [
        {key: item.get(key) for key in keys if key in item}
        for item in sorted(
            snapshot.get("statuses", []),
            key=lambda value: int(value.get("module_id") or 0),
        )
    ]


def require_preflight(snapshot: dict[str, Any], args: argparse.Namespace) -> None:
    statuses = snapshot.get("statuses", [])
    online = [
        item for item in statuses
        if item.get("http_status_online") and item.get("wifi_connected")
    ]
    if len(online) < args.expected_modules:
        raise RuntimeError(
            f"only {len(online)}/{args.expected_modules} modules are online"
        )
    if args.allow_non_fixed_rtk:
        return
    fixed = [
        item for item in online
        if item.get("gps_fix_valid")
        and int(item.get("gps_fix_quality") or 0) == 4
    ]
    if len(fixed) < args.expected_modules:
        missing = sorted(
            int(item.get("module_id") or 0)
            for item in online
            if not item.get("gps_fix_valid")
            or int(item.get("gps_fix_quality") or 0) != 4
        )
        raise RuntimeError(
            f"RTK FIX missing on {len(missing)} module(s): {missing}"
        )


def switch_protocol(args: argparse.Namespace, protocol: str) -> None:
    dashboard = args.dashboard.rstrip("/")
    payload = {
        "target_modules": "all",
        "params": {
            "mode": PROTOCOL_MODE[protocol],
            "tag": str(args.tag),
            "anchors": args.anchors,
            "uwb": "1",
            "reboot": "1",
        },
    }
    response = post_json(f"{dashboard}/api/runtime-config", payload)
    if not response.get("ok"):
        raise RuntimeError(
            f"could not activate {protocol}: "
            f"{json.dumps(response, sort_keys=True)}"
        )


def write_json(path: pathlib.Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def checksums(output_dir: pathlib.Path) -> None:
    rows = []
    for path in sorted(output_dir.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS":
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        rows.append(f"{digest}  {path.relative_to(output_dir)}")
    (output_dir / "SHA256SUMS").write_text(
        "\n".join(rows) + "\n",
        encoding="utf-8",
    )


def block_sequence(repeats: int) -> list[tuple[str, int, int]]:
    sequence = []
    protocol_counts = {protocol: 0 for protocol in PROTOCOL_MODE}
    for cycle in range(max(1, repeats)):
        order = BALANCED_ORDERS[cycle % len(BALANCED_ORDERS)]
        for ordinal, protocol in enumerate(order, start=1):
            protocol_counts[protocol] += 1
            sequence.append((protocol, protocol_counts[protocol], ordinal))
    return sequence


def main() -> int:
    args = parse_args()
    dashboard = args.dashboard.rstrip("/")
    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()
    data_dir = output_dir / "data"
    log_dir = output_dir / "logs"
    data_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    initial = fetch_json(f"{dashboard}/api/snapshot")
    require_preflight(initial, args)
    statuses = initial.get("statuses", [])
    initial_modes = {
        str(item.get("runtime_mode_name") or "")
        for item in statuses
        if item.get("http_status_online")
    }
    restore_protocol = None
    if len(initial_modes) == 1:
        restore_protocol = RUNTIME_PROTOCOL.get(next(iter(initial_modes)))

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "campaign": "four-method UWB and RTK comparison",
        "started_at": utc_now(),
        "duration_sec_per_block": args.duration_sec,
        "warmup_sec_per_switch": args.warmup_sec,
        "poll_hz": args.poll_hz,
        "gps_hz": 1.0,
        "repeats": args.repeats,
        "tag": args.tag,
        "anchors": [int(value) for value in args.anchors.split(",")],
        "ground_truth_available": args.surveyed,
        "reference_note": args.reference_note or (
            "independently surveyed reference"
            if args.surveyed else
            "No independent ground truth; precision/repeatability, delivery, "
            "and cross-method agreement only."
        ),
        "initial_status": compact_status(initial),
        "blocks": [],
    }
    manifest_path = output_dir / "campaign.json"
    write_json(manifest_path, manifest)

    collector = pathlib.Path(__file__).with_name("uwb_compare_collect.py")
    sequence = block_sequence(args.repeats)
    completed = False
    try:
        for block_number, (protocol, repeat, ordinal) in enumerate(sequence, start=1):
            block = f"{block_number:02d}-{protocol}-{repeat}"
            print(
                f"[{block_number}/{len(sequence)}] activating {protocol} "
                f"(balanced-order position {ordinal})",
                flush=True,
            )
            switch_protocol(args, protocol)
            command = [
                sys.executable,
                str(collector),
                "--protocol", protocol,
                "--block", block,
                "--output-dir", str(data_dir),
                "--dashboard", dashboard,
                "--duration-sec", str(args.duration_sec),
                "--warmup-sec", str(args.warmup_sec),
                "--poll-hz", str(args.poll_hz),
                "--gps-hz", "1",
                "--expected-modules", str(args.expected_modules),
                "--log-output", str(log_dir / f"{block}.jsonl"),
                "--tag-reference", args.tag_reference,
                "--survey-tolerance-m", str(args.survey_tolerance_m),
                "--reference-note", manifest["reference_note"],
            ]
            if args.surveyed:
                for anchor in args.anchor:
                    command.extend(("--anchor", anchor))
            else:
                command.append("--no-ground-truth")
            block_started = utc_now()
            subprocess.run(command, check=True)
            metadata_path = data_dir / f"{block}.metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            manifest["blocks"].append({
                "block": block,
                "protocol": protocol,
                "repeat": repeat,
                "balanced_order_position": ordinal,
                "started_at": block_started,
                "completed_at": utc_now(),
                "event_counts": metadata.get("event_counts", {}),
                "events_file": str(pathlib.Path("data") / f"{block}.jsonl"),
                "metadata_file": str(
                    pathlib.Path("data") / f"{block}.metadata.json"
                ),
                "timing_log_file": str(
                    pathlib.Path("logs") / f"{block}.jsonl"
                ),
            })
            write_json(manifest_path, manifest)
        completed = True
    finally:
        if not args.no_restore and restore_protocol is not None:
            print(f"restoring initial protocol {restore_protocol}", flush=True)
            try:
                switch_protocol(args, restore_protocol)
            except Exception as exc:  # Preserve the completed dataset.
                print(f"warning: protocol restore failed: {exc}", file=sys.stderr)
        manifest["completed"] = completed
        manifest["ended_at"] = utc_now()
        try:
            final = fetch_json(f"{dashboard}/api/snapshot")
            manifest["final_status"] = compact_status(final)
        except Exception as exc:
            manifest["final_status_error"] = str(exc)
        write_json(manifest_path, manifest)
        checksums(output_dir)

    print(f"campaign complete: {output_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
