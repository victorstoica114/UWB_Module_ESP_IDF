#!/usr/bin/env python3
"""Measure BQ25792 config write/readback latency through OTA HTTP config."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]

FIELDS = {
    "input_current_ma": {
        "status_key": "charger_input_current_limit_ma",
        "query_key": "input_current_ma",
        "step": 10,
        "min": 100,
        "max": 3300,
    },
    "charge_current_ma": {
        "status_key": "charger_charge_current_limit_ma",
        "query_key": "charge_current_ma",
        "step": 10,
        "min": 50,
        "max": 5000,
    },
    "charge_voltage_mv": {
        "status_key": "charger_charge_voltage_limit_mv",
        "query_key": "charge_voltage_mv",
        "step": 10,
        "min": 3000,
        "max": 18800,
    },
    "minimal_system_voltage_mv": {
        "status_key": "charger_minimal_system_voltage_mv",
        "query_key": "minimal_system_voltage_mv",
        "step": 250,
        "min": 2500,
        "max": 16000,
    },
}


def resolve_project_path(path: str) -> pathlib.Path:
    candidate = pathlib.Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    return candidate.resolve()


def read_ota_token(path: pathlib.Path) -> str:
    match = re.search(r'#define\s+APP_OTA_PASSWORD\s+"([^"]*)"', path.read_text())
    if match is None:
        raise RuntimeError(f"APP_OTA_PASSWORD was not found in {path}")
    token = match.group(1)
    if not token.strip():
        raise RuntimeError("APP_OTA_PASSWORD is empty")
    return token


def base_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/")
    return f"http://{target}"


def get_json(url: str, timeout_sec: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout_sec) as response:
        return json.loads(response.read().decode("utf-8"))


def post_config(
    target: str, params: dict[str, str], token: str, timeout_sec: float
) -> tuple[dict[str, Any], float]:
    query = urllib.parse.urlencode(params, safe=",")
    request = urllib.request.Request(
        f"{base_url(target)}/config/charger?{query}",
        data=b"",
        method="POST",
        headers={"Content-Length": "0", "X-OTA-Token": token},
    )
    start = time.monotonic()
    with urllib.request.urlopen(request, timeout=timeout_sec) as response:
        raw = response.read().decode("utf-8", errors="replace")
        elapsed = time.monotonic() - start
    return json.loads(raw), elapsed


def choose_target_value(field: dict[str, int], current: int) -> int:
    step = field["step"]
    lower = field["min"]
    upper = field["max"]
    candidate = current + step
    if candidate <= upper:
        return candidate
    candidate = current - step
    if candidate >= lower:
        return candidate
    raise ValueError(f"Cannot choose a safe adjacent value for current={current}")


def wait_for_readback(
    target: str,
    status_key: str,
    expected: int,
    timeout_sec: float,
    poll_ms: int,
) -> tuple[dict[str, Any], float, int]:
    start = time.monotonic()
    polls = 0
    last_status: dict[str, Any] = {}
    while True:
        polls += 1
        last_status = get_json(f"{base_url(target)}/status", timeout_sec=2.0)
        if int(last_status.get(status_key) or -1) == expected:
            return last_status, time.monotonic() - start, polls
        if time.monotonic() - start >= timeout_sec:
            return last_status, time.monotonic() - start, polls
        time.sleep(max(1, poll_ms) / 1000.0)


def print_measurement(label: str, result: dict[str, Any]) -> None:
    print(f"{label}:")
    for key, value in result.items():
        print(f"  {key}: {value}")


def run_once(args: argparse.Namespace, token: str) -> int:
    field = FIELDS[args.field]
    status_url = f"{base_url(args.host)}/status"
    before = get_json(status_url, timeout_sec=args.timeout_sec)
    status_key = field["status_key"]
    current = int(before.get(status_key) or 0)
    target_value = args.value if args.value is not None else choose_target_value(field, current)

    print(f"target={args.host} field={args.field} current={current} target={target_value}")
    if args.dry_run:
        return 0

    before_bg_locks = int(before.get("i2c_background_lock_count") or 0)
    before_bg_defer = int(before.get("i2c_background_deferred_count") or 0)
    before_write_count = int(before.get("charger_write_count") or 0)
    before_read_count = int(before.get("charger_read_count") or 0)

    start = time.monotonic()
    response, response_elapsed = post_config(
        args.host,
        {field["query_key"]: str(target_value)},
        token,
        timeout_sec=args.timeout_sec,
    )
    confirmed, confirm_elapsed, polls = wait_for_readback(
        args.host,
        status_key,
        target_value,
        timeout_sec=args.confirm_timeout_sec,
        poll_ms=args.poll_ms,
    )
    total_confirm_elapsed = time.monotonic() - start

    ok = int(confirmed.get(status_key) or -1) == target_value
    result = {
        "ok": ok,
        "endpoint_response_ms": round(response_elapsed * 1000.0, 2),
        "readback_after_response_ms": round(confirm_elapsed * 1000.0, 2),
        "total_confirm_ms": round(total_confirm_elapsed * 1000.0, 2),
        "polls": polls,
        "confirmed_value": confirmed.get(status_key),
        "charger_last_write_error": confirmed.get("charger_last_write_error_name"),
        "charger_last_write_reg": confirmed.get("charger_last_write_reg"),
        "charger_write_count_delta": int(confirmed.get("charger_write_count") or 0)
        - before_write_count,
        "charger_read_count_delta": int(confirmed.get("charger_read_count") or 0)
        - before_read_count,
        "i2c_background_lock_delta": int(confirmed.get("i2c_background_lock_count") or 0)
        - before_bg_locks,
        "i2c_background_deferred_delta": int(
            confirmed.get("i2c_background_deferred_count") or 0
        )
        - before_bg_defer,
        "i2c_realtime_period_us": confirmed.get("i2c_realtime_period_us"),
        "i2c_background_window_us": confirmed.get("i2c_background_window_us"),
        "endpoint_body_ok": response.get("ok"),
        "endpoint_error": response.get("error"),
    }
    print_measurement("write", result)

    if not ok:
        return 2

    if args.restore:
        restore_start = time.monotonic()
        post_config(
            args.host,
            {field["query_key"]: str(current)},
            token,
            timeout_sec=args.timeout_sec,
        )
        restored, restore_confirm, restore_polls = wait_for_readback(
            args.host,
            status_key,
            current,
            timeout_sec=args.confirm_timeout_sec,
            poll_ms=args.poll_ms,
        )
        restore_result = {
            "ok": int(restored.get(status_key) or -1) == current,
            "total_confirm_ms": round((time.monotonic() - restore_start) * 1000.0, 2),
            "readback_after_response_ms": round(restore_confirm * 1000.0, 2),
            "polls": restore_polls,
            "confirmed_value": restored.get(status_key),
            "charger_last_write_error": restored.get("charger_last_write_error_name"),
        }
        print_measurement("restore", restore_result)
        if not restore_result["ok"]:
            return 3

    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Measure BQ25792 write/readback latency on one module."
    )
    parser.add_argument("--host", required=True, help="Module IP or base URL")
    parser.add_argument("--field", choices=sorted(FIELDS), default="input_current_ma")
    parser.add_argument("--value", type=int, help="Explicit value to write")
    parser.add_argument("--no-restore", dest="restore", action="store_false")
    parser.set_defaults(restore=True)
    parser.add_argument("--poll-ms", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--confirm-timeout-sec", type=float, default=5.0)
    parser.add_argument("--secrets", default="secrets.h")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    token = read_ota_token(resolve_project_path(args.secrets))
    try:
        return run_once(args, token)
    except (urllib.error.URLError, TimeoutError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
