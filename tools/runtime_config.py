#!/usr/bin/env python3
"""Apply runtime config to one or more local UWB module HTTP endpoints."""

from __future__ import annotations

import argparse
import concurrent.futures
import pathlib
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


def resolve_project_path(path: str) -> pathlib.Path:
    candidate = pathlib.Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    return candidate.resolve()


def normalize_target(target: str | None) -> str | None:
    if target is None:
        return None

    clean = re.split(r"\s+#", target, maxsplit=1)[0].strip()
    if not clean or clean.startswith("#"):
        return None
    return clean


def read_targets(path: pathlib.Path) -> list[str]:
    return [
        target
        for target in (normalize_target(line) for line in path.read_text().splitlines())
        if target is not None
    ]


def read_ota_token(path: pathlib.Path) -> str:
    match = re.search(
        r'#define\s+APP_OTA_PASSWORD\s+"([^"]*)"', path.read_text()
    )
    if match is None:
        raise RuntimeError(f"APP_OTA_PASSWORD was not found in {path}")

    token = match.group(1)
    if not token.strip():
        raise RuntimeError("APP_OTA_PASSWORD is empty")
    return token


def get_runtime_url(target: str) -> str:
    if re.match(r"^https?://", target):
        clean = target.rstrip("/")
        if clean.endswith("/config/runtime"):
            return clean
        return clean + "/config/runtime"
    return f"http://{target}/config/runtime"


def send_runtime_config(
    target: str, params: dict[str, str], token: str, timeout_sec: int
) -> tuple[str, bool, str, float]:
    query = urllib.parse.urlencode(params, safe=",")
    url = f"{get_runtime_url(target)}?{query}"
    request = urllib.request.Request(
        url,
        data=b"",
        method="POST",
        headers={
            "Content-Length": "0",
            "X-OTA-Token": token,
        },
    )

    started_at = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            body = response.read().decode("utf-8", errors="replace").strip()
            message = body or f"HTTP {response.status}"
            return target, 200 <= response.status < 300, message, time.monotonic() - started_at
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace").strip()
        message = body or exc.reason
        return target, False, f"HTTP {exc.code}: {message}", time.monotonic() - started_at
    except urllib.error.URLError as exc:
        return target, False, str(exc.reason), time.monotonic() - started_at
    except TimeoutError:
        return target, False, "timed out", time.monotonic() - started_at


def add_optional(params: dict[str, str], key: str, value: str | None) -> None:
    if value is not None and value != "":
        params[key] = value


def collect_targets(args: argparse.Namespace) -> list[str]:
    targets: list[str] = []

    if args.host_ip.strip():
        targets.append(args.host_ip)

    if args.hosts:
        targets.extend(args.hosts)

    if args.target_list.strip():
        targets.extend(read_targets(resolve_project_path(args.target_list)))

    unique_targets: list[str] = []
    seen: set[str] = set()
    for target in (normalize_target(raw) for raw in targets):
        if target is None or target in seen:
            continue
        unique_targets.append(target)
        seen.add(target)

    if not unique_targets:
        raise RuntimeError("No runtime config targets configured")

    return unique_targets


def collect_params(args: argparse.Namespace) -> dict[str, str]:
    params: dict[str, str] = {}

    if args.clear:
        params["clear"] = "1"

    add_optional(params, "mode", args.mode)
    add_optional(params, "tag", args.tag)
    add_optional(params, "anchors", args.anchors)
    add_optional(params, "coordinator", args.coordinator)
    add_optional(params, "uwb", args.uwb)
    add_optional(params, "bno085", args.bno085)
    add_optional(params, "bno085_accel", args.bno085_accel)
    add_optional(params, "bno085_sample_hz", args.bno085_sample_hz)
    add_optional(params, "bno085_sample_ms", args.bno085_sample_ms)
    add_optional(
        params, "bno085_accel_interval_ms", args.bno085_accel_interval_ms
    )
    add_optional(params, "bno085_log_interval_ms", args.bno085_log_interval_ms)
    add_optional(params, "gps", args.gps)
    add_optional(params, "radio_channel", args.radio_channel)
    add_optional(params, "telemetry_port", args.telemetry_port)
    add_optional(params, "survey_rx_ms", args.survey_rx_ms)
    add_optional(params, "survey_delay_ms", args.survey_delay_ms)
    add_optional(params, "survey_slot_ms", args.survey_slot_ms)
    add_optional(params, "survey_gap_ms", args.survey_gap_ms)
    add_optional(params, "survey_log_every", args.survey_log_every)
    add_optional(params, "ranging_slot_ms", args.ranging_slot_ms)
    add_optional(params, "ranging_gap_ms", args.ranging_gap_ms)
    add_optional(params, "ranging_rx_ms", args.ranging_rx_ms)
    add_optional(params, "dt_peer", args.dt_peer)
    add_optional(params, "dt_initiator", args.dt_initiator)
    add_optional(params, "dt_responder", args.dt_responder)
    add_optional(params, "dt_interval_ms", args.dt_interval_ms)
    add_optional(params, "dt_rx_timeout_ms", args.dt_rx_timeout_ms)
    add_optional(params, "dt_resp_delay_ms", args.dt_resp_delay_ms)
    add_optional(params, "dt_final_delay_ms", args.dt_final_delay_ms)
    add_optional(params, "dt_report_delay_ms", args.dt_report_delay_ms)
    add_optional(params, "dt_auto_rx_delay_uus", args.dt_auto_rx_delay_uus)
    add_optional(params, "cal_method", args.cal_method)
    add_optional(params, "cal_ref", args.cal_ref)
    add_optional(params, "cal_dut", args.cal_dut)
    add_optional(params, "cal_three", args.cal_three)
    add_optional(params, "cal_known_mm", args.cal_known_mm)
    add_optional(params, "cal_d01_mm", args.cal_d01_mm)
    add_optional(params, "cal_d02_mm", args.cal_d02_mm)
    add_optional(params, "cal_d12_mm", args.cal_d12_mm)
    add_optional(params, "cal_samples", args.cal_samples)
    add_optional(params, "cal_summary", args.cal_summary)
    add_optional(params, "cal_slot_ms", args.cal_slot_ms)
    add_optional(params, "cal_min_ms", args.cal_min_ms)
    add_optional(params, "cal_round_gap_ms", args.cal_round_gap_ms)
    add_optional(params, "cal_max_ms", args.cal_max_ms)
    add_optional(params, "cal_rx_ms", args.cal_rx_ms)

    for item in args.sets:
        if "=" not in item:
            raise RuntimeError(f"Invalid --set value {item!r}; use KEY=VALUE")
        key, value = item.split("=", 1)
        key = key.strip()
        if not key:
            raise RuntimeError(f"Invalid --set value {item!r}; key is empty")
        params[key] = value.strip()

    if args.reboot:
        params["reboot"] = "1"

    if not params:
        raise RuntimeError("No runtime config parameters provided")

    return params


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply runtime UWB settings through /config/runtime."
    )
    parser.add_argument("-HostIp", "--host-ip", default="")
    parser.add_argument("-Hosts", "--hosts", nargs="*", default=[])
    parser.add_argument("-TargetList", "--target-list", default="")
    parser.add_argument("-Secrets", "--secrets", default="secrets.h")
    parser.add_argument("-Parallel", "--parallel", type=int, default=5)
    parser.add_argument("-TimeoutSec", "--timeout-sec", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--reboot", action="store_true")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--set", dest="sets", action="append", default=[], metavar="KEY=VALUE")

    parser.add_argument("--mode")
    parser.add_argument("--tag")
    parser.add_argument("--anchors")
    parser.add_argument("--coordinator", "--coord", dest="coordinator")
    parser.add_argument("--uwb")
    parser.add_argument("--bno085")
    parser.add_argument("--bno085-accel", dest="bno085_accel")
    parser.add_argument("--bno085-sample-hz", dest="bno085_sample_hz")
    parser.add_argument("--bno085-sample-ms", dest="bno085_sample_ms")
    parser.add_argument(
        "--bno085-accel-interval-ms", dest="bno085_accel_interval_ms"
    )
    parser.add_argument("--bno085-log-interval-ms", dest="bno085_log_interval_ms")
    parser.add_argument("--gps")
    parser.add_argument("--radio-channel", "--uwb-channel", dest="radio_channel")
    parser.add_argument("--telemetry-port", "--tel-port", dest="telemetry_port")
    parser.add_argument("--survey-rx-ms", dest="survey_rx_ms")
    parser.add_argument("--survey-delay-ms", dest="survey_delay_ms")
    parser.add_argument("--survey-slot-ms", dest="survey_slot_ms")
    parser.add_argument("--survey-gap-ms", dest="survey_gap_ms")
    parser.add_argument("--survey-log-every", dest="survey_log_every")
    parser.add_argument("--ranging-slot-ms", dest="ranging_slot_ms")
    parser.add_argument("--ranging-gap-ms", dest="ranging_gap_ms")
    parser.add_argument("--ranging-rx-ms", dest="ranging_rx_ms")
    parser.add_argument("--dt-peer", dest="dt_peer")
    parser.add_argument("--dt-initiator", dest="dt_initiator")
    parser.add_argument("--dt-responder", dest="dt_responder")
    parser.add_argument("--dt-interval-ms", dest="dt_interval_ms")
    parser.add_argument("--dt-rx-timeout-ms", dest="dt_rx_timeout_ms")
    parser.add_argument("--dt-resp-delay-ms", dest="dt_resp_delay_ms")
    parser.add_argument("--dt-final-delay-ms", dest="dt_final_delay_ms")
    parser.add_argument("--dt-report-delay-ms", dest="dt_report_delay_ms")
    parser.add_argument("--dt-auto-rx-delay-uus", dest="dt_auto_rx_delay_uus")
    parser.add_argument("--cal-method", dest="cal_method")
    parser.add_argument("--cal-ref", dest="cal_ref")
    parser.add_argument("--cal-dut", dest="cal_dut")
    parser.add_argument("--cal-three", dest="cal_three")
    parser.add_argument("--cal-known-mm", dest="cal_known_mm")
    parser.add_argument("--cal-d01-mm", dest="cal_d01_mm")
    parser.add_argument("--cal-d02-mm", dest="cal_d02_mm")
    parser.add_argument("--cal-d12-mm", dest="cal_d12_mm")
    parser.add_argument("--cal-samples", dest="cal_samples")
    parser.add_argument("--cal-summary", dest="cal_summary")
    parser.add_argument("--cal-slot-ms", dest="cal_slot_ms")
    parser.add_argument("--cal-min-ms", dest="cal_min_ms")
    parser.add_argument("--cal-round-gap-ms", dest="cal_round_gap_ms")
    parser.add_argument("--cal-max-ms", dest="cal_max_ms")
    parser.add_argument("--cal-rx-ms", dest="cal_rx_ms")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    targets = collect_targets(args)
    params = collect_params(args)
    parallel = max(1, args.parallel)

    query = urllib.parse.urlencode(params, safe=",")
    print(
        f"Applying runtime config to {len(targets)} target(s), "
        f"parallel={parallel}"
    )
    print(f"query: {query}")
    for target in targets:
        print(f"[{target}] {get_runtime_url(target)}")

    if args.dry_run:
        print("Dry run complete; no runtime config was sent")
        return 0

    token = read_ota_token(resolve_project_path(args.secrets))
    failures: list[str] = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=parallel) as executor:
        future_map = {
            executor.submit(send_runtime_config, target, params, token, args.timeout_sec): target
            for target in targets
        }
        for future in concurrent.futures.as_completed(future_map):
            target, ok, message, elapsed = future.result()
            status = "completed" if ok else "failed"
            print(f"[{target}] runtime config {status} in {elapsed:.2f}s")
            if message:
                print(f"[{target}] {message}")
            if not ok:
                failures.append(target)

    if failures:
        print(f"Runtime config failed for: {', '.join(failures)}", file=sys.stderr)
        return 1

    print("Runtime config completed for all targets")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
