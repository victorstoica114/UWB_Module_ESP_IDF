#!/usr/bin/env python3
"""Upload ESP-IDF firmware to one or more local OTA targets."""

from __future__ import annotations

import argparse
import concurrent.futures
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


def resolve_project_path(path: str) -> pathlib.Path:
    candidate = pathlib.Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    return candidate.resolve()


def normalize_ota_target(target: str | None) -> str | None:
    if target is None:
        return None

    clean = re.split(r"\s+#", target, maxsplit=1)[0].strip()
    if not clean or clean.startswith("#"):
        return None
    return clean


def get_ota_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target if target.endswith("/ota") else target.rstrip("/") + "/ota"
    return f"http://{target}/ota"


def read_ota_targets(path: pathlib.Path) -> list[str]:
    return [
        target
        for target in (normalize_ota_target(line) for line in path.read_text().splitlines())
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


def upload_firmware(
    target: str, firmware: bytes, token: str, timeout_sec: int
) -> tuple[str, bool, str, float]:
    url = get_ota_url(target)
    request = urllib.request.Request(
        url,
        data=firmware,
        method="POST",
        headers={
            "Content-Type": "application/octet-stream",
            "Content-Length": str(len(firmware)),
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Upload build/uwb_esp_idf.bin to ESP32 OTA endpoints."
    )
    parser.add_argument("-HostIp", "--host-ip", default="")
    parser.add_argument("-Hosts", "--hosts", nargs="*", default=[])
    parser.add_argument("-TargetList", "--target-list", default="")
    parser.add_argument("-Firmware", "--firmware", default="build/uwb_esp_idf.bin")
    parser.add_argument("-Secrets", "--secrets", default="secrets.h")
    parser.add_argument("-Parallel", "--parallel", type=int, default=5)
    parser.add_argument("-TimeoutSec", "--timeout-sec", type=int, default=120)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print resolved targets without uploading firmware.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    firmware_path = resolve_project_path(args.firmware)
    secrets_path = resolve_project_path(args.secrets)

    targets: list[str] = []
    has_explicit_targets = False

    if args.host_ip.strip():
        has_explicit_targets = True
        targets.append(args.host_ip)

    if args.hosts:
        has_explicit_targets = True
        targets.extend(args.hosts)

    if args.target_list.strip():
        has_explicit_targets = True
        targets.extend(read_ota_targets(resolve_project_path(args.target_list)))

    if not targets and not has_explicit_targets:
        targets.append("192.168.140.143")

    unique_targets: list[str] = []
    seen: set[str] = set()
    for target in (normalize_ota_target(raw) for raw in targets):
        if target is None or target in seen:
            continue
        unique_targets.append(target)
        seen.add(target)

    if not unique_targets:
        raise RuntimeError("No OTA targets configured")

    parallel = max(1, args.parallel)

    print(
        f"Uploading {firmware_path} to {len(unique_targets)} target(s), "
        f"parallel={parallel}"
    )
    for target in unique_targets:
        print(f"[{target}] {get_ota_url(target)}")

    if args.dry_run:
        print("Dry run complete; no firmware was uploaded")
        return 0

    firmware = firmware_path.read_bytes()
    token = read_ota_token(secrets_path)
    failures: list[str] = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=parallel) as executor:
        future_map = {
            executor.submit(upload_firmware, target, firmware, token, args.timeout_sec): target
            for target in unique_targets
        }
        for future in concurrent.futures.as_completed(future_map):
            target, ok, message, elapsed = future.result()
            status = "completed" if ok else "failed"
            print(f"[{target}] OTA upload {status} in {elapsed:.2f}s")
            if message:
                print(f"[{target}] {message}")
            if not ok:
                failures.append(target)

    if failures:
        print(f"OTA failed for: {', '.join(failures)}", file=sys.stderr)
        return 1

    print("OTA upload completed for all targets")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
