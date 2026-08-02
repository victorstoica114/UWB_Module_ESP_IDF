#!/usr/bin/env python3
"""Flash the validated PX1105R 01.07.33 image through one ESP32."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import lzma
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
SECRETS_PATH = PROJECT_ROOT / "secrets.h"
FIRMWARE_PATH = PROJECT_ROOT / (
    "PX1105R-firmware/"
    "STI_03.06.00-01.07.33_Phoenix_RTK_GPSL1L5_BDSB1B2a_"
    "GalileoE1E5a_GlonassG1_T_CRC_172b_115200_20240111.bin"
)
LOADER_PATH = PROJECT_ROOT / (
    "PX1105R-firmware/validated/"
    "PX1105R_GNSS_Viewer_2.1.147_loader.srec"
)
PACKED_PATH = PROJECT_ROOT / (
    "PX1105R-firmware/validated/PX1105R_01.07.33_viewer.lzma.b64"
)

RAW_SIZE = 1_116_336
RAW_SUM8 = 123
RAW_SHA256 = "15e9384ed510f85045b76cff0e4756138e2dd2beeffcf11a1d5d0c0a2e044968"
LOADER_SIZE = 67_064
LOADER_SHA256 = "e236c1397b73a9cc28499513c3ac5e418ebf7ac389c374717a6210d48beef729"
PACKED_SIZE = 471_243
PACKED_PAYLOAD_SUM8 = 189
PACKED_SHA256 = "44cb71aa9eb45a35fa97726a72ffe57835e7a7d2e33148cd5555260e1a6a4ce8"
PHOENIX_TAG_OFFSET = 1_049_152
LZMA_HEADER_SIZE = 13

EXPECTED_KERNEL = "00030600"
EXPECTED_ODM = "00010721"
EXPECTED_REVISION = "0018010b"
UPLOAD_TIMEOUT_S = 60
FLASH_TIMEOUT_S = 180
VERSION_TIMEOUT_S = 30


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_token(path: pathlib.Path) -> str:
    match = re.search(
        r'#define\s+APP_OTA_PASSWORD\s+"([^"]*)"',
        path.read_text(encoding="utf-8"),
    )
    if match is None or not match.group(1):
        raise RuntimeError(f"APP_OTA_PASSWORD is missing or empty in {path}")
    return match.group(1)


def endpoint(host: str, suffix: str) -> str:
    base = host.rstrip("/")
    if not re.match(r"^https?://", base):
        base = "http://" + base
    return base + suffix


def request_json(
    url: str,
    *,
    method: str = "GET",
    token: str | None = None,
    data: bytes | None = None,
    timeout: int = UPLOAD_TIMEOUT_S,
) -> dict:
    headers: dict[str, str] = {}
    if token is not None:
        headers["X-OTA-Token"] = token
    if data is not None:
        headers["Content-Type"] = "application/octet-stream"
        headers["Content-Length"] = str(len(data))
    request = urllib.request.Request(
        url, data=data, method=method, headers=headers
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = response.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} from {url}: {body.strip()}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Cannot reach {url}: {exc.reason}") from exc
    try:
        return json.loads(body)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Invalid JSON from {url}: {body.strip()}") from exc


def validate_srec(loader: bytes) -> None:
    if len(loader) != LOADER_SIZE or sha256(loader) != LOADER_SHA256:
        raise RuntimeError("The validated PX1105R RAM loader was modified")
    lines = loader.splitlines()
    if not lines or not lines[0].startswith(b"S0") or not lines[-1].startswith(b"S7"):
        raise RuntimeError("Invalid PX1105R S-record loader")
    for number, record in enumerate(lines, 1):
        if not record.startswith(b"S"):
            raise RuntimeError(f"Invalid S-record line {number}")
        try:
            encoded = bytes.fromhex(record[2:].decode("ascii"))
        except (UnicodeDecodeError, ValueError) as exc:
            raise RuntimeError(f"Invalid S-record line {number}") from exc
        if len(encoded) < 2 or (sum(encoded) & 0xFF) != 0xFF:
            raise RuntimeError(f"S-record checksum failure on line {number}")


def load_validated_images() -> tuple[bytes, bytes]:
    loader = LOADER_PATH.read_bytes()
    validate_srec(loader)

    raw = FIRMWARE_PATH.read_bytes()
    if len(raw) != RAW_SIZE or (sum(raw) & 0xFF) != RAW_SUM8:
        raise RuntimeError("Unexpected PX1105R raw firmware size or checksum")
    if sha256(raw) != RAW_SHA256:
        raise RuntimeError("The validated PX1105R raw firmware was modified")

    encoded = b"".join(PACKED_PATH.read_bytes().split())
    try:
        packed = base64.b64decode(encoded, validate=True)
    except ValueError as exc:
        raise RuntimeError("Invalid validated LZMA artifact") from exc
    if len(packed) != PACKED_SIZE or sha256(packed) != PACKED_SHA256:
        raise RuntimeError("The validated PX1105R LZMA image was modified")
    if packed[0] != 0x5D or int.from_bytes(packed[1:5], "little") != 8 * 1024:
        raise RuntimeError("Unexpected PX1105R LZMA properties")
    if int.from_bytes(packed[5:13], "little") != RAW_SIZE:
        raise RuntimeError("Unexpected PX1105R uncompressed image size")
    if (sum(packed[LZMA_HEADER_SIZE:]) & 0xFF) != PACKED_PAYLOAD_SUM8:
        raise RuntimeError("Unexpected PX1105R packed payload checksum")
    if lzma.decompress(packed, format=lzma.FORMAT_ALONE) != raw:
        raise RuntimeError("PX1105R LZMA image does not reproduce the raw firmware")
    if PHOENIX_TAG_OFFSET >= len(raw):
        raise RuntimeError("Invalid Phoenix tag offset")
    return loader, packed


def installed_version(status: dict) -> tuple[str, str, str] | None:
    if not status.get("gps_moving_base_software_version_valid"):
        return None
    return (
        str(status.get("gps_moving_base_software_kernel_version", "")).lower(),
        str(status.get("gps_moving_base_software_odm_version", "")).lower(),
        str(status.get("gps_moving_base_software_revision", "")).lower(),
    )


def expected_version(version: tuple[str, str, str] | None) -> bool:
    return version == (EXPECTED_KERNEL, EXPECTED_ODM, EXPECTED_REVISION)


def wait_for_expected_version(host: str) -> dict:
    deadline = time.monotonic() + VERSION_TIMEOUT_S
    last_status: dict = {}
    while time.monotonic() < deadline:
        last_status = request_json(endpoint(host, "/status"), timeout=5)
        if expected_version(installed_version(last_status)):
            return last_status
        time.sleep(0.5)
    version = installed_version(last_status)
    raise RuntimeError(
        "GNSS restarted, but the expected 00030600/00010721/0018010b "
        f"version was not reported within {VERSION_TIMEOUT_S}s; got {version}"
    )


def flash(host: str, secrets: pathlib.Path) -> dict:
    print("[1/4] Validating the fixed PX1105R loader and firmware")
    loader, packed = load_validated_images()
    token = read_token(secrets)

    print(f"[2/4] Checking ESP32 target {host}")
    before = request_json(endpoint(host, "/status"), timeout=5)
    identity = f"{before.get('hostname', '?')} / module {before.get('module_id', '?')}"
    print(f"      target: {identity}; current GNSS: {installed_version(before)}")
    if expected_version(installed_version(before)):
        print("      PX1105R 01.07.33 is already installed; nothing to flash")
        return {"ok": True, "already_current": True, "target": identity}

    print(
        f"[3/4] Staging loader ({len(loader):,} B) and firmware "
        f"({len(packed):,} B) in ESP32 PSRAM"
    )
    loader_result = request_json(
        endpoint(host, "/gnss/loader"),
        method="POST",
        token=token,
        data=loader,
    )
    firmware_result = request_json(
        endpoint(host, "/gnss/firmware"),
        method="POST",
        token=token,
        data=packed,
    )
    if not loader_result.get("ok") or not firmware_result.get("ok"):
        raise RuntimeError("ESP32 rejected the validated PSRAM staging data")

    print("[4/4] Flashing PX1105R and verifying the reported revision")
    result = request_json(
        endpoint(host, "/gnss/firmware/run"),
        method="POST",
        token=token,
        data=b"",
        timeout=FLASH_TIMEOUT_S,
    )
    if not result.get("ok"):
        raise RuntimeError(f"PX1105R update failed: {json.dumps(result)}")
    after = wait_for_expected_version(host)
    if not after.get("gps_uart_ready") or not after.get("gps_task_running"):
        raise RuntimeError("Firmware is current, but the ESP32 GNSS service is not ready")
    return {
        "ok": True,
        "already_current": False,
        "target": identity,
        "stage": result.get("stage"),
        "feedback": result.get("feedback"),
        "loader_wire_bytes": result.get("loader_bytes"),
        "packed_payload_bytes": result.get("bytes"),
        "version": installed_version(after),
        "gps_uart_ready": after.get("gps_uart_ready"),
        "gps_task_running": after.get("gps_task_running"),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stage the validated loader and firmware in ESP32 PSRAM, flash "
            "PX1105R 01.07.33, and verify the receiver revision."
        )
    )
    parser.add_argument("host", help="Exact ESP32 IP address or hostname")
    parser.add_argument(
        "--secrets",
        type=pathlib.Path,
        default=SECRETS_PATH,
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = flash(args.host, args.secrets.expanduser().resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
