#!/usr/bin/env python3
"""Local browser dashboard for UWB module logs, status, and runtime config."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import queue
import re
import socket
import socketserver
import struct
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
LOG_RE = re.compile(
    r"^\[(?P<host>[^\]]+)\]\s+\[\s*(?P<uptime>\d+)\s+ms\]\s+"
    r"\[(?P<level>[DIWE])\]\[(?P<tag>[^\]]+)\]\s*(?P<message>.*)$"
)
MODULE_RE = re.compile(r"uwb-module-(?P<id>\d+)")
ACCEL_RE = re.compile(
    r"\bBNO085 accel x=(?P<x>[-+]?\d+(?:\.\d+)?) "
    r"y=(?P<y>[-+]?\d+(?:\.\d+)?) "
    r"z=(?P<z>[-+]?\d+(?:\.\d+)?) m/s\^2 "
    r"accuracy=(?P<accuracy>\d+) reports=(?P<reports>\d+)"
)
TELEMETRY_RE = re.compile(
    r"^T,(?P<host>[^,]+),(?P<uptime>\d+),(?P<topic>[^,]+),(?P<payload>.*)$"
)
SHORT_ACCEL_RE = re.compile(
    r"^A,(?P<module>\d+),(?P<uptime>\d+),(?P<x>-?\d+),(?P<y>-?\d+),"
    r"(?P<z>-?\d+),(?P<accuracy>\d+),(?P<reports>\d+)$"
)
CAL_SAMPLE_RE = re.compile(
    r"\bUWB CAL sample (?P<method>\S+) pair=(?P<src>\d+)->(?P<dst>\d+) "
    r"sample=(?P<sample>\d+) seq=(?P<seq>\d+) "
    r"distance=(?P<distance>[-+]?\d+(?:\.\d+)?) m"
)
RANGING_RE = re.compile(
    r"\bUWB_RANGING result\s+tag=(?P<tag>\d+)\s+anchor=(?P<anchor>\d+)\s+"
    r"seq=(?P<seq>\d+)\s+distance=(?P<distance>[-+]?\d+(?:\.\d+)?)\s+m"
)
FLOAT_TEXT_RE = r"[-+]?(?:\d+(?:\.\d+)?|nan|inf)"
FLEX_TDOA_RE = re.compile(
    r"\bUWB_FLEX_TDOA obs\s+tag=(?P<tag>\d+)\s+"
    r"initiator=(?P<initiator>\d+)\s+responder=(?P<responder>\d+)\s+"
    r"seq=(?P<seq>\d+)\s+diff=(?P<diff>[-+]?\d+(?:\.\d+)?)\s+m\s+"
    r"raw=(?P<raw>[-+]?\d+(?:\.\d+)?)\s+m\s+"
    r"anchor=(?P<anchor_distance>[-+]?\d+(?:\.\d+)?)\s+m"
)
FLEX_TDOA_EXTRA_RE = re.compile(
    rf"\balt=(?P<alt>{FLOAT_TEXT_RE})\s+m\s+"
    rf"agree=(?P<agree>{FLOAT_TEXT_RE})\s+m\s+"
    rf"(?:blend=(?P<blend>{FLOAT_TEXT_RE})\s+)?"
    r"fused=(?P<fused>[01])\s+suspect=(?P<suspect>[01])"
)
FLEX_TDOA_PRIMARY_RE = re.compile(
    rf"\bprimary=(?P<primary>{FLOAT_TEXT_RE})\s+m"
)
FLEX_TDOA_ANCHOR_RE = re.compile(
    r"\bFLEX_TDOA anchor result\s+pair=(?P<initiator>\d+)-(?P<responder>\d+)\s+"
    r"seq=(?P<seq>\d+)\s+distance=(?P<distance>[-+]?\d+(?:\.\d+)?)\s+m\s+"
    r"[-+]?\d+(?:\.\d+)?\s+cm\s+raw=(?P<raw>[-+]?\d+(?:\.\d+)?)\s+m"
)
CAL_SYNC_SKIP_RE = re.compile(r"\bUWB CAL slot skipped due to sync fail\b")
TELEMETRY_BINARY_MAGIC = b"UWT1"
TELEMETRY_BINARY_HEADER_LEN = 12
TELEMETRY_STREAM_BNO085_ACCEL = 1
TELEMETRY_ACCEL_SAMPLE_LEN = 21
TELEMETRY_ACCEL_STRUCT = struct.Struct("<IIiiiB")
UWB_METERS_PER_DTU = 15.650040064102564e-12 * 299702547.0
DYNAMIC_TDOA_WINDOW_SEC = 1.5


class CalibrationCancelled(RuntimeError):
    pass


def classify_component(tag: str, message: str) -> str:
    clean_tag = tag.lower()
    clean_message = message.lower()
    if (
        "uwb" in clean_tag
        or "dw3000" in clean_tag
        or "uwb" in clean_message
        or "dw3000" in clean_message
        or "ds-twr" in clean_message
    ):
        return "uwb"
    if (
        "bno" in clean_tag
        or "accelerometer" in clean_message
        or "accel" in clean_message
    ):
        return "accelerometer"
    if "gps" in clean_tag or "gps" in clean_message:
        return "gps"
    return "system"


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
    match = re.search(r'#define\s+APP_OTA_PASSWORD\s+"([^"]*)"', path.read_text())
    if match is None:
        return ""
    return match.group(1)


def status_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/status"
    return f"http://{target}/status"


def runtime_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/config/runtime"
    return f"http://{target}/config/runtime"


def charger_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/config/charger"
    return f"http://{target}/config/charger"


def max77958_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/config/max77958"
    return f"http://{target}/config/max77958"


def antenna_delay_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/config/antenna-delay"
    return f"http://{target}/config/antenna-delay"


def cm_to_mm_text(value: Any) -> str:
    number = float(value)
    return str(int(round(number * 10.0)))


def parse_bool(value: Any, default: bool = False) -> bool:
    if value is None or value == "":
        return default
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "y"):
        return True
    if text in ("0", "false", "no", "off", "n"):
        return False
    return default


def parse_module_ids(value: Any, *, expected: int | None = None) -> list[int]:
    if value in (None, ""):
        return []
    if isinstance(value, str):
        raw_items = re.split(r"[\s,]+", value.strip())
    else:
        raw_items = [str(item) for item in value]
    ids: list[int] = []
    for raw in raw_items:
        item = str(raw).strip()
        if not item:
            continue
        module_id = int(item)
        if module_id <= 0:
            raise ValueError(f"invalid module id: {module_id}")
        if module_id not in ids:
            ids.append(module_id)
    if expected is not None and len(ids) != expected:
        raise ValueError(f"expected {expected} module id(s), got {len(ids)}")
    return ids


def parse_u16_text(value: Any) -> int:
    text = str(value).strip()
    base = 16 if text.lower().startswith("0x") else 10
    parsed = int(text, base)
    if parsed < 0 or parsed > 0xFFFF:
        raise ValueError(f"invalid u16 value: {value}")
    return parsed


def clamp_u16(value: int) -> int:
    return max(0, min(0xFFFF, int(value)))


def round_i32(value: float) -> int:
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / (len(values) - 1))


def calibration_result_labels(results: list[dict[str, Any]]) -> list[str]:
    labels: list[str] = []
    for item in sorted(results, key=lambda row: int(row.get("module_id") or 0)):
        module_id = item.get("module_id")
        correction = item.get("correction_dtu")
        if module_id is None or correction is None:
            continue
        labels.append(f"M{int(module_id)} {int(correction):+d}")
    return labels


def calibration_reason_label(reason: Any) -> str | None:
    text = str(reason or "")
    if not text:
        return None
    if text == "dry_run":
        return "dry run"
    if text.startswith("below_min_apply_dtu"):
        return "below min apply"
    if text == "reference_guard":
        return "reference guard"
    if text == "sync_miss":
        return "sync miss"
    return text.replace("_", " ")


def calibration_write_summary(results: list[dict[str, Any]]) -> str:
    applied = [item for item in results if item.get("applied")]
    skipped = [item for item in results if not item.get("applied")]
    applied_labels = calibration_result_labels(applied)
    skipped_labels = calibration_result_labels(skipped)
    parts: list[str] = []
    if applied_labels:
        parts.append("applied " + ", ".join(applied_labels))
    if skipped_labels:
        prefix = "no write" if applied_labels else "no writes; computed"
        reason_labels = sorted(
            {
                label
                for label in (
                    calibration_reason_label(item.get("reason"))
                    for item in skipped
                )
                if label
            }
        )
        reason_text = f" ({', '.join(reason_labels)})" if reason_labels else ""
        parts.append(f"{prefix} {', '.join(skipped_labels)}{reason_text}")
    return "; ".join(parts) if parts else "no writes"


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for col in range(size):
        pivot = max(range(col, size), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-9:
            raise ValueError("calibration correction matrix is singular")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        pivot_value = rows[col][col]
        rows[col] = [item / pivot_value for item in rows[col]]
        for row in range(size):
            if row == col:
                continue
            factor = rows[row][col]
            if abs(factor) < 1e-12:
                continue
            rows[row] = [
                rows[row][item] - factor * rows[col][item]
                for item in range(size + 1)
            ]
    return [rows[row][size] for row in range(size)]


def solve_least_squares(
    rows: list[list[float]], values: list[float]
) -> list[float]:
    if not rows:
        return []
    width = len(rows[0])
    normal = [[0.0 for _ in range(width)] for _ in range(width)]
    rhs = [0.0 for _ in range(width)]
    for row, value in zip(rows, values):
        if not any(abs(item) > 1e-12 for item in row):
            continue
        for i in range(width):
            rhs[i] += row[i] * value
            for j in range(width):
                normal[i][j] += row[i] * row[j]
    return solve_linear_system(normal, rhs)


def parse_port_list(text: str) -> list[int]:
    ports: list[int] = []
    for raw in re.split(r"[\s,]+", str(text or "")):
        item = raw.strip()
        if not item:
            continue
        port = int(item)
        if port < 1 or port > 65535:
            raise argparse.ArgumentTypeError(f"invalid TCP port: {port}")
        if port not in ports:
            ports.append(port)
    return ports


def binary_telemetry_frame_len(buffer: bytes) -> int | None:
    if len(buffer) < TELEMETRY_BINARY_HEADER_LEN:
        return None
    if not buffer.startswith(TELEMETRY_BINARY_MAGIC):
        return -1

    version = buffer[4]
    stream_type = buffer[5]
    sample_size = buffer[7]
    count = int.from_bytes(buffer[8:10], "little")
    payload_len = int.from_bytes(buffer[10:12], "little")
    if (
        version != 1
        or stream_type != TELEMETRY_STREAM_BNO085_ACCEL
        or sample_size != TELEMETRY_ACCEL_SAMPLE_LEN
        or payload_len != count * sample_size
        or payload_len > 4096
    ):
        return -1

    frame_len = TELEMETRY_BINARY_HEADER_LEN + payload_len
    if len(buffer) < frame_len:
        return None
    return frame_len


def parse_binary_telemetry_frame(frame: bytes) -> list[dict[str, Any]]:
    if len(frame) < TELEMETRY_BINARY_HEADER_LEN:
        return []
    if not frame.startswith(TELEMETRY_BINARY_MAGIC):
        return []
    if frame[5] != TELEMETRY_STREAM_BNO085_ACCEL:
        return []

    module_id = int(frame[6])
    sample_size = int(frame[7])
    count = int.from_bytes(frame[8:10], "little")
    payload_len = int.from_bytes(frame[10:12], "little")
    if sample_size != TELEMETRY_ACCEL_SAMPLE_LEN:
        return []
    if payload_len != count * sample_size:
        return []

    samples: list[dict[str, Any]] = []
    offset = TELEMETRY_BINARY_HEADER_LEN
    end = min(len(frame), offset + payload_len)
    while offset + TELEMETRY_ACCEL_SAMPLE_LEN <= end:
        uptime_ms, reports, x, y, z, accuracy = TELEMETRY_ACCEL_STRUCT.unpack_from(
            frame, offset
        )
        samples.append(
            {
                "module_id": module_id,
                "host": f"uwb-module-{module_id}",
                "uptime_ms": int(uptime_ms),
                "topic": "bno085.accel",
                "x": x / 1000.0,
                "y": y / 1000.0,
                "z": z / 1000.0,
                "accuracy": int(accuracy),
                "reports": int(reports),
            }
        )
        offset += TELEMETRY_ACCEL_SAMPLE_LEN
    return samples


class DashboardState:
    def __init__(self, *, max_logs: int) -> None:
        self.lock = threading.Lock()
        self.max_logs = max_logs
        self.status_online_max_age_sec = 6.0
        self.logs: deque[dict[str, Any]] = deque(maxlen=max_logs)
        self.accel_history: dict[int, deque[dict[str, Any]]] = {}
        self.max_accel_samples = 30000
        self.accel_samples: deque[dict[str, Any]] = deque(maxlen=120000)
        self.ranging_distances: dict[tuple[int, int], dict[str, Any]] = {}
        self.ranging_history: dict[tuple[int, int], deque[dict[str, Any]]] = {}
        self.max_ranging_samples = 200
        self.tdoa_observations: dict[tuple[int, int, int], dict[str, Any]] = {}
        self.tdoa_history: dict[tuple[int, int, int], deque[dict[str, Any]]] = {}
        self.tdoa_anchor_distances: dict[tuple[int, int], dict[str, Any]] = {}
        self.tdoa_anchor_history: dict[tuple[int, int], deque[dict[str, Any]]] = {}
        self.max_tdoa_samples = 200
        self.next_log_id = 1
        self.next_accel_id = 1
        self.client_count = 0
        self.telemetry_client_count = 0
        self.client_counts: dict[str, int] = {}
        self.telemetry_client_counts: dict[str, int] = {}
        self.listener_telemetry_ports: list[int] = []
        self.status_targets: list[str] = []
        self.status_by_module: dict[int, dict[str, Any]] = {}
        self.status_errors: dict[str, str] = {}

    def add_log(self, line: str, addr: tuple[str, int]) -> None:
        parsed = self.parse_line(line)
        now = time.time()
        with self.lock:
            log_id = self.next_log_id
            item = {
                "id": log_id,
                "received_at": now,
                "client": f"{addr[0]}:{addr[1]}",
                "raw": line,
                **parsed,
            }
            self.next_log_id += 1
            self.logs.append(item)
            self.record_accel_locked(item)
            self.record_ranging_locked(item)
            self.record_tdoa_anchor_locked(item)
            self.record_tdoa_locked(item)

    def add_telemetry(self, line: str, addr: tuple[str, int]) -> None:
        parsed = self.parse_telemetry(line)
        if parsed is None:
            return
        parsed["received_at"] = time.time()
        parsed["client"] = f"{addr[0]}:{addr[1]}"
        with self.lock:
            self.record_accel_sample_locked(parsed)

    def add_telemetry_samples(
        self, samples: list[dict[str, Any]], addr: tuple[str, int]
    ) -> None:
        if not samples:
            return
        now = time.time()
        client = f"{addr[0]}:{addr[1]}"
        with self.lock:
            for sample in samples:
                sample["received_at"] = now
                sample["client"] = client
                self.record_accel_sample_locked(sample)

    def parse_line(self, line: str) -> dict[str, Any]:
        match = LOG_RE.match(line)
        if match is None:
            return {
                "module_id": None,
                "host": "",
                "uptime_ms": None,
                "level": "?",
                "tag": "raw",
                "message": line,
                "component": "system",
            }

        host = match.group("host")
        tag = match.group("tag")
        message = match.group("message")
        module_match = MODULE_RE.search(host)
        module_id = int(module_match.group("id")) if module_match else None
        return {
            "module_id": module_id,
            "host": host,
            "uptime_ms": int(match.group("uptime")),
            "level": match.group("level"),
            "tag": tag,
            "message": message,
            "component": classify_component(tag, message),
        }

    def record_accel_locked(self, item: dict[str, Any]) -> None:
        module_id = item.get("module_id")
        if module_id is None:
            return
        match = ACCEL_RE.search(str(item.get("message") or ""))
        if match is None:
            return

        self.record_accel_sample_locked(
            {
                "module_id": int(module_id),
                "received_at": item.get("received_at"),
                "uptime_ms": item.get("uptime_ms"),
                "log_id": item.get("id"),
                "x": float(match.group("x")),
                "y": float(match.group("y")),
                "z": float(match.group("z")),
                "accuracy": int(match.group("accuracy")),
                "reports": int(match.group("reports")),
            }
        )

    def parse_telemetry(self, line: str) -> dict[str, Any] | None:
        clean = line.strip()
        short_match = SHORT_ACCEL_RE.match(clean)
        if short_match is not None:
            try:
                module_id = int(short_match.group("module"))
                return {
                    "module_id": module_id,
                    "host": f"uwb-module-{module_id}",
                    "uptime_ms": int(short_match.group("uptime")),
                    "topic": "bno085.accel",
                    "x": int(short_match.group("x")) / 1000.0,
                    "y": int(short_match.group("y")) / 1000.0,
                    "z": int(short_match.group("z")) / 1000.0,
                    "accuracy": int(short_match.group("accuracy")),
                    "reports": int(short_match.group("reports")),
                }
            except ValueError:
                return None

        match = TELEMETRY_RE.match(clean)
        if match is None:
            return None
        host = match.group("host")
        topic = match.group("topic")
        module_match = MODULE_RE.search(host)
        module_id = int(module_match.group("id")) if module_match else None
        if module_id is None or topic != "bno085.accel":
            return None
        parts = match.group("payload").split(",")
        if len(parts) < 5:
            return None
        try:
            return {
                "module_id": module_id,
                "host": host,
                "uptime_ms": int(match.group("uptime")),
                "topic": topic,
                "x": int(parts[0]) / 1000.0,
                "y": int(parts[1]) / 1000.0,
                "z": int(parts[2]) / 1000.0,
                "accuracy": int(parts[3]),
                "reports": int(parts[4]),
            }
        except ValueError:
            return None

    def record_accel_sample_locked(self, sample: dict[str, Any]) -> None:
        module_id = int(sample["module_id"])
        sample["sample_id"] = self.next_accel_id
        self.next_accel_id += 1
        history = self.accel_history.setdefault(
            module_id, deque(maxlen=self.max_accel_samples)
        )
        history.append(sample)
        self.accel_samples.append(sample)

    def record_ranging_locked(self, item: dict[str, Any]) -> None:
        match = RANGING_RE.search(str(item.get("message") or item.get("raw") or ""))
        if match is None:
            return

        try:
            tag_id = int(match.group("tag"))
            anchor_id = int(match.group("anchor"))
            distance_m = float(match.group("distance"))
            seq = int(match.group("seq"))
        except ValueError:
            return

        now = float(item.get("received_at") or time.time())
        key = (tag_id, anchor_id)
        sample = {
            "tag_id": tag_id,
            "anchor_id": anchor_id,
            "distance_m": distance_m,
            "seq": seq,
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "raw": item.get("raw") or item.get("message") or "",
        }
        self.ranging_distances[key] = sample
        self.ranging_history.setdefault(
            key, deque(maxlen=self.max_ranging_samples)
        ).append(sample)

    def record_tdoa_locked(self, item: dict[str, Any]) -> None:
        raw_message = str(item.get("message") or item.get("raw") or "")
        match = FLEX_TDOA_RE.search(raw_message)
        if match is None:
            return

        try:
            tag_id = int(match.group("tag"))
            initiator_id = int(match.group("initiator"))
            responder_id = int(match.group("responder"))
            seq = int(match.group("seq"))
            diff_m = float(match.group("diff"))
            raw_diff_m = float(match.group("raw"))
            anchor_distance_m = float(match.group("anchor_distance"))
        except ValueError:
            return

        extra_match = FLEX_TDOA_EXTRA_RE.search(raw_message)
        alt_diff_m = None
        primary_diff_m = None
        agreement_m = None
        blend_weight = None
        fused = False
        suspect = False
        primary_match = FLEX_TDOA_PRIMARY_RE.search(raw_message)
        if primary_match is not None:
            primary_diff_m = self.optional_float(primary_match.group("primary"))
        if extra_match is not None:
            alt_diff_m = self.optional_float(extra_match.group("alt"))
            agreement_m = self.optional_float(extra_match.group("agree"))
            blend_weight = self.optional_float(extra_match.group("blend"))
            fused = extra_match.group("fused") == "1"
            suspect = extra_match.group("suspect") == "1"

        now = float(item.get("received_at") or time.time())
        key = (tag_id, initiator_id, responder_id)
        sample = {
            "tag_id": tag_id,
            "initiator_id": initiator_id,
            "responder_id": responder_id,
            "seq": seq,
            "diff_m": diff_m,
            "raw_diff_m": raw_diff_m,
            "primary_diff_m": primary_diff_m,
            "alt_diff_m": alt_diff_m,
            "agreement_m": agreement_m,
            "blend_weight": blend_weight,
            "fused": fused,
            "suspect": suspect,
            "anchor_distance_m": anchor_distance_m,
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "raw": raw_message,
        }
        self.tdoa_observations[key] = sample
        self.tdoa_history.setdefault(
            key, deque(maxlen=self.max_tdoa_samples)
        ).append(sample)
        self.store_tdoa_anchor_distance_locked(
            initiator_id=initiator_id,
            responder_id=responder_id,
            seq=seq,
            distance_m=anchor_distance_m,
            raw_distance_m=None,
            item=item,
            source="tdoa_obs",
        )

    def record_tdoa_anchor_locked(self, item: dict[str, Any]) -> None:
        match = FLEX_TDOA_ANCHOR_RE.search(
            str(item.get("message") or item.get("raw") or "")
        )
        if match is None:
            return

        try:
            initiator_id = int(match.group("initiator"))
            responder_id = int(match.group("responder"))
            seq = int(match.group("seq"))
            distance_m = float(match.group("distance"))
            raw_distance_m = float(match.group("raw"))
        except ValueError:
            return

        self.store_tdoa_anchor_distance_locked(
            initiator_id=initiator_id,
            responder_id=responder_id,
            seq=seq,
            distance_m=distance_m,
            raw_distance_m=raw_distance_m,
            item=item,
            source="anchor_result",
        )

    def store_tdoa_anchor_distance_locked(
        self,
        *,
        initiator_id: int,
        responder_id: int,
        seq: int,
        distance_m: float,
        raw_distance_m: float | None,
        item: dict[str, Any],
        source: str,
    ) -> None:
        if initiator_id <= 0 or responder_id <= 0 or initiator_id == responder_id:
            return
        if not math.isfinite(distance_m) or distance_m <= 0:
            return

        anchor_a_id, anchor_b_id = sorted((initiator_id, responder_id))
        now = float(item.get("received_at") or time.time())
        key = (anchor_a_id, anchor_b_id)
        existing = self.tdoa_anchor_distances.get(key)
        if (
            source == "tdoa_obs"
            and existing is not None
            and existing.get("source") == "anchor_result"
            and int(existing.get("seq") or -1) == seq
        ):
            return
        sample = {
            "anchor_a_id": anchor_a_id,
            "anchor_b_id": anchor_b_id,
            "initiator_id": initiator_id,
            "responder_id": responder_id,
            "seq": seq,
            "distance_m": distance_m,
            "raw_distance_m": raw_distance_m,
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "source": source,
            "raw": item.get("raw") or item.get("message") or "",
        }
        self.tdoa_anchor_distances[key] = sample
        self.tdoa_anchor_history.setdefault(
            key, deque(maxlen=self.max_tdoa_samples)
        ).append(sample)

    @staticmethod
    def median_float(values: list[float]) -> float | None:
        clean = sorted(value for value in values if math.isfinite(value))
        if not clean:
            return None
        middle = len(clean) // 2
        if len(clean) % 2:
            return clean[middle]
        return (clean[middle - 1] + clean[middle]) / 2.0

    @staticmethod
    def optional_float(value: Any) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if math.isfinite(parsed) else None

    @staticmethod
    def sequence_delta(start: int, end: int) -> int:
        return (int(end) - int(start)) % 65536

    @classmethod
    def tdoa_sequences_are_paired(
        cls, left: dict[str, Any], right: dict[str, Any], pair_count: int
    ) -> bool:
        if pair_count <= 0:
            return False
        try:
            left_seq = int(left["seq"])
            right_seq = int(right["seq"])
        except (KeyError, TypeError, ValueError):
            return False
        forward = cls.sequence_delta(left_seq, right_seq)
        reverse = cls.sequence_delta(right_seq, left_seq)
        return 0 < min(forward, reverse) <= pair_count

    def tdoa_runtime_anchor_ids_locked(self) -> list[int]:
        for item in self.status_by_module.values():
            anchor_ids = item.get("runtime_anchor_ids")
            if isinstance(anchor_ids, list):
                ids = [
                    int(anchor_id)
                    for anchor_id in anchor_ids
                    if isinstance(anchor_id, int) and anchor_id > 0
                ]
                if len(ids) >= 3:
                    return ids
        ids: set[int] = set()
        for anchor_a_id, anchor_b_id in self.tdoa_anchor_distances:
            ids.add(int(anchor_a_id))
            ids.add(int(anchor_b_id))
        return sorted(ids)

    def tdoa_paired_observations_locked(
        self, now: float, max_age_sec: float
    ) -> dict[str, Any]:
        anchor_ids = self.tdoa_runtime_anchor_ids_locked()
        slot_count = len(anchor_ids) * (len(anchor_ids) - 1) // 2
        if slot_count <= 0:
            return {}

        tag_ids = sorted({key[0] for key in self.tdoa_history})
        paired_observations: dict[str, Any] = {}
        for tag_id in tag_ids:
            for i, initiator_id in enumerate(anchor_ids):
                for responder_id in anchor_ids[i + 1 :]:
                    left_history = [
                        sample
                        for sample in self.tdoa_history.get(
                            (tag_id, initiator_id, responder_id), []
                        )
                        if now - float(sample.get("received_at") or 0.0)
                        <= max_age_sec
                    ]
                    right_history = [
                        sample
                        for sample in self.tdoa_history.get(
                            (tag_id, responder_id, initiator_id), []
                        )
                        if now - float(sample.get("received_at") or 0.0)
                        <= max_age_sec
                    ]
                    if not left_history or not right_history:
                        continue

                    matched: list[tuple[dict[str, Any], dict[str, Any]]] = []
                    for left in left_history:
                        compatible = [
                            right
                            for right in right_history
                            if self.tdoa_sequences_are_paired(
                                left, right, slot_count
                            )
                        ]
                        if not compatible:
                            continue
                        compatible.sort(
                            key=lambda right: abs(
                                float(left.get("received_at") or 0.0)
                                - float(right.get("received_at") or 0.0)
                            )
                        )
                        matched.append((left, compatible[0]))
                    if not matched:
                        continue

                    diffs: list[float] = []
                    raw_diffs: list[float] = []
                    reverse_sums: list[float] = []
                    agreement_values: list[float] = []
                    blend_values: list[float] = []
                    fused_count = 0
                    suspect_count = 0
                    for left, right in matched:
                        left_diff = float(left["diff_m"])
                        right_diff = float(right["diff_m"])
                        diffs.append((left_diff - right_diff) / 2.0)
                        reverse_sums.append(left_diff + right_diff)
                        left_raw = float(left.get("raw_diff_m") or math.nan)
                        right_raw = float(right.get("raw_diff_m") or math.nan)
                        if math.isfinite(left_raw) and math.isfinite(right_raw):
                            raw_diffs.append((left_raw - right_raw) / 2.0)
                        for sample in (left, right):
                            if sample.get("fused"):
                                fused_count += 1
                            if sample.get("suspect"):
                                suspect_count += 1
                            agreement_m = self.optional_float(
                                sample.get("agreement_m")
                            )
                            if agreement_m is not None:
                                agreement_values.append(agreement_m)
                            blend_weight = self.optional_float(
                                sample.get("blend_weight")
                            )
                            if blend_weight is not None:
                                blend_values.append(blend_weight)

                    diff_m = self.median_float(diffs)
                    reverse_sum_m = self.median_float(reverse_sums)
                    if diff_m is None or reverse_sum_m is None:
                        continue
                    raw_diff_m = self.median_float(raw_diffs)
                    agreement_m = self.median_float(agreement_values)
                    blend_weight = self.median_float(blend_values)
                    latest_left, latest_right = max(
                        matched,
                        key=lambda pair: max(
                            float(pair[0].get("received_at") or 0.0),
                            float(pair[1].get("received_at") or 0.0),
                        ),
                    )
                    latest_left_diff = float(latest_left["diff_m"])
                    latest_right_diff = float(latest_right["diff_m"])
                    latest_diff_m = (latest_left_diff - latest_right_diff) / 2.0
                    latest_raw_diff_m = None
                    latest_left_raw = float(latest_left.get("raw_diff_m") or math.nan)
                    latest_right_raw = float(latest_right.get("raw_diff_m") or math.nan)
                    if math.isfinite(latest_left_raw) and math.isfinite(latest_right_raw):
                        latest_raw_diff_m = (
                            latest_left_raw - latest_right_raw
                        ) / 2.0
                    matched_times = [
                        float(sample.get("received_at") or 0.0)
                        for pair in matched
                        for sample in pair
                    ]
                    received_at = max(
                        float(latest_left.get("received_at") or 0.0),
                        float(latest_right.get("received_at") or 0.0),
                    )
                    dynamic_matched = [
                        pair
                        for pair in matched
                        if now
                        - max(
                            float(pair[0].get("received_at") or 0.0),
                            float(pair[1].get("received_at") or 0.0),
                        )
                        <= DYNAMIC_TDOA_WINDOW_SEC
                    ]
                    if not dynamic_matched:
                        dynamic_matched = [(latest_left, latest_right)]
                    dynamic_diffs: list[float] = []
                    dynamic_raw_diffs: list[float] = []
                    dynamic_reverse_sums: list[float] = []
                    dynamic_times = [
                        float(sample.get("received_at") or 0.0)
                        for pair in dynamic_matched
                        for sample in pair
                    ]
                    for left, right in dynamic_matched:
                        left_diff = float(left["diff_m"])
                        right_diff = float(right["diff_m"])
                        dynamic_diffs.append((left_diff - right_diff) / 2.0)
                        dynamic_reverse_sums.append(left_diff + right_diff)
                        left_raw = float(left.get("raw_diff_m") or math.nan)
                        right_raw = float(right.get("raw_diff_m") or math.nan)
                        if math.isfinite(left_raw) and math.isfinite(right_raw):
                            dynamic_raw_diffs.append((left_raw - right_raw) / 2.0)
                    dynamic_diff_m = self.median_float(dynamic_diffs)
                    dynamic_raw_diff_m = self.median_float(dynamic_raw_diffs)
                    dynamic_reverse_sum_m = self.median_float(dynamic_reverse_sums)
                    paired_observations[
                        f"{tag_id}:{initiator_id}:{responder_id}"
                    ] = {
                        "tag_id": tag_id,
                        "initiator_id": initiator_id,
                        "responder_id": responder_id,
                        "seq": int(latest_left["seq"]),
                        "reverse_seq": int(latest_right["seq"]),
                        "diff_m": diff_m,
                        "dynamic_diff_m": dynamic_diff_m
                        if dynamic_diff_m is not None
                        else latest_diff_m,
                        "latest_diff_m": latest_diff_m,
                        "raw_diff_m": raw_diff_m
                        if raw_diff_m is not None
                        else float(latest_left["raw_diff_m"]),
                        "dynamic_raw_diff_m": dynamic_raw_diff_m
                        if dynamic_raw_diff_m is not None
                        else (
                            latest_raw_diff_m
                            if latest_raw_diff_m is not None
                            else float(latest_left["raw_diff_m"])
                        ),
                        "latest_raw_diff_m": latest_raw_diff_m
                        if latest_raw_diff_m is not None
                        else float(latest_left["raw_diff_m"]),
                        "reverse_sum_m": reverse_sum_m,
                        "dynamic_reverse_sum_m": dynamic_reverse_sum_m
                        if dynamic_reverse_sum_m is not None
                        else float(latest_left["diff_m"]) + float(latest_right["diff_m"]),
                        "latest_reverse_sum_m": float(latest_left["diff_m"])
                        + float(latest_right["diff_m"]),
                        "agreement_m": agreement_m,
                        "blend_weight": blend_weight,
                        "fused_count": fused_count,
                        "suspect_count": suspect_count,
                        "suspect": suspect_count > len(matched),
                        "anchor_distance_m": float(
                            latest_left["anchor_distance_m"]
                        ),
                        "age_sec": now - received_at,
                        "history_span_sec": (
                            max(matched_times) - min(matched_times)
                            if matched_times
                            else 0.0
                        ),
                        "dynamic_span_sec": (
                            max(dynamic_times) - min(dynamic_times)
                            if dynamic_times
                            else 0.0
                        ),
                        "dynamic_samples": len(dynamic_matched),
                        "samples": len(matched),
                        "source_module_id": latest_left.get("source_module_id"),
                        "log_id": latest_left.get("log_id"),
                    }
        return paired_observations

    def ranging_snapshot_locked(self, now: float) -> dict[str, Any]:
        distances: dict[str, Any] = {}
        for (tag_id, anchor_id), item in sorted(self.ranging_distances.items()):
            history = list(self.ranging_history.get((tag_id, anchor_id), []))
            values = [
                float(sample["distance_m"])
                for sample in history
                if now - float(sample.get("received_at") or 0.0) <= 10.0
            ]
            mean_m = sum(values) / len(values) if values else None
            median_m = self.median_float(values)
            std_m = None
            if len(values) >= 2 and mean_m is not None:
                variance = sum((value - mean_m) ** 2 for value in values) / (
                    len(values) - 1
                )
                std_m = math.sqrt(max(0.0, variance))
            distances[f"{tag_id}:{anchor_id}"] = {
                "tag_id": tag_id,
                "anchor_id": anchor_id,
                "distance_m": float(item["distance_m"]),
                "seq": int(item["seq"]),
                "age_sec": now - float(item["received_at"]),
                "log_id": item.get("log_id"),
                "source_module_id": item.get("source_module_id"),
                "raw": item.get("raw") or "",
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "std_m": std_m,
                    "min_m": min(values) if values else None,
                    "max_m": max(values) if values else None,
                },
            }
        return {
            "distances": distances,
            "max_age_sec": 3.0,
        }

    def tdoa_snapshot_locked(self, now: float) -> dict[str, Any]:
        observations: dict[str, Any] = {}
        anchor_distances: dict[str, Any] = {}
        max_age_sec = 3.0
        for (tag_id, initiator_id, responder_id), item in sorted(
            self.tdoa_observations.items()
        ):
            history = list(
                self.tdoa_history.get((tag_id, initiator_id, responder_id), [])
            )
            values = [
                float(sample["diff_m"])
                for sample in history
                if now - float(sample.get("received_at") or 0.0) <= 10.0
            ]
            mean_m = sum(values) / len(values) if values else None
            median_m = self.median_float(values)
            std_m = None
            if len(values) >= 2 and mean_m is not None:
                variance = sum((value - mean_m) ** 2 for value in values) / (
                    len(values) - 1
                )
                std_m = math.sqrt(max(0.0, variance))
            observations[f"{tag_id}:{initiator_id}:{responder_id}"] = {
                "tag_id": tag_id,
                "initiator_id": initiator_id,
                "responder_id": responder_id,
                "seq": int(item["seq"]),
                "diff_m": float(item["diff_m"]),
                "raw_diff_m": float(item["raw_diff_m"]),
                "primary_diff_m": item.get("primary_diff_m"),
                "alt_diff_m": item.get("alt_diff_m"),
                "agreement_m": item.get("agreement_m"),
                "blend_weight": item.get("blend_weight"),
                "fused": bool(item.get("fused")),
                "suspect": bool(item.get("suspect")),
                "anchor_distance_m": float(item["anchor_distance_m"]),
                "age_sec": now - float(item["received_at"]),
                "log_id": item.get("log_id"),
                "source_module_id": item.get("source_module_id"),
                "raw": item.get("raw") or "",
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "std_m": std_m,
                    "min_m": min(values) if values else None,
                    "max_m": max(values) if values else None,
                },
            }
        for (anchor_a_id, anchor_b_id), item in sorted(
            self.tdoa_anchor_distances.items()
        ):
            history = list(
                self.tdoa_anchor_history.get((anchor_a_id, anchor_b_id), [])
            )
            values = [
                float(sample["distance_m"])
                for sample in history
                if now - float(sample.get("received_at") or 0.0) <= 10.0
            ]
            mean_m = sum(values) / len(values) if values else None
            median_m = self.median_float(values)
            std_m = None
            if len(values) >= 2 and mean_m is not None:
                variance = sum((value - mean_m) ** 2 for value in values) / (
                    len(values) - 1
                )
                std_m = math.sqrt(max(0.0, variance))
            anchor_distances[f"{anchor_a_id}:{anchor_b_id}"] = {
                "anchor_a_id": anchor_a_id,
                "anchor_b_id": anchor_b_id,
                "initiator_id": int(item["initiator_id"]),
                "responder_id": int(item["responder_id"]),
                "seq": int(item["seq"]),
                "distance_m": float(item["distance_m"]),
                "raw_distance_m": item.get("raw_distance_m"),
                "age_sec": now - float(item["received_at"]),
                "log_id": item.get("log_id"),
                "source_module_id": item.get("source_module_id"),
                "source": item.get("source") or "",
                "raw": item.get("raw") or "",
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "std_m": std_m,
                    "min_m": min(values) if values else None,
                    "max_m": max(values) if values else None,
                },
            }
        return {
            "observations": observations,
            "anchor_distances": anchor_distances,
            "paired_observations": self.tdoa_paired_observations_locked(
                now, max_age_sec
            ),
            "max_age_sec": max_age_sec,
        }


    def set_client_count(self, count: int, source: str = "default") -> None:
        with self.lock:
            self.client_counts[source] = max(0, count)
            self.client_count = sum(self.client_counts.values())

    def set_telemetry_client_count(
        self, count: int, source: str = "default"
    ) -> None:
        with self.lock:
            self.telemetry_client_counts[source] = max(0, count)
            self.telemetry_client_count = sum(
                self.telemetry_client_counts.values()
            )

    def set_listener_telemetry_ports(self, ports: list[int]) -> None:
        with self.lock:
            self.listener_telemetry_ports = list(dict.fromkeys(ports))

    def set_status_targets(self, targets: list[str]) -> None:
        with self.lock:
            self.status_targets = list(dict.fromkeys(targets))

    def set_status(self, module_id: int, status: dict[str, Any]) -> None:
        status["status_updated_at"] = time.time()
        with self.lock:
            self.status_by_module[module_id] = status
            target = status.get("target")
            if isinstance(target, str):
                self.status_errors.pop(target, None)

    def set_status_error(self, target: str, message: str) -> None:
        with self.lock:
            self.status_errors[target] = message

    def logs_after(self, after_id: int, limit: int) -> dict[str, Any]:
        with self.lock:
            items = [item for item in self.logs if int(item["id"]) > after_id]
            if len(items) > limit:
                items = items[-limit:]
            next_id = self.next_log_id
        return {"logs": items, "next_id": next_id, "client_count": self.client_count}

    def accel_after(self, after_id: int, limit: int) -> dict[str, Any]:
        with self.lock:
            samples = [
                sample
                for sample in self.accel_samples
                if int(sample.get("sample_id") or 0) > after_id
            ]
            if len(samples) > limit:
                samples = samples[-limit:]
            next_id = self.next_accel_id
            telemetry_client_count = self.telemetry_client_count
        return {
            "samples": samples,
            "next_id": next_id,
            "telemetry_client_count": telemetry_client_count,
        }

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            now = time.time()
            statuses = []
            seen_targets: set[str] = set()
            for status in self.status_by_module.values():
                item = dict(status)
                updated_at = float(item.get("status_updated_at") or 0.0)
                age_sec = now - updated_at if updated_at > 0.0 else None
                target = str(item.get("target") or "")
                if target:
                    seen_targets.add(target)
                error = self.status_errors.get(target)
                online = (
                    age_sec is not None
                    and age_sec <= self.status_online_max_age_sec
                    and bool(item.get("wifi_connected"))
                    and not error
                )
                item["http_status_age_sec"] = age_sec
                item["http_status_online"] = online
                if error:
                    item["http_status_error"] = error
                statuses.append(item)
            for target in self.status_targets:
                if target in seen_targets:
                    continue
                item = {
                    "module_id": None,
                    "hostname": target,
                    "target": target,
                    "ip": target,
                    "wifi_connected": False,
                    "wifi_connected_rssi": "-",
                    "wifi_disconnect_count": "-",
                    "runtime_anchor_ids": [],
                    "status_updated_at": None,
                    "http_status_age_sec": None,
                    "http_status_online": False,
                    "http_status_error": self.status_errors.get(target, "no status yet"),
                }
                statuses.append(item)
            errors = dict(self.status_errors)
            client_count = self.client_count
            telemetry_client_count = self.telemetry_client_count
            client_counts = dict(self.client_counts)
            telemetry_client_counts = dict(self.telemetry_client_counts)
            listener_telemetry_ports = list(self.listener_telemetry_ports)
            status_target_count = len(self.status_targets)
            log_count = len(self.logs)
            next_log_id = self.next_log_id
            ranging = self.ranging_snapshot_locked(now)
            tdoa = self.tdoa_snapshot_locked(now)
        statuses.sort(
            key=lambda item: (
                int(item.get("module_id") or 9999),
                str(item.get("target") or item.get("ip") or ""),
            )
        )
        return {
            "client_count": client_count,
            "telemetry_client_count": telemetry_client_count,
            "client_counts": client_counts,
            "telemetry_client_counts": telemetry_client_counts,
            "telemetry_ports": listener_telemetry_ports,
            "log_count": log_count,
            "next_log_id": next_log_id,
            "statuses": statuses,
            "status_errors": errors,
            "status_target_count": status_target_count,
            "status_online_max_age_sec": self.status_online_max_age_sec,
            "accel_history": {},
            "ranging": ranging,
            "tdoa": tdoa,
        }


class LogServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], state: DashboardState):
        super().__init__(server_address, LogHandler)
        self.state = state
        self.log_source = f"log:{server_address[1]}"
        self.telemetry_source = f"telemetry:{server_address[1]}"
        self._client_lock = threading.Lock()
        self._log_clients = 0
        self._telemetry_clients = 0

    def log_client_connected(self) -> None:
        with self._client_lock:
            self._log_clients += 1
            self.state.set_client_count(self._log_clients, self.log_source)

    def log_client_disconnected(self) -> None:
        with self._client_lock:
            self._log_clients = max(0, self._log_clients - 1)
            self.state.set_client_count(self._log_clients, self.log_source)

    def telemetry_client_connected(self) -> None:
        with self._client_lock:
            self._telemetry_clients += 1
            self.state.set_telemetry_client_count(
                self._telemetry_clients, self.telemetry_source
            )

    def telemetry_client_disconnected(self) -> None:
        with self._client_lock:
            self._telemetry_clients = max(0, self._telemetry_clients - 1)
            self.state.set_telemetry_client_count(
                self._telemetry_clients, self.telemetry_source
            )


class TelemetryServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        state: DashboardState,
        source: str | None = None,
    ):
        super().__init__(server_address, TelemetryHandler)
        self.state = state
        self.source = source or f"telemetry:{server_address[1]}"
        self._client_lock = threading.Lock()
        self._clients = 0

    def client_connected(self) -> None:
        with self._client_lock:
            self._clients += 1
            self.state.set_telemetry_client_count(self._clients, self.source)

    def client_disconnected(self) -> None:
        with self._client_lock:
            self._clients = max(0, self._clients - 1)
            self.state.set_telemetry_client_count(self._clients, self.source)


class LogHandler(socketserver.BaseRequestHandler):
    server: LogServer

    def handle(self) -> None:
        self.request.settimeout(30.0)
        buffer = b""
        addr = self.client_address
        client_kind: str | None = None

        def handle_line(text: str) -> None:
            nonlocal client_kind
            clean = text.strip()
            if (
                TELEMETRY_RE.match(clean) is not None
                or SHORT_ACCEL_RE.match(clean) is not None
            ):
                if client_kind is None:
                    client_kind = "telemetry"
                    self.server.telemetry_client_connected()
                self.server.state.add_telemetry(text, addr)
                return

            if client_kind is None:
                client_kind = "log"
                self.server.log_client_connected()
            self.server.state.add_log(text, addr)

        def handle_binary(frame: bytes) -> None:
            nonlocal client_kind
            if client_kind is None:
                client_kind = "telemetry"
                self.server.telemetry_client_connected()
            self.server.state.add_telemetry_samples(
                parse_binary_telemetry_frame(frame), addr
            )

        try:
            while True:
                try:
                    data = self.request.recv(8192)
                except socket.timeout:
                    break
                if not data:
                    break
                buffer += data
                while True:
                    if buffer.startswith(TELEMETRY_BINARY_MAGIC):
                        frame_len = binary_telemetry_frame_len(buffer)
                        if frame_len is None:
                            break
                        if frame_len < 0:
                            buffer = buffer[1:]
                            continue
                        frame = buffer[:frame_len]
                        buffer = buffer[frame_len:]
                        handle_binary(frame)
                        continue

                    newline = buffer.find(b"\n")
                    magic = buffer.find(TELEMETRY_BINARY_MAGIC)
                    if newline < 0 and magic < 0:
                        break
                    if magic > 0 and (newline < 0 or magic < newline):
                        raw = buffer[:magic]
                        buffer = buffer[magic:]
                        for chunk in raw.splitlines():
                            text = chunk.decode(
                                "utf-8", errors="replace"
                            ).strip("\r")
                            if text:
                                handle_line(text)
                        continue
                    if newline < 0:
                        break
                    raw = buffer[:newline]
                    buffer = buffer[newline + 1 :]
                    text = raw.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        handle_line(text)
        finally:
            if buffer.startswith(TELEMETRY_BINARY_MAGIC):
                frame_len = binary_telemetry_frame_len(buffer)
                if frame_len is not None and frame_len > 0:
                    handle_binary(buffer[:frame_len])
                    buffer = buffer[frame_len:]
            if buffer.strip():
                text = buffer.decode("utf-8", errors="replace").strip()
                if text:
                    handle_line(text)
            if client_kind == "telemetry":
                self.server.telemetry_client_disconnected()
            elif client_kind == "log":
                self.server.log_client_disconnected()


class TelemetryHandler(socketserver.BaseRequestHandler):
    server: TelemetryServer

    def handle(self) -> None:
        self.server.client_connected()
        self.request.settimeout(30.0)
        buffer = b""
        addr = self.client_address

        def handle_binary(frame: bytes) -> None:
            self.server.state.add_telemetry_samples(
                parse_binary_telemetry_frame(frame), addr
            )

        try:
            while True:
                try:
                    data = self.request.recv(8192)
                except socket.timeout:
                    break
                if not data:
                    break
                buffer += data
                while True:
                    if buffer.startswith(TELEMETRY_BINARY_MAGIC):
                        frame_len = binary_telemetry_frame_len(buffer)
                        if frame_len is None:
                            break
                        if frame_len < 0:
                            buffer = buffer[1:]
                            continue
                        frame = buffer[:frame_len]
                        buffer = buffer[frame_len:]
                        handle_binary(frame)
                        continue

                    newline = buffer.find(b"\n")
                    magic = buffer.find(TELEMETRY_BINARY_MAGIC)
                    if newline < 0 and magic < 0:
                        break
                    if magic > 0 and (newline < 0 or magic < newline):
                        raw = buffer[:magic]
                        buffer = buffer[magic:]
                        for chunk in raw.splitlines():
                            text = chunk.decode(
                                "utf-8", errors="replace"
                            ).strip("\r")
                            if text:
                                self.server.state.add_telemetry(text, addr)
                        continue
                    if newline < 0:
                        break
                    raw = buffer[:newline]
                    buffer = buffer[newline + 1 :]
                    text = raw.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        self.server.state.add_telemetry(text, addr)
        finally:
            if buffer.startswith(TELEMETRY_BINARY_MAGIC):
                frame_len = binary_telemetry_frame_len(buffer)
                if frame_len is not None and frame_len > 0:
                    handle_binary(buffer[:frame_len])
                    buffer = buffer[frame_len:]
            if buffer.strip():
                text = buffer.decode("utf-8", errors="replace").strip()
                if text:
                    self.server.state.add_telemetry(text, addr)
            self.server.client_disconnected()


class StatusPoller(threading.Thread):
    def __init__(self, state: DashboardState, targets: list[str], interval: float):
        super().__init__(daemon=True)
        self.state = state
        self.targets = targets
        self.interval = interval
        self.stop_event = threading.Event()

    def run(self) -> None:
        while not self.stop_event.is_set():
            for target in self.targets:
                self.poll_target(target)
            self.stop_event.wait(self.interval)

    def poll_target(self, target: str) -> None:
        try:
            with urllib.request.urlopen(status_url(target), timeout=2.0) as response:
                payload = json.loads(response.read().decode("utf-8", errors="replace"))
            module_id = int(payload.get("module_id") or 0)
            if module_id > 0:
                payload["target"] = target
                self.state.set_status(module_id, payload)
        except (OSError, TimeoutError, ValueError, urllib.error.URLError) as exc:
            self.state.set_status_error(target, str(exc))


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>UWB Dashboard</title>
<style>
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  --bg: #f7f8fb;
  --panel: #ffffff;
  --ink: #17202a;
  --muted: #667085;
  --line: #d9dee8;
  --soft: #eef1f5;
  --blue: #2b64d8;
  --green: #16833a;
  --red: #c82922;
  --orange: #b35b00;
  --purple: #7a4cc2;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); }
.app { min-height: 100vh; display: grid; grid-template-rows: auto auto 1fr; }
header {
  padding: 14px 18px 10px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 14px;
  border-bottom: 1px solid var(--line);
  background: var(--panel);
}
h1 { margin: 0; font-size: 18px; letter-spacing: 0; }
.pills { display: flex; flex-wrap: wrap; gap: 8px; justify-content: flex-end; }
.pill {
  border: 1px solid var(--line);
  border-radius: 999px;
  padding: 5px 9px;
  color: var(--muted);
  font-size: 12px;
  background: #fff;
}
.pill.good { color: var(--green); }
.pill.warn { color: var(--orange); }
.tabs {
  display: flex;
  gap: 2px;
  padding: 0 18px;
  border-bottom: 1px solid var(--line);
  background: var(--panel);
  overflow-x: auto;
}
.tab {
  border: 0;
  border-bottom: 3px solid transparent;
  background: transparent;
  padding: 11px 13px 9px;
  font: inherit;
  font-size: 13px;
  color: var(--muted);
  cursor: pointer;
  white-space: nowrap;
}
.tab.active { color: var(--ink); border-bottom-color: var(--blue); font-weight: 700; }
main { padding: 14px 18px 18px; min-height: 0; }
.page { display: none; height: calc(100vh - 116px); min-height: 520px; }
.page.active { display: block; }
#graphs { overflow: auto; }
.terminal-grid { height: 100%; min-height: 0; display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
.terminal-grid.single { grid-template-columns: minmax(360px, 1fr); max-width: 920px; }
.terminal-grid.all { grid-template-columns: 1fr; }
.terminal {
  min-width: 0;
  min-height: 0;
  height: 100%;
  background: var(--panel);
  border: 1px solid var(--line);
  display: grid;
  grid-template-rows: auto 1fr;
}
.term-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 10px;
  padding: 10px;
  border-bottom: 1px solid var(--line);
}
.term-title { font-weight: 700; font-size: 14px; }
.term-title-wrap { min-width: 120px; }
.term-meta { margin-top: 2px; color: var(--muted); font-size: 12px; }
.filters { display: flex; gap: 7px; flex-wrap: wrap; justify-content: flex-end; }
select, input, button {
  font: inherit;
  font-size: 13px;
}
select, input {
  border: 1px solid var(--line);
  background: #fff;
  color: var(--ink);
  padding: 6px 8px;
  min-height: 31px;
}
select.mixed-value, input.mixed-value {
  border-color: rgba(179, 91, 0, 0.55);
  background: #fffaf2;
}
input[type="checkbox"] {
  min-height: 0;
  width: 17px;
  height: 17px;
  padding: 0;
}
.checkbox-row {
  display: flex;
  align-items: center;
  gap: 8px;
  min-height: 31px;
}
button {
  border: 1px solid var(--line);
  background: #fff;
  color: var(--ink);
  min-height: 31px;
  padding: 6px 10px;
  cursor: pointer;
}
button:disabled {
  cursor: default;
  opacity: 0.65;
}
button.primary { background: var(--blue); border-color: var(--blue); color: #fff; }
button.danger { background: var(--red); border-color: var(--red); color: #fff; }
button.danger:hover:not(:disabled) { background: #a61b15; border-color: #a61b15; }
button.danger:disabled { background: #f8d7da; border-color: #efb5bc; color: #9f1d1d; }
.term-body {
  min-height: 0;
  overflow: auto;
  padding: 9px 10px 14px;
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 12px;
  line-height: 1.42;
  background: #fbfcfe;
}
.log-line {
  white-space: pre-wrap;
  word-break: break-word;
  border-bottom: 1px solid rgba(217, 222, 232, 0.42);
  padding: 2px 0;
}
.level-I { color: #1f5fbf; }
.level-W { color: #a35a00; font-weight: 700; }
.level-E { color: #b42318; font-weight: 700; }
.level-D { color: #4b5563; }
.level-unknown { color: #4b5563; }
.module-chip { color: var(--green); font-weight: 700; }
.tag-chip { color: var(--purple); }
.muted { color: var(--muted); }
.table-wrap, .settings {
  background: var(--panel);
  border: 1px solid var(--line);
  padding: 12px;
  height: 100%;
  overflow: auto;
}
table { width: 100%; border-collapse: collapse; font-size: 13px; }
th, td { padding: 8px 6px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: top; }
th { color: var(--muted); font-weight: 700; }
.charger-status-table { table-layout: fixed; }
.charger-status-table th,
.charger-status-table td { overflow-wrap: anywhere; }
.charger-status-table .col-module { width: 13%; }
.charger-status-table .col-power { width: 13%; }
.charger-status-table .col-adc { width: 10%; }
.charger-status-table .col-limits { width: 31%; }
.charger-status-table .col-measurements { width: 22%; }
.charger-status-table .col-write { width: 11%; }
.charger-power-cell { font-size: 12px; line-height: 1.35; }
.pd-status-table { table-layout: fixed; }
.pd-status-table th,
.pd-status-table td { overflow-wrap: anywhere; }
.pd-status-table .col-module { width: 13%; }
.pd-status-table .col-link { width: 18%; }
.pd-status-table .col-contract { width: 21%; }
.pd-status-table .col-pdos { width: 27%; }
.pd-status-table .col-ops { width: 21%; }
.pd-pdo-list { margin: 4px 0 0; padding-left: 18px; }
.pd-pdo-list li { margin: 2px 0; }
.resource-cell {
  min-width: 190px;
  max-width: 260px;
  font-size: 12px;
  line-height: 1.3;
}
.resource-meter {
  margin: 0 0 7px;
}
.resource-meter-head {
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 8px;
  margin-bottom: 3px;
}
.resource-meter-name {
  font-weight: 700;
  color: var(--ink);
}
.resource-meter-value {
  color: var(--muted);
  white-space: nowrap;
}
.resource-bar {
  height: 8px;
  border: 1px solid #d7deea;
  background: #edf2f8;
  overflow: hidden;
}
.resource-bar-fill {
  height: 100%;
  background: linear-gradient(90deg, #2f6fe4, #5c93f0);
}
.resource-meter.internal .resource-bar-fill {
  background: linear-gradient(90deg, #138a4b, #29b66f);
}
.resource-meter.psram .resource-bar-fill {
  background: linear-gradient(90deg, #2f6fe4, #6aa0f5);
}
.resource-meter.flash .resource-bar-fill {
  background: linear-gradient(90deg, #0f766e, #2bb3a3);
}
.resource-meter.cpu0 .resource-bar-fill {
  background: linear-gradient(90deg, #7a56d9, #9d7cf0);
}
.resource-meter.cpu1 .resource-bar-fill {
  background: linear-gradient(90deg, #b35b00, #df8a28);
}
.resource-meter.temp .resource-bar-fill {
  background: linear-gradient(90deg, #138a4b, #d08a00 62%, #c43a31);
}
.resource-meter.task {
  margin-bottom: 5px;
}
.resource-meter.task .resource-bar {
  height: 6px;
}
.resource-meter.task .resource-bar-fill {
  background: linear-gradient(90deg, #516070, #8996a8);
}
.resource-meter-foot {
  margin-top: 2px;
  color: var(--muted);
}
.resource-task-list {
  margin-top: 7px;
  padding-top: 6px;
  border-top: 1px solid var(--line);
}
.resource-task-title {
  color: var(--muted);
  font-weight: 700;
  margin-bottom: 5px;
}
.resource-task-name {
  max-width: 136px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.resource-meta {
  margin-top: 5px;
  padding-top: 5px;
  border-top: 1px solid var(--line);
  color: var(--muted);
}
.ok { color: var(--green); font-weight: 700; }
.warn { color: var(--orange); font-weight: 700; }
.bad { color: var(--red); font-weight: 700; }
tr.status-stale { background: #fffaf2; }
tr.status-stale td { color: #4f3b1d; }
.settings-grid { display: grid; grid-template-columns: minmax(340px, 520px) minmax(420px, 1fr); gap: 14px; align-items: start; }
.charger-grid { grid-template-columns: minmax(620px, 1.25fr) minmax(360px, 0.75fr); }
.pd-grid { grid-template-columns: minmax(680px, 1.2fr) minmax(380px, 0.8fr); }
.section { border: 1px solid var(--line); padding: 12px; margin-bottom: 12px; background: #fff; }
.section h2 { margin: 0 0 11px; font-size: 15px; }
.hidden { display: none !important; }
.form-grid { display: grid; grid-template-columns: 160px minmax(160px, 1fr); gap: 8px 10px; align-items: center; }
.form-grid.compact { grid-template-columns: 140px minmax(92px, 1fr); gap: 7px 8px; }
.form-grid label { color: var(--muted); font-size: 13px; }
.form-actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
.profile-note { max-width: 960px; line-height: 1.45; }
.profile-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 12px;
  margin-top: 12px;
}
.profile-card {
  border: 1px solid var(--line);
  background: #fbfcfe;
  padding: 12px;
}
.profile-card h3 {
  margin: 0 0 6px;
  font-size: 14px;
}
.profile-card p {
  margin: 0 0 10px;
  min-height: 34px;
  line-height: 1.35;
}
.profile-summary {
  color: var(--muted);
  font-size: 12px;
  line-height: 1.4;
  margin-top: 10px;
  padding-top: 9px;
  border-top: 1px solid var(--line);
}
.field-note {
  min-height: 31px;
  display: flex;
  align-items: center;
  color: var(--muted);
  font-size: 12px;
  line-height: 1.3;
}
.field-note.warn { color: var(--orange); font-weight: 700; }
.param-legend {
  margin-top: 12px;
  padding-top: 10px;
  border-top: 1px solid var(--line);
  display: grid;
  gap: 7px;
}
.param-legend div {
  display: grid;
  grid-template-columns: 118px minmax(0, 1fr);
  gap: 9px;
  align-items: baseline;
}
.param-legend b { font-size: 12px; }
.param-legend span { color: var(--muted); font-size: 12px; line-height: 1.3; }
.reg-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(142px, 1fr));
  gap: 7px;
  margin-top: 10px;
}
.reg-cell {
  border: 1px solid var(--line);
  background: #fbfcfe;
  padding: 7px;
  min-height: 62px;
}
.reg-cell b {
  display: block;
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 13px;
}
.reg-cell span {
  display: block;
  margin-top: 3px;
  color: var(--muted);
  font-size: 11px;
  line-height: 1.25;
}
.diagram {
  min-height: 340px;
  border: 1px solid var(--line);
  background: #fbfcfe;
  display: grid;
  place-items: center;
  padding: 12px;
}
.diagram svg { width: 100%; max-width: 520px; height: 320px; }
.node { fill: #fff; stroke: var(--green); stroke-width: 3; }
.edge { stroke: #98a2b3; stroke-width: 2; }
.edge-label { fill: var(--ink); font-size: 14px; font-weight: 700; }
.cal-results {
  display: grid;
  gap: 12px;
  font-size: 12px;
}
.cal-result-block h3 {
  margin: 0 0 6px;
  font-size: 13px;
}
.cal-result-table {
  font-size: 12px;
  table-layout: fixed;
}
.cal-result-table th,
.cal-result-table td {
  padding: 6px 5px;
  overflow-wrap: anywhere;
}
.cal-result-table th {
  white-space: nowrap;
}
.toast {
  margin-top: 10px;
  color: var(--muted);
  font-size: 13px;
  white-space: normal;
  overflow-wrap: anywhere;
  line-height: 1.35;
  max-width: 100%;
}
.toast:empty { display: none; }
.toast:not(:empty) {
  display: block;
  border: 1px solid var(--line);
  background: #fbfcfe;
  padding: 6px 8px;
}
.toast.ok { color: var(--green); border-color: rgba(22, 131, 58, 0.28); }
.toast.bad { color: var(--red); border-color: rgba(200, 41, 34, 0.28); }
.graphs-layout {
  min-height: 940px;
  display: grid;
  grid-template-columns: minmax(520px, 1fr) 280px;
  gap: 12px;
}
.chart-stack {
  display: grid;
  grid-template-rows: repeat(5, minmax(178px, 1fr));
  gap: 10px;
}
.accel-chart, .latest-panel {
  min-width: 0;
  background: var(--panel);
  border: 1px solid var(--line);
}
.accel-chart {
  display: grid;
  grid-template-columns: 92px 1fr;
  min-height: 178px;
}
.chart-label {
  border-right: 1px solid var(--line);
  padding: 10px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 4px;
}
.chart-label b { font-size: 14px; }
.chart-label span { color: var(--muted); font-size: 12px; }
.chart-wrap { min-width: 0; padding: 10px 12px; }
.accel-canvas { display: block; width: 100%; height: 100%; min-height: 156px; }
.chart-grid { stroke: #dfe5ee; stroke-width: 1; }
.chart-axis { stroke: #98a2b3; stroke-width: 1.2; }
.series-x { fill: none; stroke: #2b64d8; stroke-width: 1.25; }
.series-y { fill: none; stroke: #16833a; stroke-width: 1.25; }
.series-z { fill: none; stroke: #b35b00; stroke-width: 1.25; }
.chart-empty { fill: var(--muted); font-size: 13px; }
.chart-scale-label { fill: var(--muted); font-size: 10px; }
.latest-panel {
  max-height: calc(100vh - 116px);
  position: sticky;
  top: 14px;
  overflow: auto;
  padding: 12px;
}
.latest-panel h2 {
  margin: 0 0 10px;
  font-size: 15px;
}
.legend {
  display: flex;
  gap: 12px;
  color: var(--muted);
  font-size: 12px;
  margin-bottom: 10px;
}
.legend span::before {
  content: "";
  display: inline-block;
  width: 18px;
  height: 2px;
  margin-right: 5px;
  vertical-align: middle;
  background: currentColor;
}
.legend .lx { color: #2b64d8; }
.legend .ly { color: #16833a; }
.legend .lz { color: #b35b00; }
.graph-controls {
  border-top: 1px solid var(--line);
  border-bottom: 1px solid var(--line);
  padding: 10px 0;
  margin-bottom: 10px;
}
.graph-controls h3 {
  margin: 0 0 8px;
  font-size: 14px;
}
.graph-control-grid {
  display: grid;
  grid-template-columns: 96px 1fr;
  gap: 7px;
  align-items: center;
}
.graph-control-grid label {
  color: var(--muted);
  font-size: 12px;
}
.graph-control-actions {
  margin-top: 9px;
  display: flex;
  gap: 7px;
}
.latest-row {
  border-top: 1px solid var(--line);
  padding: 10px 0;
}
.latest-row:first-of-type { border-top: 0; }
.latest-row h3 {
  margin: 0 0 7px;
  font-size: 14px;
}
.axis-values {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 6px;
}
.axis-values div {
  border: 1px solid var(--line);
  padding: 6px;
  background: #fbfcfe;
}
.axis-values span {
  display: block;
  color: var(--muted);
  font-size: 11px;
}
.axis-values b {
  display: block;
  margin-top: 2px;
  font-size: 13px;
}
.latest-extra {
  margin-top: 7px;
  color: var(--muted);
  font-size: 12px;
  line-height: 1.4;
}
.position-layout {
  height: 100%;
  display: grid;
  grid-template-columns: minmax(520px, 1fr) 340px;
  gap: 12px;
}
.position-stage {
  min-width: 0;
  min-height: 0;
  border: 1px solid var(--line);
  background: #eef1f5;
  position: relative;
}
.position-canvas {
  display: block;
  width: 100%;
  height: 100%;
}
.position-overlay {
  position: absolute;
  inset: 0;
  display: none;
  place-items: center;
  padding: 24px;
  background: rgba(247, 248, 251, 0.88);
  text-align: center;
}
.position-overlay.active { display: grid; }
.position-overlay-panel {
  max-width: 560px;
  border: 1px solid var(--line);
  background: #fff;
  padding: 18px;
}
.position-overlay-panel h2 {
  margin: 0 0 8px;
  font-size: 20px;
}
.position-overlay-panel p {
  margin: 0 0 14px;
  color: var(--muted);
  line-height: 1.45;
}
.position-panel {
  min-width: 0;
  overflow: auto;
  border: 1px solid var(--line);
  background: var(--panel);
  padding: 12px;
}
.position-panel h2 {
  margin: 0 0 10px;
  font-size: 15px;
}
.position-readout {
  display: grid;
  gap: 8px;
  margin-bottom: 12px;
}
.position-tag-card {
  border: 1px solid var(--line);
  background: #fbfcfe;
  padding: 8px;
}
.position-tag-card b {
  display: block;
  font-size: 14px;
  margin-bottom: 3px;
}
.position-tag-card span {
  color: var(--muted);
  font-size: 12px;
}
.position-filter-card {
  border: 1px solid var(--line);
  background: #fff;
  padding: 8px;
}
.position-filter-card b {
  display: block;
  font-size: 13px;
  margin-bottom: 6px;
}
.position-filter-row {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
}
.position-pill {
  border: 1px solid var(--line);
  background: #f8fafc;
  color: var(--text);
  padding: 3px 7px;
  font-size: 12px;
}
.position-pill.good {
  border-color: #bfe8cc;
  color: #087a2a;
  background: #f4fbf6;
}
.position-pill.warn {
  border-color: #f3d29c;
  color: #a15b00;
  background: #fff9ed;
}
.position-pill.bad {
  border-color: #f2b8b5;
  color: #b3261e;
  background: #fff7f6;
}
.position-legend {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  color: var(--muted);
  font-size: 12px;
  margin: 8px 0 12px;
}
.position-legend span::before {
  content: "";
  display: inline-block;
  width: 18px;
  height: 2px;
  margin-right: 5px;
  vertical-align: middle;
  background: currentColor;
}
.position-legend .ring::before {
  width: 10px;
  height: 10px;
  border: 2px solid currentColor;
  border-radius: 999px;
  background: transparent;
}
.position-skip td {
  color: #98a2b3;
  background: #fbfcfe;
}
@media (max-width: 940px) {
  .terminal-grid, .settings-grid, .charger-grid, .pd-grid, .graphs-layout, .position-layout { grid-template-columns: 1fr; }
  .profile-grid { grid-template-columns: 1fr; }
  .page { height: auto; }
  .terminal { height: 520px; }
  .chart-stack { grid-template-rows: none; }
  .accel-chart { height: 178px; }
  .latest-panel { max-height: none; position: static; }
  .position-stage { height: 560px; }
}
</style>
</head>
<body>
<div class="app">
  <header>
    <h1>UWB Module Dashboard</h1>
    <div class="pills">
      <span id="clientPill" class="pill">0 log clients</span>
      <span id="telemetryPill" class="pill">0 telemetry clients</span>
      <span id="statusPill" class="pill warn">0 modules online</span>
      <span id="logPill" class="pill">0 logs</span>
    </div>
  </header>
  <nav class="tabs">
    <button class="tab active" data-tab="logs12">Logs 1-2</button>
    <button class="tab" data-tab="logs34">Logs 3-4</button>
    <button class="tab" data-tab="logs5">Logs 5</button>
    <button class="tab" data-tab="logsAll">All Logs</button>
    <button class="tab" data-tab="position">Position</button>
    <button class="tab" data-tab="graphs">Graphs</button>
    <button class="tab" data-tab="info">Info</button>
    <button class="tab" data-tab="batteryCharger">Battery Charger</button>
    <button class="tab" data-tab="usbPd">USB-C PD</button>
    <button class="tab" data-tab="rangingSettings">Ranging Settings</button>
    <button class="tab" data-tab="uwbSettings">UWB Settings</button>
    <button class="tab" data-tab="settings">Settings</button>
  </nav>
  <main>
    <section id="logs12" class="page active"><div class="terminal-grid">
      <div class="terminal" data-terminal="m1" data-modules="1"></div>
      <div class="terminal" data-terminal="m2" data-modules="2"></div>
    </div></section>
    <section id="logs34" class="page"><div class="terminal-grid">
      <div class="terminal" data-terminal="m3" data-modules="3"></div>
      <div class="terminal" data-terminal="m4" data-modules="4"></div>
    </div></section>
    <section id="logs5" class="page"><div class="terminal-grid single">
      <div class="terminal" data-terminal="m5" data-modules="5"></div>
    </div></section>
    <section id="logsAll" class="page"><div class="terminal-grid all">
      <div class="terminal" data-terminal="all" data-modules="all"></div>
    </div></section>
    <section id="position" class="page">
      <div class="position-layout">
        <div class="position-stage">
          <canvas id="positionCanvas" class="position-canvas"></canvas>
          <div id="positionOverlay" class="position-overlay">
            <div class="position-overlay-panel">
              <h2>Position mode is not active</h2>
              <p>Enable the selected position runtime to compute a new live position from fresh measurements.</p>
              <button class="primary" id="positionEnableRanging">Enable Position Runtime</button>
            </div>
          </div>
        </div>
        <aside class="position-panel">
          <h2>Position Setup</h2>
          <div class="form-grid">
            <label for="positionAnchorCount">Anchors used</label>
            <select id="positionAnchorCount"><option value="4">4 anchors</option><option value="3">3 anchors</option></select>
            <label for="positionSolver">Solver</label>
            <select id="positionSolver"><option value="tdoa" selected>FlexTDOA</option><option value="ranging">DS-TWR ranges</option></select>
            <label for="positionAnchors">Anchor IDs</label>
            <input id="positionAnchors" value="2,3,4,5">
            <label for="positionTags">Tag IDs</label>
            <input id="positionTags" value="1">
            <label for="positionMaxAgeSec">Fresh age s</label>
            <input id="positionMaxAgeSec" value="3" type="number" min="0.2" step="0.1">
            <label for="positionTdoaMode">TDOA filter</label>
            <select id="positionTdoaMode"><option value="static" selected>Static median</option><option value="dynamic">Dynamic 1.5s median</option><option value="auto">Auto Kalman</option></select>
          </div>
          <div class="param-legend">
            <div><b>Anchors</b><span>The first 3 or 4 IDs from the list are used for solving the position.</span></div>
            <div><b>Tags</b><span>Comma separated tag IDs. In FlexTDOA mode, tags only listen on UWB and the dashboard solves from range differences.</span></div>
            <div><b>Geometry</b><span>FlexTDOA uses live anchor-anchor ranges, so the anchors do not need to form a perfect square.</span></div>
            <div><b>TDOA filter</b><span>Static uses the median of recent paired observations. Dynamic uses a short 1.5 s median. Auto Kalman uses dynamic observations, gates outliers, and switches between static and moving behavior.</span></div>
          </div>
          <div class="form-actions">
            <button id="positionResetTrail">Reset Trail</button>
            <button class="primary" id="positionEnableRangingSide">Enable Position Runtime</button>
          </div>
          <div id="positionToast" class="toast"></div>
          <div class="section" style="margin-top:12px;">
            <h2>Measured Anchor Geometry</h2>
            <div class="muted" style="margin-bottom:8px;">Relative anchor coordinates are reconstructed from live anchor-anchor ranges.</div>
            <table>
              <thead><tr><th>Pair</th><th>med / avg</th><th>std</th><th>age</th><th>fit</th></tr></thead>
              <tbody id="positionGeometryRows"></tbody>
            </table>
          </div>
          <div class="section">
            <h2>Live Position</h2>
            <div class="position-legend"><span style="color:#d7352a">tag</span><span style="color:#2b64d8">trail</span><span class="ring" style="color:#2b64d8">anchor drift</span><span style="color:#16833a">anchor</span></div>
            <div id="positionReadout" class="position-readout"></div>
            <table>
              <thead><tr><th>Tag</th><th>est. 1σ</th><th>RMS</th><th>max</th></tr></thead>
              <tbody id="positionAccuracyRows"></tbody>
            </table>
          </div>
          <div class="section">
            <h2 id="positionMeasurementTitle">Measurements</h2>
            <table>
              <thead id="positionMeasurementHead"><tr><th>Tag</th><th>Anchor</th><th>m</th><th>age</th><th>resid.</th></tr></thead>
              <tbody id="positionDistanceRows"></tbody>
            </table>
          </div>
        </aside>
      </div>
    </section>
    <section id="graphs" class="page">
      <div class="graphs-layout">
        <div id="accelCharts" class="chart-stack"></div>
        <aside class="latest-panel">
          <h2>Accelerometer</h2>
          <div class="legend"><span class="lx">X</span><span class="ly">Y</span><span class="lz">Z</span></div>
          <div class="graph-controls">
            <h3>Scope</h3>
            <div class="graph-control-grid">
              <label for="accelTimebase">Timebase</label>
              <select id="accelTimebase">
                <option value="1">1 s/div</option>
                <option value="2">2 s/div</option>
                <option value="5" selected>5 s/div</option>
                <option value="10">10 s/div</option>
              </select>
              <label for="accelSampleHz">Samples/s</label>
              <input id="accelSampleHz" value="20" type="number" min="1" max="500" step="1" inputmode="numeric">
              <label for="accelTargets">Targets</label>
              <select id="accelTargets">
                <option value="all">all modules</option>
                <option value="1">module 1</option>
                <option value="2">module 2</option>
                <option value="3">module 3</option>
                <option value="4">module 4</option>
                <option value="5">module 5</option>
              </select>
            </div>
            <div class="graph-control-actions">
              <button class="primary" id="applyAccelSample">Apply</button>
            </div>
            <div id="accelToast" class="toast"></div>
          </div>
          <div id="accelLatestRows"></div>
        </aside>
      </div>
    </section>
    <section id="info" class="page">
      <div class="table-wrap">
        <table>
          <thead><tr><th>Module</th><th>Wi-Fi</th><th>Runtime</th><th>Components</th><th>GPS</th><th>UWB</th><th>Antenna</th><th>Logs</th><th>Resources</th><th>Battery</th></tr></thead>
          <tbody id="infoRows"></tbody>
        </table>
      </div>
    </section>
    <section id="batteryCharger" class="page">
      <div class="settings">
        <div class="settings-grid charger-grid">
          <div>
            <div class="section">
              <h2>Live Charger Status</h2>
              <table class="charger-status-table">
                <colgroup>
                  <col class="col-module">
                  <col class="col-power">
                  <col class="col-adc">
                  <col class="col-limits">
                  <col class="col-measurements">
                  <col class="col-write">
                </colgroup>
                <thead>
                  <tr>
                    <th>Module</th><th>Power</th><th>ADC</th><th>Limits</th><th>Measurements</th><th>Last Write</th>
                  </tr>
                </thead>
                <tbody id="chargerRows"></tbody>
              </table>
            </div>
            <div class="section charger-raw-tool hidden">
              <h2>Raw Registers</h2>
              <div class="form-grid">
                <label for="chargerRawModule">Module</label>
                <select id="chargerRawModule">
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
              </div>
              <div id="chargerRawRegisters" class="reg-grid"></div>
            </div>
          </div>
          <div>
            <div class="section">
              <h2>ADC</h2>
              <div class="form-grid">
                <label for="chargerAdcTargets">Targets</label>
                <select id="chargerAdcTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="chargerAdcSelectionStatus" class="field-note">waiting for status</div>
                <label for="chargerAdcEnabled">ADC</label>
                <div class="checkbox-row"><input id="chargerAdcEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerAdcRate">Rate</label>
                <select id="chargerAdcRate"><option value="continuous">continuous</option><option value="oneshot">one shot</option></select>
                <label for="chargerAdcSample">Sample</label>
                <select id="chargerAdcSample">
                  <option value="0">15 bit / 24 ms</option>
                  <option value="1">14 bit / 12 ms</option>
                  <option value="2" selected>13 bit / 6 ms</option>
                  <option value="3">12 bit / 3 ms</option>
                </select>
	                <label for="chargerAdcAvg">Average</label>
	                <div class="checkbox-row"><input id="chargerAdcAvg" type="checkbox"><span>running average</span></div>
	              </div>
	              <div class="param-legend">
	                <div><b>ADC</b><span>Enables internal charger measurements for voltage, current, temperature, D+ and D-.</span></div>
	                <div><b>Rate</b><span>Continuous keeps converting; one shot converts once per request.</span></div>
	                <div><b>Sample</b><span>Higher bit depth is slower and quieter; lower bit depth is faster and noisier.</span></div>
	                <div><b>Average</b><span>Smooths readings, but makes short changes less visible.</span></div>
	              </div>
	              <div class="form-actions">
	                <button class="primary" id="applyChargerAdc">Apply ADC</button>
	                <button id="disableChargerWatchdog">Disable Watchdog</button>
                <button id="refreshCharger">Refresh</button>
              </div>
              <div id="chargerAdcToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Charge Limits</h2>
              <div class="form-grid">
                <label for="chargerLimitTargets">Targets</label>
                <select id="chargerLimitTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="chargerLimitSelectionStatus" class="field-note">waiting for status</div>
                <label for="chargerChargeEnabled">Charging</label>
                <div class="checkbox-row"><input id="chargerChargeEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerMinimalSystemMv">VSYSMIN mV</label>
                <input id="chargerMinimalSystemMv" type="number" min="2500" max="16000" step="250">
                <label for="chargerChargeVoltageMv">Charge voltage mV</label>
                <input id="chargerChargeVoltageMv" type="number" min="3000" max="18800" step="10">
                <label for="chargerChargeCurrentMa">Charge current mA</label>
                <input id="chargerChargeCurrentMa" type="number" min="50" max="5000" step="10">
                <label for="chargerInputVoltageMv">VINDPM mV</label>
                <input id="chargerInputVoltageMv" type="number" min="3600" max="22000" step="100">
                <label for="chargerInputCurrentMa">IINDPM mA</label>
                <input id="chargerInputCurrentMa" type="number" min="100" max="3300" step="10">
                <label for="chargerExtIlimEnabled">ILIM_HIZ clamp</label>
                <div class="checkbox-row"><input id="chargerExtIlimEnabled" type="checkbox"><span>enabled</span></div>
              </div>
              <div class="param-legend">
                <div><b>Charging</b><span>Controls EN_CHG. Disabling it stops battery charging but can leave the board powered from VBUS/SYS.</span></div>
                <div><b>VSYSMIN</b><span>Minimum SYS rail target when the battery is low; step 250 mV.</span></div>
                <div><b>Charge voltage</b><span>Battery final voltage limit; sensitive for Li-Po safety; step 10 mV.</span></div>
                <div><b>Charge current</b><span>Maximum battery charge current before thermal/input limits intervene; step 10 mA.</span></div>
                <div><b>VINDPM</b><span>Input voltage DPM threshold: reduce load if VBUS falls below this; step 100 mV.</span></div>
                <div><b>IINDPM</b><span>Input current DPM limit: maximum current drawn from the adapter/USB source; step 10 mA.</span></div>
                <div><b>ILIM_HIZ clamp</b><span>Controls EN_EXTILIM. Enabled means the external ILIM_HIZ pin can clamp IINDPM; disabled lets software set IINDPM above that hardware pin limit.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyChargerLimits">Apply Limits</button>
              </div>
              <div id="chargerLimitsToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Termination / Recharge</h2>
              <div class="form-grid">
                <label for="chargerTerminationTargets">Targets</label>
                <select id="chargerTerminationTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="chargerTerminationSelectionStatus" class="field-note">waiting for status</div>
                <label for="chargerTerminationEnabled">Termination</label>
                <div class="checkbox-row"><input id="chargerTerminationEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerTerminationCurrentMa">ITERM mA</label>
                <input id="chargerTerminationCurrentMa" type="number" min="40" max="1000" step="40">
                <label for="chargerRechargeOffsetMv">VRECHG offset mV</label>
                <input id="chargerRechargeOffsetMv" type="number" min="50" max="800" step="50">
                <label for="chargerRechargeDeglitchMs">TRECHG debounce</label>
                <select id="chargerRechargeDeglitchMs">
                  <option value="64">64 ms</option>
                  <option value="256">256 ms</option>
                  <option value="1024" selected>1024 ms</option>
                  <option value="2048">2048 ms</option>
                </select>
              </div>
              <div class="param-legend">
                <div><b>Termination</b><span>Controls EN_TERM. When enabled, BQ25792 can stop charging after CV current falls below ITERM.</span></div>
                <div><b>ITERM</b><span>Termination current threshold; step 40 mA. Low values may terminate above the requested value due to comparator offset.</span></div>
                <div><b>VRECHG</b><span>Recharge threshold below VREG. Example: VREG 4200 mV and offset 200 mV restarts near 4000 mV.</span></div>
                <div><b>TRECHG</b><span>Debounce time for VBAT staying below the recharge threshold before a new charge cycle starts.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyChargerTermination">Apply Termination</button>
              </div>
              <div id="chargerTerminationToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Safety Timers</h2>
              <div class="form-grid">
                <label for="chargerTimerTargets">Targets</label>
                <select id="chargerTimerTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="chargerTimerSelectionStatus" class="field-note">waiting for status</div>
                <label for="chargerFastTimerEnabled">Fast timer</label>
                <div class="checkbox-row"><input id="chargerFastTimerEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerFastTimerHours">Fast duration</label>
                <select id="chargerFastTimerHours">
                  <option value="5">5 h</option>
                  <option value="8">8 h</option>
                  <option value="12" selected>12 h</option>
                  <option value="24">24 h</option>
                </select>
                <label for="chargerPrechargeTimerEnabled">Pre-charge timer</label>
                <div class="checkbox-row"><input id="chargerPrechargeTimerEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerPrechargeTimerMinutes">Pre-charge duration</label>
                <select id="chargerPrechargeTimerMinutes">
                  <option value="120" selected>120 min</option>
                  <option value="30">30 min</option>
                </select>
                <label for="chargerTrickleTimerEnabled">Trickle timer</label>
                <div class="checkbox-row"><input id="chargerTrickleTimerEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerTopoffTimerMinutes">Top-off timer</label>
                <select id="chargerTopoffTimerMinutes">
                  <option value="0" selected>disabled</option>
                  <option value="15">15 min</option>
                  <option value="30">30 min</option>
                  <option value="45">45 min</option>
                </select>
                <label for="chargerTimer2xEnabled">TMR2X</label>
                <div class="checkbox-row"><input id="chargerTimer2xEnabled" type="checkbox"><span>double during DPM/TREG</span></div>
              </div>
              <div class="param-legend">
                <div><b>Fast timer</b><span>Safety timeout for fast CC/CV charging. If it expires, charging can terminate before the cell reaches VREG.</span></div>
                <div><b>Pre-charge</b><span>Timeout for the low-voltage pre-charge phase before normal fast charge starts.</span></div>
                <div><b>Trickle</b><span>Fixed 1 h timeout for a deeply discharged battery before pre-charge.</span></div>
                <div><b>Top-off</b><span>Optional extra charging time after termination threshold is reached.</span></div>
                <div><b>TMR2X</b><span>Doubles active safety timers while input-current, input-voltage, or thermal regulation slows charging.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyChargerTimers">Apply Safety Timers</button>
                <button id="resetChargerCycle">Reset Charge Cycle</button>
              </div>
              <div id="chargerTimersToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Service Tools</h2>
              <div class="checkbox-row"><input id="chargerShowRawTools" type="checkbox"><span>show raw register tools</span></div>
              <div class="param-legend">
                <div><b>Raw tools</b><span>For datasheet-level experiments only. Normal charger setup should use the controls above.</span></div>
              </div>
            </div>
            <div class="section charger-raw-tool hidden">
              <h2>Raw Register Write</h2>
              <div class="form-grid">
                <label for="chargerRawReg">Register</label>
                <input id="chargerRawReg" value="0x2E">
                <label for="chargerRawValue">Value</label>
                <input id="chargerRawValue" value="0xA0">
                <label for="chargerRawMask">Mask</label>
                <input id="chargerRawMask" placeholder="optional, e.g. 0x80">
                <label for="chargerRawBits">Bits</label>
                <input id="chargerRawBits" placeholder="optional with mask">
                <label for="chargerRawConfirm">Confirm</label>
                <div class="checkbox-row"><input id="chargerRawConfirm" type="checkbox"><span>allow raw write</span></div>
              </div>
              <div class="form-actions">
                <button class="danger" id="applyChargerRaw">Apply Raw Register</button>
              </div>
              <div id="chargerRawToast" class="toast"></div>
            </div>
          </div>
        </div>
      </div>
    </section>
    <section id="usbPd" class="page">
      <div class="settings">
        <div class="settings-grid pd-grid">
          <div>
            <div class="section">
              <h2>Live USB-C PD Status</h2>
              <table class="pd-status-table">
                <colgroup>
                  <col class="col-module">
                  <col class="col-link">
                  <col class="col-contract">
                  <col class="col-pdos">
                  <col class="col-ops">
                </colgroup>
                <thead>
                  <tr>
                    <th>Module</th><th>USB-C</th><th>Contract</th><th>PDOs</th><th>Operations</th>
                  </tr>
                </thead>
                <tbody id="pdRows"></tbody>
              </table>
            </div>
            <div class="section pd-raw-tool hidden">
              <h2>Raw Registers</h2>
              <div class="form-grid">
                <label for="pdRawModule">Module</label>
                <select id="pdRawModule">
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
              </div>
              <div id="pdRawRegisters" class="reg-grid"></div>
            </div>
          </div>
          <div>
            <div class="section">
              <h2>Contract</h2>
              <div class="form-grid">
                <label for="pdContractTargets">Targets</label>
                <select id="pdContractTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="pdContractSelectionStatus" class="field-note">waiting for status</div>
                <label for="pdSourcePdoPos">Source PDO</label>
                <input id="pdSourcePdoPos" value="1" type="number" min="1" max="7" step="1">
                <label for="pdUsb2SwitchClosed">USB2 switch</label>
                <div class="checkbox-row"><input id="pdUsb2SwitchClosed" type="checkbox"><span>D+/D- pass-through closed</span></div>
              </div>
              <div class="param-legend">
                <div><b>Source PDO</b><span>Requests one advertised fixed supply profile by position. This is the normal way to ask for 5 V, 9 V, 15 V, and similar fixed rails.</span></div>
                <div><b>USB2 switch</b><span>Controls MAX77958 CTRL1 pass-through switches for D+ and D-.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyPdSource">Request Source PDO</button>
                <button id="applyPdUsb2">Apply USB2 Switch</button>
                <button id="refreshPd">Refresh</button>
                <button id="triggerPdBc">Trigger BC Detect</button>
              </div>
              <div id="pdContractToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Sink PDOs</h2>
              <div class="form-grid">
                <label for="pdSinkTargets">Targets</label>
                <select id="pdSinkTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="pdSinkSelectionStatus" class="field-note">waiting for status</div>
                <label for="pdSinkPdos">PDO list</label>
                <input id="pdSinkPdos" value="5000:3000,9000:3000,15000:3000" placeholder="mV:mA,mV:mA">
                <label for="pdSinkPdosMtp">Store</label>
                <div class="checkbox-row"><input id="pdSinkPdosMtp" type="checkbox"><span>write MTP, not only RAM</span></div>
              </div>
              <div class="param-legend">
                <div><b>PDO list</b><span>Comma separated fixed sink capabilities in millivolts and milliamps, for example 5000:3000,9000:3000.</span></div>
                <div><b>MTP</b><span>Writes the non-volatile MAX77958 profile. Keep unchecked while experimenting.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyPdSinkPdos">Apply Sink PDOs</button>
                <button id="readPdSinkRam">Read RAM</button>
                <button id="readPdSinkMtp">Read MTP</button>
              </div>
              <div id="pdSinkToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>PPS / APDO</h2>
              <div class="form-grid">
                <label for="pdPpsTargets">Targets</label>
                <select id="pdPpsTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label>Loaded</label>
                <div id="pdPpsSelectionStatus" class="field-note">waiting for status</div>
                <label for="pdPpsEnabled">Default PPS</label>
                <div class="checkbox-row"><input id="pdPpsEnabled" type="checkbox"><span>enabled</span></div>
                <label for="pdPpsVoltageMv">Default voltage mV</label>
                <input id="pdPpsVoltageMv" value="5000" type="number" min="3300" max="21000" step="20">
                <label for="pdPpsCurrentMa">Default current mA</label>
                <input id="pdPpsCurrentMa" value="1000" type="number" min="0" max="5000" step="50">
                <label for="pdApdoPos">APDO position</label>
                <input id="pdApdoPos" value="1" type="number" min="1" max="7" step="1">
                <label for="pdApdoVoltageMv">APDO voltage mV</label>
                <input id="pdApdoVoltageMv" value="5000" type="number" min="3300" max="21000" step="20">
                <label for="pdApdoCurrentMa">APDO current mA</label>
                <input id="pdApdoCurrentMa" value="1000" type="number" min="0" max="5000" step="50">
              </div>
              <div class="param-legend">
                <div><b>Default PPS</b><span>Saved policy applied at boot. Use it only with a source that advertises PPS/APDO.</span></div>
                <div><b>APDO request</b><span>Runtime programmable supply request. Voltage is encoded in 20 mV units, current in 50 mA units.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyPdPpsDefault">Apply PPS Default</button>
                <button id="requestPdApdo">Request APDO</button>
              </div>
              <div id="pdPpsToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Service Tools</h2>
              <div class="checkbox-row"><input id="pdShowRawTools" type="checkbox"><span>show raw register tools</span></div>
              <div class="param-legend">
                <div><b>Raw tools</b><span>For datasheet-level MAX77958 experiments only. Normal voltage setup should use PDO/PPS controls above.</span></div>
              </div>
            </div>
            <div class="section pd-raw-tool hidden">
              <h2>Raw Register Write</h2>
              <div class="form-grid">
                <label for="pdRawReg">Register</label>
                <input id="pdRawReg" value="0x00">
                <label for="pdRawValue">Value</label>
                <input id="pdRawValue" value="0x00">
                <label for="pdRawConfirm">Confirm</label>
                <div class="checkbox-row"><input id="pdRawConfirm" type="checkbox"><span>allow raw write</span></div>
              </div>
              <div class="form-actions">
                <button class="danger" id="applyPdRaw">Apply Raw Register</button>
              </div>
              <div id="pdRawToast" class="toast"></div>
            </div>
          </div>
        </div>
      </div>
    </section>
    <section id="rangingSettings" class="page">
      <div class="settings">
        <div class="section">
          <h2>Ranging Profiles</h2>
          <div class="form-grid">
            <label for="rangingProfileTargets">Targets</label>
            <select id="rangingProfileTargets">
              <option value="all">all modules</option>
              <option value="1">module 1</option>
              <option value="2">module 2</option>
              <option value="3">module 3</option>
              <option value="4">module 4</option>
              <option value="5">module 5</option>
            </select>
          </div>
          <p class="muted profile-note">Apply writes every timing parameter in the selected profile to ESP32 NVS through runtime config. The same profile updates FlexTDOA anchor slots and classic ranging slots.</p>
          <div class="profile-grid">
            <div class="profile-card" data-profile="baseline">
              <h3>Stable Baseline</h3>
              <p class="muted">Known-good reference profile using the currently validated timing values.</p>
              <div class="form-grid compact">
                <label for="profileBaselineSlotMs">Slot ms</label>
                <input id="profileBaselineSlotMs" value="100" type="number" min="1" step="1">
                <label for="profileBaselineRoundGapMs">Round gap ms</label>
                <input id="profileBaselineRoundGapMs" value="10" type="number" min="1" step="1">
                <label for="profileBaselineRxSliceMs">RX slice ms</label>
                <input id="profileBaselineRxSliceMs" value="100" type="number" min="1" step="1">
                <label for="profileBaselineCommandDelayMs">Command delay ms</label>
                <input id="profileBaselineCommandDelayMs" value="10" type="number" min="1" step="1">
                <label for="profileBaselineTimeoutMs">Classic DS-TWR timeout ms</label>
                <input id="profileBaselineTimeoutMs" value="90" type="number" min="1" step="1">
                <label for="profileBaselineRespDelayMs">RESP delay ms</label>
                <input id="profileBaselineRespDelayMs" value="20" type="number" min="1" step="1">
                <label for="profileBaselineFinalDelayMs">FINAL delay ms</label>
                <input id="profileBaselineFinalDelayMs" value="20" type="number" min="1" step="1">
                <label for="profileBaselineReportDelayMs">REPORT delay ms</label>
                <input id="profileBaselineReportDelayMs" value="10" type="number" min="1" step="1">
                <label for="profileBaselineAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="profileBaselineAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
              <div class="profile-summary" id="profileBaselineSummary"></div>
              <div class="form-actions">
                <button class="primary apply-ranging-profile" data-profile="baseline">Apply Stable Baseline</button>
                <button class="reset-ranging-profile" data-profile="baseline">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card" data-profile="safe">
              <h3>Safe Fast</h3>
              <p class="muted">First fast profile to try when stability matters more than minimum latency.</p>
              <div class="form-grid compact">
                <label for="profileSafeSlotMs">Slot ms</label>
                <input id="profileSafeSlotMs" value="60" type="number" min="1" step="1">
                <label for="profileSafeRoundGapMs">Round gap ms</label>
                <input id="profileSafeRoundGapMs" value="10" type="number" min="1" step="1">
                <label for="profileSafeRxSliceMs">RX slice ms</label>
                <input id="profileSafeRxSliceMs" value="60" type="number" min="1" step="1">
                <label for="profileSafeCommandDelayMs">Command delay ms</label>
                <input id="profileSafeCommandDelayMs" value="5" type="number" min="1" step="1">
                <label for="profileSafeTimeoutMs">Classic DS-TWR timeout ms</label>
                <input id="profileSafeTimeoutMs" value="35" type="number" min="1" step="1">
                <label for="profileSafeRespDelayMs">RESP delay ms</label>
                <input id="profileSafeRespDelayMs" value="15" type="number" min="1" step="1">
                <label for="profileSafeFinalDelayMs">FINAL delay ms</label>
                <input id="profileSafeFinalDelayMs" value="15" type="number" min="1" step="1">
                <label for="profileSafeReportDelayMs">REPORT delay ms</label>
                <input id="profileSafeReportDelayMs" value="5" type="number" min="1" step="1">
                <label for="profileSafeAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="profileSafeAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
              <div class="profile-summary" id="profileSafeSummary"></div>
              <div class="form-actions">
                <button class="primary apply-ranging-profile" data-profile="safe">Apply Safe Fast</button>
                <button class="reset-ranging-profile" data-profile="safe">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card" data-profile="balanced">
              <h3>Balanced</h3>
              <p class="muted">Recommended next target: much faster than current settings, still with useful margin.</p>
              <div class="form-grid compact">
                <label for="profileBalancedSlotMs">Slot ms</label>
                <input id="profileBalancedSlotMs" value="50" type="number" min="1" step="1">
                <label for="profileBalancedRoundGapMs">Round gap ms</label>
                <input id="profileBalancedRoundGapMs" value="10" type="number" min="1" step="1">
                <label for="profileBalancedRxSliceMs">RX slice ms</label>
                <input id="profileBalancedRxSliceMs" value="50" type="number" min="1" step="1">
                <label for="profileBalancedCommandDelayMs">Command delay ms</label>
                <input id="profileBalancedCommandDelayMs" value="5" type="number" min="1" step="1">
                <label for="profileBalancedTimeoutMs">Classic DS-TWR timeout ms</label>
                <input id="profileBalancedTimeoutMs" value="25" type="number" min="1" step="1">
                <label for="profileBalancedRespDelayMs">RESP delay ms</label>
                <input id="profileBalancedRespDelayMs" value="10" type="number" min="1" step="1">
                <label for="profileBalancedFinalDelayMs">FINAL delay ms</label>
                <input id="profileBalancedFinalDelayMs" value="10" type="number" min="1" step="1">
                <label for="profileBalancedReportDelayMs">REPORT delay ms</label>
                <input id="profileBalancedReportDelayMs" value="5" type="number" min="1" step="1">
                <label for="profileBalancedAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="profileBalancedAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
              <div class="profile-summary" id="profileBalancedSummary"></div>
              <div class="form-actions">
                <button class="primary apply-ranging-profile" data-profile="balanced">Apply Balanced</button>
                <button class="reset-ranging-profile" data-profile="balanced">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card" data-profile="aggressive">
              <h3>Aggressive</h3>
              <p class="muted">Lowest-latency candidate. Use after Safe Fast/Balanced look clean.</p>
              <div class="form-grid compact">
                <label for="profileAggressiveSlotMs">Slot ms</label>
                <input id="profileAggressiveSlotMs" value="40" type="number" min="1" step="1">
                <label for="profileAggressiveRoundGapMs">Round gap ms</label>
                <input id="profileAggressiveRoundGapMs" value="10" type="number" min="1" step="1">
                <label for="profileAggressiveRxSliceMs">RX slice ms</label>
                <input id="profileAggressiveRxSliceMs" value="40" type="number" min="1" step="1">
                <label for="profileAggressiveCommandDelayMs">Command delay ms</label>
                <input id="profileAggressiveCommandDelayMs" value="3" type="number" min="1" step="1">
                <label for="profileAggressiveTimeoutMs">Classic DS-TWR timeout ms</label>
                <input id="profileAggressiveTimeoutMs" value="18" type="number" min="1" step="1">
                <label for="profileAggressiveRespDelayMs">RESP delay ms</label>
                <input id="profileAggressiveRespDelayMs" value="7" type="number" min="1" step="1">
                <label for="profileAggressiveFinalDelayMs">FINAL delay ms</label>
                <input id="profileAggressiveFinalDelayMs" value="7" type="number" min="1" step="1">
                <label for="profileAggressiveReportDelayMs">REPORT delay ms</label>
                <input id="profileAggressiveReportDelayMs" value="3" type="number" min="1" step="1">
                <label for="profileAggressiveAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="profileAggressiveAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
              <div class="profile-summary" id="profileAggressiveSummary"></div>
              <div class="form-actions">
                <button class="primary apply-ranging-profile" data-profile="aggressive">Apply Aggressive</button>
                <button class="reset-ranging-profile" data-profile="aggressive">Reset Defaults</button>
              </div>
            </div>
          </div>
          <div class="form-actions">
            <button id="resetAllRangingProfiles">Reset All Profile Defaults</button>
          </div>
          <div id="rangingProfileToast" class="toast"></div>
        </div>
      </div>
    </section>
    <section id="uwbSettings" class="page">
      <div class="settings">
        <div class="settings-grid">
          <div>
            <div class="section">
              <h2>Survey Timing</h2>
              <div class="form-grid">
                <label for="uwbTargets">Targets</label>
                <select id="uwbTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label for="uwbRadioChannel">Radio channel</label>
                <select id="uwbRadioChannel">
                  <option value="5">CH5</option>
                  <option value="9">CH9</option>
                </select>
                <label for="uwbSurveyRxMs">RX slice ms</label>
                <input id="uwbSurveyRxMs" value="100" type="number" min="1" step="1">
                <label for="uwbSurveyDelayMs">Command delay ms</label>
                <input id="uwbSurveyDelayMs" value="80" type="number" min="1" step="1">
                <label for="uwbSurveySlotMs">Slot ms</label>
                <input id="uwbSurveySlotMs" value="1000" type="number" min="1" step="1">
                <label for="uwbSurveyGapMs">Round gap ms</label>
                <input id="uwbSurveyGapMs" value="1500" type="number" min="1" step="1">
                <label for="uwbSurveyLogEvery">Passive log every</label>
                <input id="uwbSurveyLogEvery" value="20" type="number" min="1" step="1">
              </div>
            </div>
            <div class="section">
              <h2>Ranging Timing</h2>
              <div class="form-grid">
                <label for="uwbRangingSlotMs">Slot ms</label>
                <input id="uwbRangingSlotMs" value="350" type="number" min="1" step="1">
                <label for="uwbRangingGapMs">Round gap ms</label>
                <input id="uwbRangingGapMs" value="500" type="number" min="1" step="1">
                <label for="uwbRangingRxMs">RX slice ms</label>
                <input id="uwbRangingRxMs" value="100" type="number" min="1" step="1">
              </div>
            </div>
            <div class="section">
              <h2>Distance Test Timing</h2>
              <div class="form-grid">
                <label for="uwbDtInitiator">Initiator</label>
                <input id="uwbDtInitiator" value="1" type="number" min="1" step="1">
                <label for="uwbDtResponder">Responder</label>
                <input id="uwbDtResponder" value="2" type="number" min="1" step="1">
                <label for="uwbDtIntervalMs">Interval ms</label>
                <input id="uwbDtIntervalMs" value="1000" type="number" min="1" step="1">
                <label for="uwbDtRespDelayMs">RESP delay ms</label>
                <input id="uwbDtRespDelayMs" value="20" type="number" min="1" step="1">
                <label for="uwbDtFinalDelayMs">FINAL delay ms</label>
                <input id="uwbDtFinalDelayMs" value="20" type="number" min="1" step="1">
                <label for="uwbDtReportDelayMs">REPORT delay ms</label>
                <input id="uwbDtReportDelayMs" value="10" type="number" min="1" step="1">
                <label for="uwbDtAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="uwbDtAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
            </div>
          </div>
          <div>
            <div class="section">
              <h2>Antenna Delay</h2>
              <div class="form-grid">
                <label for="uwbAntennaDelayHex">Delay hex</label>
                <input id="uwbAntennaDelayHex" value="0x3FCA">
                <label for="uwbAdvancedReboot">Reboot</label>
                <select id="uwbAdvancedReboot"><option value="1">yes</option><option value="0">no</option></select>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyUwbSettings">Apply UWB Settings</button>
                <button id="applyAntennaDelay">Apply Antenna Delay</button>
                <button id="clearAntennaDelay">Clear Antenna Delay</button>
              </div>
              <div id="uwbToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Radio Profile</h2>
              <table>
                <tbody id="uwbRadioRows"></tbody>
              </table>
            </div>
          </div>
        </div>
      </div>
    </section>
    <section id="settings" class="page">
      <div class="settings">
        <div class="settings-grid">
          <div>
            <div class="section">
              <h2>Runtime</h2>
              <div class="form-grid">
                <label for="runtimeTargets">Targets</label>
                <select id="runtimeTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label for="runtimeMode">Mode</label>
                <select id="runtimeMode">
                  <option value="ranging">ranging</option>
                  <option value="flex_tdoa">FlexTDOA</option>
                  <option value="survey">survey</option>
                  <option value="distance">distance</option>
                  <option value="beacon">beacon</option>
                </select>
                <label for="runtimeTag">Tag ID</label>
                <input id="runtimeTag" value="1" inputmode="numeric">
                <label for="runtimeAnchors">Anchors</label>
                <input id="runtimeAnchors" value="2,3,4,5">
                <label for="runtimeReboot">Reboot</label>
                <select id="runtimeReboot"><option value="1">yes</option><option value="0">no</option></select>
                <label for="runtimeUwb">UWB</label>
                <div class="checkbox-row"><input id="runtimeUwb" type="checkbox" checked><span>DW3000 enabled</span></div>
                <label for="runtimeBno085">Accelerometer</label>
                <div class="checkbox-row"><input id="runtimeBno085" type="checkbox"><span>BNO085 enabled</span></div>
                <label for="runtimeGps">GPS</label>
                <div class="checkbox-row"><input id="runtimeGps" type="checkbox"><span>GPS UART parser</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyRuntime">Apply Runtime</button>
                <button id="clearRuntime">Clear Runtime Override</button>
              </div>
              <div id="runtimeToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Connectivity</h2>
              <div class="form-grid">
                <label for="runtimeTelemetryPort">Telemetry port</label>
                <input id="runtimeTelemetryPort" value="6055" type="number" min="1" max="65535" step="1" list="telemetryPortOptions">
                <datalist id="telemetryPortOptions"></datalist>
              </div>
              <div class="muted" id="telemetryPortHint"></div>
              <div class="form-actions">
                <button class="primary" id="applyTelemetryPort">Apply Telemetry Port</button>
              </div>
              <div id="telemetryPortToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Calibration Settings</h2>
              <div class="form-grid">
                <label for="uwbCalSummary">Summary every</label>
                <input id="uwbCalSummary" value="25" type="number" min="1" step="1">
                <label for="uwbCalMinMs">Slot ms</label>
                <input id="uwbCalMinMs" value="100" type="number" min="1" step="1">
                <label for="uwbCalGuardUs">Guard us</label>
                <input id="uwbCalGuardUs" value="500" type="number" min="1" step="1">
                <label for="uwbDtRxTimeoutMs">DS-TWR timeout ms</label>
                <input id="uwbDtRxTimeoutMs" value="100" type="number" min="1" step="1">
                <label for="uwbCalMaxMs">Round gap ms</label>
                <input id="uwbCalMaxMs" value="10" type="number" min="1" step="1">
                <label for="uwbCalRxMs">RX slice ms</label>
                <input id="uwbCalRxMs" value="50" type="number" min="1" step="1">
              </div>
              <div class="param-legend">
                <div><b>Summary every</b><span>Log diagnostic calibration statistics every N accepted samples.</span></div>
                <div><b>Slot ms</b><span>Fixed time budget for one directed pair, for example M4 -> M2.</span></div>
                <div><b>DS-TWR timeout</b><span>Maximum wait for expected ranging frames inside the DS-TWR exchange.</span></div>
                <div><b>Round gap</b><span>Delay after all six directed pairs finish, before the next calibration round starts.</span></div>
                <div><b>RX slice</b><span>Short listen window used by passive calibration nodes while they monitor the rest of the slot.</span></div>
              </div>
              <div class="form-actions">
                <button class="primary" id="applyCalibrationSettings">Apply Calibration Settings</button>
              </div>
              <div id="calTimingToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Antenna Delay Calibration</h2>
              <div class="form-grid">
                <label for="calTargets">Targets</label>
                <select id="calTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
                <label for="calMethod">Method</label>
                <select id="calMethod"><option value="two">2 modules</option><option value="three">3 modules</option></select>
                <label class="two-only" for="calPair">Modules</label>
                <input class="two-only" id="calPair" value="1,2" placeholder="reference,DUT">
                <label class="two-only">Roles</label>
                <div class="two-only field-note">first module is the reference; second module is the DUT that gets adjusted</div>
                <label class="two-only" for="calKnownCm">Distance 0-1 cm</label>
                <input class="two-only cm-input" id="calKnownCm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calThree">Modules</label>
                <input class="three-only" id="calThree" value="1,2,3">
                <label class="three-only" for="calD01Cm">Distance 0-1 cm</label>
                <input class="three-only cm-input" id="calD01Cm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calD02Cm">Distance 0-2 cm</label>
                <input class="three-only cm-input" id="calD02Cm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calD12Cm">Distance 1-2 cm</label>
                <input class="three-only cm-input" id="calD12Cm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label for="calSamples">Samples</label>
                <input id="calSamples" value="39" type="number" min="1" step="1" inputmode="numeric">
                <label for="calAutoApply">Auto apply</label>
                <div class="checkbox-row"><input id="calAutoApply" type="checkbox" checked><span>write antenna delay</span></div>
                <label for="calMinApplyDtu">Min apply DTU</label>
                <input id="calMinApplyDtu" value="2" type="number" min="0" step="1" inputmode="numeric">
                <label for="calReferenceGuardCm">Reference guard cm</label>
                <input id="calReferenceGuardCm" value="2.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label for="calTimeoutSec">Timeout s</label>
                <input id="calTimeoutSec" value="240" type="number" min="10" step="5" inputmode="numeric">
              </div>
              <div class="form-actions">
                <button class="primary" id="autoCalibration">Auto Calibrate + Apply</button>
                <button class="danger" id="cancelCalibration" disabled>Cancel Calibration</button>
              </div>
              <div id="calAutoToast" class="toast"></div>
            </div>
          </div>
          <div>
            <div class="section">
              <h2>Calibration Geometry</h2>
              <div id="calDiagram" class="diagram"></div>
            </div>
            <div class="section">
              <h2>Calibration Results</h2>
              <div id="calResults" class="cal-results"></div>
            </div>
          </div>
        </div>
      </div>
    </section>
  </main>
</div>
<script>
const state = {
  activeTab: localStorage.getItem("uwbDash.activeTab") || "logs12",
  logs: [],
  lastId: 0,
  lastAccelId: 0,
  terminals: new Map(),
  statuses: [],
  accelHistory: {},
  accelSeen: new Set(),
  timebaseSecPerDiv: Number(localStorage.getItem("uwbDash.setting.accelTimebase") || "5"),
  chartsReady: false,
  accelRenderPending: false,
  accelFetchPending: false,
  latestAccelRenderMs: 0,
  hydratedSettings: false,
  calibrationResult: null,
  ranging: {distances: {}, max_age_sec: 3},
  tdoa: {observations: {}, anchor_distances: {}, max_age_sec: 3},
  positionTrail: {},
  positionAnchorTrail: {},
  positionFilters: {},
  positionResults: {},
  positionWasActive: false,
};
const tdoaReverseSumRejectM = 0.75;
const tdoaResidualRejectM = 0.45;
const kalmanMaxPredictionAgeSec = 2.5;
const accelLineRe = /\bBNO085 accel x=([-+]?\d+(?:\.\d+)?) y=([-+]?\d+(?:\.\d+)?) z=([-+]?\d+(?:\.\d+)?) m\/s\^2 accuracy=(\d+) reports=(\d+)/;
const maxAccelSamples = 30000;
const maxSeriesPoints = 1600;
const plot = {left: 52, right: 704, top: 14, bottom: 166, width: 652, height: 152};
const toastTimers = new Map();
let calibrationPollTimer = null;
let calibrationJobId = null;
const rangingProfileFields = [
  {key: "slotMs", suffix: "SlotMs"},
  {key: "roundGapMs", suffix: "RoundGapMs"},
  {key: "rxSliceMs", suffix: "RxSliceMs"},
  {key: "commandDelayMs", suffix: "CommandDelayMs"},
  {key: "timeoutMs", suffix: "TimeoutMs"},
  {key: "respDelayMs", suffix: "RespDelayMs"},
  {key: "finalDelayMs", suffix: "FinalDelayMs"},
  {key: "reportDelayMs", suffix: "ReportDelayMs"},
  {key: "autoRxDelayUus", suffix: "AutoRxDelayUus"},
];
const rangingProfileDefaults = {
  baseline: {
    prefix: "profileBaseline",
    label: "Stable Baseline",
    slotMs: 100,
    roundGapMs: 10,
    rxSliceMs: 100,
    commandDelayMs: 10,
    timeoutMs: 90,
    respDelayMs: 20,
    finalDelayMs: 20,
    reportDelayMs: 10,
    autoRxDelayUus: 500,
  },
  safe: {
    prefix: "profileSafe",
    label: "Safe Fast",
    slotMs: 60,
    roundGapMs: 10,
    rxSliceMs: 60,
    commandDelayMs: 5,
    timeoutMs: 35,
    respDelayMs: 15,
    finalDelayMs: 15,
    reportDelayMs: 5,
    autoRxDelayUus: 500,
  },
  balanced: {
    prefix: "profileBalanced",
    label: "Balanced",
    slotMs: 50,
    roundGapMs: 10,
    rxSliceMs: 50,
    commandDelayMs: 5,
    timeoutMs: 25,
    respDelayMs: 10,
    finalDelayMs: 10,
    reportDelayMs: 5,
    autoRxDelayUus: 500,
  },
  aggressive: {
    prefix: "profileAggressive",
    label: "Aggressive",
    slotMs: 40,
    roundGapMs: 10,
    rxSliceMs: 40,
    commandDelayMs: 3,
    timeoutMs: 18,
    respDelayMs: 7,
    finalDelayMs: 7,
    reportDelayMs: 3,
    autoRxDelayUus: 500,
  },
};
const BQ_REG_NAMES = {
  0x00: "Minimal System Voltage",
  0x01: "Charge Voltage MSB",
  0x02: "Charge Voltage LSB",
  0x03: "Charge Current MSB",
  0x04: "Charge Current LSB",
  0x05: "Input Voltage Limit",
  0x06: "Input Current MSB",
  0x07: "Input Current LSB",
  0x08: "Precharge Control",
  0x09: "Termination Control",
  0x0A: "Recharge Control",
  0x0B: "VOTG Regulation MSB",
  0x0C: "VOTG Regulation LSB",
  0x0D: "IOTG Regulation",
  0x0E: "Timer Control",
  0x0F: "Charger Control 0",
  0x10: "Charger Control 1",
  0x11: "Charger Control 2",
  0x12: "Charger Control 3",
  0x13: "Charger Control 4",
  0x14: "Charger Control 5",
  0x16: "Temperature Control",
  0x17: "NTC Control 0",
  0x18: "NTC Control 1",
  0x19: "ICO Current MSB",
  0x1A: "ICO Current LSB",
  0x1B: "Charger Status 0",
  0x1C: "Charger Status 1",
  0x1D: "Charger Status 2",
  0x1E: "Charger Status 3",
  0x1F: "Charger Status 4",
  0x20: "Fault Status 0",
  0x21: "Fault Status 1",
  0x22: "Charger Flag 0",
  0x23: "Charger Flag 1",
  0x24: "Charger Flag 2",
  0x25: "Charger Flag 3",
  0x26: "Fault Flag 0",
  0x27: "Fault Flag 1",
  0x28: "Charger Mask 0",
  0x29: "Charger Mask 1",
  0x2A: "Charger Mask 2",
  0x2B: "Charger Mask 3",
  0x2C: "Fault Mask 0",
  0x2D: "Fault Mask 1",
  0x2E: "ADC Control",
  0x2F: "ADC Disable 0",
  0x30: "ADC Disable 1",
  0x31: "IBUS ADC MSB",
  0x32: "IBUS ADC LSB",
  0x33: "IBAT ADC MSB",
  0x34: "IBAT ADC LSB",
  0x35: "VBUS ADC MSB",
  0x36: "VBUS ADC LSB",
  0x37: "VAC1 ADC MSB",
  0x38: "VAC1 ADC LSB",
  0x39: "VAC2 ADC MSB",
  0x3A: "VAC2 ADC LSB",
  0x3B: "VBAT ADC MSB",
  0x3C: "VBAT ADC LSB",
  0x3D: "VSYS ADC MSB",
  0x3E: "VSYS ADC LSB",
  0x3F: "TS ADC MSB",
  0x40: "TS ADC LSB",
  0x41: "TDIE ADC MSB",
  0x42: "TDIE ADC LSB",
  0x43: "D+ ADC MSB",
  0x44: "D+ ADC LSB",
  0x45: "D- ADC MSB",
  0x46: "D- ADC LSB",
  0x47: "DPDM Driver",
  0x48: "Part Information",
};
const MAX77958_REG_NAMES = {
  0x00: "Device ID",
  0x01: "Device Revision",
  0x02: "FW Revision",
  0x03: "FW Sub Version",
  0x04: "UIC INT",
  0x05: "CC INT",
  0x06: "PD INT",
  0x07: "Action INT",
  0x08: "USBC Status 1",
  0x09: "USBC Status 2",
  0x0A: "BC Status",
  0x0B: "DP Status",
  0x0C: "CC Status 0",
  0x0D: "CC Status 1",
  0x0E: "PD Status 0",
  0x0F: "PD Status 1",
  0x10: "UIC INT Mask",
  0x11: "CC INT Mask",
  0x12: "PD INT Mask",
  0x13: "Action INT Mask",
  0x21: "AP Data Out 0",
  0x41: "AP Data Out 32",
  0x51: "AP Data In 0",
  0x71: "AP Data In 32",
  0x80: "SW Reset",
  0xE0: "I2C Config",
};

function storageKey(id, name) { return `uwbDash.${id}.${name}`; }
function levelClass(level) { return `level-${["D","I","W","E"].includes(level) ? level : "unknown"}`; }
function esc(text) {
  return String(text ?? "").replace(/[&<>"']/g, ch => ({ "&":"&amp;", "<":"&lt;", ">":"&gt;", '"':"&quot;", "'":"&#39;" }[ch]));
}
function fmtAge(ts) {
  if (!ts) return "-";
  const age = Math.max(0, Date.now() / 1000 - ts);
  return `${age.toFixed(1)}s`;
}
function statusIsFresh(item) {
  return Boolean(item?.http_status_online);
}
function statusAgeText(item) {
  const age = Number(item?.http_status_age_sec);
  return Number.isFinite(age) && age >= 0 ? `${age.toFixed(1)}s` : fmtAge(item?.status_updated_at);
}
function renderModuleCell(item) {
  const fresh = statusIsFresh(item);
  const statusLine = fresh
    ? `<span class="ok">HTTP live</span>`
    : `<span class="warn">HTTP stale ${esc(statusAgeText(item))}</span>`;
  const error = !fresh && item.http_status_error
    ? `<br><span class="muted">${esc(item.http_status_error)}</span>`
    : "";
  return `<b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span><br>${statusLine}${error}`;
}
function renderWifiCell(item) {
  const fresh = statusIsFresh(item);
  const stateClass = item.wifi_connected ? "ok" : "bad";
  const stateText = item.wifi_connected ? "connected" : "offline";
  const prefix = fresh ? "" : "last: ";
  return `<span class="${fresh ? stateClass : "warn"}">${prefix}${stateText}</span><br>RSSI ${esc(item.wifi_connected_rssi)} dBm<br>disc ${esc(item.wifi_disconnect_count)}`;
}
function terminalMatchesModule(term, log) {
  return term.modules === "all" || term.modules.includes(Number(log.module_id));
}

function createTerminal(el) {
  const id = el.dataset.terminal;
  const modules = el.dataset.modules === "all" ? "all" : el.dataset.modules.split(",").map(Number);
  const title = modules === "all" ? "All Modules" : `Module ${modules.join(", ")}`;
  el.innerHTML = `
    <div class="term-head">
      <div class="term-title-wrap">
        <div class="term-title">${title}</div>
        <div class="term-meta">waiting for logs...</div>
      </div>
      <div class="filters">
        <select class="level-filter">
          <option value="all">All levels</option><option value="I">Info</option><option value="W">Warn</option><option value="E">Error</option><option value="D">Debug</option>
        </select>
        <select class="component-filter">
          <option value="all">All components</option>
          <option value="uwb">UWB</option>
          <option value="gps">GPS</option>
          <option value="accelerometer">Accelerometer</option>
          <option value="system">System</option>
        </select>
        ${modules === "all" ? `<select class="module-filter"><option value="all">All modules</option><option value="1">Module 1</option><option value="2">Module 2</option><option value="3">Module 3</option><option value="4">Module 4</option><option value="5">Module 5</option></select>` : ""}
        <button class="clear-filter">Clear Filter</button>
        <button class="clear-view">Clear View</button>
      </div>
    </div>
    <div class="term-body"></div>`;
  const level = el.querySelector(".level-filter");
  const component = el.querySelector(".component-filter");
  const moduleFilter = el.querySelector(".module-filter");
  level.value = localStorage.getItem(storageKey(id, "level")) || "all";
  component.value = localStorage.getItem(storageKey(id, "component")) || "all";
  if (moduleFilter) moduleFilter.value = localStorage.getItem(storageKey(id, "module")) || "all";
  const terminal = {
    id,
    el,
    modules,
    body: el.querySelector(".term-body"),
    meta: el.querySelector(".term-meta"),
    level,
    component,
    moduleFilter,
    clearedBefore: 0,
    follow: true,
  };
  terminal.body.addEventListener("scroll", () => {
    terminal.follow = terminal.body.scrollTop + terminal.body.clientHeight >= terminal.body.scrollHeight - 40;
  });
  level.addEventListener("change", () => {
    localStorage.setItem(storageKey(id, "level"), level.value);
    terminal.follow = true;
    renderTerminal(terminal);
  });
  component.addEventListener("change", () => {
    localStorage.setItem(storageKey(id, "component"), component.value);
    terminal.follow = true;
    renderTerminal(terminal);
  });
  if (moduleFilter) moduleFilter.addEventListener("change", () => {
    localStorage.setItem(storageKey(id, "module"), moduleFilter.value);
    terminal.follow = true;
    renderTerminal(terminal);
  });
  el.querySelector(".clear-filter").addEventListener("click", () => {
    level.value = "all";
    localStorage.setItem(storageKey(id, "level"), "all");
    component.value = "all";
    localStorage.setItem(storageKey(id, "component"), "all");
    if (moduleFilter) {
      moduleFilter.value = "all";
      localStorage.setItem(storageKey(id, "module"), "all");
    }
    terminal.follow = true;
    renderTerminal(terminal);
  });
  el.querySelector(".clear-view").addEventListener("click", () => {
    terminal.clearedBefore = state.lastId;
    terminal.follow = true;
    renderTerminal(terminal);
  });
  state.terminals.set(id, terminal);
}

function logAllowed(term, log) {
  if (log.id <= term.clearedBefore) return false;
  if (!terminalMatchesModule(term, log)) return false;
  if (term.modules === "all" && term.moduleFilter && term.moduleFilter.value !== "all" && String(log.module_id) !== term.moduleFilter.value) return false;
  if (term.level.value !== "all" && log.level !== term.level.value) return false;
  if (term.component.value !== "all" && log.component !== term.component.value) return false;
  return true;
}

function formatLog(log) {
  const ms = log.uptime_ms === null ? "          -" : String(log.uptime_ms).padStart(10, " ");
  const module = log.module_id ? `M${log.module_id}` : "?";
  return `<div class="log-line ${levelClass(log.level)}"><span class="muted">#${log.id}</span> <span class="module-chip">${module}</span> <span class="muted">[${ms} ms]</span> <b>[${esc(log.level)}]</b><span class="tag-chip">[${esc(log.tag)}]</span> ${esc(log.message)}</div>`;
}

function scrollTerminalToBottom(term) {
  term.follow = true;
  term.body.scrollTop = term.body.scrollHeight;
  requestAnimationFrame(() => {
    term.body.scrollTop = term.body.scrollHeight;
    term.follow = true;
  });
}

function renderTerminal(term) {
  const shouldFollow = term.follow || term.body.scrollHeight <= term.body.clientHeight + 40;
  const latest = [...state.logs].reverse().find(log => terminalMatchesModule(term, log));
  term.meta.textContent = latest ? `last log ${fmtAge(latest.received_at)} · #${latest.id}` : "no logs yet";
  const items = state.logs.filter(log => logAllowed(term, log)).slice(-1000);
  term.body.innerHTML = items.map(formatLog).join("");
  if (shouldFollow) {
    scrollTerminalToBottom(term);
  }
}

function renderAllTerminals() {
  for (const term of state.terminals.values()) renderTerminal(term);
}

function renderVisibleTerminals() {
  document.querySelectorAll(".page.active .terminal").forEach(el => {
    const term = state.terminals.get(el.dataset.terminal);
    if (term) renderTerminal(term);
  });
}

function accelSamples(moduleId) {
  return state.accelHistory[String(moduleId)] || state.accelHistory[moduleId] || [];
}

function mergeAccelSample(sample) {
  const moduleId = Number(sample.module_id);
  if (!moduleId) return;
  const key = String(moduleId);
  const seenKey = sample.log_id ? `${key}:${sample.log_id}` : "";
  if (seenKey && state.accelSeen.has(seenKey)) return;
  if (seenKey) state.accelSeen.add(seenKey);
  if (!state.accelHistory[key]) state.accelHistory[key] = [];
  const history = state.accelHistory[key];
  const previous = history[history.length - 1];
  sample.uptime_ms = Number(sample.uptime_ms);
  sample.received_at = Number(sample.received_at);
  history.push(sample);
  if (previous && Number(previous.uptime_ms || 0) > Number(sample.uptime_ms || 0)) {
    history.sort((a, b) => Number(a.uptime_ms || 0) - Number(b.uptime_ms || 0));
  }
  if (history.length > maxAccelSamples) {
    history.splice(0, history.length - maxAccelSamples);
  }
}

function ingestAccelLogs(logs) {
  for (const log of logs) {
    if (!log.module_id) continue;
    const match = accelLineRe.exec(String(log.message || ""));
    if (!match) continue;
    mergeAccelSample({
      module_id: Number(log.module_id),
      received_at: log.received_at,
      uptime_ms: log.uptime_ms,
      log_id: log.id,
      x: Number(match[1]),
      y: Number(match[2]),
      z: Number(match[3]),
      accuracy: Number(match[4]),
      reports: Number(match[5]),
    });
  }
}

function mergeAccelHistory(historyByModule) {
  for (const samples of Object.values(historyByModule || {})) {
    for (const sample of samples || []) {
      mergeAccelSample(sample);
    }
  }
}

function accelMagnitude(sample) {
  if (!sample) return null;
  return Math.sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
}

function fmtAccel(value) {
  return Number.isFinite(value) ? value.toFixed(2) : "-";
}

function fmtAxis(value) {
  return Number.isFinite(value) ? String(Math.round(value)) : "-";
}

function intervalMsToHz(ms) {
  const value = Number(ms);
  if (!Number.isFinite(value) || value <= 0) return "";
  return String(Math.max(1, Math.min(500, Math.round(1000 / value))));
}

function accelRate(samples, horizonSec = 1.0) {
  if (samples.length < 2) return null;
  const latest = samples[samples.length - 1];
  const latestUptimeMs = Number(latest.uptime_ms);
  if (!Number.isFinite(latestUptimeMs)) return null;
  const cutoffMs = latestUptimeMs - horizonSec * 1000;
  let first = null;
  let count = 0;
  for (let index = samples.length - 1; index >= 0; index--) {
    const sample = samples[index];
    const uptimeMs = Number(sample.uptime_ms);
    if (!Number.isFinite(uptimeMs) || uptimeMs > latestUptimeMs) continue;
    if (uptimeMs < cutoffMs) break;
    first = sample;
    count++;
  }
  if (!first || first === latest || count < 2) return null;
  const dtSec = (latestUptimeMs - Number(first.uptime_ms)) / 1000;
  if (!Number.isFinite(dtSec) || dtSec <= 0) return null;
  const repDelta = Number(latest.reports) - Number(first.reports);
  return {
    rxHz: (count - 1) / dtSec,
    repHz: Number.isFinite(repDelta) && repDelta >= 0 ? repDelta / dtSec : null,
  };
}

function visibleAccelSamples(samples, latest, windowSec) {
  if (!latest) return [];
  const latestUptimeMs = Number(latest.uptime_ms);
  if (!Number.isFinite(latestUptimeMs)) return [];
  const windowMs = windowSec * 1000;
  return samples.filter(sample => {
    const uptimeMs = Number(sample.uptime_ms);
    return Number.isFinite(uptimeMs) &&
      uptimeMs <= latestUptimeMs &&
      latestUptimeMs - uptimeMs <= windowMs;
  });
}

function accelScale(samples) {
  if (!samples.length) return {min: -12, max: 12};
  const values = samples.flatMap(sample => [Number(sample.x), Number(sample.y), Number(sample.z)])
    .filter(Number.isFinite);
  if (!values.length) return {min: -12, max: 12};
  let min = Math.min(0, ...values);
  let max = Math.max(0, ...values);
  const span = Math.max(1, max - min);
  const pad = Math.max(1, Math.ceil(span * 0.08));
  min = Math.floor(min - pad);
  max = Math.ceil(max + pad);
  if (max - min < 2) {
    min -= 1;
    max += 1;
  }
  return {min, max};
}

function downsampleSeries(samples, key) {
  if (samples.length <= maxSeriesPoints) return samples;

  const bucketSize = Math.max(1, Math.ceil(samples.length / (maxSeriesPoints / 2)));
  const result = [];
  for (let start = 0; start < samples.length; start += bucketSize) {
    const bucket = samples.slice(start, start + bucketSize);
    let minSample = bucket[0];
    let maxSample = bucket[0];
    for (const sample of bucket) {
      if (Number(sample[key]) < Number(minSample[key])) minSample = sample;
      if (Number(sample[key]) > Number(maxSample[key])) maxSample = sample;
    }
    if (minSample === maxSample) {
      result.push(minSample);
    } else if (Number(minSample.uptime_ms) <= Number(maxSample.uptime_ms)) {
      result.push(minSample, maxSample);
    } else {
      result.push(maxSample, minSample);
    }
  }
  return result;
}

function ensureAccelCharts() {
  const charts = document.getElementById("accelCharts");
  const latestRows = document.getElementById("accelLatestRows");
  if (!charts || !latestRows) return;
  if (state.chartsReady) return;

  charts.innerHTML = [1, 2, 3, 4, 5].map(moduleId => `
    <div class="accel-chart">
      <div class="chart-label">
        <b>Module ${moduleId}</b>
        <span id="chartAge${moduleId}">waiting</span>
      </div>
      <div class="chart-wrap">
        <canvas class="accel-canvas" id="accelCanvas${moduleId}"></canvas>
      </div>
    </div>`).join("");

  latestRows.innerHTML = [1, 2, 3, 4, 5].map(moduleId => `
    <div class="latest-row">
      <h3>Module ${moduleId}</h3>
      <div class="axis-values">
        <div><span>X</span><b id="latestX${moduleId}">-</b></div>
        <div><span>Y</span><b id="latestY${moduleId}">-</b></div>
        <div><span>Z</span><b id="latestZ${moduleId}">-</b></div>
      </div>
      <div class="latest-extra" id="latestExtra${moduleId}">no samples yet</div>
    </div>`).join("");

  state.chartsReady = true;
}

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * dpr));
  const height = Math.max(1, Math.floor(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return {ctx, width: rect.width, height: rect.height};
}

function parseIdList(text, expected = null) {
  const ids = String(text || "")
    .split(/[,\s]+/)
    .map(value => Number(value.trim()))
    .filter(value => Number.isInteger(value) && value > 0);
  const unique = [];
  for (const id of ids) {
    if (!unique.includes(id)) unique.push(id);
  }
  return expected ? unique.slice(0, expected) : unique;
}

function positionSettings() {
  const anchorCount = Math.max(3, Math.min(4, Number(document.getElementById("positionAnchorCount")?.value || 4)));
  const solver = document.getElementById("positionSolver")?.value === "tdoa" ? "tdoa" : "ranging";
  const anchorIds = parseIdList(document.getElementById("positionAnchors")?.value, anchorCount);
  const tagIds = parseIdList(document.getElementById("positionTags")?.value);
  const maxAge = Math.max(0.2, Number(document.getElementById("positionMaxAgeSec")?.value || 3));
  const rawTdoaMode = document.getElementById("positionTdoaMode")?.value || "static";
  const tdoaMode = ["static", "dynamic", "auto"].includes(rawTdoaMode)
    ? rawTdoaMode
    : "static";
  return {anchorCount, solver, anchorIds, tagIds, maxAge, tdoaMode};
}

function selectedPositionModuleIds(settings = positionSettings()) {
  const ids = [...settings.tagIds.slice(0, 1), ...settings.anchorIds];
  return [...new Set(ids)].filter(Boolean);
}

function statusForModule(moduleId) {
  return state.statuses.find(item => Number(item.module_id) === Number(moduleId));
}

function moduleHttpOnline(item) {
  return Boolean(item?.http_status_online);
}

function moduleInPositionRuntime(item, solver) {
  if (!item || !moduleHttpOnline(item)) return false;
  const mode = String(item.runtime_mode_name || item.runtime_mode || "").toLowerCase();
  const wanted = solver === "tdoa" ? "flex_tdoa" : "ranging";
  return Boolean(item.runtime_uwb_enabled) && mode.includes(wanted);
}

function positionRangingActive(settings = positionSettings()) {
  const ids = selectedPositionModuleIds(settings);
  if (!ids.length) return false;
  return ids.every(id => moduleInPositionRuntime(statusForModule(id), settings.solver));
}

function solve2x2(a00, a01, a10, a11, b0, b1) {
  const det = a00 * a11 - a01 * a10;
  if (Math.abs(det) < 1e-9) return null;
  return {
    x: (b0 * a11 - a01 * b1) / det,
    y: (a00 * b1 - b0 * a10) / det,
  };
}

function inverse2x2(a00, a01, a10, a11) {
  const det = a00 * a11 - a01 * a10;
  if (Math.abs(det) < 1e-9) return null;
  return {
    a00: a11 / det,
    a01: -a01 / det,
    a10: -a10 / det,
    a11: a00 / det,
  };
}

function solveLinearSystem(matrix, rhs) {
  const n = rhs.length;
  const a = matrix.map((row, index) => [...row, rhs[index]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    }
    if (Math.abs(a[pivot][col]) < 1e-10) return null;
    if (pivot !== col) [a[pivot], a[col]] = [a[col], a[pivot]];
    const div = a[col][col];
    for (let j = col; j <= n; j++) a[col][j] /= div;
    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const factor = a[row][col];
      for (let j = col; j <= n; j++) a[row][j] -= factor * a[col][j];
    }
  }
  return a.map(row => row[n]);
}

function trilaterate(anchors, distances) {
  const usable = Object.entries(distances)
    .map(([anchorId, distance]) => [Number(anchorId), anchors[Number(anchorId)], Number(distance)])
    .filter(([, anchor, distance]) => anchor && Number.isFinite(distance) && distance > 0)
    .sort((a, b) => a[0] - b[0]);
  if (usable.length < 3) return null;

  const [, base, r0] = usable[0];
  let nxx = 0;
  let nxy = 0;
  let nyy = 0;
  let rhsX = 0;
  let rhsY = 0;
  for (const [, anchor, distance] of usable.slice(1)) {
    const ax = 2 * (anchor.x - base.x);
    const ay = 2 * (anchor.y - base.y);
    const b = r0 * r0 - distance * distance +
      anchor.x * anchor.x - base.x * base.x +
      anchor.y * anchor.y - base.y * base.y;
    nxx += ax * ax;
    nxy += ax * ay;
    nyy += ay * ay;
    rhsX += ax * b;
    rhsY += ay * b;
  }
  return solve2x2(nxx, nxy, nxy, nyy, rhsX, rhsY);
}

function positionResiduals(position, anchors, distances) {
  const residuals = {};
  if (!position) return residuals;
  for (const [anchorId, distance] of Object.entries(distances)) {
    const anchor = anchors[Number(anchorId)];
    if (!anchor) continue;
    residuals[anchorId] = Math.hypot(position.x - anchor.x, position.y - anchor.y) - Number(distance);
  }
  return residuals;
}

function freshDistanceFor(tagId, anchorId, maxAge) {
  const item = state.ranging?.distances?.[`${tagId}:${anchorId}`];
  if (!item || Number(item.age_sec) > maxAge) return null;
  return item;
}

function anchorPairKey(a, b) {
  const aa = Number(a);
  const bb = Number(b);
  return aa < bb ? `${aa}:${bb}` : `${bb}:${aa}`;
}

function freshAnchorPairDistance(a, b, maxAge) {
  const item = state.tdoa?.anchor_distances?.[anchorPairKey(a, b)];
  if (!item || Number(item.age_sec) > maxAge) return null;
  const median = Number(item.stats?.median_m);
  if (Number.isFinite(median) && median > 0) {
    return {...item, latest_distance_m: Number(item.distance_m), distance_m: median};
  }
  return item;
}

function selectedAnchorPairs(anchorIds) {
  const pairs = [];
  for (let i = 0; i < anchorIds.length; i++) {
    for (let j = i + 1; j < anchorIds.length; j++) {
      pairs.push([Number(anchorIds[i]), Number(anchorIds[j])]);
    }
  }
  return pairs;
}

function anchorGeometryResiduals(anchors, distanceItems) {
  const residuals = {};
  for (const [key, item] of Object.entries(distanceItems || {})) {
    const a = anchors[Number(item.anchor_a_id)];
    const b = anchors[Number(item.anchor_b_id)];
    const distance = Number(item.distance_m);
    if (!a || !b || !Number.isFinite(distance)) continue;
    residuals[key] = Math.hypot(a.x - b.x, a.y - b.y) - distance;
  }
  return residuals;
}

function refineMeasuredAnchorGeometry(anchorIds, anchors, distanceItems) {
  const ids = anchorIds.map(Number);
  const variables = [];
  if (anchors[ids[1]]) {
    variables.push({id: ids[1], axis: "x"});
  }
  for (const id of ids.slice(2)) {
    if (anchors[id]) {
      variables.push({id, axis: "x"});
      variables.push({id, axis: "y"});
    }
  }
  if (!variables.length) return anchors;

  const indexFor = new Map(
    variables.map((variable, index) => [`${variable.id}:${variable.axis}`, index])
  );
  for (let iter = 0; iter < 20; iter++) {
    const n = variables.length;
    const normal = Array.from({length: n}, () => Array(n).fill(0));
    const rhs = Array(n).fill(0);
    let used = 0;

    for (const item of Object.values(distanceItems || {})) {
      const a = anchors[Number(item.anchor_a_id)];
      const b = anchors[Number(item.anchor_b_id)];
      const measured = Number(item.distance_m);
      if (!a || !b || !Number.isFinite(measured) || measured <= 0) continue;
      const dx = a.x - b.x;
      const dy = a.y - b.y;
      const predicted = Math.max(1e-6, Math.hypot(dx, dy));
      const residual = predicted - measured;
      const gradient = Array(n).fill(0);

      const ax = indexFor.get(`${item.anchor_a_id}:x`);
      const ay = indexFor.get(`${item.anchor_a_id}:y`);
      const bx = indexFor.get(`${item.anchor_b_id}:x`);
      const by = indexFor.get(`${item.anchor_b_id}:y`);
      if (ax !== undefined) gradient[ax] = dx / predicted;
      if (ay !== undefined) gradient[ay] = dy / predicted;
      if (bx !== undefined) gradient[bx] = -dx / predicted;
      if (by !== undefined) gradient[by] = -dy / predicted;

      for (let r = 0; r < n; r++) {
        rhs[r] += -gradient[r] * residual;
        for (let c = 0; c < n; c++) normal[r][c] += gradient[r] * gradient[c];
      }
      used++;
    }

    if (used < variables.length) break;
    for (let i = 0; i < n; i++) normal[i][i] += 1e-6;
    const step = solveLinearSystem(normal, rhs);
    if (!step) break;

    let stepNorm = 0;
    for (let i = 0; i < variables.length; i++) {
      const variable = variables[i];
      const delta = Math.max(-0.25, Math.min(0.25, Number(step[i]) || 0));
      anchors[variable.id][variable.axis] += delta;
      stepNorm += delta * delta;
    }
    if (Math.sqrt(stepNorm) < 0.0005) break;
  }
  return anchors;
}

function measuredAnchorGeometry(anchorIds, maxAge) {
  const ids = anchorIds.map(Number).filter(id => Number.isInteger(id) && id > 0);
  const distanceItems = {};
  const missingPairs = [];
  for (const [a, b] of selectedAnchorPairs(ids)) {
    const item = freshAnchorPairDistance(a, b, maxAge);
    if (item) distanceItems[anchorPairKey(a, b)] = item;
    else missingPairs.push([a, b]);
  }

  const result = {
    anchors: {},
    distanceItems,
    missingPairs,
    residuals: {},
    complete: false,
    status: "waiting",
  };
  if (ids.length < 3) return result;

  const distance = (a, b) => {
    const item = distanceItems[anchorPairKey(a, b)];
    return item ? Number(item.distance_m) : NaN;
  };
  const d01 = distance(ids[0], ids[1]);
  const d02 = distance(ids[0], ids[2]);
  const d12 = distance(ids[1], ids[2]);
  if (![d01, d02, d12].every(value => Number.isFinite(value) && value > 0)) {
    return result;
  }

  const p0 = {x: 0, y: 0};
  const p1 = {x: d01, y: 0};
  const x2 = (d02 * d02 + d01 * d01 - d12 * d12) / (2 * d01);
  const y2sq = d02 * d02 - x2 * x2;
  const p2 = {x: x2, y: Math.sqrt(Math.max(0, y2sq))};
  result.anchors[ids[0]] = p0;
  result.anchors[ids[1]] = p1;
  result.anchors[ids[2]] = p2;
  result.complete = missingPairs.length === 0 && y2sq >= -0.02;
  result.status = result.complete ? "ok" : "inconsistent";

  if (ids.length >= 4) {
    const d03 = distance(ids[0], ids[3]);
    const d13 = distance(ids[1], ids[3]);
    const d23 = distance(ids[2], ids[3]);
    if ([d03, d13, d23].every(value => Number.isFinite(value) && value > 0)) {
      const x3 = (d03 * d03 + d01 * d01 - d13 * d13) / (2 * d01);
      const y3sq = d03 * d03 - x3 * x3;
      const y3abs = Math.sqrt(Math.max(0, y3sq));
      const candidates = [{x: x3, y: y3abs}, {x: x3, y: -y3abs}];
      candidates.sort((left, right) =>
        Math.abs(Math.hypot(left.x - p2.x, left.y - p2.y) - d23) -
        Math.abs(Math.hypot(right.x - p2.x, right.y - p2.y) - d23));
      result.anchors[ids[3]] = candidates[0];
      result.complete = result.complete && y3sq >= -0.02;
      result.status = result.complete ? "ok" : "inconsistent";
    } else {
      result.complete = false;
      result.status = "waiting";
    }
  }

  refineMeasuredAnchorGeometry(ids, result.anchors, distanceItems);
  result.residuals = anchorGeometryResiduals(result.anchors, distanceItems);
  return result;
}

function freshTdoaObservations(tagId, anchorIds, maxAge) {
  const selected = new Set(anchorIds.map(Number));
  return Object.values(state.tdoa?.observations || {})
    .filter(item =>
      Number(item.tag_id) === Number(tagId) &&
      selected.has(Number(item.initiator_id)) &&
      selected.has(Number(item.responder_id)) &&
      Number(item.age_sec) <= maxAge &&
      Number.isFinite(Number(item.diff_m)))
    .sort((a, b) =>
      Number(a.initiator_id) - Number(b.initiator_id) ||
      Number(a.responder_id) - Number(b.responder_id));
}

function sequenceForwardDelta(from, to) {
  const start = Number(from);
  const end = Number(to);
  if (!Number.isFinite(start) || !Number.isFinite(end)) return NaN;
  return ((Math.trunc(end) - Math.trunc(start)) + 65536) % 65536;
}

function tdoaSequencesArePaired(left, right, pairCount) {
  const forward = sequenceForwardDelta(left?.seq, right?.seq);
  const reverse = sequenceForwardDelta(right?.seq, left?.seq);
  const nearest = Math.min(forward, reverse);
  return nearest > 0 && nearest <= pairCount;
}

function tdoaModeUsesDynamicObservations(mode) {
  return mode === "dynamic" || mode === "auto";
}

function tdoaObservationForMode(item, mode) {
  if (!tdoaModeUsesDynamicObservations(mode)) {
    return {...item, tdoa_filter: "static"};
  }
  const dynamicDiff = Number(item.dynamic_diff_m);
  const dynamicRawDiff = Number(item.dynamic_raw_diff_m);
  const dynamicReverseSum = Number(item.dynamic_reverse_sum_m);
  const latestDiff = Number(item.latest_diff_m);
  const latestRawDiff = Number(item.latest_raw_diff_m);
  const latestReverseSum = Number(item.latest_reverse_sum_m);
  return {
    ...item,
    diff_m: Number.isFinite(dynamicDiff)
      ? dynamicDiff
      : (Number.isFinite(latestDiff) ? latestDiff : item.diff_m),
    raw_diff_m: Number.isFinite(dynamicRawDiff)
      ? dynamicRawDiff
      : (Number.isFinite(latestRawDiff) ? latestRawDiff : item.raw_diff_m),
    reverse_sum_m: Number.isFinite(dynamicReverseSum)
      ? dynamicReverseSum
      : (Number.isFinite(latestReverseSum)
        ? latestReverseSum
        : item.reverse_sum_m),
    tdoa_filter: mode === "auto" ? "auto" : "dynamic",
  };
}

function pairedTdoaObservations(tagId, anchorIds, maxAge, mode = "static") {
  const selected = new Set(anchorIds.map(Number));
  const serverPaired = Object.values(state.tdoa?.paired_observations || {})
    .filter(item =>
      Number(item.tag_id) === Number(tagId) &&
      selected.has(Number(item.initiator_id)) &&
      selected.has(Number(item.responder_id)) &&
      Number(item.age_sec) <= maxAge &&
      Number.isFinite(Number(item.diff_m)))
    .sort((left, right) =>
      Number(left.initiator_id) - Number(right.initiator_id) ||
      Number(left.responder_id) - Number(right.responder_id));
  if (serverPaired.length) {
    return serverPaired.map(item => tdoaObservationForMode(item, mode));
  }

  const fresh = freshTdoaObservations(tagId, anchorIds, maxAge);
  const byDirection = new Map(
    fresh.map(item => [`${item.initiator_id}-${item.responder_id}`, item])
  );
  const pairs = selectedAnchorPairs(anchorIds);
  const pairCount = pairs.length;
  const paired = [];

  for (const [a, b] of pairs) {
    const ab = byDirection.get(`${a}-${b}`);
    const ba = byDirection.get(`${b}-${a}`);
    if (!ab || !ba || !tdoaSequencesArePaired(ab, ba, pairCount)) continue;

    const abDiff = Number(ab.diff_m);
    const baDiff = Number(ba.diff_m);
    if (!Number.isFinite(abDiff) || !Number.isFinite(baDiff)) continue;

    const rawAbDiff = Number(ab.raw_diff_m);
    const rawBaDiff = Number(ba.raw_diff_m);
    const age = Math.max(Number(ab.age_sec) || 0, Number(ba.age_sec) || 0);
    const reverseSum = abDiff + baDiff;
    paired.push({
      ...ab,
      initiator_id: a,
      responder_id: b,
      diff_m: (abDiff - baDiff) / 2,
      latest_diff_m: (abDiff - baDiff) / 2,
      raw_diff_m: Number.isFinite(rawAbDiff) && Number.isFinite(rawBaDiff)
        ? (rawAbDiff - rawBaDiff) / 2
        : Number(ab.raw_diff_m),
      latest_raw_diff_m: Number.isFinite(rawAbDiff) && Number.isFinite(rawBaDiff)
        ? (rawAbDiff - rawBaDiff) / 2
        : Number(ab.raw_diff_m),
      age_sec: age,
      paired: true,
      reverse_seq: ba.seq,
      reverse_age_sec: ba.age_sec,
      reverse_diff_m: baDiff,
      reverse_sum_m: reverseSum,
      latest_reverse_sum_m: reverseSum,
      seq_gap: Math.min(
        sequenceForwardDelta(ab.seq, ba.seq),
        sequenceForwardDelta(ba.seq, ab.seq)
      ),
    });
  }

  return paired.map(item => tdoaObservationForMode(item, mode)).sort((left, right) =>
    Number(left.initiator_id) - Number(right.initiator_id) ||
    Number(left.responder_id) - Number(right.responder_id));
}

function solveTdoa(anchors, observations) {
  const usable = observations
    .map(item => ({
      item,
      initiator: anchors[Number(item.initiator_id)],
      responder: anchors[Number(item.responder_id)],
      diff: Number(item.diff_m),
      weight: Number.isFinite(Number(item.solve_weight))
        ? Math.max(0.05, Number(item.solve_weight))
        : 1,
    }))
    .filter(entry => entry.initiator && entry.responder && Number.isFinite(entry.diff));
  if (usable.length < 2) return null;

  const anchorValues = Object.values(anchors);
  let x = anchorValues.reduce((sum, anchor) => sum + anchor.x, 0) / anchorValues.length;
  let y = anchorValues.reduce((sum, anchor) => sum + anchor.y, 0) / anchorValues.length;

  for (let iter = 0; iter < 24; iter++) {
    let nxx = 1e-6;
    let nxy = 0;
    let nyy = 1e-6;
    let rhsX = 0;
    let rhsY = 0;
    let used = 0;
    for (const entry of usable) {
      const ai = entry.initiator;
      const ar = entry.responder;
      const di = Math.max(1e-6, Math.hypot(x - ai.x, y - ai.y));
      const dr = Math.max(1e-6, Math.hypot(x - ar.x, y - ar.y));
      const residual = (dr - di) - entry.diff;
      const gx = (x - ar.x) / dr - (x - ai.x) / di;
      const gy = (y - ar.y) / dr - (y - ai.y) / di;
      const weight = entry.weight;
      nxx += weight * gx * gx;
      nxy += weight * gx * gy;
      nyy += weight * gy * gy;
      rhsX += -weight * gx * residual;
      rhsY += -weight * gy * residual;
      used++;
    }
    if (used < 2) return null;
    const step = solve2x2(nxx, nxy, nxy, nyy, rhsX, rhsY);
    if (!step || !Number.isFinite(step.x) || !Number.isFinite(step.y)) return null;
    const limit = 0.5;
    const dx = Math.max(-limit, Math.min(limit, step.x));
    const dy = Math.max(-limit, Math.min(limit, step.y));
    x += dx;
    y += dy;
    if (Math.hypot(dx, dy) < 0.0005) break;
  }

  return {x, y};
}

function tdoaResiduals(position, anchors, observations) {
  const residuals = {};
  if (!position) return residuals;
  for (const item of observations) {
    const initiator = anchors[Number(item.initiator_id)];
    const responder = anchors[Number(item.responder_id)];
    if (!initiator || !responder) continue;
    const key = `${item.initiator_id}-${item.responder_id}`;
    residuals[key] =
      Math.hypot(position.x - responder.x, position.y - responder.y) -
      Math.hypot(position.x - initiator.x, position.y - initiator.y) -
      Number(item.diff_m);
  }
  return residuals;
}

function tdoaObservationKey(item) {
  return `${Number(item.initiator_id)}-${Number(item.responder_id)}`;
}

function tdoaRmsForResiduals(residuals, observations) {
  let weightedSum = 0;
  let weightSum = 0;
  for (const item of observations || []) {
    const value = Number(residuals?.[tdoaObservationKey(item)]);
    if (!Number.isFinite(value)) continue;
    const weight = Number.isFinite(Number(item.solve_weight))
      ? Math.max(0.05, Number(item.solve_weight))
      : 1;
    weightedSum += weight * value * value;
    weightSum += weight;
  }
  if (weightSum <= 0) return Infinity;
  return Math.sqrt(weightedSum / weightSum);
}

function tdoaSolveWeight(item, maxAge, mode) {
  if (!tdoaModeUsesDynamicObservations(mode)) return 1;
  const age = Math.max(0, Number(item.age_sec) || 0);
  const horizon = Math.max(0.25, Number(maxAge) || 1);
  const tau = Math.max(0.2, Math.min(0.7, horizon / 2));
  const ageWeight = Math.exp(-age / tau);
  const span = Math.max(
    0,
    Number(item.dynamic_span_sec ?? item.history_span_sec) || 0
  );
  const spanWeight = 1 / (1 + Math.max(0, span - 0.15) / 0.5);
  return Math.max(0.1, Math.min(1, ageWeight * spanWeight));
}

function minTdoaObservationCount(anchorIds) {
  return Math.max(2, Math.min(3, Number(anchorIds?.length || 0) - 1));
}

function robustTdoaFit(anchorIds, anchors, observations, options = {}) {
  const strictKalmanInput = options.tdoaMode === "auto";
  const all = (observations || []).filter(item =>
    anchors[Number(item.initiator_id)] &&
    anchors[Number(item.responder_id)] &&
    Number.isFinite(Number(item.diff_m)))
    .map(item => ({
      ...item,
      solve_weight: tdoaSolveWeight(
        item,
        options.maxAge,
        options.tdoaMode || "static"
      ),
    }));
  const minCount = strictKalmanInput && Number(anchorIds?.length || 0) >= 4
    ? 4
    : minTdoaObservationCount(anchorIds);
  const notes = new Map();
  if (all.length < minCount) {
    return {
      position: null,
      used: [],
      annotated: all.map(item => ({...item, used_in_fit: false, reject_reason: "need more"})),
      notes,
    };
  }

  let candidates = all.filter(item => {
    const ok = !item.suspect;
    if (!ok) notes.set(tdoaObservationKey(item), "suspect");
    return ok;
  });
  if (candidates.length < minCount) {
    candidates = all;
    notes.clear();
  }

  let used = candidates.filter(item => {
    const reverseSum = Number(item.reverse_sum_m);
    const ok = !Number.isFinite(reverseSum) || Math.abs(reverseSum) <= tdoaReverseSumRejectM;
    if (!ok) notes.set(tdoaObservationKey(item), "rev sum");
    return ok;
  });
  if (used.length < minCount) {
    if (strictKalmanInput) {
      return {
        position: null,
        used: [],
        annotated: all.map(item => ({
          ...item,
          used_in_fit: false,
          reject_reason: notes.get(tdoaObservationKey(item)) || "need clean obs",
        })),
        notes,
      };
    }
    used = candidates;
    for (const item of candidates) {
      const key = tdoaObservationKey(item);
      if (notes.get(key) === "rev sum") notes.delete(key);
    }
  }

  let position = solveTdoa(anchors, used);
  if (!position) {
    return {
      position: null,
      used: [],
      annotated: all.map(item => ({...item, used_in_fit: false, reject_reason: notes.get(tdoaObservationKey(item)) || "fit fail"})),
      notes,
    };
  }
  let residuals = tdoaResiduals(position, anchors, used);
  let rms = tdoaRmsForResiduals(residuals, used);

  for (let iter = 0; iter < 2 && used.length > minCount; iter++) {
    let worst = null;
    let worstAbs = 0;
    for (const item of used) {
      const residual = Number(residuals[tdoaObservationKey(item)]);
      const absResidual = Math.abs(residual);
      if (Number.isFinite(absResidual) && absResidual > worstAbs) {
        worst = item;
        worstAbs = absResidual;
      }
    }
    if (!worst || worstAbs <= tdoaResidualRejectM) break;

    const trialUsed = used.filter(item => item !== worst);
    const trialPosition = solveTdoa(anchors, trialUsed);
    if (!trialPosition) break;
    const trialResiduals = tdoaResiduals(trialPosition, anchors, trialUsed);
    const trialRms = tdoaRmsForResiduals(trialResiduals, trialUsed);
    if (!(trialRms < rms * 0.85 || rms > tdoaResidualRejectM)) break;

    notes.set(tdoaObservationKey(worst), "resid");
    used = trialUsed;
    position = trialPosition;
    residuals = trialResiduals;
    rms = trialRms;
  }

  const usedKeys = new Set(used.map(tdoaObservationKey));
  const annotated = all.map(item => {
    const key = tdoaObservationKey(item);
    return {...item, used_in_fit: usedKeys.has(key), reject_reason: notes.get(key) || ""};
  });
  return {position, used, annotated, notes};
}

function positionAccuracyFromRows(rows) {
  const usable = rows.filter(row =>
    Number.isFinite(row.residual) &&
    Number.isFinite(row.gx) &&
    Number.isFinite(row.gy));
  if (!usable.length) return null;

  let sse = 0;
  let maxAbs = 0;
  let hxx = 0;
  let hxy = 0;
  let hyy = 0;
  let weightSum = 0;
  for (const row of usable) {
    const residual = Number(row.residual);
    const weight = Number.isFinite(Number(row.weight))
      ? Math.max(0.05, Number(row.weight))
      : 1;
    sse += weight * residual * residual;
    weightSum += weight;
    maxAbs = Math.max(maxAbs, Math.abs(residual));
    hxx += weight * row.gx * row.gx;
    hxy += weight * row.gx * row.gy;
    hyy += weight * row.gy * row.gy;
  }

  const count = usable.length;
  const dof = Math.max(1, count - 2);
  const rms = Math.sqrt(sse / Math.max(1e-6, weightSum));
  const variance = sse / dof;
  const inv = inverse2x2(hxx, hxy, hxy, hyy);
  let sigmaX = NaN;
  let sigmaY = NaN;
  let sigmaMajor = NaN;
  let gdop = NaN;
  if (inv && Number.isFinite(variance)) {
    const covXX = Math.max(0, variance * inv.a00);
    const covXY = variance * inv.a01;
    const covYY = Math.max(0, variance * inv.a11);
    const eigTerm = Math.sqrt(Math.max(0, (covXX - covYY) * (covXX - covYY) + 4 * covXY * covXY));
    sigmaX = Math.sqrt(covXX);
    sigmaY = Math.sqrt(covYY);
    sigmaMajor = Math.sqrt(Math.max(0, (covXX + covYY + eigTerm) / 2));
    gdop = Math.sqrt(Math.max(0, inv.a00 + inv.a11));
  }

  return {
    count,
    dof,
    rms_m: rms,
    max_abs_m: maxAbs,
    sigma_x_m: sigmaX,
    sigma_y_m: sigmaY,
    sigma_major_m: sigmaMajor,
    gdop,
  };
}

function tdoaPositionAccuracy(position, anchors, observations, residuals) {
  if (!position) return null;
  const rows = [];
  for (const item of observations || []) {
    const initiator = anchors[Number(item.initiator_id)];
    const responder = anchors[Number(item.responder_id)];
    if (!initiator || !responder) continue;
    const di = Math.max(1e-6, Math.hypot(position.x - initiator.x, position.y - initiator.y));
    const dr = Math.max(1e-6, Math.hypot(position.x - responder.x, position.y - responder.y));
    const key = `${item.initiator_id}-${item.responder_id}`;
    const residual = Number(residuals?.[key]);
    rows.push({
      residual,
      gx: (position.x - responder.x) / dr - (position.x - initiator.x) / di,
      gy: (position.y - responder.y) / dr - (position.y - initiator.y) / di,
      weight: item.solve_weight,
    });
  }
  return positionAccuracyFromRows(rows);
}

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function kalmanFilterKey(tagId, settings) {
  return [
    settings.solver,
    settings.tdoaMode,
    settings.anchorIds.join(","),
    tagId,
  ].join(":");
}

function resetPositionFilters() {
  state.positionFilters = {};
}

function initPositionFilter(position, accuracy, now) {
  const sigma = clamp(
    Math.max(
      0.08,
      Number(accuracy?.sigma_major_m) || 0,
      Number(accuracy?.rms_m) || 0
    ),
    0.08,
    0.6
  );
  const posVar = sigma * sigma;
  return {
    x: Number(position.x),
    y: Number(position.y),
    vx: 0,
    vy: 0,
    p: [
      posVar, 0, 0, 0,
      0, posVar, 0, 0,
      0, 0, 0.35, 0,
      0, 0, 0, 0.35,
    ],
    t: now,
    lastUpdate: now,
    accepted: 0,
    rejected: 0,
    movingScore: 0,
    mode: "static",
    lastInnovationM: 0,
    lastGate: "init",
  };
}

function kalmanPredict(filter, now) {
  const dt = clamp(now - Number(filter.t || now), 0.02, 1.5);
  const moving = Number(filter.movingScore || 0) > 0.45;
  const accelSigma = moving ? 1.15 : 0.22;
  const q = accelSigma * accelSigma;
  filter.x += filter.vx * dt;
  filter.y += filter.vy * dt;

  const p = filter.p.slice();
  const dt2 = dt * dt;
  const dt3 = dt2 * dt;
  const dt4 = dt2 * dt2;
  const qPos = q * dt4 / 4;
  const qCross = q * dt3 / 2;
  const qVel = q * dt2;

  filter.p = [
    p[0] + dt * (p[2] + p[8]) + dt2 * p[10] + qPos,
    p[1] + dt * (p[3] + p[9]) + dt2 * p[11],
    p[2] + dt * p[10] + qCross,
    p[3] + dt * p[11],

    p[4] + dt * (p[6] + p[12]) + dt2 * p[14],
    p[5] + dt * (p[7] + p[13]) + dt2 * p[15] + qPos,
    p[6] + dt * p[14],
    p[7] + dt * p[15] + qCross,

    p[8] + dt * p[10] + qCross,
    p[9] + dt * p[11],
    p[10] + qVel,
    p[11],

    p[12] + dt * p[14],
    p[13] + dt * p[15] + qCross,
    p[14],
    p[15] + qVel,
  ];
  filter.t = now;
  return dt;
}

function kalmanMeasurementSigma(accuracy, fitObservations) {
  const rms = Number(accuracy?.rms_m);
  const sigmaMajor = Number(accuracy?.sigma_major_m);
  const maxAbs = Number(accuracy?.max_abs_m);
  const fitCount = Number(fitObservations?.length || accuracy?.count || 0);
  let sigma = Math.max(
    0.05,
    Number.isFinite(rms) ? rms * 1.7 : 0,
    Number.isFinite(sigmaMajor) ? sigmaMajor : 0
  );
  if (fitCount <= 3) sigma *= 1.25;
  if (Number.isFinite(maxAbs) && maxAbs > 0.25) sigma *= 1.35;
  return clamp(sigma, 0.05, 0.9);
}

function kalmanUpdate2d(filter, measurement, accuracy, fitObservations, now) {
  const sigma = kalmanMeasurementSigma(accuracy, fitObservations);
  const r = sigma * sigma;
  const p = filter.p;
  const ix = Number(measurement.x) - filter.x;
  const iy = Number(measurement.y) - filter.y;
  const innovationM = Math.hypot(ix, iy);
  const s00 = p[0] + r;
  const s01 = p[1];
  const s10 = p[4];
  const s11 = p[5] + r;
  const det = s00 * s11 - s01 * s10;
  if (Math.abs(det) < 1e-12) {
    filter.lastGate = "singular";
    filter.rejected += 1;
    return false;
  }
  const inv00 = s11 / det;
  const inv01 = -s01 / det;
  const inv10 = -s10 / det;
  const inv11 = s00 / det;
  const d2 = ix * (inv00 * ix + inv01 * iy) + iy * (inv10 * ix + inv11 * iy);
  const rms = Number(accuracy?.rms_m);
  const maxAbs = Number(accuracy?.max_abs_m);
  const fitCount = Number(fitObservations?.length || accuracy?.count || 0);
  const qualityBad =
    fitCount < 3 ||
    (Number.isFinite(rms) && rms > 0.35) ||
    (Number.isFinite(maxAbs) && maxAbs > 0.75);
  const jumpBad =
    innovationM > 2.0 &&
    now - Number(filter.lastUpdate || 0) < 1.5 &&
    Math.hypot(filter.vx, filter.vy) < 1.0;
  if (qualityBad || d2 > 18 || jumpBad) {
    filter.lastGate = qualityBad ? "quality" : (jumpBad ? "jump" : "mahal");
    filter.lastInnovationM = innovationM;
    filter.rejected += 1;
    filter.movingScore = clamp(Number(filter.movingScore || 0) * 0.92, 0, 1);
    return false;
  }

  const k = [
    p[0] * inv00 + p[1] * inv10,
    p[0] * inv01 + p[1] * inv11,
    p[4] * inv00 + p[5] * inv10,
    p[4] * inv01 + p[5] * inv11,
    p[8] * inv00 + p[9] * inv10,
    p[8] * inv01 + p[9] * inv11,
    p[12] * inv00 + p[13] * inv10,
    p[12] * inv01 + p[13] * inv11,
  ];
  filter.x += k[0] * ix + k[1] * iy;
  filter.y += k[2] * ix + k[3] * iy;
  filter.vx += k[4] * ix + k[5] * iy;
  filter.vy += k[6] * ix + k[7] * iy;

  const hP0 = [p[0], p[1], p[2], p[3]];
  const hP1 = [p[4], p[5], p[6], p[7]];
  filter.p = p.map((value, index) => {
    const row = Math.floor(index / 4);
    const col = index % 4;
    return value - k[row * 2] * hP0[col] - k[row * 2 + 1] * hP1[col];
  });

  const speed = Math.hypot(filter.vx, filter.vy);
  const motionHit = speed > 0.18 || innovationM > 0.22;
  filter.movingScore = clamp(
    Number(filter.movingScore || 0) * 0.82 + (motionHit ? 0.28 : -0.08),
    0,
    1
  );
  filter.mode = filter.movingScore > 0.45 ? "dynamic" : "static";
  filter.lastInnovationM = innovationM;
  filter.lastGate = "accepted";
  filter.accepted += 1;
  filter.lastUpdate = now;
  return true;
}

function applyAutoKalman(tagId, position, accuracy, fitObservations, settings, now) {
  if (settings.solver !== "tdoa" || settings.tdoaMode !== "auto") {
    return {position, accuracy, filterInfo: null};
  }
  const key = kalmanFilterKey(tagId, settings);
  let filter = state.positionFilters[key];
  const hasMeasurement =
    position &&
    Number.isFinite(Number(position.x)) &&
    Number.isFinite(Number(position.y));
  if (!filter) {
    if (!hasMeasurement) return {position: null, accuracy, filterInfo: null};
    filter = initPositionFilter(position, accuracy, now);
    state.positionFilters[key] = filter;
    return {
      position: {x: filter.x, y: filter.y},
      accuracy,
      filterInfo: {...filter, status: "init"},
      rawPosition: position,
    };
  }

  kalmanPredict(filter, now);
  let accepted = false;
  if (hasMeasurement) {
    accepted = kalmanUpdate2d(filter, position, accuracy, fitObservations, now);
  } else {
    filter.lastGate = "predict";
    filter.movingScore = clamp(Number(filter.movingScore || 0) * 0.92, 0, 1);
    filter.mode = filter.movingScore > 0.45 ? "dynamic" : "static";
  }

  const predictionAge = now - Number(filter.lastUpdate || 0);
  if (!hasMeasurement && predictionAge > kalmanMaxPredictionAgeSec) {
    delete state.positionFilters[key];
    return {position: null, accuracy, filterInfo: null};
  }
  const sigma = Math.sqrt(Math.max(0, filter.p[0] + filter.p[5]) / 2);
  const filteredAccuracy = accuracy
    ? {...accuracy, sigma_major_m: Math.max(Number(accuracy.sigma_major_m) || 0, sigma)}
    : {count: 0, dof: 0, rms_m: NaN, max_abs_m: NaN, sigma_major_m: sigma, gdop: NaN};
  return {
    position: {x: filter.x, y: filter.y},
    accuracy: filteredAccuracy,
    filterInfo: {
      mode: filter.mode,
      status: accepted ? "accepted" : filter.lastGate,
      accepted: filter.accepted,
      rejected: filter.rejected,
      innovation_m: filter.lastInnovationM,
      speed_mps: Math.hypot(filter.vx, filter.vy),
      prediction_age_s: predictionAge,
      sigma_m: sigma,
    },
    rawPosition: hasMeasurement ? position : null,
  };
}

function rangingPositionAccuracy(position, anchors, distances, residuals) {
  if (!position) return null;
  const rows = [];
  for (const [anchorId, distance] of Object.entries(distances || {})) {
    const anchor = anchors[Number(anchorId)];
    if (!anchor || !Number.isFinite(Number(distance))) continue;
    const d = Math.max(1e-6, Math.hypot(position.x - anchor.x, position.y - anchor.y));
    rows.push({
      residual: Number(residuals?.[anchorId]),
      gx: (position.x - anchor.x) / d,
      gy: (position.y - anchor.y) / d,
    });
  }
  return positionAccuracyFromRows(rows);
}

function updatePositionAnchorTrail(anchors, now) {
  const activeIds = new Set(Object.keys(anchors).map(String));
  for (const key of Object.keys(state.positionAnchorTrail)) {
    if (!activeIds.has(key)) delete state.positionAnchorTrail[key];
  }
  for (const [anchorId, anchor] of Object.entries(anchors)) {
    const trail = state.positionAnchorTrail[anchorId] || [];
    trail.push({x: anchor.x, y: anchor.y, t: now});
    state.positionAnchorTrail[anchorId] = trail
      .filter(point => now - point.t <= 120)
      .slice(-300);
  }
}

function computePositionModel() {
  const settings = positionSettings();
  const selectedIds = selectedPositionModuleIds(settings);
  const offlineModuleIds = selectedIds.filter(id => !moduleHttpOnline(statusForModule(id)));
  const active = positionRangingActive(settings);
  const geometry = measuredAnchorGeometry(settings.anchorIds, settings.maxAge);
  const anchors = {...(geometry?.anchors || {})};

  if (!active && state.positionWasActive) {
    state.positionTrail = {};
    state.positionAnchorTrail = {};
    state.positionResults = {};
    resetPositionFilters();
  }
  state.positionWasActive = active;

  const tags = {};
  const now = Date.now() / 1000;
  updatePositionAnchorTrail(anchors, now);
  if (active) {
    for (const tagId of settings.tagIds) {
      const distances = {};
      const distanceItems = {};
      let observations = [];
      let fitObservations = [];
      let position = null;
      let rawPosition = null;
      let residuals = {};
      let accuracy = null;
      let filterInfo = null;
      if (settings.solver === "tdoa") {
        observations = pairedTdoaObservations(
          tagId,
          settings.anchorIds,
          settings.maxAge,
          settings.tdoaMode
        )
          .filter(item => anchors[Number(item.initiator_id)] && anchors[Number(item.responder_id)]);
        const fit = robustTdoaFit(settings.anchorIds, anchors, observations, settings);
        position = fit.position;
        fitObservations = fit.used || [];
        observations = fit.annotated || observations.map(item => ({...item, used_in_fit: true, reject_reason: ""}));
        residuals = tdoaResiduals(position, anchors, observations);
        accuracy = tdoaPositionAccuracy(
          position,
          anchors,
          fitObservations.length ? fitObservations : observations,
          tdoaResiduals(position, anchors, fitObservations.length ? fitObservations : observations)
        );
        rawPosition = position;
        const kalmanResult = applyAutoKalman(
          tagId,
          position,
          accuracy,
          fitObservations,
          settings,
          now
        );
        position = kalmanResult.position;
        accuracy = kalmanResult.accuracy;
        filterInfo = kalmanResult.filterInfo;
        rawPosition = kalmanResult.rawPosition || rawPosition;
      } else {
        for (const anchorId of settings.anchorIds) {
          const item = freshDistanceFor(tagId, anchorId, settings.maxAge);
          if (item && anchors[anchorId]) {
            distances[anchorId] = Number(item.distance_m);
            distanceItems[anchorId] = item;
          }
        }
        position = trilaterate(anchors, distances);
        residuals = positionResiduals(position, anchors, distances);
        accuracy = rangingPositionAccuracy(position, anchors, distances, residuals);
      }
      tags[tagId] = {
        tagId,
        distances,
        distanceItems,
        observations,
        fitObservations,
        position,
        rawPosition,
        residuals,
        accuracy,
        filterInfo,
      };
      if (position) {
        const key = String(tagId);
        const trail = state.positionTrail[key] || [];
        trail.push({x: position.x, y: position.y, t: now});
        state.positionTrail[key] = trail.filter(point => now - point.t <= 120).slice(-300);
      }
    }
  }

  state.positionResults = tags;
  return {settings, active, anchors, tags, geometry, offlineModuleIds};
}

function positionBounds(model) {
  const points = [];
  for (const anchor of Object.values(model.anchors)) points.push(anchor);
  for (const tag of Object.values(model.tags)) {
    if (tag.position) points.push(tag.position);
  }
  for (const trail of Object.values(state.positionTrail)) {
    for (const point of trail) points.push(point);
  }
  if (!points.length) {
    points.push({x: 0, y: 0}, {x: 2, y: 2});
  }
  let minX = Math.min(...points.map(point => point.x));
  let maxX = Math.max(...points.map(point => point.x));
  let minY = Math.min(...points.map(point => point.y));
  let maxY = Math.max(...points.map(point => point.y));
  const span = Math.max(1, maxX - minX, maxY - minY);
  const pad = Math.max(0.35, span * 0.14);
  return {minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad};
}

function positionTransform(model, width, height) {
  const bounds = positionBounds(model);
  const margin = 44;
  const sx = (width - margin * 2) / Math.max(0.1, bounds.maxX - bounds.minX);
  const sy = (height - margin * 2) / Math.max(0.1, bounds.maxY - bounds.minY);
  const scale = Math.max(1, Math.min(sx, sy));
  const plotW = (bounds.maxX - bounds.minX) * scale;
  const plotH = (bounds.maxY - bounds.minY) * scale;
  const ox = (width - plotW) / 2;
  const oy = (height - plotH) / 2;
  return {
    scale,
    x: value => ox + (value - bounds.minX) * scale,
    y: value => oy + (bounds.maxY - value) * scale,
  };
}

function drawPositionGrid(ctx, tx, width, height) {
  ctx.fillStyle = "#eef1f5";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d9dee8";
  ctx.lineWidth = 1;
  const gridMin = -10;
  const gridMax = 20;
  for (let value = gridMin; value <= gridMax; value += 0.5) {
    const x = tx.x(value);
    if (x >= 0 && x <= width) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    const y = tx.y(value);
    if (y >= 0 && y <= height) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
  }
}

function positionPerimeterAnchorIds(anchorIds, anchors) {
  const usable = anchorIds
    .map(Number)
    .filter(id => anchors[id]);
  if (usable.length < 3) return usable;

  const center = usable.reduce(
    (acc, id) => {
      acc.x += anchors[id].x;
      acc.y += anchors[id].y;
      return acc;
    },
    {x: 0, y: 0}
  );
  center.x /= usable.length;
  center.y /= usable.length;

  return usable.sort((left, right) =>
    Math.atan2(anchors[left].y - center.y, anchors[left].x - center.x) -
    Math.atan2(anchors[right].y - center.y, anchors[right].x - center.x));
}

function drawAnchorStabilityRings(ctx, tx, anchorIds, anchors) {
  ctx.save();
  ctx.setLineDash([3, 4]);
  for (const anchorId of anchorIds) {
    const anchor = anchors[anchorId];
    if (!anchor) continue;
    const cx = tx.x(anchor.x);
    const cy = tx.y(anchor.y);
    const trail = state.positionAnchorTrail[String(anchorId)] || [];
    const distances = trail
      .map(point => Math.hypot(tx.x(point.x) - cx, tx.y(point.y) - cy))
      .filter(value => Number.isFinite(value));
    distances.sort((a, b) => a - b);
    const p90 = distances.length ? distances[Math.floor((distances.length - 1) * 0.9)] : 0;
    const radius = Math.max(10, Math.min(42, p90 + 7));

    ctx.beginPath();
    ctx.strokeStyle = "rgba(43, 100, 216, 0.34)";
    ctx.lineWidth = 1.2;
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.restore();
}

function drawPosition(model) {
  const canvas = document.getElementById("positionCanvas");
  if (!canvas) return;
  const {ctx, width, height} = fitCanvas(canvas);
  if (width <= 1 || height <= 1) return;
  const tx = positionTransform(model, width, height);
  drawPositionGrid(ctx, tx, width, height);

  ctx.strokeStyle = "#98a2b3";
  ctx.lineWidth = 2;
  const anchorIds = model.settings.anchorIds.filter(id => model.anchors[id]);
  const perimeterAnchorIds = positionPerimeterAnchorIds(anchorIds, model.anchors);
  if (perimeterAnchorIds.length >= 2) {
    ctx.beginPath();
    perimeterAnchorIds.forEach((anchorId, index) => {
      const anchor = model.anchors[anchorId];
      const x = tx.x(anchor.x);
      const y = tx.y(anchor.y);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    if (perimeterAnchorIds.length >= 3) ctx.closePath();
    ctx.stroke();
  }

  if (model.settings.solver !== "tdoa") {
    for (const tag of Object.values(model.tags)) {
      for (const [anchorId, distance] of Object.entries(tag.distances || {})) {
        const anchor = model.anchors[Number(anchorId)];
        if (!anchor) continue;
        ctx.beginPath();
        ctx.strokeStyle = "rgba(43, 100, 216, 0.24)";
        ctx.lineWidth = 1.5;
        ctx.arc(tx.x(anchor.x), tx.y(anchor.y), Number(distance) * tx.scale, 0, Math.PI * 2);
        ctx.stroke();
      }
    }
  }

  for (const [tagId, trail] of Object.entries(state.positionTrail)) {
    if (trail.length < 2) continue;
    ctx.beginPath();
    ctx.strokeStyle = "rgba(43, 100, 216, 0.72)";
    ctx.lineWidth = 1.5;
    trail.forEach((point, index) => {
      const x = tx.x(point.x);
      const y = tx.y(point.y);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  drawAnchorStabilityRings(ctx, tx, anchorIds, model.anchors);

  ctx.font = "12px Inter, sans-serif";
  ctx.textBaseline = "middle";
  for (const anchorId of anchorIds) {
    const anchor = model.anchors[anchorId];
    const x = tx.x(anchor.x);
    const y = tx.y(anchor.y);
    ctx.fillStyle = "#16833a";
    ctx.beginPath();
    ctx.moveTo(x, y - 11);
    ctx.lineTo(x - 11, y + 10);
    ctx.lineTo(x + 11, y + 10);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = "#17202a";
    ctx.font = "700 12px Inter, sans-serif";
    ctx.fillText(`A${anchorId}`, x + 13, y - 12);
  }

  for (const tag of Object.values(model.tags)) {
    if (!tag.position) continue;
    const x = tx.x(tag.position.x);
    const y = tx.y(tag.position.y);
    ctx.fillStyle = "#d7352a";
    ctx.beginPath();
    ctx.arc(x, y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "#17202a";
    ctx.font = "700 12px Inter, sans-serif";
    ctx.fillText(`Tag ${tag.tagId}`, x + 11, y - 12);
  }
}

function renderPositionGeometryPanel(model) {
  const rows = document.getElementById("positionGeometryRows");
  if (!rows) return;

  const geometry = model.geometry || {};
  const pairRows = selectedAnchorPairs(model.settings.anchorIds).map(([a, b]) => {
    const key = anchorPairKey(a, b);
    const item = geometry.distanceItems?.[key];
    const residual = geometry.residuals?.[key];
    const stats = item?.stats || {};
    const direction = item ? `A${esc(item.initiator_id)}→A${esc(item.responder_id)}` : "waiting";
    const mean = stats.mean_m === null || stats.mean_m === undefined
      ? NaN
      : Number(stats.mean_m);
    const std = stats.std_m === null || stats.std_m === undefined
      ? NaN
      : Number(stats.std_m);
    return `<tr>
      <td>A${esc(a)}-A${esc(b)}<br><span class="muted">${direction}</span></td>
      <td>${item ? fmtFixed(item.distance_m, 3) : "-"}<br><span class="muted">${Number.isFinite(mean) ? "avg " + fmtFixed(mean, 3) : ""}</span></td>
      <td>${Number.isFinite(std) ? fmtCmFromM(std, 1) + " cm" : "-"}</td>
      <td class="${item && Number(item.age_sec) <= model.settings.maxAge ? "fresh" : "stale"}">${item ? fmtFixed(item.age_sec, 1) + "s" : "-"}</td>
      <td>${residual === undefined ? "-" : fmtFixed(residual * 100, 1) + " cm"}</td>
    </tr>`;
  });
  rows.innerHTML = pairRows.join("");
}

function fmtPositionCm(value, digits = 1) {
  return Number.isFinite(Number(value)) ? `${fmtCmFromM(value, digits)} cm` : "-";
}

function fmtPositionSigma(value, digits = 1) {
  return Number.isFinite(Number(value)) ? `±${fmtCmFromM(value, digits)} cm` : "-";
}

function positionTdoaFilterLabel(mode) {
  if (mode === "auto") return "Auto Kalman";
  if (mode === "dynamic") return "Dynamic 1.5s median";
  return "Static median";
}

function positionGateClass(status) {
  if (status === "accepted" || status === "init") return "good";
  if (status === "predict") return "warn";
  if (status) return "bad";
  return "";
}

function renderPositionFilterStatus(model) {
  const settings = model.settings || {};
  if (settings.solver !== "tdoa") {
    return `<div class="position-filter-card">
      <b>Solver Status</b>
      <div class="position-filter-row"><span class="position-pill">DS-TWR ranges</span></div>
    </div>`;
  }

  const tags = Object.values(model.tags || {});
  const filterLabel = positionTdoaFilterLabel(settings.tdoaMode);
  const activeKalman = settings.tdoaMode === "auto";
  let filterInfo = null;
  for (const tag of tags) {
    if (tag.filterInfo) {
      filterInfo = tag.filterInfo;
      break;
    }
  }

  const pills = [
    `<span class="position-pill">Filter: ${esc(filterLabel)}</span>`,
  ];
  if (activeKalman) {
    if (filterInfo) {
      const status = filterInfo.status || "waiting";
      pills.push(`<span class="position-pill ${esc(positionGateClass(status))}">Kalman ${esc(filterInfo.mode || "static")}</span>`);
      pills.push(`<span class="position-pill ${esc(positionGateClass(status))}">gate ${esc(status)}</span>`);
      pills.push(`<span class="position-pill">v ${fmtFixed(Number(filterInfo.speed_mps || 0), 2)} m/s</span>`);
      pills.push(`<span class="position-pill">ok/drop ${esc(filterInfo.accepted || 0)}/${esc(filterInfo.rejected || 0)}</span>`);
      if (Number.isFinite(Number(filterInfo.innovation_m))) {
        pills.push(`<span class="position-pill">innovation ${fmtCmFromM(filterInfo.innovation_m, 1)} cm</span>`);
      }
    } else {
      pills.push(`<span class="position-pill warn">Kalman waiting</span>`);
    }
  } else {
    pills.push(`<span class="position-pill warn">Kalman off</span>`);
  }

  return `<div class="position-filter-card">
    <b>Position Filter</b>
    <div class="position-filter-row">${pills.join("")}</div>
  </div>`;
}

function renderPositionReadout(model) {
  const readout = document.getElementById("positionReadout");
  const accuracyRows = document.getElementById("positionAccuracyRows");
  const rows = document.getElementById("positionDistanceRows");
  const head = document.getElementById("positionMeasurementHead");
  const title = document.getElementById("positionMeasurementTitle");
  const overlay = document.getElementById("positionOverlay");
  if (!readout || !accuracyRows || !rows || !head || !title || !overlay) return;
  const solverName = model.settings.solver === "tdoa" ? "FlexTDOA" : "ranging";
  const enableText = model.settings.solver === "tdoa" ? "Enable FlexTDOA" : "Enable Ranging";
  document.querySelectorAll("#positionEnableRanging, #positionEnableRangingSide")
    .forEach(button => { button.textContent = enableText; });
  renderPositionGeometryPanel(model);

  if (!model.active) {
    overlay.classList.add("active");
    if ((model.offlineModuleIds || []).length) {
      overlay.querySelector("h2").textContent = "Waiting for HTTP status";
      overlay.querySelector("p").textContent = `No live position is shown until these modules answer /status again: ${model.offlineModuleIds.join(", ")}.`;
    } else {
      overlay.querySelector("h2").textContent = `${solverName} is not active`;
      overlay.querySelector("p").textContent = "Enable the selected position runtime to clear old measurements and compute a new live position.";
    }
    readout.innerHTML = `${renderPositionFilterStatus(model)}<div class="position-tag-card"><b>${esc(solverName)} inactive</b><span>No stored position is shown while the selected modules are not in the selected runtime.</span></div>`;
    accuracyRows.innerHTML = "";
    title.textContent = model.settings.solver === "tdoa" ? "TDOA Observations" : "Distances";
    rows.innerHTML = "";
    return;
  }

  const missingCoords = model.settings.anchorIds.filter(id => !model.anchors[id]);
  if (missingCoords.length) {
    overlay.classList.add("active");
    overlay.querySelector("h2").textContent = "Waiting for measured anchor geometry";
    overlay.querySelector("p").textContent = `Waiting for fresh anchor-anchor ranges involving: ${missingCoords.join(", ")}.`;
  } else {
    overlay.classList.remove("active");
    overlay.querySelector("h2").textContent = `${solverName} is not active`;
    overlay.querySelector("p").textContent = "Enable the selected position runtime to clear old measurements and compute a new live position.";
  }

  const tagCards = Object.values(model.tags).map(tag => {
    const fitCount = model.settings.solver === "tdoa"
      ? (tag.fitObservations || []).length
      : Object.keys(tag.distances || {}).length;
    const freshCount = model.settings.solver === "tdoa"
      ? (tag.observations || []).length
      : Object.keys(tag.distances || {}).length;
    const total = model.settings.solver === "tdoa"
      ? Math.max(0, model.settings.anchorIds.length * (model.settings.anchorIds.length - 1) / 2)
      : model.settings.anchorIds.length;
    if (!tag.position) {
      return `<div class="position-tag-card"><b>Tag ${esc(tag.tagId)}</b><span>${freshCount}/${total} fresh ${model.settings.solver === "tdoa" ? "TDOA observations" : "distances"}</span></div>`;
    }
    const sigma = tag.accuracy?.sigma_major_m;
    const accuracyText = Number.isFinite(Number(sigma)) ? ` · est. ${fmtPositionSigma(sigma, 1)}` : "";
    const filterText = tag.filterInfo
      ? ` · Kalman ${esc(tag.filterInfo.mode || "static")} · ${esc(tag.filterInfo.status || "")}`
      : "";
    const rawText = tag.rawPosition && tag.filterInfo
      ? ` · raw ${fmtFixed(tag.rawPosition.x, 2)},${fmtFixed(tag.rawPosition.y, 2)}`
      : "";
    const countText = model.settings.solver === "tdoa"
      ? `${fitCount}/${total} fit · ${freshCount} fresh`
      : `${fitCount}/${total} fresh distances`;
    return `<div class="position-tag-card"><b>Tag ${esc(tag.tagId)}: x=${fmtFixed(tag.position.x, 2)} m, y=${fmtFixed(tag.position.y, 2)} m</b><span>${countText}${accuracyText}${filterText}${rawText}</span></div>`;
  });
  readout.innerHTML = `${renderPositionFilterStatus(model)}${tagCards.join("") || `<div class="position-tag-card"><b>waiting for tags</b><span>No selected tag IDs.</span></div>`}`;
  const accuracyTableRows = Object.values(model.tags).map(tag => {
    const accuracy = tag.accuracy;
    if (!tag.position || !accuracy) {
      return `<tr><td>T${esc(tag.tagId)}</td><td colspan="3"><span class="muted">waiting</span></td></tr>`;
    }
    const filter = tag.filterInfo;
    const filterText = filter
      ? `<br><span class="muted">${esc(filter.mode || "static")} · ${fmtFixed(Number(filter.speed_mps || 0), 2)} m/s · gate ${esc(filter.status || "")} · ok/drop ${esc(filter.accepted || 0)}/${esc(filter.rejected || 0)}</span>`
      : "";
    return `<tr>
      <td>T${esc(tag.tagId)}<br><span class="muted">${esc(accuracy.count)} obs · GDOP ${esc(fmtFixed(accuracy.gdop, 2))}</span>${filterText}</td>
      <td>${fmtPositionSigma(accuracy.sigma_major_m, 1)}</td>
      <td>${fmtPositionCm(accuracy.rms_m, 1)}</td>
      <td>${fmtPositionCm(accuracy.max_abs_m, 1)}</td>
    </tr>`;
  });
  accuracyRows.innerHTML = accuracyTableRows.join("") || `<tr><td colspan="4"><span class="muted">waiting</span></td></tr>`;

  if (model.settings.solver === "tdoa") {
    title.textContent = "TDOA Observations";
    head.innerHTML = `<tr><th>Tag</th><th>Pair</th><th>diff m</th><th>rev sum</th><th>age</th><th>resid.</th></tr>`;
    const tdoaRows = [];
    for (const tag of Object.values(model.tags)) {
      for (const item of tag.observations || []) {
        const key = `${item.initiator_id}-${item.responder_id}`;
        const reverseSum = Number(item.reverse_sum_m);
        const residual = tag.residuals?.[key];
        const used = item.used_in_fit !== false;
        const fitNote = used ? "fit" : `skip ${item.reject_reason || "outlier"}`;
        const agreement = Number(item.agreement_m);
        const agreementText = Number.isFinite(agreement) ? ` · agree ${fmtCmFromM(agreement, 1)} cm` : "";
        const blend = Number(item.blend_weight);
        const blendText = Number.isFinite(blend) ? ` · blend ${(blend * 100).toFixed(0)}%` : "";
        const weight = Number(item.solve_weight);
        const weightText = tdoaModeUsesDynamicObservations(model.settings.tdoaMode) && Number.isFinite(weight)
          ? ` · w ${(weight * 100).toFixed(0)}%`
          : "";
        const filterText = tdoaModeUsesDynamicObservations(item.tdoa_filter)
          ? ` · short n ${esc(item.dynamic_samples || item.samples || 1)}`
          : "";
        const suspectText = item.suspect ? " · suspect" : "";
        const fusedText = Number(item.fused_count || 0) > 0 ? ` · fused ${esc(item.fused_count)}` : "";
        tdoaRows.push(`<tr class="${used ? "" : "position-skip"}">
          <td>T${esc(tag.tagId)}</td>
          <td>A${esc(item.initiator_id)}↔A${esc(item.responder_id)}<br><span class="muted">seq ${esc(item.seq)}/${esc(item.reverse_seq)} · n ${esc(item.samples || 1)}${filterText}${agreementText}${blendText}${weightText}${fusedText}${suspectText} · ${esc(fitNote)}</span></td>
          <td>${fmtFixed(item.diff_m, 3)}</td>
          <td>${Number.isFinite(reverseSum) ? fmtCmFromM(reverseSum, 1) + " cm" : "-"}</td>
          <td class="${Number(item.age_sec) <= model.settings.maxAge ? "fresh" : "stale"}">${fmtFixed(item.age_sec, 1)}s</td>
          <td>${residual === undefined ? "-" : fmtFixed(residual * 100, 1) + " cm"}</td>
        </tr>`);
      }
    }
    rows.innerHTML = tdoaRows.join("");
  } else {
    title.textContent = "Distances";
    head.innerHTML = `<tr><th>Tag</th><th>Anchor</th><th>m</th><th>age</th><th>resid.</th></tr>`;
    const distanceRows = [];
    for (const tag of Object.values(model.tags)) {
      for (const anchorId of model.settings.anchorIds) {
        const item = tag.distanceItems?.[anchorId];
        const residual = tag.residuals?.[anchorId];
        distanceRows.push(`<tr>
          <td>T${esc(tag.tagId)}</td>
          <td>A${esc(anchorId)}</td>
          <td>${item ? fmtFixed(item.distance_m, 3) : "-"}</td>
          <td class="${item && Number(item.age_sec) <= model.settings.maxAge ? "fresh" : "stale"}">${item ? fmtFixed(item.age_sec, 1) + "s" : "-"}</td>
          <td>${residual === undefined ? "-" : fmtFixed(residual * 100, 1) + " cm"}</td>
        </tr>`);
      }
    }
    rows.innerHTML = distanceRows.join("");
  }
}

function renderPosition() {
  const model = computePositionModel();
  drawPosition(model);
  renderPositionReadout(model);
}

function canvasY(value, scale, plotArea) {
  const ratio = (Number(value) - scale.min) / Math.max(1, scale.max - scale.min);
  return plotArea.bottom - ratio * plotArea.height;
}

function canvasX(sample, latest, windowSec, plotArea) {
  const latestUptimeMs = Number(latest?.uptime_ms);
  const sampleUptimeMs = Number(sample.uptime_ms);
  if (!Number.isFinite(latestUptimeMs) || !Number.isFinite(sampleUptimeMs)) {
    return plotArea.right;
  }
  const ageSec = Math.max(0, (latestUptimeMs - sampleUptimeMs) / 1000);
  return plotArea.right - Math.min(1, ageSec / windowSec) * plotArea.width;
}

function drawSeries(ctx, samples, key, color, scale, latest, windowSec, plotArea) {
  const points = downsampleSeries(samples, key);
  if (!points.length) return;
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  for (let index = 0; index < points.length; index++) {
    const sample = points[index];
    const x = canvasX(sample, latest, windowSec, plotArea);
    const y = canvasY(sample[key], scale, plotArea);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawAccelCanvas(canvas, samples, scale, latest, windowSec) {
  const {ctx, width, height} = fitCanvas(canvas);
  const plotArea = {
    left: 48,
    right: Math.max(58, width - 8),
    top: 14,
    bottom: Math.max(30, height - 14),
  };
  plotArea.width = plotArea.right - plotArea.left;
  plotArea.height = plotArea.bottom - plotArea.top;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, width, height);
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";

  ctx.strokeStyle = "#dfe5ee";
  ctx.lineWidth = 1;
  for (let index = 0; index <= 10; index++) {
    const x = plotArea.left + (plotArea.width * index) / 10;
    ctx.beginPath();
    ctx.moveTo(x, plotArea.top);
    ctx.lineTo(x, plotArea.bottom);
    ctx.stroke();
  }

  const ticks = new Set([scale.min, scale.max]);
  for (let index = 1; index < 4; index++) {
    ticks.add(Math.round(scale.min + ((scale.max - scale.min) * index) / 4));
  }
  if (scale.min < 0 && scale.max > 0) ticks.add(0);
  for (const value of [...ticks].sort((a, b) => b - a)) {
    const y = canvasY(value, scale, plotArea);
    ctx.strokeStyle = value === 0 ? "#98a2b3" : "#dfe5ee";
    ctx.lineWidth = value === 0 ? 1.2 : 1;
    ctx.beginPath();
    ctx.moveTo(plotArea.left, y);
    ctx.lineTo(plotArea.right, y);
    ctx.stroke();
    if (value === scale.min || value === scale.max || value === 0) {
      ctx.fillStyle = "#667085";
      ctx.fillText(fmtAxis(value), plotArea.left - 8, y);
    }
  }

  if (!samples.length) {
    ctx.fillStyle = "#667085";
    ctx.textAlign = "center";
    ctx.fillText("no accelerometer samples", width / 2, height / 2);
    return;
  }

  drawSeries(ctx, samples, "x", "#2b64d8", scale, latest, windowSec, plotArea);
  drawSeries(ctx, samples, "y", "#16833a", scale, latest, windowSec, plotArea);
  drawSeries(ctx, samples, "z", "#b35b00", scale, latest, windowSec, plotArea);
}

function scheduleAccelRender() {
  if (state.accelRenderPending) return;
  state.accelRenderPending = true;
  requestAnimationFrame(() => {
    state.accelRenderPending = false;
    renderAccelGraphs();
  });
}

function renderAccelGraphs() {
  ensureAccelCharts();
  const timebaseEl = document.getElementById("accelTimebase");
  const timebaseSecPerDiv = Math.max(1, Number(timebaseEl?.value || state.timebaseSecPerDiv || 5));
  state.timebaseSecPerDiv = timebaseSecPerDiv;
  const windowSec = timebaseSecPerDiv * 10;

  for (const moduleId of [1, 2, 3, 4, 5]) {
    const allSamples = accelSamples(moduleId);
    const latest = allSamples[allSamples.length - 1];
    const samples = visibleAccelSamples(allSamples, latest, windowSec);
    const scale = accelScale(samples);
    const rate = accelRate(allSamples, 1.0);
    const canvas = document.getElementById(`accelCanvas${moduleId}`);
    if (canvas) drawAccelCanvas(canvas, samples, scale, latest, windowSec);
    const ageEl = document.getElementById(`chartAge${moduleId}`);
    if (ageEl) {
      const rateText = rate?.rxHz ? ` · ${Math.round(rate.rxHz)} rx/s` : "";
      ageEl.textContent = latest ? `last ${fmtAge(latest.received_at)}${rateText}` : "waiting";
    }
    if (!latest) {
      continue;
    }
    const magnitude = accelMagnitude(latest);
    document.getElementById(`latestX${moduleId}`).textContent = fmtAccel(latest.x);
    document.getElementById(`latestY${moduleId}`).textContent = fmtAccel(latest.y);
    document.getElementById(`latestZ${moduleId}`).textContent = fmtAccel(latest.z);
    const rateText = rate
      ? ` · rx ${Math.round(rate.rxHz)} Hz${rate.repHz ? ` · sensor ${Math.round(rate.repHz)} Hz` : ""}`
      : "";
    document.getElementById(`latestExtra${moduleId}`).textContent =
      `|a| ${fmtAccel(magnitude)} m/s^2 · accuracy ${latest.accuracy} · ${fmtAge(latest.received_at)}${rateText}`;
  }
}

function setActiveTab(id) {
  state.activeTab = id;
  localStorage.setItem("uwbDash.activeTab", id);
  document.querySelectorAll(".tab").forEach(tab => tab.classList.toggle("active", tab.dataset.tab === id));
  document.querySelectorAll(".page").forEach(page => page.classList.toggle("active", page.id === id));
  requestAnimationFrame(renderAllTerminals);
  requestAnimationFrame(renderPosition);
  scheduleAccelRender();
}

async function fetchLogs() {
  const res = await fetch(`/api/logs?after=${state.lastId}&limit=4000`, {cache: "no-store"});
  const data = await res.json();
  if (data.logs.length) {
    state.logs.push(...data.logs);
    if (state.logs.length > 8000) state.logs.splice(0, state.logs.length - 8000);
    state.lastId = Math.max(state.lastId, ...data.logs.map(item => item.id));
    ingestAccelLogs(data.logs);
    renderVisibleTerminals();
    scheduleAccelRender();
  }
  document.getElementById("clientPill").textContent = `${data.client_count} log client${data.client_count === 1 ? "" : "s"}`;
}

async function fetchAccel() {
  if (state.accelFetchPending) return;
  state.accelFetchPending = true;
  try {
    const res = await fetch(`/api/accel?after=${state.lastAccelId}&limit=12000`, {cache: "no-store"});
    const data = await res.json();
    if (data.samples.length) {
      for (const sample of data.samples) mergeAccelSample(sample);
      state.lastAccelId = Math.max(state.lastAccelId, ...data.samples.map(item => item.sample_id || 0));
      scheduleAccelRender();
    }
    const pill = document.getElementById("telemetryPill");
    if (pill) {
      pill.textContent = `${data.telemetry_client_count} telemetry client${data.telemetry_client_count === 1 ? "" : "s"}`;
    }
  } finally {
    state.accelFetchPending = false;
  }
}

function fmtMaybeNumber(value, digits = 2) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "-";
}

function fmtMaybeCoord(value) {
  const number = Number(value);
  return Number.isFinite(number) && Math.abs(number) > 0.00000001
    ? number.toFixed(7)
    : "-";
}

function fmtAgeMs(ageMs) {
  const number = Number(ageMs);
  if (!Number.isFinite(number) || number < 0 || number >= 0xffffffff) return "-";
  return `${(number / 1000).toFixed(1)}s`;
}

function fmtBytes(value) {
  const number = Number(value);
  if (!Number.isFinite(number) || number < 0) return "-";
  if (number >= 1024 * 1024) return `${(number / (1024 * 1024)).toFixed(2)} MiB`;
  return `${(number / 1024).toFixed(0)} KiB`;
}

function fmtPercent(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(1)}%` : "-";
}

function clampPercent(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return 0;
  return Math.max(0, Math.min(100, number));
}

function fmtMv(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${(number / 1000).toFixed(3)} V` : "-";
}

function fmtMa(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(0)} mA` : "-";
}

function fmtIbat(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return `<span class="muted">-</span>`;
  const className = number > 0 ? "ok" : (number < 0 ? "bad" : "muted");
  const label = number > 0 ? "charging" : (number < 0 ? "discharging" : "idle");
  const sign = number > 0 ? "+" : "";
  return `<span class="${className}" title="${label}">${sign}${number.toFixed(0)} mA</span>`;
}

function fmtSoc(item) {
  const number = Number(item?.charger_battery_soc_percent);
  return item?.charger_battery_soc_valid && Number.isFinite(number)
    ? `${number.toFixed(0)}%`
    : "-";
}

function fmtGpioLevel(value) {
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? String(number) : "-";
}

function renderGpsCell(item) {
  const enabled = Boolean(item.runtime_gps_enabled);
  const powered = Boolean(item.gps_powered);
  const uart = Boolean(item.gps_uart_ready);
  const sentences = Number(item.gps_sentence_count || 0);
  const fixText = item.gps_fix_quality_text || "unknown";
  const fixClass = item.gps_fix_valid ? "ok" : (enabled ? "warn" : "muted");
  const statusText = enabled
    ? `${powered ? "powered" : "off"} / ${uart ? "uart" : "no uart"}`
    : "disabled";
  const satsUsed = item.gps_satellites ?? "-";
  const satsView = item.gps_satellites_in_view ?? "-";
  const location = item.gps_fix_valid
    ? `${fmtMaybeCoord(item.gps_latitude_deg)}<br>${fmtMaybeCoord(item.gps_longitude_deg)}`
    : `<span class="muted">no fix</span>`;
  return `
    <span class="${enabled ? "ok" : "muted"}">${esc(statusText)}</span><br>
    <span class="${fixClass}">${esc(fixText)}</span>
    <span class="muted">q${esc(item.gps_fix_quality ?? "-")} type ${esc(item.gps_fix_type ?? "-")}</span><br>
    sats ${esc(satsUsed)}/${esc(satsView)} · hdop ${fmtMaybeNumber(item.gps_hdop, 2)}<br>
	    ${location}<br>
	    <span class="muted">rx ${fmtAgeMs(item.gps_last_rx_age_ms)} · sent ${esc(sentences)} · err ${esc(item.gps_checksum_errors ?? "-")}/${esc(item.gps_parse_errors ?? "-")}</span>`;
}

function chargerChargeStatus(item) {
  if (item.charger_charge_enabled === true) {
    return {className: "ok", text: "CHG enabled"};
  }
  if (item.charger_charge_enabled === false) {
    return {className: "warn", text: "CHG disabled"};
  }
  return {className: "muted", text: "CHG unknown"};
}

const CHARGER_CHG_STAT_NAMES = [
  "not charging", "trickle", "pre-charge", "fast CC",
  "taper CV", "reserved", "top-off", "terminated"
];
const CHARGER_VBUS_STAT_NAMES = {
  0: "no input",
  1: "USB SDP 500 mA",
  2: "USB CDP 1.5 A",
  3: "USB DCP 3.25 A",
  4: "HVDCP 1.5 A",
  5: "unknown adapter 3 A",
  6: "non-standard adapter",
  7: "OTG",
  8: "not qualified",
};

function chargerStatusByte(item, index) {
  const list = Array.isArray(item?.charger_status) ? item.charger_status : [];
  const value = Number(list[index]);
  return Number.isFinite(value) ? value : 0;
}

function chargerFaultByte(item, index) {
  const list = Array.isArray(item?.charger_fault_status) ? item.charger_fault_status : [];
  const value = Number(list[index]);
  return Number.isFinite(value) ? value : 0;
}

function chargerFlagByte(item, index) {
  const list = Array.isArray(item?.charger_flag) ? item.charger_flag : [];
  const value = Number(list[index]);
  return Number.isFinite(value) ? value : 0;
}

function chargerBoolField(item, key, fallback = false) {
  if (item && Object.prototype.hasOwnProperty.call(item, key)) {
    return item[key] === true || item[key] === "true" || item[key] === 1;
  }
  return Boolean(fallback);
}

function chargerExternalIlimEnabled(item) {
  if (item && Object.prototype.hasOwnProperty.call(item, "charger_external_input_current_limit_enabled")) {
    return chargerBoolField(item, "charger_external_input_current_limit_enabled");
  }
  const bytes = chargerRawBytes(item);
  const reg14 = bytes[0x14];
  return Number.isFinite(reg14) ? (reg14 & 0x02) !== 0 : undefined;
}

function chargerTerminationConfig(item) {
  const bytes = chargerRawBytes(item);
  const reg09 = bytes[0x09];
  const reg0a = bytes[0x0A];
  const reg0f = bytes[0x0F];
  const trechgByCode = [64, 256, 1024, 2048];
  const rawIterm = Number.isFinite(reg09) ? (reg09 & 0x1f) : undefined;
  const rawVrechg = Number.isFinite(reg0a) ? (reg0a & 0x0f) : undefined;
  const rawTrechg = Number.isFinite(reg0a) ? ((reg0a >> 4) & 0x03) : undefined;
  const offset = item.charger_recharge_threshold_offset_mv ??
    (rawVrechg !== undefined ? (rawVrechg + 1) * 50 : undefined);
  const vreg = Number(item?.charger_charge_voltage_limit_mv);
  const threshold = item.charger_recharge_threshold_mv ??
    (Number.isFinite(vreg) && Number.isFinite(Number(offset)) ? Math.max(0, vreg - Number(offset)) : undefined);
  return {
    terminationEnabled: item.charger_termination_enabled ??
      (Number.isFinite(reg0f) ? (reg0f & 0x02) !== 0 : undefined),
    terminationCurrentMa: item.charger_termination_current_ma ??
      (rawIterm !== undefined ? Math.max(40, rawIterm * 40) : undefined),
    rechargeOffsetMv: offset,
    rechargeThresholdMv: threshold,
    rechargeDeglitchMs: item.charger_recharge_deglitch_ms ??
      (rawTrechg !== undefined ? trechgByCode[rawTrechg] : undefined),
  };
}

function chargerTerminationSummary(item) {
  const config = chargerTerminationConfig(item);
  const term = config.terminationEnabled === undefined
    ? "TERM ?"
    : `TERM ${config.terminationEnabled ? "on" : "off"}`;
  const threshold = config.rechargeThresholdMv === undefined
    ? "-"
    : fmtMv(config.rechargeThresholdMv);
  return `${term} · ITERM ${fmtMa(config.terminationCurrentMa)}<br>` +
    `VRECHG -${fmtMv(config.rechargeOffsetMv)} -> ${threshold} · TRECHG ${esc(config.rechargeDeglitchMs ?? "-")} ms`;
}

function chargerChargePhase(item) {
  const code = (chargerStatusByte(item, 1) >> 5) & 0x07;
  return {code, text: CHARGER_CHG_STAT_NAMES[code] || `chg ${code}`};
}

function chargerVbusStatus(item) {
  const code = (chargerStatusByte(item, 1) >> 1) & 0x0f;
  return {code, text: CHARGER_VBUS_STAT_NAMES[code] || `reserved ${code}`};
}

function chargerVsysRegulating(item) {
  return (chargerStatusByte(item, 3) & 0x10) !== 0;
}

function chargerVbatOvp(item) {
  return (chargerFaultByte(item, 0) & 0x20) !== 0;
}

function chargerFastTimerExpired(item) {
  return chargerBoolField(item, "charger_charge_safety_timer_expired", (chargerStatusByte(item, 3) & 0x08) !== 0);
}

function chargerTimerFlags(item) {
  const flag2 = chargerFlagByte(item, 2);
  return {
    topoff: chargerBoolField(item, "charger_topoff_timer_flag", (flag2 & 0x01) !== 0),
    precharge: chargerBoolField(item, "charger_precharge_timer_flag", (flag2 & 0x02) !== 0),
    trickle: chargerBoolField(item, "charger_trickle_timer_flag", (flag2 & 0x04) !== 0),
    fast: chargerBoolField(item, "charger_fast_charge_timer_flag", (flag2 & 0x08) !== 0),
  };
}

function chargerTimerConfig(item) {
  const bytes = chargerRawBytes(item);
  const reg0d = bytes[0x0D];
  const reg0e = bytes[0x0E];
  const fastHoursByCode = [5, 8, 12, 24];
  const fromRaw = Number.isFinite(reg0e);
  return {
    topoffMinutes: item.charger_topoff_timer_minutes ?? (fromRaw ? ((reg0e >> 6) & 0x03) * 15 : undefined),
    trickleEnabled: item.charger_trickle_timer_enabled ?? (fromRaw ? (reg0e & 0x20) !== 0 : undefined),
    prechargeEnabled: item.charger_precharge_timer_enabled ?? (fromRaw ? (reg0e & 0x10) !== 0 : undefined),
    fastEnabled: item.charger_fast_charge_timer_enabled ?? (fromRaw ? (reg0e & 0x08) !== 0 : undefined),
    fastHours: item.charger_fast_charge_timer_hours ?? (fromRaw ? fastHoursByCode[(reg0e >> 1) & 0x03] : undefined),
    timer2xEnabled: item.charger_timer_2x_enabled ?? (fromRaw ? (reg0e & 0x01) !== 0 : undefined),
    prechargeMinutes: item.charger_precharge_timer_minutes ?? (Number.isFinite(reg0d) ? ((reg0d & 0x80) !== 0 ? 30 : 120) : undefined),
  };
}

function chargerTimerSummary(item) {
  const flags = chargerTimerFlags(item);
  const timer = chargerTimerConfig(item);
  const fastExpired = chargerFastTimerExpired(item);
  const activeFlags = [
    fastExpired ? "fast status" : "",
    flags.fast ? "fast flag" : "",
    flags.precharge ? "pre flag" : "",
    flags.trickle ? "trickle flag" : "",
    flags.topoff ? "top-off flag" : "",
  ].filter(Boolean);
  const fast = timer.fastEnabled
    ? `${esc(timer.fastHours ?? "-")} h`
    : "off";
  const pre = timer.prechargeEnabled
    ? `${esc(timer.prechargeMinutes ?? "-")} min`
    : "off";
  const tri = timer.trickleEnabled ? "on" : "off";
  const top = Number(timer.topoffMinutes || 0) > 0
    ? `${esc(timer.topoffMinutes)} min`
    : "off";
  const flagText = activeFlags.length
    ? `<br><span class="bad">timer ${activeFlags.join(", ")}</span>`
    : "";
  return `fast ${fast} · pre ${pre}<br>trickle ${tri} · top-off ${top} · TMR2X ${timer.timer2xEnabled ? "on" : "off"}${flagText}`;
}

function chargerTsRange(item) {
  const status4 = chargerStatusByte(item, 4);
  const tsIgnore = chargerBoolField(item, "charger_ts_ignore", false);
  const cold = chargerBoolField(item, "charger_ts_cold_active", (status4 & 0x08) !== 0);
  const cool = chargerBoolField(item, "charger_ts_cool_active", (status4 & 0x04) !== 0);
  const warm = chargerBoolField(item, "charger_ts_warm_active", (status4 & 0x02) !== 0);
  const hot = chargerBoolField(item, "charger_ts_hot_active", (status4 & 0x01) !== 0);
  if (tsIgnore) return {className: "muted", text: "ignored"};
  if (hot) return {className: "bad", text: "hot"};
  if (cold) return {className: "bad", text: "cold"};
  if (warm) return {className: "warn", text: "warm"};
  if (cool) return {className: "warn", text: "cool"};
  return {className: "ok", text: "normal"};
}

function chargerTsFlags(item) {
  const flag3 = chargerFlagByte(item, 3);
  return [
    chargerBoolField(item, "charger_ts_cold_flag", (flag3 & 0x08) !== 0) ? "cold" : "",
    chargerBoolField(item, "charger_ts_cool_flag", (flag3 & 0x04) !== 0) ? "cool" : "",
    chargerBoolField(item, "charger_ts_warm_flag", (flag3 & 0x02) !== 0) ? "warm" : "",
    chargerBoolField(item, "charger_ts_hot_flag", (flag3 & 0x01) !== 0) ? "hot" : "",
  ].filter(Boolean);
}

function chargerLimitWarning(item) {
  const vsysmin = Number(item?.charger_minimal_system_voltage_mv);
  const vreg = Number(item?.charger_charge_voltage_limit_mv);
  if (!Number.isFinite(vsysmin) || !Number.isFinite(vreg)) return "";
  if (vsysmin >= vreg) {
    return `<br><span class="bad">warning: VSYSMIN >= VREG</span>`;
  }
  if (vsysmin > 4100 && vreg <= 4990) {
    return `<br><span class="warn">high VSYSMIN for 1S</span>`;
  }
  return "";
}

function renderBatteryCell(item) {
  if (!item.charger_monitor_enabled) {
    return `<span class="muted">monitor off</span>`;
  }
  const pinLine = `Board PG ${fmtGpioLevel(item.charger_pg_gpio_level)}${item.charger_pg_asserted ? " asserted" : ""}
    · PG_STAT ${item.charger_pg_stat ? "1" : "0"}
    · INT ${fmtGpioLevel(item.charger_int_gpio_level)} / ${esc(item.charger_int_irq_count ?? "-")}
    · QON cmd ${fmtGpioLevel(item.charger_qon_gpio_level)}${item.charger_qon_asserted ? " asserted" : ""}`;
  if (!item.charger_present) {
    return `<span class="bad">BQ25792 not found</span><br><span class="muted">${esc(item.charger_last_error_name || "-")}</span><br><span class="muted">${pinLine}</span>`;
  }
  const adcClass = item.charger_adc_enabled ? "ok" : "warn";
  const charge = chargerChargeStatus(item);
  const writeText = item.charger_config_write_supported
    ? (item.charger_config_writes_enabled ? "writes enabled" : "writes disabled")
    : "read only";
  const phase = chargerChargePhase(item);
  const vbus = chargerVbusStatus(item);
  const vsysClass = chargerVsysRegulating(item) ? "warn" : "ok";
  const ovpClass = chargerVbatOvp(item) ? "bad" : "ok";
  const extIlim = chargerExternalIlimEnabled(item);
  const extIlimText = extIlim === undefined ? "?" : (extIlim ? "on" : "off");
  const extIlimClass = extIlim === undefined ? "muted" : (extIlim ? "warn" : "ok");
  const ts = chargerTsRange(item);
  const tsFlags = chargerTsFlags(item);
  const tsFlagText = tsFlags.length ? ` · flags ${tsFlags.join(",")}` : "";
  return `
    <span class="${item.charger_read_ok ? "ok" : "bad"}">BQ25792</span>
    <span class="muted">PN ${esc(item.charger_part_number ?? "-")} rev ${esc(item.charger_device_revision ?? "-")}</span><br>
    <span class="${adcClass}">ADC ${item.charger_adc_enabled ? "on" : "off"}</span>
    <span class="${charge.className}">${charge.text}</span>
    <span class="muted">${esc(writeText)}</span><br>
    phase ${esc(phase.text)} · input ${esc(vbus.text)}<br>
    <span class="${vsysClass}">VSYSMIN loop ${chargerVsysRegulating(item) ? "on" : "off"}</span>
    <span class="${extIlimClass}">ILIM_HIZ clamp ${extIlimText}</span>
    <span class="${ovpClass}">VBAT_OVP ${chargerVbatOvp(item) ? "on" : "off"}</span>${chargerLimitWarning(item)}<br>
    ${chargerTerminationSummary(item)}<br>
    ${chargerTimerSummary(item)}<br>
    VBAT ${fmtMv(item.charger_vbat_mv)} · SOC ${fmtSoc(item)}<br>
    VSYS ${fmtMv(item.charger_vsys_mv)} · VBUS ${fmtMv(item.charger_vbus_mv)}<br>
    IBUS ${fmtMa(item.charger_ibus_ma)}<br>
    IBAT ${fmtIbat(item.charger_ibat_ma)} · TDIE ${fmtMaybeNumber(item.charger_tdie_c, 1)} C<br>
    TS ext ${fmtMaybeNumber(item.charger_ts_percent, 2)}%REGN <span class="${ts.className}">${ts.text}</span>${esc(tsFlagText)}<br>
    <span class="muted">${pinLine}</span><br>
    <span class="muted">REG48 ${esc(item.charger_part_info || "-")}
      · reads ${esc(item.charger_read_count ?? "-")}
      (${esc(item.charger_full_read_count ?? "-")}/${esc(item.charger_quick_read_count ?? "-")} full/quick)
      · ${esc(item.charger_last_read_duration_ms ?? "-")} ms
      · age ${fmtAgeMs(item.charger_last_update_age_ms)}</span>`;
}

function renderResourceCell(item) {
  if (!item || !item.resource_monitor_running) {
    return `<span class="muted">not available</span>`;
  }
  const memoryMeter = (kind, label, freeValue, totalValue, minFreeValue, largestValue) => {
    const free = Number(freeValue);
    const total = Number(totalValue);
    const minFree = Number(minFreeValue);
    const largest = Number(largestValue);
    if (!Number.isFinite(free) || !Number.isFinite(total) || total <= 0) {
      return `
        <div class="resource-meter ${kind}">
          <div class="resource-meter-head">
            <span class="resource-meter-name">${esc(label)}</span>
            <span class="resource-meter-value">not available</span>
          </div>
          <div class="resource-bar"><div class="resource-bar-fill" style="width:0%"></div></div>
        </div>`;
    }
    const used = Math.max(0, total - free);
    const usedPercent = clampPercent((used * 100) / total);
    const freePercent = clampPercent((free * 100) / total);
    const minUsed = Number.isFinite(minFree) ? Math.max(0, total - minFree) : NaN;
    const minUsedPercent = Number.isFinite(minUsed) ? clampPercent((minUsed * 100) / total) : NaN;
    const minText = Number.isFinite(minUsedPercent) ? `peak ${minUsedPercent.toFixed(0)}%` : "peak -";
    const largestText = Number.isFinite(largest) ? `blk ${fmtBytes(largest)}` : "blk -";
    return `
      <div class="resource-meter ${kind}" title="${esc(label)} used ${fmtBytes(used)} / total ${fmtBytes(total)}">
        <div class="resource-meter-head">
          <span class="resource-meter-name">${esc(label)}</span>
          <span class="resource-meter-value">${fmtBytes(free)} free / ${fmtBytes(total)}</span>
        </div>
        <div class="resource-bar" aria-label="${esc(label)} ${usedPercent.toFixed(1)} percent used">
          <div class="resource-bar-fill" style="width:${usedPercent.toFixed(1)}%"></div>
        </div>
        <div class="resource-meter-foot">${usedPercent.toFixed(0)}% used · ${freePercent.toFixed(0)}% free · ${minText} · ${largestText}</div>
      </div>`;
  };
  const percentMeter = (kind, label, value, valid, foot = "") => {
    const percent = Number(value);
    if (!valid || !Number.isFinite(percent)) {
      return `
        <div class="resource-meter ${kind}">
          <div class="resource-meter-head">
            <span class="resource-meter-name">${esc(label)}</span>
            <span class="resource-meter-value">warming up</span>
          </div>
          <div class="resource-bar"><div class="resource-bar-fill" style="width:0%"></div></div>
        </div>`;
    }
    const clamped = clampPercent(percent);
    const footText = foot ? ` · ${foot}` : "";
    return `
      <div class="resource-meter ${kind}" title="${esc(label)} load ${percent.toFixed(1)}%">
        <div class="resource-meter-head">
          <span class="resource-meter-name">${esc(label)}</span>
          <span class="resource-meter-value">${percent.toFixed(1)}%</span>
        </div>
        <div class="resource-bar" aria-label="${esc(label)} ${percent.toFixed(1)} percent load">
          <div class="resource-bar-fill" style="width:${clamped.toFixed(1)}%"></div>
        </div>
        <div class="resource-meter-foot">${clamped.toFixed(0)}% load${esc(footText)}</div>
      </div>`;
  };
  const flashMeter = () => {
    const total = Number(item.resource_flash_total_bytes);
    const reserved = Number(item.resource_flash_reserved_bytes);
    const free = Number(item.resource_flash_free_bytes);
    const count = Number(item.resource_flash_partition_count);
    if (!item.resource_flash_valid || !Number.isFinite(total) || total <= 0 || !Number.isFinite(reserved)) {
      return `
        <div class="resource-meter flash">
          <div class="resource-meter-head">
            <span class="resource-meter-name">FLASH</span>
            <span class="resource-meter-value">${esc(item.resource_flash_error_name || "not available")}</span>
          </div>
          <div class="resource-bar"><div class="resource-bar-fill" style="width:0%"></div></div>
        </div>`;
    }
    const reservedPercent = clampPercent((reserved * 100) / total);
    const freePercent = Number.isFinite(free) ? clampPercent((free * 100) / total) : NaN;
    const freeText = Number.isFinite(freePercent) ? `${freePercent.toFixed(0)}% unreserved` : "unreserved -";
    const countText = Number.isFinite(count) ? `${count} partitions` : "partitions -";
    return `
      <div class="resource-meter flash" title="Flash reserved by partition layout ${fmtBytes(reserved)} / total ${fmtBytes(total)}">
        <div class="resource-meter-head">
          <span class="resource-meter-name">FLASH</span>
          <span class="resource-meter-value">${fmtBytes(reserved)} reserved / ${fmtBytes(total)}</span>
        </div>
        <div class="resource-bar" aria-label="Flash ${reservedPercent.toFixed(1)} percent reserved">
          <div class="resource-bar-fill" style="width:${reservedPercent.toFixed(1)}%"></div>
        </div>
        <div class="resource-meter-foot">${reservedPercent.toFixed(0)}% reserved · ${freeText} · ${countText}</div>
      </div>`;
  };
  const tempMeter = () => {
    const temp = Number(item.resource_temperature_c);
    if (!item.resource_temperature_valid || !Number.isFinite(temp)) {
      return `
        <div class="resource-meter temp">
          <div class="resource-meter-head">
            <span class="resource-meter-name">ESP temp</span>
            <span class="resource-meter-value">${esc(item.resource_temperature_error_name || "not available")}</span>
          </div>
          <div class="resource-bar"><div class="resource-bar-fill" style="width:0%"></div></div>
        </div>`;
    }
    const minTemp = 20;
    const maxTemp = 80;
    const tempPercent = clampPercent(((temp - minTemp) * 100) / (maxTemp - minTemp));
    return `
      <div class="resource-meter temp" title="ESP internal temperature ${temp.toFixed(1)} C">
        <div class="resource-meter-head">
          <span class="resource-meter-name">ESP temp</span>
          <span class="resource-meter-value">${temp.toFixed(1)} C</span>
        </div>
        <div class="resource-bar" aria-label="ESP temperature ${temp.toFixed(1)} C">
          <div class="resource-bar-fill" style="width:${tempPercent.toFixed(1)}%"></div>
        </div>
        <div class="resource-meter-foot">scale ${minTemp}-${maxTemp} C · ${tempPercent.toFixed(0)}%</div>
      </div>`;
  };
  const taskMeters = () => {
    const tasks = Array.isArray(item.resource_top_tasks) ? item.resource_top_tasks : [];
    if (item.resource_task_list_overflow) {
      return `<div class="resource-task-list"><span class="warn">task list overflow</span></div>`;
    }
    if (!item.resource_task_load_valid) {
      return `<div class="resource-task-list"><span class="muted">task CPU warming up</span></div>`;
    }
    if (!tasks.length) {
      return `<div class="resource-task-list"><span class="muted">no active tasks</span></div>`;
    }
    const rows = tasks.map(task => {
      const load = Number(task.load_percent);
      const loadPercent = Number.isFinite(load) ? clampPercent(load) : 0;
      const core = Number(task.core);
      const coreText = Number.isFinite(core) && core >= 0 ? `C${core}` : "core -";
      const name = String(task.name || "?");
      return `
        <div class="resource-meter task" title="${esc(name)} ${Number.isFinite(load) ? load.toFixed(1) : "-"}% · ${esc(coreText)}">
          <div class="resource-meter-head">
            <span class="resource-meter-name resource-task-name">${esc(name)}</span>
            <span class="resource-meter-value">${esc(coreText)} · ${Number.isFinite(load) ? load.toFixed(1) : "-"}%</span>
          </div>
          <div class="resource-bar" aria-label="${esc(name)} task load">
            <div class="resource-bar-fill" style="width:${loadPercent.toFixed(1)}%"></div>
          </div>
        </div>`;
    }).join("");
    return `<div class="resource-task-list"><div class="resource-task-title">Top tasks</div>${rows}</div>`;
  };
  return `
    <div class="resource-cell">
      ${memoryMeter("internal", "RAM", item.resource_internal_free_bytes, item.resource_internal_total_bytes, item.resource_internal_min_free_bytes, item.resource_internal_largest_free_block_bytes)}
      ${memoryMeter("psram", "PSRAM", item.resource_psram_free_bytes, item.resource_psram_total_bytes, item.resource_psram_min_free_bytes, item.resource_psram_largest_free_block_bytes)}
      ${flashMeter()}
      ${percentMeter("cpu0", "CORE0", item.resource_core0_load_percent, item.resource_cpu_load_valid)}
      ${percentMeter("cpu1", "CORE1", item.resource_core1_load_percent, item.resource_cpu_load_valid)}
      ${tempMeter()}
      ${taskMeters()}
      <div class="resource-meta">age ${fmtAgeMs(item.resource_last_update_age_ms)}</div>
    </div>`;
}

function hexByte(value) {
  const number = Number(value);
  return Number.isFinite(number)
    ? `0x${number.toString(16).toUpperCase().padStart(2, "0")}`
    : "-";
}

function chargerRawBytes(item) {
  const raw = String(item?.charger_raw_hex || "");
  if (!raw || raw.length % 2) return [];
  const bytes = [];
  for (let index = 0; index < raw.length; index += 2) {
    const value = Number.parseInt(raw.slice(index, index + 2), 16);
    if (!Number.isFinite(value)) return [];
    bytes.push(value);
  }
  return bytes;
}

function renderChargerRows(statuses) {
  const rows = document.getElementById("chargerRows");
  if (!rows) return;
  rows.innerHTML = (statuses || []).map(item => {
    const powerClass = item.charger_present ? "ok" : "bad";
    const adcClass = item.charger_adc_enabled ? "ok" : "warn";
    const charge = chargerChargeStatus(item);
    const phase = chargerChargePhase(item);
    const vbus = chargerVbusStatus(item);
    const status0 = chargerStatusByte(item, 0);
    const iindpm = (status0 & 0x80) !== 0;
    const vindpm = (status0 & 0x40) !== 0;
    const vsys = chargerVsysRegulating(item);
    const ovp = chargerVbatOvp(item);
    const extIlim = chargerExternalIlimEnabled(item);
    const extIlimText = extIlim === undefined ? "?" : (extIlim ? "on" : "off");
    const extIlimClass = extIlim === undefined ? "muted" : (extIlim ? "warn" : "ok");
    const timerExpired = chargerFastTimerExpired(item);
    const ts = chargerTsRange(item);
    const tsFlags = chargerTsFlags(item);
    const tsFlagText = tsFlags.length ? ` · flags ${tsFlags.join(",")}` : "";
    return `<tr>
      <td><b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span></td>
      <td class="charger-power-cell"><span class="${powerClass}">${item.charger_present ? "BQ present" : "not found"}</span><br>
        PG ${fmtGpioLevel(item.charger_pg_gpio_level)}${item.charger_pg_asserted ? " asserted" : ""}<br>
        STAT ${item.charger_pg_stat ? "1" : "0"} · INT ${fmtGpioLevel(item.charger_int_gpio_level)}<br>
        irq ${esc(item.charger_int_irq_count ?? "-")}<br>
        <span class="muted">REG48 ${esc(item.charger_part_info || "-")}<br>
          reads ${esc(item.charger_read_count ?? "-")} · ${esc(item.charger_last_read_duration_ms ?? "-")} ms<br>
          age ${fmtAgeMs(item.charger_last_update_age_ms)}</span></td>
      <td><span class="${adcClass}">ADC ${item.charger_adc_enabled ? "on" : "off"}</span><br>
        sample ${esc(item.charger_adc_sample ?? "-")} · ${item.charger_adc_continuous ? "continuous" : "one shot"}<br>
        avg ${item.charger_adc_running_average ? "on" : "off"} · EN_IBAT ${item.charger_ibat_discharge_sense_enabled ? "on" : "off"}<br>
        WD ${esc(item.charger_watchdog_setting ?? "-")} ${item.charger_watchdog_disabled ? "(disabled)" : ""}</td>
      <td><span class="${charge.className}">${charge.text}</span><br>
        phase ${esc(phase.text)}<br>
        input ${esc(vbus.text)}<br>
        <span class="${vsys ? "warn" : "ok"}">VSYSMIN ${vsys ? "on" : "off"}</span>
        <span class="${iindpm ? "warn" : "ok"}">IINDPM ${iindpm ? "on" : "off"}</span>
        <span class="${vindpm ? "warn" : "ok"}">VINDPM ${vindpm ? "on" : "off"}</span><br>
        <span class="${ovp ? "bad" : "ok"}">VBAT_OVP ${ovp ? "on" : "off"}</span>${chargerLimitWarning(item)}<br>
        <span class="${timerExpired ? "bad" : "ok"}">CHG timer ${timerExpired ? "expired" : "ok"}</span><br>
        VSYSMIN ${fmtMv(item.charger_minimal_system_voltage_mv)}<br>
        VREG ${fmtMv(item.charger_charge_voltage_limit_mv)}<br>
        ICHG ${fmtMa(item.charger_charge_current_limit_ma)}<br>
        VINDPM ${fmtMv(item.charger_input_voltage_limit_mv)}<br>
        IINDPM ${fmtMa(item.charger_input_current_limit_ma)}<br>
        <span class="${extIlimClass}">ILIM_HIZ clamp ${extIlimText}</span><br>
        ${chargerTerminationSummary(item)}<br>
        ${chargerTimerSummary(item)}</td>
      <td>SOC ${fmtSoc(item)} · VBAT ${fmtMv(item.charger_vbat_mv)}<br>
        VBUS ${fmtMv(item.charger_vbus_mv)} · VSYS ${fmtMv(item.charger_vsys_mv)}<br>
        VAC1 ${fmtMv(item.charger_vac1_mv)}<br>
        IBUS ${fmtMa(item.charger_ibus_ma)} · IBAT ${fmtIbat(item.charger_ibat_ma)}<br>
        TS ext ${fmtMaybeNumber(item.charger_ts_percent, 2)}%REGN <span class="${ts.className}">${ts.text}</span>${esc(tsFlagText)}<br>
        TDIE ${fmtMaybeNumber(item.charger_tdie_c, 1)} C</td>
      <td>writes ${esc(item.charger_write_count ?? "-")} · err ${esc(item.charger_write_error_count ?? "-")}<br>
        reg ${esc(item.charger_last_write_reg || "-")} ${item.charger_last_write_mask_used ? `mask ${esc(item.charger_last_write_mask || "-")}` : ""}<br>
        ${esc(item.charger_last_write_before || "-")} -> ${esc(item.charger_last_write_after || "-")}<br>
        <span class="muted">${esc(item.charger_last_write_error_name || "-")} · ${fmtAgeMs(item.charger_last_write_age_ms)}</span></td>
    </tr>`;
  }).join("");
}

function renderChargerRegisters() {
  const grid = document.getElementById("chargerRawRegisters");
  const select = document.getElementById("chargerRawModule");
  if (!grid || !select) return;
  const moduleId = Number(select.value || 1);
  const item = state.statuses.find(status => Number(status.module_id) === moduleId);
  const bytes = chargerRawBytes(item);
  if (!item) {
    grid.innerHTML = `<div class="muted">module ${moduleId} has no status yet</div>`;
    return;
  }
  if (!bytes.length) {
    grid.innerHTML = `<div class="muted">no raw register map available</div>`;
    return;
  }
  grid.innerHTML = bytes.map((value, reg) => `
    <div class="reg-cell">
      <b>${hexByte(reg)} = ${hexByte(value)}</b>
      <span>${esc(BQ_REG_NAMES[reg] || "Reserved")}</span>
    </div>`).join("");
}

function renderCharger(statuses) {
  renderChargerRows(statuses);
  renderChargerRegisters();
}

function pdRawBytes(item) {
  const raw = String(item?.pd_raw_hex || "");
  if (!raw || raw.length % 2) return [];
  const bytes = [];
  for (let index = 0; index < raw.length; index += 2) {
    const value = Number.parseInt(raw.slice(index, index + 2), 16);
    if (!Number.isFinite(value)) return [];
    bytes.push(value);
  }
  return bytes;
}

function pdPdoNumber(value) {
  if (typeof value === "number") return value >>> 0;
  if (typeof value === "string") return Number.parseInt(value, 0) >>> 0;
  return 0;
}

function pdFixedPdoParts(value) {
  const pdo = pdPdoNumber(value);
  if (((pdo >>> 30) & 0x03) !== 0) return null;
  return {
    mv: ((pdo >>> 10) & 0x03ff) * 50,
    ma: (pdo & 0x03ff) * 10,
  };
}

function pdPdoLabel(value, index = 0, selectedPos = 0) {
  const pdo = pdPdoNumber(value);
  const type = (pdo >>> 30) & 0x03;
  const prefix = index > 0 ? `${index}${index === selectedPos ? " selected" : ""}: ` : "";
  const fixed = pdFixedPdoParts(pdo);
  if (fixed) return `${prefix}${(fixed.mv / 1000).toFixed(2)} V / ${fixed.ma} mA`;
  if (type === 3) return `${prefix}APDO/PPS raw 0x${pdo.toString(16).toUpperCase().padStart(8, "0")}`;
  return `${prefix}PDO type ${type} raw 0x${pdo.toString(16).toUpperCase().padStart(8, "0")}`;
}

function pdPdoInputText(pdos) {
  if (!Array.isArray(pdos) || !pdos.length) return "";
  return pdos.map(item => {
    const fixed = pdFixedPdoParts(item);
    return fixed ? `${fixed.mv}:${fixed.ma}` : "";
  }).filter(Boolean).join(",");
}

function pdPdoListHtml(pdos, selectedPos = 0) {
  if (!Array.isArray(pdos) || !pdos.length) return `<span class="muted">none read yet</span>`;
  return `<ol class="pd-pdo-list">${pdos.map((pdo, index) =>
    `<li>${esc(pdPdoLabel(pdo, index + 1, selectedPos))}</li>`
  ).join("")}</ol>`;
}

function pdStatusPresent(item) {
  return item && item.module_id && item.pd_monitor_enabled !== false &&
    (item.pd_present !== undefined ||
     item.pd_device_id !== undefined ||
     item.pd_source_pdos !== undefined ||
     item.pd_raw_hex !== undefined);
}

function pdPresentClass(item) {
  if (!item?.pd_monitor_enabled) return "muted";
  return item.pd_present ? "ok" : "bad";
}

function renderPdRows(statuses) {
  const rows = document.getElementById("pdRows");
  if (!rows) return;
  rows.innerHTML = (statuses || []).map(item => {
    if (!item.pd_monitor_enabled) {
      return `<tr>
        <td><b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span></td>
        <td colspan="4"><span class="muted">MAX77958 monitor off</span></td>
      </tr>`;
    }
    const presentClass = pdPresentClass(item);
    const vbusText = item.pd_vbus_detected
      ? `${fmtMv(item.pd_vbus_mid_mv)} (${fmtMv(item.pd_vbus_min_mv)}-${fmtMv(item.pd_vbus_max_mv)})`
      : "not detected";
    const sourcePdos = Array.isArray(item.pd_source_pdos) ? item.pd_source_pdos : [];
    const sinkPdos = Array.isArray(item.pd_sink_pdos) ? item.pd_sink_pdos : [];
    return `<tr>
      <td><b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span></td>
      <td><span class="${presentClass}">${item.pd_present ? "MAX77958 present" : "not found"}</span><br>
        dev ${esc(item.pd_device_id || "-")} rev ${esc(item.pd_device_rev || "-")} fw ${esc(item.pd_fw_rev ?? "-")}.${esc(item.pd_fw_sub_ver ?? "-")}<br>
        VBUS ${esc(vbusText)}${item.pd_vbus_above_range ? ` <span class="warn">above range</span>` : ""}<br>
        BC ${esc(item.pd_chg_typ_name || "-")} · CC ${esc(item.pd_cc_pin_name || "-")} / ${esc(item.pd_cc_stat_name || "-")}<br>
        <span class="muted">sys ${esc(item.pd_sys_msg_name || "-")} · UIC ${esc(item.pd_uic_int || "-")} PD ${esc(item.pd_pd_int || "-")}</span></td>
      <td>selected source PDO ${esc(item.pd_selected_source_pdo_pos || "-")}<br>
        data role ${item.pd_data_role_dfp ? "DFP" : "UFP"} · power role ${item.pd_power_role_source ? "source" : "sink"}<br>
        PS_RDY as sink ${item.pd_psrdy_as_sink ? "yes" : "no"}<br>
        USB2 switch <span class="${item.pd_usb2_switch_closed ? "ok" : "muted"}">${item.pd_usb2_switch_closed ? "closed" : "open"}</span><br>
        PPS default ${item.pd_pps_default_enabled ? `${fmtMv(item.pd_pps_default_mv)} / ${fmtMa(item.pd_pps_default_ma)}` : "off"}</td>
      <td><b>Source</b> <span class="${item.pd_source_caps_valid ? "ok" : "muted"}">${esc(item.pd_source_pdo_count ?? 0)} PDOs</span>
        ${pdPdoListHtml(sourcePdos, Number(item.pd_selected_source_pdo_pos || 0))}
        <b>Sink</b> <span class="${item.pd_sink_pdos_valid ? "ok" : "muted"}">${esc(item.pd_sink_pdo_count ?? 0)} PDOs ${item.pd_sink_pdos_from_mtp ? "MTP" : "RAM"}</span>
        ${pdPdoListHtml(sinkPdos, 0)}</td>
      <td>ops ${esc(item.pd_operation_count ?? "-")} · err ${esc(item.pd_operation_error_count ?? "-")}<br>
        last op ${esc(item.pd_last_opcode || "-")} -> ${esc(item.pd_last_response_opcode || "-")}<br>
        result ${esc(item.pd_last_result_code ?? "-")} ${esc(item.pd_last_result_name || "")}<br>
        <span class="muted">err ${esc(item.pd_last_operation_error_name || item.pd_last_error_name || "-")}
        · reads ${esc(item.pd_read_count ?? "-")} · age ${fmtAgeMs(item.pd_last_update_age_ms)}</span></td>
    </tr>`;
  }).join("");
}

function renderPdRegisters() {
  const grid = document.getElementById("pdRawRegisters");
  const select = document.getElementById("pdRawModule");
  if (!grid || !select) return;
  const moduleId = Number(select.value || 1);
  const item = state.statuses.find(status => Number(status.module_id) === moduleId);
  const bytes = pdRawBytes(item);
  if (!item) {
    grid.innerHTML = `<div class="muted">module ${moduleId} has no status yet</div>`;
    return;
  }
  if (!bytes.length) {
    grid.innerHTML = `<div class="muted">no raw register map available</div>`;
    return;
  }
  grid.innerHTML = bytes.map((value, reg) => `
    <div class="reg-cell">
      <b>${hexByte(reg)} = ${hexByte(value)}</b>
      <span>${esc(MAX77958_REG_NAMES[reg] || "Reserved")}</span>
    </div>`).join("");
}

function renderPd(statuses) {
  renderPdRows(statuses);
  renderPdRegisters();
}

function renderInfo(snapshot) {
  state.statuses = snapshot.statuses || [];
  state.ranging = snapshot.ranging || {distances: {}, max_age_sec: 3};
  state.tdoa = snapshot.tdoa || {observations: {}, anchor_distances: {}, max_age_sec: 3};
  mergeAccelHistory(snapshot.accel_history || {});
  document.getElementById("logPill").textContent = `${snapshot.log_count} logs`;
  const telemetryPill = document.getElementById("telemetryPill");
  if (telemetryPill) {
    telemetryPill.textContent = `${snapshot.telemetry_client_count || 0} telemetry client${snapshot.telemetry_client_count === 1 ? "" : "s"}`;
  }
  const online = state.statuses.filter(item => item.http_status_online).length;
  const statusPill = document.getElementById("statusPill");
  const expectedStatuses = Number(snapshot.status_target_count || state.statuses.length || 5);
  statusPill.textContent = `${online}/${expectedStatuses} HTTP online`;
  statusPill.className = `pill ${online >= expectedStatuses ? "good" : "warn"}`;
  const telemetryPorts = snapshot.telemetry_ports || [];
  const portOptions = document.getElementById("telemetryPortOptions");
  if (portOptions && telemetryPorts.length) {
    portOptions.innerHTML = telemetryPorts.map(port => `<option value="${esc(port)}"></option>`).join("");
  }
  const telemetryPortHint = document.getElementById("telemetryPortHint");
  if (telemetryPortHint) {
    const telemetryCounts = snapshot.telemetry_client_counts || {};
    const activeTelemetry = Object.entries(telemetryCounts)
      .filter(([, count]) => Number(count) > 0)
      .map(([source, count]) => `${source.replace("telemetry:", "")}: ${count}`)
      .join(", ");
    const logCounts = snapshot.client_counts || {};
    const activeLogs = Object.entries(logCounts)
      .filter(([, count]) => Number(count) > 0)
      .map(([source, count]) => `${source.replace("log:", "")}: ${count}`)
      .join(", ");
    const listenText = telemetryPorts.length
      ? `listening: ${telemetryPorts.join(", ")}`
      : "";
    const activeText = [
      activeTelemetry ? `telemetry clients: ${activeTelemetry}` : "",
      activeLogs ? `log clients: ${activeLogs}` : "",
    ].filter(Boolean).join(" | ");
    telemetryPortHint.textContent = [listenText, activeText].filter(Boolean).join(" | ");
  }
  const rows = document.getElementById("infoRows");
  rows.innerHTML = state.statuses.map(item => `
    <tr class="${statusIsFresh(item) ? "" : "status-stale"}">
      <td>${renderModuleCell(item)}</td>
      <td>${renderWifiCell(item)}</td>
      <td>${esc(item.runtime_mode_name)}<br>tag ${esc(item.runtime_tag_id)} anchors ${(item.runtime_anchor_ids || []).join(",")}</td>
      <td>UWB <span class="${item.runtime_uwb_enabled ? "ok" : "muted"}">${item.runtime_uwb_enabled ? "on" : "off"}</span><br>BNO085 <span class="${item.runtime_bno085_accel_enabled ? "ok" : "muted"}">${item.runtime_bno085_accel_enabled ? "on" : "off"}</span><br><span class="muted">${esc(item.runtime_bno085_accel_interval_ms || "-")}/${esc(item.runtime_bno085_log_interval_ms || "-")} ms</span><br>GPS <span class="${item.runtime_gps_enabled ? "ok" : "muted"}">${item.runtime_gps_enabled ? "on" : "off"}</span></td>
      <td>${renderGpsCell(item)}</td>
      <td>${esc(item.uwb_status)}<br>tx ${esc(item.uwb_tx_count)} / rx ${esc(item.uwb_rx_count)}<br>err ${esc(item.uwb_tx_error_count)}/${esc(item.uwb_rx_error_count)}</td>
      <td>${esc(item.uwb_active_antenna_delay_hex)}<br><span class="muted">NVS ${item.uwb_antenna_delay_from_nvs ? "yes" : "no"}</span></td>
      <td>log ${esc(item.wireless_log_status)}<br>dropped ${esc(item.wireless_log_dropped)}<br>tel ${esc(item.wireless_telemetry_status || "-")}<br>port ${esc(item.wireless_telemetry_port ?? item.runtime_wireless_telemetry_port ?? "-")}<br>tel drop ${esc(item.wireless_telemetry_dropped ?? "-")}<br><span class="muted">full ${esc(item.wireless_telemetry_drop_full ?? "-")} · mutex ${esc(item.wireless_telemetry_drop_mutex ?? "-")} · fmt ${esc(item.wireless_telemetry_drop_format ?? "-")}<br>qmax ${esc(item.wireless_telemetry_queue_high_water ?? "-")}<br>bin ${esc(item.wireless_telemetry_binary_frames ?? "-")}f / ${esc(item.wireless_telemetry_binary_samples ?? "-")}s · text ${esc(item.wireless_telemetry_text_frames ?? "-")}</span><br>tel err ${esc(item.wireless_telemetry_last_error ?? "-")}<br>age ${fmtAge(item.status_updated_at)}</td>
      <td>${renderResourceCell(item)}</td>
      <td>${renderBatteryCell(item)}</td>
    </tr>`).join("");
  const freshStatus = state.statuses.find(statusIsFresh) || {};
  renderUwbRadio(freshStatus);
  renderCharger(state.statuses);
  renderPd(state.statuses);
  renderPosition();
  scheduleAccelRender();
  hydrateSettingsFromStatus(freshStatus);
  hydrateChargerSettings();
  hydratePdSettings();
}

function setSettingIfFresh(id, value, force = false) {
  const el = document.getElementById(id);
  if (!el || value === undefined || value === null) return;
  if (!force && localStorage.getItem(settingKey(id)) !== null) return;
  const token = el.type === "checkbox" ? (Boolean(value) ? "1" : "0") : String(value);
  if (el.type === "checkbox") {
    el.checked = Boolean(value);
  } else {
    el.value = value;
  }
  if (force) {
    localStorage.setItem(settingKey(id), token);
  }
}

function hydrateSettingsFromStatus(item) {
  if (!item.module_id) return;
  setSettingIfFresh(
    "accelSampleHz",
    intervalMsToHz(item.runtime_bno085_accel_interval_ms)
  );
  if (state.hydratedSettings) return;
  state.hydratedSettings = true;
  setSettingIfFresh("runtimeMode", String(item.runtime_mode_name || "").replace(/^uwb_/, "").replace("_test", "").replace("anchor_survey", "survey").replace("beacon_smoke", "beacon"));
  setSettingIfFresh("runtimeTag", item.runtime_tag_id);
  setSettingIfFresh("runtimeAnchors", (item.runtime_anchor_ids || []).filter(Boolean).join(","));
  setSettingIfFresh("runtimeUwb", item.runtime_uwb_enabled);
  setSettingIfFresh("runtimeBno085", item.runtime_bno085_accel_enabled);
  setSettingIfFresh("runtimeGps", item.runtime_gps_enabled);
  setSettingIfFresh("runtimeTelemetryPort", item.runtime_wireless_telemetry_port || item.wireless_telemetry_port);
  setSettingIfFresh("uwbRadioChannel", item.runtime_radio_channel || item.uwb_radio_channel);
  setSettingIfFresh("uwbSurveyRxMs", item.runtime_anchor_survey_rx_slice_ms);
  setSettingIfFresh("uwbSurveyDelayMs", item.runtime_anchor_survey_command_delay_ms);
  setSettingIfFresh("uwbSurveySlotMs", item.runtime_anchor_survey_slot_ms);
  setSettingIfFresh("uwbSurveyGapMs", item.runtime_anchor_survey_round_gap_ms);
  setSettingIfFresh("uwbSurveyLogEvery", item.runtime_anchor_survey_passive_tag_log_every);
  setSettingIfFresh("uwbRangingSlotMs", item.runtime_ranging_slot_ms);
  setSettingIfFresh("uwbRangingGapMs", item.runtime_ranging_round_gap_ms);
  setSettingIfFresh("uwbRangingRxMs", item.runtime_ranging_rx_slice_ms);
  setSettingIfFresh("uwbDtInitiator", item.runtime_distance_test_initiator_id);
  setSettingIfFresh("uwbDtResponder", item.runtime_distance_test_responder_id);
  setSettingIfFresh("uwbDtIntervalMs", item.runtime_distance_test_interval_ms);
  setSettingIfFresh("uwbDtRxTimeoutMs", item.runtime_distance_test_rx_timeout_ms, true);
  setSettingIfFresh("uwbDtRespDelayMs", item.runtime_distance_test_resp_delay_ms);
  setSettingIfFresh("uwbDtFinalDelayMs", item.runtime_distance_test_final_delay_ms);
  setSettingIfFresh("uwbDtReportDelayMs", item.runtime_distance_test_report_delay_ms);
  setSettingIfFresh("uwbDtAutoRxDelayUus", item.runtime_distance_test_auto_rx_delay_uus);
  setSettingIfFresh("uwbCalSummary", item.runtime_calibration_summary_every, true);
  setSettingIfFresh("uwbCalMinMs", item.runtime_calibration_min_interval_ms, true);
  setSettingIfFresh("uwbCalGuardUs", item.runtime_calibration_slot_guard_us, true);
  setSettingIfFresh("uwbCalMaxMs", item.runtime_calibration_max_interval_ms, true);
  setSettingIfFresh("uwbCalRxMs", item.runtime_calibration_rx_slice_ms, true);
  setSettingIfFresh("uwbAntennaDelayHex", item.uwb_configured_antenna_delay_hex || item.uwb_active_antenna_delay_hex);
}

function renderUwbRadio(item) {
  const rows = document.getElementById("uwbRadioRows");
  if (!rows) return;
  const fields = [
    ["profile", item.uwb_radio_profile],
    ["channel", item.uwb_radio_channel],
    ["rf channel bit", item.uwb_radio_rf_channel_bit],
    ["preamble len code", item.uwb_radio_preamble_len_code],
    ["preamble code", item.uwb_radio_preamble_code],
    ["PAC", item.uwb_radio_pac],
    ["data rate", item.uwb_radio_data_rate],
    ["PHR mode/rate", `${item.uwb_radio_phr_mode ?? "-"} / ${item.uwb_radio_phr_rate ?? "-"}`],
    ["SFD type", item.uwb_radio_sfd_type],
    ["TX PG delay", item.uwb_radio_tx_pg_delay],
    ["TX power", item.uwb_radio_tx_power],
    ["STS mode", item.uwb_sts_mode],
    ["STS length", item.uwb_sts_length_symbols],
    ["diagnostics every", item.uwb_diagnostics_enabled ? item.uwb_diagnostics_log_every : "off"],
    ["event counters every", item.uwb_event_counters_enabled ? item.uwb_event_counters_log_every : "off"],
  ];
  rows.innerHTML = fields.map(([key, value]) => `<tr><th>${esc(key)}</th><td>${esc(value ?? "-")}</td></tr>`).join("");
}

async function fetchSnapshot() {
  const res = await fetch("/api/snapshot", {cache: "no-store"});
  renderInfo(await res.json());
}

function parseCalibrationIds(inputId, expected) {
  const raw = document.getElementById(inputId)?.value || "";
  const ids = raw.split(/[,\s]+/)
    .map(value => value.trim())
    .filter(Boolean);
  while (ids.length < expected) ids.push("?");
  return ids.slice(0, expected);
}

function updateCalVisibility() {
  const method = document.getElementById("calMethod").value;
  document.querySelectorAll(".two-only").forEach(el => el.style.display = method === "two" ? "" : "none");
  document.querySelectorAll(".three-only").forEach(el => el.style.display = method === "three" ? "" : "none");
  renderDiagram();
}

function renderDiagram() {
  const method = document.getElementById("calMethod").value;
  const d = document.getElementById("calDiagram");
  if (method === "two") {
    const ids = parseCalibrationIds("calPair", 2);
    d.innerHTML = `<svg viewBox="0 0 520 320" role="img">
      <line class="edge" x1="140" y1="160" x2="380" y2="160"></line>
      <text class="edge-label" x="235" y="142">${esc(document.getElementById("calKnownCm").value)} cm</text>
      <rect class="node" x="100" y="125" width="80" height="70" rx="8"></rect>
      <rect class="node" x="340" y="125" width="80" height="70" rx="8"></rect>
      <text x="124" y="166" font-size="18" font-weight="700">M${esc(ids[0])}</text>
      <text x="364" y="166" font-size="18" font-weight="700">M${esc(ids[1])}</text>
      <text x="112" y="212" font-size="12" fill="#667085">reference</text>
      <text x="368" y="212" font-size="12" fill="#667085">DUT</text>
    </svg>`;
  } else {
    const ids = parseCalibrationIds("calThree", 3);
    d.innerHTML = `<svg viewBox="0 0 520 320" role="img">
      <line class="edge" x1="260" y1="65" x2="110" y2="245"></line>
      <line class="edge" x1="260" y1="65" x2="410" y2="245"></line>
      <line class="edge" x1="110" y1="245" x2="410" y2="245"></line>
      <text class="edge-label" x="118" y="145">${esc(document.getElementById("calD01Cm").value)} cm</text>
      <text class="edge-label" x="342" y="145">${esc(document.getElementById("calD02Cm").value)} cm</text>
      <text class="edge-label" x="240" y="282">${esc(document.getElementById("calD12Cm").value)} cm</text>
      <rect class="node" x="220" y="30" width="80" height="70" rx="8"></rect>
      <rect class="node" x="70" y="210" width="80" height="70" rx="8"></rect>
      <rect class="node" x="370" y="210" width="80" height="70" rx="8"></rect>
      <text x="244" y="71" font-size="18" font-weight="700">M${esc(ids[0] || "?")}</text>
      <text x="94" y="251" font-size="18" font-weight="700">M${esc(ids[1] || "?")}</text>
      <text x="394" y="251" font-size="18" font-weight="700">M${esc(ids[2] || "?")}</text>
    </svg>`;
  }
}

function moduleLabelForResult(result) {
  const target = String(result?.target || "");
  const status = state.statuses.find(item =>
    String(item.target || "") === target ||
    String(item.ip || "") === target ||
    target.endsWith(String(item.ip || ""))
  );
  return status?.module_id ? `M${status.module_id}` : (target || "?");
}

function resultPayloadOk(result) {
  if (!result?.ok) return false;
  if (!result.body) return true;
  try {
    const parsed = JSON.parse(result.body);
    return parsed.ok !== false;
  } catch (_) {
    return true;
  }
}

function summarizeApiResponse(data) {
  if (data?.summary) return data.summary;
  if (!data || data.ok === false && !Array.isArray(data.results)) {
    return data?.error ? `ERROR: ${data.error}` : "ERROR";
  }
  const results = data.results || [];
  if (!results.length) return data.ok ? "OK" : "ERROR";
  const labels = results.map(moduleLabelForResult);
  const okFlags = results.map(resultPayloadOk);
  const moduleIds = labels
    .map(label => /^M(\d+)$/.exec(label)?.[1])
    .filter(Boolean)
    .map(Number)
    .sort((a, b) => a - b);
  const allOk = okFlags.every(Boolean);
  if (allOk && moduleIds.length === results.length && moduleIds.length > 1) {
    const contiguous = moduleIds.every((value, index) =>
      index === 0 || value === moduleIds[index - 1] + 1
    );
    if (contiguous) return `M${moduleIds[0]}-${moduleIds[moduleIds.length - 1]} OK`;
  }
  return results.map((result, index) =>
    `${labels[index]} ${okFlags[index] ? "OK" : "ERROR"}`
  ).join(" · ");
}

function apiResponseOk(data) {
  if (!data || data.ok === false) return false;
  const results = data.results || [];
  return !results.length || results.every(resultPayloadOk);
}

function loadCalibrationResult() {
  try {
    return JSON.parse(localStorage.getItem("uwbDash.calibrationResult") || "null");
  } catch (_) {
    return null;
  }
}

function setCalibrationResult(result, persist = false) {
  state.calibrationResult = result ? {...result, ui_updated_ms: Date.now()} : null;
  if (persist) {
    if (state.calibrationResult) {
      localStorage.setItem("uwbDash.calibrationResult", JSON.stringify(state.calibrationResult));
    } else {
      localStorage.removeItem("uwbDash.calibrationResult");
    }
  }
  renderCalibrationResult();
}

function fmtFixed(value, digits = 2) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "-";
  return number.toFixed(digits);
}

function fmtCmFromM(value, digits = 1) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "-";
  return (number * 100).toFixed(digits);
}

function fmtMaybe(value) {
  if (value === undefined || value === null || value === "") return "-";
  if (Array.isArray(value)) return value.join(", ");
  if (typeof value === "boolean") return value ? "yes" : "no";
  return String(value);
}

function sortedPairEntries(value) {
  return Object.entries(value || {}).sort(([left], [right]) => {
    const parse = text => (String(text).match(/\d+/g) || []).map(item => Number(item));
    const a = parse(left);
    const b = parse(right);
    return (a[0] - b[0]) || ((a[1] || 0) - (b[1] || 0)) || left.localeCompare(right);
  });
}

function calTable(headers, rows) {
  if (!rows.length) return "";
  return `<table class="cal-result-table"><thead><tr>${
    headers.map(header => `<th>${esc(header)}</th>`).join("")
  }</tr></thead><tbody>${
    rows.map(row => `<tr>${row.map(cell => `<td>${cell}</td>`).join("")}</tr>`).join("")
  }</tbody></table>`;
}

function calBlock(title, html) {
  if (!html) return "";
  return `<div class="cal-result-block"><h3>${esc(title)}</h3>${html}</div>`;
}

function renderCalibrationResult() {
  const root = document.getElementById("calResults");
  if (!root) return;
  const envelope = state.calibrationResult;
  if (!envelope) {
    root.innerHTML = `<div class="muted">No calibration result yet</div>`;
    return;
  }
  const result = envelope.result || envelope;
  const running = Boolean(envelope.running);
  const statusText = running
    ? (envelope.state || "running")
    : result.cancelled
      ? "cancelled"
      : result.ok === false
        ? "error"
        : result.ok === true
          ? "ok"
          : (result.state || "-");
  const statusClass = result.ok === false || result.cancelled
    ? "bad"
    : running
      ? "warn"
      : result.ok === true
        ? "ok"
        : "";
  const ids = result.ids || result.participants || result.adjust_ids || [];
  const summaryRows = [
    ["Status", `<span class="${statusClass}">${esc(statusText)}</span>`],
    ["Summary", esc(result.summary || envelope.summary || "-")],
    ["Method", esc(result.method || "-")],
    ["Modules", esc(fmtMaybe(ids))],
    ["Adjust", esc(fmtMaybe(result.adjust_ids))],
    ["Samples", esc(fmtMaybe(result.sample_count))],
    ["Complete", esc(fmtMaybe(result.complete))],
    ["Valid", esc(fmtMaybe(result.valid))],
    ["Sync misses", esc(fmtMaybe(result.sync_miss_count))],
    ["Reference guard", esc(fmtMaybe(result.reference_guard_ok))],
    ["Max residual", result.fit_max_abs_residual_cm === undefined ? "-" : `${esc(fmtFixed(result.fit_max_abs_residual_cm, 2))} cm`],
  ].filter(([, value]) => value !== "-" && value !== "");

  const blocks = [
    calBlock("Summary", calTable(["Field", "Value"], summaryRows)),
  ];

  const applyRows = (result.apply_results || []).map(item => [
    esc(item.module_id ? `M${item.module_id}` : moduleLabelForResult(item)),
    esc(item.old_delay ?? "-"),
    esc(item.correction_dtu ?? "-"),
    esc(item.new_delay ?? "-"),
    item.applied ? `<span class="ok">written</span>` : `<span class="warn">${esc(item.reason || "not written")}</span>`,
  ]);
  if (applyRows.length) {
    blocks.push(calBlock("Antenna Delay Writes", calTable(
      ["Module", "Old", "Correction DTU", "New", "Result"],
      applyRows
    )));
  } else if (result.corrections) {
    const correctionRows = Object.entries(result.corrections).map(([moduleId, item]) => [
      esc(`M${moduleId}`),
      esc(item.float_dtu ?? "-"),
      esc(item.applied_dtu ?? item ?? "-"),
    ]);
    blocks.push(calBlock("Computed Corrections", calTable(
      ["Module", "Fit DTU", "Rounded DTU"],
      correctionRows
    )));
  }

  const pairRows = sortedPairEntries(result.pairs).map(([pair, item]) => [
    esc(pair),
    `${esc(fmtCmFromM(item.known_m))} cm`,
    `${esc(fmtCmFromM(item.center_m))} cm`,
    `${esc(fmtCmFromM(item.mean_m))} cm`,
    `${esc(fmtFixed(item.error_cm, 2))} cm`,
    esc(fmtFixed(item.error_dtu, 2)),
  ]);
  blocks.push(calBlock("Pair Geometry", calTable(
    ["Pair", "Known", "Median", "Mean", "Error", "Error DTU"],
    pairRows
  )));

  const directedRows = sortedPairEntries(result.directed).map(([pair, item]) => [
    esc(pair),
    esc(item.count ?? "-"),
    `${esc(fmtCmFromM(item.median_m))} cm`,
    `${esc(fmtCmFromM(item.mean_m))} cm`,
    `${esc(fmtCmFromM(item.std_m))} cm`,
    `${esc(fmtCmFromM(item.min_m))}-${esc(fmtCmFromM(item.max_m))} cm`,
  ]);
  blocks.push(calBlock("Directed Samples", calTable(
    ["Direction", "n", "Median", "Mean", "Std", "Range"],
    directedRows
  )));

  const fitRows = sortedPairEntries(result.directed_fit).map(([pair, item]) => [
    esc(pair),
    esc(fmtFixed(item.error_dtu, 2)),
    esc(fmtFixed(item.fit_dtu, 2)),
    esc(fmtFixed(item.residual_dtu, 2)),
    `${esc(fmtFixed(item.residual_cm, 2))} cm`,
  ]);
  blocks.push(calBlock("Directed Fit", calTable(
    ["Direction", "Error DTU", "Fit DTU", "Residual DTU", "Residual"],
    fitRows
  )));

  const guardRows = sortedPairEntries(result.reference_guard_failures).map(([pair, item]) => [
    esc(pair),
    `${esc(fmtFixed(item.error_cm, 2))} cm`,
    esc(fmtFixed(item.error_dtu, 2)),
  ]);
  blocks.push(calBlock("Reference Guard", calTable(
    ["Direction", "Error", "Error DTU"],
    guardRows
  )));

  const stopItems = [
    ...(result.stop_results || []),
    ...(result.cleanup_results || []),
    ...(result.results && result.summary?.includes("stop") ? result.results : []),
  ];
  const stopRows = stopItems.map(item => [
    esc(moduleLabelForResult(item)),
    item.ok ? `<span class="ok">OK</span>` : `<span class="bad">ERROR</span>`,
    esc(item.status ?? "-"),
    esc(item.error || item.reason || "-"),
  ]);
  blocks.push(calBlock("UWB Stop", calTable(
    ["Target", "Result", "HTTP", "Detail"],
    stopRows
  )));

  root.innerHTML = blocks.join("") || `<div class="muted">Calibration is running...</div>`;
}

function setToast(toastId, message, kind = "", detail = null, autoClear = true) {
  const toast = document.getElementById(toastId);
  if (!toast) return;
  if (toastTimers.has(toastId)) {
    clearTimeout(toastTimers.get(toastId));
    toastTimers.delete(toastId);
  }
  toast.className = `toast ${kind}`.trim();
  toast.textContent = message;
  toast.title = detail ? JSON.stringify(detail, null, 2) : "";
  if (autoClear) {
    toastTimers.set(toastId, setTimeout(() => {
      toast.textContent = "";
      toast.title = "";
      toast.className = "toast";
      toastTimers.delete(toastId);
    }, 4500));
  }
}

async function postJsonEndpoint(path, payload, toastId, autoClear = true) {
  setToast(toastId, "sending...", "", null, false);
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    const ok = apiResponseOk(data);
    setToast(toastId, summarizeApiResponse(data), ok ? "ok" : "bad", data, autoClear);
    return data;
  } catch (error) {
    const data = {ok: false, error: String(error)};
    setToast(toastId, summarizeApiResponse(data), "bad", data, autoClear);
    return data;
  }
}

async function postConfig(payload, toastId) {
  return postJsonEndpoint("/api/runtime-config", payload, toastId);
}

async function postCalibrationAuto(payload, toastId) {
  const button = document.getElementById("autoCalibration");
  const cancelButton = document.getElementById("cancelCalibration");
  if (button) button.disabled = true;
  if (cancelButton) cancelButton.disabled = true;
  setCalibrationResult({running: true, state: "starting", summary: "starting calibration..."});
  setToast(toastId, "starting calibration...", "", null, false);
  try {
    const res = await fetch("/api/calibration-auto", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!data.ok || !data.job_id) {
      setCalibrationResult(data, true);
      setToast(toastId, summarizeApiResponse(data), "bad", data, false);
      if (button) button.disabled = false;
      if (cancelButton) cancelButton.disabled = true;
      return data;
    }
    calibrationJobId = data.job_id;
    if (cancelButton) cancelButton.disabled = false;
    setCalibrationResult(data);
    setToast(toastId, data.summary || "calibration running...", "", data, false);
    pollCalibrationAuto(data.job_id, toastId, button);
    return data;
  } catch (error) {
    const data = {ok: false, error: String(error)};
    setCalibrationResult(data, true);
    setToast(toastId, summarizeApiResponse(data), "bad", data, false);
    if (button) button.disabled = false;
    if (cancelButton) cancelButton.disabled = true;
    return data;
  }
}

async function postCalibrationCancel(toastId) {
  const cancelButton = document.getElementById("cancelCalibration");
  if (cancelButton) cancelButton.disabled = true;
  setToast(toastId, "cancelling calibration...", "", null, false);
  try {
    const res = await fetch("/api/calibration-auto/cancel", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({job_id: calibrationJobId || ""}),
    });
    const data = await res.json();
    setCalibrationResult(data, true);
    setToast(toastId, summarizeApiResponse(data), data.ok ? "ok" : "bad", data, false);
    fetchSnapshot();
    return data;
  } catch (error) {
    const data = {ok: false, error: String(error)};
    setCalibrationResult(data, true);
    setToast(toastId, summarizeApiResponse(data), "bad", data, false);
    return data;
  }
}

async function pollCalibrationAuto(jobId, toastId, button) {
  const cancelButton = document.getElementById("cancelCalibration");
  if (calibrationPollTimer) {
    clearTimeout(calibrationPollTimer);
    calibrationPollTimer = null;
  }
  try {
    const res = await fetch(`/api/calibration-auto/status?job_id=${encodeURIComponent(jobId)}`, {cache: "no-store"});
    const data = await res.json();
    const done = !data.running && (data.state === "done" || data.state === "error" || data.state === "cancelled");
    if (done) {
      const result = data.result || data;
      const ok = apiResponseOk(result);
      setCalibrationResult(result, true);
      setToast(toastId, summarizeApiResponse(result), ok ? "ok" : "bad", result, false);
      if (button) button.disabled = false;
      if (cancelButton) cancelButton.disabled = true;
      calibrationJobId = null;
      fetchSnapshot();
      return;
    }
    calibrationJobId = jobId;
    if (cancelButton) cancelButton.disabled = Boolean(data.cancel_requested);
    setCalibrationResult(data);
    setToast(toastId, data.summary || "calibration running...", "", data, false);
    calibrationPollTimer = setTimeout(() => pollCalibrationAuto(jobId, toastId, button), 1000);
  } catch (error) {
    const data = {ok: false, error: String(error)};
    setCalibrationResult(data, true);
    setToast(toastId, summarizeApiResponse(data), "bad", data, false);
    if (button) button.disabled = false;
    if (cancelButton) cancelButton.disabled = true;
  }
}

async function postAntennaDelay(payload, toastId) {
  return postJsonEndpoint("/api/antenna-delay", payload, toastId);
}

async function postChargerConfig(payload, toastId) {
  return postJsonEndpoint("/api/charger-config", payload, toastId);
}

async function postMax77958Config(payload, toastId) {
  return postJsonEndpoint("/api/max77958-config", payload, toastId);
}

function settingKey(id) { return `uwbDash.setting.${id}`; }

const chargerAdcFields = [
  {id: "chargerAdcEnabled", param: "adc", label: "ADC", get: item => item.charger_adc_enabled},
  {id: "chargerAdcRate", param: "adc_rate", label: "ADC rate", get: item => item.charger_adc_continuous ? "continuous" : "oneshot"},
  {id: "chargerAdcSample", param: "adc_sample", label: "ADC sample", get: item => item.charger_adc_sample ?? 2},
  {id: "chargerAdcAvg", param: "adc_avg", label: "ADC average", get: item => item.charger_adc_running_average},
];
const chargerLimitFields = [
  {id: "chargerChargeEnabled", param: "charge_enabled", label: "Charging", get: item => item.charger_charge_enabled},
  {id: "chargerMinimalSystemMv", param: "minimal_system_voltage_mv", label: "VSYSMIN", get: item => item.charger_minimal_system_voltage_mv},
  {id: "chargerChargeVoltageMv", param: "charge_voltage_mv", label: "Charge voltage", get: item => item.charger_charge_voltage_limit_mv},
  {id: "chargerChargeCurrentMa", param: "charge_current_ma", label: "Charge current", get: item => item.charger_charge_current_limit_ma},
  {id: "chargerInputVoltageMv", param: "input_voltage_mv", label: "VINDPM", get: item => item.charger_input_voltage_limit_mv},
  {id: "chargerInputCurrentMa", param: "input_current_ma", label: "IINDPM", get: item => item.charger_input_current_limit_ma},
  {id: "chargerExtIlimEnabled", param: "external_input_current_limit_enabled", label: "ILIM_HIZ clamp", get: chargerExternalIlimEnabled},
];
const chargerTerminationFields = [
  {id: "chargerTerminationEnabled", param: "termination_enabled", label: "Termination", get: item => chargerTerminationConfig(item).terminationEnabled},
  {id: "chargerTerminationCurrentMa", param: "termination_current_ma", label: "ITERM", get: item => chargerTerminationConfig(item).terminationCurrentMa},
  {id: "chargerRechargeOffsetMv", param: "recharge_threshold_offset_mv", label: "VRECHG offset", get: item => chargerTerminationConfig(item).rechargeOffsetMv},
  {id: "chargerRechargeDeglitchMs", param: "recharge_deglitch_ms", label: "TRECHG", get: item => chargerTerminationConfig(item).rechargeDeglitchMs},
];
const chargerTimerFields = [
  {id: "chargerFastTimerEnabled", param: "fast_charge_timer_enabled", label: "Fast timer", get: item => chargerTimerConfig(item).fastEnabled},
  {id: "chargerFastTimerHours", param: "fast_charge_timer_hours", label: "Fast duration", get: item => chargerTimerConfig(item).fastHours},
  {id: "chargerPrechargeTimerEnabled", param: "precharge_timer_enabled", label: "Pre-charge timer", get: item => chargerTimerConfig(item).prechargeEnabled},
  {id: "chargerPrechargeTimerMinutes", param: "precharge_timer_minutes", label: "Pre-charge duration", get: item => chargerTimerConfig(item).prechargeMinutes},
  {id: "chargerTrickleTimerEnabled", param: "trickle_timer_enabled", label: "Trickle timer", get: item => chargerTimerConfig(item).trickleEnabled},
  {id: "chargerTopoffTimerMinutes", param: "topoff_timer_minutes", label: "Top-off timer", get: item => chargerTimerConfig(item).topoffMinutes},
  {id: "chargerTimer2xEnabled", param: "timer_2x_enabled", label: "TMR2X", get: item => chargerTimerConfig(item).timer2xEnabled},
];

function chargerConfigFields() {
  return [...chargerAdcFields, ...chargerLimitFields, ...chargerTerminationFields, ...chargerTimerFields];
}

function legacyChargerConfigSettingIds() {
  return chargerConfigFields().map(field => field.id);
}

function clearLegacyChargerConfigSettings() {
  for (const id of legacyChargerConfigSettingIds()) {
    localStorage.removeItem(settingKey(id));
  }
}

function controlValueToken(value) {
  if (value === undefined || value === null) return "";
  if (typeof value === "boolean") return value ? "1" : "0";
  return String(value);
}

function chargerStatusPresent(item) {
  return item && item.module_id && item.charger_present !== false &&
    (item.charger_reg0f_charger_control_0 !== undefined ||
     item.charger_charge_enabled !== undefined ||
     item.charger_adc_enabled !== undefined);
}

function selectedChargerStatuses(targetSelectId) {
  const target = document.getElementById(targetSelectId)?.value || "all";
  const items = state.statuses.filter(chargerStatusPresent);
  if (target === "all") return items;
  const moduleId = Number(target);
  return items.filter(item => Number(item.module_id) === moduleId);
}

function commonChargerValue(items, field) {
  if (!items.length) return {mixed: false, value: undefined};
  const values = items.map(item => field.get(item));
  if (values.some(value => value === undefined || value === null)) {
    return {mixed: true, value: undefined};
  }
  const first = controlValueToken(values[0]);
  const mixed = values.some(value => controlValueToken(value) !== first);
  return {mixed, value: mixed ? undefined : values[0]};
}

function setChargerControlFromStatus(field, value, mixed) {
  const el = document.getElementById(field.id);
  if (!el || el.dataset.dirty === "1") return;
  el.dataset.mixed = mixed ? "1" : "0";
  el.dataset.loadedValue = mixed ? "" : controlValueToken(value);
  el.classList.toggle("mixed-value", mixed);
  if (el.type === "checkbox") {
    el.indeterminate = mixed;
    if (!mixed) el.checked = Boolean(value);
    return;
  }
  if (mixed) {
    el.value = "";
    el.placeholder = "mixed";
  } else {
    el.value = value === undefined || value === null ? "" : String(value);
    el.placeholder = "";
  }
}

function hydrateChargerGroup(fields, targetSelectId, statusId) {
  const items = selectedChargerStatuses(targetSelectId);
  const target = document.getElementById(targetSelectId)?.value || "all";
  const status = document.getElementById(statusId);
  const mixedLabels = [];
  for (const field of fields) {
    const common = commonChargerValue(items, field);
    if (common.mixed) mixedLabels.push(field.label);
    setChargerControlFromStatus(field, common.value, common.mixed);
  }
  if (!status) return;
  if (!items.length) {
    status.textContent = `no live charger status for ${target === "all" ? "selected modules" : `M${target}`}`;
    status.className = "field-note warn";
    return;
  }
  const modules = items.map(item => `M${item.module_id}`).join(", ");
  if (mixedLabels.length) {
    status.textContent = `loaded ${modules}; mixed: ${mixedLabels.join(", ")}`;
    status.className = "field-note warn";
  } else {
    status.textContent = `loaded ${modules}`;
    status.className = "field-note";
  }
}

function hydrateChargerSettings() {
  hydrateChargerGroup(chargerAdcFields, "chargerAdcTargets", "chargerAdcSelectionStatus");
  hydrateChargerGroup(chargerLimitFields, "chargerLimitTargets", "chargerLimitSelectionStatus");
  hydrateChargerGroup(chargerTerminationFields, "chargerTerminationTargets", "chargerTerminationSelectionStatus");
  hydrateChargerGroup(chargerTimerFields, "chargerTimerTargets", "chargerTimerSelectionStatus");
}

function clearChargerDirty(fields = chargerConfigFields()) {
  for (const field of fields) {
    const el = document.getElementById(field.id);
    if (!el) continue;
    el.dataset.dirty = "0";
  }
}

function markChargerControlDirty(event) {
  const el = event.currentTarget;
  el.dataset.dirty = "1";
  el.dataset.mixed = "0";
  el.classList.remove("mixed-value");
  if (el.type === "checkbox") el.indeterminate = false;
}

function wireChargerDirtyTracking() {
  for (const field of chargerConfigFields()) {
    const el = document.getElementById(field.id);
    if (!el) continue;
    el.dataset.dirty = "0";
    el.addEventListener("input", markChargerControlDirty);
    el.addEventListener("change", markChargerControlDirty);
  }
  const groups = [
    {target: "chargerAdcTargets", fields: chargerAdcFields},
    {target: "chargerLimitTargets", fields: chargerLimitFields},
    {target: "chargerTerminationTargets", fields: chargerTerminationFields},
    {target: "chargerTimerTargets", fields: chargerTimerFields},
  ];
  for (const group of groups) {
    const targets = document.getElementById(group.target);
    if (!targets) continue;
    targets.addEventListener("change", () => {
      clearChargerDirty(group.fields);
      hydrateChargerSettings();
    });
  }
}

function readChargerControlParam(field) {
  const el = document.getElementById(field.id);
  if (!el || el.dataset.dirty !== "1") return null;
  if (el.type === "checkbox") {
    return {[field.param]: el.checked ? "1" : "0"};
  }
  if (!el.value || !el.checkValidity()) {
    return {error: `${field.label} is not valid`};
  }
  return {[field.param]: el.value};
}

function collectDirtyChargerParams(fields) {
  const params = {};
  const labels = [];
  for (const field of fields) {
    const item = readChargerControlParam(field);
    if (item === null) continue;
    if (item.error) return {error: item.error, params: {}, labels: []};
    params[field.param] = item[field.param];
    labels.push(field.label);
  }
  return {params, labels};
}

async function postDirtyChargerConfig(fields, targetSelectId, toastId) {
  const collected = collectDirtyChargerParams(fields);
  if (collected.error) {
    setToast(toastId, collected.error, "bad");
    return;
  }
  if (!Object.keys(collected.params).length) {
    setToast(toastId, "No charger changes to apply", "");
    return;
  }
  const data = await postChargerConfig({
    target_modules: document.getElementById(targetSelectId).value,
    params: collected.params,
  }, toastId);
  if (apiResponseOk(data)) {
    clearChargerDirty(fields);
    setTimeout(hydrateChargerSettings, 300);
  }
}

function delayMs(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function resetChargerCycle(targetSelectId, toastId) {
  const target_modules = document.getElementById(targetSelectId).value;
  setToast(toastId, "resetting charge cycle...", "", null, false);
  const off = await postChargerConfig({
    target_modules,
    params: {charge_enabled: "0"},
  }, toastId);
  if (!apiResponseOk(off)) return;
  await delayMs(350);
  const on = await postChargerConfig({
    target_modules,
    params: {charge_enabled: "1"},
  }, toastId);
  if (apiResponseOk(on)) {
    clearChargerDirty(chargerLimitFields);
    setTimeout(hydrateChargerSettings, 500);
  }
}

const pdContractFields = [
  {id: "pdSourcePdoPos", param: "source_pdo_pos", label: "Source PDO", get: item => item.pd_selected_source_pdo_pos || 1},
  {id: "pdUsb2SwitchClosed", param: "usb2_closed", label: "USB2 switch", get: item => item.pd_usb2_switch_closed},
];
const pdSinkFields = [
  {id: "pdSinkPdos", param: "sink_pdos", label: "Sink PDOs", get: item => pdPdoInputText(item.pd_sink_pdos)},
  {id: "pdSinkPdosMtp", param: "sink_pdos_mtp", label: "MTP", get: item => item.pd_sink_pdos_from_mtp},
];
const pdPpsFields = [
  {id: "pdPpsEnabled", param: "pps_enabled", label: "PPS enable", get: item => item.pd_pps_default_enabled},
  {id: "pdPpsVoltageMv", param: "pps_voltage_mv", label: "PPS voltage", get: item => item.pd_pps_default_mv || 5000},
  {id: "pdPpsCurrentMa", param: "pps_current_ma", label: "PPS current", get: item => item.pd_pps_default_ma || 1000},
];

function pdConfigFields() {
  return [...pdContractFields, ...pdSinkFields, ...pdPpsFields];
}

function selectedPdStatuses(targetSelectId) {
  const target = document.getElementById(targetSelectId)?.value || "all";
  const items = state.statuses.filter(pdStatusPresent);
  if (target === "all") return items;
  const moduleId = Number(target);
  return items.filter(item => Number(item.module_id) === moduleId);
}

function commonPdValue(items, field) {
  if (!items.length) return {mixed: false, value: undefined};
  const values = items.map(item => field.get(item));
  if (values.some(value => value === undefined || value === null)) {
    return {mixed: true, value: undefined};
  }
  const first = controlValueToken(values[0]);
  const mixed = values.some(value => controlValueToken(value) !== first);
  return {mixed, value: mixed ? undefined : values[0]};
}

function setPdControlFromStatus(field, value, mixed) {
  const el = document.getElementById(field.id);
  if (!el || el.dataset.dirty === "1") return;
  el.dataset.mixed = mixed ? "1" : "0";
  el.dataset.loadedValue = mixed ? "" : controlValueToken(value);
  el.classList.toggle("mixed-value", mixed);
  if (el.type === "checkbox") {
    el.indeterminate = mixed;
    if (!mixed) el.checked = Boolean(value);
    return;
  }
  if (mixed) {
    el.value = "";
    el.placeholder = "mixed";
  } else {
    el.value = value === undefined || value === null ? "" : String(value);
    el.placeholder = "";
  }
}

function hydratePdGroup(fields, targetSelectId, statusId) {
  const items = selectedPdStatuses(targetSelectId);
  const target = document.getElementById(targetSelectId)?.value || "all";
  const status = document.getElementById(statusId);
  const mixedLabels = [];
  for (const field of fields) {
    const common = commonPdValue(items, field);
    if (common.mixed) mixedLabels.push(field.label);
    setPdControlFromStatus(field, common.value, common.mixed);
  }
  if (!status) return;
  if (!items.length) {
    status.textContent = `no live MAX77958 status for ${target === "all" ? "selected modules" : `M${target}`}`;
    status.className = "field-note warn";
    return;
  }
  const modules = items.map(item => `M${item.module_id}`).join(", ");
  if (mixedLabels.length) {
    status.textContent = `loaded ${modules}; mixed: ${mixedLabels.join(", ")}`;
    status.className = "field-note warn";
  } else {
    status.textContent = `loaded ${modules}`;
    status.className = "field-note";
  }
}

function hydratePdSettings() {
  hydratePdGroup(pdContractFields, "pdContractTargets", "pdContractSelectionStatus");
  hydratePdGroup(pdSinkFields, "pdSinkTargets", "pdSinkSelectionStatus");
  hydratePdGroup(pdPpsFields, "pdPpsTargets", "pdPpsSelectionStatus");
}

function clearPdDirty(fields = pdConfigFields()) {
  for (const field of fields) {
    const el = document.getElementById(field.id);
    if (!el) continue;
    el.dataset.dirty = "0";
  }
}

function markPdControlDirty(event) {
  const el = event.currentTarget;
  el.dataset.dirty = "1";
  el.dataset.mixed = "0";
  el.classList.remove("mixed-value");
  if (el.type === "checkbox") el.indeterminate = false;
}

function wirePdDirtyTracking() {
  for (const field of pdConfigFields()) {
    const el = document.getElementById(field.id);
    if (!el) continue;
    el.dataset.dirty = "0";
    el.addEventListener("input", markPdControlDirty);
    el.addEventListener("change", markPdControlDirty);
  }
  const groups = [
    {target: "pdContractTargets", fields: pdContractFields},
    {target: "pdSinkTargets", fields: pdSinkFields},
    {target: "pdPpsTargets", fields: pdPpsFields},
  ];
  for (const group of groups) {
    const targets = document.getElementById(group.target);
    if (!targets) continue;
    targets.addEventListener("change", () => {
      clearPdDirty(group.fields);
      hydratePdSettings();
    });
  }
}

function pdParamFromControl(id) {
  const el = document.getElementById(id);
  if (!el) return "";
  if (el.type === "checkbox") return el.checked ? "1" : "0";
  return el.value.trim();
}

function pdPostPayload(targetSelectId, params) {
  return {
    target_modules: document.getElementById(targetSelectId).value,
    params,
  };
}

async function postPdConfig(targetSelectId, params, toastId, fieldsToClear = []) {
  const data = await postMax77958Config(pdPostPayload(targetSelectId, params), toastId);
  if (apiResponseOk(data)) {
    clearPdDirty(fieldsToClear);
    setTimeout(hydratePdSettings, 500);
    setTimeout(fetchSnapshot, 600);
  }
  return data;
}

function calibrationParamsFromForm() {
  const method = document.getElementById("calMethod").value;
  const params = {
    mode: "calibration",
    cal_method: method,
    cal_samples: document.getElementById("calSamples").value,
    cal_summary: document.getElementById("uwbCalSummary").value,
    cal_slot_ms: document.getElementById("uwbCalMinMs").value,
    cal_guard_us: document.getElementById("uwbCalGuardUs").value,
    cal_round_gap_ms: document.getElementById("uwbCalMaxMs").value,
    cal_rx_ms: document.getElementById("uwbCalRxMs").value,
    dt_rx_timeout_ms: document.getElementById("uwbDtRxTimeoutMs").value,
    reboot: "1",
  };
  if (method === "two") {
    const ids = parseCalibrationIds("calPair", 2);
    params.cal_ref = ids[0] === "?" ? "" : ids[0];
    params.cal_dut = ids[1] === "?" ? "" : ids[1];
    params.cal_known_cm = document.getElementById("calKnownCm").value;
  } else {
    params.cal_three = document.getElementById("calThree").value;
    params.cal_d01_cm = document.getElementById("calD01Cm").value;
    params.cal_d02_cm = document.getElementById("calD02Cm").value;
    params.cal_d12_cm = document.getElementById("calD12Cm").value;
  }
  return params;
}

function rangingProfileElementId(profileKey, suffix) {
  const profile = rangingProfileDefaults[profileKey];
  return profile ? `${profile.prefix}${suffix}` : "";
}

function rangingProfileIds() {
  const ids = ["rangingProfileTargets"];
  for (const profileKey of Object.keys(rangingProfileDefaults)) {
    for (const field of rangingProfileFields) {
      ids.push(rangingProfileElementId(profileKey, field.suffix));
    }
  }
  return ids;
}

function readRangingProfile(profileKey) {
  const result = {};
  for (const field of rangingProfileFields) {
    const id = rangingProfileElementId(profileKey, field.suffix);
    result[field.key] = Number(document.getElementById(id)?.value);
  }
  return result;
}

function writeRangingProfile(profileKey, values, persist = true) {
  for (const field of rangingProfileFields) {
    const id = rangingProfileElementId(profileKey, field.suffix);
    const el = document.getElementById(id);
    if (!el || values[field.key] === undefined) continue;
    el.value = String(values[field.key]);
    if (persist) localStorage.setItem(settingKey(id), el.value);
  }
  updateRangingProfileSummary(profileKey);
}

function rangingProfileRuntimeParams(values) {
  return {
    survey_slot_ms: String(values.slotMs),
    survey_gap_ms: String(values.roundGapMs),
    survey_rx_ms: String(values.rxSliceMs),
    survey_delay_ms: String(values.commandDelayMs),
    ranging_slot_ms: String(values.slotMs),
    ranging_gap_ms: String(values.roundGapMs),
    ranging_rx_ms: String(values.rxSliceMs),
    dt_rx_timeout_ms: String(values.timeoutMs),
    dt_resp_delay_ms: String(values.respDelayMs),
    dt_final_delay_ms: String(values.finalDelayMs),
    dt_report_delay_ms: String(values.reportDelayMs),
    dt_auto_rx_delay_uus: String(values.autoRxDelayUus),
  };
}

function mirrorRangingProfileToUwbFields(values) {
  const fields = {
    uwbSurveySlotMs: values.slotMs,
    uwbSurveyGapMs: values.roundGapMs,
    uwbSurveyRxMs: values.rxSliceMs,
    uwbSurveyDelayMs: values.commandDelayMs,
    uwbRangingSlotMs: values.slotMs,
    uwbRangingGapMs: values.roundGapMs,
    uwbRangingRxMs: values.rxSliceMs,
    uwbDtRxTimeoutMs: values.timeoutMs,
    uwbDtRespDelayMs: values.respDelayMs,
    uwbDtFinalDelayMs: values.finalDelayMs,
    uwbDtReportDelayMs: values.reportDelayMs,
    uwbDtAutoRxDelayUus: values.autoRxDelayUus,
  };
  for (const [id, value] of Object.entries(fields)) {
    const el = document.getElementById(id);
    if (!el || value === undefined || !Number.isFinite(Number(value))) continue;
    el.value = String(value);
    localStorage.setItem(settingKey(id), el.value);
  }
}

function profileSummaryText(values, anchorCount = 4) {
  const programmedDsTwrMs = profileProgrammedDsTwrMs(values);
  const classicProgrammedMs =
    values.respDelayMs + values.finalDelayMs + 2 * values.reportDelayMs;
  const marginMs = values.slotMs - programmedDsTwrMs;
  const pairCount = Math.max(1, anchorCount * (anchorCount - 1) / 2);
  const roundMs = pairCount * values.slotMs + values.roundGapMs;
  const bidirectionalMs = 2 * roundMs;
  const warnings = [];
  if (values.timeoutMs >= values.slotMs) warnings.push("timeout >= slot");
  if (values.rxSliceMs > values.slotMs) warnings.push("RX slice > slot");
  if (marginMs < 5) warnings.push("low slot margin");
  const warnText = warnings.length ? ` · ${warnings.join(", ")}` : "";
  return `${anchorCount} anchors: ${pairCount} pair slots · ~${fmtFixed(roundMs, 0)} ms/round · ~${fmtFixed(bidirectionalMs, 0)} ms bidir · programmed FlexTDOA ${fmtFixed(programmedDsTwrMs, 0)} ms · DS-TWR body ${fmtFixed(classicProgrammedMs, 0)} ms · margin ${fmtFixed(marginMs, 0)} ms${warnText}`;
}

function profileProgrammedDsTwrMs(values) {
  return values.commandDelayMs +
    values.respDelayMs +
    values.finalDelayMs +
    2 * values.reportDelayMs;
}

function updateRangingProfileSummary(profileKey) {
  const profile = rangingProfileDefaults[profileKey];
  if (!profile) return;
  const summary = document.getElementById(`${profile.prefix}Summary`);
  if (!summary) return;
  const values = readRangingProfile(profileKey);
  const valid = Object.values(values).every(value => Number.isFinite(value));
  summary.textContent = valid ? profileSummaryText(values, 4) : "incomplete profile";
  summary.className = `profile-summary ${valid && values.slotMs - profileProgrammedDsTwrMs(values) < 5 ? "warn" : ""}`.trim();
}

function updateAllRangingProfileSummaries() {
  Object.keys(rangingProfileDefaults).forEach(updateRangingProfileSummary);
}

async function applyRangingProfile(profileKey) {
  const profile = rangingProfileDefaults[profileKey];
  if (!profile) return;
  const values = readRangingProfile(profileKey);
  if (!Object.values(values).every(value => Number.isFinite(value) && value > 0)) {
    setToast("rangingProfileToast", "Profile has invalid values", "bad");
    return;
  }
  setToast("rangingProfileToast", `applying ${profile.label}...`, "", null, false);
  const data = await postConfig({
    target_modules: document.getElementById("rangingProfileTargets").value,
    params: rangingProfileRuntimeParams(values),
  }, "rangingProfileToast");
  if (apiResponseOk(data)) {
    mirrorRangingProfileToUwbFields(values);
    setTimeout(fetchSnapshot, 500);
  }
}

function persistedSettingIds() {
  return [
    "runtimeTargets", "runtimeMode", "runtimeTag", "runtimeAnchors", "runtimeReboot",
    "runtimeUwb", "runtimeBno085", "runtimeGps", "runtimeTelemetryPort",
    "accelTimebase", "accelSampleHz", "accelTargets",
    "positionAnchorCount", "positionSolver", "positionAnchors", "positionTags",
    "positionMaxAgeSec", "positionTdoaMode",
    "uwbTargets", "uwbRadioChannel", "uwbSurveyRxMs", "uwbSurveyDelayMs", "uwbSurveySlotMs",
    "uwbSurveyGapMs", "uwbSurveyLogEvery", "uwbRangingSlotMs",
    "uwbRangingGapMs", "uwbRangingRxMs", "uwbDtInitiator", "uwbDtResponder",
    "uwbDtIntervalMs", "uwbDtRxTimeoutMs", "uwbDtRespDelayMs",
    "uwbDtFinalDelayMs", "uwbDtReportDelayMs", "uwbDtAutoRxDelayUus",
    "uwbCalSummary", "uwbCalMinMs", "uwbCalGuardUs", "uwbCalMaxMs", "uwbCalRxMs",
    "uwbAntennaDelayHex", "uwbAdvancedReboot",
    "chargerAdcTargets", "chargerLimitTargets", "chargerTerminationTargets",
    "chargerTimerTargets",
    "chargerRawModule", "chargerShowRawTools", "chargerRawReg", "chargerRawValue",
    "chargerRawMask", "chargerRawBits",
    "pdContractTargets", "pdSinkTargets", "pdPpsTargets", "pdRawModule",
    "pdSourcePdoPos", "pdUsb2SwitchClosed", "pdSinkPdos", "pdSinkPdosMtp",
    "pdPpsEnabled", "pdPpsVoltageMv", "pdPpsCurrentMa", "pdApdoPos",
    "pdApdoVoltageMv", "pdApdoCurrentMa", "pdShowRawTools", "pdRawReg",
    "pdRawValue",
    "calTargets", "calMethod", "calPair", "calKnownCm", "calThree",
    "calD01Cm", "calD02Cm", "calD12Cm", "calSamples",
    "calAutoApply", "calMinApplyDtu", "calReferenceGuardCm", "calTimeoutSec",
    ...rangingProfileIds(),
  ];
}
function restoreSettings() {
  for (const id of persistedSettingIds()) {
    const el = document.getElementById(id);
    const key = settingKey(id);
    let saved = localStorage.getItem(key);
    if (!el || saved === null) continue;
    if (id === "calTimeoutSec" && saved === "180") {
      saved = "240";
      localStorage.setItem(key, saved);
    }
    if (el.type === "checkbox") {
      el.checked = saved === "1";
    } else {
      el.value = saved;
    }
  }
  migrateCalibrationPairSetting();
  migratePositionSolverSetting();
}

function migrateCalibrationPairSetting() {
  const pairEl = document.getElementById("calPair");
  if (!pairEl) return;
  const pairKey = settingKey("calPair");
  if (localStorage.getItem(pairKey) !== null) return;

  const ref = localStorage.getItem(settingKey("calRef"));
  const dut = localStorage.getItem(settingKey("calDut"));
  if (ref && dut) {
    pairEl.value = `${ref},${dut}`;
    localStorage.setItem(pairKey, pairEl.value);
  }
  localStorage.removeItem(settingKey("calRef"));
  localStorage.removeItem(settingKey("calDut"));
}

function migratePositionSolverSetting() {
  const legacyCoordsKey = settingKey("positionAnchorCoords");
  if (localStorage.getItem(legacyCoordsKey) === null) return;
  localStorage.removeItem(legacyCoordsKey);

  const solverKey = settingKey("positionSolver");
  const solverEl = document.getElementById("positionSolver");
  const savedSolver = localStorage.getItem(solverKey);
  if (savedSolver === null || savedSolver === "ranging") {
    if (solverEl) solverEl.value = "tdoa";
    localStorage.setItem(solverKey, "tdoa");
  }
}

function wireSettingPersistence() {
  for (const id of persistedSettingIds()) {
    const el = document.getElementById(id);
    if (!el) continue;
    const save = () => localStorage.setItem(
      settingKey(id),
      el.type === "checkbox" ? (el.checked ? "1" : "0") : el.value
    );
    el.addEventListener("input", save);
    el.addEventListener("change", save);
  }
}
function formatCmInput(el) {
  const number = Number(el.value);
  if (Number.isFinite(number)) el.value = number.toFixed(2);
}

function updateChargerRawVisibility() {
  const show = document.getElementById("chargerShowRawTools")?.checked;
  document.querySelectorAll(".charger-raw-tool").forEach(el => {
    el.classList.toggle("hidden", !show);
  });
}

function updatePdRawVisibility() {
  const show = document.getElementById("pdShowRawTools")?.checked;
  document.querySelectorAll(".pd-raw-tool").forEach(el => {
    el.classList.toggle("hidden", !show);
  });
}

async function enablePositionRanging() {
  const settings = positionSettings();
  const tagId = settings.tagIds[0];
  if (!tagId || settings.anchorIds.length < 3) {
    setToast("positionToast", "Set one tag ID and at least 3 anchors first.", "bad");
    return;
  }
  const anchors = settings.anchorIds.join(",");
  const mode = settings.solver === "tdoa" ? "flex_tdoa" : "ranging";
  const params = {
    mode,
    tag: String(tagId),
    anchors,
    uwb: "1",
    reboot: "1",
  };
  const runtimeMode = document.getElementById("runtimeMode");
  const runtimeTag = document.getElementById("runtimeTag");
  const runtimeAnchors = document.getElementById("runtimeAnchors");
  const runtimeUwb = document.getElementById("runtimeUwb");
  const runtimeReboot = document.getElementById("runtimeReboot");
  if (runtimeMode) runtimeMode.value = mode;
  if (runtimeTag) runtimeTag.value = String(tagId);
  if (runtimeAnchors) runtimeAnchors.value = anchors;
  if (runtimeUwb) runtimeUwb.checked = true;
  if (runtimeReboot) runtimeReboot.value = "1";
  await postConfig({target_modules: "all", params}, "positionToast");
  state.positionTrail = {};
  state.positionAnchorTrail = {};
  resetPositionFilters();
  setTimeout(fetchSnapshot, 1500);
}

function wireSettings() {
  clearLegacyChargerConfigSettings();
  restoreSettings();
  wireSettingPersistence();
  wireChargerDirtyTracking();
  wirePdDirtyTracking();
  updateChargerRawVisibility();
  updatePdRawVisibility();
  const chargerShowRawTools = document.getElementById("chargerShowRawTools");
  if (chargerShowRawTools) {
    chargerShowRawTools.addEventListener("change", updateChargerRawVisibility);
  }
  const pdShowRawTools = document.getElementById("pdShowRawTools");
  if (pdShowRawTools) {
    pdShowRawTools.addEventListener("change", updatePdRawVisibility);
  }
  const accelTimebase = document.getElementById("accelTimebase");
  if (accelTimebase) {
    state.timebaseSecPerDiv = Math.max(1, Number(accelTimebase.value || 5));
    accelTimebase.addEventListener("change", () => {
      state.timebaseSecPerDiv = Math.max(1, Number(accelTimebase.value || 5));
      renderAccelGraphs();
    });
  }
  ["positionAnchorCount", "positionSolver", "positionAnchors", "positionTags", "positionMaxAgeSec", "positionTdoaMode"].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("input", () => {
      resetPositionFilters();
      renderPosition();
    });
    el.addEventListener("change", () => {
      resetPositionFilters();
      renderPosition();
    });
  });
  document.getElementById("positionResetTrail").addEventListener("click", () => {
    state.positionTrail = {};
    state.positionAnchorTrail = {};
    resetPositionFilters();
    renderPosition();
  });
  document.getElementById("positionEnableRanging").addEventListener("click", enablePositionRanging);
  document.getElementById("positionEnableRangingSide").addEventListener("click", enablePositionRanging);
  document.querySelectorAll(".cm-input").forEach(el => {
    el.addEventListener("change", () => {
      formatCmInput(el);
      localStorage.setItem(settingKey(el.id), el.value);
      renderDiagram();
    });
  });
  document.getElementById("applyRuntime").addEventListener("click", () => {
    postConfig({
      target_modules: document.getElementById("runtimeTargets").value,
      params: {
        mode: document.getElementById("runtimeMode").value,
        tag: document.getElementById("runtimeTag").value,
        anchors: document.getElementById("runtimeAnchors").value,
        uwb: document.getElementById("runtimeUwb").checked ? "1" : "0",
        bno085: document.getElementById("runtimeBno085").checked ? "1" : "0",
        gps: document.getElementById("runtimeGps").checked ? "1" : "0",
        reboot: document.getElementById("runtimeReboot").value,
      }
    }, "runtimeToast");
  });
  document.getElementById("clearRuntime").addEventListener("click", () => {
    postConfig({params: {clear: "1", reboot: "1"}}, "runtimeToast");
  });
  document.getElementById("applyTelemetryPort").addEventListener("click", () => {
    postConfig({
      target_modules: document.getElementById("runtimeTargets").value,
      params: {
        telemetry_port: document.getElementById("runtimeTelemetryPort").value,
      }
    }, "telemetryPortToast");
  });
  document.querySelectorAll(".profile-card input").forEach(el => {
    el.addEventListener("input", () => {
      const profile = el.closest(".profile-card")?.dataset.profile;
      if (profile) updateRangingProfileSummary(profile);
    });
    el.addEventListener("change", () => {
      const profile = el.closest(".profile-card")?.dataset.profile;
      if (profile) updateRangingProfileSummary(profile);
    });
  });
  document.querySelectorAll(".apply-ranging-profile").forEach(button => {
    button.addEventListener("click", () => applyRangingProfile(button.dataset.profile));
  });
  document.querySelectorAll(".reset-ranging-profile").forEach(button => {
    button.addEventListener("click", () => {
      const profile = button.dataset.profile;
      writeRangingProfile(profile, rangingProfileDefaults[profile]);
      setToast("rangingProfileToast", "profile defaults restored locally; press Apply to write ESP NVS", "");
    });
  });
  document.getElementById("resetAllRangingProfiles").addEventListener("click", () => {
    for (const profile of Object.keys(rangingProfileDefaults)) {
      writeRangingProfile(profile, rangingProfileDefaults[profile]);
    }
    setToast("rangingProfileToast", "all profile defaults restored locally; press Apply to write ESP NVS", "");
  });
  updateAllRangingProfileSummaries();
  document.getElementById("applyCalibrationSettings").addEventListener("click", () => {
    postConfig({
      target_modules: document.getElementById("runtimeTargets").value,
      params: {
        cal_summary: document.getElementById("uwbCalSummary").value,
        cal_slot_ms: document.getElementById("uwbCalMinMs").value,
        cal_guard_us: document.getElementById("uwbCalGuardUs").value,
        dt_rx_timeout_ms: document.getElementById("uwbDtRxTimeoutMs").value,
        cal_round_gap_ms: document.getElementById("uwbCalMaxMs").value,
        cal_rx_ms: document.getElementById("uwbCalRxMs").value,
      }
    }, "calTimingToast");
  });
  document.getElementById("applyAccelSample").addEventListener("click", () => {
    const sampleHz = document.getElementById("accelSampleHz").value;
    postConfig({
      target_modules: document.getElementById("accelTargets").value,
      params: {
        bno085_sample_hz: sampleHz,
      }
    }, "accelToast");
  });
  document.getElementById("applyUwbSettings").addEventListener("click", () => {
    postConfig({
      target_modules: document.getElementById("uwbTargets").value,
      params: {
        radio_channel: document.getElementById("uwbRadioChannel").value,
        survey_rx_ms: document.getElementById("uwbSurveyRxMs").value,
        survey_delay_ms: document.getElementById("uwbSurveyDelayMs").value,
        survey_slot_ms: document.getElementById("uwbSurveySlotMs").value,
        survey_gap_ms: document.getElementById("uwbSurveyGapMs").value,
        survey_log_every: document.getElementById("uwbSurveyLogEvery").value,
        ranging_slot_ms: document.getElementById("uwbRangingSlotMs").value,
        ranging_gap_ms: document.getElementById("uwbRangingGapMs").value,
        ranging_rx_ms: document.getElementById("uwbRangingRxMs").value,
        dt_initiator: document.getElementById("uwbDtInitiator").value,
        dt_responder: document.getElementById("uwbDtResponder").value,
        dt_interval_ms: document.getElementById("uwbDtIntervalMs").value,
        dt_rx_timeout_ms: document.getElementById("uwbDtRxTimeoutMs").value,
        dt_resp_delay_ms: document.getElementById("uwbDtRespDelayMs").value,
        dt_final_delay_ms: document.getElementById("uwbDtFinalDelayMs").value,
        dt_report_delay_ms: document.getElementById("uwbDtReportDelayMs").value,
        dt_auto_rx_delay_uus: document.getElementById("uwbDtAutoRxDelayUus").value,
        cal_summary: document.getElementById("uwbCalSummary").value,
        cal_slot_ms: document.getElementById("uwbCalMinMs").value,
        cal_guard_us: document.getElementById("uwbCalGuardUs").value,
        cal_round_gap_ms: document.getElementById("uwbCalMaxMs").value,
        cal_rx_ms: document.getElementById("uwbCalRxMs").value,
        reboot: document.getElementById("uwbAdvancedReboot").value,
      }
    }, "uwbToast");
  });
  document.getElementById("applyAntennaDelay").addEventListener("click", () => {
    postAntennaDelay({
      target_modules: document.getElementById("uwbTargets").value,
      value: document.getElementById("uwbAntennaDelayHex").value,
      reboot: document.getElementById("uwbAdvancedReboot").value,
    }, "uwbToast");
  });
  document.getElementById("clearAntennaDelay").addEventListener("click", () => {
    postAntennaDelay({
      target_modules: document.getElementById("uwbTargets").value,
      clear: "1",
      reboot: document.getElementById("uwbAdvancedReboot").value,
    }, "uwbToast");
  });
  document.getElementById("chargerRawModule").addEventListener("change", renderChargerRegisters);
  document.getElementById("applyChargerAdc").addEventListener("click", () => {
    postDirtyChargerConfig(chargerAdcFields, "chargerAdcTargets", "chargerAdcToast");
  });
  document.getElementById("disableChargerWatchdog").addEventListener("click", () => {
    postChargerConfig({
      target_modules: document.getElementById("chargerAdcTargets").value,
      params: {disable_watchdog: "1"}
    }, "chargerAdcToast");
  });
  document.getElementById("refreshCharger").addEventListener("click", () => {
    postChargerConfig({
      target_modules: document.getElementById("chargerAdcTargets").value,
      params: {refresh: "1"}
    }, "chargerAdcToast");
  });
  document.getElementById("applyChargerLimits").addEventListener("click", () => {
    postDirtyChargerConfig(chargerLimitFields, "chargerLimitTargets", "chargerLimitsToast");
  });
  document.getElementById("applyChargerTermination").addEventListener("click", () => {
    postDirtyChargerConfig(chargerTerminationFields, "chargerTerminationTargets", "chargerTerminationToast");
  });
  document.getElementById("applyChargerTimers").addEventListener("click", () => {
    postDirtyChargerConfig(chargerTimerFields, "chargerTimerTargets", "chargerTimersToast");
  });
  document.getElementById("resetChargerCycle").addEventListener("click", () => {
    resetChargerCycle("chargerTimerTargets", "chargerTimersToast");
  });
  document.getElementById("applyChargerRaw").addEventListener("click", () => {
    const params = {
      reg: document.getElementById("chargerRawReg").value,
      value: document.getElementById("chargerRawValue").value,
      confirm: document.getElementById("chargerRawConfirm").checked ? "1" : "0",
    };
    const mask = document.getElementById("chargerRawMask").value.trim();
    const bits = document.getElementById("chargerRawBits").value.trim();
    if (mask) params.mask = mask;
    if (bits) params.bits = bits;
    postChargerConfig({
      target_modules: document.getElementById("chargerRawModule").value,
      params,
    }, "chargerRawToast");
  });
  document.getElementById("pdRawModule").addEventListener("change", renderPdRegisters);
  document.getElementById("refreshPd").addEventListener("click", () => {
    postPdConfig("pdContractTargets", {refresh: "1"}, "pdContractToast");
  });
  document.getElementById("triggerPdBc").addEventListener("click", () => {
    postPdConfig("pdContractTargets", {bc_trigger: "1"}, "pdContractToast");
  });
  document.getElementById("applyPdSource").addEventListener("click", () => {
    const value = pdParamFromControl("pdSourcePdoPos");
    if (!value) {
      setToast("pdContractToast", "Source PDO position is required", "bad");
      return;
    }
    postPdConfig("pdContractTargets", {source_pdo_pos: value}, "pdContractToast", pdContractFields);
  });
  document.getElementById("applyPdUsb2").addEventListener("click", () => {
    postPdConfig("pdContractTargets", {
      usb2_closed: pdParamFromControl("pdUsb2SwitchClosed"),
    }, "pdContractToast", pdContractFields);
  });
  document.getElementById("applyPdSinkPdos").addEventListener("click", () => {
    const value = pdParamFromControl("pdSinkPdos");
    if (!value) {
      setToast("pdSinkToast", "Sink PDO list is required", "bad");
      return;
    }
    postPdConfig("pdSinkTargets", {
      sink_pdos: value,
      sink_pdos_mtp: pdParamFromControl("pdSinkPdosMtp"),
    }, "pdSinkToast", pdSinkFields);
  });
  document.getElementById("readPdSinkRam").addEventListener("click", () => {
    postPdConfig("pdSinkTargets", {read_sink_mtp: "0"}, "pdSinkToast");
  });
  document.getElementById("readPdSinkMtp").addEventListener("click", () => {
    postPdConfig("pdSinkTargets", {read_sink_mtp: "1"}, "pdSinkToast");
  });
  document.getElementById("applyPdPpsDefault").addEventListener("click", () => {
    postPdConfig("pdPpsTargets", {
      pps_enabled: pdParamFromControl("pdPpsEnabled"),
      pps_voltage_mv: pdParamFromControl("pdPpsVoltageMv"),
      pps_current_ma: pdParamFromControl("pdPpsCurrentMa"),
    }, "pdPpsToast", pdPpsFields);
  });
  document.getElementById("requestPdApdo").addEventListener("click", () => {
    postPdConfig("pdPpsTargets", {
      apdo_pos: pdParamFromControl("pdApdoPos"),
      apdo_voltage_mv: pdParamFromControl("pdApdoVoltageMv"),
      apdo_current_ma: pdParamFromControl("pdApdoCurrentMa"),
    }, "pdPpsToast");
  });
  document.getElementById("applyPdRaw").addEventListener("click", () => {
    postPdConfig("pdRawModule", {
      reg: pdParamFromControl("pdRawReg"),
      value: pdParamFromControl("pdRawValue"),
      confirm: pdParamFromControl("pdRawConfirm"),
    }, "pdRawToast");
  });
  document.getElementById("autoCalibration").addEventListener("click", () => {
    postCalibrationAuto({
      target_modules: document.getElementById("calTargets").value,
      apply: document.getElementById("calAutoApply").checked ? "1" : "0",
      min_apply_dtu: document.getElementById("calMinApplyDtu").value,
      reference_guard_cm: document.getElementById("calReferenceGuardCm").value,
      timeout_sec: document.getElementById("calTimeoutSec").value,
      params: calibrationParamsFromForm(),
    }, "calAutoToast");
  });
  document.getElementById("cancelCalibration").addEventListener("click", () => {
    postCalibrationCancel("calAutoToast");
  });
  ["calMethod","calPair","calKnownCm","calThree","calD01Cm","calD02Cm","calD12Cm"].forEach(id => {
    document.getElementById(id).addEventListener("input", updateCalVisibility);
    document.getElementById(id).addEventListener("change", updateCalVisibility);
  });
  updateCalVisibility();
}

document.querySelectorAll(".terminal").forEach(createTerminal);
document.querySelectorAll(".tab").forEach(tab => tab.addEventListener("click", () => setActiveTab(tab.dataset.tab)));
setActiveTab(state.activeTab);
wireSettings();
setCalibrationResult(loadCalibrationResult());
fetchLogs();
fetchAccel();
fetchSnapshot();
setInterval(fetchLogs, 250);
setInterval(fetchAccel, 50);
setInterval(fetchSnapshot, 1500);
</script>
</body>
</html>
"""


class HttpHandler(BaseHTTPRequestHandler):
    server: "DashboardHttpServer"

    def log_message(self, fmt: str, *args: Any) -> None:
        if not self.server.quiet:
            super().log_message(fmt, *args)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self.send_html(INDEX_HTML)
            return
        if parsed.path == "/api/snapshot":
            self.send_json(self.server.state.snapshot())
            return
        if parsed.path == "/api/logs":
            query = urllib.parse.parse_qs(parsed.query)
            after = int(query.get("after", ["0"])[0] or "0")
            limit = int(query.get("limit", ["2000"])[0] or "2000")
            self.send_json(self.server.state.logs_after(after, max(1, min(limit, 8000))))
            return
        if parsed.path == "/api/accel":
            query = urllib.parse.parse_qs(parsed.query)
            after = int(query.get("after", ["0"])[0] or "0")
            limit = int(query.get("limit", ["8000"])[0] or "8000")
            self.send_json(self.server.state.accel_after(after, max(1, min(limit, 20000))))
            return
        if parsed.path == "/api/calibration-auto/status":
            query = urllib.parse.parse_qs(parsed.query)
            job_id = query.get("job_id", [""])[0] or None
            self.send_json(self.server.calibration_job_status(job_id))
            return
        self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/runtime-config":
            self.handle_runtime_config()
            return
        if parsed.path == "/api/calibration-auto":
            self.handle_calibration_auto()
            return
        if parsed.path == "/api/calibration-auto/cancel":
            self.handle_calibration_cancel()
            return
        if parsed.path == "/api/antenna-delay":
            self.handle_antenna_delay()
            return
        if parsed.path == "/api/charger-config":
            self.handle_charger_config()
            return
        if parsed.path == "/api/max77958-config":
            self.handle_max77958_config()
            return
        self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def handle_runtime_config(self) -> None:
        try:
            payload = self.read_json_body()
            params = normalize_runtime_params(payload.get("params") or {})
            results = self.server.apply_runtime_config(
                params, payload.get("target_modules")
            )
            self.send_json({"ok": all(item["ok"] for item in results), "results": results})
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def handle_antenna_delay(self) -> None:
        try:
            payload = self.read_json_body()
            results = self.server.apply_antenna_delay(payload)
            self.send_json({"ok": all(item["ok"] for item in results), "results": results})
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def handle_calibration_auto(self) -> None:
        try:
            payload = self.read_json_body()
            result = self.server.start_calibration_auto(payload)
            self.send_json(result)
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def handle_calibration_cancel(self) -> None:
        try:
            payload = self.read_json_body()
            result = self.server.cancel_calibration_auto(payload.get("job_id"))
            self.send_json(result)
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def handle_charger_config(self) -> None:
        try:
            payload = self.read_json_body()
            params = normalize_plain_params(payload.get("params") or {})
            results = self.server.apply_charger_config(
                params, payload.get("target_modules")
            )
            self.send_json({"ok": all(item["ok"] for item in results), "results": results})
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def handle_max77958_config(self) -> None:
        try:
            payload = self.read_json_body()
            params = normalize_plain_params(payload.get("params") or {})
            results = self.server.apply_max77958_config(
                params, payload.get("target_modules")
            )
            self.send_json({"ok": all(item["ok"] for item in results), "results": results})
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def send_html(self, text: str) -> None:
        raw = text.encode("utf-8")
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            return

    def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            return


def normalize_runtime_params(raw: dict[str, Any]) -> dict[str, str]:
    params: dict[str, str] = {}
    for key, value in raw.items():
        if value is None or value == "":
            continue
        if key.endswith("_cm"):
            mm_key = key[:-3] + "_mm"
            params[mm_key] = cm_to_mm_text(value)
        else:
            params[key] = str(value)
    return params


def normalize_plain_params(raw: dict[str, Any]) -> dict[str, str]:
    return {
        str(key): str(value)
        for key, value in raw.items()
        if value is not None and value != ""
    }


class DashboardHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        state: DashboardState,
        *,
        targets: list[str],
        token: str,
        quiet: bool,
        timeout_sec: float,
    ) -> None:
        super().__init__(server_address, HttpHandler)
        self.state = state
        self.targets = targets
        self.token = token
        self.quiet = quiet
        self.timeout_sec = timeout_sec
        self.calibration_job_lock = threading.Lock()
        self.calibration_job: dict[str, Any] | None = None

    def apply_runtime_config(
        self, params: dict[str, str], target_modules: Any = None
    ) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        if not params:
            raise RuntimeError("No runtime config parameters provided")
        targets = self.resolve_targets(target_modules)
        return [self.send_runtime_config(target, params) for target in targets]

    def stop_all_uwb(
        self,
        *,
        progress: (
            Callable[[str, dict[str, Any] | None, str | None], None] | None
        ) = None,
        summary: str = "stopping all UWB modules...",
    ) -> list[dict[str, Any]]:
        if progress is not None:
            progress(summary, {"target_modules": "all"}, "stopping")
        return self.apply_runtime_config({"uwb": "0", "reboot": "1"}, "all")

    def next_log_id_value(self) -> int:
        with self.state.lock:
            return int(self.state.next_log_id)

    def current_status_by_module(self) -> dict[int, dict[str, Any]]:
        with self.state.lock:
            return {
                int(module_id): dict(status)
                for module_id, status in self.state.status_by_module.items()
            }

    def start_calibration_auto(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.calibration_job_lock:
            if self.calibration_job and self.calibration_job.get("running"):
                return {
                    "ok": False,
                    "job_id": self.calibration_job.get("job_id"),
                    "running": True,
                    "summary": "calibration already running",
                }
            job_id = str(int(time.time() * 1000))
            now = time.time()
            self.calibration_job = {
                "ok": True,
                "job_id": job_id,
                "running": True,
                "state": "starting",
                "summary": "starting calibration...",
                "started_sec": now,
                "updated_sec": now,
                "elapsed_sec": 0.0,
                "result": None,
                "payload": payload,
                "cancel_requested": False,
            }

        thread = threading.Thread(
            target=self._run_calibration_auto_job,
            args=(job_id, payload),
            daemon=True,
        )
        thread.start()
        return self.calibration_job_status(job_id)

    def calibration_job_status(self, job_id: str | None = None) -> dict[str, Any]:
        with self.calibration_job_lock:
            job = self.calibration_job
            if job is None:
                return {"ok": False, "error": "no calibration job"}
            if job_id and job.get("job_id") != job_id:
                return {"ok": False, "error": "calibration job not found"}
            output = dict(job)
            output["elapsed_sec"] = round(time.time() - float(job["started_sec"]), 1)
            return output

    def update_calibration_job(
        self,
        job_id: str,
        *,
        state: str | None = None,
        summary: str | None = None,
        running: bool | None = None,
        result: dict[str, Any] | None = None,
        detail: dict[str, Any] | None = None,
    ) -> None:
        with self.calibration_job_lock:
            job = self.calibration_job
            if job is None or job.get("job_id") != job_id:
                return
            if state is not None:
                job["state"] = state
            if summary is not None:
                job["summary"] = summary
            if running is not None:
                job["running"] = running
            if result is not None:
                job["result"] = result
                job["ok"] = bool(result.get("ok"))
            if detail is not None:
                job["detail"] = detail
            job["updated_sec"] = time.time()
            job["elapsed_sec"] = round(time.time() - float(job["started_sec"]), 1)

    def calibration_cancel_requested(self, job_id: str) -> bool:
        with self.calibration_job_lock:
            job = self.calibration_job
            return bool(
                job is not None
                and job.get("job_id") == job_id
                and job.get("cancel_requested")
            )

    def calibration_participant_ids_from_payload(
        self, payload: dict[str, Any]
    ) -> list[int]:
        raw_params = payload.get("params") or {}
        params = normalize_runtime_params(raw_params)
        method = params.get("cal_method", "three")
        if method in ("two", "two_module"):
            return [
                int(params.get("cal_ref", 0)),
                int(params.get("cal_dut", 0)),
            ]
        if method in ("three", "three_module", "three_module_edm"):
            return parse_module_ids(params.get("cal_three"), expected=3)
        return []

    def cancel_calibration_auto(self, job_id: Any = None) -> dict[str, Any]:
        active_job_id: str | None = None
        payload: dict[str, Any] = {}
        running = False
        state: str | None = None
        warning: str | None = None
        with self.calibration_job_lock:
            job = self.calibration_job
            if job is None:
                warning = "no calibration job; emergency UWB stop still sent"
            elif job_id and str(job_id) != str(job.get("job_id")):
                warning = "calibration job not found; emergency UWB stop still sent"
            else:
                active_job_id = str(job.get("job_id"))
                payload = dict(job.get("payload") or {})
                running = bool(job.get("running"))
                state = str(job.get("state") or "")
                if running:
                    job["cancel_requested"] = True
                    job["state"] = "cancelling"
                    job["summary"] = "cancelling calibration..."
                    job["updated_sec"] = time.time()

        try:
            participant_ids = [
                module_id
                for module_id in self.calibration_participant_ids_from_payload(payload)
                if module_id > 0
            ]
        except Exception:
            participant_ids = []

        stop_results = self.stop_all_uwb()
        ok = all(item.get("ok") for item in stop_results)
        if active_job_id is not None:
            self.update_calibration_job(
                active_job_id,
                state="cancelling" if running else state,
                summary="cancelling calibration; emergency UWB stop requested",
                detail={
                    "participants": participant_ids,
                    "target_modules": "all",
                    "stop_results": stop_results,
                },
            )
        summary = (
            "calibration cancel requested; emergency UWB stop sent"
            if running
            else "emergency UWB stop sent"
        )
        return {
            "ok": ok,
            "job_id": active_job_id,
            "summary": summary,
            "participants": participant_ids,
            "target_modules": "all",
            "warning": warning,
            "results": stop_results,
        }

    def _run_calibration_auto_job(self, job_id: str, payload: dict[str, Any]) -> None:
        def progress(
            summary: str,
            detail: dict[str, Any] | None = None,
            state: str | None = None,
        ) -> None:
            self.update_calibration_job(
                job_id, state=state, summary=summary, detail=detail
            )

        try:
            result = self.run_calibration_auto(
                payload,
                progress=progress,
                should_cancel=lambda: self.calibration_cancel_requested(job_id),
            )
            ok = bool(result.get("ok"))
            self.update_calibration_job(
                job_id,
                state="done" if ok else "error",
                summary=str(result.get("summary") or ("OK" if ok else "ERROR")),
                running=False,
                result=result,
            )
        except CalibrationCancelled:
            result = {
                "ok": False,
                "cancelled": True,
                "summary": "auto calibration cancelled; UWB stop requested",
            }
            self.update_calibration_job(
                job_id,
                state="cancelled",
                summary=result["summary"],
                running=False,
                result=result,
            )
        except Exception as exc:
            self.update_calibration_job(
                job_id,
                state="error",
                summary=f"ERROR: {exc}",
                running=False,
                result={"ok": False, "error": str(exc)},
            )

    def target_for_module(self, module_id: int) -> str:
        target = self.resolve_targets([module_id])
        if len(target) != 1:
            raise RuntimeError(f"No unique live target for module {module_id}")
        return target[0]

    def live_nonparticipant_modules(self, participant_ids: list[int]) -> list[int]:
        statuses = self.current_status_by_module()
        return sorted(
            module_id
            for module_id, status in statuses.items()
            if module_id not in participant_ids and status.get("target")
        )

    def default_calibration_adjust_ids(
        self, requested_targets: Any, participant_ids: list[int]
    ) -> list[int]:
        if requested_targets in (None, "", "all"):
            return participant_ids
        requested_ids = parse_module_ids(requested_targets)
        participant_set = set(participant_ids)
        adjust_ids = [
            module_id for module_id in requested_ids if module_id in participant_set
        ]
        return adjust_ids or participant_ids

    def reference_guard_failures(
        self,
        reference_checks: dict[str, dict[str, Any]],
        max_error_cm: float,
    ) -> dict[str, dict[str, Any]]:
        if max_error_cm <= 0:
            return {}
        return {
            pair: data
            for pair, data in reference_checks.items()
            if abs(float(data.get("error_cm") or 0.0)) > max_error_cm
        }

    def collect_calibration_samples(
        self,
        *,
        after_id: int,
        expected_pairs: list[tuple[int, int]],
        sample_count: int,
        timeout_sec: float,
        progress: (
            Callable[[str, dict[str, Any] | None, str | None], None] | None
        ) = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> tuple[
        dict[tuple[int, int], list[float]], bool, int, list[dict[str, Any]]
    ]:
        samples = {pair: [] for pair in expected_pairs}
        expected_set = set(expected_pairs)
        sync_misses: list[dict[str, Any]] = []
        last_seen = after_id
        deadline = time.monotonic() + timeout_sec
        next_progress = 0.0
        while time.monotonic() < deadline:
            if should_cancel is not None and should_cancel():
                raise CalibrationCancelled()
            with self.state.lock:
                items = [
                    item for item in self.state.logs if int(item["id"]) > last_seen
                ]
            for item in items:
                last_seen = max(last_seen, int(item["id"]))
                message = str(item.get("message") or item.get("raw") or "")
                if CAL_SYNC_SKIP_RE.search(message):
                    sync_misses.append(
                        {
                            "id": int(item["id"]),
                            "module_id": item.get("module_id"),
                            "message": message,
                        }
                    )
                    continue
                match = CAL_SAMPLE_RE.search(message)
                if match is None:
                    continue
                pair = (int(match.group("src")), int(match.group("dst")))
                if pair not in expected_set:
                    continue
                if len(samples[pair]) >= sample_count:
                    continue
                samples[pair].append(float(match.group("distance")))
            now = time.monotonic()
            if progress is not None and now >= next_progress:
                counts = {
                    f"{a}->{b}": len(values)
                    for (a, b), values in samples.items()
                }
                min_count = min(counts.values()) if counts else 0
                complete_pairs = sum(
                    1 for value in counts.values() if value >= sample_count
                )
                sync_miss_count = len(sync_misses)
                prefix = (
                    f"sync miss detected ({sync_miss_count}); "
                    if sync_miss_count
                    else ""
                )
                progress(
                    f"{prefix}collecting samples {min_count}/{sample_count} per pair "
                    f"({complete_pairs}/{len(samples)} pairs complete)",
                    {
                        "counts": counts,
                        "sync_miss_count": sync_miss_count,
                        "sync_misses": sync_misses[-5:],
                    },
                    "invalid" if sync_miss_count else "collecting",
                )
                next_progress = now + 1.0
            if all(len(values) >= sample_count for values in samples.values()):
                return samples, True, last_seen, sync_misses
            time.sleep(0.5)
        return samples, False, last_seen, sync_misses

    def directed_stats_json(
        self, samples: dict[tuple[int, int], list[float]]
    ) -> dict[str, dict[str, Any]]:
        output: dict[str, dict[str, Any]] = {}
        for pair, values in sorted(samples.items()):
            if not values:
                output[f"{pair[0]}->{pair[1]}"] = {"n": 0}
                continue
            output[f"{pair[0]}->{pair[1]}"] = {
                "n": len(values),
                "mean_m": round(mean(values), 4),
                "median_m": round(median(values), 4),
                "std_m": round(stddev(values), 4),
                "min_m": round(min(values), 4),
                "max_m": round(max(values), 4),
            }
        return output

    def apply_calibration_corrections(
        self,
        corrections: dict[int, int],
        *,
        min_apply_dtu: int,
        apply_changes: bool,
        reboot_after_write: bool = True,
    ) -> list[dict[str, Any]]:
        statuses = self.current_status_by_module()
        results: list[dict[str, Any]] = []
        for module_id, correction in sorted(corrections.items()):
            status = statuses.get(module_id)
            if status is None:
                raise RuntimeError(f"No status for module {module_id}")
            delay_text = (
                status.get("uwb_active_antenna_delay_hex")
                or status.get("uwb_configured_antenna_delay_hex")
            )
            old_delay = parse_u16_text(delay_text)
            new_delay = clamp_u16(old_delay + correction)
            item: dict[str, Any] = {
                "module_id": module_id,
                "old_delay": f"0x{old_delay:04x}",
                "correction_dtu": correction,
                "new_delay": f"0x{new_delay:04x}",
                "applied": False,
            }
            if not apply_changes:
                item["reason"] = "dry_run"
                results.append(item)
                continue
            if abs(correction) < min_apply_dtu:
                item["reason"] = f"below_min_apply_dtu_{min_apply_dtu}"
                results.append(item)
                continue
            target = self.target_for_module(module_id)
            write_params = {"value": f"0x{new_delay:04x}"}
            if reboot_after_write:
                write_params["reboot"] = "1"
            response = self.send_antenna_delay(target, write_params)
            item["target"] = target
            item["write"] = response
            item["applied"] = bool(response.get("ok"))
            results.append(item)
        return results

    def stop_calibration_uwb(
        self,
        participant_ids: list[int],
        *,
        progress: (
            Callable[[str, dict[str, Any] | None, str | None], None] | None
        ) = None,
    ) -> list[dict[str, Any]]:
        if progress is not None:
            progress(
                "stopping calibration UWB...",
                {"participants": participant_ids},
                "stopping",
            )
        return self.apply_runtime_config({"uwb": "0", "reboot": "1"}, participant_ids)

    def run_calibration_auto(
        self,
        payload: dict[str, Any],
        *,
        progress: (
            Callable[[str, dict[str, Any] | None, str | None], None] | None
        ) = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> dict[str, Any]:
        def check_cancel() -> None:
            if should_cancel is not None and should_cancel():
                raise CalibrationCancelled()

        raw_params = payload.get("params") or {}
        params = normalize_runtime_params(raw_params)
        method = params.get("cal_method", "three")
        sample_count = int(params.get("cal_samples", raw_params.get("cal_samples", 40)))
        if sample_count <= 0:
            raise RuntimeError("sample count must be positive")
        timeout_sec = float(payload.get("timeout_sec") or 240)
        min_apply_dtu = max(0, int(payload.get("min_apply_dtu") or 2))
        reference_guard_cm = max(0.0, float(payload.get("reference_guard_cm") or 2.0))
        apply_changes = parse_bool(payload.get("apply"), True)
        if method in ("two", "two_module"):
            participant_ids = [
                int(params.get("cal_ref", 0)),
                int(params.get("cal_dut", 0)),
            ]
        elif method in ("three", "three_module", "three_module_edm"):
            participant_ids = parse_module_ids(params.get("cal_three"), expected=3)
        else:
            raise RuntimeError(f"unsupported calibration method: {method}")
        if any(module_id <= 0 for module_id in participant_ids):
            raise RuntimeError("invalid calibration module ID(s)")
        setup_targets = participant_ids

        params["mode"] = "calibration"
        params["cal_method"] = method
        params["cal_samples"] = str(sample_count)
        params.setdefault("cal_summary", str(sample_count))
        params["uwb"] = "1"
        params["reboot"] = "1"

        excluded_ids = self.live_nonparticipant_modules(participant_ids)
        excluded_results: list[dict[str, Any]] = []
        if excluded_ids:
            excluded_params = dict(params)
            excluded_params["uwb"] = "0"
            if progress is not None:
                progress(
                    "holding excluded UWB modules in reset...",
                    {"excluded_modules": excluded_ids},
                    "configuring",
                )
            excluded_results = self.apply_runtime_config(excluded_params, excluded_ids)
            if not all(item.get("ok") for item in excluded_results):
                cleanup_results = self.stop_all_uwb(
                    progress=progress,
                    summary="excluded module setup failed; stopping all UWB modules...",
                )
                return {
                    "ok": False,
                    "summary": "excluded module setup failed",
                    "results": excluded_results,
                    "cleanup_results": cleanup_results,
                }

        if progress is not None:
            progress(
                "configuring calibration participants...",
                {"method": method, "participants": participant_ids},
                "configuring",
            )
        config_results = self.apply_runtime_config(
            params, setup_targets
        )
        if not all(item.get("ok") for item in config_results):
            cleanup_results = self.stop_all_uwb(
                progress=progress,
                summary="calibration setup failed; stopping all UWB modules...",
            )
            return {
                "ok": False,
                "summary": "calibration setup failed",
                "results": config_results,
                "cleanup_results": cleanup_results,
            }

        if progress is not None:
            progress("waiting for modules to restart calibration...", None, "waiting")
        time.sleep(2.0)
        check_cancel()
        after_id = self.next_log_id_value() - 1

        if method in ("two", "two_module"):
            ref_id = int(params.get("cal_ref", 0))
            dut_id = int(params.get("cal_dut", 0))
            if ref_id <= 0 or dut_id <= 0 or ref_id == dut_id:
                raise RuntimeError("invalid two-module calibration IDs")
            known_m = int(params.get("cal_known_mm", 0)) / 1000.0
            expected_pairs = [(ref_id, dut_id)]
            (
                samples,
                complete,
                last_log_id,
                sync_misses,
            ) = self.collect_calibration_samples(
                after_id=after_id,
                expected_pairs=expected_pairs,
                sample_count=sample_count,
                timeout_sec=timeout_sec,
                progress=progress,
                should_cancel=should_cancel,
            )
            check_cancel()
            values = samples[(ref_id, dut_id)]
            if not values:
                raise RuntimeError("no calibration samples collected")
            center_m = median(values)
            mean_m = sum(values) / len(values)
            error_m = center_m - known_m
            correction = round_i32(error_m / UWB_METERS_PER_DTU)
            adjust_ids = parse_module_ids(payload.get("adjust_modules")) or [dut_id]
            if adjust_ids != [dut_id]:
                raise RuntimeError("two-module auto calibration can adjust only the DUT")
            corrections = {dut_id: correction}
            sync_ok = not sync_misses
            effective_apply = apply_changes and complete and sync_ok
            if progress is not None:
                progress(
                    "applying antenna-delay correction...",
                    {"corrections": corrections, "sync_ok": sync_ok},
                    "applying",
                )
            check_cancel()
            apply_results = self.apply_calibration_corrections(
                corrections,
                min_apply_dtu=min_apply_dtu,
                apply_changes=effective_apply,
                reboot_after_write=False,
            )
            if apply_changes and complete and not sync_ok:
                for item in apply_results:
                    if not item.get("applied"):
                        item["reason"] = "sync_miss"
            write_ok = all(
                item.get("applied") or item.get("reason") for item in apply_results
            )
            stop_results = self.stop_calibration_uwb(
                participant_ids, progress=progress
            )
            stop_ok = all(item.get("ok") for item in stop_results)
            summary_prefix = "" if sync_ok else "invalid: sync miss; "
            stop_suffix = "; UWB stopped" if stop_ok else "; UWB stop failed"
            return {
                "ok": complete and write_ok and sync_ok and stop_ok,
                "summary": (
                    "auto calibration complete: "
                    + summary_prefix
                    + calibration_write_summary(apply_results)
                    + stop_suffix
                ),
                "method": "two",
                "ids": participant_ids,
                "adjust_ids": [dut_id],
                "reference_id": ref_id,
                "dut_id": dut_id,
                "complete": complete,
                "valid": sync_ok,
                "sync_miss_count": len(sync_misses),
                "sync_misses": sync_misses[-10:],
                "sample_count": sample_count,
                "center_method": "median",
                "center_m": round(center_m, 4),
                "known_m": round(known_m, 4),
                "last_log_id": last_log_id,
                "pairs": {
                    f"{ref_id}->{dut_id}": {
                        "known_m": round(known_m, 4),
                        "center_m": round(center_m, 4),
                        "mean_m": round(mean_m, 4),
                        "error_cm": round(error_m * 100.0, 2),
                        "error_dtu": round(error_m / UWB_METERS_PER_DTU, 2),
                    }
                },
                "directed": self.directed_stats_json(samples),
                "corrections": corrections,
                "apply_results": apply_results,
                "stop_results": stop_results,
                "excluded_results": excluded_results,
                "config_results": config_results,
            }

        if method not in ("three", "three_module", "three_module_edm"):
            raise RuntimeError(f"unsupported calibration method: {method}")

        ids = parse_module_ids(params.get("cal_three"), expected=3)
        edge_mm = {
            (ids[0], ids[1]): int(params.get("cal_d01_mm", 0)),
            (ids[0], ids[2]): int(params.get("cal_d02_mm", 0)),
            (ids[1], ids[2]): int(params.get("cal_d12_mm", 0)),
        }
        for pair, distance_mm in edge_mm.items():
            if distance_mm <= 0:
                raise RuntimeError(f"invalid distance for edge {pair}: {distance_mm}")

        expected_pairs = [(src, dst) for src in ids for dst in ids if src != dst]
        (
            samples,
            complete,
            last_log_id,
            sync_misses,
        ) = self.collect_calibration_samples(
            after_id=after_id,
            expected_pairs=expected_pairs,
            sample_count=sample_count,
            timeout_sec=timeout_sec,
            progress=progress,
            should_cancel=should_cancel,
        )
        check_cancel()

        if progress is not None:
            progress("computing antenna-delay corrections...", None, "computing")
        directed_errors: dict[tuple[int, int], float] = {}
        pair_summary: dict[str, dict[str, Any]] = {}
        for raw_pair, distance_mm in edge_mm.items():
            a, b = raw_pair
            directed_centers = [
                median(samples[(a, b)]) if samples[(a, b)] else None,
                median(samples[(b, a)]) if samples[(b, a)] else None,
            ]
            valid_centers = [item for item in directed_centers if item is not None]
            if not valid_centers:
                raise RuntimeError(f"no samples for pair {a}-{b}")
            pair_center = mean(valid_centers)
            pair_mean_values = []
            if samples[(a, b)]:
                pair_mean_values.append(mean(samples[(a, b)]))
            if samples[(b, a)]:
                pair_mean_values.append(mean(samples[(b, a)]))
            pair_mean = mean(pair_mean_values) if pair_mean_values else pair_center
            known_m = distance_mm / 1000.0
            error_m = pair_center - known_m
            error_dtu = error_m / UWB_METERS_PER_DTU
            key = tuple(sorted(raw_pair))
            pair_summary[f"{key[0]}-{key[1]}"] = {
                "center_method": "median",
                "center_m": round(pair_center, 4),
                "mean_m": round(pair_mean, 4),
                "known_m": round(known_m, 4),
                "error_m": round(error_m, 4),
                "error_cm": round(error_m * 100.0, 2),
                "error_dtu": round(error_dtu, 2),
            }
            for src, dst in ((a, b), (b, a)):
                values_for_direction = samples[(src, dst)]
                if not values_for_direction:
                    continue
                directed_center = median(values_for_direction)
                directed_errors[(src, dst)] = (
                    directed_center - known_m
                ) / UWB_METERS_PER_DTU

        adjust_ids = parse_module_ids(
            payload.get("adjust_modules")
        ) or self.default_calibration_adjust_ids(payload.get("target_modules"), ids)
        for module_id in adjust_ids:
            if module_id not in ids:
                raise RuntimeError(
                    f"adjust module {module_id} is not in calibration set {ids}"
                )

        rows: list[list[float]] = []
        values: list[float] = []
        fit_pairs: list[tuple[int, int]] = []
        reference_checks: dict[str, dict[str, Any]] = {}
        for pair, error_dtu in directed_errors.items():
            row = [1.0 if module_id in pair else 0.0 for module_id in adjust_ids]
            if any(row):
                rows.append(row)
                values.append(error_dtu)
                fit_pairs.append(pair)
            else:
                reference_checks[f"{pair[0]}->{pair[1]}"] = {
                    "error_dtu": round(error_dtu, 2),
                    "error_cm": round(error_dtu * UWB_METERS_PER_DTU * 100.0, 2),
                }

        reference_guard_failures = self.reference_guard_failures(
            reference_checks, reference_guard_cm
        )
        reference_guard_ok = not reference_guard_failures
        correction_float = solve_least_squares(rows, values) if adjust_ids else []
        corrections = {
            module_id: round_i32(correction_float[index])
            for index, module_id in enumerate(adjust_ids)
        }
        residuals: dict[str, dict[str, Any]] = {}
        residual_norm = 0.0
        max_abs_residual_dtu = 0.0
        if correction_float:
            for row, value, pair in zip(rows, values, fit_pairs):
                predicted = sum(item * correction_float[index] for index, item in enumerate(row))
                residual_dtu = value - predicted
                residual_norm += residual_dtu * residual_dtu
                max_abs_residual_dtu = max(max_abs_residual_dtu, abs(residual_dtu))
                residuals[f"{pair[0]}->{pair[1]}"] = {
                    "error_dtu": round(value, 2),
                    "fit_dtu": round(predicted, 2),
                    "residual_dtu": round(residual_dtu, 2),
                    "residual_cm": round(
                        residual_dtu * UWB_METERS_PER_DTU * 100.0, 2
                    ),
                }
            residual_norm = math.sqrt(residual_norm)
        correction_details = {
            str(module_id): {
                "float_dtu": round(correction_float[index], 2),
                "applied_dtu": corrections[module_id],
            }
            for index, module_id in enumerate(adjust_ids)
        }
        sync_ok = not sync_misses
        effective_apply = apply_changes and complete and sync_ok and reference_guard_ok
        if progress is not None:
            progress(
                "applying antenna-delay corrections...",
                {
                    "corrections": corrections,
                    "sync_ok": sync_ok,
                    "reference_guard_ok": reference_guard_ok,
                },
                "applying",
            )
        check_cancel()
        apply_results = self.apply_calibration_corrections(
            corrections,
            min_apply_dtu=min_apply_dtu,
            apply_changes=effective_apply,
            reboot_after_write=False,
        )
        if apply_changes and complete and not reference_guard_ok:
            for item in apply_results:
                if not item.get("applied"):
                    item["reason"] = "reference_guard"
        if apply_changes and complete and not sync_ok:
            for item in apply_results:
                if not item.get("applied"):
                    item["reason"] = "sync_miss"
        write_ok = all(
            item.get("applied") or item.get("reason") for item in apply_results
        )
        stop_results = self.stop_calibration_uwb(participant_ids, progress=progress)
        stop_ok = all(item.get("ok") for item in stop_results)
        if not sync_ok:
            summary_action = (
                f"invalid: sync miss ({len(sync_misses)}); "
                + calibration_write_summary(apply_results)
            )
        elif reference_guard_failures:
            failed_labels = ", ".join(
                f"{pair} {data['error_cm']:+.2f} cm"
                for pair, data in sorted(reference_guard_failures.items())
            )
            summary_action = (
                f"reference guard blocked writes: {failed_labels}; "
                + calibration_write_summary(apply_results)
            )
        else:
            summary_action = calibration_write_summary(apply_results)
        summary_action += "; UWB stopped" if stop_ok else "; UWB stop failed"
        return {
            "ok": complete and write_ok and sync_ok and reference_guard_ok and stop_ok,
            "summary": f"auto calibration complete: {summary_action}",
            "method": "three",
            "ids": ids,
            "adjust_ids": adjust_ids,
            "complete": complete,
            "valid": sync_ok,
            "sync_miss_count": len(sync_misses),
            "sync_misses": sync_misses[-10:],
            "sample_count": sample_count,
            "center_method": "median",
            "last_log_id": last_log_id,
            "directed": self.directed_stats_json(samples),
            "pairs": pair_summary,
            "directed_fit": residuals,
            "fit_residual_norm_dtu": round(residual_norm, 2),
            "fit_max_abs_residual_cm": round(
                max_abs_residual_dtu * UWB_METERS_PER_DTU * 100.0, 2
            ),
            "reference_checks": reference_checks,
            "reference_guard_cm": reference_guard_cm,
            "reference_guard_ok": reference_guard_ok,
            "reference_guard_failures": reference_guard_failures,
            "corrections": correction_details,
            "apply_results": apply_results,
            "stop_results": stop_results,
            "excluded_results": excluded_results,
            "config_results": config_results,
        }

    def apply_antenna_delay(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        params: dict[str, str] = {}
        if str(payload.get("clear", "")).strip() == "1":
            params["clear"] = "1"
        else:
            value = str(payload.get("value", "")).strip()
            if not value:
                raise RuntimeError("No antenna delay value provided")
            params["value"] = value
        if str(payload.get("reboot", "")).strip() == "1":
            params["reboot"] = "1"
        targets = self.resolve_targets(payload.get("target_modules"))
        return [self.send_antenna_delay(target, params) for target in targets]

    def apply_charger_config(
        self, params: dict[str, str], target_modules: Any = None
    ) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        if not params:
            raise RuntimeError("No charger config parameters provided")
        targets = self.resolve_targets(target_modules)
        return [self.send_charger_config(target, params) for target in targets]

    def apply_max77958_config(
        self, params: dict[str, str], target_modules: Any = None
    ) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        if not params:
            raise RuntimeError("No MAX77958 config parameters provided")
        targets = self.resolve_targets(target_modules)
        return [self.send_max77958_config(target, params) for target in targets]

    def resolve_targets(self, target_modules: Any) -> list[str]:
        if isinstance(target_modules, str):
            raw_items = [target_modules]
        else:
            raw_items = list(target_modules) if target_modules is not None else ["all"]

        module_ids: list[int] = []
        for raw in raw_items:
            text = str(raw).strip()
            if not text or text == "all":
                module_ids = []
                break
            module_ids.append(int(text))

        with self.state.lock:
            now = time.time()
            max_age_sec = self.state.status_online_max_age_sec
            by_module = {
                int(status["module_id"]): str(status["target"])
                for status in self.state.status_by_module.values()
                if status.get("module_id") is not None
                and status.get("target")
                and bool(status.get("wifi_connected"))
                and now - float(status.get("status_updated_at") or 0.0) <= max_age_sec
                and str(status.get("target") or "") not in self.state.status_errors
            }

        if not module_ids:
            targets = list(dict.fromkeys(by_module.values()))
            if not targets:
                raise RuntimeError("No HTTP-live targets known")
            return targets

        targets: list[str] = []
        missing: list[int] = []
        for module_id in module_ids:
            target = by_module.get(module_id)
            if target is None:
                missing.append(module_id)
            elif target not in targets:
                targets.append(target)

        if missing:
            raise RuntimeError(
                "No HTTP-live target known for module(s): " +
                ",".join(str(item) for item in missing)
            )
        return targets

    def send_runtime_config(self, target: str, params: dict[str, str]) -> dict[str, Any]:
        query = urllib.parse.urlencode(params, safe=",")
        request = urllib.request.Request(
            f"{runtime_url(target)}?{query}",
            data=b"",
            method="POST",
            headers={"Content-Length": "0", "X-OTA-Token": self.token},
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
                return {
                    "target": target,
                    "ok": 200 <= response.status < 300,
                    "status": response.status,
                    "elapsed_sec": round(time.monotonic() - started, 3),
                    "body": body,
                }
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace").strip()
            return {"target": target, "ok": False, "status": exc.code, "body": body or exc.reason}
        except (urllib.error.URLError, TimeoutError) as exc:
            return {"target": target, "ok": False, "error": str(exc)}

    def send_antenna_delay(self, target: str, params: dict[str, str]) -> dict[str, Any]:
        query = urllib.parse.urlencode(params)
        request = urllib.request.Request(
            f"{antenna_delay_url(target)}?{query}",
            data=b"",
            method="POST",
            headers={"Content-Length": "0", "X-OTA-Token": self.token},
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
                return {
                    "target": target,
                    "ok": 200 <= response.status < 300,
                    "status": response.status,
                    "elapsed_sec": round(time.monotonic() - started, 3),
                    "body": body,
                }
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace").strip()
            return {"target": target, "ok": False, "status": exc.code, "body": body or exc.reason}
        except (urllib.error.URLError, TimeoutError) as exc:
            return {"target": target, "ok": False, "error": str(exc)}

    def send_charger_config(self, target: str, params: dict[str, str]) -> dict[str, Any]:
        query = urllib.parse.urlencode(params, safe=",")
        request = urllib.request.Request(
            f"{charger_url(target)}?{query}",
            data=b"",
            method="POST",
            headers={"Content-Length": "0", "X-OTA-Token": self.token},
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
                return {
                    "target": target,
                    "ok": 200 <= response.status < 300,
                    "status": response.status,
                    "elapsed_sec": round(time.monotonic() - started, 3),
                    "body": body,
                }
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace").strip()
            return {"target": target, "ok": False, "status": exc.code, "body": body or exc.reason}
        except (urllib.error.URLError, TimeoutError) as exc:
            return {"target": target, "ok": False, "error": str(exc)}

    def send_max77958_config(self, target: str, params: dict[str, str]) -> dict[str, Any]:
        query = urllib.parse.urlencode(params, safe=",:")
        request = urllib.request.Request(
            f"{max77958_url(target)}?{query}",
            data=b"",
            method="POST",
            headers={"Content-Length": "0", "X-OTA-Token": self.token},
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
                return {
                    "target": target,
                    "ok": 200 <= response.status < 300,
                    "status": response.status,
                    "elapsed_sec": round(time.monotonic() - started, 3),
                    "body": body,
                }
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace").strip()
            return {"target": target, "ok": False, "status": exc.code, "body": body or exc.reason}
        except (urllib.error.URLError, TimeoutError) as exc:
            return {"target": target, "ok": False, "error": str(exc)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UWB local browser dashboard.")
    parser.add_argument("--log-host", default="0.0.0.0")
    parser.add_argument("--log-port", type=int, default=6055)
    parser.add_argument("--telemetry-host", default="0.0.0.0")
    parser.add_argument("--telemetry-port", type=int, default=6060)
    parser.add_argument(
        "--telemetry-ports",
        default="6060,16060,55060",
        help="Comma/space separated telemetry TCP ports to listen on.",
    )
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8780)
    parser.add_argument("--target-list", default="tools/ota_targets.local.txt")
    parser.add_argument("--secrets", default="secrets.h")
    parser.add_argument("--status-interval", type=float, default=1.5)
    parser.add_argument("--request-timeout", type=float, default=4.0)
    parser.add_argument("--max-logs", type=int, default=20000)
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    targets = read_targets(resolve_project_path(args.target_list))
    token = read_ota_token(resolve_project_path(args.secrets))
    state = DashboardState(max_logs=args.max_logs)
    state.status_online_max_age_sec = max(
        12.0,
        args.status_interval * 3.0,
        len(targets) * 2.5 + args.status_interval,
    )
    state.set_status_targets(targets)

    log_server = LogServer((args.log_host, args.log_port), state)
    log_thread = threading.Thread(target=log_server.serve_forever, daemon=True)
    log_thread.start()

    telemetry_ports = parse_port_list(args.telemetry_ports)
    if args.telemetry_port not in telemetry_ports:
        telemetry_ports.insert(0, args.telemetry_port)
    listener_telemetry_ports = [args.log_port]
    telemetry_servers: list[TelemetryServer] = []
    telemetry_threads: list[threading.Thread] = []
    for port in telemetry_ports:
        if port == args.log_port:
            continue
        telemetry_server = TelemetryServer(
            (args.telemetry_host, port), state, source=f"telemetry:{port}"
        )
        telemetry_thread = threading.Thread(
            target=telemetry_server.serve_forever, daemon=True
        )
        telemetry_thread.start()
        telemetry_servers.append(telemetry_server)
        telemetry_threads.append(telemetry_thread)
        listener_telemetry_ports.append(port)
    state.set_listener_telemetry_ports(listener_telemetry_ports)

    poller = StatusPoller(state, targets, args.status_interval)
    poller.start()

    http_server = DashboardHttpServer(
        (args.http_host, args.http_port),
        state,
        targets=targets,
        token=token,
        quiet=args.quiet,
        timeout_sec=args.request_timeout,
    )
    url = f"http://{args.http_host}:{args.http_port}/"
    print(f"Listening for wireless logs on {args.log_host}:{args.log_port}")
    print(
        "Listening for wireless telemetry on "
        f"{args.telemetry_host}:{', '.join(str(port) for port in listener_telemetry_ports)}"
    )
    print(f"Serving UWB dashboard at {url}")
    if args.open:
        webbrowser.open(url)
    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        poller.stop_event.set()
        http_server.shutdown()
        http_server.server_close()
        log_server.shutdown()
        log_server.server_close()
        for telemetry_server in telemetry_servers:
            telemetry_server.shutdown()
            telemetry_server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
