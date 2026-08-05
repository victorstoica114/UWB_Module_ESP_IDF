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
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
LEAFLET_ROOT = pathlib.Path("/usr/share/javascript/leaflet")
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
    r"(?:frame|seq)=(?P<frame_id>\d+)\s+"
    r"distance=(?P<distance>[-+]?\d+(?:\.\d+)?)\s+m"
)
FLOAT_TEXT_RE = r"[-+]?(?:\d+(?:\.\d+)?|nan|inf)"
FLEX_TDOA_RE = re.compile(
    r"\bUWB_FLEX_TDOA obs\s+tag=(?P<tag>\d+)\s+"
    r"initiator=(?P<initiator>\d+)\s+responder=(?P<responder>\d+)\s+"
    r"seq=(?P<seq>\d+)\s+"
    r"(?:slot=(?P<slot>\d+)\s+index=(?P<index>\d+)\s+)?"
    r"diff=(?P<diff>[-+]?\d+(?:\.\d+)?)\s+m\s+"
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
    r"(?:[-+]?\d+(?:\.\d+)?\s+cm\s+)?"
    r"raw=(?P<raw>[-+]?\d+(?:\.\d+)?)\s+m"
)
CAL_SYNC_SKIP_RE = re.compile(r"\bUWB CAL slot skipped due to sync fail\b")
TELEMETRY_BINARY_MAGIC = b"UWT1"
TELEMETRY_BINARY_HEADER_LEN = 12
TELEMETRY_STREAM_BNO085_ACCEL = 1
TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION = 2
TELEMETRY_STREAM_FLEX_ANCHOR_RANGE = 3
TELEMETRY_STREAM_FLEX_POSITION = 4
TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION = 5
TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE = 6
TELEMETRY_STREAM_PASSIVE_DS_POSITION = 7
TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION_V2 = 8
TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE = 9
TELEMETRY_STREAM_PASSIVE_DS_POSITION_V2 = 10
TELEMETRY_STREAM_PASSIVE_DS_POSITION_V3 = 11
TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4 = 12
TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE = 13
TELEMETRY_STREAM_PASSIVE_DS_GEOMETRY = 14
TELEMETRY_STREAM_NATIVE_DS_POSITION = 15
TELEMETRY_STREAM_NATIVE_DS_GEOMETRY = 16
TELEMETRY_STREAM_FLEX_GEOMETRY = 17
TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION_V2 = 18
TELEMETRY_ACCEL_SAMPLE_LEN = 21
TELEMETRY_FLEX_OBSERVATION_SAMPLE_LEN = 26
TELEMETRY_FLEX_OBSERVATION_V2_SAMPLE_LEN = 49
TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN = 20
TELEMETRY_FLEX_POSITION_SAMPLE_LEN = 32
TELEMETRY_PASSIVE_DS_OBSERVATION_V2_SAMPLE_LEN = 41
TELEMETRY_PASSIVE_DS_POSITION_V2_SAMPLE_LEN = 40
TELEMETRY_PASSIVE_DS_POSITION_V3_SAMPLE_LEN = 49
TELEMETRY_PASSIVE_DS_POSITION_V4_SAMPLE_LEN = 63
TELEMETRY_PASSIVE_DS_GEOMETRY_SAMPLE_LEN = 24
TELEMETRY_ACCEL_STRUCT = struct.Struct("<IIiiiB")
TELEMETRY_FLEX_OBSERVATION_STRUCT = struct.Struct("<IIiiiHBBBB")
TELEMETRY_FLEX_OBSERVATION_V2_STRUCT = struct.Struct(
    "<IIiiiiiiiIHBBBBHB"
)
TELEMETRY_FLEX_ANCHOR_RANGE_STRUCT = struct.Struct("<IIiiHBB")
TELEMETRY_FLEX_POSITION_STRUCT = struct.Struct("<IIiiiiIHBB")
TELEMETRY_PASSIVE_DS_OBSERVATION_V2_STRUCT = struct.Struct(
    "<IIiiiiiIHBBBBBH"
)
TELEMETRY_PASSIVE_DS_POSITION_V2_STRUCT = struct.Struct(
    "<IIiiiiiiIHBB"
)
TELEMETRY_PASSIVE_DS_POSITION_V3_STRUCT = struct.Struct(
    "<IIiiiiiiIHBBIIB"
)
TELEMETRY_PASSIVE_DS_POSITION_V4_STRUCT = struct.Struct(
    "<IIiiiiiiIHBBIIBHHHHHI"
)
TELEMETRY_PASSIVE_DS_GEOMETRY_STRUCT = struct.Struct("<IIiiiBBBB")
ANCHOR_RANGE_HISTORY_MAX_AGE_SEC = 30.0
TELEMETRY_STREAM_SAMPLE_SIZES = {
    TELEMETRY_STREAM_BNO085_ACCEL: TELEMETRY_ACCEL_SAMPLE_LEN,
    TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION:
        TELEMETRY_FLEX_OBSERVATION_SAMPLE_LEN,
    TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION_V2:
        TELEMETRY_FLEX_OBSERVATION_V2_SAMPLE_LEN,
    TELEMETRY_STREAM_FLEX_ANCHOR_RANGE:
        TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN,
    TELEMETRY_STREAM_FLEX_POSITION: TELEMETRY_FLEX_POSITION_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION:
        TELEMETRY_FLEX_OBSERVATION_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE:
        TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_POSITION:
        TELEMETRY_FLEX_POSITION_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION_V2:
        TELEMETRY_PASSIVE_DS_OBSERVATION_V2_SAMPLE_LEN,
    TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE:
        TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_POSITION_V2:
        TELEMETRY_PASSIVE_DS_POSITION_V2_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_POSITION_V3:
        TELEMETRY_PASSIVE_DS_POSITION_V3_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4:
        TELEMETRY_PASSIVE_DS_POSITION_V4_SAMPLE_LEN,
    TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE:
        TELEMETRY_FLEX_ANCHOR_RANGE_SAMPLE_LEN,
    TELEMETRY_STREAM_PASSIVE_DS_GEOMETRY:
        TELEMETRY_PASSIVE_DS_GEOMETRY_SAMPLE_LEN,
    TELEMETRY_STREAM_NATIVE_DS_POSITION:
        TELEMETRY_FLEX_POSITION_SAMPLE_LEN,
    TELEMETRY_STREAM_NATIVE_DS_GEOMETRY:
        TELEMETRY_PASSIVE_DS_GEOMETRY_SAMPLE_LEN,
    TELEMETRY_STREAM_FLEX_GEOMETRY:
        TELEMETRY_PASSIVE_DS_GEOMETRY_SAMPLE_LEN,
}
UWB_METERS_PER_DTU = 15.650040064102564e-12 * 299702547.0
UWB_DTU_SECONDS = 15.650040064102564e-12


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


RUNTIME_MODE_STATUS_NAMES = {
    "beacon": "uwb_beacon_smoke",
    "beacon_smoke": "uwb_beacon_smoke",
    "uwb_beacon_smoke": "uwb_beacon_smoke",
    "distance": "uwb_distance_test",
    "distance_test": "uwb_distance_test",
    "uwb_distance_test": "uwb_distance_test",
    "calibration": "uwb_antenna_delay_calibration",
    "uwb_antenna_delay_calibration": "uwb_antenna_delay_calibration",
    "ranging": "uwb_ranging",
    "uwb_ranging": "uwb_ranging",
    "survey": "uwb_anchor_survey",
    "anchor_survey": "uwb_anchor_survey",
    "uwb_anchor_survey": "uwb_anchor_survey",
    "flex_tdoa": "uwb_flex_tdoa",
    "flextdoa": "uwb_flex_tdoa",
    "uwb_flex_tdoa": "uwb_flex_tdoa",
    "passive_ds": "uwb_passive_ds_twr",
    "passive_ds_twr": "uwb_passive_ds_twr",
    "uwb_passive_ds_twr": "uwb_passive_ds_twr",
}

RUNTIME_PARAM_STATUS_FIELDS = {
    "tag": "runtime_tag_id",
    "anchor_count": "runtime_anchor_count",
    "flex_k": "runtime_flex_tdoa_responder_count",
    "flex_guard_us": "runtime_flex_tdoa_guard_us",
    "flex_req_us": "runtime_flex_tdoa_request_subslot_us",
    "flex_req_process_us": "runtime_flex_tdoa_request_process_us",
    "flex_resp_us": "runtime_flex_tdoa_response_subslot_us",
    "flex_resp_process_us": "runtime_flex_tdoa_response_process_us",
    "coordinator": "runtime_anchor_survey_coordinator_id",
    "coord": "runtime_anchor_survey_coordinator_id",
    "survey_rx_ms": "runtime_anchor_survey_rx_slice_ms",
    "survey_delay_ms": "runtime_anchor_survey_command_delay_ms",
    "survey_slot_ms": "runtime_anchor_survey_slot_ms",
    "survey_gap_ms": "runtime_anchor_survey_round_gap_ms",
    "survey_log_every": "runtime_anchor_survey_passive_tag_log_every",
    "ranging_slot_ms": "runtime_ranging_slot_ms",
    "ranging_gap_ms": "runtime_ranging_round_gap_ms",
    "ranging_rx_ms": "runtime_ranging_rx_slice_ms",
    "ranging_timeout_ms": "runtime_ranging_rx_timeout_ms",
    "ranging_resp_delay_ms": "runtime_ranging_resp_delay_ms",
    "ranging_final_delay_ms": "runtime_ranging_final_delay_ms",
    "ranging_auto_rx_delay_uus": "runtime_ranging_auto_rx_delay_uus",
    "passive_ds_schedule": "runtime_passive_ds_schedule",
    "passive_ds_slot_ms": "runtime_passive_ds_slot_ms",
    "passive_ds_gap_ms": "runtime_passive_ds_round_gap_ms",
    "passive_ds_rx_ms": "runtime_passive_ds_rx_slice_ms",
    "passive_ds_timeout_ms": "runtime_passive_ds_rx_timeout_ms",
    "passive_ds_resp_delay_us": "runtime_passive_ds_resp_delay_us",
    "passive_ds_final_delay_us": "runtime_passive_ds_final_delay_us",
    "passive_ds_auto_rx_delay_uus":
        "runtime_passive_ds_auto_rx_delay_uus",
    "passive_ds_pipeline_mode": "runtime_passive_ds_pipeline_mode",
    "passive_ds_solve_mode": "runtime_passive_ds_solve_mode",
    "passive_ds_rolling_max_hz":
        "runtime_passive_ds_rolling_max_hz",
    "radio_channel": "runtime_radio_channel",
    "uwb_channel": "runtime_radio_channel",
    "radio_phy_mode": "runtime_radio_phy_mode",
    "uwb_phy_mode": "runtime_radio_phy_mode",
    "telemetry_port": "runtime_wireless_telemetry_port",
    "tel_port": "runtime_wireless_telemetry_port",
}


def runtime_config_matches_status(
    params: dict[str, str], status: dict[str, Any]
) -> bool:
    checked = 0
    mode = params.get("mode")
    if mode is not None:
        expected_mode = RUNTIME_MODE_STATUS_NAMES.get(str(mode).lower())
        if expected_mode is None or status.get("runtime_mode_name") != expected_mode:
            return False
        checked += 1

    anchors = params.get("anchors")
    if anchors is not None:
        expected_anchors = [
            int(value)
            for value in re.split(r"[\s,]+", str(anchors))
            if value.strip()
        ]
        actual_anchors = [
            int(value) for value in status.get("runtime_anchor_ids") or []
        ]
        if actual_anchors != expected_anchors:
            return False
        checked += 1

    for key, field in (
        ("flex_tdoa_anchor_correction_mm",
         "runtime_flex_tdoa_anchor_correction_mm"),
        ("native_ds_range_bias_mm",
         "runtime_native_ds_range_bias_mm"),
        ("passive_ds_anchor_bias_mm",
         "runtime_passive_ds_anchor_bias_mm"),
        ("passive_ds_range_bias_mm",
         "runtime_passive_ds_range_bias_mm"),
    ):
        if key not in params:
            continue
        try:
            expected_values = [
                int(value, 0)
                for value in re.split(r"[\s,;]+", str(params[key]))
                if value.strip()
            ]
            actual_values = [
                int(value) for value in status.get(field) or []
            ]
        except (TypeError, ValueError):
            return False
        if actual_values != expected_values:
            return False
        checked += 1

    if "passive_ds_calibration_clear" in params:
        if bool(status.get("runtime_passive_ds_calibration_enabled")):
            return False
        checked += 1

    if "native_ds_calibration_clear" in params:
        if bool(status.get("runtime_native_ds_calibration_enabled")):
            return False
        checked += 1

    for key, field in RUNTIME_PARAM_STATUS_FIELDS.items():
        if key not in params:
            continue
        try:
            expected = int(str(params[key]), 0)
            actual = int(status.get(field))
        except (TypeError, ValueError):
            return False
        if actual != expected:
            return False
        checked += 1

    for key, field in (
        ("uwb", "runtime_uwb_enabled"),
        ("bno085", "runtime_bno085_accel_enabled"),
        ("gps", "runtime_gps_enabled"),
    ):
        if key not in params:
            continue
        expected = str(params[key]).lower() in ("1", "true", "yes", "on")
        if bool(status.get(field)) != expected:
            return False
        checked += 1

    return checked > 0


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
    expected_sample_size = TELEMETRY_STREAM_SAMPLE_SIZES.get(stream_type)
    if (
        version != 1
        or expected_sample_size is None
        or sample_size != expected_sample_size
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

    stream_type = int(frame[5])
    module_id = int(frame[6])
    sample_size = int(frame[7])
    count = int.from_bytes(frame[8:10], "little")
    payload_len = int.from_bytes(frame[10:12], "little")
    expected_sample_size = TELEMETRY_STREAM_SAMPLE_SIZES.get(stream_type)
    if expected_sample_size is None or sample_size != expected_sample_size:
        return []
    if payload_len != count * sample_size:
        return []

    samples: list[dict[str, Any]] = []
    offset = TELEMETRY_BINARY_HEADER_LEN
    end = min(len(frame), offset + payload_len)
    while offset + sample_size <= end:
        common = {
            "module_id": module_id,
            "host": f"uwb-module-{module_id}",
        }
        if stream_type == TELEMETRY_STREAM_BNO085_ACCEL:
            uptime_ms, reports, x, y, z, accuracy = (
                TELEMETRY_ACCEL_STRUCT.unpack_from(frame, offset)
            )
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": "bno085.accel",
                    "x": x / 1000.0,
                    "y": y / 1000.0,
                    "z": z / 1000.0,
                    "accuracy": int(accuracy),
                    "reports": int(reports),
                }
            )
        elif stream_type == TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION_V2:
            (
                uptime_ms,
                slot_id,
                diff_mm,
                raw_diff_mm,
                anchor_distance_mm,
                cfo_correction_mm,
                raw_cfo_ppb,
                estimated_cfo_ppb,
                applied_cfo_ppb,
                processing_dtu,
                sequence,
                tag_id,
                initiator_id,
                responder_id,
                responder_index,
                cfo_sample_count,
                cfo_flags,
            ) = TELEMETRY_FLEX_OBSERVATION_V2_STRUCT.unpack_from(
                frame, offset
            )
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": "uwb.flex_tdoa.observation",
                    "tdoa_protocol": "flextdoa",
                    "slot_id": int(slot_id),
                    "diff_m": diff_mm / 1000.0,
                    "raw_diff_m": raw_diff_mm / 1000.0,
                    "anchor_distance_m": anchor_distance_mm / 1000.0,
                    "cfo_correction_m": cfo_correction_mm / 1000.0,
                    "cfo_raw_ppm": raw_cfo_ppb / 1000.0,
                    "cfo_estimated_ppm": estimated_cfo_ppb / 1000.0,
                    "cfo_applied_ppm": applied_cfo_ppb / 1000.0,
                    "processing_dtu": int(processing_dtu),
                    "reply_delay_us": int(
                        round(processing_dtu * UWB_DTU_SECONDS * 1e6)
                    ),
                    "cfo_sample_count": int(cfo_sample_count),
                    "cfo_ready": bool(cfo_flags & 0x01),
                    "cfo_estimate_applied": bool(cfo_flags & 0x02),
                    "cfo_reset": bool(cfo_flags & 0x04),
                    "cfo_flags": int(cfo_flags),
                    "seq": int(sequence),
                    "tag_id": int(tag_id),
                    "initiator_id": int(initiator_id),
                    "responder_id": int(responder_id),
                    "responder_index": int(responder_index),
                }
            )
        elif stream_type == TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION_V2:
            (
                uptime_ms,
                slot_id,
                diff_mm,
                raw_diff_mm,
                anchor_distance_mm,
                cfo_correction_mm,
                clock_offset_ppb,
                reply_delay_us,
                sequence,
                tag_id,
                initiator_id,
                responder_id,
                responder_index,
                range_source,
                range_age_slots,
            ) = TELEMETRY_PASSIVE_DS_OBSERVATION_V2_STRUCT.unpack_from(
                frame, offset
            )
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": "uwb.passive_ds.observation",
                    "tdoa_protocol": "passive_ds",
                    "slot_id": int(slot_id),
                    "diff_m": diff_mm / 1000.0,
                    "raw_diff_m": raw_diff_mm / 1000.0,
                    "anchor_distance_m": anchor_distance_mm / 1000.0,
                    "cfo_correction_m": cfo_correction_mm / 1000.0,
                    "clock_offset_ppm": clock_offset_ppb / 1000.0,
                    "reply_delay_us": int(reply_delay_us),
                    "range_source": (
                        "piggyback_ds"
                        if range_source == 1
                        else "fixed_geometry"
                        if range_source == 2
                        else "none"
                    ),
                    "range_age_slots": int(range_age_slots),
                    "seq": int(sequence),
                    "tag_id": int(tag_id),
                    "initiator_id": int(initiator_id),
                    "responder_id": int(responder_id),
                    "responder_index": int(responder_index),
                }
            )
        elif stream_type in (
            TELEMETRY_STREAM_FLEX_TDOA_OBSERVATION,
            TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION,
        ):
            (
                uptime_ms,
                slot_id,
                diff_mm,
                raw_diff_mm,
                anchor_distance_mm,
                sequence,
                tag_id,
                initiator_id,
                responder_id,
                responder_index,
            ) = TELEMETRY_FLEX_OBSERVATION_STRUCT.unpack_from(frame, offset)
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": (
                        "uwb.passive_ds.observation"
                        if stream_type == TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION
                        else "uwb.flex_tdoa.observation"
                    ),
                    "tdoa_protocol": (
                        "passive_ds"
                        if stream_type == TELEMETRY_STREAM_PASSIVE_DS_OBSERVATION
                        else "flextdoa"
                    ),
                    "slot_id": int(slot_id),
                    "diff_m": diff_mm / 1000.0,
                    "raw_diff_m": raw_diff_mm / 1000.0,
                    "anchor_distance_m": anchor_distance_mm / 1000.0,
                    "seq": int(sequence),
                    "tag_id": int(tag_id),
                    "initiator_id": int(initiator_id),
                    "responder_id": int(responder_id),
                    "responder_index": int(responder_index),
                }
            )
        elif stream_type in (
            TELEMETRY_STREAM_FLEX_ANCHOR_RANGE,
            TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE,
            TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE,
            TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE,
        ):
            (
                uptime_ms,
                slot_id,
                distance_mm,
                raw_distance_mm,
                sequence,
                initiator_id,
                responder_id,
            ) = TELEMETRY_FLEX_ANCHOR_RANGE_STRUCT.unpack_from(frame, offset)
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": {
                        TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE:
                            "uwb.passive_ds.anchor_range",
                        TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE:
                            "uwb.native_ds.anchor_range",
                        TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE:
                            "uwb.native_ds.tag_range",
                    }.get(
                        stream_type, "uwb.flex_tdoa.anchor_range"
                    ),
                    "tdoa_protocol": {
                        TELEMETRY_STREAM_PASSIVE_DS_ANCHOR_RANGE:
                            "passive_ds",
                        TELEMETRY_STREAM_NATIVE_DS_ANCHOR_RANGE:
                            "native_ds",
                        TELEMETRY_STREAM_NATIVE_DS_TAG_RANGE:
                            "native_ds",
                    }.get(stream_type, "flextdoa"),
                    "slot_id": int(slot_id),
                    "distance_m": distance_mm / 1000.0,
                    "raw_distance_m": raw_distance_mm / 1000.0,
                    "seq": int(sequence),
                    "initiator_id": int(initiator_id),
                    "responder_id": int(responder_id),
                }
            )
        elif stream_type in (
            TELEMETRY_STREAM_PASSIVE_DS_GEOMETRY,
            TELEMETRY_STREAM_NATIVE_DS_GEOMETRY,
            TELEMETRY_STREAM_FLEX_GEOMETRY,
        ):
            (
                uptime_ms,
                geometry_version,
                x_mm,
                y_mm,
                fit_rms_mm,
                anchor_id,
                anchor_count,
                tag_id,
                flags,
            ) = TELEMETRY_PASSIVE_DS_GEOMETRY_STRUCT.unpack_from(
                frame, offset
            )
            geometry_protocol = {
                TELEMETRY_STREAM_NATIVE_DS_GEOMETRY: "native_ds",
                TELEMETRY_STREAM_FLEX_GEOMETRY: "flextdoa",
            }.get(stream_type, "passive_ds")
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": f"uwb.{geometry_protocol}.geometry",
                    "tdoa_protocol": geometry_protocol,
                    "geometry_version": int(geometry_version),
                    "x_m": x_mm / 1000.0,
                    "y_m": y_mm / 1000.0,
                    "fit_rms_m": fit_rms_mm / 1000.0,
                    "anchor_id": int(anchor_id),
                    "anchor_count": int(anchor_count),
                    "tag_id": int(tag_id),
                    "dynamic": bool(flags & 1),
                }
            )
        elif stream_type in (
            TELEMETRY_STREAM_FLEX_POSITION,
            TELEMETRY_STREAM_PASSIVE_DS_POSITION,
            TELEMETRY_STREAM_NATIVE_DS_POSITION,
        ):
            (
                uptime_ms,
                slot_id,
                x_mm,
                y_mm,
                sigma_mm,
                rms_mm,
                geometry_version,
                observation_count,
                tag_id,
                anchor_count,
            ) = TELEMETRY_FLEX_POSITION_STRUCT.unpack_from(frame, offset)
            position_protocol = {
                TELEMETRY_STREAM_PASSIVE_DS_POSITION: "passive_ds",
                TELEMETRY_STREAM_NATIVE_DS_POSITION: "native_ds",
            }.get(stream_type, "flextdoa")
            position_topic = (
                "uwb.flex_tdoa.position"
                if position_protocol == "flextdoa"
                else f"uwb.{position_protocol}.position"
            )
            direct_esp_solve = position_protocol in (
                "flextdoa",
                "native_ds",
            )
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": position_topic,
                    "tdoa_protocol": position_protocol,
                    "slot_id": int(slot_id),
                    "x_m": x_mm / 1000.0,
                    "y_m": y_mm / 1000.0,
                    "sigma_m": sigma_mm / 1000.0,
                    "rms_m": rms_mm / 1000.0,
                    "geometry_version": int(geometry_version),
                    "observation_count": int(observation_count),
                    "tag_id": int(tag_id),
                    "anchor_count": int(anchor_count),
                    # FlexTDOA and Native DS-TWR publish the direct solution
                    # of one complete ESP32 frame.  Make that explicit in the
                    # API instead of letting the dashboard's legacy Passive
                    # DS fallback describe these samples as EKF output.
                    "position_filter": (
                        "none" if direct_esp_solve else "ekf_cv"
                    ),
                    "solver_location": (
                        "esp32_tag" if direct_esp_solve else "legacy_shared"
                    ),
                    "solution_kind": (
                        "independent_frame" if direct_esp_solve else "rolling"
                    ),
                    "independent_frame": direct_esp_solve,
                }
            )
        elif stream_type in (
            TELEMETRY_STREAM_PASSIVE_DS_POSITION_V2,
            TELEMETRY_STREAM_PASSIVE_DS_POSITION_V3,
            TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4,
        ):
            if stream_type == TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4:
                (
                    uptime_ms,
                    slot_id,
                    x_mm,
                    y_mm,
                    raw_x_mm,
                    raw_y_mm,
                    sigma_mm,
                    rms_mm,
                    geometry_version,
                    observation_count,
                    tag_id,
                    anchor_count,
                    solver_update_count,
                    independent_frame_count,
                    solution_flags,
                    batch_span_ms,
                    batch_max_age_ms,
                    observation_mask,
                    rejection_reason_mask,
                    rejected_since_last,
                    position_rejected_count,
                ) = TELEMETRY_PASSIVE_DS_POSITION_V4_STRUCT.unpack_from(
                    frame, offset
                )
            elif stream_type == TELEMETRY_STREAM_PASSIVE_DS_POSITION_V3:
                (
                    uptime_ms,
                    slot_id,
                    x_mm,
                    y_mm,
                    raw_x_mm,
                    raw_y_mm,
                    sigma_mm,
                    rms_mm,
                    geometry_version,
                    observation_count,
                    tag_id,
                    anchor_count,
                    solver_update_count,
                    independent_frame_count,
                    solution_flags,
                ) = TELEMETRY_PASSIVE_DS_POSITION_V3_STRUCT.unpack_from(
                    frame, offset
                )
                batch_span_ms = 0
                batch_max_age_ms = 0
                observation_mask = 0
                rejection_reason_mask = 0
                rejected_since_last = 0
                position_rejected_count = 0
            else:
                (
                    uptime_ms,
                    slot_id,
                    x_mm,
                    y_mm,
                    raw_x_mm,
                    raw_y_mm,
                    sigma_mm,
                    rms_mm,
                    geometry_version,
                    observation_count,
                    tag_id,
                    anchor_count,
                ) = TELEMETRY_PASSIVE_DS_POSITION_V2_STRUCT.unpack_from(
                    frame, offset
                )
                solver_update_count = 0
                independent_frame_count = 0
                solution_flags = 1 | 4
                batch_span_ms = 0
                batch_max_age_ms = 0
                observation_mask = 0
                rejection_reason_mask = 0
                rejected_since_last = 0
                position_rejected_count = 0
            samples.append(
                {
                    **common,
                    "uptime_ms": int(uptime_ms),
                    "topic": "uwb.passive_ds.position",
                    "tdoa_protocol": "passive_ds",
                    "slot_id": int(slot_id),
                    "x_m": x_mm / 1000.0,
                    "y_m": y_mm / 1000.0,
                    "raw_x_m": raw_x_mm / 1000.0,
                    "raw_y_m": raw_y_mm / 1000.0,
                    "sigma_m": sigma_mm / 1000.0,
                    "rms_m": rms_mm / 1000.0,
                    "geometry_version": int(geometry_version),
                    "observation_count": int(observation_count),
                    "tag_id": int(tag_id),
                    "anchor_count": int(anchor_count),
                    "position_filter": (
                        "none" if solution_flags & 16 else "ekf_cv"
                    ),
                    "solver_location": (
                        "esp32_tag" if solution_flags & 16 else "legacy_shared"
                    ),
                    "solution_kind": (
                        "complete_superframe"
                        if solution_flags & 2
                        else "independent_frame"
                        if solution_flags & 1
                        else "overlapping_raw_window"
                        if solution_flags & 16
                        else "rolling"
                    ),
                    "independent_frame": bool(solution_flags & 1),
                    "complete_superframe": bool(solution_flags & 2),
                    "filter_correction": (
                        False
                        if solution_flags & 16
                        else bool(solution_flags & 4)
                        if solution_flags & 8
                        else True
                    ),
                    "selective_filter_telemetry": bool(
                        solution_flags & 8
                    ),
                    "solver_update_count": int(solver_update_count),
                    "independent_frame_count": int(
                        independent_frame_count
                    ),
                    "batch_span_ms": int(batch_span_ms),
                    "batch_max_age_ms": int(batch_max_age_ms),
                    "observation_mask": int(observation_mask),
                    "rejection_reason_mask": int(
                        rejection_reason_mask
                    ),
                    "rejected_since_last": int(rejected_since_last),
                    "position_rejected_count": int(
                        position_rejected_count
                    ),
                    "coherent_batch_telemetry": (
                        stream_type ==
                        TELEMETRY_STREAM_PASSIVE_DS_POSITION_V4
                    ),
                }
            )
        offset += sample_size
    return samples


class DashboardState:
    def __init__(self, *, max_logs: int) -> None:
        self.lock = threading.Lock()
        self.position_condition = threading.Condition(self.lock)
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
        self.tdoa_local_positions: dict[int, dict[str, Any]] = {}
        self.tdoa_local_geometries: dict[str, dict[str, Any]] = {}
        self.tdoa_position_events: deque[dict[str, Any]] = deque(maxlen=4096)
        self.next_position_event_id = 1
        self.next_position_stream_event_id = 1
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
                topic = str(sample.get("topic") or "")
                if topic == "bno085.accel":
                    self.record_accel_sample_locked(sample)
                elif topic in (
                    "uwb.flex_tdoa.observation",
                    "uwb.passive_ds.observation",
                ):
                    self.record_tdoa_sample_locked(sample)
                elif topic in (
                    "uwb.flex_tdoa.anchor_range",
                    "uwb.passive_ds.anchor_range",
                    "uwb.native_ds.anchor_range",
                ):
                    self.record_tdoa_anchor_sample_locked(sample)
                elif topic == "uwb.native_ds.tag_range":
                    self.record_native_ds_range_sample_locked(sample)
                elif topic in (
                    "uwb.flex_tdoa.position",
                    "uwb.passive_ds.position",
                    "uwb.native_ds.position",
                ):
                    self.record_tdoa_position_sample_locked(sample)
                elif topic in (
                    "uwb.flextdoa.geometry",
                    "uwb.passive_ds.geometry",
                    "uwb.native_ds.geometry",
                ):
                    self.record_passive_ds_geometry_sample_locked(sample)

    def record_passive_ds_geometry_sample_locked(
        self, item: dict[str, Any]
    ) -> None:
        try:
            tag_id = int(item["tag_id"])
            anchor_id = int(item["anchor_id"])
            anchor_count = int(item["anchor_count"])
            version = int(item["geometry_version"])
            x_m = float(item["x_m"])
            y_m = float(item["y_m"])
            fit_rms_m = float(item["fit_rms_m"])
        except (KeyError, TypeError, ValueError):
            return
        if (
            tag_id <= 0
            or anchor_id <= 0
            or anchor_count < 3
            or not all(math.isfinite(value) for value in (x_m, y_m, fit_rms_m))
        ):
            return
        received_at = float(item.get("received_at") or time.time())
        protocol = str(item.get("tdoa_protocol") or "passive_ds")
        geometry_key = f"{protocol}:{tag_id}"
        geometry = self.tdoa_local_geometries.get(geometry_key)
        if geometry is None or int(geometry.get("geometry_version", -1)) != version:
            geometry = {
                "tag_id": tag_id,
                "tdoa_protocol": protocol,
                "geometry_version": version,
                "anchor_count": anchor_count,
                "fit_rms_m": fit_rms_m,
                "dynamic": bool(item.get("dynamic", True)),
                "anchors": {},
                "received_at": received_at,
            }
            self.tdoa_local_geometries[geometry_key] = geometry
        geometry["anchors"][str(anchor_id)] = {
            "id": anchor_id,
            "x": x_m,
            "y": y_m,
        }
        geometry["fit_rms_m"] = fit_rms_m
        geometry["received_at"] = received_at
        geometry["complete"] = len(geometry["anchors"]) == anchor_count

    def record_tdoa_position_sample_locked(self, item: dict[str, Any]) -> None:
        try:
            tag_id = int(item["tag_id"])
            x_m = float(item["x_m"])
            y_m = float(item["y_m"])
            sigma_m = float(item["sigma_m"])
            rms_m = float(item["rms_m"])
        except (KeyError, TypeError, ValueError):
            return
        if tag_id <= 0 or not all(
            math.isfinite(value) for value in (x_m, y_m, sigma_m, rms_m)
        ):
            return
        stored = {
            **item,
            "received_at": float(item.get("received_at") or time.time()),
            "position_event_id": self.next_position_event_id,
            "tdoa_protocol": str(item.get("tdoa_protocol") or "flextdoa"),
            "position_stream_type": (
                {
                    "passive_ds": "passive_ds_position",
                    "native_ds": "native_ds_position",
                }.get(item.get("tdoa_protocol"), "flextdoa_position")
            ),
            "position_stream_event_id": self.next_position_stream_event_id,
        }
        self.next_position_event_id += 1
        self.next_position_stream_event_id += 1
        self.tdoa_local_positions[tag_id] = stored
        self.tdoa_position_events.append(stored)
        self.position_condition.notify_all()

    def position_events_after(
        self, after: int, limit: int, wait_sec: float
    ) -> list[dict[str, Any]]:
        def collect() -> list[dict[str, Any]]:
            if not self.tdoa_position_events:
                return []
            newest_id = int(
                self.tdoa_position_events[-1].get("position_stream_event_id") or 0
            )
            if after <= 0 or after > newest_id:
                return [dict(self.tdoa_position_events[-1])]

            pending: list[dict[str, Any]] = []
            for item in reversed(self.tdoa_position_events):
                if int(item.get("position_stream_event_id") or 0) <= after:
                    break
                pending.append(item)
            pending.reverse()
            return [dict(item) for item in pending[:limit]]

        with self.position_condition:
            events = collect()
            if not events:
                self.position_condition.wait(timeout=max(0.0, wait_sec))
                events = collect()
            return events

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
        raw_message = str(item.get("message") or item.get("raw") or "")
        match = RANGING_RE.search(raw_message)
        if match is None:
            return

        try:
            tag_id = int(match.group("tag"))
            anchor_id = int(match.group("anchor"))
            distance_m = float(match.group("distance"))
            frame_id = int(match.group("frame_id"))
        except ValueError:
            return

        self.store_native_ds_range_locked(
            tag_id=tag_id,
            anchor_id=anchor_id,
            frame_id=frame_id,
            distance_m=distance_m,
            raw_distance_m=None,
            item=item,
            raw_message=raw_message,
        )

    def record_native_ds_range_sample_locked(
        self, item: dict[str, Any]
    ) -> None:
        try:
            tag_id = int(item["initiator_id"])
            anchor_id = int(item["responder_id"])
            frame_id = int(item["seq"])
            distance_m = float(item["distance_m"])
            raw_distance_m = float(item.get("raw_distance_m", distance_m))
        except (KeyError, TypeError, ValueError):
            return
        self.store_native_ds_range_locked(
            tag_id=tag_id,
            anchor_id=anchor_id,
            frame_id=frame_id,
            distance_m=distance_m,
            raw_distance_m=raw_distance_m,
            item=item,
            raw_message="binary native DS-TWR telemetry",
        )

    def store_native_ds_range_locked(
        self,
        *,
        tag_id: int,
        anchor_id: int,
        frame_id: int,
        distance_m: float,
        raw_distance_m: float | None,
        item: dict[str, Any],
        raw_message: str,
    ) -> None:
        if (
            tag_id <= 0
            or anchor_id <= 0
            or tag_id == anchor_id
            or not math.isfinite(distance_m)
            or distance_m <= 0
        ):
            return
        now = float(item.get("received_at") or time.time())
        key = (tag_id, anchor_id)
        previous = self.ranging_distances.get(key)
        duplicate = (
            previous is not None
            and int(previous.get("frame_id", -1)) == frame_id
            and abs(float(previous.get("distance_m", math.inf)) - distance_m)
            < 0.0005
            and now - float(previous.get("received_at") or 0.0) < 1.0
        )
        if duplicate:
            return
        sample = {
            "tag_id": tag_id,
            "anchor_id": anchor_id,
            "distance_m": distance_m,
            "raw_distance_m": (
                distance_m if raw_distance_m is None else raw_distance_m
            ),
            "frame_id": frame_id,
            "seq": frame_id,
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "raw": raw_message,
            "tdoa_protocol": "native_ds",
        }
        self.ranging_distances[key] = sample
        self.ranging_history.setdefault(
            key, deque(maxlen=self.max_ranging_samples)
        ).append(sample)
        # Native DS-TWR ranges remain available to the diagnostic tables and
        # history endpoint. Position-stream events contain only positions
        # solved by the ESP32 tag, so the browser never assembles or solves
        # native ranging frames itself.

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
            slot_id = (
                int(match.group("slot")) if match.group("slot") is not None else None
            )
            responder_index = (
                int(match.group("index")) if match.group("index") is not None else None
            )
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
            "slot_id": slot_id,
            "responder_index": responder_index,
            "diff_m": diff_m,
            "raw_diff_m": raw_diff_m,
            "primary_diff_m": primary_diff_m,
            "alt_diff_m": alt_diff_m,
            "agreement_m": agreement_m,
            "blend_weight": blend_weight,
            "fused": fused,
            "suspect": suspect,
            "anchor_distance_m": anchor_distance_m,
            "cfo_correction_m": item.get("cfo_correction_m"),
            "clock_offset_ppm": item.get("clock_offset_ppm"),
            "cfo_raw_ppm": item.get("cfo_raw_ppm"),
            "cfo_estimated_ppm": item.get("cfo_estimated_ppm"),
            "cfo_applied_ppm": item.get("cfo_applied_ppm"),
            "processing_dtu": item.get("processing_dtu"),
            "cfo_sample_count": item.get("cfo_sample_count"),
            "cfo_ready": item.get("cfo_ready"),
            "cfo_estimate_applied": item.get("cfo_estimate_applied"),
            "cfo_reset": item.get("cfo_reset"),
            "cfo_flags": item.get("cfo_flags"),
            "reply_delay_us": item.get("reply_delay_us"),
            "range_source": item.get("range_source"),
            "range_age_slots": item.get("range_age_slots"),
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "raw": raw_message,
            "tdoa_protocol": "flextdoa",
        }
        self.tdoa_observations[key] = sample
        self.tdoa_history.setdefault(
            key, deque(maxlen=self.max_tdoa_samples)
        ).append(sample)
        self.store_tdoa_anchor_distance_locked(
            initiator_id=initiator_id,
            responder_id=responder_id,
            seq=seq,
            slot_id=None,
            distance_m=anchor_distance_m,
            raw_distance_m=None,
            item=item,
            source="tdoa_obs",
        )

    def record_tdoa_sample_locked(self, item: dict[str, Any]) -> None:
        try:
            tag_id = int(item["tag_id"])
            initiator_id = int(item["initiator_id"])
            responder_id = int(item["responder_id"])
            seq = int(item["seq"])
            slot_id = int(item["slot_id"])
            responder_index = int(item["responder_index"])
            diff_m = float(item["diff_m"])
            raw_diff_m = float(item["raw_diff_m"])
            anchor_distance_m = float(item["anchor_distance_m"])
        except (KeyError, TypeError, ValueError):
            return

        if (
            tag_id <= 0
            or initiator_id <= 0
            or responder_id <= 0
            or not math.isfinite(diff_m)
            or not math.isfinite(raw_diff_m)
            or not math.isfinite(anchor_distance_m)
            or anchor_distance_m <= 0
        ):
            return

        now = float(item.get("received_at") or time.time())
        key = (tag_id, initiator_id, responder_id)
        sample = {
            "tag_id": tag_id,
            "initiator_id": initiator_id,
            "responder_id": responder_id,
            "seq": seq,
            "slot_id": slot_id,
            "responder_index": responder_index,
            "diff_m": diff_m,
            "raw_diff_m": raw_diff_m,
            "primary_diff_m": None,
            "alt_diff_m": None,
            "agreement_m": None,
            "blend_weight": None,
            "fused": False,
            "suspect": False,
            "anchor_distance_m": anchor_distance_m,
            "cfo_correction_m": item.get("cfo_correction_m"),
            "clock_offset_ppm": item.get("clock_offset_ppm"),
            "cfo_raw_ppm": item.get("cfo_raw_ppm"),
            "cfo_estimated_ppm": item.get("cfo_estimated_ppm"),
            "cfo_applied_ppm": item.get("cfo_applied_ppm"),
            "processing_dtu": item.get("processing_dtu"),
            "cfo_sample_count": item.get("cfo_sample_count"),
            "cfo_ready": item.get("cfo_ready"),
            "cfo_estimate_applied": item.get("cfo_estimate_applied"),
            "cfo_reset": item.get("cfo_reset"),
            "cfo_flags": item.get("cfo_flags"),
            "reply_delay_us": item.get("reply_delay_us"),
            "range_source": item.get("range_source"),
            "range_age_slots": item.get("range_age_slots"),
            "received_at": now,
            "log_id": None,
            "source_module_id": item.get("module_id"),
            "raw": "binary telemetry",
            "tdoa_protocol": str(item.get("tdoa_protocol") or "flextdoa"),
        }
        self.tdoa_observations[key] = sample
        self.tdoa_history.setdefault(
            key, deque(maxlen=self.max_tdoa_samples)
        ).append(sample)
        self.store_tdoa_anchor_distance_locked(
            initiator_id=initiator_id,
            responder_id=responder_id,
            seq=seq,
            slot_id=slot_id,
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
            slot_id=None,
            distance_m=distance_m,
            raw_distance_m=raw_distance_m,
            item=item,
            source="anchor_result",
        )

    def record_tdoa_anchor_sample_locked(self, item: dict[str, Any]) -> None:
        try:
            initiator_id = int(item["initiator_id"])
            responder_id = int(item["responder_id"])
            seq = int(item["seq"])
            slot_id = int(item["slot_id"])
            distance_m = float(item["distance_m"])
            raw_distance_m = float(item["raw_distance_m"])
        except (KeyError, TypeError, ValueError):
            return

        self.store_tdoa_anchor_distance_locked(
            initiator_id=initiator_id,
            responder_id=responder_id,
            seq=seq,
            slot_id=slot_id,
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
        slot_id: int | None,
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
            "slot_id": slot_id,
            "distance_m": distance_m,
            "raw_distance_m": raw_distance_m,
            "received_at": now,
            "log_id": item.get("id"),
            "source_module_id": item.get("module_id"),
            "source": source,
            "raw": item.get("raw") or item.get("message") or "",
            "tdoa_protocol": str(item.get("tdoa_protocol") or "flextdoa"),
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

    @classmethod
    def robust_spread_float(
        cls, values: list[float], center: float | None = None
    ) -> tuple[float | None, float | None]:
        if center is None:
            center = cls.median_float(values)
        if center is None:
            return None, None
        deviations = [abs(value - center) for value in values]
        mad_m = cls.median_float(deviations)
        robust_sigma_m = 1.4826 * mad_m if mad_m is not None else None
        return mad_m, robust_sigma_m

    @staticmethod
    def optional_float(value: Any) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if math.isfinite(parsed) else None

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
            mad_m, robust_sigma_m = self.robust_spread_float(
                values, median_m
            )
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
                "frame_id": int(item["frame_id"]),
                "seq": int(item["seq"]),
                "age_sec": now - float(item["received_at"]),
                "log_id": item.get("log_id"),
                "source_module_id": item.get("source_module_id"),
                "raw": item.get("raw") or "",
                "tdoa_protocol": str(
                    item.get("tdoa_protocol") or "flextdoa"
                ),
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "mad_m": mad_m,
                    "robust_sigma_m": robust_sigma_m,
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
        recent_observations: list[dict[str, Any]] = []
        recent_anchor_ranges: list[dict[str, Any]] = []
        # Retain enough anchor-range history for the receive-only protocols to
        # reconstruct their live geometry after a browser reconnect.
        max_age_sec = ANCHOR_RANGE_HISTORY_MAX_AGE_SEC
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
            mad_m, robust_sigma_m = self.robust_spread_float(
                values, median_m
            )
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
                "slot_id": item.get("slot_id"),
                "responder_index": item.get("responder_index"),
                "diff_m": float(item["diff_m"]),
                "raw_diff_m": float(item["raw_diff_m"]),
                "primary_diff_m": item.get("primary_diff_m"),
                "alt_diff_m": item.get("alt_diff_m"),
                "agreement_m": item.get("agreement_m"),
                "blend_weight": item.get("blend_weight"),
                "fused": bool(item.get("fused")),
                "suspect": bool(item.get("suspect")),
                "anchor_distance_m": float(item["anchor_distance_m"]),
                "cfo_correction_m": item.get("cfo_correction_m"),
                "clock_offset_ppm": item.get("clock_offset_ppm"),
                "cfo_raw_ppm": item.get("cfo_raw_ppm"),
                "cfo_estimated_ppm": item.get("cfo_estimated_ppm"),
                "cfo_applied_ppm": item.get("cfo_applied_ppm"),
                "processing_dtu": item.get("processing_dtu"),
                "cfo_sample_count": item.get("cfo_sample_count"),
                "cfo_ready": item.get("cfo_ready"),
                "cfo_estimate_applied": item.get("cfo_estimate_applied"),
                "cfo_reset": item.get("cfo_reset"),
                "cfo_flags": item.get("cfo_flags"),
                "reply_delay_us": item.get("reply_delay_us"),
                "range_source": item.get("range_source"),
                "range_age_slots": item.get("range_age_slots"),
                "age_sec": now - float(item["received_at"]),
                "log_id": item.get("log_id"),
                "source_module_id": item.get("source_module_id"),
                "raw": item.get("raw") or "",
                "tdoa_protocol": str(
                    item.get("tdoa_protocol") or "flextdoa"
                ),
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "mad_m": mad_m,
                    "robust_sigma_m": robust_sigma_m,
                    "std_m": std_m,
                    "min_m": min(values) if values else None,
                    "max_m": max(values) if values else None,
                },
            }
            for sample in history:
                age_sec = now - float(sample.get("received_at") or 0.0)
                if age_sec > 10.0:
                    continue
                recent_observations.append(
                    {
                        "tag_id": int(sample["tag_id"]),
                        "initiator_id": int(sample["initiator_id"]),
                        "responder_id": int(sample["responder_id"]),
                        "seq": int(sample["seq"]),
                        "slot_id": sample.get("slot_id"),
                        "responder_index": sample.get("responder_index"),
                        "diff_m": float(sample["diff_m"]),
                        "raw_diff_m": float(sample["raw_diff_m"]),
                        "primary_diff_m": sample.get("primary_diff_m"),
                        "anchor_distance_m": float(sample["anchor_distance_m"]),
                        "cfo_correction_m": sample.get(
                            "cfo_correction_m"
                        ),
                        "clock_offset_ppm": sample.get(
                            "clock_offset_ppm"
                        ),
                        "cfo_raw_ppm": sample.get("cfo_raw_ppm"),
                        "cfo_estimated_ppm": sample.get(
                            "cfo_estimated_ppm"
                        ),
                        "cfo_applied_ppm": sample.get(
                            "cfo_applied_ppm"
                        ),
                        "processing_dtu": sample.get("processing_dtu"),
                        "cfo_sample_count": sample.get(
                            "cfo_sample_count"
                        ),
                        "cfo_ready": sample.get("cfo_ready"),
                        "cfo_estimate_applied": sample.get(
                            "cfo_estimate_applied"
                        ),
                        "cfo_reset": sample.get("cfo_reset"),
                        "cfo_flags": sample.get("cfo_flags"),
                        "reply_delay_us": sample.get("reply_delay_us"),
                        "range_source": sample.get("range_source"),
                        "range_age_slots": sample.get(
                            "range_age_slots"
                        ),
                        "age_sec": age_sec,
                        "received_at": float(sample.get("received_at") or 0.0),
                        "log_id": sample.get("log_id"),
                        "tdoa_protocol": str(
                            sample.get("tdoa_protocol") or "flextdoa"
                        ),
                    }
                )
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
            mad_m, robust_sigma_m = self.robust_spread_float(
                values, median_m
            )
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
                "tdoa_protocol": str(
                    item.get("tdoa_protocol") or "flextdoa"
                ),
                "stats": {
                    "samples": len(values),
                    "mean_m": mean_m,
                    "median_m": median_m,
                    "mad_m": mad_m,
                    "robust_sigma_m": robust_sigma_m,
                    "std_m": std_m,
                    "min_m": min(values) if values else None,
                    "max_m": max(values) if values else None,
                },
            }
            for sample in history:
                slot_id = sample.get("slot_id")
                age_sec = now - float(sample.get("received_at") or 0.0)
                if slot_id is None or age_sec > max_age_sec or sample.get("source") != "anchor_result":
                    continue
                recent_anchor_ranges.append(
                    {
                        "anchor_a_id": int(sample["anchor_a_id"]),
                        "anchor_b_id": int(sample["anchor_b_id"]),
                        "initiator_id": int(sample["initiator_id"]),
                        "responder_id": int(sample["responder_id"]),
                        "seq": int(sample["seq"]),
                        "slot_id": int(slot_id),
                        "distance_m": float(sample["distance_m"]),
                        "raw_distance_m": sample.get("raw_distance_m"),
                        "age_sec": age_sec,
                        "received_at": float(sample.get("received_at") or 0.0),
                        "tdoa_protocol": str(
                            sample.get("tdoa_protocol") or "flextdoa"
                        ),
                    }
                )
        return {
            "observations": observations,
            "recent_observations": sorted(
                recent_observations,
                key=lambda sample: float(sample.get("received_at") or 0.0),
            )[-300:],
            "recent_anchor_ranges": sorted(
                recent_anchor_ranges,
                key=lambda sample: float(sample.get("received_at") or 0.0),
            )[-600:],
            "anchor_distances": anchor_distances,
            "local_positions": {
                str(tag_id): {
                    **item,
                    "age_sec": now - float(item.get("received_at") or 0.0),
                }
                for tag_id, item in self.tdoa_local_positions.items()
            },
            "local_geometries": {
                str(tag_id): {
                    **item,
                    "age_sec": now - float(item.get("received_at") or 0.0),
                }
                for tag_id, item in self.tdoa_local_geometries.items()
            },
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
                )
                item["http_status_age_sec"] = age_sec
                item["http_status_online"] = online
                item["http_status_degraded"] = bool(error) and online
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
        if not self.targets:
            self.stop_event.wait(self.interval)
            return

        spacing = max(0.05, self.interval / len(self.targets))
        pending: dict[str, Any] = {}
        with ThreadPoolExecutor(
            max_workers=max(1, len(self.targets)), thread_name_prefix="status"
        ) as executor:
            while not self.stop_event.is_set():
                for target in self.targets:
                    previous = pending.get(target)
                    if previous is None or previous.done():
                        pending[target] = executor.submit(self.poll_target, target)
                    if self.stop_event.wait(spacing):
                        return

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
<link rel="icon" href="data:,">
<link rel="stylesheet" href="/vendor/leaflet/leaflet.css">
<script src="/vendor/leaflet/leaflet.min.js"></script>
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
.app { min-width: 0; min-height: 100vh; display: grid; grid-template-columns: minmax(0, 1fr); grid-template-rows: auto auto 1fr; }
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
#graphs, #gps { overflow: auto; }
#map { overflow: hidden; }
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
.gps-dashboard {
  width: 100%;
  min-width: 0;
  min-height: 100%;
  overflow: hidden;
  background: var(--panel);
  border: 1px solid var(--line);
}
.gps-overview {
  display: grid;
  grid-template-columns: repeat(4, minmax(130px, 190px)) minmax(280px, 1fr);
  align-items: stretch;
  border-bottom: 1px solid var(--line);
}
.gps-stat {
  padding: 14px 16px;
  border-right: 1px solid var(--line);
}
.gps-stat span {
  display: block;
  color: var(--muted);
  font-size: 12px;
  margin-bottom: 4px;
}
.gps-stat b { font-size: 20px; }
.gps-overview-note {
  display: flex;
  align-items: center;
  padding: 12px 16px;
  color: var(--muted);
  font-size: 12px;
  line-height: 1.45;
}
.gps-table-wrap { width: 100%; max-width: 100%; overflow-x: auto; padding: 0 12px 12px; }
.gps-status-table { table-layout: fixed; min-width: 1180px; }
.gps-status-table th,
.gps-status-table td { overflow-wrap: anywhere; }
.gps-status-table .col-module { width: 13%; }
.gps-status-table .col-receiver { width: 14%; }
.gps-status-table .col-fix { width: 13%; }
.gps-status-table .col-satellites { width: 10%; }
.gps-status-table .col-position { width: 19%; }
.gps-status-table .col-navigation { width: 14%; }
.gps-status-table .col-stream { width: 17%; }
.gps-primary-value { font-weight: 700; font-size: 14px; }
.gps-detail { color: var(--muted); font-size: 12px; line-height: 1.45; }
.gps-fix-message { display: inline-block; margin-top: 2px; font-weight: 700; }
.gps-fix-explanation { display: inline-block; margin: 2px 0; font-size: 12px; line-height: 1.35; }
.gps-coordinates { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
.gps-map-layout {
  height: 100%;
  min-height: 0;
  display: grid;
  grid-template-columns: minmax(0, 1fr) 340px;
  gap: 12px;
}
.gps-map-stage {
  position: relative;
  min-width: 0;
  min-height: 0;
  overflow: hidden;
  border: 1px solid var(--line);
  background:
    linear-gradient(rgba(217, 222, 232, 0.5) 1px, transparent 1px),
    linear-gradient(90deg, rgba(217, 222, 232, 0.5) 1px, transparent 1px),
    #eef2f7;
  background-size: 40px 40px;
}
#gpsMapCanvas { width: 100%; height: 100%; min-height: 520px; }
.gps-map-banner {
  position: absolute;
  z-index: 800;
  top: 12px;
  left: 50%;
  transform: translateX(-50%);
  max-width: min(620px, calc(100% - 120px));
  padding: 8px 12px;
  border: 1px solid #e5a84a;
  background: rgba(255, 249, 235, 0.96);
  color: var(--orange);
  font-size: 12px;
  box-shadow: 0 2px 8px rgba(23, 32, 42, 0.12);
}
.gps-map-banner[hidden] { display: none; }
.gps-map-panel {
  min-width: 0;
  height: 100%;
  overflow: auto;
  border: 1px solid var(--line);
  background: var(--panel);
  padding: 12px;
}
.gps-map-panel h2 { margin: 0 0 10px; font-size: 15px; }
.gps-map-controls { display: flex; flex-wrap: wrap; gap: 7px; margin-bottom: 10px; }
.gps-map-legend {
  display: flex;
  flex-wrap: wrap;
  gap: 8px 14px;
  padding: 9px 0 11px;
  color: var(--muted);
  font-size: 12px;
  border-bottom: 1px solid var(--line);
}
.gps-map-legend span { display: inline-flex; align-items: center; gap: 5px; }
.gps-map-dot { width: 11px; height: 11px; border-radius: 50%; display: inline-block; }
.gps-map-dot.tag { background: #d7352a; }
.gps-map-dot.anchor { background: #16833a; }
.gps-map-dot.stale { background: #8792a2; }
.gps-map-summary { margin: 10px 0; font-size: 13px; line-height: 1.45; }
.gps-map-distance-section {
  margin: 12px 0;
  padding-top: 10px;
  border-top: 1px solid var(--line);
}
.gps-map-distance-section h3 { margin: 0 0 4px; font-size: 13px; }
.gps-map-distance-note { margin-bottom: 7px; color: var(--muted); font-size: 11px; line-height: 1.4; }
.gps-map-distance-table { width: 100%; font-size: 12px; }
.gps-map-distance-table th,
.gps-map-distance-table td { padding: 4px 5px; text-align: right; white-space: nowrap; }
.gps-map-distance-table th:first-child,
.gps-map-distance-table td:first-child { text-align: left; }
.gps-map-distance-tooltip {
  padding: 1px 4px;
  border: 1px solid rgba(43, 100, 216, 0.45);
  background: rgba(255, 255, 255, 0.92);
  color: #173c83;
  font-size: 11px;
  font-weight: 700;
  box-shadow: 0 1px 3px rgba(23, 32, 42, 0.15);
}
.gps-map-distance-tooltip::before { display: none; }
.gps-map-module-list { display: grid; gap: 8px; }
.gps-map-module {
  padding: 9px 10px;
  border: 1px solid var(--line);
  background: #fbfcfe;
  font-size: 12px;
  line-height: 1.45;
}
.gps-map-module-head { display: flex; align-items: baseline; justify-content: space-between; gap: 8px; }
.gps-map-module-name { font-weight: 700; font-size: 14px; }
.gps-map-coords { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
.gps-map-links { display: flex; gap: 10px; margin-top: 5px; }
.gps-map-links a { color: var(--blue); text-decoration: none; }
.gps-map-links a:hover { text-decoration: underline; }
.gps-map-div-icon { background: transparent; border: 0; }
.gps-map-marker {
  position: relative;
  width: 28px;
  height: 28px;
  transform: translate(-2px, -2px);
}
.gps-map-marker-pin {
  position: absolute;
  left: 5px;
  top: 5px;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  border: 3px solid #fff;
  box-shadow: 0 1px 5px rgba(23, 32, 42, 0.5);
  background: #16833a;
}
.gps-map-marker.tag .gps-map-marker-pin { background: #d7352a; }
.gps-map-marker.stale .gps-map-marker-pin { background: #8792a2; }
.gps-map-marker-label {
  position: absolute;
  left: 25px;
  top: 4px;
  padding: 1px 4px;
  border-radius: 3px;
  background: rgba(255,255,255,0.9);
  color: #17202a;
  font-size: 12px;
  font-weight: 700;
  white-space: nowrap;
  box-shadow: 0 1px 3px rgba(23, 32, 42, 0.16);
}
.gps-map-marker-label.left { left: auto; right: 25px; }
.gps-map-marker-label.top { top: -11px; }
.gps-map-marker-label.bottom { top: 18px; }
.leaflet-container { font-family: inherit; background: transparent; }
.leaflet-popup-content { margin: 10px 12px; line-height: 1.45; }
@media (max-width: 980px) {
  .gps-map-layout { grid-template-columns: minmax(0, 1fr); grid-template-rows: minmax(520px, 65vh) auto; }
  .gps-map-panel { height: auto; max-height: none; }
}
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
.profile-validation {
  display: inline-block;
  margin: 0 0 10px;
  padding: 3px 7px;
  border: 1px solid var(--line);
  background: #fff;
  font-size: 12px;
  font-weight: 700;
}
.profile-validation.good { border-color: #78bb98; background: #f2fbf6; color: #116f3b; }
.profile-validation.warn { border-color: #d8b55f; background: #fffaf0; color: #805b00; }
.profile-validation.bad { border-color: #e2a4a4; background: #fff5f5; color: #a32626; }
.ranging-protocol-context {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
}
.ranging-protocol-context h2 { margin-bottom: 3px; }
.ranging-protocol-badge {
  flex: 0 0 auto;
  border: 1px solid #78bb98;
  background: #f2fbf6;
  color: #116f3b;
  padding: 5px 8px;
  font-size: 12px;
  font-weight: 700;
}
.ranging-protocol-empty {
  max-width: 760px;
  color: var(--muted);
  line-height: 1.5;
}
.profile-summary {
  color: var(--muted);
  font-size: 12px;
  line-height: 1.4;
  margin-top: 10px;
  padding-top: 9px;
  border-top: 1px solid var(--line);
}
.flex-timing-head {
  display: flex;
  align-items: end;
  justify-content: space-between;
  gap: 16px;
  margin-bottom: 12px;
}
.flex-timing-head h2 { margin-bottom: 2px; }
.flex-timing-select {
  display: grid;
  grid-template-columns: auto minmax(170px, 240px);
  align-items: center;
  gap: 8px;
}
.flex-timing-select label { color: var(--muted); font-size: 12px; }
.flex-timing-metrics {
  display: grid;
  grid-template-columns: repeat(6, minmax(100px, 1fr));
  border: 1px solid var(--line);
  background: #fbfcfe;
  margin-bottom: 14px;
}
.flex-timing-metric {
  min-width: 0;
  padding: 9px 11px;
  border-right: 1px solid var(--line);
}
.flex-timing-metric:last-child { border-right: 0; }
.flex-timing-metric span {
  display: block;
  color: var(--muted);
  font-size: 11px;
  margin-bottom: 3px;
}
.flex-timing-metric strong { font-size: 14px; }
.flex-timing-scroll { overflow-x: auto; padding-bottom: 5px; }
.flex-timing-canvas { min-width: 920px; }
.flex-timing-label {
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 12px;
  margin: 11px 0 6px;
}
.flex-timing-label strong { font-size: 13px; }
.flex-timing-label span { color: var(--muted); font-size: 11px; }
.flex-frame-track {
  display: grid;
  border: 1px solid #8d9caf;
  background: #fff;
}
.flex-frame-slot {
  min-width: 0;
  min-height: 76px;
  padding: 9px 10px;
  border-right: 1px solid #8d9caf;
  background: #f7f9fc;
}
.flex-frame-slot:last-child { border-right: 0; }
.flex-frame-slot.selected {
  background: #edf4ff;
  box-shadow: inset 0 -3px 0 #2e67d1;
}
.flex-frame-slot.live { box-shadow: inset 0 3px 0 #16894b; }
.flex-frame-slot.selected.live {
  box-shadow: inset 0 3px 0 #16894b, inset 0 -3px 0 #2e67d1;
}
.ds-frame-gap {
  min-width: 72px;
  min-height: 76px;
  padding: 9px 8px;
  border-left: 1px solid #8d9caf;
  background: #fff8df;
  color: #67540f;
}
.ds-frame-gap b,
.ds-frame-gap strong,
.ds-frame-gap span { display: block; }
.ds-frame-gap b { font-size: 12px; }
.ds-frame-gap strong { font-size: 14px; margin-top: 2px; }
.ds-frame-gap span { font-size: 11px; margin-top: 5px; }
.flex-frame-slot b { display: block; font-size: 12px; }
.flex-frame-slot strong { display: block; font-size: 14px; margin-top: 2px; }
.flex-frame-slot span {
  display: block;
  color: var(--muted);
  font-size: 11px;
  margin-top: 5px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.flex-frame-axis,
.flex-slot-axis {
  position: relative;
  height: 25px;
  color: var(--muted);
  font-size: 10px;
}
.flex-slot-axis { height: 38px; }
.flex-axis-mark.stagger { top: 13px; }
.flex-axis-mark {
  position: absolute;
  top: 0;
  transform: translateX(-50%);
  white-space: nowrap;
}
.flex-axis-mark::before {
  content: "";
  display: block;
  width: 1px;
  height: 5px;
  margin: 0 auto 2px;
  background: #8d9caf;
}
.flex-axis-mark.edge-start { transform: none; }
.flex-axis-mark.edge-end { transform: translateX(-100%); }
.flex-dimensions {
  position: relative;
  height: 63px;
  margin-top: 1px;
  color: #42536a;
  font-size: 10px;
}
.flex-dimension {
  position: absolute;
  height: 23px;
}
.flex-dimension-line {
  position: absolute;
  left: 0;
  right: 0;
  top: 7px;
  border-top: 1px solid #52647a;
}
.flex-dimension-line::before,
.flex-dimension-line::after {
  content: "";
  position: absolute;
  top: -4px;
  width: 0;
  height: 0;
  border-top: 4px solid transparent;
  border-bottom: 4px solid transparent;
}
.flex-dimension-line::before {
  left: 0;
  border-left: 6px solid #52647a;
}
.flex-dimension-line::after {
  right: 0;
  border-right: 6px solid #52647a;
}
.flex-dimension-label {
  position: absolute;
  left: 50%;
  top: 0;
  transform: translateX(-50%);
  padding: 0 5px;
  background: #fff;
  font-weight: 700;
  white-space: nowrap;
}
.flex-dimension.gap .flex-dimension-line { border-color: #7a651c; }
.flex-dimension.gap .flex-dimension-line::before { border-left-color: #7a651c; }
.flex-dimension.gap .flex-dimension-line::after { border-right-color: #7a651c; }
.flex-dimension.gap .flex-dimension-label {
  left: auto;
  right: 0;
  transform: none;
  color: #67540f;
}
.flex-frame-dimensions {
  position: relative;
  height: 45px;
  margin-top: 1px;
  color: #42536a;
  font-size: 10px;
}
.flex-round-gap-zero {
  position: absolute;
  right: 0;
  top: 24px;
  height: 17px;
  border-right: 3px double #52647a;
}
.flex-round-gap-zero span {
  position: absolute;
  right: 7px;
  top: -1px;
  white-space: nowrap;
  font-weight: 700;
}
.flex-slot-track {
  display: grid;
  height: 76px;
  border: 1px solid #8d9caf;
  background: #fff;
}
.flex-slot-segment {
  min-width: 0;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  padding: 5px 3px;
  border-right: 1px solid rgba(41, 54, 73, 0.34);
  text-align: center;
  overflow: hidden;
}
.flex-slot-segment:last-child { border-right: 0; }
.flex-slot-segment b { font-size: 13px; white-space: nowrap; }
.flex-slot-segment span { font-size: 11px; margin-top: 3px; white-space: nowrap; }
.flex-slot-segment.req { color: #fff; background: #2e67d1; }
.flex-slot-segment.req-process { color: #443307; background: #f1cf72; }
.flex-slot-segment.response { color: #fff; background: #16894b; }
.flex-slot-segment.response.alt { background: #147b72; }
.flex-slot-segment.response-process { color: #263548; background: #dce4ed; }
.flex-slot-segment.guard { color: #263548; background: #eef1f5; }
.flex-timing-detail-grid {
  display: grid;
  grid-template-columns: minmax(520px, 1.25fr) minmax(300px, 0.75fr);
  gap: 14px;
  margin-top: 12px;
  align-items: start;
}
.flex-timing-table { width: 100%; font-size: 11px; }
.flex-timing-table th,
.flex-timing-table td { padding: 5px 7px; }
.flex-timing-table td:first-child { font-weight: 700; }
.flex-packet-flow {
  border-left: 3px solid #2e67d1;
  padding: 2px 0 2px 11px;
}
.flex-packet-row { margin-bottom: 9px; }
.flex-packet-row:last-child { margin-bottom: 0; }
.flex-packet-row b { display: block; font-size: 12px; }
.flex-packet-row code {
  display: block;
  margin-top: 3px;
  color: #42536a;
  font-size: 10px;
  line-height: 1.4;
  white-space: normal;
}
.flex-timing-note {
  margin-top: 10px;
  color: var(--muted);
  font-size: 11px;
  line-height: 1.4;
}
.flex-host-window {
  display: grid;
  grid-template-columns: 150px minmax(360px, 1fr) 260px;
  align-items: center;
  gap: 10px;
  margin: 3px 0 13px;
  font-size: 11px;
}
.flex-host-window strong { font-size: 12px; }
.flex-host-window > span { color: var(--muted); text-align: right; }
.flex-host-track {
  position: relative;
  height: 18px;
  border: 1px dashed #6f7e91;
  background: #fff;
  overflow: hidden;
}
.flex-host-fill {
  height: 100%;
  min-width: 2px;
  background: #e4edff;
  border-right: 2px solid #2e67d1;
}
.flex-host-measure {
  position: relative;
  height: 25px;
  margin-top: 3px;
}
.flex-host-measure .flex-dimension-line { top: 7px; }
.flex-host-measure .flex-dimension-label {
  font-size: 10px;
  color: #2455ae;
}
.flex-parameter-map { margin-top: 13px; }
.flex-parameter-table { width: 100%; font-size: 11px; }
.flex-parameter-table th,
.flex-parameter-table td { padding: 6px 8px; vertical-align: top; }
.flex-parameter-table td:nth-child(1) { width: 120px; font-weight: 700; }
.flex-parameter-table td:nth-child(2) { width: 230px; }
.flex-scope {
  display: inline-block;
  padding: 1px 5px;
  border: 1px solid #b8c4d3;
  color: #42536a;
  background: #f7f9fc;
  font-size: 10px;
  font-weight: 700;
}
.flex-scope.host { border-color: #8aa9e8; color: #2455ae; background: #f1f6ff; }
.flex-scope.fixed { border-color: #78bb98; color: #116f3b; background: #f2fbf6; }
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
.position-metric-note {
  margin: 7px 0 0;
  color: var(--muted);
  font-size: 11px;
  line-height: 1.35;
}
.position-error-table {
  table-layout: fixed;
  font-size: 12px;
}
.position-error-table th,
.position-error-table td {
  padding: 6px 3px;
  text-align: right;
  white-space: nowrap;
}
.position-error-table th:first-child,
.position-error-table td:first-child {
  text-align: left;
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
  .flex-timing-head { align-items: stretch; flex-direction: column; }
  .flex-timing-select { grid-template-columns: 100px minmax(0, 1fr); }
  .flex-timing-metrics { grid-template-columns: repeat(3, 1fr); }
  .flex-timing-metric:nth-child(3) { border-right: 0; }
  .flex-timing-metric:nth-child(-n+3) { border-bottom: 1px solid var(--line); }
  .flex-timing-detail-grid { grid-template-columns: 1fr; }
  .flex-host-window { grid-template-columns: 120px minmax(300px, 1fr) 220px; }
  .gps-overview { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .gps-stat:nth-child(2n) { border-right: 0; }
  .gps-overview-note { grid-column: 1 / -1; border-top: 1px solid var(--line); }
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
    <button class="tab" data-tab="gps">GPS</button>
    <button class="tab" data-tab="map">Map</button>
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
              <p>Apply the selected position runtime to compute a new live position from fresh measurements.</p>
              <button class="primary" id="positionEnableRanging">Apply</button>
            </div>
          </div>
        </div>
        <aside class="position-panel">
          <h2>Position Setup</h2>
          <div class="form-grid">
            <label for="positionAnchorCount">Anchors used</label>
            <select id="positionAnchorCount"><option value="4">4 anchors</option><option value="3">3 anchors</option></select>
            <label for="positionSolver">Ranging method</label>
            <select id="positionSolver"><option value="flextdoa" selected>FlexTDOA</option><option value="passive_ds">Passive DS-TWR</option><option value="ranging">Native DS-TWR</option></select>
            <label for="positionAnchors">Anchor IDs</label>
            <input id="positionAnchors" value="2,3,4,5">
            <label for="positionTags">Tag IDs</label>
            <input id="positionTags" value="1">
            <label for="positionMaxAgeSec">Fresh age s</label>
            <input id="positionMaxAgeSec" value="3" type="number" min="0.2" step="0.1">
            <label for="positionReferenceMode">Known reference</label>
            <select id="positionReferenceMode">
              <option value="gps_rtk" selected>GPS RTK tag (time aligned)</option>
              <option value="centroid">anchor centroid</option>
              <option value="manual">manual coordinates</option>
              <option value="none">disabled</option>
            </select>
            <label for="positionReferenceX">Reference X m</label>
            <input id="positionReferenceX" value="1.50" type="number" step="0.001">
            <label for="positionReferenceY">Reference Y m</label>
            <input id="positionReferenceY" value="1.50" type="number" step="0.001">
            <label for="positionErrorWindowSec">Error window s</label>
            <input id="positionErrorWindowSec" value="30" type="number" min="1" max="120" step="1">
          </div>
          <div class="form-actions">
            <button id="positionResetTrail">Reset Trail</button>
            <button class="dynamic-geometry-option" id="positionRestartAnchorSelfLocalization">Reset Live Geometry Estimate</button>
            <button class="primary" id="positionEnableRangingSide">Apply</button>
          </div>
          <div class="param-legend">
            <div><b>Anchors</b><span>The first 3 or 4 IDs from the list are used for solving the position.</span></div>
            <div><b>Ranging method</b><span>FlexTDOA and Passive DS-TWR use receive-only tags. Native DS-TWR ranges each active tag to every anchor. Each protocol keeps independent timing profiles.</span></div>
            <div class="native-ds-position-option"><b>Native calculation</b><span>The ESP32 tag solves each complete coherent frame. The Raspberry only receives and displays positions and diagnostics.</span></div>
            <div><b>Tags</b><span>Comma separated tag IDs. In both passive protocols every non-anchor module only listens on UWB, so additional tags consume no radio slots.</span></div>
            <div><b>Geometry</b><span>FlexTDOA uses fixed GPS RTK ENU anchor geometry. Passive and Native DS-TWR retain their protocol-specific range geometry.</span></div>
            <div><b>Known reference</b><span>GPS RTK compares each UWB point with the nearest RTK-fixed tag sample in the same ENU frame. Centroid and manual coordinates remain available for static tests.</span></div>
          </div>
          <div id="positionToast" class="toast"></div>
          <div class="section" style="margin-top:12px;">
            <h2>Anchor Geometry</h2>
            <div id="positionGeometryStatus" class="muted" style="margin-bottom:8px;">Waiting for live anchor ranges.</div>
            <div id="positionGeometryTableNote" class="position-metric-note" style="margin-bottom:8px;"></div>
            <table>
              <thead id="positionGeometryHead"><tr><th>Pair</th><th>live range / stable</th><th>range robust σ</th><th>age</th><th>fit</th></tr></thead>
              <tbody id="positionGeometryRows"></tbody>
            </table>
          </div>
          <div class="section">
            <h2>Live Position</h2>
            <div class="position-legend"><span style="color:#d7352a">live marker (all updates)</span><span style="color:#7b8798">pre-filter current (when available)</span><span style="color:#2b64d8">displayed independent trail</span><span style="color:#7b8798">pre-filter independent trail</span><span style="color:#6d4c9f">known reference</span><span class="ring" style="color:#2b64d8">anchor drift</span><span style="color:#16833a">anchor</span></div>
            <div id="positionReadout" class="position-readout"></div>
            <table>
              <thead><tr><th>Tag</th><th>axis σ</th><th>equation RMS</th><th>max residual</th></tr></thead>
              <tbody id="positionAccuracyRows"></tbody>
            </table>
            <p class="position-metric-note">Solver diagnostics come from equation residuals; they are not measured position error. FlexTDOA currently reports equation RMS, not coordinate-axis uncertainty.</p>
            <div id="positionReferenceStatus" class="muted" style="margin:12px 0 5px;">Known position reference disabled.</div>
            <table class="position-error-table">
              <thead><tr><th>Tag</th><th>now</th><th>bias</th><th>RMSE</th><th>P95</th><th>max</th></tr></thead>
              <tbody id="positionErrorRows"></tbody>
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
              <label for="accelEnabled">Sensor</label>
              <div class="checkbox-row"><input id="accelEnabled" type="checkbox"><span id="accelEnabledLabel">BNO085 disabled</span></div>
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
    <section id="gps" class="page">
      <div class="gps-dashboard">
        <div class="gps-overview">
          <div class="gps-stat"><span>Receivers enabled</span><b id="gpsEnabledCount">0 / 5</b></div>
          <div class="gps-stat"><span>NMEA streaming</span><b id="gpsStreamingCount">0 / 5</b></div>
          <div class="gps-stat"><span>Valid fixes</span><b id="gpsFixCount">0 / 5</b></div>
          <div class="gps-stat"><span>Satellites used</span><b id="gpsSatelliteCount">0</b></div>
          <div id="gpsOverviewNote" class="gps-overview-note">Waiting for module status...</div>
        </div>
        <div class="gps-table-wrap">
          <table class="gps-status-table">
            <colgroup>
              <col class="col-module">
              <col class="col-receiver">
              <col class="col-fix">
              <col class="col-satellites">
              <col class="col-position">
              <col class="col-navigation">
              <col class="col-stream">
            </colgroup>
            <thead>
              <tr><th>Module</th><th>Receiver</th><th>Fix</th><th>Satellites</th><th>Position</th><th>Navigation</th><th>NMEA stream</th></tr>
            </thead>
            <tbody id="gpsRows"></tbody>
          </table>
        </div>
      </div>
    </section>
    <section id="map" class="page">
      <div class="gps-map-layout">
        <div class="gps-map-stage">
          <div id="gpsMapCanvas" aria-label="Live GPS map"></div>
          <div id="gpsMapBanner" class="gps-map-banner" hidden></div>
        </div>
        <aside class="gps-map-panel">
          <h2>Absolute GPS Positions</h2>
          <div class="gps-map-controls">
            <button class="primary" id="gpsMapFitAll">Fit all modules</button>
            <button id="gpsMapCenterTag">Center on tag</button>
            <button id="gpsMapClearTrail">Clear tag trail</button>
          </div>
          <div class="checkbox-row">
            <input id="gpsMapFollowTag" type="checkbox">
            <label for="gpsMapFollowTag">Follow tag</label>
          </div>
          <div class="checkbox-row">
            <input id="gpsMapShowTrail" type="checkbox" checked>
            <label for="gpsMapShowTrail">Show tag trail</label>
          </div>
          <div class="checkbox-row">
            <input id="gpsMapShowDistances" type="checkbox" checked>
            <label for="gpsMapShowDistances">Show GPS pair distances</label>
          </div>
          <div class="gps-map-legend">
            <span><i class="gps-map-dot tag"></i>tag</span>
            <span><i class="gps-map-dot anchor"></i>anchor</span>
            <span><i class="gps-map-dot stale"></i>last known / stale</span>
          </div>
          <div id="gpsMapSummary" class="gps-map-summary muted">Waiting for GPS fixes...</div>
          <div class="gps-map-distance-section">
            <h3>GPS pair distances</h3>
            <div class="gps-map-distance-note">Direct horizontal distance from fresh GPS coordinates. 3D also includes the reported altitude difference.</div>
            <div id="gpsMapDistanceList" class="muted">Waiting for at least two fresh fixes...</div>
          </div>
          <div class="gps-map-distance-section">
            <h3>FlexTDOA RTK geometry</h3>
            <div class="gps-map-distance-note">Collects only RTK-fixed (quality 4) samples. Robust median anchor coordinates are converted to one ENU frame with M2 as origin. Tag GPS is shown only as ground truth.</div>
            <div id="gpsRtkGeometryStatus" class="muted">Waiting for RTK-fixed anchor samples...</div>
            <div class="form-actions" style="margin-top:8px">
              <button class="primary" id="gpsApplyFlexGeometry" disabled>Apply RTK geometry to all modules</button>
              <button id="gpsClearRtkGeometrySamples">Clear RTK samples</button>
            </div>
            <div id="gpsRtkGeometryToast" class="toast"></div>
          </div>
          <div id="gpsMapModuleList" class="gps-map-module-list"></div>
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
        <div class="section ranging-protocol-context">
          <div>
            <h2 id="rangingProtocolTitle">FlexTDOA Settings</h2>
            <div id="rangingProtocolHint" class="muted">Protocol selected in Position Setup.</div>
          </div>
          <span id="rangingProtocolBadge" class="ranging-protocol-badge">FlexTDOA</span>
        </div>
        <div id="flexTdoaRangingPanel" class="section ranging-protocol-panel" data-ranging-protocol="flextdoa">
          <div class="flex-timing-head">
            <div>
              <h2>FlexTDOA Protocol Timing</h2>
              <div class="muted">Live CI-CR frame structure and tuned DW3000 delayed-TX timing.</div>
            </div>
            <div class="flex-timing-select">
              <label for="flexTimingSlotSelect">Inspect slot</label>
              <select id="flexTimingSlotSelect"></select>
            </div>
          </div>
          <div id="flexTdoaTimingDiagram" class="muted">Waiting for FlexTDOA runtime status...</div>
        </div>
        <div id="nativeDsTwrRangingPanel" class="section ranging-protocol-panel hidden" data-ranging-protocol="ranging">
          <div class="flex-timing-head">
            <div>
              <h2>Native DS-TWR Protocol Timing</h2>
              <div class="muted">Live POLL → RESP → FINAL → RESULT frame structure, read from the active modules.</div>
            </div>
            <div class="flex-timing-select">
              <label for="dsTimingSlotSelect">Inspect exchange</label>
              <select id="dsTimingSlotSelect"></select>
            </div>
          </div>
          <div id="nativeDsTwrTimingDiagram" class="muted">Waiting for native DS-TWR runtime status...</div>
        </div>
        <div id="passiveDsRangingPanel" class="section ranging-protocol-panel hidden" data-ranging-protocol="passive_ds">
          <div class="flex-timing-head">
            <div>
              <h2>Passive DS-TWR Protocol Timing</h2>
              <div class="muted">Three on-air packets per anchor-pair exchange: POLL → RESP → FINAL. Receive-only tags listen to all three packets and transmit none.</div>
            </div>
          </div>
          <div id="passiveDsTimingDiagram" class="muted">Waiting for Passive DS-TWR runtime status...</div>
          <div class="profile-card" style="margin-top:12px">
            <h3>Experimental pipeline and position-window policy</h3>
            <p class="muted">For Multipoint Full-DS, Single coherent star produces one independent position from each radio exchange. Three-star precision preserves the validated static reference. Neither option applies a temporal position filter.</p>
            <div class="form-grid">
              <label for="passiveDsExperimentTargets">Targets</label>
              <select id="passiveDsExperimentTargets">
                <option value="all">all modules</option>
                <option value="1">module 1</option>
                <option value="2">module 2</option>
                <option value="3">module 3</option>
                <option value="4">module 4</option>
                <option value="5">module 5</option>
              </select>
              <label for="passiveDsPipelineMode">Anchor pipeline</label>
              <select id="passiveDsPipelineMode">
                <option value="0">Legacy control</option>
                <option value="1">DW3000 deadline state machine</option>
              </select>
              <label for="passiveDsSolveMode">Position-window policy</label>
              <select id="passiveDsSolveMode">
                <option value="0">Single coherent star (dynamic)</option>
                <option value="1">Legacy mixed rolling (A/B control)</option>
                <option value="2">Three-star precision window</option>
                <option value="3">Coherent superframe correction + prediction</option>
                <option value="4">Motion-compensated rolling (experimental)</option>
              </select>
              <label for="passiveDsRollingMaxHz">Rolling cap Hz</label>
              <input id="passiveDsRollingMaxHz" value="100" type="number" min="1" max="500" step="1">
            </div>
            <div class="form-actions">
              <button id="applyPassiveDsExperimentMode" class="primary">Apply position-window policy</button>
            </div>
            <div id="passiveDsExperimentToast" class="toast"></div>
          </div>
          <div class="profile-card" style="margin-top:12px">
            <h3>Pipeline diagnostics</h3>
            <p class="muted">Cumulative in-RAM counters. Stage timing is measured locally and exported only through the normal status poll; no per-packet log is generated.</p>
            <div id="passiveDsPipelineDiagnostics" class="muted">Waiting for instrumented firmware status...</div>
          </div>
        </div>
        <div id="flexTdoaProfilesSection" class="section">
          <h2>FlexTDOA Frame Profiles</h2>
          <div class="form-grid">
            <label for="flexProfileTargets">Targets</label>
            <select id="flexProfileTargets">
              <option value="all">all modules</option>
              <option value="1">module 1</option>
              <option value="2">module 2</option>
              <option value="3">module 3</option>
              <option value="4">module 4</option>
              <option value="5">module 5</option>
            </select>
          </div>
          <p class="muted profile-note">Each card is a complete on-air CI-CR timing profile. Applying it writes the timing to ESP NVS and reboots the selected modules.</p>
          <div class="profile-grid">
            <div class="profile-card flex-profile-card" data-flex-profile="paper20200">
              <h3>20.20 ms Paper Reference</h3>
              <p class="muted">Equation (20) timing from the FlexTDOA paper for four anchors and K=3.</p>
              <div class="profile-validation good">reference default · paper-faithful timing</div>
              <div class="form-grid compact">
                <label for="flexProfile20200GuardUs">Guard us</label>
                <input id="flexProfile20200GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile20200ReqUs">REQ us</label>
                <input id="flexProfile20200ReqUs" value="2000" type="number" min="1" max="65535" step="10">
                <label for="flexProfile20200ReqProcessUs">Process REQ us</label>
                <input id="flexProfile20200ReqProcessUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile20200RespUs">RESP subslot us</label>
                <input id="flexProfile20200RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile20200RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile20200RespProcessUs" value="600" type="number" min="1" max="65535" step="10">
                <label for="flexProfile20200RxSliceMs">RX host slice ms</label>
                <input id="flexProfile20200RxSliceMs" value="6" type="number" min="1" max="60000" step="1">
                <label for="flexProfile20200FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile20200FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile20200Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="paper20200">Apply Paper Reference</button>
                <button class="reset-flex-profile" data-flex-profile="paper20200">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card flex-profile-card" data-flex-profile="frame14800">
              <h3>14.80 ms Frame</h3>
              <p class="muted">Fastest clean steady-state profile measured on all five modules.</p>
              <div class="profile-validation good">recommended · 0 TX failures / 60 s</div>
              <div class="form-grid compact">
                <label for="flexProfile14800GuardUs">Guard us</label>
                <input id="flexProfile14800GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14800ReqUs">REQ us</label>
                <input id="flexProfile14800ReqUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14800ReqProcessUs">Process REQ us</label>
                <input id="flexProfile14800ReqProcessUs" value="1250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14800RespUs">RESP subslot us</label>
                <input id="flexProfile14800RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14800RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile14800RespProcessUs" value="400" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14800RxSliceMs">RX host slice ms</label>
                <input id="flexProfile14800RxSliceMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="flexProfile14800FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile14800FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile14800Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="frame14800">Apply 14.80 ms</button>
                <button class="reset-flex-profile" data-flex-profile="frame14800">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card flex-profile-card" data-flex-profile="frame14200">
              <h3>14.20 ms Frame</h3>
              <p class="muted">First timing boundary below the recommended profile.</p>
              <div class="profile-validation warn">borderline · 1 delayed-TX failure / 30 s</div>
              <div class="form-grid compact">
                <label for="flexProfile14200GuardUs">Guard us</label>
                <input id="flexProfile14200GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14200ReqUs">REQ us</label>
                <input id="flexProfile14200ReqUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14200ReqProcessUs">Process REQ us</label>
                <input id="flexProfile14200ReqProcessUs" value="1250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14200RespUs">RESP subslot us</label>
                <input id="flexProfile14200RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14200RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile14200RespProcessUs" value="350" type="number" min="1" max="65535" step="10">
                <label for="flexProfile14200RxSliceMs">RX host slice ms</label>
                <input id="flexProfile14200RxSliceMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="flexProfile14200FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile14200FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile14200Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="frame14200">Apply 14.20 ms</button>
                <button class="reset-flex-profile" data-flex-profile="frame14200">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card flex-profile-card" data-flex-profile="frame13600">
              <h3>13.60 ms Frame</h3>
              <p class="muted">Higher response rate with insufficient delayed-TX margin.</p>
              <div class="profile-validation warn">borderline · 1 delayed-TX failure / 30 s</div>
              <div class="form-grid compact">
                <label for="flexProfile13600GuardUs">Guard us</label>
                <input id="flexProfile13600GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile13600ReqUs">REQ us</label>
                <input id="flexProfile13600ReqUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile13600ReqProcessUs">Process REQ us</label>
                <input id="flexProfile13600ReqProcessUs" value="1250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile13600RespUs">RESP subslot us</label>
                <input id="flexProfile13600RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile13600RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile13600RespProcessUs" value="300" type="number" min="1" max="65535" step="10">
                <label for="flexProfile13600RxSliceMs">RX host slice ms</label>
                <input id="flexProfile13600RxSliceMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="flexProfile13600FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile13600FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile13600Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="frame13600">Apply 13.60 ms</button>
                <button class="reset-flex-profile" data-flex-profile="frame13600">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card flex-profile-card" data-flex-profile="frame12600">
              <h3>12.60 ms Frame</h3>
              <p class="muted">Limit-search profile that repeatedly missed delayed TX.</p>
              <div class="profile-validation bad">failed · 2 delayed-TX failures / 30 s</div>
              <div class="form-grid compact">
                <label for="flexProfile12600GuardUs">Guard us</label>
                <input id="flexProfile12600GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12600ReqUs">REQ us</label>
                <input id="flexProfile12600ReqUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12600ReqProcessUs">Process REQ us</label>
                <input id="flexProfile12600ReqProcessUs" value="1000" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12600RespUs">RESP subslot us</label>
                <input id="flexProfile12600RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12600RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile12600RespProcessUs" value="300" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12600RxSliceMs">RX host slice ms</label>
                <input id="flexProfile12600RxSliceMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="flexProfile12600FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile12600FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile12600Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="frame12600">Apply 12.60 ms</button>
                <button class="reset-flex-profile" data-flex-profile="frame12600">Reset Defaults</button>
              </div>
            </div>
            <div class="profile-card flex-profile-card" data-flex-profile="frame12000">
              <h3>12.00 ms Frame</h3>
              <p class="muted">One-kilohertz response-rate experiment beyond the robust limit.</p>
              <div class="profile-validation bad">failed · 3 TX failures / 30 s</div>
              <div class="form-grid compact">
                <label for="flexProfile12000GuardUs">Guard us</label>
                <input id="flexProfile12000GuardUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12000ReqUs">REQ us</label>
                <input id="flexProfile12000ReqUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12000ReqProcessUs">Process REQ us</label>
                <input id="flexProfile12000ReqProcessUs" value="1000" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12000RespUs">RESP subslot us</label>
                <input id="flexProfile12000RespUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12000RespProcessUs">Process RESP us / response</label>
                <input id="flexProfile12000RespProcessUs" value="250" type="number" min="1" max="65535" step="10">
                <label for="flexProfile12000RxSliceMs">RX host slice ms</label>
                <input id="flexProfile12000RxSliceMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="flexProfile12000FreshAgeSec">Observation freshness s</label>
                <input id="flexProfile12000FreshAgeSec" value="0.5" type="number" min="0.1" step="0.1">
              </div>
              <div class="profile-summary" id="flexProfile12000Summary"></div>
              <div class="form-actions">
                <button class="primary apply-flex-profile" data-flex-profile="frame12000">Apply 12.00 ms</button>
                <button class="reset-flex-profile" data-flex-profile="frame12000">Reset Defaults</button>
              </div>
            </div>
          </div>
          <div class="form-actions">
            <button id="resetAllFlexProfiles">Reset FlexTDOA Profile Defaults</button>
          </div>
          <div id="flexProfileToast" class="toast"></div>
        </div>
        <div id="passiveDsProfilesSection" class="section hidden">
          <h2>Passive DS-TWR Frame Profiles</h2>
          <div class="form-grid">
            <label for="passiveDsProfileTargets">Targets</label>
            <select id="passiveDsProfileTargets">
              <option value="all">all modules</option>
              <option value="1">module 1</option>
              <option value="2">module 2</option>
              <option value="3">module 3</option>
              <option value="4">module 4</option>
              <option value="5">module 5</option>
            </select>
          </div>
          <p class="muted profile-note">Fast Star and Robust Rotating use one native three-packet exchange per anchor pair. Multipoint Full-DS combines the same full double-sided equations into one broadcast POLL, three staggered RESP frames and one broadcast FINAL, while every tag remains receive-only.</p>
          <div id="passiveDsActiveProfile" class="profile-validation">Waiting for live Passive DS-TWR timing...</div>
          <div class="form-actions">
            <button class="primary apply-passive-ds-quick-profile" data-passive-ds-profile="fast">Apply Fast Star · 10 ms Maximum</button>
            <button class="primary apply-passive-ds-quick-profile" data-passive-ds-profile="robust">Apply Robust Rotating · 10 ms Maximum</button>
            <button class="primary apply-passive-ds-quick-profile" data-passive-ds-profile="multi_precision">Apply Multipoint · Static Precision</button>
            <button class="primary apply-passive-ds-quick-profile" data-passive-ds-profile="multi_dynamic">Apply Multipoint · Single-Star Dynamic</button>
          </div>
          <div class="profile-grid">
            <div class="profile-card passive-ds-profile-card" data-passive-ds-profile="fast">
              <h3>Fast Star</h3>
              <p class="muted">The first configured anchor initiates one exchange to every other anchor for three frames; the fourth is a rotating geometry-maintenance frame.</p>
              <div class="profile-validation">implementation baseline · hardware validation pending</div>
              <div class="form-grid compact">
                <label for="passiveDsFastSpeedPreset">Speed preset</label><select id="passiveDsFastSpeedPreset" class="passive-ds-speed-preset"><option value="safe">16 ms Safe · 62.5 Hz</option><option value="balanced">13 ms Balanced · 76.9 Hz</option><option value="maximum">10 ms Maximum · 100 Hz</option><option value="custom">Custom timing</option></select>
                <label for="passiveDsFastSlotMs">Slot ms</label><input id="passiveDsFastSlotMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="passiveDsFastGapMs">Frame gap ms</label><input id="passiveDsFastGapMs" value="1" type="number" min="1" max="60000" step="1">
                <label for="passiveDsFastRxMs">Anchor RX slice ms</label><input id="passiveDsFastRxMs" value="100" type="number" min="1" max="60000" step="1">
                <label for="passiveDsFastTimeoutMs">RX timeout ms</label><input id="passiveDsFastTimeoutMs" value="4" type="number" min="1" max="60000" step="1">
                <label for="passiveDsFastRespUs">RESP delay µs</label><input id="passiveDsFastRespUs" value="1000" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsFastFinalUs">FINAL delay µs</label><input id="passiveDsFastFinalUs" value="1000" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsFastAutoRxUus">Auto RX delay UUS</label><input id="passiveDsFastAutoRxUus" value="500" type="number" min="0" max="65535" step="10">
                <label for="passiveDsFastFreshSec">Observation freshness s</label><input id="passiveDsFastFreshSec" value="0.2" type="number" min="0.2" step="0.1">
              </div>
              <div class="profile-summary" id="passiveDsFastSummary"></div>
              <div class="form-actions"><button class="primary apply-passive-ds-profile" data-passive-ds-profile="fast">Apply Fast Star</button><button class="reset-passive-ds-profile" data-passive-ds-profile="fast">Reset Defaults</button></div>
            </div>
            <div class="profile-card passive-ds-profile-card" data-passive-ds-profile="robust">
              <h3>Robust Rotating</h3>
              <p class="muted">The reference rotates through the configured anchor order after every frame. Every position frame remains solvable, with diversified directed paths over a superframe.</p>
              <div class="profile-validation">diversity profile · hardware validation pending</div>
              <div class="form-grid compact">
                <label for="passiveDsRobustSpeedPreset">Speed preset</label><select id="passiveDsRobustSpeedPreset" class="passive-ds-speed-preset"><option value="safe">16 ms Safe · 62.5 Hz</option><option value="balanced">13 ms Balanced · 76.9 Hz</option><option value="maximum">10 ms Maximum · 100 Hz</option><option value="custom">Custom timing</option></select>
                <label for="passiveDsRobustSlotMs">Slot ms</label><input id="passiveDsRobustSlotMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="passiveDsRobustGapMs">Frame gap ms</label><input id="passiveDsRobustGapMs" value="1" type="number" min="1" max="60000" step="1">
                <label for="passiveDsRobustRxMs">Anchor RX slice ms</label><input id="passiveDsRobustRxMs" value="100" type="number" min="1" max="60000" step="1">
                <label for="passiveDsRobustTimeoutMs">RX timeout ms</label><input id="passiveDsRobustTimeoutMs" value="4" type="number" min="1" max="60000" step="1">
                <label for="passiveDsRobustRespUs">RESP delay µs</label><input id="passiveDsRobustRespUs" value="1000" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsRobustFinalUs">FINAL delay µs</label><input id="passiveDsRobustFinalUs" value="1000" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsRobustAutoRxUus">Auto RX delay UUS</label><input id="passiveDsRobustAutoRxUus" value="500" type="number" min="0" max="65535" step="10">
                <label for="passiveDsRobustFreshSec">Observation freshness s</label><input id="passiveDsRobustFreshSec" value="0.2" type="number" min="0.2" step="0.1">
              </div>
              <div class="profile-summary" id="passiveDsRobustSummary"></div>
              <div class="form-actions"><button class="primary apply-passive-ds-profile" data-passive-ds-profile="robust">Apply Robust Rotating</button><button class="reset-passive-ds-profile" data-passive-ds-profile="robust">Reset Defaults</button></div>
            </div>
            <div class="profile-card passive-ds-profile-card" data-passive-ds-profile="multi">
              <h3>Multipoint Full-DS · N+2</h3>
              <p class="muted">One rotating reference broadcasts POLL, all three other anchors answer in native delayed-TX slots, then the reference broadcasts one aggregate FINAL. Three coherent full-DS passive observations from only five packets.</p>
              <div class="profile-validation">experimental N+2 protocol · isolated hardware validation</div>
              <div class="form-grid compact">
                <label for="passiveDsMultiSolveMode">Position window</label><select id="passiveDsMultiSolveMode"><option value="2">Three-star precision (validated static reference)</option><option value="0">Single coherent star (dynamic, no overlap)</option></select>
                <label for="passiveDsMultiSlotMs">Exchange budget ms</label><input id="passiveDsMultiSlotMs" value="8" type="number" min="5" max="60000" step="1">
                <label for="passiveDsMultiGapMs">Frame gap ms</label><input id="passiveDsMultiGapMs" value="1" type="number" min="1" max="60000" step="1">
                <label for="passiveDsMultiRxMs">Anchor RX slice ms</label><input id="passiveDsMultiRxMs" value="100" type="number" min="1" max="60000" step="1">
                <label for="passiveDsMultiTimeoutMs">RX timeout ms</label><input id="passiveDsMultiTimeoutMs" value="5" type="number" min="1" max="60000" step="1">
                <label for="passiveDsMultiRespUs">First RESP delay µs</label><input id="passiveDsMultiRespUs" value="1500" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsMultiFinalUs">Last RESP → FINAL µs</label><input id="passiveDsMultiFinalUs" value="1500" type="number" min="100" max="1000000" step="50">
                <label for="passiveDsMultiAutoRxUus">Auto RX delay UUS</label><input id="passiveDsMultiAutoRxUus" value="500" type="number" min="0" max="65535" step="10">
                <label for="passiveDsMultiFreshSec">Observation freshness s</label><input id="passiveDsMultiFreshSec" value="0.2" type="number" min="0.05" step="0.05">
              </div>
              <div class="profile-summary" id="passiveDsMultiSummary"></div>
              <div class="form-actions"><button class="primary apply-passive-ds-profile" data-passive-ds-profile="multi">Apply Multipoint Full-DS</button><button class="reset-passive-ds-profile" data-passive-ds-profile="multi">Reset Defaults</button></div>
            </div>
          </div>
          <div class="profile-card" style="margin-top:12px">
            <h3>Passive DS-TWR calibration</h3>
            <p class="muted">Optional hardware-bias calibration for receive-only tags. Apply it to each passive tag, not to the ranging anchors. Anchor bias values are ordered like the configured anchors. Range bias values use unordered pair order: A1-A2, A1-A3, …, A2-A3, … . The first anchor bias must be zero.</p>
            <div class="form-grid">
              <label for="passiveDsCalibrationTargets">Passive tag target</label>
              <select id="passiveDsCalibrationTargets">
                <option value="1" selected>module 1 (current tag)</option>
                <option value="2">module 2</option>
                <option value="3">module 3</option>
                <option value="4">module 4</option>
                <option value="5">module 5</option>
                <option value="all">all modules (advanced)</option>
              </select>
              <label for="passiveDsAnchorBiasMm">Anchor observation bias mm</label>
              <input id="passiveDsAnchorBiasMm" value="0,34,-16,-55">
              <label for="passiveDsRangeBiasMm">Anchor-pair range bias mm</label>
              <input id="passiveDsRangeBiasMm" value="71,56,-49,67,66,-29">
            </div>
            <div id="passiveDsCalibrationStatus" class="profile-summary">calibration status unavailable</div>
            <div class="form-actions">
              <button id="applyPassiveDsCalibration" class="primary">Apply Calibration</button>
              <button id="clearPassiveDsCalibration">Clear Calibration</button>
            </div>
          </div>
          <div id="passiveDsProfileToast" class="toast"></div>
        </div>
        <div id="rangingProfilesSection" class="section">
          <h2>Native DS-TWR Frame Profiles</h2>
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
          <p id="rangingProfileNote" class="muted profile-note">Each slot contains POLL, RESP, FINAL and one one-shot RESULT. A complete frame ranges the tag to every configured anchor, then applies the frame gap.</p>
          <div id="rangingActiveProfile" class="profile-validation">Waiting for live Native DS-TWR timing...</div>
          <div id="rangingProfileGrid" class="profile-grid"></div>
          <div class="profile-card" style="margin-top:12px">
            <h3>Native DS-TWR range calibration</h3>
            <p class="muted">Static per-anchor range bias, ordered like the configured anchor IDs. Each value is measured range minus RTK truth in millimetres and is subtracted before the raw independent-frame solver. The defaults below were validated on channel 9 with the 46 ms profile. This is a hardware/timing calibration, not a temporal filter.</p>
            <div class="form-grid">
              <label for="nativeDsCalibrationTargets">Targets</label>
              <select id="nativeDsCalibrationTargets">
                <option value="all" selected>all modules</option>
                <option value="1">module 1</option>
                <option value="2">module 2</option>
                <option value="3">module 3</option>
                <option value="4">module 4</option>
                <option value="5">module 5</option>
              </select>
              <label for="nativeDsRangeBiasMm">Range bias mm</label>
              <input id="nativeDsRangeBiasMm" value="-13,92,39,-17">
            </div>
            <div id="nativeDsCalibrationStatus" class="profile-summary">calibration status unavailable</div>
            <div class="form-actions">
              <button id="applyNativeDsCalibration" class="primary">Apply Calibration</button>
              <button id="clearNativeDsCalibration">Clear Calibration</button>
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
              <h2>FlexTDOA Network</h2>
              <div class="form-grid">
                <label for="uwbFlexAnchors">Anchor IDs</label>
                <input id="uwbFlexAnchors" value="2,3,4,5">
                <label for="uwbFlexK">Responders K</label>
                <input id="uwbFlexK" value="3" type="number" min="1" max="9" step="1">
                <label for="uwbFlexSlots">Slot initiators</label>
                <input id="uwbFlexSlots" value="2,3,4,5">
                <label for="uwbFlexMasks">Responder masks</label>
                <input id="uwbFlexMasks" value="14,13,11,7">
                <label for="uwbFlexGeneration">Generation</label>
                <input id="uwbFlexGeneration" value="-" readonly>
              </div>
              <div class="param-legend">
                <div><b>N</b><span>The number of IDs in Anchor IDs. Firmware supports 3 to 10 anchors.</span></div>
                <div><b>K</b><span>How many responders are selected in every request slot.</span></div>
                <div><b>M</b><span>The number of comma separated Slot initiators.</span></div>
                <div><b>Masks</b><span>One decimal or 0x hexadecimal bit mask per slot. Bits follow the Anchor IDs order; the initiator bit must be clear.</span></div>
                <div><b>Propagation</b><span>Write one module and reboot it. A newer generation is rebroadcast over UWB and persisted by the other FlexTDOA nodes.</span></div>
              </div>
            </div>
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
                <label for="uwbRadioPhyMode">Radio PHY</label>
                <select id="uwbRadioPhyMode">
                  <option value="0">Fast · 6.8 Mb/s · preamble 128</option>
                  <option value="1">Long range · 850 kb/s · preamble 1024</option>
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
              <h2>Native DS-TWR Ranging Timing</h2>
              <div class="form-grid">
                <label for="uwbRangingSlotMs">Slot ms</label>
                <input id="uwbRangingSlotMs" value="30" type="number" min="1" step="1">
                <label for="uwbRangingGapMs">Frame gap ms</label>
                <input id="uwbRangingGapMs" value="4" type="number" min="1" step="1">
                <label for="uwbRangingRxMs">Anchor RX slice ms</label>
                <input id="uwbRangingRxMs" value="10" type="number" min="1" step="1">
                <label for="uwbRangingTimeoutMs">RX timeout ms</label>
                <input id="uwbRangingTimeoutMs" value="12" type="number" min="1" step="1">
                <label for="uwbRangingRespDelayMs">RESP delay ms</label>
                <input id="uwbRangingRespDelayMs" value="5" type="number" min="1" step="1">
                <label for="uwbRangingFinalDelayMs">FINAL delay ms</label>
                <input id="uwbRangingFinalDelayMs" value="5" type="number" min="1" step="1">
                <label for="uwbRangingAutoRxDelayUus">Auto RX delay UUS</label>
                <input id="uwbRangingAutoRxDelayUus" value="500" type="number" min="1" step="1">
              </div>
              <div class="param-legend">
                <div><b>Native exchange</b><span>Position ranging uses POLL, RESP, FINAL and one one-shot RESULT carrying the raw range. Session and frame IDs prevent stale reports.</span></div>
                <div><b>Frame</b><span>For N anchors, frame duration is N × Slot + Frame gap.</span></div>
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
                  <option value="passive_ds_twr">Passive DS-TWR</option>
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
const requestedInitialTab = String(location.hash || "").replace(/^#/, "");
const state = {
  activeTab: (requestedInitialTab && document.getElementById(requestedInitialTab))
    ? requestedInitialTab
    : (localStorage.getItem("uwbDash.activeTab") || "logs12"),
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
  logFetchPending: false,
  snapshotFetchPending: false,
  latestAccelRenderMs: 0,
  hydratedSettings: false,
  calibrationResult: null,
  ranging: {distances: {}, max_age_sec: 3},
  tdoa: {observations: {}, anchor_distances: {}, local_positions: {}, local_geometries: {}, max_age_sec: 3},
  positionTrail: {},
  positionRawTrail: {},
  positionTrailTokens: {},
  positionAnchorTrail: {},
  positionResults: {},
  positionModel: null,
  positionWasActive: false,
  positionGeometry: {key: "", ekf: null},
  positionSeeds: {},
  positionStream: null,
  positionStreamConnected: false,
  positionStreamRenderPending: false,
  positionStreamRxTimes: [],
  positionStreamIndependentTimes: [],
  positionStreamSuperframeTimes: [],
  positionStreamCorrectionTimes: [],
  positionStreamRenderTimes: [],
  positionStreamRenderLatencies: [],
  positionStreamLatestEvent: null,
  positionStreamLastRenderedEventToken: "",
  positionSettingsSolver: null,
  positionSetupDirty: false,
  positionApplyInFlight: false,
  flexTimingSlotIndex: Number(localStorage.getItem("uwbDash.setting.flexTimingSlotSelect") || 0),
  dsTimingSlotIndex: Number(localStorage.getItem("uwbDash.setting.dsTimingSlotSelect") || 0),
  gpsMap: null,
  gpsMapTileLayer: null,
  gpsMapMarkers: new Map(),
  gpsMapLastValid: new Map(),
  gpsMapTrail: [],
  gpsMapTrailLayer: null,
  gpsMapAnchorPolygon: null,
  gpsMapDistanceLayer: null,
  gpsMapLastTrailToken: "",
  gpsMapHasFit: false,
  gpsMapTileErrors: 0,
  gpsRtkSamples: new Map(),
  gpsRtkLastTokens: new Map(),
};
const accelLineRe = /\bBNO085 accel x=([-+]?\d+(?:\.\d+)?) y=([-+]?\d+(?:\.\d+)?) z=([-+]?\d+(?:\.\d+)?) m\/s\^2 accuracy=(\d+) reports=(\d+)/;
const maxAccelSamples = 30000;
const maxSeriesPoints = 1600;
const maxTerminalRenderLines = 1000;
const positionTrailMaxAgeSec = 120;
const positionTrailMaxPoints = 12000;
const positionTrailMaxDrawPoints = 2500;
const plot = {left: 52, right: 704, top: 14, bottom: 166, width: 652, height: 152};
const toastTimers = new Map();
let calibrationPollTimer = null;
let calibrationJobId = null;
const rangingProfileFields = [
  {key: "dsPositionMaxAgeSec", suffix: "DsFreshAgeSec", label: "Distance freshness s", min: 0.1, step: 0.1},
  {key: "slotMs", suffix: "SlotMs", label: "Slot ms", min: 1, step: 1},
  {key: "roundGapMs", suffix: "RoundGapMs", label: "Frame gap ms", min: 1, step: 1},
  {key: "dsRxSliceMs", suffix: "DsRxSliceMs", label: "Anchor RX slice ms", min: 1, step: 1},
  {key: "timeoutMs", suffix: "TimeoutMs", label: "RX timeout ms", min: 1, step: 1},
  {key: "respDelayMs", suffix: "RespDelayMs", label: "RESP delay ms", min: 1, step: 1},
  {key: "finalDelayMs", suffix: "FinalDelayMs", label: "FINAL delay ms", min: 1, step: 1},
  {key: "autoRxDelayUus", suffix: "AutoRxDelayUus", label: "Auto RX delay UUS", min: 1, step: 1},
];
const rangingProfileDefaults = {
  reference100: {
    prefix: "profileReference100",
    label: "100 ms RTK-Validated Baseline",
    description: "Field-validated four-packet Native DS-TWR baseline. Each anchor has a 100 ms exchange slot; four anchors plus the 10 ms frame gap produce a 410 ms complete position frame.",
    validationClass: "good",
    validationText: "channel 9 · 1.17 cm RTK RMSE · no position filter",
    buttonLabel: "Apply 100 ms Reference",
    dsPositionMaxAgeSec: 0.5,
    slotMs: 100,
    roundGapMs: 10,
    dsRxSliceMs: 100,
    timeoutMs: 30,
    respDelayMs: 20,
    finalDelayMs: 20,
    autoRxDelayUus: 500,
  },
  frame64: {
    prefix: "profileFrame64",
    label: "64 ms Validated Intermediate",
    description: "Four 15 ms tag-anchor slots plus a 4 ms frame gap. This is the intermediate step between the conservative and high-rate profiles.",
    validationClass: "good",
    validationText: "channel 9 · 1.45 cm RTK RMSE · zero delayed-TX/overruns",
    buttonLabel: "Apply 64 ms Intermediate",
    dsPositionMaxAgeSec: 0.2,
    slotMs: 15,
    roundGapMs: 4,
    dsRxSliceMs: 100,
    timeoutMs: 8,
    respDelayMs: 2,
    finalDelayMs: 2,
    autoRxDelayUus: 500,
  },
  frame46: {
    prefix: "profileFrame46",
    label: "46 ms RTK-Validated Fast",
    description: "Four 11 ms tag-anchor slots plus a 2 ms frame gap, paced by the high-resolution blocking timer. This is the recommended high-rate profile.",
    validationClass: "good",
    validationText: "channel 9 · 19.16 positions/s · 1.76 cm RTK RMSE · no filter",
    buttonLabel: "Apply 46 ms Fast",
    dsPositionMaxAgeSec: 0.2,
    slotMs: 11,
    roundGapMs: 2,
    dsRxSliceMs: 100,
    timeoutMs: 5,
    respDelayMs: 2,
    finalDelayMs: 2,
    autoRxDelayUus: 500,
  },
};

function renderNativeDsActiveProfile() {
  const root = document.getElementById("rangingActiveProfile");
  if (!root) return;
  const statuses = state.statuses.filter(item =>
    statusIsFresh(item) && item.runtime_mode_name === "uwb_ranging"
  );
  if (!statuses.length) {
    root.textContent = "Native DS-TWR is not active on any fresh module.";
    root.className = "profile-validation warn";
    return;
  }
  const timing = item => ({
    slotMs: Number(item.runtime_ranging_slot_ms),
    roundGapMs: Number(item.runtime_ranging_round_gap_ms),
    dsRxSliceMs: Number(item.runtime_ranging_rx_slice_ms),
    timeoutMs: Number(item.runtime_ranging_rx_timeout_ms),
    respDelayMs: Number(item.runtime_ranging_resp_delay_ms),
    finalDelayMs: Number(item.runtime_ranging_final_delay_ms),
    autoRxDelayUus: Number(item.runtime_ranging_auto_rx_delay_uus),
  });
  const live = timing(statuses[0]);
  const fields = [
    "slotMs", "roundGapMs", "dsRxSliceMs", "timeoutMs",
    "respDelayMs", "finalDelayMs", "autoRxDelayUus",
  ];
  const consistent = statuses.every(item => {
    const candidate = timing(item);
    return fields.every(field => candidate[field] === live[field]);
  });
  const match = Object.values(rangingProfileDefaults).find(profile =>
    fields.every(field => Number(profile[field]) === live[field])
  );
  const profileLabel = match?.label || "Custom Native DS-TWR timing";
  root.textContent = consistent
    ? `Active on ${statuses.length}/${state.statuses.length || statuses.length} modules: ${profileLabel} · slot ${live.slotMs} ms · gap ${live.roundGapMs} ms · timeout ${live.timeoutMs} ms · RESP/FINAL/RESULT ${live.respDelayMs}+${live.finalDelayMs}+${live.respDelayMs} ms.`
    : `Native DS-TWR timing differs between the ${statuses.length} active modules.`;
  root.className = `profile-validation ${consistent && match ? match.validationClass : "warn"}`;
}

function renderRangingProfileCards() {
  const grid = document.getElementById("rangingProfileGrid");
  if (!grid) return;
  grid.innerHTML = Object.entries(rangingProfileDefaults).map(([key, profile]) => {
    const fields = rangingProfileFields.map(field => {
      const id = `${profile.prefix}${field.suffix}`;
      return `
        <label for="${esc(id)}">${esc(field.label)}</label>
        <input id="${esc(id)}" value="${esc(profile[field.key])}" type="number"
          min="${esc(field.min)}" step="${esc(field.step)}">`;
    }).join("");
    return `
      <div class="profile-card" data-profile="${esc(key)}">
        <h3>${esc(profile.label)}</h3>
        <p id="${esc(profile.prefix)}Description" class="muted">${esc(profile.description)}</p>
        <div class="profile-validation ${esc(profile.validationClass)}">${esc(profile.validationText)}</div>
        <div class="form-grid compact">${fields}</div>
        <div class="profile-summary" id="${esc(profile.prefix)}Summary"></div>
        <div class="form-actions">
          <button class="primary apply-ranging-profile" data-profile="${esc(key)}">${esc(profile.buttonLabel)}</button>
          <button class="reset-ranging-profile" data-profile="${esc(key)}">Reset Defaults</button>
        </div>
      </div>`;
  }).join("");
}
const flexProfileFields = [
  {key: "guardUs", suffix: "GuardUs"},
  {key: "requestUs", suffix: "ReqUs"},
  {key: "requestProcessUs", suffix: "ReqProcessUs"},
  {key: "responseUs", suffix: "RespUs"},
  {key: "responseProcessUs", suffix: "RespProcessUs"},
  {key: "rxSliceMs", suffix: "RxSliceMs"},
  {key: "positionMaxAgeSec", suffix: "FreshAgeSec"},
];
const flexProfileDefaults = {
  paper20200: {
    prefix: "flexProfile20200",
    label: "20.20 ms Paper Reference",
    guardUs: 250,
    requestUs: 2000,
    requestProcessUs: 250,
    responseUs: 250,
    responseProcessUs: 600,
    rxSliceMs: 6,
    positionMaxAgeSec: 0.5,
  },
  frame14800: {
    prefix: "flexProfile14800",
    label: "14.80 ms Frame",
    guardUs: 250,
    requestUs: 250,
    requestProcessUs: 1250,
    responseUs: 250,
    responseProcessUs: 400,
    rxSliceMs: 5,
    positionMaxAgeSec: 0.5,
  },
  frame14200: {
    prefix: "flexProfile14200",
    label: "14.20 ms Frame",
    guardUs: 250,
    requestUs: 250,
    requestProcessUs: 1250,
    responseUs: 250,
    responseProcessUs: 350,
    rxSliceMs: 5,
    positionMaxAgeSec: 0.5,
  },
  frame13600: {
    prefix: "flexProfile13600",
    label: "13.60 ms Frame",
    guardUs: 250,
    requestUs: 250,
    requestProcessUs: 1250,
    responseUs: 250,
    responseProcessUs: 300,
    rxSliceMs: 5,
    positionMaxAgeSec: 0.5,
  },
  frame12600: {
    prefix: "flexProfile12600",
    label: "12.60 ms Frame",
    guardUs: 250,
    requestUs: 250,
    requestProcessUs: 1000,
    responseUs: 250,
    responseProcessUs: 300,
    rxSliceMs: 5,
    positionMaxAgeSec: 0.5,
  },
  frame12000: {
    prefix: "flexProfile12000",
    label: "12.00 ms Frame",
    guardUs: 250,
    requestUs: 250,
    requestProcessUs: 1000,
    responseUs: 250,
    responseProcessUs: 250,
    rxSliceMs: 5,
    positionMaxAgeSec: 0.5,
  },
};
const rangingProtocolProfileFields = {
  flextdoa: new Set(),
  passive_ds: new Set(),
  ranging: new Set([
    "dsPositionMaxAgeSec", "slotMs", "roundGapMs", "dsRxSliceMs", "timeoutMs",
    "respDelayMs", "finalDelayMs", "autoRxDelayUus",
  ]),
};
const rangingProfileDefaultsVersion = "2026-08-05-native-ds-speed-v3";
const flexProfileDefaultsVersion = "2026-08-04-flextdoa-paper-reference-v3";
const passiveDsProfileDefaultsVersion = "2026-08-01-passive-ds-single-star-v2";
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
  let latest = null;
  const items = [];
  for (let i = state.logs.length - 1; i >= 0; i--) {
    const log = state.logs[i];
    if (!latest && terminalMatchesModule(term, log)) latest = log;
    if (logAllowed(term, log)) {
      items.push(log);
      if (items.length >= maxTerminalRenderLines && latest) break;
    }
  }
  items.reverse();
  term.meta.textContent = latest ? `last log ${fmtAge(latest.received_at)} · #${latest.id}` : "no logs yet";
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

function parseCoordinateList(text, expected) {
  const values = String(text || "")
    .split(/[,;\s]+/)
    .map(value => Number(value.trim()))
    .filter(Number.isFinite);
  return values.length === expected ? values : [];
}

function normalizePositionSolver(value) {
  if (value === "passive_ds" || value === "passive_ds_twr") return "passive_ds";
  if (value === "ranging") return "ranging";
  if (value === "tdoa") return "flextdoa";
  return "flextdoa";
}

function positionProtocolUsesTdoa(solver) {
  return solver === "flextdoa" || solver === "passive_ds";
}

function positionRuntimeModeForSolver(solver) {
  if (solver === "passive_ds") return "passive_ds_twr";
  if (solver === "flextdoa") return "flex_tdoa";
  return "ranging";
}

const positionAnchorRangeHistoryMaxAgeSec = 30;

function positionGeometryMaxAge(settings) {
  // Geometry maintenance is deliberately much slower than tag observations.
  // Retain the last live UWB geometry while waiting for its next maintenance
  // update; tag/range freshness remains governed by the strict per-protocol
  // Position Setup value.  This is not a fixed or GPS-derived geometry.
  return Math.max(
    positionAnchorRangeHistoryMaxAgeSec,
    Number(settings.maxAge) || 0
  );
}

function positionGeometryProtocol(solver) {
  if (solver === "passive_ds") return "passive_ds";
  if (solver === "ranging") return "native_ds";
  return "flextdoa";
}

function nativeDsFramePeriodSec(settings) {
  if (settings?.solver !== "ranging") return 0;
  const status = state.statuses.find(item =>
    statusIsFresh(item) && item.runtime_mode_name === "uwb_ranging"
  ) || state.statuses.find(item =>
    Number.isFinite(Number(item.runtime_ranging_slot_ms))
  );
  const slotMs = Number(status?.runtime_ranging_slot_ms);
  const gapMs = Number(status?.runtime_ranging_round_gap_ms || 0);
  if (!Number.isFinite(slotMs) || slotMs <= 0) return 0;
  const anchorCount = Math.max(1, Number(settings.anchorIds?.length || settings.anchorCount || 1));
  return (anchorCount * slotMs + Math.max(0, gapMs)) / 1000;
}

function positionDisplayMaxAge(settings) {
  // A short browser/network scheduling pause must not erase an otherwise
  // valid marker between high-rate samples. This is a display retention floor;
  // ESP32 solving and the protocol's own coherence/freshness checks are not
  // changed by it.
  const configured = Math.max(0.5, Number(settings?.maxAge) || 0.5);
  const framePeriodSec = nativeDsFramePeriodSec(settings);
  if (!(framePeriodSec > 0)) return configured;
  // Native DS-TWR publishes one coherent position only after it has ranged all
  // selected anchors. In field captures an otherwise healthy four-anchor
  // runtime occasionally skips two publication opportunities (for example
  // while an HTTP/status request shares the ESP32).  Hold the last valid marker
  // across that gap. This changes presentation only, never ranging or solving.
  return Math.max(configured, framePeriodSec * 3.5);
}

function positionSolverLabel(solver) {
  if (solver === "passive_ds") return "Passive DS-TWR";
  if (solver === "flextdoa") return "FlexTDOA";
  return "Native DS-TWR";
}

const positionProtocolDefaults = {
  flextdoa: {maxAgeSec: 0.5},
  passive_ds: {maxAgeSec: 0.2},
  ranging: {maxAgeSec: 0.5},
};

function positionProtocolMaxAgeKey(solver) {
  return `uwbDash.position.${normalizePositionSolver(solver)}.maxAgeSec`;
}

function savePositionProtocolSettings(solver = null) {
  const selected = normalizePositionSolver(
    solver || state.positionSettingsSolver ||
    document.getElementById("positionSolver")?.value || "flextdoa"
  );
  const value = Number(document.getElementById("positionMaxAgeSec")?.value);
  if (Number.isFinite(value) && value >= 0.2) {
    localStorage.setItem(positionProtocolMaxAgeKey(selected), String(value));
  }
}

function switchPositionProtocolSettings() {
  const solver = normalizePositionSolver(
    document.getElementById("positionSolver")?.value || "flextdoa"
  );
  if (state.positionSettingsSolver &&
      state.positionSettingsSolver !== solver) {
    savePositionProtocolSettings(state.positionSettingsSolver);
    resetPositionTagTrails();
    state.positionAnchorTrail = {};
    state.positionStreamRxTimes = [];
    state.positionStreamIndependentTimes = [];
    state.positionStreamSuperframeTimes = [];
    state.positionStreamCorrectionTimes = [];
    state.positionStreamRenderTimes = [];
    // A tag ID is shared by every protocol. Never let a late sample from the
    // previous runtime become the current protocol's display state.
    if (state.tdoa?.local_positions) {
      state.tdoa.local_positions = {};
    }
    state.positionStreamLatestEvent = null;
    state.positionStreamLastRenderedEventToken = "";
  }
  const maxAge = document.getElementById("positionMaxAgeSec");
  const saved = Number(localStorage.getItem(positionProtocolMaxAgeKey(solver)));
  if (maxAge) {
    maxAge.value = String(
      Number.isFinite(saved) && saved >= 0.2
        ? saved
        : positionProtocolDefaults[solver].maxAgeSec
    );
  }
  state.positionSettingsSolver = solver;
  document.querySelectorAll(".native-ds-position-option").forEach(element => {
    element.classList.toggle("hidden", solver !== "ranging");
  });
  document.querySelectorAll(".dynamic-geometry-option").forEach(element => {
    element.classList.toggle("hidden", solver !== "passive_ds");
  });
  const restartGeometry =
    document.getElementById("positionRestartAnchorSelfLocalization");
  if (restartGeometry) {
    restartGeometry.disabled = false;
    restartGeometry.title =
      "Restart the Passive DS-TWR live anchor estimate.";
  }
}

function positionSettings() {
  const anchorCount = Math.max(3, Math.min(4, Number(document.getElementById("positionAnchorCount")?.value || 4)));
  const solver = normalizePositionSolver(document.getElementById("positionSolver")?.value || "flextdoa");
  const anchorIds = parseIdList(document.getElementById("positionAnchors")?.value, anchorCount);
  const tagIds = parseIdList(document.getElementById("positionTags")?.value);
  const maxAge = Math.max(0.2, Number(document.getElementById("positionMaxAgeSec")?.value || 3));
  const referenceMode = String(document.getElementById("positionReferenceMode")?.value || "none");
  const referenceX = Number(document.getElementById("positionReferenceX")?.value);
  const referenceY = Number(document.getElementById("positionReferenceY")?.value);
  const errorWindowSec = Math.max(
    1, Math.min(120, Number(document.getElementById("positionErrorWindowSec")?.value || 30)));
  return {
    anchorCount,
    solver,
    anchorIds,
    tagIds,
    maxAge,
    referenceMode,
    referenceX,
    referenceY,
    errorWindowSec,
    nativeDsUpdateMode: "rolling",
    nativeDsFit: "all_anchor",
  };
}

function synchronizePositionAnchorsFromRuntime(statuses) {
  const countElement = document.getElementById("positionAnchorCount");
  const anchorsElement = document.getElementById("positionAnchors");
  const solverElement = document.getElementById("positionSolver");
  if (!countElement || !anchorsElement || !solverElement) return false;

  const solver = normalizePositionSolver(solverElement.value || "flextdoa");
  const expectedMode = {
    flextdoa: "uwb_flex_tdoa",
    ranging: "uwb_ranging",
    passive_ds: "uwb_passive_ds_twr",
  }[solver];
  if (!expectedMode) return false;

  const status = (statuses || []).find(item =>
    statusIsFresh(item) &&
    String(item.runtime_mode_name || "") === expectedMode &&
    Array.isArray(item.runtime_anchor_ids) &&
    item.runtime_anchor_ids.length >= 3);
  if (!status) return false;

  const runtimeIds = status.runtime_anchor_ids
    .map(Number)
    .filter(id => Number.isInteger(id) && id > 0)
    .slice(0, 4);
  if (runtimeIds.length < 3) return false;

  const configuredCount = Math.max(
    3, Math.min(4, Number(countElement.value || 4)));
  const configuredIds = parseIdList(anchorsElement.value, configuredCount);
  // Position Setup must describe the topology that is actually running on
  // the radios.  Treating a valid subset as synchronized left the browser in
  // the temporary three-anchor field workaround even after the fourth link
  // had recovered, so the dashboard silently ignored A2.  A deliberate
  // three-anchor setup is still supported by configuring three anchors in the
  // runtime; a four-anchor runtime is displayed and solved as four anchors.
  const selectionIsActive = configuredIds.length === configuredCount &&
    configuredCount === runtimeIds.length &&
    runtimeIds.every(id => configuredIds.includes(id));
  if (selectionIsActive) {
    // An Apply has reached the modules. Runtime synchronization can resume,
    // but it no longer needs to change the form.
    state.positionSetupDirty = false;
    return false;
  }
  // While the operator is preparing a new setup, the 250 ms status refresh
  // must not restore the configuration that is still running on the radios.
  // Apply is the explicit boundary between editable form state and runtime.
  if (state.positionSetupDirty) return false;

  const nextCount = Math.max(3, Math.min(4, runtimeIds.length));
  const nextIds = runtimeIds.slice(0, nextCount);
  countElement.value = String(nextCount);
  anchorsElement.value = nextIds.join(",");
  localStorage.setItem(settingKey("positionAnchorCount"), String(nextCount));
  localStorage.setItem(settingKey("positionAnchors"), nextIds.join(","));
  resetPositionTagTrails();
  state.positionSeeds = {};
  state.positionStreamRxTimes = [];
  state.positionStreamIndependentTimes = [];
  state.positionStreamRenderTimes = [];
  state.positionGeometry = {key: "", ekf: null};
  return true;
}

function positionKnownReference(settings, anchors) {
  if (settings.referenceMode === "gps_rtk") {
    const tagId = Number(settings.tagIds?.[0] || 1);
    const track = gpsRtkTagTrack(tagId);
    const latest = track[track.length - 1];
    if (!latest) return null;
    return {
      x: latest.x,
      y: latest.y,
      label: `GPS RTK tag M${tagId}`,
      dynamicGps: true,
      tagId,
      capturedAt: latest.t,
    };
  }
  if (settings.referenceMode === "centroid") {
    const points = settings.anchorIds.map(id => anchors[id]).filter(Boolean);
    if (points.length !== settings.anchorIds.length || !points.length) return null;
    return {
      x: points.reduce((sum, point) => sum + Number(point.x), 0) / points.length,
      y: points.reduce((sum, point) => sum + Number(point.y), 0) / points.length,
      label: "anchor centroid",
    };
  }
  if (settings.referenceMode === "manual" &&
      Number.isFinite(settings.referenceX) && Number.isFinite(settings.referenceY)) {
    return {x: settings.referenceX, y: settings.referenceY, label: "manual"};
  }
  return null;
}

function percentile(values, fraction) {
  if (!values.length) return NaN;
  const sorted = [...values].sort((left, right) => left - right);
  const index = (sorted.length - 1) * fraction;
  const lower = Math.floor(index);
  const upper = Math.min(sorted.length - 1, lower + 1);
  const blend = index - lower;
  return sorted[lower] * (1 - blend) + sorted[upper] * blend;
}

function positionReferenceErrorStats(tagId, position, reference, windowSec) {
  if (!position || !reference) return null;
  if (reference.dynamicGps) {
    return positionGpsReferenceErrorStats(
      tagId, position, reference, windowSec);
  }
  const now = Date.now() / 1000;
  const samples = (state.positionTrail[String(tagId)] || []).filter(point =>
    Number.isFinite(Number(point.x)) &&
    Number.isFinite(Number(point.y)) &&
    now - Number(point.t) <= windowSec);
  const currentErrorM = Math.hypot(
    Number(position.x) - reference.x, Number(position.y) - reference.y);
  if (!samples.length) {
    return {
      count: 1,
      spanSec: 0,
      currentErrorM,
      biasM: currentErrorM,
      rmseM: currentErrorM,
      p95M: currentErrorM,
      maxM: currentErrorM,
    };
  }
  const errors = samples.map(point =>
    Math.hypot(Number(point.x) - reference.x, Number(point.y) - reference.y));
  const meanX = samples.reduce((sum, point) => sum + Number(point.x), 0) / samples.length;
  const meanY = samples.reduce((sum, point) => sum + Number(point.y), 0) / samples.length;
  return {
    count: samples.length,
    spanSec: Math.max(0, Number(samples[samples.length - 1].t) - Number(samples[0].t)),
    currentErrorM,
    biasM: Math.hypot(meanX - reference.x, meanY - reference.y),
    rmseM: Math.sqrt(errors.reduce((sum, value) => sum + value * value, 0) / errors.length),
    p95M: percentile(errors, 0.95),
    maxM: Math.max(...errors),
  };
}

function selectedPositionModuleIds(settings = positionSettings()) {
  const ids = [...settings.tagIds, ...settings.anchorIds];
  return [...new Set(ids)].filter(Boolean);
}

function statusForModule(moduleId) {
  return state.statuses.find(item => Number(item.module_id) === Number(moduleId));
}

function selectedAccelModuleIds() {
  const target = String(document.getElementById("accelTargets")?.value || "all");
  return target === "all" ? [1, 2, 3, 4, 5] : [Number(target)];
}

function updateAccelEnabledControl() {
  const checkbox = document.getElementById("accelEnabled");
  const label = document.getElementById("accelEnabledLabel");
  if (!checkbox || !label) return;
  const statuses = selectedAccelModuleIds()
    .map(statusForModule)
    .filter(Boolean);
  if (!statuses.length) {
    checkbox.indeterminate = false;
    checkbox.checked = false;
    label.textContent = "BNO085 status unavailable";
    return;
  }
  const enabled = statuses.filter(item => Boolean(item.runtime_bno085_accel_enabled)).length;
  checkbox.indeterminate = enabled > 0 && enabled < statuses.length;
  checkbox.checked = enabled === statuses.length;
  label.textContent = checkbox.indeterminate
    ? `BNO085 mixed (${enabled}/${statuses.length} enabled)`
    : `BNO085 ${checkbox.checked ? "enabled" : "disabled"}`;
}

function moduleHttpOnline(item) {
  return Boolean(item?.http_status_online);
}

function moduleInPositionRuntime(item, solver) {
  if (!item || !moduleHttpOnline(item)) return false;
  if (item.uwb_runtime_switching ||
      String(item.uwb_status || "").toLowerCase() !== "ready") return false;
  const mode = String(item.runtime_mode_name || item.runtime_mode || "").toLowerCase();
  const wanted = positionRuntimeModeForSolver(solver);
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

function robustTrilaterate(anchors, distances, seed = null) {
  const ids = Object.keys(distances)
    .map(Number)
    .filter(id =>
      anchors[id] &&
      Number.isFinite(Number(distances[id])) &&
      Number(distances[id]) > 0)
    .sort((left, right) => left - right);
  if (ids.length < 3) return null;

  const subsets = [ids];
  if (ids.length > 3) {
    for (let omitted = 0; omitted < ids.length; omitted++) {
      subsets.push(ids.filter((_, index) => index !== omitted));
    }
  }
  const candidates = [];
  for (const subset of subsets) {
    const subsetDistances = Object.fromEntries(
      subset.map(id => [id, Number(distances[id])]));
    const position = trilaterate(anchors, subsetDistances);
    if (!position ||
        !Number.isFinite(position.x) || !Number.isFinite(position.y)) continue;
    const residuals = positionResiduals(position, anchors, distances);
    const absolute = Object.entries(residuals)
      .map(([id, residual]) => [Number(id), Math.abs(Number(residual))])
      .filter(([, residual]) => Number.isFinite(residual))
      .sort((left, right) => left[1] - right[1]);
    const inlierLimitM = 0.12;
    const inlierIds = absolute
      .filter(([, residual]) => residual <= inlierLimitM)
      .map(([id]) => id);
    const trimmed = absolute.slice(0, Math.min(3, absolute.length));
    const trimmedRmsM = trimmed.length
      ? Math.sqrt(trimmed.reduce(
        (sum, [, residual]) => sum + residual * residual, 0) / trimmed.length)
      : Infinity;
    const seedDistanceM = seed &&
        Number.isFinite(Number(seed.x)) && Number.isFinite(Number(seed.y))
      ? Math.hypot(position.x - Number(seed.x), position.y - Number(seed.y))
      : 0;
    candidates.push({
      position,
      residuals,
      inlierIds,
      inlierCount: inlierIds.length,
      trimmedRmsM,
      seedDistanceM,
    });
  }
  candidates.sort((left, right) =>
    right.inlierCount - left.inlierCount ||
    left.trimmedRmsM - right.trimmedRmsM ||
    left.seedDistanceM - right.seedDistanceM);
  const best = candidates[0];
  if (!best || best.inlierCount < 3 || best.trimmedRmsM > 0.12) return null;

  const usedDistances = Object.fromEntries(
    best.inlierIds.map(id => [id, Number(distances[id])]));
  const refined = trilaterate(anchors, usedDistances) || best.position;
  const residuals = positionResiduals(refined, anchors, distances);
  const usedIds = Object.keys(usedDistances).map(Number);
  return {
    position: refined,
    residuals,
    usedDistances,
    usedIds,
    rejectedIds: ids.filter(id => !usedIds.includes(id)),
  };
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

function minimumObservableAnchorEdges(anchorIds) {
  const count = anchorIds.map(Number).filter(Number.isFinite).length;
  // A planar graph has 2N-3 independent degrees of freedom after removing
  // translation and rotation.  Three anchors still need their full triangle;
  // four anchors can therefore be reconstructed from five independent edges.
  return Math.max(0, 2 * count - 3);
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

function anchorGeometryFitQuality(anchorIds, residuals) {
  const values = Object.values(residuals || {})
    .map(Number)
    .filter(Number.isFinite);
  const expected = selectedAnchorPairs(anchorIds).length;
  const minimum = minimumObservableAnchorEdges(anchorIds);
  const rmsM = values.length
    ? Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length)
    : NaN;
  const maxM = values.length ? Math.max(...values.map(Math.abs)) : NaN;
  // The paper uses R = 10 cm^2. Three standard deviations is 9.49 cm.
  const fixLimitM = 3 * Math.sqrt(10) * 0.01;
  return {
    complete: values.length >= minimum,
    fullPairGraph: values.length === expected,
    observedEdges: values.length,
    expectedEdges: expected,
    rmsM,
    maxM,
    fixLimitM,
    acceptable: values.length >= minimum && Number.isFinite(rmsM) && rmsM <= fixLimitM,
  };
}

const paperGeometryGuard = Object.freeze({
  minGateM: 0.10,
  maxGateM: 0.16,
  candidateToleranceM: 0.08,
  candidateConfirmations: 3,
  candidateMinSpanSec: 0.8,
  candidateMaxAgeSec: 4,
  relocationMinEdges: 2,
  relocationFitLimitM: 0.12,
  // Anchor-to-anchor maintenance is deliberately sparse and a marginal
  // outdoor edge may need many retries.  Once a complete UWB geometry has
  // been measured, losing one maintenance edge must not blank the tag.  Keep
  // that last-good edge as an initialization/continuity constraint until a
  // new accepted measurement or an explicit geometry reset replaces it.
  // Tag ranges remain governed by the strict Position Setup freshness.
  acceptedHoldSec: Number.POSITIVE_INFINITY,
  publishedMaxStepM: 0.05,
});

function geometryMeasurementToken(item) {
  if (!item) return "";
  return `${item.slot_id ?? "-"}:${item.seq ?? "-"}:` +
    `${Number(item.received_at || 0).toFixed(6)}`;
}

function geometryRobustSigma(item) {
  const value = Number(item?.stats?.robust_sigma_m);
  return Number.isFinite(value) && value >= 0 ? value : 0;
}

function geometryMeasurementGate(item) {
  return Math.min(
    paperGeometryGuard.maxGateM,
    Math.max(paperGeometryGuard.minGateM, geometryRobustSigma(item) * 6)
  );
}

function geometryBootstrapItem(item) {
  if (!item) return null;
  const sampleCount = Number(item?.stats?.samples || 0);
  const median = Number(item?.stats?.median_m);
  const measured = Number(item.distance_m);
  const distance = sampleCount >= 3 && Number.isFinite(median) && median > 0
    ? median
    : measured;
  if (!Number.isFinite(distance) || distance <= 0) return null;
  return {
    ...item,
    raw_latest_distance_m: measured,
    accepted_distance_m: distance,
    distance_m: distance,
    geometry_gate_status: sampleCount >= 3 ? "robust bootstrap" : "bootstrap",
  };
}

function geometryItemAge(item, now) {
  const receivedAt = Number(item?.received_at);
  if (Number.isFinite(receivedAt) && receivedAt > 0) {
    return Math.max(0, now - receivedAt);
  }
  return Math.max(0, Number(item?.age_sec) || 0);
}

function createGeometryPairState(item, now) {
  const accepted = geometryBootstrapItem(item);
  if (!accepted) return null;
  return {
    acceptedDistanceM: Number(accepted.distance_m),
    acceptedItem: accepted,
    acceptedAt: now,
    lastSeenToken: geometryMeasurementToken(item),
    lastStatus: accepted.geometry_gate_status,
    candidateDistanceM: NaN,
    candidateItem: null,
    candidateCount: 0,
    candidateFirstAt: 0,
    candidateLastAt: 0,
    rejectedCount: 0,
  };
}

function geometryPairStatesFromItems(distanceItems, now) {
  const pairStates = {};
  for (const [key, item] of Object.entries(distanceItems || {})) {
    const pairState = createGeometryPairState(item, now);
    if (pairState) pairStates[key] = pairState;
  }
  return pairStates;
}

function acceptedGeometryItems(ekf, now) {
  const items = {};
  for (const [key, pairState] of Object.entries(ekf?.pairStates || {})) {
    if (!pairState?.acceptedItem) continue;
    const ageSec = geometryItemAge(pairState.acceptedItem, now);
    if (ageSec > paperGeometryGuard.acceptedHoldSec) continue;
    items[key] = {
      ...pairState.acceptedItem,
      age_sec: ageSec,
      distance_m: Number(pairState.acceptedDistanceM),
      accepted_distance_m: Number(pairState.acceptedDistanceM),
    };
  }
  return items;
}

function updateGeometryCandidate(pairState, item, measured, now) {
  const tolerance = Math.max(
    paperGeometryGuard.candidateToleranceM,
    Math.min(0.12, geometryRobustSigma(item) * 4)
  );
  if (Number.isFinite(pairState.candidateDistanceM) &&
      Math.abs(measured - pairState.candidateDistanceM) <= tolerance &&
      now - pairState.candidateLastAt <= paperGeometryGuard.candidateMaxAgeSec) {
    const count = pairState.candidateCount + 1;
    pairState.candidateDistanceM +=
      (measured - pairState.candidateDistanceM) / count;
    pairState.candidateCount = count;
  } else {
    pairState.candidateDistanceM = measured;
    pairState.candidateCount = 1;
    pairState.candidateFirstAt = now;
  }
  pairState.candidateItem = {
    ...item,
    distance_m: pairState.candidateDistanceM,
    accepted_distance_m: pairState.candidateDistanceM,
  };
  pairState.candidateLastAt = now;
}

function confirmedGeometryRelocation(ekf, anchorIds, now) {
  const confirmed = Object.entries(ekf.pairStates || {}).filter(([, pairState]) =>
    pairState.candidateCount >= paperGeometryGuard.candidateConfirmations &&
    pairState.candidateLastAt - pairState.candidateFirstAt >=
      paperGeometryGuard.candidateMinSpanSec &&
    now - pairState.candidateLastAt <=
      paperGeometryGuard.candidateMaxAgeSec);
  if (confirmed.length < paperGeometryGuard.relocationMinEdges) return null;

  const incident = new Map();
  for (const [key] of confirmed) {
    const [a, b] = key.split(":").map(Number);
    incident.set(a, (incident.get(a) || 0) + 1);
    incident.set(b, (incident.get(b) || 0) + 1);
  }
  const moving = [...incident.entries()]
    .filter(([, count]) => count >= paperGeometryGuard.relocationMinEdges)
    .sort((left, right) => right[1] - left[1])[0];
  if (!moving) return null;

  const movingAnchorId = moving[0];
  const proposedItems = acceptedGeometryItems(ekf, now);
  let changedEdges = 0;
  for (const [key, pairState] of confirmed) {
    const ids = key.split(":").map(Number);
    if (!ids.includes(movingAnchorId) || !pairState.candidateItem) continue;
    proposedItems[key] = {
      ...pairState.candidateItem,
      distance_m: Number(pairState.candidateDistanceM),
      accepted_distance_m: Number(pairState.candidateDistanceM),
      geometry_gate_status: "confirmed relocation",
    };
    changedEdges++;
  }
  if (changedEdges < paperGeometryGuard.relocationMinEdges ||
      Object.keys(proposedItems).length !== selectedAnchorPairs(anchorIds).length) {
    return null;
  }

  const proposedAnchors = initialPaperAnchorCoordinates(
    anchorIds, proposedItems);
  if (!proposedAnchors) return null;
  const residuals = anchorGeometryResiduals(proposedAnchors, proposedItems);
  const fit = anchorGeometryFitQuality(anchorIds, residuals);
  if (!fit.complete || !Number.isFinite(fit.rmsM) ||
      fit.rmsM > paperGeometryGuard.relocationFitLimitM) {
    return null;
  }
  return {
    movingAnchorId,
    distanceItems: proposedItems,
    fit,
  };
}

function conditionPaperAnchorBatch(ekf, batch, now) {
  const updateItems = {};
  const displayItems = {};
  const expectedPairs = selectedAnchorPairs(batch.ids);
  for (const [a, b] of expectedPairs) {
    const key = anchorPairKey(a, b);
    const rawItem = batch.distanceItems[key];
    let pairState = ekf.pairStates[key];
    if (!pairState && rawItem) {
      pairState = createGeometryPairState(rawItem, now);
      if (pairState) {
        ekf.pairStates[key] = pairState;
        updateItems[key] = pairState.acceptedItem;
      }
    }
    if (!pairState) continue;

    const token = geometryMeasurementToken(rawItem);
    if (rawItem && token && token !== pairState.lastSeenToken) {
      pairState.lastSeenToken = token;
      const measured = Number(rawItem.distance_m);
      const delta = measured - Number(pairState.acceptedDistanceM);
      const gateM = geometryMeasurementGate(rawItem);
      if (Number.isFinite(measured) && measured > 0 &&
          Math.abs(delta) <= gateM) {
        pairState.acceptedDistanceM = measured;
        pairState.acceptedItem = {
          ...rawItem,
          raw_latest_distance_m: measured,
          accepted_distance_m: measured,
          distance_m: measured,
          geometry_gate_status: "accepted",
        };
        pairState.acceptedAt = now;
        pairState.lastStatus = "accepted";
        pairState.candidateDistanceM = NaN;
        pairState.candidateItem = null;
        pairState.candidateCount = 0;
        pairState.candidateFirstAt = 0;
        pairState.candidateLastAt = 0;
        updateItems[key] = pairState.acceptedItem;
        ekf.lastGoodAt = now;
      } else {
        pairState.rejectedCount++;
        pairState.lastStatus = "rejected spike";
        updateGeometryCandidate(pairState, rawItem, measured, now);
        ekf.rejectedCount++;
        ekf.lastRejected = {
          key,
          measuredM: measured,
          acceptedM: Number(pairState.acceptedDistanceM),
          deltaM: delta,
          at: now,
          count: ekf.rejectedCount,
        };
      }
    }

    const acceptedAge = geometryItemAge(pairState.acceptedItem, now);
    const held = !rawItem ||
      geometryMeasurementToken(rawItem) !==
        geometryMeasurementToken(pairState.acceptedItem);
    displayItems[key] = {
      ...(rawItem || pairState.acceptedItem),
      age_sec: rawItem
        ? geometryItemAge(rawItem, now)
        : acceptedAge,
      raw_latest_distance_m: Number(rawItem?.distance_m),
      accepted_distance_m: Number(pairState.acceptedDistanceM),
      geometry_gate_status: pairState.lastStatus,
      geometry_held: held,
      geometry_rejected_count: pairState.rejectedCount,
    };
  }
  return {
    updateItems,
    displayItems,
    acceptedItems: acceptedGeometryItems(ekf, now),
    relocation: confirmedGeometryRelocation(ekf, batch.ids, now),
  };
}

function currentAnchorDistanceBatch(anchorIds, maxAge, solver) {
  const ids = anchorIds.map(Number).filter(id => Number.isInteger(id) && id > 0);
  const expectedProtocol = positionGeometryProtocol(solver);
  const selected = new Set(ids);
  const expectedPairs = selectedAnchorPairs(ids);
  const liveItems = {};
  const groups = new Map();
  for (const sample of state.tdoa?.recent_anchor_ranges || []) {
    const initiator = Number(sample.initiator_id);
    const responder = Number(sample.responder_id);
    const slotId = Number(sample.slot_id);
    const protocol = String(sample.tdoa_protocol || "flextdoa");
    if (!selected.has(initiator) || !selected.has(responder) ||
        !Number.isInteger(slotId) || Number(sample.age_sec) > maxAge ||
        protocol !== expectedProtocol) continue;

    const pair = anchorPairKey(initiator, responder);
    const previous = liveItems[pair];
    if (!previous ||
        Number(sample.received_at || 0) > Number(previous.received_at || 0)) {
      const summary = freshAnchorPairDistance(
        Number(sample.anchor_a_id), Number(sample.anchor_b_id), maxAge);
      liveItems[pair] = {
        ...(String(summary?.tdoa_protocol || "") === expectedProtocol
          ? summary
          : {}),
        ...sample,
      };
    }

    if (expectedProtocol === "native_ds") continue;
    const geometryCycleSlots = expectedProtocol === "passive_ds"
      ? ids.length * Math.max(1, ids.length - 1)
      : ids.length;
    const frameId = Math.floor(slotId / geometryCycleSlots);
    const group = groups.get(frameId) || {frameId, items: {}, newestAt: 0};
    group.items[pair] = {...(liveItems[pair] || {}), ...sample};
    group.newestAt = Math.max(group.newestAt, Number(sample.received_at) || 0);
    groups.set(frameId, group);
  }

  // The detailed event list is intentionally bounded, while the server also
  // retains the latest validated result for every anchor pair. Native DS uses
  // those summaries to survive a browser refresh between sparse survey
  // exchanges. They are never used for tag-range freshness.
  if (expectedProtocol === "native_ds") {
    const now = Date.now() / 1000;
    for (const [a, b] of expectedPairs) {
      const key = anchorPairKey(a, b);
      if (liveItems[key]) continue;
      const summary = freshAnchorPairDistance(a, b, maxAge);
      if (!summary ||
          String(summary.tdoa_protocol || "") !== expectedProtocol ||
          !Number.isFinite(Number(summary.distance_m)) ||
          Number(summary.distance_m) <= 0) continue;
      liveItems[key] = {
        ...summary,
        received_at: now - Math.max(0, Number(summary.age_sec) || 0),
      };
    }
  }

  const complete = [...groups.values()]
    .filter(group => expectedPairs.every(([a, b]) => group.items[anchorPairKey(a, b)]))
    .sort((left, right) => right.frameId - left.frameId)[0];
  const distanceItems = complete?.items || liveItems;
  const missingPairs = expectedPairs.filter(
    ([a, b]) => !distanceItems[anchorPairKey(a, b)]);
  const observedEdgeCount = expectedPairs.length - missingPairs.length;
  const sparseObservableGeometry = expectedProtocol !== "passive_ds" &&
    observedEdgeCount >= minimumObservableAnchorEdges(ids);
  const updateToken = complete
    ? `${expectedProtocol}:frame:${complete.frameId}`
    : `${expectedProtocol}:live:` + expectedPairs.map(([a, b]) => {
      const item = distanceItems[anchorPairKey(a, b)];
      return `${a}-${b}:${item?.slot_id ?? "-"}:${item?.seq ?? "-"}:` +
        `${Number(item?.received_at || 0).toFixed(6)}`;
    }).join("|");
  return {
    ids,
    distanceItems,
    missingPairs,
    coherent: Boolean(complete) || missingPairs.length === 0 ||
      sparseObservableGeometry,
    frameId: complete?.frameId ?? null,
    updateToken,
    protocol: expectedProtocol,
  };
}

function circleIntersectionCandidates(first, firstRadius, second, secondRadius) {
  const dx = second.x - first.x;
  const dy = second.y - first.y;
  const baseline = Math.hypot(dx, dy);
  if (!Number.isFinite(baseline) || baseline < 1e-6 ||
      baseline > firstRadius + secondRadius + 0.02 ||
      baseline < Math.abs(firstRadius - secondRadius) - 0.02) return [];
  const along = (firstRadius * firstRadius - secondRadius * secondRadius +
    baseline * baseline) / (2 * baseline);
  const heightSquared = firstRadius * firstRadius - along * along;
  if (heightSquared < -0.02) return [];
  const height = Math.sqrt(Math.max(0, heightSquared));
  const ux = dx / baseline;
  const uy = dy / baseline;
  const center = {
    x: first.x + along * ux,
    y: first.y + along * uy,
  };
  return [
    {x: center.x - height * uy, y: center.y + height * ux},
    {x: center.x + height * uy, y: center.y - height * ux},
  ];
}

function normalizePaperAnchorCoordinates(anchorIds, anchors) {
  const ids = anchorIds.map(Number);
  const origin = anchors[ids[0]];
  const axis = anchors[ids[1]];
  if (!origin || !axis) return null;
  const dx = axis.x - origin.x;
  const dy = axis.y - origin.y;
  const length = Math.hypot(dx, dy);
  if (!Number.isFinite(length) || length < 1e-6) return null;
  const ux = dx / length;
  const uy = dy / length;
  return Object.fromEntries(ids.map(id => {
    const point = anchors[id];
    const tx = point.x - origin.x;
    const ty = point.y - origin.y;
    return [id, {
      x: uy * tx - ux * ty,
      y: ux * tx + uy * ty,
    }];
  }));
}

function sparsePaperAnchorCoordinates(anchorIds, distanceItems) {
  const ids = anchorIds.map(Number);
  if (ids.length !== 4) return null;
  const distance = (a, b) =>
    Number(distanceItems[anchorPairKey(a, b)]?.distance_m);
  const triangles = [];
  for (let i = 0; i < ids.length; i++) {
    for (let j = i + 1; j < ids.length; j++) {
      for (let k = j + 1; k < ids.length; k++) {
        const triangle = [ids[i], ids[j], ids[k]];
        const edges = [
          distance(triangle[0], triangle[1]),
          distance(triangle[0], triangle[2]),
          distance(triangle[1], triangle[2]),
        ];
        if (edges.every(value => Number.isFinite(value) && value > 0)) {
          triangles.push({triangle, edges});
        }
      }
    }
  }
  // Prefer the triangle with the largest baseline.  It gives the most stable
  // circle intersection when one of the six four-anchor edges is unavailable.
  triangles.sort((left, right) =>
    Math.max(...right.edges) - Math.max(...left.edges));
  for (const candidate of triangles) {
    const [firstId, secondId, thirdId] = candidate.triangle;
    const d01 = distance(firstId, secondId);
    const d02 = distance(firstId, thirdId);
    const d12 = distance(secondId, thirdId);
    const y2 = (d02 * d02 + d01 * d01 - d12 * d12) / (2 * d01);
    const x2Squared = d02 * d02 - y2 * y2;
    if (x2Squared < -0.02) continue;
    const placed = {
      [firstId]: {x: 0, y: 0},
      [secondId]: {x: 0, y: d01},
      [thirdId]: {x: Math.sqrt(Math.max(0, x2Squared)), y: y2},
    };
    const remainingId = ids.find(id => !candidate.triangle.includes(id));
    const links = candidate.triangle
      .map(id => ({id, radius: distance(id, remainingId)}))
      .filter(link => Number.isFinite(link.radius) && link.radius > 0);
    if (links.length < 2) continue;
    const intersections = circleIntersectionCandidates(
      placed[links[0].id], links[0].radius,
      placed[links[1].id], links[1].radius);
    if (!intersections.length) continue;

    let selected = intersections[0];
    if (links.length >= 3) {
      intersections.sort((left, right) =>
        Math.abs(Math.hypot(
          left.x - placed[links[2].id].x,
          left.y - placed[links[2].id].y) - links[2].radius) -
        Math.abs(Math.hypot(
          right.x - placed[links[2].id].x,
          right.y - placed[links[2].id].y) - links[2].radius));
      selected = intersections[0];
    } else if (intersections.length > 1) {
      // Five edges leave a mirror ambiguity.  In this deployment A2-A3 and
      // A4-A5 are the two opposing perimeter sides.  If one of those sides is
      // missing, its two endpoints must stay on the same side of the measured
      // opposite edge.  This convention is UWB-only and matches both the
      // original square and the current field layout.
      const baselineA = placed[links[0].id];
      const baselineB = placed[links[1].id];
      const otherId = candidate.triangle.find(
        id => id !== links[0].id && id !== links[1].id);
      const side = point =>
        (baselineB.x - baselineA.x) * (point.y - baselineA.y) -
        (baselineB.y - baselineA.y) * (point.x - baselineA.x);
      const otherSide = side(placed[otherId]);
      const sameSide = intersections.find(point => side(point) * otherSide > 0);
      selected = sameSide || intersections
        .slice()
        .sort((left, right) => Math.abs(side(right)) - Math.abs(side(left)))[0];
    }
    placed[remainingId] = selected;
    const normalized = normalizePaperAnchorCoordinates(ids, placed);
    if (normalized) return normalized;
  }
  return null;
}

function initialPaperAnchorCoordinates(anchorIds, distanceItems) {
  const ids = anchorIds.map(Number);
  if (ids.length < 3 || ids.length > 4) return null;
  const distance = (a, b) => Number(distanceItems[anchorPairKey(a, b)]?.distance_m);
  const d01 = distance(ids[0], ids[1]);
  const d02 = distance(ids[0], ids[2]);
  const d12 = distance(ids[1], ids[2]);
  if (![d01, d02, d12].every(value => Number.isFinite(value) && value > 0)) {
    return sparsePaperAnchorCoordinates(ids, distanceItems);
  }

  // Paper frame convention adapted to 2D: A0=(0,0), A1 on +Y, A2 on +X.
  const y2 = (d02 * d02 + d01 * d01 - d12 * d12) / (2 * d01);
  const x2sq = d02 * d02 - y2 * y2;
  if (x2sq < -0.02) return null;
  const anchors = {
    [ids[0]]: {x: 0, y: 0},
    [ids[1]]: {x: 0, y: d01},
    [ids[2]]: {x: Math.sqrt(Math.max(0, x2sq)), y: y2},
  };

  if (ids.length === 4) {
    const d03 = distance(ids[0], ids[3]);
    const d13 = distance(ids[1], ids[3]);
    const d23 = distance(ids[2], ids[3]);
    if (![d03, d13, d23].every(value => Number.isFinite(value) && value > 0)) {
      return sparsePaperAnchorCoordinates(ids, distanceItems);
    }
    const y3 = (d03 * d03 + d01 * d01 - d13 * d13) / (2 * d01);
    const x3sq = d03 * d03 - y3 * y3;
    if (x3sq < -0.02) return null;
    const x3abs = Math.sqrt(Math.max(0, x3sq));
    const candidates = [{x: x3abs, y: y3}, {x: -x3abs, y: y3}];
    candidates.sort((left, right) =>
      Math.abs(Math.hypot(left.x - anchors[ids[2]].x, left.y - anchors[ids[2]].y) - d23) -
      Math.abs(Math.hypot(right.x - anchors[ids[2]].x, right.y - anchors[ids[2]].y) - d23));
    anchors[ids[3]] = candidates[0];
  }
  return anchors;
}

function paperAnchorVariables(anchorIds) {
  const ids = anchorIds.map(Number);
  const variables = [{id: ids[1], axis: "y"}];
  for (const id of ids.slice(2)) {
    variables.push({id, axis: "x"}, {id, axis: "y"});
  }
  return variables;
}

function paperAnchorStateFromCoordinates(variables, anchors) {
  return variables.map(variable => Number(anchors[variable.id][variable.axis]));
}

function paperAnchorCoordinatesFromState(anchorIds, variables, vector) {
  const ids = anchorIds.map(Number);
  const anchors = {[ids[0]]: {x: 0, y: 0}, [ids[1]]: {x: 0, y: 0}};
  for (const id of ids.slice(2)) anchors[id] = {x: 0, y: 0};
  variables.forEach((variable, index) => {
    anchors[variable.id][variable.axis] = Number(vector[index]);
  });
  return anchors;
}

function invertMatrix(matrix) {
  const n = matrix.length;
  const inverse = Array.from({length: n}, () => Array(n).fill(0));
  for (let col = 0; col < n; col++) {
    const rhs = Array(n).fill(0);
    rhs[col] = 1;
    const solution = solveLinearSystem(matrix, rhs);
    if (!solution) return null;
    for (let row = 0; row < n; row++) inverse[row][col] = solution[row];
  }
  return inverse;
}

function multiplyMatrixVector(matrix, vector) {
  return matrix.map(row => row.reduce(
    (sum, value, index) => sum + value * vector[index], 0));
}

function updatePaperAnchorEkf(
  ekf, anchorIds, distanceItems, options = {}
) {
  const variables = ekf.variables;
  const n = variables.length;
  const measurements = Object.values(distanceItems || {});
  if (!measurements.length) return false;
  const anchors = paperAnchorCoordinatesFromState(anchorIds, variables, ekf.state);
  const indexFor = new Map(
    variables.map((variable, index) => [`${variable.id}:${variable.axis}`, index])
  );

  // FlexTDOA paper parameters: sigma_Q^2=1 cm^2, sigma_R^2=10 cm^2.
  const qVarianceM2 = 1 * 0.01 * 0.01;
  const rVarianceM2 = 10 * 0.01 * 0.01;
  const predictedCovariance = ekf.covariance.map((row, r) =>
    row.map((value, c) => value + (r === c ? qVarianceM2 : 0)));
  const information = invertMatrix(predictedCovariance);
  if (!information) return false;
  const rhs = Array(n).fill(0);

  for (const item of measurements) {
    const a = anchors[Number(item.anchor_a_id)];
    const b = anchors[Number(item.anchor_b_id)];
    const measured = Number(item.distance_m);
    if (!a || !b || !Number.isFinite(measured) || measured <= 0) return false;
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    const predicted = Math.max(1e-6, Math.hypot(dx, dy));
    const innovation = measured - predicted;
    const huberScaleM = 0.06;
    const robustWeight = options.robust === false
      ? 1
      : Math.min(
        1, huberScaleM / Math.max(huberScaleM, Math.abs(innovation)));
    const effectiveVariance = rVarianceM2 / robustWeight;
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
      rhs[r] += gradient[r] * innovation / effectiveVariance;
      for (let c = 0; c < n; c++) {
        information[r][c] +=
          gradient[r] * gradient[c] / effectiveVariance;
      }
    }
  }

  const posteriorCovariance = invertMatrix(information);
  if (!posteriorCovariance) return false;
  const correction = multiplyMatrixVector(posteriorCovariance, rhs);
  if (!correction.every(Number.isFinite)) return false;
  ekf.state = ekf.state.map((value, index) => value + correction[index]);
  ekf.covariance = posteriorCovariance;
  ekf.updates++;
  ekf.lastUpdateAt = Date.now() / 1000;
  return true;
}

function positionGeometryKey(anchorIds) {
  return anchorIds.map(Number).filter(Number.isFinite).join(",");
}

function positionGeometryStorageKey(anchorIds) {
  return `uwbDash.flexTdoaPaperGeometry.v1.${positionGeometryKey(anchorIds)}`;
}

function cloneAnchorCoordinates(anchors) {
  return Object.fromEntries(Object.entries(anchors || {}).map(([id, point]) => [
    id,
    {x: Number(point.x), y: Number(point.y)},
  ]));
}

function persistedModuleAnchorGeometry(anchorIds) {
  const selectedIds = anchorIds
    .map(Number)
    .filter(id => Number.isInteger(id) && id > 0);
  if (selectedIds.length < 3) return null;

  const candidates = [];
  for (const status of state.statuses || []) {
    if (!moduleHttpOnline(status) ||
        status.runtime_flex_tdoa_geometry_fixed !== true) continue;
    const ids = Array.isArray(status.runtime_anchor_ids)
      ? status.runtime_anchor_ids.map(Number)
      : [];
    const xMm = Array.isArray(status.runtime_flex_tdoa_anchor_x_mm)
      ? status.runtime_flex_tdoa_anchor_x_mm.map(Number)
      : [];
    const yMm = Array.isArray(status.runtime_flex_tdoa_anchor_y_mm)
      ? status.runtime_flex_tdoa_anchor_y_mm.map(Number)
      : [];
    if (ids.length !== xMm.length || ids.length !== yMm.length) continue;

    const anchors = {};
    for (const anchorId of selectedIds) {
      const index = ids.indexOf(anchorId);
      if (index < 0 || !Number.isFinite(xMm[index]) ||
          !Number.isFinite(yMm[index])) break;
      anchors[anchorId] = {
        x: xMm[index] / 1000,
        y: yMm[index] / 1000,
      };
    }
    if (Object.keys(anchors).length !== selectedIds.length) continue;

    const first = anchors[selectedIds[0]];
    const second = anchors[selectedIds[1]];
    const third = anchors[selectedIds[2]];
    const doubleArea = Math.abs(
      (second.x - first.x) * (third.y - first.y) -
      (second.y - first.y) * (third.x - first.x));
    if (!Number.isFinite(doubleArea) || doubleArea < 0.01) continue;

    candidates.push({
      anchors,
      generation:
        Number(status.runtime_flex_tdoa_geometry_generation) || 0,
      sourceModuleId: Number(status.module_id) || null,
      ageSec: Number(status.http_status_age_sec) || 0,
    });
  }
  candidates.sort((left, right) =>
    right.generation - left.generation ||
    left.ageSec - right.ageSec ||
    Number(left.sourceModuleId || 9999) -
      Number(right.sourceModuleId || 9999));
  return candidates[0] || null;
}

function persistedModuleGeometryResult(anchorIds, persisted, protocol) {
  const distanceItems = {};
  const residuals = {};
  const geometryGateStatus = "persisted in modules";
  for (const [a, b] of selectedAnchorPairs(anchorIds)) {
    const key = anchorPairKey(a, b);
    const distanceM = Math.hypot(
      persisted.anchors[a].x - persisted.anchors[b].x,
      persisted.anchors[a].y - persisted.anchors[b].y);
    distanceItems[key] = {
      anchor_a_id: a,
      anchor_b_id: b,
      initiator_id: a,
      responder_id: b,
      distance_m: distanceM,
      accepted_distance_m: distanceM,
      age_sec: persisted.ageSec,
      geometry_gate_status: geometryGateStatus,
      geometry_held: true,
      stats: {robust_sigma_m: null},
    };
    residuals[key] = 0;
  }
  const fitQuality = anchorGeometryFitQuality(anchorIds, residuals);
  return {
    anchors: cloneAnchorCoordinates(persisted.anchors),
    distanceItems,
    missingPairs: [],
    residuals,
    fitQuality,
    complete: true,
    positionReady: true,
    canFix: false,
    status: "persisted",
    protocol,
    sourceModuleId: persisted.sourceModuleId,
    sourceKind: persisted.sourceKind || "module-persisted",
    sourceLabel: persisted.sourceLabel || "",
    generation: persisted.generation,
    lastUpdateAgeSec: persisted.ageSec,
    estimateAgeSec: persisted.ageSec,
    updates: 0,
    relocationCount: 0,
    rejectedCount: 0,
    candidatePairs: [],
    heldPairs: selectedAnchorPairs(anchorIds).map(
      ([a, b]) => anchorPairKey(a, b)),
  };
}

function resetPaperAnchorSelfLocalization(
  anchorIds, solver, forgetStored = true
) {
  if (forgetStored) localStorage.removeItem(positionGeometryStorageKey(anchorIds));
  state.positionGeometry = {
    key: `${positionGeometryProtocol(solver)}:${positionGeometryKey(anchorIds)}`,
    ekf: null,
  };
  state.positionSeeds = {};
  state.positionAnchorTrail = {};
}

function createPaperAnchorEkf(batch, previous = null) {
  const now = Date.now() / 1000;
  const bootstrapItems = Object.fromEntries(
    Object.entries(batch.distanceItems || {})
      .map(([key, item]) => [key, geometryBootstrapItem(item)])
      .filter(([, item]) => item)
  );
  const anchors = initialPaperAnchorCoordinates(
    batch.ids, bootstrapItems);
  if (!anchors) return null;
  const variables = paperAnchorVariables(batch.ids);
  const n = variables.length;
  const ekf = {
    variables,
    state: paperAnchorStateFromCoordinates(variables, anchors),
    covariance: Array.from({length: n}, (_, r) =>
      Array.from({length: n}, (_, c) => r === c ? 1 : 0)),
    lastUpdateToken: batch.updateToken,
    lastFrameId: batch.frameId,
    updates: Number(previous?.updates || 0),
    startedAt: Number(previous?.startedAt || now),
    lastUpdateAt: now,
    lastGoodAt: now,
    relocationCount: Number(previous?.relocationCount || 0),
    lastRelocationAt: Number(previous?.lastRelocationAt || 0),
    rejectedCount: Number(previous?.rejectedCount || 0),
    lastRejected: previous?.lastRejected || null,
    pairStates: geometryPairStatesFromItems(bootstrapItems, now),
    publishedAnchors: previous?.publishedAnchors
      ? cloneAnchorCoordinates(previous.publishedAnchors)
      : cloneAnchorCoordinates(anchors),
  };
  // Balance all six robust bootstrap edges before the first paint. The direct
  // coordinate construction exactly satisfies its first triangle and can
  // otherwise leave the fourth anchor visibly biased until its next range.
  // Large outdoor layouts need a few more linearization steps than the former
  // 3 m test square. Stop as soon as the same paper-derived fit gate used by
  // the live estimator is satisfied; the extra work happens only at startup.
  for (let iteration = 0; iteration < 12; iteration++) {
    if (!updatePaperAnchorEkf(
      ekf,
      batch.ids,
      bootstrapItems,
      {robust: batch.protocol !== "native_ds"})) break;
    const fit = anchorGeometryFitQuality(
      batch.ids,
      anchorGeometryResiduals(
        paperAnchorCoordinatesFromState(
          batch.ids, ekf.variables, ekf.state),
        bootstrapItems));
    if (iteration >= 2 && fit.acceptable) break;
  }
  ekf.publishedAnchors = previous?.publishedAnchors
    ? cloneAnchorCoordinates(previous.publishedAnchors)
    : paperAnchorCoordinatesFromState(
      batch.ids, ekf.variables, ekf.state);
  return ekf;
}

function smoothPublishedAnchorCoordinates(ekf, targetAnchors, updated) {
  if (!ekf.publishedAnchors) {
    ekf.publishedAnchors = cloneAnchorCoordinates(targetAnchors);
    return ekf.publishedAnchors;
  }
  if (!updated) return ekf.publishedAnchors;
  for (const [id, target] of Object.entries(targetAnchors)) {
    const current = ekf.publishedAnchors[id];
    if (!current) {
      ekf.publishedAnchors[id] = {x: target.x, y: target.y};
      continue;
    }
    const dx = Number(target.x) - Number(current.x);
    const dy = Number(target.y) - Number(current.y);
    const distance = Math.hypot(dx, dy);
    const scale = distance > paperGeometryGuard.publishedMaxStepM
      ? paperGeometryGuard.publishedMaxStepM / distance
      : 1;
    current.x += dx * scale;
    current.y += dy * scale;
  }
  return ekf.publishedAnchors;
}

function paperAnchorGeometry(anchorIds, maxAge, solver) {
  const protocol = positionGeometryProtocol(solver);
  const key = `${protocol}:${positionGeometryKey(anchorIds)}`;
  // Remove the browser-side fixed coordinates written by older dashboard
  // releases. They are intentionally not a fallback for live positioning.
  localStorage.removeItem(positionGeometryStorageKey(anchorIds));
  if (state.positionGeometry.key !== key) {
    state.positionGeometry = {key, ekf: null};
    state.positionSeeds = {};
  }

  const batch = currentAnchorDistanceBatch(anchorIds, maxAge, solver);
  if (["flextdoa", "passive_ds", "native_ds"].includes(protocol)) {
    const expectedIds = anchorIds.map(Number);
    const candidates = Object.values(state.tdoa?.local_geometries || {})
      .filter(item => {
        if (!item?.complete || Number(item.age_sec) > maxAge) return false;
        if (String(item?.tdoa_protocol || "passive_ds") !== protocol) return false;
        const ids = Object.keys(item.anchors || {}).map(Number);
        return ids.length === expectedIds.length &&
          expectedIds.every(id => ids.includes(id));
      })
      .sort((left, right) =>
        Number(right.geometry_version || 0) -
        Number(left.geometry_version || 0));
    const espGeometry = candidates[0] || null;
    if (!espGeometry) {
      return {
        anchors: {},
        distanceItems: batch.distanceItems,
        missingPairs: batch.missingPairs,
        residuals: {},
        complete: false,
        positionReady: false,
        canFix: false,
        status: "waiting_esp",
        protocol,
      };
    }
    const anchors = Object.fromEntries(
      Object.entries(espGeometry.anchors || {}).map(([id, anchor]) => [
        Number(id),
        {x: Number(anchor.x), y: Number(anchor.y)},
      ])
    );
    const fitRmsM = Number(espGeometry.fit_rms_m);
    return {
      anchors,
      distanceItems: batch.distanceItems,
      missingPairs: [],
      residuals: {},
      fitQuality: {
        rmsM: fitRmsM,
        acceptable: Number.isFinite(fitRmsM),
      },
      complete: true,
      positionReady: true,
      canFix: false,
      status: protocol === "flextdoa" || espGeometry.dynamic === false
        ? "esp_fixed_rtk"
        : "esp_dynamic",
      protocol,
      updates: Number(espGeometry.geometry_version || 0),
      frameId: Number(espGeometry.geometry_version || 0),
      lastUpdateAgeSec: Number(espGeometry.age_sec || 0),
      estimateAgeSec: Number(espGeometry.age_sec || 0),
      maxSigmaM: NaN,
      relocationCount: 0,
      rejectedCount: 0,
      heldPairs: [],
      sourceTagId: Number(espGeometry.tag_id),
    };
  }
  const session = state.positionGeometry;
  let geometryUpdated = false;
  let bootstrapBatch = batch;
  if (!session.ekf && protocol === "native_ds" &&
      (!batch.coherent || batch.missingPairs.length)) {
    // Native DS anchor maintenance intentionally runs more slowly than the
    // tag ranging loop.  A single missed long-link exchange can therefore
    // make one pair older than the strict 3 s live threshold even though its
    // latest measurements form a tight, trustworthy cluster.  Bootstrap the
    // geometry from the existing last-good hold window; subsequent updates
    // still use only the normal fresh batch above.  geometryBootstrapItem()
    // selects the robust median once at least three samples are available.
    bootstrapBatch = currentAnchorDistanceBatch(
      anchorIds,
      paperGeometryGuard.acceptedHoldSec,
      solver);
  }
  const bootstrapEdgeCount = Object.keys(
    bootstrapBatch.distanceItems || {}).length;
  const sparseObservableBootstrap = protocol !== "passive_ds" &&
    bootstrapEdgeCount >= minimumObservableAnchorEdges(anchorIds);
  if (!session.ekf && bootstrapBatch.coherent &&
      (bootstrapBatch.missingPairs.length === 0 ||
       sparseObservableBootstrap)) {
    session.ekf = createPaperAnchorEkf(bootstrapBatch);
    geometryUpdated = Boolean(session.ekf);
  }

  if (!session.ekf) {
    const persisted = protocol === "native_ds"
      ? null
      : persistedModuleAnchorGeometry(anchorIds);
    if (persisted) {
      return persistedModuleGeometryResult(
        anchorIds, persisted, protocol);
    }
    return {
      anchors: {},
      distanceItems: batch.distanceItems,
      missingPairs: batch.missingPairs,
      residuals: {},
      complete: false,
      positionReady: false,
      canFix: false,
      status: batch.missingPairs.length ? "waiting" : "initializing",
      protocol,
    };
  }

  const now = Date.now() / 1000;
  let conditioned = conditionPaperAnchorBatch(session.ekf, batch, now);
  if (conditioned.relocation) {
    const relocationBatch = {
      ...batch,
      distanceItems: conditioned.relocation.distanceItems,
      missingPairs: [],
      coherent: true,
      updateToken: `${batch.protocol}:relocation:${now}`,
    };
    const replacement = createPaperAnchorEkf(relocationBatch, session.ekf);
    if (replacement) {
      replacement.relocationCount++;
      replacement.lastRelocationAt = now;
      session.ekf = replacement;
      state.positionSeeds = {};
      resetPositionTagTrails();
      state.positionAnchorTrail = {};
      conditioned = conditionPaperAnchorBatch(session.ekf, batch, now);
      geometryUpdated = true;
    }
  }
  const geometryUpdateItems = protocol === "native_ds"
    ? conditioned.acceptedItems
    : conditioned.updateItems;
  if (Object.keys(conditioned.updateItems).length &&
      updatePaperAnchorEkf(
        session.ekf,
        batch.ids,
        geometryUpdateItems,
        {robust: protocol !== "native_ds"})) {
    session.ekf.lastUpdateToken = batch.updateToken;
    session.ekf.lastFrameId = batch.frameId;
    geometryUpdated = true;
  }

  let targetAnchors = paperAnchorCoordinatesFromState(
    batch.ids, session.ekf.variables, session.ekf.state);
  let anchors = cloneAnchorCoordinates(
    smoothPublishedAnchorCoordinates(
      session.ekf, targetAnchors, geometryUpdated));
  const acceptedItems = acceptedGeometryItems(session.ekf, now);
  let residuals = anchorGeometryResiduals(anchors, acceptedItems);
  let fitQuality = anchorGeometryFitQuality(anchorIds, residuals);
  const expectedPairCount = selectedAnchorPairs(anchorIds).length;

  // All live protocols measure the same six physical anchor edges. A browser
  // that stayed open while anchors moved can retain a published EKF shape that
  // no longer matches those edges, even though the radio measurements are
  // already correct. Rebuild only when a fresh, complete distance graph gives
  // a substantially better deterministic fit. This changes neither the UWB
  // measurements nor the tag solver and never consults GPS.
  if (Object.keys(acceptedItems).length === expectedPairCount &&
      Number.isFinite(fitQuality.rmsM) && fitQuality.rmsM > 0.25) {
    const rebuildBatch = {
      ...batch,
      distanceItems: acceptedItems,
      missingPairs: [],
      coherent: true,
      updateToken: `${batch.protocol}:consistency:${now}`,
    };
    const rebuilt = createPaperAnchorEkf(
      rebuildBatch,
      {...session.ekf, publishedAnchors: null}
    );
    if (rebuilt) {
      const rebuiltAnchors = paperAnchorCoordinatesFromState(
        batch.ids, rebuilt.variables, rebuilt.state);
      const rebuiltResiduals = anchorGeometryResiduals(
        rebuiltAnchors, acceptedItems);
      const rebuiltFit = anchorGeometryFitQuality(
        anchorIds, rebuiltResiduals);
      if (Number.isFinite(rebuiltFit.rmsM) &&
          rebuiltFit.rmsM + 0.10 < fitQuality.rmsM) {
        session.ekf = rebuilt;
        targetAnchors = rebuiltAnchors;
        anchors = cloneAnchorCoordinates(rebuiltAnchors);
        residuals = rebuiltResiduals;
        fitQuality = rebuiltFit;
        state.positionSeeds = {};
        resetPositionTagTrails();
        state.positionAnchorTrail = {};
      }
    }
  }
  const requiredPairCount = protocol === "passive_ds"
    ? expectedPairCount
    : minimumObservableAnchorEdges(anchorIds);
  const acceptedComplete = Object.keys(acceptedItems).length >= requiredPairCount;
  const estimateAgeSec = Math.max(
    0, now - Number(session.ekf.lastGoodAt || 0));
  // A complete, recent range graph is sufficient to obtain the best-fit 2D
  // geometry.  Do not use the paper's 3-sigma residual as an availability
  // switch: outdoor UWB ranges are three-dimensional, so anchors at different
  // heights can legitimately leave a residual above that planar diagnostic
  // limit.  Keep fitQuality.acceptable as a visible quality warning, while
  // continuing to solve and display the position.
  const positionReady = acceptedComplete &&
    estimateAgeSec <= paperGeometryGuard.acceptedHoldSec &&
    Number.isFinite(fitQuality.rmsM);
  const sigmaValues = session.ekf.covariance.map(
    (row, index) => Math.sqrt(Math.max(0, Number(row[index]))));
  const candidatePairs = Object.entries(session.ekf.pairStates || {})
    .filter(([, pairState]) => pairState.candidateCount > 0)
    .map(([key, pairState]) => ({
      key,
      count: pairState.candidateCount,
      deltaM: Number(pairState.candidateDistanceM) -
        Number(pairState.acceptedDistanceM),
    }));
  const heldPairs = Object.entries(conditioned.displayItems)
    .filter(([, item]) => item.geometry_held)
    .map(([key]) => key);
  return {
    anchors,
    distanceItems: conditioned.displayItems,
    missingPairs: batch.missingPairs,
    residuals,
    fitQuality,
    complete: true,
    positionReady,
    canFix: false,
    status: "dynamic",
    protocol,
    updates: session.ekf.updates,
    frameId: session.ekf.lastFrameId,
    elapsedSec: now - session.ekf.startedAt,
    lastUpdateAgeSec: Math.max(
      0, now - Number(session.ekf.lastUpdateAt || 0)),
    estimateAgeSec,
    maxSigmaM: sigmaValues.length ? Math.max(...sigmaValues) : NaN,
    relocationCount: session.ekf.relocationCount,
    lastRelocationAt: session.ekf.lastRelocationAt,
    rejectedCount: session.ekf.rejectedCount,
    lastRejected: session.ekf.lastRejected,
    candidatePairs,
    heldPairs,
  };
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

function normalizeTdoaObservation(item) {
  return {
    ...item,
    diff_m: Number(item.diff_m),
    used_in_fit: true,
    reject_reason: "",
  };
}

function coherentTdoaBatch(tagId, anchorIds, maxAge, protocol = "flextdoa") {
  const selected = new Set(anchorIds.map(Number));
  const recent = Array.isArray(state.tdoa?.recent_observations) && state.tdoa.recent_observations.length
    ? state.tdoa.recent_observations
    : freshTdoaObservations(tagId, anchorIds, maxAge);
  const groups = new Map();
  for (const rawItem of recent) {
    if (Number(rawItem.tag_id) !== Number(tagId) ||
        !selected.has(Number(rawItem.initiator_id)) ||
        !selected.has(Number(rawItem.responder_id)) ||
        Number(rawItem.age_sec) > maxAge) continue;
    const rawProtocol = String(rawItem.tdoa_protocol || "flextdoa");
    if (rawProtocol !== protocol) continue;
    const item = normalizeTdoaObservation(rawItem);
    if (!Number.isFinite(Number(item.diff_m))) continue;
    const slotValue = Number(item.slot_id);
    const frameSlotCount = protocol === "passive_ds"
      ? Math.max(1, anchorIds.length - 1)
      : 1;
    const groupSlotValue = Number.isInteger(slotValue)
      ? Math.floor(slotValue / frameSlotCount)
      : null;
    const slotKey = Number.isInteger(groupSlotValue)
      ? `${protocol === "passive_ds" ? "frame" : "slot"}:${groupSlotValue}`
      : `sample:${Number(item.seq)}:${Number(item.initiator_id)}`;
    const group = groups.get(slotKey) || {
      key: slotKey,
      slotId: Number.isInteger(slotValue) ? slotValue : null,
      seq: Number(item.seq),
      initiatorId: Number(item.initiator_id),
      itemsByResponder: new Map(),
      newestAt: 0,
      oldestAt: Number.POSITIVE_INFINITY,
    };
    const receivedAt = Number(item.received_at) || (Date.now() / 1000 - Number(item.age_sec || 0));
    group.itemsByResponder.set(
      protocol === "passive_ds"
        ? `${Number(item.initiator_id)}:${Number(item.responder_id)}`
        : Number(item.responder_id),
      item
    );
    group.newestAt = Math.max(group.newestAt, receivedAt);
    group.oldestAt = Math.min(group.oldestAt, receivedAt);
    groups.set(slotKey, group);
  }

  const expected = Math.max(2, anchorIds.length - 1);
  const ordered = [...groups.values()].sort((left, right) => right.newestAt - left.newestAt);
  const selectedGroup = ordered.find(group => group.itemsByResponder.size >= expected) || ordered[0];
  if (!selectedGroup) {
    return {items: [], complete: false, expected, slotId: null, seq: null, spanMs: null};
  }
  const items = [...selectedGroup.itemsByResponder.values()].sort((left, right) =>
    Number(left.responder_index ?? 99) - Number(right.responder_index ?? 99));
  return {
    items,
    complete: items.length >= expected,
    expected,
    slotId: selectedGroup.slotId,
    seq: selectedGroup.seq,
    initiatorId: selectedGroup.initiatorId,
    frameId: selectedGroup.slotId === null ? null : Math.floor(
      selectedGroup.slotId /
      (protocol === "passive_ds"
        ? Math.max(1, anchorIds.length - 1)
        : anchorIds.length)
    ),
    spanMs: Math.max(0, (selectedGroup.newestAt - selectedGroup.oldestAt) * 1000),
  };
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

function appendPositionTrailPoint(store, key, position, timestamp, metrics = null) {
  if (!position ||
      !Number.isFinite(Number(position.x)) ||
      !Number.isFinite(Number(position.y))) return;
  const trail = store[key] || [];
  trail.push({
    x: Number(position.x),
    y: Number(position.y),
    t: timestamp,
    sigma_m: Number(metrics?.sigma_m),
    rms_m: Number(metrics?.rms_m),
  });
  let staleCount = 0;
  while (staleCount < trail.length &&
         timestamp - trail[staleCount].t > positionTrailMaxAgeSec) {
    staleCount++;
  }
  if (staleCount > 0) trail.splice(0, staleCount);
  if (trail.length > positionTrailMaxPoints) {
    trail.splice(0, trail.length - positionTrailMaxPoints);
  }
  store[key] = trail;
}

function recordPositionTrailPoint(
  tagId, position, receivedAt, token, metrics = null, rawPosition = null
) {
  if (!position || !Number.isFinite(Number(position.x)) || !Number.isFinite(Number(position.y))) return;
  const key = String(tagId);
  const pointToken = String(token ?? `${receivedAt}:${position.x}:${position.y}`);
  if (state.positionTrailTokens[key] === pointToken) return;
  state.positionTrailTokens[key] = pointToken;
  const timestamp = Number.isFinite(Number(receivedAt)) ? Number(receivedAt) : Date.now() / 1000;
  appendPositionTrailPoint(
    state.positionTrail, key, position, timestamp, metrics);
  appendPositionTrailPoint(
    state.positionRawTrail, key, rawPosition, timestamp, metrics);
}

function resetPositionTagTrails() {
  state.positionTrail = {};
  state.positionRawTrail = {};
  state.positionTrailTokens = {};
  state.positionStreamRenderLatencies = [];
  state.positionStreamLatestEvent = null;
  state.positionStreamLastRenderedEventToken = "";
}

function positionTrailDrawSamples(trail) {
  if (!Array.isArray(trail) || trail.length <= positionTrailMaxDrawPoints) {
    return trail || [];
  }
  const stride = Math.max(
    1, Math.ceil(trail.length / positionTrailMaxDrawPoints));
  const samples = [];
  for (let index = 0; index < trail.length; index += stride) {
    samples.push(trail[index]);
  }
  if (samples[samples.length - 1] !== trail[trail.length - 1]) {
    samples.push(trail[trail.length - 1]);
  }
  return samples;
}

function localPositionAge(item, now = Date.now() / 1000) {
  const receivedAt = Number(item?.received_at);
  if (Number.isFinite(receivedAt) && receivedAt > 0) return Math.max(0, now - receivedAt);
  return Number(item?.age_sec);
}

function observedFlexTagIds(settings, now = Date.now() / 1000) {
  const anchorIds = new Set(settings.anchorIds.map(Number));
  const expectedProtocol = positionGeometryProtocol(settings.solver);
  const displayMaxAge = positionDisplayMaxAge(settings);
  return Object.values(state.tdoa?.local_positions || {})
    .filter(item =>
      Number.isFinite(Number(item?.tag_id)) &&
      !anchorIds.has(Number(item.tag_id)) &&
      String(item?.tdoa_protocol || "flextdoa") === expectedProtocol &&
      localPositionAge(item, now) <= displayMaxAge)
    .map(item => Number(item.tag_id))
    .filter((value, index, values) => values.indexOf(value) === index)
    .sort((left, right) => left - right);
}

function computePositionModel() {
  const settings = positionSettings();
  const displayMaxAge = positionDisplayMaxAge(settings);
  const selectedIds = selectedPositionModuleIds(settings);
  const offlineModuleIds = selectedIds.filter(id => !moduleHttpOnline(statusForModule(id)));
  const switchingModuleIds = selectedIds.filter(id => {
    const item = statusForModule(id);
    return moduleHttpOnline(item) && Boolean(item?.uwb_runtime_switching);
  });
  const active = positionRangingActive(settings);
  const geometry = paperAnchorGeometry(
    settings.anchorIds, positionGeometryMaxAge(settings), settings.solver);
  const anchors = {...(geometry?.anchors || {})};

  if (!active && state.positionWasActive) {
    resetPositionTagTrails();
    state.positionAnchorTrail = {};
    state.positionResults = {};
    state.positionSeeds = {};
  }
  state.positionWasActive = active;

  const tags = {};
  const now = Date.now() / 1000;
  const observedTagIds = observedFlexTagIds(settings, now);
  const configuredTagIds = settings.tagIds.filter(id => !settings.anchorIds.includes(id));
  // A receive-only protocol can be heard by every non-anchor module, but the
  // Position Setup tag list is the display/analysis allow-list.  Do not turn
  // every observed passive receiver into a plotted tag: with a three-anchor
  // selection the fourth physical anchor is also able to publish a passive
  // position, even when the operator requested only T1.
  const effectiveTagIds = [...new Set(configuredTagIds)];
  const unexpectedTagIds = observedTagIds.filter(id => !settings.tagIds.includes(id));
  const missingTagIds = settings.tagIds.filter(id => !observedTagIds.includes(id));
  updatePositionAnchorTrail(anchors, now);
  if (active && geometry.positionReady) {
    for (const tagId of effectiveTagIds) {
      const distances = {};
      const distanceItems = {};
      const wantedProtocol = positionGeometryProtocol(settings.solver);
      const observations = positionProtocolUsesTdoa(settings.solver)
        ? freshTdoaObservations(tagId, settings.anchorIds, settings.maxAge)
            .filter(item => String(item?.tdoa_protocol || "") === wantedProtocol)
        : [];
      const fitObservations = observations;
      let position = null;
      const residuals = {};
      let accuracy = null;
      let localPosition = state.tdoa?.local_positions?.[String(tagId)] || null;
      let rawPosition = null;
      if (String(localPosition?.tdoa_protocol || "flextdoa") !== wantedProtocol) {
        localPosition = null;
      }
      if (!positionProtocolUsesTdoa(settings.solver)) {
        for (const anchorId of settings.anchorIds) {
          const item = state.ranging?.distances?.[`${tagId}:${anchorId}`];
          if (!item || localPositionAge(item, now) > displayMaxAge) continue;
          distances[anchorId] = Number(item.distance_m);
          distanceItems[anchorId] = item;
        }
      }
      const positionIsFresh = Boolean(
        localPosition &&
        localPositionAge(localPosition, now) <= displayMaxAge &&
        Number.isFinite(Number(localPosition.x_m)) &&
        Number.isFinite(Number(localPosition.y_m))
      );
      if (positionIsFresh) {
        position = {
          x: Number(localPosition.x_m),
          y: Number(localPosition.y_m),
        };
        if (Number.isFinite(Number(localPosition.raw_x_m)) &&
            Number.isFinite(Number(localPosition.raw_y_m))) {
          rawPosition = {
            x: Number(localPosition.raw_x_m),
            y: Number(localPosition.raw_y_m),
          };
        }
        accuracy = {
          count: Number(localPosition.observation_count),
          sigma_major_m: Number(localPosition.sigma_m),
          rms_m: Number(localPosition.rms_m),
          max_abs_m: NaN,
          gdop: NaN,
        };
      }
      const coherence = {
        complete: positionIsFresh,
        items: observations,
        expected: Number(localPosition?.observation_count) || observations.length,
        slotId: Number.isFinite(Number(localPosition?.slot_id))
          ? Number(localPosition.slot_id)
          : null,
        seq: Number(localPosition?.frame_id),
        frameId: Number(localPosition?.frame_id ?? localPosition?.slot_id),
        source: "esp32_tag",
      };
      tags[tagId] = {
        tagId,
        distances,
        distanceItems,
        observations,
        fitObservations,
        position,
        metricPosition: position,
        residuals,
        accuracy,
        coherence,
        rawPosition,
        nativeFrame: null,
        heldPosition: false,
        positionAgeSec: positionIsFresh ? localPositionAge(localPosition, now) : 0,
        positionHoldSec: 0,
        solverSource: positionIsFresh ? "ESP32 tag" : "waiting for ESP32 tag",
      };
    }
  }

  state.positionResults = tags;
  const reference = positionKnownReference(settings, anchors);
  return {
    settings,
    active,
    anchors,
    tags,
    geometry,
    offlineModuleIds,
    switchingModuleIds,
    reference,
    observedTagIds,
    unexpectedTagIds,
    missingTagIds,
  };
}

function positionBounds(model) {
  const points = [];
  for (const anchor of Object.values(model.anchors)) points.push(anchor);
  for (const tag of Object.values(model.tags)) {
    if (tag.position) points.push(tag.position);
    if (tag.metricPosition) points.push(tag.metricPosition);
  }
  for (const tagId of Object.keys(model.tags)) {
    const trail = state.positionTrail[tagId] || [];
    for (const point of trail) points.push(point);
  }
  if (model.reference) points.push(model.reference);
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

function drawTagTrail(ctx, tx, trail, color, dashed = false) {
  const samples = positionTrailDrawSamples(trail);
  if (samples.length < 2) return;
  ctx.save();
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = dashed ? 1.15 : 1.6;
  ctx.setLineDash(dashed ? [4, 3] : []);
  samples.forEach((point, index) => {
    const x = tx.x(point.x);
    const y = tx.y(point.y);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawPosition(model) {
  const canvas = document.getElementById("positionCanvas");
  if (!canvas) return;
  const {ctx, width, height} = fitCanvas(canvas);
  if (width <= 1 || height <= 1) return;
  const tx = positionTransform(model, width, height);
  drawPositionGrid(ctx, tx, width, height);

  if (model.reference) {
    const x = tx.x(model.reference.x);
    const y = tx.y(model.reference.y);
    ctx.save();
    ctx.strokeStyle = "#6d4c9f";
    ctx.fillStyle = "#6d4c9f";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.arc(x, y, 9, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(x - 6, y);
    ctx.lineTo(x + 6, y);
    ctx.moveTo(x, y - 6);
    ctx.lineTo(x, y + 6);
    ctx.stroke();
    ctx.restore();
  }

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

  if (!positionProtocolUsesTdoa(model.settings.solver)) {
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

  for (const tagId of Object.keys(model.tags)) {
    drawTagTrail(
      ctx,
      tx,
      state.positionRawTrail[tagId] || [],
      "rgba(123, 135, 152, 0.62)",
      true
    );
    drawTagTrail(
      ctx,
      tx,
      state.positionTrail[tagId] || [],
      "rgba(43, 100, 216, 0.76)"
    );
  }

  if (model.reference) {
    for (const tag of Object.values(model.tags)) {
      if (!tag.position) continue;
      ctx.save();
      ctx.strokeStyle = "rgba(109, 76, 159, 0.68)";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(tx.x(model.reference.x), tx.y(model.reference.y));
      ctx.lineTo(tx.x(tag.position.x), tx.y(tag.position.y));
      ctx.stroke();
      ctx.restore();
    }
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
    if (tag.rawPosition) {
      const rawX = tx.x(tag.rawPosition.x);
      const rawY = tx.y(tag.rawPosition.y);
      ctx.save();
      ctx.strokeStyle = "rgba(123, 135, 152, 0.88)";
      ctx.lineWidth = 1.25;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(rawX, rawY);
      ctx.lineTo(tx.x(tag.position.x), tx.y(tag.position.y));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.arc(rawX, rawY, 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
    const x = tx.x(tag.position.x);
    const y = tx.y(tag.position.y);
    if (tag.heldPosition) {
      ctx.save();
      ctx.strokeStyle = "#b46a00";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(x, y, 11, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
    ctx.fillStyle = "#d7352a";
    ctx.beginPath();
    ctx.arc(x, y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "#17202a";
    ctx.font = "700 12px Inter, sans-serif";
    ctx.fillText(
      `Tag ${tag.tagId}${tag.heldPosition ? " · held" : ""}`,
      x + 11, y - 12);
  }
}

function renderPositionGeometryPanel(model) {
  const rows = document.getElementById("positionGeometryRows");
  const status = document.getElementById("positionGeometryStatus");
  const note = document.getElementById("positionGeometryTableNote");
  const head = document.getElementById("positionGeometryHead");
  if (!rows) return;

  const geometry = model.geometry || {};
  const fixedRtk = geometry.status === "esp_fixed_rtk";
  const fixedNativeDs = fixedRtk &&
    positionGeometryProtocol(model.settings.solver) === "native_ds";
  if (head && fixedNativeDs) {
    head.innerHTML = `<tr><th>Pair</th><th>RTK distance</th><th>survey</th><th>age</th><th>delta</th></tr>`;
  } else if (head) {
    head.innerHTML = fixedRtk
      ? `<tr><th>Pair</th><th>live TWR</th><th>TWR robust σ (10 s)</th><th>age</th><th>Δ vs RTK</th></tr>`
      : `<tr><th>Pair</th><th>live range / stable</th><th>range robust σ</th><th>age</th><th>fit</th></tr>`;
  }
  if (note && fixedNativeDs) {
    note.textContent =
      "Native DS-TWR solves directly in the fixed GPS RTK ENU geometry. Inter-anchor survey traffic is disabled in the positioning frame.";
  } else if (note) {
    note.textContent = fixedRtk
      ? "Diagnostic only: FlexTDOA uses the fixed GPS RTK geometry, not the live inter-anchor TWR ranges shown below. Δ = live TWR − RTK distance."
      : "Live inter-anchor ranges used to maintain the protocol-specific geometry.";
  }
  if (status) {
    if (fixedRtk) {
      status.textContent =
        `Fixed ${positionSolverLabel(model.settings.solver)} geometry · GPS RTK ENU · ` +
        `ESP32 tag M${geometry.sourceTagId || "?"} · ` +
        `generation ${geometry.updates || 0} · ` +
        `latest ${fmtFixed(geometry.lastUpdateAgeSec, 1)} s · fit N/A`;
      status.className = "muted fresh";
    } else if (geometry.status === "esp_dynamic") {
      const fitText = Number.isFinite(Number(geometry.fitQuality?.rmsM))
        ? ` · fit RMS ${fmtPositionCm(geometry.fitQuality.rmsM, 1)}`
        : "";
      status.textContent =
        `Live ${positionSolverLabel(model.settings.solver)} geometry · ` +
        `ESP32 tag M${geometry.sourceTagId || "?"} · ` +
        `generation ${geometry.updates || 0} · ` +
        `latest ${fmtFixed(geometry.lastUpdateAgeSec, 1)} s${fitText}`;
      status.className = geometry.fitQuality?.acceptable
        ? "muted fresh"
        : "muted stale";
    } else if (geometry.status === "dynamic") {
      const fitText = Number.isFinite(Number(geometry.fitQuality?.rmsM))
        ? ` · fit RMS ${fmtPositionCm(geometry.fitQuality.rmsM, 1)}`
        : "";
      const relocationText = Number(geometry.relocationCount || 0) > 0
        ? ` · ${geometry.relocationCount} automatic relocation reset(s)`
        : "";
      const rejectText = Number(geometry.rejectedCount || 0) > 0
        ? ` · ${geometry.rejectedCount} rejected range spike(s)`
        : "";
      const heldText = Number(geometry.heldPairs?.length || 0) > 0
        ? ` · holding ${geometry.heldPairs.length} last-good pair(s)`
        : "";
      const lastRejected = geometry.lastRejected;
      const lastRejectedText = lastRejected &&
          Number.isFinite(Number(lastRejected.at))
        ? ` · last A${String(lastRejected.key).replace(":", "-A")} ` +
          `${Number(lastRejected.deltaM) >= 0 ? "+" : ""}` +
          `${fmtFixed(Number(lastRejected.deltaM) * 100, 1)} cm ` +
          `${fmtFixed(Math.max(0, Date.now() / 1000 - Number(lastRejected.at)), 1)} s ago`
        : "";
      status.textContent =
        `Live ${positionSolverLabel(model.settings.solver)} geometry · ` +
        `${geometry.updates || 0} EKF updates · latest ${fmtFixed(geometry.lastUpdateAgeSec, 1)} s · ` +
        `max σ ${fmtPositionCm(geometry.maxSigmaM, 1)}${fitText}` +
        `${rejectText}${heldText}${lastRejectedText}${relocationText}`;
      status.className = geometry.fitQuality?.acceptable
        ? "muted fresh"
        : "muted stale";
    } else if (geometry.status === "known") {
      status.textContent =
        "Known Native DS-TWR geometry · position solved on Raspberry · " +
        "no anchor-to-anchor traffic";
      status.className = "muted fresh";
    } else if (geometry.status === "persisted") {
      status.textContent =
        geometry.sourceLabel
          ? `${geometry.sourceLabel} · live anchor geometry unavailable in this firmware.`
          : `Persisted module geometry · generation ${geometry.generation || 0}` +
            `${geometry.sourceModuleId ? ` · source M${geometry.sourceModuleId}` : ""}` +
            " · used until fresh anchor-to-anchor ranges are available.";
      status.className = "muted fresh";
    } else {
      const missing = (geometry.missingPairs || [])
        .map(([a, b]) => `A${a}-A${b}`).join(", ");
      status.textContent = missing
        ? `Waiting for fresh ${positionSolverLabel(model.settings.solver)} anchor ranges: ${missing}.`
        : "Initializing the live anchor geometry.";
      status.className = "muted stale";
    }
  }
  const pairRows = selectedAnchorPairs(model.settings.anchorIds).map(([a, b]) => {
    const key = anchorPairKey(a, b);
    const item = geometry.distanceItems?.[key];
    const residual = geometry.residuals?.[key];
    const stats = item?.stats || {};
    const gateStatus = String(item?.geometry_gate_status || "");
    const gateLabel = item?.geometry_held
      ? gateStatus === "rejected spike"
        ? "rejected spike; holding stable"
        : "holding last-good"
      : gateStatus;
    const direction = item
      ? `A${esc(item.initiator_id)}→A${esc(item.responder_id)}`
      : "waiting";
    const gateHtml = gateLabel
      ? `<br><span class="${gateStatus === "rejected spike" ? "stale" : "muted"}">${esc(gateLabel)}</span>`
      : "";
    const accepted = Number(item?.accepted_distance_m);
    const robustSigma = stats.robust_sigma_m === null ||
      stats.robust_sigma_m === undefined
      ? NaN
      : Number(stats.robust_sigma_m);
    const anchorA = geometry.anchors?.[a];
    const anchorB = geometry.anchors?.[b];
    const rtkDistance = anchorA && anchorB
      ? Math.hypot(
          Number(anchorA.x) - Number(anchorB.x),
          Number(anchorA.y) - Number(anchorB.y))
      : NaN;
    const displayDistance = fixedNativeDs ? rtkDistance : Number(item?.distance_m);
    const surveySigmaText = fixedNativeDs
      ? "disabled"
      : Number.isFinite(robustSigma)
        ? fmtCmFromM(robustSigma, 1) + " cm"
        : "-";
    const rtkDelta = fixedRtk && item && Number.isFinite(rtkDistance)
      ? Number(item.distance_m) - rtkDistance
      : NaN;
    const fitCell = fixedNativeDs
      ? "-"
      : fixedRtk
      ? Number.isFinite(rtkDelta)
        ? `${rtkDelta >= 0 ? "+" : ""}${fmtFixed(rtkDelta * 100, 1)} cm`
        : "-"
      : residual === undefined
        ? "-"
        : `${fmtFixed(residual * 100, 1)} cm`;
    return `<tr>
      <td>A${esc(a)}-A${esc(b)}<br><span class="muted">${direction}</span>${gateHtml}</td>
      <td>${Number.isFinite(displayDistance) ? fmtFixed(displayDistance, 3) : "-"}<br><span class="muted">${!fixedNativeDs && Number.isFinite(accepted) ? "stable " + fmtFixed(accepted, 3) : ""}</span></td>
      <td>${surveySigmaText}</td>
      <td class="${item && Number(item.age_sec) <= model.settings.maxAge ? "fresh" : "stale"}">${item ? fmtFixed(item.age_sec, 1) + "s" : "-"}</td>
      <td>${fitCell}</td>
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

function passiveDsRejectionReasonText(mask) {
  const reasons = [
    [1, "incomplete"],
    [2, "frame mismatch"],
    [3, "too few"],
    [4, "singular"],
    [5, "bounds"],
    [6, "RMS"],
    [7, "out of order"],
    [8, "stale prediction"],
  ];
  return reasons
    .filter(([bit]) => Number(mask) & (1 << bit))
    .map(([, label]) => label)
    .join(", ");
}

function renderPositionSolverStatus(model) {
  const settings = model.settings || {};
  const pills = [
    `<span class="position-pill">Protocol: ${esc(positionSolverLabel(settings.solver))}</span>`,
    `<span class="position-pill good">ESP32 tag solver</span>`,
    `<span class="position-pill good">dashboard display only</span>`,
    `<span class="position-pill">fresh ${fmtFixed(settings.maxAge, 1)} s</span>`,
    `<span class="position-pill" id="positionStreamMetrics">position stream connecting</span>`,
    `<span class="position-pill" id="positionTrailMetrics">trail waiting</span>`,
  ];
  const firstTag = Object.values(model.tags || {})[0];
  if (firstTag?.solverSource) {
    pills.push(`<span class="position-pill good">${esc(firstTag.solverSource)}</span>`);
  }
  const liveItem = firstTag
    ? state.tdoa?.local_positions?.[String(firstTag.tagId)]
    : null;
  if (positionIsDirectEspSolve(liveItem)) {
    pills.push(`<span class="position-pill good">raw position · no filter</span>`);
  }
  if (liveItem?.tdoa_protocol === "passive_ds" &&
      liveItem.coherent_batch_telemetry) {
    const rejectText = passiveDsRejectionReasonText(
      liveItem.rejection_reason_mask
    );
    pills.push(
      `<span class="position-pill good">batch ${esc(liveItem.batch_span_ms)} ms · age ${esc(liveItem.batch_max_age_ms)} ms · mask 0x${Number(liveItem.observation_mask || 0).toString(16)}</span>`
    );
    if (Number(liveItem.rejected_since_last || 0) > 0 || rejectText) {
      pills.push(
        `<span class="position-pill warn">${esc(liveItem.rejected_since_last || 0)} rejected · ${esc(rejectText || "reason pending")}</span>`
      );
    }
  }
  const coherence = firstTag?.coherence;
  if (coherence) {
    const slotText = coherence.slotId === null ? `seq ${coherence.seq ?? "-"}` : `slot ${coherence.slotId}`;
    const observationCount = Number.isFinite(Number(firstTag?.accuracy?.count))
      ? Number(firstTag.accuracy.count)
      : coherence.items?.length || 0;
    pills.push(`<span class="position-pill ${coherence.complete ? "good" : "warn"}">${esc(slotText)} · ${observationCount}/${coherence.expected} coherent</span>`);
    if (Number.isFinite(Number(coherence.frameId))) {
      pills.push(`<span class="position-pill">frame ${esc(coherence.frameId)} · span ${fmtFixed(coherence.spanMs, 1)} ms</span>`);
    }
  }
  if (model.unexpectedTagIds?.length || model.missingTagIds?.length) {
    const requested = settings.tagIds?.length
      ? settings.tagIds.map(id => `T${id}`).join(",")
      : "none";
    const observed = model.observedTagIds?.length
      ? model.observedTagIds.map(id => `T${id}`).join(",")
      : "none";
    pills.push(
      `<span class="position-pill warn">setup ${esc(requested)} · radio ${esc(observed)}</span>`
    );
  }
  const geometryStatus = String(model.geometry?.status || "waiting");
  const geometryReady = [
    "dynamic", "esp_dynamic", "esp_fixed_rtk", "known", "persisted",
  ].includes(geometryStatus);
  const geometryLabel = geometryStatus === "esp_fixed_rtk"
    ? "fixed RTK"
    : geometryStatus === "esp_dynamic"
      ? "ESP32 dynamic"
      : geometryStatus;
  pills.push(`<span class="position-pill ${geometryReady ? "good" : "warn"}">geometry ${esc(geometryLabel)}</span>`);

  return `<div class="position-filter-card">
    <b>Position Solver</b>
    <div class="position-filter-row">${pills.join("")}</div>
  </div>`;
}

function renderPositionReadout(model) {
  const readout = document.getElementById("positionReadout");
  const accuracyRows = document.getElementById("positionAccuracyRows");
  const referenceStatus = document.getElementById("positionReferenceStatus");
  const errorRows = document.getElementById("positionErrorRows");
  const rows = document.getElementById("positionDistanceRows");
  const head = document.getElementById("positionMeasurementHead");
  const title = document.getElementById("positionMeasurementTitle");
  const overlay = document.getElementById("positionOverlay");
  if (!readout || !accuracyRows || !rows || !head || !title || !overlay) return;
  const solverName = positionSolverLabel(model.settings.solver);
  const enableText = "Apply";
  const overlayButton = document.getElementById("positionEnableRanging");
  const sideButton = document.getElementById("positionEnableRangingSide");
  const applyInFlight = Boolean(state.positionApplyInFlight);
  if (sideButton) {
    sideButton.textContent = applyInFlight ? "Applying…" : enableText;
    sideButton.disabled = applyInFlight;
  }
  if (overlayButton) {
    overlayButton.textContent = applyInFlight ? "Applying…" : enableText;
    overlayButton.disabled = applyInFlight;
    overlayButton.dataset.action = applyInFlight ? "wait" : "enable";
  }
  renderPositionGeometryPanel(model);

  if (!model.active) {
    overlay.classList.add("active");
    if (applyInFlight || (model.switchingModuleIds || []).length) {
      overlay.querySelector("h2").textContent = "Switching UWB protocol";
      const transitionIds = (model.switchingModuleIds || []).length
        ? model.switchingModuleIds
        : selectedPositionModuleIds(model.settings);
      overlay.querySelector("p").textContent =
        `DW3000 is changing runtime on modules ${transitionIds.join(", ")}. ` +
        "Wi-Fi, HTTP and telemetry remain online.";
      if (overlayButton) {
        overlayButton.textContent = "Switching…";
        overlayButton.disabled = true;
        overlayButton.dataset.action = "wait";
      }
    } else if ((model.offlineModuleIds || []).length) {
      overlay.querySelector("h2").textContent = "Waiting for HTTP status";
      overlay.querySelector("p").textContent = `No live position is shown until these modules answer /status again: ${model.offlineModuleIds.join(", ")}.`;
    } else {
      overlay.querySelector("h2").textContent = `${solverName} is not active`;
      overlay.querySelector("p").textContent = "Apply the selected position runtime to clear old measurements and compute a new live position.";
    }
    readout.innerHTML = `${renderPositionSolverStatus(model)}<div class="position-tag-card"><b>${esc(solverName)} inactive</b><span>No stored position is shown while the selected modules are not in the selected runtime.</span></div>`;
    accuracyRows.innerHTML = "";
    referenceStatus.textContent = model.reference
      ? `${model.reference.label}: x=${fmtFixed(model.reference.x, 3)} m, y=${fmtFixed(model.reference.y, 3)} m`
      : "Known position reference disabled.";
    errorRows.innerHTML = `<tr><td colspan="6"><span class="muted">position runtime inactive</span></td></tr>`;
    title.textContent = positionProtocolUsesTdoa(model.settings.solver) ? "TDOA Observations" : "Distances";
    rows.innerHTML = "";
    return;
  }

  const missingCoords = model.settings.anchorIds.filter(id => !model.anchors[id]);
  if (missingCoords.length) {
    overlay.classList.add("active");
    const fixedFlex = model.settings.solver === "flextdoa";
    overlay.querySelector("h2").textContent = fixedFlex
      ? "Waiting for fixed GPS RTK geometry"
      : "Waiting for live anchor geometry";
    overlay.querySelector("p").textContent = fixedFlex
      ? `Apply RTK-fixed ENU geometry for anchors ${missingCoords.join(", ")} from the GPS map panel.`
      : `Waiting for fresh ${solverName} anchor-to-anchor ranges involving: ${missingCoords.join(", ")}. Positioning starts automatically when the dynamic geometry is complete.`;
    if (overlayButton) {
      overlayButton.textContent = fixedFlex
        ? "Waiting for RTK Geometry"
        : "Waiting for Anchor Ranges";
      overlayButton.disabled = true;
      overlayButton.dataset.action = "wait";
    }
  } else if (!model.geometry?.positionReady) {
    overlay.classList.add("active");
    const fixedFlex = model.settings.solver === "flextdoa";
    overlay.querySelector("h2").textContent = fixedFlex
      ? "Fixed FlexTDOA geometry is not active"
      : "Dynamic anchor self-localization in progress";
    overlay.querySelector("p").textContent = fixedFlex
      ? "Collect RTK-fixed GPS samples and apply the ENU geometry from the GPS map panel."
      : "No geometry has to be fixed. The TWR-EKF starts positioning automatically after a complete live anchor-range set.";
    if (overlayButton) {
      overlayButton.textContent = fixedFlex
        ? "Waiting for RTK Geometry"
        : "Waiting for Dynamic Geometry";
      overlayButton.disabled = true;
      overlayButton.dataset.action = "wait";
    }
  } else {
    overlay.classList.remove("active");
    overlay.querySelector("h2").textContent = `${solverName} is not active`;
    overlay.querySelector("p").textContent = "Apply the selected position runtime to clear old measurements and compute a new live position.";
  }

  const tagCards = Object.values(model.tags).map(tag => {
    const usesTdoa = positionProtocolUsesTdoa(model.settings.solver);
    const fitCount = usesTdoa
      ? (tag.fitObservations || []).length
      : Object.keys(tag.distances || {}).length;
    const freshCount = usesTdoa
      ? (tag.observations || []).length
      : Object.keys(tag.distances || {}).length;
    const total = usesTdoa
      ? Math.max(0, model.settings.anchorIds.length * (model.settings.anchorIds.length - 1))
      : model.settings.anchorIds.length;
    if (!tag.position) {
      return `<div class="position-tag-card"><b>Tag ${esc(tag.tagId)}</b><span>${freshCount}/${total} fresh ${usesTdoa ? "TDOA observations" : "distances"}</span></div>`;
    }
    const sigma = tag.accuracy?.sigma_major_m;
    const accuracyText = model.settings.solver === "flextdoa"
      ? Number.isFinite(Number(tag.accuracy?.rms_m))
        ? ` · equation RMS ${fmtPositionCm(tag.accuracy.rms_m, 1)}`
        : ""
      : Number.isFinite(Number(sigma))
        ? ` · solver σaxis ${fmtPositionSigma(sigma, 1)}`
        : "";
    const referenceStats = positionReferenceErrorStats(
      tag.tagId,
      tag.metricPosition || tag.position,
      model.reference,
      model.settings.errorWindowSec);
    const referenceText = referenceStats
      ? ` · actual ${fmtPositionCm(referenceStats.currentErrorM, 1)}`
      : "";
    const countText = tag.solverSource === "ESP32 tag" &&
      Number.isFinite(Number(tag.accuracy?.count))
      ? `${tag.accuracy.count} raw obs`
      : usesTdoa
      ? `${fitCount}/${total} fit · ${freshCount} fresh`
      : tag.heldPosition
        ? `holding last-good · age ${fmtFixed(tag.positionAgeSec, 2)} s`
        : `${fitCount}/${total} fresh distances`;
    return `<div class="position-tag-card"><b id="positionTagSummary${esc(tag.tagId)}">Tag ${esc(tag.tagId)}: x=${fmtFixed(tag.position.x, 2)} m, y=${fmtFixed(tag.position.y, 2)} m</b><span id="positionTagMeta${esc(tag.tagId)}">${countText}${accuracyText}${referenceText}</span></div>`;
  });
  const emptyTagCard = !model.geometry?.positionReady
    ? `<div class="position-tag-card"><b>waiting for geometry</b><span>${model.settings.solver === "flextdoa" ? "Apply fixed RTK ENU anchor coordinates from the GPS map." : "Positioning starts automatically after fresh anchor-to-anchor ranges initialize the geometry."}</span></div>`
    : `<div class="position-tag-card"><b>waiting for tags</b><span>No selected tag IDs.</span></div>`;
  readout.innerHTML = `${renderPositionSolverStatus(model)}${tagCards.join("") || emptyTagCard}`;
  const accuracyTableRows = Object.values(model.tags).map(tag => {
    const accuracy = tag.accuracy;
    if (!tag.position || !accuracy) {
      return `<tr><td>T${esc(tag.tagId)}</td><td colspan="3"><span class="muted">waiting</span></td></tr>`;
    }
    return `<tr id="positionAccuracyRow${esc(tag.tagId)}">
      <td>T${esc(tag.tagId)}<br><span class="muted">${esc(accuracy.count)} raw obs · GDOP ${esc(fmtFixed(accuracy.gdop, 2))}</span></td>
      <td id="positionSolverSigma${esc(tag.tagId)}">${model.settings.solver === "flextdoa" ? "-" : fmtPositionSigma(accuracy.sigma_major_m, 1)}</td>
      <td id="positionTdoaRms${esc(tag.tagId)}">${fmtPositionCm(accuracy.rms_m, 1)}</td>
      <td id="positionTdoaMax${esc(tag.tagId)}">${fmtPositionCm(accuracy.max_abs_m, 1)}</td>
    </tr>`;
  });
  accuracyRows.innerHTML = accuracyTableRows.join("") || `<tr><td colspan="4"><span class="muted">waiting</span></td></tr>`;
  if (model.reference) {
    referenceStatus.textContent =
      `${model.reference.label}: x=${fmtFixed(model.reference.x, 3)} m, y=${fmtFixed(model.reference.y, 3)} m · rolling ${fmtFixed(model.settings.errorWindowSec, 0)} s`;
    errorRows.innerHTML = Object.values(model.tags).map(tag => {
      const stats = positionReferenceErrorStats(
        tag.tagId,
        tag.metricPosition || tag.position,
        model.reference,
        model.settings.errorWindowSec);
      if (!stats) return `<tr><td>T${esc(tag.tagId)}</td><td colspan="5">waiting</td></tr>`;
      return `<tr id="positionErrorRow${esc(tag.tagId)}">
        <td>T${esc(tag.tagId)}<br><span class="muted" id="positionErrorCount${esc(tag.tagId)}">${esc(stats.count)} pts · ${fmtFixed(stats.spanSec, 1)} s</span></td>
        <td id="positionErrorNow${esc(tag.tagId)}">${fmtPositionCm(stats.currentErrorM, 1)}</td>
        <td id="positionErrorBias${esc(tag.tagId)}">${fmtPositionCm(stats.biasM, 1)}</td>
        <td id="positionErrorRmse${esc(tag.tagId)}">${fmtPositionCm(stats.rmseM, 1)}</td>
        <td id="positionErrorP95${esc(tag.tagId)}">${fmtPositionCm(stats.p95M, 1)}</td>
        <td id="positionErrorMax${esc(tag.tagId)}">${fmtPositionCm(stats.maxM, 1)}</td>
      </tr>`;
    }).join("");
  } else {
    referenceStatus.textContent = "Known position reference disabled.";
    errorRows.innerHTML = `<tr><td colspan="6"><span class="muted">enable a known reference in Position Setup</span></td></tr>`;
  }

  if (positionProtocolUsesTdoa(model.settings.solver)) {
    title.textContent = "TDOA Observations";
    head.innerHTML = `<tr><th>Tag</th><th>Observation</th><th>diff m</th><th>raw m</th><th>age</th><th>resid.</th></tr>`;
    const tdoaRows = [];
    for (const tag of Object.values(model.tags)) {
      for (const item of tag.observations || []) {
        const key = `${item.initiator_id}-${item.responder_id}`;
        const residual = tag.residuals?.[key];
        const used = item.used_in_fit !== false;
        const fitNote = used ? "raw fit" : `skip ${item.reject_reason || "unusable"}`;
        const agreement = Number(item.agreement_m);
        const agreementText = Number.isFinite(agreement) ? ` · agree ${fmtCmFromM(agreement, 1)} cm` : "";
        const blend = Number(item.blend_weight);
        const blendText = Number.isFinite(blend) ? ` · blend ${(blend * 100).toFixed(0)}%` : "";
        const protocolText = item.tdoa_protocol === "passive_ds"
          ? "Passive DS-TWR"
          : (item.tdoa_protocol === "flextdoa" ? "FlexTDOA" : "unknown");
        const cfoPpm = Number(item.clock_offset_ppm);
        const cfoCorrectionM = Number(item.cfo_correction_m);
        const passiveDiag = item.tdoa_protocol === "passive_ds"
          ? ` · CFO ${Number.isFinite(cfoPpm) ? fmtFixed(cfoPpm, 3) + " ppm" : "-"} / ` +
            `${Number.isFinite(cfoCorrectionM) ? fmtFixed(cfoCorrectionM * 100, 1) + " cm" : "-"} · ` +
            `reply ${Number.isFinite(Number(item.reply_delay_us)) ? esc(item.reply_delay_us) + " µs" : "-"} · ` +
            `range ${esc(item.range_source || "-")}` +
            `${Number.isFinite(Number(item.range_age_slots)) ? " age " + esc(item.range_age_slots) + " slots" : ""}`
          : "";
        const suspectText = item.suspect ? " · suspect" : "";
        const fusedText = Number(item.fused_count || 0) > 0 ? ` · fused ${esc(item.fused_count)}` : "";
        tdoaRows.push(`<tr class="${used ? "" : "position-skip"}">
          <td>T${esc(tag.tagId)}</td>
          <td>A${esc(item.initiator_id)}→A${esc(item.responder_id)}<br><span class="muted">${esc(protocolText)} · seq ${esc(item.seq)}${passiveDiag}${agreementText}${blendText}${fusedText}${suspectText} · ${esc(fitNote)}</span></td>
          <td>${fmtFixed(item.diff_m, 3)}</td>
          <td>${Number.isFinite(Number(item.raw_diff_m)) ? fmtFixed(item.raw_diff_m, 3) : "-"}</td>
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
        const used = item?.used_in_fit !== false;
        const fitNote = item
          ? used
            ? ""
            : `<br><span class="stale">skip ${esc(item.reject_reason || "unusable")}</span>`
          : "";
        distanceRows.push(`<tr class="${used ? "" : "position-skip"}">
          <td>T${esc(tag.tagId)}</td>
          <td>A${esc(anchorId)}${fitNote}</td>
          <td>${item ? fmtFixed(item.distance_m, 3) : "-"}</td>
          <td class="${item && Number(item.age_sec) <= model.settings.maxAge ? "fresh" : "stale"}">${item ? fmtFixed(item.age_sec, 1) + "s" : "-"}</td>
          <td>${residual === undefined ? "-" : fmtFixed(residual * 100, 1) + " cm"}</td>
        </tr>`);
      }
    }
    rows.innerHTML = distanceRows.join("");
  }
}

function trimPositionRateWindow(times, nowMs) {
  while (times.length && nowMs - times[0] > 1000) times.shift();
  return times.length;
}

function recordPositionRenderLatency() {
  const pending = state.positionStreamLatestEvent;
  if (!pending ||
      pending.token === state.positionStreamLastRenderedEventToken) return;
  const nowMs = performance.now();
  state.positionStreamLastRenderedEventToken = pending.token;
  state.positionStreamRenderLatencies.push({
    atMs: nowMs,
    valueMs: Math.max(0, nowMs - Number(pending.arrivalMs || nowMs)),
  });
}

function trimPositionRenderLatencies(nowMs) {
  state.positionStreamRenderLatencies =
    state.positionStreamRenderLatencies.filter(
      sample => nowMs - Number(sample.atMs) <= 10000);
  return state.positionStreamRenderLatencies;
}

function positionTrailSummary() {
  const keys = new Set([
    ...Object.keys(state.positionTrail || {}),
    ...Object.keys(state.positionRawTrail || {}),
  ]);
  let ekfCount = 0;
  let rawCount = 0;
  let oldest = Infinity;
  let newest = -Infinity;
  for (const key of keys) {
    const ekf = state.positionTrail[key] || [];
    const raw = state.positionRawTrail[key] || [];
    ekfCount += ekf.length;
    rawCount += raw.length;
    const representative = ekf.length ? ekf : raw;
    if (!representative.length) continue;
    oldest = Math.min(oldest, Number(representative[0].t));
    newest = Math.max(
      newest, Number(representative[representative.length - 1].t));
  }
  return {
    ekfCount,
    rawCount,
    spanSec: Number.isFinite(oldest) && Number.isFinite(newest)
      ? Math.max(0, newest - oldest)
      : 0,
  };
}

function positionIsDirectEspSolve(item) {
  if (!item) return false;
  const protocol = String(item.tdoa_protocol || "flextdoa");
  return protocol === "flextdoa" ||
    protocol === "native_ds" ||
    (item.position_filter === "none" && item.solver_location === "esp32_tag");
}

function updatePositionStreamMetrics() {
  const nowMs = performance.now();
  const rxRate = trimPositionRateWindow(state.positionStreamRxTimes, nowMs);
  const independentRate = trimPositionRateWindow(
    state.positionStreamIndependentTimes,
    nowMs
  );
  const superframeRate = trimPositionRateWindow(
    state.positionStreamSuperframeTimes,
    nowMs
  );
  const correctionRate = trimPositionRateWindow(
    state.positionStreamCorrectionTimes,
    nowMs
  );
  const renderRate = trimPositionRateWindow(state.positionStreamRenderTimes, nowMs);
  const element = document.getElementById("positionStreamMetrics");
  const trailElement = document.getElementById("positionTrailMetrics");
  if (!element) return;
  if (!state.positionStreamConnected) {
    element.textContent = "position stream reconnecting";
    element.className = "position-pill warn";
    if (trailElement) {
      trailElement.textContent = "trail waiting for stream";
      trailElement.className = "position-pill warn";
    }
    return;
  }
  const updatesPerRender = renderRate > 0 ? rxRate / renderRate : 0;
  const passiveDs = positionSettings().solver === "passive_ds";
  const passivePosition = Object.values(
    state.tdoa?.local_positions || {}
  ).find(item => item?.tdoa_protocol === "passive_ds");
  const passiveUnfiltered = passiveDs &&
    passivePosition?.position_filter === "none";
  const displayedPosition = Object.values(
    state.tdoa?.local_positions || {}
  ).find(item => positionTdoaProtocolMatches(
    item, positionSettings().solver));
  const directEspSolve = positionIsDirectEspSolve(displayedPosition);
  const overlappingRate = Math.max(0, rxRate - independentRate);
  element.textContent = directEspSolve
    ? `${rxRate} raw solves/s · ${independentRate} independent/s · ` +
      `${passiveUnfiltered ? `${overlappingRate} overlapping/s · ` : ""}` +
      `${renderRate} fps · ` +
      `${fmtFixed(updatesPerRender, 1)} updates/render`
    : passiveDs
    ? `${rxRate} solver/s · ${independentRate} frames/s · ` +
      `${superframeRate} superframes/s · ${correctionRate} EKF corrections/s · ` +
      `${renderRate} fps · ${fmtFixed(updatesPerRender, 1)} updates/render`
    : `${rxRate} solver updates/s · ${independentRate} independent frames/s · ` +
      `${renderRate} fps · ${fmtFixed(updatesPerRender, 1)} updates/render`;
  element.className = "position-pill good";
  if (trailElement) {
    const trail = positionTrailSummary();
    const latencySamples = trimPositionRenderLatencies(nowMs);
    const latencyValues = latencySamples.map(sample => sample.valueMs);
    const latestLatency = latencyValues.length
      ? latencyValues[latencyValues.length - 1]
      : NaN;
    const p95Latency = percentile(latencyValues, 0.95);
    const latencyText = Number.isFinite(latestLatency)
      ? ` · event→render ${fmtFixed(latestLatency, 1)} ms` +
        ` / p95 ${fmtFixed(p95Latency, 1)} ms`
      : "";
    const usesTdoa = positionProtocolUsesTdoa(positionSettings().solver);
    const nativeMode = positionSettings().nativeDsUpdateMode;
    trailElement.textContent = directEspSolve
      ? `independent raw trail: ${passiveUnfiltered ? trail.rawCount || trail.ekfCount : trail.ekfCount} points · ` +
        `${fmtFixed(trail.spanSec, 1)} s${latencyText}`
      : usesTdoa
      ? `independent trail: EKF ${trail.ekfCount} · raw ${trail.rawCount} · ` +
        `${fmtFixed(trail.spanSec, 1)} s${latencyText}`
      : `${nativeMode === "rolling" ? "rolling" : "coherent"} trail: ` +
        `${trail.ekfCount} points · ` +
        `${fmtFixed(trail.spanSec, 1)} s${latencyText}`;
    trailElement.className = "position-pill good";
  }
}

function positionTdoaProtocolMatches(item, solver) {
  const actual = String(item?.tdoa_protocol || "flextdoa");
  const expected = positionGeometryProtocol(solver);
  return actual === expected;
}

function applyStreamPositionToModel(model) {
  if (!model?.active || !model?.geometry?.positionReady) return;
  const now = Date.now() / 1000;
  const displayMaxAge = positionDisplayMaxAge(model.settings);
  for (const tag of Object.values(model.tags || {})) {
    const item = state.tdoa?.local_positions?.[String(tag.tagId)];
    if (!item ||
        !positionTdoaProtocolMatches(item, model.settings.solver) ||
        localPositionAge(item, now) > displayMaxAge) continue;
    const x = Number(item.x_m);
    const y = Number(item.y_m);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    tag.position = {x, y};
    const rawX = Number(item.raw_x_m);
    const rawY = Number(item.raw_y_m);
    tag.rawPosition =
      Number.isFinite(rawX) && Number.isFinite(rawY)
        ? {x: rawX, y: rawY}
        : null;
    tag.solverSource = "ESP32 tag";
    tag.accuracy = {
      count: Number(item.observation_count),
      sigma_major_m: Number(item.sigma_m),
      rms_m: Number(item.rms_m),
      max_abs_m: NaN,
      gdop: NaN,
    };
  }
}

function updatePositionLiveMetrics(model) {
  for (const tag of Object.values(model.tags || {})) {
    const item = state.tdoa?.local_positions?.[String(tag.tagId)];
    if (!item ||
        !positionTdoaProtocolMatches(item, model.settings.solver) ||
        !tag.position) continue;
    const summary = document.getElementById(`positionTagSummary${tag.tagId}`);
    const meta = document.getElementById(`positionTagMeta${tag.tagId}`);
    const sigma = Number(item.sigma_m);
    const referenceStats = positionReferenceErrorStats(
      tag.tagId, tag.position, model.reference, model.settings.errorWindowSec);
    if (summary) {
      const rawDelta = tag.rawPosition
        ? Math.hypot(
            tag.rawPosition.x - tag.position.x,
            tag.rawPosition.y - tag.position.y
          )
        : NaN;
      const unfilteredEsp = positionIsDirectEspSolve(item);
      summary.textContent = unfilteredEsp
        ? `Tag ${tag.tagId}: x=${fmtFixed(tag.position.x, 3)} m, ` +
          `y=${fmtFixed(tag.position.y, 3)} m`
        : `Tag ${tag.tagId}: EKF x=${fmtFixed(tag.position.x, 3)} m, ` +
          `y=${fmtFixed(tag.position.y, 3)} m` +
          (tag.rawPosition
            ? ` · raw x=${fmtFixed(tag.rawPosition.x, 3)}, ` +
              `y=${fmtFixed(tag.rawPosition.y, 3)} · Δ ${fmtPositionCm(rawDelta, 1)}`
            : "");
    }
    if (meta) {
      const sigmaText = item.tdoa_protocol === "flextdoa"
        ? Number.isFinite(Number(item.rms_m))
          ? ` · equation RMS ${fmtPositionCm(item.rms_m, 1)}`
          : ""
        : Number.isFinite(sigma)
          ? ` · solver σaxis ${fmtPositionSigma(sigma, 1)}`
          : "";
      const referenceText = referenceStats
        ? ` · actual ${fmtPositionCm(referenceStats.currentErrorM, 1)}`
        : "";
      const filterText = item.tdoa_protocol === "flextdoa"
        ? " · raw ESP32 solve · independent frame"
        : item.tdoa_protocol === "native_ds"
        ? " · raw ESP32 solve · coherent frame"
        : item.tdoa_protocol === "passive_ds"
        ? item.position_filter === "none"
          ? " · raw ESP32 solve" +
            (item.independent_frame
              ? " · independent three-star window"
              : " · overlapping three-star window")
          : ` · ${item.filter_correction ? "EKF correction" : "EKF predict-only"}` +
            (item.complete_superframe
              ? " · complete superframe"
              : item.independent_frame
              ? " · independent frame"
              : " · rolling")
        : "";
      const rejectionText = passiveDsRejectionReasonText(
        item.rejection_reason_mask
      );
      const batchText = item.tdoa_protocol === "passive_ds" &&
        item.coherent_batch_telemetry
        ? ` · batch ${item.batch_span_ms} ms / age ${item.batch_max_age_ms} ms` +
          ` / mask 0x${Number(item.observation_mask || 0).toString(16)}` +
          (Number(item.rejected_since_last || 0) > 0
            ? ` · rejected ${item.rejected_since_last}` +
              `${rejectionText ? " (" + rejectionText + ")" : ""}`
            : "")
        : "";
      meta.textContent =
        `${item.observation_count || 0} raw obs${sigmaText}${referenceText}` +
        `${filterText}${batchText} · live`;
    }
    const solverSigma = document.getElementById(`positionSolverSigma${tag.tagId}`);
    const tdoaRms = document.getElementById(`positionTdoaRms${tag.tagId}`);
    const tdoaMax = document.getElementById(`positionTdoaMax${tag.tagId}`);
    if (solverSigma) solverSigma.textContent = item.tdoa_protocol === "flextdoa"
      ? "-"
      : fmtPositionSigma(item.sigma_m, 1);
    if (tdoaRms) tdoaRms.textContent = fmtPositionCm(item.rms_m, 1);
    if (tdoaMax) tdoaMax.textContent = "-";
    if (referenceStats) {
      const values = {
        positionErrorNow: referenceStats.currentErrorM,
        positionErrorBias: referenceStats.biasM,
        positionErrorRmse: referenceStats.rmseM,
        positionErrorP95: referenceStats.p95M,
        positionErrorMax: referenceStats.maxM,
      };
      for (const [prefix, value] of Object.entries(values)) {
        const element = document.getElementById(`${prefix}${tag.tagId}`);
        if (element) element.textContent = fmtPositionCm(value, 1);
      }
      const count = document.getElementById(`positionErrorCount${tag.tagId}`);
      if (count) count.textContent =
        `${referenceStats.count} pts · ${fmtFixed(referenceStats.spanSec, 1)} s`;
    }
  }
}

function renderPositionStreamFrame() {
  state.positionStreamRenderPending = false;
  if (state.activeTab !== "position") return;
  if (positionSettings().solver === "ranging") {
    renderPosition();
    recordPositionRenderLatency();
    state.positionStreamRenderTimes.push(performance.now());
    updatePositionStreamMetrics();
    return;
  }
  if (!state.positionModel) {
    renderPosition();
    return;
  }
  const now = Date.now() / 1000;
  const displayMaxAge = positionDisplayMaxAge(state.positionModel.settings);
  const needsRecoveryRender = Object.values(state.positionModel.tags || {}).some(tag => {
    const item = state.tdoa?.local_positions?.[String(tag.tagId)];
    const livePosition = item &&
      positionTdoaProtocolMatches(item, state.positionModel.settings.solver) &&
      localPositionAge(item, now) <= displayMaxAge &&
      Number.isFinite(Number(item.x_m)) &&
      Number.isFinite(Number(item.y_m));
    return livePosition && (
      !tag.position ||
      tag.solverSource !== "ESP32 tag" ||
      !document.getElementById(`positionTagSummary${tag.tagId}`) ||
      !document.getElementById(`positionAccuracyRow${tag.tagId}`)
    );
  });
  applyStreamPositionToModel(state.positionModel);
  if (needsRecoveryRender) {
    renderPosition();
    void fetchSnapshot();
    return;
  }
  drawPosition(state.positionModel);
  updatePositionLiveMetrics(state.positionModel);
  recordPositionRenderLatency();
  const nowMs = performance.now();
  state.positionStreamRenderTimes.push(nowMs);
  updatePositionStreamMetrics();
}

function schedulePositionStreamRender() {
  if (state.activeTab !== "position" || state.positionStreamRenderPending) return;
  state.positionStreamRenderPending = true;
  requestAnimationFrame(renderPositionStreamFrame);
}

function ingestPositionStreamSample(item) {
  const tagId = Number(item?.tag_id);
  const eventId = Number(item?.position_event_id);
  if (!Number.isFinite(tagId) || tagId <= 0 || !Number.isFinite(eventId)) return;
  const settings = positionSettings();
  // EventSource may deliver the tail of the previous runtime after Apply. A
  // protocol-mismatched sample must not replace the current tag entry, even
  // briefly, because local_positions is keyed only by tag ID.
  if (!positionTdoaProtocolMatches(item, settings.solver)) return;
  const key = String(tagId);
  const previous = state.tdoa?.local_positions?.[key];
  if (positionTdoaProtocolMatches(previous, settings.solver) &&
      Number(previous?.position_event_id || 0) >= eventId) return;
  if (!state.tdoa.local_positions) state.tdoa.local_positions = {};
  item.age_sec = localPositionAge(item);
  state.tdoa.local_positions[key] = item;
  const nowMs = performance.now();
  const tagIsSelected = settings.tagIds.includes(tagId) &&
    !settings.anchorIds.includes(tagId);
  if (tagIsSelected) {
    state.positionStreamLatestEvent = {
      token: `position:${eventId}`,
      arrivalMs: nowMs,
    };
    if (item.independent_frame !== false) {
      const rawX = Number(item.raw_x_m);
      const rawY = Number(item.raw_y_m);
      recordPositionTrailPoint(
        tagId,
        {x: Number(item.x_m), y: Number(item.y_m)},
        item.received_at,
        eventId,
        item,
        Number.isFinite(rawX) && Number.isFinite(rawY)
          ? {x: rawX, y: rawY}
          : null
      );
    }
    state.positionStreamRxTimes.push(nowMs);
    trimPositionRateWindow(state.positionStreamRxTimes, nowMs);
    if (item.independent_frame !== false) {
      state.positionStreamIndependentTimes.push(nowMs);
      trimPositionRateWindow(state.positionStreamIndependentTimes, nowMs);
    }
    if (item.complete_superframe === true) {
      state.positionStreamSuperframeTimes.push(nowMs);
      trimPositionRateWindow(state.positionStreamSuperframeTimes, nowMs);
    }
    if (item.filter_correction === true) {
      state.positionStreamCorrectionTimes.push(nowMs);
      trimPositionRateWindow(state.positionStreamCorrectionTimes, nowMs);
    }
    schedulePositionStreamRender();
  }
}

function startPositionStream() {
  state.positionStream?.close();
  const stream = new EventSource("/api/position-stream");
  state.positionStream = stream;
  stream.onopen = () => {
    state.positionStreamConnected = true;
    updatePositionStreamMetrics();
    // Catch up atomically after an EventSource reconnect; the stream endpoint
    // intentionally returns only the newest event when it has no Last-Event-ID.
    void fetchSnapshot();
  };
  stream.onmessage = event => {
    try {
      ingestPositionStreamSample(JSON.parse(event.data));
    } catch (_) {
      // EventSource reconnects automatically; a malformed sample is isolated.
    }
  };
  stream.onerror = () => {
    state.positionStreamConnected = false;
    updatePositionStreamMetrics();
  };
}

function renderPosition() {
  const model = computePositionModel();
  state.positionModel = model;
  drawPosition(model);
  renderPositionReadout(model);
  updatePositionStreamMetrics();
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
  if (state.activeTab !== "graphs") return;
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
      const enabled = Boolean(statusForModule(moduleId)?.runtime_bno085_accel_enabled);
      ageEl.textContent = enabled
        ? (latest ? `last ${fmtAge(latest.received_at)}${rateText}` : "waiting")
        : (latest ? `disabled · last ${fmtAge(latest.received_at)}` : "disabled");
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
  requestAnimationFrame(renderVisibleTerminals);
  requestAnimationFrame(renderPosition);
  if (id === "rangingSettings") requestAnimationFrame(updateRangingSettingsProtocol);
  if (id === "graphs") fetchAccel();
  if (id === "map") {
    requestAnimationFrame(() => {
      renderGpsMap(state.statuses);
      if (state.gpsMap) {
        state.gpsMap.invalidateSize();
        if (!state.gpsMapHasFit) fitGpsMapToModules();
      }
    });
  }
  scheduleAccelRender();
}

async function fetchLogs() {
  if (state.logFetchPending) return;
  state.logFetchPending = true;
  try {
    const res = await fetch(
      `/api/logs?after=${state.lastId}&limit=4000`, {cache: "no-store"});
    if (!res.ok) throw new Error(`logs HTTP ${res.status}`);
    const data = await res.json();
    if (!data.logs.length && Number(data.next_id) <= state.lastId) {
      state.lastId = 0;
      return;
    }
    if (data.logs.length) {
      state.logs.push(...data.logs);
      if (state.logs.length > 8000) {
        state.logs.splice(0, state.logs.length - 8000);
      }
      state.lastId = Math.max(state.lastId, ...data.logs.map(item => item.id));
      ingestAccelLogs(data.logs);
      renderVisibleTerminals();
      scheduleAccelRender();
    }
    document.getElementById("clientPill").textContent =
      `${data.client_count} log client${data.client_count === 1 ? "" : "s"}`;
  } catch (_) {
    // A dashboard restart or short network outage is expected to interrupt a
    // poll. The next interval retries; no rejected promise escapes globally.
  } finally {
    state.logFetchPending = false;
  }
}

async function fetchAccel() {
  if (state.activeTab !== "graphs") return;
  if (state.accelFetchPending) return;
  state.accelFetchPending = true;
  try {
    const res = await fetch(`/api/accel?after=${state.lastAccelId}&limit=12000`, {cache: "no-store"});
    const data = await res.json();
    if (!data.samples.length && Number(data.next_id) <= state.lastAccelId) {
      state.lastAccelId = 0;
      return;
    }
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
  const fixState = gpsFixState(item);
  const fixText = item.gps_fix_quality_text || "unknown";
  const statusText = enabled
    ? `${powered ? "powered" : "off"} / ${uart ? "uart" : "no uart"}`
    : "disabled";
  const satsUsed = item.gps_satellites ?? "-";
  const satsView = item.gps_satellites_in_view ?? "-";
  const location = item.gps_fix_valid
    ? `${fmtMaybeCoord(item.gps_latitude_deg)}<br>${fmtMaybeCoord(item.gps_longitude_deg)}`
    : `<span class="muted">no fix</span>`;
  const movingRole = String(item.gps_moving_base_role || "none");
  const movingBase = movingRole !== "none"
    ? `<br><span class="${item.gps_moving_base_active ? "ok" : "bad"}">${esc(movingRole.replaceAll("_", " "))}</span>`
    : "";
  const linkAge = movingRole === "local_base"
    ? `up ${fmtAgeMs(item.gps_moving_base_last_uplink_age_ms)}`
    : `down ${fmtAgeMs(item.gps_moving_base_last_downlink_age_ms)} · gaps ${esc(item.gps_moving_base_downlink_gaps ?? 0)}`;
  const ntrip = item.gps_ntrip_configured
    ? `<br><span class="${item.gps_ntrip_stream_active ? "ok" : "bad"}">NTRIP ${esc(item.gps_ntrip_state || "unknown")}</span> · HTTP ${esc(item.gps_ntrip_http_status || "-")} · ${esc(item.gps_ntrip_rtcm_frames ?? 0)} RTCM`
    : "";
  return `
    <span class="${enabled ? "ok" : "muted"}">${esc(statusText)}</span><br>
    <span class="gps-fix-message ${fixState.className}">${esc(fixState.label)}</span><br>
    <span class="muted">${esc(fixText)} · q${esc(item.gps_fix_quality ?? "-")} · ${gpsFixTypeText(item.gps_fix_type)}</span><br>
    <span class="gps-fix-explanation ${fixState.className}">${esc(fixState.explanation)}</span><br>
    sats ${esc(satsUsed)}/${esc(satsView)} · hdop ${fmtMaybeNumber(item.gps_hdop, 2)}<br>
	    ${location}<br>
	    <span class="muted">rx ${fmtAgeMs(item.gps_last_rx_age_ms)} · sent ${esc(sentences)} · err ${esc(item.gps_checksum_errors ?? "-")}/${esc(item.gps_parse_errors ?? "-")}</span>${movingBase}${ntrip}<br>
        <span class="muted">${esc(linkAge)}</span>`;
}

function gpsRxIsFresh(item) {
  const ageMs = Number(item?.gps_last_rx_age_ms);
  return Boolean(
    item?.runtime_gps_enabled &&
    item?.gps_powered &&
    item?.gps_task_running &&
    item?.gps_uart_ready &&
    Number.isFinite(ageMs) &&
    ageMs >= 0 &&
    ageMs < 3000
  );
}

function gpsFixTypeText(value) {
  const fixType = Number(value);
  if (fixType === 2) return "2D";
  if (fixType === 3) return "3D";
  if (fixType === 1) return "no fix";
  return "unknown";
}

function gpsFixState(item) {
  if (!item?.runtime_gps_enabled) {
    return {
      className: "muted",
      label: "GPS disabled",
      explanation: "The receiver is disabled in the runtime configuration.",
    };
  }
  if (!item?.gps_powered) {
    return {
      className: "bad",
      label: "GPS not powered",
      explanation: "The receiver is enabled, but its power rail is off.",
    };
  }
  if (!item?.gps_task_running) {
    return {
      className: "bad",
      label: "GPS task stopped",
      explanation: "The receiver task is not running.",
    };
  }
  if (!item?.gps_uart_ready) {
    return {
      className: "bad",
      label: "GPS UART unavailable",
      explanation: "The receiver is powered, but the UART is not ready.",
    };
  }
  if (!gpsRxIsFresh(item)) {
    return {
      className: "bad",
      label: "No recent GPS data",
      explanation: "No fresh NMEA sentence was received in the last 3 seconds.",
    };
  }

  const quality = Number(item.gps_fix_quality);
  const fixType = Number(item.gps_fix_type);
  const dimension = fixType === 3 ? "3D" : (fixType === 2 ? "2D" : "Position");
  if (!item.gps_fix_valid || !Number.isFinite(quality) || quality === 0) {
    return {
      className: "warn",
      label: "Searching for a fix",
      explanation: "NMEA data is healthy, but no valid position fix is available yet.",
    };
  }

  switch (quality) {
    case 1:
      return {
        className: "ok",
        label: `${dimension} standalone fix`,
        explanation: "Valid GNSS position; RTK corrections are not active.",
      };
    case 2:
      return {
        className: "ok",
        label: `${dimension} differential fix`,
        explanation: "Differential corrections are active; this is not an RTK solution.",
      };
    case 3:
      return {
        className: "ok",
        label: `${dimension} PPS fix`,
        explanation: "A valid PPS-quality position fix is available.",
      };
    case 4:
      return {
        className: "ok",
        label: `${dimension} RTK fixed`,
        explanation: "RTK integer solution acquired.",
      };
    case 5:
      return {
        className: "warn",
        label: `${dimension} RTK float`,
        explanation: "RTK corrections are active; waiting for an integer-fixed solution.",
      };
    case 6:
      return {
        className: "warn",
        label: "Estimated position",
        explanation: "The reported position is estimated, not a live satellite fix.",
      };
    case 7:
      return {
        className: "warn",
        label: "Manual position",
        explanation: "The reported coordinates were entered manually.",
      };
    case 8:
      return {
        className: "warn",
        label: "Simulated position",
        explanation: "The reported coordinates come from simulation mode.",
      };
    default:
      return {
        className: "ok",
        label: `${dimension} valid fix`,
        explanation: "A valid position is available with an unrecognized quality code.",
      };
  }
}

function formatGpsUtcTime(value) {
  const raw = String(value || "");
  if (raw.length < 6) return "-";
  const fraction = raw.slice(6).replace(/^\./, "");
  return `${raw.slice(0, 2)}:${raw.slice(2, 4)}:${raw.slice(4, 6)}${fraction ? `.${fraction}` : ""}`;
}

function formatGpsUtcDate(value) {
  const raw = String(value || "");
  if (raw.length < 6) return "-";
  return `${raw.slice(0, 2)}/${raw.slice(2, 4)}/${raw.slice(4, 6)}`;
}

function gpsMapModuleLabel(item) {
  const moduleId = Number(item?.module_id || 0);
  return moduleId === 1 ? "T1" : `A${moduleId || "?"}`;
}

function gpsMapHasValidCoordinates(item) {
  const lat = Number(item?.gps_latitude_deg);
  const lng = Number(item?.gps_longitude_deg);
  return Boolean(
    item?.gps_fix_valid &&
    gpsRxIsFresh(item) &&
    Number.isFinite(lat) && lat >= -90 && lat <= 90 &&
    Number.isFinite(lng) && lng >= -180 && lng <= 180
  );
}

const gpsRtkAnchorIds = [2, 3, 4, 5];
const gpsRtkMinimumSamples = 5;
const gpsRtkMaximumSamples = 120;

function medianFinite(values) {
  const sorted = values.map(Number).filter(Number.isFinite).sort((a, b) => a - b);
  if (!sorted.length) return NaN;
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2
    ? sorted[middle]
    : (sorted[middle - 1] + sorted[middle]) / 2;
}

function recordGpsRtkSamples(statuses) {
  for (const item of statuses || []) {
    const moduleId = Number(item?.module_id);
    const latitude = Number(item?.gps_latitude_deg);
    const longitude = Number(item?.gps_longitude_deg);
    const altitude = Number(item?.gps_altitude_m);
    if (!Number.isInteger(moduleId) || moduleId < 1 || moduleId > 5 ||
        Number(item?.gps_fix_quality) !== 4 || !item?.gps_fix_valid ||
        !gpsRxIsFresh(item) || !Number.isFinite(latitude) ||
        !Number.isFinite(longitude) || !Number.isFinite(altitude)) continue;
    const token = [
      item.gps_utc_date || "",
      item.gps_utc_time || "",
      latitude.toFixed(9), longitude.toFixed(9), altitude.toFixed(4),
    ].join(":");
    if (state.gpsRtkLastTokens.get(moduleId) === token) continue;
    state.gpsRtkLastTokens.set(moduleId, token);
    const samples = state.gpsRtkSamples.get(moduleId) || [];
    samples.push({latitude, longitude, altitude, capturedAt: Date.now()});
    if (samples.length > gpsRtkMaximumSamples) {
      samples.splice(0, samples.length - gpsRtkMaximumSamples);
    }
    state.gpsRtkSamples.set(moduleId, samples);
  }
}

function gpsRtkEcef(latitudeDeg, longitudeDeg, altitudeM) {
  const a = 6378137.0;
  const flattening = 1 / 298.257223563;
  const eccentricitySquared = flattening * (2 - flattening);
  const latitude = latitudeDeg * Math.PI / 180;
  const longitude = longitudeDeg * Math.PI / 180;
  const sinLatitude = Math.sin(latitude);
  const cosLatitude = Math.cos(latitude);
  const normal = a / Math.sqrt(1 - eccentricitySquared * sinLatitude * sinLatitude);
  return [
    (normal + altitudeM) * cosLatitude * Math.cos(longitude),
    (normal + altitudeM) * cosLatitude * Math.sin(longitude),
    (normal * (1 - eccentricitySquared) + altitudeM) * sinLatitude,
  ];
}

function gpsRtkEcefToEnu(point, origin, latitudeDeg, longitudeDeg) {
  const latitude = latitudeDeg * Math.PI / 180;
  const longitude = longitudeDeg * Math.PI / 180;
  const dx = point[0] - origin[0];
  const dy = point[1] - origin[1];
  const dz = point[2] - origin[2];
  return {
    east: -Math.sin(longitude) * dx + Math.cos(longitude) * dy,
    north: -Math.sin(latitude) * Math.cos(longitude) * dx -
      Math.sin(latitude) * Math.sin(longitude) * dy + Math.cos(latitude) * dz,
    up: Math.cos(latitude) * Math.cos(longitude) * dx +
      Math.cos(latitude) * Math.sin(longitude) * dy + Math.sin(latitude) * dz,
  };
}

function gpsRtkGeometryModel() {
  const originSamples = state.gpsRtkSamples.get(2) || [];
  if (originSamples.length < gpsRtkMinimumSamples) return null;
  for (const anchorId of gpsRtkAnchorIds) {
    if ((state.gpsRtkSamples.get(anchorId) || []).length < gpsRtkMinimumSamples) {
      return null;
    }
  }
  const originLatitude = medianFinite(originSamples.map(sample => sample.latitude));
  const originLongitude = medianFinite(originSamples.map(sample => sample.longitude));
  const originAltitude = medianFinite(originSamples.map(sample => sample.altitude));
  const originEcef = gpsRtkEcef(originLatitude, originLongitude, originAltitude);
  const points = new Map();
  for (const moduleId of [1, ...gpsRtkAnchorIds]) {
    const samples = state.gpsRtkSamples.get(moduleId) || [];
    if (!samples.length) continue;
    const enuSamples = samples.map(sample => gpsRtkEcefToEnu(
      gpsRtkEcef(sample.latitude, sample.longitude, sample.altitude),
      originEcef, originLatitude, originLongitude
    ));
    points.set(moduleId, moduleId === 2
      ? {east: 0, north: 0, up: 0}
      : {
          east: medianFinite(enuSamples.map(point => point.east)),
          north: medianFinite(enuSamples.map(point => point.north)),
          up: medianFinite(enuSamples.map(point => point.up)),
        });
  }
  const anchorUps = gpsRtkAnchorIds.map(id => points.get(id)?.up).filter(Number.isFinite);
  return {
    originLatitude,
    originLongitude,
    originAltitude,
    points,
    verticalSpreadM: anchorUps.length
      ? Math.max(...anchorUps) - Math.min(...anchorUps)
      : NaN,
  };
}

function gpsRtkTagTrack(moduleId = 1) {
  const geometry = gpsRtkGeometryModel();
  const samples = state.gpsRtkSamples.get(Number(moduleId)) || [];
  if (!geometry || !samples.length) return [];
  const originEcef = gpsRtkEcef(
    geometry.originLatitude, geometry.originLongitude,
    geometry.originAltitude
  );
  return samples.map(sample => {
    const point = gpsRtkEcefToEnu(
      gpsRtkEcef(sample.latitude, sample.longitude, sample.altitude),
      originEcef, geometry.originLatitude, geometry.originLongitude
    );
    return {
      x: point.east,
      y: point.north,
      z: point.up,
      t: sample.capturedAt / 1000,
    };
  });
}

function positionGpsReferenceErrorStats(tagId, position, reference, windowSec) {
  const track = gpsRtkTagTrack(reference.tagId || tagId);
  if (!track.length) return null;
  const now = Date.now() / 1000;
  const uwbSamples = (state.positionTrail[String(tagId)] || []).filter(point =>
    Number.isFinite(Number(point.x)) && Number.isFinite(Number(point.y)) &&
    now - Number(point.t) <= windowSec
  );
  const gpsSamples = track.filter(point => now - point.t <= windowSec + 2);
  const aligned = [];
  for (const uwb of uwbSamples) {
    let nearest = null;
    let nearestAge = Infinity;
    for (const gps of gpsSamples) {
      const age = Math.abs(Number(uwb.t) - gps.t);
      if (age < nearestAge) {
        nearest = gps;
        nearestAge = age;
      }
    }
    if (nearest && nearestAge <= 2.0) {
      const dx = Number(uwb.x) - nearest.x;
      const dy = Number(uwb.y) - nearest.y;
      aligned.push({t: Number(uwb.t), dx, dy, error: Math.hypot(dx, dy)});
    }
  }
  const latestGps = track[track.length - 1];
  const currentErrorM = Math.hypot(
    Number(position.x) - latestGps.x,
    Number(position.y) - latestGps.y
  );
  if (!aligned.length) {
    return {
      count: 1, spanSec: 0, currentErrorM, biasM: currentErrorM,
      rmseM: currentErrorM, p95M: currentErrorM, maxM: currentErrorM,
    };
  }
  const meanDx = aligned.reduce((sum, item) => sum + item.dx, 0) / aligned.length;
  const meanDy = aligned.reduce((sum, item) => sum + item.dy, 0) / aligned.length;
  const errors = aligned.map(item => item.error);
  return {
    count: aligned.length,
    spanSec: Math.max(0, aligned[aligned.length - 1].t - aligned[0].t),
    currentErrorM,
    biasM: Math.hypot(meanDx, meanDy),
    rmseM: Math.sqrt(errors.reduce((sum, value) => sum + value * value, 0) / errors.length),
    p95M: percentile(errors, 0.95),
    maxM: Math.max(...errors),
  };
}

function renderGpsRtkGeometryStatus() {
  const status = document.getElementById("gpsRtkGeometryStatus");
  const applyButton = document.getElementById("gpsApplyFlexGeometry");
  if (!status || !applyButton) return;
  const counts = gpsRtkAnchorIds.map(
    id => `M${id}: ${(state.gpsRtkSamples.get(id) || []).length}`
  );
  const geometry = gpsRtkGeometryModel();
  applyButton.disabled = !geometry;
  if (!geometry) {
    status.innerHTML = `${counts.map(esc).join(" · ")}<br>` +
      `<span class="warn">Need at least ${gpsRtkMinimumSamples} RTK-fixed samples from every anchor.</span>`;
    return;
  }
  const rows = gpsRtkAnchorIds.map(id => {
    const point = geometry.points.get(id);
    return `<tr><td>M${id}</td><td>${point.east.toFixed(3)}</td>` +
      `<td>${point.north.toFixed(3)}</td><td>${point.up.toFixed(3)}</td></tr>`;
  }).join("");
  const tag = geometry.points.get(1);
  const tagText = tag
    ? `<br>Tag GPS ground truth now: E ${tag.east.toFixed(3)} m, N ${tag.north.toFixed(3)} m, U ${tag.up.toFixed(3)} m.`
    : `<br><span class="muted">No RTK-fixed tag sample in the current buffer.</span>`;
  status.innerHTML = `${counts.map(esc).join(" · ")}<br>` +
    `<table class="gps-map-distance-table"><thead><tr><th>Anchor</th><th>E m</th><th>N m</th><th>U m</th></tr></thead>` +
    `<tbody>${rows}</tbody></table>` +
    `Anchor vertical spread ${geometry.verticalSpreadM.toFixed(3)} m.${tagText}`;
}

async function applyGpsRtkFlexGeometry() {
  const geometry = gpsRtkGeometryModel();
  if (!geometry) {
    setToast("gpsRtkGeometryToast", "Not enough RTK-fixed anchor samples", "bad");
    return;
  }
  const encoded = gpsRtkAnchorIds.map(id => {
    const point = geometry.points.get(id);
    return `${id}:${Math.round(point.east * 1000)}:${Math.round(point.north * 1000)}`;
  }).join(",");
  setToast("gpsRtkGeometryToast", "writing fixed ENU geometry...", "", null, false);
  const data = await postConfig({
    target_modules: "all",
    params: {flex_geometry: encoded, reboot: "1"},
  }, "gpsRtkGeometryToast");
  if (apiResponseOk(data)) setTimeout(fetchSnapshot, 1800);
}

function clearGpsRtkGeometrySamples() {
  state.gpsRtkSamples.clear();
  state.gpsRtkLastTokens.clear();
  renderGpsRtkGeometryStatus();
  setToast("gpsRtkGeometryToast", "RTK sample buffer cleared", "");
}

function setGpsMapBanner(message = "") {
  const banner = document.getElementById("gpsMapBanner");
  if (!banner) return;
  banner.textContent = message;
  banner.hidden = !message;
}

function ensureGpsMap() {
  if (state.gpsMap) return true;
  const container = document.getElementById("gpsMapCanvas");
  if (!container) return false;
  if (typeof L === "undefined") {
    setGpsMapBanner("Leaflet is unavailable. Install libjs-leaflet and restart the dashboard.");
    return false;
  }

  state.gpsMap = L.map(container, {
    preferCanvas: true,
    zoomControl: true,
    attributionControl: true,
    minZoom: 2,
    maxZoom: 24,
    zoomSnap: 0.25,
    zoomDelta: 0.5,
  }).setView([44.33622, 25.94750], 18);
  state.gpsMapTileLayer = L.tileLayer(
    "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    {
      minZoom: 2,
      maxNativeZoom: 19,
      maxZoom: 24,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap contributors</a>',
    }
  );
  state.gpsMapTileLayer.on("tileerror", () => {
    state.gpsMapTileErrors += 1;
    if (state.gpsMapTileErrors >= 3) {
      setGpsMapBanner("Map background unavailable. Live GPS markers remain active; check the Internet connection.");
    }
  });
  state.gpsMapTileLayer.on("tileload", () => {
    state.gpsMapTileErrors = 0;
    setGpsMapBanner("");
  });
  state.gpsMapTileLayer.addTo(state.gpsMap);
  L.control.scale({imperial: false, metric: true, maxWidth: 180}).addTo(state.gpsMap);

  state.gpsMapTrailLayer = L.polyline([], {
    color: "#2b64d8",
    weight: 3,
    opacity: 0.82,
    lineJoin: "round",
  });
  if (document.getElementById("gpsMapShowTrail")?.checked !== false) {
    state.gpsMapTrailLayer.addTo(state.gpsMap);
  }
  state.gpsMapAnchorPolygon = L.polygon([], {
    color: "#16833a",
    weight: 2,
    opacity: 0.7,
    fillColor: "#16833a",
    fillOpacity: 0.06,
    dashArray: "7 5",
  }).addTo(state.gpsMap);
  state.gpsMapDistanceLayer = L.layerGroup();
  if (document.getElementById("gpsMapShowDistances")?.checked !== false) {
    state.gpsMapDistanceLayer.addTo(state.gpsMap);
  }
  return true;
}

function gpsMapMarkerIcon(item, stale) {
  const tag = Number(item?.module_id) === 1;
  const moduleId = Number(item?.module_id || 0);
  const labelClass = moduleId === 2
    ? "left top"
    : (moduleId === 3 ? "left bottom" : (moduleId === 4 ? "top" : (moduleId === 5 ? "bottom" : "")));
  const classes = ["gps-map-marker", tag ? "tag" : "anchor", stale ? "stale" : ""]
    .filter(Boolean).join(" ");
  return L.divIcon({
    className: "gps-map-div-icon",
    html: `<div class="${classes}"><span class="gps-map-marker-pin"></span><span class="gps-map-marker-label ${labelClass}">${esc(gpsMapModuleLabel(item))}</span></div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  });
}

function gpsMapPopup(item, position, stale) {
  const fixState = gpsFixState(item);
  const lat = Number(position.lat);
  const lng = Number(position.lng);
  const googleUrl = `https://www.google.com/maps?q=${lat.toFixed(8)},${lng.toFixed(8)}`;
  const osmUrl = `https://www.openstreetmap.org/?mlat=${lat.toFixed(8)}&mlon=${lng.toFixed(8)}#map=19/${lat.toFixed(8)}/${lng.toFixed(8)}`;
  return `<b>${esc(gpsMapModuleLabel(item))} · ${esc(item.hostname || `module ${item.module_id}`)}</b><br>
    <span class="${stale ? "warn" : fixState.className}">${stale ? "last known position" : esc(fixState.label)}</span><br>
    <span class="gps-map-coords">${lat.toFixed(8)}, ${lng.toFixed(8)}</span><br>
    altitude ${fmtMaybeNumber(item.gps_altitude_m, 2)} m · HDOP ${fmtMaybeNumber(item.gps_hdop, 2)}<br>
    satellites ${esc(item.gps_satellites ?? "-")} / ${esc(item.gps_satellites_in_view ?? "-")} · fix age ${fmtAgeMs(item.gps_last_fix_age_ms)}<br>
    <a href="${googleUrl}" target="_blank" rel="noopener">Google Maps</a> ·
    <a href="${osmUrl}" target="_blank" rel="noopener">OpenStreetMap</a>`;
}

function gpsMapVisiblePositions() {
  return [...state.gpsMapLastValid.values()].map(position => [position.lat, position.lng]);
}

function gpsMapDistanceMeters(first, second) {
  const lat1 = Number(first?.lat) * Math.PI / 180;
  const lat2 = Number(second?.lat) * Math.PI / 180;
  const deltaLat = lat2 - lat1;
  const deltaLng = (Number(second?.lng) - Number(first?.lng)) * Math.PI / 180;
  if (![lat1, lat2, deltaLat, deltaLng].every(Number.isFinite)) return NaN;
  const sinLat = Math.sin(deltaLat / 2);
  const sinLng = Math.sin(deltaLng / 2);
  const a = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLng * sinLng;
  return 6371008.8 * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(Math.max(0, 1 - a)));
}

function gpsMapDistanceText(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "-";
  if (number < 10) return `${number.toFixed(2)} m`;
  if (number < 100) return `${number.toFixed(1)} m`;
  return `${number.toFixed(0)} m`;
}

function gpsMapPairDistances(sorted, currentValid) {
  const available = sorted
    .map(item => ({item, position: currentValid.get(Number(item.module_id))}))
    .filter(entry => entry.position);
  const pairs = [];
  for (let firstIndex = 0; firstIndex < available.length; firstIndex += 1) {
    for (let secondIndex = firstIndex + 1; secondIndex < available.length; secondIndex += 1) {
      const first = available[firstIndex];
      const second = available[secondIndex];
      const horizontal = gpsMapDistanceMeters(first.position, second.position);
      const firstAltitude = Number(first.item.gps_altitude_m);
      const secondAltitude = Number(second.item.gps_altitude_m);
      const altitudeDelta = Number.isFinite(firstAltitude) && Number.isFinite(secondAltitude)
        ? secondAltitude - firstAltitude
        : NaN;
      const distance3d = Number.isFinite(horizontal) && Number.isFinite(altitudeDelta)
        ? Math.hypot(horizontal, altitudeDelta)
        : NaN;
      pairs.push({
        first,
        second,
        horizontal,
        altitudeDelta,
        distance3d,
        label: `${gpsMapModuleLabel(first.item)}–${gpsMapModuleLabel(second.item)}`,
      });
    }
  }
  return pairs;
}

function fitGpsMapToModules() {
  if (!ensureGpsMap()) return;
  const points = gpsMapVisiblePositions();
  if (!points.length) return;
  if (points.length === 1) {
    state.gpsMap.setView(points[0], 22);
  } else {
    state.gpsMap.fitBounds(L.latLngBounds(points), {padding: [45, 45], maxZoom: 22});
  }
  state.gpsMapHasFit = true;
}

function centerGpsMapOnTag() {
  if (!ensureGpsMap()) return;
  const tag = state.gpsMapLastValid.get(1);
  if (tag) state.gpsMap.setView([tag.lat, tag.lng], Math.max(22, state.gpsMap.getZoom()));
}

function clearGpsMapTrail() {
  state.gpsMapTrail = [];
  state.gpsMapLastTrailToken = "";
  if (state.gpsMapTrailLayer) state.gpsMapTrailLayer.setLatLngs([]);
  renderGpsMap(state.statuses);
}

function wireGpsMapControls() {
  const follow = document.getElementById("gpsMapFollowTag");
  const showTrail = document.getElementById("gpsMapShowTrail");
  const showDistances = document.getElementById("gpsMapShowDistances");
  if (follow) {
    follow.checked = localStorage.getItem("uwbDash.gpsMapFollowTag") === "1";
    follow.addEventListener("change", () => {
      localStorage.setItem("uwbDash.gpsMapFollowTag", follow.checked ? "1" : "0");
    });
  }
  if (showTrail) {
    showTrail.checked = localStorage.getItem("uwbDash.gpsMapShowTrail") !== "0";
    showTrail.addEventListener("change", () => {
      localStorage.setItem("uwbDash.gpsMapShowTrail", showTrail.checked ? "1" : "0");
      if (!ensureGpsMap() || !state.gpsMapTrailLayer) return;
      if (showTrail.checked && !state.gpsMap.hasLayer(state.gpsMapTrailLayer)) {
        state.gpsMapTrailLayer.addTo(state.gpsMap);
      } else if (!showTrail.checked && state.gpsMap.hasLayer(state.gpsMapTrailLayer)) {
        state.gpsMap.removeLayer(state.gpsMapTrailLayer);
      }
    });
  }
  if (showDistances) {
    showDistances.checked = localStorage.getItem("uwbDash.gpsMapShowDistances") !== "0";
    showDistances.addEventListener("change", () => {
      localStorage.setItem("uwbDash.gpsMapShowDistances", showDistances.checked ? "1" : "0");
      if (!ensureGpsMap() || !state.gpsMapDistanceLayer) return;
      if (showDistances.checked && !state.gpsMap.hasLayer(state.gpsMapDistanceLayer)) {
        state.gpsMapDistanceLayer.addTo(state.gpsMap);
      } else if (!showDistances.checked && state.gpsMap.hasLayer(state.gpsMapDistanceLayer)) {
        state.gpsMap.removeLayer(state.gpsMapDistanceLayer);
      }
    });
  }
  document.getElementById("gpsMapFitAll")?.addEventListener("click", fitGpsMapToModules);
  document.getElementById("gpsMapCenterTag")?.addEventListener("click", centerGpsMapOnTag);
  document.getElementById("gpsMapClearTrail")?.addEventListener("click", clearGpsMapTrail);
  document.getElementById("gpsApplyFlexGeometry")?.addEventListener("click", applyGpsRtkFlexGeometry);
  document.getElementById("gpsClearRtkGeometrySamples")?.addEventListener("click", clearGpsRtkGeometrySamples);
}

function renderGpsMap(statuses) {
  const sorted = [...(statuses || [])].sort((a, b) => Number(a.module_id || 0) - Number(b.module_id || 0));
  recordGpsRtkSamples(sorted);
  renderGpsRtkGeometryStatus();
  const currentValid = new Map();
  for (const item of sorted) {
    if (!gpsMapHasValidCoordinates(item)) continue;
    const position = {
      lat: Number(item.gps_latitude_deg),
      lng: Number(item.gps_longitude_deg),
      updatedAt: Date.now(),
      item,
    };
    currentValid.set(Number(item.module_id), position);
    state.gpsMapLastValid.set(Number(item.module_id), position);
  }

  const tagItem = sorted.find(item => Number(item.module_id) === 1);
  const tagPosition = currentValid.get(1);
  if (tagItem && tagPosition) {
    const token = [
      tagItem.gps_utc_date || "",
      tagItem.gps_utc_time || "",
      tagPosition.lat.toFixed(8),
      tagPosition.lng.toFixed(8),
    ].join(":");
    if (token !== state.gpsMapLastTrailToken) {
      state.gpsMapLastTrailToken = token;
      state.gpsMapTrail.push({lat: tagPosition.lat, lng: tagPosition.lng, at: Date.now()});
      if (state.gpsMapTrail.length > 3600) {
        state.gpsMapTrail.splice(0, state.gpsMapTrail.length - 3600);
      }
    }
  }

  const validCount = currentValid.size;
  const pairDistances = gpsMapPairDistances(sorted, currentValid);
  const list = document.getElementById("gpsMapModuleList");
  const summary = document.getElementById("gpsMapSummary");
  const distanceList = document.getElementById("gpsMapDistanceList");
  if (summary) {
    summary.innerHTML = validCount
      ? `<span class="ok">${validCount}/${sorted.length || 5} live GPS fixes</span> · tag trail ${state.gpsMapTrail.length} point${state.gpsMapTrail.length === 1 ? "" : "s"}`
      : `<span class="warn">No fresh GPS fix is available yet.</span>`;
  }
  if (distanceList) {
    distanceList.innerHTML = pairDistances.length
      ? `<table class="gps-map-distance-table">
           <thead><tr><th>Pair</th><th>horizontal</th><th>Δalt</th><th>3D</th></tr></thead>
           <tbody>${pairDistances.map(pair => `<tr>
             <td>${esc(pair.label)}</td>
             <td><b>${gpsMapDistanceText(pair.horizontal)}</b></td>
             <td>${Number.isFinite(pair.altitudeDelta) ? `${pair.altitudeDelta >= 0 ? "+" : ""}${pair.altitudeDelta.toFixed(2)} m` : "-"}</td>
             <td>${gpsMapDistanceText(pair.distance3d)}</td>
           </tr>`).join("")}</tbody>
         </table>`
      : `<span class="muted">Waiting for at least two fresh fixes...</span>`;
  }
  if (list) {
    list.innerHTML = sorted.map(item => {
      const moduleId = Number(item.module_id);
      const current = currentValid.get(moduleId);
      const position = current || state.gpsMapLastValid.get(moduleId);
      const stale = Boolean(position && !current);
      const fixState = gpsFixState(item);
      const location = position
        ? `<div class="gps-map-coords">${position.lat.toFixed(8)}, ${position.lng.toFixed(8)}</div>
           <div>alt ${fmtMaybeNumber(item.gps_altitude_m, 2)} m · HDOP ${fmtMaybeNumber(item.gps_hdop, 2)} · ${esc(item.gps_satellites ?? "-")} sats</div>
           <div class="gps-map-links">
             <a href="https://www.google.com/maps?q=${position.lat.toFixed(8)},${position.lng.toFixed(8)}" target="_blank" rel="noopener">Google Maps</a>
             <a href="https://www.openstreetmap.org/?mlat=${position.lat.toFixed(8)}&mlon=${position.lng.toFixed(8)}#map=19/${position.lat.toFixed(8)}/${position.lng.toFixed(8)}" target="_blank" rel="noopener">OSM</a>
           </div>`
        : `<div class="muted">No position received</div>`;
      return `<div class="gps-map-module">
        <div class="gps-map-module-head">
          <span class="gps-map-module-name">${esc(gpsMapModuleLabel(item))}</span>
          <span class="${stale ? "warn" : fixState.className}">${stale ? "last known" : esc(fixState.label)}</span>
        </div>
        ${location}
      </div>`;
    }).join("");
  }

  if (state.activeTab !== "map" || !ensureGpsMap()) return;
  state.gpsMap.invalidateSize();
  for (const item of sorted) {
    const moduleId = Number(item.module_id);
    const current = currentValid.get(moduleId);
    const position = current || state.gpsMapLastValid.get(moduleId);
    if (!position) continue;
    const stale = !current;
    const styleKey = `${gpsMapModuleLabel(item)}:${stale ? "stale" : "live"}`;
    let marker = state.gpsMapMarkers.get(moduleId);
    if (!marker) {
      marker = L.marker([position.lat, position.lng], {
        icon: gpsMapMarkerIcon(item, stale),
        zIndexOffset: moduleId === 1 ? 1000 : 0,
      }).addTo(state.gpsMap);
      state.gpsMapMarkers.set(moduleId, marker);
    } else {
      marker.setLatLng([position.lat, position.lng]);
      if (marker._uwbGpsStyle !== styleKey) marker.setIcon(gpsMapMarkerIcon(item, stale));
    }
    marker._uwbGpsStyle = styleKey;
    marker.bindPopup(gpsMapPopup(item, position, stale));
  }

  const anchorPoints = sorted
    .filter(item => Number(item.module_id) !== 1 && currentValid.has(Number(item.module_id)))
    .map(item => currentValid.get(Number(item.module_id)));
  if (anchorPoints.length >= 3) {
    const centerLat = anchorPoints.reduce((sum, point) => sum + point.lat, 0) / anchorPoints.length;
    const centerLng = anchorPoints.reduce((sum, point) => sum + point.lng, 0) / anchorPoints.length;
    anchorPoints.sort((a, b) =>
      Math.atan2(a.lat - centerLat, a.lng - centerLng) - Math.atan2(b.lat - centerLat, b.lng - centerLng));
    state.gpsMapAnchorPolygon.setLatLngs([anchorPoints.map(point => [point.lat, point.lng])]);
  } else {
    state.gpsMapAnchorPolygon.setLatLngs([]);
  }
  if (state.gpsMapTrailLayer) {
    state.gpsMapTrailLayer.setLatLngs(state.gpsMapTrail.map(point => [point.lat, point.lng]));
  }
  if (state.gpsMapDistanceLayer) {
    state.gpsMapDistanceLayer.clearLayers();
    for (const pair of pairDistances) {
      const tagPair = Number(pair.first.item.module_id) === 1 || Number(pair.second.item.module_id) === 1;
      L.polyline(
        [[pair.first.position.lat, pair.first.position.lng], [pair.second.position.lat, pair.second.position.lng]],
        {
          color: tagPair ? "#d7352a" : "#2b64d8",
          weight: tagPair ? 2 : 1.5,
          opacity: tagPair ? 0.72 : 0.52,
          dashArray: tagPair ? "5 5" : "3 6",
          interactive: false,
        }
      ).bindTooltip(
        `${pair.label} · ${gpsMapDistanceText(pair.horizontal)}`,
        {permanent: true, direction: "center", className: "gps-map-distance-tooltip", opacity: 0.96}
      ).addTo(state.gpsMapDistanceLayer);
    }
  }
  if (!state.gpsMapHasFit && state.gpsMapLastValid.size) fitGpsMapToModules();
  if (document.getElementById("gpsMapFollowTag")?.checked && tagPosition) {
    state.gpsMap.panTo([tagPosition.lat, tagPosition.lng], {animate: false});
  }
}

function renderGps(statuses) {
  const rows = document.getElementById("gpsRows");
  if (!rows) return;

  const enabledCount = statuses.filter(item => item.runtime_gps_enabled).length;
  const streamingCount = statuses.filter(gpsRxIsFresh).length;
  const fixCount = statuses.filter(item => item.gps_fix_valid && gpsRxIsFresh(item)).length;
  const satellites = statuses.reduce((sum, item) => sum + Math.max(0, Number(item.gps_satellites || 0)), 0);
  const expected = Math.max(5, statuses.length);

  document.getElementById("gpsEnabledCount").textContent = `${enabledCount} / ${expected}`;
  document.getElementById("gpsStreamingCount").textContent = `${streamingCount} / ${expected}`;
  document.getElementById("gpsFixCount").textContent = `${fixCount} / ${expected}`;
  document.getElementById("gpsSatelliteCount").textContent = String(satellites);

  const overview = document.getElementById("gpsOverviewNote");
  if (overview) {
    if (fixCount > 0) {
      overview.innerHTML = `<span class="ok">${fixCount} receiver${fixCount === 1 ? " has" : "s have"} a valid position fix.</span>`;
    } else if (streamingCount > 0) {
      overview.innerHTML = `<span class="warn">NMEA is healthy on ${streamingCount} receiver${streamingCount === 1 ? "" : "s"}, but no valid satellite fix is available yet.</span>`;
    } else if (enabledCount > 0) {
      overview.innerHTML = `<span class="bad">GPS is enabled, but no recent NMEA stream is reaching the dashboard.</span>`;
    } else {
      overview.textContent = "GPS is disabled on all modules. Enable selected receivers from Settings when needed.";
    }
  }

  rows.innerHTML = statuses.map(item => {
    const enabled = Boolean(item.runtime_gps_enabled);
    const streaming = gpsRxIsFresh(item);
    const fixed = Boolean(item.gps_fix_valid && streaming);
    const fixState = gpsFixState(item);
    const receiverClass = !enabled ? "muted" : (streaming ? "ok" : "bad");
    const receiverText = !enabled ? "disabled" : (streaming ? "NMEA streaming" : "no recent NMEA");
    const lastError = item.gps_last_error_name || item.gps_last_error || "-";
    const position = fixed
      ? `<div class="gps-primary-value gps-coordinates">${fmtMaybeCoord(item.gps_latitude_deg)}, ${fmtMaybeCoord(item.gps_longitude_deg)}</div>
         <div class="gps-detail">altitude ${fmtMaybeNumber(item.gps_altitude_m, 2)} m</div>`
      : `<span class="muted">waiting for valid coordinates</span>`;
    const rtkAge = Number(item.gps_rtk_age_s);
    const rtkRatio = Number(item.gps_rtk_ratio);
    const rtkText = (Number.isFinite(rtkAge) && rtkAge > 0) || (Number.isFinite(rtkRatio) && rtkRatio > 0)
      ? `RTK age ${fmtMaybeNumber(rtkAge, 1)} s · ratio ${fmtMaybeNumber(rtkRatio, 2)}`
      : "RTK corrections unavailable";
    const movingRole = String(item.gps_moving_base_role || "none");
    const movingRoleText = movingRole.replaceAll("_", " ");
    const downlinkRole = movingRole === "precise_base" || movingRole === "moving_rover" || movingRole === "rtk_rover";
    const movingLink = movingRole === "none"
      ? `<span class="muted">local moving-base role not assigned</span>`
      : `<span class="${item.gps_moving_base_active ? "ok" : "bad"}">${esc(movingRoleText)}</span><br>
         config ${item.gps_moving_base_receiver_config_sent ? "sent" : "pending"} · ACK/NACK ${esc(item.gps_moving_base_receiver_ack_count ?? 0)}/${esc(item.gps_moving_base_receiver_nack_count ?? 0)}<br>
         uplink ${esc(item.gps_moving_base_uplink_packets ?? 0)} pkt / ${fmtBytes(item.gps_moving_base_uplink_bytes)} · age ${fmtAgeMs(item.gps_moving_base_last_uplink_age_ms)}<br>
         ${downlinkRole ? `downlink ${esc(item.gps_moving_base_downlink_packets ?? 0)} pkt / ${fmtBytes(item.gps_moving_base_downlink_bytes)} · age ${fmtAgeMs(item.gps_moving_base_last_downlink_age_ms)}<br>source M${esc(item.gps_moving_base_last_downlink_source_id ?? 0)} · gaps/errors ${esc(item.gps_moving_base_downlink_gaps ?? 0)}/${esc(item.gps_moving_base_downlink_errors ?? 0)}` : "RTCM source stream"}`;
    const ntripLink = item.gps_ntrip_configured
      ? `<br><span class="${item.gps_ntrip_stream_active ? "ok" : "bad"}">NTRIP ${esc(item.gps_ntrip_state || "unknown")}</span> · TLS ${item.gps_ntrip_tls_connected ? "connected" : "down"} · HTTP ${esc(item.gps_ntrip_http_status || "-")}<br>
         RTCM ${esc(item.gps_ntrip_rtcm_frames ?? 0)} frames / ${fmtBytes(item.gps_ntrip_rtcm_bytes)} · age ${fmtAgeMs(item.gps_ntrip_last_data_age_ms)} · reconnect/errors ${esc(item.gps_ntrip_reconnect_count ?? 0)}/${esc(item.gps_ntrip_error_count ?? 0)}`
      : "";
    const baselineText = item.gps_baseline_valid
      ? `<span class="ok">PSTI${String(item.gps_baseline_source || "").padStart(3, "0")} ${fmtMaybeNumber(item.gps_baseline_length_m, 3)} m</span><br>
         course ${fmtMaybeNumber(item.gps_baseline_course_deg, 2)}° · E/N/U ${fmtMaybeNumber(item.gps_baseline_east_m, 3)} / ${fmtMaybeNumber(item.gps_baseline_north_m, 3)} / ${fmtMaybeNumber(item.gps_baseline_up_m, 3)} m`
      : `<span class="muted">waiting for PSTI032/PSTI035 baseline</span>`;
    const headingText = item.gps_true_heading_valid
      ? `<br><span class="ok">THS ${fmtMaybeNumber(item.gps_true_heading_deg, 2)}° (${esc(item.gps_true_heading_mode || "-")})</span>`
      : "";

    return `<tr class="${statusIsFresh(item) ? "" : "status-stale"}">
      <td>${renderModuleCell(item)}</td>
      <td><div class="gps-primary-value ${receiverClass}">${receiverText}</div>
        <div class="gps-detail">power ${item.gps_powered ? "on" : "off"} · task ${item.gps_task_running ? "running" : "stopped"}<br>
        UART ${item.gps_uart_ready ? "ready" : "not ready"} · error ${esc(lastError)}</div></td>
      <td><div class="gps-primary-value ${fixState.className}">${esc(fixState.label)}</div>
        <div class="gps-detail">quality ${esc(item.gps_fix_quality ?? "-")} · ${gpsFixTypeText(item.gps_fix_type)}<br>
        RMC ${esc(item.gps_rmc_status || "-")} · mode ${esc(item.gps_rmc_mode || "-")}<br>
        fix age ${fmtAgeMs(item.gps_last_fix_age_ms)}<br>
        <span class="${fixState.className}">${esc(fixState.explanation)}</span></div></td>
      <td><div class="gps-primary-value">${esc(item.gps_satellites ?? "-")} used</div>
        <div class="gps-detail">${esc(item.gps_satellites_in_view ?? "-")} in view<br>HDOP ${fmtMaybeNumber(item.gps_hdop, 2)}</div></td>
      <td>${position}</td>
      <td><div>speed ${fmtMaybeNumber(item.gps_speed_mps, 2)} m/s<br>course ${fmtMaybeNumber(item.gps_course_deg, 1)} deg</div>
        <div class="gps-detail">UTC ${formatGpsUtcTime(item.gps_utc_time)}<br>date ${formatGpsUtcDate(item.gps_utc_date)}<br>${rtkText}<br>${baselineText}${headingText}<br>${movingLink}${ntripLink}</div></td>
      <td><div class="gps-primary-value">${esc(item.gps_sentence_count ?? 0)} sentences</div>
        <div class="gps-detail">${fmtBytes(item.gps_byte_count)} · RX age ${fmtAgeMs(item.gps_last_rx_age_ms)}<br>
        GGA ${esc(item.gps_gga_count ?? 0)} · RMC ${esc(item.gps_rmc_count ?? 0)} · GSA ${esc(item.gps_gsa_count ?? 0)}<br>
        GSV ${esc(item.gps_gsv_count ?? 0)} · PSTI030/032/035 ${esc(item.gps_psti030_count ?? 0)}/${esc(item.gps_psti032_count ?? 0)}/${esc(item.gps_psti035_count ?? 0)} · THS ${esc(item.gps_ths_count ?? 0)}<br>
        checksum / parse errors ${esc(item.gps_checksum_errors ?? 0)} / ${esc(item.gps_parse_errors ?? 0)}<br>
        last sentence ${esc(item.gps_last_sentence || "-")}</div></td>
    </tr>`;
  }).join("");
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
        <span class="muted">HS r/w/err ${esc(item.pd_i2c_hs_direct_read_count ?? "-")}/${esc(item.pd_i2c_hs_direct_write_count ?? "-")}/${esc(item.pd_i2c_hs_direct_error_count ?? "-")}</span><br>
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
  synchronizePositionAnchorsFromRuntime(state.statuses);
  state.ranging = snapshot.ranging || {distances: {}, max_age_sec: 3};
  const selectedSolver = positionSettings().solver;
  const expectedPositionProtocol = positionGeometryProtocol(selectedSolver);
  const displayMaxAge = positionDisplayMaxAge(positionSettings());
  const now = Date.now() / 1000;
  const previousLocalPositions = state.tdoa?.local_positions || {};
  const nextTdoa = snapshot.tdoa || {observations: {}, anchor_distances: {}, local_positions: {}, local_geometries: {}, max_age_sec: 3};
  nextTdoa.local_positions = nextTdoa.local_positions || {};
  for (const [tagId, previous] of Object.entries(previousLocalPositions)) {
    const incoming = nextTdoa.local_positions[tagId];
    const previousProtocol = String(previous?.tdoa_protocol || "flextdoa");
    const incomingProtocol = String(incoming?.tdoa_protocol || "flextdoa");
    const previousIsUsable = previousProtocol === expectedPositionProtocol &&
      localPositionAge(previous, now) <= displayMaxAge;
    const incomingMatches = incoming && incomingProtocol === expectedPositionProtocol;
    // Snapshot and EventSource are independent transports. Preserve a newer
    // stream sample only within the selected protocol and only while it is
    // display-fresh. Protocol changes and stale samples remain authoritative
    // deletion boundaries instead of being resurrected indefinitely.
    if (previousIsUsable &&
        (!incomingMatches ||
         Number(previous?.position_event_id || 0) >
           Number(incoming?.position_event_id || 0))) {
      nextTdoa.local_positions[tagId] = previous;
    }
  }
  state.tdoa = nextTdoa;
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
      <td>log ${esc(item.wireless_log_status)}<br>dropped ${esc(item.wireless_log_dropped)}<br>tel ${esc(item.wireless_telemetry_status || "-")}<br>port ${esc(item.wireless_telemetry_port ?? item.runtime_wireless_telemetry_port ?? "-")}<br>tel drop ${esc(item.wireless_telemetry_dropped ?? "-")}<br><span class="muted">full ${esc(item.wireless_telemetry_drop_full ?? "-")} · mutex ${esc(item.wireless_telemetry_drop_mutex ?? "-")} · fmt ${esc(item.wireless_telemetry_drop_format ?? "-")}<br>queue ${esc(item.wireless_telemetry_queue_depth ?? "-")} / 1024 · max ${esc(item.wireless_telemetry_queue_high_water ?? "-")}<br>connect ${esc(item.wireless_telemetry_connect_count ?? "-")} · fail ${esc(item.wireless_telemetry_send_failures ?? "-")} · timeout ${esc(item.wireless_telemetry_send_timeouts ?? "-")} · close ${esc(item.wireless_telemetry_socket_closes ?? "-")}<br>send ${esc(item.wireless_telemetry_last_send_ms ?? "-")} ms · max ${esc(item.wireless_telemetry_max_send_ms ?? "-")} ms<br>bin ${esc(item.wireless_telemetry_binary_frames ?? "-")}f / ${esc(item.wireless_telemetry_binary_samples ?? "-")}s · text ${esc(item.wireless_telemetry_text_frames ?? "-")}</span><br>tel err ${esc(item.wireless_telemetry_last_error ?? "-")}<br>age ${fmtAge(item.status_updated_at)}</td>
      <td>${renderResourceCell(item)}</td>
      <td>${renderBatteryCell(item)}</td>
    </tr>`).join("");
  const freshStatus = state.statuses.find(statusIsFresh) || {};
  renderUwbRadio(freshStatus);
  renderGps(state.statuses);
  renderGpsMap(state.statuses);
  renderCharger(state.statuses);
  renderPd(state.statuses);
  renderPosition();
  renderFlexTdoaTimingDiagram();
  renderNativeDsTwrTimingDiagram();
  renderNativeDsCalibration();
  renderPassiveDsTimingDiagram();
  renderPassiveDsActiveProfile();
  renderPassiveDsExperimentControls();
  renderPassiveDsCalibration();
  scheduleAccelRender();
  updateAccelEnabledControl();
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
  setSettingIfFresh("uwbRadioPhyMode", item.runtime_radio_phy_mode ?? 0);
  setSettingIfFresh("uwbFlexAnchors", (item.runtime_anchor_ids || []).filter(Boolean).join(","));
  setSettingIfFresh("uwbFlexK", item.runtime_flex_tdoa_responder_count);
  setSettingIfFresh("uwbFlexSlots", (item.runtime_flex_tdoa_slot_initiator_ids || []).join(","));
  setSettingIfFresh("uwbFlexMasks", (item.runtime_flex_tdoa_slot_responder_masks || []).join(","));
  setSettingIfFresh("uwbFlexGeneration", item.runtime_flex_tdoa_config_generation, true);
  setSettingIfFresh("uwbSurveyRxMs", item.runtime_anchor_survey_rx_slice_ms);
  setSettingIfFresh("uwbSurveyDelayMs", item.runtime_anchor_survey_command_delay_ms);
  setSettingIfFresh("uwbSurveySlotMs", item.runtime_anchor_survey_slot_ms);
  setSettingIfFresh("uwbSurveyGapMs", item.runtime_anchor_survey_round_gap_ms);
  setSettingIfFresh("uwbSurveyLogEvery", item.runtime_anchor_survey_passive_tag_log_every);
  setSettingIfFresh("uwbRangingSlotMs", item.runtime_ranging_slot_ms);
  setSettingIfFresh("uwbRangingGapMs", item.runtime_ranging_round_gap_ms);
  setSettingIfFresh("uwbRangingRxMs", item.runtime_ranging_rx_slice_ms);
  setSettingIfFresh("uwbRangingTimeoutMs", item.runtime_ranging_rx_timeout_ms);
  setSettingIfFresh("uwbRangingRespDelayMs", item.runtime_ranging_resp_delay_ms);
  setSettingIfFresh("uwbRangingFinalDelayMs", item.runtime_ranging_final_delay_ms);
  setSettingIfFresh("uwbRangingAutoRxDelayUus", item.runtime_ranging_auto_rx_delay_uus);
  setSettingIfFresh(
    "nativeDsRangeBiasMm",
    (item.runtime_native_ds_range_bias_mm || []).join(",")
  );
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
    ["PHY", Number(item.runtime_radio_phy_mode) === 1 ? "long range · 850 kb/s · preamble 1024" : "fast · 6.8 Mb/s · preamble 128"],
    ["channel", item.uwb_radio_channel],
    ["rf channel bit", item.uwb_radio_rf_channel_bit],
    ["preamble len code", item.uwb_radio_preamble_len_code],
    ["preamble code", item.uwb_radio_preamble_code],
    ["PAC", item.uwb_radio_pac],
    ["data rate", item.uwb_radio_data_rate],
    ["PHR mode/rate", `${item.uwb_radio_phr_mode ?? "-"} / ${item.uwb_radio_phr_rate ?? "-"}`],
    ["SFD type", item.uwb_radio_sfd_type],
    ["SFD timeout", item.uwb_radio_sfd_timeout],
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
  if (state.snapshotFetchPending) return;
  state.snapshotFetchPending = true;
  try {
    const res = await fetch("/api/snapshot", {cache: "no-store"});
    if (!res.ok) throw new Error(`snapshot HTTP ${res.status}`);
    renderInfo(await res.json());
  } catch (error) {
    console.warn("snapshot refresh failed; retrying", error);
  } finally {
    state.snapshotFetchPending = false;
  }
}

function snapshotPollDelayMs() {
  if (state.activeTab === "position") return 250;
  if (state.activeTab === "map") return 1000;
  return 1500;
}

function scheduleSnapshotPoll() {
  setTimeout(async () => {
    try {
      await fetchSnapshot();
    } finally {
      scheduleSnapshotPoll();
    }
  }, snapshotPollDelayMs());
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
  const ids = ["rangingProfileTargets", "flexProfileTargets"];
  for (const profileKey of Object.keys(rangingProfileDefaults)) {
    for (const field of rangingProfileFields) {
      ids.push(rangingProfileElementId(profileKey, field.suffix));
    }
  }
  for (const profileKey of Object.keys(flexProfileDefaults)) {
    const profile = flexProfileDefaults[profileKey];
    for (const field of flexProfileFields) {
      ids.push(`${profile.prefix}${field.suffix}`);
    }
  }
  return ids;
}

function readFlexProfile(profileKey) {
  const profile = flexProfileDefaults[profileKey];
  const result = {};
  if (!profile) return result;
  for (const field of flexProfileFields) {
    result[field.key] = Number(
      document.getElementById(`${profile.prefix}${field.suffix}`)?.value
    );
  }
  return result;
}

function writeFlexProfile(profileKey, values, persist = true) {
  const profile = flexProfileDefaults[profileKey];
  if (!profile) return;
  for (const field of flexProfileFields) {
    const id = `${profile.prefix}${field.suffix}`;
    const el = document.getElementById(id);
    if (!el || values[field.key] === undefined) continue;
    el.value = String(values[field.key]);
    if (persist) localStorage.setItem(settingKey(id), el.value);
  }
  updateFlexProfileSummary(profileKey);
}

function flexProfileMetrics(values) {
  const runtime = flexTimingRuntimeConfig();
  const K = Math.max(1, Number(runtime.responderCount || 3));
  const M = Math.max(1, Number(runtime.slotCount || 4));
  const responseTotalUs = K * values.responseUs;
  const responseProcessTotalUs = K * values.responseProcessUs;
  const slotUs = values.guardUs + values.requestUs + values.requestProcessUs +
    responseTotalUs + responseProcessTotalUs;
  const frameUs = M * slotUs;
  return {
    K,
    M,
    responseTotalUs,
    responseProcessTotalUs,
    slotUs,
    frameUs,
    frameHz: frameUs > 0 ? 1000000 / frameUs : NaN,
  };
}

function updateFlexProfileSummary(profileKey) {
  const profile = flexProfileDefaults[profileKey];
  const summary = document.getElementById(`${profile?.prefix || ""}Summary`);
  if (!profile || !summary) return;
  const values = readFlexProfile(profileKey);
  const valid = flexProfileFields.every(field =>
    Number.isFinite(values[field.key]) && values[field.key] > 0
  );
  if (!valid) {
    summary.textContent = "incomplete profile";
    summary.className = "profile-summary warn";
    return;
  }
  const metrics = flexProfileMetrics(values);
  summary.textContent =
    `Guard ${values.guardUs} + REQ ${values.requestUs} + Process REQ ${values.requestProcessUs} + ` +
    `${metrics.K} × RESP ${metrics.responseTotalUs} + ${metrics.K} × Process RESP ${metrics.responseProcessTotalUs} = ` +
    `slot ${fmtFixed(metrics.slotUs / 1000, 3)} ms · frame ${fmtFixed(metrics.frameUs / 1000, 3)} ms · ` +
    `${fmtFixed(metrics.frameHz, 2)} Hz`;
  summary.className = "profile-summary";
}

function updateAllFlexProfileSummaries() {
  Object.keys(flexProfileDefaults).forEach(updateFlexProfileSummary);
}

async function applyFlexProfile(profileKey) {
  const profile = flexProfileDefaults[profileKey];
  if (!profile) return;
  const values = readFlexProfile(profileKey);
  if (!flexProfileFields.every(field =>
    Number.isFinite(values[field.key]) && values[field.key] > 0
  )) {
    setToast("flexProfileToast", "Profile has invalid values", "bad");
    return;
  }
  const metrics = flexProfileMetrics(values);
  setToast(
    "flexProfileToast",
    `applying ${fmtFixed(metrics.frameUs / 1000, 3)} ms frame...`,
    "", null, false
  );
  const data = await postConfig({
    target_modules: document.getElementById("flexProfileTargets").value,
    params: {
      flex_guard_us: String(values.guardUs),
      flex_req_us: String(values.requestUs),
      flex_req_process_us: String(values.requestProcessUs),
      flex_resp_us: String(values.responseUs),
      flex_resp_process_us: String(values.responseProcessUs),
      survey_rx_ms: String(values.rxSliceMs),
      reboot: "1",
    },
  }, "flexProfileToast");
  if (apiResponseOk(data)) {
    const freshAge = document.getElementById("positionMaxAgeSec");
    if (freshAge) {
      freshAge.value = String(values.positionMaxAgeSec);
      localStorage.setItem(
        positionProtocolMaxAgeKey("flextdoa"), freshAge.value
      );
    }
    setTimeout(fetchSnapshot, 1800);
  }
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

function rangingSettingsSolver() {
  return normalizePositionSolver(
    document.getElementById("positionSolver")?.value || "flextdoa"
  );
}

function rangingProfileRuntimeParams(values, solver) {
  if (solver === "ranging") {
    return {
      ranging_slot_ms: String(values.slotMs),
      ranging_gap_ms: String(values.roundGapMs),
      ranging_rx_ms: String(values.dsRxSliceMs),
      ranging_timeout_ms: String(values.timeoutMs),
      ranging_resp_delay_ms: String(values.respDelayMs),
      ranging_final_delay_ms: String(values.finalDelayMs),
      ranging_auto_rx_delay_uus: String(values.autoRxDelayUus),
      reboot: "1",
    };
  }
  return {};
}

function mirrorRangingProfileToUwbFields(values, solver) {
  const fields = solver === "ranging" ? {
    uwbRangingSlotMs: values.slotMs,
    uwbRangingGapMs: values.roundGapMs,
    uwbRangingRxMs: values.dsRxSliceMs,
    uwbRangingTimeoutMs: values.timeoutMs,
    uwbRangingRespDelayMs: values.respDelayMs,
    uwbRangingFinalDelayMs: values.finalDelayMs,
    uwbRangingAutoRxDelayUus: values.autoRxDelayUus,
  } : {};
  for (const [id, value] of Object.entries(fields)) {
    const el = document.getElementById(id);
    if (!el || value === undefined || !Number.isFinite(Number(value))) continue;
    el.value = String(value);
    if (id === "positionMaxAgeSec") {
      localStorage.setItem(positionProtocolMaxAgeKey(solver), el.value);
    } else {
      localStorage.setItem(settingKey(id), el.value);
    }
  }
}

function applyRangingProfilePositionSettings(values, solver) {
  const fields = {
    positionMaxAgeSec: values.dsPositionMaxAgeSec,
  };
  for (const [id, value] of Object.entries(fields)) {
    const el = document.getElementById(id);
    if (!el || value === undefined || value === null) continue;
    el.value = String(value);
    if (id === "positionMaxAgeSec") {
      localStorage.setItem(positionProtocolMaxAgeKey(solver), el.value);
    } else {
      localStorage.setItem(settingKey(id), el.value);
    }
  }
}

function setRangingProfileFieldVisible(profileKey, field, visible) {
  const id = rangingProfileElementId(profileKey, field.suffix);
  const input = document.getElementById(id);
  const label = document.querySelector(`label[for="${id}"]`);
  input?.classList.toggle("hidden", !visible);
  label?.classList.toggle("hidden", !visible);
}

function rangingProfileDescription(values, solver) {
  if (solver !== "ranging") return "";
  const baseFrameMs = 4 * values.slotMs + values.roundGapMs;
  return `${fmtFixed(values.dsPositionMaxAgeSec, 1)} s distance freshness; ` +
    `native POLL → RESP → FINAL → RESULT base frame is ${fmtFixed(baseFrameMs, 0)} ms ` +
    "for four anchors. No geometry-maintenance packets are inserted.";
}

function updateRangingSettingsProtocol() {
  const solver = rangingSettingsSolver();
  const protocol = {
    flextdoa: {
      title: "FlexTDOA Settings",
      label: "FlexTDOA",
      hint: "Selected in Position Setup. Only native FlexTDOA CI-CR timing and frame profiles are shown.",
      note: "FlexTDOA frame profiles are stored and applied independently from native DS-TWR profiles.",
    },
    ranging: {
      title: "DS-TWR Settings",
      label: "Native DS-TWR",
      hint: "Selected in Position Setup. The clean baseline uses POLL, delayed RESP, delayed FINAL and a one-shot delayed RESULT for each tag-anchor range.",
      note: "Use the 46 ms RTK-validated profile for high-rate operation, 64 ms as an intermediate step, and 100 ms as the conservative fallback. Applying a profile persists it and reboots the selected modules; FlexTDOA, Passive DS-TWR, distance-test and calibration timing remain untouched.",
    },
    passive_ds: {
      title: "Passive DS-TWR Settings",
      label: "Passive DS-TWR",
      hint: "Selected in Position Setup. Anchors exchange native three-packet DS-TWR while every non-anchor tag only receives.",
      note: "Fast Star uses one rotating maintenance frame in four to keep the full anchor geometry observable. Robust Rotating changes reference every frame. Their timing remains separate from native DS-TWR and FlexTDOA.",
    },
  }[solver];

  document.getElementById("rangingProtocolTitle").textContent = protocol.title;
  document.getElementById("rangingProtocolHint").textContent = protocol.hint;
  document.getElementById("rangingProtocolBadge").textContent = protocol.label;
  document.querySelectorAll(".ranging-protocol-panel").forEach(panel => {
    panel.classList.toggle("hidden", panel.dataset.rangingProtocol !== solver);
  });

  const profilesSection = document.getElementById("rangingProfilesSection");
  profilesSection.classList.toggle("hidden", solver !== "ranging");
  document.getElementById("flexTdoaProfilesSection")
    ?.classList.toggle("hidden", solver !== "flextdoa");
  document.getElementById("passiveDsProfilesSection")
    ?.classList.toggle("hidden", solver !== "passive_ds");
  document.getElementById("rangingProfileNote").textContent = protocol.note;
  renderNativeDsActiveProfile();

  const visibleFields = rangingProtocolProfileFields[solver];
  for (const profileKey of Object.keys(rangingProfileDefaults)) {
    for (const field of rangingProfileFields) {
      setRangingProfileFieldVisible(profileKey, field, visibleFields.has(field.key));
    }
  }
  updateAllRangingProfileSummaries();
  updateAllFlexProfileSummaries();
  if (solver === "flextdoa") renderFlexTdoaTimingDiagram();
  if (solver === "ranging") renderNativeDsTwrTimingDiagram();
  if (solver === "passive_ds") {
    renderPassiveDsTimingDiagram();
    renderPassiveDsActiveProfile();
    renderPassiveDsExperimentControls();
  }
}

const flexTimingFallback = {
  guardUs: 500,
  requestUs: 250,
  requestProcessUs: 1500,
  responseUs: 250,
  responseProcessUs: 600,
};

function flexTimingRuntimeConfig() {
  const candidates = state.statuses.filter(item =>
    Array.isArray(item.runtime_anchor_ids) && item.runtime_anchor_ids.length >= 3
  );
  const status = candidates.find(item =>
    statusIsFresh(item) && item.runtime_mode_name === "uwb_flex_tdoa"
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
  const anchorIds = (status.runtime_anchor_ids || [2, 3, 4, 5])
    .map(Number)
    .filter(Number.isFinite);
  const initiators = (status.runtime_flex_tdoa_slot_initiator_ids || anchorIds)
    .map(Number)
    .filter(Number.isFinite);
  const slotCount = Math.max(1, Math.min(
    Number(status.runtime_flex_tdoa_slot_count || initiators.length || anchorIds.length),
    initiators.length || anchorIds.length
  ));
  const responderCount = Math.max(1, Math.min(
    Number(status.runtime_flex_tdoa_responder_count || anchorIds.length - 1),
    Math.max(1, anchorIds.length - 1)
  ));
  const masks = (status.runtime_flex_tdoa_slot_responder_masks || [])
    .map(Number);
  const timing = {
    guardUs: Number(status.runtime_flex_tdoa_guard_us || flexTimingFallback.guardUs),
    requestUs: Number(status.runtime_flex_tdoa_request_subslot_us || flexTimingFallback.requestUs),
    requestProcessUs: Number(status.runtime_flex_tdoa_request_process_us || flexTimingFallback.requestProcessUs),
    responseUs: Number(status.runtime_flex_tdoa_response_subslot_us || flexTimingFallback.responseUs),
    responseProcessUs: Number(status.runtime_flex_tdoa_response_process_us || flexTimingFallback.responseProcessUs),
  };
  return {
    status,
    anchorIds,
    initiators: initiators.slice(0, slotCount),
    masks,
    slotCount,
    responderCount,
    timing,
    live: Boolean(statusIsFresh(status)),
  };
}

function flexTimingLatestSlotId() {
  const values = [];
  for (const item of Object.values(state.tdoa.local_positions || {})) {
    if (Number.isFinite(Number(item.slot_id))) values.push(Number(item.slot_id));
  }
  for (const item of Object.values(state.tdoa.observations || {})) {
    if (Number.isFinite(Number(item.slot_id))) values.push(Number(item.slot_id));
  }
  return values.length ? Math.max(...values) : 0;
}

function flexTimingResponders(config, initiatorId, absoluteSlotId, slotIndex) {
  const mask = Number(config.masks[slotIndex]);
  let allowed = config.anchorIds.filter((anchorId, anchorIndex) =>
    anchorId !== initiatorId &&
    (!Number.isFinite(mask) || (mask & (1 << anchorIndex)) !== 0)
  );
  if (!allowed.length) {
    allowed = config.anchorIds.filter(anchorId => anchorId !== initiatorId);
  }
  if (!allowed.length) return [];
  const rotation = ((absoluteSlotId % allowed.length) + allowed.length) % allowed.length;
  const ordered = [];
  for (let index = 0; index < config.responderCount; index += 1) {
    ordered.push(allowed[(index + rotation) % allowed.length]);
  }
  return ordered;
}

function flexTimingAxisMark(position, label, edge = "", stagger = false) {
  return `<span class="flex-axis-mark ${edge} ${stagger ? "stagger" : ""}" style="left:${position}%">${esc(label)}</span>`;
}

function flexTimingRuntimeConsensus(field, unit = "") {
  const values = state.statuses
    .filter(statusIsFresh)
    .map(item => Number(item[field]))
    .filter(Number.isFinite);
  if (!values.length) return {value: null, text: "unavailable"};
  const unique = [...new Set(values)];
  if (unique.length > 1) {
    return {value: null, text: `mixed (${unique.join(", ")})${unit ? ` ${unit}` : ""}`};
  }
  return {value: unique[0], text: `${unique[0]}${unit ? ` ${unit}` : ""}`};
}

function renderFlexTdoaTimingDiagram() {
  const root = document.getElementById("flexTdoaTimingDiagram");
  const selector = document.getElementById("flexTimingSlotSelect");
  if (!root || !selector) return;
  if (rangingSettingsSolver() !== "flextdoa") return;

  const config = flexTimingRuntimeConfig();
  if (config.anchorIds.length < 3 || config.initiators.length < 1) {
    root.className = "muted";
    root.textContent = "FlexTDOA topology is unavailable.";
    return;
  }

  const latestSlotId = flexTimingLatestSlotId();
  const frameStartSlot = latestSlotId - (latestSlotId % config.slotCount);
  const liveSlotIndex = latestSlotId % config.slotCount;
  const selectedIndex = Math.max(0, Math.min(
    config.slotCount - 1,
    Number(state.flexTimingSlotIndex || 0)
  ));
  state.flexTimingSlotIndex = selectedIndex;
  selector.innerHTML = config.initiators.map((initiatorId, index) =>
    `<option value="${index}">slot[${index}] · A${esc(initiatorId)}</option>`
  ).join("");
  selector.value = String(selectedIndex);

  const K = config.responderCount;
  const M = config.slotCount;
  const timing = config.timing;
  const responseProcessTotalUs = K * timing.responseProcessUs;
  const slotUs = timing.guardUs + timing.requestUs + timing.requestProcessUs +
    K * timing.responseUs + responseProcessTotalUs;
  const frameUs = M * slotUs;
  const frameHz = 1000000 / frameUs;
  const responseHz = M * K * frameHz;
  const rxSlice = flexTimingRuntimeConsensus(
    "runtime_anchor_survey_rx_slice_ms", "ms"
  );
  const rxSliceUs = Number.isFinite(rxSlice.value) ? rxSlice.value * 1000 : 0;
  const rxSliceWidth = rxSliceUs > 0 ? Math.min(100, 100 * rxSliceUs / slotUs) : 0;
  const rxSliceSlots = rxSliceUs > 0 ? rxSliceUs / slotUs : 0;
  const selectedSlotId = frameStartSlot + selectedIndex;
  const selectedInitiator = config.initiators[selectedIndex];
  const selectedResponders = flexTimingResponders(
    config, selectedInitiator, selectedSlotId, selectedIndex
  );

  const frameSlots = config.initiators.map((initiatorId, index) => {
    const absoluteSlotId = frameStartSlot + index;
    const responders = flexTimingResponders(config, initiatorId, absoluteSlotId, index);
    const classes = [
      "flex-frame-slot",
      index === selectedIndex ? "selected" : "",
      config.live && index === liveSlotIndex ? "live" : "",
    ].filter(Boolean).join(" ");
    return `<div class="${classes}">
      <b>slot[${index}] · seq ${esc(absoluteSlotId)}</b>
      <strong>A${esc(initiatorId)} initiator</strong>
      <span title="${esc(responders.map(id => `A${id}`).join(" → "))}">RESP ${esc(responders.map(id => `A${id}`).join(" → "))}</span>
    </div>`;
  }).join("");
  const frameAxis = Array.from({length: M + 1}, (_, index) =>
    flexTimingAxisMark(
      100 * index / M,
      `${fmtFixed(index * slotUs / 1000, 2)} ms`,
      index === 0 ? "edge-start" : (index === M ? "edge-end" : "")
    )
  ).join("");

  const segments = [
    {key: "REQ subslot", short: "REQ", duration: timing.requestUs, cls: "req", detail: `A${selectedInitiator} TX at slot boundary; budget includes frame airtime`},
    {key: "Process REQ", short: "P_REQ", duration: timing.requestProcessUs, cls: "req-process", detail: "complete RX + decode + arm delayed TX"},
    ...selectedResponders.map((anchorId, index) => ({
      key: `RESP[${index}]`,
      short: `R${index}`,
      duration: timing.responseUs,
      cls: `response ${index % 2 ? "alt" : ""}`,
      detail: `A${anchorId} delayed TX`,
    })),
    {key: "Process RESP", short: "P_RESP", duration: responseProcessTotalUs, cls: "response-process", detail: `${K} × ${timing.responseProcessUs} us`},
    {key: "Guard time", short: "GUARD TIME", duration: timing.guardUs, cls: "guard", detail: "quiet guard before next REQ"},
  ];
  const segmentCells = segments.map(segment => {
    return `<div class="flex-slot-segment ${segment.cls}" title="${esc(`${segment.key}: ${segment.detail}, ${segment.duration} us`)}">
      <b>${esc(segment.short)}</b><span>${esc(segment.duration)} us</span>
    </div>`;
  }).join("");

  const boundaries = [0];
  let elapsedUs = 0;
  for (const segment of segments) {
    elapsedUs += segment.duration;
    boundaries.push(elapsedUs);
  }
  const slotAxis = boundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / slotUs,
      value >= 1000 ? `${fmtFixed(value / 1000, value % 1000 ? 2 : 1)} ms` : `${value} us`,
      index === 0 ? "edge-start" : (index === boundaries.length - 1 ? "edge-end" : ""),
      index > 1 && index < boundaries.length - 2 && index % 2 === 1
    )
  ).join("");

  elapsedUs = 0;
  const detailRows = segments.map((segment, index) => {
    const startUs = elapsedUs;
    const endUs = startUs + segment.duration;
    elapsedUs = endUs;
    let action = segment.detail;
    if (segment.key.startsWith("RESP[")) {
      const responseIndex = Number(segment.key.match(/\d+/)?.[0] || 0);
      const responderId = selectedResponders[responseIndex];
      const delayedUs = timing.requestUs + timing.requestProcessUs +
        responseIndex * timing.responseUs;
      action = `A${responderId} delayed TX at REQ_RX + ${fmtFixed(delayedUs / 1000, 2)} ms`;
    }
    return `<tr>
      <td>${esc(segment.key)}</td>
      <td>${fmtFixed(startUs / 1000, 2)}</td>
      <td>${fmtFixed(endUs / 1000, 2)}</td>
      <td>${esc(segment.duration)} us</td>
      <td>${esc(action)}</td>
    </tr>`;
  }).join("");

  const responseFlow = selectedResponders.map((anchorId, index) => {
    const delayedUs = timing.requestUs + timing.requestProcessUs + index * timing.responseUs;
    return `<div class="flex-packet-row">
      <b>RESP[${index}] · A${esc(anchorId)} → broadcast · +${fmtFixed(delayedUs / 1000, 2)} ms</b>
      <code>header(type, source, seq) | slot32 | destination_count=0 | processing_dtu=reply | previous_twr(responder, distance_mm, slot16)</code>
    </div>`;
  }).join("");

  root.className = "";
  root.innerHTML = `
    <div class="flex-timing-metrics">
      <div class="flex-timing-metric"><span>Topology</span><strong>N=${config.anchorIds.length} · K=${K} · M=${M}</strong></div>
      <div class="flex-timing-metric"><span>Slot period</span><strong>${fmtFixed(slotUs / 1000, 3)} ms</strong></div>
      <div class="flex-timing-metric"><span>Frame period</span><strong>${fmtFixed(frameUs / 1000, 3)} ms</strong></div>
      <div class="flex-timing-metric"><span>Frame rate</span><strong>${fmtFixed(frameHz, 2)} Hz</strong></div>
      <div class="flex-timing-metric"><span>TDOA / frame</span><strong>${M * K}</strong></div>
      <div class="flex-timing-metric"><span>Response rate</span><strong>${fmtFixed(responseHz, 1)} /s</strong></div>
    </div>
    <div class="flex-timing-scroll">
      <div class="flex-timing-canvas">
        <div class="flex-timing-label">
          <strong>Frame · ${M} initiator slots</strong>
          <span>${config.live ? `live frame containing slot ${latestSlotId}` : "configured topology"}</span>
        </div>
        <div class="flex-frame-track" style="grid-template-columns:repeat(${M}, minmax(170px, 1fr))">${frameSlots}</div>
        <div class="flex-frame-axis">${frameAxis}</div>
        <div class="flex-frame-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">Frame = ${M} × ${fmtFixed(slotUs / 1000, 3)} ms = ${fmtFixed(frameUs / 1000, 3)} ms</span>
          </div>
          <div class="flex-round-gap-zero"><span>FlexTDOA frame gap = 0 ms · next frame starts immediately</span></div>
        </div>
        <div class="flex-timing-label">
          <strong>Selected slot ${selectedSlotId} · A${esc(selectedInitiator)} initiates · ${K} response subslots</strong>
          <span>REQ-to-REQ time reference</span>
        </div>
        <div class="flex-slot-track" style="grid-template-columns:${segments.map(segment => `${segment.duration}fr`).join(" ")}">${segmentCells}</div>
        <div class="flex-slot-axis">${slotAxis}</div>
        <div class="flex-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">FlexTDOA slot = ${fmtFixed(slotUs / 1000, 3)} ms</span>
          </div>
          <div class="flex-dimension gap" style="left:${100 * (slotUs - timing.guardUs) / slotUs}%;width:${100 * timing.guardUs / slotUs}%;top:31px">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">GUARD TIME = ${timing.guardUs} us</span>
          </div>
        </div>
        <div class="flex-host-window">
          <strong>RX host window</strong>
          <div>
            <div class="flex-host-track" title="Maximum receive-call duration; not radio airtime">
              <div class="flex-host-fill" style="width:${rxSliceWidth}%"></div>
            </div>
            <div class="flex-host-measure" style="width:${rxSliceWidth || 100}%">
              <div class="flex-dimension-line"></div>
              <span class="flex-dimension-label">RX slice = ${esc(rxSlice.text)}</span>
            </div>
          </div>
          <span>${esc(rxSlice.text)}${rxSliceSlots > 0 ? ` · ${fmtFixed(rxSliceSlots, 2)} slot${rxSliceSlots === 1 ? "" : "s"} max` : ""}</span>
        </div>
      </div>
    </div>
    <div class="flex-timing-detail-grid">
      <table class="flex-timing-table">
        <thead><tr><th>Interval</th><th>Start ms</th><th>End ms</th><th>Budget</th><th>Radio action</th></tr></thead>
        <tbody>${detailRows}</tbody>
      </table>
      <div class="flex-packet-flow">
        <div class="flex-packet-row">
          <b>REQ · A${esc(selectedInitiator)} → ${esc(selectedResponders.map(id => `A${id}`).join(", "))}</b>
          <code>header(type, source, seq) | slot32 | destination_count=${K} | responders[${K}] | processing_dtu=0 | previous_twr(responder, distance_mm, slot16)</code>
        </div>
        ${responseFlow}
        <div class="flex-packet-row">
          <b>Passive tag</b>
          <code>RX timestamps REQ and every RESP; emits no UWB packet.</code>
        </div>
      </div>
    </div>
    <div class="flex-parameter-map">
      <div class="flex-timing-label">
        <strong>Live runtime parameter map</strong>
        <span>Values read from all fresh modules; “mixed” is reported explicitly.</span>
      </div>
      <table class="flex-parameter-table">
        <thead><tr><th>Scope</th><th>Parameter and live value</th><th>Where it acts</th></tr></thead>
        <tbody>
          <tr>
            <td><span class="flex-scope fixed">FlexTDOA radio</span></td>
            <td>Compact CI-CR timing · ${fmtFixed(slotUs / 1000, 3)} ms/slot</td>
            <td>Defines the colored REQ, P_REQ, RESP, P_RESP and guard widths. The active FlexTDOA frame profile writes these values to every ESP and persists them in NVS.</td>
          </tr>
          <tr>
            <td><span class="flex-scope host">FlexTDOA host</span></td>
            <td>RX slice · ${esc(rxSlice.text)}</td>
            <td>Maximum duration of one software receive call, shown by the outlined bar above. It is not airtime; an anchor schedule alarm can end it early.</td>
          </tr>
        </tbody>
      </table>
    </div>
    <div class="flex-timing-note">The cyclic view starts at REQ. Colored widths are protocol time budgets, not packet airtime: REQ and each RESP transmit at their subslot boundary. The final ${timing.guardUs} us GUARD TIME is the configured quiet interval inside every slot; pure FlexTDOA has no separate frame-level round gap. P_RESP is K × ${timing.responseProcessUs} us. Response offsets are native DW3000 delayed-TX targets relative to REQ_RX.</div>`;
}

const dsTimingFallback = {
  slotMs: 3,
  roundGapMs: 1,
  rxSliceMs: 100,
  timeoutMs: 3,
  respDelayMs: 1,
  finalDelayMs: 1,
  autoRxDelayUus: 500,
};

function dsTimingRuntimeConfig() {
  const candidates = state.statuses.filter(item =>
    Array.isArray(item.runtime_anchor_ids) && item.runtime_anchor_ids.length >= 3
  );
  const status = candidates.find(item =>
    statusIsFresh(item) && item.runtime_mode_name === "uwb_ranging"
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
  const anchorIds = (status.runtime_anchor_ids || [2, 3, 4, 5])
    .map(Number)
    .filter(Number.isFinite);
  return {
    status,
    anchorIds,
    tagId: Number(status.runtime_tag_id || 1),
    slotMs: Number(status.runtime_ranging_slot_ms || dsTimingFallback.slotMs),
    roundGapMs: Number(
      status.runtime_ranging_round_gap_ms ?? dsTimingFallback.roundGapMs
    ),
    rxSliceMs: Number(
      status.runtime_ranging_rx_slice_ms || dsTimingFallback.rxSliceMs
    ),
    timeoutMs: Number(
      status.runtime_ranging_rx_timeout_ms || dsTimingFallback.timeoutMs
    ),
    respDelayMs: Number(
      status.runtime_ranging_resp_delay_ms || dsTimingFallback.respDelayMs
    ),
    finalDelayMs: Number(
      status.runtime_ranging_final_delay_ms || dsTimingFallback.finalDelayMs
    ),
    autoRxDelayUus: Number(
      status.runtime_ranging_auto_rx_delay_uus ||
      dsTimingFallback.autoRxDelayUus
    ),
    live: Boolean(
      statusIsFresh(status) && status.runtime_mode_name === "uwb_ranging"
    ),
  };
}

function dsTimingLatestResult(config) {
  return Object.values(state.ranging?.distances || {})
    .filter(item =>
      Number(item.tag_id) === config.tagId &&
      config.anchorIds.includes(Number(item.anchor_id)) &&
      Number.isFinite(Number(item.frame_id ?? item.seq)))
    .sort((left, right) => {
      const leftTime = Number(left.received_at);
      const rightTime = Number(right.received_at);
      if (Number.isFinite(leftTime) && Number.isFinite(rightTime)) {
        return rightTime - leftTime;
      }
      return Number(left.age_sec || 0) - Number(right.age_sec || 0);
    })[0] || null;
}

function nativeDsPipelineDiagnosticsHtml() {
  const rows = state.statuses
    .filter(statusIsFresh)
    .map(item => {
      const stats = item.native_ds_pipeline_stats;
      if (!stats || stats.poll_tx === undefined) return "";
      const moduleId = Number(item.module_id);
      const tagId = Number(item.runtime_tag_id || 1);
      const role = moduleId === tagId ? "tag" : "anchor";
      return `<tr>
        <td>M${esc(item.module_id)}</td>
        <td>${role}</td>
        <td>${esc(stats.poll_tx ?? 0)}/${esc(stats.poll_rx ?? 0)}</td>
        <td>${esc(stats.response_tx ?? 0)}/${esc(stats.response_rx ?? 0)}</td>
        <td>${esc(stats.final_tx ?? 0)}/${esc(stats.final_rx ?? 0)}</td>
        <td>${esc(stats.result_tx ?? 0)}/${esc(stats.result_rx ?? 0)}</td>
        <td>${esc(stats.completed_ranges ?? 0)}</td>
        <td>${esc(stats.rx_timeouts ?? 0)}</td>
        <td>${esc(stats.invalid_frames ?? 0)}/${esc(stats.delayed_tx_errors ?? 0)}/${esc(stats.rejected_ranges ?? 0)}</td>
        <td>${esc(stats.slot_overruns ?? 0)}</td>
        <td>${Number.isFinite(Number(stats.last_distance_mm)) ? fmtFixed(Number(stats.last_distance_mm) / 1000, 3) : "-"}</td>
      </tr>`;
    })
    .filter(Boolean)
    .join("");
  if (!rows) {
    return `<div class="flex-timing-note warn">Waiting for clean Native DS-TWR counters from the modules.</div>`;
  }
  return `<div class="flex-parameter-map">
    <div class="flex-timing-label">
      <strong>Live protocol counters</strong>
      <span>Direct counters from the clean POLL → RESP → FINAL → RESULT implementation.</span>
    </div>
    <div class="table-wrap"><table class="flex-parameter-table">
      <thead><tr>
        <th>Module</th><th>Role</th><th>POLL TX/RX</th>
        <th>RESP TX/RX</th><th>FINAL TX/RX</th><th>RESULT TX/RX</th><th>Ranges</th>
        <th>RX timeout</th><th>invalid/delayed/rejected</th>
        <th>slot overrun</th><th>last m</th>
      </tr></thead><tbody>${rows}</tbody>
    </table></div>
  </div>`;
}

function renderNativeDsTwrTimingDiagram() {
  const root = document.getElementById("nativeDsTwrTimingDiagram");
  const selector = document.getElementById("dsTimingSlotSelect");
  if (!root || !selector) return;
  if (rangingSettingsSolver() !== "ranging") return;

  const config = dsTimingRuntimeConfig();
  const N = config.anchorIds.length;
  if (N < 3) {
    root.className = "muted";
    root.textContent = "Native DS-TWR topology is unavailable.";
    return;
  }

  const selectedIndex = Math.max(
    0, Math.min(N - 1, Number(state.dsTimingSlotIndex || 0))
  );
  state.dsTimingSlotIndex = selectedIndex;
  const latest = dsTimingLatestResult(config);
  const liveFrameAnchorIds = [...config.anchorIds];
  selector.innerHTML = liveFrameAnchorIds.map((anchorId, index) =>
    `<option value="${index}">slot[${index}] · T${esc(config.tagId)} ↔ A${esc(anchorId)}</option>`
  ).join("");
  selector.value = String(selectedIndex);

  const liveAnchorIndex = latest
    ? liveFrameAnchorIds.indexOf(Number(latest.anchor_id))
    : -1;
  const latestFrameId = Number(latest?.frame_id ?? latest?.seq);
  const frameId = Number.isInteger(latestFrameId) ? latestFrameId : null;

  const frameMs = N * config.slotMs + config.roundGapMs;
  const frameHz = frameMs > 0 ? 1000 / frameMs : NaN;
  const exchangeHz = N * frameHz;
  const slotRemainderMs = Math.max(
    0, config.slotMs - 2 * config.respDelayMs - config.finalDelayMs
  );
  const timingOverrun = 2 * config.respDelayMs + config.finalDelayMs >
    config.slotMs;

  const frameSlots = liveFrameAnchorIds.map((anchorId, index) => {
    const classes = [
      "flex-frame-slot",
      index === selectedIndex ? "selected" : "",
      config.live && index === liveAnchorIndex ? "live" : "",
    ].filter(Boolean).join(" ");
    const slotFrameId = frameId;
    return `<div class="${classes}">
      <b>slot[${index}]${slotFrameId === null ? "" : ` · frame ${esc(slotFrameId)}`}</b>
      <strong>T${esc(config.tagId)} ↔ A${esc(anchorId)}</strong>
      <span>POLL → RESP → FINAL → RESULT</span>
    </div>`;
  }).join("");
  const gapCell = config.roundGapMs > 0
    ? `<div class="ds-frame-gap"><b>frame gap</b><strong>${fmtFixed(config.roundGapMs, 0)} ms</strong><span>quiet scheduler gap</span></div>`
    : "";
  const frameColumns = [
    ...config.anchorIds.map(() => `${Math.max(0.001, config.slotMs)}fr`),
    ...(config.roundGapMs > 0
      ? [`${Math.max(0.001, config.roundGapMs)}fr`]
      : []),
  ].join(" ");
  const frameBoundaries = [0];
  for (let index = 1; index <= N; index += 1) {
    frameBoundaries.push(index * config.slotMs);
  }
  if (config.roundGapMs > 0) frameBoundaries.push(frameMs);
  const frameAxis = frameBoundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / frameMs,
      `${fmtFixed(value, 0)} ms`,
      index === 0 ? "edge-start" :
        (index === frameBoundaries.length - 1 ? "edge-end" : "")
    )
  ).join("");

  const selectedAnchorId = liveFrameAnchorIds[selectedIndex];
  const segments = [
    {
      key: "POLL → RESP",
      short: "POLL → RESP",
      duration: config.respDelayMs,
      cls: "req",
      detail: `A${selectedAnchorId} schedules RESP at POLL_RX + ${fmtFixed(config.respDelayMs, 0)} ms`,
    },
    {
      key: "RESP → FINAL",
      short: "RESP → FINAL",
      duration: config.finalDelayMs,
      cls: "response",
      detail: `T${config.tagId} schedules FINAL at RESP_RX + ${fmtFixed(config.finalDelayMs, 0)} ms`,
    },
    {
      key: "FINAL → RESULT",
      short: "FINAL → RESULT",
      duration: config.respDelayMs,
      cls: "req",
      detail: `A${selectedAnchorId} computes the raw range and schedules its one-shot RESULT + ${fmtFixed(config.respDelayMs, 0)} ms after FINAL_RX`,
    },
    {
      key: "Guard time",
      short: "GUARD TIME",
      duration: slotRemainderMs,
      cls: "guard",
      detail: "responder computes the range; scheduler then waits to the next slot boundary",
    },
  ];
  const visibleSegments = segments.filter(segment => segment.duration > 0);
  const segmentCells = visibleSegments.map(segment =>
    `<div class="flex-slot-segment ${segment.cls}" title="${esc(`${segment.key}: ${segment.detail}`)}">
      <b>${esc(segment.short)}</b><span>${fmtFixed(segment.duration, 0)} ms</span>
    </div>`
  ).join("");
  const slotColumns = visibleSegments
    .map(segment => `${Math.max(0.001, segment.duration)}fr`)
    .join(" ");
  let elapsedMs = 0;
  const slotBoundaries = [0];
  for (const segment of visibleSegments) {
    elapsedMs += segment.duration;
    slotBoundaries.push(elapsedMs);
  }
  const slotAxis = slotBoundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / config.slotMs,
      `${fmtFixed(value, 0)} ms`,
      index === 0 ? "edge-start" :
        (index === slotBoundaries.length - 1 ? "edge-end" : "")
    )
  ).join("");

  elapsedMs = 0;
  const detailRows = visibleSegments.map(segment => {
    const startMs = elapsedMs;
    const endMs = startMs + segment.duration;
    elapsedMs = endMs;
    return `<tr>
      <td>${esc(segment.key)}</td>
      <td>${fmtFixed(startMs, 3)}</td>
      <td>${fmtFixed(endMs, 3)}</td>
      <td>${fmtFixed(segment.duration, 3)} ms</td>
      <td>${esc(segment.detail)}</td>
    </tr>`;
  }).join("");

  root.className = "";
  root.innerHTML = `
    <div class="flex-timing-metrics">
      <div class="flex-timing-metric"><span>Topology</span><strong>T${esc(config.tagId)} · N=${N}</strong></div>
      <div class="flex-timing-metric"><span>Slot period</span><strong>${fmtFixed(config.slotMs, 3)} ms</strong></div>
      <div class="flex-timing-metric"><span>Position frame</span><strong>${fmtFixed(frameMs, 3)} ms</strong></div>
      <div class="flex-timing-metric"><span>Nominal position rate</span><strong>${fmtFixed(frameHz, 2)} Hz</strong></div>
      <div class="flex-timing-metric"><span>Exchanges / frame</span><strong>${N}</strong></div>
      <div class="flex-timing-metric"><span>Scheduled range rate</span><strong>${fmtFixed(exchangeHz, 1)} /s</strong></div>
    </div>
    <div class="flex-timing-scroll">
      <div class="flex-timing-canvas">
        <div class="flex-timing-label">
          <strong>Base frame · ${N} tag-anchor exchanges + frame gap</strong>
          <span>${config.live && latest ? `latest frame ${esc(latest.frame_id ?? latest.seq)} from A${esc(latest.anchor_id)}` : "configured topology"} · fixed anchor order · no maintenance packets</span>
        </div>
        <div class="flex-frame-track" style="grid-template-columns:${frameColumns}">${frameSlots}${gapCell}</div>
        <div class="flex-frame-axis">${frameAxis}</div>
        <div class="flex-frame-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">Frame = ${N} × ${fmtFixed(config.slotMs, 0)} ms + ${fmtFixed(config.roundGapMs, 0)} ms = ${fmtFixed(frameMs, 0)} ms</span>
          </div>
        </div>
        <div class="flex-timing-label">
          <strong>Selected exchange · T${esc(config.tagId)} ↔ A${esc(selectedAnchorId)}${frameId === null ? "" : ` · frame ${esc(frameId)}`}</strong>
          <span>native four-packet DS-TWR with one-shot result</span>
        </div>
        <div class="flex-slot-track" style="grid-template-columns:${slotColumns}">${segmentCells}</div>
        <div class="flex-slot-axis">${slotAxis}</div>
        <div class="flex-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">DS-TWR slot budget = ${fmtFixed(config.slotMs, 3)} ms</span>
          </div>
        </div>
      </div>
    </div>
    <div class="flex-timing-detail-grid">
      <table class="flex-timing-table">
        <thead><tr><th>Interval</th><th>Start ms</th><th>End ms</th><th>Budget</th><th>Radio action</th></tr></thead>
        <tbody>${detailRows}</tbody>
      </table>
      <div class="flex-packet-flow">
        <div class="flex-packet-row">
          <b>POLL · T${esc(config.tagId)} → A${esc(selectedAnchorId)}</b>
          <code>native POLL frame · tag starts the exchange and enables RX</code>
        </div>
        <div class="flex-packet-row">
          <b>RESP · A${esc(selectedAnchorId)} → T${esc(config.tagId)} · +${fmtFixed(config.respDelayMs, 0)} ms</b>
          <code>native delayed TX target relative to POLL_RX</code>
        </div>
        <div class="flex-packet-row">
          <b>FINAL · T${esc(config.tagId)} → A${esc(selectedAnchorId)} · +${fmtFixed(config.finalDelayMs, 0)} ms</b>
          <code>carries POLL_TX, RESP_RX and programmed FINAL_TX timestamps; the anchor computes the range</code>
        </div>
        <div class="flex-packet-row">
          <b>RESULT · A${esc(selectedAnchorId)} → T${esc(config.tagId)} · +${fmtFixed(config.respDelayMs, 0)} ms</b>
          <code>one-shot raw distance; session and 32-bit frame must match before the tag accepts it</code>
        </div>
      </div>
    </div>
    ${nativeDsPipelineDiagnosticsHtml()}
    <div class="flex-parameter-map">
      <div class="flex-timing-label">
        <strong>Live runtime parameter map</strong>
        <span>Read from the active native DS-TWR runtime.</span>
      </div>
      <table class="flex-parameter-table">
        <thead><tr><th>Scope</th><th>Live value</th><th>Where it acts</th></tr></thead>
        <tbody>
          <tr>
            <td><span class="flex-scope fixed">DS-TWR radio</span></td>
            <td>RESP ${fmtFixed(config.respDelayMs, 0)} ms · FINAL ${fmtFixed(config.finalDelayMs, 0)} ms · RESULT ${fmtFixed(config.respDelayMs, 0)} ms · timeout ${fmtFixed(config.timeoutMs, 0)} ms · auto RX ${esc(config.autoRxDelayUus)} UUS</td>
            <td>Defines delayed-TX targets and the receive timeout inside each POLL → RESP → FINAL → RESULT exchange.</td>
          </tr>
          <tr>
            <td><span class="flex-scope host">DS-TWR scheduler</span></td>
            <td>${fmtFixed(config.slotMs, 0)} ms/slot · ${fmtFixed(config.roundGapMs, 0)} ms frame gap · RX slice ${fmtFixed(config.rxSliceMs, 0)} ms</td>
            <td>The tag schedules one exchange per anchor; anchors use the RX slice only as a host-side listening window.</td>
          </tr>
        </tbody>
      </table>
    </div>
    <div class="flex-timing-note ${timingOverrun ? "warn" : ""}">Colored widths show delayed-TX timing budgets, not packet airtime. PROPAGATION and packet airtime are much smaller than the millisecond scale shown here. A range is accepted once at the tag after RESULT; a position is emitted only after all ${N} anchors complete the same frame.${timingOverrun ? " Warning: RESP + FINAL + RESULT exceeds the configured slot." : ""}</div>`;
}

const passiveDsProfileDefaults = {
  fast: {
    prefix: "passiveDsFast",
    label: "Fast Star",
    schedule: 0,
    slotMs: 5,
    gapMs: 1,
    rxMs: 100,
    timeoutMs: 4,
    respUs: 1000,
    finalUs: 1000,
    autoRxUus: 500,
    freshSec: 0.2,
  },
  robust: {
    prefix: "passiveDsRobust",
    label: "Robust Rotating",
    schedule: 1,
    slotMs: 5,
    gapMs: 1,
    rxMs: 100,
    timeoutMs: 4,
    respUs: 1000,
    finalUs: 1000,
    autoRxUus: 500,
    freshSec: 0.2,
  },
  multi: {
    prefix: "passiveDsMulti",
    label: "Multipoint Full-DS",
    schedule: 2,
    slotMs: 5,
    gapMs: 1,
    rxMs: 100,
    timeoutMs: 4,
    respUs: 1500,
    finalUs: 1500,
    autoRxUus: 500,
    freshSec: 0.2,
    solveMode: 2,
  },
};
const passiveDsFastGeometryFrameInterval = 4;
const passiveDsMultipointResponseSpacingUs = 750;
const passiveDsDynamicGuardUs = 250;
const passiveDsMultipointDynamicDefaults = {
  ...passiveDsProfileDefaults.multi,
  slotMs: 10,
  timeoutMs: 5,
  respUs: 4500,
  finalUs: 2500,
  solveMode: 0,
};
const passiveDsSpeedPresets = {
  safe: {
    label: "16 ms Safe",
    slotMs: 5,
    gapMs: 1,
    timeoutMs: 4,
  },
  balanced: {
    label: "13 ms Balanced",
    slotMs: 4,
    gapMs: 1,
    timeoutMs: 4,
  },
  maximum: {
    label: "10 ms Maximum",
    slotMs: 3,
    gapMs: 1,
    timeoutMs: 3,
  },
};
const passiveDsProfileFieldSuffixes = {
  slotMs: "SlotMs",
  gapMs: "GapMs",
  rxMs: "RxMs",
  timeoutMs: "TimeoutMs",
  respUs: "RespUs",
  finalUs: "FinalUs",
  autoRxUus: "AutoRxUus",
  freshSec: "FreshSec",
};

function readPassiveDsProfile(key) {
  const profile = passiveDsProfileDefaults[key];
  if (!profile) return null;
  const values = {...profile};
  for (const [field, suffix] of Object.entries(passiveDsProfileFieldSuffixes)) {
    values[field] = Number(document.getElementById(`${profile.prefix}${suffix}`)?.value);
  }
  if (key === "multi") {
    values.solveMode = Number(
      document.getElementById("passiveDsMultiSolveMode")?.value ??
      profile.solveMode
    );
  }
  return values;
}

function passiveDsProfileIds() {
  return Object.values(passiveDsProfileDefaults).flatMap(profile =>
    Object.values(passiveDsProfileFieldSuffixes).map(
      suffix => `${profile.prefix}${suffix}`
    )
  );
}

function writePassiveDsProfile(key, values = passiveDsProfileDefaults[key]) {
  const profile = passiveDsProfileDefaults[key];
  if (!profile || !values) return;
  for (const [field, suffix] of Object.entries(passiveDsProfileFieldSuffixes)) {
    const el = document.getElementById(`${profile.prefix}${suffix}`);
    if (el) el.value = String(values[field]);
  }
  if (key === "multi") {
    const solveMode = document.getElementById("passiveDsMultiSolveMode");
    if (solveMode) solveMode.value = String(values.solveMode ?? 2);
  }
  updatePassiveDsProfileSummary(key);
}

function passiveDsSpeedPresetElement(key) {
  const profile = passiveDsProfileDefaults[key];
  return profile
    ? document.getElementById(`${profile.prefix}SpeedPreset`)
    : null;
}

function matchingPassiveDsSpeedPreset(values) {
  return Object.entries(passiveDsSpeedPresets).find(([, preset]) =>
    Number(values.slotMs) === preset.slotMs &&
    Number(values.gapMs) === preset.gapMs &&
    Number(values.timeoutMs) === preset.timeoutMs
  )?.[0] || "custom";
}

function applyPassiveDsSpeedPreset(key, presetKey) {
  const preset = passiveDsSpeedPresets[presetKey];
  if (!preset) return;
  const values = readPassiveDsProfile(key);
  writePassiveDsProfile(key, {
    ...values,
    slotMs: preset.slotMs,
    gapMs: preset.gapMs,
    timeoutMs: preset.timeoutMs,
  });
  const selector = passiveDsSpeedPresetElement(key);
  if (selector) selector.value = presetKey;
}

function updatePassiveDsProfileSummary(key) {
  const values = readPassiveDsProfile(key);
  const profile = passiveDsProfileDefaults[key];
  const summary = profile
    ? document.getElementById(`${profile.prefix}Summary`)
    : null;
  if (!values || !summary) return;
  const numeric = Object.keys(passiveDsProfileFieldSuffixes)
    .every(field => Number.isFinite(values[field]));
  if (!numeric) {
    summary.textContent = "incomplete profile";
    summary.className = "profile-summary warn";
    return;
  }
  const anchorCount = Math.max(
    3, Number(document.getElementById("positionAnchorCount")?.value || 4)
  );
  const multipoint = key === "multi";
  const singleStar = multipoint && values.solveMode === 0;
  const guardMs = singleStar ? passiveDsDynamicGuardUs / 1000 : values.gapMs;
  const frameMs = multipoint
    ? values.slotMs + guardMs
    : (anchorCount - 1) * values.slotMs + values.gapMs;
  const frameHz = frameMs > 0 ? 1000 / frameMs : NaN;
  const superframeMs = key === "robust" || multipoint
    ? anchorCount * frameMs
    : (anchorCount - 1) * passiveDsFastGeometryFrameInterval * frameMs;
  const speedPresetKey = matchingPassiveDsSpeedPreset(values);
  const speedPreset = passiveDsSpeedPresets[speedPresetKey];
  const profileVariant = multipoint
    ? (singleStar
        ? `Single-Star Dynamic · ${passiveDsDynamicGuardUs} µs guard`
        : "Static Precision · three stars")
    : (speedPreset?.label || "Custom timing");
  const selector = passiveDsSpeedPresetElement(key);
  if (selector && selector.value !== speedPresetKey) {
    selector.value = speedPresetKey;
  }
  const warnings = [];
  const exchangeUs = multipoint
    ? values.respUs +
      passiveDsMultipointResponseSpacingUs * (anchorCount - 2) +
      values.finalUs
    : values.respUs + values.finalUs;
  if (exchangeUs >= values.slotMs * 1000) {
    warnings.push(multipoint
      ? "response train + FINAL >= exchange budget"
      : "RESP + FINAL >= slot");
  }
  if (values.timeoutMs > values.slotMs) warnings.push("timeout > slot");
  summary.textContent =
    `${profileSummaryScheduleLabel(key)} · ` +
    `${profileVariant} · POLL → RESP → FINAL · ` +
    `${fmtFixed(frameMs, multipoint ? 2 : 0)} ms radio star · ` +
    `${singleStar ? "one independent position/star" : "three-star precision windows"} · ` +
    `${fmtFixed(frameHz, 2)} radio stars/s · ` +
    `${fmtFixed(superframeMs, multipoint ? 2 : 0)} ms ${key === "fast" ? "full geometry maintenance cycle" : "rotating superframe"}` +
    (warnings.length ? ` · ${warnings.join(", ")}` : "");
  summary.className = `profile-summary ${warnings.length ? "warn" : ""}`.trim();
}

function profileSummaryScheduleLabel(key) {
  if (key === "multi") return "Multipoint Full-DS · N+2";
  return key === "robust" ? "Robust Rotating" : "Fast Star";
}

function renderPassiveDsActiveProfile() {
  const root = document.getElementById("passiveDsActiveProfile");
  if (!root) return;
  const freshStatuses = state.statuses.filter(statusIsFresh);
  const statuses = freshStatuses.filter(item =>
    item.runtime_mode_name === "uwb_passive_ds_twr"
  );
  if (!statuses.length) {
    root.textContent = "Passive DS-TWR is not active on any fresh module.";
    root.className = "profile-validation warn";
    return;
  }
  const timing = item => ({
    schedule: Number(item.runtime_passive_ds_schedule),
    slotMs: Number(item.runtime_passive_ds_slot_ms),
    gapMs: Number(item.runtime_passive_ds_round_gap_ms),
    rxMs: Number(item.runtime_passive_ds_rx_slice_ms),
    timeoutMs: Number(item.runtime_passive_ds_rx_timeout_ms),
    respUs: Number(item.runtime_passive_ds_resp_delay_us),
    finalUs: Number(item.runtime_passive_ds_final_delay_us),
    autoRxUus: Number(item.runtime_passive_ds_auto_rx_delay_uus),
    solveMode: Number(item.runtime_passive_ds_solve_mode),
    anchorCount: Math.max(3, (item.runtime_anchor_ids || []).length),
  });
  const live = timing(statuses[0]);
  const fields = [
    "schedule", "slotMs", "gapMs", "rxMs", "timeoutMs",
    "respUs", "finalUs", "autoRxUus", "solveMode", "anchorCount",
  ];
  const consistent = statuses.every(item => {
    const candidate = timing(item);
    return fields.every(field => candidate[field] === live[field]);
  });
  if (!consistent) {
    root.textContent =
      `Passive DS-TWR timing differs between the ${statuses.length} active modules.`;
    root.className = "profile-validation warn";
    return;
  }
  const presetKey = matchingPassiveDsSpeedPreset(live);
  const scheduleLabel = live.schedule === 2
    ? "Multipoint Full-DS · N+2"
    : (live.schedule === 1 ? "Robust Rotating" : "Fast Star");
  const singleStar = live.schedule === 2 && live.solveMode === 0;
  const presetLabel = live.schedule === 2
    ? (singleStar
        ? `Single-Star Dynamic · ${passiveDsDynamicGuardUs} µs guard`
        : "Static Precision · three stars")
    : (passiveDsSpeedPresets[presetKey]?.label || "Custom timing");
  const liveGuardMs = singleStar
    ? passiveDsDynamicGuardUs / 1000
    : live.gapMs;
  const frameMs = live.schedule === 2
    ? live.slotMs + liveGuardMs
    : (live.anchorCount - 1) * live.slotMs + live.gapMs;
  const frameHz = frameMs > 0 ? 1000 / frameMs : 0;
  const activeTimingText = live.schedule === 2
    ? `${fmtFixed(frameMs, 2)} ms radio star · ` +
      (singleStar
        ? `one independent position/star · ${fmtFixed(frameHz, 2)} Hz`
        : `${fmtFixed(frameMs * 3, 2)} ms precision window · ` +
          `${fmtFixed(frameHz / 3, 2)} independent Hz`)
    : `${fmtFixed(frameMs, 0)} ms frame · ${fmtFixed(frameHz, 2)} Hz`;
  root.textContent =
    `Active on ${statuses.length}/${freshStatuses.length || statuses.length} modules: ` +
    `${scheduleLabel} · ${presetLabel} · ${activeTimingText} · ` +
    `RESP/FINAL ${live.respUs}+${live.finalUs} µs.`;
  root.className = "profile-validation good";
}

async function applyPassiveDsQuickProfile(key) {
  if (key === "multi_precision" || key === "multi_dynamic") {
    writePassiveDsProfile("multi", {
      ...(key === "multi_dynamic"
        ? passiveDsMultipointDynamicDefaults
        : passiveDsProfileDefaults.multi),
    });
    await applyPassiveDsProfile("multi");
    return;
  } else {
    applyPassiveDsSpeedPreset(key, "maximum");
  }
  await applyPassiveDsProfile(key);
}

function nativeDsCalibrationStatus() {
  const candidates = state.statuses || [];
  return candidates.find(item =>
    statusIsFresh(item) && item.runtime_mode_name === "uwb_ranging"
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
}

function renderNativeDsCalibration() {
  const root = document.getElementById("nativeDsCalibrationStatus");
  if (!root) return;
  const status = nativeDsCalibrationStatus();
  if (!statusIsFresh(status)) {
    root.textContent = "calibration status unavailable";
    root.className = "profile-summary warn";
    return;
  }
  const anchorIds = (status.runtime_anchor_ids || []).map(Number);
  const rangeBias = status.runtime_native_ds_range_bias_mm || [];
  if (!status.runtime_native_ds_calibration_enabled) {
    root.textContent = "disabled · solver receives uncorrected raw DS ranges";
    root.className = "profile-summary";
    return;
  }
  root.textContent =
    `enabled · generation ${status.runtime_native_ds_calibration_generation || 0} · ` +
    `[${anchorIds.map((id, index) => `A${id}:${rangeBias[index] || 0}`).join(", ")}] mm · ` +
    "subtracted before the independent-frame solver";
  root.className = "profile-summary good";
}

async function applyNativeDsCalibration(clear = false) {
  const params = clear
    ? {native_ds_calibration_clear: "1", reboot: "1"}
    : {
        native_ds_range_bias_mm:
          document.getElementById("nativeDsRangeBiasMm").value,
        reboot: "1",
      };
  setToast(
    "rangingProfileToast",
    clear ? "clearing Native DS-TWR calibration..." : "applying Native DS-TWR calibration...",
    "",
    null,
    false
  );
  const data = await postConfig({
    target_modules: document.getElementById("nativeDsCalibrationTargets").value,
    params,
  }, "rangingProfileToast");
  if (apiResponseOk(data)) setTimeout(fetchSnapshot, 500);
}

async function applyPassiveDsProfile(key) {
  const profile = passiveDsProfileDefaults[key];
  const values = readPassiveDsProfile(key);
  if (!profile || !values) return;
  const valid = Object.keys(passiveDsProfileFieldSuffixes)
    .every(field => Number.isFinite(values[field]) && values[field] >= 0);
  const anchorCount = Math.max(
    3, Number(document.getElementById("positionAnchorCount")?.value || 4)
  );
  const exchangeUs = key === "multi"
    ? values.respUs +
      passiveDsMultipointResponseSpacingUs * (anchorCount - 2) +
      values.finalUs
    : values.respUs + values.finalUs;
  if (!valid || values.slotMs < 1 || values.gapMs < 1 ||
      values.timeoutMs < 1 ||
      values.respUs < 100 || values.finalUs < 100 ||
      values.freshSec < 0.2 ||
      exchangeUs >= values.slotMs * 1000) {
    setToast(
      "passiveDsProfileToast",
      key === "multi"
        ? "Invalid timing: all RESP slots and FINAL guard must fit inside the exchange budget."
        : "Invalid timing: RESP + FINAL must fit strictly inside the slot.",
      "bad"
    );
    return;
  }
  setToast(
    "passiveDsProfileToast",
    `applying ${profile.label}...`,
    "",
    null,
    false
  );
  const data = await postConfig({
    target_modules: document.getElementById("passiveDsProfileTargets").value,
    params: {
      passive_ds_schedule: String(profile.schedule),
      passive_ds_slot_ms: String(values.slotMs),
      passive_ds_gap_ms: String(values.gapMs),
      passive_ds_rx_ms: String(values.rxMs),
      passive_ds_timeout_ms: String(values.timeoutMs),
      passive_ds_resp_delay_us: String(values.respUs),
      passive_ds_final_delay_us: String(values.finalUs),
      passive_ds_auto_rx_delay_uus: String(values.autoRxUus),
      passive_ds_solve_mode: String(
        key === "multi" ? values.solveMode : 0
      ),
      reboot: "1",
    },
  }, "passiveDsProfileToast");
  if (apiResponseOk(data)) {
    const maxAge = document.getElementById("positionMaxAgeSec");
    if (maxAge) maxAge.value = String(values.freshSec);
    localStorage.setItem(
      positionProtocolMaxAgeKey("passive_ds"),
      String(values.freshSec)
    );
    setTimeout(fetchSnapshot, 500);
  }
}

function passiveDsCalibrationStatus() {
  const candidates = state.statuses || [];
  return candidates.find(item =>
    statusIsFresh(item) &&
    String(item.runtime_mode_name || "").includes("passive_ds")
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
}

function renderPassiveDsCalibration() {
  const root = document.getElementById("passiveDsCalibrationStatus");
  if (!root) return;
  const status = passiveDsCalibrationStatus();
  const anchorIds = (status.runtime_anchor_ids || []).map(Number);
  const anchorBias = status.runtime_passive_ds_anchor_bias_mm || [];
  const rangeBias = status.runtime_passive_ds_range_bias_mm || [];
  const pairLabels = [];
  for (let a = 0; a < anchorIds.length; a += 1) {
    for (let b = a + 1; b < anchorIds.length; b += 1) {
      pairLabels.push(`A${anchorIds[a]}-A${anchorIds[b]}`);
    }
  }
  if (!statusIsFresh(status)) {
    root.textContent = "calibration status unavailable";
    root.className = "profile-summary warn";
    return;
  }
  if (!status.runtime_passive_ds_calibration_enabled) {
    root.textContent = "disabled · raw piggybacked DS ranges and passive observations";
    root.className = "profile-summary";
    return;
  }
  root.textContent =
    `enabled · generation ${status.runtime_passive_ds_calibration_generation || 0} · ` +
    `anchors [${anchorIds.map((id, index) => `A${id}:${anchorBias[index] || 0}`).join(", ")}] mm · ` +
    `ranges [${pairLabels.map((label, index) => `${label}:${rangeBias[index] || 0}`).join(", ")}] mm`;
  root.className = "profile-summary";
}

async function applyPassiveDsCalibration(clear = false) {
  const params = clear
    ? {passive_ds_calibration_clear: "1", reboot: "1"}
    : {
        passive_ds_anchor_bias_mm:
          document.getElementById("passiveDsAnchorBiasMm").value,
        passive_ds_range_bias_mm:
          document.getElementById("passiveDsRangeBiasMm").value,
        reboot: "1",
      };
  setToast(
    "passiveDsProfileToast",
    clear ? "clearing Passive DS-TWR calibration..." : "applying Passive DS-TWR calibration...",
    "",
    null,
    false
  );
  const data = await postConfig({
    target_modules: document.getElementById("passiveDsCalibrationTargets").value,
    params,
  }, "passiveDsProfileToast");
  if (apiResponseOk(data)) setTimeout(fetchSnapshot, 500);
}

function passiveDsRuntimeConfig() {
  const candidates = state.statuses || [];
  const status = candidates.find(item =>
    statusIsFresh(item) &&
    String(item.runtime_mode_name || "").includes("passive_ds")
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
  const anchorIds = (status.runtime_anchor_ids || [2, 3, 4, 5])
    .map(Number)
    .filter(Number.isFinite);
  return {
    anchorIds,
    schedule: Number(status.runtime_passive_ds_schedule || 0),
    slotMs: Number(status.runtime_passive_ds_slot_ms || 5),
    gapMs: Number(status.runtime_passive_ds_round_gap_ms ?? 1),
    rxMs: Number(status.runtime_passive_ds_rx_slice_ms || 100),
    timeoutMs: Number(status.runtime_passive_ds_rx_timeout_ms || 4),
    respUs: Number(status.runtime_passive_ds_resp_delay_us || 1000),
    finalUs: Number(status.runtime_passive_ds_final_delay_us || 1000),
    autoRxUus: Number(status.runtime_passive_ds_auto_rx_delay_uus || 500),
    pipelineMode: Number(status.runtime_passive_ds_pipeline_mode || 0),
    solveMode: Number(status.runtime_passive_ds_solve_mode || 0),
    rollingMaxHz: Number(status.runtime_passive_ds_rolling_max_hz || 100),
    live: Boolean(
      statusIsFresh(status) &&
      String(status.runtime_mode_name || "").includes("passive_ds")
    ),
  };
}

function passiveDsSolveModeLabel(mode, rollingMaxHz = 100) {
  if (mode === 1) {
    return `legacy mixed rolling ≤ ${rollingMaxHz} Hz`;
  }
  if (mode === 2) {
    return "three-star precision window · no temporal filter";
  }
  if (mode === 3) {
    return `coherent superframes · prediction ≤ ${rollingMaxHz} Hz`;
  }
  if (mode === 4) {
    return `motion-compensated rolling ≤ ${rollingMaxHz} Hz`;
  }
  return "single coherent star · no overlap/filter";
}

async function applyPassiveDsExperimentMode() {
  const rollingMaxHz = Number(
    document.getElementById("passiveDsRollingMaxHz")?.value
  );
  if (!Number.isInteger(rollingMaxHz) ||
      rollingMaxHz < 1 || rollingMaxHz > 500) {
    setToast(
      "passiveDsExperimentToast",
      "Rolling cap must be an integer between 1 and 500 Hz.",
      "bad"
    );
    return;
  }
  setToast(
    "passiveDsExperimentToast",
    "applying pipeline and EKF policy...",
    "",
    null,
    false
  );
  const data = await postConfig({
    target_modules:
      document.getElementById("passiveDsExperimentTargets").value,
    params: {
      passive_ds_pipeline_mode:
        document.getElementById("passiveDsPipelineMode").value,
      passive_ds_solve_mode:
        document.getElementById("passiveDsSolveMode").value,
      passive_ds_rolling_max_hz: String(rollingMaxHz),
      reboot: "1",
    },
  }, "passiveDsExperimentToast");
  if (apiResponseOk(data)) {
    resetPositionTagTrails();
    state.positionStreamRxTimes = [];
    state.positionStreamIndependentTimes = [];
    state.positionStreamSuperframeTimes = [];
    state.positionStreamCorrectionTimes = [];
    state.positionStreamRenderTimes = [];
    setTimeout(fetchSnapshot, 500);
  }
}

function passiveDsStageMetric(stage) {
  if (!stage) return "—";
  return `${esc(stage.avg_us ?? 0)}/${esc(stage.max_us ?? 0)} µs` +
    ` · ${esc(stage.fail ?? 0)}/${esc(stage.count ?? 0)} fail`;
}

function renderPassiveDsExperimentControls() {
  const candidates = state.statuses || [];
  const status = candidates.find(item =>
    statusIsFresh(item) &&
    String(item.runtime_mode_name || "").includes("passive_ds")
  ) || candidates.find(statusIsFresh) || candidates[0] || {};
  const config = {
    pipelineMode: Number(status.runtime_passive_ds_pipeline_mode || 0),
    solveMode: Number(status.runtime_passive_ds_solve_mode || 0),
    rollingMaxHz: Number(status.runtime_passive_ds_rolling_max_hz || 100),
  };
  [
    ["passiveDsPipelineMode", config.pipelineMode],
    ["passiveDsSolveMode", config.solveMode],
    ["passiveDsRollingMaxHz", config.rollingMaxHz],
  ].forEach(([id, value]) => {
    const el = document.getElementById(id);
    if (el && document.activeElement !== el) el.value = String(value);
  });

  const root = document.getElementById("passiveDsPipelineDiagnostics");
  if (!root) return;
  const rows = candidates
    .filter(statusIsFresh)
    .map(item => {
      const stats = item.passive_ds_pipeline_stats;
      if (!stats || !stats.stages) return "";
      const stages = stats.stages;
      return `<tr>
        <td>M${esc(item.module_id)}</td>
        <td>${stats.deadline_active ? "deadline" : "legacy"}</td>
        <td>${esc(stats.completed ?? 0)}</td>
        <td>${esc(stats.response_timeouts ?? 0)}/${esc(stats.final_timeouts ?? 0)}</td>
        <td>${esc(stats.invalid_frames ?? 0)}/${esc(stats.state_collisions ?? 0)}/${esc(stats.schedule_overruns ?? 0)}</td>
        <td>${passiveDsStageMetric(stages.poll_tx)}</td>
        <td>${passiveDsStageMetric(stages.response_tx)}</td>
        <td>${passiveDsStageMetric(stages.final_tx)}</td>
        <td>${passiveDsStageMetric(stages.final_rx)}</td>
        <td>${passiveDsStageMetric(stages.cia_read)}</td>
        <td>${passiveDsStageMetric(stages.rx_rearm)}</td>
      </tr>`;
    })
    .filter(Boolean)
    .join("");
  if (!rows) {
    root.className = "muted";
    root.textContent = "Waiting for instrumented firmware status...";
    return;
  }
  root.className = "table-wrap";
  root.innerHTML = `<table>
    <thead><tr>
      <th>Module</th><th>Pipeline</th><th>Complete</th>
      <th>RESP/FINAL timeout</th><th>invalid/collision/overrun</th>
      <th>POLL TX avg/max</th><th>RESP TX avg/max</th>
      <th>FINAL TX avg/max</th><th>FINAL RX avg/max</th>
      <th>CIA avg/max</th><th>RX re-arm avg/max</th>
    </tr></thead><tbody>${rows}</tbody>
  </table>`;
}

function renderPassiveDsMultipointTiming(root, config) {
  const N = config.anchorIds.length;
  const responderCount = N - 1;
  const firstResponseDelayMs = config.respUs / 1000;
  const responseSpacingMs = passiveDsMultipointResponseSpacingUs / 1000;
  const finalDelayMs = config.finalUs / 1000;
  const radioMs = firstResponseDelayMs +
    Math.max(0, responderCount - 1) * responseSpacingMs + finalDelayMs;
  const schedulerSlackMs = Math.max(0, config.slotMs - radioMs);
  const singleStar = config.solveMode === 0;
  const frameGuardMs = singleStar
    ? passiveDsDynamicGuardUs / 1000
    : config.gapMs;
  const radioStarMs = config.slotMs + frameGuardMs;
  const radioStarHz = radioStarMs > 0 ? 1000 / radioStarMs : NaN;
  const starsPerPosition = singleStar ? 1 : 3;
  const independentWindowMs = radioStarMs * starsPerPosition;
  const independentHz = radioStarHz / starsPerPosition;
  const nominalRawSolveHz = singleStar
    ? radioStarHz
    : independentHz + 2 * radioStarHz / 6;
  const segments = [
    ...Array.from({length: responderCount}, (_, index) => ({
      key: index === 0 ? "POLL → RESP[0]" : `RESP[${index - 1}] → RESP[${index}]`,
      duration: index === 0 ? firstResponseDelayMs : responseSpacingMs,
      cls: index % 2 ? "response" : "req",
    })),
    {key: `RESP[${responderCount - 1}] → FINAL`, duration: finalDelayMs, cls: "response"},
    {key: "SCHEDULER SLACK", duration: schedulerSlackMs, cls: "guard"},
    {key: "GUARD TIME", duration: frameGuardMs, cls: "guard"},
  ].filter(segment => segment.duration > 0);
  const columns = segments.map(segment =>
    `${Math.max(0.001, segment.duration)}fr`).join(" ");
  const cells = segments.map(segment => `
    <div class="flex-slot-segment ${segment.cls}">
      <b>${esc(segment.key)}</b><span>${fmtFixed(segment.duration, 3)} ms</span>
    </div>`).join("");
  let elapsedMs = 0;
  const boundaries = [0];
  for (const segment of segments) {
    elapsedMs += segment.duration;
    boundaries.push(elapsedMs);
  }
  const axis = boundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / radioStarMs,
      `${fmtFixed(value, 3)} ms`,
      index === 0 ? "edge-start" :
        (index === boundaries.length - 1 ? "edge-end" : "")
    )
  ).join("");
  const overrun = radioMs >= config.slotMs;
  root.className = "";
  root.innerHTML = `
    <div class="flex-timing-metrics">
      <div class="flex-timing-metric"><span>Protocol</span><strong>Multipoint Full-DS · N+2</strong></div>
      <div class="flex-timing-metric"><span>Reference</span><strong>rotates every radio star</strong></div>
      <div class="flex-timing-metric"><span>Position policy</span><strong>${singleStar ? "single coherent star" : "three-star precision"}</strong></div>
      <div class="flex-timing-metric"><span>Radio star</span><strong>${fmtFixed(radioStarMs, 2)} ms · ${fmtFixed(radioStarHz, 2)} Hz</strong></div>
      <div class="flex-timing-metric"><span>Independent position</span><strong>${fmtFixed(independentWindowMs, 2)} ms · ${fmtFixed(independentHz, 2)} Hz</strong></div>
      <div class="flex-timing-metric"><span>Raw solver output</span><strong>up to ${fmtFixed(nominalRawSolveHz, 2)} /s*</strong></div>
      <div class="flex-timing-metric"><span>UWB packets</span><strong>${N + 1} per radio star</strong></div>
      <div class="flex-timing-metric"><span>Tag solver</span><strong>raw GLS/AlgMin on ESP32</strong></div>
      <div class="flex-timing-metric"><span>Tag airtime</span><strong>0 packets</strong></div>
    </div>
    <div class="flex-timing-scroll">
      <div class="flex-timing-canvas">
        <div class="flex-timing-label">
          <strong>One coherent multipoint radio star</strong>
          <span>1 broadcast POLL + ${responderCount} staggered RESP + 1 aggregate broadcast FINAL</span>
        </div>
        <div class="flex-slot-track" style="grid-template-columns:${columns}">${cells}</div>
        <div class="flex-slot-axis">${axis}</div>
        <div class="flex-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">Exchange budget ${fmtFixed(config.slotMs, 3)} ms + guard time ${fmtFixed(frameGuardMs, 3)} ms = ${fmtFixed(radioStarMs, 3)} ms</span>
          </div>
        </div>
      </div>
    </div>
    <div class="flex-packet-flow" style="margin-top:12px">
      <div class="flex-packet-row"><b>POLL · rotating reference → broadcast</b><code>frame32 | previous full-DS range</code></div>
      <div class="flex-packet-row"><b>RESP[0..${responderCount - 1}] · delayed native TX</b><code>frame32 | responder index | exact reply_dtu32 | previous full-DS range</code></div>
      <div class="flex-packet-row"><b>FINAL · reference → broadcast</b><code>poll_tx40 | final_tx40 | responder ID + resp_rx40 for each response</code></div>
      <div class="flex-packet-row"><b>Receive-only tags</b><code>${responderCount} full three-clock double-sided observations; no CFO, no absolute clock synchronization, no filter</code></div>
    </div>
    <div class="flex-timing-note ${overrun ? "warn" : ""}">
      ${config.live ? "Live configuration" : "Configured fallback"} ·
      first RESP ${fmtFixed(firstResponseDelayMs, 3)} ms · response spacing ${fmtFixed(responseSpacingMs, 3)} ms ·
      FINAL delay ${fmtFixed(finalDelayMs, 3)} ms · scheduler slack ${fmtFixed(schedulerSlackMs, 3)} ms ·
      guard time ${fmtFixed(frameGuardMs, 3)} ms.
      ${singleStar
        ? "Every star is solved exactly once and reported as an independent position; there are no overlapping windows and no temporal filter."
        : "Independent results use non-overlapping groups of three stars. Two overlapping three-star windows per six-star cycle are explicitly marked non-independent; no temporal filter is used."}
      ${overrun ? " Warning: response train and FINAL delay exceed the exchange budget." : ""}
    </div>`;
}

function renderPassiveDsTimingDiagram() {
  const root = document.getElementById("passiveDsTimingDiagram");
  if (!root || rangingSettingsSolver() !== "passive_ds") return;
  const config = passiveDsRuntimeConfig();
  const N = config.anchorIds.length;
  if (N < 3) {
    root.textContent = "Passive DS-TWR topology is unavailable.";
    return;
  }
  if (config.schedule === 2) {
    renderPassiveDsMultipointTiming(root, config);
    return;
  }
  const slots = N - 1;
  const frameMs = slots * config.slotMs + config.gapMs;
  const frameHz = frameMs > 0 ? 1000 / frameMs : NaN;
  const robust = config.schedule === 1;
  const initiator = robust
    ? "rotates after each frame"
    : `A${config.anchorIds[0]} · maintenance every 4th`;
  const slotCards = config.anchorIds.slice(1).map((anchorId, index) => `
    <div class="flex-frame-slot">
      <b>slot[${index}] · ${fmtFixed(index * config.slotMs, 0)} ms</b>
      <strong>${robust ? "current reference" : `A${esc(config.anchorIds[0])}`} ↔ A${esc(anchorId)}</strong>
      <span>POLL → RESP → FINAL</span>
    </div>`).join("");
  const gapCell = config.gapMs > 0
    ? `<div class="ds-frame-gap"><b>frame gap</b><strong>${fmtFixed(config.gapMs, 0)} ms</strong><span>quiet scheduler gap</span></div>`
    : "";
  const frameColumns = [
    ...Array.from({length: slots}, () =>
      `${Math.max(0.001, config.slotMs)}fr`),
    ...(config.gapMs > 0
      ? [`${Math.max(0.001, config.gapMs)}fr`]
      : []),
  ].join(" ");
  const frameBoundaries = [0];
  for (let index = 1; index <= slots; index += 1) {
    frameBoundaries.push(index * config.slotMs);
  }
  if (config.gapMs > 0) frameBoundaries.push(frameMs);
  const frameAxis = frameBoundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / frameMs,
      `${fmtFixed(value, 0)} ms`,
      index === 0 ? "edge-start" :
        (index === frameBoundaries.length - 1 ? "edge-end" : "")
    )
  ).join("");

  const respMs = config.respUs / 1000;
  const finalMs = config.finalUs / 1000;
  const guardMs = Math.max(0, config.slotMs - respMs - finalMs);
  const timingOverrun = respMs + finalMs > config.slotMs;
  const segments = [
    {key: "POLL → RESP", duration: respMs, cls: "req"},
    {key: "RESP → FINAL", duration: finalMs, cls: "response"},
    {key: "GUARD TIME", duration: guardMs, cls: "guard"},
  ].filter(segment => segment.duration > 0);
  const segmentCells = segments.map(segment =>
    `<div class="flex-slot-segment ${segment.cls}">
      <b>${esc(segment.key)}</b><span>${fmtFixed(segment.duration, 3)} ms</span>
    </div>`
  ).join("");
  const slotColumns = segments
    .map(segment => `${Math.max(0.001, segment.duration)}fr`)
    .join(" ");
  let elapsedMs = 0;
  const slotBoundaries = [0];
  for (const segment of segments) {
    elapsedMs += segment.duration;
    slotBoundaries.push(elapsedMs);
  }
  const slotAxis = slotBoundaries.map((value, index) =>
    flexTimingAxisMark(
      100 * value / config.slotMs,
      `${fmtFixed(value, 3)} ms`,
      index === 0 ? "edge-start" :
        (index === slotBoundaries.length - 1 ? "edge-end" : "")
    )
  ).join("");
  root.className = "";
  root.innerHTML = `
    <div class="flex-timing-metrics">
      <div class="flex-timing-metric"><span>Schedule</span><strong>${robust ? "Robust Rotating" : "Fast Star"}</strong></div>
      <div class="flex-timing-metric"><span>Reference</span><strong>${esc(initiator)}</strong></div>
      <div class="flex-timing-metric"><span>Position frame</span><strong>${fmtFixed(frameMs, 0)} ms</strong></div>
      <div class="flex-timing-metric"><span>Nominal FPS</span><strong>${fmtFixed(frameHz, 2)}</strong></div>
      <div class="flex-timing-metric"><span>Anchor pipeline</span><strong>${config.pipelineMode === 1 ? "deadline" : "legacy control"}</strong></div>
      <div class="flex-timing-metric"><span>Position / EKF</span><strong>${esc(passiveDsSolveModeLabel(config.solveMode, config.rollingMaxHz))}</strong></div>
      <div class="flex-timing-metric"><span>Tag airtime</span><strong>0 packets</strong></div>
    </div>
    <div class="flex-timing-scroll">
      <div class="flex-timing-canvas">
        <div class="flex-timing-label">
          <strong>Position frame · ${slots} anchor-pair slots + frame gap</strong>
          <span>${robust ? "reference rotates after every frame" : "three star frames + one maintenance frame"}</span>
        </div>
        <div class="flex-frame-track" style="grid-template-columns:${frameColumns}">${slotCards}${gapCell}</div>
        <div class="flex-frame-axis">${frameAxis}</div>
        <div class="flex-frame-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">Frame = ${slots} × ${fmtFixed(config.slotMs, 0)} ms + ${fmtFixed(config.gapMs, 0)} ms = ${fmtFixed(frameMs, 0)} ms</span>
          </div>
        </div>
        <div class="flex-timing-label">
          <strong>One anchor-pair exchange</strong>
          <span>native three-packet DS-TWR</span>
        </div>
        <div class="flex-slot-track" style="grid-template-columns:${slotColumns}">${segmentCells}</div>
        <div class="flex-slot-axis">${slotAxis}</div>
        <div class="flex-dimensions">
          <div class="flex-dimension" style="left:0%;width:100%;top:0">
            <div class="flex-dimension-line"></div>
            <span class="flex-dimension-label">DS-TWR slot budget = ${fmtFixed(config.slotMs, 3)} ms</span>
          </div>
        </div>
      </div>
    </div>
    <div class="flex-packet-flow" style="margin-top:12px">
      <div class="flex-packet-row"><b>POLL · reference → responder</b><code>header | slot32 | previous DS-TWR range</code></div>
      <div class="flex-packet-row"><b>RESP · responder → reference · +${esc(config.respUs)} µs</b><code>header | slot32 | exact_reply_dtu32 | previous DS-TWR range</code></div>
      <div class="flex-packet-row"><b>FINAL · reference → responder · +${esc(config.finalUs)} µs</b><code>header | poll_tx40 | resp_rx40 | final_tx40 | slot32</code></div>
      <div class="flex-packet-row"><b>Receive-only tags</b><code>timestamp POLL_RX, RESP_RX and FINAL_RX, then use the three-clock double-sided intervals; no CFO estimate or absolute clock synchronization</code></div>
    </div>
    <div class="flex-timing-note ${config.live ? "" : "warn"}">
      ${config.live ? "Live configuration" : "Configured fallback (Passive DS-TWR is not active)"} ·
      slot ${esc(config.slotMs)} ms · gap ${esc(config.gapMs)} ms ·
      timeout ${esc(config.timeoutMs)} ms · RX slice ${esc(config.rxMs)} ms ·
      auto RX ${esc(config.autoRxUus)} UUS.
      ${robust
        ? `Robust Rotating spans ${fmtFixed(N * frameMs, 0)} ms before every anchor has served as reference.`
        : `Fast Star keeps its reference for three frames and rotates the fourth; a full maintenance cycle spans ${fmtFixed((N - 1) * passiveDsFastGeometryFrameInterval * frameMs, 0)} ms.`}
      ${timingOverrun ? " Warning: RESP + FINAL exceeds the configured slot." : ""}
    </div>`;
}

function profileSummaryText(values, solver, anchorCount = 4) {
  if (solver !== "ranging") return "";
  const dsCycleMs = anchorCount * values.slotMs + values.roundGapMs;
  const frameHz = dsCycleMs > 0 ? 1000 / dsCycleMs : NaN;
  const warnings = [];
  if (values.timeoutMs > values.slotMs) warnings.push("timeout > slot");
  if (2 * values.respDelayMs + values.finalDelayMs >= values.slotMs) {
    warnings.push("RESP + FINAL + RESULT >= slot");
  }
  const warnText = warnings.length ? ` · ${warnings.join(", ")}` : "";
  return `fresh ${fmtFixed(values.dsPositionMaxAgeSec, 1)} s · POLL → delayed RESP → delayed FINAL → delayed RESULT · ${fmtFixed(values.slotMs, 0)} ms slot · ${fmtFixed(values.roundGapMs, 0)} ms gap · ${fmtFixed(dsCycleMs, 0)} ms frame · ${fmtFixed(frameHz, 2)} Hz${warnText}`;
}

function updateRangingProfileSummary(profileKey) {
  const profile = rangingProfileDefaults[profileKey];
  if (!profile) return;
  const summary = document.getElementById(`${profile.prefix}Summary`);
  if (!summary) return;
  const solver = rangingSettingsSolver();
  const values = readRangingProfile(profileKey);
  const visibleFields = rangingProtocolProfileFields[solver];
  const valid = [...visibleFields].every(key => Number.isFinite(values[key]));
  summary.textContent = valid
    ? `${positionSolverLabel(solver)} · ${profileSummaryText(values, solver, 4)}`
    : "incomplete profile";
  const warn = solver === "ranging" && (
    values.timeoutMs > values.slotMs ||
    2 * values.respDelayMs + values.finalDelayMs >= values.slotMs
  );
  summary.className = `profile-summary ${valid && warn ? "warn" : ""}`.trim();
}

function updateAllRangingProfileSummaries() {
  Object.keys(rangingProfileDefaults).forEach(updateRangingProfileSummary);
}

async function applyRangingProfile(profileKey) {
  const profile = rangingProfileDefaults[profileKey];
  if (!profile) return;
  const solver = rangingSettingsSolver();
  if (solver !== "ranging") return;
  const values = readRangingProfile(profileKey);
  const visibleFields = rangingProtocolProfileFields[solver];
  if (![...visibleFields].every(key => Number.isFinite(values[key]) && values[key] > 0)) {
    setToast("rangingProfileToast", "Profile has invalid values", "bad");
    return;
  }
  if (2 * values.respDelayMs + values.finalDelayMs >= values.slotMs) {
    setToast(
      "rangingProfileToast",
      "Invalid timing: RESP + FINAL + RESULT must fit strictly inside the slot.",
      "bad"
    );
    return;
  }
  setToast("rangingProfileToast", `applying ${profile.label} to ${positionSolverLabel(solver)}...`, "", null, false);
  const data = await postConfig({
    target_modules: document.getElementById("rangingProfileTargets").value,
    params: rangingProfileRuntimeParams(values, solver),
  }, "rangingProfileToast");
  if (apiResponseOk(data)) {
    mirrorRangingProfileToUwbFields(values, solver);
    applyRangingProfilePositionSettings(values, solver);
    setTimeout(fetchSnapshot, 500);
  }
}

function persistedSettingIds() {
  return [
    "runtimeTargets", "runtimeMode", "runtimeTag", "runtimeAnchors", "runtimeReboot",
    "runtimeUwb", "runtimeBno085", "runtimeGps", "runtimeTelemetryPort",
    "accelTimebase", "accelSampleHz", "accelTargets",
    "positionAnchorCount", "positionSolver", "positionAnchors", "positionTags",
    "positionReferenceMode", "positionReferenceX",
    "positionReferenceY", "positionErrorWindowSec",
    "uwbTargets", "uwbRadioChannel", "uwbRadioPhyMode", "uwbFlexAnchors", "uwbFlexK",
    "uwbFlexSlots", "uwbFlexMasks", "uwbSurveyRxMs", "uwbSurveyDelayMs", "uwbSurveySlotMs",
    "uwbSurveyGapMs", "uwbSurveyLogEvery", "uwbRangingSlotMs",
    "uwbRangingGapMs", "uwbRangingRxMs", "uwbRangingTimeoutMs",
    "uwbRangingRespDelayMs", "uwbRangingFinalDelayMs",
    "uwbRangingAutoRxDelayUus", "uwbDtInitiator", "uwbDtResponder",
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
    "passiveDsProfileTargets",
    "nativeDsCalibrationTargets", "nativeDsRangeBiasMm",
    "passiveDsCalibrationTargets",
    "passiveDsAnchorBiasMm", "passiveDsRangeBiasMm",
    ...passiveDsProfileIds(),
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
  migrateRangingProfileDefaults();
  migrateFlexProfileDefaults();
  migratePassiveDsProfileDefaults();
  migrateCalibrationPairSetting();
  migratePositionSolverSetting();
}

function migrateRangingProfileDefaults() {
  const key = "uwbDash.rangingProfileDefaultsVersion";
  if (localStorage.getItem(key) === rangingProfileDefaultsVersion) return;
  for (const profile of Object.keys(rangingProfileDefaults)) {
    writeRangingProfile(profile, rangingProfileDefaults[profile]);
  }
  localStorage.setItem(key, rangingProfileDefaultsVersion);
}

function migrateFlexProfileDefaults() {
  const key = "uwbDash.flexProfileDefaultsVersion";
  if (localStorage.getItem(key) === flexProfileDefaultsVersion) return;
  for (const profile of Object.keys(flexProfileDefaults)) {
    writeFlexProfile(profile, flexProfileDefaults[profile]);
  }
  localStorage.setItem(key, flexProfileDefaultsVersion);
}

function migratePassiveDsProfileDefaults() {
  const key = "uwbDash.passiveDsProfileDefaultsVersion";
  if (localStorage.getItem(key) === passiveDsProfileDefaultsVersion) return;
  for (const profile of Object.keys(passiveDsProfileDefaults)) {
    writePassiveDsProfile(profile, passiveDsProfileDefaults[profile]);
  }
  localStorage.setItem(key, passiveDsProfileDefaultsVersion);
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
  if (localStorage.getItem(legacyCoordsKey) !== null) {
    localStorage.removeItem(legacyCoordsKey);
  }

  const solverKey = settingKey("positionSolver");
  const solverEl = document.getElementById("positionSolver");
  const savedSolver = localStorage.getItem(solverKey);
  if (savedSolver === null || savedSolver === "tdoa") {
    if (solverEl) solverEl.value = "flextdoa";
    localStorage.setItem(solverKey, "flextdoa");
  } else if (solverEl) {
    solverEl.value = normalizePositionSolver(savedSolver);
    localStorage.setItem(solverKey, solverEl.value);
  }

  const selectedSolver = normalizePositionSolver(
    solverEl?.value || "flextdoa"
  );
  const legacyMaxAgeKey = settingKey("positionMaxAgeSec");
  const legacyMaxAge = Number(localStorage.getItem(legacyMaxAgeKey));
  for (const [solver, defaults] of Object.entries(positionProtocolDefaults)) {
    const key = positionProtocolMaxAgeKey(solver);
    if (localStorage.getItem(key) !== null) continue;
    const initial = solver === selectedSolver &&
      Number.isFinite(legacyMaxAge) && legacyMaxAge >= 0.2
      ? legacyMaxAge
      : defaults.maxAgeSec;
    localStorage.setItem(key, String(initial));
  }
  localStorage.removeItem(legacyMaxAgeKey);
  switchPositionProtocolSettings();
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
  if (state.positionApplyInFlight) {
    setToast(
      "positionToast",
      "A UWB protocol transition is already in progress.",
      "warn"
    );
    return;
  }
  const settings = positionSettings();
  const tagId = settings.tagIds[0];
  if (settings.anchorIds.length !== settings.anchorCount) {
    setToast(
      "positionToast",
      `Set exactly ${settings.anchorCount} unique anchor IDs first.`,
      "bad"
    );
    return;
  }
  if (!tagId) {
    setToast("positionToast", "Set at least one tag ID first.", "bad");
    return;
  }
  const overlappingIds = settings.tagIds.filter(id => settings.anchorIds.includes(id));
  if (overlappingIds.length) {
    setToast(
      "positionToast",
      `Module${overlappingIds.length === 1 ? "" : "s"} ${overlappingIds.join(", ")} ` +
      `${overlappingIds.length === 1 ? "is" : "are"} selected as both anchor and tag.`,
      "bad"
    );
    return;
  }
  if (!positionProtocolUsesTdoa(settings.solver) && settings.tagIds.length > 1) {
    setToast(
      "positionToast",
      "Native DS-TWR supports one active tag. Select FlexTDOA or Passive DS-TWR for multiple receive-only tags.",
      "bad"
    );
    return;
  }
  const anchors = settings.anchorIds.join(",");
  const mode = positionRuntimeModeForSolver(settings.solver);
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
  state.positionApplyInFlight = true;
  [
    "positionAnchorCount", "positionSolver", "positionAnchors", "positionTags",
  ].forEach(id => {
    const element = document.getElementById(id);
    if (element) element.disabled = true;
  });
  renderPosition();
  try {
    const result = await postConfig(
      {target_modules: "all", params},
      "positionToast"
    );
    if (!apiResponseOk(result)) return;
    resetPositionTagTrails();
    state.positionAnchorTrail = {};
    state.positionSeeds = {};
    state.positionSetupDirty = false;
    await fetchSnapshot();
    setTimeout(fetchSnapshot, 250);
    setTimeout(fetchSnapshot, 750);
  } finally {
    state.positionApplyInFlight = false;
    [
      "positionAnchorCount", "positionSolver", "positionAnchors", "positionTags",
    ].forEach(id => {
      const element = document.getElementById(id);
      if (element) element.disabled = false;
    });
    renderPosition();
  }
}

function runPositionOverlayAction(event) {
  const action = event.currentTarget?.dataset?.action || "enable";
  if (action === "restart_geometry") {
    restartAnchorSelfLocalization();
  } else if (action === "enable") {
    enablePositionRanging();
  }
}

async function restartAnchorSelfLocalization() {
    const settings = positionSettings();
    if (!window.confirm(
      "Clear any fixed geometry from every module and reset the live " +
      "anchor estimate? Positioning resumes automatically after fresh ranges arrive."
    )) {
      return;
    }
    const result = await postConfig({
      target_modules: "all",
      params: {flex_geometry_clear: "1"},
    }, "positionToast");
    if (!apiResponseOk(result)) return;
    resetPaperAnchorSelfLocalization(
      settings.anchorIds, settings.solver, true);
    resetPositionTagTrails();
    setToast(
      "positionToast",
      "Fixed geometry cleared; live anchor tracking restarted",
      "good"
    );
    renderPosition();
}

function wireSettings() {
  clearLegacyChargerConfigSettings();
  restoreSettings();
  wireSettingPersistence();
  wireChargerDirtyTracking();
  wirePdDirtyTracking();
  updateChargerRawVisibility();
  updatePdRawVisibility();
  const flexTimingSlotSelect = document.getElementById("flexTimingSlotSelect");
  if (flexTimingSlotSelect) {
    flexTimingSlotSelect.addEventListener("change", () => {
      state.flexTimingSlotIndex = Number(flexTimingSlotSelect.value || 0);
      localStorage.setItem(
        settingKey("flexTimingSlotSelect"),
        String(state.flexTimingSlotIndex)
      );
      renderFlexTdoaTimingDiagram();
    });
  }
  const dsTimingSlotSelect = document.getElementById("dsTimingSlotSelect");
  if (dsTimingSlotSelect) {
    dsTimingSlotSelect.addEventListener("change", () => {
      state.dsTimingSlotIndex = Number(dsTimingSlotSelect.value || 0);
      localStorage.setItem(
        settingKey("dsTimingSlotSelect"),
        String(state.dsTimingSlotIndex)
      );
      renderNativeDsTwrTimingDiagram();
    });
  }
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
  [
    "positionAnchorCount", "positionSolver", "positionAnchors", "positionTags",
    "positionMaxAgeSec", "positionReferenceMode", "positionReferenceX",
    "positionReferenceY", "positionErrorWindowSec",
  ].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    const update = () => {
      if ([
        "positionAnchorCount", "positionSolver", "positionAnchors",
        "positionTags",
      ].includes(id)) {
        state.positionSetupDirty = true;
      }
      if (id === "positionSolver") switchPositionProtocolSettings();
      if (id === "positionMaxAgeSec") savePositionProtocolSettings();
      if (id === "positionAnchors" || id === "positionTags") {
        resetPositionTagTrails();
        state.positionSeeds = {};
        state.positionStreamRxTimes = [];
        state.positionStreamIndependentTimes = [];
        state.positionStreamRenderTimes = [];
      }
      renderPosition();
      if (id === "positionSolver") updateRangingSettingsProtocol();
    };
    el.addEventListener("input", update);
    el.addEventListener("change", update);
  });
  const updatePositionReferenceControls = () => {
    const manual = document.getElementById("positionReferenceMode")?.value === "manual";
    document.getElementById("positionReferenceX").disabled = !manual;
    document.getElementById("positionReferenceY").disabled = !manual;
  };
  document.getElementById("positionReferenceMode")?.addEventListener(
    "change", updatePositionReferenceControls);
  updatePositionReferenceControls();
  document.getElementById("positionResetTrail").addEventListener("click", () => {
    resetPositionTagTrails();
    state.positionAnchorTrail = {};
    renderPosition();
  });
  document.getElementById("positionRestartAnchorSelfLocalization").addEventListener("click", restartAnchorSelfLocalization);
  document.getElementById("positionEnableRanging").addEventListener("click", runPositionOverlayAction);
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
  document.querySelectorAll(".profile-card[data-profile] input").forEach(el => {
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
  document.getElementById("applyNativeDsCalibration")?.addEventListener(
    "click", () => applyNativeDsCalibration(false)
  );
  document.getElementById("clearNativeDsCalibration")?.addEventListener(
    "click", () => applyNativeDsCalibration(true)
  );
  document.querySelectorAll(".flex-profile-card input").forEach(el => {
    const update = () => {
      const profile = el.closest(".flex-profile-card")?.dataset.flexProfile;
      if (profile) updateFlexProfileSummary(profile);
    };
    el.addEventListener("input", update);
    el.addEventListener("change", update);
  });
  document.querySelectorAll(".apply-flex-profile").forEach(button => {
    button.addEventListener("click", () => applyFlexProfile(button.dataset.flexProfile));
  });
  document.querySelectorAll(".reset-flex-profile").forEach(button => {
    button.addEventListener("click", () => {
      const profile = button.dataset.flexProfile;
      writeFlexProfile(profile, flexProfileDefaults[profile]);
      setToast("flexProfileToast", "profile defaults restored locally; press Apply to write ESP NVS", "");
    });
  });
  document.getElementById("resetAllFlexProfiles")?.addEventListener("click", () => {
    for (const profile of Object.keys(flexProfileDefaults)) {
      writeFlexProfile(profile, flexProfileDefaults[profile]);
    }
    setToast("flexProfileToast", "all FlexTDOA defaults restored locally; press Apply to write ESP NVS", "");
  });
  document.querySelectorAll(".passive-ds-profile-card input").forEach(el => {
    const update = () => {
      const profile = el.closest(".passive-ds-profile-card")
        ?.dataset.passiveDsProfile;
      if (profile) updatePassiveDsProfileSummary(profile);
    };
    el.addEventListener("input", update);
    el.addEventListener("change", update);
  });
  document.getElementById("passiveDsMultiSolveMode")?.addEventListener(
    "change", () => updatePassiveDsProfileSummary("multi")
  );
  document.querySelectorAll(".passive-ds-speed-preset").forEach(el => {
    el.addEventListener("change", () => {
      const profile = el.closest(".passive-ds-profile-card")
        ?.dataset.passiveDsProfile;
      if (profile && el.value !== "custom") {
        applyPassiveDsSpeedPreset(profile, el.value);
      }
    });
  });
  document.querySelectorAll(".apply-passive-ds-profile").forEach(button => {
    button.addEventListener("click", () =>
      applyPassiveDsProfile(button.dataset.passiveDsProfile)
    );
  });
  document.querySelectorAll(".apply-passive-ds-quick-profile").forEach(button => {
    button.addEventListener("click", () =>
      applyPassiveDsQuickProfile(button.dataset.passiveDsProfile)
    );
  });
  document.querySelectorAll(".reset-passive-ds-profile").forEach(button => {
    button.addEventListener("click", () => {
      const profile = button.dataset.passiveDsProfile;
      writePassiveDsProfile(profile);
      setToast(
        "passiveDsProfileToast",
        "profile defaults restored locally; press Apply to write ESP NVS",
        ""
      );
    });
  });
  document.getElementById("applyPassiveDsCalibration")?.addEventListener(
    "click", () => applyPassiveDsCalibration(false)
  );
  document.getElementById("clearPassiveDsCalibration")?.addEventListener(
    "click", () => applyPassiveDsCalibration(true)
  );
  document.getElementById("applyPassiveDsExperimentMode")
    ?.addEventListener("click", applyPassiveDsExperimentMode);
  Object.keys(passiveDsProfileDefaults).forEach(updatePassiveDsProfileSummary);
  updateAllRangingProfileSummaries();
  updateAllFlexProfileSummaries();
  updateRangingSettingsProtocol();
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
    const enabled = document.getElementById("accelEnabled");
    if (enabled.indeterminate) {
      setToast("accelToast", "Choose enabled or disabled for the mixed target set", "bad");
      return;
    }
    postConfig({
      target_modules: document.getElementById("accelTargets").value,
      params: {
        bno085: enabled.checked ? "1" : "0",
        bno085_sample_hz: sampleHz,
      }
    }, "accelToast");
  });
  document.getElementById("accelTargets").addEventListener("change", updateAccelEnabledControl);
  document.getElementById("accelEnabled").addEventListener("change", event => {
    event.currentTarget.indeterminate = false;
    document.getElementById("accelEnabledLabel").textContent =
      `BNO085 ${event.currentTarget.checked ? "enabled" : "disabled"}`;
  });
  document.getElementById("applyUwbSettings").addEventListener("click", () => {
    postConfig({
      target_modules: document.getElementById("uwbTargets").value,
      params: {
        anchors: document.getElementById("uwbFlexAnchors").value,
        flex_k: document.getElementById("uwbFlexK").value,
        flex_slots: document.getElementById("uwbFlexSlots").value,
        flex_masks: document.getElementById("uwbFlexMasks").value,
        radio_channel: document.getElementById("uwbRadioChannel").value,
        radio_phy_mode: document.getElementById("uwbRadioPhyMode").value,
        survey_rx_ms: document.getElementById("uwbSurveyRxMs").value,
        survey_delay_ms: document.getElementById("uwbSurveyDelayMs").value,
        survey_slot_ms: document.getElementById("uwbSurveySlotMs").value,
        survey_gap_ms: document.getElementById("uwbSurveyGapMs").value,
        survey_log_every: document.getElementById("uwbSurveyLogEvery").value,
        ranging_slot_ms: document.getElementById("uwbRangingSlotMs").value,
        ranging_gap_ms: document.getElementById("uwbRangingGapMs").value,
        ranging_rx_ms: document.getElementById("uwbRangingRxMs").value,
        ranging_timeout_ms: document.getElementById("uwbRangingTimeoutMs").value,
        ranging_resp_delay_ms: document.getElementById("uwbRangingRespDelayMs").value,
        ranging_final_delay_ms: document.getElementById("uwbRangingFinalDelayMs").value,
        ranging_auto_rx_delay_uus: document.getElementById("uwbRangingAutoRxDelayUus").value,
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
wireGpsMapControls();
setActiveTab(state.activeTab);
renderRangingProfileCards();
wireSettings();
setCalibrationResult(loadCalibrationResult());
fetchLogs();
fetchAccel();
fetchSnapshot();
startPositionStream();
setInterval(fetchLogs, 250);
setInterval(fetchAccel, 50);
scheduleSnapshotPoll();
</script>
</body>
</html>
"""


class HttpHandler(BaseHTTPRequestHandler):
    server: "DashboardHttpServer"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        if not self.server.quiet:
            super().log_message(fmt, *args)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self.send_html(INDEX_HTML)
            return
        if parsed.path.startswith("/vendor/leaflet/"):
            self.send_leaflet_asset(parsed.path[len("/vendor/leaflet/") :])
            return
        if parsed.path == "/api/snapshot":
            self.send_json(self.server.state.snapshot())
            return
        if parsed.path == "/api/position-stream":
            self.handle_position_stream(parsed)
            return
        if parsed.path == "/api/position-events":
            query = urllib.parse.parse_qs(parsed.query)
            after = max(0, int(query.get("after", ["0"])[0] or "0"))
            limit = max(
                1, min(int(query.get("limit", ["512"])[0] or "512"), 4096)
            )
            events = self.server.state.position_events_after(
                after, limit, 0.0
            )
            self.send_json(
                {
                    "events": events,
                    "next_event_id": max(
                        [after]
                        + [
                            int(
                                item.get("position_stream_event_id")
                                or 0
                            )
                            for item in events
                        ]
                    ),
                }
            )
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

    def handle_position_stream(self, parsed: urllib.parse.ParseResult) -> None:
        query = urllib.parse.parse_qs(parsed.query)
        header_after = self.headers.get("Last-Event-ID", "")
        after_text = header_after or query.get("after", ["0"])[0] or "0"
        try:
            after = max(0, int(after_text))
        except ValueError:
            after = 0

        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            self.wfile.write(b"retry: 500\n\n")
            self.wfile.flush()

            while True:
                events = self.server.state.position_events_after(after, 512, 10.0)
                if not events:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                for item in events:
                    event_id = int(item.get("position_stream_event_id") or 0)
                    payload = json.dumps(item, separators=(",", ":"))
                    message = f"id: {event_id}\ndata: {payload}\n\n".encode("utf-8")
                    self.wfile.write(message)
                    # A restarted dashboard begins event IDs at 1, while the
                    # browser may reconnect with a Last-Event-ID from the old
                    # process. Adopt the current stream cursor so that case
                    # cannot replay the newest event in a tight loop.
                    after = event_id
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

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

    def send_leaflet_asset(self, relative_path: str) -> None:
        relative = pathlib.PurePosixPath(urllib.parse.unquote(relative_path))
        if relative.is_absolute() or ".." in relative.parts:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        candidate = (LEAFLET_ROOT / pathlib.Path(*relative.parts)).resolve()
        try:
            candidate.relative_to(LEAFLET_ROOT.resolve())
        except ValueError:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        content_types = {
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".png": "image/png",
            ".svg": "image/svg+xml",
        }
        raw = candidate.read_bytes()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                content_types.get(candidate.suffix.lower(), "application/octet-stream"),
            )
            self.send_header("Cache-Control", "public, max-age=86400")
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
        self.runtime_transition_lock = threading.Lock()
        self.calibration_job_lock = threading.Lock()
        self.calibration_job: dict[str, Any] | None = None

    def apply_runtime_config(
        self, params: dict[str, str], target_modules: Any = None
    ) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        if not params:
            raise RuntimeError("No runtime config parameters provided")
        # Older dashboard tabs requested an in-place DW3000 hot switch.  That
        # leaves the Passive DS-TWR frame state out of phase across modules and
        # can produce fresh anchor ranges without a tag position.  Treat every
        # dashboard hot-switch request as a coordinated parallel reboot.  The
        # direct module endpoint remains available for firmware diagnostics.
        if str(params.get("hot_switch") or "") == "1":
            params = dict(params)
            params.pop("hot_switch", None)
            params["reboot"] = "1"
        if not self.runtime_transition_lock.acquire(blocking=False):
            raise RuntimeError(
                "Another UWB runtime transition is still in progress"
            )
        try:
            targets = self.resolve_targets(target_modules)
            with ThreadPoolExecutor(max_workers=min(5, len(targets))) as executor:
                results = list(
                    executor.map(
                        lambda target: self.send_runtime_config(target, params),
                        targets,
                    )
                )
            if not all(item.get("ok") for item in results):
                return results

            # A successful /config/runtime response means that the request was
            # accepted, not that the coordinated reboot has completed.  Keep
            # the transition lock until every module reports the requested
            # runtime again.  The short delay prevents accepting a status from
            # the old process in the interval between its HTTP reply and reset.
            if str(params.get("reboot") or "") == "1":
                time.sleep(0.6)
                with ThreadPoolExecutor(max_workers=min(5, len(targets))) as executor:
                    verified = list(
                        executor.map(
                            lambda target: self.wait_for_runtime_config(
                                target, params
                            ),
                            targets,
                        )
                    )
                for result, status in zip(results, verified):
                    if status is None:
                        result["ok"] = False
                        result["error"] = (
                            "runtime was accepted but not confirmed after reboot"
                        )
                        continue
                    result["verified_after_reboot"] = True
                    result["module_id"] = status.get("module_id")
            return results
        finally:
            self.runtime_transition_lock.release()

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
            }

        if not module_ids:
            # "all" must always mean the complete configured module set.  A
            # freshly restarted dashboard may have polled only a subset of the
            # modules; deriving the fan-out from that transient subset can
            # leave the UWB network split across two protocols.
            targets = list(dict.fromkeys(self.targets))
            if not targets:
                targets = list(dict.fromkeys(by_module.values()))
            if not targets:
                raise RuntimeError("No configured UWB targets known")
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

    def wait_for_runtime_config(
        self, target: str, params: dict[str, str]
    ) -> dict[str, Any] | None:
        # A runtime write may reboot the module before the HTTP response is
        # transmitted. Allow enough time for ESP-IDF to boot and reconnect to
        # Wi-Fi, then verify the persisted settings through /status.
        deadline = time.monotonic() + max(30.0, self.timeout_sec * 3.0)
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(
                    status_url(target), timeout=2.0
                ) as response:
                    status = json.loads(
                        response.read().decode("utf-8", errors="replace")
                    )
                if runtime_config_matches_status(params, status):
                    switching = bool(status.get("uwb_runtime_switching"))
                    uwb_ready = str(status.get("uwb_status") or "").lower()
                    if not switching and uwb_ready in ("", "ready"):
                        return status
            except (
                json.JSONDecodeError,
                urllib.error.URLError,
                TimeoutError,
                ConnectionError,
                OSError,
            ):
                pass
            time.sleep(0.4)
        return None

    def send_runtime_config(self, target: str, params: dict[str, str]) -> dict[str, Any]:
        query = urllib.parse.urlencode(params, safe=",:")
        started = time.monotonic()
        reboot_requested = str(params.get("reboot") or "") == "1"
        attempts = 2 if reboot_requested else 1
        last_error = ""
        for attempt in range(1, attempts + 1):
            request = urllib.request.Request(
                f"{runtime_url(target)}?{query}",
                data=b"",
                method="POST",
                headers={
                    "Content-Length": "0",
                    "X-OTA-Token": self.token,
                },
            )
            try:
                with urllib.request.urlopen(
                    request, timeout=self.timeout_sec
                ) as response:
                    body = response.read().decode(
                        "utf-8", errors="replace"
                    ).strip()
                    return {
                        "target": target,
                        "ok": 200 <= response.status < 300,
                        "status": response.status,
                        "elapsed_sec": round(
                            time.monotonic() - started, 3
                        ),
                        "body": body,
                        "attempt": attempt,
                    }
            except urllib.error.HTTPError as exc:
                body = exc.read().decode(
                    "utf-8", errors="replace"
                ).strip()
                return {
                    "target": target,
                    "ok": False,
                    "status": exc.code,
                    "body": body or exc.reason,
                    "attempt": attempt,
                }
            except (
                urllib.error.URLError,
                TimeoutError,
                ConnectionError,
                OSError,
            ) as exc:
                last_error = str(exc)
                if reboot_requested:
                    verified = self.wait_for_runtime_config(
                        target, params
                    )
                    if verified is not None:
                        return {
                            "target": target,
                            "ok": True,
                            "status": 200,
                            "elapsed_sec": round(
                                time.monotonic() - started, 3
                            ),
                            "body": (
                                "configuration verified after reboot"
                            ),
                            "verified_after_reboot": True,
                            "module_id": verified.get("module_id"),
                            "attempt": attempt,
                        }
                if attempt < attempts:
                    continue
        return {
            "target": target,
            "ok": False,
            "error": last_error or "runtime config verification failed",
            "attempt": attempts,
        }

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
