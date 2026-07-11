#!/usr/bin/env python3
"""Local browser dashboard for UWB module logs, status, and runtime config."""

from __future__ import annotations

import argparse
import json
import pathlib
import queue
import re
import socket
import socketserver
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


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


def antenna_delay_url(target: str) -> str:
    if re.match(r"^https?://", target):
        return target.rstrip("/") + "/config/antenna-delay"
    return f"http://{target}/config/antenna-delay"


def cm_to_mm_text(value: Any) -> str:
    number = float(value)
    return str(int(round(number * 10.0)))


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


class DashboardState:
    def __init__(self, *, max_logs: int) -> None:
        self.lock = threading.Lock()
        self.max_logs = max_logs
        self.logs: deque[dict[str, Any]] = deque(maxlen=max_logs)
        self.accel_history: dict[int, deque[dict[str, Any]]] = {}
        self.max_accel_samples = 30000
        self.accel_samples: deque[dict[str, Any]] = deque(maxlen=120000)
        self.next_log_id = 1
        self.next_accel_id = 1
        self.client_count = 0
        self.telemetry_client_count = 0
        self.client_counts: dict[str, int] = {}
        self.telemetry_client_counts: dict[str, int] = {}
        self.listener_telemetry_ports: list[int] = []
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

    def add_telemetry(self, line: str, addr: tuple[str, int]) -> None:
        parsed = self.parse_telemetry(line)
        if parsed is None:
            return
        parsed["received_at"] = time.time()
        parsed["client"] = f"{addr[0]}:{addr[1]}"
        with self.lock:
            self.record_accel_sample_locked(parsed)

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
        match = TELEMETRY_RE.match(line.strip())
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
            statuses = list(self.status_by_module.values())
            errors = dict(self.status_errors)
            client_count = self.client_count
            telemetry_client_count = self.telemetry_client_count
            client_counts = dict(self.client_counts)
            telemetry_client_counts = dict(self.telemetry_client_counts)
            listener_telemetry_ports = list(self.listener_telemetry_ports)
            log_count = len(self.logs)
            next_log_id = self.next_log_id
        statuses.sort(key=lambda item: int(item.get("module_id") or 0))
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
            "accel_history": {},
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
            if TELEMETRY_RE.match(text.strip()) is not None:
                if client_kind is None:
                    client_kind = "telemetry"
                    self.server.telemetry_client_connected()
                self.server.state.add_telemetry(text, addr)
                return

            if client_kind is None:
                client_kind = "log"
                self.server.log_client_connected()
            self.server.state.add_log(text, addr)

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
                    newline = buffer.find(b"\n")
                    if newline < 0:
                        break
                    raw = buffer[:newline]
                    buffer = buffer[newline + 1 :]
                    text = raw.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        handle_line(text)
        finally:
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
                    newline = buffer.find(b"\n")
                    if newline < 0:
                        break
                    raw = buffer[:newline]
                    buffer = buffer[newline + 1 :]
                    text = raw.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        self.server.state.add_telemetry(text, addr)
        finally:
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
button.primary { background: var(--blue); border-color: var(--blue); color: #fff; }
button.danger { color: var(--red); }
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
.ok { color: var(--green); font-weight: 700; }
.warn { color: var(--orange); font-weight: 700; }
.bad { color: var(--red); font-weight: 700; }
.settings-grid { display: grid; grid-template-columns: minmax(340px, 520px) minmax(420px, 1fr); gap: 14px; align-items: start; }
.charger-grid { grid-template-columns: minmax(620px, 1.25fr) minmax(360px, 0.75fr); }
.section { border: 1px solid var(--line); padding: 12px; margin-bottom: 12px; background: #fff; }
.section h2 { margin: 0 0 11px; font-size: 15px; }
.form-grid { display: grid; grid-template-columns: 160px minmax(160px, 1fr); gap: 8px 10px; align-items: center; }
.form-grid label { color: var(--muted); font-size: 13px; }
.form-actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
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
.toast { margin-top: 10px; color: var(--muted); font-size: 13px; white-space: nowrap; }
.toast:empty { display: none; }
.toast:not(:empty) {
  display: inline-block;
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
@media (max-width: 940px) {
  .terminal-grid, .settings-grid, .charger-grid, .graphs-layout { grid-template-columns: 1fr; }
  .page { height: auto; }
  .terminal { height: 520px; }
  .chart-stack { grid-template-rows: none; }
  .accel-chart { height: 178px; }
  .latest-panel { max-height: none; position: static; }
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
    <button class="tab" data-tab="graphs">Graphs</button>
    <button class="tab" data-tab="info">Info</button>
    <button class="tab" data-tab="batteryCharger">Battery Charger</button>
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
          <thead><tr><th>Module</th><th>Wi-Fi</th><th>Runtime</th><th>Components</th><th>GPS</th><th>UWB</th><th>Antenna</th><th>Logs</th><th>Battery</th></tr></thead>
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
              <table>
                <thead>
                  <tr>
                    <th>Module</th><th>Power</th><th>ADC</th><th>Limits</th><th>Measurements</th><th>Last Write</th>
                  </tr>
                </thead>
                <tbody id="chargerRows"></tbody>
              </table>
            </div>
            <div class="section">
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
                <label for="chargerTargets">Targets</label>
                <select id="chargerTargets">
                  <option value="all">all modules</option>
                  <option value="1">module 1</option>
                  <option value="2">module 2</option>
                  <option value="3">module 3</option>
                  <option value="4">module 4</option>
                  <option value="5">module 5</option>
                </select>
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
              <div id="chargerToast" class="toast"></div>
            </div>
            <div class="section">
              <h2>Charge Limits</h2>
              <div class="form-grid">
                <label for="chargerChargeEnabled">Charging</label>
                <div class="checkbox-row"><input id="chargerChargeEnabled" type="checkbox"><span>enabled</span></div>
                <label for="chargerMinimalSystemMv">VSYSMIN mV</label>
                <input id="chargerMinimalSystemMv" type="number" min="2500" max="16000" step="250">
                <label for="chargerChargeVoltageMv">Charge voltage mV</label>
                <input id="chargerChargeVoltageMv" type="number" min="3000" max="18800" step="10">
                <label for="chargerChargeCurrentMa">Charge current mA</label>
                <input id="chargerChargeCurrentMa" type="number" min="50" max="5000" step="10">
                <label for="chargerInputVoltageMv">Input voltage mV</label>
                <input id="chargerInputVoltageMv" type="number" min="3600" max="22000" step="100">
	                <label for="chargerInputCurrentMa">Input current mA</label>
	                <input id="chargerInputCurrentMa" type="number" min="100" max="3300" step="10">
              </div>
              <div class="param-legend">
                <div><b>Charging</b><span>Controls EN_CHG. Disabling it stops battery charging but can leave the board powered from VBUS/SYS.</span></div>
                <div><b>VSYSMIN</b><span>Minimum SYS rail target when the battery is low; step 250 mV.</span></div>
                <div><b>Charge voltage</b><span>Battery final voltage limit; sensitive for Li-Po safety; step 10 mV.</span></div>
                <div><b>Charge current</b><span>Maximum battery charge current before thermal/input limits intervene; step 10 mA.</span></div>
	                <div><b>Input voltage</b><span>VINDPM threshold: reduce load if VBUS falls below this; step 100 mV.</span></div>
	                <div><b>Input current</b><span>IINDPM limit: maximum current drawn from the adapter/USB source; step 10 mA.</span></div>
	              </div>
	              <div class="form-actions">
	                <button class="primary" id="applyChargerLimits">Apply Limits</button>
	              </div>
            </div>
            <div class="section">
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
            </div>
          </div>
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
                <label for="uwbDtRxTimeoutMs">RX timeout ms</label>
                <input id="uwbDtRxTimeoutMs" value="250" type="number" min="1" step="1">
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
              <h2>Calibration Timing</h2>
              <div class="form-grid">
                <label for="uwbCalSummary">Summary every</label>
                <input id="uwbCalSummary" value="25" type="number" min="1" step="1">
                <label for="uwbCalMinMs">Min interval ms</label>
                <input id="uwbCalMinMs" value="800" type="number" min="1" step="1">
                <label for="uwbCalMaxMs">Max interval ms</label>
                <input id="uwbCalMaxMs" value="1600" type="number" min="1" step="1">
                <label for="uwbCalRxMs">RX slice ms</label>
                <input id="uwbCalRxMs" value="50" type="number" min="1" step="1">
              </div>
            </div>
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
                <label class="two-only" for="calRef">Reference</label>
                <input class="two-only" id="calRef" value="1" type="number" min="1" step="1" inputmode="numeric">
                <label class="two-only" for="calDut">DUT</label>
                <input class="two-only" id="calDut" value="2" type="number" min="1" step="1" inputmode="numeric">
                <label class="two-only" for="calKnownCm">Distance cm</label>
                <input class="two-only cm-input" id="calKnownCm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calThree">Modules</label>
                <input class="three-only" id="calThree" value="1,2,3">
                <label class="three-only" for="calD01Cm">Distance 0-1 cm</label>
                <input class="three-only cm-input" id="calD01Cm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calD02Cm">Distance 0-2 cm</label>
                <input class="three-only cm-input" id="calD02Cm" value="200.00" type="number" min="0" step="0.01" inputmode="decimal">
                <label class="three-only" for="calD12Cm">Distance 1-2 cm</label>
                <input class="three-only cm-input" id="calD12Cm" value="282.84" type="number" min="0" step="0.01" inputmode="decimal">
                <label for="calSamples">Samples</label>
                <input id="calSamples" value="40" type="number" min="1" step="1" inputmode="numeric">
              </div>
              <div class="form-actions">
                <button class="primary" id="applyCalibration">Calibrate Antenna Delay</button>
              </div>
              <div id="calToast" class="toast"></div>
            </div>
          </div>
          <div class="section">
            <h2>Calibration Geometry</h2>
            <div id="calDiagram" class="diagram"></div>
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
};
const accelLineRe = /\bBNO085 accel x=([-+]?\d+(?:\.\d+)?) y=([-+]?\d+(?:\.\d+)?) z=([-+]?\d+(?:\.\d+)?) m\/s\^2 accuracy=(\d+) reports=(\d+)/;
const maxAccelSamples = 30000;
const maxSeriesPoints = 1600;
const plot = {left: 52, right: 704, top: 14, bottom: 166, width: 652, height: 152};
const toastTimers = new Map();
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

function fmtMv(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${(number / 1000).toFixed(3)} V` : "-";
}

function fmtMa(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(0)} mA` : "-";
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
  return `
    <span class="${item.charger_read_ok ? "ok" : "bad"}">BQ25792</span>
    <span class="muted">PN ${esc(item.charger_part_number ?? "-")} rev ${esc(item.charger_device_revision ?? "-")}</span><br>
    <span class="${adcClass}">ADC ${item.charger_adc_enabled ? "on" : "off"}</span>
    <span class="${charge.className}">${charge.text}</span>
    <span class="muted">${esc(writeText)}</span><br>
    VBAT ${fmtMv(item.charger_vbat_mv)} · SOC ${fmtSoc(item)}<br>
    VSYS ${fmtMv(item.charger_vsys_mv)} · VBUS ${fmtMv(item.charger_vbus_mv)}<br>
    IBUS ${fmtMa(item.charger_ibus_ma)}<br>
    IBAT ${fmtMa(item.charger_ibat_ma)} · TDIE ${fmtMaybeNumber(item.charger_tdie_c, 1)} C<br>
    <span class="muted">${pinLine}</span><br>
    <span class="muted">REG48 ${esc(item.charger_part_info || "-")}
      · reads ${esc(item.charger_read_count ?? "-")}
      (${esc(item.charger_full_read_count ?? "-")}/${esc(item.charger_quick_read_count ?? "-")} full/quick)
      · ${esc(item.charger_last_read_duration_ms ?? "-")} ms
      · age ${fmtAgeMs(item.charger_last_update_age_ms)}</span>`;
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
    return `<tr>
      <td><b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span></td>
      <td><span class="${powerClass}">${item.charger_present ? "BQ25792 present" : "not found"}</span><br>
        Board PG ${fmtGpioLevel(item.charger_pg_gpio_level)} ${item.charger_pg_asserted ? "asserted" : ""}<br>
        BQ PG_STAT ${item.charger_pg_stat ? "1" : "0"} · INT ${fmtGpioLevel(item.charger_int_gpio_level)} / ${esc(item.charger_int_irq_count ?? "-")}<br>
        <span class="muted">REG48 ${esc(item.charger_part_info || "-")}
          · reads ${esc(item.charger_read_count ?? "-")}
          (${esc(item.charger_full_read_count ?? "-")}/${esc(item.charger_quick_read_count ?? "-")} full/quick)
          · ${esc(item.charger_last_read_duration_ms ?? "-")} ms
          · age ${fmtAgeMs(item.charger_last_update_age_ms)}</span></td>
      <td><span class="${adcClass}">ADC ${item.charger_adc_enabled ? "on" : "off"}</span><br>
        sample ${esc(item.charger_adc_sample ?? "-")} · ${item.charger_adc_continuous ? "continuous" : "one shot"}<br>
        avg ${item.charger_adc_running_average ? "on" : "off"} · EN_IBAT ${item.charger_ibat_discharge_sense_enabled ? "on" : "off"}<br>
        WD ${esc(item.charger_watchdog_setting ?? "-")} ${item.charger_watchdog_disabled ? "(disabled)" : ""}</td>
      <td><span class="${charge.className}">${charge.text}</span><br>
        VSYSMIN ${fmtMv(item.charger_minimal_system_voltage_mv)}<br>
        VREG ${fmtMv(item.charger_charge_voltage_limit_mv)}<br>
        ICHG ${fmtMa(item.charger_charge_current_limit_ma)}<br>
        VINDPM ${fmtMv(item.charger_input_voltage_limit_mv)}<br>
        IINDPM ${fmtMa(item.charger_input_current_limit_ma)}</td>
      <td>SOC ${fmtSoc(item)} · VBAT ${fmtMv(item.charger_vbat_mv)}<br>
        VBUS ${fmtMv(item.charger_vbus_mv)} · VSYS ${fmtMv(item.charger_vsys_mv)}<br>
        VAC1 ${fmtMv(item.charger_vac1_mv)}<br>
        IBUS ${fmtMa(item.charger_ibus_ma)} · IBAT ${fmtMa(item.charger_ibat_ma)}<br>
        TS ${fmtMaybeNumber(item.charger_ts_percent, 2)}% · TDIE ${fmtMaybeNumber(item.charger_tdie_c, 1)} C</td>
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

function renderInfo(snapshot) {
  state.statuses = snapshot.statuses || [];
  mergeAccelHistory(snapshot.accel_history || {});
  document.getElementById("logPill").textContent = `${snapshot.log_count} logs`;
  const telemetryPill = document.getElementById("telemetryPill");
  if (telemetryPill) {
    telemetryPill.textContent = `${snapshot.telemetry_client_count || 0} telemetry client${snapshot.telemetry_client_count === 1 ? "" : "s"}`;
  }
  const online = state.statuses.filter(item => item.wifi_connected).length;
  const statusPill = document.getElementById("statusPill");
  statusPill.textContent = `${online} modules online`;
  statusPill.className = `pill ${online >= 5 ? "good" : "warn"}`;
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
    <tr>
      <td><b>${esc(item.hostname)}</b><br><span class="muted">${esc(item.ip || item.target || "")}</span></td>
      <td><span class="${item.wifi_connected ? "ok" : "bad"}">${item.wifi_connected ? "connected" : "offline"}</span><br>RSSI ${esc(item.wifi_connected_rssi)} dBm<br>disc ${esc(item.wifi_disconnect_count)}</td>
      <td>${esc(item.runtime_mode_name)}<br>tag ${esc(item.runtime_tag_id)} anchors ${(item.runtime_anchor_ids || []).join(",")}</td>
      <td>UWB <span class="${item.runtime_uwb_enabled ? "ok" : "muted"}">${item.runtime_uwb_enabled ? "on" : "off"}</span><br>BNO085 <span class="${item.runtime_bno085_accel_enabled ? "ok" : "muted"}">${item.runtime_bno085_accel_enabled ? "on" : "off"}</span><br><span class="muted">${esc(item.runtime_bno085_accel_interval_ms || "-")}/${esc(item.runtime_bno085_log_interval_ms || "-")} ms</span><br>GPS <span class="${item.runtime_gps_enabled ? "ok" : "muted"}">${item.runtime_gps_enabled ? "on" : "off"}</span></td>
      <td>${renderGpsCell(item)}</td>
      <td>${esc(item.uwb_status)}<br>tx ${esc(item.uwb_tx_count)} / rx ${esc(item.uwb_rx_count)}<br>err ${esc(item.uwb_tx_error_count)}/${esc(item.uwb_rx_error_count)}</td>
      <td>${esc(item.uwb_active_antenna_delay_hex)}<br><span class="muted">NVS ${item.uwb_antenna_delay_from_nvs ? "yes" : "no"}</span></td>
      <td>log ${esc(item.wireless_log_status)}<br>dropped ${esc(item.wireless_log_dropped)}<br>tel ${esc(item.wireless_telemetry_status || "-")}<br>port ${esc(item.wireless_telemetry_port ?? item.runtime_wireless_telemetry_port ?? "-")}<br>tel drop ${esc(item.wireless_telemetry_dropped ?? "-")}<br>tel err ${esc(item.wireless_telemetry_last_error ?? "-")}<br>age ${fmtAge(item.status_updated_at)}</td>
      <td>${renderBatteryCell(item)}</td>
    </tr>`).join("");
  renderUwbRadio(state.statuses[0] || {});
  renderCharger(state.statuses);
  scheduleAccelRender();
  hydrateSettingsFromStatus(state.statuses[0] || {});
}

function setSettingIfFresh(id, value) {
  const el = document.getElementById(id);
  if (!el || value === undefined || value === null || localStorage.getItem(settingKey(id)) !== null) return;
  if (el.type === "checkbox") {
    el.checked = Boolean(value);
  } else {
    el.value = value;
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
  setSettingIfFresh("uwbDtRxTimeoutMs", item.runtime_distance_test_rx_timeout_ms);
  setSettingIfFresh("uwbDtRespDelayMs", item.runtime_distance_test_resp_delay_ms);
  setSettingIfFresh("uwbDtFinalDelayMs", item.runtime_distance_test_final_delay_ms);
  setSettingIfFresh("uwbDtReportDelayMs", item.runtime_distance_test_report_delay_ms);
  setSettingIfFresh("uwbDtAutoRxDelayUus", item.runtime_distance_test_auto_rx_delay_uus);
  setSettingIfFresh("uwbCalSummary", item.runtime_calibration_summary_every);
  setSettingIfFresh("uwbCalMinMs", item.runtime_calibration_min_interval_ms);
  setSettingIfFresh("uwbCalMaxMs", item.runtime_calibration_max_interval_ms);
  setSettingIfFresh("uwbCalRxMs", item.runtime_calibration_rx_slice_ms);
  setSettingIfFresh("uwbAntennaDelayHex", item.uwb_configured_antenna_delay_hex || item.uwb_active_antenna_delay_hex);
  setSettingIfFresh("chargerAdcEnabled", item.charger_adc_enabled);
  setSettingIfFresh("chargerAdcRate", item.charger_adc_continuous ? "continuous" : "oneshot");
  setSettingIfFresh("chargerAdcSample", item.charger_adc_sample ?? 2);
  setSettingIfFresh("chargerAdcAvg", item.charger_adc_running_average);
  setSettingIfFresh("chargerChargeEnabled", item.charger_charge_enabled);
  setSettingIfFresh("chargerMinimalSystemMv", item.charger_minimal_system_voltage_mv);
  setSettingIfFresh("chargerChargeVoltageMv", item.charger_charge_voltage_limit_mv);
  setSettingIfFresh("chargerChargeCurrentMa", item.charger_charge_current_limit_ma);
  setSettingIfFresh("chargerInputVoltageMv", item.charger_input_voltage_limit_mv);
  setSettingIfFresh("chargerInputCurrentMa", item.charger_input_current_limit_ma);
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
    d.innerHTML = `<svg viewBox="0 0 520 320" role="img">
      <line class="edge" x1="140" y1="160" x2="380" y2="160"></line>
      <text class="edge-label" x="235" y="142">${esc(document.getElementById("calKnownCm").value)} cm</text>
      <rect class="node" x="100" y="125" width="80" height="70" rx="8"></rect>
      <rect class="node" x="340" y="125" width="80" height="70" rx="8"></rect>
      <text x="124" y="166" font-size="18" font-weight="700">M${esc(document.getElementById("calRef").value)}</text>
      <text x="364" y="166" font-size="18" font-weight="700">M${esc(document.getElementById("calDut").value)}</text>
    </svg>`;
  } else {
    const ids = document.getElementById("calThree").value.split(",").map(v => v.trim());
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

async function postJsonEndpoint(path, payload, toastId) {
  setToast(toastId, "sending...", "", null, false);
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    const ok = apiResponseOk(data);
    setToast(toastId, summarizeApiResponse(data), ok ? "ok" : "bad", data);
    return data;
  } catch (error) {
    const data = {ok: false, error: String(error)};
    setToast(toastId, summarizeApiResponse(data), "bad", data);
    return data;
  }
}

async function postConfig(payload, toastId) {
  return postJsonEndpoint("/api/runtime-config", payload, toastId);
}

async function postAntennaDelay(payload, toastId) {
  return postJsonEndpoint("/api/antenna-delay", payload, toastId);
}

async function postChargerConfig(payload, toastId) {
  return postJsonEndpoint("/api/charger-config", payload, toastId);
}

function settingKey(id) { return `uwbDash.setting.${id}`; }
function persistedSettingIds() {
  return [
    "runtimeTargets", "runtimeMode", "runtimeTag", "runtimeAnchors", "runtimeReboot",
    "runtimeUwb", "runtimeBno085", "runtimeGps", "runtimeTelemetryPort",
    "accelTimebase", "accelSampleHz", "accelTargets",
    "uwbTargets", "uwbRadioChannel", "uwbSurveyRxMs", "uwbSurveyDelayMs", "uwbSurveySlotMs",
    "uwbSurveyGapMs", "uwbSurveyLogEvery", "uwbRangingSlotMs",
    "uwbRangingGapMs", "uwbRangingRxMs", "uwbDtInitiator", "uwbDtResponder",
    "uwbDtIntervalMs", "uwbDtRxTimeoutMs", "uwbDtRespDelayMs",
    "uwbDtFinalDelayMs", "uwbDtReportDelayMs", "uwbDtAutoRxDelayUus",
    "uwbCalSummary", "uwbCalMinMs", "uwbCalMaxMs", "uwbCalRxMs",
    "uwbAntennaDelayHex", "uwbAdvancedReboot",
    "chargerTargets", "chargerRawModule", "chargerAdcEnabled",
    "chargerAdcRate", "chargerAdcSample", "chargerAdcAvg",
    "chargerChargeEnabled", "chargerMinimalSystemMv", "chargerChargeVoltageMv",
    "chargerChargeCurrentMa", "chargerInputVoltageMv",
    "chargerInputCurrentMa", "chargerRawReg", "chargerRawValue",
    "chargerRawMask", "chargerRawBits",
    "calTargets", "calMethod", "calRef", "calDut", "calKnownCm", "calThree",
    "calD01Cm", "calD02Cm", "calD12Cm", "calSamples",
  ];
}
function restoreSettings() {
  for (const id of persistedSettingIds()) {
    const el = document.getElementById(id);
    const saved = localStorage.getItem(settingKey(id));
    if (!el || saved === null) continue;
    if (el.type === "checkbox") {
      el.checked = saved === "1";
    } else {
      el.value = saved;
    }
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

function wireSettings() {
  restoreSettings();
  wireSettingPersistence();
  const accelTimebase = document.getElementById("accelTimebase");
  if (accelTimebase) {
    state.timebaseSecPerDiv = Math.max(1, Number(accelTimebase.value || 5));
    accelTimebase.addEventListener("change", () => {
      state.timebaseSecPerDiv = Math.max(1, Number(accelTimebase.value || 5));
      renderAccelGraphs();
    });
  }
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
        cal_min_ms: document.getElementById("uwbCalMinMs").value,
        cal_max_ms: document.getElementById("uwbCalMaxMs").value,
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
    postChargerConfig({
      target_modules: document.getElementById("chargerTargets").value,
      params: {
        adc: document.getElementById("chargerAdcEnabled").checked ? "1" : "0",
        adc_rate: document.getElementById("chargerAdcRate").value,
        adc_sample: document.getElementById("chargerAdcSample").value,
        adc_avg: document.getElementById("chargerAdcAvg").checked ? "1" : "0",
      }
    }, "chargerToast");
  });
  document.getElementById("disableChargerWatchdog").addEventListener("click", () => {
    postChargerConfig({
      target_modules: document.getElementById("chargerTargets").value,
      params: {disable_watchdog: "1"}
    }, "chargerToast");
  });
  document.getElementById("refreshCharger").addEventListener("click", () => {
    postChargerConfig({
      target_modules: document.getElementById("chargerTargets").value,
      params: {refresh: "1"}
    }, "chargerToast");
  });
  document.getElementById("applyChargerLimits").addEventListener("click", () => {
    const params = {
      charge_enabled: document.getElementById("chargerChargeEnabled").checked ? "1" : "0",
      minimal_system_voltage_mv: document.getElementById("chargerMinimalSystemMv").value,
      charge_voltage_mv: document.getElementById("chargerChargeVoltageMv").value,
      charge_current_ma: document.getElementById("chargerChargeCurrentMa").value,
      input_voltage_mv: document.getElementById("chargerInputVoltageMv").value,
      input_current_ma: document.getElementById("chargerInputCurrentMa").value,
    };
    postChargerConfig({
      target_modules: document.getElementById("chargerTargets").value,
      params,
    }, "chargerToast");
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
      target_modules: document.getElementById("chargerTargets").value,
      params,
    }, "chargerToast");
  });
  document.getElementById("applyCalibration").addEventListener("click", () => {
    const method = document.getElementById("calMethod").value;
    const params = {mode: "calibration", cal_method: method, cal_samples: document.getElementById("calSamples").value, reboot: "1"};
    if (method === "two") {
      params.cal_ref = document.getElementById("calRef").value;
      params.cal_dut = document.getElementById("calDut").value;
      params.cal_known_cm = document.getElementById("calKnownCm").value;
    } else {
      params.cal_three = document.getElementById("calThree").value;
      params.cal_d01_cm = document.getElementById("calD01Cm").value;
      params.cal_d02_cm = document.getElementById("calD02Cm").value;
      params.cal_d12_cm = document.getElementById("calD12Cm").value;
    }
    postConfig({target_modules: document.getElementById("calTargets").value, params}, "calToast");
  });
  ["calMethod","calRef","calDut","calKnownCm","calThree","calD01Cm","calD02Cm","calD12Cm"].forEach(id => {
    document.getElementById(id).addEventListener("input", updateCalVisibility);
    document.getElementById(id).addEventListener("change", updateCalVisibility);
  });
  updateCalVisibility();
}

document.querySelectorAll(".terminal").forEach(createTerminal);
document.querySelectorAll(".tab").forEach(tab => tab.addEventListener("click", () => setActiveTab(tab.dataset.tab)));
setActiveTab(state.activeTab);
wireSettings();
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
        self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/runtime-config":
            self.handle_runtime_config()
            return
        if parsed.path == "/api/antenna-delay":
            self.handle_antenna_delay()
            return
        if parsed.path == "/api/charger-config":
            self.handle_charger_config()
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

    def apply_runtime_config(
        self, params: dict[str, str], target_modules: Any = None
    ) -> list[dict[str, Any]]:
        if not self.token:
            raise RuntimeError("APP_OTA_PASSWORD missing in secrets.h")
        if not params:
            raise RuntimeError("No runtime config parameters provided")
        targets = self.resolve_targets(target_modules)
        return [self.send_runtime_config(target, params) for target in targets]

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

    def resolve_targets(self, target_modules: Any) -> list[str]:
        if target_modules in (None, "", "all"):
            return self.targets

        if isinstance(target_modules, str):
            raw_items = [target_modules]
        else:
            raw_items = list(target_modules)

        module_ids: list[int] = []
        for raw in raw_items:
            text = str(raw).strip()
            if not text or text == "all":
                return self.targets
            module_ids.append(int(text))

        with self.state.lock:
            by_module = {
                int(status["module_id"]): str(status["target"])
                for status in self.state.status_by_module.values()
                if status.get("module_id") is not None and status.get("target")
            }

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
                "No live target known for module(s): " +
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
