#!/usr/bin/env python3
"""Live browser view for UWB ranging logs.

The script listens for the same TCP wireless logs as wireless_log_listener.py,
parses UWB_RANGING result lines, trilaterates the configured tag position from
the latest anchor distances, and serves a small browser UI.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import socketserver
import sys
import threading
import time
import urllib.parse
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


DEFAULT_ANCHORS = {
    4: (0.0, 0.0),
    5: (2.0, 0.0),
    3: (0.0, 2.0),
    2: (2.0, 2.0),
}

RANGING_RE = re.compile(
    r"UWB_RANGING result\s+tag=(?P<tag>\d+)\s+anchor=(?P<anchor>\d+)\s+"
    r"seq=(?P<seq>\d+)\s+distance=(?P<distance>[+-]?\d+(?:\.\d+)?)\s+m"
)


def parse_anchor(text: str) -> tuple[int, tuple[float, float]]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("anchor must look like ID=X,Y")
    raw_id, raw_xy = text.split("=", 1)
    try:
        anchor_id = int(raw_id.strip())
        raw_x, raw_y = raw_xy.split(",", 1)
        return anchor_id, (float(raw_x), float(raw_y))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("anchor must look like ID=X,Y") from exc


def default_square_anchors(side_m: float) -> dict[int, tuple[float, float]]:
    return {
        4: (0.0, 0.0),
        5: (side_m, 0.0),
        3: (0.0, side_m),
        2: (side_m, side_m),
    }


def solve_2x2(a00: float, a01: float, a10: float, a11: float,
              b0: float, b1: float) -> tuple[float, float] | None:
    det = a00 * a11 - a01 * a10
    if abs(det) < 1e-9:
        return None
    return ((b0 * a11 - a01 * b1) / det,
            (a00 * b1 - b0 * a10) / det)


def trilaterate(
    anchors: dict[int, tuple[float, float]],
    distances: dict[int, float],
) -> tuple[float, float] | None:
    usable = [
        (anchor_id, anchors[anchor_id], distances[anchor_id])
        for anchor_id in sorted(distances)
        if anchor_id in anchors and distances[anchor_id] > 0.0
    ]
    if len(usable) < 3:
        return None

    _, (x0, y0), r0 = usable[0]
    normal_xx = 0.0
    normal_xy = 0.0
    normal_yy = 0.0
    rhs_x = 0.0
    rhs_y = 0.0

    for _, (xi, yi), ri in usable[1:]:
        ax = 2.0 * (xi - x0)
        ay = 2.0 * (yi - y0)
        b = (
            r0 * r0
            - ri * ri
            + xi * xi
            - x0 * x0
            + yi * yi
            - y0 * y0
        )
        normal_xx += ax * ax
        normal_xy += ax * ay
        normal_yy += ay * ay
        rhs_x += ax * b
        rhs_y += ay * b

    return solve_2x2(normal_xx, normal_xy, normal_xy, normal_yy, rhs_x, rhs_y)


def residuals_for_position(
    position: tuple[float, float] | None,
    anchors: dict[int, tuple[float, float]],
    distances: dict[int, float],
) -> dict[int, float]:
    if position is None:
        return {}
    px, py = position
    residuals: dict[int, float] = {}
    for anchor_id, distance in distances.items():
        if anchor_id not in anchors:
            continue
        ax, ay = anchors[anchor_id]
        estimated = math.hypot(px - ax, py - ay)
        residuals[anchor_id] = estimated - distance
    return residuals


def mean_std(values: list[float]) -> tuple[float | None, float | None]:
    if not values:
        return None, None
    mean = sum(values) / len(values)
    if len(values) < 2:
        return mean, 0.0
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return mean, math.sqrt(max(0.0, variance))


class RangingState:
    def __init__(
        self,
        *,
        tag_id: int,
        anchors: dict[int, tuple[float, float]],
        max_age_sec: float,
        distance_alpha: float,
        position_alpha: float,
        trail_len: int,
        stats_window: int,
    ) -> None:
        self.tag_id = tag_id
        self.anchors = anchors
        self.max_age_sec = max_age_sec
        self.distance_alpha = distance_alpha
        self.position_alpha = position_alpha
        self.trail_len = trail_len
        self.stats_window = stats_window
        self.lock = threading.Lock()
        self.distances: dict[int, dict[str, Any]] = {}
        self.distance_history: dict[int, list[tuple[float, float]]] = {}
        self.position: tuple[float, float] | None = None
        self.raw_position: tuple[float, float] | None = None
        self.trail: list[tuple[float, float, float]] = []
        self.position_history: list[tuple[float, float, float]] = []
        self.log_clients = 0
        self.last_line = ""
        self.last_update_monotonic = 0.0

    def update_distance(self, anchor_id: int, distance_m: float, seq: int,
                        raw_line: str) -> None:
        now = time.monotonic()
        with self.lock:
            history = self.distance_history.setdefault(anchor_id, [])
            history.append((now, distance_m))
            if len(history) > self.stats_window:
                del history[:-self.stats_window]

            previous = self.distances.get(anchor_id)
            if previous is None:
                filtered = distance_m
            else:
                filtered = (
                    self.distance_alpha * distance_m
                    + (1.0 - self.distance_alpha) * float(previous["distance_m"])
                )
            self.distances[anchor_id] = {
                "distance_m": filtered,
                "raw_distance_m": distance_m,
                "seq": seq,
                "updated_age_sec": 0.0,
                "updated_at": now,
            }
            self.last_line = raw_line
            self.last_update_monotonic = now
            self._recalculate_locked(now, append_trail=True)

    def set_client_count(self, count: int) -> None:
        with self.lock:
            self.log_clients = count

    def _fresh_distances_locked(self, now: float) -> dict[int, float]:
        fresh: dict[int, float] = {}
        for anchor_id, item in self.distances.items():
            age = now - float(item["updated_at"])
            if age <= self.max_age_sec:
                fresh[anchor_id] = float(item["distance_m"])
        return fresh

    def _recalculate_locked(self, now: float, *, append_trail: bool) -> None:
        fresh = self._fresh_distances_locked(now)
        raw_position = trilaterate(self.anchors, fresh)
        self.raw_position = raw_position
        if raw_position is None:
            return

        if self.position is None:
            self.position = raw_position
        else:
            px, py = self.position
            rx, ry = raw_position
            self.position = (
                self.position_alpha * rx + (1.0 - self.position_alpha) * px,
                self.position_alpha * ry + (1.0 - self.position_alpha) * py,
            )

        assert self.position is not None
        if append_trail:
            self.trail.append((self.position[0], self.position[1], now))
            if len(self.trail) > self.trail_len:
                self.trail = self.trail[-self.trail_len:]
            self.position_history.append((self.position[0], self.position[1], now))
            if len(self.position_history) > self.stats_window:
                del self.position_history[:-self.stats_window]

    def _position_stats_locked(self, now: float) -> dict[str, Any]:
        fresh = [
            (x, y, sample_time)
            for x, y, sample_time in self.position_history
            if now - sample_time <= self.max_age_sec * 3.0
        ]
        if not fresh:
            return {"samples": 0}

        xs = [item[0] for item in fresh]
        ys = [item[1] for item in fresh]
        mean_x, std_x = mean_std(xs)
        mean_y, std_y = mean_std(ys)
        return {
            "samples": len(fresh),
            "mean_x": mean_x,
            "mean_y": mean_y,
            "std_x": std_x,
            "std_y": std_y,
            "span_x": max(xs) - min(xs),
            "span_y": max(ys) - min(ys),
        }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            self._recalculate_locked(now, append_trail=False)
            distances: dict[str, Any] = {}
            fresh = self._fresh_distances_locked(now)
            for anchor_id in sorted(self.distances):
                item = dict(self.distances[anchor_id])
                item["updated_age_sec"] = now - float(item["updated_at"])
                item["fresh"] = anchor_id in fresh
                raw_values = [
                    distance
                    for sample_time, distance in self.distance_history.get(anchor_id, [])
                    if now - sample_time <= self.max_age_sec * 3.0
                ]
                raw_mean, raw_std = mean_std(raw_values)
                item["raw_stats"] = {
                    "samples": len(raw_values),
                    "mean_m": raw_mean,
                    "std_m": raw_std,
                    "min_m": min(raw_values) if raw_values else None,
                    "max_m": max(raw_values) if raw_values else None,
                }
                item.pop("updated_at", None)
                distances[str(anchor_id)] = item

            position = None
            if self.position is not None:
                position = {"x": self.position[0], "y": self.position[1]}
            raw_position = None
            if self.raw_position is not None:
                raw_position = {"x": self.raw_position[0], "y": self.raw_position[1]}

            residuals = residuals_for_position(self.position, self.anchors, fresh)
            return {
                "tag_id": self.tag_id,
                "anchors": {
                    str(anchor_id): {"x": xy[0], "y": xy[1]}
                    for anchor_id, xy in sorted(self.anchors.items())
                },
                "distances": distances,
                "fresh_anchor_count": len(fresh),
                "position": position,
                "raw_position": raw_position,
                "position_stats": self._position_stats_locked(now),
                "trail": [
                    {"x": x, "y": y, "age_sec": now - t}
                    for x, y, t in self.trail
                ],
                "residuals_m": {
                    str(anchor_id): residual
                    for anchor_id, residual in sorted(residuals.items())
                },
                "log_clients": self.log_clients,
                "last_line": self.last_line,
                "last_update_age_sec": (
                    None
                    if self.last_update_monotonic == 0.0
                    else now - self.last_update_monotonic
                ),
                "max_age_sec": self.max_age_sec,
            }


class LogServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int],
                 handler_class: type[socketserver.BaseRequestHandler],
                 state: RangingState) -> None:
        super().__init__(server_address, handler_class)
        self.state = state
        self.client_lock = threading.Lock()
        self.client_count = 0

    def client_connected(self) -> None:
        with self.client_lock:
            self.client_count += 1
            self.state.set_client_count(self.client_count)

    def client_disconnected(self) -> None:
        with self.client_lock:
            self.client_count = max(0, self.client_count - 1)
            self.state.set_client_count(self.client_count)


class LogHandler(socketserver.BaseRequestHandler):
    server: LogServer

    def handle(self) -> None:
        self.server.client_connected()
        buffer = b""
        try:
            while True:
                data = self.request.recv(4096)
                if not data:
                    break
                buffer += data
                while True:
                    newline = buffer.find(b"\n")
                    if newline < 0:
                        break
                    raw = buffer[:newline]
                    buffer = buffer[newline + 1:]
                    self.process_line(raw.decode("utf-8", errors="replace").strip())
        finally:
            if buffer.strip():
                self.process_line(buffer.decode("utf-8", errors="replace").strip())
            self.server.client_disconnected()

    def process_line(self, line: str) -> None:
        if not line:
            return
        match = RANGING_RE.search(line)
        if match is None:
            return
        tag_id = int(match.group("tag"))
        if tag_id != self.server.state.tag_id:
            return
        anchor_id = int(match.group("anchor"))
        distance_m = float(match.group("distance"))
        seq = int(match.group("seq"))
        self.server.state.update_distance(anchor_id, distance_m, seq, line)


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>UWB Live View</title>
<style>
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  --bg: #f7f8fb;
  --panel: #ffffff;
  --ink: #17202a;
  --muted: #667085;
  --line: #d9dee8;
  --accent: #d7352a;
  --blue: #2b64d8;
  --green: #16833a;
  --orange: #c26700;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--ink);
}
.shell {
  min-height: 100vh;
  display: grid;
  grid-template-columns: minmax(320px, 1fr) 340px;
}
.stage {
  padding: 18px;
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.topbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
}
h1 {
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  letter-spacing: 0;
}
.status {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
  justify-content: flex-end;
}
.pill {
  border: 1px solid var(--line);
  background: var(--panel);
  border-radius: 999px;
  padding: 5px 9px;
  font-size: 12px;
  color: var(--muted);
  white-space: nowrap;
}
.pill.good { color: var(--green); }
.pill.warn { color: var(--orange); }
.canvas-wrap {
  position: relative;
  flex: 1;
  min-height: 480px;
  border: 1px solid var(--line);
  background: #eef1f5;
}
canvas {
  width: 100%;
  height: 100%;
  display: block;
}
.sidebar {
  border-left: 1px solid var(--line);
  background: var(--panel);
  padding: 18px;
  overflow: auto;
}
.metric {
  border-bottom: 1px solid var(--line);
  padding: 13px 0;
}
.metric:first-child { padding-top: 0; }
.label {
  color: var(--muted);
  font-size: 12px;
  margin-bottom: 5px;
}
.value {
  font-size: 22px;
  font-weight: 700;
  letter-spacing: 0;
}
table {
  width: 100%;
  border-collapse: collapse;
  font-size: 13px;
}
th, td {
  padding: 8px 4px;
  border-bottom: 1px solid var(--line);
  text-align: right;
}
th:first-child, td:first-child { text-align: left; }
th { color: var(--muted); font-weight: 600; }
.fresh { color: var(--green); }
.stale { color: var(--orange); }
.last-line {
  margin-top: 10px;
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 11px;
  line-height: 1.45;
  color: var(--muted);
  word-break: break-word;
}
@media (max-width: 860px) {
  .shell { grid-template-columns: 1fr; }
  .sidebar { border-left: 0; border-top: 1px solid var(--line); }
  .canvas-wrap { min-height: 420px; }
}
</style>
</head>
<body>
<div class="shell">
  <main class="stage">
    <div class="topbar">
      <h1>UWB Live Tag Position</h1>
      <div class="status">
        <span id="clients" class="pill">0 log clients</span>
        <span id="fresh" class="pill warn">0 anchors</span>
        <span id="age" class="pill warn">waiting</span>
      </div>
    </div>
    <div class="canvas-wrap"><canvas id="plot"></canvas></div>
  </main>
  <aside class="sidebar">
    <div class="metric">
      <div class="label">Tag Position</div>
      <div id="position" class="value">waiting</div>
    </div>
    <div class="metric">
      <div class="label">Position Stability</div>
      <table>
        <tbody>
          <tr><td>std x</td><td id="stdX">-</td></tr>
          <tr><td>std y</td><td id="stdY">-</td></tr>
          <tr><td>span x</td><td id="spanX">-</td></tr>
          <tr><td>span y</td><td id="spanY">-</td></tr>
        </tbody>
      </table>
    </div>
    <div class="metric">
      <div class="label">Distances</div>
      <table>
        <thead>
          <tr><th>Anchor</th><th>m</th><th>std</th><th>age</th><th>resid.</th></tr>
        </thead>
        <tbody id="distances"></tbody>
      </table>
    </div>
    <div class="metric">
      <div class="label">Last Ranging Line</div>
      <div id="lastLine" class="last-line">waiting for logs</div>
    </div>
  </aside>
</div>
<script>
const canvas = document.getElementById("plot");
const ctx = canvas.getContext("2d");
let state = null;

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  return Number(value).toFixed(digits);
}

function cm(value) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  return `${(Number(value) * 100).toFixed(1)} cm`;
}

function boundsFor(data) {
  const pts = [];
  for (const item of Object.values(data.anchors || {})) pts.push([item.x, item.y]);
  if (data.position) pts.push([data.position.x, data.position.y]);
  for (const item of data.trail || []) pts.push([item.x, item.y]);
  let minX = Math.min(...pts.map(p => p[0]), -0.2);
  let maxX = Math.max(...pts.map(p => p[0]), 2.2);
  let minY = Math.min(...pts.map(p => p[1]), -0.2);
  let maxY = Math.max(...pts.map(p => p[1]), 2.2);
  const pad = Math.max(0.35, 0.12 * Math.max(maxX - minX, maxY - minY));
  return {minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad};
}

function makeTransform(data, width, height) {
  const b = boundsFor(data);
  const margin = 48;
  const sx = (width - margin * 2) / (b.maxX - b.minX);
  const sy = (height - margin * 2) / (b.maxY - b.minY);
  const scale = Math.max(1, Math.min(sx, sy));
  const plotW = (b.maxX - b.minX) * scale;
  const plotH = (b.maxY - b.minY) * scale;
  const ox = (width - plotW) / 2;
  const oy = (height - plotH) / 2;
  return {
    scale,
    x: value => ox + (value - b.minX) * scale,
    y: value => oy + (b.maxY - value) * scale
  };
}

function drawGrid(tx, width, height) {
  ctx.fillStyle = "#eef1f5";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d9dee8";
  ctx.lineWidth = 1;
  ctx.font = "12px system-ui, sans-serif";
  ctx.fillStyle = "#667085";
  for (let m = -1; m <= 4; m += 0.5) {
    const x = tx.x(m);
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
    const y = tx.y(m);
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function draw(data) {
  resizeCanvas();
  const rect = canvas.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;
  const tx = makeTransform(data, width, height);
  drawGrid(tx, width, height);

  ctx.save();
  ctx.strokeStyle = "#9aa4b2";
  ctx.lineWidth = 2;
  const ids = ["4", "5", "2", "3", "4"];
  ctx.beginPath();
  ids.forEach((id, index) => {
    const a = data.anchors[id];
    if (!a) return;
    const x = tx.x(a.x);
    const y = tx.y(a.y);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.restore();

  for (const [id, anchor] of Object.entries(data.anchors || {})) {
    const distance = data.distances[id];
    if (distance && distance.fresh) {
      ctx.beginPath();
      ctx.strokeStyle = "rgba(43, 100, 216, 0.24)";
      ctx.lineWidth = 2;
      ctx.arc(tx.x(anchor.x), tx.y(anchor.y), distance.distance_m * tx.scale, 0, Math.PI * 2);
      ctx.stroke();
    }
  }

  if ((data.trail || []).length > 1) {
    ctx.beginPath();
    for (let i = 0; i < data.trail.length; i++) {
      const p = data.trail[i];
      const x = tx.x(p.x);
      const y = tx.y(p.y);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = "rgba(43, 100, 216, 0.72)";
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  for (const [id, anchor] of Object.entries(data.anchors || {})) {
    const x = tx.x(anchor.x);
    const y = tx.y(anchor.y);
    ctx.beginPath();
    ctx.fillStyle = "#16833a";
    ctx.moveTo(x, y - 11);
    ctx.lineTo(x - 10, y + 9);
    ctx.lineTo(x + 10, y + 9);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = "#17202a";
    ctx.font = "700 13px system-ui, sans-serif";
    ctx.fillText(`A${id}`, x + 12, y - 10);
  }

  if (data.position) {
    const x = tx.x(data.position.x);
    const y = tx.y(data.position.y);
    ctx.beginPath();
    ctx.fillStyle = "#d7352a";
    ctx.arc(x, y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.fillStyle = "#17202a";
    ctx.font = "700 13px system-ui, sans-serif";
    ctx.fillText(`Tag ${data.tag_id}`, x + 13, y - 13);
  }
}

function updateSidebar(data) {
  document.getElementById("clients").textContent = `${data.log_clients} log client${data.log_clients === 1 ? "" : "s"}`;
  const fresh = document.getElementById("fresh");
  fresh.textContent = `${data.fresh_anchor_count} fresh anchors`;
  fresh.className = `pill ${data.fresh_anchor_count >= 3 ? "good" : "warn"}`;
  const age = document.getElementById("age");
  if (data.last_update_age_sec === null) {
    age.textContent = "waiting";
    age.className = "pill warn";
  } else {
    age.textContent = `${fmt(data.last_update_age_sec, 1)}s ago`;
    age.className = `pill ${data.last_update_age_sec < data.max_age_sec ? "good" : "warn"}`;
  }

  const pos = document.getElementById("position");
  if (data.position) pos.textContent = `x=${fmt(data.position.x)} m, y=${fmt(data.position.y)} m`;
  else pos.textContent = "waiting";

  const stats = data.position_stats || {};
  document.getElementById("stdX").textContent = cm(stats.std_x);
  document.getElementById("stdY").textContent = cm(stats.std_y);
  document.getElementById("spanX").textContent = cm(stats.span_x);
  document.getElementById("spanY").textContent = cm(stats.span_y);

  const tbody = document.getElementById("distances");
  tbody.innerHTML = "";
  for (const id of Object.keys(data.anchors || {}).sort((a, b) => Number(a) - Number(b))) {
    const item = data.distances[id];
    const row = document.createElement("tr");
    if (!item) {
      row.innerHTML = `<td>A${id}</td><td>-</td><td>-</td><td>-</td><td>-</td>`;
    } else {
      const residual = data.residuals_m[id];
      const cls = item.fresh ? "fresh" : "stale";
      const rawStats = item.raw_stats || {};
      row.innerHTML = `<td>A${id}</td><td>${fmt(item.distance_m, 3)}</td><td>${cm(rawStats.std_m)}</td><td class="${cls}">${fmt(item.updated_age_sec, 1)}s</td><td>${fmt(residual, 3)}</td>`;
    }
    tbody.appendChild(row);
  }
  document.getElementById("lastLine").textContent = data.last_line || "waiting for logs";
}

function applyState(data) {
  state = data;
  updateSidebar(data);
  draw(data);
}

window.addEventListener("resize", () => { if (state) draw(state); });
const events = new EventSource("/events");
events.onmessage = event => applyState(JSON.parse(event.data));
events.onerror = () => {
  const age = document.getElementById("age");
  age.textContent = "viewer disconnected";
  age.className = "pill warn";
};
</script>
</body>
</html>
"""


class HttpHandler(BaseHTTPRequestHandler):
    server: "LiveHttpServer"

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.server.quiet:
            return
        super().log_message(fmt, *args)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))
            return
        if parsed.path == "/state":
            self.send_json(self.server.state.snapshot())
            return
        if parsed.path == "/events":
            self.stream_events()
            return
        self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def send_json(self, payload: dict[str, Any]) -> None:
        raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def stream_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        while True:
            raw = json.dumps(
                self.server.state.snapshot(), separators=(",", ":")
            ).encode("utf-8")
            try:
                self.wfile.write(b"data: " + raw + b"\n\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            time.sleep(self.server.event_interval_sec)


class LiveHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        state: RangingState,
        *,
        event_interval_sec: float,
        quiet: bool,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.state = state
        self.event_interval_sec = event_interval_sec
        self.quiet = quiet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live 2D UWB tag viewer from wireless log ranging lines."
    )
    parser.add_argument("--log-host", default="0.0.0.0")
    parser.add_argument("--log-port", type=int, default=6055)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--tag", type=int, default=1)
    parser.add_argument("--side-m", type=float, default=2.0)
    parser.add_argument(
        "--anchor",
        action="append",
        type=parse_anchor,
        default=[],
        help="Override/add an anchor coordinate, e.g. --anchor 2=2,2",
    )
    parser.add_argument("--max-age-sec", type=float, default=4.0)
    parser.add_argument("--distance-alpha", type=float, default=0.55)
    parser.add_argument("--position-alpha", type=float, default=0.45)
    parser.add_argument("--trail-len", type=int, default=240)
    parser.add_argument("--stats-window", type=int, default=80)
    parser.add_argument("--event-interval-sec", type=float, default=0.15)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def run_self_test() -> int:
    anchors = DEFAULT_ANCHORS
    expected = (1.0, 1.0)
    distances = {
        anchor_id: math.hypot(expected[0] - xy[0], expected[1] - xy[1])
        for anchor_id, xy in anchors.items()
    }
    got = trilaterate(anchors, distances)
    if got is None:
        print("self-test failed: no position")
        return 1
    err = math.hypot(got[0] - expected[0], got[1] - expected[1])
    if err > 1e-6:
        print(f"self-test failed: got={got}, expected={expected}, err={err}")
        return 1

    sample = (
        "[uwb-module-5] [ 123 ms] [I][uwb_dw3000] "
        "UWB_RANGING result tag=1 anchor=5 seq=42 distance=1.667 m"
    )
    match = RANGING_RE.search(sample)
    if match is None or int(match.group("anchor")) != 5:
        print("self-test failed: parser did not match sample line")
        return 1
    print("self-test ok")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()

    anchors = default_square_anchors(args.side_m)
    for anchor_id, xy in args.anchor:
        anchors[anchor_id] = xy

    state = RangingState(
        tag_id=args.tag,
        anchors=anchors,
        max_age_sec=args.max_age_sec,
        distance_alpha=args.distance_alpha,
        position_alpha=args.position_alpha,
        trail_len=args.trail_len,
        stats_window=args.stats_window,
    )
    log_server = LogServer((args.log_host, args.log_port), LogHandler, state)
    http_server = LiveHttpServer(
        (args.http_host, args.http_port),
        HttpHandler,
        state,
        event_interval_sec=args.event_interval_sec,
        quiet=args.quiet,
    )

    log_thread = threading.Thread(target=log_server.serve_forever, daemon=True)
    log_thread.start()

    url = f"http://{args.http_host}:{args.http_port}/"
    print(
        f"Listening for UWB logs on {args.log_host}:{args.log_port}; "
        f"viewer at {url}",
        flush=True,
    )
    print(
        "Anchor coordinates: "
        + ", ".join(
            f"{anchor_id}=({xy[0]:.3f},{xy[1]:.3f})"
            for anchor_id, xy in sorted(anchors.items())
        ),
        flush=True,
    )
    if args.open_browser:
        webbrowser.open(url)

    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
        http_server.shutdown()
        log_server.shutdown()
        http_server.server_close()
        log_server.server_close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
