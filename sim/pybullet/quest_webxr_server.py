#!/usr/bin/env python3
"""Quest Browser WebXR server and controller-state receiver.

This module intentionally does not know about PyBullet. It serves the WebXR
page over HTTPS, receives controller poses over WebSocket, and stores the latest
right-controller state for another control loop to consume.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import socket
import ssl
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CERT_DIR = PROJECT_ROOT / ".local_certs"
CERT_PATH = CERT_DIR / "quest_webxr.crt"
KEY_PATH = CERT_DIR / "quest_webxr.key"
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def point_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5


class MotionRecorder:
    def __init__(self, out_root: Path, motion_threshold_m: float, idle_split_s: float) -> None:
        self.motion_threshold_m = motion_threshold_m
        self.idle_split_s = idle_split_s
        self.out_dir = out_root / datetime.now().strftime("quest_tracking_%Y%m%d_%H%M%S")
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = self.out_dir / "samples.jsonl"
        self.summary_path = self.out_dir / "summary.json"
        self.samples_file = self.samples_path.open("a")
        self.prev_position: tuple[float, float, float] | None = None
        self.active_segment = False
        self.segment_id = 0
        self.last_motion_time = 0.0
        self.last_summary_write = 0.0
        self.sample_count = 0
        self.motion_sample_count = 0
        self.total_path_m = 0.0
        self.min_xyz = [None, None, None]
        self.max_xyz = [None, None, None]
        self.segment_stats: dict[int, dict] = {}
        print(f"Quest tracking recorder: {self.out_dir.resolve()}")

    def _update_ranges(self, position: tuple[float, float, float]) -> None:
        for i, value in enumerate(position):
            self.min_xyz[i] = value if self.min_xyz[i] is None else min(self.min_xyz[i], value)
            self.max_xyz[i] = value if self.max_xyz[i] is None else max(self.max_xyz[i], value)

    def _update_segment(self, sample: dict) -> None:
        sid = sample["segment_id"]
        if sid <= 0:
            return
        stats = self.segment_stats.setdefault(
            sid,
            {
                "segment_id": sid,
                "start_time": sample["time"],
                "end_time": sample["time"],
                "samples": 0,
                "path_m": 0.0,
                "min_xyz": [None, None, None],
                "max_xyz": [None, None, None],
            },
        )
        stats["end_time"] = sample["time"]
        stats["samples"] += 1
        stats["path_m"] += sample["delta_m"]
        position = sample["position"]
        for i, value in enumerate(position):
            stats["min_xyz"][i] = value if stats["min_xyz"][i] is None else min(stats["min_xyz"][i], value)
            stats["max_xyz"][i] = value if stats["max_xyz"][i] is None else max(stats["max_xyz"][i], value)

    def record(self, sample: dict) -> dict:
        position = tuple(float(v) for v in sample["position"])
        now = float(sample["time"])
        delta = point_distance(self.prev_position, position) if self.prev_position is not None else 0.0
        moving_now = delta >= self.motion_threshold_m
        if moving_now:
            self.last_motion_time = now
            if not self.active_segment:
                self.segment_id += 1
                self.active_segment = True
        elif self.active_segment and now - self.last_motion_time > self.idle_split_s:
            self.active_segment = False

        record = dict(sample)
        record["delta_m"] = delta
        record["moving"] = moving_now
        record["segment_id"] = self.segment_id if self.active_segment else 0
        self.samples_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.samples_file.flush()

        self.sample_count += 1
        if moving_now:
            self.motion_sample_count += 1
        self.total_path_m += delta
        self._update_ranges(position)
        self._update_segment(record)
        self.prev_position = position
        if now - self.last_summary_write > 1.0:
            self.write_summary()
            self.last_summary_write = now
        return record

    def summary(self) -> dict:
        span_xyz = [
            None if self.min_xyz[i] is None or self.max_xyz[i] is None else self.max_xyz[i] - self.min_xyz[i]
            for i in range(3)
        ]
        return {
            "samples_path": str(self.samples_path),
            "sample_count": self.sample_count,
            "motion_sample_count": self.motion_sample_count,
            "motion_threshold_m": self.motion_threshold_m,
            "idle_split_s": self.idle_split_s,
            "total_path_m": self.total_path_m,
            "min_xyz": self.min_xyz,
            "max_xyz": self.max_xyz,
            "span_xyz": span_xyz,
            "segments": list(self.segment_stats.values()),
        }

    def write_summary(self) -> None:
        self.summary_path.write_text(json.dumps(self.summary(), indent=2) + "\n")

    def close(self) -> None:
        self.write_summary()
        self.samples_file.close()


INDEX_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Quest PyBullet Teleop</title>
  <style>
    html, body { margin: 0; min-height: 100%; background: #0a0a0a; color: #eee; font: 16px system-ui, sans-serif; }
    main { max-width: 760px; margin: 0 auto; padding: 24px; }
    button { font: inherit; padding: 12px 16px; border: 0; background: #4ca3ff; color: #04111f; border-radius: 6px; font-weight: 700; }
    pre { white-space: pre-wrap; background: #151515; padding: 14px; border-radius: 6px; color: #cfd8dc; }
    .muted { color: #aaa; }
  </style>
</head>
<body>
<main>
  <h1>Quest PyBullet Teleop</h1>
  <p class="muted">Use the right controller pose to move the Panda gripper. Trigger or grip closes the gripper.</p>
  <button id="start">Start VR Teleop</button>
  <pre id="log">Waiting...</pre>
  <canvas id="xrCanvas" width="1024" height="1024" style="display:none"></canvas>
</main>
<script>
const logEl = document.getElementById("log");
const startBtn = document.getElementById("start");
const canvas = document.getElementById("xrCanvas");
const gl = canvas.getContext("webgl", {xrCompatible: true, antialias: false});
let ws = null;
let session = null;
let refSpace = null;
let lastSent = 0;
let lastRightSource = null;

function log(msg) {
  logEl.textContent = msg;
}

function wsUrl() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/ws`;
}

function connectWs() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  ws = new WebSocket(wsUrl());
  ws.onopen = () => log("WebSocket connected. Press Start VR Teleop.");
  ws.onmessage = event => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "haptic") pulseHaptic(msg.intensity || 0.2, msg.duration_ms || 30);
    } catch (err) {
      // Ignore malformed control messages.
    }
  };
  ws.onclose = () => {
    log("WebSocket closed. Reconnecting...");
    ws = null;
    setTimeout(connectWs, 1000);
  };
  ws.onerror = () => log("WebSocket error.");
}

function buttonState(gamepad, index) {
  if (!gamepad || !gamepad.buttons || index >= gamepad.buttons.length) {
    return {pressed: false, value: 0};
  }
  const b = gamepad.buttons[index];
  return {pressed: !!b.pressed, value: b.value || 0};
}

function sourceName(source) {
  if (source.handedness === "left") return "left";
  if (source.handedness === "right") return "right";
  return "unknown";
}

function pulseHaptic(intensity, durationMs) {
  const gp = lastRightSource && lastRightSource.gamepad;
  if (!gp) return;
  const boundedIntensity = Math.max(0.0, Math.min(1.0, intensity));
  const boundedDuration = Math.max(10, Math.min(200, durationMs));
  if (gp.hapticActuators && gp.hapticActuators.length > 0 && gp.hapticActuators[0].pulse) {
    gp.hapticActuators[0].pulse(boundedIntensity, boundedDuration);
    return;
  }
  if (gp.vibrationActuator && gp.vibrationActuator.playEffect) {
    gp.vibrationActuator.playEffect("dual-rumble", {
      duration: boundedDuration,
      strongMagnitude: boundedIntensity,
      weakMagnitude: boundedIntensity * 0.5
    });
  }
}

function onFrame(t, frame) {
  session.requestAnimationFrame(onFrame);
  const pose = frame.getViewerPose(refSpace);
  if (pose) {
    gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
    gl.clearColor(0.02, 0.02, 0.025, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  }
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  if (performance.now() - lastSent < 33) return;
  lastSent = performance.now();

  const controllers = {};
  for (const source of session.inputSources) {
    if (!source.gripSpace) continue;
    const pose = frame.getPose(source.gripSpace, refSpace);
    if (!pose) continue;
    if (source.handedness === "right") lastRightSource = source;
    const p = pose.transform.position;
    const q = pose.transform.orientation;
    const gp = source.gamepad;
    controllers[sourceName(source)] = {
      position: [p.x, p.y, p.z],
      orientation: [q.x, q.y, q.z, q.w],
      trigger: buttonState(gp, 0),
      grip: buttonState(gp, 1),
      primary: buttonState(gp, 4),
      secondary: buttonState(gp, 5)
    };
  }

  ws.send(JSON.stringify({
    type: "xr_controller_pose",
    time: performance.now() / 1000,
    controllers
  }));
  const r = controllers.right;
  log(r ? `right xyz=${r.position.map(v => v.toFixed(3)).join(", ")} trigger=${r.trigger.value.toFixed(2)} grip=${r.grip.value.toFixed(2)}` : "No right controller pose yet.");
}

async function startVr() {
  connectWs();
  if (!navigator.xr) {
    log("WebXR not available. Use Meta Quest Browser over HTTPS.");
    return;
  }
  if (!gl) {
    log("WebGL not available, so WebXR cannot start.");
    return;
  }
  const ok = await navigator.xr.isSessionSupported("immersive-vr");
  if (!ok) {
    log("immersive-vr session not supported in this browser/device.");
    return;
  }
  log("Requesting immersive VR session...");
  session = await navigator.xr.requestSession("immersive-vr", {optionalFeatures: ["local-floor", "bounded-floor"]});
  await gl.makeXRCompatible();
  session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl)});
  session.addEventListener("end", () => {
    log("XR session ended. Press Start VR Teleop to reconnect.");
    session = null;
  });
  try {
    refSpace = await session.requestReferenceSpace("local-floor");
  } catch (err) {
    refSpace = await session.requestReferenceSpace("local");
  }
  log("XR session running. Move the right controller.");
  session.requestAnimationFrame(onFrame);
}

connectWs();
startBtn.onclick = startVr;
</script>
</body>
</html>
"""


DEBUG_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Quest Tracking Debug</title>
  <style>
    html, body { margin: 0; min-height: 100%; background: #080808; color: #eee; font: 14px system-ui, sans-serif; }
    main { max-width: 1180px; margin: 0 auto; padding: 16px; }
    h1 { margin: 0 0 12px; font-size: 22px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 12px; }
    .panel { background: #141414; border: 1px solid #2a2a2a; border-radius: 6px; padding: 10px; }
    canvas { width: 100%; height: 220px; background: #050505; border-radius: 4px; }
    pre { white-space: pre-wrap; margin: 0; color: #cfd8dc; }
    .muted { color: #aaa; }
  </style>
</head>
<body>
<main>
  <h1>Quest Tracking Debug</h1>
  <p class="muted">Live raw right-controller pose from WebXR. Use this to see range, packet flow, dead zones, and jitter.</p>
  <div class="grid">
    <section class="panel"><h2>Status</h2><pre id="status">waiting...</pre></section>
    <section class="panel"><h2>XYZ vs Time</h2><canvas id="timePlot" width="720" height="260"></canvas></section>
    <section class="panel"><h2>Top View: X/Z</h2><canvas id="topPlot" width="520" height="320"></canvas></section>
    <section class="panel"><h2>Front View: X/Y</h2><canvas id="frontPlot" width="520" height="320"></canvas></section>
  </div>
</main>
<script>
const statusEl = document.getElementById("status");
const timeCanvas = document.getElementById("timePlot");
const topCanvas = document.getElementById("topPlot");
const frontCanvas = document.getElementById("frontPlot");

function drawAxes(ctx, w, h, labelX, labelY) {
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#050505";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#333";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const x = 36 + (w - 56) * i / 4;
    const y = 16 + (h - 48) * i / 4;
    ctx.beginPath(); ctx.moveTo(x, 16); ctx.lineTo(x, h - 32); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(36, y); ctx.lineTo(w - 20, y); ctx.stroke();
  }
  ctx.fillStyle = "#aaa";
  ctx.fillText(labelX, w - 70, h - 10);
  ctx.fillText(labelY, 8, 20);
}

function range(values, fallback) {
  if (!values.length) return fallback;
  let lo = Math.min(...values), hi = Math.max(...values);
  if (Math.abs(hi - lo) < 0.02) { lo -= 0.01; hi += 0.01; }
  const pad = (hi - lo) * 0.12;
  return [lo - pad, hi + pad];
}

function map(v, lo, hi, outLo, outHi) {
  return outLo + (v - lo) * (outHi - outLo) / Math.max(1e-9, hi - lo);
}

function drawTime(history) {
  const c = timeCanvas, ctx = c.getContext("2d"), w = c.width, h = c.height;
  drawAxes(ctx, w, h, "time", "m");
  if (!history.length) return;
  const t0 = history[0].time;
  const ts = history.map(s => s.time - t0);
  const vals = history.flatMap(s => s.position || []);
  const [vlo, vhi] = range(vals, [-1, 1]);
  const [tlo, thi] = range(ts, [0, 1]);
  const colors = ["#ff6b6b", "#8bd450", "#4ca3ff"];
  ["x", "y", "z"].forEach((name, axis) => {
    ctx.strokeStyle = colors[axis];
    ctx.lineWidth = 2;
    ctx.beginPath();
    history.forEach((s, i) => {
      if (!s.position) return;
      const x = map(ts[i], tlo, thi, 36, w - 20);
      const y = map(s.position[axis], vlo, vhi, h - 32, 16);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.fillStyle = colors[axis];
    ctx.fillText(name, 48 + axis * 34, 28);
  });
}

function drawScatter(canvas, history, axes, labels) {
  const ctx = canvas.getContext("2d"), w = canvas.width, h = canvas.height;
  drawAxes(ctx, w, h, labels[0], labels[1]);
  const pts = history.filter(s => s.position).map(s => [s.position[axes[0]], s.position[axes[1]]]);
  if (!pts.length) return;
  const [xlo, xhi] = range(pts.map(p => p[0]), [-0.5, 0.5]);
  const [ylo, yhi] = range(pts.map(p => p[1]), [-0.5, 0.5]);
  pts.forEach((p, i) => {
    const age = i / Math.max(1, pts.length - 1);
    ctx.fillStyle = `rgba(76, 163, 255, ${0.15 + 0.85 * age})`;
    ctx.beginPath();
    ctx.arc(map(p[0], xlo, xhi, 36, w - 20), map(p[1], ylo, yhi, h - 32, 16), 3, 0, Math.PI * 2);
    ctx.fill();
  });
}

async function tick() {
  try {
    const [status, history] = await Promise.all([
      fetch("/status", {cache: "no-store"}).then(r => r.json()),
      fetch("/history", {cache: "no-store"}).then(r => r.json())
    ]);
    const now = Date.now() / 1000;
    const age = status.last_time ? now - status.last_time : null;
    const positions = history.filter(s => s.position).map(s => s.position);
    const mins = [0, 1, 2].map(i => positions.length ? Math.min(...positions.map(p => p[i])) : null);
    const maxs = [0, 1, 2].map(i => positions.length ? Math.max(...positions.map(p => p[i])) : null);
    statusEl.textContent = JSON.stringify({
      packets: status.packets,
      websocket_clients: status.websocket_clients,
      websocket_connects: status.websocket_connects,
      websocket_disconnects: status.websocket_disconnects,
      packet_age_s: age === null ? null : Number(age.toFixed(3)),
      position: status.right_position,
      orientation_xyzw: status.right_orientation,
      trigger: status.trigger_value,
      grip: status.grip_value,
      samples: history.length,
      min_xyz: mins,
      max_xyz: maxs
    }, null, 2);
    drawTime(history);
    drawScatter(topCanvas, history, [0, 2], ["x", "z"]);
    drawScatter(frontCanvas, history, [0, 1], ["x", "y"]);
  } catch (err) {
    statusEl.textContent = String(err);
  }
  setTimeout(tick, 100);
}
tick();
</script>
</body>
</html>
"""


@dataclass
class TeleopState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    right_position: tuple[float, float, float] | None = None
    right_orientation: tuple[float, float, float, float] | None = None
    grip_value: float = 0.0
    trigger_value: float = 0.0
    packets: int = 0
    last_time: float = 0.0
    haptic_seq: int = 0
    haptic_intensity: float = 0.0
    haptic_duration_ms: int = 0
    haptic_reason: str = ""
    history: deque = field(default_factory=lambda: deque(maxlen=1200))
    websocket_clients: int = 0
    websocket_connects: int = 0
    websocket_disconnects: int = 0
    last_ws_connect_time: float = 0.0
    last_ws_disconnect_time: float = 0.0
    last_ws_message_time: float = 0.0
    recorder: MotionRecorder | None = None

    def websocket_connected(self) -> None:
        with self.lock:
            self.websocket_clients += 1
            self.websocket_connects += 1
            self.last_ws_connect_time = time.time()

    def websocket_disconnected(self) -> None:
        with self.lock:
            self.websocket_clients = max(0, self.websocket_clients - 1)
            self.websocket_disconnects += 1
            self.last_ws_disconnect_time = time.time()

    def update(self, message: dict) -> None:
        right = message.get("controllers", {}).get("right")
        with self.lock:
            self.last_ws_message_time = time.time()
        if not right:
            return
        position = right.get("position")
        orientation = right.get("orientation")
        if not position or len(position) != 3:
            return
        with self.lock:
            self.right_position = tuple(float(v) for v in position)
            if orientation and len(orientation) == 4:
                self.right_orientation = tuple(float(v) for v in orientation)
            self.trigger_value = float(right.get("trigger", {}).get("value", 0.0))
            self.grip_value = float(right.get("grip", {}).get("value", 0.0))
            self.packets += 1
            self.last_time = time.time()
            sample = {
                "time": self.last_time,
                "position": self.right_position,
                "orientation": self.right_orientation,
                "trigger": self.trigger_value,
                "grip": self.grip_value,
                "packets": self.packets,
            }
            if self.recorder is not None:
                recorded = self.recorder.record(sample)
                sample["delta_m"] = recorded["delta_m"]
                sample["moving"] = recorded["moving"]
                sample["segment_id"] = recorded["segment_id"]
            self.history.append(sample)
            if self.packets == 1 or self.packets % 100 == 0:
                print(
                    "Quest pose",
                    f"packets={self.packets}",
                    f"pos={tuple(round(v, 3) for v in self.right_position)}",
                    f"trigger={self.trigger_value:.2f}",
                    f"grip={self.grip_value:.2f}",
                )

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "right_position": self.right_position,
                "right_orientation": self.right_orientation,
                "grip_value": self.grip_value,
                "trigger_value": self.trigger_value,
                "packets": self.packets,
                "last_time": self.last_time,
                "haptic_seq": self.haptic_seq,
                "haptic_reason": self.haptic_reason,
                "websocket_clients": self.websocket_clients,
                "websocket_connects": self.websocket_connects,
                "websocket_disconnects": self.websocket_disconnects,
                "last_ws_connect_time": self.last_ws_connect_time,
                "last_ws_disconnect_time": self.last_ws_disconnect_time,
                "last_ws_message_time": self.last_ws_message_time,
                "recording_dir": str(self.recorder.out_dir) if self.recorder else None,
                "recording_summary": self.recorder.summary() if self.recorder else None,
            }

    def history_snapshot(self) -> list[dict]:
        with self.lock:
            return list(self.history)

    def request_haptic(self, intensity: float, duration_ms: int, reason: str = "") -> int:
        with self.lock:
            self.haptic_seq += 1
            self.haptic_intensity = max(0.0, min(1.0, float(intensity)))
            self.haptic_duration_ms = max(10, min(200, int(duration_ms)))
            self.haptic_reason = reason
            return self.haptic_seq

    def haptic_snapshot(self) -> dict:
        with self.lock:
            return {
                "seq": self.haptic_seq,
                "intensity": self.haptic_intensity,
                "duration_ms": self.haptic_duration_ms,
                "reason": self.haptic_reason,
            }


def recv_exact(conn, nbytes: int) -> bytes:
    chunks = []
    remaining = nbytes
    while remaining:
        chunk = conn.recv(remaining)
        if not chunk:
            raise EOFError("websocket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_ws_text(conn) -> str:
    header = recv_exact(conn, 2)
    b0, b1 = header
    opcode = b0 & 0x0F
    masked = b1 & 0x80
    length = b1 & 0x7F
    if length == 126:
        length = struct.unpack(">H", recv_exact(conn, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", recv_exact(conn, 8))[0]
    mask = recv_exact(conn, 4) if masked else b""
    payload = bytearray(recv_exact(conn, length))
    if masked:
        for i in range(length):
            payload[i] ^= mask[i % 4]
    if opcode == 8:
        raise EOFError("websocket close frame")
    if opcode != 1:
        return ""
    return payload.decode("utf-8", errors="replace")


def send_ws_text(conn, text: str) -> None:
    payload = text.encode("utf-8")
    header = bytearray([0x81])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(struct.pack(">H", length))
    else:
        header.append(127)
        header.extend(struct.pack(">Q", length))
    conn.sendall(bytes(header) + payload)


def make_handler(state: TeleopState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args) -> None:  # noqa: A002
            return

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/":
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/debug":
                body = DEBUG_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/status":
                body = json.dumps(state.snapshot(), separators=(",", ":")).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/history":
                body = json.dumps(state.history_snapshot(), separators=(",", ":")).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/ws":
                self.handle_websocket()
                return
            self.send_error(404)

        def handle_websocket(self) -> None:
            key = self.headers.get("Sec-WebSocket-Key")
            if not key:
                self.send_error(400, "Missing websocket key")
                return
            accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
            self.send_response(101)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()
            state.websocket_connected()
            print("Quest WebSocket connected")
            last_haptic_seq = 0
            try:
                while True:
                    text = recv_ws_text(self.connection)
                    if not text:
                        continue
                    try:
                        state.update(json.loads(text))
                    except json.JSONDecodeError:
                        continue
                    haptic = state.haptic_snapshot()
                    if haptic["seq"] > last_haptic_seq:
                        last_haptic_seq = haptic["seq"]
                        send_ws_text(
                            self.connection,
                            json.dumps(
                                {
                                    "type": "haptic",
                                    "seq": haptic["seq"],
                                    "intensity": haptic["intensity"],
                                    "duration_ms": haptic["duration_ms"],
                                    "reason": haptic["reason"],
                                },
                                separators=(",", ":"),
                            ),
                        )
            except EOFError:
                print("Quest WebSocket disconnected")
            except OSError:
                print("Quest WebSocket socket closed")
            finally:
                state.websocket_disconnected()

    return Handler


def get_lan_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def ensure_cert(host_ip: str) -> None:
    if CERT_PATH.exists() and KEY_PATH.exists():
        return
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        "openssl",
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-nodes",
        "-keyout",
        str(KEY_PATH),
        "-out",
        str(CERT_PATH),
        "-days",
        "30",
        "-subj",
        "/CN=quest-webxr.local",
        "-addext",
        f"subjectAltName=IP:{host_ip},DNS:localhost",
    ]
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError as exc:
        raise SystemExit("openssl is required to create a local HTTPS cert for Quest WebXR.") from exc
    os.chmod(KEY_PATH, 0o600)


def serve_https(host: str, port: int, advertise_ip: str, state: TeleopState) -> ThreadingHTTPServer:
    lan_ip = advertise_ip or get_lan_ip()
    ensure_cert(lan_ip)
    server = ThreadingHTTPServer((host, port), make_handler(state))
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=str(CERT_PATH), keyfile=str(KEY_PATH))
    server.socket = context.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Quest WebXR page: https://{lan_ip}:{port}/")
    print("Open this URL in Quest Browser. Accept the local certificate warning if prompted.")
    return server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--advertise-ip", default="", help="LAN IP shown for the Quest URL. Default auto-detects.")
    parser.add_argument("--run-seconds", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--record", action="store_true", help="Record Quest controller samples to captures/quest_tracking_*.")
    parser.add_argument("--out", default="captures")
    parser.add_argument("--motion-threshold-m", type=float, default=0.004)
    parser.add_argument("--idle-split-s", type=float, default=0.35)
    args = parser.parse_args()

    recorder = (
        MotionRecorder(Path(args.out), args.motion_threshold_m, args.idle_split_s)
        if args.record
        else None
    )
    state = TeleopState(recorder=recorder)
    server = serve_https(args.host, args.port, args.advertise_ip, state)
    started = time.time()
    try:
        while args.run_seconds <= 0 or time.time() - started < args.run_seconds:
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    main()
