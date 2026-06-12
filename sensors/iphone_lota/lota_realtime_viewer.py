#!/usr/bin/env python3
"""Low-latency browser viewer for LOTA TCP depth and PLY streams.

LOTA's TCP ports are mode-dependent:

- Point Cloud mode: TCP 9847 Float32 LiDAR depth, TCP 9848 PLY XYZ/RGB.
- Depth Image mode: TCP 9847 Float32 LiDAR depth.
- Color mode: TCP 9847 H264 color video, not Float32 depth.
- Neural Depth mode: NDI only, no TCP 9847/9848 payload.
- Motion mode: IMU/compass/pressure data, not parsed by this viewer yet.

OSC camera pose is separate UDP data, usually on port 9000. This viewer can
listen for it in parallel and show the latest pose in the browser status bar.

Unlike the Plotly viewer, this sends compact binary frames and always overwrites
old frames with the newest one. That keeps visualization latency low.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

from lota_osc_receiver import parse_osc_message, update_state as update_osc_state
from lota_receiver import PLY_POINT_BYTES, depth_stats, read_lota_depth_frame, recv_exact


PLY_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


@dataclass
class Frame:
    index: int
    kind: str
    meta: dict
    payload: bytes


class LatestFrameStore:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame: Frame | None = None
        self.index = 0
        self.started_at = time.time()
        self.lota_mode = "unknown"
        self.depth_peer: str | None = None
        self.ply_peer: str | None = None
        self.osc_peer: str | None = None
        self.depth_error: str | None = None
        self.ply_error: str | None = None
        self.osc_error: str | None = None
        self.osc_packets = 0
        self.osc_state: dict = {}

    def set_lota_mode(self, mode: str) -> None:
        with self.lock:
            self.lota_mode = mode

    def set_peer(self, kind: str, peer: str | None) -> None:
        with self.lock:
            if kind == "depth":
                self.depth_peer = peer
            else:
                self.ply_peer = peer

    def set_error(self, kind: str, error: str | None) -> None:
        with self.lock:
            if kind == "depth":
                self.depth_error = error
            elif kind == "ply":
                self.ply_error = error
            else:
                self.osc_error = error

    def update_osc(self, peer: str, messages: list[dict]) -> None:
        with self.lock:
            self.osc_peer = peer
            self.osc_packets += 1
            self.osc_state["last_messages"] = messages
            for msg in messages:
                update_osc_state(self.osc_state, msg)
            self.osc_state["time"] = time.time()
            self.osc_state["packets"] = self.osc_packets

    def update(self, kind: str, meta: dict, payload: bytes) -> int:
        with self.lock:
            self.index += 1
            meta = dict(meta)
            meta["index"] = self.index
            meta["kind"] = kind
            meta["time"] = time.time()
            meta["depth_peer"] = self.depth_peer
            meta["ply_peer"] = self.ply_peer
            meta["osc_peer"] = self.osc_peer
            meta["depth_error"] = self.depth_error
            meta["ply_error"] = self.ply_error
            meta["osc_error"] = self.osc_error
            meta["osc"] = dict(self.osc_state)
            self.frame = Frame(self.index, kind, meta, payload)
            return self.index

    def snapshot(self) -> Frame | None:
        with self.lock:
            if self.frame is None:
                return None
            return Frame(self.frame.index, self.frame.kind, dict(self.frame.meta), bytes(self.frame.payload))

    def status(self) -> dict:
        with self.lock:
            return {
                "uptime_s": time.time() - self.started_at,
                "lota_mode": self.lota_mode,
                "latest_index": self.index,
                "latest_kind": self.frame.kind if self.frame else None,
                "latest_time": self.frame.meta.get("time") if self.frame else None,
                "depth_peer": self.depth_peer,
                "ply_peer": self.ply_peer,
                "osc_peer": self.osc_peer,
                "depth_error": self.depth_error,
                "ply_error": self.ply_error,
                "osc_error": self.osc_error,
                "osc": dict(self.osc_state),
            }


def tcp_server(host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    return server


def accept_connection(server: socket.socket, label: str) -> socket.socket:
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"{label}: connected from {addr[0]}:{addr[1]}")
    return conn


def osc_receiver(args: argparse.Namespace, store: LatestFrameStore) -> None:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.host, args.osc_port))
        sock.settimeout(0.5)
    except OSError as exc:
        store.set_error("osc", f"bind failed: {exc}")
        print(f"osc: bind failed on {args.host}:{args.osc_port}: {exc}")
        return

    print(f"osc: listening on UDP {args.host}:{args.osc_port}")
    last_print = 0.0
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception as exc:
                store.set_error("osc", str(exc))
                time.sleep(0.25)
                continue
            try:
                messages = parse_osc_message(data)
            except Exception as exc:
                store.set_error("osc", f"parse failed: {exc}")
                continue
            peer = f"{addr[0]}:{addr[1]}"
            store.update_osc(peer, messages)
            store.set_error("osc", None)
            now = time.time()
            if now - last_print >= args.print_every:
                last_print = now
                state = store.status().get("osc", {})
                print(
                    "osc",
                    "pos=", state.get("position"),
                    "quat=", state.get("rotation_quat"),
                    "euler=", state.get("euler"),
                    "fps=", state.get("fps"),
                    "mode=", state.get("mode"),
                )
    finally:
        sock.close()


def depth_to_u8(depth: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0.0)
    clipped = np.clip(depth, min_depth, max_depth)
    gray = 255.0 * (1.0 - (clipped - min_depth) / max(1e-6, max_depth - min_depth))
    image = np.zeros(depth.shape, dtype=np.uint8)
    image[valid] = gray[valid].astype(np.uint8)
    return image


def depth_receiver(args: argparse.Namespace, store: LatestFrameStore) -> None:
    try:
        server = tcp_server(args.host, args.depth_port)
    except OSError as exc:
        store.set_error("depth", f"bind failed: {exc}")
        print(f"depth: bind failed on {args.host}:{args.depth_port}: {exc}")
        return

    print(f"depth: listening on {args.host}:{args.depth_port}")
    conn: socket.socket | None = None
    last_t = time.time()
    last_print = 0.0
    fps = 0.0
    try:
        while True:
            if conn is None:
                print("depth: waiting for LOTA TCP depth/mode connection...")
                store.set_peer("depth", None)
                conn = accept_connection(server, "depth")
                store.set_peer("depth", str(conn.getpeername()))
            try:
                depth, frame_header = read_lota_depth_frame(conn)
            except EOFError:
                print("depth: peer disconnected before complete frame")
                store.set_error("depth", "peer disconnected")
                conn.close()
                conn = None
                continue
            except Exception as exc:
                print(f"depth: receiver error: {exc}")
                store.set_error("depth", str(exc))
                conn.close()
                conn = None
                time.sleep(0.25)
                continue

            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.85 * fps + 0.15 * (1.0 / dt) if fps > 0 else 1.0 / dt
            stats = depth_stats(depth)
            image = depth_to_u8(depth, args.min_depth, args.max_depth)
            meta = {
                "width": int(depth.shape[1]),
                "height": int(depth.shape[0]),
                "fps": fps,
                "stats": stats,
                "frame_header": frame_header,
                "min_depth": args.min_depth,
                "max_depth": args.max_depth,
            }
            idx = store.update("depth", meta, image.tobytes())
            store.set_error("depth", None)
            if now - last_print >= args.print_every:
                last_print = now
                print(
                    f"depth frame={idx} {depth.shape[1]}x{depth.shape[0]} fps={fps:.1f} "
                    f"valid={stats['valid_ratio']:.2f} median={stats['median_m']}"
                )
    finally:
        if conn is not None:
            conn.close()
        server.close()


def h264_status_receiver(args: argparse.Namespace, store: LatestFrameStore) -> None:
    """Receive color-mode TCP bytes on 9847 and report throughput.

    LOTA Color mode sends H264 on the same port used by Float32 depth in other
    modes. We do not decode it here yet; this receiver exists so camera mode
    does not get misparsed as depth and so the UI can show that H264 is alive.
    """

    try:
        server = tcp_server(args.host, args.depth_port)
    except OSError as exc:
        store.set_error("depth", f"bind failed: {exc}")
        print(f"h264: bind failed on {args.host}:{args.depth_port}: {exc}")
        return

    print(f"h264: listening on {args.host}:{args.depth_port}")
    conn: socket.socket | None = None
    total_bytes = 0
    window_bytes = 0
    last_t = time.time()
    last_print = 0.0
    mbps = 0.0
    try:
        while True:
            if conn is None:
                print("h264: waiting for LOTA Color-mode TCP connection...")
                store.set_peer("depth", None)
                conn = accept_connection(server, "h264")
                store.set_peer("depth", str(conn.getpeername()))
                total_bytes = 0
                window_bytes = 0
                last_t = time.time()
            try:
                chunk = conn.recv(args.h264_chunk_bytes)
                if not chunk:
                    raise EOFError("connection closed")
            except EOFError:
                print("h264: peer disconnected")
                store.set_error("depth", "peer disconnected")
                conn.close()
                conn = None
                continue
            except Exception as exc:
                print(f"h264: receiver error: {exc}")
                store.set_error("depth", str(exc))
                conn.close()
                conn = None
                time.sleep(0.25)
                continue

            now = time.time()
            total_bytes += len(chunk)
            window_bytes += len(chunk)
            dt = now - last_t
            if dt >= 0.25:
                instant_mbps = (window_bytes * 8.0) / max(dt, 1e-6) / 1_000_000.0
                mbps = 0.85 * mbps + 0.15 * instant_mbps if mbps > 0 else instant_mbps
                window_bytes = 0
                last_t = now
                meta = {
                    "fps": 0.0,
                    "bytes_total": int(total_bytes),
                    "mbps": mbps,
                    "note": "LOTA Color mode H264 detected on TCP 9847; video decode/display is not implemented in this viewer yet.",
                }
                idx = store.update("h264", meta, b"")
                store.set_error("depth", None)
                if now - last_print >= args.print_every:
                    last_print = now
                    print(f"h264 frame={idx} total_bytes={total_bytes} throughput={mbps:.2f} Mbps")
    finally:
        if conn is not None:
            conn.close()
        server.close()


def recv_ply_cloud(conn: socket.socket, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    count_raw = recv_exact(conn, 4)
    point_count = struct.unpack("<I", count_raw)[0]
    if point_count > max_points:
        raise ValueError(f"Refusing huge point frame: {point_count} > {max_points}")
    raw = recv_exact(conn, point_count * PLY_POINT_BYTES)
    records = np.frombuffer(raw, dtype=PLY_DTYPE, count=point_count)
    return records["xyz"].copy(), records["rgb"].copy()


def prepare_ply_payload(points: np.ndarray, colors: np.ndarray, args: argparse.Namespace) -> tuple[dict, bytes]:
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]

    if len(points) == 0:
        meta = {
            "point_count": 0,
            "raw_points": 0,
            "kept_points": 0,
            "front_pos_z": 0,
            "front_neg_z": 0,
            "camera_forward": args.camera_forward,
            "ranges": {},
        }
        return meta, b""

    depth = -points[:, 2] if args.camera_forward == "neg-z" else points[:, 2]
    front_pos_z = int(np.count_nonzero((points[:, 2] >= args.min_depth) & (points[:, 2] <= args.max_depth)))
    front_neg_z = int(np.count_nonzero((-points[:, 2] >= args.min_depth) & (-points[:, 2] <= args.max_depth)))
    mask = (depth >= args.min_depth) & (depth <= args.max_depth)
    raw_count = len(points)
    points = points[mask]
    colors = colors[mask]
    depth = depth[mask]
    kept_count = len(points)

    if args.max_display_points > 0 and len(points) > args.max_display_points:
        # Even spacing is deterministic and cheap; the important latency win is
        # that we send only the latest sampled frame to the browser.
        idx = np.linspace(0, len(points) - 1, args.max_display_points).astype(np.int64)
        points = points[idx]
        colors = colors[idx]
        depth = depth[idx]

    points = np.ascontiguousarray(points.astype("<f4"))
    colors = np.ascontiguousarray(colors.astype("u1"))
    if len(points):
        ranges = {
            "x": [float(np.percentile(points[:, 0], 1)), float(np.percentile(points[:, 0], 99))],
            "y": [float(np.percentile(points[:, 1], 1)), float(np.percentile(points[:, 1], 99))],
            "z": [float(np.percentile(points[:, 2], 1)), float(np.percentile(points[:, 2], 99))],
            "depth": [float(depth.min()), float(depth.max())],
        }
    else:
        ranges = {}
    meta = {
        "point_count": int(len(points)),
        "raw_points": int(raw_count),
        "kept_points": int(kept_count),
        "front_pos_z": front_pos_z,
        "front_neg_z": front_neg_z,
        "camera_forward": args.camera_forward,
        "ranges": ranges,
        "xyz_bytes": int(points.nbytes),
        "rgb_bytes": int(colors.nbytes),
    }
    return meta, points.tobytes() + colors.tobytes()


def ply_receiver(args: argparse.Namespace, store: LatestFrameStore) -> None:
    try:
        server = tcp_server(args.host, args.ply_port)
    except OSError as exc:
        store.set_error("ply", f"bind failed: {exc}")
        print(f"ply: bind failed on {args.host}:{args.ply_port}: {exc}")
        return

    print(f"ply: listening on {args.host}:{args.ply_port}")
    conn: socket.socket | None = None
    last_t = time.time()
    last_print = 0.0
    fps = 0.0
    try:
        while True:
            if conn is None:
                print("ply: waiting for LOTA PLY connection...")
                store.set_peer("ply", None)
                conn = accept_connection(server, "ply")
                store.set_peer("ply", str(conn.getpeername()))
            try:
                points, colors = recv_ply_cloud(conn, args.max_points)
            except EOFError:
                print("ply: peer disconnected before complete frame")
                store.set_error("ply", "peer disconnected")
                conn.close()
                conn = None
                continue
            except Exception as exc:
                print(f"ply: receiver error: {exc}")
                store.set_error("ply", str(exc))
                conn.close()
                conn = None
                time.sleep(0.25)
                continue

            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.85 * fps + 0.15 * (1.0 / dt) if fps > 0 else 1.0 / dt
            meta, payload = prepare_ply_payload(points, colors, args)
            meta["fps"] = fps
            idx = store.update("ply", meta, payload)
            store.set_error("ply", None)
            if now - last_print >= args.print_every:
                last_print = now
                depth = meta.get("ranges", {}).get("depth")
                depth_text = f"{depth[0]:.3f}..{depth[1]:.3f}" if depth else "none"
                print(
                    f"ply frame={idx} raw={meta['raw_points']} kept={meta['kept_points']} "
                    f"shown={meta['point_count']} fps={fps:.1f} depth={depth_text} "
                    f"+z={meta['front_pos_z']} -z={meta['front_neg_z']}"
                )
    finally:
        if conn is not None:
            conn.close()
        server.close()


def pack_frame(frame: Frame) -> bytes:
    header = json.dumps(frame.meta, separators=(",", ":")).encode("utf-8")
    prefix = struct.pack("<I", len(header)) + header
    pad = b"\0" * ((4 - (len(prefix) % 4)) % 4)
    return prefix + pad + frame.payload


def make_index_html(args: argparse.Namespace) -> bytes:
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>LOTA Realtime Viewer</title>
  <style>
    html, body {{ margin: 0; height: 100%; background: #060606; color: #eee; font-family: system-ui, sans-serif; overflow: hidden; }}
    #bar {{ height: 44px; display: flex; align-items: center; gap: 14px; padding: 0 12px; background: #101010; font-size: 13px; white-space: nowrap; overflow: hidden; }}
    #depth, #gl {{ position: absolute; top: 44px; left: 0; width: 100vw; height: calc(100vh - 44px); }}
    #depth {{ image-rendering: pixelated; display: none; }}
    #gl {{ display: none; }}
    .muted {{ color: #aaa; }}
  </style>
</head>
<body>
  <div id="bar">
    <strong>LOTA Realtime</strong>
    <span id="mode" class="muted">waiting</span>
    <span id="status" class="muted">starting...</span>
  </div>
  <canvas id="depth"></canvas>
  <canvas id="gl"></canvas>
  <script>
    const fpsLimit = {args.browser_fps};
    const depthCanvas = document.getElementById("depth");
    const depthCtx = depthCanvas.getContext("2d");
    const glCanvas = document.getElementById("gl");
    const modeEl = document.getElementById("mode");
    const statusEl = document.getElementById("status");
    const decoder = new TextDecoder();
    let lastIndex = 0;
    let rotX = -0.75, rotY = 0.45, zoom = 1.0;
    let dragging = false, lastMouseX = 0, lastMouseY = 0;

    function resize() {{
      const w = window.innerWidth;
      const h = window.innerHeight - 44;
      depthCanvas.style.width = w + "px";
      depthCanvas.style.height = h + "px";
      glCanvas.width = w * devicePixelRatio;
      glCanvas.height = h * devicePixelRatio;
      glCanvas.style.width = w + "px";
      glCanvas.style.height = h + "px";
      if (gl) gl.viewport(0, 0, glCanvas.width, glCanvas.height);
    }}
    window.addEventListener("resize", resize);

    const gl = glCanvas.getContext("webgl", {{antialias: false, preserveDrawingBuffer: false}});
    function compileShader(type, source) {{
      const shader = gl.createShader(type);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
      return shader;
    }}
    const vs = compileShader(gl.VERTEX_SHADER, `
      attribute vec3 aPosition;
      attribute vec3 aColor;
      uniform vec3 uCenter;
      uniform float uScale;
      uniform float uRotX;
      uniform float uRotY;
      uniform float uZoom;
      uniform float uPointSize;
      varying vec3 vColor;
      void main() {{
        vec3 p = (aPosition - uCenter) * uScale * uZoom;
        float cx = cos(uRotX), sx = sin(uRotX);
        float cy = cos(uRotY), sy = sin(uRotY);
        p = vec3(p.x, cx * p.y - sx * p.z, sx * p.y + cx * p.z);
        p = vec3(cy * p.x + sy * p.z, p.y, -sy * p.x + cy * p.z);
        gl_Position = vec4(p.x, p.y, p.z, 1.0);
        gl_PointSize = uPointSize;
        vColor = aColor / 255.0;
      }}
    `);
    const fs = compileShader(gl.FRAGMENT_SHADER, `
      precision mediump float;
      varying vec3 vColor;
      void main() {{
        gl_FragColor = vec4(vColor, 1.0);
      }}
    `);
    const program = gl.createProgram();
    gl.attachShader(program, vs);
    gl.attachShader(program, fs);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
    gl.useProgram(program);
    const posLoc = gl.getAttribLocation(program, "aPosition");
    const colorLoc = gl.getAttribLocation(program, "aColor");
    const centerLoc = gl.getUniformLocation(program, "uCenter");
    const scaleLoc = gl.getUniformLocation(program, "uScale");
    const rotXLoc = gl.getUniformLocation(program, "uRotX");
    const rotYLoc = gl.getUniformLocation(program, "uRotY");
    const zoomLoc = gl.getUniformLocation(program, "uZoom");
    const pointSizeLoc = gl.getUniformLocation(program, "uPointSize");
    const posBuffer = gl.createBuffer();
    const colorBuffer = gl.createBuffer();
    let pointCount = 0;
    let center = [0, 0, 0];
    let scale = 1;

    function render3d() {{
      glCanvas.style.display = "block";
      depthCanvas.style.display = "none";
      gl.clearColor(0.02, 0.02, 0.02, 1.0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      gl.useProgram(program);
      gl.uniform3f(centerLoc, center[0], center[1], center[2]);
      gl.uniform1f(scaleLoc, scale);
      gl.uniform1f(rotXLoc, rotX);
      gl.uniform1f(rotYLoc, rotY);
      gl.uniform1f(zoomLoc, zoom);
      gl.uniform1f(pointSizeLoc, {args.point_size});
      gl.bindBuffer(gl.ARRAY_BUFFER, posBuffer);
      gl.enableVertexAttribArray(posLoc);
      gl.vertexAttribPointer(posLoc, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
      gl.enableVertexAttribArray(colorLoc);
      gl.vertexAttribPointer(colorLoc, 3, gl.UNSIGNED_BYTE, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, pointCount);
    }}

    function update3d(meta, payload, payloadOffset) {{
      const n = meta.point_count;
      const xyzBytes = meta.xyz_bytes;
      const xyz = new Float32Array(payload, payloadOffset, n * 3);
      const rgb = new Uint8Array(payload, payloadOffset + xyzBytes, n * 3);
      pointCount = n;
      gl.bindBuffer(gl.ARRAY_BUFFER, posBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, xyz, gl.DYNAMIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, rgb, gl.DYNAMIC_DRAW);
      const r = meta.ranges || {{}};
      if (r.x && r.y && r.z) {{
        center = [(r.x[0] + r.x[1]) / 2, (r.y[0] + r.y[1]) / 2, (r.z[0] + r.z[1]) / 2];
        const span = Math.max(r.x[1] - r.x[0], r.y[1] - r.y[0], r.z[1] - r.z[0], 0.01);
        scale = 1.65 / span;
      }}
      render3d();
    }}

    glCanvas.addEventListener("mousedown", e => {{ dragging = true; lastMouseX = e.clientX; lastMouseY = e.clientY; }});
    window.addEventListener("mouseup", () => dragging = false);
    window.addEventListener("mousemove", e => {{
      if (!dragging) return;
      rotY += (e.clientX - lastMouseX) * 0.006;
      rotX += (e.clientY - lastMouseY) * 0.006;
      lastMouseX = e.clientX; lastMouseY = e.clientY;
      if (pointCount) render3d();
    }});
    glCanvas.addEventListener("wheel", e => {{
      e.preventDefault();
      zoom *= Math.exp(-e.deltaY * 0.001);
      zoom = Math.max(0.2, Math.min(8, zoom));
      if (pointCount) render3d();
    }}, {{passive: false}});

    function colorize(v) {{
      // Small blue->cyan->green->yellow->red ramp; v is 0..255, closer is brighter.
      const t = v / 255;
      const r = Math.max(0, Math.min(255, 510 * t - 128));
      const g = Math.max(0, Math.min(255, 510 * (1 - Math.abs(t - 0.5))));
      const b = Math.max(0, Math.min(255, 255 - 510 * t));
      return [r, g, b];
    }}

    function updateDepth(meta, payload, payloadOffset) {{
      glCanvas.style.display = "none";
      depthCanvas.style.display = "block";
      const w = meta.width, h = meta.height;
      depthCanvas.width = w;
      depthCanvas.height = h;
      const src = new Uint8Array(payload, payloadOffset, w * h);
      const img = depthCtx.createImageData(w, h);
      for (let i = 0, j = 0; i < src.length; i++, j += 4) {{
        const [r, g, b] = colorize(src[i]);
        img.data[j] = r; img.data[j + 1] = g; img.data[j + 2] = b; img.data[j + 3] = 255;
      }}
      depthCtx.putImageData(img, 0, 0);
    }}

    function parseFrame(buffer) {{
      const view = new DataView(buffer);
      const headerLen = view.getUint32(0, true);
      const headerBytes = buffer.slice(4, 4 + headerLen);
      const meta = JSON.parse(decoder.decode(headerBytes));
      const payloadOffset = (4 + headerLen + 3) & ~3;
      return [meta, buffer, payloadOffset];
    }}

    function updateStatus(meta) {{
      modeEl.textContent = meta.kind || "waiting";
      if (meta.kind === "h264") {{
        const osc = meta.osc || {{}};
        statusEl.textContent =
          `color/H264 detected on 9847 | throughput ${{(meta.mbps || 0).toFixed(2)}} Mbps | ` +
          `decode/display not implemented yet | peer ${{meta.depth_peer || "none"}} | ` +
          `osc ${{osc.mode || "none"}} pos ${{JSON.stringify(osc.position || null)}}`;
        glCanvas.style.display = "none";
        depthCanvas.style.display = "none";
        return;
      }}
      const d = meta.ranges?.depth;
      const stats = meta.stats;
      const osc = meta.osc || {{}};
      const oscMessages = osc.last_messages ? osc.last_messages.map(m => m.address).join(",") : "none";
      const depthText = d ? `${{d[0].toFixed(3)}}..${{d[1].toFixed(3)}}m` : (stats?.median_m ? `median ${{stats.median_m.toFixed(3)}}m` : "depth none");
      statusEl.textContent =
        `frame ${{meta.index}} | ${{depthText}} | fps ${{(meta.fps || 0).toFixed(1)}} | ` +
        `depthPeer ${{meta.depth_peer || "none"}} plyPeer ${{meta.ply_peer || "none"}} | ` +
        `oscMode ${{osc.mode || "none"}} oscFps ${{osc.fps || "none"}} pos ${{JSON.stringify(osc.position || null)}} oscMsg ${{oscMessages}}`;
    }}

    async function poll() {{
      try {{
        const res = await fetch(`/frame.bin?last=${{lastIndex}}`, {{cache: "no-store"}});
        if (res.status === 204) return;
        if (!res.ok) throw new Error(`HTTP ${{res.status}}`);
        const buffer = await res.arrayBuffer();
        const [meta, payload, payloadOffset] = parseFrame(buffer);
        lastIndex = meta.index;
        if (meta.kind === "depth") updateDepth(meta, payload, payloadOffset);
        if (meta.kind === "ply") update3d(meta, payload, payloadOffset);
        updateStatus(meta);
      }} catch (err) {{
        const res = await fetch("/status", {{cache: "no-store"}});
        const status = await res.json();
        modeEl.textContent = "waiting";
        const oscMessages = status.osc?.last_messages ? status.osc.last_messages.map(m => m.address).join(",") : "none";
        statusEl.textContent =
          `waiting | mode ${{status.lota_mode || "unknown"}} | latest ${{status.latest_kind || "none"}} #${{status.latest_index}} | ` +
          `depthPeer ${{status.depth_peer || "none"}} plyPeer ${{status.ply_peer || "none"}} oscPeer ${{status.osc_peer || "none"}} | ` +
          `depthErr ${{status.depth_error || "none"}} plyErr ${{status.ply_error || "none"}} oscErr ${{status.osc_error || "none"}} oscMsg ${{oscMessages}}`;
      }}
    }}

    async function loop() {{
      await poll();
      setTimeout(loop, 1000 / fpsLimit);
    }}
    resize();
    loop();
  </script>
</body>
</html>
"""
    return html.encode("utf-8")


def make_handler(args: argparse.Namespace, store: LatestFrameStore):
    index_html = make_index_html(args)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *values) -> None:  # noqa: A002
            if args.http_log:
                super().log_message(format, *values)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(index_html)))
                self.end_headers()
                self.wfile.write(index_html)
                return
            if path == "/status":
                payload = json.dumps(store.status(), separators=(",", ":")).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if path == "/frame.bin":
                query = parse_qs(urlparse(self.path).query)
                last = int(query.get("last", ["0"])[0])
                frame = store.snapshot()
                if frame is None or frame.index <= last:
                    self.send_response(204)
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    return
                payload = pack_frame(frame)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            self.send_error(404)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--depth-port", type=int, default=9847)
    parser.add_argument("--ply-port", type=int, default=9848)
    parser.add_argument("--osc-port", type=int, default=9000)
    parser.add_argument("--no-osc", action="store_true", help="Do not listen for OSC UDP pose/status.")
    parser.add_argument(
        "--lota-mode",
        choices=["point-cloud", "depth-image", "color", "camera", "neural-depth", "motion", "custom"],
        default="point-cloud",
        help="LOTA app mode. Use custom to manually control --streams.",
    )
    parser.add_argument("--streams", choices=["both", "depth", "ply"], default="both")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--camera-forward", choices=["neg-z", "pos-z"], default="neg-z")
    parser.add_argument("--min-depth", type=float, default=0.05)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    parser.add_argument("--max-display-points", type=int, default=8_000)
    parser.add_argument("--h264-chunk-bytes", type=int, default=64_000)
    parser.add_argument("--browser-fps", type=float, default=12.0)
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--http-log", action="store_true")
    args = parser.parse_args()

    store = LatestFrameStore()
    store.set_lota_mode(args.lota_mode)
    if not args.no_osc:
        threading.Thread(target=osc_receiver, args=(args, store), daemon=True).start()
    active_streams = args.streams
    if args.lota_mode == "point-cloud":
        active_streams = "both"
    elif args.lota_mode == "depth-image":
        active_streams = "depth"
    elif args.lota_mode in {"color", "camera"}:
        if args.lota_mode == "camera":
            print("Note: LOTA calls this mode 'color'; 'camera' is accepted as a compatibility alias.")
        active_streams = "camera"
    elif args.lota_mode in {"neural-depth", "motion"}:
        active_streams = "none"

    if active_streams in {"both", "depth"}:
        threading.Thread(target=depth_receiver, args=(args, store), daemon=True).start()
    if active_streams == "camera":
        threading.Thread(target=h264_status_receiver, args=(args, store), daemon=True).start()
    if active_streams in {"both", "ply"}:
        threading.Thread(target=ply_receiver, args=(args, store), daemon=True).start()
    if active_streams == "none":
        print(f"{args.lota_mode}: no TCP 9847/9848 receiver started by this viewer.")
        print("Neural Depth uses NDI; Motion needs a separate sensor parser.")

    httpd = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(args, store))
    print(f"Realtime browser viewer: http://127.0.0.1:{args.http_port}/")
    print("Open this URL in Windows/Chrome/Edge. Start or toggle LOTA streaming after this is running.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("Stopping realtime viewer")
    finally:
        httpd.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    main()
