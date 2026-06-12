#!/usr/bin/env python3
"""Browser/WebGL live 3D viewer for LOTA TCP PLY point clouds.

Open3D needs a working OpenGL desktop context, which can be unreliable under
WSL. This viewer keeps the sensor receiver in Python and renders in the browser
with Plotly WebGL over a local Server-Sent Events stream.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from plotly.offline import get_plotlyjs

from lota_receiver import PLY_POINT_BYTES, recv_exact


PLY_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


class FrameStore:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame: dict | None = None
        self.frame_idx = 0
        self.connected_peer: str | None = None
        self.last_error: str | None = None
        self.started_at = time.time()

    def set_peer(self, peer: str | None) -> None:
        with self.lock:
            self.connected_peer = peer

    def set_error(self, error: str | None) -> None:
        with self.lock:
            self.last_error = error

    def update(self, frame: dict) -> None:
        with self.lock:
            self.frame_idx += 1
            frame["type"] = "frame"
            frame["frame_idx"] = self.frame_idx
            frame["peer"] = self.connected_peer
            frame["error"] = self.last_error
            self.frame = frame

    def frame_snapshot(self) -> dict | None:
        with self.lock:
            if self.frame is None:
                return None
            return dict(self.frame)

    def status_snapshot(self) -> dict:
        with self.lock:
            status = "streaming" if self.frame is not None else "waiting_for_ply"
            return {
                "type": "status",
                "status": status,
                "frame_idx": self.frame_idx,
                "peer": self.connected_peer,
                "error": self.last_error,
                "uptime_s": time.time() - self.started_at,
                "last_frame_time": self.frame.get("time") if self.frame is not None else None,
                "shown_points": self.frame.get("shown_points") if self.frame is not None else None,
                "kept_points": self.frame.get("kept_points") if self.frame is not None else None,
                "raw_points": self.frame.get("raw_points") if self.frame is not None else None,
                "recv_fps": self.frame.get("recv_fps") if self.frame is not None else None,
                "camera_forward": self.frame.get("camera_forward") if self.frame is not None else None,
                "front_pos_z": self.frame.get("front_pos_z") if self.frame is not None else None,
                "front_neg_z": self.frame.get("front_neg_z") if self.frame is not None else None,
                "ranges": self.frame.get("ranges") if self.frame is not None else None,
            }


def open_server(host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    return server


def accept_connection(server: socket.socket) -> socket.socket:
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connected from {addr[0]}:{addr[1]}")
    return conn


def recv_cloud(conn: socket.socket, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    count_raw = recv_exact(conn, 4)
    point_count = struct.unpack("<I", count_raw)[0]
    if point_count > max_points:
        raise ValueError(f"Refusing huge point frame: {point_count} > {max_points}")
    raw = recv_exact(conn, point_count * PLY_POINT_BYTES)
    records = np.frombuffer(raw, dtype=PLY_DTYPE, count=point_count)
    return records["xyz"].copy(), records["rgb"].copy()


def rgb_hex(colors: np.ndarray) -> list[str]:
    colors = colors.astype(np.uint8)
    return [f"#{r:02x}{g:02x}{b:02x}" for r, g, b in colors]


def prepare_frame(
    points: np.ndarray,
    colors: np.ndarray,
    args: argparse.Namespace,
    raw_point_count: int,
    recv_fps: float,
) -> dict:
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]

    depth = -points[:, 2] if args.camera_forward == "neg-z" else points[:, 2]
    front_pos_z = int(np.count_nonzero((points[:, 2] >= args.min_depth) & (points[:, 2] <= args.max_depth)))
    front_neg_z = int(np.count_nonzero((-points[:, 2] >= args.min_depth) & (-points[:, 2] <= args.max_depth)))
    mask = (depth >= args.min_depth) & (depth <= args.max_depth)
    points = points[mask]
    colors = colors[mask]
    depth = depth[mask]
    kept_count = len(points)

    if args.max_display_points > 0 and len(points) > args.max_display_points:
        # Deterministic spatially-neutral thinning. Fast enough for live use.
        idx = np.linspace(0, len(points) - 1, args.max_display_points).astype(np.int64)
        points = points[idx]
        colors = colors[idx]
        depth = depth[idx]

    if len(points) == 0:
        ranges = {}
    else:
        ranges = {
            "x": [float(np.percentile(points[:, 0], 1)), float(np.percentile(points[:, 0], 99))],
            "y": [float(np.percentile(points[:, 1], 1)), float(np.percentile(points[:, 1], 99))],
            "z": [float(np.percentile(points[:, 2], 1)), float(np.percentile(points[:, 2], 99))],
            "depth": [float(depth.min()), float(depth.max())],
        }

    return {
        "time": time.time(),
        "raw_points": int(raw_point_count),
        "kept_points": int(kept_count),
        "shown_points": int(len(points)),
        "recv_fps": recv_fps,
        "camera_forward": args.camera_forward,
        "front_pos_z": front_pos_z,
        "front_neg_z": front_neg_z,
        "ranges": ranges,
        "x": np.round(points[:, 0], args.round_digits).tolist(),
        "y": np.round(points[:, 1], args.round_digits).tolist(),
        "z": np.round(points[:, 2], args.round_digits).tolist(),
        "color": rgb_hex(colors),
    }


def receiver_loop(args: argparse.Namespace, store: FrameStore) -> None:
    server = open_server(args.ply_host, args.ply_port)
    print(f"Listening for LOTA PLY point cloud on {args.ply_host}:{args.ply_port}")
    conn: socket.socket | None = None
    last_t = time.time()
    last_print = 0.0
    recv_fps = 0.0
    try:
        while True:
            if conn is None:
                print("Waiting for LOTA PLY point-cloud connection...")
                store.set_peer(None)
                conn = accept_connection(server)
                store.set_peer(str(conn.getpeername()))
            try:
                points, colors = recv_cloud(conn, args.max_points)
            except EOFError:
                print("Peer disconnected before a complete point-cloud frame")
                store.set_error("peer disconnected")
                conn.close()
                conn = None
                continue
            except Exception as exc:
                print(f"Receiver error: {exc}")
                store.set_error(str(exc))
                if conn is not None:
                    conn.close()
                conn = None
                time.sleep(0.25)
                continue

            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                recv_fps = 0.9 * recv_fps + 0.1 * (1.0 / dt) if recv_fps > 0 else 1.0 / dt
            store.set_error(None)
            frame = prepare_frame(points, colors, args, len(points), recv_fps)
            store.update(frame)
            if now - last_print >= args.print_every:
                last_print = now
                depth = frame.get("ranges", {}).get("depth")
                depth_text = f"{depth[0]:.3f}..{depth[1]:.3f}" if depth else "none"
                print(
                    f"frame={store.frame_idx} raw={frame['raw_points']} kept={frame['kept_points']} "
                    f"shown={frame['shown_points']} fps={recv_fps:.1f} depth={depth_text} "
                    f"+z={frame['front_pos_z']} -z={frame['front_neg_z']} fwd={args.camera_forward}"
                )
    finally:
        if conn is not None:
            conn.close()
        server.close()


def make_index_html(args: argparse.Namespace) -> bytes:
    plotly_js = get_plotlyjs()
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>LOTA Live 3D Point Cloud</title>
  <style>
    html, body {{ margin: 0; height: 100%; background: #070707; color: #eee; font-family: system-ui, sans-serif; }}
    #plot {{ width: 100vw; height: calc(100vh - 42px); }}
    #bar {{ height: 42px; display: flex; align-items: center; gap: 18px; padding: 0 12px; background: #111; font-size: 13px; white-space: nowrap; overflow: hidden; }}
    .muted {{ color: #aaa; }}
  </style>
  <script>{plotly_js}</script>
</head>
<body>
  <div id="bar">
    <strong>LOTA Live 3D</strong>
    <span id="status" class="muted">waiting for frames...</span>
    <span class="muted">mouse: rotate/pan/zoom</span>
  </div>
  <div id="plot"></div>
  <script>
    const plot = document.getElementById("plot");
    const statusEl = document.getElementById("status");
    let initialized = false;
    let lastFrame = -1;

    const layout = {{
      paper_bgcolor: "#070707",
      plot_bgcolor: "#070707",
      margin: {{l: 0, r: 0, t: 0, b: 0}},
      scene: {{
        bgcolor: "#070707",
        aspectmode: "data",
        xaxis: {{title: "PLY X", color: "#ddd", gridcolor: "#333", zerolinecolor: "#666"}},
        yaxis: {{title: "PLY Y", color: "#ddd", gridcolor: "#333", zerolinecolor: "#666"}},
        zaxis: {{title: "PLY Z", color: "#ddd", gridcolor: "#333", zerolinecolor: "#666"}}
      }}
    }};

    function fmt(n, digits = 3) {{
      return Number.isFinite(n) ? n.toFixed(digits) : "none";
    }}

    function updateStatus(msg) {{
      const d = msg.ranges?.depth || [null, null];
      if (msg.status === "waiting_for_ply") {{
        statusEl.textContent =
          `HTTP ready | waiting for LOTA TCP PLY on port {args.ply_port} | peer ${{msg.peer || "none"}} | uptime ${{fmt(msg.uptime_s, 1)}}s`;
        return;
      }}
      statusEl.textContent =
        `frame ${{msg.frame_idx}} | shown ${{msg.shown_points}} / kept ${{msg.kept_points}} / raw ${{msg.raw_points}} | ` +
        `fps ${{fmt(msg.recv_fps, 1)}} | depth ${{fmt(d[0])}}..${{fmt(d[1])}} | ` +
        `fwd ${{msg.camera_forward}} | +z ${{msg.front_pos_z}} -z ${{msg.front_neg_z}} | peer ${{msg.peer || "none"}}`;
    }}

    function updateFrame(frame) {{
      if (frame.frame_idx === lastFrame) return;
      lastFrame = frame.frame_idx;
      const trace = {{
        type: "scatter3d",
        mode: "markers",
        x: frame.x,
        y: frame.y,
        z: frame.z,
        marker: {{
          size: {args.point_size},
          color: frame.color,
          opacity: {args.opacity}
        }}
      }};
      if (!initialized) {{
        Plotly.newPlot(plot, [trace], layout, {{responsive: true, displaylogo: false}});
        initialized = true;
      }} else {{
        Plotly.react(plot, [trace], layout, {{responsive: true, displaylogo: false}});
      }}
      updateStatus(frame);
    }}

    const events = new EventSource("/events");
    events.onopen = () => {{
      statusEl.textContent = "browser connected; waiting for LOTA frames...";
    }};
    events.onmessage = (event) => {{
      const msg = JSON.parse(event.data);
      if (msg.type === "frame") {{
        updateFrame(msg);
      }} else {{
        updateStatus(msg);
      }}
    }};
    events.onerror = () => {{
      statusEl.textContent = "event stream disconnected; browser will retry";
    }};
  </script>
</body>
</html>
"""
    return html.encode("utf-8")


def make_handler(args: argparse.Namespace, store: FrameStore):
    index = make_index_html(args)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *values) -> None:  # noqa: A002
            if args.http_log:
                super().log_message(format, *values)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(index)))
                self.end_headers()
                self.wfile.write(index)
                return
            if path == "/status":
                payload = json.dumps(store.status_snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                last_sent = -1
                last_status = 0.0
                while True:
                    frame = store.frame_snapshot()
                    if frame is not None and frame.get("frame_idx") != last_sent:
                        last_sent = frame["frame_idx"]
                        payload = json.dumps(frame, separators=(",", ":")).encode("utf-8")
                        try:
                            self.wfile.write(b"data: " + payload + b"\n\n")
                            self.wfile.flush()
                        except BrokenPipeError:
                            break
                    elif time.time() - last_status >= 1.0:
                        last_status = time.time()
                        payload = json.dumps(store.status_snapshot(), separators=(",", ":")).encode("utf-8")
                        try:
                            self.wfile.write(b"data: " + payload + b"\n\n")
                            self.wfile.flush()
                        except BrokenPipeError:
                            break
                    time.sleep(max(0.01, 1.0 / args.browser_fps))
                return
            self.send_error(404)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ply-host", default="0.0.0.0")
    parser.add_argument("--ply-port", type=int, default=9848)
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--camera-forward", choices=["neg-z", "pos-z"], default="neg-z")
    parser.add_argument("--min-depth", type=float, default=0.05)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    parser.add_argument("--max-display-points", type=int, default=15_000)
    parser.add_argument("--browser-fps", type=float, default=3.0)
    parser.add_argument("--point-size", type=float, default=1.6)
    parser.add_argument("--opacity", type=float, default=0.95)
    parser.add_argument("--round-digits", type=int, default=4)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--http-log", action="store_true")
    args = parser.parse_args()

    store = FrameStore()
    receiver = threading.Thread(target=receiver_loop, args=(args, store), daemon=True)
    receiver.start()

    httpd = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(args, store))
    print(f"Browser viewer: http://127.0.0.1:{args.http_port}/")
    print("Open this URL in Windows/Chrome/Edge. Ctrl+C stops the server.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("Stopping browser viewer")
    finally:
        httpd.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    main()
