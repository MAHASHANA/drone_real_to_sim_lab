# iPhone 14 Pro RGB-D Streamer

This folder starts the iPhone side of the real-to-sim assembly pipeline.

The MVP is:

```text
iPhone 14 Pro ARKit
  -> RGB JPEG
  -> LiDAR sceneDepth Float32 map
  -> camera intrinsics
  -> camera transform
  -> Flask receiver on WSL
  -> saved RGB/depth/metadata frames
```

Later, SAM/SAM2 runs on the saved or live RGB frames, and each mask is fused
with the depth map to produce one 3D point cloud segment.

## LOTA Receiver

If you are using LOTA instead of the custom ARKit streamer, use
`lota_receiver.py`.

For a live window in WSLg/X11, use `lota_live_viewer.py`.

In LOTA:

- Set `Receiver IP` to your computer's LAN IP, for example `192.168.1.100`.
- For raw depth: use `Depth`, `Point Cloud`, or `Blob Track` mode, enable
  `TCP/UDP Output`, set `Protocol` to `TCP`, and use port `9847`.
- For live point cloud: enable `PLY Streaming` and use port `9848`.
- For camera pose: enable `OSC` and use UDP port `9000`.

On the computer:

```bash
cd /home/satya/ai_agents/drones/drone_real_to_sim_lab/sensors/iphone_lota
python3 lota_receiver.py --mode depth --host 0.0.0.0 --port 9847
```

Live viewer:

```bash
python3 lota_live_viewer.py --host 0.0.0.0 --port 9847 --min-depth 0.2 --max-depth 2.5
```

Press `q` or `Esc` to quit. Press `s` to save the current depth frame and the
colorized view.

Test the viewer window without the phone:

```bash
python3 lota_live_viewer.py --demo
```

### WSL NAT / Windows Wi-Fi Forwarding

If LOTA is on the same Wi-Fi as Windows, but the viewer stays at:

```text
Waiting for LOTA TCP depth on 0.0.0.0:9847
```

then the iPhone is probably sending to Windows, while the Python listener is
inside WSL. Forward the Windows Wi-Fi port into WSL.

Current detected values on this machine:

```text
Windows Wi-Fi IP: 10.0.0.6
WSL eth0 IP:      172.31.204.166
```

Run this in an **Administrator PowerShell** window on Windows:

```powershell
cd \\wsl.localhost\Ubuntu\home\satya\ai_agents\drones\drone_real_to_sim_lab\sensors\iphone_lota
.\tools\setup_lota_wsl_portproxy.ps1
```

Then set LOTA `Receiver IP` to:

```text
10.0.0.6
```

WSL IPs can change after restarting WSL. If it stops working later, run
`hostname -I` inside WSL, update `$WslAddress` in
`tools/setup_lota_wsl_portproxy.ps1`, and rerun the PowerShell script.

For PLY point clouds:

```bash
python3 lota_receiver.py --mode ply --host 0.0.0.0 --port 9848
```

Live point-cloud viewer for LOTA LiDAR/PLY data on port `9848`:

```bash
python3 lota_ply_live_viewer.py --host 0.0.0.0 --port 9848
```

Low-latency browser viewer. Set `--lota-mode` to match the current LOTA app
mode:

- `point-cloud`: TCP `9847` Float32 LiDAR depth + TCP `9848` PLY XYZ/RGB
- `depth-image`: TCP `9847` Float32 LiDAR depth
- `color`: TCP `9847` H264 color video; detected but not decoded yet
- `neural-depth`: NDI only, no TCP `9847/9848`
- `motion`: IMU/compass/pressure; currently handled only as generic OSC/status
  messages if LOTA sends them over OSC

OSC camera pose is separate UDP data, usually on port `9000`. The realtime
viewer listens for it in parallel by default and shows the latest OSC mode,
FPS, and pose in the browser status bar.

```bash
python3 lota_realtime_viewer.py \
  --host 0.0.0.0 \
  --lota-mode point-cloud \
  --depth-port 9847 \
  --ply-port 9848 \
  --osc-port 9000 \
  --http-host 0.0.0.0 \
  --http-port 8765 \
  --camera-forward neg-z
```

Open `http://localhost:8765/` in Windows/Chrome/Edge. The page can open before
the phone connects; start the viewer first, then restart/toggle LOTA
streaming so the iPhone reconnects to the active TCP listener.

For PLY-only lower bandwidth:

```bash
python3 lota_realtime_viewer.py --lota-mode custom --streams ply --max-display-points 5000 --browser-fps 15
```

For Color mode status/throughput:

```bash
python3 lota_realtime_viewer.py --lota-mode color
```

If OSC UDP cannot reach WSL, the visual stream can still work but the OSC
fields will stay empty. Use `tools/lota_windows_udp_forwarder.ps1` or the
Windows OSC logger when running through WSL2 NAT.

The OSC parser preserves unknown addresses in `last_messages`, so Motion-mode
OSC packets can still be inspected even before we add named fields for IMU,
compass, or pressure.

Test the point-cloud viewer window without the phone:

```bash
python3 lota_ply_live_viewer.py --demo
```

If LOTA connects and immediately disconnects, inspect the stream:

```bash
python3 tools/lota_tcp_probe.py --host 0.0.0.0 --port 9848
```

Start LOTA streaming while the probe waits. If the probe receives `0` bytes,
the connection path works but LOTA is not sending the expected TCP payload on
that port/mode.

OSC camera pose logger:

```bash
python3 lota_osc_receiver.py --host 0.0.0.0 --port 9000
```

LOTA OSC addresses include:

```text
/lota/camera/position
/lota/camera/rotation
/lota/camera/euler
/lota/fps
/lota/mode
```

Important WSL note: OSC is UDP. Windows `netsh interface portproxy` forwards
TCP only, not UDP. If the OSC receiver runs in WSL2 NAT and does not receive
packets, either:

- run `lota_osc_receiver.py` from Windows Python,
- use WSL mirrored networking if available on your Windows build,
- or add a small Windows UDP forwarder from `10.0.0.6:9000` to the current WSL
  IP on port `9000`.

Windows UDP forwarder for OSC:

```powershell
powershell -ExecutionPolicy Bypass -File "\\wsl.localhost\Ubuntu-22.04\home\satya\ai_agents\drones\drone_real_to_sim_lab\sensors\iphone_lota\tools\lota_windows_udp_forwarder.ps1" `
  -ListenAddress 10.0.0.6 -ListenPort 9000 `
  -ForwardAddress 172.31.204.166 -ForwardPort 9000
```

Then run the WSL receiver:

```bash
python3 lota_osc_receiver.py --host 0.0.0.0 --port 9000
```

Outputs are saved under `captures/lota_depth_*` or `captures/lota_ply_*`.

The `tools/` folder is mostly for WSL/Windows networking diagnostics. On a
pure Linux machine on the same Wi-Fi as the iPhone, LOTA TCP/UDP can usually
send directly to the Linux host IP and these Windows forwarding helpers are not
needed.

For our drone assembly task, start with depth mode. It gives a steady 256x192
Float32 depth map that we can threshold, remove the table plane from, and cluster
into candidate physical parts.

## 1. Start the WSL Receiver

```bash
cd /home/satya/ai_agents/drones/drone_real_to_sim_lab/sensors/iphone_lota
python3 receiver.py --host 0.0.0.0 --port 5000
```

Get your WSL IP:

```bash
hostname -I
```

Use the first IP address in the iPhone app URL, for example:

```text
http://172.20.10.5:5000/frame
```

If Windows Firewall blocks the phone, allow inbound traffic to port `5000`.

## 2. Validate Receiver Without iPhone

```bash
python3 test_sender.py --url http://127.0.0.1:5000/frame
```

You should see frames appear under:

```text
captures/session_YYYYmmdd_HHMMSS/
```

## 3. Build the iPhone App in Xcode

Create a new iOS app:

- Product Name: `DepthStreamer`
- Interface: `SwiftUI`
- Language: `Swift`
- Minimum iOS: iOS 16 or newer

Add these privacy keys to `Info.plist`:

- `NSCameraUsageDescription`: `Camera and LiDAR are used to stream RGB-D frames for drone assembly tracking.`
- `NSLocalNetworkUsageDescription`: `The app streams frames to a local workstation.`

Replace the generated Swift file contents with `ARDepthStreamer.swift`, or add
that file to the app target and set it as the app entry point.

Edit this line in `ARDepthStreamer.swift`:

```swift
@StateObject private var streamer = DepthStreamer(serverURL: "http://YOUR_WSL_IP:5000/frame")
```

Then run the app on your iPhone 14 Pro.

## Output Format

Each received frame writes:

- `frame_000001_rgb.jpg`
- `frame_000001_depth.npy`
- `frame_000001_meta.json`

Depth units are meters. The saved depth array is `Float32` with shape
`[height, width]`.
