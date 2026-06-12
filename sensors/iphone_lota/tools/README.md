# LOTA Network Tools

Diagnostics and WSL/Windows helpers for LOTA networking.

These scripts are not part of the normal perception/tracking pipeline. They are
kept because WSL2 networking can make it hard to tell whether LOTA is sending
data, whether Windows received it, and whether it reached the Linux process.

## WSL/Linux probes

```bash
python3 tools/lota_tcp_probe.py --host 0.0.0.0 --port 9848
python3 tools/lota_stream_status.py --host 0.0.0.0 --seconds 15
```

## Windows PowerShell probes

Run from:

```powershell
cd \\wsl.localhost\Ubuntu-22.04\home\path_to_ws\drone_real_to_sim_lab\sensors\iphone_lota\tools
```

TCP payload probe:

```powershell
powershell -ExecutionPolicy Bypass -File .\lota_windows_tcp_probe.ps1 -ListenAddress 10.0.0.6 -Port 9848
```

UDP/OSC probe:

```powershell
powershell -ExecutionPolicy Bypass -File .\lota_windows_udp_probe.ps1 -ListenAddress 10.0.0.6 -Port 9000
```

UDP forwarder for OSC into WSL:

```powershell
powershell -ExecutionPolicy Bypass -File .\lota_windows_udp_forwarder.ps1 `
  -ListenAddress 10.0.0.6 -ListenPort 9000 `
  -ForwardAddress 172.31.204.166 -ForwardPort 9000
```

TCP portproxy setup for depth/PLY into WSL:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup_lota_wsl_portproxy.ps1
```

On pure Linux, these Windows-specific helpers are usually unnecessary. Set LOTA
to send directly to the Linux host IP instead.
