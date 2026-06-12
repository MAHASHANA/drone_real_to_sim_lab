# Forward LOTA OSC UDP packets from Windows Wi-Fi into WSL2.
#
# Why this exists:
#   Windows netsh interface portproxy forwards TCP only. LOTA OSC is UDP, so
#   portproxy will not deliver OSC packets to WSL. This script binds the Windows
#   Wi-Fi IP/port and forwards each UDP datagram to WSL.
#
# Typical use:
#   1. Start WSL receiver:
#      python3 lota_osc_receiver.py --host 0.0.0.0 --port 9000
#
#   2. Run this in Windows PowerShell:
#      powershell -ExecutionPolicy Bypass -File .\lota_windows_udp_forwarder.ps1 `
#        -ListenAddress 10.0.0.6 -ListenPort 9000 `
#        -ForwardAddress 172.31.204.166 -ForwardPort 9000
#
#   3. In LOTA:
#      OSC Receiver IP: 10.0.0.6
#      OSC Port: 9000

param(
    [string]$ListenAddress = "10.0.0.6",
    [int]$ListenPort = 9000,
    [string]$ForwardAddress = "172.31.204.166",
    [int]$ForwardPort = 9000,
    [int]$MaxPackets = 0,
    [int]$PrintEvery = 30
)

$listenIp = [System.Net.IPAddress]::Parse($ListenAddress)
$listenEndpoint = [System.Net.IPEndPoint]::new($listenIp, $ListenPort)
$forwardIp = [System.Net.IPAddress]::Parse($ForwardAddress)
$forwardEndpoint = [System.Net.IPEndPoint]::new($forwardIp, $ForwardPort)

$listener = [System.Net.Sockets.UdpClient]::new($listenEndpoint)
$sender = [System.Net.Sockets.UdpClient]::new()
$remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)

Write-Host "Forwarding UDP ${ListenAddress}:${ListenPort} -> ${ForwardAddress}:${ForwardPort}"
Write-Host "Press Ctrl+C to stop."

$count = 0
$lastPrint = Get-Date

try {
    while ($true) {
        if ($MaxPackets -gt 0 -and $count -ge $MaxPackets) {
            break
        }

        $bytes = $listener.Receive([ref]$remote)
        [void]$sender.Send($bytes, $bytes.Length, $forwardEndpoint)
        $count += 1

        $now = Get-Date
        if ($count -eq 1 -or ($PrintEvery -gt 0 -and $count % $PrintEvery -eq 0) -or (($now - $lastPrint).TotalSeconds -ge 2.0)) {
            Write-Host "forwarded packet $count bytes=$($bytes.Length) from $remote"
            $lastPrint = $now
        }
    }
} finally {
    $listener.Close()
    $sender.Close()
    Write-Host "Stopped after forwarding $count packet(s)."
}

