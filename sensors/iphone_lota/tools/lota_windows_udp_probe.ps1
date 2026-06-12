# Probe LOTA OSC/UDP packets directly on Windows.
#
# Run in PowerShell:
#   powershell -ExecutionPolicy Bypass -File .\lota_windows_udp_probe.ps1 -ListenAddress 10.0.0.6 -Port 9000
#
# In LOTA:
#   OSC Receiver IP: 10.0.0.6
#   OSC Port: 9000

param(
    [string]$ListenAddress = "10.0.0.6",
    [int]$Port = 9000,
    [int]$Seconds = 20,
    [int]$MaxPackets = 20
)

function Format-HexLine {
    param([byte[]]$Bytes, [int]$Count)
    $n = [Math]::Min($Count, 128)
    for ($i = 0; $i -lt $n; $i += 16) {
        $end = [Math]::Min($i + 15, $n - 1)
        $slice = $Bytes[$i..$end]
        $hex = (($slice | ForEach-Object { $_.ToString("x2") }) -join " ")
        $ascii = (($slice | ForEach-Object {
            if ($_ -ge 32 -and $_ -lt 127) { [char]$_ } else { "." }
        }) -join "")
        Write-Host ("{0:x8}  {1,-48}  {2}" -f $i, $hex, $ascii)
    }
}

$ip = [System.Net.IPAddress]::Parse($ListenAddress)
$local = [System.Net.IPEndPoint]::new($ip, $Port)
$udp = [System.Net.Sockets.UdpClient]::new($local)
$udp.Client.ReceiveTimeout = 500
$remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)

Write-Host "Listening for UDP on ${ListenAddress}:${Port} for ${Seconds}s"
$deadline = (Get-Date).AddSeconds($Seconds)
$count = 0

try {
    while ((Get-Date) -lt $deadline -and $count -lt $MaxPackets) {
        try {
            $bytes = $udp.Receive([ref]$remote)
        } catch [System.Net.Sockets.SocketException] {
            continue
        }
        $count += 1
        Write-Host ""
        Write-Host "Packet $count from $remote, bytes=$($bytes.Length)"
        Format-HexLine -Bytes $bytes -Count $bytes.Length
    }
} finally {
    $udp.Close()
}

Write-Host ""
Write-Host "Total packets: $count"
if ($count -eq 0) {
    Write-Host "No UDP packets received. Check LOTA OSC IP/port, iOS Local Network permission, and Windows firewall."
}

