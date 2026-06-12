# Probe LOTA TCP payload directly on Windows, bypassing WSL portproxy.
#
# Run in PowerShell. If portproxy already owns the port, temporarily delete
# that rule before running this script.
#
# Example:
#   netsh interface portproxy delete v4tov4 listenaddress=10.0.0.6 listenport=9847
#   .\lota_windows_tcp_probe.ps1 -ListenAddress 10.0.0.6 -Port 9847
#
# Then start LOTA streaming to 10.0.0.6:9847.

param(
    [string]$ListenAddress = "10.0.0.6",
    [int]$Port = 9847,
    [int]$Seconds = 8,
    [int]$MaxBytes = 4096
)

$ip = [System.Net.IPAddress]::Parse($ListenAddress)
$listener = [System.Net.Sockets.TcpListener]::new($ip, $Port)
$listener.Start()
Write-Host "Waiting for TCP connection on ${ListenAddress}:${Port}"

$client = $listener.AcceptTcpClient()
$remote = $client.Client.RemoteEndPoint
Write-Host "Connected from $remote"

$stream = $client.GetStream()
$buffer = New-Object byte[] $MaxBytes
$total = 0
$deadline = (Get-Date).AddSeconds($Seconds)

while ((Get-Date) -lt $deadline -and $total -lt $MaxBytes) {
    if ($stream.DataAvailable) {
        $read = $stream.Read($buffer, $total, $MaxBytes - $total)
        if ($read -le 0) {
            break
        }
        $total += $read
        Write-Host "received $read bytes, total $total"
    } else {
        Start-Sleep -Milliseconds 50
    }
}

$client.Close()
$listener.Stop()

Write-Host ""
Write-Host "Total bytes: $total"
if ($total -eq 0) {
    Write-Host "No payload received."
    exit
}

$n = [Math]::Min($total, 128)
for ($i = 0; $i -lt $n; $i += 16) {
    $count = [Math]::Min(16, $n - $i)
    $hex = (($buffer[$i..($i + $count - 1)] | ForEach-Object { $_.ToString("x2") }) -join " ")
    Write-Host ("{0:x8}  {1}" -f $i, $hex)
}

if ($total -ge 4) {
    $firstU32 = [BitConverter]::ToUInt32($buffer, 0)
    Write-Host ""
    Write-Host "First UInt32 LE: $firstU32"
}

