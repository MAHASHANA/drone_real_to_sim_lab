# Receive and parse LOTA OSC camera-pose packets directly on Windows.
#
# Why:
#   LOTA OSC is UDP. Windows netsh portproxy only forwards TCP, so receiving
#   OSC directly in PowerShell is simpler than fighting WSL2 UDP NAT.
#
# Typical run from Windows PowerShell:
#   powershell -ExecutionPolicy Bypass -File .\lota_windows_osc_logger.ps1 `
#     -ListenAddress 10.0.0.6 -Port 9000 `
#     -OutDir "\\wsl.localhost\Ubuntu-22.04\home\satya\ai_agents\drones\drone_real_to_sim_lab\sensors\iphone_lota\captures"
#
# LOTA settings:
#   OSC Receiver IP: 10.0.0.6
#   OSC Port: 9000

param(
    [string]$ListenAddress = "10.0.0.6",
    [int]$Port = 9000,
    [string]$OutDir = "\\wsl.localhost\Ubuntu-22.04\home\satya\ai_agents\drones\drone_real_to_sim_lab\sensors\iphone_lota\captures",
    [int]$MaxPackets = 0,
    [double]$PrintEverySeconds = 0.25,
    [switch]$SelfTest
)

function Align4 {
    param([int]$Offset)
    return (($Offset + 3) -band (-bnot 3))
}

function Read-OscString {
    param([byte[]]$Bytes, [ref]$Offset)

    $start = $Offset.Value
    $end = $start
    while ($end -lt $Bytes.Length -and $Bytes[$end] -ne 0) {
        $end += 1
    }
    if ($end -ge $Bytes.Length) {
        throw "OSC string is not null-terminated"
    }

    $count = $end - $start
    $value = [System.Text.Encoding]::UTF8.GetString($Bytes, $start, $count)
    $Offset.Value = Align4 ($end + 1)
    return $value
}

function Read-BigEndianInt32 {
    param([byte[]]$Bytes, [ref]$Offset)
    $tmp = New-Object byte[] 4
    [Array]::Copy($Bytes, $Offset.Value, $tmp, 0, 4)
    if ([BitConverter]::IsLittleEndian) {
        [Array]::Reverse($tmp)
    }
    $Offset.Value += 4
    return [BitConverter]::ToInt32($tmp, 0)
}

function Read-BigEndianFloat32 {
    param([byte[]]$Bytes, [ref]$Offset)
    $tmp = New-Object byte[] 4
    [Array]::Copy($Bytes, $Offset.Value, $tmp, 0, 4)
    if ([BitConverter]::IsLittleEndian) {
        [Array]::Reverse($tmp)
    }
    $Offset.Value += 4
    return [BitConverter]::ToSingle($tmp, 0)
}

function Read-OscAtom {
    param([byte[]]$Bytes, [ref]$Offset, [char]$Tag)

    switch ($Tag) {
        'i' { return Read-BigEndianInt32 $Bytes $Offset }
        'f' { return Read-BigEndianFloat32 $Bytes $Offset }
        's' { return Read-OscString $Bytes $Offset }
        'T' { return $true }
        'F' { return $false }
        'N' { return $null }
        default { throw "Unsupported OSC type tag: $Tag" }
    }
}

function Parse-OscPacket {
    param([byte[]]$Bytes)

    $bundlePrefix = [System.Text.Encoding]::ASCII.GetBytes("#bundle")
    $isBundle = $Bytes.Length -ge 8
    for ($i = 0; $i -lt $bundlePrefix.Length -and $isBundle; $i++) {
        if ($Bytes[$i] -ne $bundlePrefix[$i]) {
            $isBundle = $false
        }
    }
    if ($isBundle) {
        $offset = 16 # "#bundle\0" + 8-byte timetag
        $messages = @()
        while ($offset -lt $Bytes.Length) {
            $sizeRef = [ref]$offset
            $size = Read-BigEndianInt32 $Bytes $sizeRef
            $offset = $sizeRef.Value
            if ($size -le 0 -or ($offset + $size) -gt $Bytes.Length) {
                throw "Bad OSC bundle element size"
            }
            $element = New-Object byte[] $size
            [Array]::Copy($Bytes, $offset, $element, 0, $size)
            $messages += Parse-OscPacket $element
            $offset += $size
        }
        return $messages
    }

    $offset = 0
    $offsetRef = [ref]$offset
    $address = Read-OscString $Bytes $offsetRef
    $offset = $offsetRef.Value
    $offsetRef = [ref]$offset
    $typeTags = Read-OscString $Bytes $offsetRef
    $offset = $offsetRef.Value

    if (-not $typeTags.StartsWith(",")) {
        throw "Bad OSC type tag string: $typeTags"
    }

    $values = @()
    foreach ($tag in $typeTags.Substring(1).ToCharArray()) {
        $offsetRef = [ref]$offset
        $values += Read-OscAtom $Bytes $offsetRef $tag
        $offset = $offsetRef.Value
    }

    return @([PSCustomObject]@{
        address = $address
        values = $values
    })
}

function Pad4 {
    param([byte[]]$Bytes)
    $pad = (4 - ($Bytes.Length % 4)) % 4
    if ($pad -eq 0) {
        return $Bytes
    }
    $out = New-Object byte[] ($Bytes.Length + $pad)
    [Array]::Copy($Bytes, $out, $Bytes.Length)
    return $out
}

function Make-OscString {
    param([string]$Value)
    return Pad4 ([System.Text.Encoding]::UTF8.GetBytes($Value + [char]0))
}

function Update-State {
    param([hashtable]$State, [object]$Message)

    switch ($Message.address) {
        "/lota/camera/position" { $State.position = $Message.values }
        "/lota/camera/rotation" { $State.rotation_quat = $Message.values }
        "/lota/camera/euler" { $State.euler = $Message.values }
        "/lota/fps" { if ($Message.values.Count -gt 0) { $State.fps = $Message.values[0] } }
        "/lota/mode" { if ($Message.values.Count -gt 0) { $State.mode = $Message.values[0] } }
        default { $State.last_other = $Message }
    }
}

if ($SelfTest) {
    $packet = @()
    $packet += Make-OscString "/lota/camera/position"
    $packet += Make-OscString ",fff"
    foreach ($v in @(1.0, 2.0, 3.0)) {
        $bytes = [BitConverter]::GetBytes([single]$v)
        if ([BitConverter]::IsLittleEndian) { [Array]::Reverse($bytes) }
        $packet += $bytes
    }
    $messages = Parse-OscPacket ([byte[]]$packet)
    $messages | ConvertTo-Json -Depth 8
    exit
}

$session = "lota_osc_windows_" + (Get-Date -Format "yyyyMMdd_HHmmss")
$captureDir = Join-Path $OutDir $session
New-Item -ItemType Directory -Force -Path $captureDir | Out-Null
$logPath = Join-Path $captureDir "osc_messages.jsonl"

$listenIp = [System.Net.IPAddress]::Parse($ListenAddress)
$endpoint = [System.Net.IPEndPoint]::new($listenIp, $Port)
$udp = [System.Net.Sockets.UdpClient]::new($endpoint)
$remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
$state = @{}
$count = 0
$lastPrint = Get-Date

Write-Host "Listening for LOTA OSC UDP on ${ListenAddress}:${Port}"
Write-Host "Writing JSONL to $logPath"
Write-Host "Press Ctrl+C to stop."

try {
    while ($true) {
        if ($MaxPackets -gt 0 -and $count -ge $MaxPackets) {
            break
        }

        $bytes = $udp.Receive([ref]$remote)
        $count += 1
        $now = Get-Date
        $unixMs = [DateTimeOffset]::Now.ToUnixTimeMilliseconds()

        try {
            $messages = Parse-OscPacket $bytes
        } catch {
            Write-Host "Bad OSC packet from ${remote}: $_"
            continue
        }

        foreach ($msg in $messages) {
            Update-State $state $msg
            $record = [ordered]@{
                time_unix_ms = $unixMs
                remote = $remote.ToString()
                address = $msg.address
                values = $msg.values
            }
            ($record | ConvertTo-Json -Compress -Depth 8) | Add-Content -Path $logPath
        }

        if (($now - $lastPrint).TotalSeconds -ge $PrintEverySeconds) {
            $lastPrint = $now
            Write-Host ("packets={0} pos={1} quat={2} euler={3} fps={4} mode={5}" -f `
                $count, `
                (($state.position | ConvertTo-Json -Compress -Depth 4) -replace "`n", ""), `
                (($state.rotation_quat | ConvertTo-Json -Compress -Depth 4) -replace "`n", ""), `
                (($state.euler | ConvertTo-Json -Compress -Depth 4) -replace "`n", ""), `
                $state.fps, `
                $state.mode)
        }
    }
} finally {
    $udp.Close()
    Write-Host "Stopped after $count packet(s)."
    Write-Host "Log: $logPath"
}
