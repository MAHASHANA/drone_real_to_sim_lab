# Run this in an Administrator PowerShell window on Windows.
#
# Purpose:
#   LOTA on the iPhone can reach Windows on the Wi-Fi LAN, but cannot directly
#   reach WSL's NAT IP. These rules forward Windows TCP ports into WSL.
#
# Current values detected from WSL:
#   Windows Wi-Fi IP: 10.0.0.6
#   WSL eth0 IP:      172.31.204.166
#
# If WSL restarts, rerun `hostname -I` inside WSL and update $WslAddress.

$WindowsListenAddress = "10.0.0.6"
$WslAddress = "172.31.204.166"
$Ports = @(9847, 9848)

foreach ($Port in $Ports) {
    netsh interface portproxy delete v4tov4 listenaddress=$WindowsListenAddress listenport=$Port | Out-Null
    netsh interface portproxy add v4tov4 `
        listenaddress=$WindowsListenAddress `
        listenport=$Port `
        connectaddress=$WslAddress `
        connectport=$Port
}

foreach ($Port in $Ports) {
    $Name = "LOTA to WSL TCP $Port"
    $Existing = Get-NetFirewallRule -DisplayName $Name -ErrorAction SilentlyContinue
    if (-not $Existing) {
        New-NetFirewallRule `
            -DisplayName $Name `
            -Direction Inbound `
            -Action Allow `
            -Protocol TCP `
            -LocalAddress $WindowsListenAddress `
            -LocalPort $Port | Out-Null
    }
}

Write-Host "Configured LOTA TCP forwarding:"
netsh interface portproxy show v4tov4
Write-Host ""
Write-Host "Set LOTA Receiver IP to $WindowsListenAddress"
Write-Host "Depth TCP port: 9847"
Write-Host "PLY TCP port:   9848"

