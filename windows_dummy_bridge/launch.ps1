param([switch]$NoBrowser, [switch]$LocalOnly)
$ErrorActionPreference = 'Stop'
$health = $null
try { $health = Invoke-RestMethod 'http://127.0.0.1:8765/health' -TimeoutSec 2 } catch {}
if ($health) {
    if ($health.protocol_version -ne 4 -or $health.mode -ne 'transport') { throw 'Port 8765 hosts another version. Stop this project first.' }
    Write-Host 'V4 bridge already running.'
} else { & (Join-Path $PSScriptRoot 'start_bridge.ps1') }
if (!$LocalOnly) {
    try { & (Join-Path $PSScriptRoot 'start_tunnel.ps1') }
    catch { Write-Warning "Tunnel unavailable: $($_.Exception.Message). Local dashboard remains available." }
}
Write-Host 'Dashboard: http://127.0.0.1:8765/'
Write-Host 'No unlock or heartbeat required. Closing this window leaves services running.'
Write-Host 'Use stop.cmd to stop communication; it does not stop the robot.'
if (!$NoBrowser) { Start-Process 'http://127.0.0.1:8765/' }
