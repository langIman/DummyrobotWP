$ErrorActionPreference = 'Stop'
$serviceRoot = $PSScriptRoot
$pidPath = Join-Path $serviceRoot 'runtime\perception.pid'
$moveitPidPath = Join-Path $serviceRoot 'runtime\moveit.pid'

function Stop-MoveItGateway {
    try {
        Invoke-RestMethod -Method Post 'http://127.0.0.1:8801/disarm' `
            -ContentType 'application/json' -Body '{}' -TimeoutSec 2 | Out-Null
    } catch {}
    $moveitPid = $null
    if (Test-Path -LiteralPath $moveitPidPath) {
        $text = (Get-Content -Raw -LiteralPath $moveitPidPath).Trim()
        if ($text -match '^\d+$') { $moveitPid = [int]$text }
    }
    if ($moveitPid -and (Get-Process -Id $moveitPid -ErrorAction SilentlyContinue)) {
        Stop-Process -Id $moveitPid -Force
        Write-Host 'MoveIt safety gateway stopped.'
    }
    # Terminating the Windows wsl.exe wrapper does not always terminate its
    # Linux children. These patterns are unique to this project's gateway.
    $distro = if ($env:DUMMY_MOVEIT_WSL_DISTRO) { $env:DUMMY_MOVEIT_WSL_DISTRO } else { 'SmolVLA-Lab-Recovered' }
    & wsl.exe -d $distro -- bash -lc `
        "pkill -INT -f '[m]oveit_servo_gateway.py' 2>/dev/null || true; pkill -INT -f '[r]un_safe_launch.py' 2>/dev/null || true" 2>$null
    Start-Sleep -Milliseconds 700
    & wsl.exe -d $distro -- bash -lc `
        "pkill -KILL -f '[m]oveit_servo_gateway.py' 2>/dev/null || true; pkill -KILL -f '[r]un_safe_launch.py' 2>/dev/null || true" 2>$null
}

$running = $false
try {
    $health = Invoke-RestMethod 'http://127.0.0.1:8770/api/health' -TimeoutSec 2
    $running = $health.service -eq 'dummyv2-perception'
} catch {}
if (!$running) {
    Write-Host 'Perception service is not running.'
    Stop-MoveItGateway
    exit 0
}
try {
    $current = Invoke-RestMethod 'http://127.0.0.1:8770/api/recordings/current' -TimeoutSec 2
    if ($current.active) {
        Write-Host 'Finalizing active recording...'
        Invoke-RestMethod -Method Post 'http://127.0.0.1:8770/api/recordings/stop' -ContentType 'application/json' -Body '{}' -TimeoutSec 60 | Out-Null
    }
} catch { Write-Warning "Recording finalization reported: $($_.Exception.Message)" }
Invoke-RestMethod -Method Post 'http://127.0.0.1:8770/api/shutdown' -ContentType 'application/json' -Body '{}' -TimeoutSec 5 | Out-Null

$servicePid = $null
if (Test-Path -LiteralPath $pidPath) {
    $text = (Get-Content -Raw -LiteralPath $pidPath).Trim()
    if ($text -match '^\d+$') { $servicePid = [int]$text }
}
for ($attempt = 0; $attempt -lt 50; $attempt++) {
    Start-Sleep -Milliseconds 200
    if (!$servicePid -or !(Get-Process -Id $servicePid -ErrorAction SilentlyContinue)) { break }
}
if ($servicePid -and (Get-Process -Id $servicePid -ErrorAction SilentlyContinue)) {
    Write-Warning 'Graceful shutdown timed out; stopping the perception process.'
    Stop-Process -Id $servicePid -Force
}
Write-Host 'Perception service stopped. Dummy bridge was left running.'
Stop-MoveItGateway
