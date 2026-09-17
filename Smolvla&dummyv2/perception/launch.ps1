param([switch]$NoBrowser, [switch]$Foreground, [switch]$NoMoveIt)
$ErrorActionPreference = 'Stop'
$serviceRoot = $PSScriptRoot
$venvPython = Join-Path $serviceRoot '.venv\Scripts\python.exe'
$requirements = Join-Path $serviceRoot 'requirements.txt'
$requirementsMarker = Join-Path $serviceRoot '.venv\requirements.sha256'
$basePython = 'D:\anaconda\python.exe'
if (!(Test-Path -LiteralPath $basePython)) {
    $basePython = (Get-Command python -ErrorAction Stop).Source
}

if (!(Test-Path -LiteralPath $venvPython)) {
    Write-Host 'Creating perception Python environment...'
    & $basePython -m venv (Join-Path $serviceRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Could not create .venv.' }
}
$requiredHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $requirements).Hash
$installedHash = if (Test-Path -LiteralPath $requirementsMarker) { (Get-Content -Raw -LiteralPath $requirementsMarker).Trim() } else { '' }
if ($installedHash -ne $requiredHash) {
    Write-Host 'Installing pinned perception dependencies...'
    & $venvPython -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
    Set-Content -LiteralPath $requirementsMarker -Value $requiredHash -Encoding ascii
}

$existing = $null
try { $existing = Invoke-RestMethod 'http://127.0.0.1:8770/api/health' -TimeoutSec 2 } catch {}
if ($existing.service -eq 'dummyv2-perception') {
    Write-Host 'Perception service is already running.'
    if (!$NoBrowser) { Start-Process 'http://127.0.0.1:8770/' }
    exit 0
}

$bridgeRoot = 'D:\DummyRobot_workplace\windows_dummy_bridge'
$moveitScript = Join-Path $serviceRoot 'motion\start_moveit.ps1'
$moveitPidPath = Join-Path $serviceRoot 'runtime\moveit.pid'
$bridgeReady = $false
try {
    $bridgeHealth = Invoke-RestMethod 'http://127.0.0.1:8765/health' -TimeoutSec 2
    $bridgeReady = $bridgeHealth.protocol_version -eq 4 -and $bridgeHealth.mode -eq 'transport'
} catch {}
if (!$bridgeReady) {
    Write-Host 'Starting DummyV2 bridge without its legacy dashboard...'
    try {
        & (Join-Path $bridgeRoot 'start_bridge.ps1')
    } catch {
        Write-Warning "DummyV2 bridge could not start: $($_.Exception.Message). Camera service will continue."
    }
}

if (!$NoMoveIt -and (Test-Path -LiteralPath $moveitScript)) {
    $moveitReady = $false
    try {
        $moveitHealth = Invoke-RestMethod 'http://127.0.0.1:8801/health' -TimeoutSec 1
        $moveitReady = $moveitHealth.service -eq 'dummy-moveit-safe-gateway'
    } catch {}
    if (!$moveitReady) {
        Write-Host 'Starting isolated MoveIt safety gateway (no USB access)...'
        $moveitLog = Join-Path $serviceRoot 'runtime\moveit.stdout.log'
        $moveitErr = Join-Path $serviceRoot 'runtime\moveit.stderr.log'
        $moveitProcess = Start-Process -FilePath 'powershell.exe' -ArgumentList @(
            '-NoLogo', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $moveitScript
        ) -WorkingDirectory $serviceRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput $moveitLog -RedirectStandardError $moveitErr
        $moveitProcess.Id | Set-Content -LiteralPath $moveitPidPath -Encoding ascii
        for ($attempt = 0; $attempt -lt 75; $attempt++) {
            Start-Sleep -Milliseconds 200
            $moveitProcess.Refresh()
            if ($moveitProcess.HasExited) { break }
            try {
                $moveitHealth = Invoke-RestMethod 'http://127.0.0.1:8801/health' -TimeoutSec 1
                if ($moveitHealth.service -eq 'dummy-moveit-safe-gateway') { $moveitReady = $true; break }
            } catch {}
        }
        if ($moveitReady) {
            Write-Host "MoveIt safety gateway running: PID $($moveitProcess.Id)"
        } else {
            Write-Warning 'MoveIt safety gateway is unavailable; data collection remains record-only.'
            if (Test-Path -LiteralPath $moveitErr) { Get-Content -LiteralPath $moveitErr -Tail 12 }
        }
    }
}

$listeners = @(Get-NetTCPConnection -State Listen -LocalPort 8770 -ErrorAction SilentlyContinue)
if ($listeners.Count) {
    $listeners | Select-Object LocalAddress,LocalPort,OwningProcess | Format-Table
    throw 'Port 8770 is occupied by another process.'
}

New-Item -ItemType Directory -Path (Join-Path $serviceRoot 'runtime') -Force | Out-Null
if ($Foreground) {
    & $venvPython (Join-Path $serviceRoot 'app.py')
    exit $LASTEXITCODE
}
$process = Start-Process -FilePath $venvPython -ArgumentList @('app.py') -WorkingDirectory $serviceRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $serviceRoot 'runtime\service.stdout.log') -RedirectStandardError (Join-Path $serviceRoot 'runtime\service.stderr.log')
$process.Id | Set-Content -LiteralPath (Join-Path $serviceRoot 'runtime\perception.pid') -Encoding ascii
$ready = $false
for ($attempt = 0; $attempt -lt 150; $attempt++) {
    Start-Sleep -Milliseconds 200
    $process.Refresh()
    if ($process.HasExited) { break }
    try {
        $health = Invoke-RestMethod 'http://127.0.0.1:8770/api/health' -TimeoutSec 1
        if ($health.service -eq 'dummyv2-perception') { $ready = $true; break }
    } catch {}
}
if (!$ready) {
    if (Test-Path -LiteralPath (Join-Path $serviceRoot 'runtime\service.stderr.log')) {
        Get-Content -LiteralPath (Join-Path $serviceRoot 'runtime\service.stderr.log') -Tail 30
    }
    throw 'Perception service did not become ready.'
}
Write-Host "Perception service running: PID $($process.Id)"
Write-Host 'Dashboard: http://127.0.0.1:8770/'
Write-Host 'The service starts no robot motion and does not open the legacy camera page.'
if (!$NoBrowser) { Start-Process 'http://127.0.0.1:8770/' }
