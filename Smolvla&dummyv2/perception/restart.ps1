param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$serviceRoot = $PSScriptRoot
$mutex = New-Object System.Threading.Mutex($false, 'Local\DummyV2PerceptionRestart8770')
$ownsMutex = $false
try {
    try { $ownsMutex = $mutex.WaitOne(0) } catch [System.Threading.AbandonedMutexException] { $ownsMutex = $true }
    if (!$ownsMutex) {
        Write-Host 'A restart is already in progress. Please wait for its window.'
        exit 0
    }

    $health = $null
    try { $health = Invoke-RestMethod 'http://127.0.0.1:8770/api/health' -TimeoutSec 3 } catch {}
    if ($health -and $health.service -ne 'dummyv2-perception') {
        throw 'Port 8770 belongs to another service. No process was stopped.'
    }
    if ($health.service -eq 'dummyv2-perception') {
        $recording = Invoke-RestMethod 'http://127.0.0.1:8770/api/recordings/current' -TimeoutSec 5
        if ($recording.active) {
            Write-Host 'Finalizing active recording...'
            Invoke-RestMethod -Method Post 'http://127.0.0.1:8770/api/recordings/stop' -ContentType 'application/json' -Body '{}' -TimeoutSec 45 | Out-Null
        }
        Write-Host 'Closing perception service and releasing cameras...'
        Invoke-RestMethod -Method Post 'http://127.0.0.1:8770/api/shutdown' -ContentType 'application/json' -Body '{}' -TimeoutSec 5 | Out-Null

        # Wait for the actual owner of the listening socket, not a possibly stale PID file.
        $deadline = [DateTime]::UtcNow.AddSeconds(30)
        do {
            Start-Sleep -Milliseconds 300
            $listeners = @(Get-NetTCPConnection -LocalPort 8770 -State Listen -ErrorAction SilentlyContinue)
        } while ($listeners.Count -and [DateTime]::UtcNow -lt $deadline)
        if ($listeners.Count) {
            throw 'Graceful shutdown did not finish. No process was forcibly killed; check runtime logs.'
        }
        # The shutdown endpoint exits only after closing devices. A short wait also
        # lets the exiting process release redirected log file handles.
        Start-Sleep -Milliseconds 500
    } else {
        Write-Host 'Perception is offline; starting it again.'
        $listeners = @(Get-NetTCPConnection -LocalPort 8770 -State Listen -ErrorAction SilentlyContinue)
        if ($listeners.Count) {
            throw 'Port 8770 is occupied but its health check failed. No process was forcibly killed; check runtime logs.'
        }
        $expectedPython = Join-Path $serviceRoot '.venv\Scripts\python.exe'
        $remaining = @(Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" | Where-Object {
            $_.ExecutablePath -eq $expectedPython -and $_.CommandLine -match '(?:^|[\s"\\])app\.py(?:[\s"]|$)'
        })
        if ($remaining.Count) {
            throw 'The previous perception process still exists without a healthy listener. No duplicate instance was started; check runtime logs.'
        }
    }

    # launch.ps1 truncates redirected logs, so retain the previous run for diagnosis.
    $archive = Join-Path $serviceRoot ('runtime\restart-history\' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
    foreach ($name in @('service.stdout.log', 'service.stderr.log')) {
        $source = Join-Path $serviceRoot ('runtime\' + $name)
        if (Test-Path -LiteralPath $source) {
            New-Item -ItemType Directory -Path $archive -Force | Out-Null
            Copy-Item -LiteralPath $source -Destination (Join-Path $archive $name)
        }
    }
    Write-Host 'Starting perception; existing bridge and MoveIt gateway will be reused.'
    & powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File (Join-Path $serviceRoot 'launch.ps1') -NoBrowser
    if ($LASTEXITCODE -ne 0) { throw 'Startup failed. See the launcher output above.' }
    $health = Invoke-RestMethod 'http://127.0.0.1:8770/api/health' -TimeoutSec 5
    if ($health.service -ne 'dummyv2-perception' -or $health.status -ne 'alive') {
        throw 'Startup health check failed.'
    }
    Write-Host ('Restart complete. Version: ' + $health.version)
    Write-Host 'Open: http://127.0.0.1:8770/collect'
    Write-Host 'Robot control is not automatically enabled.'
    if (!$NoBrowser) { Start-Process 'http://127.0.0.1:8770/collect' }
} catch {
    Write-Host ('Restart failed: ' + $_.Exception.Message) -ForegroundColor Red
    exit 1
} finally {
    if ($ownsMutex) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
