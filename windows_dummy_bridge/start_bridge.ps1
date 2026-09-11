param([switch]$Foreground)
$ErrorActionPreference = 'Stop'
$bridgeRoot = $PSScriptRoot
$pythonExe = Join-Path $bridgeRoot '.venv\Scripts\python.exe'
$bridgeFile = Join-Path $bridgeRoot 'bridge.py'
if (!(Test-Path -LiteralPath $pythonExe)) { throw 'Missing .venv. See README.md.' }
$listeners = @(Get-NetTCPConnection -State Listen -LocalPort 8765 -ErrorAction SilentlyContinue)
if ($listeners.Count) {
    $listeners | Select-Object LocalAddress,LocalPort,OwningProcess | Format-Table
    throw 'Port 8765 is occupied. No existing process was stopped.'
}
if ($Foreground) {
    & $pythonExe $bridgeFile
    exit $LASTEXITCODE
}
$runtimeDir = Join-Path $bridgeRoot 'runtime'
New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null
$bridgeArgs = @(('"' + $bridgeFile + '"'))
$proc = Start-Process -FilePath $pythonExe -ArgumentList $bridgeArgs -WorkingDirectory $bridgeRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $runtimeDir 'bridge.stdout.log') -RedirectStandardError (Join-Path $runtimeDir 'bridge.stderr.log')
$proc.Id | Set-Content (Join-Path $runtimeDir 'bridge.pid')
$ready = $false
for ($i=0; $i -lt 25; $i++) {
    Start-Sleep -Milliseconds 200
    $proc.Refresh()
    if ($proc.HasExited) { break }
    try {
        $health = Invoke-RestMethod 'http://127.0.0.1:8765/health' -TimeoutSec 1
        if ($health.mode -eq 'transport' -and $health.protocol_version -eq 4) { $ready=$true; break }
    } catch {}
}
if (!$ready) {
    Get-Content (Join-Path $runtimeDir 'bridge.stderr.log') -Tail 20
    throw 'Bridge did not become ready. Inspect runtime logs.'
}
Write-Host "Bridge running: PID $($proc.Id), 127.0.0.1:8765, V4 transport"
$health | ConvertTo-Json
Write-Host 'Dashboard: http://127.0.0.1:8765/ (startup sends no hardware command)'
