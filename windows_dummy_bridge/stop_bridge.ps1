$ErrorActionPreference = 'Stop'
$runtimeDir = Join-Path $PSScriptRoot 'runtime'
$tunnelPidFile = Join-Path $runtimeDir 'tunnel.pid'
if (Test-Path $tunnelPidFile) {
    $tunnelPid = [int](Get-Content $tunnelPidFile)
    $tunnelProc = Get-CimInstance Win32_Process -Filter "ProcessId=$tunnelPid"
    if ($tunnelProc -and $tunnelProc.Name -eq 'ssh.exe' -and $tunnelProc.CommandLine -like '*127.0.0.1:18765:127.0.0.1:8765*') {
        Stop-Process -Id $tunnelPid
    }
}
$bridgePidFile = Join-Path $runtimeDir 'bridge.pid'
if (Test-Path $bridgePidFile) {
    $bridgePid = [int](Get-Content $bridgePidFile)
    $processes = @(Get-CimInstance Win32_Process)
    $rootProc = $processes | Where-Object { $_.ProcessId -eq $bridgePid }
    $bridgeFile = Join-Path $PSScriptRoot 'bridge.py'
    if ($rootProc -and $rootProc.Name -eq 'python.exe' -and $rootProc.CommandLine.Contains($bridgeFile)) {
        function Stop-BridgeTree([int]$targetPid) {
            foreach ($child in $processes | Where-Object { $_.ParentProcessId -eq $targetPid }) {
                Stop-BridgeTree $child.ProcessId
            }
            Stop-Process -Id $targetPid -ErrorAction SilentlyContinue
        }
        Stop-BridgeTree $bridgePid
    }
}
Write-Host 'Stopped bridge/tunnel communication only. No hardware command sent.'
