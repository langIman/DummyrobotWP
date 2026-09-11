param([string]$SshHost = 'frp-era.com')
$ErrorActionPreference = 'Stop'
if ($SshHost -notmatch '^[A-Za-z0-9._-]+$') { throw 'Use an SSH config host alias.' }
$runtimeDir = Join-Path $PSScriptRoot 'runtime'
New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null
$health = Invoke-RestMethod 'http://127.0.0.1:8765/health' -TimeoutSec 3
if ($health.mode -ne 'transport' -or $health.protocol_version -ne 4) { throw 'Expected Dummy V4 transport bridge.' }
$pidFile = Join-Path $runtimeDir 'tunnel.pid'
if (Test-Path $pidFile) {
    $oldPid = [int](Get-Content $pidFile)
    $old = Get-CimInstance Win32_Process -Filter "ProcessId=$oldPid"
    if ($old -and $old.Name -eq 'ssh.exe' -and $old.CommandLine -like '*127.0.0.1:18765:127.0.0.1:8765*') {
        Write-Host "Managed tunnel already running: PID $oldPid. No duplicate started."
        return
    }
}
$sshExe = (Get-Command ssh.exe).Source
# Use existing host/user/port/key configuration; never read the private key.
$probe = & $sshExe -o BatchMode=yes -o ConnectTimeout=10 $SshHost "ss -H -ltn 'sport = :18765'" 2>&1
if ($LASTEXITCODE -ne 0) { throw "SSH preflight failed: $probe" }
if ($probe) { throw "Remote port 18765 occupied: $probe" }
$argsList = @('-N','-T','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3','-R','127.0.0.1:18765:127.0.0.1:8765',$SshHost)
$proc = Start-Process -FilePath $sshExe -ArgumentList $argsList -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $runtimeDir 'tunnel.stdout.log') -RedirectStandardError (Join-Path $runtimeDir 'tunnel.stderr.log')
$proc.Id | Set-Content $pidFile
try {
    Start-Sleep -Seconds 2
    $proc.Refresh()
    if ($proc.HasExited) {
        $detail = Get-Content (Join-Path $runtimeDir 'tunnel.stderr.log') -Raw
        throw "SSH forwarding failed: $detail"
    }
    $listener = & $sshExe -o BatchMode=yes -o ConnectTimeout=10 $SshHost "ss -H -ltn 'sport = :18765'" 2>&1
    if ($LASTEXITCODE -ne 0 -or "$listener" -notmatch '127\.0\.0\.1:18765' -or "$listener" -match '0\.0\.0\.0:18765|\*:18765|\[::\]:18765') {
        throw "Remote listener is not verified loopback-only: $listener"
    }
    $response = & $sshExe -o BatchMode=yes -o ConnectTimeout=10 $SshHost 'curl --fail --silent --show-error --max-time 5 http://127.0.0.1:18765/health' 2>&1
    if ($LASTEXITCODE -ne 0) { throw "Remote health check failed: $response" }
    $remoteHealth = "$response" | ConvertFrom-Json
    if ($remoteHealth.instance_id -ne $health.instance_id) { throw 'Remote bridge instance mismatch.' }
    Write-Host "Tunnel verified: PID $($proc.Id), $SshHost loopback 18765 -> Windows loopback 8765"
    $remoteHealth | ConvertTo-Json
} catch {
    if (!$proc.HasExited) { Stop-Process -Id $proc.Id -ErrorAction SilentlyContinue }
    throw
}
