@echo off
powershell.exe -NoLogo -NoProfile -Command "try { Invoke-RestMethod http://127.0.0.1:8770/api/status -TimeoutSec 3 | ConvertTo-Json -Depth 8 } catch { Write-Error $_ }"
pause

