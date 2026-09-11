@echo off
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -Command "Invoke-RestMethod http://127.0.0.1:8765/health | ConvertTo-Json"
pause
