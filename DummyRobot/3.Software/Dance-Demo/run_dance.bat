@echo off
setlocal
set "PYTHON=%~dp0..\..\.venv\Scripts\python.exe"

if not exist "%PYTHON%" (
    echo Python environment not found: %PYTHON%
    echo Run this repository's Python setup first.
    pause
    exit /b 1
)

"%PYTHON%" "%~dp0dummy_dance.py" %*
set "EXIT_CODE=%ERRORLEVEL%"
echo.
pause
exit /b %EXIT_CODE%
