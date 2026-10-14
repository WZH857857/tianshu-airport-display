@echo off
rem Launcher for the serial host console.  Double click, pick the COM port
rem and press connect.  The window stays open if the app cannot start.
setlocal
set "HERE=%~dp0"
set "PY=E:\anaconda3_install\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" "%HERE%airport_console.py" %*
if errorlevel 1 pause
