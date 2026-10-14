@echo off
rem Launch the E104 emergency-monitor console without a console window.
cd /d "%~dp0"
rem Main board serial defaults to 115200. HC-06 Bluetooth connects separately at 9600.
start "" "E:\anaconda3_install\pythonw.exe" -X utf8 "e104_console.py" --baud 115200
