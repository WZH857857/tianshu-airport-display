@echo off
rem Open the existing E104 console UI without changing any E104 console file.
rem The launcher wraps its clock tick, reads the displayed RTC and sends
rem CMD_TIME only to ADV20 HDMI.
cd /d "%~dp0"
start "" "E:\anaconda3_install\pythonw.exe" -X utf8 "%~dp0e104_time_sync_launcher.py" --baud 115200
