@echo off
cd /d "%~dp0pc_bridge"
python fg_udp_bridge.py COM4
pause
