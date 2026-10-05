@echo off
"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" ^
  --aircraft=A320-200-CFM ^
  --airport=LFMD ^
  --timeofday=noon ^
  --prop:/systems/start-up-on-start=true ^
  --generic=socket,in,45,,5501,udp,mpu-input ^
  --telnet=5500
