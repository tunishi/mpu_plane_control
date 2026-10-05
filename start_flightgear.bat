@echo off
"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" ^
  --aircraft=c172p ^
  --airport=LFMD ^
  --offset-distance=15 ^
  --offset-azimuth=0 ^
  --altitude=5000 ^
  --vc=100 ^
  --heading=270 ^
  --timeofday=noon ^
  --prop:/engines/engine[0]/running=true ^
  --prop:/controls/engines/engine[0]/throttle=0.7 ^
  --prop:/controls/engines/engine[0]/mixture=1 ^
  --prop:/controls/engines/engine[0]/magnetos=3 ^
  --generic=socket,in,45,,5501,udp,mpu-input ^
  --telnet=5500
