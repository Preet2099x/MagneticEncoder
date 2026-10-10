@echo off
rem Opens a serial monitor window for each Teensy. Double-click to run.
rem If a board shows up on a different COM port, change it here.
set ACTUATOR_PORT=COM7
set ENCODER_PORT=COM14
set BAUD=115200
set PIO=%USERPROFILE%\.platformio\penv\Scripts\pio.exe

start "Actuator (%ACTUATOR_PORT%)" cmd /k ""%PIO%" device monitor -p %ACTUATOR_PORT% -b %BAUD%"
start "Encoder (%ENCODER_PORT%)" cmd /k ""%PIO%" device monitor -p %ENCODER_PORT% -b %BAUD%"
