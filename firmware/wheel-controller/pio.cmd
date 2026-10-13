@echo off
rem Thin wrapper so you never have to remember the venv path.
rem Batch, not PowerShell -- immune to ExecutionPolicy.
rem   pio run
rem   pio run -t upload
rem   pio device monitor
"%~dp0.venv\Scripts\platformio.exe" %*
