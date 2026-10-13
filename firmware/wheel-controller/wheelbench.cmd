@echo off
rem Wheel-mode bench panel. python.exe, not pythonw: errors stay visible (ISSUES R9).
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\wheelbench.py" %*
