@echo off
rem Mouse control panel. Batch, not PowerShell -- immune to ExecutionPolicy.
"%~dp0.venv\Scripts\pythonw.exe" "%~dp0tools\hoverctl.py" %*
