@echo off
rem One-click launcher for the HC-05 PID tuner (no console window).
rem ASCII only: keeps cmd.exe encoding safe on Chinese Windows.
if not exist "D:\Anaconda\pythonw.exe" (
  echo Python runtime not found: D:\Anaconda\pythonw.exe
  pause
  exit /b 1
)
start "" "D:\Anaconda\pythonw.exe" "%~dp0pid_tuner.py"
