@echo off
rem Troubleshooting launcher: keeps the console open so errors are visible.
if not exist "D:\Anaconda\python.exe" (
  echo Python runtime not found: D:\Anaconda\python.exe
  pause
  exit /b 1
)
"D:\Anaconda\python.exe" "%~dp0pid_tuner.py"
echo.
echo Exit code: %errorlevel%
pause
