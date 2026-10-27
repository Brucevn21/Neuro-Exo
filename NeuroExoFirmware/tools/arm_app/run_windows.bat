@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    py -3 -m venv .venv
    if errorlevel 1 goto failed
)

".venv\Scripts\python.exe" -c "import bleak" >nul 2>&1
if errorlevel 1 (
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt
    if errorlevel 1 goto failed
)

".venv\Scripts\python.exe" arm_app.py --mobile
if errorlevel 1 goto failed
exit /b 0

:failed
echo.
echo NeuroExo could not start. Review the error above.
echo Install Python 3.10 or newer with the Python launcher and Tcl/Tk support.
pause
exit /b 1
