@echo off
REM Setup and launch NetworkTables Vision Monitor (Windows Batch)
REM This script creates a virtual environment, installs dependencies, and runs the monitor

setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set VENV_DIR=%SCRIPT_DIR%venv

echo ==========================================
echo NetworkTables Vision Monitor Setup
echo ==========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3 and add it to PATH
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version 2^>^&1') do set PYTHON_VERSION=%%i
echo ✓ Python found: %PYTHON_VERSION%
echo.

REM Create virtual environment if it doesn't exist
if not exist "%VENV_DIR%" (
    echo Creating virtual environment...
    python -m venv "%VENV_DIR%"
    echo ✓ Virtual environment created
) else (
    echo ✓ Virtual environment already exists
)
echo.

REM Activate virtual environment
echo Activating virtual environment...
call "%VENV_DIR%\Scripts\activate.bat"
echo ✓ Virtual environment activated
echo.

REM Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip --quiet
echo ✓ pip upgraded
echo.

REM Install requirements
echo Installing dependencies...
pip install -r "%SCRIPT_DIR%requirements.txt" --quiet
echo ✓ Dependencies installed
echo.

REM Launch the monitor
echo ==========================================
echo Launching Vision Monitor
echo ==========================================
echo.
echo Usage examples:
echo   Team number: Add --team 2026
echo   IP address:  Add --ip 10.20.26.2
echo   Simulator:   Add --simulator
echo.

REM Launch Python script with all arguments
python "%SCRIPT_DIR%nt_vision_monitor.py" %*

REM Deactivate virtual environment
call deactivate

endlocal
