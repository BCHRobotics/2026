# Setup and launch NetworkTables Vision Monitor (Windows PowerShell)
# This script creates a virtual environment, installs dependencies, and runs the monitor

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $ScriptDir "venv"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "NetworkTables Vision Monitor Setup" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Check if Python is installed
try {
    $pythonVersion = python --version 2>&1
    Write-Host "✓ Python found: $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "❌ Error: Python is not installed or not in PATH" -ForegroundColor Red
    Write-Host "Please install Python 3 and add it to PATH" -ForegroundColor Yellow
    exit 1
}
Write-Host ""

# Create virtual environment if it doesn't exist
if (-not (Test-Path $VenvDir)) {
    Write-Host "📦 Creating virtual environment..." -ForegroundColor Yellow
    python -m venv $VenvDir
    Write-Host "✓ Virtual environment created" -ForegroundColor Green
} else {
    Write-Host "✓ Virtual environment already exists" -ForegroundColor Green
}
Write-Host ""

# Activate virtual environment
Write-Host "🔧 Activating virtual environment..." -ForegroundColor Yellow
$ActivateScript = Join-Path $VenvDir "Scripts\Activate.ps1"
& $ActivateScript
Write-Host "✓ Virtual environment activated" -ForegroundColor Green
Write-Host ""

# Upgrade pip
Write-Host "⬆️  Upgrading pip..." -ForegroundColor Yellow
python -m pip install --upgrade pip --quiet
Write-Host "✓ pip upgraded" -ForegroundColor Green
Write-Host ""

# Install requirements
Write-Host "📥 Installing dependencies..." -ForegroundColor Yellow
$RequirementsFile = Join-Path $ScriptDir "requirements.txt"
pip install -r $RequirementsFile --quiet
Write-Host "✓ Dependencies installed" -ForegroundColor Green
Write-Host ""

# Launch the monitor
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "🚀 Launching Vision Monitor" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Usage examples:" -ForegroundColor Yellow
Write-Host "  Team number: Add --team 2026" -ForegroundColor White
Write-Host "  IP address:  Add --ip 10.20.26.2" -ForegroundColor White
Write-Host "  Simulator:   Add --simulator" -ForegroundColor White
Write-Host ""

# Launch Python script with all arguments passed through
$PythonScript = Join-Path $ScriptDir "nt_vision_monitor.py"
python $PythonScript $args

# Deactivate virtual environment
deactivate
