# PX4 DXP Dev Launcher
# Starts both backend (FastAPI) and frontend (static file server)
# Usage: .\run_dev.ps1

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  PX4 DXP Dev Environment Launcher" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# -- Backend ------------------------------------------------------------
Write-Host "[1/2] Starting FastAPI backend on http://localhost:5001 ..." -ForegroundColor Green
$backendJob = Start-Job -ScriptBlock {
    Set-Location "D:\Vetri\3WD_GCS\PX4_DXP\server"
    $env:ROVER_DISABLE_AUTH = "1"
    # Adjust Python path if needed; assumes uvicorn is in PATH
    uvicorn main:app --reload --host 0.0.0.0 --port 5001
}
Start-Sleep -Seconds 3

# -- Frontend -----------------------------------------------------------
Write-Host "[2/2] Starting static file server on http://localhost:3000 ..." -ForegroundColor Green
$frontendJob = Start-Job -ScriptBlock {
    Set-Location "D:\Vetri\3WD_GCS\PX4_DXP\front-end"
    python -m http.server 3000
}
Start-Sleep -Seconds 2

Write-Host ""
Write-Host "  Backend : http://localhost:5001" -ForegroundColor Yellow
Write-Host "  Frontend: http://localhost:3000" -ForegroundColor Yellow
Write-Host "  API docs: http://localhost:5001/docs" -ForegroundColor Yellow
Write-Host ""
Write-Host "Press Ctrl+C to stop both servers." -ForegroundColor DarkGray
Write-Host ""

# Keep running and print job output
while ($true) {
    Receive-Job -Job $backendJob
    Receive-Job -Job $frontendJob
    Start-Sleep -Milliseconds 500
}
