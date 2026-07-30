param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$PythonVersion = "3.11"
)

$ErrorActionPreference = "Stop"
$zedSdk = "C:\Program Files (x86)\ZED SDK"
$helper = Join-Path $zedSdk "get_python_api.py"
$venv = Join-Path $ProjectRoot ".venv-zed"
$python = Join-Path $venv "Scripts\python.exe"

if (-not (Test-Path $helper)) {
    throw "ZED SDK bulunamadi. Once resmi Windows ZED SDK'yi kurun: https://www.stereolabs.com/developers/release/"
}
if (-not (Get-Command py.exe -ErrorAction SilentlyContinue)) {
    throw "Python Launcher bulunamadi. Python $PythonVersion x64 kurun."
}

if (-not (Test-Path $python)) {
    & py.exe "-$PythonVersion" -m venv $venv
}
& $python -m pip install --upgrade pip
& $python -m pip install -r (Join-Path $ProjectRoot "requirements-zed.txt")

Push-Location $zedSdk
try {
    & $python $helper
} finally {
    Pop-Location
}

& $python -c "import pyzed.sl as sl; import cv2; print('ZED Python API OK')"
Write-Host "ZED ortami hazir: $python" -ForegroundColor Green
Write-Host "Kamera testi:"
Write-Host "& '$python' '$ProjectRoot\zed_g1_skeleton.py' --list-devices"
