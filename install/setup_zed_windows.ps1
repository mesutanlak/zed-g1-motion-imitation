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
if ($LASTEXITCODE -ne 0) {
    throw "pip guncellemesi basarisiz oldu: $LASTEXITCODE"
}
& $python -m pip install -r (Join-Path $ProjectRoot "requirements-zed.txt")
if ($LASTEXITCODE -ne 0) {
    throw "ZED Python bagimliliklari yuklenemedi: $LASTEXITCODE"
}

$apiDownloadDir = Join-Path $env:TEMP "zed-python-api"
New-Item -ItemType Directory -Force -Path $apiDownloadDir | Out-Null
& $python $helper --path $apiDownloadDir
if ($LASTEXITCODE -ne 0) {
    throw "ZED Python API kurulumu basarisiz oldu: $LASTEXITCODE"
}

& $python -c "import pyzed.sl as sl; import cv2; print('ZED Python API OK')"
if ($LASTEXITCODE -ne 0) {
    throw "ZED Python API dogrulamasi basarisiz oldu: $LASTEXITCODE"
}
Write-Host "ZED ortami hazir: $python" -ForegroundColor Green
Write-Host "Kamera testi:"
Write-Host "& '$python' '$ProjectRoot\zed_g1_skeleton.py' --list-devices"
