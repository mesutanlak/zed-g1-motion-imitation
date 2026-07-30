$ErrorActionPreference = "Stop"

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$zedPython = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path $zedPython)) {
    $zedPython = "python"
}
$wslAddressLine = wsl -d Ubuntu-22.04 -- hostname -I
$wslAddress = ($wslAddressLine.Trim() -split "\s+")[0]
if (-not $wslAddress) {
    throw "WSL IP adresi alınamadı."
}

Write-Host "ZED BODY38 hedefi: ${wslAddress}:15050"
Set-Location -LiteralPath $projectDir
& $zedPython .\zed_g1_skeleton.py `
  --model accurate `
  --fps 30 `
  --record `
  --record-svo2 `
  --stream-host $wslAddress `
  --stream-port 15050
