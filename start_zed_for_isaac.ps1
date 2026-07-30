param(
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "accurate",
    [switch]$Record,
    [switch]$RecordSvo2
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$zedPython = Join-Path $project ".venv-zed\Scripts\python.exe"
if (-not (Test-Path $zedPython)) {
    $zedPython = "python"
}
$wslAddressText = (& wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim()
$wslAddress = ($wslAddressText -split "\s+")[0]
if (-not $wslAddress) {
    throw "WSL IP adresi bulunamadi."
}

$arguments = @(
    (Join-Path $project "zed_g1_skeleton.py"),
    "--model", $Model,
    "--fps", "30",
    "--stream-host", $wslAddress,
    "--stream-port", "15050",
    "--stream-max-hz", "30"
)
if ($Record) { $arguments += "--record" }
if ($RecordSvo2) { $arguments += "--record-svo2" }

Write-Host "ZED BODY_38 -> WSL $wslAddress`:15050"
& $zedPython @arguments
exit $LASTEXITCODE
