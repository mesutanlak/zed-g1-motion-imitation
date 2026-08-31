param(
    [Parameter(Mandatory = $true)]
    [string]$FusionConfig,
    [Parameter(Mandatory = $true)]
    [int[]]$LocalSerial,
    [Parameter(Mandatory = $true)]
    [string[]]$RemoteCamera,
    [ValidateSet("15", "30", "60")]
    [int]$Fps = 15,
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "fast",
    [ValidateSet("performance", "neural-light", "neural", "ultra")]
    [string]$DepthMode = "neural-light",
    [ValidateRange(0, 86400)]
    [double]$Duration = 180,
    [ValidateSet("1", "2", "3", "4")]
    [int]$MinimumCameras = 2,
    [string]$Record = ""
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$projectDir = Split-Path -Parent $scriptDir
$python = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi. Once install\setup_zed_windows.ps1 calistirin."
}
if ($LocalSerial.Count -ne 2 -or $RemoteCamera.Count -ne 2) {
    throw "Dort-kamera testi iki -LocalSerial ve iki -RemoteCamera SERIAL@IP:PORT degeri ister."
}
if (-not (Test-Path -LiteralPath $FusionConfig)) {
    throw "ZED360 Fusion JSON bulunamadi: $FusionConfig"
}

$arguments = @(
    (Join-Path $scriptDir "fusion_test.py"),
    "--fusion-config", (Resolve-Path -LiteralPath $FusionConfig).Path,
    "--fps", $Fps,
    "--model", $Model,
    "--depth-mode", $DepthMode,
    "--duration", $Duration,
    "--min-cameras", $MinimumCameras
)
foreach ($serial in $LocalSerial) {
    $arguments += @("--local-serial", $serial)
}
foreach ($camera in $RemoteCamera) {
    $arguments += @("--remote-camera", $camera)
}
if ($Record) {
    $arguments += @("--record", $Record)
}
Write-Host "Dort-kamera BODY_38 Fusion testi basliyor. Bu arac G1'e veri veya komut gondermez."
Set-Location -LiteralPath $projectDir
& $python @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Dort-kamera Fusion testi hata ile kapandi: $LASTEXITCODE"
}
