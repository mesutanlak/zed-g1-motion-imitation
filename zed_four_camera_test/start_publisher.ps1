param(
    [Parameter(Mandatory = $true)]
    [string[]]$Camera,
    [ValidateSet("15", "30", "60")]
    [int]$Fps = 15,
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "fast",
    [ValidateSet("body18", "body38")]
    [string]$BodyFormat = "body38",
    [ValidateSet("performance", "neural-light", "neural", "ultra")]
    [string]$DepthMode = "neural-light",
    [double]$Duration = 0,
    [switch]$SdkVerbose
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$projectDir = Split-Path -Parent $scriptDir
$python = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi. Once install\setup_zed_windows.ps1 calistirin."
}
if ($Camera.Count -ne 2) {
    throw "Bu host publisher'i icin iki -Camera SERIAL:PORT degeri verin."
}

$arguments = @(
    (Join-Path $scriptDir "publisher.py"),
    "--fps", $Fps,
    "--model", $Model,
    "--body-format", $BodyFormat,
    "--depth-mode", $DepthMode,
    "--duration", $Duration
)
foreach ($item in $Camera) {
    $arguments += @("--camera", $item)
}
if ($SdkVerbose) {
    $arguments += "--sdk-verbose"
}
Write-Host "Fusion publisher basliyor ($BodyFormat): $($Camera -join ', ')"
Write-Host "Durdurmak icin Ctrl+C kullanin. Bu arac G1'e veri veya komut gondermez."
Set-Location -LiteralPath $projectDir
& $python @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Fusion publisher hata ile kapandi: $LASTEXITCODE"
}
