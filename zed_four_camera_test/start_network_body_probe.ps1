param(
    [Parameter(Mandatory = $true)]
    [string[]]$Camera,
    [ValidateRange(1, 300)]
    [double]$Duration = 20
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$projectDir = Split-Path -Parent $scriptDir
$python = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi. Once install\setup_zed_windows.ps1 calistirin."
}
if ($Camera.Count -ne 4) {
    throw "Dort adet -Camera SERIAL@IP:PORT degeri verin."
}

$arguments = @((Join-Path $scriptDir "network_body_probe.py"), "--duration", $Duration)
foreach ($item in $Camera) {
    $arguments += @("--camera", $item)
}
Set-Location -LiteralPath $projectDir
& $python @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Ag ham-iskele tespiti hata ile kapandi: $LASTEXITCODE"
}
