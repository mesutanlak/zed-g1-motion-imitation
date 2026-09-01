param(
    [Parameter(Mandatory = $true)]
    [string]$InputConfig,
    [Parameter(Mandatory = $true)]
    [string]$OutputExtrinsics,
    [Parameter(Mandatory = $true)]
    [string]$WorldPosesJsonl,
    [Parameter(Mandatory = $true)]
    [long]$ReferenceSerial
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$converter = Join-Path $PSScriptRoot "convert_zed360_extrinsics.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi: $python"
}
if (-not (Test-Path -LiteralPath $InputConfig)) {
    throw "ZED360 kalibrasyon dosyasi bulunamadi: $InputConfig"
}
$arguments = @(
    $converter,
    "--input", $InputConfig,
    "--output", $OutputExtrinsics,
    "--world-poses-jsonl", $WorldPosesJsonl,
    "--reference-serial", "$ReferenceSerial"
)
& $python @arguments
exit $LASTEXITCODE
