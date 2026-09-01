param(
    [Parameter(Mandatory = $true)]
    [Alias("Input")]
    [string]$CapturePath,
    [Parameter(Mandatory = $true)]
    [Alias("Output")]
    [string]$OutputPath,
    [Parameter(Mandatory = $true)]
    [long]$ReferenceSerial,
    [string]$WorldPosesJsonl = ""
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$calibrator = Join-Path $PSScriptRoot "calibrate_distributed_body38.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
$arguments = @(
    $calibrator,
    "--input", $CapturePath,
    "--output", $OutputPath,
    "--reference-serial", "$ReferenceSerial"
)
if ($WorldPosesJsonl) {
    $arguments += "--world-poses-jsonl"
    $arguments += $WorldPosesJsonl
}
& $python @arguments
exit $LASTEXITCODE
