param(
    [Parameter(Mandatory = $true)]
    [string]$Input,
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [Parameter(Mandatory = $true)]
    [long]$ReferenceSerial
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$calibrator = Join-Path $PSScriptRoot "calibrate_distributed_body38.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
& $python $calibrator --input $Input --output $Output --reference-serial $ReferenceSerial
exit $LASTEXITCODE
