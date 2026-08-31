param(
    [Parameter(Mandatory = $true)]
    [string[]]$Source,
    [string]$Extrinsics = "",
    [string]$OutputHost = "",
    [ValidateRange(1024, 65535)]
    [int]$OutputPort = 15050,
    [string]$CalibrationRecord = "",
    [ValidateRange(1, 60)]
    [int]$Fps = 15
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$receiver = Join-Path $PSScriptRoot "distributed_body38_fusion.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
if ($Source.Count -ne 4) {
    throw "Dort -Source 'SERIAL:PORT' degeri gerekli."
}

$arguments = @($receiver)
foreach ($item in $Source) {
    $arguments += "--source"
    $arguments += $item
}
if ($Extrinsics) {
    $arguments += "--extrinsics"
    $arguments += $Extrinsics
}
if ($OutputHost) {
    $arguments += "--output-host"
    $arguments += $OutputHost
    $arguments += "--output-port"
    $arguments += "$OutputPort"
    $arguments += "--output-max-hz"
    $arguments += "$Fps"
}
if ($CalibrationRecord) {
    $arguments += "--calibration-record"
    $arguments += $CalibrationRecord
}
& $python @arguments
exit $LASTEXITCODE
