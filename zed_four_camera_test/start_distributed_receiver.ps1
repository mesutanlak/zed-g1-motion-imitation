param(
    [Parameter(Mandatory = $true)]
    [string[]]$Source,
    [string]$Extrinsics = "",
    [string]$OutputHost = "",
    [ValidateRange(1024, 65535)]
    [int]$OutputPort = 15050,
    [string]$CalibrationRecord = "",
    [ValidateRange(1, 60)]
    [int]$Fps = 15,
    [ValidateSet(2, 3, 4)]
    [int]$MinimumSources = 2,
    [ValidateRange(20, 1000)]
    [double]$MaxSyncMs = 110,
    [ValidateRange(100, 5000)]
    [double]$SourceTimeoutMs = 750,
    [ValidateRange(0, 86400)]
    [double]$Duration = 0
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
$arguments += "--minimum-sources"
$arguments += "$MinimumSources"
$arguments += "--max-sync-ms"
$arguments += "$MaxSyncMs"
$arguments += "--source-timeout-ms"
$arguments += "$SourceTimeoutMs"
$arguments += "--duration"
$arguments += "$Duration"
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
