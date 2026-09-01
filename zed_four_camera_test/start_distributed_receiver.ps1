param(
    [Parameter(Mandatory = $true)]
    [string[]]$Source,
    [string[]]$PreviewSource = @(),
    [string]$Extrinsics = "",
    [string]$OutputHost = "",
    [ValidateRange(1024, 65535)]
    [int]$OutputPort = 15050,
    [string]$MonitorHost = "",
    [ValidateRange(1024, 65535)]
    [int]$MonitorPort = 15052,
    [ValidateRange(1, 60)]
    [int]$MonitorHz = 15,
    [string]$RosHost = "",
    [ValidateRange(1024, 65535)]
    [int]$RosPort = 15054,
    [ValidateRange(1, 60)]
    [int]$RosHz = 15,
    [string]$CalibrationRecord = "",
    [ValidateRange(1, 60)]
    [int]$Fps = 15,
    [ValidateSet(2, 3, 4)]
    [int]$MinimumSources = 2,
    [ValidateRange(20, 1000)]
    [double]$MaxSyncMs = 110,
    [ValidateRange(100, 5000)]
    [double]$SourceTimeoutMs = 750,
    [ValidateRange(1, 30)]
    [double]$PreviewHz = 10,
    [string]$OutputDir = "",
    [string]$RecordStem = "four_body38_fusion",
    [switch]$Record,
    [switch]$Headless,
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
foreach ($item in $PreviewSource) {
    $arguments += "--preview-source"
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
if ($MonitorHost) {
    $arguments += @("--monitor-host", $MonitorHost, "--monitor-port", "$MonitorPort", "--monitor-max-hz", "$MonitorHz")
}
if ($RosHost) {
    $arguments += @("--ros-host", $RosHost, "--ros-port", "$RosPort", "--ros-max-hz", "$RosHz")
}
if ($CalibrationRecord) {
    $arguments += "--calibration-record"
    $arguments += $CalibrationRecord
}
$arguments += @("--preview-hz", "$PreviewHz", "--record-stem", $RecordStem)
if ($OutputDir) {
    $arguments += @("--output-dir", $OutputDir)
}
if ($Record) {
    $arguments += "--record"
}
if ($Headless) {
    $arguments += "--headless"
}
& $python @arguments
exit $LASTEXITCODE
