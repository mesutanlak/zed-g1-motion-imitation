param(
    [Parameter(Mandatory = $true)]
    [long]$Serial,
    [Parameter(Mandatory = $true)]
    [string]$TargetHost,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1024, 65535)]
    [int]$TargetPort,
    [string]$SourceHostId = $env:COMPUTERNAME,
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "medium",
    [ValidateSet(15, 30, 60)]
    [int]$Fps = 15,
    [ValidateSet("neural-light", "neural", "performance")]
    [string]$DepthMode = "performance",
    [ValidateSet("strict", "monitor", "off")]
    [string]$FrameIntegrityMode = "off",
    [ValidateRange(0.3, 20.0)]
    [double]$DistanceMin = 1.0,
    [ValidateRange(0.5, 30.0)]
    [double]$DistanceMax = 5.25,
    [switch]$DisableDistanceGate,
    [string]$PreviewHost = "",
    [ValidateRange(1024, 65535)]
    [int]$PreviewPort = 16100,
    [ValidateRange(1, 15)]
    [double]$PreviewHz = 5,
    [ValidateRange(320, 960)]
    [int]$PreviewWidth = 640,
    [ValidateRange(35, 90)]
    [int]$PreviewJpegQuality = 65,
    [switch]$RecordLocal,
    [switch]$RecordSvo2,
    [string]$OutputDir = "",
    [string]$RecordStem = ""
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$source = Join-Path $root "zed_g1_skeleton.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
if ($DistanceMax -le $DistanceMin) {
    throw "DistanceMax, DistanceMin degerinden buyuk olmali."
}

$arguments = @(
    $source,
    "--serial", "$Serial",
    "--fps", "$Fps",
    "--model", $Model,
    "--depth-mode", $DepthMode,
    "--headless",
    "--frame-integrity-mode", $FrameIntegrityMode,
    "--source-host-id", $SourceHostId,
    "--distance-min", "$DistanceMin",
    "--distance-max", "$DistanceMax",
    "--stream-host", $TargetHost,
    "--stream-port", "$TargetPort",
    "--stream-max-hz", "$Fps"
)
if (-not $DisableDistanceGate) {
    $arguments += "--enforce-distance-gate"
}
if ($PreviewHost) {
    $arguments += @(
        "--preview-stream-host", $PreviewHost,
        "--preview-stream-port", "$PreviewPort",
        "--preview-stream-max-hz", "$PreviewHz",
        "--preview-stream-width", "$PreviewWidth",
        "--preview-jpeg-quality", "$PreviewJpegQuality"
    )
}
if ($OutputDir) {
    $arguments += @("--output-dir", $OutputDir)
}
if ($RecordStem) {
    $arguments += @("--record-stem", $RecordStem)
}
if ($RecordSvo2) {
    $arguments += @("--record", "--record-svo2")
}
elseif ($RecordLocal) {
    $arguments += "--record"
}
& $python @arguments
exit $LASTEXITCODE
