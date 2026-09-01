param(
    [Parameter(Mandatory = $true)]
    [long]$Serial,
    [Parameter(Mandatory = $true)]
    [string]$TargetHost,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1024, 65535)]
    [int]$TargetPort,
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "medium",
    [ValidateSet(15, 30, 60)]
    [int]$Fps = 15,
    [ValidateSet("neural-light", "neural", "performance")]
    [string]$DepthMode = "performance",
    [ValidateSet("strict", "monitor", "off")]
    [string]$FrameIntegrityMode = "strict",
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
    [switch]$RecordSvo2
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$source = Join-Path $root "zed_g1_skeleton.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}

$arguments = @(
    $source,
    "--serial", "$Serial",
    "--fps", "$Fps",
    "--model", $Model,
    "--depth-mode", $DepthMode,
    "--headless",
    "--frame-integrity-mode", $FrameIntegrityMode,
    "--stream-host", $TargetHost,
    "--stream-port", "$TargetPort",
    "--stream-max-hz", "$Fps"
)
if ($PreviewHost) {
    $arguments += @(
        "--preview-stream-host", $PreviewHost,
        "--preview-stream-port", "$PreviewPort",
        "--preview-stream-max-hz", "$PreviewHz",
        "--preview-stream-width", "$PreviewWidth",
        "--preview-jpeg-quality", "$PreviewJpegQuality"
    )
}
if ($RecordSvo2) {
    $arguments += @("--record", "--record-svo2")
}
elseif ($RecordLocal) {
    $arguments += "--record"
}
& $python @arguments
exit $LASTEXITCODE
