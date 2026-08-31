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
    [string]$FrameIntegrityMode = "strict"
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$source = Join-Path $root "zed_g1_skeleton.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}

& $python $source `
    --serial $Serial `
    --fps $Fps `
    --model $Model `
    --depth-mode $DepthMode `
    --headless `
    --frame-integrity-mode $FrameIntegrityMode `
    --stream-host $TargetHost `
    --stream-port $TargetPort `
    --stream-max-hz $Fps
exit $LASTEXITCODE
