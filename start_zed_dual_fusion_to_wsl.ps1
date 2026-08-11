param(
    [switch]$Record,
    [switch]$SkipGmrCheck,
    [switch]$NoAnalysisStream,
    [switch]$NoRosStream,
    [ValidateSet("dual_safe_15", "dual_balanced_30", "dual_60_experimental")]
    [string]$Profile = "dual_balanced_30",
    [string]$FusionConfig = "",
    [string]$AnalysisHost = "127.0.0.1",
    [ValidateRange(1, 60)]
    [int]$AnalysisHz = 15,
    [ValidateRange(1, 60)]
    [int]$RosHz = 30
)

$ErrorActionPreference = "Stop"
$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $FusionConfig) {
    $FusionConfig = Join-Path $projectDir "config\zed_dual\calibration_33773329_39504762.json"
}
if (-not (Test-Path -LiteralPath $FusionConfig)) {
    throw "Dual ZED Fusion calibration file not found: $FusionConfig"
}
$zedPython = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path $zedPython)) {
    $zedPython = "python"
}
$wslAddressLine = wsl.exe -d Ubuntu-22.04 -- hostname -I
$wslAddress = ($wslAddressLine.Trim() -split "\s+")[0]
if (-not $wslAddress) {
    throw "WSL Ubuntu-22.04 IP address could not be resolved."
}
if (-not $SkipGmrCheck) {
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
    if ($LASTEXITCODE -ne 0) {
        throw @"
WSL Ubuntu-22.04 UDP 15050 listener is not running.
Start start_g1_isaaclab_live.ps1 in a separate PowerShell first.
"@
    }
}

if ($Profile -eq "dual_safe_15") {
    $captureFps = "15"; $model = "medium"; $smoothing = "0.12"; $streamHz = "15"
    Write-Host "Dual ZED safe profile: 2x BODY_38 MEDIUM, HD720@15"
}
elseif ($Profile -eq "dual_60_experimental") {
    $captureFps = "60"; $model = "fast"; $smoothing = "0.05"; $streamHz = "60"
    Write-Warning "2x HD720@60 BODY_38 is experimental. Validate effective FPS, USB drops and GPU load."
}
else {
    $captureFps = "30"; $model = "medium"; $smoothing = "0.08"; $streamHz = "30"
    Write-Host "Dual ZED recommended profile: 2x BODY_38 MEDIUM, HD720@30, NEURAL_LIGHT"
}

$arguments = @(
    ".\zed_dual_body38_fusion.py",
    "--fusion-config", $FusionConfig,
    "--fps", $captureFps,
    "--model", $model,
    "--depth-mode", "neural-light",
    "--confidence", "40",
    "--fusion-smoothing", $smoothing,
    "--filter-tau", "0.0",
    "--operator-acquire-frames", "10",
    "--calibration-seconds", "4.0",
    "--camera-timeout", "5.0",
    "--stream-host", $wslAddress,
    "--stream-port", "15050",
    "--stream-max-hz", $streamHz
)
if (-not $NoAnalysisStream) {
    $arguments += @("--monitor-host", $AnalysisHost, "--monitor-port", "15052", "--monitor-max-hz", "$AnalysisHz")
}
if (-not $NoRosStream) {
    $arguments += @("--ros-host", $wslAddress, "--ros-port", "15054", "--ros-max-hz", "$RosHz")
}
if ($Record) {
    $arguments += "--record"
}

Write-Host "Fusion config: $FusionConfig"
Write-Host "GMR UDP: ${wslAddress}:15050"
Write-Host "Single-camera launcher remains unchanged: start_zed_live_smooth_to_wsl.ps1"
Set-Location -LiteralPath $projectDir
& $zedPython @arguments
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "Dual ZED Fusion exited with code: $exitCode"
}
