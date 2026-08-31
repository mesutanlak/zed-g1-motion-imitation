param(
    [switch]$Record,
    [switch]$SkipGmrCheck,
    [switch]$NoAnalysisStream,
    [switch]$NoRosStream,
    [switch]$Headless,
    [ValidateSet("dual_safe_15", "dual_balanced_30", "dual_fast_30", "dual_60_experimental")]
    [string]$Profile = "dual_balanced_30",
    [string]$FusionConfig = "",
    [switch]$ValidateFusionConfigOnly,
    [string]$TeleopConfig = "",
    [string]$AnalysisHost = "127.0.0.1",
    [ValidateRange(1, 60)]
    [int]$AnalysisHz = 30,
    [ValidateRange(1, 60)]
    [int]$RosHz = 30,
    [ValidateRange(1, 30)]
    [double]$PreviewHz = 15
)

$ErrorActionPreference = "Stop"
$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $FusionConfig) {
    $calibrationInbox = Join-Path $projectDir "dual json"
    if (-not (Test-Path -LiteralPath $calibrationInbox -PathType Container)) {
        New-Item -ItemType Directory -Path $calibrationInbox -Force | Out-Null
    }

    $calibrationFiles = @(
        Get-ChildItem -LiteralPath $calibrationInbox -File -Filter "*.json" |
            Sort-Object -Property Name
    )
    if ($calibrationFiles.Count -eq 0) {
        throw "Kalibrasyon bulunamadi. '$calibrationInbox' klasorune tam olarak bir ZED Fusion JSON dosyasi koyun."
    }
    if ($calibrationFiles.Count -gt 1) {
        $names = ($calibrationFiles.Name -join ", ")
        throw "Birden fazla kalibrasyon bulundu ($names). Yanlis extrinsic secilmemesi icin '$calibrationInbox' klasorunde yalnizca bir JSON birakin."
    }

    $FusionConfig = $calibrationFiles[0].FullName
    Write-Host "Dual ZED kalibrasyon klasorunden otomatik secildi: $FusionConfig"
}
if (-not (Test-Path -LiteralPath $FusionConfig)) {
    throw "Dual ZED Fusion calibration file not found: $FusionConfig"
}
$FusionConfig = (Resolve-Path -LiteralPath $FusionConfig).Path

try {
    $fusionJson = Get-Content -LiteralPath $FusionConfig -Raw | ConvertFrom-Json
}
catch {
    throw "Dual ZED kalibrasyon JSON dosyasi okunamadi: $FusionConfig`n$($_.Exception.Message)"
}

$fusionEntries = @(
    $fusionJson.PSObject.Properties |
        ForEach-Object { $_.Value.FusionConfiguration } |
        Where-Object { $null -ne $_ }
)
if ($fusionEntries.Count -lt 2) {
    throw "Dual ZED kalibrasyonu en az iki FusionConfiguration kamera kaydi icermeli: $FusionConfig"
}
$fusionSerials = @($fusionEntries | ForEach-Object { [string]$_.serial_number })
if (($fusionSerials | Where-Object { [string]::IsNullOrWhiteSpace($_) }).Count -gt 0) {
    throw "Dual ZED kalibrasyonunda seri numarasi eksik: $FusionConfig"
}
if (($fusionSerials | Sort-Object -Unique).Count -ne $fusionSerials.Count) {
    throw "Dual ZED kalibrasyonunda yinelenen kamera seri numarasi var: $($fusionSerials -join ', ')"
}
foreach ($entry in $fusionEntries) {
    $poseValues = @(([string]$entry.pose -split '\s+') | Where-Object { $_ })
    if ($poseValues.Count -ne 16) {
        throw "Kamera $($entry.serial_number) icin pose 4x4 matris (16 deger) degil: $FusionConfig"
    }
}
Write-Host "Kalibrasyon dogrulandi. Kameralar: $($fusionSerials -join ', ')"

if ($ValidateFusionConfigOnly) {
    Write-Host "Kalibrasyon kontrolu tamamlandi; kameralar baslatilmadi."
    exit 0
}

if (-not $TeleopConfig) {
    $TeleopConfig = Join-Path $projectDir "config\dual_teleoperation.json"
}
if (-not (Test-Path -LiteralPath $TeleopConfig)) {
    throw "Dual teleoperation config file not found: $TeleopConfig"
}
$TeleopConfig = (Resolve-Path -LiteralPath $TeleopConfig).Path
# The ZED SDK configuration reader cannot open paths containing non-ASCII
# characters on Windows. Keep the source JSON in the project and provide the
# SDK with a fresh working copy under the ASCII-only Isaac cache path.
if ($FusionConfig -match "[^\x00-\x7F]") {
    $fusionCache = "C:\g1il\cache\zed_dual"
    New-Item -ItemType Directory -Force -Path $fusionCache | Out-Null
    $sdkFusionConfig = Join-Path $fusionCache "fusion_config.json"
    Copy-Item -LiteralPath $FusionConfig -Destination $sdkFusionConfig -Force
    Write-Host "Fusion kaynak dosyasi: $FusionConfig"
    Write-Host "ZED SDK ASCII calisma kopyasi: $sdkFusionConfig"
    $FusionConfig = $sdkFusionConfig
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
    $captureFps = "15"; $model = "medium"; $depth = "ultra"; $smoothing = "0.12"; $streamHz = "15"
    Write-Host "Dual ZED quality profile: 2x BODY_38 MEDIUM, ULTRA, HD720@15"
}
elseif ($Profile -eq "dual_60_experimental") {
    $captureFps = "60"; $model = "fast"; $depth = "neural-light"; $smoothing = "0.05"; $streamHz = "60"
    Write-Warning "2x HD720@60 BODY_38 is experimental. Validate effective FPS, USB drops and GPU load."
}
elseif ($Profile -eq "dual_fast_30") {
    $captureFps = "30"; $model = "fast"; $depth = "neural-light"; $smoothing = "0.10"; $streamHz = "30"
    Write-Warning "FAST@30 prioritizes rate over arm geometry. Use only after validating BODY_38 bone stability."
}
else {
    # Do not add a 20 Hz software throttle: both cameras were measured at the
    # same ~24 BODY_38/s, so latest-valid output may use every produced frame.
    $captureFps = "30"; $model = "medium"; $depth = "ultra"; $smoothing = "0.10"; $streamHz = "30"
    Write-Host "Dual ZED quality profile: 2x BODY_38 MEDIUM, ULTRA, HD720@30 capture, latest-valid control <=30 Hz"
}

$arguments = @(
    ".\zed_dual_body38_fusion.py",
    "--fusion-config", $FusionConfig,
    "--teleop-config", $TeleopConfig,
    "--fps", $captureFps,
    "--model", $model,
    "--depth-mode", $depth,
    "--confidence", "40",
    "--fusion-smoothing", $smoothing,
    "--filter-tau", "0.0",
    "--operator-acquire-frames", "10",
    "--fallback-serial", "33773329",
    "--max-cross-view-mpjpe", "0.12",
    "--max-camera-sync-ms", "50",
    "--calibration-seconds", "4.0",
    "--camera-timeout", "5.0",
    "--stream-host", $wslAddress,
    "--stream-port", "15050",
    "--stream-max-hz", $streamHz,
    "--preview-hz", "$PreviewHz"
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
if ($Headless) {
    $arguments += "--headless"
}

Write-Host "Fusion config: $FusionConfig"
Write-Host "GMR UDP: ${wslAddress}:15050"
Write-Host "Gercek zamanli oncelik: kamera alimlari paralel, GMR paketi kayit/analizden once; preview=${PreviewHz}Hz"
Write-Host "Single-camera launcher remains unchanged: start_zed_live_smooth_to_wsl.ps1"
if (-not $Headless) {
    Write-Host "Dual preview: both ZED images and BODY_38 skeletons will open in one window."
}
Set-Location -LiteralPath $projectDir
& $zedPython @arguments
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "Dual ZED Fusion exited with code: $exitCode"
}
