param(
    [switch]$Record,
    [switch]$RecordSvo2,
    [switch]$SkipGmrCheck,
    [switch]$NoAnalysisStream,
    [switch]$NoRosStream,
    [switch]$AllowSvoWithIsaac,
    [string]$AnalysisHost = "127.0.0.1",
    [ValidateSet("akc", "legacy")]
    [string]$ArmRecoveryMode = "akc",
    [ValidateSet("shared_gpu_safe", "balanced_30", "realtime_60", "quality")]
    [string]$Profile = "realtime_60"
)

$ErrorActionPreference = "Stop"

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$zedPython = Join-Path $projectDir ".venv-zed\Scripts\python.exe"
if (-not (Test-Path $zedPython)) {
    $zedPython = "python"
}
$wslAddressLine = wsl.exe -d Ubuntu-22.04 -- hostname -I
$wslAddress = ($wslAddressLine.Trim() -split "\s+")[0]
if (-not $wslAddress) {
    throw "WSL IP adresi alinamadi."
}

# SVO2 dataset capture is intentionally allowed as a standalone camera job.
# Live imitation still requires the GMR listener unless -SkipGmrCheck is explicit.
$standaloneSvoCapture = $RecordSvo2 -and -not $AllowSvoWithIsaac
if (-not $SkipGmrCheck -and -not $standaloneSvoCapture) {
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
    if ($LASTEXITCODE -ne 0) {
        throw @"
WSL Ubuntu-22.04 icinde UDP 15050 dinleyicisi bulunamadi.
Once ayri bir PowerShell'de start_g1_isaaclab_live.ps1 dosyasini baslatin.
"@
    }
}
elseif ($standaloneSvoCapture) {
    Write-Host "Bagimsiz SVO2 veri seti kaydi: GMR/Isaac UDP 15050 dinleyicisi zorunlu degil."
}

$isaacRunning = Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -like "*isaac_g1_23dof_live.py*" }
if ($RecordSvo2 -and $isaacRunning -and -not $AllowSvoWithIsaac) {
    throw @"
SVO2 kaydi Isaac Sim ile ayni GPU'da guvenli canli profilde devre disidir.
Son SVO2 kaydinda yatay kare parcalanmasi dogrulandi.
Canli taklit icin -RecordSvo2 kullanmayin; JSONL kaydi icin yalnizca -Record kullanin.
Kamera veri setini Isaac kapaliyken ayri oturumda SVO2 olarak kaydedin.
"@
}

$bodyModel = "medium"
$predictionTimeout = "0.25"
if ($Profile -eq "shared_gpu_safe") {
    $captureFps = "15"
    $depthMode = "performance"
    $filterTau = "0.025"
    $streamHz = "15"
    $skeletonSmoothing = "0.10"
    Write-Host "ZED Isaac-uyumlu profil: BODY_38 MEDIUM, HD720@15, PERFORMANCE, low-latency"
}
elseif ($Profile -eq "balanced_30") {
    $captureFps = "30"
    $depthMode = "neural-light"
    $filterTau = "0.0"
    $streamHz = "30"
    $skeletonSmoothing = "0.10"
    Write-Host "ZED canli profil: BODY_38 MEDIUM, HD720@30, NEURAL_LIGHT, pelvis-local"
}
elseif ($Profile -eq "realtime_60") {
    # ZED 2i officially exposes HD720@60. FAST + NEURAL_LIGHT keeps the
    # BODY_38 inference path light enough to share the GPU with Isaac. The
    # effective body rate must still be measured; camera FPS alone is not a
    # guarantee that body inference sustained 60 Hz.
    $captureFps = "60"
    $depthMode = "neural-light"
    $filterTau = "0.0"
    $streamHz = "60"
    $skeletonSmoothing = "0.05"
    $bodyModel = "fast"
    $predictionTimeout = "0.12"
    Write-Host "ZED dusuk gecikme profili: BODY_38 FAST, HD720@60, NEURAL_LIGHT, pelvis-local"
    Write-Host "Not: Bu profil kamera 60 FPS ister; etkili BODY FPS ve p95 gecikme olculerek dogrulanmalidir."
}
else {
    $captureFps = "30"
    $depthMode = "neural-light"
    $filterTau = "0.0"
    $streamHz = "30"
    $skeletonSmoothing = "0.15"
    Write-Host "ZED kalite profili: BODY_38 MEDIUM, HD720@30, NEURAL_LIGHT"
}
Write-Host "GMR UDP hedefi: ${wslAddress}:15050"
if (-not $NoAnalysisStream) {
    Write-Host "3B analiz UDP hedefi: ${AnalysisHost}:15052 (Windows native panel)"
}
if (-not $NoRosStream) {
    Write-Host "BODY_38 ROS 2 UDP hedefi: ${wslAddress}:15054"
}
if (-not $Record -and -not $RecordSvo2) {
    Write-Host "Baslangicta kayit kapali (en dusuk gecikme)."
    Write-Host "S tusu yalnizca JSONL kaydini acar. SVO2 icin scripti -RecordSvo2 ile baslatin."
}
elseif ($RecordSvo2) {
    Write-Host "Kayit modu: JSONL + SVO2/H264 baslangicta acik."
}
else {
    Write-Host "Kayit modu: JSONL baslangicta acik; SVO2 kapali."
}
Set-Location -LiteralPath $projectDir
$zedArguments = @(
    ".\zed_g1_skeleton.py",
    "--model", $bodyModel,
    "--fps", $captureFps,
    "--depth-mode", $depthMode,
    "--confidence", "40",
    "--filter-tau", $filterTau,
    "--prediction-timeout", $predictionTimeout,
    "--arm-recovery-mode", $ArmRecoveryMode,
    "--skeleton-smoothing", $skeletonSmoothing,
    "--operator-acquire-frames", "10",
    "--calibration-seconds", "4.0",
    "--reduced-precision",
    "--camera-timeout", "5.0",
    "--stream-host", $wslAddress,
    "--stream-port", "15050",
    "--stream-max-hz", $streamHz,
    # Isolated USB/UVC corruption is dropped while the last valid frame and
    # safe robot command are held. Restart only after roughly one full second.
    "--max-corrupt-consecutive", "30"
)
if (-not $NoAnalysisStream) {
    $zedArguments += @(
        "--monitor-host", $AnalysisHost,
        "--monitor-port", "15052"
    )
}
if (-not $NoRosStream) {
    $zedArguments += @(
        "--ros-host", $wslAddress,
        "--ros-port", "15054"
    )
}

if ($Record -or $RecordSvo2) {
    $zedArguments += "--record"
}
if ($RecordSvo2) {
    $zedArguments += @("--record-svo2", "--svo-compression", "h264")
}

$maxAttempts = 3
for ($attempt = 1; $attempt -le $maxAttempts; $attempt++) {
    & $zedPython @zedArguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -eq 0) {
        exit 0
    }
    if ($exitCode -ne 7 -or $attempt -eq $maxAttempts) {
        throw "ZED uygulamasi hata koduyla kapandi: $exitCode"
    }
    Write-Warning (
        "ZED USB/CUDA akisi bozuldu. Kamera yeniden aciliyor " +
        "(deneme $($attempt + 1)/$maxAttempts)..."
    )
    Start-Sleep -Seconds 2
}
