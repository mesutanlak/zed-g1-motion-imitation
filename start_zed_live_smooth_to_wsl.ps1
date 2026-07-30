param(
    [switch]$Record,
    [switch]$RecordSvo2,
    [switch]$SkipGmrCheck,
    [switch]$NoAnalysisStream,
    [switch]$NoRosStream,
    [switch]$AllowSvoWithIsaac,
    [string]$AnalysisHost = "127.0.0.1",
    [ValidateSet("shared_gpu_safe", "quality")]
    [string]$Profile = "shared_gpu_safe"
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

if (-not $SkipGmrCheck) {
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
    if ($LASTEXITCODE -ne 0) {
        throw @"
WSL Ubuntu-22.04 icinde UDP 15050 dinleyicisi bulunamadi.
Once ayri bir PowerShell'de start_g1_isaaclab_live.ps1 dosyasini baslatin.
"@
    }
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

if ($Profile -eq "shared_gpu_safe") {
    $captureFps = "15"
    $depthMode = "performance"
    $filterTau = "0.025"
    $streamHz = "15"
    $skeletonSmoothing = "0.10"
    Write-Host "ZED Isaac-uyumlu profil: BODY_38 MEDIUM, HD720@15, PERFORMANCE, low-latency"
}
else {
    $captureFps = "30"
    $depthMode = "neural-light"
    $filterTau = "0.055"
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
    "--model", "medium",
    "--fps", $captureFps,
    "--depth-mode", $depthMode,
    "--confidence", "40",
    "--filter-tau", $filterTau,
    "--prediction-timeout", "0.25",
    "--skeleton-smoothing", $skeletonSmoothing,
    "--reduced-precision",
    "--camera-timeout", "5.0",
    "--stream-host", $wslAddress,
    "--stream-port", "15050",
    "--stream-max-hz", $streamHz,
    "--max-corrupt-consecutive", "6"
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
