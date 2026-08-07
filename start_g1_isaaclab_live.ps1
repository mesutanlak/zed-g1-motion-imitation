param(
    [ValidateSet("upper_body", "whole_body")]
    [string]$Mode = "upper_body",
    [switch]$AcceptNvidiaEula,
    [switch]$Headless,
    [switch]$AllowUnvalidatedDriver,
    [switch]$NoFallArrest,
    [switch]$NoMirrorRescue,
    [switch]$NoAnatomicalBranchContinuity,
    [ValidateRange(1, 16)]
    [int]$MirrorWorkers = 4,
    [ValidateSet("shared_gpu_safe", "gpu_max")]
    [string]$RuntimeProfile = "shared_gpu_safe",
    [ValidateSet("fixed_double_support", "balance_policy")]
    [string]$StanceMode = "fixed_double_support",
    [ValidateRange(1.0, 60.0)]
    [double]$InputFps = 60.0,
    [ValidateRange(0.5, 20.0)]
    [double]$UpperCutoffHz = 10.0,
    [ValidateRange(0.1, 10.0)]
    [double]$UpperMinCutoffHz = 2.0,
    [ValidateRange(0.0, 10.0)]
    [double]$UpperVelocityBeta = 1.2,
    [ValidateRange(0.0, 1.0)]
    [double]$MimicBlend = 1.0,
    [ValidateRange(0.1, 5.0)]
    [double]$UpperStiffnessScale = 2.0,
    [ValidateRange(0.1, 5.0)]
    [double]$UpperDampingScale = 2.0,
    [ValidateRange(0.0, 3.0)]
    [double]$StaleReturnDelay = 0.25,
    [ValidateRange(0.05, 5.0)]
    [double]$StaleReturnTau = 0.60,
    [ValidateRange(1.2, 2.2)]
    [double]$HumanHeightM = 1.80,
    [string]$InstallRoot = "C:\g1il",
    [ValidateRange(0, 1000000)]
    [int]$MaxSteps = 0
)

$ErrorActionPreference = "Stop"
if (-not $AcceptNvidiaEula) {
    Write-Warning @"
Isaac Sim ilk calistirmada NVIDIA Omniverse EULA kabulunu gerektirir:
https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html
Kosullari okuyup kabul ediyorsaniz komutu -AcceptNvidiaEula ile yeniden calistirin.
"@
    exit 2
}

$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProject = [System.IO.Path]::GetFullPath($project)
if ($resolvedProject -notmatch "^[A-Za-z]:\\") {
    throw "Windows proje yolu taninamadi: $resolvedProject"
}
$drive = $resolvedProject.Substring(0, 1).ToLowerInvariant()
$rest = $resolvedProject.Substring(2).Replace("\", "/")
$wslProject = "/mnt/$drive$rest"

$resolvedInstallRoot = [System.IO.Path]::GetFullPath($InstallRoot)
$nativePython = Join-Path $resolvedInstallRoot "env\Scripts\python.exe"
$nativeUrdf = Join-Path $resolvedInstallRoot "repos\unitree_ros\robots\g1_description\g1_23dof_rev_1_0.urdf"
$nativeUsd = Join-Path $resolvedInstallRoot "cache\g1_23dof\g1_23dof_rev_1_0.usd"
$nativePolicy = "$resolvedProject\policies\g1_23dof_velocity\policy.onnx"
$nativePolicyCfg = "$resolvedProject\policies\g1_23dof_velocity\deploy.yaml"
$wslHome = (& wsl.exe -d Ubuntu-22.04 -- bash -lc 'printf %s "$HOME"').Trim()
if (-not $wslHome.StartsWith("/")) {
    throw "WSL kullanici dizini bulunamadi: $wslHome"
}
$gmrPython = "$wslHome/g1_isaaclab_project/envs/gmr_zed/bin/python"
$gmrScript = "$wslProject/isaaclab_bridge/gmr_live_bridge.py"

foreach ($requiredPath in @(
    $nativePython,
    $nativeUrdf,
    $nativeUsd,
    $nativePolicy,
    $nativePolicyCfg
)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Gerekli Isaac Lab dosyasi bulunamadi: $requiredPath"
    }
}

$existingIsaac = Get-CimInstance Win32_Process |
    Where-Object {
        $_.CommandLine -like "*isaac_g1_23dof_live.py*" -and
        $_.ProcessId -ne $PID
    }
if ($existingIsaac) {
    $ids = ($existingIsaac.ProcessId -join ", ")
    throw "Baska bir G1 Isaac Lab sureci zaten calisiyor (PID: $ids). Once onu kapatin."
}

& wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
if ($LASTEXITCODE -eq 0) {
    throw "WSL UDP 15050 zaten kullanimda. Eski GMR koprusunu kapatip yeniden deneyin."
}

$routeLine = (& wsl.exe -d Ubuntu-22.04 -- ip route show default | Select-Object -First 1)
$routeParts = $routeLine.Trim() -split "\s+"
if ($routeParts.Count -lt 3 -or $routeParts[1] -ne "via") {
    throw "WSL -> Windows ag gecidi bulunamadi: $routeLine"
}
$windowsHost = $routeParts[2]

$driverText = (& nvidia-smi --query-gpu=driver_version --format=csv,noheader |
    Select-Object -First 1).Trim()
$driverVersion = $null
if (-not [version]::TryParse($driverText, [ref]$driverVersion)) {
    throw "NVIDIA surucu surumu okunamadi: $driverText"
}
if (-not $Headless -and $driverVersion.Major -ge 595 -and -not $AllowUnvalidatedDriver) {
    Write-Host ""
    Write-Host "ISAAC GUI BASLATILMADI" -ForegroundColor Red
    Write-Host "NVIDIA surucu: $driverText"
    Write-Host "Bu surucu Isaac Sim 5.x ile testlerimizde GPU device-lost olusturdu."
    Write-Host ""
    Write-Host "GUI cozumu:" -ForegroundColor Yellow
    Write-Host "  NVIDIA 581.42 WHQL surucusunu temiz kurun ve Windows'u yeniden baslatin."
    Write-Host "  https://www.nvidia.com/en-us/drivers/details/257264/"
    Write-Host ""
    Write-Host "Surucu degisene kadar ekransiz calistirma:"
    Write-Host "  .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula -Headless"
    Write-Host ""
    exit 3
}

Write-Host "G1 Isaac Lab canli zinciri"
Write-Host "  ZED -> WSL/GMR : WSL_IP:15050"
Write-Host "  WSL/GMR -> Isaac: ${windowsHost}:15051"
Write-Host "  Mod: $Mode"
Write-Host "  NVIDIA surucu: $driverText"
Write-Host "  Grafik API: D3D12 (Vulkan yolu devre disi)"
Write-Host "  Calisma profili: $RuntimeProfile"
Write-Host "  Alt beden: $StanceMode"
Write-Host "  Canli takip: ${InputFps} Hz, adaptive cutoff=${UpperMinCutoffHz}-${UpperCutoffHz} Hz, beta=$UpperVelocityBeta, blend=$MimicBlend"
Write-Host "  Insan boyu / GMR olcegi: ${HumanHeightM} m"
Write-Host "  Ust govde PD olcegi: Kp=$UpperStiffnessScale Kd=$UpperDampingScale"
Write-Host "  AKC/GMR dirsek dal surekliligi: $(-not $NoAnatomicalBranchContinuity)"
Write-Host "  Olay tetiklemeli continuation rescue: $(-not $NoMirrorRescue) (workers=$MirrorWorkers)"
if (-not $Headless) {
    Write-Host "  GUI notu: ilk D3D12/RTX onbellek acilisi 3-4 dakika surebilir."
    Write-Host "  'G1 scene initialization complete' gorulene kadar pencereyi kapatmayin."
}

$bridgeArguments = @(
    "-d", "Ubuntu-22.04", "--",
    "env", "PYTHONUNBUFFERED=1",
    $gmrPython, $gmrScript,
    "--listen-host", "0.0.0.0",
    "--listen-port", "15050",
    "--output-host", $windowsHost,
    "--output-port", "15051",
    "--telemetry-host", $windowsHost,
    "--telemetry-port", "15053",
    "--mode", $Mode,
    "--input-fps", "$InputFps",
    "--cutoff-hz", "$UpperCutoffHz",
    "--min-cutoff-hz", "$UpperMinCutoffHz",
    "--velocity-beta", "$UpperVelocityBeta",
    "--human-height", "$HumanHeightM",
    "--no-gmr-velocity-limit"
)
if ($NoMirrorRescue) {
    $bridgeArguments += "--no-mirror-rescue"
}
else {
    $bridgeArguments += @("--mirror-rescue", "--mirror-workers", "$MirrorWorkers")
}
if ($NoAnatomicalBranchContinuity) {
    $bridgeArguments += "--no-anatomical-branch-continuity"
}
else {
    $bridgeArguments += "--anatomical-branch-continuity"
}

$bridgeProcess = $null
try {
    $bridgeProcess = Start-Process `
        -FilePath "wsl.exe" `
        -ArgumentList $bridgeArguments `
        -NoNewWindow `
        -PassThru
    $bridgeReady = $false
    for ($attempt = 0; $attempt -lt 40; $attempt++) {
        Start-Sleep -Milliseconds 500
        if ($bridgeProcess.HasExited) {
            throw "GMR koprusu erken kapandi (kod=$($bridgeProcess.ExitCode))."
        }
        & wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
        if ($LASTEXITCODE -eq 0) {
            $bridgeReady = $true
            break
        }
    }
    if (-not $bridgeReady) {
        throw "GMR koprusu 20 saniye icinde WSL UDP 15050 portunu acamadi."
    }
    Write-Host "  WSL GMR UDP 15050: HAZIR"

    $env:OMNI_KIT_ACCEPT_EULA = "YES"
    $env:PRIVACY_CONSENT = "Y"
    $env:G1IL_ROOT = $resolvedInstallRoot
    # Isaac Sim 5.0 on this Windows host fails to construct the ground stage
    # reliably with CPU PhysX. Keep the validated CUDA physics path; the shared
    # profile saves GPU time through rendering settings instead.
    $simulationDevice = "cuda:0"
    $renderingMode = if ($RuntimeProfile -eq "shared_gpu_safe") {
        "performance"
    } else {
        "balanced"
    }
    $kitSettings = @(
        "--/app/vulkan=false",
        "--/rtx/post/dlss/execMode=0",
        "--/renderer/raytracingMotion/enabled=false",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=false"
    ) -join " "
    $isaacArguments = @(
        "$resolvedProject\isaaclab_bridge\isaac_g1_23dof_live.py",
        "--device", $simulationDevice,
        "--rendering_mode", $renderingMode,
        "--mode", $Mode,
        "--listen-host", "0.0.0.0",
        "--listen-port", "15051",
        "--telemetry-host", "127.0.0.1",
        "--telemetry-port", "15053",
        "--urdf", $nativeUrdf,
        "--usd", $nativeUsd,
        "--balance-policy", $nativePolicy,
        "--balance-config", $nativePolicyCfg,
        "--stance-mode", $StanceMode,
        "--mimic-blend", "$MimicBlend",
        "--upper-stiffness-scale", "$UpperStiffnessScale",
        "--upper-damping-scale", "$UpperDampingScale",
        "--stale-return-delay", "$StaleReturnDelay",
        "--stale-return-tau", "$StaleReturnTau",
        "--kit_args=$kitSettings"
    )
    if ($Headless) {
        $isaacArguments += "--headless"
    }
    if ($MaxSteps -gt 0) {
        $isaacArguments += @("--max-steps", "$MaxSteps")
    }
    if ($NoFallArrest) {
        $isaacArguments += "--no-fall-arrest"
    }

    & $nativePython @isaacArguments
    exit $LASTEXITCODE
}
finally {
    if ($null -ne $bridgeProcess -and -not $bridgeProcess.HasExited) {
        Stop-Process -Id $bridgeProcess.Id -Force
        $bridgeProcess.WaitForExit()
    }
}
