param(
    [ValidateSet("upper_body", "whole_body")]
    [string]$Mode = "upper_body",
    [ValidateSet("dynamic", "kinematic_debug")]
    [string]$ImitationMode = "kinematic_debug",
    [switch]$AcceptNvidiaEula,
    [switch]$Headless,
    [switch]$AllowUnvalidatedDriver,
    [switch]$ResetIsaacUserConfig,
    [switch]$NoFallArrest,
    [switch]$NoMirrorRescue,
    [switch]$NoAnatomicalBranchContinuity,
    [switch]$RestrictBackwardArms,
    [ValidateRange(1, 16)]
    [int]$MirrorWorkers = 4,
    [ValidateSet("shared_gpu_safe", "gpu_max")]
    [string]$RuntimeProfile = "shared_gpu_safe",
    [ValidateSet("fixed_double_support", "balance_policy")]
    [string]$StanceMode = "fixed_double_support",
    [ValidateRange(1.0, 60.0)]
    [double]$InputFps = 15.0,
    [ValidateRange(0.5, 20.0)]
    [double]$UpperCutoffHz = 10.0,
    [ValidateRange(0.1, 10.0)]
    [double]$UpperMinCutoffHz = 2.0,
    [ValidateRange(0.0, 10.0)]
    [double]$UpperVelocityBeta = 1.2,
    [ValidateRange(0.0, 3.0)]
    [double]$StationaryDeadbandScale = 1.0,
    [ValidateRange(0.0, 1.0)]
    [double]$MimicBlend = 1.0,
    [ValidateSet("low_latency", "smooth_bounded")]
    [string]$ReferenceTrackingMode = "low_latency",
    [ValidateRange(0.2, 10.0)]
    [double]$ReferenceResponseHz = 5.5,
    [ValidateRange(0.2, 5.0)]
    [double]$ReferenceMaxVelocity = 0.85,
    [ValidateRange(0.5, 80.0)]
    [double]$ReferenceMaxAcceleration = 4.0,
    [ValidateRange(1.0, 500.0)]
    [double]$ReferenceMaxJerk = 35.0,
    [ValidateRange(0.1, 5.0)]
    [double]$UpperStiffnessScale = 2.0,
    [ValidateRange(0.1, 5.0)]
    [double]$UpperDampingScale = 2.0,
    [ValidateRange(1, 8)]
    [int]$RenderInterval = 8,
    [string]$ReferencePolicyPath = "",
    [string]$ReferencePolicyMetadata = "",
    [ValidateRange(0.0, 1.0)]
    [double]$ReferencePolicyBlend = 1.0,
    [switch]$PolicyPowered,
    [ValidateRange(0.0, 3.0)]
    [double]$StaleReturnDelay = 0.25,
    [ValidateRange(0.05, 5.0)]
    [double]$StaleReturnTau = 0.60,
    [ValidateRange(1.2, 2.2)]
    [double]$HumanHeightM = 1.80,
    [string]$InstallRoot = "C:\g1il",
    [switch]$Dex3,
    [string]$UnitreeSimRoot = "",
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
$nativeReferencePolicy = if ([string]::IsNullOrWhiteSpace($ReferencePolicyPath)) {
    Join-Path $project "policies\g1_reference_upper_body\policy.onnx"
} else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $ReferencePolicyPath))
}
$nativeReferencePolicyMetadata = if ([string]::IsNullOrWhiteSpace($ReferencePolicyMetadata)) {
    Join-Path $project "policies\g1_reference_upper_body\policy_metadata.json"
} else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $ReferencePolicyMetadata))
}
if ($PolicyPowered -and (
    -not (Test-Path -LiteralPath $nativeReferencePolicy) -or
    -not (Test-Path -LiteralPath $nativeReferencePolicyMetadata)
)) {
    throw "PolicyPowered istendi fakat egitilmis policy/metadata bulunamadi: $nativeReferencePolicy"
}
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
$nativeUnitreeSimRoot = if ([string]::IsNullOrWhiteSpace($UnitreeSimRoot)) {
    Join-Path $resolvedInstallRoot "repos\unitree_sim_isaaclab"
} else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $UnitreeSimRoot))
}
$nativeDex3Usd = Join-Path $nativeUnitreeSimRoot "assets\robots\g1-29dof-dex3-base-fix-usd\g1_29dof_with_dex3_base_fix.usd"
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
    Write-Host "GUI daha once bu bilgisayarda calistiysa, riski kabul ederek devam:"
    Write-Host "  .\start_g1_isaaclab_live.ps1 -Mode upper_body -AcceptNvidiaEula -AllowUnvalidatedDriver"
    Write-Host "  Not: device-lost olursa bu secenegi tekrar kullanmayin; Headless calistirin."
    Write-Host ""
    exit 3
}
if ($Dex3 -and -not (Test-Path -LiteralPath $nativeDex3Usd -PathType Leaf)) {
    throw "Resmi Unitree Dex3 USD bulunamadi: $nativeDex3Usd`nOnce .\install\install_unitree_dex3_sim.ps1 calistirin."
}

if (-not $Headless -and $driverVersion.Major -ge 595 -and $AllowUnvalidatedDriver) {
    Write-Warning (
        "NVIDIA $driverText ile GUI kullanici onayiyla baslatiliyor. " +
        "GPU device-lost olursa Headless moda gecin."
    )
}

Write-Host "G1 Isaac Lab canli zinciri"
Write-Host "  ZED -> WSL/GMR : WSL_IP:15050"
Write-Host "  WSL/GMR -> Isaac: ${windowsHost}:15051"
Write-Host "  Mod: $Mode"
Write-Host "  Imitation fizigi: $ImitationMode"
Write-Host "  Asset: $(if ($Dex3) { 'Resmi Unitree G1-29DOF + Dex3 (DDS KAPALI)' } else { 'G1-23DOF' })"
Write-Host "  NVIDIA surucu: $driverText"
    Write-Host "  Grafik API: D3D12"
Write-Host "  Calisma profili: $RuntimeProfile"
Write-Host "  Alt beden: $StanceMode"
Write-Host "  Canli takip: ${InputFps} Hz, adaptive cutoff=${UpperMinCutoffHz}-${UpperCutoffHz} Hz, beta=$UpperVelocityBeta, stationary deadband=$StationaryDeadbandScale, blend=$MimicBlend"
Write-Host "  Insan boyu / GMR olcegi: ${HumanHeightM} m"
Write-Host "  Ust govde PD olcegi: Kp=$UpperStiffnessScale Kd=$UpperDampingScale"
Write-Host "  Isaac referans profili: Unitree G1 fiziksel zarf | mode=$ReferenceTrackingMode response=${ReferenceResponseHz}Hz vel<=${ReferenceMaxVelocity}rad/s acc<=${ReferenceMaxAcceleration}rad/s2 jerk<=${ReferenceMaxJerk}rad/s3"
Write-Host "  Isaac zamanlama: 200 Hz fizik, render her $RenderInterval adim (~$([math]::Round(200.0 / $RenderInterval)) Hz GUI)"
if (Test-Path -LiteralPath $nativeReferencePolicy) {
    Write-Host "  Kontrol: Dual ZED veya Isaac penceresinde P = Normal IK <-> Policy Powered"
    Write-Host "  Policy: $nativeReferencePolicy"
} else {
    Write-Host "  Kontrol: Normal IK (policy egitimden sonra P aktif olacak)"
}
Write-Host "  AKC/GMR dirsek dal surekliligi: $(-not $NoAnatomicalBranchContinuity)"
Write-Host "  Olay tetiklemeli continuation rescue: $(-not $NoMirrorRescue) (workers=$MirrorWorkers)"
Write-Host "  Arka kol erisimi: $(if ($RestrictBackwardArms) { 'eski +0.75rad omuz siniri' } else { 'ACIK; eklem limiti + govde carpisma bariyeri' })"
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
    "--stationary-deadband-scale", "$StationaryDeadbandScale",
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
if ($RestrictBackwardArms) {
    $bridgeArguments += "--restrict-backward-arms"
}
else {
    $bridgeArguments += "--no-restrict-backward-arms"
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
    $kitSettings = @("--/app/vulkan=false")
    if ($ResetIsaacUserConfig) {
        # NVIDIA's supported recovery option for incompatible/corrupt Kit
        # preferences and shader-related configuration after driver changes.
        $kitSettings += "--reset-user"
    }
    if (-not $Headless -and $RuntimeProfile -eq "shared_gpu_safe") {
        # Avoid allocating two 4K render targets on the dual-monitor desktop.
        $kitSettings += @(
            "--/app/window/width=1920",
            "--/app/window/height=1080",
            "--/app/renderer/resolution/width=1920",
            "--/app/renderer/resolution/height=1080"
        )
    }
    $kitSettings += @(
        "--/renderer/activeGpu=0",
        "--/rtx/post/dlss/execMode=0",
        "--/renderer/raytracingMotion/enabled=false",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=false"
    )
    $kitSettings = $kitSettings -join " "
    $isaacArguments = @(
        "$resolvedProject\isaaclab_bridge\isaac_g1_23dof_live.py",
        "--device", $simulationDevice,
        "--rendering_mode", $renderingMode,
        "--mode", $Mode,
        "--imitation-mode", $ImitationMode,
        "--listen-host", "0.0.0.0",
        "--listen-port", "15051",
        "--telemetry-host", "127.0.0.1",
        "--telemetry-port", "15053",
        "--urdf", $nativeUrdf,
        "--usd", $nativeUsd,
        "--asset-profile", $(if ($Dex3) { "g1_29dof_dex3" } else { "g1_23dof" }),
        "--unitree-sim-root", $nativeUnitreeSimRoot,
        "--balance-policy", $nativePolicy,
        "--balance-config", $nativePolicyCfg,
        "--reference-policy", $nativeReferencePolicy,
        "--reference-policy-metadata", $nativeReferencePolicyMetadata,
        "--reference-policy-blend", "$ReferencePolicyBlend",
        "--stance-mode", $StanceMode,
        "--mimic-blend", "$MimicBlend",
        "--upper-stiffness-scale", "$UpperStiffnessScale",
        "--upper-damping-scale", "$UpperDampingScale",
        "--stale-return-delay", "$StaleReturnDelay",
        "--stale-return-tau", "$StaleReturnTau",
        "--input-fps", "$InputFps",
        "--reference-tracking-mode", $ReferenceTrackingMode,
        "--reference-response-hz", "$ReferenceResponseHz",
        "--reference-max-velocity", "$ReferenceMaxVelocity",
        "--reference-max-acceleration", "$ReferenceMaxAcceleration",
        "--reference-max-jerk", "$ReferenceMaxJerk",
        "--render-interval", "$RenderInterval",
        "--kit_args=$kitSettings"
    )
    if ($Headless) {
        $isaacArguments += "--headless"
    }
    if ($PolicyPowered) {
        $isaacArguments += "--reference-policy-start-enabled"
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
