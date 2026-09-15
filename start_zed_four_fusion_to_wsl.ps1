param(
    [switch]$Record,
    [switch]$SkipGmrCheck,
    [switch]$NoAnalysisStream,
    [switch]$NoRosStream,
    [switch]$Headless,
    [ValidateSet(2, 3, 4)]
    [int]$MinimumSources = 3,
    [ValidateRange(1, 15)]
    [int]$FusionHz = 15,
    [ValidateRange(1, 15)]
    [int]$PreviewHz = 10,
    [ValidateRange(0, 80)]
    [double]$PreferredFullSetSpreadMs = 40,
    [ValidateRange(0, 50)]
    [double]$FullSetWaitMs = 20,
    [ValidateRange(0.0, 20.0)]
    [double]$WorkspaceXMinM = 2.0,
    [ValidateRange(0.1, 30.0)]
    [double]$WorkspaceXMaxM = 4.0,
    [ValidateRange(0.0, 1.0)]
    [double]$WorkspaceHysteresisM = 0.15,
    [switch]$HandTracking,
    [switch]$DisableDex3Retargeting,
    [ValidateRange(10, 150)]
    [double]$HandMaxAgeMs = 120,
    [ValidateRange(5, 80)]
    [double]$HandMaxSpreadMs = 70,
    [switch]$DisableSingleViewHandDepth,
    [string]$Dex3OfficialRoot = "",
    [string]$Dex3OfficialPython = "",
    [string]$Extrinsics = "",
    [string]$FusionConfig = "",
    [long]$ReferenceSerial = 33773329,
    [string]$AnalysisHost = "127.0.0.1",
    [switch]$ValidateCalibrationOnly
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
if ($WorkspaceXMaxM -le $WorkspaceXMinM) {
    throw "WorkspaceXMaxM, WorkspaceXMinM degerinden buyuk olmali."
}
$expectedSerials = @("31571870", "33773329", "34760587", "39504762")
$calibrationOrigin = ""
if (-not $Extrinsics) {
    $calibrationInbox = Join-Path $project "four json"
    if (-not (Test-Path -LiteralPath $calibrationInbox -PathType Container)) {
        New-Item -ItemType Directory -Path $calibrationInbox -Force | Out-Null
    }

    if (-not $FusionConfig) {
        $fusionFiles = @(
            Get-ChildItem -LiteralPath $calibrationInbox -File -Filter "*.json" |
                Sort-Object -Property Name
        )
        if ($fusionFiles.Count -eq 0) {
            throw "Kalibrasyon bulunamadi. '$calibrationInbox' klasorune ZED360 Finish Calibration ile kaydedilen tam olarak bir fourkamera JSON dosyasi koyun. Alternatif: -Extrinsics ile dogrulanmis uygulama kalibrasyonu verin."
        }
        if ($fusionFiles.Count -gt 1) {
            $names = ($fusionFiles.Name -join ", ")
            throw "Birden fazla four kalibrasyonu bulundu ($names). Yanlis poz secilmemesi icin '$calibrationInbox' klasorunde yalnizca bir JSON birakin."
        }
        $FusionConfig = $fusionFiles[0].FullName
    }

    if (-not (Test-Path -LiteralPath $FusionConfig -PathType Leaf)) {
        throw "ZED360 four kamera JSON dosyasi bulunamadi: $FusionConfig"
    }
    $FusionConfig = (Resolve-Path -LiteralPath $FusionConfig).Path
    $calibrationOrigin = $FusionConfig
    Write-Host "Four ZED kalibrasyonu 'four json' akisindan secildi: $FusionConfig"
    try {
        $candidateDocument = Get-Content -LiteralPath $FusionConfig -Raw | ConvertFrom-Json
    }
    catch {
        throw "'four json' kalibrasyon JSON dosyasi okunamadi: $FusionConfig`n$($_.Exception.Message)"
    }
    if ([string]$candidateDocument.schema -eq "zed_body38_distributed_extrinsics/v1") {
        # The BODY_38 fallback calibrator already emits the runtime schema.
        # It still passes the same independent quality validator below.
        $Extrinsics = $FusionConfig
        Write-Host "Four JSON tipi: dogrudan BODY_38 uygulama extrinsic"
    }
    else {
        # The ZED SDK configuration reader is unreliable with non-ASCII
        # Windows paths. Preserve the source but convert an ASCII-only copy.
        $fusionCache = "C:\g1il\cache\zed_four"
        New-Item -ItemType Directory -Force -Path $fusionCache | Out-Null
        $sdkFusionConfig = Join-Path $fusionCache "four_camera_zed360.json"
        Copy-Item -LiteralPath $FusionConfig -Destination $sdkFusionConfig -Force

        $convertedExtrinsics = Join-Path $project "config\zed_four\active_zed360_body38_extrinsics.json"
        $worldPoses = Join-Path $project "config\zed_four\active_zed360_camera_world_poses.jsonl"
        $converter = Join-Path $project "zed_four_camera_test\convert_zed360_extrinsics.ps1"
        & $converter `
            -InputConfig $sdkFusionConfig `
            -OutputExtrinsics $convertedExtrinsics `
            -WorldPosesJsonl $worldPoses `
            -ReferenceSerial $ReferenceSerial
        if ($LASTEXITCODE -ne 0) {
            throw "ZED360 fourkamera JSON, BODY_38 ortak dunya extrinsic dosyasina donusturulemedi. Cikis kodu: $LASTEXITCODE"
        }
        $Extrinsics = $convertedExtrinsics
        Write-Host "Fourkamera ZED360 JSON -> BODY_38 extrinsic: $Extrinsics"
        Write-Host "Fourkamera dunya pozlari: $worldPoses"
    }
}
if (-not (Test-Path -LiteralPath $Extrinsics -PathType Leaf)) {
    throw "4-ZED extrinsic dosyasi bulunamadi: $Extrinsics"
}
$Extrinsics = (Resolve-Path -LiteralPath $Extrinsics).Path
$calibrationSource = $Extrinsics
if (-not $calibrationOrigin) {
    $calibrationOrigin = $calibrationSource
}
$fusionCache = "C:\g1il\cache\zed_four"
New-Item -ItemType Directory -Force -Path $fusionCache | Out-Null
$activeExtrinsics = Join-Path $fusionCache "active_body38_extrinsics.json"
if ([string]::Compare($Extrinsics, $activeExtrinsics, $true) -ne 0) {
    Copy-Item -LiteralPath $Extrinsics -Destination $activeExtrinsics -Force
}
$Extrinsics = $activeExtrinsics
$python = Join-Path $project ".venv-zed\Scripts\python.exe"
$validator = Join-Path $project "zed_four_camera_test\validate_distributed_extrinsics.py"
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    throw "ZED Python ortami bulunamadi: $python"
}
& $python $validator `
    --input $Extrinsics `
    --expected-serials ($expectedSerials -join ",") `
    --reference-serial $ReferenceSerial
if ($LASTEXITCODE -ne 0) {
    throw "4-ZED kalibrasyon kalite kapisini gecemedi. Yukaridaki HATA satirini duzeltin."
}
try {
    $calibrationBytes = [System.IO.File]::ReadAllBytes($Extrinsics)
    $calibrationText = [System.Text.Encoding]::UTF8.GetString($calibrationBytes)
    $calibration = $calibrationText | ConvertFrom-Json
}
catch {
    throw "Dogrulanan 4-ZED extrinsic JSON tekrar okunamadi: $Extrinsics`n$($_.Exception.Message)"
}
$calibrationItem = Get-Item -LiteralPath $calibrationOrigin
$sha256 = [System.Security.Cryptography.SHA256]::Create()
try {
    $calibrationHash = ([System.BitConverter]::ToString($sha256.ComputeHash($calibrationBytes))).Replace("-", "")
}
finally {
    $sha256.Dispose()
}
Write-Host "4-ZED kalibrasyon dogrulandi ve sabit runtime kopyasi olusturuldu: $Extrinsics"
Write-Host "Kalibrasyon kaynak dosyasi: $calibrationOrigin"
Write-Host "Referans kamera: $($calibration.reference_world_serial)"
Write-Host "Kaynak dosya zamani: $($calibrationItem.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss')) | runtime SHA256=$calibrationHash"
if ($ValidateCalibrationOnly) {
    exit 0
}

$wslAddressLine = wsl.exe -d Ubuntu-22.04 -- hostname -I
$wslAddress = ($wslAddressLine.Trim() -split "\s+")[0]
if (-not $wslAddress) {
    throw "WSL Ubuntu-22.04 IP adresi bulunamadi."
}
if (-not $SkipGmrCheck) {
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "ss -lun | grep -q ':15050'"
    if ($LASTEXITCODE -ne 0) {
        throw "WSL GMR UDP 15050 dinleyicisi yok. Once .\start_g1_isaaclab_live.ps1 baslatin."
    }
}

$source = @("39504762:16000", "31571870:16002", "33773329:16004", "34760587:16006")
$previewSource = @("39504762:16100", "31571870:16102", "33773329:16104", "34760587:16106")
$receiver = Join-Path $project "zed_four_camera_test\start_distributed_receiver.ps1"
$receiverArguments = @{
    Source = $source
    Extrinsics = $Extrinsics
    OutputHost = $wslAddress
    OutputPort = 15050
    Fps = $FusionHz
    MinimumSources = $MinimumSources
    MaxSyncMs = 80
    PreferredFullSetSpreadMs = $PreferredFullSetSpreadMs
    FullSetWaitMs = $FullSetWaitMs
    SourceTimeoutMs = 250
    MaxJointSpreadM = 0.18
    MaxTemporalPredictionMs = 70
    MaxAlignmentTranslationM = 0.25
    WorkspaceXMinM = $WorkspaceXMinM
    WorkspaceXMaxM = $WorkspaceXMaxM
    WorkspaceHysteresisM = $WorkspaceHysteresisM
    PreviewHz = $PreviewHz
    RecordStem = "four_body38_fusion"
    RecordDetail = "research"
}
if (-not $Headless) {
    $receiverArguments.PreviewSource = $previewSource
}
if ($HandTracking) {
    if (-not $Dex3OfficialRoot -and -not $Dex3OfficialPython) {
        $officialCandidate = "C:\g1il\repos\xr_teleoperate"
        $pythonCandidate = "C:\g1il\envs\dex3\Scripts\python.exe"
        $officialConfig = Join-Path $officialCandidate "assets\unitree_hand\unitree_dex3.yml"
        $officialReady = $false
        if ((Test-Path -LiteralPath $officialConfig -PathType Leaf) -and
            (Test-Path -LiteralPath $pythonCandidate -PathType Leaf)) {
            & $pythonCandidate -c "import dex_retargeting, pinocchio, yaml" 2>$null
            $officialReady = ($LASTEXITCODE -eq 0)
        }
        if ($officialReady) {
            $Dex3OfficialRoot = $officialCandidate
            $Dex3OfficialPython = $pythonCandidate
        }
        elseif (Test-Path -LiteralPath $officialConfig -PathType Leaf) {
            Write-Warning "Resmi DexPilot Windows bagimliliklari kullanilabilir degil; dusuk gecikmeli yerel 21-landmark retarget fallback secildi."
        }
    }
    $receiverArguments.HandTracking = $true
    $receiverArguments.HandMaxAgeMs = $HandMaxAgeMs
    $receiverArguments.HandMaxSpreadMs = $HandMaxSpreadMs
    if ($DisableDex3Retargeting) {
        $receiverArguments.DisableDex3Retargeting = $true
    }
    if ($DisableSingleViewHandDepth) {
        $receiverArguments.DisableSingleViewHandDepth = $true
    }
    if ($Dex3OfficialRoot) {
        $receiverArguments.Dex3OfficialRoot = $Dex3OfficialRoot
    }
    if ($Dex3OfficialPython) {
        $receiverArguments.Dex3OfficialPython = $Dex3OfficialPython
    }
}
if (-not $NoAnalysisStream) {
    $receiverArguments.MonitorHost = $AnalysisHost
    $receiverArguments.MonitorPort = 15052
    $receiverArguments.MonitorHz = $FusionHz
}
if (-not $NoRosStream) {
    $receiverArguments.RosHost = $wslAddress
    $receiverArguments.RosPort = 15054
    $receiverArguments.RosHz = $FusionHz
}
if ($Record) { $receiverArguments.Record = $true }
if ($Headless) { $receiverArguments.Headless = $true }

Write-Host "GMR/Isaac: ${wslAddress}:15050 | Rerun: ${AnalysisHost}:15052 | ROS: ${wslAddress}:15054"
Write-Host "Fusion: en az $MinimumSources/4 taze kamera, azami ${FusionHz}Hz | arayuz=${PreviewHz}Hz | dortlu tercih <=${PreferredFullSetSpreadMs}ms, bekleme <=${FullSetWaitMs}ms"
Write-Host "Operator ortak-dunya kapisi: referans kamera X=${WorkspaceXMinM}-${WorkspaceXMaxM} m | cikis toleransi=${WorkspaceHysteresisM}m"
if ($HandTracking) {
    $retargetLabel = if ($Dex3OfficialRoot) { "resmi Unitree xr_teleoperate" } else { "yerel fallback" }
    Write-Host "El katmani: ACIK | retarget=$retargetLabel | Isaac kompakt Dex3 + Rerun 21-landmark | fiziksel robot el cikisi KAPALI"
}
Write-Host "JSONL klasoru: $(Join-Path $project 'recordings')"
Set-Location -LiteralPath $project
& $receiver @receiverArguments
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "4-ZED fusion alicisi hata ile kapandi: $exitCode"
}
