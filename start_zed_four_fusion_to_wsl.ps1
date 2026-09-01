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
    [string]$Extrinsics = "",
    [string]$AnalysisHost = "127.0.0.1",
    [switch]$ValidateCalibrationOnly
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$expectedSerials = @("31571870", "33773329", "34760587", "39504762")
$activeExtrinsics = Join-Path $project "config\zed_four\active_distributed_body38_extrinsics.json"
if (-not $Extrinsics) {
    if (Test-Path -LiteralPath $activeExtrinsics) {
        $Extrinsics = $activeExtrinsics
    }
    else {
        $candidates = @(Get-ChildItem -LiteralPath (Join-Path $project "config\zed_four") -File -Filter "distributed_body38_extrinsics*.json" -ErrorAction SilentlyContinue)
        if ($candidates.Count -ne 1) {
            throw "Aktif kalibrasyon yok. -Extrinsics ile tek dosya verin veya start_distributed_calibration.ps1 ... -Activate calistirin. Aday sayisi: $($candidates.Count)"
        }
        $Extrinsics = $candidates[0].FullName
        Write-Warning "Aktif dosya bulunmadigi icin tek kalibrasyon adayi kullaniliyor: $Extrinsics"
    }
}
if (-not (Test-Path -LiteralPath $Extrinsics -PathType Leaf)) {
    throw "4-ZED extrinsic dosyasi bulunamadi: $Extrinsics"
}
$Extrinsics = (Resolve-Path -LiteralPath $Extrinsics).Path
try {
    $calibration = Get-Content -LiteralPath $Extrinsics -Raw | ConvertFrom-Json
}
catch {
    throw "4-ZED extrinsic JSON okunamadi: $Extrinsics`n$($_.Exception.Message)"
}
if ([string]$calibration.schema -ne "zed_body38_distributed_extrinsics/v1") {
    throw "Yanlis kalibrasyon semasi: $($calibration.schema)"
}
if ([string]$calibration.coordinate_system -ne "RIGHT_HANDED_Z_UP_X_FWD") {
    throw "Kalibrasyon koordinat sistemi RIGHT_HANDED_Z_UP_X_FWD olmali."
}
$actualSerials = @($calibration.cameras.PSObject.Properties.Name | Sort-Object)
if (($actualSerials -join ',') -ne (($expectedSerials | Sort-Object) -join ',')) {
    throw "Kalibrasyon seri seti yanlis. Beklenen=$($expectedSerials -join ',') Gelen=$($actualSerials -join ',')"
}
foreach ($serial in $expectedSerials) {
    $camera = $calibration.cameras.$serial
    if (@($camera.rotation_camera_to_world).Count -ne 3 -or @($camera.translation_camera_to_world_m).Count -ne 3) {
        throw "ZED $serial icin rotation/translation eksik."
    }
}
Write-Host "4-ZED kalibrasyon dogrulandi: $Extrinsics"
Write-Host "Referans kamera: $($calibration.reference_world_serial)"
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
$arguments = @(
    "-Source", $source,
    "-Extrinsics", $Extrinsics,
    "-OutputHost", $wslAddress,
    "-OutputPort", 15050,
    "-Fps", $FusionHz,
    "-MinimumSources", $MinimumSources,
    "-MaxSyncMs", 110,
    "-SourceTimeoutMs", 750,
    "-PreviewHz", $PreviewHz,
    "-RecordStem", "four_body38_fusion"
)
if (-not $Headless) {
    $arguments += @("-PreviewSource", $previewSource)
}
if (-not $NoAnalysisStream) {
    $arguments += @("-MonitorHost", $AnalysisHost, "-MonitorPort", 15052, "-MonitorHz", $FusionHz)
}
if (-not $NoRosStream) {
    $arguments += @("-RosHost", $wslAddress, "-RosPort", 15054, "-RosHz", $FusionHz)
}
if ($Record) { $arguments += "-Record" }
if ($Headless) { $arguments += "-Headless" }

Write-Host "GMR/Isaac: ${wslAddress}:15050 | Rerun: ${AnalysisHost}:15052 | ROS: ${wslAddress}:15054"
Write-Host "Fusion: en az $MinimumSources/4 taze kamera, azami ${FusionHz}Hz | arayuz=${PreviewHz}Hz"
Write-Host "JSONL klasoru: $(Join-Path $project 'recordings')"
Set-Location -LiteralPath $project
& $receiver @arguments
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "4-ZED fusion alicisi hata ile kapandi: $exitCode"
}
