param(
    [Parameter(Mandatory = $true)]
    [string[]]$Source,
    [string[]]$PreviewSource = @(),
    [string]$Extrinsics = "",
    [string]$OutputHost = "",
    [ValidateRange(1024, 65535)]
    [int]$OutputPort = 15050,
    [string]$MonitorHost = "",
    [ValidateRange(1024, 65535)]
    [int]$MonitorPort = 15052,
    [ValidateRange(1, 60)]
    [int]$MonitorHz = 15,
    [string]$RosHost = "",
    [ValidateRange(1024, 65535)]
    [int]$RosPort = 15054,
    [ValidateRange(1, 60)]
    [int]$RosHz = 15,
    [string]$CalibrationRecord = "",
    [ValidateRange(1, 60)]
    [int]$Fps = 15,
    [ValidateSet(2, 3, 4)]
    [int]$MinimumSources = 2,
    [ValidateRange(20, 1000)]
    [double]$MaxSyncMs = 80,
    [ValidateRange(0, 80)]
    [double]$PreferredFullSetSpreadMs = 40,
    [ValidateRange(0, 50)]
    [double]$FullSetWaitMs = 20,
    [ValidateRange(100, 5000)]
    [double]$SourceTimeoutMs = 250,
    [ValidateRange(0.05, 1.0)]
    [double]$MaxJointSpreadM = 0.18,
    [ValidateRange(0.05, 1.0)]
    [double]$MaxPoseDisagreementM = 0.22,
    [ValidateRange(0.25, 3.0)]
    [double]$MaxAlignmentTranslationM = 0.25,
    [ValidateRange(0, 100)]
    [double]$MaxTemporalPredictionMs = 70,
    [ValidateRange(0.0, 20.0)]
    [double]$WorkspaceXMinM = 2.0,
    [ValidateRange(0.1, 30.0)]
    [double]$WorkspaceXMaxM = 4.0,
    [ValidateRange(0.0, 1.0)]
    [double]$WorkspaceHysteresisM = 0.15,
    [ValidateRange(1, 30)]
    [double]$PreviewHz = 10,
    [switch]$HandTracking,
    [switch]$DisableDex3Retargeting,
    [ValidateRange(10, 150)]
    [double]$HandMaxAgeMs = 120,
    [ValidateRange(5, 80)]
    [double]$HandMaxSpreadMs = 70,
    [switch]$DisableSingleViewHandDepth,
    [string]$Dex3OfficialRoot = "",
    [string]$Dex3OfficialPython = "",
    [string]$OutputDir = "",
    [string]$RecordStem = "four_body38_fusion",
    [switch]$Record,
    [ValidateSet("minimal", "research", "full")]
    [string]$RecordDetail = "research",
    [switch]$Headless,
    [ValidateRange(0, 86400)]
    [double]$Duration = 0
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$receiver = Join-Path $PSScriptRoot "distributed_body38_fusion.py"
if ($WorkspaceXMaxM -le $WorkspaceXMinM) {
    throw "WorkspaceXMaxM, WorkspaceXMinM degerinden buyuk olmali."
}
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
if ($Source.Count -ne 4) {
    throw "Dort -Source 'SERIAL:PORT' degeri gerekli."
}

# Calibration must be observable.  The standard four-camera launcher sends a
# presentation-only JPEG stream on BODY_PORT + 100.  Derive those endpoints
# automatically so a calibration capture cannot accidentally run blind merely
# because -PreviewSource was omitted from a long PowerShell command.
if ($CalibrationRecord -and $PreviewSource.Count -eq 0 -and -not $Headless) {
    $derivedPreviewSources = @()
    foreach ($item in $Source) {
        if ($item -notmatch '^(\d+):(\d+)$') {
            throw "Preview otomatik kesfi icin Source 'SERIAL:PORT' biciminde olmali: $item"
        }
        $serial = $Matches[1]
        $bodyPort = [int]$Matches[2]
        $previewPort = $bodyPort + 100
        if ($previewPort -gt 65535) {
            throw "ZED $serial icin otomatik preview portu gecersiz: $previewPort"
        }
        $derivedPreviewSources += "${serial}:${previewPort}"
    }
    $PreviewSource = $derivedPreviewSources
    Write-Host "KALIBRASYON ONIZLEMESI OTOMATIK: $($PreviewSource -join ', ')"
    Write-Host "2x2 pencerede yesil iskelet secilen operatoru; secim=LOCKED aktif kilidi gosterir."
}

$arguments = @($receiver)
foreach ($item in $Source) {
    $arguments += "--source"
    $arguments += $item
}
foreach ($item in $PreviewSource) {
    $arguments += "--preview-source"
    $arguments += $item
}
if ($HandTracking) {
    if (-not $Dex3OfficialRoot -and -not $Dex3OfficialPython) {
        $officialCandidate = "C:\g1il\repos\xr_teleoperate"
        $pythonCandidate = "C:\g1il\envs\dex3\Scripts\python.exe"
        $officialConfig = Join-Path $officialCandidate "assets\unitree_hand\unitree_dex3.yml"
        $officialReady = $false
        if ((Test-Path -LiteralPath $officialConfig -PathType Leaf) -and
            (Test-Path -LiteralPath $pythonCandidate -PathType Leaf)) {
            $probe = Start-Process -FilePath $pythonCandidate `
                -ArgumentList @("-c", "import dex_retargeting, pinocchio, yaml") `
                -WindowStyle Hidden -Wait -PassThru
            $officialReady = ($probe.ExitCode -eq 0)
        }
        if ($officialReady) {
            $Dex3OfficialRoot = $officialCandidate
            $Dex3OfficialPython = $pythonCandidate
        }
        elseif (Test-Path -LiteralPath $officialConfig -PathType Leaf) {
            Write-Warning "Resmi DexPilot Windows bagimliliklari kullanilabilir degil; dusuk gecikmeli yerel 21-landmark retarget fallback secildi."
        }
    }
    foreach ($item in $Source) {
        if ($item -notmatch '^(\d+):(\d+)$') {
            throw "Hand port turetmek icin Source SERIAL:PORT biciminde olmali: $item"
        }
        $handPort = [int]$Matches[2] + 200
        $arguments += @("--hand-source", "$($Matches[1]):$handPort")
    }
    $arguments += @("--hand-tracking", "--hand-max-age-ms", "$HandMaxAgeMs", "--hand-max-spread-ms", "$HandMaxSpreadMs")
    if ($DisableDex3Retargeting) { $arguments += "--no-dex3-retargeting" }
    if ($DisableSingleViewHandDepth) { $arguments += "--no-hand-single-view-depth" }
    if ($Dex3OfficialRoot) { $arguments += @("--dex3-official-root", $Dex3OfficialRoot) }
    if ($Dex3OfficialPython) { $arguments += @("--dex3-official-python", $Dex3OfficialPython) }
}
$arguments += "--minimum-sources"
$arguments += "$MinimumSources"
$arguments += "--max-sync-ms"
$arguments += "$MaxSyncMs"
$arguments += "--preferred-full-set-spread-ms"
$arguments += "$PreferredFullSetSpreadMs"
$arguments += "--full-set-wait-ms"
$arguments += "$FullSetWaitMs"
$arguments += "--source-timeout-ms"
$arguments += "$SourceTimeoutMs"
$arguments += "--max-joint-spread-m"
$arguments += "$MaxJointSpreadM"
$arguments += "--max-pose-disagreement-m"
$arguments += "$MaxPoseDisagreementM"
$arguments += "--max-alignment-translation-m"
$arguments += "$MaxAlignmentTranslationM"
$arguments += "--max-temporal-prediction-ms"
$arguments += "$MaxTemporalPredictionMs"
$arguments += "--workspace-x-min-m"
$arguments += "$WorkspaceXMinM"
$arguments += "--workspace-x-max-m"
$arguments += "$WorkspaceXMaxM"
$arguments += "--workspace-hysteresis-m"
$arguments += "$WorkspaceHysteresisM"
$arguments += "--duration"
$arguments += "$Duration"
if ($Extrinsics) {
    $arguments += "--extrinsics"
    $arguments += $Extrinsics
}
if ($OutputHost) {
    $arguments += "--output-host"
    $arguments += $OutputHost
    $arguments += "--output-port"
    $arguments += "$OutputPort"
    $arguments += "--output-max-hz"
    $arguments += "$Fps"
}
if ($MonitorHost) {
    $arguments += @("--monitor-host", $MonitorHost, "--monitor-port", "$MonitorPort", "--monitor-max-hz", "$MonitorHz")
}
if ($RosHost) {
    $arguments += @("--ros-host", $RosHost, "--ros-port", "$RosPort", "--ros-max-hz", "$RosHz")
}
if ($CalibrationRecord) {
    $arguments += "--calibration-record"
    $arguments += $CalibrationRecord
}
$arguments += @("--preview-hz", "$PreviewHz", "--record-stem", $RecordStem)
$arguments += @("--record-detail", $RecordDetail)
if ($OutputDir) {
    $arguments += @("--output-dir", $OutputDir)
}
if ($Record) {
    $arguments += "--record"
}
if ($Headless) {
    $arguments += "--headless"
}
& $python @arguments
exit $LASTEXITCODE
