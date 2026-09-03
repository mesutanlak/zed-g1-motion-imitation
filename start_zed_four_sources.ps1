param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Laptop", "MainPc")]
    [string]$Role,
    [string]$MainPcHost = "192.168.50.10",
    [ValidateSet(15, 30)]
    [int]$Fps = 15,
    [ValidateSet("fast", "medium", "accurate")]
    [string]$Model = "medium",
    [ValidateSet("neural-light", "neural", "performance")]
    [string]$DepthMode = "neural-light",
    [ValidateSet("strict", "monitor", "off")]
    [string]$FrameIntegrityMode = "off",
    [ValidateRange(0.3, 20.0)]
    [double]$DistanceMin = 1.0,
    [ValidateRange(0.5, 30.0)]
    [double]$DistanceMax = 5.25,
    [switch]$CalibrationMode,
    [switch]$DisableCalibrationDistanceGate,
    [ValidateRange(1, 10)]
    [double]$PreviewHz = 8,
    [switch]$RecordLocal,
    [switch]$RecordSvo2,
    [string]$SvoRecordDir = ""
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $project ".venv-zed\Scripts\python.exe"
$deviceProbe = Join-Path $project "zed_g1_skeleton.py"
$sourceLauncher = Join-Path $project "zed_four_camera_test\start_distributed_source.ps1"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi: $python"
}
if ($DistanceMax -le $DistanceMin) {
    throw "DistanceMax, DistanceMin degerinden buyuk olmali."
}

if ($Role -eq "Laptop") {
    $definitions = @(
        @{ Serial = 39504762; BodyPort = 16000; PreviewPort = 16100 },
        @{ Serial = 34760587; BodyPort = 16006; PreviewPort = 16106 }
    )
    $bodyTarget = $MainPcHost
    $previewTarget = $MainPcHost
}
else {
    $definitions = @(
        @{ Serial = 31571870; BodyPort = 16002; PreviewPort = 16102 },
        @{ Serial = 33773329; BodyPort = 16004; PreviewPort = 16104 }
    )
    $bodyTarget = "127.0.0.1"
    $previewTarget = "127.0.0.1"
}

# ZED's native recorder can fail on Windows paths containing non-ASCII
# characters. Keep SVO2 output in a short ASCII-only local path by default.
# A serial-numbered stem also prevents the two cameras started in the same
# second from trying to open the same JSONL/SVO2 pair.
$recordSession = Get-Date -Format "yyyyMMdd_HHmmss"
if ($RecordSvo2 -and -not $SvoRecordDir) {
    $SvoRecordDir = Join-Path $env:LOCALAPPDATA "ZED_G1\recordings"
}

$probeOutput = @(& $python $deviceProbe --list-devices 2>&1)
if ($LASTEXITCODE -ne 0) {
    throw "ZED aygit listesi alinamadi:`n$($probeOutput -join [Environment]::NewLine)"
}
$available = @(
    $probeOutput | ForEach-Object {
        if ([string]$_ -match 'serial=(\d+)') { [long]$Matches[1] }
    }
)
$missing = @($definitions | Where-Object { $available -notcontains [long]$_.Serial })
if ($missing.Count -gt 0) {
    throw "Bu bilgisayarda beklenen ZED bulunamadi: $((@($missing | ForEach-Object { $_.Serial })) -join ', '). Bulunan: $($available -join ', ')"
}

foreach ($definition in $definitions) {
    $arguments = @(
        "-NoExit", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", ('"{0}"' -f $sourceLauncher),
        "-Serial", "$($definition.Serial)",
        "-TargetHost", $bodyTarget,
        "-TargetPort", "$($definition.BodyPort)",
        "-SourceHostId", $Role,
        "-Fps", "$Fps",
        "-Model", $Model,
        "-DepthMode", $DepthMode,
        "-FrameIntegrityMode", $FrameIntegrityMode,
        "-DistanceMin", "$DistanceMin",
        "-DistanceMax", "$DistanceMax",
        "-PreviewHost", $previewTarget,
        "-PreviewPort", "$($definition.PreviewPort)",
        "-PreviewHz", "$PreviewHz"
    )
    if ($RecordSvo2) {
        $recordStem = "zed_body38_$($definition.Serial)_$recordSession"
        $arguments += @(
            "-RecordSvo2",
            "-OutputDir", $SvoRecordDir,
            "-RecordStem", $recordStem
        )
    }
    elseif ($RecordLocal) {
        $recordStem = "zed_body38_$($definition.Serial)_$recordSession"
        $arguments += @("-RecordLocal", "-RecordStem", $recordStem)
    }
    if ($CalibrationMode -and $DisableCalibrationDistanceGate) {
        $arguments += "-DisableDistanceGate"
    }
    Start-Process -FilePath "powershell.exe" -ArgumentList $arguments
    Start-Sleep -Milliseconds 800
}

Write-Host "4-ZED kaynaklari baslatildi | rol=$Role | BODY hedefi=$bodyTarget | JPEG hedefi=$previewTarget"
if ($CalibrationMode) {
    if ($DisableCalibrationDistanceGate) {
        Write-Warning "KALIBRASYON MODU: kamera-yerel mesafe kapisi elle kapatildi; ortamda yalniz tek kisi olsun."
    }
    else {
        Write-Host "KALIBRASYON MODU: kamera-yerel operator kapisi ${DistanceMin}-${DistanceMax} m ACIK."
        Write-Warning "Dort kameranin da ayni hareketli operatoru kilitledigini onizlemelerden dogrulayin; ortamda ikinci kisi bulunmasin."
    }
}
else {
    Write-Host "Kamera-yerel kaba operator kapisi: ${DistanceMin}-${DistanceMax} m (kilitli kiside +/-0.20 m histerezis)"
    Write-Host "Kesin 2-4 m operator hacmi ana PC'de, kalibre edilmis ortak dunya koordinatinda uygulanir."
}
Write-Host "Seriler: $((@($definitions | ForEach-Object { $_.Serial })) -join ', ')"
if ($RecordSvo2) {
    Write-Host "SVO2/JSONL yerel kayit klasoru: $SvoRecordDir"
}
Write-Host "Her kaynak penceresini acik birakin. Operator kilidi gerekirse yalniz ilgili kaynak penceresinde R tusuna basin."
