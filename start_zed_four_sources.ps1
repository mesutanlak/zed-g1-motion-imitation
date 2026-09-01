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
    [string]$FrameIntegrityMode = "monitor",
    [ValidateRange(1, 10)]
    [double]$PreviewHz = 10,
    [switch]$RecordLocal,
    [switch]$RecordSvo2
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $project ".venv-zed\Scripts\python.exe"
$deviceProbe = Join-Path $project "zed_g1_skeleton.py"
$sourceLauncher = Join-Path $project "zed_four_camera_test\start_distributed_source.ps1"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi: $python"
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
        "-Fps", "$Fps",
        "-Model", $Model,
        "-DepthMode", $DepthMode,
        "-FrameIntegrityMode", $FrameIntegrityMode,
        "-PreviewHost", $previewTarget,
        "-PreviewPort", "$($definition.PreviewPort)",
        "-PreviewHz", "$PreviewHz"
    )
    if ($RecordSvo2) {
        $arguments += "-RecordSvo2"
    }
    elseif ($RecordLocal) {
        $arguments += "-RecordLocal"
    }
    Start-Process -FilePath "powershell.exe" -ArgumentList $arguments
    Start-Sleep -Milliseconds 800
}

Write-Host "4-ZED kaynaklari baslatildi | rol=$Role | BODY hedefi=$bodyTarget | JPEG hedefi=$previewTarget"
Write-Host "Seriler: $((@($definitions | ForEach-Object { $_.Serial })) -join ', ')"
Write-Host "Her kaynak penceresini acik birakin. Operator kilidi gerekirse yalniz ilgili kaynak penceresinde R tusuna basin."
