param(
    [Parameter(Mandatory = $true)]
    [Alias("Input")]
    [string]$CapturePath,
    [Parameter(Mandatory = $true)]
    [Alias("Output")]
    [string]$OutputPath,
    [Parameter(Mandatory = $true)]
    [long]$ReferenceSerial,
    [ValidateRange(60, 100000)]
    [int]$MaxSamples = 5000,
    [string]$WorldPosesJsonl = "",
    [switch]$Activate
)

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$calibrator = Join-Path $PSScriptRoot "calibrate_distributed_body38.py"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}
$arguments = @(
    $calibrator,
    "--input", $CapturePath,
    "--output", $OutputPath,
    "--reference-serial", "$ReferenceSerial",
    "--max-samples", "$MaxSamples"
)
if ($WorldPosesJsonl) {
    $arguments += "--world-poses-jsonl"
    $arguments += $WorldPosesJsonl
}
& $python @arguments
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    exit $exitCode
}

if ($Activate) {
    $resolvedOutput = (Resolve-Path -LiteralPath $OutputPath).Path
    if ($WorldPosesJsonl) {
        $resolvedWorldPoses = (Resolve-Path -LiteralPath $WorldPosesJsonl).Path
    }
    else {
        $outputItem = Get-Item -LiteralPath $resolvedOutput
        $generatedWorldPoses = Join-Path $outputItem.DirectoryName ($outputItem.BaseName + "_world_poses.jsonl")
        if (-not (Test-Path -LiteralPath $generatedWorldPoses)) {
            throw "Kalibrator world-pose JSONL uretmedi: $generatedWorldPoses"
        }
        $resolvedWorldPoses = (Resolve-Path -LiteralPath $generatedWorldPoses).Path
    }
    $activeDir = Join-Path $root "config\zed_four"
    New-Item -ItemType Directory -Path $activeDir -Force | Out-Null
    $activeExtrinsics = Join-Path $activeDir "active_distributed_body38_extrinsics.json"
    $activeWorldPoses = Join-Path $activeDir "active_four_camera_world_poses.jsonl"
    $fourJsonDir = Join-Path $root "four json"
    $fourJsonExtrinsics = Join-Path $fourJsonDir "fourkamera.json"
    New-Item -ItemType Directory -Path $fourJsonDir -Force | Out-Null
    # Keep the same single-inbox contract as the dual-camera launcher.  An
    # explicit -Activate is the user's request to replace the runtime
    # calibration; the four-camera launcher will pick this file automatically.
    Get-ChildItem -LiteralPath $fourJsonDir -File -Filter "*.json" |
        Where-Object { $_.FullName -ne $fourJsonExtrinsics } |
        ForEach-Object {
            throw "'four json' klasorunde baska JSON var: $($_.FullName). Yanlis dosyayi silmeden/arsivlemeden etkinlestirme yapilmadi."
        }
    Copy-Item -LiteralPath $resolvedOutput -Destination $activeExtrinsics -Force
    Copy-Item -LiteralPath $resolvedWorldPoses -Destination $activeWorldPoses -Force
    if ([string]::Compare($resolvedOutput, $fourJsonExtrinsics, $true) -ne 0) {
        Copy-Item -LiteralPath $resolvedOutput -Destination $fourJsonExtrinsics -Force
    }
    Write-Host "AKTIF 4-ZED EXTRINSIC: $activeExtrinsics"
    Write-Host "AKTIF 4-ZED WORLD POSES: $activeWorldPoses"
    Write-Host "FOUR JSON RUNTIME KALIBRASYONU: $fourJsonExtrinsics"
}
exit 0
