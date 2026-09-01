param(
    [Parameter(Mandatory = $true)]
    [Alias("Input")]
    [string]$CapturePath,
    [Parameter(Mandatory = $true)]
    [Alias("Output")]
    [string]$OutputPath,
    [Parameter(Mandatory = $true)]
    [long]$ReferenceSerial,
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
    "--reference-serial", "$ReferenceSerial"
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
    Copy-Item -LiteralPath $resolvedOutput -Destination $activeExtrinsics -Force
    Copy-Item -LiteralPath $resolvedWorldPoses -Destination $activeWorldPoses -Force
    Write-Host "AKTIF 4-ZED EXTRINSIC: $activeExtrinsics"
    Write-Host "AKTIF 4-ZED WORLD POSES: $activeWorldPoses"
}
exit 0
