param(
    [Parameter(Mandatory = $true)]
    [string]$InputPath,
    [string]$OutputDir = ".\datasets\reference_motion",
    [ValidateSet("upper_body", "whole_body")]
    [string]$Mode = "upper_body",
    [ValidateSet("position", "direction", "direct", "none")]
    [string]$ArmIK = "position",
    [ValidateRange(0.0, 1.0)]
    [double]$MinimumConfidence = 0.60,
    [ValidateRange(0.5, 30.0)]
    [double]$MinimumSeconds = 2.0,
    [string]$InstallRoot = "C:\g1il",
    [switch]$AcceptNvidiaEula
)

$ErrorActionPreference = "Stop"

function Convert-ToWslPath([string]$NativePath) {
    $resolved = [System.IO.Path]::GetFullPath($NativePath)
    if ($resolved -notmatch "^[A-Za-z]:\\") {
        throw "WSL yoluna cevrilemeyen Windows yolu: $resolved"
    }
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $rest = $resolved.Substring(2).Replace("\", "/")
    return "/mnt/$drive$rest"
}

function Copy-WithRetry(
    [string]$Source,
    [string]$Destination,
    [int]$Attempts = 12
) {
    $lastError = $null
    for ($attempt = 1; $attempt -le $Attempts; $attempt++) {
        try {
            Copy-Item -LiteralPath $Source -Destination $Destination -Force
            return
        }
        catch {
            $lastError = $_
            if ($attempt -lt $Attempts) {
                Start-Sleep -Milliseconds (150 * $attempt)
            }
        }
    }
    throw "Dosya OneDrive kilidinden sonra da kopyalanamadi: $Source`n$lastError"
}

$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$input = (Resolve-Path -LiteralPath $InputPath).Path
$output = [System.IO.Path]::GetFullPath($OutputDir)
New-Item -ItemType Directory -Force -Path $output | Out-Null
$windowsPython = Join-Path $InstallRoot "env\Scripts\python.exe"
$zedPython = Join-Path $project ".venv-zed\Scripts\python.exe"
$robotXml = Join-Path $InstallRoot "repos\unitree_ros\robots\g1_description\g1_23dof.xml"
foreach ($required in @($windowsPython, $robotXml)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Required file not found: $required"
    }
}

$extension = [System.IO.Path]::GetExtension($input).ToLowerInvariant()
if ($extension -eq ".svo2") {
    if (-not (Test-Path -LiteralPath $zedPython)) {
        throw "ZED Python environment not found: $zedPython"
    }
    $stem = [System.IO.Path]::GetFileNameWithoutExtension($input) + "_offline"
    & $zedPython (Join-Path $project "zed_g1_skeleton.py") `
        --svo-input $input --record --headless --record-stem $stem `
        --output-dir $output --model medium --depth-mode neural --fps 30
    if ($LASTEXITCODE -ne 0) { throw "SVO2 -> BODY_38 JSONL conversion failed." }
    $jsonl = Join-Path $output "$stem.jsonl"
} elseif ($extension -eq ".jsonl") {
    $jsonl = $input
} else {
    throw "InputPath must be a .svo2 or BODY_38 .jsonl file."
}
if (-not (Test-Path -LiteralPath $jsonl)) {
    throw "BODY_38 JSONL was not produced: $jsonl"
}

$wslHome = (& wsl.exe -d Ubuntu-22.04 -- bash -lc 'printf %s "$HOME"').Trim()
if (-not $wslHome.StartsWith("/")) { throw "WSL home directory not found." }
$gmrPython = "$wslHome/g1_isaaclab_project/envs/gmr_zed/bin/python"
$gmrRoot = "$wslHome/g1_isaaclab_project/repos/GMR"
$wslProject = Convert-ToWslPath $project
$wslJsonl = Convert-ToWslPath $jsonl
$wslRobotXml = Convert-ToWslPath $robotXml
$gmrNpz = Join-Path $output (([System.IO.Path]::GetFileNameWithoutExtension($jsonl)) + "_gmr.npz")
$wslGmrNpz = Convert-ToWslPath $gmrNpz

& wsl.exe -d Ubuntu-22.04 -- $gmrPython `
    "$wslProject/isaaclab_bridge/gmr_retarget_jsonl.py" $wslJsonl `
    --output $wslGmrNpz --gmr-root $gmrRoot --mode $Mode --arm-ik $ArmIK
if ($LASTEXITCODE -ne 0) { throw "BODY_38 -> GMR 23-DOF conversion failed." }

$clipsDir = Join-Path $output "clips"
& $windowsPython (Join-Path $project "tools\prepare_reference_motion_clips.py") `
    $gmrNpz $clipsDir --source-jsonl $jsonl --output-fps 50 `
    --minimum-confidence $MinimumConfidence --minimum-seconds $MinimumSeconds `
    --robot-xml $robotXml
if ($LASTEXITCODE -ne 0) { throw "Clean reference clip preparation failed." }

$manifestPath = Join-Path $clipsDir "clips_manifest.json"
$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
$cacheRoot = Join-Path $InstallRoot "cache\reference_dataset"
New-Item -ItemType Directory -Force -Path $cacheRoot | Out-Null
$conversionCache = Join-Path $cacheRoot ([guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Force -Path $conversionCache | Out-Null
try {
    foreach ($clip in $manifest.clips) {
        $npz = [string]$clip.reference_npz
        $cacheStem = [string]$clip.id
        $cacheCsv = Join-Path $conversionCache "$cacheStem.csv"
        $cacheMetadata = Join-Path $conversionCache "$cacheStem.metadata.npz"
        $cacheNpz = Join-Path $conversionCache "$cacheStem.npz"

        # OneDrive may briefly expose a generated CSV as a reparse point and
        # reject WSL opens with EACCES. Hydrate/copy it on the Windows side,
        # then let MuJoCo read and write only in the ASCII local Isaac cache.
        Copy-WithRetry ([string]$clip.csv) $cacheCsv
        Copy-WithRetry ([string]$clip.metadata_npz) $cacheMetadata
        $wslCsv = Convert-ToWslPath $cacheCsv
        $wslMetadata = Convert-ToWslPath $cacheMetadata
        $wslNpz = Convert-ToWslPath $cacheNpz
        & wsl.exe -d Ubuntu-22.04 -- $gmrPython `
            "$wslProject/reference_policy/mujoco_csv_to_npz_23dof.py" `
            --input-file $wslCsv --metadata-file $wslMetadata --output-file $wslNpz `
            --robot-xml $wslRobotXml --fps 50
        if ($LASTEXITCODE -ne 0) { throw "MuJoCo FK conversion failed for $($clip.csv)" }
        & $windowsPython (Join-Path $project "tools\validate_reference_motion_npz.py") `
            $cacheNpz --robot-xml $robotXml
        if ($LASTEXITCODE -ne 0) { throw "Reference validation failed for $npz" }
        Copy-WithRetry $cacheNpz $npz
        if (-not (Test-Path -LiteralPath $npz -PathType Leaf)) {
            throw "Validated reference clip could not be published: $npz"
        }
    }
}
finally {
    $resolvedCache = [System.IO.Path]::GetFullPath($conversionCache)
    $resolvedCacheRoot = [System.IO.Path]::GetFullPath($cacheRoot).TrimEnd('\') + '\'
    if ($resolvedCache.StartsWith($resolvedCacheRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        Remove-Item -LiteralPath $resolvedCache -Recurse -Force -ErrorAction SilentlyContinue
    }
}

# Rebuild the persistent catalog from every complete session.  This is done
# only after all NPZ clips were validated/published, so an interrupted prepare
# can never poison the training manifest.
$datasetRoot = Join-Path $project "datasets\reference_motion"
$catalogPath = Join-Path $datasetRoot "multi_session_manifest.json"
& $windowsPython (Join-Path $project "tools\update_reference_motion_catalog.py") `
    --dataset-root $datasetRoot --output $catalogPath
if ($LASTEXITCODE -ne 0) { throw "Multi-session policy catalog update failed." }
$catalog = Get-Content -LiteralPath $catalogPath -Raw -Encoding UTF8 | ConvertFrom-Json

Write-Host "Reference dataset ready: $manifestPath" -ForegroundColor Green
Write-Host "TRAIN ($($manifest.splits.train.Count) clips):" -ForegroundColor Cyan
foreach ($clipId in $manifest.splits.train) { Write-Host "  $clipId.npz" }
Write-Host "VALIDATION (held out):" -ForegroundColor Yellow
foreach ($clipId in $manifest.splits.validation) { Write-Host "  $clipId.npz" }
Write-Host "All clips stay independent; every Isaac environment samples a train clip." -ForegroundColor Cyan
Write-Host (
    "MULTI-SESSION: sessions=$($catalog.summary.session_count) " +
    "train=$($catalog.summary.train_clip_count) validation=$($catalog.summary.validation_clip_count)"
) -ForegroundColor Magenta
Write-Host "Catalog: $catalogPath" -ForegroundColor Magenta
Write-Host "Training command:" -ForegroundColor Green
Write-Host "  powershell -NoProfile -ExecutionPolicy Bypass -File .\start_g1_reference_policy_training.ps1 -ClipsManifest `"$catalogPath`" -NumEnvs 512 -MaxIterations 2000 -AcceptNvidiaEula"
