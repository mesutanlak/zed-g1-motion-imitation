param(
    [string]$Recording = "",
    [ValidateRange(1.0, 60.0)]
    [double]$TargetFps = 30.0,
    [switch]$SkipGmrReplay
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = (Get-Command python -ErrorAction Stop).Source
Set-Location $project

function Convert-ToWslPath([string]$WindowsPath) {
    $full = [System.IO.Path]::GetFullPath($WindowsPath).Replace('\', '/')
    if ($full -notmatch '^([A-Za-z]):/(.*)$') {
        throw "Cannot convert path to WSL: $WindowsPath"
    }
    return "/mnt/$($Matches[1].ToLower())/$($Matches[2])"
}

Write-Host "[1/4] ZED/BODY_38 schema self-test"
& $python ".\zed_g1_skeleton.py" --self-test
if ($LASTEXITCODE -ne 0) { throw "ZED self-test failed." }

Write-Host "[2/4] Deterministic unit/regression tests"
$testRunner = @'
import runpy
for path in (r"tests/test_motion_pipeline.py", r"tests/test_capture_benchmark.py"):
    module = runpy.run_path(path)
    for name, function in sorted(module.items()):
        if name.startswith("test_") and callable(function):
            function()
            print(f"PASS {name}")
'@
$testRunner | & $python -
if ($LASTEXITCODE -ne 0) { throw "Regression tests failed." }

Write-Host "[3/4] Independent Rerun analyzer smoke test"
& $python ".\rerun_analysis\test_rerun_analysis.py"
if ($LASTEXITCODE -ne 0) { throw "Rerun analyzer test failed." }

if (-not $Recording) {
    $Recording = Get-ChildItem ".\recordings" -Filter "*.jsonl" |
        Where-Object Length -GT 100000 |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $Recording -or -not (Test-Path -LiteralPath $Recording)) {
    throw "A valid BODY_38 JSONL recording is required."
}
$resolvedRecording = (Resolve-Path -LiteralPath $Recording).Path
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$report = Join-Path $project "reports\validation_$stamp.json"
Write-Host "[4/4] Capture benchmark: $resolvedRecording"
& $python ".\tools\benchmark_motion_capture.py" `
    $resolvedRecording --target-fps "$TargetFps" --output $report
if ($LASTEXITCODE -ne 0) { throw "Capture benchmark failed." }

if (-not $SkipGmrReplay) {
    $wslHome = (& wsl.exe -d Ubuntu-22.04 -- bash -lc 'printf %s "$HOME"').Trim()
    if (-not $wslHome.StartsWith("/")) {
        throw "WSL home directory could not be resolved: $wslHome"
    }
    $gmrPython = "$wslHome/g1_isaaclab_project/envs/gmr_zed/bin/python"
    $wslProject = Convert-ToWslPath $project
    $wslRecording = Convert-ToWslPath $resolvedRecording
    & wsl.exe -d Ubuntu-22.04 -- bash -lc `
        "cd '$wslProject/isaaclab_bridge' && '$gmrPython' test_body38_orientation.py"
    if ($LASTEXITCODE -ne 0) {
        throw "BODY_38 orientation hierarchy test failed."
    }
    & wsl.exe -d Ubuntu-22.04 -- bash -lc `
        "cd '$wslProject/isaaclab_bridge' && '$gmrPython' test_occlusion_resilience.py '$wslRecording'"
    if ($LASTEXITCODE -ne 0) {
        throw "Occlusion resilience replay failed."
    }
    & wsl.exe -d Ubuntu-22.04 -- bash -lc `
        "cd '$wslProject' && '$gmrPython' isaaclab_bridge/test_live_bridge.py '$wslRecording'"
    if ($LASTEXITCODE -ne 0) {
        throw "GMR replay failed. Ensure no live process is using UDP 15050/15051."
    }
}

Write-Host "VALIDATION_OK"
Write-Host "Report: $report"
