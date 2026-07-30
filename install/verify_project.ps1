param(
    [string]$InstallRoot = "C:\g1il",
    [string]$Distro = "Ubuntu-22.04"
)

$ErrorActionPreference = "Continue"
$project = Split-Path -Parent $PSScriptRoot
$zedPython = Join-Path $project ".venv-zed\Scripts\python.exe"
$isaacPython = Join-Path $InstallRoot "env\Scripts\python.exe"

Write-Host "1/5 Proje Python syntax"
& $zedPython -m compileall -q $project
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "2/5 ZED Python API"
& $zedPython -c "import pyzed.sl as sl; import cv2; print('ZED API OK')"
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "3/5 Isaac Lab paketleri"
& $isaacPython (Join-Path $project "isaaclab_bridge\test_environment.py")
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "4/5 GMR paketleri"
& wsl.exe -d $Distro -- bash -lc '$HOME/g1_isaaclab_project/envs/gmr_zed/bin/python -c "import mujoco,mink,general_motion_retargeting; print(\"GMR OK\")"'
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "5/5 PowerShell betikleri"
$parseErrors = @()
Get-ChildItem $project -Filter *.ps1 | ForEach-Object {
    [void][System.Management.Automation.Language.Parser]::ParseFile(
        $_.FullName, [ref]$null, [ref]$parseErrors
    )
}
if ($parseErrors.Count -gt 0) {
    $parseErrors
    exit 1
}
Write-Host "Tum temel dogrulamalar BASARILI." -ForegroundColor Green
