param(
    [Parameter(Mandatory = $true)]
    [string]$Recording
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $project ".venv-rerun\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "Rerun ortami bulunamadi. Once .\start_g1_rerun.ps1 calistirin."
}
$resolved = (Resolve-Path -LiteralPath $Recording).Path
$scriptsDirectory = Join-Path $project ".venv-rerun\Scripts"
$viewer = Join-Path $scriptsDirectory "rerun.exe"
if (-not (Test-Path -LiteralPath $viewer)) {
    throw "Rerun Viewer bulunamadi. Once: python -m pip install -r .\requirements-rerun.txt"
}
Write-Host "Rerun kaydi aciliyor: $resolved"
& $viewer $resolved
