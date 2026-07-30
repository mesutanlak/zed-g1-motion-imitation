param(
    [Parameter(Mandatory = $true)]
    [string]$Recording
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$resolved = (Resolve-Path -LiteralPath $Recording).Path
$scriptsDirectory = (& python -c "import sysconfig; print(sysconfig.get_path('scripts'))").Trim()
$viewer = Join-Path $scriptsDirectory "rerun.exe"
if (-not (Test-Path -LiteralPath $viewer)) {
    throw "Rerun Viewer bulunamadi. Once: python -m pip install -r .\requirements-rerun.txt"
}
Write-Host "Rerun kaydi aciliyor: $resolved"
& $viewer $resolved
