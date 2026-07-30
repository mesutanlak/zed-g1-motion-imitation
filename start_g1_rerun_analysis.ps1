param(
    [ValidateSet("live", "playback", "demo")]
    [string]$Mode = "live",
    [string]$InputFile = "",
    [string]$Svo2Path = "",
    [string]$ListenHost = "0.0.0.0",
    [int]$ListenPort = 15052,
    [switch]$NoViewer,
    [switch]$Headless,
    [switch]$NoRealtime
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$requirements = Join-Path $project "requirements-rerun.txt"
$output = Join-Path $project "rerun_recordings"

python -c "import rerun, numpy" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Rerun bağımlılıkları kuruluyor..."
    python -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) {
        throw "Rerun bağımlılıkları kurulamadı."
    }
}

$arguments = @(
    "-m", "rerun_analysis.app",
    "--listen-host", $ListenHost,
    "--listen-port", "$ListenPort",
    "--output-dir", $output
)
switch ($Mode) {
    "demo" { $arguments += "--demo" }
    "playback" {
        if (-not $InputFile) {
            throw "Playback için -InputFile zorunludur."
        }
        $resolvedInput = (Resolve-Path -LiteralPath $InputFile).Path
        $arguments += @("--input", $resolvedInput)
    }
}
if ($Svo2Path) {
    $resolvedSvo = (Resolve-Path -LiteralPath $Svo2Path).Path
    $arguments += @("--svo2", $resolvedSvo)
}
if ($NoViewer) { $arguments += "--no-viewer" }
if ($Headless) { $arguments += "--headless" }
if ($NoRealtime) { $arguments += "--no-realtime" }

Write-Host "Bağımsız Rerun BODY_38 analiz sistemi başlatılıyor..."
Write-Host "Mod: $Mode | UDP: ${ListenHost}:$ListenPort"
Write-Host "Mevcut analysis_panel dosyaları kullanılmıyor/değiştirilmiyor."
Push-Location $project
try {
    & python @arguments
}
finally {
    Pop-Location
}
