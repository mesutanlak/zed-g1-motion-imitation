param(
    [ValidateSet("live", "playback", "demo")]
    [string]$Mode = "live",
    [string]$InputFile = "",
    [string]$Svo2Path = "",
    [string]$ListenHost = "0.0.0.0",
    [int]$ListenPort = 15052,
    [int]$GmrListenPort = 15053,
    [ValidateRange(1, 60)]
    [int]$LiveMaxHz = 15,
    [ValidateRange(1, 60)]
    [int]$GmrLogMaxHz = 15,
    [switch]$DetailedJointEntities,
    [switch]$NoViewer,
    [switch]$Headless,
    [switch]$NoRealtime
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$requirements = Join-Path $project "requirements-rerun.txt"
$output = Join-Path $project "rerun_recordings"
$venv = Join-Path $project ".venv-rerun"
$python = Join-Path $venv "Scripts\python.exe"

if (-not (Test-Path -LiteralPath $python)) {
    Write-Host "Rerun icin bagimsiz Python ortami olusturuluyor..."
    & py.exe -3.11 -m venv $venv
    if ($LASTEXITCODE -ne 0) {
        throw "Rerun Python ortami olusturulamadi. Python 3.11 kurulumunu kontrol edin."
    }
}

& $python -c "import rerun, numpy" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Rerun bagimliliklari kuruluyor..."
    & $python -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) {
        throw "Rerun bagimliliklari kurulamadi."
    }
}

# The virtual environment path is already known to PowerShell. Building the
# Scripts path locally avoids corrupting non-ASCII characters when Windows
# PowerShell decodes Python's stdout (for example, "Masaustu" with an umlaut).
$scriptsDirectory = Join-Path $venv "Scripts"
$rerunViewer = Join-Path $scriptsDirectory "rerun.exe"
if (-not (Test-Path -LiteralPath $rerunViewer)) {
    throw "rerun.exe bulunamadi: $rerunViewer"
}
$env:PATH = "$scriptsDirectory;$env:PATH"

$arguments = @(
    "-m", "rerun_analysis.app",
    "--listen-host", $ListenHost,
    "--listen-port", "$ListenPort",
    "--gmr-listen-port", "$GmrListenPort",
    "--live-max-hz", "$LiveMaxHz",
    "--gmr-log-max-hz", "$GmrLogMaxHz",
    "--output-dir", $output
)

switch ($Mode) {
    "demo" {
        $arguments += "--demo"
    }
    "playback" {
        if (-not $InputFile) {
            throw "Playback icin -InputFile zorunludur."
        }
        $resolvedInput = (Resolve-Path -LiteralPath $InputFile).Path
        $arguments += @("--input", $resolvedInput)
    }
}

if ($Svo2Path) {
    $resolvedSvo = (Resolve-Path -LiteralPath $Svo2Path).Path
    $arguments += @("--svo2", $resolvedSvo)
}
if ($NoViewer) {
    $arguments += "--no-viewer"
}
if ($Headless) {
    $arguments += "--headless"
}
if ($NoRealtime) {
    $arguments += "--no-realtime"
}
if ($DetailedJointEntities) {
    $arguments += "--detailed-joint-entities"
}

Write-Host "Bagimsiz Rerun BODY_38 + Dex3 el analiz sistemi baslatiliyor..."
Write-Host "Mod: $Mode | BODY_38 UDP: ${ListenHost}:$ListenPort | GMR: $GmrListenPort"
Write-Host "Analiz hizi: ${LiveMaxHz} Hz | GMR telemetri kaydi: ${GmrLogMaxHz} Hz/sema"
Write-Host "Mevcut analysis_panel dosyalari kullanilmiyor/degistirilmiyor."

Push-Location $project
try {
    & $python @arguments
}
finally {
    Pop-Location
}
