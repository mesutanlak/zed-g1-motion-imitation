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
    Write-Host "Rerun bagimliliklari kuruluyor..."
    python -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) {
        throw "Rerun bagimliliklari kurulamadi."
    }
}

# pip installs rerun.exe under the active Python Scripts directory. Add that
# directory explicitly because it is not necessarily present in Windows PATH.
$scriptsDirectory = (& python -c "import sysconfig; print(sysconfig.get_path('scripts'))").Trim()
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

Write-Host "Bagimsiz Rerun BODY_38 analiz sistemi baslatiliyor..."
Write-Host "Mod: $Mode | BODY_38 UDP: ${ListenHost}:$ListenPort | GMR: $GmrListenPort"
Write-Host "Analiz hizi: ${LiveMaxHz} Hz | GMR telemetri kaydi: ${GmrLogMaxHz} Hz/sema"
Write-Host "Mevcut analysis_panel dosyalari kullanilmiyor/degistirilmiyor."

Push-Location $project
try {
    & python @arguments
}
finally {
    Pop-Location
}
