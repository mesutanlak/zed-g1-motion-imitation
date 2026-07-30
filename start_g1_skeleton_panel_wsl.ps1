param(
    [ValidateRange(1, 65535)]
    [int]$Port = 15052,
    [ValidateRange(0, 7)]
    [int]$ScreenIndex = 0,
    [switch]$Record,
    [switch]$NoCsv,
    [switch]$UseWslg
)

$ErrorActionPreference = "Stop"
$distro = "Ubuntu-22.04"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProject = [System.IO.Path]::GetFullPath($project)
$recordDirectory = Join-Path $resolvedProject "analysis_recordings"
$panelPython = Join-Path $resolvedProject ".venv-zed\Scripts\python.exe"
if (-not (Test-Path $panelPython)) {
    $panelPython = "python"
}

if (-not $UseWslg) {
    # The sensor and Isaac/GMR paths remain connected to WSL. Only the display
    # runs natively because WSLg can leave Qt windows in RDP COPY MODE.
    $viewer = Join-Path $resolvedProject `
        "analysis_panel\g1_skeleton_3d_viewer_tk.py"
    if (-not (Test-Path -LiteralPath $viewer)) {
        throw "Windows native 3B analiz paneli bulunamadi: $viewer"
    }

    & $panelPython -c "import tkinter; print('Tkinter OK - Windows native panel')"
    if ($LASTEXITCODE -ne 0) {
        throw "Windows Python icinde Tkinter kullanilamiyor."
    }

    $existing = Get-NetUDPEndpoint -LocalPort $Port -ErrorAction SilentlyContinue
    if ($existing) {
        throw "Windows UDP $Port zaten kullanimda. Eski analiz panelini kapatin."
    }

    Write-Host "G1 canli 3B iskelet paneli"
    Write-Host "  Calisma modu : Windows native (WSLg COPY MODE kullanilmaz)"
    Write-Host "  UDP dinleme  : 0.0.0.0:$Port"
    Write-Host "  Gorunum      : Front view (Y-Z), X derinlik rengi"
    Write-Host "  Analiz kaydi : S tusu ile ac/kapat"
    Write-Host "  Kayit klasoru: $recordDirectory"
    Write-Host "ZED baslaticisini ayri PowerShell'de calistirin."

    $viewerArguments = @(
        $viewer,
        "--listen-host", "0.0.0.0",
        "--listen-port", "$Port",
        "--record-dir", $recordDirectory
    )
    if ($Record) {
        $viewerArguments += "--record"
    }
    if ($NoCsv) {
        $viewerArguments += "--no-csv"
    }

    & $panelPython @viewerArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Windows native 3B analiz paneli hata koduyla kapandi: $LASTEXITCODE"
    }
    exit 0
}

# Optional diagnostic fallback. Normal project use should stay in native mode.
if ($resolvedProject -notmatch "^[A-Za-z]:\\") {
    throw "Windows proje yolu taninamadi: $resolvedProject"
}
$drive = $resolvedProject.Substring(0, 1).ToLowerInvariant()
$rest = $resolvedProject.Substring(2).Replace("\", "/")
$wslProject = "/mnt/$drive$rest"
$viewer = "$wslProject/analysis_panel/g1_skeleton_3d_viewer.py"

$distros = (& wsl.exe --list --quiet) -replace "`0", ""
if (-not ($distros | Where-Object { $_.Trim() -eq $distro })) {
    throw "WSL dagitimi bulunamadi: $distro"
}
& wsl.exe -d $distro -- python3 -c "import PyQt5; print('PyQt5 OK - WSLg fallback')"
if ($LASTEXITCODE -ne 0) {
    throw "Ubuntu 22.04 icinde PyQt5 kullanilamiyor."
}
& wsl.exe -d $distro -- bash -lc "ss -lun | grep -q ':$Port'"
if ($LASTEXITCODE -eq 0) {
    throw "WSL UDP $Port zaten kullanimda. Eski analiz panelini kapatin."
}

$viewerArguments = @(
    "-d", $distro, "--cd", $wslProject, "--",
    "python3", $viewer,
    "--listen-host", "0.0.0.0",
    "--listen-port", "$Port",
    "--screen-index", "$ScreenIndex"
)
if ($Record) {
    $viewerArguments += "--record"
}
if ($NoCsv) {
    $viewerArguments += "--no-csv"
}
& wsl.exe @viewerArguments
if ($LASTEXITCODE -ne 0) {
    throw "WSLg 3B analiz paneli hata koduyla kapandi: $LASTEXITCODE"
}
