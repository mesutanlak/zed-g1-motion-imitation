param([string]$Distro = "Ubuntu-22.04")
$ErrorActionPreference = "Stop"
$scriptPath = Join-Path $PSScriptRoot "setup_gmr_wsl.sh"
$drive = $scriptPath.Substring(0, 1).ToLowerInvariant()
$rest = $scriptPath.Substring(2).Replace("\", "/")
$wslScript = "/mnt/$drive$rest"
& wsl.exe -d $Distro -- bash $wslScript
if ($LASTEXITCODE -ne 0) {
    throw "GMR WSL kurulumu basarisiz (kod=$LASTEXITCODE)."
}
