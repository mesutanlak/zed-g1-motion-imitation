$ErrorActionPreference = "Stop"

$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$drive = $project.Substring(0, 1).ToLowerInvariant()
$rest = $project.Substring(2).Replace("\", "/")
$launcher = "/mnt/$drive$rest/sim_mujoco/run_g1_zed_obstacles.sh"

Write-Host "WSL Ubuntu-22.04: G1 ZED fizik + obstacle + ROS 2 baslatiliyor..."
& wsl.exe -d Ubuntu-22.04 -- bash $launcher
if ($LASTEXITCODE -ne 0) {
    throw "G1 simulasyonu hata koduyla kapandi: $LASTEXITCODE"
}
