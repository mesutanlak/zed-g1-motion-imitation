$ErrorActionPreference = "Stop"

$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$drive = $project.Substring(0, 1).ToLowerInvariant()
$rest = $project.Substring(2).Replace("\", "/")
$config = "/mnt/$drive$rest/rviz/g1_sensors.rviz"
$command = @"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=42
exec rviz2 -d '$config'
"@

& wsl.exe -d Ubuntu-22.04 -- bash -lc $command
if ($LASTEXITCODE -ne 0) {
    throw "RViz hata koduyla kapandi: $LASTEXITCODE"
}
