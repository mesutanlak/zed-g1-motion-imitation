$ErrorActionPreference = "Stop"

$distro = "Ubuntu-22.04"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProject = [System.IO.Path]::GetFullPath($project)
$drive = $resolvedProject.Substring(0, 1).ToLowerInvariant()
$rest = $resolvedProject.Substring(2).Replace("\", "/")
$wslProject = "/mnt/$drive$rest"
$launcher = "$wslProject/ros2_bridge/start_body38_ros_bridge.sh"

& wsl.exe -d $distro -- bash -lc "ss -lun | grep -q ':15054'"
if ($LASTEXITCODE -eq 0) {
    throw "UDP 15054 zaten kullanimda. Eski BODY_38 ROS koprusunu kapatin."
}

Write-Host "ZED BODY_38 -> ROS 2 koprusu"
Write-Host "  UDP giris : 0.0.0.0:15054"
Write-Host "  ROS domain: 42"
Write-Host "  Topicler  : /zed/body38/markers, /zed/body38/keypoints"
Write-Host "Bu pencere kopru calistigi surece acik kalmalidir."

& wsl.exe -d $distro -- bash $launcher
if ($LASTEXITCODE -ne 0) {
    throw "BODY_38 ROS koprusu hata koduyla kapandi: $LASTEXITCODE"
}
