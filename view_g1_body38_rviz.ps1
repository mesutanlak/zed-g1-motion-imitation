$ErrorActionPreference = "Stop"

$distro = "Ubuntu-22.04"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProject = [System.IO.Path]::GetFullPath($project)
$drive = $resolvedProject.Substring(0, 1).ToLowerInvariant()
$rest = $resolvedProject.Substring(2).Replace("\", "/")
$wslProject = "/mnt/$drive$rest"
$launcher = "$wslProject/ros2_bridge/view_body38_rviz.sh"

Write-Host "RViz: ZED BODY_38 iskeleti aciliyor..."
Write-Host "  Fixed Frame: zed_camera"
Write-Host "  Topic      : /zed/body38/markers"

& wsl.exe -d $distro -- bash $launcher
if ($LASTEXITCODE -ne 0) {
    throw "BODY_38 RViz hata koduyla kapandi: $LASTEXITCODE"
}
