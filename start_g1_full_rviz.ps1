param(
    [switch]$Headless
)

$ErrorActionPreference = "Stop"
$distro = "Ubuntu-22.04"
$project = [System.IO.Path]::GetFullPath((Split-Path -Parent $MyInvocation.MyCommand.Path))
$drive = $project.Substring(0, 1).ToLowerInvariant()
$rest = $project.Substring(2).Replace("\", "/")
$wslProject = "/mnt/$drive$rest"

foreach ($port in 15053, 15054) {
    & wsl.exe -d $distro -- bash -lc "ss -lun | grep -q ':$port '"
    if ($LASTEXITCODE -eq 0) {
        throw "UDP $port zaten kullanimda. Eski ROS koprusunu kapatin. Kontrol: wsl -d Ubuntu-22.04 -- ss -lunp"
    }
}

$headlessValue = if ($Headless) { "1" } else { "0" }
Write-Host "G1 + ZED + GMR + Isaac + sensor RViz sistemi baslatiliyor..."
Write-Host "ROS_DOMAIN_ID=42 | BODY38=15054 | retarget/Isaac=15053"
Write-Host "Bu pencere acik kalmalidir; kapatmak icin Ctrl+C kullanin."

& wsl.exe -d $distro -- bash -lc "export G1_RVIZ_HEADLESS=$headlessValue; exec bash '$wslProject/ros2_bridge/start_g1_full_rviz.sh'"
if ($LASTEXITCODE -ne 0) {
    throw "G1 tam RViz sistemi hata koduyla kapandi: $LASTEXITCODE"
}
