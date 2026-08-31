param(
    [string]$InstallRoot = "C:\g1il"
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$source = Join-Path $project "reference_policy\unitree_rl_lab_task\reference_motion"
$robotTasks = Join-Path $InstallRoot "repos\unitree_rl_lab\source\unitree_rl_lab\unitree_rl_lab\tasks\mimic\robots\g1_23dof"
$target = Join-Path $robotTasks "reference_motion"

if (-not (Test-Path -LiteralPath $source)) {
    throw "Reference policy source not found: $source"
}
New-Item -ItemType Directory -Force -Path $robotTasks, $target | Out-Null
$packageMarker = Join-Path $robotTasks "__init__.py"
if (-not (Test-Path -LiteralPath $packageMarker)) {
    New-Item -ItemType File -Path $packageMarker | Out-Null
}

Get-ChildItem -LiteralPath $source -File -Recurse | ForEach-Object {
    $relative = $_.FullName.Substring($source.Length).TrimStart("\")
    $destination = Join-Path $target $relative
    $destinationParent = Split-Path -Parent $destination
    New-Item -ItemType Directory -Force -Path $destinationParent | Out-Null
    Copy-Item -LiteralPath $_.FullName -Destination $destination -Force
}

Write-Host "G1 23-DOF reference-motion task installed: $target" -ForegroundColor Green
