$ErrorActionPreference = "Stop"

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$drive = $projectDir.Substring(0, 1).ToLowerInvariant()
$rest = $projectDir.Substring(2).Replace("\", "/")
$wslScript = "/mnt/$drive$rest/sim_mujoco/run_g1_zed_mimic.sh"
wsl -d Ubuntu-22.04 -- bash $wslScript
