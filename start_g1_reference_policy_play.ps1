param(
    [Parameter(Mandatory = $true)]
    [string]$MotionNpz,
    [Parameter(Mandatory = $true)]
    [string]$Checkpoint,
    [string]$InstallRoot = "C:\g1il",
    [switch]$Headless,
    [switch]$AcceptNvidiaEula
)

$ErrorActionPreference = "Stop"
if (-not $AcceptNvidiaEula) {
    throw "Isaac Sim EULA'yi kabul ediyorsaniz -AcceptNvidiaEula ekleyin."
}
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$motion = (Resolve-Path -LiteralPath $MotionNpz).Path
$checkpointPath = (Resolve-Path -LiteralPath $Checkpoint).Path
$python = Join-Path $InstallRoot "env\Scripts\python.exe"
$play = Join-Path $InstallRoot "repos\unitree_rl_lab\scripts\rsl_rl\play.py"
$usd = Join-Path $InstallRoot "cache\g1_23dof\g1_23dof_rev_1_0.usd"
& (Join-Path $project "install_reference_policy_task.ps1") -InstallRoot $InstallRoot
$env:G1_REFERENCE_MOTION = $motion
$env:G1_23DOF_USD = $usd
$env:G1_PROJECT_ROOT = $project
$env:PYTHONPATH = "$project;$env:PYTHONPATH"
$env:OMNI_KIT_ACCEPT_EULA = "YES"
$env:PRIVACY_CONSENT = "Y"

$arguments = @(
    $play,
    "--task", "Unitree-G1-23dof-Reference-Motion",
    "--num_envs", 1,
    "--checkpoint", $checkpointPath,
    "--real-time"
)
if ($Headless) { $arguments += "--headless" }
& $python @arguments
exit $LASTEXITCODE
