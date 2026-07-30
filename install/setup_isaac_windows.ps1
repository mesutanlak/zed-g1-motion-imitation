param(
    [string]$InstallRoot = "C:\g1il",
    [switch]$AcceptNvidiaEula
)

$ErrorActionPreference = "Stop"
if (-not $AcceptNvidiaEula) {
    throw "NVIDIA Omniverse EULA'yi okuyup kabul ediyorsaniz -AcceptNvidiaEula ekleyin: https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html"
}

$repos = Join-Path $InstallRoot "repos"
$venv = Join-Path $InstallRoot "env"
$python = Join-Path $venv "Scripts\python.exe"
New-Item -ItemType Directory -Force -Path $repos, (Join-Path $InstallRoot "cache") | Out-Null

if (-not (Test-Path $python)) {
    & py.exe -3.11 -m venv $venv
}
& $python -m pip install --upgrade pip setuptools wheel
& $python -m pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 `
    --index-url https://download.pytorch.org/whl/cu128
& $python -m pip install "isaacsim[all,extscache]==5.0.0" `
    --extra-index-url https://pypi.nvidia.com

function Sync-Repo(
    [string]$Name,
    [string]$Url,
    [string]$Commit
) {
    $target = Join-Path $repos $Name
    if (-not (Test-Path (Join-Path $target ".git"))) {
        git clone $Url $target
    }
    git -C $target fetch --all --tags
    git -C $target checkout $Commit
}

Sync-Repo "IsaacLab" "https://github.com/isaac-sim/IsaacLab.git" `
    "46dff135f44683f031edf346e544fcfd8456b2bb"
Sync-Repo "unitree_rl_lab" "https://github.com/unitreerobotics/unitree_rl_lab.git" `
    "4960b84732b0c2ec593dccbfe963fda1bcd7b1e3"
Sync-Repo "unitree_ros" "https://github.com/unitreerobotics/unitree_ros.git" `
    "ac7714828e1a4cc2a8ddd5c78a1305475b913a73"

$isaacLab = Join-Path $repos "IsaacLab"
foreach ($package in @("isaaclab", "isaaclab_assets", "isaaclab_rl", "isaaclab_tasks")) {
    & $python -m pip install -e (Join-Path $isaacLab "source\$package")
}
& $python -m pip install -e (Join-Path $repos "unitree_rl_lab\source\unitree_rl_lab")
& $python -m pip install onnxruntime==1.28.0 PyYAML==6.0.2 toml==0.10.2

$env:OMNI_KIT_ACCEPT_EULA = "YES"
$env:PRIVACY_CONSENT = "Y"
& $python -c "import isaacsim, isaaclab, unitree_rl_lab, torch; print('Isaac/Unitree OK, CUDA=', torch.cuda.is_available())"
Write-Host "Isaac Lab ortami hazir: $python" -ForegroundColor Green
