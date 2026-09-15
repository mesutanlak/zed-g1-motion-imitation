param(
    [string]$InstallRoot = "C:\g1il",
    [switch]$FetchAssets,
    [switch]$SkipRetargetEnvironment,
    [switch]$TryWindowsOfficialRetargeting
)

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent $PSScriptRoot
$lockPath = Join-Path $project "config\unitree_official_sources.lock.json"
$lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json
$repos = Join-Path $InstallRoot "repos"
New-Item -ItemType Directory -Path $repos -Force | Out-Null

function Install-PinnedRepository {
    param([string]$Name, [object]$Source, [switch]$Recursive)
    $destination = Join-Path $repos $Name
    if (-not (Test-Path -LiteralPath (Join-Path $destination ".git"))) {
        $arguments = @("clone")
        if ($Recursive) { $arguments += "--recursive" }
        $arguments += @($Source.url, $destination)
        # Native stdout is part of a PowerShell function's return stream.
        # Send git progress to the host so $simRoot/$xrRoot receive only the
        # explicit destination string returned at the end of this function.
        & git @arguments | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "$Name clone basarisiz." }
    }
    & git -C $destination fetch origin $Source.commit | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Name pinned commit fetch basarisiz." }
    & git -C $destination checkout --detach $Source.commit | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Name pinned commit checkout basarisiz." }
    if ($Recursive) {
        & git -C $destination submodule update --init --recursive | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "$Name submodule kurulumu basarisiz." }
    }
    Write-Output $destination
}

$simRoot = Install-PinnedRepository "unitree_sim_isaaclab" $lock.sources.unitree_sim_isaaclab
$xrRoot = Install-PinnedRepository "xr_teleoperate" $lock.sources.xr_teleoperate -Recursive

if (-not $SkipRetargetEnvironment -and $TryWindowsOfficialRetargeting) {
    $basePython = Join-Path $InstallRoot "env\Scripts\python.exe"
    if (-not (Test-Path -LiteralPath $basePython -PathType Leaf)) {
        throw "Isaac Python bulunamadi: $basePython"
    }
    $dexEnv = Join-Path $InstallRoot "envs\dex3"
    $dexPython = Join-Path $dexEnv "Scripts\python.exe"
    if (-not (Test-Path -LiteralPath $dexPython -PathType Leaf)) {
        & $basePython -m venv $dexEnv
        if ($LASTEXITCODE -ne 0) { throw "Izole Dex3 Python ortami olusturulamadi." }
    }
    & $dexPython -m pip install --upgrade pip
    if ($LASTEXITCODE -ne 0) { throw "Dex3 pip guncellemesi basarisiz." }
    $dexPackage = Join-Path $xrRoot "teleop\robot_control\dex-retargeting"
    & $dexPython -m pip install $dexPackage
    if ($LASTEXITCODE -ne 0) { throw "Resmi dex-retargeting kurulumu basarisiz." }
    Write-Host "Izole resmi DexPilot Python: $dexPython"
}
elseif (-not $SkipRetargetEnvironment) {
    Write-Warning (
        "Resmi dex-retargeting Pinocchio bagimliligi Windows wheel saglamiyor. " +
        "Canli sistem dusuk gecikmeli 21-landmark fallback kullanacak; " +
        "deneysel Windows derlemesi icin -TryWindowsOfficialRetargeting verilebilir."
    )
}

if ($FetchAssets) {
    $resolved = [System.IO.Path]::GetFullPath($simRoot)
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $rest = $resolved.Substring(2).Replace("\", "/")
    $wslRoot = "/mnt/$drive$rest"
    Write-Warning "Resmi Unitree asset arsivi 1 GB'den buyuktur. Resmi fetch_assets.sh simdi calisacak."
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "command -v git-lfs >/dev/null 2>&1 && command -v unzip >/dev/null 2>&1 || { sudo apt-get update && sudo apt-get install -y git-lfs unzip; }"
    if ($LASTEXITCODE -ne 0) { throw "WSL git-lfs/unzip kurulumu basarisiz." }
    # Git for Windows may checkout the official shell script with CRLF. Keep
    # the pinned repository untouched and normalize only the stream Bash sees.
    & wsl.exe -d Ubuntu-22.04 -- bash -lc "cd '$wslRoot' && sed 's/\r$//' fetch_assets.sh | bash"
    if ($LASTEXITCODE -ne 0) { throw "Resmi Unitree asset indirme/acma islemi basarisiz." }
}

$dex3Usd = Join-Path $simRoot $lock.sources.unitree_sim_isaaclab.asset
$retargetConfig = Join-Path $xrRoot $lock.sources.xr_teleoperate.required_config
if (-not (Test-Path -LiteralPath $retargetConfig -PathType Leaf)) {
    throw "Resmi Dex3 retarget config bulunamadi: $retargetConfig"
}
if (-not (Test-Path -LiteralPath $dex3Usd -PathType Leaf)) {
    Write-Warning "Dex3 USD henuz yok. Asset icin komutu -FetchAssets ile tekrar calistirin."
} else {
    Write-Host "Resmi Dex3 USD hazir: $dex3Usd"
}
Write-Host "Resmi xr_teleoperate retarget kok dizini: $xrRoot"
Write-Host "DDS kurulmadı/baslatilmadi; fiziksel robot cikisi KAPALI."
