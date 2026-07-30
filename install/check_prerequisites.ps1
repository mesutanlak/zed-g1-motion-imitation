param(
    [string]$Distro = "Ubuntu-22.04",
    [string]$InstallRoot = "C:\g1il"
)

$ErrorActionPreference = "Continue"
$failed = $false

function Check-Command([string]$Name, [string]$Hint) {
    if (Get-Command $Name -ErrorAction SilentlyContinue) {
        Write-Host "[OK] $Name"
    } else {
        Write-Host "[EKSIK] $Name - $Hint" -ForegroundColor Red
        $script:failed = $true
    }
}

Write-Host "ZED G1 yeni bilgisayar on-kontrolu"
$os = Get-CimInstance Win32_OperatingSystem
$ramGb = [math]::Round($os.TotalVisibleMemorySize / 1MB, 1)
Write-Host "Windows: $($os.Caption) build $($os.BuildNumber)"
Write-Host "RAM: ${ramGb} GB (Isaac Lab icin 32 GB+, tercihen daha fazla)"
if ($ramGb -lt 32) {
    Write-Warning "Isaac Lab resmi taban gereksiniminin altinda RAM."
}

Check-Command "git" "Git for Windows kurun."
Check-Command "wsl.exe" "Windows Optional Features icinden WSL2'yi etkinlestirin."
Check-Command "nvidia-smi" "NVIDIA Production Branch surucusunu kurun."
Check-Command "py.exe" "64-bit Python 3.11 kurun."

if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
}

if (Get-Command py.exe -ErrorAction SilentlyContinue) {
    & py.exe -3.11 -c "import sys; print('[OK] Python', sys.version)" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[EKSIK] Python 3.11 x64" -ForegroundColor Red
        $failed = $true
    }
}

$distros = & wsl.exe --list --quiet 2>$null
if ($distros -contains $Distro) {
    Write-Host "[OK] WSL dagitimi: $Distro"
} else {
    Write-Host "[EKSIK] WSL dagitimi: $Distro" -ForegroundColor Red
    Write-Host "Yonetici PowerShell: wsl --install -d Ubuntu-22.04"
    $failed = $true
}

$zedRoot = "C:\Program Files (x86)\ZED SDK"
if (Test-Path (Join-Path $zedRoot "get_python_api.py")) {
    Write-Host "[OK] ZED SDK"
} else {
    Write-Host "[EKSIK] ZED SDK - Windows/CUDA uyumlu resmi kurucuyu yukleyin." -ForegroundColor Red
    $failed = $true
}

if (Test-Path $InstallRoot) {
    Write-Host "[BILGI] Isaac kok dizini mevcut: $InstallRoot"
}

if ($failed) {
    Write-Host "On-kontrol EKSIK. INSTALL_TR.md adimlarini tamamlayin." -ForegroundColor Yellow
    exit 1
}
Write-Host "On-kontrol BASARILI." -ForegroundColor Green
