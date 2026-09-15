param(
    [string]$ModelPath = "C:\ZED_G1\models\hand_landmarker.task",
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$requirements = Join-Path $root "requirements-hand.txt"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED sanal ortami bulunamadi: $python"
}

& $python -c "import sys; assert (3,9) <= sys.version_info[:2] <= (3,12), sys.version; print(sys.version)"
if ($LASTEXITCODE -ne 0) { throw "MediaPipe icin desteklenmeyen Python surumu." }

if (-not $SkipInstall) {
    & $python -m pip install -r $requirements
    if ($LASTEXITCODE -ne 0) { throw "El takip dependency kurulumu basarisiz." }
}

& $python -c "import cv2, mediapipe, numpy, pyzed.sl as sl; print('opencv', cv2.__version__, 'mediapipe', mediapipe.__version__, 'numpy', numpy.__version__, 'ZED', sl.Camera.get_sdk_version())"
if ($LASTEXITCODE -ne 0) { throw "pyzed/MediaPipe birlikte import edilemedi." }

if (Test-Path -LiteralPath $ModelPath) {
    $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $ModelPath
    Write-Host "MODEL OK | $($hash.Path) | SHA256=$($hash.Hash)"
}
else {
    Write-Warning "Model bulunamadi: $ModelPath"
    Write-Warning "Resmi hand_landmarker.task dosyasini elle indirin; uygulama internetten indirmez."
}

Write-Host "El takip ortami hazir. Fiziksel robot cikisi kurulmaz veya etkinlestirilmez."
