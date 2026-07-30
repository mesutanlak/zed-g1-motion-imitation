$ErrorActionPreference = "Stop"

$processes = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "^python(?:\.exe)?$" -and
    $_.CommandLine -match "zed_g1_skeleton\.py"
}

if (-not $processes) {
    Write-Host "Çalışan ZED BODY_38 süreci bulunamadı."
    exit 0
}

foreach ($process in $processes) {
    Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
    Write-Host "ZED süreci kapatıldı: PID $($process.ProcessId)"
}

