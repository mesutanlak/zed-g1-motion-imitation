param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("MainPC", "Laptop")]
    [string]$Role,
    [Parameter(Mandatory = $true)]
    [string]$ExpectedIPv4,
    [Parameter(Mandatory = $true)]
    [string]$PeerIPv4,
    [Parameter(Mandatory = $true)]
    [long[]]$Serial,
    [int[]]$Ports = @(30000, 30010, 30020, 30030, 16000, 16002, 16004, 16006)
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
$cameraScript = Join-Path $root "zed_g1_skeleton.py"
$failures = [System.Collections.Generic.List[string]]::new()

if ($Serial.Count -ne 2 -or ($Serial | Sort-Object -Unique).Count -ne 2) {
    throw "Bu host icin iki benzersiz ZED seri numarasi verin."
}
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi: $python"
}

Write-Host "=== $Role DORT KAMERA ON KONTROL ==="
$address = Get-NetIPAddress -AddressFamily IPv4 -IPAddress $ExpectedIPv4 -ErrorAction SilentlyContinue | Select-Object -First 1
if ($null -eq $address) {
    $failures.Add("Beklenen Ethernet IPv4 bulunamadi: $ExpectedIPv4")
}
else {
    $adapter = Get-NetAdapter -InterfaceIndex $address.InterfaceIndex
    Write-Host "Ethernet: $($adapter.Name) | durum=$($adapter.Status) | hiz=$($adapter.LinkSpeed) | IP=$ExpectedIPv4/$($address.PrefixLength)"
    if ($adapter.Status -ne "Up") {
        $failures.Add("Ethernet bagdastiricisi Up degil.")
    }
    if ($address.PrefixLength -ne 24) {
        $failures.Add("Bu kurulumda PrefixLength 24 (255.255.255.0) olmali; bulunan: $($address.PrefixLength)")
    }
}

$pingReplies = @(Test-Connection -ComputerName $PeerIPv4 -Count 8 -BufferSize 1400 -ErrorAction SilentlyContinue)
if ($pingReplies.Count -eq 0) {
    $failures.Add("Es bilgisayara ping yok: $PeerIPv4")
}
else {
    $latencies = @($pingReplies | ForEach-Object {
        if ($null -ne $_.ResponseTime) { [double]$_.ResponseTime }
        elseif ($null -ne $_.Latency) { [double]$_.Latency }
    })
    $average = if ($latencies.Count) { [math]::Round(($latencies | Measure-Object -Average).Average, 2) } else { "?" }
    Write-Host "Ping: $($pingReplies.Count)/8 yanit | 1400 byte | ortalama=${average}ms"
    if ($pingReplies.Count -lt 8) {
        $failures.Add("Ping kaybi var: $($pingReplies.Count)/8 yanit.")
    }
}

Write-Host "Saat (UTC): $((Get-Date).ToUniversalTime().ToString('yyyy-MM-dd HH:mm:ss.fff'))"
$timeService = Get-Service -Name W32Time -ErrorAction SilentlyContinue
if ($null -eq $timeService -or $timeService.Status -ne "Running") {
    $failures.Add("Windows Time hizmeti calismiyor. Yonetici PowerShell: Start-Service W32Time; w32tm /resync /force")
}
try {
    $timeSource = (& w32tm.exe /query /source 2>$null | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Windows saat kaynagi bu oturumdan okunamadi: $timeSource. Yonetici PowerShell ciktisini esas alin."
    }
    else {
        Write-Host "Windows saat kaynagi: $timeSource"
    }
}
catch {
    Write-Warning "w32tm saat kaynagi okunamadi."
    $failures.Add("Windows Time hizmeti/saat kaynagi okunamadi.")
}

$sdk = & $python -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
if ($LASTEXITCODE -ne 0) {
    $failures.Add("pyzed yuklenemedi.")
}
else {
    Write-Host "ZED SDK: $sdk"
    if ($sdk.Trim() -ne "5.4.1") {
        $failures.Add("Bugun dogrulanan surum 5.4.1; bulunan: $sdk")
    }
}

$deviceText = (& $python $cameraScript --list-devices 2>&1 | Out-String)
Write-Host $deviceText.Trim()
foreach ($item in $Serial) {
    if ($deviceText -notmatch "serial=$item\s") {
        $failures.Add("ZED $item bu hostta AVAILABLE listesinde yok.")
    }
}

$busy = @(Get-NetUDPEndpoint -ErrorAction SilentlyContinue | Where-Object { $_.LocalPort -in $Ports })
if ($busy.Count) {
    Write-Warning "Test portlarindan bazilari zaten kullanimda:"
    $busy | Sort-Object LocalPort | Format-Table LocalAddress, LocalPort, OwningProcess -AutoSize
    $failures.Add("Publisher/receiver baslamadan once test UDP portlari bos olmali.")
}
else {
    Write-Host "UDP portlari bos: $($Ports -join ', ')"
}

$cameraProcesses = @(Get-Process -ErrorAction SilentlyContinue | Where-Object {
    $_.ProcessName -match "ZED|python"
})
if ($cameraProcesses.Count) {
    Write-Warning "Kamerayi veya test portunu tutabilecek surecler var; denemeden once kapatin:"
    $cameraProcesses | Select-Object ProcessName, Id, Path | Format-Table -AutoSize
}

$profiles = Get-NetFirewallProfile -ErrorAction SilentlyContinue
if ($profiles) {
    $profileText = (@($profiles | ForEach-Object { "$($_.Name)=$($_.Enabled)" })) -join ", "
    Write-Host "Windows Firewall: $profileText"
}
$norton = @(Get-Service -ErrorAction SilentlyContinue | Where-Object {
    $_.DisplayName -match "Norton" -and $_.Status -eq "Running"
})
if ($norton.Count) {
    Write-Warning "Norton etkin. ZED360.exe ve .venv-zed\Scripts\python.exe icin ozel ag/UDP izni Norton icinden verilmelidir."
}

if ($failures.Count) {
    Write-Host "ON KONTROL: BASARISIZ" -ForegroundColor Red
    $failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 2
}
Write-Host "ON KONTROL: BASARILI" -ForegroundColor Green
Write-Host "Not: Iki bilgisayarda bu komutu arka arkaya calistirin; UTC satirlari arasindaki fark 20 ms altinda olmali."
exit 0
