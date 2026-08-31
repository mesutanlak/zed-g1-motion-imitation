param(
    [Parameter(Mandatory = $true)]
    [string]$SourceConfig,
    [Parameter(Mandatory = $true)]
    [string[]]$Camera,
    [Parameter(Mandatory = $true)]
    [string]$OutputConfig
)

$ErrorActionPreference = "Stop"
if (-not (Test-Path -LiteralPath $SourceConfig)) {
    throw "Kaynak ZED360 JSON bulunamadi: $SourceConfig"
}
if ($Camera.Count -ne 4) {
    throw "Dort adet -Camera SERIAL@IP:PORT degeri verin."
}

$endpoints = @{}
foreach ($item in $Camera) {
    try {
        $serialText, $endpoint = $item -split "@", 2
        $ip, $portText = $endpoint -split ":", 2
        $serial = [int]$serialText
        $port = [int]$portText
    }
    catch {
        throw "Gecersiz kamera bicimi '$item'. SERIAL@IP:PORT kullanin."
    }
    if ($endpoints.ContainsKey($serial) -or [string]::IsNullOrWhiteSpace($ip) -or $port -lt 1 -or $port -gt 65535 -or ($port % 2 -ne 0)) {
        throw "Gecersiz veya yinelenen kamera endpoint'i: $item"
    }
    $endpoints[$serial] = @{ Ip = $ip; Port = $port }
}

try {
    $configuration = Get-Content -LiteralPath $SourceConfig -Raw | ConvertFrom-Json
}
catch {
    throw "Kaynak JSON okunamadi: $($_.Exception.Message)"
}

$sourceSerials = @($configuration.PSObject.Properties | ForEach-Object { [int]$_.Value.FusionConfiguration.serial_number })
if (($sourceSerials | Sort-Object -Unique).Count -ne 4 -or ($sourceSerials | Sort-Object -Unique | Compare-Object ($endpoints.Keys | Sort-Object))) {
    throw "Kaynak JSON ve -Camera serileri ayni dort kamerayi icermeli. JSON: $($sourceSerials -join ', ')"
}

foreach ($property in $configuration.PSObject.Properties) {
    $fusion = $property.Value.FusionConfiguration
    $serial = [int]$fusion.serial_number
    $endpoint = $endpoints[$serial]
    $communication = $fusion.communication_parameters.CommunicationParameters
    $communication.communication_type = "LOCAL NETWORK"
    $communication.ip_add = $endpoint.Ip
    $communication.ip_port = $endpoint.Port
    # ZED360 bu seed'i yeni tripod duzeni icin optimize eder; eski extrinsic
    # degerler yeni rigde kullanilamaz.
    $fusion.pose = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
}

$parent = Split-Path -Parent $OutputConfig
if ($parent) {
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
}
$configuration | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $OutputConfig -Encoding UTF8
Write-Host "ZED360 ag kalibrasyon seed dosyasi yazildi: $OutputConfig"
Write-Host "Bu dosyayi ZED360 icinde LOAD ile yukleyin; kalibrasyondan sonra yeni JSON'u kaydedin."
