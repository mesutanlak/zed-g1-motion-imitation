param(
    [Parameter(Mandatory = $true)]
    [string]$SourceConfig,
    [Parameter(Mandatory = $true)]
    [int[]]$LocalSerial,
    [Parameter(Mandatory = $true)]
    [string[]]$RemoteCamera,
    [Parameter(Mandatory = $true)]
    [string]$OutputConfig
)

$ErrorActionPreference = "Stop"
if (-not (Test-Path -LiteralPath $SourceConfig)) {
    throw "Kaynak ZED360 JSON bulunamadi: $SourceConfig"
}
if ($LocalSerial.Count -ne 2 -or $RemoteCamera.Count -ne 2) {
    throw "Iki -LocalSerial ve iki -RemoteCamera SERIAL@IP:PORT degeri verin."
}

$remoteEndpoints = @{}
foreach ($item in $RemoteCamera) {
    try {
        $serialText, $endpoint = $item -split "@", 2
        $ip, $portText = $endpoint -split ":", 2
        $serial = [int]$serialText
        $port = [int]$portText
    }
    catch {
        throw "Gecersiz uzak kamera bicimi '$item'. SERIAL@IP:PORT kullanin."
    }
    if ($remoteEndpoints.ContainsKey($serial) -or [string]::IsNullOrWhiteSpace($ip) -or $port -lt 1 -or $port -gt 65535 -or ($port % 2 -ne 0)) {
        throw "Gecersiz veya yinelenen uzak kamera endpoint'i: $item"
    }
    $remoteEndpoints[$serial] = @{ Ip = $ip; Port = $port }
}

if (($LocalSerial | Sort-Object -Unique).Count -ne 2 -or ($LocalSerial | Where-Object { $remoteEndpoints.ContainsKey($_) }).Count -ne 0) {
    throw "Yerel ve uzak kamera seri numaralari benzersiz olmali."
}

try {
    $configuration = Get-Content -LiteralPath $SourceConfig -Raw | ConvertFrom-Json
}
catch {
    throw "Kaynak JSON okunamadi: $($_.Exception.Message)"
}

$sourceSerials = @($configuration.PSObject.Properties | ForEach-Object { [int]$_.Value.FusionConfiguration.serial_number })
$requestedSerials = @($LocalSerial + $remoteEndpoints.Keys)
if (($sourceSerials | Sort-Object -Unique).Count -ne 4 -or ($sourceSerials | Sort-Object -Unique | Compare-Object ($requestedSerials | Sort-Object))) {
    throw "Kaynak JSON tam olarak bu dort seri numarasini icermeli. JSON: $($sourceSerials -join ', ')"
}

foreach ($property in $configuration.PSObject.Properties) {
    $fusion = $property.Value.FusionConfiguration
    $serial = [int]$fusion.serial_number
    $communication = $fusion.communication_parameters.CommunicationParameters
    if ($remoteEndpoints.ContainsKey($serial)) {
        $endpoint = $remoteEndpoints[$serial]
        $communication.communication_type = "LOCAL NETWORK"
        $communication.ip_add = $endpoint.Ip
        $communication.ip_port = $endpoint.Port
    }
    else {
        # ZED360 opens these cameras itself through the main PC's USB ports.
        $communication.communication_type = "INTRA PROCESS"
        $communication.ip_add = ""
        $communication.ip_port = 0
    }
    # The prior table/rig pose cannot be used for new tripod calibration.
    $fusion.pose = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
}

$parent = Split-Path -Parent $OutputConfig
if ($parent) {
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
}
$configuration | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $OutputConfig -Encoding UTF8
Write-Host "ZED360 hibrit kalibrasyon dosyasi yazildi: $OutputConfig"
Write-Host "ZED360 icinde LOAD ile acin. Yerel USB kameralar ZED360 tarafindan; uzak kameralar laptop publisher tarafindan acilir."
