param(
    [ValidatePattern('^\d{1,3}(\.\d{1,3}){3}$')]
    [string]$LaptopIPv4 = "192.168.50.11"
)

$ErrorActionPreference = "Stop"
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Bu betigi ANA PC'de Yonetici olarak acilmis PowerShell icinden calistirin."
}

$rules = @(
    @{
        Name = "ZED-G1 Four BODY38 from Laptop"
        Ports = @(16000, 16006)
    },
    @{
        Name = "ZED-G1 Four Preview from Laptop"
        Ports = @(16100, 16106)
    },
    @{
        Name = "ZED-G1 Four Hand Tracking from Laptop"
        Ports = @(16200, 16206)
    }
)

foreach ($definition in $rules) {
    $existing = Get-NetFirewallRule -DisplayName $definition.Name -ErrorAction SilentlyContinue
    if ($null -eq $existing) {
        New-NetFirewallRule `
            -DisplayName $definition.Name `
            -Direction Inbound `
            -Action Allow `
            -Enabled True `
            -Profile Any `
            -Protocol UDP `
            -LocalPort $definition.Ports `
            -RemoteAddress $LaptopIPv4 | Out-Null
        Write-Host "EKLENDI: $($definition.Name) | UDP $($definition.Ports -join ',') | uzak=$LaptopIPv4"
    }
    else {
        Set-NetFirewallRule -DisplayName $definition.Name -Enabled True -Direction Inbound -Action Allow -Profile Any
        Write-Host "MEVCUT/AKTIF: $($definition.Name)"
    }
}

$norton = @(Get-Service -ErrorAction SilentlyContinue | Where-Object {
    $_.DisplayName -match "Norton" -and $_.Status -eq "Running"
})
if ($norton.Count) {
    Write-Warning "Norton etkin: ayni UDP portlarini Norton Firewall icinde de 192.168.50.11 icin izinli yapin."
}

Write-Host "ANA PC ZED UDP kurallari hazir. Receiver acikken Get-NetUDPEndpoint ile 16000/16006/16100/16106/16200/16206 portlarini gorebilirsiniz."
