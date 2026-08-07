$ErrorActionPreference = "Stop"
Write-Host "Bu dosya artik tum kopruleri de baslatir."
& (Join-Path $PSScriptRoot "start_g1_full_rviz.ps1")
