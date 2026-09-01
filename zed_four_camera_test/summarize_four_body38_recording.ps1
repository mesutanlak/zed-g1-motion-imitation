param(
    [string]$InputPath = ""
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root ".venv-zed\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "ZED Python ortami bulunamadi: $python"
}
if (-not $InputPath) {
    $latest = Get-ChildItem -LiteralPath (Join-Path $root "recordings") -File -Filter "four_body38_fusion_*.jsonl" |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $latest) {
        throw "recordings altinda four_body38_fusion_*.jsonl bulunamadi."
    }
    $InputPath = $latest.FullName
}
if (-not (Test-Path -LiteralPath $InputPath -PathType Leaf)) {
    throw "Fusion kaydi bulunamadi: $InputPath"
}
& $python (Join-Path $PSScriptRoot "summarize_four_body38_recording.py") --input $InputPath
exit $LASTEXITCODE
