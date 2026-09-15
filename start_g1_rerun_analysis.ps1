param(
    [ValidateSet("live", "playback", "demo")]
    [string]$Mode = "live",
    [string]$InputFile = "",
    [string]$Svo2Path = "",
    [string]$ListenHost = "0.0.0.0",
    [int]$ListenPort = 15052,
    [int]$GmrListenPort = 15053,
    [ValidateRange(1, 60)]
    [int]$LiveMaxHz = 10,
    [ValidateRange(1, 60)]
    [int]$GmrLogMaxHz = 10,
    [switch]$DetailedJointEntities,
    [switch]$NoViewer,
    [switch]$Headless,
    [switch]$NoRealtime
)

# Backwards-compatible name retained for existing project notes/shortcuts.
$launcher = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "start_g1_rerun.ps1"
& $launcher `
    -Mode $Mode `
    -InputFile $InputFile `
    -Svo2Path $Svo2Path `
    -ListenHost $ListenHost `
    -ListenPort $ListenPort `
    -GmrListenPort $GmrListenPort `
    -LiveMaxHz $LiveMaxHz `
    -GmrLogMaxHz $GmrLogMaxHz `
    -DetailedJointEntities:$DetailedJointEntities `
    -NoViewer:$NoViewer `
    -Headless:$Headless `
    -NoRealtime:$NoRealtime
exit $LASTEXITCODE
