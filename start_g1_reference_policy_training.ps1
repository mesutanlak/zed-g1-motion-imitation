[CmdletBinding(DefaultParameterSetName = "Manifest")]
param(
    [Parameter(Mandatory = $true, ParameterSetName = "Manifest")]
    [string]$ClipsManifest,
    [Parameter(Mandatory = $true, ParameterSetName = "Single")]
    [string]$MotionNpz,
    [ValidateRange(1, 16384)]
    [int]$NumEnvs = 4096,
    [ValidateRange(1, 100000)]
    [int]$MaxIterations = 20000,
    [ValidateRange(50, 5000)]
    [int]$ValidationSteps = 500,
    [ValidateRange(0, 2147483647)]
    [int]$Seed = 42,
    [string]$PolicyOutputDir = "",
    [string]$ResumeCheckpoint = "",
    [string]$InstallRoot = "C:\g1il",
    [switch]$Gui,
    [switch]$ResetIsaacUserConfig,
    [switch]$ValidateOnly,
    [switch]$FreshStart,
    [switch]$AcceptNvidiaEula
)

$ErrorActionPreference = "Stop"
if (-not $AcceptNvidiaEula) {
    throw "Isaac Sim EULA'yi kabul ediyorsaniz -AcceptNvidiaEula ekleyin."
}
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedPolicyOutput = if ([string]::IsNullOrWhiteSpace($PolicyOutputDir)) {
    Join-Path $project "policies\g1_reference_upper_body"
} else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $PolicyOutputDir))
}
$manifestPath = $null
$motions = @()
if ($PSCmdlet.ParameterSetName -eq "Manifest") {
    $manifestPath = (Resolve-Path -LiteralPath $ClipsManifest).Path
    $manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $supportedSchemas = @(
        "g1_reference_motion_clips/v2",
        "g1_reference_motion_multisession/v1"
    )
    if ($supportedSchemas -notcontains [string]$manifest.schema) {
        throw "Unsupported manifest schema. Run prepare_g1_reference_dataset.ps1 again."
    }
    $trainIds = @($manifest.splits.train)
    $validationIds = @($manifest.splits.validation)
    if ($trainIds.Count -lt 1 -or $validationIds.Count -lt 1) {
        throw "Manifest must contain non-empty train and validation splits."
    }
    $overlap = @($trainIds | Where-Object { $validationIds -contains $_ })
    if ($overlap.Count -gt 0) { throw "Train/validation leakage: $($overlap -join ', ')" }
    $allIds = @($trainIds + $validationIds)
    foreach ($clipId in $allIds) {
        $clip = @($manifest.clips | Where-Object { [string]$_.id -eq [string]$clipId })
        if ($clip.Count -ne 1) { throw "Manifest clip id is missing or duplicated: $clipId" }
        $referenceNpz = [string]$clip[0].reference_npz
        if (-not (Test-Path -LiteralPath $referenceNpz -PathType Leaf)) {
            throw @"
Manifest dataset hazirlanmasi yarim kalmis; eksik policy klibi:
  $referenceNpz
Once ayni OutputDir ile prepare_g1_reference_dataset.ps1 komutunu yeniden calistirin.
"@
        }
        $motions += (Resolve-Path -LiteralPath $referenceNpz).Path
    }
    $motion = $motions[0]
} else {
    $motion = (Resolve-Path -LiteralPath $MotionNpz).Path
    $motions = @($motion)
    Write-Warning "Single-clip mode has no held-out validation. Use -ClipsManifest for deployable policy training."
}
$python = Join-Path $InstallRoot "env\Scripts\python.exe"
$train = Join-Path $project "reference_policy\train_reference_policy.py"
$usd = Join-Path $InstallRoot "cache\g1_23dof\g1_23dof_rev_1_0.usd"
$robotXml = Join-Path $InstallRoot "repos\unitree_ros\robots\g1_description\g1_23dof.xml"
foreach ($required in @($python, $train, $usd, $robotXml) + $motions) {
    if (-not (Test-Path -LiteralPath $required)) { throw "Required file not found: $required" }
}

# Fail before starting the expensive Isaac GUI. Unitree RL Lab and the pinned
# Isaac Lab training scripts in this workspace require the 2.3.1 API.
& $python -c "import importlib.util as u, importlib.metadata as m, sys; sys.exit(0 if u.find_spec('rsl_rl') and m.version('rsl-rl-lib') == '2.3.1' else 1)"
if ($LASTEXITCODE -ne 0) {
    throw @"
RSL-RL 2.3.1 is not installed in the Isaac environment.
Install it once with:
  C:\g1il\env\Scripts\python.exe -m pip install rsl-rl-lib==2.3.1
"@
}

foreach ($clipMotion in $motions) {
    & $python (Join-Path $project "tools\validate_reference_motion_npz.py") $clipMotion --robot-xml $robotXml
    if ($LASTEXITCODE -ne 0) {
        throw "Reference motion validation failed; training was not started: $clipMotion"
    }
}

if ($ValidateOnly) {
    Write-Host (
        "POLICY_TRAINING_PREFLIGHT_OK clips=$($motions.Count) " +
        "train=$(@($trainIds).Count) validation=$(@($validationIds).Count)"
    ) -ForegroundColor Green
    exit 0
}

& (Join-Path $project "install_reference_policy_task.ps1") -InstallRoot $InstallRoot
$env:G1_REFERENCE_MOTION = $motion
if ($manifestPath) {
    $env:G1_REFERENCE_MANIFEST = $manifestPath
} else {
    Remove-Item Env:G1_REFERENCE_MANIFEST -ErrorAction SilentlyContinue
}
$env:G1_23DOF_USD = $usd
$env:G1_ISAAC_INSTALL_ROOT = $InstallRoot
$env:G1_PROJECT_ROOT = $project
$env:OMNI_KIT_ACCEPT_EULA = "YES"
$env:PRIVACY_CONSENT = "Y"

$kitSettings = @(
    "--/app/vulkan=false",
    "--/renderer/activeGpu=0",
    "--/rtx/post/dlss/execMode=0",
    "--/renderer/raytracingMotion/enabled=false",
    "--/renderer/raytracingMotion/enableHydraEngineMasking=false"
)
if ($Gui) {
    # Keep the training viewport away from the dual-monitor 4K allocation that
    # previously made RTX startup less reliable on this host.
    $kitSettings += @(
        "--/app/window/width=1920",
        "--/app/window/height=1080",
        "--/app/renderer/resolution/width=1920",
        "--/app/renderer/resolution/height=1080"
    )
}
if ($ResetIsaacUserConfig) {
    $kitSettings += "--reset-user"
}
$kitSettings = $kitSettings -join " "

$arguments = @(
    $train,
    "--num_envs", $NumEnvs,
    "--max_iterations", $MaxIterations,
    "--validation-steps", $ValidationSteps,
    "--policy-output-dir", $resolvedPolicyOutput,
    "--seed", $Seed,
    "--device", "cuda:0",
    "--rendering_mode", "performance",
    "--kit_args=$kitSettings"
)
$resolvedResume = $null
if (-not [string]::IsNullOrWhiteSpace($ResumeCheckpoint)) {
    $resolvedResume = (Resolve-Path -LiteralPath $ResumeCheckpoint).Path
} elseif (-not $FreshStart) {
    $stableCheckpoint = Join-Path $resolvedPolicyOutput "policy_checkpoint.pt"
    if (Test-Path -LiteralPath $stableCheckpoint -PathType Leaf) {
        $resolvedResume = $stableCheckpoint
    } else {
        # Backward-compatible migration: older deployments did not copy the
        # accepted checkpoint next to policy.onnx. Recover the last checkpoint
        # from the training_run recorded in metadata once.
        $oldMetadata = Join-Path $resolvedPolicyOutput "policy_metadata.json"
        if (Test-Path -LiteralPath $oldMetadata -PathType Leaf) {
            $old = Get-Content -LiteralPath $oldMetadata -Raw -Encoding UTF8 | ConvertFrom-Json
            if ($old.training_run -and (Test-Path -LiteralPath ([string]$old.training_run))) {
                $candidate = Get-ChildItem -LiteralPath ([string]$old.training_run) -Filter "model_*.pt" -File |
                    Sort-Object { [int]($_.BaseName -replace '^model_', '') } |
                    Select-Object -Last 1
                if ($candidate) { $resolvedResume = $candidate.FullName }
            }
        }
    }
}
if ($resolvedResume) {
    $arguments += @("--resume-checkpoint", $resolvedResume)
    Write-Host "Continual learning: onceki checkpoint korunuyor ve yeni dataset ile devam ediliyor." -ForegroundColor Green
    Write-Host "Resume: $resolvedResume" -ForegroundColor DarkGreen
} elseif ($FreshStart) {
    Write-Warning "FreshStart secildi: onceki policy bilgisi yuklenmeyecek. Mevcut canli policy validation gecmeden silinmez."
} else {
    Write-Host "Ilk policy egitimi: resume checkpoint bulunamadi." -ForegroundColor Yellow
}
if (-not $Gui) { $arguments += "--headless" }
if ($Gui) {
    Write-Host "Isaac Lab GUI: D3D12 | ortam=$NumEnvs | 1920x1080" -ForegroundColor Cyan
}
Write-Host "Egitim hedefi: sabit alt beden + BODY_38/IK ust-govde residual policy" -ForegroundColor Cyan
if ($manifestPath) {
    Write-Host "Multi-clip: train=$($trainIds.Count) | held-out validation=$($validationIds.Count)" -ForegroundColor Cyan
    Write-Host "Manifest: $manifestPath" -ForegroundColor DarkCyan
}
Write-Host "Canli policy cikisi: $resolvedPolicyOutput\policy.onnx" -ForegroundColor Cyan
& $python @arguments
exit $LASTEXITCODE
