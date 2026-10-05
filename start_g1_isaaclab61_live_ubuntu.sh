#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
gmr_python="${GMR_PYTHON:-$root/envs/gmr_zed/bin/python}"
isaac_python="${ISAAC_PYTHON:-$root/envs/isaaclab30/bin/python}"

mode="upper_body"
imitation_mode="kinematic_debug"
runtime_profile="shared_gpu_safe"
stance_mode="fixed_double_support"
input_fps="15"
upper_cutoff_hz="10"
upper_min_cutoff_hz="2"
upper_velocity_beta="1.2"
stationary_deadband_scale="1"
human_height="1.80"
mimic_blend="1"
upper_stiffness_scale="2"
upper_damping_scale="2"
reference_tracking_mode="low_latency"
reference_response_hz="5.5"
reference_max_velocity="0.85"
reference_max_acceleration="4"
reference_max_jerk="35"
reference_stationary_deadband_scale="1"
stale_return_delay="0.25"
stale_return_tau="0.60"
render_interval="8"
mirror_workers="4"
max_steps="0"
headless=0
accept_eula=0
no_fall_arrest=0
no_mirror_rescue=0
no_anatomical_branch_continuity=0
restrict_backward_arms=0
asset_profile="g1_23dof"

usage() {
  cat <<'EOF'
Kullanim: ./start_g1_isaaclab61_live_ubuntu.sh --accept-nvidia-eula [secenekler]
  --mode upper_body|whole_body
  --imitation-mode kinematic_debug|dynamic
  --headless
  --runtime-profile shared_gpu_safe|gpu_max
  --stance-mode fixed_double_support|balance_policy
  --input-fps N
  --human-height M
  --asset-profile g1_23dof|g1_29dof_dex3
  --max-steps N
  --no-fall-arrest
  --no-mirror-rescue
  --no-anatomical-branch-continuity
  --restrict-backward-arms
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --accept-nvidia-eula) accept_eula=1; shift ;;
    --mode) mode="${2:?}"; shift 2 ;;
    --imitation-mode) imitation_mode="${2:?}"; shift 2 ;;
    --runtime-profile) runtime_profile="${2:?}"; shift 2 ;;
    --stance-mode) stance_mode="${2:?}"; shift 2 ;;
    --input-fps) input_fps="${2:?}"; shift 2 ;;
    --upper-cutoff-hz) upper_cutoff_hz="${2:?}"; shift 2 ;;
    --upper-min-cutoff-hz) upper_min_cutoff_hz="${2:?}"; shift 2 ;;
    --upper-velocity-beta) upper_velocity_beta="${2:?}"; shift 2 ;;
    --stationary-deadband-scale) stationary_deadband_scale="${2:?}"; shift 2 ;;
    --human-height) human_height="${2:?}"; shift 2 ;;
    --asset-profile) asset_profile="${2:?}"; shift 2 ;;
    --mimic-blend) mimic_blend="${2:?}"; shift 2 ;;
    --upper-stiffness-scale) upper_stiffness_scale="${2:?}"; shift 2 ;;
    --upper-damping-scale) upper_damping_scale="${2:?}"; shift 2 ;;
    --reference-tracking-mode) reference_tracking_mode="${2:?}"; shift 2 ;;
    --reference-response-hz) reference_response_hz="${2:?}"; shift 2 ;;
    --reference-max-velocity) reference_max_velocity="${2:?}"; shift 2 ;;
    --reference-max-acceleration) reference_max_acceleration="${2:?}"; shift 2 ;;
    --reference-max-jerk) reference_max_jerk="${2:?}"; shift 2 ;;
    --reference-stationary-deadband-scale) reference_stationary_deadband_scale="${2:?}"; shift 2 ;;
    --stale-return-delay) stale_return_delay="${2:?}"; shift 2 ;;
    --stale-return-tau) stale_return_tau="${2:?}"; shift 2 ;;
    --render-interval) render_interval="${2:?}"; shift 2 ;;
    --mirror-workers) mirror_workers="${2:?}"; shift 2 ;;
    --max-steps) max_steps="${2:?}"; shift 2 ;;
    --headless) headless=1; shift ;;
    --no-fall-arrest) no_fall_arrest=1; shift ;;
    --no-mirror-rescue) no_mirror_rescue=1; shift ;;
    --no-anatomical-branch-continuity) no_anatomical_branch_continuity=1; shift ;;
    --restrict-backward-arms) restrict_backward_arms=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if (( ! accept_eula )); then
  echo "NVIDIA Omniverse EULA'yi okuyup kabul ediyorsaniz --accept-nvidia-eula ekleyin:" >&2
  echo "https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html" >&2
  exit 2
fi
case "$mode" in upper_body|whole_body) ;; *) echo "Gecersiz mode: $mode" >&2; exit 2 ;; esac
case "$imitation_mode" in dynamic|kinematic_debug) ;; *) echo "Gecersiz imitation mode" >&2; exit 2 ;; esac
case "$runtime_profile" in shared_gpu_safe|gpu_max) ;; *) echo "Gecersiz runtime profile" >&2; exit 2 ;; esac
case "$stance_mode" in fixed_double_support|balance_policy) ;; *) echo "Gecersiz stance mode" >&2; exit 2 ;; esac
case "$asset_profile" in g1_23dof|g1_29dof_dex3) ;; *) echo "Gecersiz asset profile" >&2; exit 2 ;; esac

for executable in "$gmr_python" "$isaac_python"; do
  [[ -x "$executable" ]] || { echo "Python bulunamadi: $executable" >&2; exit 1; }
done

urdf="$root/repos/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf"
usd="$root/cache/g1_23dof/g1_23dof_rev_1_0.usd"
unitree_sim_root="$root/repos/unitree_sim_isaaclab"
balance_policy="$project/policies/g1_23dof_velocity/policy.onnx"
balance_config="$project/policies/g1_23dof_velocity/deploy.yaml"
reference_policy="$project/policies/g1_reference_upper_body/policy.onnx"
reference_metadata="$project/policies/g1_reference_upper_body/policy_metadata.json"

for required in "$urdf" "$balance_policy" "$balance_config"; do
  [[ -f "$required" ]] || { echo "Gerekli dosya bulunamadi: $required" >&2; exit 1; }
done
if [[ "$asset_profile" == "g1_29dof_dex3" ]]; then
  dex3_asset="$unitree_sim_root/assets/robots/g1-29dof-dex3-base-fix-usd/g1_29dof_with_dex3_base_fix.usd"
  [[ -s "$dex3_asset" ]] || { echo "Resmi G1-29 + Dex3 USD bulunamadi: $dex3_asset" >&2; exit 1; }
fi

driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)"
if ss -lun | grep -q ':15050 '; then
  echo "UDP 15050 zaten kullanimda." >&2
  exit 1
fi

bridge_args=(
  "$project/isaaclab_bridge/gmr_live_bridge.py"
  --listen-host 0.0.0.0 --listen-port 15050
  --output-host 127.0.0.1 --output-port 15051
  --telemetry-host 127.0.0.1 --telemetry-port 15053
  --mode "$mode" --input-fps "$input_fps"
  --end-effector-profile "$asset_profile"
  --cutoff-hz "$upper_cutoff_hz" --min-cutoff-hz "$upper_min_cutoff_hz"
  --velocity-beta "$upper_velocity_beta"
  --stationary-deadband-scale "$stationary_deadband_scale"
  --human-height "$human_height" --no-gmr-velocity-limit
)
(( no_mirror_rescue )) && bridge_args+=(--no-mirror-rescue) || bridge_args+=(--mirror-rescue --mirror-workers "$mirror_workers")
(( no_anatomical_branch_continuity )) && bridge_args+=(--no-anatomical-branch-continuity) || bridge_args+=(--anatomical-branch-continuity)
(( restrict_backward_arms )) && bridge_args+=(--restrict-backward-arms) || bridge_args+=(--no-restrict-backward-arms)

bridge_pid=""
cleanup() {
  if [[ -n "$bridge_pid" ]] && kill -0 "$bridge_pid" 2>/dev/null; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

PYTHONUNBUFFERED=1 "$gmr_python" "${bridge_args[@]}" &
bridge_pid=$!
for _ in {1..40}; do
  kill -0 "$bridge_pid" 2>/dev/null || { echo "GMR koprusu erken kapandi." >&2; exit 1; }
  ss -lun | grep -q ':15050 ' && break
  sleep 0.5
done
ss -lun | grep -q ':15050 ' || { echo "GMR UDP 15050 acilmadi." >&2; exit 1; }

kit_settings=(
  --/renderer/activeGpu=0
  --/rtx/post/dlss/execMode=0
  --/renderer/raytracingMotion/enabled=false
  --/renderer/raytracingMotion/enableHydraEngineMasking=false
)
if [[ "$runtime_profile" == "shared_gpu_safe" ]]; then
  if (( ! headless )); then
    kit_settings+=(
      --/app/window/width=1920 --/app/window/height=1080
      --/app/renderer/resolution/width=1920 --/app/renderer/resolution/height=1080
    )
  fi
fi
kit_args="${kit_settings[*]}"
visualizer="kit"
(( headless )) && visualizer="none"

isaac_args=(
  "$project/isaaclab_bridge/isaac_g1_23dof_live.py"
  --device cuda:0 --visualizer "$visualizer"
  --mode "$mode" --imitation-mode "$imitation_mode"
  --listen-host 0.0.0.0 --listen-port 15051
  --telemetry-host 127.0.0.1 --telemetry-port 15053
  --urdf "$urdf" --usd "$usd"
  --asset-profile "$asset_profile" --unitree-sim-root "$unitree_sim_root"
  --balance-policy "$balance_policy" --balance-config "$balance_config"
  --reference-policy "$reference_policy" --reference-policy-metadata "$reference_metadata"
  --stance-mode "$stance_mode" --mimic-blend "$mimic_blend"
  --upper-stiffness-scale "$upper_stiffness_scale"
  --upper-damping-scale "$upper_damping_scale"
  --stale-return-delay "$stale_return_delay" --stale-return-tau "$stale_return_tau"
  --input-fps "$input_fps" --reference-tracking-mode "$reference_tracking_mode"
  --reference-response-hz "$reference_response_hz"
  --reference-max-velocity "$reference_max_velocity"
  --reference-max-acceleration "$reference_max_acceleration"
  --reference-max-jerk "$reference_max_jerk"
  --reference-stationary-deadband-scale "$reference_stationary_deadband_scale"
  --render-interval "$render_interval" --max-steps "$max_steps"
  "--kit_args=$kit_args"
)
(( no_fall_arrest )) && isaac_args+=(--no-fall-arrest)

export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
export G1IL_ROOT="$root"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
export PATH="$CUDA_HOME/bin:$PATH"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
export WARN_ON_TORCH_QUATF_ACCESS="${WARN_ON_TORCH_QUATF_ACCESS:-0}"

echo "G1 yerel Ubuntu zinciri: ZED :15050 -> GMR :15051 -> Isaac, telemetry :15053"
echo "NVIDIA=$driver mode=$mode profile=$runtime_profile asset=$asset_profile headless=$headless"
"$isaac_python" "${isaac_args[@]}"
