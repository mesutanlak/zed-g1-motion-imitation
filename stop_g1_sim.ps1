$ErrorActionPreference = "Stop"

Write-Host "WSL icindeki proje G1 simulasyon surecleri kapatiliyor..."
$command = @'
patterns=(
  "g1_zed_live_mimic.py"
  "unitree_mujoco.py"
  "g1_ctrl --network=lo"
)
for pattern in "${patterns[@]}"; do
  while read -r pid; do
    [[ -z "$pid" || "$pid" == "$$" ]] && continue
    echo "Durduruluyor: PID $pid ($pattern)"
    kill -INT "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" || true)
done
'@
& wsl.exe -d Ubuntu-22.04 -- bash -lc $command
if ($LASTEXITCODE -ne 0) {
    throw "WSL simulasyon durdurma komutu basarisiz: $LASTEXITCODE"
}
