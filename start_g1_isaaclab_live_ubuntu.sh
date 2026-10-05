#!/usr/bin/env bash
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$project/start_g1_isaaclab61_live_ubuntu.sh" "$@"
