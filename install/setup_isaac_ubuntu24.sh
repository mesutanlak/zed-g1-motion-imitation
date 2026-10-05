#!/usr/bin/env bash
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project/install/setup_isaac61_ubuntu24.sh" "$@"
