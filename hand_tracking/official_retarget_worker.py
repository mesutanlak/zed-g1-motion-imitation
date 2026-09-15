"""Persistent official DexPilot worker isolated from MediaPipe's NumPy env."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    # Import after argument parsing so startup errors reach the parent stderr.
    from hand_tracking.retargeting import OfficialDexRetargetingSolver
    solver = OfficialDexRetargetingSolver(args.root)
    print("DEX3_WORKER_READY", flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            q, metrics = solver(
                request["normalized"], request["side"], request["previous"]
            )
            response = {"q": q.tolist(), "metrics": metrics}
        except Exception as exc:
            response = {"error": f"{type(exc).__name__}: {exc}"}
        print("DEX3_RESULT " + json.dumps(response, separators=(",", ":")), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
