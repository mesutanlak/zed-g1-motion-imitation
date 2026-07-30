"""Fast headless regression checks for the independent Rerun analyzer."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    project = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="zed_g1_rerun_test_") as temp:
        output = Path(temp)
        completed = subprocess.run(
            [
                sys.executable, "-m", "rerun_analysis.app", "--demo",
                "--no-viewer", "--headless", "--seconds", "0.25",
                "--output-dir", str(output),
            ],
            cwd=project,
            check=False,
            text=True,
            capture_output=True,
            timeout=45,
        )
        if completed.returncode:
            print(completed.stdout)
            print(completed.stderr, file=sys.stderr)
            return completed.returncode
        manifests = list(output.glob("rerun_body38_*/session_manifest.json"))
        rrd_files = list(output.glob("*.rrd"))
        if len(manifests) != 1 or len(rrd_files) != 1:
            raise AssertionError((manifests, rrd_files))
        manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
        if manifest.get("frame_count", 0) < 2:
            raise AssertionError(manifest)
        session = manifests[0].parent
        for name in ("skeleton_analysis.jsonl", "frames.csv", "joints.csv", "angles.csv"):
            path = session / name
            if not path.exists() or path.stat().st_size < 50:
                raise AssertionError(path)
        print(
            "RERUN_ANALYSIS_OK "
            f"frames={manifest['frame_count']} rrd={rrd_files[0].name}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

