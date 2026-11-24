"""The two fusion launchers must agree on the coarse hand option."""

from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]
RECEIVER = ROOT / "zed_four_camera_test/start_distributed_receiver_ubuntu.sh"


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    # The launcher uses this executable only after argument validation. Echo
    # exposes the final argv without opening sockets or cameras.
    return subprocess.run(
        [str(RECEIVER), *args],
        cwd=ROOT,
        env={**os.environ, "ZED_PYTHON": "/bin/echo"},
        text=True,
        capture_output=True,
        check=False,
    )


def test_coarse_hand_option_reaches_fusion_process() -> None:
    result = _run("--hand-tracking", "--hand-gesture-only", "--headless")
    assert result.returncode == 0, result.stderr
    assert "distributed_body38_fusion.py" in result.stdout
    assert "--hand-tracking" in result.stdout
    assert "--hand-gesture-only" in result.stdout


def test_coarse_hand_requires_hand_tracking() -> None:
    result = _run("--hand-gesture-only")
    assert result.returncode == 2
    assert "--hand-tracking gerekli" in result.stderr
