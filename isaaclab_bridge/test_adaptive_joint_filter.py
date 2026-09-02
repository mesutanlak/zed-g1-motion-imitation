"""Deterministic checks for the live adaptive joint-target filter."""

from __future__ import annotations

import numpy as np

from gmr_live_bridge import AdaptiveJointFilter, calibration_ready_for_control


def test_distributed_fusion_accepts_one_ready_operator_profile() -> None:
    frame = {
        "schema": "zed_body38_live/v1",
        "source_host_id": "fusion_receiver",
        "fusion": {
            "implementation": "application_level_weighted_body38/v1",
        },
        "calibration": {
            "state": "ACQUIRING",
            "profile": {"standing_height_m": 1.8},
            "ready_source_serials": [39504762],
        },
    }
    assert calibration_ready_for_control(frame)


def test_nonfusion_acquiring_profile_remains_blocked() -> None:
    frame = {
        "schema": "zed_body38_live/v1",
        "source_host_id": "Laptop",
        "calibration": {
            "state": "ACQUIRING",
            "profile": {"standing_height_m": 1.8},
            "ready_source_serials": [39504762],
        },
    }
    assert not calibration_ready_for_control(frame)


def test_stationary_deadband_holds_small_joint_target_noise() -> None:
    filter_ = AdaptiveJointFilter(
        min_cutoff_hz=2.0,
        max_cutoff_hz=6.75,
        velocity_beta=1.2,
        derivative_cutoff_hz=1.0,
        deadband_rad=0.025,
    )
    dt = 1.0 / 15.0
    velocity_limit = np.full(23, 8.0)
    filter_.update(np.zeros(23), dt, velocity_limit)
    for value in (0.010, -0.018, 0.024, -0.006):
        output = filter_.update(np.full(23, value), dt, velocity_limit)
        assert np.allclose(output, 0.0)

    moved = filter_.update(np.full(23, 0.20), dt, velocity_limit)
    assert float(np.min(moved)) > 0.10


def main() -> int:
    dt = 1.0 / 15.0
    velocity_limit = np.full(23, 8.0)
    filter_ = AdaptiveJointFilter(
        min_cutoff_hz=2.0,
        max_cutoff_hz=6.75,
        velocity_beta=1.2,
        derivative_cutoff_hz=1.0,
    )
    rng = np.random.default_rng(42)
    raw_noise = rng.normal(0.0, 0.03, size=(120, 23))
    filtered_noise = np.stack(
        [filter_.update(row, dt, velocity_limit) for row in raw_noise]
    )
    raw_rms = float(np.sqrt(np.mean(raw_noise[20:] ** 2)))
    filtered_rms = float(np.sqrt(np.mean(filtered_noise[20:] ** 2)))
    if not filtered_rms < 0.80 * raw_rms:
        raise AssertionError((raw_rms, filtered_rms))

    filter_.reset()
    filter_.update(np.zeros(23), dt, velocity_limit)
    first = filter_.update(np.ones(23), dt, velocity_limit)
    second = filter_.update(np.ones(23), dt, velocity_limit)
    if not (float(first.min()) >= 0.50 and float(second.min()) >= 0.90):
        raise AssertionError((first.min(), second.min()))
    print(
        "ADAPTIVE_FILTER_OK "
        f"noise_rms={raw_rms:.4f}->{filtered_rms:.4f} "
        f"step={first.min():.3f}->{second.min():.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
