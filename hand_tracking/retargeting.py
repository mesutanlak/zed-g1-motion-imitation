from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import sys
import threading
from typing import Callable, Sequence

import numpy as np

from .contracts import DEX3_JOINT_ORDER

DEX3_ORDER = DEX3_JOINT_ORDER


def hand_task_vector(points: Sequence[Sequence[float]]) -> np.ndarray:
    p = np.asarray(points, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError("expected finite 21x3 normalized landmarks")
    pairs = ((0, 4), (0, 8), (0, 12), (4, 8), (4, 12), (2, 4), (5, 8), (9, 12))
    return np.concatenate([p[b] - p[a] for a, b in pairs])


class NormalizedTaskSolver:
    """Deterministic 21-point task solver; replaceable by official Dex FK.

    This fallback maps flexion and pinch task distances, rather than copying
    human joint angles.  ``solver`` injection supports the official Unitree
    dex-retargeting model without fabricating its expected 25-point input.
    """

    def __call__(self, normalized: np.ndarray, side: str, previous: np.ndarray) -> tuple[np.ndarray, dict]:
        p = np.asarray(normalized, dtype=float)
        if p.shape != (21, 3) or not np.isfinite(p).all():
            raise ValueError("expected finite 21x3 normalized landmarks")

        def bend(first: int, centre: int, last: int) -> float:
            incoming = p[first] - p[centre]
            outgoing = p[last] - p[centre]
            denominator = max(float(np.linalg.norm(incoming) * np.linalg.norm(outgoing)), 1.0e-8)
            straight_angle = float(np.arccos(np.clip(np.dot(incoming, outgoing) / denominator, -1.0, 1.0)))
            return float(np.clip(np.pi - straight_angle, 0.0, np.pi))

        def finger_curl(wrist: int, mcp: int, pip: int, dip: int, tip: int) -> float:
            # A closed fist bends at MCP, PIP and DIP.  Endpoint shortening
            # alone barely changes for a clean arc and produced near-zero Dex3
            # targets, so use the actual five-finger joint geometry.
            total = 0.45 * bend(wrist, mcp, pip) + bend(mcp, pip, dip) + bend(pip, dip, tip)
            return float(np.clip(total / 2.75, 0.0, 1.0))

        thumb = float(np.clip((bend(1, 2, 3) + bend(2, 3, 4)) / 2.2, 0.0, 1.0))
        index = finger_curl(0, 5, 6, 7, 8)
        middle = finger_curl(0, 9, 10, 11, 12)
        ring = finger_curl(0, 13, 14, 15, 16)
        pinky = finger_curl(0, 17, 18, 19, 20)
        grouped = float(np.clip(np.median([middle, ring, pinky]), 0.0, 1.0))
        pinch = float(np.clip(1.0 - np.linalg.norm(p[4] - p[8]) / 1.10, 0.0, 1.0))
        opposition = float(np.clip(0.65 * pinch + 0.35 * thumb, 0.0, 1.0))
        if side == "left":
            raw = np.array([
                0.80 * opposition, 0.95 * thumb, 1.60 * thumb,
                -1.42 * grouped, -1.68 * grouped,
                -1.42 * index, -1.68 * index,
            ])
        else:
            raw = np.array([
                0.80 * opposition, 0.72 * thumb, -1.60 * thumb,
                1.42 * grouped, 1.68 * grouped,
                1.42 * index, 1.68 * index,
            ])
        residual = float(np.linalg.norm(hand_task_vector(p)))
        fist_score = float(np.mean([index, middle, ring, pinky]))
        pose = "FIST" if fist_score >= 0.62 else "OPEN" if fist_score <= 0.22 else "PARTIAL"
        return raw, {
            "backend": "normalized_21_angle_task_v2",
            "residual": residual,
            "iterations": 1,
            "pose": pose,
            "fist_score": fist_score,
            "pinch_score": pinch,
            "human_finger_curl": {
                "thumb": thumb, "index": index, "middle": middle,
                "ring": ring, "pinky": pinky,
            },
            "dex3_grouped_curl": grouped,
        }


class OfficialDexRetargetingSolver:
    """Use Unitree xr_teleoperate's pinned Dex3 URDF without DDS imports.

    MediaPipe landmarks are converted directly to the six vectors expected by
    the official DexPilot objective.  No fake 25-point skeleton is created.
    """

    def __init__(
        self,
        xr_teleoperate_root: str | Path,
        python_executable: str | Path | None = None,
    ) -> None:
        root = Path(xr_teleoperate_root).resolve()
        self._process: subprocess.Popen[str] | None = None
        self._process_lock = threading.Lock()
        if python_executable is not None:
            worker = Path(__file__).with_name("official_retarget_worker.py")
            self._process = subprocess.Popen(
                [str(Path(python_executable).resolve()), "-u", str(worker), "--root", str(root)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
                text=True, bufsize=1,
            )
            assert self._process.stdin is not None and self._process.stdout is not None
            ready = ""
            while True:
                line = self._process.stdout.readline()
                if not line:
                    break
                ready = line.strip()
                if ready == "DEX3_WORKER_READY":
                    break
            if ready != "DEX3_WORKER_READY":
                self.close()
                raise RuntimeError(f"Resmi Dex3 worker baslatilamadi: {ready}")
            self.backends = {}
            self.output_indices = {}
            return
        dex_source = root / "teleop" / "robot_control" / "dex-retargeting" / "src"
        config_path = root / "assets" / "unitree_hand" / "unitree_dex3.yml"
        if not dex_source.is_dir() or not config_path.is_file():
            raise FileNotFoundError(
                "xr_teleoperate eksik: dex-retargeting submodule ve assets/unitree_hand gerekli"
            )
        if str(dex_source) not in sys.path:
            sys.path.insert(0, str(dex_source))
        try:
            import yaml
            from dex_retargeting import RetargetingConfig
        except ImportError as exc:
            raise RuntimeError("Resmi Dex3 backend icin xr_teleoperate dependency'leri eksik") from exc
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        RetargetingConfig.set_default_urdf_dir(str(root / "assets"))
        self.backends = {
            side: RetargetingConfig.from_dict(document[side]).build()
            for side in ("left", "right")
        }
        self.output_indices: dict[str, list[int]] = {}
        for side, backend in self.backends.items():
            wanted = [f"{side}_hand_{name}_joint" for name in DEX3_ORDER]
            self.output_indices[side] = [backend.joint_names.index(name) for name in wanted]

    def __call__(self, normalized: np.ndarray, side: str, previous: np.ndarray) -> tuple[np.ndarray, dict]:
        if self._process is not None:
            request = json.dumps({
                "side": side,
                "normalized": np.asarray(normalized, dtype=float).tolist(),
                "previous": np.asarray(previous, dtype=float).tolist(),
            }, separators=(",", ":"))
            with self._process_lock:
                assert self._process.stdin is not None and self._process.stdout is not None
                self._process.stdin.write(request + "\n")
                self._process.stdin.flush()
                while True:
                    line = self._process.stdout.readline()
                    if not line:
                        raise RuntimeError("Resmi Dex3 worker beklenmedik bicimde kapandi")
                    if line.startswith("DEX3_RESULT "):
                        response = json.loads(line[len("DEX3_RESULT "):])
                        if response.get("error"):
                            raise RuntimeError(response["error"])
                        return np.asarray(response["q"], dtype=float), dict(response["metrics"])
        p = np.asarray(normalized, dtype=float)
        # Official 25-point convention uses wrist/thumb/index/middle at
        # 0/4/9/14. Equivalent MediaPipe endpoints are 0/4/8/12.
        pairs = ((8, 4), (12, 4), (12, 8), (0, 4), (0, 8), (0, 12))
        reference_vectors = np.stack([p[b] - p[a] for a, b in pairs])
        started = __import__("time").perf_counter()
        result = np.asarray(self.backends[side].retarget(reference_vectors), dtype=float)
        elapsed_ms = (__import__("time").perf_counter() - started) * 1000.0
        selected = result[self.output_indices[side]]
        return selected, {
            "backend": "unitree_xr_teleoperate_dexpilot_21_vector_adapter",
            "residual": None, "iterations": None, "time_ms": elapsed_ms,
        }

    def close(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()


@dataclass(frozen=True)
class SafetyConfig:
    maximum_velocity_rad_s: float = 2.5
    maximum_acceleration_rad_s2: float = 10.0
    hold_s: float = 0.65
    fist_hold_s: float = 1.5
    fade_s: float = 1.0
    minimum_confidence: float = 0.30
    measurement_deadband_rad: float = 0.07
    measurement_min_alpha: float = 0.06
    measurement_max_alpha: float = 0.82
    measurement_full_scale_rad: float = 0.55
    settle_epsilon_rad: float = 0.002


class Dex3SafetyFilter:
    def __init__(self, minimum: Sequence[float], maximum: Sequence[float], neutral: Sequence[float], config: SafetyConfig) -> None:
        self.minimum = np.asarray(minimum, dtype=float)
        self.maximum = np.asarray(maximum, dtype=float)
        self.neutral = np.asarray(neutral, dtype=float)
        self.config = config
        self.position = self.neutral.copy()
        self.last_desired = self.neutral.copy()
        self.velocity = np.zeros(7)
        self.last_valid_ns: int | None = None
        self.last_update_ns: int | None = None
        self.last_pose = "UNKNOWN"

    def reset(self) -> None:
        self.position = self.neutral.copy()
        self.last_desired = self.neutral.copy()
        self.velocity.fill(0.0)
        self.last_valid_ns = self.last_update_ns = None
        self.last_pose = "UNKNOWN"

    def update(
        self,
        raw: Sequence[float] | None,
        timestamp_ns: int,
        confidence: float,
        pose: str | None = None,
    ) -> tuple[np.ndarray, dict]:
        dt = 1 / 30 if self.last_update_ns is None else float(np.clip((timestamp_ns - self.last_update_ns) / 1e9, 1e-3, 0.2))
        valid = raw is not None and confidence >= self.config.minimum_confidence
        if valid:
            raw_array = np.asarray(raw, dtype=float)
            clipped = np.clip(raw_array, self.minimum, self.maximum)
            measurement_delta = clipped - self.last_desired
            deadband = max(0.0, self.config.measurement_deadband_rad)
            span = max(
                self.config.measurement_full_scale_rad - deadband,
                1.0e-6,
            )
            motion = np.clip((np.abs(measurement_delta) - deadband) / span, 0.0, 1.0)
            alpha_reference = (
                self.config.measurement_min_alpha
                + (self.config.measurement_max_alpha - self.config.measurement_min_alpha)
                * motion
            )
            measurement_dt = (
                0.1
                if self.last_valid_ns is None
                else float(np.clip((timestamp_ns - self.last_valid_ns) / 1e9, 0.02, 0.30))
            )
            alpha = 1.0 - np.power(1.0 - alpha_reference, measurement_dt / 0.1)
            alpha = np.where(np.abs(measurement_delta) <= deadband, 0.0, alpha)
            desired = self.last_desired + alpha * measurement_delta
            self.last_desired = desired.copy()
            joint_limit_clipped_count = int(np.count_nonzero(np.abs(raw_array - clipped) > 1e-8))
            self.last_valid_ns = timestamp_ns
            self.last_pose = str(pose or "UNKNOWN")
            state = "TRACKING"
        else:
            joint_limit_clipped_count = 0
            lost_s = float("inf") if self.last_valid_ns is None else max(0.0, (timestamp_ns - self.last_valid_ns) / 1e9)
            hold_s = (
                self.config.fist_hold_s
                if self.last_pose == "FIST"
                else self.config.hold_s
            )
            if lost_s <= hold_s:
                # Continue the bounded trajectory toward the last measured
                # pose. Holding the current position here used to brake the
                # fingers on every 30 Hz body frame between 10 Hz hand frames,
                # so a fist could never close.
                desired = self.last_desired.copy()
                state = "HOLD"
            else:
                alpha = np.clip(dt / max(self.config.fade_s, 1e-3), 0.0, 1.0)
                desired = self.position * (1.0 - alpha) + self.neutral * alpha
                state = "FADE_TO_NEUTRAL"
        requested_velocity = (desired - self.position) / dt
        max_dv = self.config.maximum_acceleration_rad_s2 * dt
        acceleration_limited_velocity = np.clip(requested_velocity, self.velocity - max_dv, self.velocity + max_dv)
        acceleration_limited_count = int(np.count_nonzero(np.abs(acceleration_limited_velocity - requested_velocity) > 1e-8))
        velocity = np.clip(acceleration_limited_velocity, -self.config.maximum_velocity_rad_s, self.config.maximum_velocity_rad_s)
        velocity_limited_count = int(np.count_nonzero(np.abs(velocity - acceleration_limited_velocity) > 1e-8))
        position = np.clip(self.position + velocity * dt, self.minimum, self.maximum)
        # Stop exactly at the target. A pure acceleration limiter otherwise
        # crosses a static target with non-zero velocity and oscillates forever,
        # which appeared as constant finger motion in Isaac.
        old_error = desired - self.position
        new_error = desired - position
        crossed = (old_error * new_error <= 0.0) & (
            np.abs(old_error) > self.config.settle_epsilon_rad
        )
        settled = np.abs(old_error) <= self.config.settle_epsilon_rad
        stop = crossed | settled
        position = np.where(stop, desired, position)
        velocity = np.where(stop, 0.0, velocity)
        saturated = bool(np.any(np.abs(position - desired) > 1e-8) or joint_limit_clipped_count)
        self.position, self.velocity, self.last_update_ns = position, velocity, timestamp_ns
        return position.copy(), {
            "watchdog": state,
            "saturated": saturated,
            "confidence": float(confidence),
            "joint_limit_clipped_count": joint_limit_clipped_count,
            "velocity_limited_count": velocity_limited_count,
            "acceleration_limited_count": acceleration_limited_count,
            "last_pose": self.last_pose,
        }


class Dex3Retargeter:
    def __init__(self, config_path: str | Path, solver: Callable | None = None) -> None:
        document = json.loads(Path(config_path).read_text(encoding="utf-8"))
        hands = document["hands"]
        if tuple(hands["joint_order_per_hand"]) != DEX3_ORDER:
            raise ValueError("Dex3 joint order does not match the repository contract")
        safety_document = hands.get("safety") or {}
        safety = SafetyConfig(**{k: safety_document[k] for k in SafetyConfig.__dataclass_fields__ if k in safety_document})
        neutral = hands.get("neutral_rad", [0.0] * 7)
        self.filters = {
            side: Dex3SafetyFilter(hands[f"{side}_limits_rad"]["min"], hands[f"{side}_limits_rad"]["max"], neutral, safety)
            for side in ("left", "right")
        }
        self.solver = solver or NormalizedTaskSolver()

    def reset(self) -> None:
        for item in self.filters.values():
            item.reset()

    def update(self, side: str, normalized: np.ndarray | None, timestamp_ns: int, confidence: float) -> dict:
        raw = None
        solver_metrics = {"backend": "none", "residual": None, "iterations": 0}
        if normalized is not None and confidence > 0.0:
            raw, solver_metrics = self.solver(normalized, side, self.filters[side].position.copy())
        safe, safety_metrics = self.filters[side].update(
            raw,
            timestamp_ns,
            confidence,
            str(solver_metrics.get("pose")) if raw is not None else None,
        )
        return {
            "joint_order": list(DEX3_ORDER),
            "raw_q_rad": None if raw is None else np.asarray(raw, dtype=float).tolist(),
            "safe_q_rad": safe.tolist(),
            "solver": solver_metrics,
            "safety": safety_metrics,
        }


def compose_targets(q_body: Sequence[float], q_left: Sequence[float], q_right: Sequence[float]) -> dict:
    body = np.asarray(q_body, dtype=float)
    left = np.asarray(q_left, dtype=float)
    right = np.asarray(q_right, dtype=float)
    if body.shape != (23,) or left.shape != (7,) or right.shape != (7,):
        raise ValueError("target shapes must be body=23, left=7, right=7")
    return {
        "q_body": body.tolist(), "q_left_dex3": left.tolist(),
        "q_right_dex3": right.tolist(),
        "q_target": np.concatenate((body, left, right)).tolist(),
        "physical_robot_output_enabled": False,
    }


def dex3_control_contract(timestamp_ns: int, left: dict, right: dict) -> dict:
    """Build the compact, versioned and explicitly DDS-free hand contract."""
    return {
        "schema": "unitree_g1_dex3_control/v1",
        "timestamp_ns": int(timestamp_ns),
        "joint_order_per_hand": list(DEX3_ORDER),
        "q_left": [float(value) for value in left["safe_q_rad"]],
        "q_right": [float(value) for value in right["safe_q_rad"]],
        "confidence_left": float((left.get("safety") or {}).get("confidence", 0.0)),
        "confidence_right": float((right.get("safety") or {}).get("confidence", 0.0)),
        "watchdog_left": str((left.get("safety") or {}).get("watchdog", "FADE_TO_NEUTRAL")),
        "watchdog_right": str((right.get("safety") or {}).get("watchdog", "FADE_TO_NEUTRAL")),
        "physical_robot_output_enabled": False,
    }
