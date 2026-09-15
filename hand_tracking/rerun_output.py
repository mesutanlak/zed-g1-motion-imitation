from __future__ import annotations

from typing import Any

import numpy as np


HAND_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)


class RerunHandLogger:
    """Lazy Rerun adapter. Visualization failure cannot affect perception."""

    def __init__(self, application_id: str = "zed_g1_dex3_hand_replay", spawn: bool = True) -> None:
        try:
            import rerun as rr
        except ImportError as exc:
            raise RuntimeError("Rerun icin 'pip install -r requirements-rerun.txt' gerekli") from exc
        self.rr = rr
        rr.init(application_id, spawn=spawn)

    def log(self, packet: dict[str, Any]) -> None:
        rr = self.rr
        timestamp_ns = int(packet.get("timestamp_ns") or packet.get("capture_timestamp_ns") or 0)
        rr.set_time("capture", timestamp=timestamp_ns * 1e-9)
        fused = packet.get("hand_tracking") or packet
        for camera in fused.get("per_camera", []):
            serial = camera.get("camera_serial")
            for hand in camera.get("hands", []):
                side = hand.get("side")
                if hand.get("landmarks_px"):
                    rr.log(f"camera/{serial}/{side}/landmarks2d", rr.Points2D(hand["landmarks_px"], radii=3.0))
                if hand.get("roi_xywh"):
                    x, y, width, height = hand["roi_xywh"]
                    rr.log(f"camera/{serial}/{side}/roi", rr.Boxes2D(mins=[[x, y]], sizes=[[width, height]]))
        for hand in fused.get("hands", []):
            side = hand.get("side")
            points_value = hand.get("landmarks_world_m")
            if not points_value:
                continue
            points = np.asarray(points_value, dtype=float)
            rr.log(f"fusion/{side}/landmarks", rr.Points3D(points, radii=0.006))
            rr.log(f"fusion/{side}/skeleton", rr.LineStrips3D([[points[a], points[b]] for a, b in HAND_EDGES]))
            normalization = hand.get("normalization") or {}
            if normalization:
                origin = np.asarray(normalization["wrist_world_m"], dtype=float)
                rotation = np.asarray(normalization["rotation_world_from_palm"], dtype=float)
                scale = float(normalization["scale_m"])
                colors = ([255, 0, 0], [0, 255, 0], [0, 128, 255])
                for axis, color in enumerate(colors):
                    rr.log(f"fusion/{side}/palm_axis_{axis}", rr.Arrows3D(origins=[origin], vectors=[rotation[:, axis] * scale], colors=[color]))
        for side in ("left", "right"):
            target = (packet.get("dex3_targets") or {}).get(side) or {}
            for index, value in enumerate(target.get("safe_q_rad") or []):
                rr.log(f"dex3/{side}/q_{index}", rr.Scalars(float(value)))
            solver = target.get("solver") or {}
            if solver.get("residual") is not None:
                rr.log(f"dex3/{side}/solver_residual", rr.Scalars(float(solver["residual"])))
