"""Reference motion loader with explicit name mapping and confidence metadata."""

from __future__ import annotations

import json
import numpy as np
import torch
from dataclasses import MISSING
from pathlib import Path

from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_inv, quat_mul, yaw_quat
from unitree_rl_lab.tasks.mimic.mdp.commands import MotionCommand, MotionCommandCfg


def _string_list(values: np.ndarray, key: str) -> list[str]:
    result = []
    for value in np.asarray(values).reshape(-1):
        if isinstance(value, bytes):
            result.append(value.decode("utf-8"))
        else:
            result.append(str(value))
    if not result:
        raise ValueError(f"{key} is empty")
    return result


def _index_map(stored: list[str], requested: list[str], key: str) -> list[int]:
    duplicates = {name for name in stored if stored.count(name) > 1}
    if duplicates:
        raise ValueError(f"duplicate {key}: {sorted(duplicates)}")
    missing = [name for name in requested if name not in stored]
    if missing:
        raise ValueError(f"motion is missing {key}: {missing}")
    return [stored.index(name) for name in requested]


class ReferenceMotionLoader:
    """Loads the extended NPZ produced by ``isaac_csv_to_npz_23dof.py``.

    Stored joint/body arrays are reordered by name. This prevents a silent 23-DOF
    mapping error when USD, MuJoCo and SDK joint orders differ.
    """

    def __init__(
        self,
        motion_file: str,
        joint_names: list[str],
        body_names: list[str],
        device: str,
    ):
        data = np.load(motion_file, allow_pickle=False)
        required = {
            "fps",
            "joint_pos",
            "joint_vel",
            "body_pos_w",
            "body_quat_w",
            "body_lin_vel_w",
            "body_ang_vel_w",
            "joint_names",
            "body_names",
        }
        missing = sorted(required.difference(data.files))
        if missing:
            raise ValueError(f"reference motion is missing NPZ keys: {missing}")

        stored_joints = _string_list(data["joint_names"], "joint_names")
        stored_bodies = _string_list(data["body_names"], "body_names")
        joint_ids = _index_map(stored_joints, joint_names, "joint names")
        body_ids = _index_map(stored_bodies, body_names, "body names")

        self.fps = float(np.asarray(data["fps"]).reshape(-1)[0])
        self.joint_pos = torch.as_tensor(data["joint_pos"][:, joint_ids], dtype=torch.float32, device=device)
        self.joint_vel = torch.as_tensor(data["joint_vel"][:, joint_ids], dtype=torch.float32, device=device)
        self._body_pos_w = torch.as_tensor(data["body_pos_w"][:, body_ids], dtype=torch.float32, device=device)
        self._body_quat_w = torch.as_tensor(data["body_quat_w"][:, body_ids], dtype=torch.float32, device=device)
        self._body_lin_vel_w = torch.as_tensor(
            data["body_lin_vel_w"][:, body_ids], dtype=torch.float32, device=device
        )
        self._body_ang_vel_w = torch.as_tensor(
            data["body_ang_vel_w"][:, body_ids], dtype=torch.float32, device=device
        )

        frame_count = self.joint_pos.shape[0]
        confidence = data["reference_confidence"] if "reference_confidence" in data.files else np.ones(frame_count)
        phase = data["phase"] if "phase" in data.files else np.linspace(0.0, 1.0, frame_count, dtype=np.float32)
        foot_contact = data["foot_contact"] if "foot_contact" in data.files else np.ones((frame_count, 2))
        self.reference_confidence = torch.as_tensor(confidence, dtype=torch.float32, device=device).reshape(-1).clamp(0, 1)
        self.phase = torch.as_tensor(phase, dtype=torch.float32, device=device).reshape(-1).clamp(0, 1)
        self.foot_contact = torch.as_tensor(foot_contact, dtype=torch.float32, device=device).reshape(frame_count, 2)
        self.time_step_total = frame_count

        tensors = (
            self.joint_pos,
            self.joint_vel,
            self._body_pos_w,
            self._body_quat_w,
            self._body_lin_vel_w,
            self._body_ang_vel_w,
            self.reference_confidence,
        )
        if frame_count < 3 or any(not torch.isfinite(value).all() for value in tensors):
            raise ValueError("reference motion must contain at least three finite frames")

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self._body_pos_w

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self._body_quat_w

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        return self._body_lin_vel_w

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        return self._body_ang_vel_w


class ReferenceMotionCollection:
    """Padded, name-mapped motion clips addressed by ``(clip, frame)``.

    Clips remain independent.  Padding is storage-only and is never sampled;
    each environment carries its own clip id and valid frame counter.
    """

    def __init__(
        self,
        manifest_file: str,
        joint_names: list[str],
        body_names: list[str],
        device: str,
    ) -> None:
        manifest_path = Path(manifest_file).expanduser().resolve()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        supported_schemas = {
            "g1_reference_motion_clips/v2",
            "g1_reference_motion_multisession/v1",
        }
        if manifest.get("schema") not in supported_schemas:
            raise ValueError(
                "multi-clip training requires a supported clips/multisession manifest; "
                "run prepare_g1_reference_dataset.ps1 again"
            )
        clips = manifest.get("clips")
        splits = manifest.get("splits")
        if not isinstance(clips, list) or not clips or not isinstance(splits, dict):
            raise ValueError("reference-motion manifest has no clips/splits")
        by_id = {str(clip.get("id")): clip for clip in clips}
        if len(by_id) != len(clips):
            raise ValueError("reference-motion manifest contains duplicate clip ids")
        for split in ("train", "validation"):
            ids = splits.get(split)
            if not isinstance(ids, list) or not ids:
                raise ValueError(f"reference-motion manifest split is empty: {split}")
            missing = [clip_id for clip_id in ids if clip_id not in by_id]
            if missing:
                raise ValueError(f"manifest {split} split refers to unknown clips: {missing}")
        overlap = set(splits["train"]).intersection(splits["validation"])
        if overlap:
            raise ValueError(f"train/validation leakage detected: {sorted(overlap)}")

        ordered_ids = list(splits["train"]) + list(splits["validation"])
        # Catalog entries outside both splits are quarantined safety negatives.
        # Keep their metadata on disk for diagnostics, but never load or sample
        # their unsafe q_ref trajectories in the imitation environment.
        loaders: list[ReferenceMotionLoader] = []
        paths: list[str] = []
        for clip_id in ordered_ids:
            raw_path = Path(str(by_id[clip_id].get("reference_npz", "")))
            path = raw_path if raw_path.is_absolute() else manifest_path.parent / raw_path
            if not path.is_file():
                raise FileNotFoundError(f"reference clip not found: {path}")
            loaders.append(ReferenceMotionLoader(str(path), joint_names, body_names, device))
            paths.append(str(path.resolve()))
        fps_values = {round(loader.fps, 6) for loader in loaders}
        if len(fps_values) != 1:
            raise ValueError(f"all reference clips must have the same fps: {sorted(fps_values)}")

        self.manifest_path = str(manifest_path)
        self.clip_ids = ordered_ids
        self.clip_paths = paths
        self.fps = loaders[0].fps
        self.lengths = torch.as_tensor(
            [loader.time_step_total for loader in loaders], dtype=torch.long, device=device
        )
        self.time_step_total = int(self.lengths.max().item())
        self.split_indices = {
            split: torch.as_tensor(
                [ordered_ids.index(clip_id) for clip_id in splits[split]],
                dtype=torch.long,
                device=device,
            )
            for split in ("train", "validation")
        }

        def pad(name: str) -> torch.Tensor:
            sources = [getattr(loader, name) for loader in loaders]
            output = torch.zeros(
                (len(sources), self.time_step_total, *sources[0].shape[1:]),
                dtype=sources[0].dtype,
                device=device,
            )
            for clip_index, source in enumerate(sources):
                output[clip_index, : source.shape[0]] = source
            return output

        self.joint_pos = pad("joint_pos")
        self.joint_vel = pad("joint_vel")
        self._body_pos_w = pad("body_pos_w")
        self._body_quat_w = pad("body_quat_w")
        self._body_lin_vel_w = pad("body_lin_vel_w")
        self._body_ang_vel_w = pad("body_ang_vel_w")
        self.reference_confidence = pad("reference_confidence")
        self.phase = pad("phase")
        self.foot_contact = pad("foot_contact")

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self._body_pos_w

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self._body_quat_w

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        return self._body_lin_vel_w

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        return self._body_ang_vel_w


class ReferenceMotionCommand(MotionCommand):
    """Unitree motion command augmented with phase and confidence."""

    cfg: "ReferenceMotionCommandCfg"

    def __init__(self, cfg: "ReferenceMotionCommandCfg", env):
        # The parent initializes sampling, metrics and debug visualization.
        super().__init__(cfg, env)
        if cfg.motion_manifest:
            self.motion = ReferenceMotionCollection(
                cfg.motion_manifest,
                list(self.robot.joint_names),
                list(cfg.body_names),
                device=self.device,
            )
            self.clip_indices = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
            self.active_split = cfg.active_split
            if self.active_split not in self.motion.split_indices:
                raise ValueError(f"unknown reference-motion split: {self.active_split}")
            # Parent adaptive bins describe a single linear clip.  Multi-clip
            # sampling is categorical and balanced, so expose clip statistics
            # through the same metric slots without concatenating timelines.
            self.bin_count = len(self.motion.clip_ids)
            self.bin_failed_count = torch.zeros(self.bin_count, dtype=torch.float, device=self.device)
            self._current_bin_failed = torch.zeros_like(self.bin_failed_count)
        else:
            self.motion = ReferenceMotionLoader(
                cfg.motion_file,
                list(self.robot.joint_names),
                list(cfg.body_names),
                device=self.device,
            )
            self.clip_indices = None
        policy_hz = 1.0 / (env.cfg.decimation * env.cfg.sim.dt)
        if abs(self.motion.fps - policy_hz) > 1.0e-3:
            raise ValueError(
                f"motion fps ({self.motion.fps:g}) must equal policy rate ({policy_hz:g} Hz); "
                "convert the clip again instead of skipping/duplicating frames at training time"
            )

    def _sample(self, values: torch.Tensor) -> torch.Tensor:
        if self.clip_indices is None:
            return values[self.time_steps]
        return values[self.clip_indices, self.time_steps]

    @property
    def command(self) -> torch.Tensor:
        return torch.cat([self.joint_pos, self.joint_vel], dim=1)

    @property
    def joint_pos(self) -> torch.Tensor:
        return self._sample(self.motion.joint_pos)

    @property
    def joint_vel(self) -> torch.Tensor:
        return self._sample(self.motion.joint_vel)

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_pos_w) + self._env.scene.env_origins[:, None, :]

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_quat_w)

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_lin_vel_w)

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_ang_vel_w)

    @property
    def anchor_pos_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_pos_w)[:, self.motion_anchor_body_index] + self._env.scene.env_origins

    @property
    def anchor_quat_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_quat_w)[:, self.motion_anchor_body_index]

    @property
    def anchor_lin_vel_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_lin_vel_w)[:, self.motion_anchor_body_index]

    @property
    def anchor_ang_vel_w(self) -> torch.Tensor:
        return self._sample(self.motion.body_ang_vel_w)[:, self.motion_anchor_body_index]

    def set_active_split(self, split: str) -> None:
        if self.clip_indices is None:
            raise RuntimeError("a single motion file has no dataset splits")
        if split not in self.motion.split_indices:
            raise ValueError(f"unknown reference-motion split: {split}")
        self.active_split = split

    def _adaptive_sampling(self, env_ids):
        if self.clip_indices is None:
            return super()._adaptive_sampling(env_ids)
        count = len(env_ids)
        if count == 0:
            return
        candidates = self.motion.split_indices[self.active_split]
        selected = candidates[torch.randint(len(candidates), (count,), device=self.device)]
        self.clip_indices[env_ids] = selected
        lengths = self.motion.lengths[selected]
        self.time_steps[env_ids] = torch.floor(
            torch.rand(count, device=self.device) * (lengths - 1).clamp(min=1)
        ).long()
        probability = 1.0 / float(len(candidates))
        entropy = 0.0 if len(candidates) == 1 else 1.0
        self.metrics["sampling_entropy"][env_ids] = entropy
        self.metrics["sampling_top1_prob"][env_ids] = probability
        self.metrics["sampling_top1_bin"][env_ids] = selected.float() / max(len(self.motion.clip_ids) - 1, 1)

    def _update_command(self):
        if self.clip_indices is None:
            return super()._update_command()
        self.time_steps += 1
        clip_lengths = self.motion.lengths[self.clip_indices]
        env_ids = torch.where(self.time_steps >= clip_lengths)[0]
        self._resample_command(env_ids)

        anchor_pos_w_repeat = self.anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        anchor_quat_w_repeat = self.anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_pos_w_repeat = self.robot_anchor_pos_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)
        robot_anchor_quat_w_repeat = self.robot_anchor_quat_w[:, None, :].repeat(1, len(self.cfg.body_names), 1)

        delta_pos_w = robot_anchor_pos_w_repeat
        delta_pos_w[..., 2] = anchor_pos_w_repeat[..., 2]
        delta_ori_w = yaw_quat(quat_mul(robot_anchor_quat_w_repeat, quat_inv(anchor_quat_w_repeat)))
        self.body_quat_relative_w = quat_mul(delta_ori_w, self.body_quat_w)
        self.body_pos_relative_w = delta_pos_w + quat_apply(delta_ori_w, self.body_pos_w - anchor_pos_w_repeat)

    @property
    def reference_confidence(self) -> torch.Tensor:
        return self._sample(self.motion.reference_confidence)

    @property
    def phase(self) -> torch.Tensor:
        return self._sample(self.motion.phase)

    @property
    def reference_foot_contact(self) -> torch.Tensor:
        return self._sample(self.motion.foot_contact)


@configclass
class ReferenceMotionCommandCfg(MotionCommandCfg):
    class_type: type = ReferenceMotionCommand
    motion_file: str = MISSING
    motion_manifest: str | None = None
    active_split: str = "train"
