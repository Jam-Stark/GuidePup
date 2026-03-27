from __future__ import annotations

import math
import json
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    import torch
except ImportError:
    torch = None


DEFAULT_COMMAND_SCALES = (
    2.0,
    2.0,
    0.25,
    0.5,
    1.0,
    1.3,
    0.5,
    0.5,
    0.5,
    0.01,
    0.01,
    0.01,
    0.01,
    0.01,
    0.01,
)


def _default_joint_names() -> tuple[str, ...]:
    return (
        "FL_hip_joint",
        "FR_hip_joint",
        "RL_hip_joint",
        "RR_hip_joint",
        "FL_thigh_joint",
        "FR_thigh_joint",
        "RL_thigh_joint",
        "RR_thigh_joint",
        "FL_calf_joint",
        "FR_calf_joint",
        "RL_calf_joint",
        "RR_calf_joint",
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
    )


def _default_obs_joint_names() -> tuple[str, ...]:
    return (
        "FL_hip_joint",
        "FR_hip_joint",
        "RL_hip_joint",
        "RR_hip_joint",
        "FL_thigh_joint",
        "FR_thigh_joint",
        "RL_thigh_joint",
        "RR_thigh_joint",
        "arm_joint1",
        "FL_calf_joint",
        "FR_calf_joint",
        "RL_calf_joint",
        "RR_calf_joint",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
    )


def _default_joint_positions() -> tuple[float, ...]:
    return (
        0.1,
        -0.1,
        0.1,
        -0.1,
        0.5,
        0.5,
        0.5,
        0.5,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        0.0,
        1.48,
        -0.63,
        -0.84,
        0.0,
    )


def _default_obs_joint_positions() -> tuple[float, ...]:
    return (
        0.1,
        -0.1,
        0.1,
        -0.1,
        0.5,
        0.5,
        0.5,
        0.5,
        0.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        1.48,
        -0.63,
        -0.84,
        0.0,
    )


def _default_auxiliary_joint_defaults() -> dict[str, float]:
    return {"arm_joint6": 1.57}


def _default_joint_actuator_gains() -> dict[str, tuple[float, float]]:
    return {}


@dataclass(frozen=True)
class UniFPPolicyMetadata:
    joint_names: tuple[str, ...] = field(default_factory=_default_joint_names)
    default_joint_pos: tuple[float, ...] = field(default_factory=_default_joint_positions)
    obs_joint_names: tuple[str, ...] = field(default_factory=_default_obs_joint_names)
    obs_default_joint_pos: tuple[float, ...] = field(default_factory=_default_obs_joint_positions)
    auxiliary_joint_defaults: dict[str, float] = field(default_factory=_default_auxiliary_joint_defaults)
    action_scale: float = 0.25
    action_dim: int = 17
    leg_action_dim: int = 12
    per_frame_obs_dim: int = 73
    history_length: int = 32
    sim_dt: float = 1.0 / 200.0
    control_decimation: int = 4
    gait_frequency_hz: float = 2.0
    joint_vel_scale: float = 0.05
    base_ang_vel_scale: float = 0.25
    command_scales: tuple[float, ...] = DEFAULT_COMMAND_SCALES
    default_locomotion_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    default_ee_sphere_cmd: tuple[float, float, float] = (0.55, 0.0, 0.0)
    default_gripper_force_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    default_base_force_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    freeze_arm_actions: bool = False
    joint_actuator_gains: dict[str, tuple[float, float]] = field(default_factory=_default_joint_actuator_gains)

    @property
    def control_dt(self) -> float:
        return self.sim_dt * self.control_decimation

    @property
    def flattened_obs_dim(self) -> int:
        return self.per_frame_obs_dim * self.history_length

    @property
    def proprio_dim(self) -> int:
        return 2 + 3 + len(self.obs_joint_names) + len(self.obs_joint_names)

    def to_json_dict(self) -> dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "default_joint_pos": list(self.default_joint_pos),
            "obs_joint_names": list(self.obs_joint_names),
            "obs_default_joint_pos": list(self.obs_default_joint_pos),
            "auxiliary_joint_defaults": dict(self.auxiliary_joint_defaults),
            "action_scale": self.action_scale,
            "action_dim": self.action_dim,
            "leg_action_dim": self.leg_action_dim,
            "per_frame_obs_dim": self.per_frame_obs_dim,
            "history_length": self.history_length,
            "sim_dt": self.sim_dt,
            "control_decimation": self.control_decimation,
            "gait_frequency_hz": self.gait_frequency_hz,
            "joint_vel_scale": self.joint_vel_scale,
            "base_ang_vel_scale": self.base_ang_vel_scale,
            "command_scales": list(self.command_scales),
            "default_locomotion_cmd": list(self.default_locomotion_cmd),
            "default_ee_sphere_cmd": list(self.default_ee_sphere_cmd),
            "default_gripper_force_cmd": list(self.default_gripper_force_cmd),
            "default_base_force_cmd": list(self.default_base_force_cmd),
            "freeze_arm_actions": self.freeze_arm_actions,
            "joint_actuator_gains": {
                joint_name: [kp, kv] for joint_name, (kp, kv) in self.joint_actuator_gains.items()
            },
        }

    def write_json(self, path: Path) -> None:
        path.write_text(json.dumps(self.to_json_dict(), indent=2, sort_keys=True) + "\n")

    @classmethod
    def from_dict(
        cls,
        raw: dict[str, object],
        *,
        fallback: "UniFPPolicyMetadata | None" = None,
    ) -> "UniFPPolicyMetadata":
        base = fallback if fallback is not None else cls()
        joint_actuator_gains_raw = raw.get("joint_actuator_gains", base.joint_actuator_gains)
        joint_actuator_gains = {
            str(joint_name): (float(values[0]), float(values[1]))
            for joint_name, values in dict(joint_actuator_gains_raw).items()
        }
        return cls(
            joint_names=tuple(raw.get("joint_names", base.joint_names)),
            default_joint_pos=tuple(float(v) for v in raw.get("default_joint_pos", base.default_joint_pos)),
            obs_joint_names=tuple(raw.get("obs_joint_names", base.obs_joint_names)),
            obs_default_joint_pos=tuple(float(v) for v in raw.get("obs_default_joint_pos", base.obs_default_joint_pos)),
            auxiliary_joint_defaults={
                str(name): float(value)
                for name, value in dict(raw.get("auxiliary_joint_defaults", base.auxiliary_joint_defaults)).items()
            },
            action_scale=float(raw.get("action_scale", base.action_scale)),
            action_dim=int(raw.get("action_dim", base.action_dim)),
            leg_action_dim=int(raw.get("leg_action_dim", base.leg_action_dim)),
            per_frame_obs_dim=int(raw.get("per_frame_obs_dim", base.per_frame_obs_dim)),
            history_length=int(raw.get("history_length", base.history_length)),
            sim_dt=float(raw.get("sim_dt", base.sim_dt)),
            control_decimation=int(raw.get("control_decimation", base.control_decimation)),
            gait_frequency_hz=float(raw.get("gait_frequency_hz", base.gait_frequency_hz)),
            joint_vel_scale=float(raw.get("joint_vel_scale", base.joint_vel_scale)),
            base_ang_vel_scale=float(raw.get("base_ang_vel_scale", base.base_ang_vel_scale)),
            command_scales=tuple(float(v) for v in raw.get("command_scales", base.command_scales)),
            default_locomotion_cmd=tuple(float(v) for v in raw.get("default_locomotion_cmd", base.default_locomotion_cmd)),
            default_ee_sphere_cmd=tuple(float(v) for v in raw.get("default_ee_sphere_cmd", base.default_ee_sphere_cmd)),
            default_gripper_force_cmd=tuple(
                float(v) for v in raw.get("default_gripper_force_cmd", base.default_gripper_force_cmd)
            ),
            default_base_force_cmd=tuple(
                float(v) for v in raw.get("default_base_force_cmd", base.default_base_force_cmd)
            ),
            freeze_arm_actions=bool(raw.get("freeze_arm_actions", base.freeze_arm_actions)),
            joint_actuator_gains=joint_actuator_gains,
        )

    @classmethod
    def from_json_path(
        cls,
        path: Path,
        *,
        fallback: "UniFPPolicyMetadata | None" = None,
    ) -> "UniFPPolicyMetadata":
        raw = json.loads(path.read_text())
        return cls.from_dict(raw, fallback=fallback)


def default_runtime_metadata() -> UniFPPolicyMetadata:
    return UniFPPolicyMetadata()


def default_staged_eval_metadata() -> UniFPPolicyMetadata:
    gains = {
        "FL_hip_joint": (180.0, 4.0),
        "FR_hip_joint": (180.0, 4.0),
        "RL_hip_joint": (180.0, 4.0),
        "RR_hip_joint": (180.0, 4.0),
        "FL_thigh_joint": (180.0, 4.0),
        "FR_thigh_joint": (180.0, 4.0),
        "RL_thigh_joint": (180.0, 4.0),
        "RR_thigh_joint": (180.0, 4.0),
        "FL_calf_joint": (300.0, 6.5),
        "FR_calf_joint": (300.0, 6.5),
        "RL_calf_joint": (300.0, 6.5),
        "RR_calf_joint": (300.0, 6.5),
        "arm_joint1": (64.0, 2.0),
        "arm_joint2": (128.0, 3.0),
        "arm_joint3": (64.0, 2.0),
        "arm_joint4": (64.0, 2.0),
        "arm_joint5": (64.0, 2.0),
    }
    return replace(
        default_runtime_metadata(),
        gait_frequency_hz=3.0,
        default_locomotion_cmd=(0.0, 0.0, 0.0),
        default_ee_sphere_cmd=(0.60, 0.0, 0.0),
        default_gripper_force_cmd=(0.0, 0.0, 0.0),
        default_base_force_cmd=(0.0, 0.0, 0.0),
        freeze_arm_actions=True,
        joint_actuator_gains=gains,
    )


def infer_metadata_path(policy_path: Path) -> Path:
    return policy_path.with_suffix(".json")


def load_policy_metadata(
    policy_path: Path,
    *,
    fallback: UniFPPolicyMetadata | None = None,
) -> UniFPPolicyMetadata:
    metadata_path = infer_metadata_path(policy_path)
    if metadata_path.exists():
        return UniFPPolicyMetadata.from_json_path(metadata_path, fallback=fallback)
    return fallback if fallback is not None else default_runtime_metadata()


class FrameStacker:
    def __init__(self, frame_dim: int, history_length: int):
        self.frame_dim = frame_dim
        self.history_length = history_length
        self._frames: deque[np.ndarray] = deque(maxlen=history_length)

    def reset(self, frame: np.ndarray) -> None:
        self._frames.clear()
        for _ in range(self.history_length):
            self._frames.append(frame.copy())

    def append(self, frame: np.ndarray) -> None:
        self._frames.append(frame.copy())

    def flattened(self) -> np.ndarray:
        if len(self._frames) != self.history_length:
            raise RuntimeError("Frame stack is not full.")
        return np.concatenate(tuple(self._frames), axis=0)


class FixedDelayBuffer:
    def __init__(self, vector_dim: int, delay_steps: int):
        self.vector_dim = vector_dim
        self.delay_steps = delay_steps
        self._buffer: deque[np.ndarray] = deque(maxlen=delay_steps + 1)

    def reset(self) -> None:
        self._buffer.clear()

    def push(self, vector: np.ndarray) -> np.ndarray:
        vector = vector.astype(np.float32, copy=True)
        self._buffer.append(vector)
        if self.delay_steps == 0:
            return vector
        if len(self._buffer) < self.delay_steps + 1:
            return self._buffer[0].copy()
        return self._buffer[0].copy()


class LegActionDelay:
    def __init__(self, leg_action_dim: int, delay_steps: int):
        self.leg_action_dim = leg_action_dim
        self.delay_steps = delay_steps
        self._buffer: deque[np.ndarray] = deque(maxlen=delay_steps + 1)

    def reset(self) -> None:
        self._buffer.clear()

    def apply(self, action: np.ndarray) -> np.ndarray:
        delayed = action.astype(np.float32, copy=True)
        if self.delay_steps == 0:
            return delayed

        leg_action = delayed[: self.leg_action_dim].copy()
        self._buffer.append(leg_action)
        if len(self._buffer) < self.delay_steps + 1:
            delayed[: self.leg_action_dim] = self._buffer[0]
        else:
            delayed[: self.leg_action_dim] = self._buffer[0]
        return delayed


@dataclass
class PolicyRuntimeState:
    base_orientation_history: FrameStacker
    base_ang_vel_history: FrameStacker
    joint_pos_history: FrameStacker
    joint_vel_history: FrameStacker
    last_action_history: FrameStacker
    gait_history: FrameStacker
    command_history: FrameStacker
    base_orientation_delay: FixedDelayBuffer
    base_ang_vel_delay: FixedDelayBuffer
    joint_pos_delay: FixedDelayBuffer
    joint_vel_delay: FixedDelayBuffer
    leg_action_delay: LegActionDelay
    last_action: np.ndarray

    def policy_obs(self) -> np.ndarray:
        return np.concatenate(
            (
                self.base_orientation_history.flattened(),
                self.base_ang_vel_history.flattened(),
                self.joint_pos_history.flattened(),
                self.joint_vel_history.flattened(),
                self.last_action_history.flattened(),
                self.gait_history.flattened(),
                self.command_history.flattened(),
            ),
            axis=0,
        ).astype(np.float32)


@dataclass(frozen=True)
class ModelBindings:
    joint_ids: tuple[int, ...]
    qpos_indices: np.ndarray
    qvel_indices: np.ndarray
    obs_joint_ids: tuple[int, ...]
    obs_qpos_indices: np.ndarray
    obs_qvel_indices: np.ndarray
    actuator_indices: np.ndarray
    auxiliary_actuators: dict[int, float]


def parse_triplet(values: Iterable[float]) -> np.ndarray:
    values = tuple(values)
    if len(values) != 3:
        raise ValueError(f"Expected exactly 3 values, received {len(values)}")
    return np.asarray(values, dtype=np.float32)


def quat_conjugate(quat_wxyz: np.ndarray) -> np.ndarray:
    return np.array(
        [quat_wxyz[0], -quat_wxyz[1], -quat_wxyz[2], -quat_wxyz[3]],
        dtype=np.float64,
    )


def quat_rotate(quat_wxyz: np.ndarray, vector_xyz: np.ndarray) -> np.ndarray:
    w = quat_wxyz[0]
    xyz = quat_wxyz[1:]
    uv = np.cross(xyz, vector_xyz)
    uuv = np.cross(xyz, uv)
    return vector_xyz + 2.0 * (w * uv + uuv)


def quat_inverse_rotate(quat_wxyz: np.ndarray, vector_xyz: np.ndarray) -> np.ndarray:
    return quat_rotate(quat_conjugate(quat_wxyz), vector_xyz)


def yaw_from_quat(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def base_yaw_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    return quat_from_yaw(yaw_from_quat(quat_wxyz))


def projected_gravity_xy(quat_wxyz: np.ndarray) -> np.ndarray:
    gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    gravity_body = quat_inverse_rotate(quat_wxyz, gravity_world)
    return gravity_body[:2].astype(np.float32)


def world_to_base_yaw(vector_w: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    return quat_inverse_rotate(base_yaw_quat(quat_wxyz), vector_w).astype(np.float32)


def base_yaw_to_world(vector_b: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    return quat_rotate(base_yaw_quat(quat_wxyz), vector_b).astype(np.float32)


def normalize_xy(vector_xy: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    norm = float(np.linalg.norm(vector_xy))
    if norm <= eps:
        return np.array([1.0, 0.0], dtype=np.float32)
    return (vector_xy / norm).astype(np.float32)


def base_forward_axis_xy(base_quat_wxyz: np.ndarray) -> np.ndarray:
    return normalize_xy(base_yaw_to_world(np.array([1.0, 0.0, 0.0], dtype=np.float64), base_quat_wxyz)[:2])


def build_command_vector(
    *,
    locomotion_cmd: Iterable[float],
    ee_sphere_cmd: Iterable[float],
    gripper_force_cmd: Iterable[float],
    base_force_cmd: Iterable[float],
    metadata: UniFPPolicyMetadata,
    ee_orientation_cmd: Iterable[float] | None = None,
) -> np.ndarray:
    if ee_orientation_cmd is None:
        ee_orientation_cmd = (0.0, 0.0, 0.0)
    command = np.concatenate(
        (
            parse_triplet(locomotion_cmd),
            parse_triplet(ee_sphere_cmd),
            parse_triplet(ee_orientation_cmd),
            parse_triplet(gripper_force_cmd),
            parse_triplet(base_force_cmd),
        ),
        axis=0,
    )
    scales = np.asarray(metadata.command_scales, dtype=np.float32)
    return command * scales


@dataclass(frozen=True)
class SphereCommandSegment:
    start_sphere: np.ndarray
    end_sphere: np.ndarray
    move_steps: int
    hold_steps: int

    @property
    def total_steps(self) -> int:
        return self.move_steps + self.hold_steps


class StagedEvalTargetPositionCommand:
    """Approximate the Isaac staged-eval reset trajectory for target_position_commands.

    Isaac does not start this command at the steady-state `(0.60, 0, 0)` value. The first
    reset uses a fixed start sphere `(0.5, pi/8, 0)` and a fixed initial target `(0.7, 0, 0)`,
    then later converges back to the eval steady-state target. Reproducing that transient keeps
    the early policy observations closer to the Isaac rollout.
    """

    def __init__(
        self,
        *,
        control_dt: float,
        steady_target_sphere: Iterable[float],
        initial_start_sphere: Iterable[float] = (0.5, math.pi / 8.0, 0.0),
        initial_target_sphere: Iterable[float] = (0.7, 0.0, 0.0),
        initial_move_time_s: float = 2.25,
        initial_hold_time_s: float = 0.85,
        settle_move_time_s: float = 2.25,
    ):
        self.control_dt = float(control_dt)
        self.steady_target_sphere = parse_triplet(steady_target_sphere)
        self.initial_start_sphere = parse_triplet(initial_start_sphere)
        self.initial_target_sphere = parse_triplet(initial_target_sphere)
        self.initial_move_time_s = float(initial_move_time_s)
        self.initial_hold_time_s = float(initial_hold_time_s)
        self.settle_move_time_s = float(settle_move_time_s)
        self._segments: tuple[SphereCommandSegment, ...] = ()
        self._segment_index = 0
        self._segment_step = 0
        self.reset()

    def reset(self) -> None:
        self._segments = (
            SphereCommandSegment(
                start_sphere=self.initial_start_sphere.copy(),
                end_sphere=self.initial_target_sphere.copy(),
                move_steps=self._seconds_to_steps(self.initial_move_time_s),
                hold_steps=self._seconds_to_steps(self.initial_hold_time_s),
            ),
            SphereCommandSegment(
                start_sphere=self.initial_target_sphere.copy(),
                end_sphere=self.steady_target_sphere.copy(),
                move_steps=self._seconds_to_steps(self.settle_move_time_s),
                hold_steps=0,
            ),
        )
        self._segment_index = 0
        self._segment_step = 0

    def current_sphere_command(self) -> np.ndarray:
        segment = self._current_segment()
        if segment is None:
            return self.steady_target_sphere.copy()

        denom = max(segment.move_steps, 1)
        ratio = min(max(self._segment_step / denom, 0.0), 1.0)
        command = segment.start_sphere * (1.0 - ratio) + segment.end_sphere * ratio
        return command.astype(np.float32, copy=False)

    def advance(self) -> None:
        segment = self._current_segment()
        if segment is None:
            return

        self._segment_step += 1
        if self._segment_step < segment.total_steps:
            return

        self._segment_index += 1
        self._segment_step = 0

    def _current_segment(self) -> SphereCommandSegment | None:
        if self._segment_index >= len(self._segments):
            return None
        return self._segments[self._segment_index]

    def _seconds_to_steps(self, duration_s: float) -> int:
        return max(int(round(float(duration_s) / self.control_dt)), 1)


def gait_clock(data_time_s: float, metadata: UniFPPolicyMetadata) -> np.ndarray:
    phase = (data_time_s * metadata.gait_frequency_hz) % 1.0
    return np.array(
        [math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)],
        dtype=np.float32,
    )


def compute_proprio(
    data: "mujoco.MjData",
    qpos_indices: np.ndarray,
    qvel_indices: np.ndarray,
    metadata: UniFPPolicyMetadata,
) -> np.ndarray:
    base_quat = data.qpos[3:7].copy()
    base_orientation = projected_gravity_xy(base_quat)
    base_ang_vel = (data.qvel[3:6].astype(np.float64) * metadata.base_ang_vel_scale).astype(np.float32)
    joint_pos_rel = (data.qpos[qpos_indices] - np.asarray(metadata.obs_default_joint_pos, dtype=np.float64)).astype(
        np.float32
    )
    joint_vel = (data.qvel[qvel_indices] * metadata.joint_vel_scale).astype(np.float32)
    return np.concatenate((base_orientation, base_ang_vel, joint_pos_rel, joint_vel), axis=0).astype(np.float32)


def compute_policy_terms(
    data: "mujoco.MjData",
    last_action: np.ndarray,
    command_obs: np.ndarray,
    delayed_base_orientation: np.ndarray,
    delayed_base_ang_vel: np.ndarray,
    delayed_joint_pos: np.ndarray,
    delayed_joint_vel: np.ndarray,
    metadata: UniFPPolicyMetadata,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gait = gait_clock(float(data.time), metadata)
    frame = np.concatenate(
        (
            delayed_base_orientation,
            delayed_base_ang_vel,
            delayed_joint_pos,
            delayed_joint_vel,
            last_action,
            gait,
            command_obs,
        ),
        axis=0,
    ).astype(np.float32)
    if frame.shape != (metadata.per_frame_obs_dim,):
        raise RuntimeError(
            f"Observation frame shape mismatch. Expected {(metadata.per_frame_obs_dim,)}, received {frame.shape}"
        )
    return (
        delayed_base_orientation.astype(np.float32, copy=False),
        delayed_base_ang_vel.astype(np.float32, copy=False),
        delayed_joint_pos.astype(np.float32, copy=False),
        delayed_joint_vel.astype(np.float32, copy=False),
        last_action.astype(np.float32, copy=False),
        gait.astype(np.float32, copy=False),
        command_obs.astype(np.float32, copy=False),
    )


def load_torchscript_policy(policy_path: Path):
    if torch is None:
        raise RuntimeError("Torch is not installed. Install dependencies from scripts/mujoco/requirements.txt first.")
    policy = torch.jit.load(str(policy_path), map_location="cpu")
    policy.eval()
    return policy


def infer_policy_outputs(
    policy,
    obs: np.ndarray,
    *,
    action_dim: int,
) -> tuple[np.ndarray, np.ndarray | None]:
    if policy is None:
        return np.zeros(action_dim, dtype=np.float32), None
    if torch is None:
        raise RuntimeError("Torch is required for policy inference.")

    with torch.inference_mode():
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        outputs = policy(obs_tensor)

    if isinstance(outputs, torch.Tensor):
        action_tensor = outputs
        pred_tensor = None
    elif isinstance(outputs, (tuple, list)) and outputs:
        action_tensor = outputs[0]
        pred_tensor = outputs[1] if len(outputs) > 1 and isinstance(outputs[1], torch.Tensor) else None
    else:
        raise RuntimeError(f"Unsupported TorchScript output type: {type(outputs)!r}")

    action = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)
    if action.shape != (action_dim,):
        raise RuntimeError(f"Policy action shape mismatch. Expected {(action_dim,)}, received {action.shape}")

    pred_est = None
    if pred_tensor is not None:
        pred_est = pred_tensor.squeeze(0).cpu().numpy().astype(np.float32)
    return action, pred_est


def resolve_joint_ids(model: "mujoco.MjModel", joint_names: tuple[str, ...]) -> tuple[int, ...]:
    joint_ids: list[int] = []
    for joint_name in joint_names:
        joint_id = model.joint(joint_name).id
        if joint_id < 0:
            raise RuntimeError(f"Joint '{joint_name}' is missing from the MuJoCo model.")
        joint_ids.append(joint_id)
    return tuple(joint_ids)


def resolve_qpos_qvel_indices(
    model: "mujoco.MjModel",
    joint_ids: tuple[int, ...],
) -> tuple[np.ndarray, np.ndarray]:
    qpos_indices = np.asarray([model.jnt_qposadr[joint_id] for joint_id in joint_ids], dtype=np.int32)
    qvel_indices = np.asarray([model.jnt_dofadr[joint_id] for joint_id in joint_ids], dtype=np.int32)
    return qpos_indices, qvel_indices


def resolve_joint_actuator_indices(model: "mujoco.MjModel", joint_ids: tuple[int, ...]) -> np.ndarray:
    actuator_indices: list[int] = []
    for joint_id in joint_ids:
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if model.actuator_trnid[actuator_id, 0] == joint_id
            and model.actuator_trntype[actuator_id] == 0
        ]
        if len(matches) != 1:
            raise RuntimeError(
                "Expected exactly one joint actuator per controlled joint. "
                f"Joint id {joint_id} matched {len(matches)} actuators."
            )
        actuator_indices.append(matches[0])
    return np.asarray(actuator_indices, dtype=np.int32)


def resolve_auxiliary_actuators(
    model: "mujoco.MjModel",
    auxiliary_joint_defaults: dict[str, float],
) -> dict[int, float]:
    resolved: dict[int, float] = {}
    for joint_name, default_value in auxiliary_joint_defaults.items():
        try:
            joint_id = model.joint(joint_name).id
        except KeyError:
            continue
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if model.actuator_trnid[actuator_id, 0] == joint_id and model.actuator_trntype[actuator_id] == 0
        ]
        for actuator_id in matches:
            resolved[actuator_id] = float(default_value)
    return resolved


def resolve_body_id(model: "mujoco.MjModel", body_name: str) -> int:
    return model.body(body_name).id


def create_model_bindings(model: "mujoco.MjModel", metadata: UniFPPolicyMetadata) -> ModelBindings:
    joint_ids = resolve_joint_ids(model, metadata.joint_names)
    qpos_indices, qvel_indices = resolve_qpos_qvel_indices(model, joint_ids)
    obs_joint_ids = resolve_joint_ids(model, metadata.obs_joint_names)
    obs_qpos_indices, obs_qvel_indices = resolve_qpos_qvel_indices(model, obs_joint_ids)
    actuator_indices = resolve_joint_actuator_indices(model, joint_ids)
    auxiliary_actuators = resolve_auxiliary_actuators(model, metadata.auxiliary_joint_defaults)
    return ModelBindings(
        joint_ids=joint_ids,
        qpos_indices=qpos_indices,
        qvel_indices=qvel_indices,
        obs_joint_ids=obs_joint_ids,
        obs_qpos_indices=obs_qpos_indices,
        obs_qvel_indices=obs_qvel_indices,
        actuator_indices=actuator_indices,
        auxiliary_actuators=auxiliary_actuators,
    )


def reset_model_state(
    mujoco,
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    metadata: UniFPPolicyMetadata,
    qpos_indices: np.ndarray,
) -> None:
    mujoco.mj_resetData(model, data)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        if model.nq >= 7:
            data.qpos[0:3] = np.array([0.0, 0.0, 0.5], dtype=np.float64)
            data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        data.qpos[qpos_indices] = np.asarray(metadata.default_joint_pos, dtype=np.float64)
        for joint_name, joint_default in metadata.auxiliary_joint_defaults.items():
            try:
                joint = model.joint(joint_name)
            except KeyError:
                continue
            data.qpos[model.jnt_qposadr[joint.id]] = joint_default
    mujoco.mj_forward(model, data)


def initialize_runtime_state(
    data: "mujoco.MjData",
    bindings: ModelBindings,
    metadata: UniFPPolicyMetadata,
    *,
    obs_delay_steps: int,
    leg_action_delay_steps: int,
    command_obs: np.ndarray,
) -> PolicyRuntimeState:
    base_orientation_history = FrameStacker(2, metadata.history_length)
    base_ang_vel_history = FrameStacker(3, metadata.history_length)
    joint_dim = len(metadata.obs_joint_names)
    joint_pos_history = FrameStacker(joint_dim, metadata.history_length)
    joint_vel_history = FrameStacker(joint_dim, metadata.history_length)
    last_action_history = FrameStacker(metadata.action_dim, metadata.history_length)
    gait_history = FrameStacker(2, metadata.history_length)
    command_history = FrameStacker(len(metadata.command_scales), metadata.history_length)
    base_orientation_delay = FixedDelayBuffer(vector_dim=2, delay_steps=obs_delay_steps)
    base_ang_vel_delay = FixedDelayBuffer(vector_dim=3, delay_steps=obs_delay_steps)
    joint_pos_delay = FixedDelayBuffer(vector_dim=joint_dim, delay_steps=obs_delay_steps)
    joint_vel_delay = FixedDelayBuffer(vector_dim=joint_dim, delay_steps=obs_delay_steps)
    leg_action_delay = LegActionDelay(metadata.leg_action_dim, leg_action_delay_steps)
    base_orientation_delay.reset()
    base_ang_vel_delay.reset()
    joint_pos_delay.reset()
    joint_vel_delay.reset()
    leg_action_delay.reset()

    last_action = np.zeros(metadata.action_dim, dtype=np.float32)
    initial_proprio = compute_proprio(data, bindings.obs_qpos_indices, bindings.obs_qvel_indices, metadata)
    delayed_base_orientation = base_orientation_delay.push(initial_proprio[0:2])
    delayed_base_ang_vel = base_ang_vel_delay.push(initial_proprio[2:5])
    delayed_joint_pos = joint_pos_delay.push(initial_proprio[5 : 5 + joint_dim])
    delayed_joint_vel = joint_vel_delay.push(initial_proprio[5 + joint_dim : 5 + 2 * joint_dim])
    initial_terms = compute_policy_terms(
        data=data,
        last_action=last_action,
        command_obs=command_obs,
        delayed_base_orientation=delayed_base_orientation,
        delayed_base_ang_vel=delayed_base_ang_vel,
        delayed_joint_pos=delayed_joint_pos,
        delayed_joint_vel=delayed_joint_vel,
        metadata=metadata,
    )
    base_orientation_history.reset(initial_terms[0])
    base_ang_vel_history.reset(initial_terms[1])
    joint_pos_history.reset(initial_terms[2])
    joint_vel_history.reset(initial_terms[3])
    last_action_history.reset(initial_terms[4])
    gait_history.reset(initial_terms[5])
    command_history.reset(initial_terms[6])
    return PolicyRuntimeState(
        base_orientation_history=base_orientation_history,
        base_ang_vel_history=base_ang_vel_history,
        joint_pos_history=joint_pos_history,
        joint_vel_history=joint_vel_history,
        last_action_history=last_action_history,
        gait_history=gait_history,
        command_history=command_history,
        base_orientation_delay=base_orientation_delay,
        base_ang_vel_delay=base_ang_vel_delay,
        joint_pos_delay=joint_pos_delay,
        joint_vel_delay=joint_vel_delay,
        leg_action_delay=leg_action_delay,
        last_action=last_action,
    )


def update_runtime_state_after_step(
    data: "mujoco.MjData",
    bindings: ModelBindings,
    metadata: UniFPPolicyMetadata,
    runtime_state: PolicyRuntimeState,
    *,
    action: np.ndarray,
    command_obs: np.ndarray,
) -> None:
    runtime_state.last_action = action.astype(np.float32, copy=True)
    proprio = compute_proprio(data, bindings.obs_qpos_indices, bindings.obs_qvel_indices, metadata)
    joint_dim = len(metadata.obs_joint_names)
    delayed_base_orientation = runtime_state.base_orientation_delay.push(proprio[0:2])
    delayed_base_ang_vel = runtime_state.base_ang_vel_delay.push(proprio[2:5])
    delayed_joint_pos = runtime_state.joint_pos_delay.push(proprio[5 : 5 + joint_dim])
    delayed_joint_vel = runtime_state.joint_vel_delay.push(proprio[5 + joint_dim : 5 + 2 * joint_dim])
    terms = compute_policy_terms(
        data=data,
        last_action=runtime_state.last_action,
        command_obs=command_obs,
        delayed_base_orientation=delayed_base_orientation,
        delayed_base_ang_vel=delayed_base_ang_vel,
        delayed_joint_pos=delayed_joint_pos,
        delayed_joint_vel=delayed_joint_vel,
        metadata=metadata,
    )
    runtime_state.base_orientation_history.append(terms[0])
    runtime_state.base_ang_vel_history.append(terms[1])
    runtime_state.joint_pos_history.append(terms[2])
    runtime_state.joint_vel_history.append(terms[3])
    runtime_state.last_action_history.append(terms[4])
    runtime_state.gait_history.append(terms[5])
    runtime_state.command_history.append(terms[6])


def build_target_joint_positions(
    action: np.ndarray,
    runtime_state: PolicyRuntimeState,
    metadata: UniFPPolicyMetadata,
) -> np.ndarray:
    delayed_action = runtime_state.leg_action_delay.apply(action)
    target_joint_pos = np.asarray(metadata.default_joint_pos, dtype=np.float32).copy()
    target_joint_pos[: metadata.leg_action_dim] += metadata.action_scale * delayed_action[: metadata.leg_action_dim]
    if not metadata.freeze_arm_actions:
        target_joint_pos[metadata.leg_action_dim :] += (
            metadata.action_scale * delayed_action[metadata.leg_action_dim :]
        )
    return target_joint_pos


def apply_control_targets(
    data: "mujoco.MjData",
    bindings: ModelBindings,
    target_joint_pos: np.ndarray,
) -> None:
    data.ctrl[bindings.actuator_indices] = target_joint_pos
    for actuator_id, actuator_target in bindings.auxiliary_actuators.items():
        data.ctrl[actuator_id] = actuator_target


def set_body_force(
    data: "mujoco.MjData",
    body_id: int,
    force_w: np.ndarray,
) -> None:
    data.xfrc_applied[:] = 0.0
    data.xfrc_applied[body_id, :3] = np.asarray(force_w, dtype=np.float64)


def clear_body_forces(data: "mujoco.MjData") -> None:
    data.xfrc_applied[:] = 0.0


def apply_joint_actuator_gains(
    model: "mujoco.MjModel",
    bindings: ModelBindings,
    metadata: UniFPPolicyMetadata,
) -> None:
    if not metadata.joint_actuator_gains:
        return
    for joint_name, (kp, kv) in metadata.joint_actuator_gains.items():
        try:
            joint_id = model.joint(joint_name).id
        except KeyError:
            continue
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if model.actuator_trnid[actuator_id, 0] == joint_id and model.actuator_trntype[actuator_id] == 0
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"Expected exactly one joint actuator for gain override on '{joint_name}', got {len(matches)}."
            )
        actuator_id = matches[0]
        model.actuator_gainprm[actuator_id, 0] = kp
        model.actuator_biasprm[actuator_id, 1] = -kp
        model.actuator_biasprm[actuator_id, 2] = -kv


def finite_or_nan(value: float) -> float:
    return value if math.isfinite(value) else float("nan")


def format_optional(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.3f}"


def summarize_joint_actuator_gains(metadata: UniFPPolicyMetadata) -> str:
    if not metadata.joint_actuator_gains:
        return "none"
    counts: dict[tuple[float, float], int] = {}
    for kp, kv in metadata.joint_actuator_gains.values():
        key = (float(kp), float(kv))
        counts[key] = counts.get(key, 0) + 1
    summary_parts = [
        f"kp={kp:.1f}/kv={kv:.1f} x{count}"
        for (kp, kv), count in sorted(counts.items(), key=lambda item: (item[0][0], item[0][1], item[1]))
    ]
    return ", ".join(summary_parts)


def configure_tracking_viewer_camera(
    mujoco,
    viewer,
    data: "mujoco.MjData",
    *,
    track_body_id: int,
    distance: float = 3.5,
    azimuth: float = 180.0,
    elevation: float = -18.0,
    z_offset: float = 0.35,
) -> None:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = int(track_body_id)
    viewer.cam.distance = float(distance)
    viewer.cam.azimuth = float(azimuth)
    viewer.cam.elevation = float(elevation)
    viewer.cam.lookat[:] = data.xpos[track_body_id]
    viewer.cam.lookat[2] += float(z_offset)
