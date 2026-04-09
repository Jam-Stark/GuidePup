#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import sys

import torch

SCRIPTS_DIR = Path(__file__).resolve().parents[2]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher


LONG_RESAMPLING_S = 1.0e9
DEFAULT_EPISODE_LENGTH_S = 60.0
DEFAULT_CHECKPOINT_PATH = (
    "logs/rsl_rl/uniFP_aliengo_position_force/2026-03-17_22-31-52_cfg33-Random-upSpeed/model_19400.pt"
)

FORCE_INTERP_LINEAR = "linear"
FORCE_INTERP_QUADRATIC = "quadratic"

FORCE_SCHEDULE_NONE = "none"
FORCE_SCHEDULE_DEMO = "demo"

DIR_NONE = "none"
DIR_FORWARD = "forward"
DIR_LEFT = "left"
DIR_BACKWARD = "backward"

MODE_STANDING = 0
MODE_FORCE_WALKING = 1
MODE_COMMAND_WALKING = 2

MODE_NAME_BY_ID = {
    MODE_STANDING: "standing",
    MODE_FORCE_WALKING: "force_walking",
    MODE_COMMAND_WALKING: "command_walking",
}


@dataclass(frozen=True)
class ForceSegment:
    segment_name: str
    direction_name: str
    phase_name: str
    start_s: float
    end_s: float
    force_start_n: float = 0.0
    force_end_n: float = 0.0
    capture_axis: bool = False
    force_interp: str = FORCE_INTERP_LINEAR

    @property
    def is_active(self) -> bool:
        return self.direction_name != DIR_NONE

    def force_target_at(self, time_s: float) -> float:
        if not self.is_active:
            return 0.0
        if self.end_s <= self.start_s or math.isclose(self.force_start_n, self.force_end_n):
            return float(self.force_end_n)
        ratio = (float(time_s) - self.start_s) / (self.end_s - self.start_s)
        ratio = max(0.0, min(ratio, 1.0))
        if self.force_interp == FORCE_INTERP_QUADRATIC:
            ratio = ratio * ratio
        return float(self.force_start_n + ratio * (self.force_end_n - self.force_start_n))


@dataclass
class ForceModeSwitchState:
    mode: torch.Tensor
    command_zero: torch.Tensor
    force_xy: torch.Tensor
    force_lp: torch.Tensor
    enter_timer: torch.Tensor
    exit_timer: torch.Tensor
    standing_mask: torch.Tensor
    walking_mask: torch.Tensor


parser = argparse.ArgumentParser(
    description=(
        "Play an existing UniFP Aliengo checkpoint with a runtime standing/walking arbiter. "
        "The outer loop keeps the policy weights unchanged and only rewrites gait standing semantics "
        "based on command clip rules plus the base force estimator."
    )
)
policy_group = parser.add_mutually_exclusive_group(required=False)
policy_group.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help=(
        "Path to an RSL-RL checkpoint. "
        f"Defaults to {DEFAULT_CHECKPOINT_PATH} when neither --checkpoint nor --policy-jit is provided."
    ),
)
policy_group.add_argument(
    "--policy-jit",
    type=str,
    help=(
        "Path to an exported JIT policy. Note: the stock IsaacLab exporter is not compatible with this task's "
        "AsymmetricActorCritic adaptation policy."
    ),
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
parser.add_argument("--episode-length-s", type=float, default=DEFAULT_EPISODE_LENGTH_S, help="Episode horizon.")
parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
parser.add_argument("--cmd-vx", type=float, default=0.0, help="Fixed body-frame x velocity command.")
parser.add_argument("--cmd-vy", type=float, default=0.0, help="Fixed body-frame y velocity command.")
parser.add_argument("--cmd-wz", type=float, default=0.0, help="Fixed yaw-rate command.")
parser.add_argument(
    "--force-schedule",
    type=str,
    default=FORCE_SCHEDULE_DEMO,
    choices=[FORCE_SCHEDULE_NONE, FORCE_SCHEDULE_DEMO],
    help="External base-force schedule. Default is a zero-command demo that triggers force_walking.",
)
parser.add_argument(
    "--walking-gait-frequency",
    type=float,
    default=3.0,
    help="Walking gait frequency (Hz) used when mode is command_walking or force_walking.",
)
parser.add_argument("--force-lp-tau-s", type=float, default=0.15, help="Low-pass filter time constant.")
parser.add_argument("--enter-threshold-n", type=float, default=0.5, help="Enter walking threshold on F_lp.")
parser.add_argument("--exit-threshold-n", type=float, default=0.25, help="Exit to standing threshold on F_lp.")
parser.add_argument("--enter-hold-s", type=float, default=0.3, help="Duration required to enter force_walking.")
parser.add_argument("--exit-hold-s", type=float, default=0.5, help="Duration required to return to standing.")
parser.add_argument("--log-interval", type=int, default=60, help="Console logging period in simulation steps.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.checkpoint is None and args_cli.policy_jit is None:
    args_cli.checkpoint = DEFAULT_CHECKPOINT_PATH

livestream_enabled = str(os.environ.get("LIVESTREAM", "0")) not in ("", "0")
if getattr(args_cli, "livestream", -1) not in (-1, 0):
    livestream_enabled = True
if livestream_enabled and not getattr(args_cli, "enable_cameras", False):
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import acc  # noqa: F401

import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.math import quat_from_angle_axis, quat_from_euler_xyz

from acc.tasks.manager_based.uniFP_aliengo.mdp.utils import base_yaw_to_world, get_base_yaw_quat
from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg


def resolve_output_csv(path_str: str | None) -> Path:
    if path_str:
        output_path = Path(path_str).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = Path("logs") / "experiments" / "unifp_smart_gait" / f"smart_gait_{stamp}.csv"
        output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env_cfg.commands.locomotion_commands.resampling_time_range = (LONG_RESAMPLING_S, LONG_RESAMPLING_S)
    env_cfg.commands.locomotion_commands.rel_standing_envs = 0.0
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (float(args_cli.cmd_vx), float(args_cli.cmd_vx))
    env_cfg.commands.locomotion_commands.ranges.lin_vel_y = (float(args_cli.cmd_vy), float(args_cli.cmd_vy))
    env_cfg.commands.locomotion_commands.ranges.ang_vel_z = (float(args_cli.cmd_wz), float(args_cli.cmd_wz))
    env_cfg.commands.gait_phase.gait_frequency = float(args_cli.walking_gait_frequency)
    env_cfg.commands.locomotion_commands.debug_vis = False
    env_cfg.commands.target_position_commands.debug_vis = False

    # Isaac Lab managers skip None-valued terms during preparation, which disables this early termination.
    env_cfg.terminations.base_height = None
    return env_cfg


def load_raw_checkpoint_policy(checkpoint_path: str, env: ManagerBasedRLEnv):
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    from acc.algorithms.rsl_rl.runner import OnPolicyRunnerAdaptation
    from acc.tasks.manager_based.uniFP_aliengo.agents.rsl_rl_ppo_cfg import PPORunnerCfg

    agent_cfg = PPORunnerCfg()
    agent_cfg.device = env.device

    wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunnerAdaptation(wrapped_env, agent_cfg.to_dict(), log_dir=None, device=env.device)
    runner.load(checkpoint_path)
    policy = runner.get_inference_policy(device=env.device)
    estimator_model = runner.alg.policy
    estimator_model.eval()
    return policy, wrapped_env, estimator_model


def normalize_xy(vectors_xy: torch.Tensor, eps: float = 1.0e-6) -> torch.Tensor:
    norms = torch.linalg.norm(vectors_xy, dim=-1, keepdim=True)
    fallback = torch.zeros_like(vectors_xy)
    fallback[:, 0] = 1.0
    return torch.where(norms > eps, vectors_xy / torch.clamp(norms, min=eps), fallback)


def base_forward_axis_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    yaw_quat = get_base_yaw_quat(env)
    local_forward = torch.zeros((env.num_envs, 3), device=env.device)
    local_forward[:, 0] = 1.0
    forward_w = base_yaw_to_world(local_forward, yaw_quat)
    return normalize_xy(forward_w[:, :2])


def direction_axis_from_forward(forward_axis_xy: torch.Tensor, direction_name: str) -> torch.Tensor:
    if direction_name == DIR_FORWARD:
        return forward_axis_xy
    if direction_name == DIR_LEFT:
        return torch.stack([-forward_axis_xy[:, 1], forward_axis_xy[:, 0]], dim=-1)
    if direction_name == DIR_BACKWARD:
        return -forward_axis_xy
    return torch.zeros_like(forward_axis_xy)


def extract_force_estimator_outputs(
    obs,
    estimator_model,
    *,
    gripper_force_scale: float,
    base_force_scale: float,
):
    num_envs = obs.batch_size[0]
    device = obs.device
    nan_force = torch.full((num_envs, 3), float("nan"), device=device)
    nan_err = torch.full((num_envs,), float("nan"), device=device)

    if estimator_model is None or not hasattr(estimator_model, "predict") or "pred" not in obs:
        return {
            "gripper_force_est_local": nan_force.clone(),
            "gripper_force_tgt_local": nan_force.clone(),
            "gripper_force_err_l2": nan_err.clone(),
            "base_force_est_local": nan_force.clone(),
            "base_force_tgt_local": nan_force.clone(),
            "base_force_err_l2": nan_err.clone(),
        }

    pred_target_scaled = obs["pred"]
    pred_est_scaled = estimator_model.predict(obs)

    gripper_force_est_local = pred_est_scaled[:, 6:9] / gripper_force_scale
    gripper_force_tgt_local = pred_target_scaled[:, 6:9] / gripper_force_scale
    base_force_est_local = pred_est_scaled[:, 9:12] / base_force_scale
    base_force_tgt_local = pred_target_scaled[:, 9:12] / base_force_scale

    return {
        "gripper_force_est_local": gripper_force_est_local,
        "gripper_force_tgt_local": gripper_force_tgt_local,
        "gripper_force_err_l2": torch.linalg.norm(gripper_force_est_local - gripper_force_tgt_local, dim=-1),
        "base_force_est_local": base_force_est_local,
        "base_force_tgt_local": base_force_tgt_local,
        "base_force_err_l2": torch.linalg.norm(base_force_est_local - base_force_tgt_local, dim=-1),
    }


def build_demo_force_segments(episode_length_s: float) -> tuple[ForceSegment, ...]:
    tail_end = max(float(episode_length_s), 60.0)
    return (
        ForceSegment("idle_0", DIR_NONE, "idle", 0.0, 5.0),
        ForceSegment("forward_low_10", DIR_FORWARD, "low", 5.0, 6.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("forward_low_15", DIR_FORWARD, "low", 6.0, 7.0, 15.0, 15.0),
        ForceSegment("forward_low_5", DIR_FORWARD, "low", 7.0, 8.0, 5.0, 5.0),
        ForceSegment(
            "forward_ramp_quad",
            DIR_FORWARD,
            "ramp",
            8.0,
            11.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("forward_high", DIR_FORWARD, "high", 11.0, 13.0, 40.0, 40.0),
        ForceSegment("idle_1", DIR_NONE, "idle", 13.0, 20.0),
        ForceSegment("left_low_10", DIR_LEFT, "low", 20.0, 21.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("left_low_15", DIR_LEFT, "low", 21.0, 22.0, 15.0, 15.0),
        ForceSegment("left_low_5", DIR_LEFT, "low", 22.0, 23.0, 5.0, 5.0),
        ForceSegment(
            "left_ramp_quad",
            DIR_LEFT,
            "ramp",
            23.0,
            26.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("left_high", DIR_LEFT, "high", 26.0, 28.0, 40.0, 40.0),
        ForceSegment("idle_2", DIR_NONE, "idle", 28.0, 40.0),
        ForceSegment("backward_low_10", DIR_BACKWARD, "low", 40.0, 41.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("backward_low_15", DIR_BACKWARD, "low", 41.0, 42.0, 15.0, 15.0),
        ForceSegment("backward_low_5", DIR_BACKWARD, "low", 42.0, 43.0, 5.0, 5.0),
        ForceSegment(
            "backward_ramp_quad",
            DIR_BACKWARD,
            "ramp",
            43.0,
            46.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("backward_high", DIR_BACKWARD, "high", 46.0, 48.0, 40.0, 40.0),
        ForceSegment("idle_3", DIR_NONE, "idle", 48.0, tail_end),
    )


class ScriptedForceScheduler:
    """Deterministic base-force schedule used for the default zero-command demo."""

    def __init__(self, num_envs: int, device: torch.device | str, segments: tuple[ForceSegment, ...]):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.segments = segments
        self.current_segment_name: str | None = None
        self.current_axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

    def segment_for_time(self, time_s: float) -> ForceSegment:
        if time_s >= self.segments[-1].start_s:
            return self.segments[-1]
        for segment in self.segments:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return self.segments[-1]

    def update(self, env: ManagerBasedRLEnv, time_s: float) -> dict[str, torch.Tensor | float | str | bool]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name

        if segment_changed and segment.capture_axis and segment.is_active:
            forward_axis_xy = base_forward_axis_xy(env)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)

        force_target_n = torch.full((self.num_envs,), segment.force_target_at(time_s), device=self.device)
        force_w = torch.zeros((self.num_envs, 3), device=self.device)
        axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

        if segment.is_active:
            axis_xy = self.current_axis_xy.clone()
            force_w[:, :2] = axis_xy * force_target_n.unsqueeze(-1)

        self.current_segment_name = segment.segment_name
        return {
            "segment_name": segment.segment_name,
            "direction_name": segment.direction_name,
            "phase_name": segment.phase_name,
            "force_target_n": force_target_n,
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": segment.is_active,
        }


class SmartGaitMarkers:
    """Visualization markers for command velocity, current velocity, and external base force."""

    def __init__(self, env: ManagerBasedRLEnv):
        self.env = env
        self.device = env.device
        self.cmd_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/cmd_velocity",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(0.6, 0.6, 0.6),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                    ),
                },
            )
        )
        self.curr_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/current_velocity",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(0.6, 0.6, 0.6),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.9, 0.9)),
                    ),
                },
            )
        )
        self.base_force_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/base_force_ext",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.4, 1.0)),
                    ),
                },
            )
        )
        self.cmd_vel_markers.set_visibility(True)
        self.curr_vel_markers.set_visibility(True)
        self.base_force_markers.set_visibility(True)
        self._cmd_base_scale = torch.tensor((0.6, 0.6, 0.6), device=self.device, dtype=torch.float)
        self._curr_base_scale = torch.tensor((0.6, 0.6, 0.6), device=self.device, dtype=torch.float)

    def update(self):
        robot = self.env.scene["robot"]
        base_pos_w = robot.data.root_pos_w.clone()
        vel_marker_pos_w = base_pos_w.clone()
        vel_marker_pos_w[:, 2] += 0.5
        force_marker_pos_w = base_pos_w.clone()
        force_marker_pos_w[:, 2] += 0.2

        base_yaw_quat = get_base_yaw_quat(self.env)
        cmd_vel_b = self.env.command_manager.get_command("locomotion_commands")
        cmd_vel_w = torch.zeros((self.env.num_envs, 3), device=self.device)
        cmd_vel_w[:, :2] = base_yaw_to_world(
            torch.nn.functional.pad(cmd_vel_b[:, :2], (0, 1)),
            base_yaw_quat,
        )[:, :2]

        base_lin_vel_w = robot.data.root_lin_vel_w
        base_force_ext_w = self.env.command_manager.get_command("base_force_ext")

        self.cmd_vel_markers.visualize(
            vel_marker_pos_w,
            self._vectors_to_yaw_quat(cmd_vel_w),
            scales=self._apply_speed_scale(cmd_vel_w, self._cmd_base_scale),
        )
        self.curr_vel_markers.visualize(
            vel_marker_pos_w,
            self._vectors_to_yaw_quat(base_lin_vel_w),
            scales=self._apply_speed_scale(base_lin_vel_w, self._curr_base_scale),
        )
        self.base_force_markers.visualize(
            force_marker_pos_w,
            self._forces_to_quat(base_force_ext_w),
            scales=self._forces_to_scales(base_force_ext_w),
        )

    def _vectors_to_yaw_quat(self, vectors_w: torch.Tensor) -> torch.Tensor:
        yaw = torch.atan2(vectors_w[:, 1], vectors_w[:, 0])
        zeros = torch.zeros_like(yaw)
        return quat_from_euler_xyz(zeros, zeros, yaw)

    def _apply_speed_scale(self, vel_w: torch.Tensor, base_scale: torch.Tensor) -> torch.Tensor:
        speed = torch.norm(vel_w[:, :2], dim=-1)
        speed_ref = 0.3
        scale_factor = torch.clamp(speed / max(speed_ref, 1.0e-3), min=1.2, max=3.0)
        scales = base_scale.unsqueeze(0).repeat(vel_w.shape[0], 1)
        scales[:, 0] = scales[:, 0] * scale_factor
        return scales

    def _forces_to_quat(self, forces_w: torch.Tensor) -> torch.Tensor:
        eps = 1.0e-6
        mag = torch.linalg.norm(forces_w, dim=1)
        direction = forces_w / (mag.unsqueeze(-1) + eps)
        dot = torch.clamp(direction[:, 0], -1.0, 1.0)
        angle = torch.acos(dot)
        axis = torch.stack([torch.zeros_like(dot), -direction[:, 2], direction[:, 1]], dim=1)
        axis_norm = torch.linalg.norm(axis, dim=1, keepdim=True)
        default_axis = torch.zeros_like(axis)
        default_axis[:, 2] = 1.0
        axis = torch.where(axis_norm > eps, axis, default_axis)
        axis = axis / torch.clamp(torch.linalg.norm(axis, dim=1, keepdim=True), min=eps)
        quat = quat_from_angle_axis(angle, axis)
        identity = torch.zeros((forces_w.shape[0], 4), device=forces_w.device)
        identity[:, 0] = 1.0
        return torch.where(mag.unsqueeze(-1) > eps, quat, identity)

    def _forces_to_scales(self, forces_w: torch.Tensor) -> torch.Tensor:
        mag = torch.linalg.norm(forces_w, dim=1)
        length = torch.clamp(mag * 0.02, max=0.6)
        thickness = torch.full_like(length, 0.05)
        return torch.stack([length, thickness, thickness], dim=1)


class ForceModeSwitcher:
    """Runtime outer-loop arbiter for standing vs. walking gait semantics."""

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        lin_vel_x_clip: float,
        lin_vel_y_clip: float,
        ang_vel_yaw_clip: float,
        lp_tau_s: float,
        enter_th_n: float,
        exit_th_n: float,
        enter_hold_s: float,
        exit_hold_s: float,
    ):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.lin_vel_x_clip = float(lin_vel_x_clip)
        self.lin_vel_y_clip = float(lin_vel_y_clip)
        self.ang_vel_yaw_clip = float(ang_vel_yaw_clip)
        self.lp_tau_s = float(lp_tau_s)
        self.enter_th_n = float(enter_th_n)
        self.exit_th_n = float(exit_th_n)
        self.enter_hold_s = float(enter_hold_s)
        self.exit_hold_s = float(exit_hold_s)

        self.mode = torch.full((self.num_envs,), MODE_STANDING, device=self.device, dtype=torch.long)
        self.force_state = torch.full((self.num_envs,), MODE_STANDING, device=self.device, dtype=torch.long)
        self.force_lp = torch.zeros((self.num_envs,), device=self.device)
        self.enter_timer = torch.zeros((self.num_envs,), device=self.device)
        self.exit_timer = torch.zeros((self.num_envs,), device=self.device)
        self.last_force_xy = torch.zeros((self.num_envs,), device=self.device)
        self.last_command_zero = torch.ones((self.num_envs,), device=self.device, dtype=torch.bool)

    def classify_command_zero(self, cmd_vel_b: torch.Tensor) -> torch.Tensor:
        is_small_x = torch.abs(cmd_vel_b[:, 0]) < self.lin_vel_x_clip
        is_small_y = torch.abs(cmd_vel_b[:, 1]) < self.lin_vel_y_clip
        is_small_yaw = torch.abs(cmd_vel_b[:, 2]) < self.ang_vel_yaw_clip
        return is_small_x & is_small_y & is_small_yaw

    def update(self, cmd_vel_b: torch.Tensor, base_force_est_local: torch.Tensor, *, dt: float) -> ForceModeSwitchState:
        dt = float(dt)
        command_zero = self.classify_command_zero(cmd_vel_b)

        force_xy = torch.linalg.norm(base_force_est_local[:, :2], dim=-1)
        force_xy = torch.where(torch.isfinite(force_xy), force_xy, torch.zeros_like(force_xy))
        self.last_force_xy = force_xy
        self.last_command_zero = command_zero

        command_walking_mask = ~command_zero
        if command_walking_mask.any():
            self.mode[command_walking_mask] = MODE_COMMAND_WALKING
            self.force_state[command_walking_mask] = MODE_STANDING
            self.force_lp[command_walking_mask] = 0.0
            self.enter_timer[command_walking_mask] = 0.0
            self.exit_timer[command_walking_mask] = 0.0

        zero_ids = torch.nonzero(command_zero, as_tuple=False).squeeze(-1)
        if zero_ids.numel() > 0:
            alpha = dt / max(self.lp_tau_s + dt, 1.0e-8)
            self.force_lp[zero_ids] = self.force_lp[zero_ids] + alpha * (force_xy[zero_ids] - self.force_lp[zero_ids])

            standing_ids = zero_ids[self.force_state[zero_ids] == MODE_STANDING]
            if standing_ids.numel() > 0:
                self.exit_timer[standing_ids] = 0.0
                enter_condition = self.force_lp[standing_ids] >= self.enter_th_n
                self.enter_timer[standing_ids] = torch.where(
                    enter_condition,
                    self.enter_timer[standing_ids] + dt,
                    torch.zeros_like(self.enter_timer[standing_ids]),
                )
                enter_ids = standing_ids[self.enter_timer[standing_ids] >= self.enter_hold_s]
                if enter_ids.numel() > 0:
                    self.force_state[enter_ids] = MODE_FORCE_WALKING
                    self.enter_timer[enter_ids] = 0.0
                    self.exit_timer[enter_ids] = 0.0

            walking_ids = zero_ids[self.force_state[zero_ids] == MODE_FORCE_WALKING]
            if walking_ids.numel() > 0:
                self.enter_timer[walking_ids] = 0.0
                exit_condition = self.force_lp[walking_ids] <= self.exit_th_n
                self.exit_timer[walking_ids] = torch.where(
                    exit_condition,
                    self.exit_timer[walking_ids] + dt,
                    torch.zeros_like(self.exit_timer[walking_ids]),
                )
                exit_ids = walking_ids[self.exit_timer[walking_ids] >= self.exit_hold_s]
                if exit_ids.numel() > 0:
                    self.force_state[exit_ids] = MODE_STANDING
                    self.enter_timer[exit_ids] = 0.0
                    self.exit_timer[exit_ids] = 0.0

        force_walking_mask = command_zero & (self.force_state == MODE_FORCE_WALKING)
        standing_mask = command_zero & ~force_walking_mask
        walking_mask = ~standing_mask

        self.mode[standing_mask] = MODE_STANDING
        self.mode[force_walking_mask] = MODE_FORCE_WALKING

        return ForceModeSwitchState(
            mode=self.mode.clone(),
            command_zero=command_zero.clone(),
            force_xy=force_xy.clone(),
            force_lp=self.force_lp.clone(),
            enter_timer=self.enter_timer.clone(),
            exit_timer=self.exit_timer.clone(),
            standing_mask=standing_mask.clone(),
            walking_mask=walking_mask.clone(),
        )


def mode_name(mode_id: int) -> str:
    return MODE_NAME_BY_ID.get(int(mode_id), f"unknown_{int(mode_id)}")


def override_standing_mask(env: ManagerBasedRLEnv, standing_mask: torch.Tensor) -> None:
    locomotion_term = env.command_manager.get_term("locomotion_commands")
    if not hasattr(locomotion_term, "is_standing_env"):
        raise AttributeError("locomotion_commands term does not expose is_standing_env.")
    locomotion_term.is_standing_env[:] = standing_mask.to(device=env.device, dtype=torch.bool)


def write_csv_rows(
    writer: csv.writer,
    *,
    step: int,
    time_s: float,
    num_envs: int,
    cmd_vel_b: torch.Tensor,
    base_lin_vel_local: torch.Tensor,
    estimator_outputs: dict[str, torch.Tensor],
    mode_state: ForceModeSwitchState,
    gait_phase: torch.Tensor,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
    force_target_n: torch.Tensor,
):
    base_force_est_local = estimator_outputs["base_force_est_local"]
    base_force_tgt_local = estimator_outputs["base_force_tgt_local"]
    for env_id in range(num_envs):
        mode_id = int(mode_state.mode[env_id].item())
        writer.writerow(
            [
                env_id,
                step,
                f"{time_s:.6f}",
                mode_id,
                mode_name(mode_id),
                int(mode_state.command_zero[env_id].item()),
                int(mode_state.standing_mask[env_id].item()),
                int(mode_state.walking_mask[env_id].item()),
                f"{cmd_vel_b[env_id, 0].item():.6f}",
                f"{cmd_vel_b[env_id, 1].item():.6f}",
                f"{cmd_vel_b[env_id, 2].item():.6f}",
                force_segment_name,
                force_direction_name,
                force_phase_name,
                f"{force_target_n[env_id].item():.6f}",
                f"{base_force_est_local[env_id, 0].item():.6f}",
                f"{base_force_est_local[env_id, 1].item():.6f}",
                f"{base_force_est_local[env_id, 2].item():.6f}",
                f"{base_force_tgt_local[env_id, 0].item():.6f}",
                f"{base_force_tgt_local[env_id, 1].item():.6f}",
                f"{base_force_tgt_local[env_id, 2].item():.6f}",
                f"{base_lin_vel_local[env_id, 0].item():.6f}",
                f"{base_lin_vel_local[env_id, 1].item():.6f}",
                f"{base_lin_vel_local[env_id, 2].item():.6f}",
                f"{mode_state.force_xy[env_id].item():.6f}",
                f"{mode_state.force_lp[env_id].item():.6f}",
                f"{mode_state.enter_timer[env_id].item():.6f}",
                f"{mode_state.exit_timer[env_id].item():.6f}",
                f"{gait_phase[env_id, 0].item():.6f}",
            ]
        )


def maybe_log_step(
    *,
    step: int,
    time_s: float,
    cmd_vel_b: torch.Tensor,
    mode_state: ForceModeSwitchState,
    gait_phase: torch.Tensor,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
) -> None:
    if args_cli.log_interval <= 0 or step % args_cli.log_interval != 0:
        return
    env_id = 0
    mode_id = int(mode_state.mode[env_id].item())
    print(
        f"[SMART_GAIT][step={step:05d}] "
        f"t={time_s:6.2f}s "
        f"segment={force_segment_name}/{force_direction_name}/{force_phase_name} "
        f"mode={mode_name(mode_id)} "
        f"command_zero={bool(mode_state.command_zero[env_id].item())} "
        f"standing_mask={bool(mode_state.standing_mask[env_id].item())} "
        f"cmd=({cmd_vel_b[env_id, 0].item():+.3f}, {cmd_vel_b[env_id, 1].item():+.3f}, {cmd_vel_b[env_id, 2].item():+.3f}) "
        f"F_xy={mode_state.force_xy[env_id].item():.3f}N "
        f"F_lp={mode_state.force_lp[env_id].item():.3f}N "
        f"enter_t={mode_state.enter_timer[env_id].item():.3f}s "
        f"exit_t={mode_state.exit_timer[env_id].item():.3f}s "
        f"gait_phase={gait_phase[env_id, 0].item():.3f}"
    )


def main():
    output_csv = resolve_output_csv(args_cli.output_csv)
    env_cfg = build_env_cfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    gripper_force_obs_scale = float(env_cfg.observations.pred.gripper_forces_local.scale)
    base_force_obs_scale = float(env_cfg.observations.pred.base_forces_local.scale)

    checkpoint_path = retrieve_file_path(args_cli.checkpoint) if args_cli.checkpoint else None
    jit_path = retrieve_file_path(args_cli.policy_jit) if args_cli.policy_jit else None

    wrapped_env = None
    estimator_model = None
    if checkpoint_path:
        print(f"[INFO] Loading raw checkpoint: {checkpoint_path}")
        policy, wrapped_env, estimator_model = load_raw_checkpoint_policy(checkpoint_path, env)
        env.reset()
        obs = wrapped_env.get_observations()
    elif jit_path:
        raise RuntimeError(
            "This task uses AsymmetricActorCritic with a history encoder, but IsaacLab's stock "
            "export_policy_as_jit() only exports policy.actor and drops the encoder/adaptation path. "
            "As a result, the exported policy.pt is not a faithful inference artifact for this task. "
            "Use --checkpoint for smart_gait, or add a custom exporter that wraps the full act_inference() path."
        )
    else:
        raise RuntimeError("Either --checkpoint or --policy-jit must be provided.")

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError("commands.base_force_ext must be a scripted force term exposing set_force()/clear_force().")
    markers = SmartGaitMarkers(env)

    scheduler = None
    if args_cli.force_schedule == FORCE_SCHEDULE_DEMO:
        scheduler = ScriptedForceScheduler(
            num_envs=env.num_envs,
            device=env.device,
            segments=build_demo_force_segments(float(args_cli.episode_length_s)),
        )

    locomotion_cfg = env_cfg.commands.locomotion_commands
    switcher = ForceModeSwitcher(
        num_envs=env.num_envs,
        device=env.device,
        lin_vel_x_clip=float(locomotion_cfg.lin_vel_x_clip),
        lin_vel_y_clip=float(locomotion_cfg.lin_vel_y_clip),
        ang_vel_yaw_clip=float(locomotion_cfg.ang_vel_yaw_clip),
        lp_tau_s=float(args_cli.force_lp_tau_s),
        enter_th_n=float(args_cli.enter_threshold_n),
        exit_th_n=float(args_cli.exit_threshold_n),
        enter_hold_s=float(args_cli.enter_hold_s),
        exit_hold_s=float(args_cli.exit_hold_s),
    )

    num_envs = env.num_envs
    device = env.device
    step_dt = float(env.step_dt)
    max_steps = max(int(round(float(env_cfg.episode_length_s) / step_dt)), 1)
    mode_counts = torch.zeros((num_envs, 3), device=device, dtype=torch.long)

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "env_id",
                "step",
                "time_s",
                "mode_id",
                "mode_name",
                "command_zero",
                "standing_mask",
                "walking_mask",
                "cmd_vx",
                "cmd_vy",
                "cmd_wz",
                "force_segment_name",
                "force_direction_name",
                "force_phase_name",
                "scheduled_force_target_n",
                "base_force_est_local_x",
                "base_force_est_local_y",
                "base_force_est_local_z",
                "base_force_tgt_local_x",
                "base_force_tgt_local_y",
                "base_force_tgt_local_z",
                "base_lin_vel_local_x",
                "base_lin_vel_local_y",
                "base_lin_vel_local_z",
                "base_force_est_xy",
                "force_lp",
                "enter_timer",
                "exit_timer",
                "gait_phase",
            ]
        )

        for step in range(max_steps):
            if not simulation_app.is_running():
                break

            time_s = step * step_dt
            robot = env.scene["robot"]
            cmd_vel_b = env.command_manager.get_command("locomotion_commands").clone()
            base_lin_vel_local = robot.data.root_lin_vel_b.clone()

            if scheduler is None:
                scheduler_outputs = {
                    "segment_name": FORCE_SCHEDULE_NONE,
                    "direction_name": DIR_NONE,
                    "phase_name": "idle",
                    "force_target_n": torch.zeros((num_envs,), device=device),
                    "force_w": torch.zeros((num_envs, 3), device=device),
                    "axis_xy": torch.full((num_envs, 2), float("nan"), device=device),
                    "is_active": False,
                }
            else:
                scheduler_outputs = scheduler.update(env, time_s)

            force_term.clear_force()
            if scheduler_outputs["is_active"]:
                force_term.set_force(scheduler_outputs["force_w"])

            with torch.inference_mode():
                estimator_outputs = extract_force_estimator_outputs(
                    obs,
                    estimator_model,
                    gripper_force_scale=gripper_force_obs_scale,
                    base_force_scale=base_force_obs_scale,
                )

            mode_state = switcher.update(
                cmd_vel_b,
                estimator_outputs["base_force_est_local"],
                dt=step_dt,
            )
            override_standing_mask(env, mode_state.standing_mask)

            for mode_id in range(3):
                mode_counts[:, mode_id] += (mode_state.mode == mode_id).to(dtype=torch.long)

            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = wrapped_env.step(actions)

            gait_phase = env.command_manager.get_command("gait_phase").clone()
            write_csv_rows(
                writer,
                step=step,
                time_s=time_s,
                num_envs=num_envs,
                cmd_vel_b=cmd_vel_b,
                base_lin_vel_local=base_lin_vel_local,
                estimator_outputs=estimator_outputs,
                mode_state=mode_state,
                gait_phase=gait_phase,
                force_segment_name=str(scheduler_outputs["segment_name"]),
                force_direction_name=str(scheduler_outputs["direction_name"]),
                force_phase_name=str(scheduler_outputs["phase_name"]),
                force_target_n=scheduler_outputs["force_target_n"],
            )
            maybe_log_step(
                step=step,
                time_s=time_s,
                cmd_vel_b=cmd_vel_b,
                mode_state=mode_state,
                gait_phase=gait_phase,
                force_segment_name=str(scheduler_outputs["segment_name"]),
                force_direction_name=str(scheduler_outputs["direction_name"]),
                force_phase_name=str(scheduler_outputs["phase_name"]),
            )

            markers.update()

            done_mask = dones.to(dtype=torch.bool, device=device)
            if done_mask.any():
                break

    force_term.clear_force()
    print(f"[INFO] CSV written to: {output_csv}")
    for env_id in range(num_envs):
        standing_s = mode_counts[env_id, MODE_STANDING].item() * step_dt
        force_walking_s = mode_counts[env_id, MODE_FORCE_WALKING].item() * step_dt
        command_walking_s = mode_counts[env_id, MODE_COMMAND_WALKING].item() * step_dt
        print(
            f"[SUMMARY][env {env_id}] "
            f"standing={standing_s:.2f}s "
            f"force_walking={force_walking_s:.2f}s "
            f"command_walking={command_walking_s:.2f}s"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
