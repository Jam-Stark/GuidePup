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


EPISODE_LENGTH_S = 60.0
LONG_RESAMPLING_S = 1.0e9
FORCE_INTERP_LINEAR = "linear"
FORCE_INTERP_QUADRATIC = "quadratic"

DIR_NONE = "none"
DIR_FORWARD = "forward"
DIR_LEFT = "left"
DIR_BACKWARD = "backward"
ACTIVE_DIRECTIONS = (DIR_FORWARD, DIR_LEFT, DIR_BACKWARD)
DIRECTION_INDEX = {name: idx for idx, name in enumerate(ACTIVE_DIRECTIONS)}


@dataclass(frozen=True)
class StageSegment:
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


STAGE_SEGMENTS = (
    StageSegment("idle_0", DIR_NONE, "idle", 0.0, 5.0),
    StageSegment("forward_low_10", DIR_FORWARD, "low", 5.0, 6.0, 10.0, 10.0, capture_axis=True),
    StageSegment("forward_low_15", DIR_FORWARD, "low", 6.0, 7.0, 15.0, 15.0),
    StageSegment("forward_low_5", DIR_FORWARD, "low", 7.0, 8.0, 5.0, 5.0),
    StageSegment(
        "forward_ramp_quad",
        DIR_FORWARD,
        "ramp",
        8.0,
        11.0,
        10.0,
        40.0,
        force_interp=FORCE_INTERP_QUADRATIC,
    ),
    StageSegment("forward_high", DIR_FORWARD, "high", 11.0, 13.0, 40.0, 40.0),
    StageSegment("idle_1", DIR_NONE, "idle", 13.0, 20.0),
    StageSegment("left_low_10", DIR_LEFT, "low", 20.0, 21.0, 10.0, 10.0, capture_axis=True),
    StageSegment("left_low_15", DIR_LEFT, "low", 21.0, 22.0, 15.0, 15.0),
    StageSegment("left_low_5", DIR_LEFT, "low", 22.0, 23.0, 5.0, 5.0),
    StageSegment(
        "left_ramp_quad",
        DIR_LEFT,
        "ramp",
        23.0,
        26.0,
        10.0,
        40.0,
        force_interp=FORCE_INTERP_QUADRATIC,
    ),
    StageSegment("left_high", DIR_LEFT, "high", 26.0, 28.0, 40.0, 40.0),
    StageSegment("idle_2", DIR_NONE, "idle", 28.0, 40.0),
    StageSegment("backward_low_10", DIR_BACKWARD, "low", 40.0, 41.0, 10.0, 10.0, capture_axis=True),
    StageSegment("backward_low_15", DIR_BACKWARD, "low", 41.0, 42.0, 15.0, 15.0),
    StageSegment("backward_low_5", DIR_BACKWARD, "low", 42.0, 43.0, 5.0, 5.0),
    StageSegment(
        "backward_ramp_quad",
        DIR_BACKWARD,
        "ramp",
        43.0,
        46.0,
        10.0,
        40.0,
        force_interp=FORCE_INTERP_QUADRATIC,
    ),
    StageSegment("backward_high", DIR_BACKWARD, "high", 46.0, 48.0, 40.0, 40.0),
    StageSegment("idle_3", DIR_NONE, "idle", 48.0, EPISODE_LENGTH_S),
)


parser = argparse.ArgumentParser(
    description=(
        "Evaluate a UniFP Aliengo policy under a deterministic 60s standing staged force schedule with "
        "quadratic 3s force ramps."
    )
)
policy_group = parser.add_mutually_exclusive_group(required=True)
policy_group.add_argument("--checkpoint", type=str, help="Path to an RSL-RL checkpoint.")
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
parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

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
        output_path = (
            Path("logs")
            / "experiments"
            / "unifp_staged_direction_force_ref_standing"
            / f"staged_direction_force_ref_standing_{stamp}.csv"
        ).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = EPISODE_LENGTH_S
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env_cfg.commands.locomotion_commands.resampling_time_range = (LONG_RESAMPLING_S, LONG_RESAMPLING_S)
    env_cfg.commands.locomotion_commands.rel_standing_envs = 1.0
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (0.0, 0.0)
    env_cfg.commands.locomotion_commands.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.locomotion_commands.ranges.ang_vel_z = (0.0, 0.0)

    # Isaac Lab managers skip None-valued terms during preparation, which disables early termination here.
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
    zeros = torch.zeros_like(forward_axis_xy)
    return zeros


def finite_or_nan(value: float) -> float:
    return value if math.isfinite(value) else float("nan")


def format_optional(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.3f}"


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


class StagedDirectionForceMarkers:
    """Standalone visualization markers for command/current velocity and base force."""

    def __init__(self, env: ManagerBasedRLEnv):
        self.env = env
        self.device = env.device
        self.cmd_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/StagedDirectionForce/cmd_velocity",
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
                prim_path="/Visuals/StagedDirectionForce/current_velocity",
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
                prim_path="/Visuals/StagedDirectionForce/base_force_ext",
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


class StagedDirectionForceScheduler:
    """Generate the deterministic staged-direction force schedule."""

    def __init__(self, num_envs: int, device: torch.device | str):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.current_segment_name: str | None = None
        self.current_axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)
        self.current_block_start_root_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

    def segment_for_time(self, time_s: float) -> StageSegment:
        if time_s >= STAGE_SEGMENTS[-1].start_s:
            return STAGE_SEGMENTS[-1]
        for segment in STAGE_SEGMENTS:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return STAGE_SEGMENTS[-1]

    def update(self, env: ManagerBasedRLEnv, time_s: float) -> dict[str, torch.Tensor | float | str]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name

        if segment_changed and segment.capture_axis:
            forward_axis_xy = base_forward_axis_xy(env)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)
            robot = env.scene["robot"]
            self.current_block_start_root_xy = robot.data.root_pos_w[:, :2].clone()

        force_target_n = torch.full((self.num_envs,), segment.force_target_at(time_s), device=self.device)
        force_w = torch.zeros((self.num_envs, 3), device=self.device)

        if segment.is_active:
            axis_xy = self.current_axis_xy.clone()
            block_start_root_xy = self.current_block_start_root_xy.clone()
            force_w[:, :2] = axis_xy * force_target_n.unsqueeze(-1)
        else:
            axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)
            block_start_root_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

        self.current_segment_name = segment.segment_name
        return {
            "segment_name": segment.segment_name,
            "direction_name": segment.direction_name,
            "phase_name": segment.phase_name,
            "force_target_n": force_target_n,
            "force_w": force_w,
            "axis_xy": axis_xy,
            "block_start_root_xy": block_start_root_xy,
            "is_active": segment.is_active,
        }


def write_csv_rows(
    writer: csv.writer,
    *,
    step: int,
    time_s: float,
    num_envs: int,
    segment_name: str,
    direction_name: str,
    phase_name: str,
    cmd_vel_b: torch.Tensor,
    force_target_n: torch.Tensor,
    force_w: torch.Tensor,
    axis_xy: torch.Tensor,
    proj_speed: torch.Tensor,
    cross_speed: torch.Tensor,
    proj_disp: torch.Tensor,
    cross_disp: torch.Tensor,
    root_pos_xy: torch.Tensor,
    estimator_outputs: dict[str, torch.Tensor],
):
    gripper_force_est_local = estimator_outputs["gripper_force_est_local"]
    gripper_force_tgt_local = estimator_outputs["gripper_force_tgt_local"]
    gripper_force_err_l2 = estimator_outputs["gripper_force_err_l2"]
    base_force_est_local = estimator_outputs["base_force_est_local"]
    base_force_tgt_local = estimator_outputs["base_force_tgt_local"]
    base_force_err_l2 = estimator_outputs["base_force_err_l2"]

    for env_id in range(num_envs):
        writer.writerow(
            [
                env_id,
                step,
                f"{time_s:.6f}",
                segment_name,
                direction_name,
                phase_name,
                f"{cmd_vel_b[env_id, 0].item():.6f}",
                f"{cmd_vel_b[env_id, 1].item():.6f}",
                f"{cmd_vel_b[env_id, 2].item():.6f}",
                f"{force_target_n[env_id].item():.6f}",
                f"{force_w[env_id, 0].item():.6f}",
                f"{force_w[env_id, 1].item():.6f}",
                f"{axis_xy[env_id, 0].item():.6f}",
                f"{axis_xy[env_id, 1].item():.6f}",
                f"{proj_speed[env_id].item():.6f}",
                f"{cross_speed[env_id].item():.6f}",
                f"{proj_disp[env_id].item():.6f}",
                f"{cross_disp[env_id].item():.6f}",
                f"{root_pos_xy[env_id, 0].item():.6f}",
                f"{root_pos_xy[env_id, 1].item():.6f}",
                f"{gripper_force_est_local[env_id, 0].item():.6f}",
                f"{gripper_force_est_local[env_id, 1].item():.6f}",
                f"{gripper_force_est_local[env_id, 2].item():.6f}",
                f"{gripper_force_tgt_local[env_id, 0].item():.6f}",
                f"{gripper_force_tgt_local[env_id, 1].item():.6f}",
                f"{gripper_force_tgt_local[env_id, 2].item():.6f}",
                f"{gripper_force_err_l2[env_id].item():.6f}",
                f"{base_force_est_local[env_id, 0].item():.6f}",
                f"{base_force_est_local[env_id, 1].item():.6f}",
                f"{base_force_est_local[env_id, 2].item():.6f}",
                f"{base_force_tgt_local[env_id, 0].item():.6f}",
                f"{base_force_tgt_local[env_id, 1].item():.6f}",
                f"{base_force_tgt_local[env_id, 2].item():.6f}",
                f"{base_force_err_l2[env_id].item():.6f}",
            ]
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
            "Use --checkpoint for staged_direction_force_eval, or add a custom exporter that wraps the full "
            "act_inference() path."
        )
    else:
        raise RuntimeError("Either --checkpoint or --policy-jit must be provided.")

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError("commands.base_force_ext must be a scripted force term exposing set_force()/clear_force().")
    markers = StagedDirectionForceMarkers(env)
    scheduler = StagedDirectionForceScheduler(num_envs=env.num_envs, device=env.device)

    if livestream_enabled:
        print("[INFO] LIVESTREAM enabled; cameras forced on for WebRTC compatibility.")

    num_envs = env.num_envs
    device = env.device
    step_dt = float(env.step_dt)
    max_steps = max(int(round(float(env_cfg.episode_length_s) / step_dt)), 1)

    direction_count = len(ACTIVE_DIRECTIONS)
    block_last_proj_disp = torch.full((num_envs, direction_count), float("nan"), device=device)
    block_last_cross_disp = torch.full((num_envs, direction_count), float("nan"), device=device)
    block_peak_abs_cross_speed = torch.full((num_envs, direction_count), float("-inf"), device=device)
    active_force_sample_count = torch.zeros(num_envs, dtype=torch.long, device=device)
    base_force_err_active_sum = torch.zeros(num_envs, device=device)
    base_force_err_active_max = torch.zeros(num_envs, device=device)
    gripper_force_err_active_sum = torch.zeros(num_envs, device=device)
    gripper_force_err_active_max = torch.zeros(num_envs, device=device)

    ended_early = False
    end_step = max_steps
    last_logged_payload: dict[str, object] | None = None

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "env_id",
                "step",
                "time_s",
                "segment_name",
                "direction_name",
                "phase_name",
                "cmd_vx",
                "cmd_vy",
                "cmd_wz",
                "force_target_n",
                "force_x",
                "force_y",
                "axis_x",
                "axis_y",
                "proj_speed",
                "cross_speed",
                "proj_disp",
                "cross_disp",
                "root_x",
                "root_y",
                "gripper_force_est_local_x",
                "gripper_force_est_local_y",
                "gripper_force_est_local_z",
                "gripper_force_tgt_local_x",
                "gripper_force_tgt_local_y",
                "gripper_force_tgt_local_z",
                "gripper_force_err_l2",
                "base_force_est_local_x",
                "base_force_est_local_y",
                "base_force_est_local_z",
                "base_force_tgt_local_x",
                "base_force_tgt_local_y",
                "base_force_tgt_local_z",
                "base_force_err_l2",
            ]
        )

        for step in range(max_steps):
            if not simulation_app.is_running():
                ended_early = True
                end_step = step
                break

            time_s = step * step_dt
            robot = env.scene["robot"]
            root_pos_xy = robot.data.root_pos_w[:, :2].clone()
            root_vel_xy = robot.data.root_lin_vel_w[:, :2].clone()
            cmd_vel_b = env.command_manager.get_command("locomotion_commands").clone()

            scheduler_outputs = scheduler.update(env, time_s)
            force_term.clear_force()
            if scheduler_outputs["is_active"]:
                force_term.set_force(scheduler_outputs["force_w"])

            axis_xy = scheduler_outputs["axis_xy"]
            block_start_root_xy = scheduler_outputs["block_start_root_xy"]
            proj_speed = torch.full((num_envs,), float("nan"), device=device)
            cross_speed = torch.full((num_envs,), float("nan"), device=device)
            proj_disp = torch.full((num_envs,), float("nan"), device=device)
            cross_disp = torch.full((num_envs,), float("nan"), device=device)

            if scheduler_outputs["is_active"]:
                cross_axis_xy = torch.stack([-axis_xy[:, 1], axis_xy[:, 0]], dim=-1)
                proj_speed = torch.sum(root_vel_xy * axis_xy, dim=-1)
                cross_speed = torch.sum(root_vel_xy * cross_axis_xy, dim=-1)
                rel_pos_xy = root_pos_xy - block_start_root_xy
                proj_disp = torch.sum(rel_pos_xy * axis_xy, dim=-1)
                cross_disp = torch.sum(rel_pos_xy * cross_axis_xy, dim=-1)

            with torch.inference_mode():
                estimator_outputs = extract_force_estimator_outputs(
                    obs,
                    estimator_model,
                    gripper_force_scale=gripper_force_obs_scale,
                    base_force_scale=base_force_obs_scale,
                )

            write_csv_rows(
                writer,
                step=step,
                time_s=time_s,
                num_envs=num_envs,
                segment_name=scheduler_outputs["segment_name"],
                direction_name=scheduler_outputs["direction_name"],
                phase_name=scheduler_outputs["phase_name"],
                cmd_vel_b=cmd_vel_b,
                force_target_n=scheduler_outputs["force_target_n"],
                force_w=scheduler_outputs["force_w"],
                axis_xy=axis_xy,
                proj_speed=proj_speed,
                cross_speed=cross_speed,
                proj_disp=proj_disp,
                cross_disp=cross_disp,
                root_pos_xy=root_pos_xy,
                estimator_outputs=estimator_outputs,
            )

            last_logged_payload = {
                "step": step,
                "time_s": time_s,
                "cmd_vel_b": cmd_vel_b.clone(),
                "root_pos_xy": root_pos_xy.clone(),
                "estimator_outputs": {
                    key: value.clone() for key, value in estimator_outputs.items()
                },
            }

            if scheduler_outputs["is_active"]:
                direction_idx = DIRECTION_INDEX[scheduler_outputs["direction_name"]]
                block_last_proj_disp[:, direction_idx] = proj_disp
                block_last_cross_disp[:, direction_idx] = cross_disp
                block_peak_abs_cross_speed[:, direction_idx] = torch.maximum(
                    block_peak_abs_cross_speed[:, direction_idx],
                    torch.abs(cross_speed),
                )
                active_force_sample_count += 1
                base_force_err_active_sum += estimator_outputs["base_force_err_l2"]
                base_force_err_active_max = torch.maximum(
                    base_force_err_active_max,
                    estimator_outputs["base_force_err_l2"],
                )
                gripper_force_err_active_sum += estimator_outputs["gripper_force_err_l2"]
                gripper_force_err_active_max = torch.maximum(
                    gripper_force_err_active_max,
                    estimator_outputs["gripper_force_err_l2"],
                )

            markers.update()

            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = wrapped_env.step(actions)
                done_mask = dones.to(dtype=torch.bool, device=device)

            if done_mask.any():
                end_step = step + 1
                if end_step < max_steps:
                    ended_early = True
                break

        if last_logged_payload is not None and last_logged_payload["time_s"] < EPISODE_LENGTH_S:
            final_segment = scheduler.segment_for_time(EPISODE_LENGTH_S)
            nan_axis_xy = torch.full((num_envs, 2), float("nan"), device=device)
            nan_series = torch.full((num_envs,), float("nan"), device=device)
            zero_force = torch.zeros((num_envs, 3), device=device)
            zero_force_mag = torch.zeros((num_envs,), device=device)
            write_csv_rows(
                writer,
                step=max_steps,
                time_s=EPISODE_LENGTH_S,
                num_envs=num_envs,
                segment_name=final_segment.segment_name,
                direction_name=final_segment.direction_name,
                phase_name=final_segment.phase_name,
                cmd_vel_b=last_logged_payload["cmd_vel_b"],
                force_target_n=zero_force_mag,
                force_w=zero_force,
                axis_xy=nan_axis_xy,
                proj_speed=nan_series,
                cross_speed=nan_series,
                proj_disp=nan_series,
                cross_disp=nan_series,
                root_pos_xy=last_logged_payload["root_pos_xy"],
                estimator_outputs=last_logged_payload["estimator_outputs"],
            )

    force_term.clear_force()

    print(f"[INFO] CSV written to: {output_csv}")
    if ended_early:
        print(f"[WARN] Evaluation ended before the planned timeout-only horizon at step={end_step}.")
    else:
        print(f"[INFO] Evaluation reached the planned timeout horizon at step={end_step}.")

    for env_id in range(num_envs):
        active_samples = int(active_force_sample_count[env_id].item())
        base_force_err_active_mean = (
            (base_force_err_active_sum[env_id] / active_force_sample_count[env_id]).item()
            if active_samples > 0
            else float("nan")
        )
        gripper_force_err_active_mean = (
            (gripper_force_err_active_sum[env_id] / active_force_sample_count[env_id]).item()
            if active_samples > 0
            else float("nan")
        )
        base_force_err_peak = base_force_err_active_max[env_id].item() if active_samples > 0 else float("nan")
        gripper_force_err_peak = (
            gripper_force_err_active_max[env_id].item() if active_samples > 0 else float("nan")
        )

        forward_proj_disp = finite_or_nan(block_last_proj_disp[env_id, DIRECTION_INDEX[DIR_FORWARD]].item())
        left_proj_disp = finite_or_nan(block_last_proj_disp[env_id, DIRECTION_INDEX[DIR_LEFT]].item())
        backward_proj_disp = finite_or_nan(block_last_proj_disp[env_id, DIRECTION_INDEX[DIR_BACKWARD]].item())

        forward_cross_disp = finite_or_nan(block_last_cross_disp[env_id, DIRECTION_INDEX[DIR_FORWARD]].item())
        left_cross_disp = finite_or_nan(block_last_cross_disp[env_id, DIRECTION_INDEX[DIR_LEFT]].item())
        backward_cross_disp = finite_or_nan(block_last_cross_disp[env_id, DIRECTION_INDEX[DIR_BACKWARD]].item())

        forward_cross_speed_peak = finite_or_nan(
            block_peak_abs_cross_speed[env_id, DIRECTION_INDEX[DIR_FORWARD]].item()
        )
        left_cross_speed_peak = finite_or_nan(block_peak_abs_cross_speed[env_id, DIRECTION_INDEX[DIR_LEFT]].item())
        backward_cross_speed_peak = finite_or_nan(
            block_peak_abs_cross_speed[env_id, DIRECTION_INDEX[DIR_BACKWARD]].item()
        )

        if forward_cross_speed_peak < 0.0:
            forward_cross_speed_peak = float("nan")
        if left_cross_speed_peak < 0.0:
            left_cross_speed_peak = float("nan")
        if backward_cross_speed_peak < 0.0:
            backward_cross_speed_peak = float("nan")

        print(
            f"[SUMMARY][env {env_id}] "
            f"forward_proj_disp={format_optional(forward_proj_disp)} "
            f"forward_cross_disp={format_optional(forward_cross_disp)} "
            f"forward_cross_speed_peak={format_optional(forward_cross_speed_peak)} "
            f"left_proj_disp={format_optional(left_proj_disp)} "
            f"left_cross_disp={format_optional(left_cross_disp)} "
            f"left_cross_speed_peak={format_optional(left_cross_speed_peak)} "
            f"backward_proj_disp={format_optional(backward_proj_disp)} "
            f"backward_cross_disp={format_optional(backward_cross_disp)} "
            f"backward_cross_speed_peak={format_optional(backward_cross_speed_peak)} "
            f"base_force_err_active_mean={format_optional(base_force_err_active_mean)} "
            f"base_force_err_peak={format_optional(base_force_err_peak)} "
            f"gripper_force_err_active_mean={format_optional(gripper_force_err_active_mean)} "
            f"gripper_force_err_peak={format_optional(gripper_force_err_peak)}"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
