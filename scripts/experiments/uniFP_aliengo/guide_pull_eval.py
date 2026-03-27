#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
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


PHASE_IDLE = 0
PHASE_DISTURB = 1
PHASE_RAMP = 2
PHASE_PEAK = 3
PHASE_RECOVER = 4
PHASE_NAMES = ("idle", "disturb", "ramp", "peak", "recover")


parser = argparse.ArgumentParser(
    description="Evaluate a UniFP Aliengo policy under a staged guide-pull disturbance."
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
parser.add_argument(
    "--episode-length-s",
    type=float,
    default=20.0,
    help="Episode length in seconds. Default matches the staged guide-pull design.",
)
parser.add_argument(
    "--velocity-min",
    type=float,
    default=0.8,
    help="Minimum forward velocity command sampled once at reset.",
)
parser.add_argument(
    "--velocity-max",
    type=float,
    default=1.0,
    help="Maximum forward velocity command sampled once at reset.",
)
parser.add_argument("--disturb-start-s", type=float, default=3.0, help="Time when the continuous guide pull starts.")
parser.add_argument("--ramp-start-s", type=float, default=10.0, help="Time when the force ramp-up starts.")
parser.add_argument("--ramp-end-s", type=float, default=11.0, help="Time when the ramp-up reaches the peak force.")
parser.add_argument("--peak-end-s", type=float, default=16.0, help="Time when the peak-force hold ends.")
parser.add_argument(
    "--force-mean-n",
    type=float,
    default=12.0,
    help="Mean force magnitude during the continuous disturbance phases.",
)
parser.add_argument(
    "--force-range-n",
    type=float,
    default=3.0,
    help="Half-range around the mean force magnitude during the continuous disturbance phases.",
)
parser.add_argument("--peak-force-n", type=float, default=30.0, help="Peak force magnitude during the hold stage.")
parser.add_argument(
    "--angle-min-deg",
    type=float,
    default=5.0,
    help="Minimum absolute angular disturbance around the reverse direction.",
)
parser.add_argument(
    "--angle-max-deg",
    type=float,
    default=15.0,
    help="Maximum absolute angular disturbance around the reverse direction.",
)
parser.add_argument(
    "--resample-interval-min-s",
    type=float,
    default=0.25,
    help="Minimum interval between random target updates for force magnitude/direction.",
)
parser.add_argument(
    "--resample-interval-max-s",
    type=float,
    default=0.60,
    help="Maximum interval between random target updates for force magnitude/direction.",
)
parser.add_argument(
    "--mag-smoothing-tau-s",
    type=float,
    default=0.30,
    help="Time constant for smoothing force magnitude in the disturbance/recovery phases.",
)
parser.add_argument(
    "--angle-smoothing-tau-s",
    type=float,
    default=0.20,
    help="Time constant for smoothing force-direction perturbations.",
)
parser.add_argument(
    "--direction-vel-threshold",
    type=float,
    default=0.08,
    help="Velocity threshold below which the script falls back to the robot forward axis.",
)
parser.add_argument(
    "--recovery-threshold-ratio",
    type=float,
    default=0.9,
    help="Recovery is detected when forward speed reaches this ratio of the sampled command speed.",
)
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
        output_path = (Path("logs") / "experiments" / "unifp_guide_pull" / f"guide_pull_{stamp}.csv").resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    v_min = min(float(args_cli.velocity_min), float(args_cli.velocity_max))
    v_max = max(float(args_cli.velocity_min), float(args_cli.velocity_max))
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (v_min, v_max)
    env_cfg.commands.locomotion_commands.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.locomotion_commands.ranges.ang_vel_z = (0.0, 0.0)
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


def base_forward_axis_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    yaw_quat = get_base_yaw_quat(env)
    local_forward = torch.zeros((env.num_envs, 3), device=env.device)
    local_forward[:, 0] = 1.0
    forward_w = base_yaw_to_world(local_forward, yaw_quat)
    forward_xy = forward_w[:, :2]
    return normalize_xy(forward_xy)


def normalize_xy(vectors_xy: torch.Tensor, eps: float = 1.0e-6) -> torch.Tensor:
    norms = torch.linalg.norm(vectors_xy, dim=-1, keepdim=True)
    safe = torch.zeros_like(vectors_xy)
    safe[:, 0] = 1.0
    return torch.where(norms > eps, vectors_xy / torch.clamp(norms, min=eps), safe)


def rotate_xy(vectors_xy: torch.Tensor, angles_rad: torch.Tensor) -> torch.Tensor:
    cos_a = torch.cos(angles_rad)
    sin_a = torch.sin(angles_rad)
    x = vectors_xy[:, 0]
    y = vectors_xy[:, 1]
    return torch.stack([cos_a * x - sin_a * y, sin_a * x + cos_a * y], dim=-1)


def format_optional(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.3f}"


def metric_mean(sum_tensor: torch.Tensor, count_tensor: torch.Tensor, env_id: int, phase_idx: int) -> float:
    count = int(count_tensor[env_id, phase_idx].item())
    if count <= 0:
        return float("nan")
    return (sum_tensor[env_id, phase_idx] / count_tensor[env_id, phase_idx]).item()


def finite_min_or_nan(value: float) -> float:
    return value if math.isfinite(value) else float("nan")


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


class GuidePullMarkers:
    """Standalone visualization markers for command/current velocity and base force."""

    def __init__(self, env: ManagerBasedRLEnv):
        self.env = env
        self.device = env.device
        self.cmd_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/GuidePull/cmd_velocity",
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
                prim_path="/Visuals/GuidePull/current_velocity",
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
                prim_path="/Visuals/GuidePull/base_force_ext",
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


class GuidePullScheduler:
    """Generate staged guide-pull forces with smooth stochastic disturbances."""

    def __init__(self, num_envs: int, device: torch.device | str, step_dt: float):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.step_dt = float(step_dt)
        self.all_env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)

        self.disturb_start_step = self._seconds_to_step(args_cli.disturb_start_s)
        self.ramp_start_step = self._seconds_to_step(args_cli.ramp_start_s)
        self.ramp_end_step = self._seconds_to_step(args_cli.ramp_end_s)
        self.peak_end_step = self._seconds_to_step(args_cli.peak_end_s)

        self.force_mean_n = float(args_cli.force_mean_n)
        self.force_range_n = max(float(args_cli.force_range_n), 0.0)
        self.force_low_n = max(self.force_mean_n - self.force_range_n, 0.0)
        self.force_high_n = max(self.force_mean_n + self.force_range_n, self.force_low_n)
        self.peak_force_n = float(args_cli.peak_force_n)

        self.angle_min_deg = min(abs(float(args_cli.angle_min_deg)), abs(float(args_cli.angle_max_deg)))
        self.angle_max_deg = max(abs(float(args_cli.angle_min_deg)), abs(float(args_cli.angle_max_deg)))
        self.resample_min_step = max(self._seconds_to_step(args_cli.resample_interval_min_s), 1)
        self.resample_max_step = max(self._seconds_to_step(args_cli.resample_interval_max_s), self.resample_min_step)
        self.mag_alpha = self._time_constant_to_alpha(args_cli.mag_smoothing_tau_s)
        self.angle_alpha = self._time_constant_to_alpha(args_cli.angle_smoothing_tau_s)

        self.next_resample_step = torch.full(
            (self.num_envs,), self.disturb_start_step, dtype=torch.long, device=self.device
        )
        self.mag_target = torch.full((self.num_envs,), self.force_mean_n, device=self.device)
        self.mag_state = torch.zeros((self.num_envs,), device=self.device)
        self.angle_target = torch.zeros((self.num_envs,), device=self.device)
        self.angle_state = torch.zeros((self.num_envs,), device=self.device)
        self.ramp_start_mag = torch.zeros((self.num_envs,), device=self.device)
        self.prev_phase = PHASE_IDLE

    def phase_index_for_step(self, step: int) -> int:
        time_s = float(step) * self.step_dt
        if time_s < float(args_cli.disturb_start_s):
            return PHASE_IDLE
        if time_s < float(args_cli.ramp_start_s):
            return PHASE_DISTURB
        if time_s < float(args_cli.ramp_end_s):
            return PHASE_RAMP
        if time_s < float(args_cli.peak_end_s):
            return PHASE_PEAK
        return PHASE_RECOVER

    def phase_name_for_step(self, step: int) -> str:
        return PHASE_NAMES[self.phase_index_for_step(step)]

    def update(self, step: int, motion_dir_xy: torch.Tensor) -> dict[str, torch.Tensor | str | int]:
        phase_idx = self.phase_index_for_step(step)
        phase_changed = phase_idx != self.prev_phase

        if phase_changed:
            if phase_idx == PHASE_DISTURB:
                self._resample_targets(self.all_env_ids, step, resample_magnitude=True, resample_angle=True)
                self.mag_state.copy_(self.mag_target)
                self.angle_state.copy_(self.angle_target)
            elif phase_idx == PHASE_RAMP:
                self.ramp_start_mag.copy_(self.mag_state)
                self._resample_targets(self.all_env_ids, step, resample_magnitude=False, resample_angle=True)
            elif phase_idx == PHASE_PEAK:
                self.mag_state.fill_(self.peak_force_n)
                self._resample_targets(self.all_env_ids, step, resample_magnitude=False, resample_angle=True)
            elif phase_idx == PHASE_RECOVER:
                self._resample_targets(self.all_env_ids, step, resample_magnitude=True, resample_angle=True)

        if phase_idx != PHASE_IDLE:
            due_ids = self.all_env_ids[self.next_resample_step <= step]
            if due_ids.numel() > 0:
                self._resample_targets(
                    due_ids,
                    step,
                    resample_magnitude=phase_idx in (PHASE_DISTURB, PHASE_RECOVER),
                    resample_angle=True,
                )

        if phase_idx == PHASE_IDLE:
            self.mag_state.zero_()
            self.angle_state.zero_()
            force_mag_target = torch.zeros_like(self.mag_state)
        else:
            angle_delta = self._wrap_to_pi(self.angle_target - self.angle_state)
            self.angle_state += self.angle_alpha * angle_delta

            if phase_idx in (PHASE_DISTURB, PHASE_RECOVER):
                self.mag_state += self.mag_alpha * (self.mag_target - self.mag_state)
                force_mag_target = self.mag_target.clone()
            elif phase_idx == PHASE_RAMP:
                ramp_span = max(self.ramp_end_step - self.ramp_start_step, 1)
                ratio = float(step - self.ramp_start_step + 1) / float(ramp_span)
                ratio = max(0.0, min(ratio, 1.0))
                force_mag_target = self.ramp_start_mag + (self.peak_force_n - self.ramp_start_mag) * ratio
                self.mag_state.copy_(force_mag_target)
            else:
                force_mag_target = torch.full_like(self.mag_state, self.peak_force_n)
                self.mag_state.copy_(force_mag_target)

        nominal_reverse_dir_xy = -normalize_xy(motion_dir_xy)
        force_dir_xy = rotate_xy(nominal_reverse_dir_xy, self.angle_state)
        force_w = torch.zeros((self.num_envs, 3), device=self.device)
        force_w[:, :2] = force_dir_xy * self.mag_state.unsqueeze(-1)

        self.prev_phase = phase_idx
        return {
            "phase_idx": phase_idx,
            "phase_name": PHASE_NAMES[phase_idx],
            "force_w": force_w,
            "force_mag_target": force_mag_target,
            "force_mag_applied": self.mag_state.clone(),
            "force_angle_target_rad": self.angle_target.clone(),
            "force_angle_applied_rad": self.angle_state.clone(),
            "nominal_reverse_dir_xy": nominal_reverse_dir_xy,
            "force_dir_xy": force_dir_xy,
        }

    def _seconds_to_step(self, seconds: float) -> int:
        return max(int(round(float(seconds) / self.step_dt)), 0)

    def _time_constant_to_alpha(self, tau_seconds: float) -> float:
        tau = float(tau_seconds)
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-self.step_dt / tau)

    def _sample_interval_steps(self, count: int) -> torch.Tensor:
        return torch.randint(self.resample_min_step, self.resample_max_step + 1, (count,), device=self.device)

    def _sample_force_targets(self, count: int) -> torch.Tensor:
        return torch.empty((count,), device=self.device).uniform_(self.force_low_n, self.force_high_n)

    def _sample_angle_targets(self, count: int) -> torch.Tensor:
        magnitudes_deg = torch.empty((count,), device=self.device).uniform_(self.angle_min_deg, self.angle_max_deg)
        signs = torch.where(
            torch.rand((count,), device=self.device) < 0.5,
            -torch.ones((count,), device=self.device),
            torch.ones((count,), device=self.device),
        )
        return torch.deg2rad(magnitudes_deg * signs)

    def _resample_targets(
        self,
        env_ids: torch.Tensor,
        step: int,
        *,
        resample_magnitude: bool,
        resample_angle: bool,
    ):
        if env_ids.numel() == 0:
            return
        if resample_magnitude:
            self.mag_target[env_ids] = self._sample_force_targets(env_ids.numel())
        if resample_angle:
            self.angle_target[env_ids] = self._sample_angle_targets(env_ids.numel())
        self.next_resample_step[env_ids] = step + self._sample_interval_steps(env_ids.numel())

    def _wrap_to_pi(self, angles: torch.Tensor) -> torch.Tensor:
        return torch.atan2(torch.sin(angles), torch.cos(angles))


def main():
    output_csv = resolve_output_csv(args_cli.output_csv)
    env_cfg = build_env_cfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    gripper_force_obs_scale = float(env_cfg.observations.pred.gripper_forces_local.scale)
    base_force_obs_scale = float(env_cfg.observations.pred.base_forces_local.scale)

    checkpoint_path = retrieve_file_path(args_cli.checkpoint) if args_cli.checkpoint else None
    jit_path = retrieve_file_path(args_cli.policy_jit) if args_cli.policy_jit else None

    policy = None
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
            "Use --checkpoint for guide_pull_eval, or add a custom exporter that wraps the full act_inference() path."
        )
    else:
        raise RuntimeError("Either --checkpoint or --policy-jit must be provided.")

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError("commands.base_force_ext must be a scripted force term exposing set_force()/clear_force().")
    markers = GuidePullMarkers(env)

    if livestream_enabled:
        print("[INFO] LIVESTREAM enabled; cameras forced on for WebRTC compatibility.")

    num_envs = env.num_envs
    device = env.device
    step_dt = float(env.step_dt)
    max_steps = max(int(round(float(env_cfg.episode_length_s) / step_dt)), 1)
    scheduler = GuidePullScheduler(num_envs=num_envs, device=device, step_dt=step_dt)

    sampled_cmd_vx = env.command_manager.get_command("locomotion_commands")[:, 0].clone()
    phase_count = len(PHASE_NAMES)
    phase_sample_count = torch.zeros((num_envs, phase_count), dtype=torch.long, device=device)
    phase_speed_err_sum = torch.zeros((num_envs, phase_count), device=device)
    phase_lateral_speed_peak = torch.full((num_envs, phase_count), float("-inf"), device=device)
    phase_forward_speed_min = torch.full((num_envs, phase_count), float("inf"), device=device)
    phase_force_mag_sum = torch.zeros((num_envs, phase_count), device=device)
    phase_base_force_err_sum = torch.zeros((num_envs, phase_count), device=device)
    phase_gripper_force_err_sum = torch.zeros((num_envs, phase_count), device=device)

    active_force_sample_count = torch.zeros(num_envs, dtype=torch.long, device=device)
    base_force_err_active_sum = torch.zeros(num_envs, device=device)
    base_force_err_active_max = torch.zeros(num_envs, device=device)
    gripper_force_err_active_sum = torch.zeros(num_envs, device=device)
    gripper_force_err_active_max = torch.zeros(num_envs, device=device)
    recovery_time_s = torch.full((num_envs,), float("nan"), device=device)

    phase_start_root_xy = torch.zeros((num_envs, 2), device=device)
    phase_start_forward_xy = torch.zeros((num_envs, 2), device=device)
    phase_start_forward_xy[:, 0] = 1.0
    prev_phase_idx = None

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "env_id",
                "step",
                "time_s",
                "phase_name",
                "sampled_cmd_vx",
                "cmd_vx",
                "base_vx",
                "base_vy",
                "speed_xy",
                "forward_speed_proj",
                "lateral_speed_proj",
                "force_x",
                "force_y",
                "force_mag_target",
                "force_mag_applied",
                "force_angle_target_deg",
                "force_angle_applied_deg",
                "nominal_reverse_dir_x",
                "nominal_reverse_dir_y",
                "force_dir_x",
                "force_dir_y",
                "phase_proj_disp",
                "phase_lateral_disp",
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
                break

            robot = env.scene["robot"]
            root_vel_xy_pre = robot.data.root_lin_vel_w[:, :2]
            root_pos_xy_pre = robot.data.root_pos_w[:, :2]
            speed_xy_pre = torch.norm(root_vel_xy_pre, dim=-1)
            command_forward_xy_pre = base_forward_axis_xy(env)
            motion_dir_xy = command_forward_xy_pre.clone()
            moving_mask = speed_xy_pre >= float(args_cli.direction_vel_threshold)
            if moving_mask.any():
                motion_dir_xy[moving_mask] = root_vel_xy_pre[moving_mask] / speed_xy_pre[moving_mask].unsqueeze(-1)
            motion_dir_xy = normalize_xy(motion_dir_xy)

            phase_idx = scheduler.phase_index_for_step(step)
            if prev_phase_idx is None or phase_idx != prev_phase_idx:
                phase_start_root_xy.copy_(root_pos_xy_pre)
                phase_start_forward_xy.copy_(command_forward_xy_pre)
                prev_phase_idx = phase_idx

            scheduler_outputs = scheduler.update(step, motion_dir_xy)
            force_term.set_force(scheduler_outputs["force_w"])

            with torch.inference_mode():
                if wrapped_env is not None:
                    actions = policy(obs)
                    obs, _, dones, _ = wrapped_env.step(actions)
                    done_mask = dones.to(dtype=torch.bool, device=device)
                else:
                    obs_tensor = obs["policy"] if isinstance(obs, dict) else obs
                    actions = policy(obs_tensor)
                    obs, _, terminated, truncated, _ = env.step(actions)
                    done_mask = (terminated | truncated).to(dtype=torch.bool, device=device)

                estimator_outputs = extract_force_estimator_outputs(
                    obs,
                    estimator_model,
                    gripper_force_scale=gripper_force_obs_scale,
                    base_force_scale=base_force_obs_scale,
                )

            markers.update()

            robot = env.scene["robot"]
            root_vel_xy = robot.data.root_lin_vel_w[:, :2]
            root_pos_xy = robot.data.root_pos_w[:, :2]
            speed_xy = torch.norm(root_vel_xy, dim=-1)
            command_forward_xy = base_forward_axis_xy(env)
            lateral_axis_xy = torch.stack([-command_forward_xy[:, 1], command_forward_xy[:, 0]], dim=-1)
            phase_lateral_axis_xy = torch.stack([-phase_start_forward_xy[:, 1], phase_start_forward_xy[:, 0]], dim=-1)
            forward_speed_proj = torch.sum(root_vel_xy * command_forward_xy, dim=-1)
            lateral_speed_proj = torch.sum(root_vel_xy * lateral_axis_xy, dim=-1)
            phase_proj_disp = torch.sum((root_pos_xy - phase_start_root_xy) * phase_start_forward_xy, dim=-1)
            phase_lateral_disp = torch.sum((root_pos_xy - phase_start_root_xy) * phase_lateral_axis_xy, dim=-1)
            cmd_vx = env.command_manager.get_command("locomotion_commands")[:, 0]

            gripper_force_est_local = estimator_outputs["gripper_force_est_local"]
            gripper_force_tgt_local = estimator_outputs["gripper_force_tgt_local"]
            gripper_force_err_l2 = estimator_outputs["gripper_force_err_l2"]
            base_force_est_local = estimator_outputs["base_force_est_local"]
            base_force_tgt_local = estimator_outputs["base_force_tgt_local"]
            base_force_err_l2 = estimator_outputs["base_force_err_l2"]

            speed_tracking_err = torch.abs(forward_speed_proj - sampled_cmd_vx)
            phase_sample_count[:, phase_idx] += 1
            phase_speed_err_sum[:, phase_idx] += speed_tracking_err
            phase_lateral_speed_peak[:, phase_idx] = torch.maximum(
                phase_lateral_speed_peak[:, phase_idx],
                torch.abs(lateral_speed_proj),
            )
            phase_forward_speed_min[:, phase_idx] = torch.minimum(
                phase_forward_speed_min[:, phase_idx],
                forward_speed_proj,
            )
            phase_force_mag_sum[:, phase_idx] += scheduler_outputs["force_mag_applied"]
            phase_base_force_err_sum[:, phase_idx] += base_force_err_l2
            phase_gripper_force_err_sum[:, phase_idx] += gripper_force_err_l2

            if phase_idx != PHASE_IDLE:
                active_force_sample_count += 1
                base_force_err_active_sum += base_force_err_l2
                base_force_err_active_max = torch.maximum(base_force_err_active_max, base_force_err_l2)
                gripper_force_err_active_sum += gripper_force_err_l2
                gripper_force_err_active_max = torch.maximum(gripper_force_err_active_max, gripper_force_err_l2)

            if phase_idx == PHASE_RECOVER:
                recovered_mask = torch.isnan(recovery_time_s) & (
                    forward_speed_proj >= sampled_cmd_vx * float(args_cli.recovery_threshold_ratio)
                )
                recovery_time_s[recovered_mask] = max((step + 1) * step_dt - float(args_cli.peak_end_s), 0.0)

            log_time_s = (step + 1) * step_dt
            phase_name = scheduler_outputs["phase_name"]
            force_mag_target = scheduler_outputs["force_mag_target"]
            force_mag_applied = scheduler_outputs["force_mag_applied"]
            force_angle_target_deg = torch.rad2deg(scheduler_outputs["force_angle_target_rad"])
            force_angle_applied_deg = torch.rad2deg(scheduler_outputs["force_angle_applied_rad"])
            nominal_reverse_dir_xy = scheduler_outputs["nominal_reverse_dir_xy"]
            force_dir_xy = scheduler_outputs["force_dir_xy"]
            applied_force_w = scheduler_outputs["force_w"]

            for env_id in range(num_envs):
                writer.writerow(
                    [
                        env_id,
                        step + 1,
                        f"{log_time_s:.6f}",
                        phase_name,
                        f"{sampled_cmd_vx[env_id].item():.6f}",
                        f"{cmd_vx[env_id].item():.6f}",
                        f"{root_vel_xy[env_id, 0].item():.6f}",
                        f"{root_vel_xy[env_id, 1].item():.6f}",
                        f"{speed_xy[env_id].item():.6f}",
                        f"{forward_speed_proj[env_id].item():.6f}",
                        f"{lateral_speed_proj[env_id].item():.6f}",
                        f"{applied_force_w[env_id, 0].item():.6f}",
                        f"{applied_force_w[env_id, 1].item():.6f}",
                        f"{force_mag_target[env_id].item():.6f}",
                        f"{force_mag_applied[env_id].item():.6f}",
                        f"{force_angle_target_deg[env_id].item():.6f}",
                        f"{force_angle_applied_deg[env_id].item():.6f}",
                        f"{nominal_reverse_dir_xy[env_id, 0].item():.6f}",
                        f"{nominal_reverse_dir_xy[env_id, 1].item():.6f}",
                        f"{force_dir_xy[env_id, 0].item():.6f}",
                        f"{force_dir_xy[env_id, 1].item():.6f}",
                        f"{phase_proj_disp[env_id].item():.6f}",
                        f"{phase_lateral_disp[env_id].item():.6f}",
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

            if done_mask.any():
                break

    force_term.clear_force()

    print(f"[INFO] CSV written to: {output_csv}")
    for env_id in range(num_envs):
        disturb_mean_speed_err = metric_mean(phase_speed_err_sum, phase_sample_count, env_id, PHASE_DISTURB)
        peak_mean_speed_err = metric_mean(phase_speed_err_sum, phase_sample_count, env_id, PHASE_PEAK)
        recover_mean_speed_err = metric_mean(phase_speed_err_sum, phase_sample_count, env_id, PHASE_RECOVER)
        disturb_mean_force = metric_mean(phase_force_mag_sum, phase_sample_count, env_id, PHASE_DISTURB)
        peak_mean_force = metric_mean(phase_force_mag_sum, phase_sample_count, env_id, PHASE_PEAK)
        recover_mean_force = metric_mean(phase_force_mag_sum, phase_sample_count, env_id, PHASE_RECOVER)
        recover_base_force_err_mean = metric_mean(phase_base_force_err_sum, phase_sample_count, env_id, PHASE_RECOVER)
        recover_gripper_force_err_mean = metric_mean(
            phase_gripper_force_err_sum, phase_sample_count, env_id, PHASE_RECOVER
        )

        disturb_min_forward = finite_min_or_nan(phase_forward_speed_min[env_id, PHASE_DISTURB].item())
        peak_min_forward = finite_min_or_nan(phase_forward_speed_min[env_id, PHASE_PEAK].item())
        recover_max_lateral = finite_min_or_nan(phase_lateral_speed_peak[env_id, PHASE_RECOVER].item())
        if recover_max_lateral < 0.0:
            recover_max_lateral = float("nan")

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
        recovery_time = recovery_time_s[env_id].item()

        print(
            f"[SUMMARY][env {env_id}] sampled_cmd_vx={format_optional(sampled_cmd_vx[env_id].item())} "
            f"disturb_mean_speed_err={format_optional(disturb_mean_speed_err)} "
            f"peak_mean_speed_err={format_optional(peak_mean_speed_err)} "
            f"recover_mean_speed_err={format_optional(recover_mean_speed_err)} "
            f"disturb_min_forward_speed={format_optional(disturb_min_forward)} "
            f"peak_min_forward_speed={format_optional(peak_min_forward)} "
            f"recover_max_lateral_speed={format_optional(recover_max_lateral)} "
            f"disturb_mean_force={format_optional(disturb_mean_force)} "
            f"peak_mean_force={format_optional(peak_mean_force)} "
            f"recover_mean_force={format_optional(recover_mean_force)} "
            f"recovery_time={format_optional(recovery_time)} "
            f"base_force_err_active_mean={format_optional(base_force_err_active_mean)} "
            f"base_force_err_peak={format_optional(base_force_err_peak)} "
            f"gripper_force_err_active_mean={format_optional(gripper_force_err_active_mean)} "
            f"gripper_force_err_peak={format_optional(gripper_force_err_peak)} "
            f"recover_base_force_err_mean={format_optional(recover_base_force_err_mean)} "
            f"recover_gripper_force_err_mean={format_optional(recover_gripper_force_err_mean)}"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
