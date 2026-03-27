#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import sys

import torch

SCRIPTS_DIR = Path(__file__).resolve().parents[2]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Evaluate a UniFP Aliengo policy with a scripted reverse push.")
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
parser.add_argument("--forward-vel-x", type=float, default=0.4, help="Fixed forward velocity command.")
parser.add_argument("--push-trigger-s", type=float, default=6.0, help="Time to trigger the reverse push.")
parser.add_argument("--push-duration-s", type=float, default=2.0, help="Duration of the reverse push.")
parser.add_argument("--push-force-n", type=float, required=True, help="Magnitude of the reverse push force.")
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
from isaaclab.utils.math import quat_from_euler_xyz, quat_from_angle_axis

from acc.tasks.manager_based.uniFP_aliengo.mdp.utils import base_yaw_to_world, get_base_yaw_quat
from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg


def resolve_output_csv(path_str: str | None) -> Path:
    if path_str:
        output_path = Path(path_str).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = (Path("logs") / "experiments" / "unifp_reverse_push" / f"reverse_push_{stamp}.csv").resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (args_cli.forward_vel_x, args_cli.forward_vel_x)
    return env_cfg


def load_jit_policy(policy_path: str, device: str | torch.device):
    policy_model = torch.jit.load(policy_path)
    policy_model.eval()
    policy_model.to(device)
    return policy_model


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
    return forward_xy / torch.clamp(torch.norm(forward_xy, dim=-1, keepdim=True), min=1e-6)


def format_optional(value: float) -> str:
    return "n/a" if value != value else f"{value:.3f}"


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


class ReversePushMarkers:
    """Standalone visualization markers for command/current velocity and base force."""

    def __init__(self, env: ManagerBasedRLEnv):
        self.env = env
        self.device = env.device
        self.cmd_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/ReversePush/cmd_velocity",
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
                prim_path="/Visuals/ReversePush/current_velocity",
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
                prim_path="/Visuals/ReversePush/base_force_ext",
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
    else:
        raise RuntimeError(
            "This task uses AsymmetricActorCritic with a history encoder, but IsaacLab's stock "
            "export_policy_as_jit() only exports policy.actor and drops the encoder/adaptation path. "
            "As a result, the exported policy.pt is not a faithful inference artifact for this task. "
            "Use --checkpoint for reverse_push_eval, or add a custom exporter that wraps the full act_inference() path."
        )

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError("commands.base_force_ext must be a scripted force term exposing set_force()/clear_force().")
    markers = ReversePushMarkers(env)

    if livestream_enabled:
        print("[INFO] LIVESTREAM enabled; cameras forced on for WebRTC compatibility.")

    step_dt = float(env.step_dt)
    max_steps = max(int(round(float(env_cfg.episode_length_s) / step_dt)), 1)
    push_trigger_step = max(int(round(args_cli.push_trigger_s / step_dt)), 0)
    push_duration_steps = max(int(round(args_cli.push_duration_s / step_dt)), 1)
    push_delay_limit_steps = max(int(round(1.0 / step_dt)), 0)
    min_trigger_speed = 0.05
    stop_threshold = 0.02
    reverse_threshold = -0.05

    num_envs = env.num_envs
    device = env.device

    triggered = torch.zeros(num_envs, dtype=torch.bool, device=device)
    push_start_step = torch.full((num_envs,), -1, dtype=torch.long, device=device)
    forward_axis_xy = torch.zeros((num_envs, 2), device=device)
    trigger_speed = torch.full((num_envs,), float("nan"), device=device)
    trigger_root_xy = torch.zeros((num_envs, 2), device=device)
    stop_time_s = torch.full((num_envs,), float("nan"), device=device)
    min_forward_speed = torch.full((num_envs,), float("inf"), device=device)
    reverse_detected = torch.zeros(num_envs, dtype=torch.bool, device=device)
    min_proj_disp = torch.full((num_envs,), float("inf"), device=device)
    max_proj_disp = torch.full((num_envs,), float("-inf"), device=device)
    push_sample_count = torch.zeros(num_envs, dtype=torch.long, device=device)
    base_force_err_push_sum = torch.zeros(num_envs, device=device)
    base_force_err_push_max = torch.zeros(num_envs, device=device)
    gripper_force_err_push_sum = torch.zeros(num_envs, device=device)
    gripper_force_err_push_max = torch.zeros(num_envs, device=device)

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "env_id",
                "step",
                "time_s",
                "push_active",
                "cmd_vx",
                "base_vx",
                "base_vy",
                "speed_xy",
                "forward_speed_proj",
                "force_x",
                "force_y",
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
            root_vel_xy = robot.data.root_lin_vel_w[:, :2]
            root_pos_xy = robot.data.root_pos_w[:, :2]
            speed_xy = torch.norm(root_vel_xy, dim=-1)

            waiting_mask = (~triggered) & (step >= push_trigger_step)
            if waiting_mask.any():
                fallback_axis = base_forward_axis_xy(env)
                ready_mask = waiting_mask & (
                    (speed_xy >= min_trigger_speed) | (step >= push_trigger_step + push_delay_limit_steps)
                )
                if ready_mask.any():
                    forward_axis_xy[ready_mask] = fallback_axis[ready_mask]
                    fast_mask = ready_mask & (speed_xy >= min_trigger_speed)
                    if fast_mask.any():
                        forward_axis_xy[fast_mask] = root_vel_xy[fast_mask] / speed_xy[fast_mask].unsqueeze(-1)
                    triggered[ready_mask] = True
                    push_start_step[ready_mask] = step
                    trigger_speed[ready_mask] = speed_xy[ready_mask]
                    trigger_root_xy[ready_mask] = root_pos_xy[ready_mask]

            push_active = triggered & ((step - push_start_step) < push_duration_steps)
            force_term.clear_force()
            if push_active.any():
                active_ids = push_active.nonzero(as_tuple=False).squeeze(-1)
                forces = torch.zeros((active_ids.numel(), 3), device=device)
                forces[:, :2] = -forward_axis_xy[active_ids] * float(args_cli.push_force_n)
                force_term.set_force(forces, env_ids=active_ids)

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
            force_xy = env.command_manager.get_command("base_force_ext")[:, :2]
            cmd_vx = env.command_manager.get_command("locomotion_commands")[:, 0]
            gripper_force_est_local = estimator_outputs["gripper_force_est_local"]
            gripper_force_tgt_local = estimator_outputs["gripper_force_tgt_local"]
            gripper_force_err_l2 = estimator_outputs["gripper_force_err_l2"]
            base_force_est_local = estimator_outputs["base_force_est_local"]
            base_force_tgt_local = estimator_outputs["base_force_tgt_local"]
            base_force_err_l2 = estimator_outputs["base_force_err_l2"]

            forward_speed_proj = torch.zeros(num_envs, device=device)
            proj_disp = torch.zeros(num_envs, device=device)
            if triggered.any():
                triggered_ids = triggered.nonzero(as_tuple=False).squeeze(-1)
                forward_speed_proj[triggered_ids] = torch.sum(
                    root_vel_xy[triggered_ids] * forward_axis_xy[triggered_ids], dim=-1
                )
                proj_disp[triggered_ids] = torch.sum(
                    (root_pos_xy[triggered_ids] - trigger_root_xy[triggered_ids]) * forward_axis_xy[triggered_ids],
                    dim=-1,
                )

                just_stopped = triggered & torch.isnan(stop_time_s) & (forward_speed_proj <= stop_threshold)
                stop_time_s[just_stopped] = (step + 1) * step_dt
                min_forward_speed[triggered_ids] = torch.minimum(
                    min_forward_speed[triggered_ids], forward_speed_proj[triggered_ids]
                )
                reverse_detected[triggered_ids] |= forward_speed_proj[triggered_ids] < reverse_threshold
                min_proj_disp[triggered_ids] = torch.minimum(min_proj_disp[triggered_ids], proj_disp[triggered_ids])
                max_proj_disp[triggered_ids] = torch.maximum(max_proj_disp[triggered_ids], proj_disp[triggered_ids])

            if push_active.any():
                active_ids = push_active.nonzero(as_tuple=False).squeeze(-1)
                push_sample_count[active_ids] += 1
                base_force_err_push_sum[active_ids] += base_force_err_l2[active_ids]
                base_force_err_push_max[active_ids] = torch.maximum(
                    base_force_err_push_max[active_ids], base_force_err_l2[active_ids]
                )
                gripper_force_err_push_sum[active_ids] += gripper_force_err_l2[active_ids]
                gripper_force_err_push_max[active_ids] = torch.maximum(
                    gripper_force_err_push_max[active_ids], gripper_force_err_l2[active_ids]
                )

            log_time_s = (step + 1) * step_dt
            for env_id in range(num_envs):
                writer.writerow(
                    [
                        env_id,
                        step + 1,
                        f"{log_time_s:.6f}",
                        int(push_active[env_id].item()),
                        f"{cmd_vx[env_id].item():.6f}",
                        f"{root_vel_xy[env_id, 0].item():.6f}",
                        f"{root_vel_xy[env_id, 1].item():.6f}",
                        f"{speed_xy[env_id].item():.6f}",
                        f"{forward_speed_proj[env_id].item():.6f}",
                        f"{force_xy[env_id, 0].item():.6f}",
                        f"{force_xy[env_id, 1].item():.6f}",
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
        trig_speed = trigger_speed[env_id].item()
        stop_time = stop_time_s[env_id].item()
        min_speed = min_forward_speed[env_id].item()
        min_disp = min_proj_disp[env_id].item()
        max_disp = max_proj_disp[env_id].item()
        push_samples = int(push_sample_count[env_id].item())
        base_force_err_push_mean = (
            (base_force_err_push_sum[env_id] / push_sample_count[env_id]).item() if push_samples > 0 else float("nan")
        )
        base_force_err_peak = base_force_err_push_max[env_id].item() if push_samples > 0 else float("nan")
        gripper_force_err_push_mean = (
            (gripper_force_err_push_sum[env_id] / push_sample_count[env_id]).item()
            if push_samples > 0
            else float("nan")
        )
        gripper_force_err_peak = gripper_force_err_push_max[env_id].item() if push_samples > 0 else float("nan")
        if min_speed == float("inf"):
            min_speed = float("nan")
        if min_disp == float("inf"):
            min_disp = float("nan")
        if max_disp == float("-inf"):
            max_disp = float("nan")
        print(
            f"[SUMMARY][env {env_id}] trigger_speed={format_optional(trig_speed)} "
            f"stop_time={format_optional(stop_time)} min_forward_speed={format_optional(min_speed)} "
            f"reverse={bool(reverse_detected[env_id].item())} "
            f"proj_disp_min={format_optional(min_disp)} proj_disp_max={format_optional(max_disp)} "
            f"base_force_err_push_mean={format_optional(base_force_err_push_mean)} "
            f"base_force_err_peak={format_optional(base_force_err_peak)} "
            f"gripper_force_err_push_mean={format_optional(gripper_force_err_push_mean)} "
            f"gripper_force_err_peak={format_optional(gripper_force_err_peak)}"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
