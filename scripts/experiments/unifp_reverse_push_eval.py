#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys

import torch

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Evaluate a UniFP Aliengo policy with a scripted reverse push.")
policy_group = parser.add_mutually_exclusive_group(required=True)
policy_group.add_argument("--checkpoint", type=str, help="Path to an RSL-RL checkpoint.")
policy_group.add_argument("--policy-jit", type=str, help="Path to an exported JIT policy.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
parser.add_argument("--forward-vel-x", type=float, default=0.4, help="Fixed forward velocity command.")
parser.add_argument("--push-trigger-s", type=float, default=6.0, help="Time to trigger the reverse push.")
parser.add_argument("--push-duration-s", type=float, default=2.0, help="Duration of the reverse push.")
parser.add_argument("--push-force-n", type=float, required=True, help="Magnitude of the reverse push force.")
parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import acc  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.assets import retrieve_file_path

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
    return policy, wrapped_env


def base_forward_axis_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    yaw_quat = get_base_yaw_quat(env)
    local_forward = torch.zeros((env.num_envs, 3), device=env.device)
    local_forward[:, 0] = 1.0
    forward_w = base_yaw_to_world(local_forward, yaw_quat)
    forward_xy = forward_w[:, :2]
    return forward_xy / torch.clamp(torch.norm(forward_xy, dim=-1, keepdim=True), min=1e-6)


def format_optional(value: float) -> str:
    return "n/a" if value != value else f"{value:.3f}"


def main():
    output_csv = resolve_output_csv(args_cli.output_csv)
    env_cfg = build_env_cfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    checkpoint_path = retrieve_file_path(args_cli.checkpoint) if args_cli.checkpoint else None
    jit_path = retrieve_file_path(args_cli.policy_jit) if args_cli.policy_jit else None

    policy = None
    wrapped_env = None
    if checkpoint_path:
        print(f"[INFO] Loading raw checkpoint: {checkpoint_path}")
        policy, wrapped_env = load_raw_checkpoint_policy(checkpoint_path, env)
        env.reset()
        obs = wrapped_env.get_observations()
    else:
        print(f"[INFO] Loading JIT policy: {jit_path}")
        policy = load_jit_policy(jit_path, env.device)
        obs, _ = env.reset()

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError("commands.base_force_ext must be a scripted force term exposing set_force()/clear_force().")

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

            robot = env.scene["robot"]
            root_vel_xy = robot.data.root_lin_vel_w[:, :2]
            root_pos_xy = robot.data.root_pos_w[:, :2]
            speed_xy = torch.norm(root_vel_xy, dim=-1)
            force_xy = env.command_manager.get_command("base_force_ext")[:, :2]
            cmd_vx = env.command_manager.get_command("locomotion_commands")[:, 0]

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
            f"proj_disp_min={format_optional(min_disp)} proj_disp_max={format_optional(max_disp)}"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
