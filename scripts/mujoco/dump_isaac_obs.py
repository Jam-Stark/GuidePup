#!/usr/bin/env python3
"""Dump initial observations from IsaacLab for comparison with MuJoCo runtime.

Run in the isaaclab conda environment:
    cd /home/baoquanc/workspace/ForceControl
    conda activate isaaclab
    python scripts/mujoco/dump_isaac_obs.py --task Template-UniFPAliengo-v0 --headless --num_envs 1

Output: scripts/mujoco/isaac_obs_dump.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Template-UniFPAliengo-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--checkpoint", type=str, default=None)
parser.add_argument("--n_steps", type=int, default=5)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import numpy as np
import torch
import gymnasium as gym
import isaaclab_tasks  # noqa: F401
import acc.tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

from acc.algorithms.rsl_rl.runner import OnPolicyRunnerAdaptation
from acc.tasks.manager_based.uniFP_aliengo.agents.rsl_rl_ppo_cfg import PPORunnerCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env = gym.make(args_cli.task, cfg=env_cfg)
    unwrapped = env.unwrapped

    # Wrap for RSL-RL if checkpoint provided
    agent_cfg = PPORunnerCfg()
    agent_cfg.device = unwrapped.device
    wrapped_env = RslRlVecEnvWrapper(unwrapped, clip_actions=agent_cfg.clip_actions)

    policy = None
    if args_cli.checkpoint:
        runner = OnPolicyRunnerAdaptation(
            wrapped_env, agent_cfg.to_dict(), log_dir=None, device=unwrapped.device
        )
        runner.load(args_cli.checkpoint)
        policy = runner.get_inference_policy(device=unwrapped.device)

    # Reset
    obs_dict, _ = env.reset()

    # Dump articulation info
    robot = unwrapped.scene["robot"]
    print(f"Joint names: {robot.joint_names}")
    print(f"Joint positions: {robot.data.joint_pos[0].cpu().numpy()}")
    print(f"Default joint pos: {robot.data.default_joint_pos[0].cpu().numpy()}")

    # Dump per-term observations
    obs_mgr = unwrapped.observation_manager
    policy_obs = obs_dict["policy"][0].cpu().numpy()
    print(f"\nPolicy obs shape: {policy_obs.shape}")
    print(f"Policy obs range: [{policy_obs.min():.4f}, {policy_obs.max():.4f}]")

    # Get per-term breakdown
    group_term_names = obs_mgr._group_obs_term_names["policy"]
    group_term_dims = obs_mgr.group_obs_term_dim["policy"]
    print(f"\nPer-term breakdown:")
    offset = 0
    term_data = {}
    for name, dims in zip(group_term_names, group_term_dims):
        total_dim = 1
        for d in dims:
            total_dim *= d
        term_slice = policy_obs[offset : offset + total_dim]
        print(f"  {name}: dims={dims} range=[{term_slice.min():.6f}, {term_slice.max():.6f}]")
        # Show last frame (most recent) for history terms
        if len(dims) > 0:
            per_frame = dims[-1] if len(dims) == 1 else dims[-1]
            n_frames = total_dim // per_frame
            if n_frames > 1:
                last_frame = term_slice[-per_frame:]
                print(f"    last frame ({per_frame}d): {last_frame}")
            else:
                print(f"    values: {term_slice}")
        term_data[name] = term_slice
        offset += total_dim

    # Run a few steps and collect data
    all_obs = [policy_obs]
    all_actions = []

    for step in range(args_cli.n_steps):
        wrapped_obs = wrapped_env.get_observations()
        if policy is not None:
            with torch.inference_mode():
                action = policy(wrapped_obs)
        else:
            action = torch.zeros(args_cli.num_envs, unwrapped.num_actions, device=unwrapped.device)

        action_np = action[0].cpu().numpy()
        all_actions.append(action_np)

        obs_dict, _, _, _, _ = env.step(action)
        policy_obs = obs_dict["policy"][0].cpu().numpy()
        all_obs.append(policy_obs)

        qpos = robot.data.joint_pos[0].cpu().numpy()
        root_pos = robot.data.root_pos_w[0].cpu().numpy()
        root_quat = robot.data.root_quat_w[0].cpu().numpy()
        root_vel = robot.data.root_lin_vel_w[0].cpu().numpy()
        root_ang_vel_b = robot.data.root_ang_vel_b[0].cpu().numpy()

        print(f"\nStep {step}: root_pos={root_pos}, h={root_pos[2]:.3f}, "
              f"vel={root_vel}, ang_vel_b={root_ang_vel_b}")
        print(f"  action range: [{action_np.min():.4f}, {action_np.max():.4f}]")
        print(f"  obs range: [{policy_obs.min():.4f}, {policy_obs.max():.4f}]")

    # Save
    out_path = Path(__file__).parent / "isaac_obs_dump.npz"
    np.savez(
        out_path,
        obs=np.array(all_obs),
        actions=np.array(all_actions) if all_actions else np.array([]),
        joint_names=robot.joint_names,
        default_joint_pos=robot.data.default_joint_pos[0].cpu().numpy(),
    )
    print(f"\nSaved to {out_path}")

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
