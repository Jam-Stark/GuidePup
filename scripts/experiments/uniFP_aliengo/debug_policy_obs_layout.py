#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import torch

SCRIPTS_DIR = Path(__file__).resolve().parents[2]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Inspect the Isaac policy observation layout used by the UniFP Aliengo eval environment."
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to instantiate.")
parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import acc  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnv

from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg


EPISODE_LENGTH_S = 60.0
LONG_RESAMPLING_S = 1.0e9


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = EPISODE_LENGTH_S
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env_cfg.commands.locomotion_commands.resampling_time_range = (LONG_RESAMPLING_S, LONG_RESAMPLING_S)
    env_cfg.commands.locomotion_commands.rel_standing_envs = 0.0
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (0.0, 0.0)
    env_cfg.commands.locomotion_commands.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.locomotion_commands.ranges.ang_vel_z = (0.0, 0.0)
    env_cfg.terminations.base_height = None
    return env_cfg


def main() -> None:
    env = ManagerBasedRLEnv(cfg=build_env_cfg())
    obs_dict, _ = env.reset()

    manager = env.observation_manager
    policy_obs = obs_dict["policy"]
    term_names = list(manager._group_obs_term_names["policy"])
    history_buffers = manager._group_obs_term_history_buffer["policy"]

    term_histories: list[torch.Tensor] = []
    current_terms: list[torch.Tensor] = []
    for term_name in term_names:
        history = history_buffers[term_name].buffer.clone()
        term_histories.append(history)
        current_terms.append(history[:, -1, :])

    current_frame = torch.cat(current_terms, dim=-1)
    term_major = torch.cat([history.reshape(history.shape[0], -1) for history in term_histories], dim=-1)
    frame_major = torch.cat(
        [torch.cat([history[:, frame_idx, :] for history in term_histories], dim=-1) for frame_idx in range(term_histories[0].shape[1])],
        dim=-1,
    )

    last_block = policy_obs[:, -current_frame.shape[-1] :]
    first_block = policy_obs[:, : current_frame.shape[-1]]

    def max_abs_diff(lhs: torch.Tensor, rhs: torch.Tensor) -> float:
        return torch.max(torch.abs(lhs - rhs)).item()

    target_position_cmd = env.command_manager.get_command("target_position_commands")
    locomotion_cmd = env.command_manager.get_command("locomotion_commands")
    gait_phase_cmd = env.command_manager.get_command("gait_phase")

    print(f"[INFO] policy_obs shape: {tuple(policy_obs.shape)}")
    print(f"[INFO] policy term order: {term_names}")
    print(f"[INFO] current_frame shape: {tuple(current_frame.shape)}")
    print(f"[INFO] term_major diff: {max_abs_diff(policy_obs, term_major):.6e}")
    print(f"[INFO] frame_major diff: {max_abs_diff(policy_obs, frame_major):.6e}")
    print(f"[INFO] last_block_vs_current_frame diff: {max_abs_diff(last_block, current_frame):.6e}")
    print(f"[INFO] first_block_vs_current_frame diff: {max_abs_diff(first_block, current_frame):.6e}")
    print(f"[INFO] locomotion command reset value: {locomotion_cmd[0, :3].detach().cpu().tolist()}")
    print(f"[INFO] target_position command reset value: {target_position_cmd[0, :3].detach().cpu().tolist()}")
    print(f"[INFO] gait_phase reset value: {gait_phase_cmd[0].detach().cpu().tolist()}")
    print(f"[INFO] current_frame first 20 values: {current_frame[0, :20].detach().cpu().tolist()}")
    print(f"[INFO] policy_obs last 20 values: {policy_obs[0, -20:].detach().cpu().tolist()}")

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
