
import argparse
import os
import sys

from _extension_source import append_acc_source_dir

append_acc_source_dir()

from isaaclab.app import AppLauncher

# Define args
parser = argparse.ArgumentParser(description="Export VBC policy.")
parser.add_argument("--task", type=str, default="Template-Vbc-v0", help="Name of the task.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the checkpoint.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
# AppLauncher args
AppLauncher.add_app_launcher_args(parser)
# Parse args
args_cli, hydra_args = parser.parse_known_args()

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# Launch app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import acc to register tasks
import acc

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import export_policy_as_jit, RslRlVecEnvWrapper
from isaaclab_tasks.utils.hydra import hydra_task_config

@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"
    
    # Create env
    print(f"Creating environment for task: {args_cli.task}")
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    
    # Load runner
    print(f"Loading checkpoint from: {args_cli.checkpoint}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=env.device)
    runner.load(args_cli.checkpoint)
    
    # Extract policy and normalizer
    try:
        # version 2.3 onwards
        policy_nn = runner.alg.policy
    except AttributeError:
        # version 2.2 and below
        policy_nn = runner.alg.actor_critic

    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None
    
    # Export
    export_dir = os.path.join(os.path.dirname(args_cli.checkpoint), "exported")
    print(f"Exporting to {export_dir}...")
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_dir, filename="policy.pt")
    print("Export complete.")
    
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
