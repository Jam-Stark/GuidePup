"""
Script to play the trained ACC BC policy.
"""

import argparse
import torch
import sys
import os
from isaaclab.app import AppLauncher

# Argument parsing
parser = argparse.ArgumentParser(description="Play trained ACC BC policy.")
parser.add_argument("--task", type=str, default="Isaac-ACC-Task-v0", help="Name of the task.")
parser.add_argument("--model", type=str, default="acc_bc_policy_jit.pt", help="Path to the JIT policy model.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")

# Append AppLauncher arguments
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# Launch Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Import Isaac Lab modules
script_dir = os.path.dirname(os.path.abspath(__file__))
acc_source_dir = os.path.join(script_dir, "../source/acc")
sys.path.append(acc_source_dir)

import acc  # noqa: F401
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg


def main():
    # 1. Load Environment Configuration
    print(f"Loading task: {args.task}")
    try:
        device = getattr(args, "device", "cuda:0")
        env_cfg = parse_env_cfg(args.task, device=device, num_envs=args.num_envs)
    except TypeError:
        env_cfg = parse_env_cfg(args.task, num_envs=args.num_envs)

    # 2. Create Environment
    print("Creating environment...")
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 3. Load Policy
    print(f"Loading policy from: {args.model}")
    try:
        policy_model = torch.jit.load(args.model)
        policy_model.eval()
        policy_model.to(env.device)
        print("Successfully loaded JIT policy model.")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # 4. Simulation Loop
    print("Starting simulation...")
    obs, _ = env.reset()
    
    while simulation_app.is_running():
        with torch.inference_mode():
            # Policy Inference
            if isinstance(obs, dict):
                # The environment returns a dict, but our BC model expects a tensor.
                # The 'policy' key usually contains the observation vector.
                obs_tensor = obs['policy']
            else:
                obs_tensor = obs
            
            # The model handles normalization internally if it was saved with it (which my train_bc.py does not do for JIT!)
            # Wait, my train_bc.py saves the model state_dict, and also traces it.
            # The traced model captures the forward method, which INCLUDES normalization using the buffers.
            # So it should work fine.
            
            actions = policy_model(obs_tensor)
            
            # Step Environment
            obs, rewards, terminated, truncated, extras = env.step(actions)
            
            # Reset if needed (handled by env automatically usually, but good to know)

    env.close()


if __name__ == "__main__":
    main()
