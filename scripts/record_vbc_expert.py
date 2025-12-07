"""
Script to record expert data from a trained VBC policy for ACC initialization.
This script runs the VBC environment, computes the observations and actions required by the ACC project,
and saves them to an HDF5 file.

Usage:
    python scripts/record_vbc_expert.py --task Isaac-VBC-Task-v0 --checkpoint /path/to/policy.pt --num_envs 10 --num_steps 1000
"""

import argparse
import h5py
import torch
import sys
import os
from isaaclab.app import AppLauncher

# Argument parsing
parser = argparse.ArgumentParser(description="Record expert data from VBC policy for ACC initialization.")
parser.add_argument("--task", type=str, default="Isaac-VBC-Task-v0", help="Name of the VBC task (e.g., Isaac-VBC-B1-v0).")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the exported VBC policy model (.pt is preferred for JIT loading).")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--num_steps", type=int, default=1000, help="Number of steps to record.")
parser.add_argument("--output_file", type=str, default="vbc_expert_data.hdf5", help="Output HDF5 file path.")
# parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode.")

# Append AppLauncher arguments
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# Launch Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Import Isaac Lab modules (must be after launching app)
# Add the 'acc' package to sys.path so we can import it
# This assumes the script is located in /workspace/acc/scripts/
# and the package is in /workspace/acc/source/acc/
script_dir = os.path.dirname(os.path.abspath(__file__))
acc_source_dir = os.path.join(script_dir, "../source/acc")
sys.path.append(acc_source_dir)

# Import the custom package to register the tasks
import acc

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import quat_apply_inverse
from isaaclab_tasks.utils import parse_env_cfg


def main():
    # 1. Load Environment Configuration
    print(f"Loading task: {args.task}")
    try:
        # parse_env_cfg signature might have changed. 
        # Trying with 'device' instead of 'use_gpu', or just num_envs.
        # Assuming args has device from AppLauncher, or defaulting to cuda:0
        device = getattr(args, "device", "cuda:0")
        env_cfg = parse_env_cfg(args.task, device=device, num_envs=args.num_envs)
    except TypeError as e:
        print(f"Caught TypeError: {e}")
        print("Retrying with only num_envs...")
        env_cfg = parse_env_cfg(args.task, num_envs=args.num_envs)
    except Exception as e:
        print(f"Error loading task config: {e}")
        print("Please ensure the task name is correct and the extension is enabled.")
        return

    # Force override num_envs for recording
    env_cfg.scene.num_envs = args.num_envs
    
    # 2. Create Environment
    print("Creating environment...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # --- Identify Joint Indices for ACC (18 joints) ---
    # ACC expects 12 leg joints + 6 arm joints.
    # We need to find their indices in the VBC robot (which might have 19 joints).
    robot = env.scene["robot"]
    all_joint_names = robot.data.joint_names
    print(f"Robot has {len(all_joint_names)} joints: {all_joint_names}")
    
    target_joint_names = [
        # Legs (12)
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        # Arm (6)
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]
    
    acc_joint_indices = []
    for name in target_joint_names:
        if name in all_joint_names:
            acc_joint_indices.append(all_joint_names.index(name))
        else:
            print(f"[WARNING] Joint {name} not found in robot!")
            
    acc_joint_indices = torch.tensor(acc_joint_indices, device=env.device, dtype=torch.long)
    print(f"Selected {len(acc_joint_indices)} joints for ACC: {acc_joint_indices}")
    
    if len(acc_joint_indices) != 18:
        print(f"[ERROR] Expected 18 joints, found {len(acc_joint_indices)}. Please check joint names.")
        return

    # 3. Load Policy
    # We recommend using an exported JIT model (.pt) for simplicity.
    # If you have a .pth checkpoint, you might need to instantiate the network class manually.
    print(f"Loading policy from: {args.checkpoint}")
    try:
        policy_model = torch.jit.load(args.checkpoint)
        policy_model.eval()
        policy_model.to(env.device)
        print("Successfully loaded JIT policy model.")
    except Exception as e:
        print(f"Could not load as JIT model: {e}")
        print("Attempting to load as state dict (requires model definition)...")
        # Placeholder: If you need to load a raw checkpoint, you would instantiate your ActorCritic here
        # and load_state_dict. For this demo, we assume JIT.
        print("Error: This script currently supports exported JIT models (.pt). Please export your policy first.")
        return

    # 4. Prepare HDF5 Dataset
    print(f"Preparing HDF5 file: {args.output_file}")
    h5_file = h5py.File(args.output_file, 'w')
    
    # ACC Observation Dimension Calculation:
    # base_lin_vel(3) + base_ang_vel(3) + project_gravity(3) + command(3) +
    # dof_err(18) + dof_vel(18) + curr_ee_pos_cart(3) + curr_ee_goal_pos_cart(3) +
    # tracking_error(3) + last_action(18) = 75
    obs_dim = 75
    action_dim = 18  # ACC uses 18-dof actions
    
    total_samples = args.num_steps * args.num_envs
    dset_obs = h5_file.create_dataset("obs", (total_samples, obs_dim), dtype='f4')
    dset_actions = h5_file.create_dataset("actions", (total_samples, action_dim), dtype='f4')
    dset_dones = h5_file.create_dataset("dones", (total_samples, 1), dtype='u1')
    
    # 5. Simulation Loop
    print("Starting recording...")
    obs, _ = env.reset()
    
    # Initialize 'last_action' for ACC (18-dim), initially zero
    last_action_acc = torch.zeros((args.num_envs, 18), device=env.device)
    
    # ACC Action Scale (from acc_env_cfg.py)
    ACC_ACTION_SCALE = 0.25
    
    step_count = 0
    total_recorded = 0
    
    with torch.inference_mode():
        while simulation_app.is_running() and step_count < args.num_steps:
            # --- 1. VBC Policy Inference ---
            # Input: VBC observations (from env.reset/step)
            # Output: VBC actions (12-dim leg actions)
            if isinstance(obs, dict):
                vbc_actions = policy_model(obs['policy'])
            else:
                vbc_actions = policy_model(obs)
            
            # --- 2. Step Environment ---
            # The environment will apply VBC actions to legs and IK to arm
            obs, rewards, terminated, truncated, extras = env.step(vbc_actions)
            dones = terminated | truncated
            
            # --- 3. Compute ACC Data ---
            robot = env.scene["robot"]
            
            # (1) Base Linear Velocity (3)
            base_lin_vel = robot.data.root_lin_vel_b
            
            # (2) Base Angular Velocity (3)
            base_ang_vel = robot.data.root_ang_vel_b
            
            # (3) Projected Gravity (3)
            projected_gravity = robot.data.projected_gravity_b
            
            # (4) Command (3) - Locomotion (lin_x, lin_y, ang_yaw)
            # Assuming VBC has 'locomotion_commands'
            cmd_loco = env.command_manager.get_command("locomotion_commands")[:, :3]
            
            # (5) DOF Error (18)
            # ACC uses all 18 joints (legs + arm)
            # Slice to keep only the 18 ACC joints
            joint_pos = robot.data.joint_pos[:, acc_joint_indices]
            default_joint_pos = robot.data.default_joint_pos[:, acc_joint_indices]
            dof_err = joint_pos - default_joint_pos
            
            # (6) DOF Velocity (18)
            dof_vel = robot.data.joint_vel[:, acc_joint_indices]
            
            # (7) Current EE Position in Base Frame (3)
            # Find gripper body index
            gripper_ids = robot.find_bodies("gripperMover")[0]
            # Note: If multiple envs, find_bodies returns indices for all. 
            # But robot.data.body_pos_w is (num_envs, num_bodies, 3).
            # We need the index relative to the robot articulation.
            # Usually find_bodies returns indices into the body list of the articulation.
            # Let's assume "gripperMover" is a valid link name in the URDF.
            gripper_idx = gripper_ids[0] # Take the first one (should be same for all clones)
            
            ee_pos_w = robot.data.body_pos_w[:, gripper_idx]
            root_pos_w = robot.data.root_pos_w
            root_quat_w = robot.data.root_quat_w
            curr_ee_pos_cart = quat_apply_inverse(root_quat_w, ee_pos_w - root_pos_w)
            
            # (8) Current EE Goal Position in Base Frame (3)
            # VBC uses 'target_position_commands'
            curr_ee_goal_pos_cart = env.command_manager.get_command("target_position_commands")[:, :3]
            
            # (9) Tracking Error (3)
            tracking_error = curr_ee_goal_pos_cart - curr_ee_pos_cart
            
            # (10) Last Action (18)
            # Use the expert action computed in the PREVIOUS step
            current_last_action = last_action_acc.clone()
            
            # --- Construct ACC Observation Vector ---
            acc_obs = torch.cat([
                base_lin_vel,       # 3
                base_ang_vel,       # 3
                projected_gravity,  # 3
                cmd_loco,           # 3
                dof_err,            # 18
                dof_vel,            # 18
                curr_ee_pos_cart,   # 3
                curr_ee_goal_pos_cart, # 3
                tracking_error,     # 3
                current_last_action # 18
            ], dim=-1)
            
            # --- Compute Expert Action (Label) ---
            # We need the 18-dim joint targets that produce the expert behavior.
            # VBC environment applies actions to legs (from policy) and arm (from IK).
            # 'joint_pos_target' contains the final targets for all joints.
            
            # FIX: The Arm joints (controlled by IK) sometimes show numerical instability in 
            # robot.data.joint_pos_target (values like +/- 3000 rad).
            # However, the actual robot state (joint_pos) is stable.
            # For the Arm (indices 12-17), we will use the CURRENT joint_pos as the expert target.
            # This is a valid approximation for BC: "do what the expert actually did".
            
            # 1. Get targets for Legs (0-11) from the buffer (Policy output)
            leg_targets = robot.data.joint_pos_target[:, acc_joint_indices[:12]]
            
            # 2. Get targets for Arm (12-17) from current position (Stable proxy)
            arm_targets = robot.data.joint_pos[:, acc_joint_indices[12:]]
            
            # 3. Combine
            final_targets = torch.cat([leg_targets, arm_targets], dim=1)
            
            # Reverse normalization: action = (target - default) / scale
            expert_action = (final_targets - default_joint_pos) / ACC_ACTION_SCALE
            
            # Update last_action for next step
            last_action_acc = expert_action.clone()
            
            # Handle resets: if an env reset, its last_action should be zeroed
            env_ids_reset = env.reset_buf.nonzero(as_tuple=False).flatten()
            if len(env_ids_reset) > 0:
                last_action_acc[env_ids_reset] = 0.0
            
            # --- Save to Buffer ---
            start_idx = total_recorded
            end_idx = total_recorded + args.num_envs
            
            # Handle potential overflow if num_steps is reached mid-batch
            if end_idx > total_samples:
                valid_num = total_samples - start_idx
                dset_obs[start_idx:] = acc_obs[:valid_num].cpu().numpy()
                dset_actions[start_idx:] = expert_action[:valid_num].cpu().numpy()
                dset_dones[start_idx:] = dones[:valid_num].cpu().unsqueeze(-1).numpy()
                break
            else:
                dset_obs[start_idx:end_idx] = acc_obs.cpu().numpy()
                dset_actions[start_idx:end_idx] = expert_action.cpu().numpy()
                dset_dones[start_idx:end_idx] = dones.cpu().unsqueeze(-1).numpy()
            
            total_recorded += args.num_envs
            step_count += 1
            
            if step_count % 100 == 0:
                print(f"Recorded {step_count}/{args.num_steps} steps...")

    h5_file.close()
    print(f"Data recording complete. Saved {total_recorded} samples to {args.output_file}")
    env.close()

if __name__ == "__main__":
    main()
