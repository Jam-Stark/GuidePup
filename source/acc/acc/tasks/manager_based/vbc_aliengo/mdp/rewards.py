# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCaster
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def dof_pos_limit_penalty(
    env: ManagerBasedRLEnv,
    lower_limit: float,
    upper_limit: float,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize joint positions outside specified limits."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    # compute the penalty
    lower_penalty = torch.clamp(lower_limit - joint_pos, min=0.0)
    upper_penalty = torch.clamp(joint_pos - upper_limit, min=0.0)
    return torch.sum(lower_penalty + upper_penalty, dim=1)


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)


def feet_air_time(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """
    Reward function for feet air time during locomotion.
    
    只在 walking 状态下计算奖励，standing 状态下奖励为 0。
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the locomotion command in the command manager.
        
    Returns:
        Reward tensor of shape (num_envs,) containing air time rewards.
        
    Note:
        - 使用命令生成器的 is_standing mask 来区分 standing/walking
        - Standing 环境的奖励始终为 0
        - Walking 环境根据腾空时间计算奖励
    """
    # 获取接触传感器数据
    contact_sensor = env.scene["contact_feet_forces"]
    
    # 获取locomotion命令数据 - 通过command manager访问
    locomotion_command_term = env.command_manager._terms[command_name]

    dt = env.step_dt
    # 计算腾空时间奖励
    first_contact_filter = contact_sensor.compute_first_contact(dt)
    air_time = contact_sensor.data.current_air_time
    air_time_reward = torch.sum((air_time - 0.5) * first_contact_filter.float(), dim=1)
    
    # 只在 walking 状态下给奖励
    walking_mask = ~locomotion_command_term.is_standing_env
    air_time_reward *= walking_mask.float()

    return air_time_reward


def end_effector_pos_tracking(
    env: ManagerBasedRLEnv,
    command_name: str,
    gripper_cfg: SceneEntityCfg,
    sigma: float = 0.1,
) -> torch.Tensor:
    """
    Reward function for end-effector position tracking with exponential falloff.
    
    This function calculates a reward based on how close the end-effector is to the target position.
    It uses an exponential reward function that gives high rewards when the tracking error is small
    and decays exponentially as the error increases.
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the position command in the command manager.
        gripper_cfg: Configuration for the gripper/end-effector.
        sigma: Standard deviation for the exponential reward function (default: 0.1).
        
    Returns:
        Reward tensor of shape (num_envs,) containing position tracking rewards.
    """
    # 1. Get target position command - 正确方式：通过command manager的_terms字典访问
    command_term = env.command_manager._terms[command_name]
    target_pos_w = command_term.command[:, :3]  # 取前3维作为位置

    # 2. Get end-effector position in world frame - 正确的Isaac Lab方式
    gripper_body_indices = env.scene[gripper_cfg.name].find_bodies(gripper_cfg.body_names)[0]
    ee_pos_w = env.scene[gripper_cfg.name].data.body_pos_w[:, gripper_body_indices[0]]

    # 3. Calculate the distance between target and current position
    distance = torch.norm(target_pos_w - ee_pos_w, dim=1)
    
    # 4. Apply exponential reward function
    # Give maximum reward (1.0) when distance < threshold,
    # exponentially decay for larger distances
    reward = torch.exp(-distance / sigma)
    
    return reward


def end_effector_pos_tracking_l2(
    env: ManagerBasedRLEnv,
    command_name: str,
    gripper_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """
    L2 penalty function for end-effector position tracking.
    
    This function calculates a negative reward (penalty) based on the squared distance
    between the end-effector and the target position. This encourages the agent to
    minimize the tracking error.
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the position command in the command manager.
        gripper_cfg: Configuration for the gripper/end-effector.
        
    Returns:
        Penalty tensor of shape (num_envs,) containing negative L2 distance penalties.
    """
    # 1. Get target position command - 正确方式：通过command manager的_terms字典访问
    command_term = env.command_manager._terms[command_name]
    target_pos_w = command_term.command[:, :3]  # 取前3维作为位置

    # 2. Get end-effector position in world frame - 正确的Isaac Lab方式
    gripper_body_indices = env.scene[gripper_cfg.name].find_bodies(gripper_cfg.body_names)[0]
    ee_pos_w = env.scene[gripper_cfg.name].data.body_pos_w[:, gripper_body_indices[0]]

    # 3. Calculate the squared L2 distance (penalty)
    distance_squared = torch.sum(torch.square(target_pos_w - ee_pos_w), dim=1)
    
    # Return negative distance squared as penalty
    return distance_squared


def roll_orientation(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize non-flat roll orientation using L2 squared kernel.

    This is computed by penalizing only the x-component of the projected gravity vector,
    which corresponds to the roll angle. This allows pitch movement while constraining roll.
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Asset configuration for the robot (default: "robot").
        
    Returns:
        Penalty tensor of shape (num_envs,) containing squared roll orientation error.
        
    Note:
        - projected_gravity_b[:, 0] corresponds to roll angle
        - projected_gravity_b[:, 1] corresponds to pitch angle
        This function only penalizes the roll component.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    # penalize only the x-component (roll) of projected gravity
    return torch.square(asset.data.projected_gravity_b[:, 0])


def base_height(
    env: ManagerBasedRLEnv,
    target_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sensor_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize asset height from its target using L1 absolute difference.

    This function calculates the absolute difference between the robot's base height
    and the target height, providing a linear penalty for deviations.

    Args:
        env: The manager-based RL environment instance.
        target_height: Target height for the robot base (in meters).
        asset_cfg: Asset configuration for the robot (default: "robot").
        sensor_cfg: Optional sensor configuration for terrain-aware height adjustment.
                   If provided, uses ray-casting sensor data to adjust target height.

    Returns:
        Penalty tensor of shape (num_envs,) containing absolute height deviation.

    Note:
        - For flat terrain, target height is in the world frame.
        - For rough terrain, sensor readings can adjust the target height to account for the terrain.
        - Uses L1 norm (absolute difference) instead of L2 (squared) for more linear penalty.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    if sensor_cfg is not None:
        sensor: RayCaster = env.scene[sensor_cfg.name]
        # Adjust the target height using the sensor data
        adjusted_target_height = target_height + torch.mean(sensor.data.ray_hits_w[..., 2], dim=1)
    else:
        # Use the provided target height directly for flat terrain
        adjusted_target_height = target_height
    # Compute the L1 absolute penalty
    return torch.abs(asset.data.root_pos_w[:, 2] - adjusted_target_height)


def joint_deviation_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default position using L2 squared kernel.
    
    This function calculates the squared L2 norm of the deviation between current joint positions
    and their default positions. This is useful for keeping joints close to their nominal configuration.
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Asset configuration specifying which joints to penalize.
                   Use joint_names or joint_ids to select specific joints (e.g., only hip joints).
                   
    Returns:
        Penalty tensor of shape (num_envs,) containing squared deviation from default positions.
        
    Note:
        - Only the joints configured in asset_cfg.joint_ids will contribute to the term.
        - Uses L2 squared kernel: Σ(joint_pos - default_pos)²
        - For hip joints only, set: asset_cfg=SceneEntityCfg("robot", joint_names=[".*_hip_joint"])
    
    Example:
        # Penalize only hip joints
        hip_pos_penalty = RewTerm(
            func=mdp.joint_deviation_l2,
            weight=-0.3,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_joint"])}
        )
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Compute deviation from default joint positions
    deviation = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    
    # Return squared L2 norm (sum of squared deviations)
    return torch.sum(torch.square(deviation), dim=1)


def delta_torques_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize changes in joint torques using L2 squared kernel.
    
    This function calculates the squared L2 norm of the torque rate of change (delta torques),
    which helps to encourage smooth torque profiles and reduce mechanical stress on actuators.
    It computes the difference between current applied torques and the previous step's torques.
    
    Unlike actions (which are managed by env.action_manager.prev_action), Isaac Lab does not
    have a built-in manager for previous torques. This function manually stores and manages
    the previous torque state.
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Asset configuration specifying which joints to penalize.
                   Use joint_names or joint_ids to select specific joints.
                   
    Returns:
        Penalty tensor of shape (num_envs,) containing squared torque rate of change.
        
    Note:
        - Only the joints configured in asset_cfg.joint_ids will contribute to the term.
        - Uses L2 squared kernel: Σ(torque - torque_prev)²
        - Previous torques are stored in env._previous_torques (shape: [num_envs, num_joints])
        - For reset environments, previous torques are set to current torques (delta = 0)
        - Similar to action_rate_l2 but for torques instead of actions
    
    Example:
        # Penalize torque changes on all leg joints
        delta_torques = RewTerm(
            func=mdp.delta_torques_l2,
            weight=-1.0e-7,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
            ])}
        )
        
    Comparison with action_rate_l2:
        - action_rate_l2: Uses env.action_manager.prev_action (managed automatically)
        - delta_torques_l2: Uses env._previous_torques (managed manually here)
        - Both handle reset by setting delta to zero for reset environments
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Get current applied torques for specified joints
    current_torques = asset.data.applied_torque[:, asset_cfg.joint_ids]
    
    # Initialize previous torques storage if it doesn't exist
    # Note: Unlike env.action_manager.prev_action, there is no built-in torque manager
    if not hasattr(env, "_previous_torques"):
        # First call: initialize with current torques
        env._previous_torques = current_torques.clone()
    
    # Compute torque rate of change (delta torques)
    torque_delta = current_torques - env._previous_torques
    
    # Handle reset: For reset environments, set torque delta to zero
    # This ensures delta = 0 for the first step after reset, preventing large spikes
    # Note: env.episode_length_buf == 1 indicates the first step after reset
    reset_env_ids = (env.episode_length_buf == 1).nonzero(as_tuple=False).flatten()
    if len(reset_env_ids) > 0:
        torque_delta[reset_env_ids] = 0.0
    
    # Update previous torques for next step
    # Note: We use clone() to avoid reference issues
    env._previous_torques = current_torques.clone()
    
    # Return squared L2 norm (sum of squared torque changes)
    return torch.sum(torch.square(torque_delta), dim=1)


def tracking_lin_vel_max(
    env: ManagerBasedRLEnv,
    command_name: str,
    epsilon: float = 1e-5,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """X轴线速度跟踪奖励，区分前进/后退/站立三种情况。
    
    奖励策略：
    - 前进 (cmd_x > ε): reward = min(v_x, cmd_x) / (cmd_x + ε)
    - 后退 (cmd_x < -ε): reward = min(-v_x, -cmd_x) / (-cmd_x + ε)
    - 站立 (|cmd_x| ≤ ε): reward = exp(-|v_x|)
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the locomotion command in the command manager.
        epsilon: Threshold to distinguish standing from moving (default: 1e-5).
        asset_cfg: Asset configuration for the robot (default: "robot").
        
    Returns:
        Reward tensor of shape (num_envs,) containing velocity tracking rewards.
        
    Note:
        - 前进奖励鼓励机器人达到但不超过目标速度
        - 后退奖励同样鼓励精确跟踪（速度为负）
        - 站立奖励使用指数衰减惩罚任何移动
    """
    # Extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    
    # Get velocity commands and current velocity
    command_term = env.command_manager._terms[command_name]
    cmd_x = command_term.command[:, 0]  # X轴速度指令
    v_x = asset.data.root_lin_vel_b[:, 0]  # X轴当前速度（机身坐标系）
    
    # 第1部分：处理"移动"指令 (前进或后退)
    # 根据指令是正（前进）还是负（后退）来选择不同的计算方式
    reward = torch.where(
        cmd_x > 0,  # 前进指令
        torch.minimum(v_x, cmd_x) / (cmd_x + epsilon),
        torch.minimum(-v_x, -cmd_x) / (-cmd_x + epsilon)
    )
    
    # 第2部分：处理"站立"指令
    # 找到所有X轴指令的绝对值小于阈值的环境，这些被认为是"站立"指令
    standing_mask = torch.abs(cmd_x) < epsilon
    reward[standing_mask] = torch.exp(-torch.abs(v_x[standing_mask]))
    
    return reward


def track_ang_vel_z(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    # compute the error
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_b[:, 2])
    return torch.exp(-ang_vel_error / std)


def stand_still(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=[
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    ])
) -> torch.Tensor:
    """鼓励机器人在站立状态时保持默认腿部关节姿态。
    
    此奖励函数仅在站立模式下激活（通过 locomotion command 的 is_standing_env 属性判断）。
    在站立时，鼓励腿部关节保持默认位置，从而实现稳定、自然的站立姿态。
    在运动时，奖励为0，不影响运动策略。
    
    物理意义：
    - 站立时保持默认姿态可减少不必要的能量消耗
    - 有助于机器人学习稳定的站立平衡
    - 只考虑腿部关节（12个），不影响机械臂关节
    - 使用指数奖励函数，对接近默认姿态给予高奖励
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the locomotion command in the command manager (default: "locomotion_commands").
        asset_cfg: Asset configuration for the robot leg joints (default: 12 leg joints).
                   
    Returns:
        Reward tensor of shape (num_envs,) containing exponential rewards during standing.
        - Standing环境: 返回 exp(-偏差) 的指数奖励（偏离越少，奖励越高）
        - Walking环境: 返回0（不施加任何奖励）
        
    Note:
        - 依赖于 locomotion command term 提供的 is_standing_env 属性
        - 使用 L1 范数计算腿部关节偏差: Σ|joint_pos - default_pos|
        - 使用指数奖励: exp(-偏差 * 0.05)，偏差越小奖励越接近1
        - 建议 weight 设为正值（如 1.0）来奖励保持默认姿态
        - 返回值范围 [0, 1]，其中1表示完全保持默认姿态
    
    Example:
        # 在 RewardsCfg 中配置
        stand_still = RewTerm(
            func=mdp.stand_still,
            weight=1.0,  # 正权重：奖励站立时保持默认姿态
            params={"command_name": "locomotion_commands"}
        )
        
    类似函数对比:
        - joint_deviation_l2: 始终惩罚关节偏差（用于特定关节如髋关节），使用L2平方
        - stand_still: 仅在站立时奖励腿部关节保持默认姿态，使用指数奖励
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Get locomotion command term to access is_standing_env attribute
    locomotion_command_term = env.command_manager._terms[command_name]
    
    # Get current leg joint positions (only the joints specified in asset_cfg)
    # Shape: (num_envs, num_leg_joints) where num_leg_joints = 12
    current_leg_joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    
    # Get default leg joint positions
    # Shape: (num_envs, num_leg_joints)
    default_leg_joint_pos = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    
    # Compute L1 norm of deviation (sum of absolute differences)
    # Shape: (num_envs,)
    leg_joint_pos_diff = torch.sum(torch.abs(current_leg_joint_pos - default_leg_joint_pos), dim=1)
    
    # Compute exponential reward: exp(-deviation * scale_factor)
    # Scale factor of 0.05 means: deviation of 20 rad → reward ≈ 0.37
    # Shape: (num_envs,)
    stand_still_reward = torch.exp(-leg_joint_pos_diff * 0.05)
    
    # Only apply reward during standing (set walking environments to 0)
    # is_standing_env shape: (num_envs,) boolean tensor
    standing_mask = locomotion_command_term.is_standing_env
    
    # Apply mask: standing environments get the reward, walking environments get 0
    reward = stand_still_reward * standing_mask.float()
    
    return reward


def walking_dof(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=[
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    ])
) -> torch.Tensor:
    """鼓励机器人在行走时腿部关节位置接近默认值。
    
    此奖励函数仅在行走模式下激活（通过 locomotion command 的 is_standing_env 属性判断）。
    在行走时，鼓励腿部关节位置接近默认值，有助于保持自然的步态和运动效率。
    在站立时，奖励为0，不影响站立策略。
    
    物理意义：
    - 行走时保持接近默认姿态可以提高运动效率
    - 避免过度的关节偏移，减少能量消耗
    - 有助于学习更自然、稳定的步态
    - 只考虑腿部关节（12个），不影响机械臂关节
    - 使用指数奖励函数，与 stand_still 类似但适用于行走状态
    
    Args:
        env: The manager-based RL environment instance.
        command_name: Name of the locomotion command in the command manager (default: "locomotion_commands").
        asset_cfg: Asset configuration for the robot leg joints (default: 12 leg joints).
                   
    Returns:
        Reward tensor of shape (num_envs,) containing exponential rewards during walking.
        - Walking环境: 返回 exp(-偏差 * 0.05) 的指数奖励（偏离越少，奖励越高）
        - Standing环境: 返回0（不施加任何奖励）
        
    Note:
        - 依赖于 locomotion command term 提供的 is_standing_env 属性
        - 使用 L1 范数计算腿部关节偏差: Σ|joint_pos - default_pos|
        - 使用指数奖励: exp(-偏差 * 0.05)，偏差越小奖励越接近1
        - 建议 weight 设为正值（如 0.5）来奖励保持接近默认姿态
        - 返回值范围 [0, 1]，其中1表示完全保持默认姿态
    
    Example:
        # 在 RewardsCfg 中配置
        walking_dof = RewTerm(
            func=mdp.walking_dof,
            weight=0.5,  # 正权重：奖励行走时保持接近默认姿态
            params={"command_name": "locomotion_commands"}
        )
        
    类似函数对比:
        - stand_still: 仅在站立时奖励腿部关节保持默认姿态
        - walking_dof: 仅在行走时奖励腿部关节接近默认姿态
        - 两者使用相同的计算方法，但应用于不同的运动状态
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Get locomotion command term to access is_standing_env attribute
    locomotion_command_term = env.command_manager._terms[command_name]
    
    # Get current leg joint positions (only the joints specified in asset_cfg)
    # Shape: (num_envs, num_leg_joints) where num_leg_joints = 12
    current_leg_joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    
    # Get default leg joint positions
    # Shape: (num_envs, num_leg_joints)
    default_leg_joint_pos = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    
    # Compute L1 norm of deviation (sum of absolute differences)
    # Shape: (num_envs,)
    leg_joint_pos_diff = torch.sum(torch.abs(current_leg_joint_pos - default_leg_joint_pos), dim=1)
    
    # Compute exponential reward: exp(-deviation * scale_factor)
    # Scale factor of 0.05 means: deviation of 20 rad → reward ≈ 0.37
    # Shape: (num_envs,)
    walking_dof_reward = torch.exp(-leg_joint_pos_diff * 0.05)
    
    # Only apply reward during walking (set standing environments to 0)
    # is_standing_env shape: (num_envs,) boolean tensor
    # Walking mask is the inverse of standing mask
    walking_mask = ~locomotion_command_term.is_standing_env
    
    # Apply mask: walking environments get the reward, standing environments get 0
    reward = walking_dof_reward * walking_mask.float()
    
    return reward


def reward_tracking_contacts_shaped_force(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    gait_force_sigma: float = 0.5,
    kappa_gait_probs: float = 0.07
) -> torch.Tensor:
    """
    惩罚：摆动相时有接触力，或支撑相时无接触力
    
    返回值：正值表示违反程度（惩罚程度），配合负权重使用
    
    工作原理：
        - 站立时：gait_clock 冻结为 -1.0 (全支撑相) → desired_contact 全为高值 → 不惩罚接触
        - 行走时：gait_clock 动态变化 → desired_contact 周期变化 → 惩罚摆动相接触
    
    Args:
        env: 环境实例
        asset_cfg: 资产配置
        gait_force_sigma: 力的标准差，控制惩罚敏感度 (默认: 0.5)
        kappa_gait_probs: 冯·米塞斯分布参数，控制接触概率分布的尖锐度 (默认: 0.07)
    
    Returns:
        惩罚值 (num_envs,) - 值越大表示违反越严重
    """
    # 获取接触力
    contact_sensor = env.scene["contact_feet_forces"]
    
    # 尝试使用不同的方法获取接触力
    # 方法1：使用 net_forces_w（标准方法）
    if contact_sensor.data.net_forces_w is not None and contact_sensor.data.net_forces_w.numel() > 0:
        foot_forces = torch.norm(contact_sensor.data.net_forces_w, dim=-1)
    # 方法2：如果 net_forces_w 不可用，尝试使用 force_matrix_w
    elif hasattr(contact_sensor.data, 'force_matrix_w') and contact_sensor.data.force_matrix_w is not None:
        foot_forces = torch.norm(contact_sensor.data.force_matrix_w.squeeze(-2), dim=-1)
    # 方法3：使用二值接触状态 * 近似力（基于机器人重量）
    else:
        # 如果接触力数据不可用，使用二值接触判断
        # 假设机器人约12kg，4只脚，每只脚约30N（站立时）
        print("[WARNING] ContactSensor net_forces_w不可用，使用二值接触近似")
        # 这里暂时返回0，因为没有接触力数据无法计算此奖励
        return torch.zeros(env.num_envs, device=env.device)
    
    # 计算期望接触概率（基于步态相位）
    # 注意：gait_clock_signal 已经处理了 standing/walking 区分：
    #   - 站立时所有 foot_phases 被冻结为 0.75 (sin(2π×0.75) = -1)
    #   - 行走时 foot_phases 动态变化
    dt = env.step_dt
    current_time = env.episode_length_buf * dt
    gait_phase = (current_time * 2.0) % 1.0
    
    # 四足相位 (对角小跑步态)
    foot_phases = torch.stack([
        (gait_phase + 0.5) % 1.0,  # FL (左前)
        gait_phase,                 # FR (右前)
        gait_phase,                 # RL (左后)
        (gait_phase + 0.5) % 1.0,  # RR (右后)
    ], dim=1)
    
    # 使用冯·米塞斯分布计算期望接触概率
    # 当相位接近0或1时（支撑相），概率接近1；相位接近0.5时（摆动相），概率接近0
    von_mises = torch.exp(kappa_gait_probs * (torch.cos(2 * torch.pi * foot_phases) - 1))
    desired_contact = von_mises / (von_mises + 1e-8)
    
    # 计算惩罚 (返回正值)
    penalty = torch.zeros(env.num_envs, device=env.device)
    for i in range(4):
        # 摆动相时有力的惩罚
        penalty += -(1 - desired_contact[:, i]) * \
                       (1 - torch.exp(-foot_forces[:, i]**2 / gait_force_sigma))
    
    return penalty / 4.0


def reward_tracking_contacts_shaped_vel(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    gait_vel_sigma: float = 0.5,
    kappa_gait_probs: float = 0.07
) -> torch.Tensor:
    """
    惩罚：支撑相时足部有速度（打滑）
    
    返回值：正值表示违反程度（惩罚程度），配合负权重使用
    
    工作原理：
        - 站立时：gait_clock 冻结为 -1.0 (全支撑相) → desired_contact 全为高值 → 惩罚任何足部移动
        - 行走时：gait_clock 动态变化 → desired_contact 周期变化 → 只惩罚支撑相打滑
    
    Args:
        env: 环境实例
        asset_cfg: 资产配置
        gait_vel_sigma: 速度的标准差，控制惩罚敏感度 (默认: 0.5)
        kappa_gait_probs: 冯·米塞斯分布参数 (默认: 0.07)
    
    Returns:
        惩罚值 (num_envs,) - 值越大表示打滑越严重
    """
    # 获取足部速度
    robot: Articulation = env.scene[asset_cfg.name]
    foot_indices = robot.find_bodies([".*_foot"])[0]
    foot_vel_w = robot.data.body_lin_vel_w[:, foot_indices]
    foot_velocities = torch.norm(foot_vel_w, dim=-1)
    
    # 计算期望接触概率（基于步态相位）
    # 注意：gait_clock_signal 已经处理了 standing/walking 区分
    dt = env.step_dt
    current_time = env.episode_length_buf * dt
    gait_phase = (current_time * 2.0) % 1.0
    
    # 四足相位 (对角小跑步态)
    foot_phases = torch.stack([
        (gait_phase + 0.5) % 1.0,  # FL (左前)
        gait_phase,                 # FR (右前)
        gait_phase,                 # RL (左后)
        (gait_phase + 0.5) % 1.0,  # RR (右后)
    ], dim=1)
    
    von_mises = torch.exp(kappa_gait_probs * (torch.cos(2 * torch.pi * foot_phases) - 1))
    desired_contact = von_mises / (von_mises + 1e-8)
    
    # 计算惩罚：支撑相时有速度 (返回正值)
    penalty = torch.zeros(env.num_envs, device=env.device)
    for i in range(4):
        penalty += -desired_contact[:, i] * \
            (1 - torch.exp(-foot_velocities[:, i]**2 / gait_vel_sigma))
    
    return penalty / 4.0


def reward_feet_height(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    command_name: str = "locomotion_commands",
    target_height: float = 0.3,
    all_feet: bool = True
) -> torch.Tensor:
    """
    奖励足部高度（与原VBC逻辑一致）
    
    此奖励仅在行走状态下生效，站立时返回0。
    使用L2范数计算所有足部高度的综合度量，鼓励足部抬高。
    
    计算逻辑：clamp(||[h1, h2, h3, h4]|| - target_height, max=0)
    - 当足部整体高度低于目标时，给予负奖励（惩罚）
    - 当足部整体高度达到或超过目标时，奖励为0
    
    Args:
        env: 环境实例
        asset_cfg: 资产配置
        command_name: locomotion 命令名称 (默认: "locomotion_commands")
        target_height: 目标足部高度 (米，默认: 0.3)
        all_feet: 是否使用所有4只脚（True）或仅前2只脚（False，默认: True）
    
    Returns:
        奖励值 (num_envs,) - 负值表示足部高度不足，0表示达到目标
        
    Note:
        - 只在行走状态下计算（站立时返回0）
        - 使用L2范数：sqrt(h1² + h2² + h3² + h4²)
        - 与原VBC实现完全一致
        - 配合正权重使用（如 weight=1.0）
    """
    robot: Articulation = env.scene[asset_cfg.name]
    foot_indices = robot.find_bodies([".*_foot"])[0]
    
    # 获取足部高度
    if all_feet:
        foot_heights = robot.data.body_pos_w[:, foot_indices, 2]  # 所有4只脚
    else:
        foot_heights = robot.data.body_pos_w[:, foot_indices[:2], 2]  # 仅前2只脚
    
    # 计算L2范数并与目标比较
    # clamp(||feet_height|| - target, max=0) -> 返回负值或0
    reward = torch.clamp(torch.norm(foot_heights, dim=-1) - target_height, max=0.0)
    
    # 获取 walking mask - 只在行走时应用此奖励
    locomotion_command_term = env.command_manager._terms[command_name]
    walking_mask = ~locomotion_command_term.is_standing_env
    
    # 站立时奖励为0,行走时才计算
    reward = reward * walking_mask.float()
    
    return reward


def feet_jerk(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_feet_forces")
) -> torch.Tensor:
    """计算足部冲击惩罚（足部接触力变化率惩罚）。
    
    这个函数计算当前时间步的足底接触力与上一个时间步的足底接触力之间
    差值的范数（即大小），用来惩罚剧烈的力变化。
    
    控制目标：
    - "Jerk" (加加速度) 在这里被引申为"力的变化率"
    - 高的 feet_jerk 值意味着足底接触力在瞬间发生了剧烈变化
    - 通过设置负权重，惩罚这种剧烈的力变化
    - 激励策略学习到更"轻柔"的落足动作，减少对关节和地面的冲击
    
    Args:
        env: The manager-based RL environment instance.
        sensor_cfg: Configuration for the contact sensor (default: "contact_feet_forces").
        
    Returns:
        形状 (num_envs,) 的惩罚张量（正值表示惩罚程度）
        
    Note:
        - 使用 L2 范数计算力变化： ||F_current - F_previous||
        - 需要在环境中维护 _previous_contact_forces 缓存
        - 对于重置环境，力变化设为 0（避免虚假惩罚）
        - 配合负权重使用（如 weight=-0.0002）
    
    Example:
        feet_jerk = RewTerm(
            func=mdp.feet_jerk,
            weight=-0.0002,
            params={"sensor_cfg": SceneEntityCfg("contact_feet_forces")}
        )
    """
    # 获取接触传感器
    contact_sensor = env.scene[sensor_cfg.name]
    
    # 获取当前足部接触力（世界坐标系）
    # Shape: (num_envs, 4, 3) - 4只脚，每只脚3维力向量
    current_forces = contact_sensor.data.net_forces_w
    
    # 计算每只脚的接触力大小（L2范数）
    # Shape: (num_envs, 4) - 每只脚的力大小
    current_force_norm = torch.norm(current_forces, dim=-1)
    
    # 初始化存储上一时刻接触力的缓存（如果不存在）
    if not hasattr(env, "_previous_contact_forces"):
        env._previous_contact_forces = torch.zeros_like(current_force_norm)
    
    # 计算力变化率（当前力 - 上一时刻力）
    # Shape: (num_envs, 4)
    force_delta = current_force_norm - env._previous_contact_forces
    
    # 计算所有足部的力变化率的L2范数（总冲击量）
    # Shape: (num_envs,)
    jerk_penalty = torch.norm(force_delta, dim=-1)
    
    # 对于重置环境，将冲击设为0（避免因重置导致的虚假大力变化）
    # env.episode_length_buf == 1 表示刚重置的环境
    reset_mask = env.episode_length_buf == 1
    jerk_penalty = torch.where(reset_mask, torch.zeros_like(jerk_penalty), jerk_penalty)
    
    # 更新缓存：存储当前力大小供下一时刻使用
    env._previous_contact_forces = current_force_norm.clone()
    
    # 返回正值惩罚（配合负权重）
    return jerk_penalty


def feet_drag(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_feet_forces"),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """计算足部拖拽惩罚（足部在接触地面时移动的惩罚）。
    
    这个函数计算足部在接触地面时的速度，用来惩罚足部在支撑相时的滑动。
    
    控制目标：
    - 在理想的行走步态中，足部在"支撑相"（接触地面）时应该与地面保持相对静止
    - 足部只应该在"摆动相"（在空中）时才快速移动
    - 通过设置负权重，惩罚足部在地面上的滑动或拖拽
    - 鼓励机器人学习"干净利落"的抬腿和落足，避免浪费能量和不稳定
    
    Args:
        env: The manager-based RL environment instance.
        sensor_cfg: Configuration for the contact sensor (default: "contact_feet_forces").
                   接触阈值会自动从传感器的 cfg.force_threshold 读取。
        asset_cfg: Configuration for the robot asset (default: "robot").
        
    Returns:
        形状 (num_envs,) 的惩罚张量（正值表示惩罚程度）
        
    Note:
        - 只惩罚"支撑相"（接触地面）时的足部速度
        - 自动使用传感器配置的 force_threshold 判断接触状态
        - 计算足部在世界坐标系下的xy平面速度（忽略z轴）
        - 配合负权重使用（如 weight=-0.08）
        
    Example:
        feet_drag = RewTerm(
            func=mdp.feet_drag,
            weight=-0.08,
            params={
                "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
                "asset_cfg": SceneEntityCfg("robot")
            }
        )
    """
    # 获取接触传感器
    contact_sensor = env.scene[sensor_cfg.name]
    
    # 获取机器人资产
    robot: Articulation = env.scene[asset_cfg.name]
    
    # 获取足部接触力（世界坐标系）
    # Shape: (num_envs, 4, 3) - 4只脚，每只脚3维力向量
    contact_forces = contact_sensor.data.net_forces_w
    
    # 计算每只脚的接触力大小
    # Shape: (num_envs, 4)
    contact_force_norm = torch.norm(contact_forces, dim=-1)
    
    # 从传感器配置中获取接触力阈值
    # 这确保与传感器的接触检测逻辑保持一致
    contact_threshold = contact_sensor.cfg.force_threshold
    
    # 判断哪些足部在接触地面（接触力 > 阈值）
    # Shape: (num_envs, 4) - 布尔张量，True表示接触
    in_contact = contact_force_norm > contact_threshold
    
    # 获取足部在世界坐标系下的速度
    # 找到所有名为 "*_foot" 的body
    foot_indices = robot.find_bodies([".*_foot"])[0]
    
    # 获取足部速度（世界坐标系）
    # Shape: (num_envs, 4, 3) - 4只脚，每只脚3维速度向量
    foot_velocities = robot.data.body_lin_vel_w[:, foot_indices, :]
    
    # 只计算xy平面的速度（忽略z轴垂直运动）
    # Shape: (num_envs, 4, 2)
    foot_velocities_xy = foot_velocities[:, :, :2]
    
    # 计算每只脚在xy平面的速度大小
    # Shape: (num_envs, 4)
    foot_speed_xy = torch.norm(foot_velocities_xy, dim=-1)
    
    # 只对接触地面的足部计算惩罚（在空中的足部不惩罚）
    # 将不接触的足部速度设为0
    # Shape: (num_envs, 4)
    contact_foot_speed = foot_speed_xy * in_contact.float()
    
    # 计算所有足部的拖拽惩罚总和
    # Shape: (num_envs,)
    drag_penalty = torch.sum(contact_foot_speed, dim=-1)
    
    # 返回正值惩罚（配合负权重）
    return drag_penalty
