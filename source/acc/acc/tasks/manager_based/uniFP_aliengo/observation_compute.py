# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script defines the Markov Decision Process (MDP) for the acc environment.
"""

from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_inv

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import transform_points


def ee_pos_in_base_frame(env, gripper_cfg: SceneEntityCfg) -> torch.Tensor:
    """Computes the position of the end-effector in the base frame."""
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get end-effector position in world frame
    gripper_body_indices = env.scene[gripper_cfg.name].find_bodies(gripper_cfg.body_names)[0]
    ee_pos_w = env.scene[gripper_cfg.name].data.body_pos_w[:, gripper_body_indices[0]]

    # 3. Transform ee position to robot base frame
    ee_pos_b = transform_points(ee_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)

    return ee_pos_b.view(env.num_envs, -1)


def ee_goal_pos_in_base_frame(env, command_name: str) -> torch.Tensor:
    """Computes the target position command in the base frame (Cartesian)."""
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get target position command
    command_term = env.command_manager._terms[command_name]
    
    # Handle DynamicManipulationCommand which stores world pos in pos_w
    if hasattr(command_term, "pos_w"):
        target_pos_w = command_term.pos_w
    else:
        # Fallback for other commands (assuming they store Cartesian in command)
        target_pos_w = command_term.command[:, :3]

    # 3. Transform target position to robot base frame
    target_pos_b = transform_points(target_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)
    
    return target_pos_b.view(env.num_envs, -1)


def ee_goal_pos_in_sphere(env, command_name: str) -> torch.Tensor:
    """Computes the target position command in spherical coordinates."""
    command_term = env.command_manager._terms[command_name]
    # The command is already in spherical coordinates [r, p, y]
    return command_term.command[:, :3].view(env.num_envs, -1)


def ee_tracking_error_in_base_frame(env, command_name: str, gripper_cfg: SceneEntityCfg) -> torch.Tensor:
    """Computes the tracking error of the end-effector in the base frame.

    The error is the difference between the target position command and the current
    end-effector position, both expressed in the robot's base frame.
    """
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get target position command - correct way to access command data
    # Access command through _terms dictionary as official Isaac Lab API
    command_term = env.command_manager._terms[command_name]
    
    # Check if command term has pos_w property (DynamicManipulationCommand)
    if hasattr(command_term, "pos_w"):
        target_pos_w = command_term.pos_w
    else:
        target_pos_w = command_term.command[:, :3]

    # 3. Get end-effector position in world frame
    # We need to find the body index for the gripper
    gripper_body_indices = env.scene[gripper_cfg.name].find_bodies(gripper_cfg.body_names)[0]
    # Assuming we want the first found body
    ee_pos_w = env.scene[gripper_cfg.name].data.body_pos_w[:, gripper_body_indices[0]]

    # 4. Transform target and ee positions to robot base frame
    # Use Isaac Lab's transform_points utility which handles coordinate transformations
    # Note: transform_points transforms FROM world frame TO base frame using inverse transform
    target_pos_b = transform_points(target_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)
    ee_pos_b = transform_points(ee_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)

    # 5. Compute tracking error
    tracking_error = target_pos_b - ee_pos_b
    return tracking_error.view(env.num_envs, -1)


def base_orientation_2d(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """
    Get the 2D orientation of the base (roll and pitch from projected gravity).
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Configuration for the robot asset.
        
    Returns:
        Tensor of shape (num_envs, 2) containing x and y components of
        projected gravity.
    """
    # Get the robot asset
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Calculate projected gravity (roll and pitch information)
    # Get the rotation matrix from the robot's orientation
    quat_w = asset.data.root_quat_w
    
    # Transform gravity to body frame using quaternion
    gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=env.device)
    gravity_vec = gravity_vec.expand(env.num_envs, -1)
    
    # Use quat_apply to transform gravity vector to body frame
    proj_grav = quat_apply(quat_inv(quat_w), gravity_vec)
    
    return proj_grav[..., :2]


def foot_contact_binary(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """
    Get binary foot contact information based on contact force threshold.
    
    Args:
        env: The manager-based RL environment instance.
        sensor_cfg: Configuration for the contact sensor.
        threshold: Force threshold for determining contact.
        
    Returns:
        Tensor containing binary contact states for each foot.
    """
    # Get contact forces from the sensor
    contact_forces = env.scene[sensor_cfg.name].data.net_forces_w
    
    # Calculate the norm of contact forces and check against threshold
    contact_binary = (torch.norm(contact_forces, dim=-1) > threshold).float()
    
    return contact_binary


def ee_goal_orient_placeholder(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Placeholder for end-effector goal orientation (hardcoded to zeros).
    
    Args:
        env: The manager-based RL environment instance.
        
    Returns:
        Tensor of shape (num_envs, 3) filled with zeros.
    """
    return torch.zeros(env.num_envs, 3, device=env.device)


# ========== Privileged Observations ==========

def privileged_mass_params(env: ManagerBasedRLEnv) -> torch.Tensor:
    """返回缓存的质量随机化参数 (5维)。
    
    Returns:
        形状为 (num_envs, 5) 的张量，包含：
        - [0] base_mass_delta: 添加到基座的质量 (kg)
        - [1] com_x: 质心X轴偏移 (m)
        - [2] com_y: 质心Y轴偏移 (m)
        - [3] com_z: 质心Z轴偏移 (m)
        - [4] gripper_mass_delta: 添加到夹爪的质量 (kg)
    
    说明：这些值由 cache_base_mass_randomization 等事件函数按随机化分布采样并缓存，
    Isaac Lab 当前未提供从引擎回读已应用值的接口。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    return env._privileged_mass_params


def privileged_friction(env: ManagerBasedRLEnv) -> torch.Tensor:
    """返回缓存的摩擦随机化参数 (1维)。
    
    Returns:
        形状为 (num_envs, 1) 的张量，包含静摩擦系数值。
    
    说明：这个值由 cache_friction_randomization 事件函数按随机化分布采样并缓存。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_friction'):
        env._privileged_friction = torch.ones(env.num_envs, 1, device=env.device)
    
    return env._privileged_friction


def privileged_motor_strengths(env: ManagerBasedRLEnv) -> torch.Tensor:
    """从物理引擎读取随机化的电机刚度 (12维)。
    
    Returns:
        形状为 (num_envs, 12) 的张量，包含12个腿部关节的实际刚度值：
        - 关节顺序: FL hip, FL thigh, FL calf, FR hip, FR thigh, FR calf,
                   RL hip, RL thigh, RL calf, RR hip, RR thigh, RR calf
    
    说明：这些值由 cache_motor_strength_randomization 事件函数从执行器直接读取。
    从 articulation.actuators["legs"].stiffness 获取的是物理引擎中真实应用的刚度值。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_motor_strengths'):
        env._privileged_motor_strengths = torch.ones(env.num_envs, 12, device=env.device)
    
    return env._privileged_motor_strengths


# ========== Gait Clock Observations ==========

def _update_gait_state(
    env: ManagerBasedRLEnv,
    command_name: str,
    gait_frequency: float
) -> torch.Tensor:
    """
    统一更新步态相位状态，防止在同一步骤中多次调用导致重复更新。
    """
    # 1. 初始化状态变量
    if not hasattr(env, "gait_indices"):
        env.gait_indices = torch.zeros(env.num_envs, device=env.device)
    if not hasattr(env, "gait_last_update_step"):
        env.gait_last_update_step = -1

    # 2. 检查是否需要在当前步更新 (防重入)
    # env.common_step_counter 在每一步仿真开始时递增
    if env.common_step_counter > env.gait_last_update_step:
        dt = env.step_dt
        cycle_time = 1.0 / gait_frequency
        
        # 累加相位
        env.gait_indices = torch.remainder(env.gait_indices + dt / cycle_time, 1.0)
        
        # 处理站立重置
        try:
            locomotion_command_term = env.command_manager._terms[command_name]
            is_standing = locomotion_command_term.is_standing_env
            env.gait_indices[is_standing] = 0.0
        except KeyError:
            # 如果找不到命令，保持原样或打印警告
            pass
            
        # 更新时间戳
        env.gait_last_update_step = env.common_step_counter
    
    return env.gait_indices


def gait_clock_signal(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    gait_frequency: float = 2.0,
    phase_offset: float = 0.5
) -> torch.Tensor:
    """
    计算步态时钟信号（4维，每只脚一个正弦波）
    """
    # 1. 获取统一更新后的相位
    gait_phase = _update_gait_state(env, command_name, gait_frequency)
    
    # 2. 为四只脚分配相位（小跑步态：对角足同相）
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    
    # FL (左前) 和 RR (右后) 是一对对角足 - 相位 + offset
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    
    # FR (右前) 和 RL (左后) 是另一对对角足 - 相位 + 0
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL
    
    # 3. 转换为正弦信号
    clock_signals = torch.sin(2 * torch.pi * foot_phases)
    
    # 4. 处理站立状态输出
    # 虽然相位已重置为0，但为了明确物理含义，站立时输出 -1.0 (支撑相)
    locomotion_command_term = env.command_manager._terms[command_name]
    is_standing = locomotion_command_term.is_standing_env
    
    standing_signal = torch.full((env.num_envs, 4), -1.0, device=env.device)
    
    final_signals = torch.where(
        is_standing.unsqueeze(1),
        standing_signal,
        clock_signals
    )
    
    return final_signals


def gait_clock_signal(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    gait_frequency: float = 2.0
) -> torch.Tensor:
    """
    计算全局步态时钟信号（2维：sin, cos）
    """
    # 1. 获取统一更新后的相位
    gait_phase = _update_gait_state(env, command_name, gait_frequency)
    
    # 2. 计算 2维信号: [sin(2πφ), cos(2πφ)]
    # 站立时 gait_phase=0 -> [0, 1]
    clock_signals = torch.stack([
        torch.sin(2 * torch.pi * gait_phase),
        torch.cos(2 * torch.pi * gait_phase)
    ], dim=1)
    
    return clock_signals


def stance_mask(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    gait_frequency: float = 2.0,
    phase_offset: float = 0.5
) -> torch.Tensor:
    """
    计算步态接触计划（4维，0/1 二值信号）
    
    功能：明确告知哪只脚应该落地（支撑相）。
    基于 gait_clock_signal 的相位逻辑，将连续信号二值化。
    
    Args:
        env: 环境实例
        command_name: 命令名称
        gait_frequency: 步态频率
        phase_offset: 相位偏移
        
    Returns:
        Tensor (num_envs, 4):
            - 1.0: 计划支撑相 (Stance / Ground)
            - 0.0: 计划摆动相 (Swing / Air)
    """
    # 1. 获取统一更新后的相位
    gait_phase = _update_gait_state(env, command_name, gait_frequency)
    
    # 2. 为四只脚分配相位
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL
    
    # 3. 计算正弦波
    # 根据当前约定：负值是支撑相，正值是摆动相
    sin_waves = torch.sin(2 * torch.pi * foot_phases)
    
    # 4. 生成接触掩码 (0/1)
    # sin_waves <= 0 表示支撑相 (1.0)
    # sin_waves > 0  表示摆动相 (0.0)
    contact_schedule = (sin_waves <= 0.0).float()
    
    # 5. 处理站立状态
    locomotion_command_term = env.command_manager._terms[command_name]
    is_standing = locomotion_command_term.is_standing_env
    
    # 站立时全为支撑相 (1.0)
    contact_schedule[is_standing] = 1.0
    
    return contact_schedule

