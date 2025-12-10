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
    """Computes the target position command in the base frame."""
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get target position command - access command data correctly using Isaac Lab API
    command_term = env.command_manager._terms[command_name]
    target_pos_w = command_term.command[:, :3]  # First 3 components are position

    # 3. Transform target position to robot base frame
    target_pos_b = transform_points(target_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)
    
    return target_pos_b.view(env.num_envs, -1)


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
    target_pos_w = env.command_manager._terms[command_name].command[:, :3]

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
    """
    Get the randomized mass parameters (5 dimensions).
    
    Returns:
        Tensor of shape (num_envs, 5) containing:
        - base_mass_delta: Added mass to base (kg)
        - com_x, com_y, com_z: Center of mass offset (m)
        - gripper_mass_delta: Added mass to gripper (kg)
    
    Note: These values are set during startup/reset events and cached in the environment.
    """
    # Initialize if not exists
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    return env._privileged_mass_params


def privileged_friction(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Get the randomized friction coefficient (1 dimension).
    
    Returns:
        Tensor of shape (num_envs, 1) containing static_friction value.
    
    Note: This value is set during reset events and cached in the environment.
    """
    # Initialize if not exists
    if not hasattr(env, '_privileged_friction'):
        env._privileged_friction = torch.ones(env.num_envs, 1, device=env.device)
    
    return env._privileged_friction


def privileged_motor_strengths(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Get the randomized motor strength scaling factors (12 dimensions).
    
    Returns:
        Tensor of shape (num_envs, 12) containing stiffness scaling factors
        for the 12 leg joints (FL/FR/RL/RR hip, thigh, calf).
    
    Note: These values are set during reset events and cached in the environment.
    """
    # Initialize if not exists
    if not hasattr(env, '_privileged_motor_strengths'):
        env._privileged_motor_strengths = torch.ones(env.num_envs, 12, device=env.device)
    
    return env._privileged_motor_strengths


# ========== Gait Clock Observations ==========

def gait_clock_signal(
    env: ManagerBasedRLEnv,
    command_name: str = "locomotion_commands",
    gait_frequency: float = 2.0,
    phase_offset: float = 0.5
) -> torch.Tensor:
    """
    计算步态时钟信号（4维，每只脚一个正弦波）
    
    实现 VBC 论文中的显式步态时钟，用于引导机器人学习稳定的对角小跑步态。
    
    步态参数：
        - 频率: 2 Hz (周期 0.5 秒)
        - 步态模式: Trotting (小跑)
        - 支撑相/摆动相: 各占 50%
        - 相位差: 0.5 (180度，对角足同相)
    
    足部相位分配（实现对角小跑）：
        - 行走时 (walking):
            - FL (左前): gait_phase + 0.5
            - FR (右前): gait_phase + 0.0
            - RL (左后): gait_phase + 0.0
            - RR (右后): gait_phase + 0.5
        - 站立时 (standing):
            - 所有足部: 相位 = 0.0 (支撑相中期，sin(0) = 0 → 实际上 sin(2π×0) = 0)
            - 或者更准确地：相位使得 sin 值为 -1（完全支撑相）
    
    Args:
        env: The manager-based RL environment instance.
        command_name: locomotion 命令名称 (默认: "locomotion_commands")
        gait_frequency: 步态频率 (Hz)，默认 2.0 Hz
        phase_offset: 对角足之间的相位偏移，默认 0.5 (180度)
    
    Returns:
        Tensor of shape (num_envs, 4) 包含每只脚的正弦波时钟信号。
        - 行走时，值范围 [-1, 1]：
            - 接近 1: 摆动相的最高点
            - 接近 -1: 支撑相的中间点
            - 0: 相位转换点
        - 站立时，所有值为 -1（表示持续支撑相）
    """
    # 获取 locomotion 命令以判断站立/行走状态
    locomotion_command_term = env.command_manager._terms[command_name]
    is_standing = locomotion_command_term.is_standing_env
    
    # 获取仿真时间步长
    dt = env.step_dt
    
    # 计算当前 episode 的时间（秒）
    current_time = env.episode_length_buf.float() * dt
    
    # 主时钟相位 (0 到 1 循环)
    # 使用模运算确保相位在 [0, 1) 范围内循环
    gait_phase = (current_time * gait_frequency) % 1.0
    
    # 为四只脚分配相位（小跑步态：对角足同相）
    # 创建 shape (num_envs, 4) 的张量
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    
    # FL (左前) 和 RR (右后) 是一对对角足 - 相位 + offset
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    
    # FR (右前) 和 RL (左后) 是另一对对角足 - 相位 + 0
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL
    
    # 转换为正弦信号：sin(2π × phase)
    # 这提供了平滑的、周期性的时钟信号给策略网络
    clock_signals = torch.sin(2 * torch.pi * foot_phases)
    
    # 站立状态：所有足部都应该在支撑相
    # 设置时钟信号为 -1（对应 sin(2π × 0.75) = -1，表示支撑相中期）
    # 这告诉策略："所有足部应该保持接触地面"
    standing_signal = torch.full((env.num_envs, 4), -1.0, device=env.device)
    
    # 根据 is_standing mask 选择信号
    # 站立环境使用 standing_signal,行走环境使用 clock_signals
    final_signals = torch.where(
        is_standing.unsqueeze(1),  # (num_envs, 1) - 广播到 (num_envs, 4)
        standing_signal,
        clock_signals
    )
    
    return final_signals


def joint_pos_rel_scaled(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    scale: float = 1.0
) -> torch.Tensor:
    """关节位置相对于默认位置的偏移（带缩放）
    
    Args:
        env: 环境实例
        asset_cfg: 资产配置，指定要读取的关节
        scale: 缩放系数，用于归一化观测值
    
    Returns:
        缩放后的关节位置偏移 (num_envs, num_joints)
    """
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos_rel = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return joint_pos_rel * scale


def joint_vel_scaled(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    scale: float = 1.0
) -> torch.Tensor:
    """关节速度（带缩放）
    
    Args:
        env: 环境实例
        asset_cfg: 资产配置，指定要读取的关节
        scale: 缩放系数，用于归一化观测值
    
    Returns:
        缩放后的关节速度 (num_envs, num_joints)
    """
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.joint_vel[:, asset_cfg.joint_ids] * scale
