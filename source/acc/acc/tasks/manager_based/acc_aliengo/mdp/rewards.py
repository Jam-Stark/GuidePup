# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
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
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Asset configuration for the robot (default: "robot").
        
    Returns:
        Reward tensor of shape (num_envs,) containing air time rewards.
    """
    # 获取接触传感器数据
    contact_sensor = env.scene["contact_feet_forces"]
    
    # 正确获取locomotion命令数据 - 通过command manager访问
    locomotion_command = env.command_manager._terms[command_name]

    dt = env.step_dt
    # 计算腾空时间奖励
    first_contact_filter = contact_sensor.compute_first_contact(dt)
    air_time = contact_sensor.data.current_air_time
    air_time_reward = torch.sum((air_time - 0.5) * first_contact_filter.float(), dim=1)
    air_time_reward *= torch.norm(locomotion_command.command[:, :2], dim=1) > 0.1

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
    