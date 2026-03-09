# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script defines the Markov Decision Process (MDP) for the acc environment.
"""

from __future__ import annotations

import torch
import math

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

    # 2. Get target position command in world frame
    # Access command through _terms dictionary as official Isaac Lab API
    target_pos_w = env.command_manager._terms[command_name].command[:, :3]

    # 3. Transform target position to robot base frame
    # Use Isaac Lab's transform_points utility which handles coordinate transformations
    # Note: transform_points transforms FROM world frame TO base frame using inverse transform
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


# custom_commands.py
from typing import Tuple
import numpy as np

from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.sim as sim_utils


class CircularPoseCommand(CommandTerm):
    """
    Generates a smooth circular trajectory for the end-effector target.
    The circle is drawn on the Y-Z vertical plane in front of the robot's base frame,
    creating a vertical circular motion in front of the robot.
    """

    cfg: "CircularPoseCommandCfg"

    def __init__(self, cfg: "CircularPoseCommandCfg", env: ManagerBasedRLEnv):
        """Initialize the command term."""
        super().__init__(cfg, env)
        
        # 初始化命令缓冲区 (位置 + 四元数)
        self._command = torch.zeros(self.num_envs, 7, device=self.device)
        
        # 计算默认姿态并复制到所有环境
        self._default_quat = torch.tensor(
            self.cfg.default_quat,
            device=self.device,
            dtype=torch.float32
        ).unsqueeze(0).repeat(self.num_envs, 1)

        # 计算圆心位置
        self._center = torch.tensor(
            self.cfg.center,
            device=self.device,
            dtype=torch.float32
        )
        
        # 初始化时间偏移量，用于处理环境重置
        self._time_offset = torch.zeros(self.num_envs, device=self.device, dtype=torch.float32)
        
        # 初始化步数计数器，用于动态半径计算
        self._step_count = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        
        # 初始化可视化标记器（如果启用）
        self._goal_markers = None
        if self.cfg.debug_vis:
            self._create_goal_markers()

    def _create_goal_markers(self):
        """创建末端执行器目标位置的可视化标记器。"""
        marker_cfg = VisualizationMarkersCfg(
            prim_path="/World/Visuals/ee_goal_markers",
            markers={
                "goal_sphere": sim_utils.SphereCfg(
                    radius=0.02,  # 小球半径 2cm
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(1.0, 0.0, 0.0)  # 红色
                    ),
                ),
                "goal_arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.1, 0.1, 0.1),  # 缩小箭头
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.0, 1.0, 0.0)  # 绿色
                    ),
                ),
            },
        )
        self._goal_markers = VisualizationMarkers(marker_cfg)

    @property
    def command(self) -> torch.Tensor:
        """The desired pose command (position + quaternion)."""
        return self._command

    def _update_command(self):
        """Update the command term."""
        # 更新步数计数器
        self._step_count += 1
        
        # 获取当前仿真时间 - 使用正确的Isaac Lab API
        sim_time = self._env.sim.current_time
        
        # 为每个环境计算调整后的时间
        adjusted_time = sim_time + self._time_offset
        
        # 计算当前在圆上的角度
        angles = 2 * math.pi * adjusted_time / self.cfg.period
        
        # 计算动态半径：基础半径 + 随步数增长的增量
        # 使用平方根增长来避免半径增长过快
        current_radius = self.cfg.base_radius + self.cfg.radius_growth_rate * torch.sqrt(self._step_count.float() / self.cfg.radius_growth_steps)
        
        # 限制最大半径以避免机械臂超出工作空间
        current_radius = torch.clamp(current_radius, min=self.cfg.base_radius, max=self.cfg.max_radius)
        
        # 获取机器人基座位置和姿态
        robot_base_pos_w = self._env.scene["robot"].data.root_pos_w
        robot_base_quat_w = self._env.scene["robot"].data.root_quat_w
        
        # 计算相对于基座的圆心偏移量
        center_offset = self._center.unsqueeze(0).repeat(self.num_envs, 1)
        
        # 计算圆上的相对位置偏移（在Y-Z垂直面上绕圆）- 使用动态半径
        circle_offset = torch.zeros_like(center_offset)
        circle_offset[:, 1] = current_radius * torch.cos(angles)  # y 坐标偏移（左右方向）
        circle_offset[:, 2] = current_radius * torch.sin(angles)  # z 坐标偏移（上下方向）
        
        # 总的相对位置偏移 (圆心偏移 + 圆上位置偏移)
        total_offset = center_offset + circle_offset
        
        # 将相对位置变换到世界坐标系中 (相对于机器人基座)
        target_pos_relative = quat_apply(robot_base_quat_w, total_offset)
        target_pos_w = robot_base_pos_w + target_pos_relative
        
        # 填充指令缓冲区
        self._command[:, 0:3] = target_pos_w
        self._command[:, 3:7] = self._default_quat
        
        # 更新可视化（如果启用）
        if self.cfg.debug_vis and self._goal_markers is not None:
            self._update_goal_visualization(target_pos_w)

    def _update_goal_visualization(self, target_positions: torch.Tensor):
        """更新目标位置的可视化。"""
        # 转换为numpy数组用于可视化
        positions_np = target_positions.detach().cpu().numpy()
        
        # 创建标记索引：所有环境使用球形标记（索引0）
        marker_indices = np.zeros(self.num_envs, dtype=np.int32)
        
        # 可视化目标位置
        self._goal_markers.visualize(
            translations=positions_np,
            marker_indices=marker_indices
        )

    def _reset(self, env_ids: torch.Tensor) -> None:
        """Reset the command term for specific environments."""
        if len(env_ids) == 0:
            return
        # 重置步数计数器
        self._step_count[env_ids] = 0
        self._resample_command(env_ids)

    def _resample_command(self, env_ids: torch.Tensor):
        """Resample the command term for specific environments."""
        if len(env_ids) == 0:
            return
            
        # 为重置的环境生成新的时间偏移量，可以选择随机化起始位置
        # 这里设置为0，从固定位置开始圆形轨迹
        self._time_offset[env_ids] = 0.0
        
        # 获取当前仿真时间并计算初始位置
        sim_time = self._env.sim.current_time
        adjusted_time = sim_time + self._time_offset[env_ids]
        angles = 2 * math.pi * adjusted_time / self.cfg.period
        
        # 获取对应环境的机器人基座位置和姿态
        robot_base_pos_w = self._env.scene["robot"].data.root_pos_w[env_ids]
        robot_base_quat_w = self._env.scene["robot"].data.root_quat_w[env_ids]
        
        # 计算动态半径（重置时从基础半径开始）
        current_radius = self.cfg.base_radius + self.cfg.radius_growth_rate * torch.sqrt(self._step_count[env_ids].float() / self.cfg.radius_growth_steps)
        current_radius = torch.clamp(current_radius, min=self.cfg.base_radius, max=self.cfg.max_radius)
        
        # 计算相对于基座的圆心偏移量
        center_offset = self._center.unsqueeze(0).repeat(len(env_ids), 1)
        
        # 计算圆上的相对位置偏移（在Y-Z垂直面上绕圆）- 使用动态半径
        circle_offset = torch.zeros_like(center_offset)
        circle_offset[:, 1] = current_radius * torch.cos(angles)  # y 坐标偏移（左右方向）
        circle_offset[:, 2] = current_radius * torch.sin(angles)  # z 坐标偏移（上下方向）
        
        # 总的相对位置偏移 (圆心偏移 + 圆上位置偏移)
        total_offset = center_offset + circle_offset
        
        # 将相对位置变换到世界坐标系中 (相对于机器人基座)
        target_pos_relative = quat_apply(robot_base_quat_w, total_offset)
        target_pos_w = robot_base_pos_w + target_pos_relative
        
        # 为给定的环境填充指令缓冲区
        self._command[env_ids, 0:3] = target_pos_w
        self._command[env_ids, 3:7] = self._default_quat[env_ids]

    def _update_metrics(self):
        """Update the metrics."""
        # 可以在这里计算一些度量指标，如跟踪误差等
        pass


@configclass
class CircularPoseCommandCfg(CommandTermCfg):
    """Configuration for the circular pose command generator."""

    # 必需字段：指定命令生成器的类类型
    class_type: type = CircularPoseCommand

    # 重采样时间范围（秒）
    resampling_time_range: Tuple[float, float] = (8.0, 8.0)

    # --- 圆形轨迹参数 ---
    center: Tuple[float, float, float] = (0.6, 0.0, 0.2)  # 圆心相对于机器人基座的位置 (x, y, z) - 机器人面前YZ垂直面
    
    # 动态半径参数
    base_radius: float = 0.3  # 初始基础半径（较小开始）
    max_radius: float = 0.6   # 最大半径限制（避免超出机械臂工作空间）
    radius_growth_rate: float = 0.5  # 半径增长速率
    radius_growth_steps: float = 1000.0  # 半径增长的步数基准（用于归一化）
    
    # 兼容性参数（保持向后兼容）
    radius: float = 0.5  # 如果不使用动态半径，可以作为 max_radius 的默认值
    
    period: float = 12.0  # 完整圆形轨迹的周期（秒）

    # 默认姿态 (单位四元数: w, x, y, z)
    default_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    # 调试可视化选项
    debug_vis: bool = True  # 是否启用末端执行器目标位置的可视化



