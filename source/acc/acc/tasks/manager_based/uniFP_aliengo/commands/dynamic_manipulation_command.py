# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Dynamic manipulation command generator with spherical coordinate sampling and trajectory interpolation."""

from __future__ import annotations

import math
import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils

from ..mdp.utils import base_yaw_to_world, get_base_yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from .dynamic_manipulation_command_cfg import DynamicManipulationCommandCfg


class DynamicManipulationCommand(CommandTerm):
    r"""动态操作命令生成器 - 球坐标系采样 + 轨迹插值。
    
    采样规范：
    1. 球坐标采样：l∈[0.4,0.95], p∈[-π/2.5,π/3], y∈[-1.2,1.2]
    2. 球心位置：固定世界Z=0.7m, 机身XY=(0.3,0.0), 仅偏航旋转
    3. 姿态增量：基础姿态(π/2,0,0) + 随机增量±0.5
    4. 轨迹插值：位置LERP(球坐标) + 姿态SLERP(四元数)
    5. 时间参数：运动[1.0,3.0]s + 保持[0.5,2.0]s
    6. 碰撞检测：X∈[-0.8,0.1], Y∈[-0.2,0.2], Z∈[-0.7,-0.05]
    7. 初始化：固定起点(0.5,π/8,0) → 固定终点(0.7,0,0), 姿态(π/2,0,0)
    
    命令格式 (7维): [radius, pitch, yaw, quat_w, quat_x, quat_y, quat_z]
    内部状态: 轨迹插值参数、起始/目标球坐标、时间计数器
    """

    cfg: DynamicManipulationCommandCfg

    def __init__(self, cfg: DynamicManipulationCommandCfg, env: ManagerBasedRLEnv):
        """初始化命令生成器。"""
        super().__init__(cfg, env)

        # 获取机器人资产
        self.robot: Articulation = env.scene[cfg.asset_name]
        body_ids, _ = self.robot.find_bodies(cfg.body_name)
        self._ee_body_id = body_ids[0] if len(body_ids) > 0 else None
        
        # 命令缓冲区: [radius, pitch, yaw, quat_w, quat_x, quat_y, quat_z]
        self._command = torch.zeros(self.num_envs, 7, device=self.device)
        
        # 额外缓冲区：世界坐标系下的位置 [x, y, z] (用于奖励计算和可视化)
        self._pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        
        # 轨迹状态缓冲区
        self._current_step = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._total_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        
        # 起始和目标球坐标 [l, p, y] (radius, pitch, yaw in spherical coords)
        self._start_sphere = torch.zeros(self.num_envs, 3, device=self.device)
        self._target_sphere = torch.zeros(self.num_envs, 3, device=self.device)
        
        # 起始和目标姿态四元数 [w, x, y, z]
        self._start_quat = torch.zeros(self.num_envs, 4, device=self.device)
        self._target_quat = torch.zeros(self.num_envs, 4, device=self.device)
        
        # 运动和保持步数
        self._move_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._hold_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        
        # 度量指标
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        
        # 控制周期 (秒)
        self._control_dt = env.step_dt  # 通常是 0.02秒 (仿真0.005 * 降频4)

    def __str__(self) -> str:
        """命令的字符串表示"""
        msg = "DynamicManipulationCommand (Spherical Sampling + Trajectory Interpolation):\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape)}\n"
        msg += f"\tSpherical ranges: l={self.cfg.ranges.radius}, p={self.cfg.ranges.pitch_angle}, y={self.cfg.ranges.yaw_angle}\n"
        msg += f"\tMove time: {self.cfg.move_time_range}, Hold time: {self.cfg.hold_time_range}\n"
        msg += f"\tControl dt: {self._control_dt:.4f}s"
        return msg

    @property
    def command(self) -> torch.Tensor:
        """返回命令张量。形状为 (num_envs, 7): [sphere(3), quat(4)]"""
        return self._command

    @property
    def pos_w(self) -> torch.Tensor:
        """返回世界坐标系下的目标位置。形状为 (num_envs, 3): [x, y, z]"""
        return self._pos_w

    def _update_metrics(self):
        """更新度量指标"""
        if hasattr(self.cfg, 'body_name') and self.cfg.body_name:
            body_ids = self.robot.find_bodies(self.cfg.body_name)[0]
            ee_pos_w = self.robot.data.body_pos_w[:, body_ids[0], :]
            target_pos_w = self._pos_w  # 使用缓存的世界坐标位置
            self.metrics["position_error"] = torch.norm(target_pos_w - ee_pos_w, dim=1)

    def _resample_command(self, env_ids: Sequence[int]):
        """为指定环境重新采样命令 - 球坐标采样 + 碰撞检测。
        
        采样流程：
        1. 采样目标球坐标 (带碰撞检测)
        2. 采样目标姿态增量
        3. 采样运动和保持时间
        4. 初始化轨迹插值状态
        
        Args:
            env_ids: 需要重新采样的环境ID列表
        """
        if len(env_ids) == 0:
            return
        
        n = len(env_ids)
        env_ids_tensor = torch.tensor(env_ids, dtype=torch.long, device=self.device).detach().clone()
        
        # 1. 判断是否为初始化 (首次采样)
        is_initial = self._total_steps[env_ids_tensor] == 0
        
        # 2. 保存旧目标（在修改之前）- 关键：避免起始姿态被覆盖
        if (~is_initial).any():
            old_target_sphere = self._target_sphere[env_ids_tensor[~is_initial]].clone()
            old_target_quat = self._target_quat[env_ids_tensor[~is_initial]].clone()
        
        # 3. 设置起始位置和姿态（必须在设置新目标之前）
        if is_initial.any():
            # 初始化时：固定起始位置 [0.5, π/8, 0.0]
            self._start_sphere[env_ids_tensor[is_initial], 0] = 0.5  # radius
            self._start_sphere[env_ids_tensor[is_initial], 1] = math.pi / 8  # pitch
            self._start_sphere[env_ids_tensor[is_initial], 2] = 0.0  # yaw
            
            # 初始化时：起始姿态 = 目标姿态（垂直向下）
            base_roll = math.pi / 2
            base_pitch = 0.0
            base_yaw = 0.0
            num_initial = is_initial.sum().item()
            init_roll = torch.full((num_initial,), base_roll, device=self.device)
            init_pitch = torch.full((num_initial,), base_pitch, device=self.device)
            init_yaw = torch.full((num_initial,), base_yaw, device=self.device)
            init_quat = quat_from_euler_xyz(init_roll, init_pitch, init_yaw)
            self._start_quat[env_ids_tensor[is_initial]] = init_quat
        
        if (~is_initial).any():
            # 非初始化时：旧目标成为新起点（保证平滑过渡）
            self._start_sphere[env_ids_tensor[~is_initial]] = old_target_sphere
            self._start_quat[env_ids_tensor[~is_initial]] = old_target_quat
        
        # 4. 设置新目标位置
        if is_initial.any():
            # 初始化时：固定目标位置 [0.7, 0.0, 0.0]
            self._target_sphere[env_ids_tensor[is_initial], 0] = 0.7  # radius
            self._target_sphere[env_ids_tensor[is_initial], 1] = 0.0  # pitch
            self._target_sphere[env_ids_tensor[is_initial], 2] = 0.0  # yaw
        
        if (~is_initial).any():
            # 非初始化时：随机采样目标位置 (带碰撞检测)
            non_initial_ids = env_ids_tensor[~is_initial]
            self._target_sphere[non_initial_ids] = self._sample_spherical_coordinates(non_initial_ids)
        
        # 5. 设置新目标姿态
        # 基础姿态: (π/2, 0, 0) - 垂直向下
        base_roll = math.pi / 2
        base_pitch = 0.0
        base_yaw = 0.0
        
        if is_initial.any():
            # 初始化时：目标姿态 = 起始姿态（固定的垂直向下）
            self._target_quat[env_ids_tensor[is_initial]] = self._start_quat[env_ids_tensor[is_initial]].clone()
        
        if (~is_initial).any():
            # 非初始化时：基础姿态 + 随机增量 ±0.5 弧度
            num_non_initial = (~is_initial).sum().item()
            delta_roll = sample_uniform(-0.5, 0.5, (num_non_initial,), self.device)
            delta_pitch = sample_uniform(-0.5, 0.5, (num_non_initial,), self.device)
            delta_yaw = sample_uniform(-0.5, 0.5, (num_non_initial,), self.device)
            
            target_roll = base_roll + delta_roll
            target_pitch = base_pitch + delta_pitch
            target_yaw = base_yaw + delta_yaw
            
            self._target_quat[env_ids_tensor[~is_initial]] = quat_from_euler_xyz(target_roll, target_pitch, target_yaw)
        
        # 6. 采样时间参数
        move_time = sample_uniform(
            self.cfg.move_time_range[0], self.cfg.move_time_range[1], n, self.device
        )
        hold_time = sample_uniform(
            self.cfg.hold_time_range[0], self.cfg.hold_time_range[1], n, self.device
        )
        
        # 转换为步数
        self._move_steps[env_ids_tensor] = (move_time / self._control_dt).long()
        self._hold_steps[env_ids_tensor] = (hold_time / self._control_dt).long()
        self._total_steps[env_ids_tensor] = self._move_steps[env_ids_tensor] + self._hold_steps[env_ids_tensor]
        
        # 7. 重置计数器
        self._current_step[env_ids_tensor] = 0

    def _update_command(self):
        """更新命令 - 执行轨迹插值。
        
        更新流程：
        1. 执行轨迹插值 (使用当前步数)
        2. 增加步数计数器
        3. 检查是否需要重采样 (为下一帧准备新轨迹)
        
        关键：先插值再增加步数，避免闪现问题
        """
        # 1. 先执行轨迹插值（使用当前步数）
        self._interpolate_trajectory()
        
        # 2. 增加步数计数器
        self._current_step += 1
        
        # 3. 检查需要重采样的环境（为下一帧准备）
        needs_resample = self._current_step >= self._total_steps
        if needs_resample.any():
            resample_env_ids = needs_resample.nonzero(as_tuple=False).squeeze(-1).tolist()
            self._resample(resample_env_ids)
    
    def _interpolate_trajectory(self):
        """执行轨迹插值 - 球坐标LERP + 四元数SLERP。
        
        插值策略：
        - 移动阶段（0 ≤ step < move_steps）：t ∈ [0, 1] 线性增长
        - 保持阶段（step ≥ move_steps）：t = 1.0 固定在目标
        """
        # 计算插值参数 t ∈ [0, 1]
        # 避免除零：当 move_steps <= 0 时，直接跳到目标（t=1）
        safe_move_steps = torch.clamp(self._move_steps.float(), min=1.0)
        t = torch.clamp(self._current_step.float() / safe_move_steps, 0.0, 1.0)
        t = t.unsqueeze(1)  # (num_envs, 1)
        
        # 1. 球坐标线性插值 (LERP)
        current_sphere = self._start_sphere * (1 - t) + self._target_sphere * t
        
        # 2. 球坐标转笛卡尔坐标
        current_pos_b = self._spherical_to_cartesian(current_sphere)
        
        # 3. 转换到世界坐标系
        current_pos_w = self._transform_to_world(current_pos_b)
        
        # 4. 四元数球面线性插值 (SLERP)
        current_quat = self._slerp(self._start_quat, self._target_quat, t.squeeze(1))
        
        # 5. 更新命令缓冲区
        self._command[:, :3] = current_sphere  # 存储球坐标 [r, p, y]
        self._command[:, 3:7] = current_quat
        
        # 6. 更新世界坐标位置缓存
        self._pos_w[:] = current_pos_w
    
    def _sample_spherical_coordinates(self, env_ids: torch.Tensor) -> torch.Tensor:
        """采样球坐标 (带轨迹碰撞检测)。
        
        采样范围：
        - 径向距离 l: [0.4, 0.95] 米
        - 俯仰角 p: [-π/2.5, π/3] 弧度
        - 方位角 y: [-1.2, 1.2] 弧度
        
        碰撞检测 (禁飞区 - 机器人躯干):
        - X: [-0.8, 0.1] 米
        - Y: [-0.2, 0.2] 米
        - Z: [-0.7, -0.05] 米
        - 采样10个轨迹中间点，检查是否穿过禁飞区
        
        Returns:
            球坐标张量 (num_envs, 3): [l, p, y]
        """
        n = len(env_ids)
        sphere_coords = torch.zeros(n, 3, device=self.device)
        valid_samples = torch.zeros(n, dtype=torch.bool, device=self.device)
        
        for attempt in range(self.cfg.max_collision_attempts):
            # 采样仍未成功的环境
            remaining_ids = (~valid_samples).nonzero(as_tuple=False).squeeze(-1)
            if len(remaining_ids) == 0:
                break
            
            m = len(remaining_ids)

            # 采样球坐标
            radius = sample_uniform(self.cfg.ranges.radius[0], self.cfg.ranges.radius[1], m, self.device)
            pitch = sample_uniform(self.cfg.ranges.pitch_angle[0], self.cfg.ranges.pitch_angle[1], m, self.device)
            yaw = sample_uniform(self.cfg.ranges.yaw_angle[0], self.cfg.ranges.yaw_angle[1], m, self.device)

            temp_sphere = torch.stack([radius, pitch, yaw], dim=1)

            # 执行轨迹碰撞检测 (检查从起点到目标的路径)
            collision_free = self._check_trajectory_collision(
                self._start_sphere[env_ids[remaining_ids]],
                temp_sphere,
                env_ids[remaining_ids]
            )
            
            # 更新有效样本
            sphere_coords[remaining_ids[collision_free]] = temp_sphere[collision_free]
            valid_samples[remaining_ids[collision_free]] = True
        
        # 如果仍有环境未找到有效采样，使用最后一次采样并发出警告
        if not valid_samples.all():
            import warnings
            warnings.warn(f"未能为 {(~valid_samples).sum().item()} 个环境找到无碰撞目标")
        
        return sphere_coords
    
    def _check_trajectory_collision(
        self,
        start_sphere: torch.Tensor,
        end_sphere: torch.Tensor,
        env_ids: torch.Tensor,
        num_samples: int = 10
    ) -> torch.Tensor:
        """检查轨迹是否与禁飞区碰撞。
        
        方法：在起点和终点之间线性插值num_samples个点，检查是否有任何点在禁飞区内。
        碰撞检测在相对于机器人根部的坐标系中进行。
        
        Args:
            start_sphere: (N, 3) 起始球坐标 [l, p, y]
            end_sphere: (N, 3) 目标球坐标 [l, p, y]
            env_ids: (N,) 环境ID
            num_samples: 轨迹采样点数量 (默认10)
            
        Returns:
            (N,) 布尔张量，True表示无碰撞（安全）
        """
        n = len(env_ids)
        
        # 在球坐标空间中线性插值轨迹点
        # t: (num_samples,) from 0 to 1
        t = torch.linspace(0, 1, num_samples, device=self.device)
        
        # 广播到 (N, num_samples, 3)
        start_expanded = start_sphere.unsqueeze(1)  # (N, 1, 3)
        end_expanded = end_sphere.unsqueeze(1)      # (N, 1, 3)
        t_expanded = t.view(1, num_samples, 1)      # (1, num_samples, 1)
        
        # 插值球坐标
        trajectory_sphere = start_expanded * (1 - t_expanded) + end_expanded * t_expanded  # (N, num_samples, 3)
        
        # 展平用于批量转换: (N*num_samples, 3)
        trajectory_sphere_flat = trajectory_sphere.reshape(-1, 3)
        
        # 转换为笛卡尔坐标（相对于球心）
        trajectory_cart_flat = self._spherical_to_cartesian(trajectory_sphere_flat)
        
        # 恢复形状: (N, num_samples, 3)
        trajectory_cart = trajectory_cart_flat.reshape(n, num_samples, 3)
        
        # 关键修改：将相对球心的坐标转换为相对机器人根部的坐标
        # 球心在机身坐标系: (0.3, 0.0, sphere_center_z - robot_z)
        # 但碰撞检测需要相对根部，所以需要减去球心偏移
        sphere_center_offset = torch.zeros(n, 1, 3, device=self.device)
        sphere_center_offset[:, :, 0] = self.cfg.sphere_center[0]  # X偏移 0.3m
        sphere_center_offset[:, :, 1] = self.cfg.sphere_center[1]  # Y偏移 0.0m
        
        # 获取机器人根部Z坐标
        robot_pos_w = self.robot.data.root_pos_w[env_ids]
        sphere_z_relative = self.cfg.sphere_center_z - robot_pos_w[:, 2]  # 球心相对根部的Z偏移
        sphere_center_offset[:, :, 2] = sphere_z_relative.unsqueeze(1)
        
        # 转换为相对根部的坐标（只需要旋转XY，Z保持相对偏移）
        robot_quat_w = self.robot.data.root_quat_w[env_ids]
        yaw_angle = torch.atan2(
            2 * (robot_quat_w[:, 0] * robot_quat_w[:, 3] + robot_quat_w[:, 1] * robot_quat_w[:, 2]),
            1 - 2 * (robot_quat_w[:, 2]**2 + robot_quat_w[:, 3]**2)
        )
        
        from isaaclab.utils.math import quat_from_euler_xyz, quat_apply
        robot_quat_yaw = quat_from_euler_xyz(
            torch.zeros_like(yaw_angle),
            torch.zeros_like(yaw_angle),
            yaw_angle
        )
        
        # 展平用于旋转
        trajectory_cart_flat2 = trajectory_cart.reshape(-1, 3)
        robot_quat_yaw_repeated = robot_quat_yaw[torch.arange(n, device=self.device).unsqueeze(1).repeat(1, num_samples).reshape(-1)]
        
        # 应用偏航旋转
        trajectory_relative_flat = quat_apply(robot_quat_yaw_repeated, trajectory_cart_flat2)
        trajectory_relative = trajectory_relative_flat.reshape(n, num_samples, 3)
        
        # 加上球心偏移得到相对根部的坐标
        trajectory_rel_root = trajectory_relative + sphere_center_offset
        
        # 碰撞检测：检查是否有任何点在禁飞区内（AABB内部，相对根部坐标系）
        in_collision_box = (
            (trajectory_rel_root[..., 0] >= self.cfg.collision_box.x_min) &
            (trajectory_rel_root[..., 0] <= self.cfg.collision_box.x_max) &
            (trajectory_rel_root[..., 1] >= self.cfg.collision_box.y_min) &
            (trajectory_rel_root[..., 1] <= self.cfg.collision_box.y_max) &
            (trajectory_rel_root[..., 2] >= self.cfg.collision_box.z_min) &
            (trajectory_rel_root[..., 2] <= self.cfg.collision_box.z_max)
        )  # (N, num_samples)
        
        # 如果轨迹上有任何一个点在碰撞区内，则该轨迹有碰撞
        has_collision = in_collision_box.any(dim=1)  # (N,)
        
        # 返回无碰撞的掩码（取反）
        collision_free = ~has_collision
        
        return collision_free
    
    def _spherical_to_cartesian(self, sphere_coords: torch.Tensor) -> torch.Tensor:
        """
        将球坐标转换为笛卡尔坐标，使用Isaac Gym/机器人学约定。
        
        Args:
            sphere_coords: [..., 3] 球坐标 [l, p, y]
                - l (length/rho): 径向距离
                - p (pitch): 仰角，与XY平面的夹角（-π/2 到 π/2）
                - y (yaw): 偏航角，在XY平面上的角度（-π 到 π）
        
        Returns:
            cart_coords: [..., 3] 笛卡尔坐标 [x, y, z]
        
        Isaac Gym约定（机器人学标准）：
            x = l * cos(p) * cos(y)
            y = l * cos(p) * sin(y) 
            z = l * sin(p)
        
        与标准数学约定的区别：
            - pitch是仰角（与XY平面夹角），而非极角（与Z轴夹角）
            - z分量使用sin(pitch)而非cos(theta)
        """
        radius = sphere_coords[..., 0]  # 径向距离
        pitch = sphere_coords[..., 1]   # 仰角（与XY平面夹角）
        yaw = sphere_coords[..., 2]     # 偏航角（在XY平面上的角度）

        # Isaac Gym/机器人学约定的转换公式
        x = radius * torch.cos(pitch) * torch.cos(yaw)
        y = radius * torch.cos(pitch) * torch.sin(yaw)
        z = radius * torch.sin(pitch)  # 关键区别：使用sin(pitch)而非cos(theta)
        
        cart_coords = torch.stack([x, y, z], dim=-1)
        return cart_coords
    
    def _transform_to_world(self, pos_b: torch.Tensor, env_ids: torch.Tensor = None) -> torch.Tensor:
        """将机身坐标系位置转换到世界坐标系。
        
        🔧 关键理解：
        - pos_b 是在"球心为原点的机身坐标系"中的位置
        - 球坐标的 yaw 是相对机身的角度，已经在 _spherical_to_cartesian 中应用
        - 这里只需要：旋转到世界坐标系 + 平移到球心世界位置
        
        转换流程：
        1. 获取机身偏航角（忽略俯仰和翻滚）
        2. 用偏航角旋转 pos_b（将机身坐标系方向转到世界坐标系）
        3. 计算球心世界坐标（机身世界位置 + 偏航旋转后的偏移）
        4. 加上球心偏移得到最终世界坐标
        
        Args:
            pos_b: (N, 3) 机身坐标系位置（相对球心）
            env_ids: 环境ID (可选)
            
        Returns:
            (N, 3) 世界坐标系位置
        """
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        
        # 1. 获取机器人基座位置和姿态
        robot_pos_w = self.robot.data.root_pos_w[env_ids]
        robot_quat_w = self.robot.data.root_quat_w[env_ids]
        
        # 2. 提取偏航角 (忽略俯仰和翻滚)
        yaw_angle = torch.atan2(
            2 * (robot_quat_w[:, 0] * robot_quat_w[:, 3] + robot_quat_w[:, 1] * robot_quat_w[:, 2]),
            1 - 2 * (robot_quat_w[:, 2]**2 + robot_quat_w[:, 3]**2)
        )
        
        # 3. 创建仅包含偏航的四元数
        robot_quat_yaw = quat_from_euler_xyz(
            torch.zeros_like(yaw_angle),
            torch.zeros_like(yaw_angle),
            yaw_angle
        )
        
        # 4. 旋转目标位置（从机身坐标系到世界坐标系方向）
        from isaaclab.utils.math import quat_apply
        pos_rotated = quat_apply(robot_quat_yaw, pos_b)
        
        # 5. 计算球心在世界坐标系中的位置
        # 球心在机身坐标系: (0.3, 0.0, Z相对偏移)
        sphere_center_b = torch.zeros_like(pos_b)
        sphere_center_b[:, 0] = self.cfg.sphere_center[0]  # 0.3
        sphere_center_b[:, 1] = self.cfg.sphere_center[1]  # 0.0
        sphere_center_b[:, 2] = 0.0  # Z在机身坐标系中暂时为0
        
        # 旋转球心偏移
        sphere_center_xy = quat_apply(robot_quat_yaw, sphere_center_b)
        
        # 球心世界坐标 = 机身世界位置 + 旋转后的XY偏移 + 固定的世界Z
        sphere_center_w = torch.zeros_like(pos_b)
        sphere_center_w[:, 0] = robot_pos_w[:, 0] + sphere_center_xy[:, 0]
        sphere_center_w[:, 1] = robot_pos_w[:, 1] + sphere_center_xy[:, 1]
        sphere_center_w[:, 2] = self.cfg.sphere_center_z  # 固定世界Z = 0.7
        
        # 6. 最终世界坐标 = 旋转后的位置 + 球心世界坐标
        pos_w = pos_rotated + sphere_center_w
        
        return pos_w
    
    def _slerp(self, q1: torch.Tensor, q2: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        """四元数球面线性插值 (SLERP)。
        
        Args:
            q1: (N, 4) 起始四元数 [w, x, y, z]
            q2: (N, 4) 目标四元数 [w, x, y, z]
            t: (N,) 插值参数 ∈ [0, 1]
            
        Returns:
            (N, 4) 插值后的四元数
        """
        # 确保四元数归一化
        q1 = q1 / torch.norm(q1, dim=1, keepdim=True)
        q2 = q2 / torch.norm(q2, dim=1, keepdim=True)
        
        # 计算点积
        dot = torch.sum(q1 * q2, dim=1)
        
        # 如果点积为负，反转一个四元数以选择较短路径
        q2 = torch.where(dot.unsqueeze(1) < 0, -q2, q2)
        dot = torch.abs(dot)
        
        # 如果四元数非常接近，使用线性插值
        DOT_THRESHOLD = 0.9995
        linear_interp = dot > DOT_THRESHOLD
        
        # SLERP公式
        theta = torch.acos(torch.clamp(dot, -1.0, 1.0))
        sin_theta = torch.sin(theta)
        
        # 避免除以零
        safe_sin_theta = torch.where(sin_theta > 1e-6, sin_theta, torch.ones_like(sin_theta))
        
        w1 = torch.sin((1 - t) * theta) / safe_sin_theta
        w2 = torch.sin(t * theta) / safe_sin_theta
        
        # 线性插值作为fallback
        w1_linear = 1 - t
        w2_linear = t
        
        # 选择插值权重
        w1 = torch.where(linear_interp, w1_linear, w1).unsqueeze(1)
        w2 = torch.where(linear_interp, w2_linear, w2).unsqueeze(1)
        
        return w1 * q1 + w2 * q2

    def _set_debug_vis_impl(self, debug_vis: bool):
        """设置调试可视化 - 显示目标/位置与力控信息"""
        if debug_vis:
            # 1. 目标/位置可视化
            if not hasattr(self, "ee_goal_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/EE/goal",
                    markers={
                        "sphere": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Shapes/sphere.usd",
                            scale=(0.06, 0.06, 0.06),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0)),
                        ),
                    },
                )
                self.ee_goal_markers = VisualizationMarkers(marker_cfg)
            self.ee_goal_markers.set_visibility(True)

            if not hasattr(self, "ee_actual_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/EE/actual",
                    markers={
                        "sphere": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Shapes/sphere.usd",
                            scale=(0.06, 0.06, 0.06),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.1, 0.6)),
                        ),
                    },
                )
                self.ee_actual_markers = VisualizationMarkers(marker_cfg)
            self.ee_actual_markers.set_visibility(True)

            if not hasattr(self, "ee_goal_offset_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/EE/goal_offset",
                    markers={
                        "sphere": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Shapes/sphere.usd",
                            scale=(0.06, 0.06, 0.06),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 1.0)),
                        ),
                    },
                )
                self.ee_goal_offset_markers = VisualizationMarkers(marker_cfg)
            self.ee_goal_offset_markers.set_visibility(True)

            # 2. 力控可视化
            if not hasattr(self, "gripper_force_ext_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/Force/gripper_ext",
                    markers={
                        "arrow": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                            scale=(1.0, 1.0, 1.0),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.4, 1.0)),
                        ),
                    },
                )
                self.gripper_force_ext_markers = VisualizationMarkers(marker_cfg)
            self.gripper_force_ext_markers.set_visibility(True)

            if not hasattr(self, "gripper_force_cmd_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/Force/gripper_cmd",
                    markers={
                        "arrow": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                            scale=(1.0, 1.0, 1.0),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                        ),
                    },
                )
                self.gripper_force_cmd_markers = VisualizationMarkers(marker_cfg)
            self.gripper_force_cmd_markers.set_visibility(True)

            if not hasattr(self, "base_force_ext_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/Force/base_ext",
                    markers={
                        "arrow": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                            scale=(1.0, 1.0, 1.0),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.4, 1.0)),
                        ),
                    },
                )
                self.base_force_ext_markers = VisualizationMarkers(marker_cfg)
            self.base_force_ext_markers.set_visibility(True)

            if not hasattr(self, "base_force_cmd_markers"):
                marker_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Command/Force/base_cmd",
                    markers={
                        "arrow": sim_utils.UsdFileCfg(
                            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                            scale=(1.0, 1.0, 1.0),
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                        ),
                    },
                )
                self.base_force_cmd_markers = VisualizationMarkers(marker_cfg)
            self.base_force_cmd_markers.set_visibility(True)
        else:
            for marker_name in (
                "ee_goal_markers",
                "ee_actual_markers",
                "ee_goal_offset_markers",
                "gripper_force_ext_markers",
                "gripper_force_cmd_markers",
                "base_force_ext_markers",
                "base_force_cmd_markers",
            ):
                if hasattr(self, marker_name):
                    getattr(self, marker_name).set_visibility(False)

    def _forces_to_quat(self, forces: torch.Tensor) -> torch.Tensor:
        eps = 1.0e-6
        mag = torch.linalg.norm(forces, dim=1)
        direction = forces / (mag.unsqueeze(-1) + eps)
        dot = torch.clamp(direction[:, 0], -1.0, 1.0)
        angle = torch.acos(dot)
        axis = torch.stack([torch.zeros_like(dot), -direction[:, 2], direction[:, 1]], dim=1)
        axis_norm = torch.linalg.norm(axis, dim=1, keepdim=True)
        default_axis = torch.zeros_like(axis)
        default_axis[:, 2] = 1.0
        axis = torch.where(axis_norm > eps, axis, default_axis)
        axis = axis / torch.linalg.norm(axis, dim=1, keepdim=True)
        quat = math_utils.quat_from_angle_axis(angle, axis)
        identity = torch.zeros((forces.shape[0], 4), device=forces.device)
        identity[:, 0] = 1.0
        return torch.where(mag.unsqueeze(-1) > eps, quat, identity)

    def _forces_to_scales(self, forces: torch.Tensor) -> torch.Tensor:
        mag = torch.linalg.norm(forces, dim=1)
        length = torch.clamp(mag * 0.02, max=0.6)
        thickness = torch.full_like(length, 0.05)
        return torch.stack([length, thickness, thickness], dim=1)

    def _debug_vis_callback(self, event):
        """调试可视化回调 - 目标/位置与力控信息"""
        # 检查环境是否仍然有效 (避免在关闭时报错)
        if not hasattr(self._env, "scene") or self._env.scene is None:
            return

        if not self.robot.is_initialized:
            return

        if self._ee_body_id is None:
            return

        base_yaw_quat = get_base_yaw_quat(self._env)
        ee_pos_w = self.robot.data.body_pos_w[:, self._ee_body_id, :]
        base_pos_w = self.robot.data.root_pos_w.clone()
        base_pos_w[:, 2] += 0.2

        gripper_force_ext_w = self._env.command_manager.get_command("gripper_force_ext")
        gripper_force_cmd_l = self._env.command_manager.get_command("gripper_force_command")
        gripper_force_cmd_w = base_yaw_to_world(gripper_force_cmd_l, base_yaw_quat)
        gripper_impedance_cmd = self._env.command_manager.get_command("gripper_impedance_command")
        kps = torch.clamp(gripper_impedance_cmd[:, 0], min=1.0e-6)
        ee_goal_w = self._pos_w
        ee_goal_offset_w = ee_goal_w + (gripper_force_ext_w + gripper_force_cmd_w) / kps.unsqueeze(-1)

        base_force_ext_w = self._env.command_manager.get_command("base_force_ext")
        base_force_cmd_l = self._env.command_manager.get_command("base_force_command")
        base_force_cmd_w = base_yaw_to_world(base_force_cmd_l, base_yaw_quat)

        if hasattr(self, "ee_goal_markers"):
            ee_goal_quat = torch.zeros((self.num_envs, 4), device=self.device)
            ee_goal_quat[:, 0] = 1.0
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            self.ee_goal_markers.visualize(ee_goal_w, ee_goal_quat, marker_indices=marker_indices)

        if hasattr(self, "ee_actual_markers"):
            ee_actual_quat = torch.zeros((self.num_envs, 4), device=self.device)
            ee_actual_quat[:, 0] = 1.0
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            self.ee_actual_markers.visualize(ee_pos_w, ee_actual_quat, marker_indices=marker_indices)

        if hasattr(self, "ee_goal_offset_markers"):
            ee_offset_quat = torch.zeros((self.num_envs, 4), device=self.device)
            ee_offset_quat[:, 0] = 1.0
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            self.ee_goal_offset_markers.visualize(ee_goal_offset_w, ee_offset_quat, marker_indices=marker_indices)

        if hasattr(self, "gripper_force_ext_markers"):
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            gripper_ext_quat = self._forces_to_quat(gripper_force_ext_w)
            gripper_ext_scales = self._forces_to_scales(gripper_force_ext_w)
            self.gripper_force_ext_markers.visualize(
                ee_pos_w, gripper_ext_quat, scales=gripper_ext_scales, marker_indices=marker_indices
            )

        if hasattr(self, "gripper_force_cmd_markers"):
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            gripper_cmd_quat = self._forces_to_quat(gripper_force_cmd_w)
            gripper_cmd_scales = self._forces_to_scales(gripper_force_cmd_w)
            self.gripper_force_cmd_markers.visualize(
                ee_pos_w, gripper_cmd_quat, scales=gripper_cmd_scales, marker_indices=marker_indices
            )

        if hasattr(self, "base_force_ext_markers"):
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            base_ext_quat = self._forces_to_quat(base_force_ext_w)
            base_ext_scales = self._forces_to_scales(base_force_ext_w)
            self.base_force_ext_markers.visualize(
                base_pos_w, base_ext_quat, scales=base_ext_scales, marker_indices=marker_indices
            )

        if hasattr(self, "base_force_cmd_markers"):
            marker_indices = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)
            base_cmd_quat = self._forces_to_quat(base_force_cmd_w)
            base_cmd_scales = self._forces_to_scales(base_force_cmd_w)
            self.base_force_cmd_markers.visualize(
                base_pos_w, base_cmd_quat, scales=base_cmd_scales, marker_indices=marker_indices
            )
