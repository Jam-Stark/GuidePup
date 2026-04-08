# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""自定义速度命令生成器 - 支持站立环境同时设置线速度和角速度为0。"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Sequence
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# 导入父类和配置类
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from ..mdp.utils import base_yaw_to_world, get_base_yaw_quat, world_to_base_yaw


@configclass
class ExtendedUniformVelocityCommandCfg(UniformVelocityCommandCfg):
    """Configuration for the extended uniform velocity command generator."""
    rel_standing_envs: float = 0.0
    ang_vel_yaw_clip: float = 0.0
    lin_vel_x_clip: float = 0.0
    lin_vel_y_clip: float = 0.0
    base_force_ext_name: str = "base_force_ext"
    base_force_kd: float = 100.0
    debug_vis_speed_ref: float = 0.5
    debug_vis_min_scale: float = 0.15
    debug_vis_max_scale: float = 2.0


class ExtendedUniformVelocityCommand(UniformVelocityCommand):
    """扩展的均匀速度命令生成器 - 支持站立环境同时将角速度设为0。
    
    继承自 UniformVelocityCommand，主要区别：
    - 当环境被标记为"站立"时（通过 rel_standing_envs 控制），
      同时将 lin_vel_x, lin_vel_y, ang_vel_z 都设置为 0
    - 原版只将 lin_vel_x, lin_vel_y 设为 0，角速度仍然从范围中随机采样
    
    配置参数：
        - rel_standing_envs: 站立环境的比例 (0.0-1.0)
        - ranges: 速度范围配置
          - lin_vel_x: X方向线速度范围
          - lin_vel_y: Y方向线速度范围
          - ang_vel_z: Z轴角速度范围
          - heading: 航向角范围（如果启用）
        - ang_vel_yaw_clip: 偏航角速度裁剪阈值
        - lin_vel_x_clip: X方向线速度裁剪阈值
        - lin_vel_y_clip: Y方向线速度裁剪阈值
    """

    cfg: ExtendedUniformVelocityCommandCfg
    """命令配置"""

    def __init__(self, cfg: ExtendedUniformVelocityCommandCfg, env: ManagerBasedRLEnv):
        """初始化命令生成器。
        
        Args:
            cfg: 命令配置
            env: 环境实例
        """
        super().__init__(cfg, env)
        self.metrics.setdefault("force_error_vel_xy", torch.zeros(self.num_envs, device=self.device))
        
        # 注意：父类已经创建了以下属性：
        # - self.robot: 机器人资产
        # - self.vel_command_b: 速度命令缓冲区 (num_envs, 3)
        # - self.heading_target: 航向目标
        # - self.is_heading_env: 航向环境标记
        # - self.is_standing_env: 站立环境标记（父类已提供）

    def __str__(self) -> str:
        """命令的字符串表示"""
        msg = f"ExtendedUniformVelocityCommand (站立环境比例: {self.cfg.rel_standing_envs}):\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape)}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tHeading command: {self.cfg.heading_command}\n"
        msg += f"\tStanding envs ratio: {self.cfg.rel_standing_envs}\n"
        return msg

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        """更新度量指标 - 计算命令误差"""
        # 使用父类的实现即可
        super()._update_metrics()
        try:
            base_force_ext_w = self._env.command_manager.get_command(self.cfg.base_force_ext_name)
        except KeyError:
            return

        max_command_time = self.cfg.resampling_time_range[1]
        max_command_step = max_command_time / self._env.step_dt

        base_yaw_quat = get_base_yaw_quat(self._env)
        base_force_ext_l = world_to_base_yaw(base_force_ext_w, base_yaw_quat)
        base_kd = max(float(self.cfg.base_force_kd), 1e-6)

        base_lin_vel_cmd = torch.zeros_like(base_force_ext_l)
        base_lin_vel_cmd[:, :2] = self.vel_command_b[:, :2]
        base_lin_vel_offset = base_force_ext_l / base_kd + base_lin_vel_cmd

        base_lin_vel_w = quat_apply(self.robot.data.root_quat_w, self.robot.data.root_lin_vel_b)
        base_lin_vel_yaw = world_to_base_yaw(base_lin_vel_w, base_yaw_quat)

        error_xy = torch.norm(base_lin_vel_offset[:, :2] - base_lin_vel_yaw[:, :2], dim=-1)
        self.metrics["force_error_vel_xy"] += error_xy / max_command_step

    def _resample_command(self, env_ids: Sequence[int]):
        """为指定环境重新采样命令 - 支持站立环境同时设置角速度为0。
        
        与父类的区别：
        - 当环境被标记为"站立"时，同时将 lin_vel_x, lin_vel_y, ang_vel_z 都设置为 0
        - 父类只将 lin_vel_x, lin_vel_y 设为 0，角速度仍然从范围中随机采样
        
        Args:
            env_ids: 需要重新采样的环境ID列表
        """
        if len(env_ids) == 0:
            return

        # 转换为张量
        env_ids_tensor = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)

        # 1. 采样线速度和角速度（使用父类的范围配置）
        # -- linear velocity - x direction
        self.vel_command_b[env_ids_tensor, 0] = torch.empty(
            len(env_ids), device=self.device
        ).uniform_(*self.cfg.ranges.lin_vel_x)
        # -- linear velocity - y direction
        self.vel_command_b[env_ids_tensor, 1] = torch.empty(
            len(env_ids), device=self.device
        ).uniform_(*self.cfg.ranges.lin_vel_y)
        # -- angular velocity - z direction
        self.vel_command_b[env_ids_tensor, 2] = torch.empty(
            len(env_ids), device=self.device
        ).uniform_(*self.cfg.ranges.ang_vel_z)

        # 2. 处理航向命令（如果启用）
        if self.cfg.heading_command:
            # sample heading commands
            self.heading_target[env_ids_tensor] = torch.empty(
                len(env_ids), device=self.device
            ).uniform_(*self.cfg.ranges.heading)
            # update heading envs
            rand_samples = torch.rand(len(env_ids), device=self.device)
            self.is_heading_env[env_ids_tensor] = rand_samples <= self.cfg.rel_heading_envs

        # 3. 处理站立环境
        # sample which envs should be standing
        rand_samples = torch.rand(len(env_ids), device=self.device)
        self.is_standing_env[env_ids_tensor] = rand_samples <= self.cfg.rel_standing_envs
        
        # 【关键修改】将站立环境的所有速度（包括角速度）设为0
        standing_mask = self.is_standing_env[env_ids_tensor]
        if standing_mask.any():
            self.vel_command_b[env_ids_tensor[standing_mask], 0] = 0.0  # lin_vel_x = 0
            self.vel_command_b[env_ids_tensor[standing_mask], 1] = 0.0  # lin_vel_y = 0
            self.vel_command_b[env_ids_tensor[standing_mask], 2] = 0.0  # ang_vel_z = 0 （新增）

        # Apply clipping
        # Check if all components are within their respective clipping thresholds
        is_small_x = torch.abs(self.vel_command_b[env_ids_tensor, 0]) < self.cfg.lin_vel_x_clip
        is_small_y = torch.abs(self.vel_command_b[env_ids_tensor, 1]) < self.cfg.lin_vel_y_clip
        is_small_yaw = torch.abs(self.vel_command_b[env_ids_tensor, 2]) < self.cfg.ang_vel_yaw_clip
        
        # Only if ALL components are small, we set everything to zero (stand command)
        is_stand = is_small_x & is_small_y & is_small_yaw
        
        if is_stand.any():
            self.vel_command_b[env_ids_tensor[is_stand], 0] = 0.0
            self.vel_command_b[env_ids_tensor[is_stand], 1] = 0.0
            self.vel_command_b[env_ids_tensor[is_stand], 2] = 0.0

    def _update_command(self):
        """更新命令 - 无需操作，因为命令是静态的直到重采样"""
        pass

    def _set_debug_vis_impl(self, debug_vis: bool):
        """设置调试可视化。
        
        创建速度命令的可视化标记（箭头）。
        注意：父类会自动处理回调的注册，这里只需要设置可见性。
        """
        # set visibility of markers
        # note: parent only deals with callbacks. not their visibility
        if debug_vis:
            # create markers if necessary for the first time
            if not hasattr(self, "goal_vel_visualizer"):
                self.goal_vel_visualizer = VisualizationMarkers(self.cfg.goal_vel_visualizer_cfg)
                self._goal_marker_base_scale = self._resolve_marker_base_scale(self.cfg.goal_vel_visualizer_cfg)
            if not hasattr(self, "current_vel_visualizer") and hasattr(self.cfg, "current_vel_visualizer_cfg"):
                self.current_vel_visualizer = VisualizationMarkers(self.cfg.current_vel_visualizer_cfg)
                self._current_marker_base_scale = self._resolve_marker_base_scale(
                    self.cfg.current_vel_visualizer_cfg
                )
            # set their visibility to true
            self.goal_vel_visualizer.set_visibility(True)
            if hasattr(self, "current_vel_visualizer"):
                self.current_vel_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_vel_visualizer"):
                self.goal_vel_visualizer.set_visibility(False)
            if hasattr(self, "current_vel_visualizer"):
                self.current_vel_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        """调试可视化回调 - 更新速度箭头位置和方向。"""
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        
        # check if visualizer exists
        if not hasattr(self, "goal_vel_visualizer"):
            return

        # get marker location
        # -- base state
        base_pos_w = self.robot.data.root_pos_w.clone()
        base_pos_w[:, 2] += 0.5  # 在机器人上方0.5米显示箭头

        # -- resolve the scales and quaternions
        # 计算箭头的方向（基于速度命令 + 力偏置）
        # 只考虑 xy 平面的速度
        vel_command_b = self.command[:, :2]  # (num_envs, 2) - [vx, vy]
        base_lin_vel_offset = torch.zeros((self.num_envs, 3), device=self.device)
        base_lin_vel_offset[:, :2] = vel_command_b
        base_yaw_quat = None
        if hasattr(self, "_env") and hasattr(self._env, "command_manager"):
            base_yaw_quat = get_base_yaw_quat(self._env)
            base_force_ext_w = self._env.command_manager.get_command(self.cfg.base_force_ext_name)
            base_force_ext_l = world_to_base_yaw(base_force_ext_w, base_yaw_quat)
            base_kd = max(float(self.cfg.base_force_kd), 1e-6)
            base_lin_vel_offset = base_force_ext_l / base_kd + base_lin_vel_offset

        if base_yaw_quat is None:
            base_yaw_quat = get_base_yaw_quat(self._env)
        vel_command_w = base_yaw_to_world(base_lin_vel_offset, base_yaw_quat)

        # 计算偏航角（速度方向）
        yaw = torch.atan2(vel_command_w[:, 1], vel_command_w[:, 0])  # (num_envs,)

        # 从偏航角创建四元数
        zeros = torch.zeros_like(yaw)
        arrow_quat_w = quat_from_euler_xyz(zeros, zeros, yaw)

        # display markers (scale by command speed)
        goal_scales = self._apply_speed_scale(vel_command_w, self._goal_marker_base_scale)
        self.goal_vel_visualizer.visualize(base_pos_w, arrow_quat_w, scales=goal_scales)

        if hasattr(self, "current_vel_visualizer"):
            base_lin_vel_w = quat_apply(self.robot.data.root_quat_w, self.robot.data.root_lin_vel_b)
            yaw_current = torch.atan2(base_lin_vel_w[:, 1], base_lin_vel_w[:, 0])
            zeros = torch.zeros_like(yaw_current)
            current_quat_w = quat_from_euler_xyz(zeros, zeros, yaw_current)
            current_scales = self._apply_speed_scale(base_lin_vel_w, self._current_marker_base_scale)
            self.current_vel_visualizer.visualize(base_pos_w, current_quat_w, scales=current_scales)

    def _resolve_marker_base_scale(self, vis_cfg: VisualizationMarkersCfg) -> torch.Tensor:
        marker_cfg = None
        if hasattr(vis_cfg, "markers") and len(vis_cfg.markers) > 0:
            marker_cfg = vis_cfg.markers.get("arrow", None)
            if marker_cfg is None:
                marker_cfg = next(iter(vis_cfg.markers.values()))
        scale = getattr(marker_cfg, "scale", None) if marker_cfg is not None else None
        if scale is None:
            scale = (1.0, 1.0, 1.0)
        return torch.tensor(scale, device=self.device, dtype=torch.float)

    def _apply_speed_scale(self, vel_w: torch.Tensor, base_scale: torch.Tensor) -> torch.Tensor:
        speed = torch.norm(vel_w[:, :2], dim=-1)
        speed_ref = max(float(self.cfg.debug_vis_speed_ref), 1e-3)
        scale_factor = speed / speed_ref
        scale_factor = torch.clamp(
            scale_factor,
            min=float(self.cfg.debug_vis_min_scale),
            max=float(self.cfg.debug_vis_max_scale),
        )
        scales = base_scale.unsqueeze(0).repeat(vel_w.shape[0], 1)
        scales[:, 0] = scales[:, 0] * scale_factor
        return scales
