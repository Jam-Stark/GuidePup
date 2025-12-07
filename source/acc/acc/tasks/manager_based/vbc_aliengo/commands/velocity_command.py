# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""自定义速度命令生成器 - 支持站立环境同时设置线速度和角速度为0。"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Sequence
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import quat_from_euler_xyz

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# 导入父类和配置类
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg


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
    """

    cfg: UniformVelocityCommandCfg
    """命令配置"""

    def __init__(self, cfg: UniformVelocityCommandCfg, env: ManagerBasedRLEnv):
        """初始化命令生成器。
        
        Args:
            cfg: 命令配置
            env: 环境实例
        """
        super().__init__(cfg, env)
        
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
        env_ids_tensor = torch.tensor(env_ids, dtype=torch.long, device=self.device)

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
            # set their visibility to true
            self.goal_vel_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_vel_visualizer"):
                self.goal_vel_visualizer.set_visibility(False)

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
        # 计算箭头的方向（基于速度命令）
        # 只考虑 xy 平面的速度
        vel_command = self.command[:, :2]  # (num_envs, 2) - [vx, vy]
        
        # 计算偏航角（速度方向）
        yaw = torch.atan2(vel_command[:, 1], vel_command[:, 0])  # (num_envs,)
        
        # 从偏航角创建四元数
        zeros = torch.zeros_like(yaw)
        arrow_quat_w = quat_from_euler_xyz(zeros, zeros, yaw)
        
        # display markers
        self.goal_vel_visualizer.visualize(base_pos_w, arrow_quat_w)

