# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for dynamic manipulation command generator with spherical sampling."""

from __future__ import annotations

import math
from dataclasses import MISSING
from typing import Tuple

from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.utils import configclass

from .dynamic_manipulation_command import DynamicManipulationCommand


@configclass
class DynamicManipulationCommandCfg(CommandTermCfg):
    """动态操作命令生成器配置 - 球坐标采样 + 轨迹插值。
    
    该配置定义了如何使用球坐标系采样目标位置，并通过轨迹插值生成平滑的操作命令。
    """

    class_type: type = DynamicManipulationCommand
    """命令生成器类型"""

    # 重采样时间范围 - 由轨迹时间控制，此处设为形式值
    resampling_time_range: Tuple[float, float] = (1.0, 1.0)
    """名义重采样时间。实际采样由轨迹完成触发。"""

    # 资产配置
    asset_name: str = MISSING
    """场景中的资产名称"""

    body_name: str = MISSING
    """末端执行器的body名称"""

    # 球心配置
    sphere_center: Tuple[float, float] = (0.3, 0.0)
    """球心在机身坐标系XY的位置 (x, y)，单位：米。
    默认 (0.3, 0.0) 表示机器人前方0.5米。
    """

    sphere_center_z: float = 0.7
    """球心在世界坐标系中的固定Z高度，单位：米。
    这个高度不随机器人俯仰/翻滚变化，仅跟随XY位置和偏航角。
    """

    # 时间配置
    move_time_range: Tuple[float, float] = (1.0, 3.0)
    """运动时长范围（秒）。
    机械臂从起点插值到目标所需的时间。
    """

    hold_time_range: Tuple[float, float] = (0.5, 2.0)
    """保持时长范围（秒）。
    机械臂到达目标后保持稳定的时间。
    """

    max_collision_attempts: int = 10
    """碰撞检测的最大尝试次数。"""

    @configclass
    class Ranges:
        """球坐标采样范围。"""

        radius: Tuple[float, float] = (0.4, 0.95)
        """径向距离范围 l（米）"""

        pitch_angle: Tuple[float, float] = (-math.pi / 2.5, math.pi / 3)
        """俯仰角范围 p（弧度）: [-π/2.5, π/3] ≈ [-72°, 60°]"""

        yaw_angle: Tuple[float, float] = (-1.2, 1.2)
        """方位角范围 y（弧度）: [-1.2, 1.2] ≈ [-69°, 69°]"""

    ranges: Ranges = Ranges()
    """球坐标采样范围"""

    @configclass
    class CollisionBox:
        """碰撞检测边界框（世界坐标系）。"""

        x_min: float = -0.8
        x_max: float = 0.1
        y_min: float = -0.2
        y_max: float = 0.2
        z_min: float = -0.7
        z_max: float = -0.05
        """Z轴下限-0.7米表示地面以下限制"""

    collision_box: CollisionBox = CollisionBox()
    """碰撞检测边界框"""

    # 可视化配置
    debug_vis: bool = False
    """是否启用调试可视化"""

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(
        prim_path="/Visuals/Command/goal_position"
    )
    """目标位姿的可视化配置"""
