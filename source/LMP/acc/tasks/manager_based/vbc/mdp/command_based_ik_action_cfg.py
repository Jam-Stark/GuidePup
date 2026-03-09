# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""配置: 从 Command 管理器读取目标的 IK Action"""

from __future__ import annotations

from dataclasses import MISSING

from isaaclab.controllers import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions import ActionTermCfg
from isaaclab.utils import configclass

from .command_based_ik_action import CommandBasedIKAction


@configclass
class CommandBasedIKActionCfg(ActionTermCfg):
    """从 Command 管理器读取目标的 IK 控制动作配置
    
    与标准的 DifferentialInverseKinematicsActionCfg 的区别:
    - 动作维度为 0 (不需要策略网络输出)
    - 目标从 Command 管理器的指定命令读取
    - 适用于专家控制、数据收集等场景
    """

    class_type: type = CommandBasedIKAction

    # 资产配置
    asset_name: str = MISSING
    """机器人资产名称"""

    joint_names: list[str] = MISSING
    """控制的关节名称列表"""

    body_name: str = MISSING
    """末端执行器 body 名称"""

    # 命令配置
    command_name: str = MISSING
    """Command 管理器中的命令名称 (如 'target_position_commands')"""

    # IK 控制器配置
    controller: DifferentialIKControllerCfg = MISSING
    """差分 IK 控制器配置"""

    # Body offset (可选)
    @configclass
    class OffsetCfg:
        """末端执行器相对于 parent body 的偏移"""

        pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        """位置偏移"""

        rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        """四元数旋转偏移 (w, x, y, z)"""

    body_offset: OffsetCfg | None = None
    """Body 偏移配置 (如果末端执行器不是实际的刚体)"""
