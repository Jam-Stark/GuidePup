# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""自定义 IK Action: 从 Command 管理器读取目标，而非策略网络动作"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

import omni.log

from isaaclab.assets import Articulation
from isaaclab.controllers import DifferentialIKController
from isaaclab.envs.mdp.actions import ActionTerm
from isaaclab.utils.math import subtract_frame_transforms, matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv
    from .command_based_ik_action_cfg import CommandBasedIKActionCfg


class CommandBasedIKAction(ActionTerm):
    """从 Command 管理器读取目标的 IK 控制动作。
    
    与标准的 DifferentialInverseKinematicsAction 不同：
    - 标准版本: 使用策略网络的动作输出作为 IK 目标
    - 本版本: 从 Command 管理器读取目标，动作维度为 0
    
    这适用于：
    1. 专家数据收集 (目标由命令生成器提供)
    2. 层次化控制 (高层控制器提供目标)
    """

    cfg: CommandBasedIKActionCfg
    """配置"""

    _asset: Articulation
    """机器人资产"""

    _ik_controller: DifferentialIKController
    """差分 IK 控制器"""

    def __init__(self, cfg: CommandBasedIKActionCfg, env: ManagerBasedEnv):
        # 初始化基类
        super().__init__(cfg, env)

        # 解析关节索引
        self._joint_ids, self._joint_names = self._asset.find_joints(self.cfg.joint_names)
        self._num_joints = len(self._joint_ids)

        # 解析 body 索引
        body_ids, body_names = self._asset.find_bodies(self.cfg.body_name)
        if len(body_ids) != 1:
            raise ValueError(
                f"Expected one match for the body name: {self.cfg.body_name}. Found {len(body_ids)}: {body_names}."
            )
        self._body_idx = body_ids[0]
        self._body_name = body_names[0]

        # 处理浮动基座的 Jacobian 索引
        if self._asset.is_fixed_base:
            self._jacobi_body_idx = self._body_idx - 1
            self._jacobi_joint_ids = self._joint_ids
        else:
            self._jacobi_body_idx = self._body_idx
            self._jacobi_joint_ids = [i + 6 for i in self._joint_ids]

        # 日志输出
        omni.log.info(
            f"Resolved joint names for {self.__class__.__name__}: {self._joint_names} [{self._joint_ids}]"
        )
        omni.log.info(
            f"Resolved body name for {self.__class__.__name__}: {self._body_name} [{self._body_idx}]"
        )

        # 优化：如果控制所有关节，则使用 slice
        if self._num_joints == self._asset.num_joints:
            self._joint_ids = slice(None)

        # 创建 IK 控制器
        self._ik_controller = DifferentialIKController(
            cfg=self.cfg.controller, num_envs=self.num_envs, device=self.device
        )

        # 处理 body offset
        if self.cfg.body_offset is not None:
            self._offset_pos = torch.tensor(self.cfg.body_offset.pos, device=self.device).repeat(self.num_envs, 1)
            self._offset_rot = torch.tensor(self.cfg.body_offset.rot, device=self.device).repeat(self.num_envs, 1)
        else:
            self._offset_pos, self._offset_rot = None, None

    """
    Properties
    """

    @property
    def action_dim(self) -> int:
        """动作维度为 0，因为目标来自 Command 管理器"""
        return 0

    @property
    def raw_actions(self) -> torch.Tensor:
        """返回空张量"""
        return torch.zeros(self.num_envs, 0, device=self.device)

    @property
    def processed_actions(self) -> torch.Tensor:
        """返回从 Command 管理器获取的目标"""
        return self._get_command_target()

    """
    Operations
    """

    def process_actions(self, actions: torch.Tensor):
        """处理动作 - 从 Command 管理器获取目标
        
        注意: 
        1. 命令目标在世界坐标系中
        2. 为了数值稳定性，我们将所有计算转换到机器人根坐标系 (Base Frame)
        """
        # 从 Command 管理器获取目标 (世界坐标系)
        target_w = self._get_command_target()
        
        # 获取根部位姿
        root_pos_w = self._asset.data.root_pos_w
        root_quat_w = self._asset.data.root_quat_w
        
        # 转换目标到根坐标系
        if target_w.shape[1] == 3:
            # 只有位置
            target_pos_b, _ = self._transform_to_root_frame(
                target_w, self._asset.data.root_quat_w, root_pos_w, root_quat_w
            )
            target_b = target_pos_b
        else:
            # 位置 + 姿态
            target_pos_b, target_quat_b = self._transform_to_root_frame(
                target_w[:, :3], target_w[:, 3:7], root_pos_w, root_quat_w
            )
            target_b = torch.cat([target_pos_b, target_quat_b], dim=1)
        
        # 获取当前末端执行器位姿 (根坐标系)
        ee_pos_b, ee_quat_b = self._compute_frame_pose()
        
        # 调试输出 (每100步打印一次)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:
            omni.log.info(f"[IK Action] Target pos (base): {target_b[0, :3]}")
            omni.log.info(f"[IK Action] Current EE pos (base): {ee_pos_b[0]}")
            omni.log.info(f"[IK Action] Position error: {torch.norm(target_b[0, :3] - ee_pos_b[0]):.4f}m")
        
        # 设置 IK 控制器命令 (都在根坐标系)
        self._ik_controller.set_command(target_b)

    def apply_actions(self):
        """应用 IK 求解的关节目标"""
        # 获取当前末端执行器位姿 (根坐标系)
        ee_pos_b, ee_quat_b = self._compute_frame_pose()
        joint_pos = self._asset.data.joint_pos[:, self._joint_ids]
        
        # 计算 IK (使用根坐标系 Jacobian)
        if ee_quat_b.norm() != 0:
            jacobian = self._compute_frame_jacobian_in_root_frame()
            joint_pos_des = self._ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        else:
            joint_pos_des = joint_pos.clone()
        
        # 设置关节位置目标
        self._asset.set_joint_position_target(joint_pos_des, self._joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        """重置 - 无需操作"""
        pass

    """
    Helper methods
    """

    def _get_command_target(self) -> torch.Tensor:
        """从 Command 管理器获取 IK 目标
        
        Returns:
            目标张量:
            - command_type="position": (num_envs, 3) 位置
            - command_type="pose": (num_envs, 7) 位置+四元数
        """
        # 从命令管理器获取命令
        command = self._env.command_manager.get_command(self.cfg.command_name)
        
        # 根据 command_type 提取相应部分
        if self.cfg.controller.command_type == "position":
            # 只需要位置 (前3维)
            return command[:, :3]
        elif self.cfg.controller.command_type == "pose":
            # 需要位置+姿态 (7维: pos(3) + quat(4))
            return command[:, :7]
        else:
            raise ValueError(f"Unsupported command_type: {self.cfg.controller.command_type}")

    def _compute_frame_pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        """计算末端执行器在根坐标系下的位姿
        
        Returns:
            (position, quaternion) 元组
        """
        # 获取 body 在世界坐标系下的位姿
        ee_pos_w = self._asset.data.body_pos_w[:, self._body_idx]
        ee_quat_w = self._asset.data.body_quat_w[:, self._body_idx]
        
        # 应用 offset (如果有)
        if self._offset_pos is not None:
            ee_pos_w, ee_quat_w = self._apply_offset(ee_pos_w, ee_quat_w)
        
        # 转换到根坐标系
        root_pos_w = self._asset.data.root_pos_w
        root_quat_w = self._asset.data.root_quat_w
        
        ee_pos_b, ee_quat_b = self._transform_to_root_frame(
            ee_pos_w, ee_quat_w, root_pos_w, root_quat_w
        )
        
        return ee_pos_b, ee_quat_b

    def _compute_frame_pose_world(self) -> tuple[torch.Tensor, torch.Tensor]:
        """计算末端执行器在世界坐标系下的位姿
        
        Returns:
            (position, quaternion) 元组，都在世界坐标系
        """
        # 获取 body 在世界坐标系下的位姿
        ee_pos_w = self._asset.data.body_pos_w[:, self._body_idx]
        ee_quat_w = self._asset.data.body_quat_w[:, self._body_idx]
        
        # 应用 offset (如果有)
        if self._offset_pos is not None:
            ee_pos_w, ee_quat_w = self._apply_offset(ee_pos_w, ee_quat_w)
        
        return ee_pos_w, ee_quat_w

    def _compute_frame_jacobian(self) -> torch.Tensor:
        """计算末端执行器的 Jacobian 矩阵
        
        Returns:
            Jacobian 张量 (num_envs, 6, num_controlled_joints)
        """
        # 获取完整 Jacobian: (num_envs, num_bodies, 6, num_dofs)
        jacobian = self._asset.root_physx_view.get_jacobians()
        
        # 提取末端执行器的 Jacobian: (num_envs, 6, num_dofs)
        ee_jacobian = jacobian[:, self._jacobi_body_idx, :, :]
        
        # 提取受控关节的列: (num_envs, 6, num_controlled_joints)
        ee_jacobian = ee_jacobian[:, :, self._jacobi_joint_ids]
        
        return ee_jacobian

    def _compute_frame_jacobian_in_root_frame(self) -> torch.Tensor:
        """计算末端执行器在根坐标系下的 Jacobian 矩阵
        
        Returns:
            Jacobian 张量 (num_envs, 6, num_controlled_joints)
        """
        # 获取世界坐标系下的 Jacobian
        jacobian_w = self._compute_frame_jacobian()
        
        # 获取根部旋转矩阵
        root_quat_w = self._asset.data.root_quat_w
        from isaaclab.utils.math import matrix_from_quat
        root_rot_mat = matrix_from_quat(root_quat_w)
        # 计算逆旋转矩阵 (转置)
        root_rot_mat_inv = root_rot_mat.transpose(-1, -2)
        
        # 旋转 Jacobian 的列向量 (线速度和角速度部分都需要旋转)
        # jacobian_w: (N, 6, num_dofs)
        # root_rot_mat_inv: (N, 3, 3)
        
        jacobian_b = jacobian_w.clone()
        # 旋转线速度部分 (前3行)
        jacobian_b[:, 0:3, :] = torch.bmm(root_rot_mat_inv, jacobian_w[:, 0:3, :])
        # 旋转角速度部分 (后3行)
        jacobian_b[:, 3:6, :] = torch.bmm(root_rot_mat_inv, jacobian_w[:, 3:6, :])
        
        return jacobian_b

    def _apply_offset(
        self, pos: torch.Tensor, quat: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """应用 body offset"""
        from isaaclab.utils.math import quat_apply, quat_mul
        
        # 旋转 offset 位置
        pos_offset = quat_apply(quat, self._offset_pos)
        pos = pos + pos_offset
        
        # 组合四元数
        quat = quat_mul(quat, self._offset_rot)
        
        return pos, quat

    def _transform_to_root_frame(
        self,
        pos_w: torch.Tensor,
        quat_w: torch.Tensor,
        root_pos_w: torch.Tensor,
        root_quat_w: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """将位姿从世界坐标系转换到根坐标系"""
        from isaaclab.utils.math import subtract_frame_transforms
        
        pos_b, quat_b = subtract_frame_transforms(
            root_pos_w, root_quat_w, pos_w, quat_w
        )
        
        return pos_b, quat_b
