# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class LegTorqueMetricsCommand(CommandTerm):
    """Tracks joint torque utilization (|tau|/limit) statistics for logging."""

    cfg: LegTorqueMetricsCommandCfg

    def __init__(self, cfg: LegTorqueMetricsCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self._command = torch.zeros((self.num_envs, 1), device=self.device)

        self._asset = env.scene[cfg.asset_name]
        self._front_leg_joint_ids = self._asset.find_joints(list(cfg.front_leg_joint_names))[0]
        self._rear_leg_joint_ids = self._asset.find_joints(list(cfg.rear_leg_joint_names))[0]
        self._front_hip_joint_ids = self._asset.find_joints(list(cfg.front_hip_joint_names))[0]
        self._front_thigh_joint_ids = self._asset.find_joints(list(cfg.front_thigh_joint_names))[0]
        self._front_calf_joint_ids = self._asset.find_joints(list(cfg.front_calf_joint_names))[0]
        self._rear_hip_joint_ids = self._asset.find_joints(list(cfg.rear_hip_joint_names))[0]
        self._rear_thigh_joint_ids = self._asset.find_joints(list(cfg.rear_thigh_joint_names))[0]
        self._rear_calf_joint_ids = self._asset.find_joints(list(cfg.rear_calf_joint_names))[0]
        self._arm_joint_ids = self._asset.find_joints(list(cfg.arm_joint_names))[0]
        self._arm_shoulder_joint_ids = self._asset.find_joints(list(cfg.arm_shoulder_joint_names))[0]
        self._arm_other_joint_ids = self._asset.find_joints(list(cfg.arm_other_joint_names))[0]

        self.metrics["front_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_hip_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_hip_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_thigh_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_thigh_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_calf_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["front_calf_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_hip_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_hip_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_thigh_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_thigh_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_calf_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["rear_calf_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_shoulder_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_shoulder_max_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_other_mean_util"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["arm_other_max_util"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        torques = torch.abs(self._asset.data.applied_torque)
        limits = self._asset.data.joint_effort_limits
        limits = torch.where(limits > 0, limits, torch.ones_like(limits))
        utilization = torques / limits

        front = utilization[:, self._front_leg_joint_ids]
        rear = utilization[:, self._rear_leg_joint_ids]
        def _util_stats(joint_ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
            values = utilization[:, joint_ids]
            return values.mean(dim=1), values.max(dim=1).values

        self.metrics["front_mean_util"] = front.mean(dim=1)
        self.metrics["rear_mean_util"] = rear.mean(dim=1)
        self.metrics["front_max_util"] = front.max(dim=1).values
        self.metrics["rear_max_util"] = rear.max(dim=1).values
        front_hip_mean, front_hip_max = _util_stats(self._front_hip_joint_ids)
        front_thigh_mean, front_thigh_max = _util_stats(self._front_thigh_joint_ids)
        front_calf_mean, front_calf_max = _util_stats(self._front_calf_joint_ids)
        rear_hip_mean, rear_hip_max = _util_stats(self._rear_hip_joint_ids)
        rear_thigh_mean, rear_thigh_max = _util_stats(self._rear_thigh_joint_ids)
        rear_calf_mean, rear_calf_max = _util_stats(self._rear_calf_joint_ids)
        arm_mean, arm_max = _util_stats(self._arm_joint_ids)
        arm_shoulder_mean, arm_shoulder_max = _util_stats(self._arm_shoulder_joint_ids)
        arm_other_mean, arm_other_max = _util_stats(self._arm_other_joint_ids)

        self.metrics["front_hip_mean_util"] = front_hip_mean
        self.metrics["front_hip_max_util"] = front_hip_max
        self.metrics["front_thigh_mean_util"] = front_thigh_mean
        self.metrics["front_thigh_max_util"] = front_thigh_max
        self.metrics["front_calf_mean_util"] = front_calf_mean
        self.metrics["front_calf_max_util"] = front_calf_max
        self.metrics["rear_hip_mean_util"] = rear_hip_mean
        self.metrics["rear_hip_max_util"] = rear_hip_max
        self.metrics["rear_thigh_mean_util"] = rear_thigh_mean
        self.metrics["rear_thigh_max_util"] = rear_thigh_max
        self.metrics["rear_calf_mean_util"] = rear_calf_mean
        self.metrics["rear_calf_max_util"] = rear_calf_max
        self.metrics["arm_mean_util"] = arm_mean
        self.metrics["arm_max_util"] = arm_max
        self.metrics["arm_shoulder_mean_util"] = arm_shoulder_mean
        self.metrics["arm_shoulder_max_util"] = arm_shoulder_max
        self.metrics["arm_other_mean_util"] = arm_other_mean
        self.metrics["arm_other_max_util"] = arm_other_max

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = self._resolve_env_ids(env_ids)
        if env_ids_tensor.numel() == 0:
            return
        self._command[env_ids_tensor] = 0.0

    def _update_command(self):
        return

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.tensor(list(env_ids), device=self.device, dtype=torch.long)


@configclass
class LegTorqueMetricsCommandCfg(CommandTermCfg):
    class_type = LegTorqueMetricsCommand

    resampling_time_range = (1.0e9, 1.0e9)
    command_name: str = "leg_torque_metrics"
    dim: int = 1

    asset_name: str = "robot"
    front_leg_joint_names: tuple[str, ...] = (
        "FL_hip_joint",
        "FR_hip_joint",
        "FL_thigh_joint",
        "FR_thigh_joint",
        "FL_calf_joint",
        "FR_calf_joint",
    )
    rear_leg_joint_names: tuple[str, ...] = (
        "RL_hip_joint",
        "RR_hip_joint",
        "RL_thigh_joint",
        "RR_thigh_joint",
        "RL_calf_joint",
        "RR_calf_joint",
    )
    front_hip_joint_names: tuple[str, ...] = ("FL_hip_joint", "FR_hip_joint")
    front_thigh_joint_names: tuple[str, ...] = ("FL_thigh_joint", "FR_thigh_joint")
    front_calf_joint_names: tuple[str, ...] = ("FL_calf_joint", "FR_calf_joint")
    rear_hip_joint_names: tuple[str, ...] = ("RL_hip_joint", "RR_hip_joint")
    rear_thigh_joint_names: tuple[str, ...] = ("RL_thigh_joint", "RR_thigh_joint")
    rear_calf_joint_names: tuple[str, ...] = ("RL_calf_joint", "RR_calf_joint")
    arm_joint_names: tuple[str, ...] = (
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
    )
    arm_shoulder_joint_names: tuple[str, ...] = ("arm_joint2",)
    arm_other_joint_names: tuple[str, ...] = ("arm_joint1", "arm_joint3", "arm_joint4", "arm_joint5")
