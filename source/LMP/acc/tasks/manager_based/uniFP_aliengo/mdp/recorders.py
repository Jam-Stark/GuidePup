# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import RecorderTermCfg
from isaaclab.managers.recorder_manager import RecorderTerm
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class FrontBackLegTorquesRecorder(RecorderTerm):
    """Record applied joint torques for front and rear legs."""

    def __init__(self, cfg: FrontBackLegTorquesRecorderCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._asset = None
        self._front_leg_joint_ids = None
        self._rear_leg_joint_ids = None

    def _resolve_joint_ids(self) -> None:
        if self._asset is None:
            self._asset = self._env.scene[self.cfg.asset_name]
        if self._front_leg_joint_ids is None:
            self._front_leg_joint_ids = self._asset.find_joints(
                list(self.cfg.front_leg_joint_names), preserve_order=True
            )[0]
        if self._rear_leg_joint_ids is None:
            self._rear_leg_joint_ids = self._asset.find_joints(
                list(self.cfg.rear_leg_joint_names), preserve_order=True
            )[0]

    def record_post_step(self) -> tuple[str | None, torch.Tensor | dict | None]:
        self._resolve_joint_ids()
        applied_torque = self._asset.data.applied_torque
        return (
            "torques/legs",
            {
                "front": applied_torque[:, self._front_leg_joint_ids],
                "rear": applied_torque[:, self._rear_leg_joint_ids],
            },
        )


@configclass
class FrontBackLegTorquesRecorderCfg(RecorderTermCfg):
    class_type = FrontBackLegTorquesRecorder

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
