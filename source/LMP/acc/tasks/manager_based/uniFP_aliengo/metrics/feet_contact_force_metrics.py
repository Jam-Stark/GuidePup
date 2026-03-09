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
    from isaaclab.sensors import ContactSensor


class FeetContactForceMetricsCommand(CommandTerm):
    """Tracks per-step feet contact force statistics for logging."""

    cfg: FeetContactForceMetricsCommandCfg

    def __init__(self, cfg: FeetContactForceMetricsCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self._command = torch.zeros((self.num_envs, 1), device=self.device)
        self._sensor: ContactSensor = env.scene[cfg.sensor_name]

        self.metrics["feet_force_mean"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["feet_force_max"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        forces = self._sensor.data.net_forces_w
        force_norms = torch.norm(forces, dim=-1)
        self.metrics["feet_force_mean"] = force_norms.mean(dim=1)
        self.metrics["feet_force_max"] = force_norms.max(dim=1).values

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
class FeetContactForceMetricsCommandCfg(CommandTermCfg):
    class_type = FeetContactForceMetricsCommand

    resampling_time_range = (1.0e9, 1.0e9)
    command_name: str = "feet_contact_force_metrics"
    dim: int = 1

    sensor_name: str = "contact_feet_forces"
