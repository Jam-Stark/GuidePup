# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Virtual impedance command generator for kp/kd/prop_kd."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import CommandTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class VirtualImpedanceCommand(CommandTerm):
    """Command term that outputs per-env virtual impedance parameters.

    Command format: (kp, kd, prop_kd).
    - kp is resampled on reset within configured ranges.
    - kd is derived from kp when prop_kd > 0, otherwise sampled from kd_range.
    - prop_kd is fixed for the full episode.
    """

    cfg: "VirtualImpedanceCommandCfg"

    def __init__(self, cfg: "VirtualImpedanceCommandCfg", env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)

        self._command = torch.zeros((self.num_envs, 3), device=self.device)
        self._command[:, 2] = float(self.cfg.prop_kd)

    def __str__(self) -> str:
        msg = "VirtualImpedanceCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape)}\n"
        msg += f"\tKP range: {self.cfg.kp_range}\n"
        msg += f"\tKD range: {self.cfg.kd_range}\n"
        msg += f"\tProp KD: {self.cfg.prop_kd}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        return

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = self._resolve_env_ids(env_ids)
        if env_ids_tensor.numel() == 0:
            return

        kp_min, kp_max = self.cfg.kp_range
        kps = torch.empty((env_ids_tensor.numel(),), device=self.device).uniform_(float(kp_min), float(kp_max))
        self._command[env_ids_tensor, 0] = kps

        if float(self.cfg.prop_kd) > 0.0:
            kds = kps * float(self.cfg.prop_kd)
        else:
            kd_min, kd_max = self.cfg.kd_range
            kds = torch.empty((env_ids_tensor.numel(),), device=self.device).uniform_(float(kd_min), float(kd_max))
        self._command[env_ids_tensor, 1] = kds
        self._command[env_ids_tensor, 2] = float(self.cfg.prop_kd)

    def _update_command(self):
        return

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.tensor(list(env_ids), device=self.device, dtype=torch.long)
