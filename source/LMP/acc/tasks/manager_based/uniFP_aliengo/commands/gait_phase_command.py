# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gait phase command generator.

This command term maintains an open-loop gait phase variable in [0, 1) and updates it
once per environment step. The phase is reset/frozen to zero for environments that are
marked as standing by the locomotion command term.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import CommandTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .gait_phase_command_cfg import GaitPhaseCommandCfg


class GaitPhaseCommand(CommandTerm):
    """Command term that outputs the scalar gait phase (0..1)."""

    cfg: GaitPhaseCommandCfg

    def __init__(self, cfg: GaitPhaseCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        # Output command: (num_envs, 1) phase in [0, 1).
        self._command = torch.zeros((self.num_envs, 1), device=self.device)

        # Tracks when the phase was last updated to guarantee at-most-once update per env-step.
        self._last_update_step = torch.full((self.num_envs,), -1, device=self.device, dtype=torch.long)

        # Cache dt since _update_command() doesn't receive it directly.
        self._step_dt = float(env.step_dt)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        return

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = self._resolve_env_ids(env_ids)
        if env_ids_tensor.numel() == 0:
            return

        init_phase = float(getattr(self.cfg, "initial_phase", 0.0)) % 1.0
        self._command[env_ids_tensor, 0] = init_phase

        # If locomotion marks some envs as standing, reset their phase to zero immediately.
        locomotion_name = getattr(self.cfg, "locomotion_command_name", None)
        if locomotion_name:
            try:
                locomotion_term = self._env.command_manager.get_term(locomotion_name)
                is_standing = getattr(locomotion_term, "is_standing_env", None)
                if is_standing is not None:
                    standing_ids = env_ids_tensor[is_standing[env_ids_tensor]]
                    if standing_ids.numel() > 0:
                        self._command[standing_ids, 0] = 0.0
            except KeyError:
                pass

        # Prevent the phase from being advanced again within the same env-step as the reset.
        current_step = int(getattr(self._env, "common_step_counter", 0))
        self._last_update_step[env_ids_tensor] = current_step

        self._sync_env_cache()

    def _update_command(self):
        current_step = int(getattr(self._env, "common_step_counter", 0))
        update_mask = self._last_update_step < current_step

        if update_mask.any():
            phase_inc = self._step_dt * float(self.cfg.gait_frequency)
            self._command[update_mask, 0] = torch.remainder(self._command[update_mask, 0] + phase_inc, 1.0)
            self._last_update_step[update_mask] = current_step

        # Standing: freeze/reset phase to zero.
        locomotion_name = getattr(self.cfg, "locomotion_command_name", None)
        if locomotion_name:
            try:
                locomotion_term = self._env.command_manager.get_term(locomotion_name)
                is_standing = getattr(locomotion_term, "is_standing_env", None)
                if is_standing is not None:
                    self._command[is_standing, 0] = 0.0
            except KeyError:
                pass

        self._sync_env_cache()

    def _sync_env_cache(self):
        """Synchronize legacy env buffers (best-effort).

        This keeps older reward/observation terms that read `env.gait_indices` working while
        migrating to the command-term-based gait clock.
        """
        try:
            self._env.gait_indices = self._command[:, 0]
            self._env.gait_last_update_step = self._last_update_step
        except Exception:
            pass

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.tensor(list(env_ids), device=self.device, dtype=torch.long)
