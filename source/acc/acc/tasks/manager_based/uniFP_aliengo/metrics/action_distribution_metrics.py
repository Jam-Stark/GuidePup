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


class ActionDistributionMetricsCommand(CommandTerm):
    """Tracks action distribution statistics for logging."""

    cfg: ActionDistributionMetricsCommandCfg

    def __init__(self, cfg: ActionDistributionMetricsCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self._command = torch.zeros((self.num_envs, 1), device=self.device)
        self._action_term_names = tuple(cfg.action_term_names)
        self._tracked_terms: dict[str, object] = {}
        self._terms_resolved = False

        self.metrics["all_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["all_max_abs"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["all_near_one_frac"] = torch.zeros(self.num_envs, device=self.device)
        for term_name in self._action_term_names:
            self.metrics[f"{term_name}_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"{term_name}_max_abs"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics[f"{term_name}_near_one_frac"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _resolve_action_terms(self) -> bool:
        if self._terms_resolved:
            return True
        if not hasattr(self._env, "action_manager"):
            return False
        for term_name in self._action_term_names:
            if term_name not in self._env.action_manager.active_terms:
                raise ValueError(
                    f"Action term '{term_name}' not found. Available terms: {self._env.action_manager.active_terms}."
                )
            term = self._env.action_manager.get_term(term_name)
            if getattr(term, "action_dim", 0) <= 0:
                continue
            self._tracked_terms[term_name] = term
        self._terms_resolved = True
        return True

    def _update_metrics(self):
        if not self._resolve_action_terms():
            return
        threshold = float(self.cfg.near_one_threshold)

        all_actions = self._env.action_manager.action
        if all_actions.numel() == 0:
            self.metrics["all_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics["all_max_abs"] = torch.zeros(self.num_envs, device=self.device)
            self.metrics["all_near_one_frac"] = torch.zeros(self.num_envs, device=self.device)
        else:
            all_abs = torch.abs(all_actions)
            self.metrics["all_mean_abs"] = all_abs.mean(dim=1)
            self.metrics["all_max_abs"] = all_abs.max(dim=1).values
            self.metrics["all_near_one_frac"] = (all_abs >= threshold).float().mean(dim=1)

        for term_name, term in self._tracked_terms.items():
            term_actions = term.raw_actions
            if term_actions.numel() == 0:
                self.metrics[f"{term_name}_mean_abs"] = torch.zeros(self.num_envs, device=self.device)
                self.metrics[f"{term_name}_max_abs"] = torch.zeros(self.num_envs, device=self.device)
                self.metrics[f"{term_name}_near_one_frac"] = torch.zeros(self.num_envs, device=self.device)
                continue
            term_abs = torch.abs(term_actions)
            self.metrics[f"{term_name}_mean_abs"] = term_abs.mean(dim=1)
            self.metrics[f"{term_name}_max_abs"] = term_abs.max(dim=1).values
            self.metrics[f"{term_name}_near_one_frac"] = (term_abs >= threshold).float().mean(dim=1)

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
class ActionDistributionMetricsCommandCfg(CommandTermCfg):
    class_type = ActionDistributionMetricsCommand

    resampling_time_range = (1.0e9, 1.0e9)
    command_name: str = "action_distribution_metrics"
    dim: int = 1

    action_term_names: tuple[str, ...] = ("leg_actions", "arm_actions")
    near_one_threshold: float = 0.95
