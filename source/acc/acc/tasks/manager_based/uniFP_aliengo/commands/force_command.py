from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class VariableForceCommand(CommandTerm):
    """Generates a per-env 3D force command with a trapezoidal time profile."""

    cfg: VariableForceCommandCfg

    def __init__(self, cfg: VariableForceCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        self._command = torch.zeros((self.num_envs, 3), device=self.device)

        self._step_dt = float(env.step_dt)
        self._push_interval_range_steps = self._seconds_range_to_steps(cfg.push_interval_range_s, cfg.push_interval_range)
        self._duration_range_steps = self._seconds_range_to_steps(cfg.duration_range_s, cfg.duration_range)
        self._settling_time_steps = self._seconds_to_steps(cfg.settling_time_s, cfg.settling_time_steps)

        self.push_interval = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.push_end_step = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.push_duration_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

        self.selected_envs = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.freed_envs = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)

        self.force_target = torch.zeros((self.num_envs, 3), device=self.device)

        self._sample_new_intervals(torch.arange(self.num_envs, device=self.device))

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self):
        return

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = self._resolve_env_ids(env_ids)
        if env_ids_tensor.numel() == 0:
            return

        self.selected_envs[env_ids_tensor] = False
        self.freed_envs[env_ids_tensor] = False
        self.force_target[env_ids_tensor] = 0.0
        self._command[env_ids_tensor] = 0.0
        self.push_end_step[env_ids_tensor] = 0
        self.push_duration_steps[env_ids_tensor] = 0

        self._sample_new_intervals(env_ids_tensor)

    def _update_command(self):
        current_step = self._env.episode_length_buf

        trigger_mask = (current_step % self.push_interval == 0) & (current_step > 0)
        new_ids = trigger_mask.nonzero(as_tuple=False).squeeze(-1)

        if new_ids.numel() > 0:
            rand_vals = torch.rand((new_ids.numel(),), device=self.device)
            freed_mask = rand_vals < float(self.cfg.prob_freed)

            freed_ids = new_ids[freed_mask]
            active_ids = new_ids[~freed_mask]

            if freed_ids.numel() > 0:
                self.freed_envs[freed_ids] = True
                self.selected_envs[freed_ids] = False
                self.force_target[freed_ids] = 0.0
                self._command[freed_ids] = 0.0

            if active_ids.numel() > 0:
                self.freed_envs[active_ids] = False
                self.selected_envs[active_ids] = True

                f_min, f_max = self.cfg.force_range
                self.force_target[active_ids] = torch.empty((active_ids.numel(), 3), device=self.device).uniform_(
                    float(f_min), float(f_max)
                )

                d_min, d_max = self._duration_range_steps
                durations = torch.randint(int(d_min), int(d_max) + 1, (active_ids.numel(),), device=self.device)

                interval = self.push_interval[active_ids]
                max_dur = torch.clamp((interval - int(self._settling_time_steps)) // 2, min=1)
                durations = torch.minimum(durations, max_dur)
                durations = torch.clamp(durations, min=1)

                self.push_duration_steps[active_ids] = durations
                self.push_end_step[active_ids] = current_step[active_ids] + durations

        active_ids = self.selected_envs.nonzero(as_tuple=False).squeeze(-1)
        if active_ids.numel() > 0:
            curr_t = current_step[active_ids]
            end_t = self.push_end_step[active_ids]
            dur_f = self.push_duration_steps[active_ids].float()
            settling = int(self._settling_time_steps)
            target = self.force_target[active_ids]

            ramp_up_mask = curr_t < end_t
            if ramp_up_mask.any():
                ru_ids = active_ids[ramp_up_mask]
                ru_curr = curr_t[ramp_up_mask]
                ru_end = end_t[ramp_up_mask]
                ru_dur = dur_f[ramp_up_mask]
                start_t = ru_end - ru_dur
                ratio = torch.clamp((ru_curr - start_t) / ru_dur, 0.0, 1.0)
                self._command[ru_ids] = target[ramp_up_mask] * ratio.unsqueeze(-1)

            hold_mask = (curr_t >= end_t) & (curr_t <= end_t + settling)
            if hold_mask.any():
                h_ids = active_ids[hold_mask]
                self._command[h_ids] = self.force_target[h_ids]

            ramp_down_mask = curr_t > end_t + settling
            if ramp_down_mask.any():
                rd_ids = active_ids[ramp_down_mask]
                rd_curr = curr_t[ramp_down_mask]
                rd_end = end_t[ramp_down_mask]
                rd_dur = dur_f[ramp_down_mask]
                start_decay = rd_end + settling
                ratio = torch.clamp((rd_curr - start_decay) / rd_dur, 0.0, 1.0)
                self._command[rd_ids] = target[ramp_down_mask] * (1.0 - ratio.unsqueeze(-1))

            done_mask = curr_t >= (end_t + settling + self.push_duration_steps[active_ids])
            if done_mask.any():
                done_ids = active_ids[done_mask]
                self.selected_envs[done_ids] = False
                self.force_target[done_ids] = 0.0
                self._command[done_ids] = 0.0
                self.push_end_step[done_ids] = 0
                self.push_duration_steps[done_ids] = 0
                self._sample_new_intervals(done_ids)

        if self.freed_envs.any():
            self._command[self.freed_envs] = 0.0

    def _seconds_range_to_steps(self, seconds_range: tuple[float, float] | None, fallback: tuple[int, int]) -> tuple[int, int]:
        if seconds_range is None:
            low, high = fallback
            return (max(int(low), 1), max(int(high), max(int(low), 1)))
        low_s, high_s = seconds_range
        low = max(int(round(float(low_s) / self._step_dt)), 1)
        high = max(int(round(float(high_s) / self._step_dt)), low)
        return (low, high)

    def _seconds_to_steps(self, seconds: float | None, fallback: int) -> int:
        if seconds is None:
            return max(int(fallback), 0)
        return max(int(round(float(seconds) / self._step_dt)), 0)

    def _sample_new_intervals(self, env_ids: torch.Tensor):
        if env_ids.numel() == 0:
            return
        low, high = self._push_interval_range_steps
        self.push_interval[env_ids] = torch.randint(int(low), int(high) + 1, (env_ids.numel(),), device=self.device)

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.tensor(list(env_ids), device=self.device, dtype=torch.long)


@configclass
class VariableForceCommandCfg(CommandTermCfg):
    """Configuration for variable force command generation (trapezoidal profile)."""

    class_type = VariableForceCommand

    # Disable the built-in time_left resampling; this term manages its own schedule.
    resampling_time_range: tuple[float, float] = (9e9, 9e9)

    # Seconds-based parameters (preferred); step-based parameters kept as fallback.
    push_interval_range_s: tuple[float, float] | None = (3.5, 9.0)
    duration_range_s: tuple[float, float] | None = (1.0, 3.0)
    settling_time_s: float | None = 1.0

    push_interval_range: tuple[int, int] = (175, 450)
    duration_range: tuple[int, int] = (50, 150)
    settling_time_steps: int = 50

    force_range: tuple[float, float] = (-50.0, 50.0)
    prob_freed: float = 0.2


