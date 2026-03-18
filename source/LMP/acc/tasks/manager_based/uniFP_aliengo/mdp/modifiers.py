from __future__ import annotations

from collections.abc import Sequence

import torch
from isaaclab.utils.buffers import DelayBuffer
from isaaclab.utils.modifiers import ModifierBase, ModifierCfg


class RandomizedDelayModifier(ModifierBase):
    """Episode-fixed observation delay backed by IsaacLab's DelayBuffer."""

    def __init__(self, cfg: ModifierCfg, data_dim: tuple[int, ...], device: str) -> None:
        super().__init__(cfg, data_dim, device)

        self._min_delay_steps = int(cfg.params.get("min_delay_steps", 0))
        self._max_delay_steps = int(cfg.params.get("max_delay_steps", 0))
        if self._min_delay_steps < 0:
            raise ValueError(f"min_delay_steps must be non-negative, received: {self._min_delay_steps}")
        if self._max_delay_steps < self._min_delay_steps:
            raise ValueError(
                "max_delay_steps must be >= min_delay_steps, "
                f"received: {(self._min_delay_steps, self._max_delay_steps)}"
            )

        self._delay_buffer = DelayBuffer(self._max_delay_steps, batch_size=data_dim[0], device=device)
        self.reset()

    @property
    def time_lags(self) -> torch.Tensor:
        """Return the currently configured per-environment time lags."""
        return self._delay_buffer.time_lags

    def _resolve_batch_ids(self, env_ids: Sequence[int] | None) -> tuple[Sequence[int] | slice, int]:
        if env_ids is None or env_ids == slice(None):
            return slice(None), self._data_dim[0]
        if isinstance(env_ids, slice):
            start, stop, step = env_ids.indices(self._data_dim[0])
            return env_ids, len(range(start, stop, step))
        if isinstance(env_ids, torch.Tensor):
            env_ids = env_ids.to(device=self._device, dtype=torch.long)
            return env_ids, int(env_ids.numel())
        env_ids = torch.tensor(list(env_ids), device=self._device, dtype=torch.long)
        return env_ids, int(env_ids.numel())

    def _sample_time_lags(self, num_envs: int) -> int | torch.Tensor:
        if self._min_delay_steps == self._max_delay_steps:
            return self._min_delay_steps
        return torch.randint(
            low=self._min_delay_steps,
            high=self._max_delay_steps + 1,
            size=(num_envs,),
            dtype=torch.int,
            device=self._device,
        )

    def reset(self, env_ids: Sequence[int] | None = None):
        batch_ids, num_envs = self._resolve_batch_ids(env_ids)
        time_lags = self._sample_time_lags(num_envs)
        self._delay_buffer.set_time_lag(time_lag=time_lags, batch_ids=batch_ids)
        self._delay_buffer.reset(batch_ids)

    def __call__(
        self,
        data: torch.Tensor,
        min_delay_steps: int | None = None,
        max_delay_steps: int | None = None,
    ) -> torch.Tensor:
        del min_delay_steps, max_delay_steps
        return self._delay_buffer.compute(data)
