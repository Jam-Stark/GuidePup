# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch

from rsl_rl.storage import RolloutStorage


class AdaptationRolloutStorage(RolloutStorage):
    """Rollout storage extended with supervised prediction targets."""

    class Transition(RolloutStorage.Transition):
        def __init__(self):
            super().__init__()
            self.obs_pred_targets: torch.Tensor | None = None

        def clear(self):
            super().clear()
            self.obs_pred_targets = None

    def __init__(
        self,
        training_type: str,
        num_envs: int,
        num_transitions_per_env: int,
        obs,
        actions_shape,
        *,
        obs_pred_targets_dim: int,
        device: str = "cpu",
    ):
        super().__init__(training_type, num_envs, num_transitions_per_env, obs, actions_shape, device=device)

        if training_type != "rl":
            raise ValueError("AdaptationRolloutStorage is only supported for reinforcement learning training.")
        if obs_pred_targets_dim <= 0:
            raise ValueError(f"obs_pred_targets_dim must be > 0, got {obs_pred_targets_dim}.")

        self.obs_pred_targets = torch.zeros(
            num_transitions_per_env, num_envs, obs_pred_targets_dim, device=self.device
        )

    def add_transitions(self, transition: Transition):  # type: ignore[override]
        if transition.obs_pred_targets is None:
            raise ValueError(
                "Missing obs_pred_targets on transition. "
                "Set PPOAdaptation.transition.obs_pred_targets before calling add_transitions()."
            )
        if self.step >= self.num_transitions_per_env:
            raise OverflowError("Rollout buffer overflow! You should call clear() before adding new transitions.")

        self.obs_pred_targets[self.step].copy_(transition.obs_pred_targets)
        super().add_transitions(transition)

    def mini_batch_generator(self, num_mini_batches: int, num_epochs: int = 8):  # type: ignore[override]
        if self.training_type != "rl":
            raise ValueError("This function is only available for reinforcement learning training.")

        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        indices = torch.randperm(num_mini_batches * mini_batch_size, requires_grad=False, device=self.device)

        # Core
        observations = self.observations.flatten(0, 1)
        actions = self.actions.flatten(0, 1)
        values = self.values.flatten(0, 1)
        returns = self.returns.flatten(0, 1)

        # For PPO
        old_actions_log_prob = self.actions_log_prob.flatten(0, 1)
        advantages = self.advantages.flatten(0, 1)
        old_mu = self.mu.flatten(0, 1)
        old_sigma = self.sigma.flatten(0, 1)

        # Supervision targets
        obs_pred_targets = self.obs_pred_targets.flatten(0, 1)

        for _epoch in range(num_epochs):
            for i in range(num_mini_batches):
                start = i * mini_batch_size
                end = (i + 1) * mini_batch_size
                batch_idx = indices[start:end]

                # -- Core
                obs_batch = observations[batch_idx]
                actions_batch = actions[batch_idx]

                # -- PPO
                target_values_batch = values[batch_idx]
                returns_batch = returns[batch_idx]
                old_actions_log_prob_batch = old_actions_log_prob[batch_idx]
                advantages_batch = advantages[batch_idx]
                old_mu_batch = old_mu[batch_idx]
                old_sigma_batch = old_sigma[batch_idx]

                # -- Supervision
                obs_pred_targets_batch = obs_pred_targets[batch_idx]

                yield (
                    obs_batch,
                    actions_batch,
                    target_values_batch,
                    advantages_batch,
                    returns_batch,
                    old_actions_log_prob_batch,
                    old_mu_batch,
                    old_sigma_batch,
                    (None, None),
                    None,
                    obs_pred_targets_batch,
                )

