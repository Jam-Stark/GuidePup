# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import torch.nn as nn
import torch.optim as optim

from rsl_rl.algorithms import PPO

from .storage import AdaptationRolloutStorage

class PPOAdaptation(PPO):
    """PPO with an auxiliary weighted-MSE loss and dual optimizers.

    Optimizer split (per request):
    - Opt1: actor + critic + encoder (+ action std parameter), updated by PPO loss.
    - Opt2: encoder + state_estimator, updated by supervised loss only.
    PPO and adaptation losses are backpropagated separately.
    """

    def __init__(
        self,
        policy,
        *,
        device: str = "cpu",
        adaptation_coef: float = 1.0,
        adaptation_learning_rate: float = 1e-5,
        obs_pred_weights: list[float] | None = None,
        obs_pred_target_key: str = "obs_pred_targets",
        obs_pred_target_obs_group: str | None = None,
        require_obs_pred_targets: bool | None = None,
        **kwargs,
    ):
        super().__init__(policy, device=device, **kwargs)

        self.adaptation_coef = float(adaptation_coef)
        self.adaptation_learning_rate = float(adaptation_learning_rate)
        self.obs_pred_target_key = obs_pred_target_key
        self.obs_pred_target_obs_group = obs_pred_target_obs_group

        self.obs_pred_targets_dim = int(getattr(self.policy, "prediction_dim", 12))
        if require_obs_pred_targets is None:
            require_obs_pred_targets = self.adaptation_coef > 0.0
        self.require_obs_pred_targets = bool(require_obs_pred_targets)

        # Weights for the supervised prediction loss.
        if obs_pred_weights is None:
            self.obs_pred_weights = torch.ones(self.obs_pred_targets_dim, device=self.device)
        else:
            w = torch.as_tensor(obs_pred_weights, dtype=torch.float32, device=self.device).flatten()
            if w.numel() != self.obs_pred_targets_dim:
                raise ValueError(
                    "obs_pred_weights length mismatch: "
                    f"expected {self.obs_pred_targets_dim}, got {w.numel()}."
                )
            self.obs_pred_weights = w

        # Replace the default PPO optimizer with two optimizers.
        # PPO optimizer updates actor + critic + encoder so RL gradients flow into the encoder.
        actor_critic_params = (
            list(self.policy.actor.parameters())
            + list(self.policy.critic.parameters())
            + list(self.policy.encoder.parameters())
        )
        if getattr(self.policy, "noise_std_type", "scalar") == "scalar":
            actor_critic_params.append(self.policy.std)
        elif getattr(self.policy, "noise_std_type", "scalar") == "log":
            actor_critic_params.append(self.policy.log_std)
        else:
            raise ValueError(
                f"Unknown standard deviation type: {getattr(self.policy, 'noise_std_type', None)}. "
                "Should be 'scalar' or 'log'."
            )
        self.optimizer = optim.Adam(actor_critic_params, lr=self.learning_rate)

        # Adaptation optimizer updates encoder + state estimator using only the supervised loss.
        adaptation_params = list(self.policy.encoder.parameters()) + list(self.policy.state_estimator.parameters())
        self.adaptation_optimizer = optim.Adam(adaptation_params, lr=self.adaptation_learning_rate)

        # Use the extended transition so we can store supervised labels.
        self.transition = AdaptationRolloutStorage.Transition()

    def init_storage(self, training_type, num_envs, num_transitions_per_env, obs, actions_shape):
        self.storage = AdaptationRolloutStorage(
            training_type,
            num_envs,
            num_transitions_per_env,
            obs,
            actions_shape,
            obs_pred_targets_dim=self.obs_pred_targets_dim,
            device=self.device,
        )

    def _weighted_mse(self, prediction: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # prediction/target: [batch, dim]
        diff = (prediction - target) * self.obs_pred_weights
        return diff.pow(2).mean()

    def act(self, obs):
        actions = super().act(obs)
        # Optionally record labels from observation dict at the same timestep (pre env.step()).
        if self.obs_pred_target_obs_group is not None and self.obs_pred_target_obs_group in obs:
            self.transition.obs_pred_targets = obs[self.obs_pred_target_obs_group].to(self.device)
        return actions

    def process_env_step(self, obs, rewards, dones, extras):
        # update the normalizers
        self.policy.update_normalization(obs)
        if self.rnd:
            self.rnd.update_normalization(obs)

        # Record the rewards and dones
        self.transition.rewards = rewards.clone()
        self.transition.dones = dones

        # Compute the intrinsic rewards and add to extrinsic rewards
        if self.rnd:
            self.intrinsic_rewards = self.rnd.get_intrinsic_reward(obs)
            self.transition.rewards += self.intrinsic_rewards

        # Bootstrapping on time outs
        if "time_outs" in extras:
            self.transition.rewards += self.gamma * torch.squeeze(
                self.transition.values * extras["time_outs"].unsqueeze(1).to(self.device), 1
            )

        # Record supervision targets for the current transition.
        if self.transition.obs_pred_targets is None:
            targets = None
            if self.obs_pred_target_key and self.obs_pred_target_key in extras:
                targets = extras[self.obs_pred_target_key]
            elif self.obs_pred_target_obs_group is not None and self.obs_pred_target_obs_group in obs:
                targets = obs[self.obs_pred_target_obs_group]

            if targets is None:
                if self.require_obs_pred_targets:
                    raise KeyError(
                        "Missing supervised prediction targets. "
                        f"Expected extras['{self.obs_pred_target_key}'] or obs['{self.obs_pred_target_obs_group}']."
                    )
                targets = torch.zeros(
                    self.transition.observations.batch_size[0],  # type: ignore[union-attr]
                    self.obs_pred_targets_dim,
                    device=self.device,
                )

            self.transition.obs_pred_targets = torch.as_tensor(targets, device=self.device)

        # record the transition
        self.storage.add_transitions(self.transition)  # type: ignore[arg-type]
        self.transition.clear()
        self.policy.reset(dones)

    def update(self):  # noqa: C901
        mean_value_loss = 0.0
        mean_surrogate_loss = 0.0
        mean_entropy = 0.0
        mean_obs_pred_loss = 0.0

        # generator for mini batches
        if self.policy.is_recurrent:
            raise ValueError("PPOAdaptation currently supports only feed-forward policies.")
        generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        for (
            obs_batch,
            actions_batch,
            target_values_batch,
            advantages_batch,
            returns_batch,
            old_actions_log_prob_batch,
            old_mu_batch,
            old_sigma_batch,
            hid_states_batch,
            masks_batch,
            obs_pred_targets_batch,
        ) in generator:
            # original batch size (needed for symmetry augmentation compatibility)
            original_batch_size = obs_batch.batch_size[0]

            # check if we should normalize advantages per mini batch
            if self.normalize_advantage_per_mini_batch:
                with torch.no_grad():
                    advantages_batch = (advantages_batch - advantages_batch.mean()) / (advantages_batch.std() + 1e-8)

            # Recompute actions log prob and entropy for current batch of transitions
            self.policy.act(obs_batch, masks=masks_batch, hidden_states=hid_states_batch[0])
            actions_log_prob_batch = self.policy.get_actions_log_prob(actions_batch)
            value_batch = self.policy.evaluate(obs_batch, masks=masks_batch, hidden_states=hid_states_batch[1])

            mu_batch = self.policy.action_mean[:original_batch_size]
            sigma_batch = self.policy.action_std[:original_batch_size]
            entropy_batch = self.policy.entropy[:original_batch_size]

            # KL schedule (keep in sync with base PPO)
            if self.desired_kl is not None and self.schedule == "adaptive":
                with torch.inference_mode():
                    kl = torch.sum(
                        torch.log(sigma_batch / old_sigma_batch + 1.0e-5)
                        + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch))
                        / (2.0 * torch.square(sigma_batch))
                        - 0.5,
                        axis=-1,
                    )
                    kl_mean = torch.mean(kl)

                    # Reduce the KL divergence across all GPUs
                    if self.is_multi_gpu:
                        torch.distributed.all_reduce(kl_mean, op=torch.distributed.ReduceOp.SUM)
                        kl_mean /= self.gpu_world_size

                    # Update learning rate on main process
                    if self.gpu_global_rank == 0:
                        if kl_mean > self.desired_kl * 2.0:
                            self.learning_rate = max(1e-5, self.learning_rate / 1.5)
                        elif 0.0 < kl_mean < self.desired_kl / 2.0:
                            self.learning_rate = min(1e-2, self.learning_rate * 1.5)

                    # Broadcast updated learning rate
                    if self.is_multi_gpu:
                        lr_tensor = torch.tensor(self.learning_rate, device=self.device)
                        torch.distributed.broadcast(lr_tensor, src=0)
                        self.learning_rate = lr_tensor.item()

                    for param_group in self.optimizer.param_groups:
                        param_group["lr"] = self.learning_rate

            # Surrogate loss
            ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
            surrogate = -torch.squeeze(advantages_batch) * ratio
            surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(
                ratio, 1.0 - self.clip_param, 1.0 + self.clip_param
            )
            surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

            # Value loss
            if self.use_clipped_value_loss:
                value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(
                    -self.clip_param, self.clip_param
                )
                value_losses = (value_batch - returns_batch).pow(2)
                value_losses_clipped = (value_clipped - returns_batch).pow(2)
                value_loss = torch.max(value_losses, value_losses_clipped).mean()
            else:
                value_loss = (returns_batch - value_batch).pow(2).mean()

            loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()

            # PPO update
            self.optimizer.zero_grad()
            loss.backward()

            # Collect gradients from all GPUs
            if self.is_multi_gpu:
                self.reduce_parameters()

            nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
            self.optimizer.step()

            # Clear PPO grads before adaptation step
            self.policy.zero_grad(set_to_none=True)

            # Adaptation update (separate backward)
            if self.adaptation_coef > 0.0:
                self.adaptation_optimizer.zero_grad()
                obs_pred = self.policy.predict(obs_batch[:original_batch_size])
                obs_pred_loss = self._weighted_mse(obs_pred, obs_pred_targets_batch[:original_batch_size])
                (self.adaptation_coef * obs_pred_loss).backward()

                if self.is_multi_gpu:
                    self.reduce_parameters()

                self.adaptation_optimizer.step()
            else:
                obs_pred_loss = torch.tensor(0.0, device=self.device)

            mean_value_loss += float(value_loss.item())
            mean_surrogate_loss += float(surrogate_loss.item())
            mean_entropy += float(entropy_batch.mean().item())
            mean_obs_pred_loss += float(obs_pred_loss.item())

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_entropy /= num_updates
        mean_obs_pred_loss /= num_updates

        self.storage.clear()

        return {
            "value_function": mean_value_loss,
            "surrogate": mean_surrogate_loss,
            "entropy": mean_entropy,
            "obs_pred": mean_obs_pred_loss,
        }
