# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import torch.nn as nn
from rsl_rl.algorithms import PPO

class PPOAdaptation(PPO):
    def __init__(self, actor_critic, device='cpu', adaptation_coef=1.0, **kwargs):
        super().__init__(actor_critic, device=device, **kwargs)
        self.adaptation_coef = adaptation_coef
        
        # Optimizer for adaptation module
        self.adaptation_optimizer = torch.optim.Adam(
            list(self.actor_critic.encoder.parameters()) + 
            list(self.actor_critic.decoder.parameters()),
            lr=self.learning_rate
        )

    def act(self, obs, privileged_obs, obs_pred=None):
        # Override act to handle extra return values if needed, 
        # but standard PPO.act just calls actor_critic.act(obs)
        # We can use the parent method if we don't need special handling during inference/rollout
        # However, the user's runner calls: actions = self.alg.act(obs, privileged_obs, obs_pred)
        # So we should match that signature.
        
        # Note: Standard PPO.act signature is act(self, obs, privileged_obs)
        # We ignore obs_pred here as it's not used for action selection
        return super().act(obs, privileged_obs)

    def update(self):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        mean_adaptation_loss = 0

        generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        
        for obs_batch, critic_obs_batch, actions_batch, target_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch, \
            old_mu_batch, old_sigma_batch, masks_batch, *extra_batch in generator:

            # --- Forward Pass ---
            # Call our custom evaluate
            actions_log_prob_batch, value_batch, mu_batch, sigma_batch, entropy_batch, obs_pred_batch, latent_batch = \
                self.actor_critic.evaluate(obs_batch, critic_obs_batch, actions_batch)

            # --- Adaptation Loss ---
            # Target is critic_obs_batch (privileged info)
            adaptation_loss = nn.functional.mse_loss(obs_pred_batch, critic_obs_batch)
            
            # --- PPO Loss (Standard) ---
            # Adapted from rsl_rl PPO.update
            if self.desired_kl is not None and self.schedule == 'adaptive':
                with torch.inference_mode():
                    kl = torch.sum(torch.log(sigma_batch / old_sigma_batch + 1.e-5) + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch)) / (2.0 * torch.square(sigma_batch)) - 0.5, axis=-1)
                    kl_mean = torch.mean(kl)

                    if kl_mean > self.desired_kl * 2.0:
                        self.learning_rate = max(1e-5, self.learning_rate / 1.5)
                    elif kl_mean < self.desired_kl / 2.0 and kl_mean > 0.0:
                        self.learning_rate = min(1e-2, self.learning_rate * 1.5)

                    for param_group in self.optimizer.param_groups:
                        param_group['lr'] = self.learning_rate
                    for param_group in self.adaptation_optimizer.param_groups:
                        param_group['lr'] = self.learning_rate

            # Surrogate loss
            ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
            surrogate = -torch.squeeze(advantages_batch) * ratio
            surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                            1.0 + self.clip_param)
            surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

            # Value function loss
            if self.use_clipped_value_loss:
                value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(-self.clip_param,
                                                                                                self.clip_param)
                value_losses = (value_batch - returns_batch).pow(2)
                value_losses_clipped = (value_clipped - returns_batch).pow(2)
                value_loss = torch.max(value_losses, value_losses_clipped).mean()
            else:
                value_loss = (returns_batch - value_batch).pow(2).mean()

            loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()
            
            # Add Adaptation Loss
            loss += self.adaptation_coef * adaptation_loss

            # --- Backward ---
            self.optimizer.zero_grad()
            self.adaptation_optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
            self.optimizer.step()
            self.adaptation_optimizer.step()

            mean_value_loss += value_loss.item()
            mean_surrogate_loss += surrogate_loss.item()
            mean_adaptation_loss += adaptation_loss.item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_adaptation_loss /= num_updates
        
        # Return dict compatible with runner logging
        return mean_value_loss, mean_surrogate_loss, mean_adaptation_loss, {"adaptation_loss": mean_adaptation_loss}
