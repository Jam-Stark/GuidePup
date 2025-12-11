# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch
import torch.nn as nn
from rsl_rl.modules import ActorCritic
from rsl_rl.modules.utils import get_activation

class AsymmetricActorCritic(ActorCritic):
    def __init__(self, num_actor_obs, num_critic_obs, num_actions, 
                 actor_hidden_dims=[256, 256, 256], 
                 critic_hidden_dims=[256, 256, 256], 
                 activation='elu', 
                 init_noise_std=1.0, 
                 **kwargs):
        
        self.latent_dim = kwargs.get('latent_dim', 16)
        adaptation_hidden_dims = kwargs.get('adaptation_hidden_dims', [128, 64])
        
        # We assume num_actor_obs includes the history. 
        # If you need to separate proprioception and history, you might need to adjust input dimensions.
        
        super().__init__(num_actor_obs=num_actor_obs,
                         num_critic_obs=num_critic_obs, 
                         num_actions=num_actions,
                         actor_hidden_dims=actor_hidden_dims,
                         critic_hidden_dims=critic_hidden_dims,
                         activation=activation,
                         init_noise_std=init_noise_std,
                         **kwargs)

        activation_fn = get_activation(activation)

        # 1. Adaptation Encoder: History (obs) -> Latent
        encoder_layers = []
        encoder_layers.append(nn.Linear(num_actor_obs, adaptation_hidden_dims[0]))
        encoder_layers.append(activation_fn)
        for l in range(len(adaptation_hidden_dims) - 1):
            encoder_layers.append(nn.Linear(adaptation_hidden_dims[l], adaptation_hidden_dims[l + 1]))
            encoder_layers.append(activation_fn)
        encoder_layers.append(nn.Linear(adaptation_hidden_dims[-1], self.latent_dim))
        self.encoder = nn.Sequential(*encoder_layers)

        # 2. Adaptation Decoder: Latent -> Privileged Info Prediction
        # We use the reverse of adaptation_hidden_dims for decoder
        decoder_layers = []
        decoder_layers.append(nn.Linear(self.latent_dim, adaptation_hidden_dims[-1]))
        decoder_layers.append(activation_fn)
        for l in range(len(adaptation_hidden_dims) - 1, 0, -1):
            decoder_layers.append(nn.Linear(adaptation_hidden_dims[l], adaptation_hidden_dims[l - 1]))
            decoder_layers.append(activation_fn)
        decoder_layers.append(nn.Linear(adaptation_hidden_dims[0], num_critic_obs))
        self.decoder = nn.Sequential(*decoder_layers)

    def forward(self, obs, critic_obs=None):
        # 1. Encode
        latent = self.encoder(obs)
        
        # 2. Decode (for training supervision)
        obs_pred = self.decoder(latent)
        
        # 3. Actor
        # Standard rsl_rl ActorCritic uses self.actor(obs)
        # If you want to inject latent into actor, you would need to modify self.actor 
        # or concatenate latent to obs if self.actor expects it.
        # Here we assume the standard actor takes the raw obs (history).
        # If you want the actor to take (proprio + latent), you need to change the architecture.
        # For this implementation, we stick to the standard flow where actor sees history directly,
        # and the encoder is an auxiliary task to shape the representation.
        actions = self.actor(obs)
        
        # 4. Critic
        value = self.critic(critic_obs) if critic_obs is not None else None
        
        return actions, value, obs_pred, latent
    
    def get_student_latent(self, obs):
        return self.encoder(obs)
    
    def evaluate(self, obs, critic_obs, actions=None):
        # Used in PPO update
        actions_log_prob, value, mu, sigma, entropy = super().evaluate(obs, critic_obs, actions)
        
        latent = self.encoder(obs)
        obs_pred = self.decoder(latent)
        
        return actions_log_prob, value, mu, sigma, entropy, obs_pred, latent
