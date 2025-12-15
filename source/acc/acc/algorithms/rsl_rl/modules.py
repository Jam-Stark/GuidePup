# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal
from tensordict import TensorDict

from rsl_rl.networks import EmpiricalNormalization, MLP


class AsymmetricActorCritic(nn.Module):
    """Actor-Critic with an adaptation encoder and a supervised state estimator.

    Design goals (per request):
    - Encoder consumes a flattened history (32 frames) and outputs a latent.
    - Actor consumes current single-frame obs concatenated with latent (so PPO gradients flow to encoder).
    - Critic consumes only privileged observations (stacked frames), and never sees the latent.
    - State estimator (a.k.a decoder) predicts a 12D label from the latent for auxiliary supervised loss.
    """

    is_recurrent = False

    def __init__(
        self,
        obs: TensorDict,
        obs_groups: dict[str, list[str]],
        num_actions: int,
        *,
        # Observation handling
        obs_curr_dim: int | None = None,
        history_num_frames: int = 32,
        history_obs_normalization: bool = False,
        # Networks
        latent_dim: int = 64,
        prediction_dim: int = 12,
        history_encoder_hidden_dims: list[int] = [512, 256, 128],
        state_estimator_hidden_dims: list[int] = [128, 64],
        actor_hidden_dims: list[int] = [256, 256, 256],
        critic_hidden_dims: list[int] = [256, 256, 256],
        activation: str = "elu",
        # Normalization
        actor_obs_normalization: bool = False,
        critic_obs_normalization: bool = False,
        # Action noise
        init_noise_std: float = 1.0,
        noise_std_type: str = "scalar",
        **kwargs,
    ):
        if kwargs:
            print(
                "AsymmetricActorCritic.__init__ got unexpected arguments, which will be ignored: "
                + str([key for key in kwargs.keys()])
            )
        super().__init__()

        self.obs_groups = obs_groups
        self.latent_dim = int(latent_dim)
        self.prediction_dim = int(prediction_dim)
        self.history_num_frames = int(history_num_frames)

        # Resolve critic input dim from configured groups.
        self.critic_groups = obs_groups.get("critic", [])
        if not self.critic_groups:
            raise ValueError("obs_groups must contain a non-empty 'critic' list.")
        critic_in_dim = self._resolve_groups_dim(obs, self.critic_groups)

        # Resolve current obs + history dims.
        # Option A: a dedicated 'history' observation set.
        if "history" in obs_groups:
            self.policy_groups = obs_groups.get("policy", [])
            self.history_groups = obs_groups.get("history", [])
            if not self.policy_groups:
                raise ValueError("obs_groups must contain a non-empty 'policy' list.")
            if not self.history_groups:
                raise ValueError("obs_groups['history'] can not be an empty list if provided.")
            curr_in_dim = self._resolve_groups_dim(obs, self.policy_groups)
            history_in_dim = self._resolve_groups_dim(obs, self.history_groups)
            self._split_policy_obs = None
        else:
            # Option B: policy obs already contains [curr | history] and needs splitting.
            self.policy_groups = obs_groups.get("policy", [])
            if not self.policy_groups:
                raise ValueError("obs_groups must contain a non-empty 'policy' list.")
            policy_total_dim = self._resolve_groups_dim(obs, self.policy_groups)
            resolved_curr_dim = self._resolve_curr_dim_from_policy_total(policy_total_dim, obs_curr_dim)
            curr_in_dim = resolved_curr_dim
            history_in_dim = policy_total_dim - resolved_curr_dim
            if history_in_dim <= 0:
                raise ValueError(
                    "Policy observation does not contain a history tail. "
                    f"Got policy_total_dim={policy_total_dim}, resolved obs_curr_dim={resolved_curr_dim}."
                )
            self.history_groups = []
            self._split_policy_obs = (resolved_curr_dim, history_in_dim)

        self.curr_obs_dim = int(curr_in_dim)
        self.history_obs_dim = int(history_in_dim)

        # Encoder: history -> latent
        self.encoder = MLP(self.history_obs_dim, self.latent_dim, history_encoder_hidden_dims, activation)
        print(f"History encoder MLP: {self.encoder}")

        # State estimator: latent -> 12D labels
        self.state_estimator = MLP(self.latent_dim, self.prediction_dim, state_estimator_hidden_dims, activation)
        print(f"State estimator MLP: {self.state_estimator}")

        # Actor: [curr | latent] -> action mean
        actor_in_dim = self.curr_obs_dim + self.latent_dim
        self.actor = MLP(actor_in_dim, num_actions, actor_hidden_dims, activation)
        print(f"Actor MLP: {self.actor}")

        # Critic: privileged -> value
        self.critic = MLP(critic_in_dim, 1, critic_hidden_dims, activation)
        print(f"Critic MLP: {self.critic}")

        # Normalization modules
        self.actor_obs_normalization = bool(actor_obs_normalization)
        self.history_obs_normalization = bool(history_obs_normalization)
        self.critic_obs_normalization = bool(critic_obs_normalization)

        self.curr_obs_normalizer = (
            EmpiricalNormalization(self.curr_obs_dim) if self.actor_obs_normalization else nn.Identity()
        )
        self.history_obs_normalizer = (
            EmpiricalNormalization(self.history_obs_dim) if self.history_obs_normalization else nn.Identity()
        )
        self.critic_obs_normalizer = (
            EmpiricalNormalization(critic_in_dim) if self.critic_obs_normalization else nn.Identity()
        )

        # Action distribution
        self.noise_std_type = noise_std_type
        if self.noise_std_type == "scalar":
            self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        elif self.noise_std_type == "log":
            self.log_std = nn.Parameter(torch.log(init_noise_std * torch.ones(num_actions)))
        else:
            raise ValueError(f"Unknown standard deviation type: {self.noise_std_type}. Should be 'scalar' or 'log'")
        self.distribution: Normal | None = None
        Normal.set_default_validate_args(False)

    @staticmethod
    def _resolve_groups_dim(obs: TensorDict, groups: list[str]) -> int:
        dim = 0
        for group in groups:
            if group not in obs:
                raise ValueError(f"Observation group '{group}' not found in env observations: {list(obs.keys())}")
            if len(obs[group].shape) != 2:
                raise ValueError(
                    "AsymmetricActorCritic only supports 1D observations (shape [num_envs, dim]). "
                    f"Got obs['{group}'].shape={tuple(obs[group].shape)}"
                )
            dim += int(obs[group].shape[-1])
        return dim

    def _resolve_curr_dim_from_policy_total(self, policy_total_dim: int, obs_curr_dim: int | None) -> int:
        if obs_curr_dim is not None:
            resolved = int(obs_curr_dim)
            if resolved <= 0 or resolved >= policy_total_dim:
                raise ValueError(
                    f"Invalid obs_curr_dim={resolved} for policy_total_dim={policy_total_dim}. "
                    "Expected 0 < obs_curr_dim < policy_total_dim."
                )
            return resolved

        denom = self.history_num_frames + 1
        if denom > 0 and policy_total_dim % denom == 0:
            return policy_total_dim // denom

        raise ValueError(
            "Unable to infer obs_curr_dim from policy observations. "
            f"Got policy_total_dim={policy_total_dim}, history_num_frames={self.history_num_frames}. "
            "Please set obs_curr_dim in the policy cfg, or provide a dedicated 'history' observation set in obs_groups."
        )

    @property
    def action_mean(self) -> torch.Tensor:
        if self.distribution is None:
            raise RuntimeError("Distribution not initialized. Call act() first.")
        return self.distribution.mean

    @property
    def action_std(self) -> torch.Tensor:
        if self.distribution is None:
            raise RuntimeError("Distribution not initialized. Call act() first.")
        return self.distribution.stddev

    @property
    def entropy(self) -> torch.Tensor:
        if self.distribution is None:
            raise RuntimeError("Distribution not initialized. Call act() first.")
        return self.distribution.entropy().sum(dim=-1)

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    def _cat_obs_groups(self, obs: TensorDict, groups: list[str]) -> torch.Tensor:
        return torch.cat([obs[group] for group in groups], dim=-1) if len(groups) > 1 else obs[groups[0]]

    def _get_curr_and_history_obs(self, obs: TensorDict) -> tuple[torch.Tensor, torch.Tensor]:
        if self._split_policy_obs is None:
            curr = self._cat_obs_groups(obs, self.policy_groups)
            history = self._cat_obs_groups(obs, self.history_groups)
            return curr, history

        curr_dim, _history_dim = self._split_policy_obs
        policy_vec = self._cat_obs_groups(obs, self.policy_groups)
        curr = policy_vec[:, :curr_dim]
        history = policy_vec[:, curr_dim:]
        return curr, history

    def _get_critic_obs(self, obs: TensorDict) -> torch.Tensor:
        return self._cat_obs_groups(obs, self.critic_groups)

    def encode(self, obs: TensorDict) -> torch.Tensor:
        _, history = self._get_curr_and_history_obs(obs)
        history = self.history_obs_normalizer(history)
        return self.encoder(history)

    def predict(self, obs: TensorDict) -> torch.Tensor:
        latent = self.encode(obs)
        return self.state_estimator(latent)

    def update_distribution(self, actor_input: torch.Tensor):
        mean = self.actor(actor_input)
        if self.noise_std_type == "scalar":
            std = self.std.expand_as(mean)
        elif self.noise_std_type == "log":
            std = torch.exp(self.log_std).expand_as(mean)
        else:
            raise ValueError(f"Unknown standard deviation type: {self.noise_std_type}. Should be 'scalar' or 'log'")
        self.distribution = Normal(mean, std)

    def act(self, obs: TensorDict, **kwargs) -> torch.Tensor:
        curr, _history = self._get_curr_and_history_obs(obs)
        curr = self.curr_obs_normalizer(curr)
        latent = self.encode(obs)
        actor_input = torch.cat([curr, latent], dim=-1)
        self.update_distribution(actor_input)
        return self.distribution.sample()  # type: ignore[union-attr]

    def act_inference(self, obs: TensorDict) -> torch.Tensor:
        curr, _history = self._get_curr_and_history_obs(obs)
        curr = self.curr_obs_normalizer(curr)
        latent = self.encode(obs)
        actor_input = torch.cat([curr, latent], dim=-1)
        return self.actor(actor_input)

    def evaluate(self, obs: TensorDict, **kwargs) -> torch.Tensor:
        critic_obs = self._get_critic_obs(obs)
        critic_obs = self.critic_obs_normalizer(critic_obs)
        return self.critic(critic_obs)

    def get_actions_log_prob(self, actions: torch.Tensor) -> torch.Tensor:
        if self.distribution is None:
            raise RuntimeError("Distribution not initialized. Call act() before get_actions_log_prob().")
        return self.distribution.log_prob(actions).sum(dim=-1)

    def update_normalization(self, obs: TensorDict):
        if self.actor_obs_normalization or self.history_obs_normalization:
            curr, history = self._get_curr_and_history_obs(obs)
            if self.actor_obs_normalization and isinstance(self.curr_obs_normalizer, EmpiricalNormalization):
                self.curr_obs_normalizer.update(curr)
            if self.history_obs_normalization and isinstance(self.history_obs_normalizer, EmpiricalNormalization):
                self.history_obs_normalizer.update(history)
        if self.critic_obs_normalization and isinstance(self.critic_obs_normalizer, EmpiricalNormalization):
            critic_obs = self._get_critic_obs(obs)
            self.critic_obs_normalizer.update(critic_obs)

    def load_state_dict(self, state_dict, strict: bool = True) -> bool:  # type: ignore[override]
        super().load_state_dict(state_dict, strict=strict)
        return True  # training resumes
