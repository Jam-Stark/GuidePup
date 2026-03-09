# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import os
import warnings
import torch

from rsl_rl.runners import OnPolicyRunner
from rsl_rl.env import VecEnv
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, resolve_rnd_config, resolve_symmetry_config

from .modules import AsymmetricActorCritic
from .algorithm import PPOAdaptation


class OnPolicyRunnerAdaptation(OnPolicyRunner):
    def _construct_algorithm(self, obs):
        """Construct PPOAdaptation + AsymmetricActorCritic for Isaac Lab (TensorDict obs)."""

        # resolve RND config
        self.alg_cfg = resolve_rnd_config(self.alg_cfg, obs, self.cfg["obs_groups"], self.env)
        if self.alg_cfg.get("rnd_cfg") is not None:
            raise NotImplementedError("PPOAdaptation.update currently doesn't train RND. Disable rnd_cfg for now.")

        # resolve symmetry config
        self.alg_cfg = resolve_symmetry_config(self.alg_cfg, self.env)
        if self.alg_cfg.get("symmetry_cfg") is not None:
            warnings.warn("PPOAdaptation ignores symmetry loss/augmentation in update().", stacklevel=2)

        # resolve deprecated normalization config
        if self.cfg.get("empirical_normalization") is not None:
            warnings.warn(
                "The `empirical_normalization` parameter is deprecated. Please set `actor_obs_normalization` and "
                "`critic_obs_normalization` as part of the `policy` configuration instead.",
                DeprecationWarning,
            )
            if self.policy_cfg.get("actor_obs_normalization") is None:
                self.policy_cfg["actor_obs_normalization"] = self.cfg["empirical_normalization"]
            if self.policy_cfg.get("critic_obs_normalization") is None:
                self.policy_cfg["critic_obs_normalization"] = self.cfg["empirical_normalization"]

        # Choose policy implementation.
        policy_class_name = self.policy_cfg.pop("class_name", "ActorCritic")
        want_adaptation_policy = policy_class_name == "AsymmetricActorCritic" or any(
            key in self.policy_cfg
            for key in (
                "latent_dim",
                "history_encoder_hidden_dims",
                "state_estimator_hidden_dims",
                "history_num_frames",
                "obs_curr_dim",
                "obs_pred_target_obs_group",
            )
        )
        if want_adaptation_policy:
            policy_cls = AsymmetricActorCritic
        elif policy_class_name == "ActorCritic":
            policy_cls = ActorCritic
        elif policy_class_name == "ActorCriticRecurrent":
            policy_cls = ActorCriticRecurrent
        else:
            raise ValueError(f"Unsupported policy.class_name='{policy_class_name}'.")

        policy = policy_cls(obs, self.cfg["obs_groups"], self.env.num_actions, **self.policy_cfg).to(self.device)

        # Choose algorithm implementation.
        alg_class_name = self.alg_cfg.pop("class_name", "PPO")
        want_adaptation_alg = alg_class_name == "PPOAdaptation" or (
            alg_class_name == "PPO"
            and any(key in self.alg_cfg for key in ("adaptation_coef", "obs_pred_weights", "obs_pred_target_key"))
        )
        if want_adaptation_alg:
            alg_cls = PPOAdaptation
        else:
            raise ValueError(
                f"Unsupported algorithm.class_name='{alg_class_name}'. Use 'PPOAdaptation' for adaptation training."
            )

        alg = alg_cls(policy, device=self.device, **self.alg_cfg, multi_gpu_cfg=self.multi_gpu_cfg)

        alg.init_storage(
            "rl",
            self.env.num_envs,
            self.num_steps_per_env,
            obs,
            [self.env.num_actions],
        )

        return alg

    def save(self, path: str, infos=None):
        saved_dict = {
            "model_state_dict": self.alg.policy.state_dict(),
            "optimizer_state_dict": self.alg.optimizer.state_dict(),
            "adaptation_optimizer_state_dict": self.alg.adaptation_optimizer.state_dict(),
            "iter": self.current_learning_iteration,
            "infos": infos,
        }
        torch.save(saved_dict, path)

        # upload model to external logging service
        if self.logger_type in ["neptune", "wandb"] and not self.disable_logs:
            self.writer.save_model(path, self.current_learning_iteration)

    def load(self, path: str, load_optimizer: bool = True, map_location: str | None = None):
        loaded_dict = torch.load(path, weights_only=False, map_location=map_location)
        resumed_training = self.alg.policy.load_state_dict(loaded_dict["model_state_dict"])
        if load_optimizer and resumed_training:
            self.alg.optimizer.load_state_dict(loaded_dict["optimizer_state_dict"])
            if "adaptation_optimizer_state_dict" in loaded_dict:
                self.alg.adaptation_optimizer.load_state_dict(loaded_dict["adaptation_optimizer_state_dict"])
        if resumed_training:
            self.current_learning_iteration = loaded_dict["iter"]
        return loaded_dict.get("infos")
