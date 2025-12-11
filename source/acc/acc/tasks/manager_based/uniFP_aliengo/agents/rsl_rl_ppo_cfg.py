# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class CustomPpoActorCriticCfg(RslRlPpoActorCriticCfg):
    latent_dim: int = 16
    adaptation_hidden_dims: list[int] = [128, 64]

@configclass
class CustomPpoAlgorithmCfg(RslRlPpoAlgorithmCfg):
    adaptation_coef: float = 1.0

@configclass
class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    class_name = "OnPolicyRunnerAdaptation"
    num_steps_per_env = 24
    max_iterations = 20000
    save_interval = 200
    experiment_name = "uniFP_aliengo_position_force"
    policy = CustomPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
        latent_dim=16,
        adaptation_hidden_dims=[512, 256, 128],
    )
    obs_groups = {
        "policy": ["policy"],
        "critic": ["policy", "privileged"],
    }
    algorithm = CustomPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1e-3,
        
        # 学习率调度
        # Isaac Lab支持 "adaptive" 或 "fixed"
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=None,
        max_grad_norm=1.0,
        adaptation_coef=1.0,
    )