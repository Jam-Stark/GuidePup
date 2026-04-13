# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg

from acc.algorithms.rsl_rl import RslRlPpoActorCriticCfg

@configclass
class CustomPpoActorCriticCfg(RslRlPpoActorCriticCfg):
    class_name: str = "AsymmetricActorCritic"
    latent_dim: int = 64
    prediction_dim: int = 6
    history_num_frames: int = 32
    obs_curr_dim: int = 46
    policy_history_only: bool = True
    history_encoder_hidden_dims: list[int] = [512, 256, 128]
    state_estimator_hidden_dims: list[int] = [128, 64]
    history_obs_normalization: bool = False

@configclass
class CustomPpoAlgorithmCfg(RslRlPpoAlgorithmCfg):
    class_name: str = "PPOAdaptation"
    adaptation_coef: float = 1.0
    adaptation_learning_rate: float = 1e-5
    obs_pred_weights: list[float] = [0.2] * 3 + [1.0] * 3
    obs_pred_target_obs_group: str = "pred"
    obs_pred_target_key: str | None = None

@configclass
class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    class_name = "OnPolicyRunnerAdaptation"
    num_steps_per_env = 24
    max_iterations = 60000
    save_interval = 400
    experiment_name = "uniFP_aliengo_position_force"
    policy = CustomPpoActorCriticCfg(
        init_noise_std=1.0,
        noise_std_type="log",
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    obs_groups = {
        "policy": ["policy"],
        "critic": ["privileged"],
        "pred": ["pred"],
    }
    algorithm = CustomPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1e-3,
        
        # Isaac Lab支持 "adaptive" 或 "fixed"
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        adaptation_coef=1.0,
    )
