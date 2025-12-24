# Force Estimator Integration in Custom RSL-RL (Isaac Lab)

This document explains how the custom RSL-RL pipeline was extended to match the
force-estimator (adaptation module) logic from the original FP project, and how
it connects to Isaac Lab's manager-based observations. It is intended for new
contributors who need a full mental model before touching the code.

## Goal and Scope

We add a supervised force-estimator head to an otherwise standard PPO training
loop. The estimator is trained alongside PPO using a second optimizer. The key
design points are:

- History encoder produces a latent from stacked policy observations.
- Actor uses current-frame observations + latent.
- Critic uses privileged observations only (no latent).
- A state estimator (decoder) predicts 12D labels used for supervised loss.
- Isaac Lab provides the label group via an extra observation group (`pred`).

## What Changed vs. Vanilla RSL-RL

1. **Policy network**: an adaptation encoder + state estimator were added.
2. **Policy input**: actor input is `[current_obs | latent]`.
3. **Critic input**: privileged obs only.
4. **Rollout storage**: stores `obs_pred_targets` for supervised learning.
5. **Algorithm**: PPO update + supervised loss (weighted MSE) with a separate
   optimizer and learning rate.

## Data Flow (Per Training Step)

1. Isaac Lab `ObservationManager` builds observation groups:
   - `policy`: stacked history (flattened)
   - `privileged`: critic-only observations
   - `pred`: 12D supervised targets
2. `OnPolicyRunnerAdaptation` calls `alg.act(obs)` and `alg.process_env_step`.
3. `PPOAdaptation` stores `obs_pred_targets` in the rollout buffer.
4. `PPOAdaptation.update()` runs:
   - PPO loss (actor + critic + encoder)
   - Supervised loss (encoder + state_estimator)

## History Stacking Order (Isaac Lab)

Isaac Lab stores history in a circular buffer where **oldest samples come first**
and the **most recent sample is last**. When `flatten_history_dim=True`, the
history is flattened in that order. Therefore, for `policy_history_only=True`,
the **last block is the current frame**, matching the logic in the custom policy.

## Key Files and Responsibilities

- `source/acc/acc/algorithms/rsl_rl/modules.py`
  - `AsymmetricActorCritic`: encoder + state_estimator + actor/critic split.
- `source/acc/acc/algorithms/rsl_rl/algorithm.py`
  - `PPOAdaptation`: PPO loss + supervised loss, separate optimizer.
- `source/acc/acc/algorithms/rsl_rl/storage.py`
  - `AdaptationRolloutStorage`: adds `obs_pred_targets`.
- `source/acc/acc/algorithms/rsl_rl/runner.py`
  - `OnPolicyRunnerAdaptation`: builds policy/algorithm and saves optimizers.
- `source/acc/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
  - Defines `policy`, `privileged`, and `pred` observation groups.
- `source/acc/acc/tasks/manager_based/uniFP_aliengo/mdp/utils.py`
  - `gripper_forces_local`, `base_forces_local` (yaw-only local frame).
- `source/acc/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py`
  - All training hyperparameters for the force estimator.

## Training Configuration (Single Source of Truth)

In `rsl_rl_ppo_cfg.py`:

- **Policy config** (`CustomPpoActorCriticCfg`)
  - `history_num_frames`: 32
  - `obs_curr_dim`: 73
  - `policy_history_only`: True
  - `latent_dim`: 64
  - `prediction_dim`: 12
- **Algorithm config** (`CustomPpoAlgorithmCfg`)
  - `adaptation_coef`: 1.0
  - `adaptation_learning_rate`: 1e-5
  - `obs_pred_weights`: `[0.2, 0.2, 1.0, 1.0]` per 3D block (total 12 dims)
  - `obs_pred_target_obs_group`: `"pred"`

## Label Definition (Pred Group)

The `pred` group provides a 12D label vector:

1. Base linear velocity (3)
2. End-effector goal position in spherical coordinates (3)
3. Gripper external force (yaw-local, 3)
4. Base external force (yaw-local, 3)

These terms are defined in:
`source/acc/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
and computed by functions in:
`source/acc/acc/tasks/manager_based/uniFP_aliengo/mdp/utils.py`.

## Sanity Checks for New Contributors

- `policy` obs size = `history_num_frames * obs_curr_dim` (32 * 73 = 2336).
- `prediction_dim` == `obs_pred_weights` length (12).
- `pred` group exists in `obs_groups` and is provided by the env.
- Actor uses last 73 dims as current frame (history order is oldest -> newest).
- Adaptation optimizer uses `adaptation_learning_rate` (1e-5 by default).
