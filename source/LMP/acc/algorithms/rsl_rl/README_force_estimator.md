# Force Estimator Integration in Custom RSL-RL (Isaac Lab)

This document explains how the custom RSL-RL pipeline is wired for the current
`uniFP_aliengo` adaptation setup. The architecture still follows the original
force-estimator pattern, but the active supervised target is now the
base-force-only variant rather than the older 12D label layout.

## Goal and Scope

We add a supervised estimator head to an otherwise standard PPO training loop.
The estimator is trained alongside PPO using a second optimizer. The key design
points are:

- History encoder produces a latent from stacked policy observations.
- Actor uses current-frame observations plus the latent.
- Critic uses privileged observations only and never consumes the latent.
- A state estimator (decoder) predicts task-specific supervised labels.
- For the current `uniFP_aliengo` task, the supervised label is a 6D vector:
  base linear velocity (3) plus base external force in the yaw-local frame (3).

## What Changed vs. Vanilla RSL-RL

1. **Policy network**: an adaptation encoder and state estimator were added.
2. **Policy input**: actor input is `[current_obs | latent]`.
3. **Critic input**: privileged observations only.
4. **Rollout storage**: stores `obs_pred_targets` for supervised learning.
5. **Algorithm**: PPO update plus weighted supervised MSE with a separate
   optimizer and learning rate.

## Current Base-Force-Only Target Layout

Historically, this project used a 12D `pred` target that included end-effector
goal position and gripper force. That is no longer the active configuration for
`uniFP_aliengo`.

The current `pred` group contains only:

1. `base_lin_vel` (3)
2. `base_forces_local` (3)

In the current environment config:

- `pred.ee_goal_pos = None`
- `pred.gripper_forces_local = None`
- `pred.base_forces_local` uses `command_name="base_force_ext"` with
  `use_applied=True`

So the estimator head predicts 6 values total, not 12.

## Data Flow (Per Training Step)

1. Isaac Lab `ObservationManager` builds observation groups:
   - `policy`: stacked history, flattened
   - `privileged`: critic-only observations
   - `pred`: 6D supervised targets for the estimator head
2. `OnPolicyRunnerAdaptation` calls `alg.act(obs)` and
   `alg.process_env_step(...)`.
3. `PPOAdaptation` stores `obs_pred_targets` in the rollout buffer.
4. `PPOAdaptation.update()` runs:
   - PPO loss for actor and critic
   - supervised loss for encoder and state estimator

## History Stacking Order (Isaac Lab)

Isaac Lab stores history in a circular buffer where the oldest samples come
first and the most recent sample is last. When `flatten_history_dim=True`, the
history is flattened in that order. Therefore, for `policy_history_only=True`,
the last block is the current frame, matching the logic in the custom policy.

## Key Files and Responsibilities

- `source/LMP/acc/algorithms/rsl_rl/modules.py`
  - `AsymmetricActorCritic`: encoder plus state estimator plus actor/critic
    split.
  - The module is generic; the actual estimator output size is controlled by
    `prediction_dim` from task config.
- `source/LMP/acc/algorithms/rsl_rl/algorithm.py`
  - `PPOAdaptation`: PPO loss plus supervised loss, with a separate optimizer.
- `source/LMP/acc/algorithms/rsl_rl/storage.py`
  - `AdaptationRolloutStorage`: adds `obs_pred_targets`.
- `source/LMP/acc/algorithms/rsl_rl/runner.py`
  - `OnPolicyRunnerAdaptation`: builds policy and algorithm, wires obs groups,
    and saves optimizer state.
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
  - Defines `policy`, `privileged`, and `pred` observation groups.
  - In the current `pred` group, only `base_lin_vel` and
    `base_forces_local` are active.
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/utils.py`
  - `base_forces_local`: transforms the force vector into the base yaw-only
    local frame.
  - When `use_applied=True`, it reads from `_applied_force_commands` if that
    cache exists, otherwise it falls back to the command manager.
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py`
  - The single source of truth for estimator dimensions and supervised-loss
    weights.

## Training Configuration (Single Source of Truth)

In `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py`:

- **Policy config** (`CustomPpoActorCriticCfg`)
  - `history_num_frames`: 32
  - `obs_curr_dim`: 46
  - `policy_history_only`: `True`
  - `latent_dim`: 64
  - `prediction_dim`: 6
- **Algorithm config** (`CustomPpoAlgorithmCfg`)
  - `adaptation_coef`: 1.0
  - `adaptation_learning_rate`: 1e-5
  - `obs_pred_weights`: `[0.2, 0.2, 0.2, 1.0, 1.0, 1.0]`
  - `obs_pred_target_obs_group`: `"pred"`

Interpretation of `obs_pred_weights`:

- dims `0:3` (`base_lin_vel`) use weight `0.2`
- dims `3:6` (`base_forces_local`) use weight `1.0`

## Label Definition (Pred Group)

The current `pred` group provides a 6D label vector:

1. Base linear velocity (3)
2. Base external force in the base yaw-local frame (3)

The active observation terms are configured as:

- `base_lin_vel`
  - source function: `mdp.base_lin_vel`
  - scale in observation manager: `2.0`
- `base_forces_local`
  - source function: `mdp.base_forces_local`
  - params: `command_name="base_force_ext", use_applied=True`
  - scale in observation manager: `0.01`

The yaw-local frame is built from the robot base yaw only, so the force target
is invariant to roll and pitch. Because `use_applied=True`, the supervised
target tracks the applied base-force command cache when present, not a raw
contact-force sensor reading.

## Notes for Evaluation and Debugging

- Estimator outputs should be decoded as:
  - `pred_est[:, 0:3]`: estimated `base_lin_vel`
  - `pred_est[:, 3:6]`: estimated `base_forces_local`
- Scripts such as `smart_gait.py` currently read only the second block when
  evaluating base-force estimation error.
- If you still see 12D references in older comments or experiments, treat them
  as stale unless the task config explicitly restores `prediction_dim=12` and
  re-enables the missing `pred` terms.

## Sanity Checks for New Contributors

- `policy` single-frame obs size is `46`.
- `policy` stacked obs size is `history_num_frames * obs_curr_dim`
  = `32 * 46 = 1472`.
- `prediction_dim` must match `obs_pred_weights` length, which is `6`.
- `pred` must exist in `obs_groups` and concatenate exactly two active terms.
- Actor uses the last `46` dims of the flattened history as the current frame.
- Adaptation optimizer uses `adaptation_learning_rate` (`1e-5` by default).
