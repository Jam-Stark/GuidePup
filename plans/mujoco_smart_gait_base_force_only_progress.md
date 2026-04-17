# MuJoCo Smart Gait Base-Force-Only Progress

## Context

- Target experiment: smart gait sim2sim to MuJoCo for the `base-force-only` policy line.
- Reference Isaac experiment entry: [`scripts/experiments/uniFP_aliengo/smart_gait.py`](../scripts/experiments/uniFP_aliengo/smart_gait.py).
- MuJoCo entry added for this line: [`scripts/mujoco/smart_gait.py`](../scripts/mujoco/smart_gait.py).
- Current robot resources remain under [`resources/aliengoZ1/mujoco`](../resources/aliengoZ1/mujoco).

## Goal

- Re-export an IO description that matches the new smart gait policy contract.
- Re-export a MuJoCo-ready policy wrapper that matches the same history length and observation layout.
- Run the smart gait `base-force-only` variant in MuJoCo without changing robot resources.

## Current Status

- `base-force-only` MuJoCo smart gait runner has been added in [`scripts/mujoco/smart_gait.py`](../scripts/mujoco/smart_gait.py).
- The shared MuJoCo runtime contract has been updated in [`scripts/mujoco/runtime.py`](../scripts/mujoco/runtime.py) to match the current locomotion-only policy interface.
- IO descriptor export has been updated in [`scripts/export_io_descriptors.py`](../scripts/export_io_descriptors.py) so smart gait can export from `reverse_push_eval` and keep `--policy-history-length` configurable.
- MuJoCo policy export has been updated in [`scripts/mujoco/export_policy_wrapper.py`](../scripts/mujoco/export_policy_wrapper.py) so `--policy-history-length` remains configurable and the exported metadata records gait frequency.
- The current MuJoCo model resources are considered reusable because the experiment changed policy IO, not robot geometry or kinematics.

## Policy Contract Assumed by the New MuJoCo Path

- Action space: `12` leg actions.
- Per-frame policy observation: `46` dims.
- Command observation: locomotion-only `3` dims after scaling.
- Arm actions: frozen in MuJoCo runtime for this policy line.
- Estimator output: primary target is the new `6`-dim output, with fallback compatibility kept for legacy `12`-dim outputs.

## Completed Code Work

- [`scripts/mujoco/runtime.py`](../scripts/mujoco/runtime.py)
  - Default metadata switched to the current locomotion-only smart gait contract.
  - Command-vector assembly now supports both new short command metadata and older long command metadata.
  - Runtime state update supports `gait_override` so standing mode can freeze/reset gait phase.
- [`scripts/mujoco/export_policy_wrapper.py`](../scripts/mujoco/export_policy_wrapper.py)
  - Added `--policy-history-length`.
  - Reconstructs env config and agent config with the requested history length before exporting.
  - Writes matching `policy.json` metadata next to the exported TorchScript policy.
- [`scripts/export_io_descriptors.py`](../scripts/export_io_descriptors.py)
  - Added `--env-cfg reverse_push_eval`.
  - Added `--policy-history-length`.
  - Added `locomotion_commands_with_scales` handling so the descriptor reflects the current smart gait command layout.
  - `reverse_push_eval` export path no longer depends on `--task`.
- [`scripts/mujoco/smart_gait.py`](../scripts/mujoco/smart_gait.py)
  - Added MuJoCo rollout path for the smart gait experiment.
  - Supports `demo`, `stop_go`, and `human_guide` force schedules.
  - Supports configurable `--policy-history-length`.
  - Implements standing/walking gait phase handling through a local phase clock.
  - Uses only the base-force estimator path for switching and optional force-guided turning.
- [`scripts/mujoco/staged_direction_force_eval.py`](../scripts/mujoco/staged_direction_force_eval.py)
  - Updated estimator parsing for compatibility with both `6`-dim and `12`-dim outputs.
- [`experiments/SMART_GAIT.md`](../experiments/SMART_GAIT.md)
  - Added MuJoCo export and rollout commands for the smart gait line.

## Export Progress

- Smart gait IO descriptor has been exported.
- Matching smart gait MuJoCo policy has also been exported.
- The critical invariant is now:
  - IO descriptor history length
  - exported `policy.json` history length
  - MuJoCo runtime `--policy-history-length`
  must all match.

## Validation Completed

- Python syntax check passed for the updated MuJoCo/runtime/export scripts via `python -m py_compile`.

## Validation Still Pending

- End-to-end headless MuJoCo smoke test with the newly exported smart gait policy.
- Viewer-based qualitative behavior check in MuJoCo.
- Isaac vs MuJoCo observation parity check if rollout behavior is suspicious.
- CSV-level review of mode switching behavior under `human_guide`.

## Known Assumptions

- [`resources/aliengoZ1/mujoco/aliengoZ1.xml`](../resources/aliengoZ1/mujoco/aliengoZ1.xml) remains valid for this experiment line.
- `base_force_only` logic only depends on the base-force estimator output and ignores gripper-force estimation in the new smart gait path.
- Standing semantics in MuJoCo are modeled by freezing/resetting gait phase to zero; this still needs runtime parity confirmation against Isaac rollouts.

## Immediate Next Steps

1. Run headless MuJoCo smart gait smoke test with the exported `policy.pt`.
2. Confirm exported `policy.json` reports the expected `history_length`, `per_frame_obs_dim`, and `action_dim`.
3. If rollout is unstable or clearly mismatched, compare Isaac and MuJoCo observations using [`scripts/mujoco/dump_isaac_obs.py`](../scripts/mujoco/dump_isaac_obs.py).
4. Save the first valid MuJoCo rollout CSV under `logs/experiments/unifp_smart_gait_mujoco/`.

## Risks to Watch

- History length mismatch between IO export, policy export, and runtime launch arguments.
- Accidentally using stale `policy.json` metadata from an older export.
- Running MuJoCo with an old policy export that still assumes arm-inclusive observations or actions.
- Treating a successful compile/export as proof of rollout parity; that still needs runtime verification.

## Notes

- This document tracks the MuJoCo integration progress for the `base-force-only` smart gait line only.
- Result tuning and behavior evaluation should continue to be recorded separately from this integration status if needed.
