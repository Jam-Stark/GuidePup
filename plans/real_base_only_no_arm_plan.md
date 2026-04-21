# Real-Base-Only No-Arm Conversion Plan

## Summary

- Baseline: convert the active `real-base-only` branch in place to a pure no-arm Aliengo path.
- No compatibility layer: do not keep dual env cfgs, dual gym IDs, arm/no-arm switches, or arm fallback metadata.
- Resource work is out of scope: do not modify `resources/`; later you will import a standalone no-arm Aliengo URDF/MJCF and then fill in the single active asset/model paths.
- Plan backup target for execution mode: `plans/real_base_only_no_arm_plan.md`.
- First-pass scope: active core code only.
  Includes train env, reverse-push eval cfg, IO/policy export, MuJoCo runtime, `play_policy.py`, and `smart_gait.py`.
  Excludes legacy PhaseA/guide-pull lines, doc cleanup, committed descriptor refresh, and `staged_direction_force_eval.py`.

## Key Changes

### 1. Isaac train/eval config: convert active `uniFP_aliengo` to leg-only

- Keep existing active env entrypoints and names on this branch.
  `Template-UniFPAliengo-v0`, `UniFPAliengoEnvCfg`, and `UniFPAliengoReversePushEvalCfg` remain, but their semantics become no-arm only.
- In `unifpaliengo_env_cfg.py`, remove all active arm dependencies:
  arm entries in `init_state`, arm actuators, `arm_actions`, arm joints from reset lists, `randomize_gripper_mass`, and any active `gripperMover` / `arm_joint*` lookups.
- Keep the active locomotion policy contract unchanged unless code proves otherwise:
  policy per-frame obs stays `46`, action dim stays `12`, and task focus remains locomotion + base-force disturbance.
- Remove dormant arm/manipulation config surface from the active branch where it no longer serves the no-arm line.
- In `unifpaliengo_eval_cfg.py`, make reset/tuning leg-only as well.
  `RESET_JOINT_NAMES` becomes leg-only, and eval post-init no longer touches arm actuators.

### 2. Manager terms and metrics: make active runtime fully leg-only

- Keep `leg_torque_metrics` as the active term name, but remove all arm joint resolution and arm utilization metrics from its implementation and cfg defaults.
- Keep `action_distribution_metrics`, but track only `("leg_actions",)`.
- Do not preserve compatibility shims for old arm metric keys.
  Logging schema is allowed to change on this branch.

### 3. Export and policy contract: regenerate no-arm metadata from the same env

- Keep `scripts/export_io_descriptors.py` using the same env construction flow, but ensure exported articulation metadata becomes no-arm once the env cfg is converted.
- Do not add a parallel no-arm export mode.
  `registered_task` and `reverse_push_eval` stay as the only active export construction paths.
- Keep `scripts/mujoco/export_policy_wrapper.py` exporting from `UniFPAliengoReversePushEvalCfg`, but make emitted `policy.json` fully leg-only.
- First pass uses descriptor export as validation only; tracked `io_descriptors/*.yaml` refresh is not required.

### 4. MuJoCo runtime: remove arm assumptions from active runners

- In `scripts/mujoco/runtime.py`, convert `UniFPPolicyMetadata` to the real no-arm contract:
  leg-only `joint_names`, leg-only `default_joint_pos`, no auxiliary arm defaults, and leg-only actuator gains.
- Remove dead arm/EE/gripper metadata from the active locomotion contract if they are not consumed by the active path.
  This includes `default_ee_sphere_cmd`, `default_gripper_force_cmd`, and `freeze_arm_actions` if they remain unused after simplification.
- Simplify `build_command_vector()` to the active locomotion-only command layout.
  Do not keep truncated long-command compatibility.
- Simplify control target construction to assume only the 12 controlled leg joints.
- Update active MuJoCo entrypoints to the same no-arm metadata contract:
  `play_policy.py` and `smart_gait.py`.
- `staged_direction_force_eval.py` is not part of the maintained no-arm path.
  Do not adapt it; remove it from the first-pass maintenance scope and allow later deletion if desired.
- Do not modify model assets in this pass.
  Keep only one active model-path expectation so later no-arm MJCF import becomes a straightforward path swap.

## Test Plan

- Isaac env construction smoke test:
  instantiate the registered task and `UniFPAliengoReversePushEvalCfg` with no `arm_joint*` or `gripperMover` lookups.
- Descriptor export validation:
  export policy IO and confirm `policy` obs remain `46`, action dim remains `12`, and `articulations.robot.joint_names` contains no arm joints.
- Policy export validation:
  run `scripts/mujoco/export_policy_wrapper.py` and confirm exported `policy.json` is leg-only and matches the env-derived contract.
- MuJoCo runtime smoke tests after the no-arm model is imported:
  `play_policy.py --headless` and `smart_gait.py --headless` must initialize without arm joint resolution, auxiliary arm defaults, or frozen-arm logic.
- Regression checks:
  `last_action` history stays `12`, locomotion command obs stays locomotion-only, and smart-gait runtime still reports the expected history length, action dim, and gait frequency.

## Assumptions And Defaults

- No-arm conversion is in-place on `real-base-only`; there will be no sibling `NoArm` cfg and no compatibility switch.
- The later imported no-arm URDF/MJCF preserves the active locomotion naming the code depends on:
  leg joint names, `trunk`, `base`, and foot naming remain compatible.
- Legacy arm/manipulation files outside the active chain are intentionally left alone in the first pass.
- `staged_direction_force_eval.py` is explicitly excluded from the no-arm adaptation plan.
- When execution mode is enabled, the implementation should also write this plan to `plans/real_base_only_no_arm_plan.md`.
