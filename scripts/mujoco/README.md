# MuJoCo Sim2Sim Handoff

This document is the handoff entry point for the MuJoCo-side sim2sim work for the UniFP Aliengo task.

Remote visualization and Xpra operations are documented in `scripts/mujoco/XPRA_OPERATIONS.md`.

## Latest Update

The MuJoCo side now includes the staged-direction-force evaluation path needed to mirror
`scripts/experiments/uniFP_aliengo/staged_direction_force_eval.py`.

New entry points:

- `scripts/mujoco/runtime.py`
  - shared policy/runtime helpers used by both playback and staged evaluation
  - owns history stacking, delay buffers, quaternion math, actuator lookup, control-target assembly, metadata loading
- `scripts/mujoco/export_policy_wrapper.py`
  - exports the Isaac adaptation policy into a MuJoCo-ready TorchScript module
  - writes `policy.pt` and sibling `policy.json` metadata into the chosen output directory
  - validates exported `(action, pred_est)` against the original Isaac policy on real `obs["policy"]`
- `scripts/mujoco/staged_direction_force_eval.py`
  - MuJoCo reproduction of the 60-second staged-direction-force experiment
  - reproduces the stage schedule, CSV schema, and terminal summary used by the Isaac-side experiment
- `scripts/mujoco/play_policy.py`
  - now consumes the shared runtime module instead of carrying a separate copy of the observation/control logic

Expected export flow:

```bash
conda run -n isaaclab python scripts/mujoco/export_policy_wrapper.py \
  --checkpoint <path-to-checkpoint>
```

Expected MuJoCo staged-eval flow:

```bash
conda run -n mujoco python scripts/mujoco/staged_direction_force_eval.py \
  --policy-path <export-dir>/policy.pt \
  --headless --duration-s 1.0
```

## Background

The Isaac training side already exists in this repository. The MuJoCo side is intended to reuse the trained policy and reconstruct the policy runtime, not to reimplement the whole Isaac task stack.

Relevant Isaac-side references:

- Policy export already exists in `scripts/export_vbc.py:68-71` and `scripts/rsl_rl/play.py:191-194`.
- The current Isaac task is `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`.
- Isaac actuator settings are in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:480-539`.
- Isaac action interface is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:575-612`.
- Isaac policy observation layout is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:648-745`.
- Isaac randomization/events are in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1114-1261`.
- Isaac sim timing is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1554-1562`.

Current policy/runtime assumptions copied from Isaac:

- 17 policy-controlled joints
- 73 observation dimensions per frame
- 32-frame history stack
- 200 Hz physics, 50 Hz policy loop
- optional observation delay `0-2`
- optional leg-action delay `0-1`

Those values are mirrored in `scripts/mujoco/play_policy.py:33-108`.

## What Was Done

### 1. Added a MuJoCo runtime scaffold under `scripts/mujoco/`

Files:

- `scripts/mujoco/requirements.txt:1-7`
- `scripts/mujoco/check_setup.py:17-31`
- `scripts/mujoco/play_policy.py:33-108`
- `scripts/mujoco/model_loader.py:1-57`

Purpose:

- isolate MuJoCo-side work from the Isaac training code
- provide a minimal validation path before connecting the real policy
- keep all runtime logic in one place for the next engineer

### 2. Added a first MJCF skeleton

File:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml:1-205`

What is inside:

- MuJoCo compiler and physics options
- ground plane
- floating base with trunk inertia
- arm and leg body/joint tree
- structured contact defaults and contact filtering
- 18 position actuators
- basic sensors
- `home` keyframe

Important sections:

- compiler and timestep: `resources/aliengoZ1/mujoco/aliengoZ1.xml:1-3`
- contact defaults: `resources/aliengoZ1/mujoco/aliengoZ1.xml:10-33`
- ground and trunk body: `resources/aliengoZ1/mujoco/aliengoZ1.xml:35-46`
- arm chain: `resources/aliengoZ1/mujoco/aliengoZ1.xml:48-88`
- quadruped legs: `resources/aliengoZ1/mujoco/aliengoZ1.xml:90-156`
- contact excludes: `resources/aliengoZ1/mujoco/aliengoZ1.xml:160-168`
- actuator block: `resources/aliengoZ1/mujoco/aliengoZ1.xml:170-192`
- sensor block: `resources/aliengoZ1/mujoco/aliengoZ1.xml:194-198`
- home keyframe: `resources/aliengoZ1/mujoco/aliengoZ1.xml:200-203`

### 3. Solved MuJoCo resource loading for URDF and MJCF

Problem:

- MuJoCo failed to load the original URDF because mesh path handling was brittle.
- The repository contains many non-mesh files under `resources/aliengoZ1`, so naive asset loading also caused collisions.

Fix:

- `scripts/mujoco/model_loader.py:20-57`

What it does:

- strips XML comments before parsing asset references
- extracts only actually referenced `file=` or `filename=` paths
- builds a MuJoCo `assets` map from only the needed files
- handles URDF and MJCF differently:
  - URDF path mode uses basename keys
  - MJCF path mode uses paths relative to the model file

### 4. Pointed the default runtime to the new MJCF

Files:

- `scripts/mujoco/check_setup.py:20-23`
- `scripts/mujoco/play_policy.py` argument parser `--model-path` default

Current default model:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml`

### 5. Refined `aliengoZ1.xml` contact taxonomy and self-collision baseline

Files:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml:10-205`

Relevant Isaac-side references:

- terrain material and ground friction baseline: `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:373-390`
- URDF spawn and self-collision enable: `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:403-445`
- foot contact sensor target bodies: `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:548-559`
- penalized contacts on `thigh`, `calf`, `trunk`: `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:561-568`

What changed:

- renamed the floating-base body from `base` to `trunk` so MuJoCo naming aligns better with Isaac-side contact semantics
- added explicit geom names for trunk, arm mount, arm links, gripper, hips, thighs, calves, and feet
- introduced MuJoCo contact classes for:
  - ground
  - feet
  - locomotion body geoms (`trunk`, `hip`, `thigh`, `calf`)
  - arm / gripper geoms
- assigned first-pass `contype` / `conaffinity` bitmasks to avoid all-geoms-collide-with-all behavior
- tuned first-pass contact parameters per class with explicit `friction`, `condim`, `solref`, `solimp`, `margin`, and `priority`
- preserved separate foot-ground behavior by making foot geoms higher-priority contacts than generic body contacts
- preserved penalized-contact observability by keeping `trunk`, `thigh`, and `calf` geoms collidable with ground/world contacts instead of globally excluding them
- added a minimal `contact/exclude` policy for high-risk chatter pairs:
  - trunk vs nearby arm links
  - selected non-neighbor arm link pairs that are prone to persistent self-contact
- updated the IMU / base orientation sensor attachment to reference the renamed `trunk` body

Implementation map:

- contact class defaults: `resources/aliengoZ1/mujoco/aliengoZ1.xml:10-33`
- trunk and arm-mount collision geoms: `resources/aliengoZ1/mujoco/aliengoZ1.xml:38-46`
- arm collision geoms and gripper naming: `resources/aliengoZ1/mujoco/aliengoZ1.xml:48-88`
- leg collision geoms and explicit foot contact geoms: `resources/aliengoZ1/mujoco/aliengoZ1.xml:90-156`
- self-collision suppression list: `resources/aliengoZ1/mujoco/aliengoZ1.xml:160-168`
- sensor body rename fix: `resources/aliengoZ1/mujoco/aliengoZ1.xml:194-198`

Why this matters:

- the MuJoCo model can now map more cleanly onto Isaac-side feet contact and penalized-contact concepts
- self-collision is no longer purely “everything collides with everything”; it is now a controlled first-pass approximation of Isaac `self_collision=True`
- the runner remains compatible while giving the next tuning pass cleaner structure for contact logging, penalties, and terrain upgrades

### 6. Added a MuJoCo-compatible visual mesh layer on top of the physics model

Files:

- `plans/mujoco_visual_mesh_plan.md:1-183`
- `scripts/mujoco/convert_visual_meshes.py:1-444`
- generated assets under `resources/aliengoZ1/mujoco/visual_meshes/`
- `resources/aliengoZ1/mujoco/aliengoZ1.xml:5-197`

Relevant references:

- URDF trunk visual/collision split: `resources/aliengoZ1/urdf/aliengoZ1.urdf:304-323`
- URDF Z1 visual meshes: `resources/aliengoZ1/urdf/aliengoZ1.urdf:357-596`
- URDF quadruped visual meshes: `resources/aliengoZ1/urdf/aliengoZ1.urdf:685-1494`
- MuJoCo mesh format constraints and mesh centering/alignment behavior were captured in `plans/mujoco_visual_mesh_plan.md`

What changed:

- added a dedicated implementation plan for the visual-only render layer in `plans/mujoco_visual_mesh_plan.md:1-183`
- added `scripts/mujoco/convert_visual_meshes.py:1-444` to convert approved URDF visual `.dae` meshes into MuJoCo-compatible `.obj` assets
- converted the V1 visual mesh set into `resources/aliengoZ1/mujoco/visual_meshes/`
- recorded conversion metadata, including source path, unit scale, bbox, extent, and vertex/face counts, in `resources/aliengoZ1/mujoco/visual_meshes/visual_mesh_manifest.json:1-353`
- extended the MJCF `asset` block with visual mesh assets for:
  - trunk
  - hip
  - thigh
  - thigh_mirror
  - calf
  - z1_Link00 to z1_Link06
  - z1_GripperStator and z1_GripperMover
- added a `visual_only` default class in `resources/aliengoZ1/mujoco/aliengoZ1.xml:48-50`
  - `contype=0`
  - `conaffinity=0`
  - `group=2`
- attached visual-only mesh geoms to the existing trunk, arm, leg, and gripper bodies while leaving all physics geoms intact
- kept feet on the existing sphere representation for V1 instead of introducing a new foot mesh pipeline

Implementation map:

- conversion tool entry point: `scripts/mujoco/convert_visual_meshes.py:414-444`
- COLLADA-to-OBJ conversion core: `scripts/mujoco/convert_visual_meshes.py:252-389`
- conversion spec inventory: `scripts/mujoco/convert_visual_meshes.py:24-95`
- generated visual mesh manifest: `resources/aliengoZ1/mujoco/visual_meshes/visual_mesh_manifest.json:1-353`
- MJCF visual asset declarations: `resources/aliengoZ1/mujoco/aliengoZ1.xml:5-21`
- MJCF `visual_only` class: `resources/aliengoZ1/mujoco/aliengoZ1.xml:48-50`
- trunk and arm visual geoms: `resources/aliengoZ1/mujoco/aliengoZ1.xml:62-110`
- quadruped leg visual geoms: `resources/aliengoZ1/mujoco/aliengoZ1.xml:118-193`

Scale and frame notes:

- the converter reads COLLADA `unit` and `up_axis`, and bakes them into the exported OBJ coordinates
- the generated manifest shows that most assets are `unit_meter=1.0`, while `calf.dae` is `unit_meter=0.0254`, which is now explicitly captured and handled by the conversion pipeline rather than left implicit
- MuJoCo compiler-side `mesh_pos`, `mesh_quat`, and `mesh_scale` were inspected after load for the visual mesh assets to assess whether `refpos`, `refquat`, or `scale` corrections were needed
- based on the current viewer result, no first-pass `refpos`, `refquat`, or `scale` compensation was required

Why this matters:

- the MuJoCo viewer now shows a model much closer to the URDF visual appearance instead of mostly collision primitives
- visual fidelity is improved without changing the existing contact/self-collision physics layer
- the conversion script and manifest make future mesh regeneration auditable and repeatable

## Current Code Map

### Environment setup

- Dependencies: `scripts/mujoco/requirements.txt:1-7`
- Runtime validation entry point: `scripts/mujoco/check_setup.py:34-83`
- Model loader used by both checker and runner: `scripts/mujoco/model_loader.py:30-57`

### Policy/runtime interface

- Isaac-compatible policy constants: `scripts/mujoco/play_policy.py:33-108`
- frame stack logic: `scripts/mujoco/play_policy.py:111-128`
- fixed observation delay: `scripts/mujoco/play_policy.py:131-147`
- fixed leg action delay: `scripts/mujoco/play_policy.py:150-170`
- command vector assembly: `scripts/mujoco/play_policy.py:212-224`

### MuJoCo model interface

- joint lookup: `scripts/mujoco/play_policy.py:227-242`
- actuator lookup: `scripts/mujoco/play_policy.py:245-273`
- reset to Isaac-like initial state: `scripts/mujoco/play_policy.py:347-363`

### Observation reconstruction

- projected gravity / body-frame math: `scripts/mujoco/play_policy.py:180-209`
- single-frame observation assembly: `scripts/mujoco/play_policy.py:290-322`

### Main runner

- CLI: `scripts/mujoco/play_policy.py:366-429`
- main entry: `scripts/mujoco/play_policy.py:432-551`
- control loop:
  - policy inference
  - delayed action application
  - MuJoCo stepping
  - delayed proprio rebuild
  are in `scripts/mujoco/play_policy.py:492-537`

## What Was Verified

The following commands were run in the `mujoco` conda environment and succeeded:

```bash
python scripts/mujoco/check_setup.py
python scripts/mujoco/play_policy.py --headless --duration-s 0.1
```

After the contact / self-collision refinement, the following commands were also rerun successfully in the same environment:

```bash
python scripts/mujoco/check_setup.py --model-path resources/aliengoZ1/mujoco/aliengoZ1.xml
python scripts/mujoco/play_policy.py --model-path resources/aliengoZ1/mujoco/aliengoZ1.xml --headless --duration-s 0.1
```

After the visual mesh layer was added, the following commands also succeeded:

```bash
python scripts/mujoco/convert_visual_meshes.py --overwrite
python scripts/mujoco/check_setup.py --model-path resources/aliengoZ1/mujoco/aliengoZ1.xml
python scripts/mujoco/play_policy.py --model-path resources/aliengoZ1/mujoco/aliengoZ1.xml --headless --duration-s 0.1
```

Observed result for the current MJCF:

- `nq=25`
- `nv=24`
- `nu=18`
- `nbody=20`
- `njnt=19`

The original URDF still loads through the checker and reports:

- `nq=18`
- `nv=18`
- `nu=0`

That is expected and confirms the current split:

- URDF is still useful as geometry/reference input
- MJCF is now the actual sim2sim runtime model
- visual meshes are now an additive render layer on top of the physics-focused MJCF geoms

## Known Gaps

These are still open by design:

- actuator tuning is only a first-pass mapping from Isaac stiffness/damping to MuJoCo `position` actuators
- contact tuning now has a first-pass taxonomy / bitmask / exclude structure, but it is not yet calibrated against Isaac traces
- self-collision behavior now has a conservative baseline policy, but it still needs iterative validation against Isaac behavior
- terrain is still a plane, not the Isaac rough-flat terrain
- the runner does not yet reproduce Isaac command-manager logic for force injection or end-effector goal generation
- policy replay with a real exported `policy.pt` has not been validated yet

## Important Caveat

`resources/aliengoZ1/mujoco/aliengoZ1.xml` is currently ignored by git because the repository ignores `/resources/` from `.gitignore`.

That means:

- the file exists on disk
- the runtime uses it
- `git status` will not show it until ignore rules are adjusted

## Recommended Next Step

The next engineer should start here:

1. Export or locate the real `policy.pt`.
2. Run `python scripts/mujoco/check_setup.py --policy-path path/to/policy.pt`.
3. Run `python scripts/mujoco/play_policy.py --headless --duration-s 1.0 --policy-path path/to/policy.pt`.
4. Only after basic policy replay works, start refining MuJoCo physics fidelity.

## Initial Plan: Refine `aliengoZ1.xml` Contact / Self-Collision / HField

This is the initial plan for making the MuJoCo model closer to Isaac physics.

### Phase 1. Contact geometry audit

Goal:

- make sure trunk, thighs, calves, feet, and arm collision shapes reflect the bodies that matter in Isaac

Relevant Isaac reference:

- contact sensors and penalized contacts are in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:548-568`

Tasks:

- inspect which MuJoCo geoms should count as feet, thighs, calves, and trunk
- add explicit geom names and consistent grouping
- ensure the next engineer can map Isaac contact terms to MuJoCo contact pairs cleanly

Expected files:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml`

### Phase 2. Self-collision policy

Goal:

- make MuJoCo self-collision behavior closer to Isaac `self_collision=True`

Relevant Isaac reference:

- robot spawn/self-collision setting is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:403-445`

Tasks:

- decide which body pairs should collide
- use `contype`, `conaffinity`, and `contact/exclude` to suppress obviously bad pairs
- preserve leg-ground and relevant leg-body interactions
- prevent pathological arm-trunk or neighboring-link collision chatter

Expected files:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml`

### Phase 3. Ground contact tuning

Goal:

- tune foot-ground behavior before introducing rough terrain

Relevant Isaac reference:

- terrain material is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:373-390`
- randomized friction range is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1156-1167`

Tasks:

- tune `geom friction`
- tune `solref` and `solimp`
- tune foot shape and contact dimensionality
- compare whether stance timing and foot slip roughly match Isaac flat-ground behavior

Expected files:

- `resources/aliengoZ1/mujoco/aliengoZ1.xml`
- maybe `scripts/mujoco/play_policy.py` if extra debug logging is needed

### Phase 4. Isaac rough-flat to MuJoCo hfield

Goal:

- replace the current plane with a MuJoCo `hfield` that approximates Isaac rough-flat terrain

Relevant Isaac reference:

- terrain generator is in `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:330-362`

Tasks:

- reproduce the rough-flat noise range and resolution in a standalone script
- generate either:
  - an in-XML heightfield asset
  - a binary/PNG-backed heightfield asset
- attach the `hfield` geom to world
- validate that the base runner still works before combining with policy replay

Expected new files:

- `scripts/mujoco/build_hfield.py`
- generated terrain asset under `resources/aliengoZ1/mujoco/`
- updates to `resources/aliengoZ1/mujoco/aliengoZ1.xml`

### Phase 5. Comparison and iteration loop

Goal:

- turn MuJoCo refinement into a measurable iteration loop

Tasks:

- record contact statistics, base height, stance timing, and foot slip
- compare MuJoCo traces against Isaac traces on the same command sequence
- only then start tuning actuator/contact/hfield parameters together

Expected files:

- `scripts/mujoco/compare_trace.py`
- optional debug instrumentation in `scripts/mujoco/play_policy.py`

## Suggested Ownership For The Next Engineer

If one person continues this work, the critical path is:

1. validate real policy playback on the current MJCF
2. refine contact and self-collision
3. add hfield
4. add Isaac-vs-MuJoCo trace comparison

If multiple people split the work:

- one person owns `resources/aliengoZ1/mujoco/aliengoZ1.xml`
- one person owns `scripts/mujoco/play_policy.py`
- one person owns terrain generation and comparison tooling
