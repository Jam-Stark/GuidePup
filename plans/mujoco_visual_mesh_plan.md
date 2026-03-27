# MuJoCo Visual Mesh Layer Plan for AliengoZ1

## Goal

- Add a visual-only mesh layer to [`resources/aliengoZ1/mujoco/aliengoZ1.xml`](../resources/aliengoZ1/mujoco/aliengoZ1.xml) so the viewer looks closer to the URDF visual model while keeping the existing collision-first physics baseline intact.
- Do this only with MuJoCo-supported mesh assets, not by referencing URDF `.dae` files directly.
- Preserve the current contact and self-collision setup centered on [`contact.exclude`](../resources/aliengoZ1/mujoco/aliengoZ1.xml), [`geom.contype`](../resources/aliengoZ1/mujoco/aliengoZ1.xml), and [`geom.conaffinity`](../resources/aliengoZ1/mujoco/aliengoZ1.xml).

## Approved Scope for V1

- Include:
  - trunk
  - quadruped main links: hip, thigh, calf
  - Z1 main links: arm_link00 to arm_link06
  - gripperStator and gripperMover
  - foot indexing remains in scope, but V1 rendering keeps the existing sphere representation instead of introducing a new mesh asset
- Exclude in V1:
  - rotor links
  - [`base`](../resources/aliengoZ1/urdf/aliengoZ1.urdf)
  - [`imu_link`](../resources/aliengoZ1/urdf/aliengoZ1.urdf)
  - terrain and hfield work

## Current Situation

- The current MuJoCo model is physics-oriented and mostly renders simplified collision geoms from [`resources/aliengoZ1/mujoco/aliengoZ1.xml`](../resources/aliengoZ1/mujoco/aliengoZ1.xml).
- The only mesh-based rendering already present in MJCF is the gripper collision mesh pair.
- The URDF already contains a complete visual-mesh layer for trunk, legs, Z1 arm, and gripper in [`resources/aliengoZ1/urdf/aliengoZ1.urdf`](../resources/aliengoZ1/urdf/aliengoZ1.urdf).

## MuJoCo Constraints That Change the Design

- MuJoCo mesh assets only accept `.stl`, `.obj`, or `.msh`, so the URDF visual `.dae` files are source assets, not directly loadable MJCF mesh inputs.
- V1 should therefore introduce an explicit visual-mesh conversion step from URDF `.dae` sources to MuJoCo-compatible `.obj` targets.
- The conversion step must also validate unit and scale consistency after `dae -> obj`; if a converted visual mesh comes in at the wrong size, the preferred correction point is the mesh asset `scale`, not the physics geom layer.
- MuJoCo re-centers and principal-axis aligns compiled mesh assets, so XML `pos` and `quat` cannot be treated as the full story for final mesh placement.
- The implementation must include mesh-frame validation and, where needed, compensation through mesh-asset transforms such as `refpos` and `refquat`.
- The default viewer only guarantees visibility for low-numbered geom groups, so the visual-only layer should use an explicit visible group number instead of an arbitrary debug group.
- Do not enable compiler-side visual discarding for this pass, otherwise `contype=0` and `conaffinity=0` visual-only geoms may be removed during compilation.

## Asset Inventory

### URDF source visual assets in DAE

- trunk: [`resources/aliengoZ1/meshes/trunk.dae`](../resources/aliengoZ1/meshes/trunk.dae)
- hip: [`resources/aliengoZ1/meshes/hip.dae`](../resources/aliengoZ1/meshes/hip.dae)
- left-side thigh: [`resources/aliengoZ1/meshes/thigh.dae`](../resources/aliengoZ1/meshes/thigh.dae)
- right-side thigh: [`resources/aliengoZ1/meshes/thigh_mirror.dae`](../resources/aliengoZ1/meshes/thigh_mirror.dae)
- calf: [`resources/aliengoZ1/meshes/calf.dae`](../resources/aliengoZ1/meshes/calf.dae)

### Z1 arm visual meshes

- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link00.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link00.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link01.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link01.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link02.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link02.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link03.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link03.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link04.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link04.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link05.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link05.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link06.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link06.dae)

### Gripper visual meshes

- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperStator.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperStator.dae)
- [`resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperMover.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperMover.dae)

### Existing gripper collision meshes already used by MJCF

- [`resources/aliengoZ1/meshes/z1_description/meshes/collision/z1_GripperStator.STL`](../resources/aliengoZ1/meshes/z1_description/meshes/collision/z1_GripperStator.STL)
- [`resources/aliengoZ1/meshes/z1_description/meshes/collision/z1_GripperMover.STL`](../resources/aliengoZ1/meshes/z1_description/meshes/collision/z1_GripperMover.STL)

### Planned MuJoCo-compatible visual mesh outputs

- Output directory: [`resources/aliengoZ1/mujoco/visual_meshes/`](../resources/aliengoZ1/mujoco/visual_meshes/)
- Preferred target format: `.obj`
- Planned first-pass converted assets:
  - `trunk.obj`
  - `hip.obj`
  - `thigh.obj`
  - `thigh_mirror.obj`
  - `calf.obj`
  - `z1_Link00.obj` to `z1_Link06.obj`
  - `z1_GripperStator.obj`
  - `z1_GripperMover.obj`

## URDF to Mesh to MJCF Mapping Index

| Area | URDF link | URDF visual source | URDF collision source | Current MJCF body or geom | Planned visual-only attachment |
|---|---|---|---|---|---|
| trunk | `trunk` | [`trunk.dae`](../resources/aliengoZ1/meshes/trunk.dae) via [`resources/aliengoZ1/urdf/aliengoZ1.urdf`](../resources/aliengoZ1/urdf/aliengoZ1.urdf) | box | `trunk` body with `trunk_geom` | add converted `trunk.obj` visual geom on `trunk` body |
| arm base | `arm_link00` | [`z1_Link00.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link00.dae) via [`resources/aliengoZ1/urdf/aliengoZ1.urdf`](../resources/aliengoZ1/urdf/aliengoZ1.urdf) | cylinder | currently approximated by `arm_mount_geom` on `trunk` | add converted `z1_Link00.obj` visual geom at fixed pose from `trunk_arm_joint0` |
| arm 1 | `arm_link01` | [`z1_Link01.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link01.dae) | cylinder | `arm_link01_geom` | add converted `z1_Link01.obj` visual geom on `arm_link01` |
| arm 2 | `arm_link02` | [`z1_Link02.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link02.dae) | 3 cylinders | `arm_link02_shoulder_geom`, `arm_link02_upper_geom`, `arm_link02_elbow_geom` | add converted `z1_Link02.obj` visual geom on `arm_link02` |
| arm 3 | `arm_link03` | [`z1_Link03.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link03.dae) | 2 cylinders | `arm_link03_forearm_geom`, `arm_link03_wrist_geom` | add converted `z1_Link03.obj` visual geom on `arm_link03` |
| arm 4 | `arm_link04` | [`z1_Link04.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link04.dae) | cylinder | `arm_link04_geom` | add converted `z1_Link04.obj` visual geom on `arm_link04` |
| arm 5 | `arm_link05` | [`z1_Link05.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link05.dae) | none in URDF | `arm_link05_geom` as MuJoCo-side approximation | add converted `z1_Link05.obj` visual geom on `arm_link05` |
| arm 6 | `arm_link06` | [`z1_Link06.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_Link06.dae) | cylinder | `arm_link06_stub_geom` | add converted `z1_Link06.obj` visual geom on `arm_link06` |
| gripper stator | `gripperStator` | [`z1_GripperStator.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperStator.dae) | STL mesh | `gripper_stator_geom` using collision STL | keep collision geom, add separate converted `z1_GripperStator.obj` visual geom |
| gripper mover | `gripperMover` | [`z1_GripperMover.dae`](../resources/aliengoZ1/meshes/z1_description/meshes/visual/z1_GripperMover.dae) | STL mesh | `gripper_mover_geom` using collision STL | keep collision geom, add separate converted `z1_GripperMover.obj` visual geom |
| FR hip | `FR_hip` | [`hip.dae`](../resources/aliengoZ1/meshes/hip.dae) | cylinder | `FR_hip_geom` | add converted `hip.obj` visual geom on `FR_hip` |
| FR thigh | `FR_thigh` | [`thigh_mirror.dae`](../resources/aliengoZ1/meshes/thigh_mirror.dae) | box | `FR_thigh_geom` | add converted `thigh_mirror.obj` visual geom on `FR_thigh` |
| FR calf | `FR_calf` | [`calf.dae`](../resources/aliengoZ1/meshes/calf.dae) | box | `FR_calf_geom` | add converted `calf.obj` visual geom on `FR_calf` |
| FR foot | `FR_foot` | small sphere | larger sphere | `FR_foot_geom` | optional keep sphere-only visual in V1 |
| FL hip | `FL_hip` | [`hip.dae`](../resources/aliengoZ1/meshes/hip.dae) | cylinder | `FL_hip_geom` | add converted `hip.obj` visual geom on `FL_hip` |
| FL thigh | `FL_thigh` | [`thigh.dae`](../resources/aliengoZ1/meshes/thigh.dae) | box | `FL_thigh_geom` | add converted `thigh.obj` visual geom on `FL_thigh` |
| FL calf | `FL_calf` | [`calf.dae`](../resources/aliengoZ1/meshes/calf.dae) | box | `FL_calf_geom` | add converted `calf.obj` visual geom on `FL_calf` |
| FL foot | `FL_foot` | small sphere | larger sphere | `FL_foot_geom` | optional keep sphere-only visual in V1 |
| RR hip | `RR_hip` | [`hip.dae`](../resources/aliengoZ1/meshes/hip.dae) with mirrored visual pose | cylinder | `RR_hip_geom` | add converted `hip.obj` visual geom on `RR_hip` |
| RR thigh | `RR_thigh` | [`thigh_mirror.dae`](../resources/aliengoZ1/meshes/thigh_mirror.dae) | box | `RR_thigh_geom` | add converted `thigh_mirror.obj` visual geom on `RR_thigh` |
| RR calf | `RR_calf` | [`calf.dae`](../resources/aliengoZ1/meshes/calf.dae) | box | `RR_calf_geom` | add converted `calf.obj` visual geom on `RR_calf` |
| RR foot | `RR_foot` | small sphere | larger sphere | `RR_foot_geom` | optional keep sphere-only visual in V1 |
| RL hip | `RL_hip` | [`hip.dae`](../resources/aliengoZ1/meshes/hip.dae) with mirrored visual pose | cylinder | `RL_hip_geom` | add converted `hip.obj` visual geom on `RL_hip` |
| RL thigh | `RL_thigh` | [`thigh.dae`](../resources/aliengoZ1/meshes/thigh.dae) | box | `RL_thigh_geom` | add converted `thigh.obj` visual geom on `RL_thigh` |
| RL calf | `RL_calf` | [`calf.dae`](../resources/aliengoZ1/meshes/calf.dae) | box | `RL_calf_geom` | add converted `calf.obj` visual geom on `RL_calf` |
| RL foot | `RL_foot` | small sphere | larger sphere | `RL_foot_geom` | optional keep sphere-only visual in V1 |

## Pose and Mirror Rules to Preserve

### Trunk and arm root

- Use the URDF trunk visual origin as the initial target pose, then validate the compiled MuJoCo mesh frame after conversion.
- Represent `arm_link00` as a visual-only mesh attached using the fixed joint pose from `trunk` to `arm_link00`, but treat that URDF transform as a pre-compilation target that may still need mesh-asset correction.

### Z1 main links

- For `arm_link01` to `arm_link06`, use the URDF visual `origin xyz/rpy` as the initial alignment target from their corresponding link visual blocks in [`resources/aliengoZ1/urdf/aliengoZ1.urdf`](../resources/aliengoZ1/urdf/aliengoZ1.urdf).
- Do not infer mesh orientation from the simplified collision geoms; the URDF visual transform is the source of truth before MuJoCo mesh compilation offsets are accounted for.
- After conversion to `.obj`, explicitly validate whether the compiled mesh frame requires `refpos` or `refquat` compensation.

### Quadruped legs

- Preserve the distinct hip visual rotations already encoded in the URDF for:
  - `FR_hip`
  - `FL_hip`
  - `RR_hip`
  - `RL_hip`
- Preserve the thigh mesh split:
  - `FR_thigh` and `RR_thigh` use [`thigh_mirror.dae`](../resources/aliengoZ1/meshes/thigh_mirror.dae)
  - `FL_thigh` and `RL_thigh` use [`thigh.dae`](../resources/aliengoZ1/meshes/thigh.dae)
- Calf uses the same [`calf.dae`](../resources/aliengoZ1/meshes/calf.dae) across all four legs.
- Hip, thigh, and calf placements must be checked after mesh compilation because MuJoCo may shift the effective geom frame relative to the source mesh frame.

## Visual Layer Design

### MJCF asset strategy

- Extend the [`asset`](../resources/aliengoZ1/mujoco/aliengoZ1.xml) block with converted `.obj` visual meshes for:
  - trunk
  - Z1 Link00 to Link06
  - gripper visual meshes
  - hip, thigh, thigh_mirror, calf
- Do not reference `.dae` directly from MJCF mesh assets.
- If some converted meshes exhibit frame drift after import, store the correction in the mesh asset using `refpos` and `refquat` instead of distorting the physics geom layer.
- If some converted meshes exhibit size drift after import, store the correction in the mesh asset `scale` instead of changing the physics geoms.

### Visual-only geom policy

- Add a dedicated visual-only default class.
- Set visual-only geoms to:
  - `contype=0`
  - `conaffinity=0`
- Put visual-only geoms into viewer `group=2` so they remain visible in the default viewer while still being distinguishable from the collision layer.
- Keep collision geoms in their current default-visible grouping rather than moving them into a hidden debug-only group.
- Keep all current physics geoms unchanged unless a pose fix is strictly necessary for visual alignment.
- Keep the current [`compiler`](../resources/aliengoZ1/mujoco/aliengoZ1.xml) configuration free of discard-visual behavior while this visual-only layer is being introduced.

### Foot policy for V1

- Default choice: keep feet as spheres in both physics and rendering for the first pass.
- Feet are still part of the indexing table and body mapping, but they are not part of the visual-mesh conversion batch in V1.
- Reason: the URDF foot visual is already sphere-based, and this avoids unnecessary complexity while the major appearance gap is really trunk plus leg meshes plus Z1 meshes.

## Non-Goals for This Pass

- No rotor visuals in V1.
- No visual polish for [`base`](../resources/aliengoZ1/urdf/aliengoZ1.urdf) or [`imu_link`](../resources/aliengoZ1/urdf/aliengoZ1.urdf).
- No changes to contact tuning, self-collision bitmasks, terrain, or hfield.
- No attempt to merge visual mesh geoms with physics geoms.

## Implementation Sequence

1. Create a conversion batch from the URDF visual `.dae` sources to MuJoCo-compatible `.obj` files under [`resources/aliengoZ1/mujoco/visual_meshes/`](../resources/aliengoZ1/mujoco/visual_meshes/).
2. Add the converted visual-mesh asset declarations to [`resources/aliengoZ1/mujoco/aliengoZ1.xml`](../resources/aliengoZ1/mujoco/aliengoZ1.xml).
3. Add a `visual_only` default class with `contype=0`, `conaffinity=0`, and `group=2`.
4. Attach trunk visual mesh.
5. Attach `arm_link00` visual mesh to `trunk` using the URDF fixed-joint pose as the initial target.
6. Attach `arm_link01` to `arm_link06` visual meshes to their matching MuJoCo bodies.
7. Add gripper visual `.obj` meshes while keeping the existing collision STLs for physics.
8. Attach hip, thigh, and calf visual meshes for all four legs, preserving URDF leg-specific `rpy` and the `thigh` vs `thigh_mirror` split.
9. Keep current foot spheres unless a later viewer-only refinement is justified.
10. Validate compiled mesh placement for trunk, arm base, and all mirrored leg links; use mesh-asset `refpos` and `refquat` where required.
11. Validate converted mesh units and overall scale against the URDF and existing physics geoms; use mesh-asset `scale` where required.
12. Verify viewer appearance, confirm no visual-only geoms were discarded by compiler settings, then rerun the existing setup and headless checks.

## Validation Checklist

- Viewer shows visual mesh layer and collision layer coexisting.
- Visual appearance is closer to the URDF visual model than the current collision-only look.
- Existing physics geoms remain in place and still explain contact behavior.
- No MJCF asset directly references `.dae` as a MuJoCo mesh input.
- Converted `.obj` assets load successfully without compile-time mesh format errors.
- Trunk, `arm_link00`, and all four hips visually align after checking for MuJoCo mesh centering and principal-axis alignment effects.
- Converted visual meshes have expected overall size relative to the URDF and the existing collision geoms.
- No compiler option is discarding the `contype=0` and `conaffinity=0` visual-only geoms.
- [`scripts/mujoco/check_setup.py`](../scripts/mujoco/check_setup.py) still loads the model.
- [`scripts/mujoco/play_policy.py`](../scripts/mujoco/play_policy.py) still runs in headless mode on the updated model.
- No new self-collision chatter appears due to visual-only mesh additions.

## Handoff Note

- This plan deliberately separates visual fidelity from physics fidelity.
- The visual-only layer should be treated as an additive rendering pass on top of the already-established contact and self-collision baseline.
