# Coordinate Frames (uniFP_aliengo)

This document names the coordinate frames used in observations/commands.

## Frames
- W (world): global world frame.
- B (base): origin at robot base; axes follow full base orientation `root_quat_w` (roll/pitch/yaw).
- B_yaw (base-yaw): origin at robot base; axes follow base yaw only (roll/pitch removed).
- C_sphere_w (sphere center in world): world position of the spherical center used by `target_position_commands`.
  - xy = base_pos + yaw_rot([sphere_center_x, sphere_center_y, 0])
  - z = sphere_center_z (fixed world height)
- S_yaw (sphere-centered yaw-only): origin at C_sphere_w; axes aligned with B_yaw.

## Conventions
- Base yaw is extracted from `root_quat_w` and built as `quat_from_euler_xyz(0, 0, yaw)`.
- Yaw-only rotations use `quat_apply` / `quat_apply_inverse` with `base_yaw_quat`.
- Spherical coordinates are `[radius, pitch, yaw]` with:
  - radius = ||[x, y, z]||
  - pitch = atan2(z, sqrt(x^2 + y^2))
  - yaw = atan2(y, x)
- Units: meters and radians.

## Observation/command mapping
- `ee_pos_in_sphere`: EE position in S_yaw, then converted to spherical coords.
- `ee_goal_pos_in_sphere`: goal command already in S_yaw spherical coords (`command[:, :3]`).
- `ee_goal_offset_local_sphere`: goal in W -> add force offset (W) -> subtract C_sphere_w
  -> rotate to B_yaw -> convert to spherical (S_yaw).
- `ee_pos_in_base_frame`: EE position in B.
- `ee_goal_pos_in_base_frame`: goal position in B.
- `ee_tracking_error_in_base_frame`: (goal - ee) in B.
- `gripper_forces_local` / `base_forces_local` / `forces_local`: world vectors -> B_yaw.

## Source references
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/utils.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
