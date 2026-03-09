import torch
import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sensors import ContactSensor


def get_base_yaw_quat(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Return the robot base yaw-only quaternion in world frame."""
    base_quat = env.scene["robot"].data.root_quat_w
    _, _, yaw = math_utils.euler_xyz_from_quat(base_quat)
    return math_utils.quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)


def world_to_base_yaw(points_w: torch.Tensor, base_yaw_quat: torch.Tensor) -> torch.Tensor:
    """Transform world-frame vectors into the base yaw-only frame."""
    return math_utils.quat_apply_inverse(base_yaw_quat, points_w)


def base_yaw_to_world(points_b: torch.Tensor, base_yaw_quat: torch.Tensor) -> torch.Tensor:
    """Transform base yaw-only frame vectors into the world frame."""
    return math_utils.quat_apply(base_yaw_quat, points_b)


def resolve_sphere_center_w(
    env: ManagerBasedRLEnv,
    command_name: str | None = None,
    command_term=None,
    base_yaw_quat: torch.Tensor | None = None,
) -> tuple[torch.Tensor, bool]:
    """Resolve the sphere center in world frame for a command (if configured)."""
    if command_term is None:
        if command_name is None:
            raise ValueError("Either command_name or command_term must be provided.")
        command_term = env.command_manager.get_term(command_name)

    robot_pos_w = env.scene["robot"].data.root_pos_w
    sphere_center_w = robot_pos_w.clone()
    has_sphere_center = hasattr(command_term, "cfg") and hasattr(command_term.cfg, "sphere_center") and hasattr(
        command_term.cfg, "sphere_center_z"
    )
    if has_sphere_center:
        if base_yaw_quat is None:
            base_yaw_quat = get_base_yaw_quat(env)
        sphere_center_xy = command_term.cfg.sphere_center
        sphere_center_z = command_term.cfg.sphere_center_z
        sphere_center_b = torch.zeros_like(robot_pos_w)
        sphere_center_b[:, 0] = float(sphere_center_xy[0])
        sphere_center_b[:, 1] = float(sphere_center_xy[1])
        sphere_center_xy_w = base_yaw_to_world(sphere_center_b, base_yaw_quat)
        sphere_center_w = torch.zeros_like(robot_pos_w)
        sphere_center_w[:, 0] = robot_pos_w[:, 0] + sphere_center_xy_w[:, 0]
        sphere_center_w[:, 1] = robot_pos_w[:, 1] + sphere_center_xy_w[:, 1]
        sphere_center_w[:, 2] = float(sphere_center_z)
    return sphere_center_w, has_sphere_center


def spherical_to_cartesian(sphere_coords: torch.Tensor) -> torch.Tensor:
    """Convert spherical coordinates [r, pitch, yaw] to Cartesian [x, y, z]."""
    radius = sphere_coords[..., 0]
    pitch = sphere_coords[..., 1]
    yaw = sphere_coords[..., 2]

    x = radius * torch.cos(pitch) * torch.cos(yaw)
    y = radius * torch.cos(pitch) * torch.sin(yaw)
    z = radius * torch.sin(pitch)
    return torch.stack([x, y, z], dim=-1)


def cartesian_to_spherical(cart_coords: torch.Tensor) -> torch.Tensor:
    """Convert Cartesian [x, y, z] to spherical coordinates [r, pitch, yaw]."""
    radius = torch.norm(cart_coords, dim=-1)
    xy_norm = torch.norm(cart_coords[:, :2], dim=-1)
    pitch = torch.atan2(cart_coords[:, 2], xy_norm)
    yaw = torch.atan2(cart_coords[:, 1], cart_coords[:, 0])
    return torch.stack([radius, pitch, yaw], dim=-1)


def get_body_pos_w(env: ManagerBasedRLEnv, body_cfg: SceneEntityCfg) -> torch.Tensor:
    """Return the world position of the first matching body."""
    body_indices = env.scene[body_cfg.name].find_bodies(body_cfg.body_names)[0]
    return env.scene[body_cfg.name].data.body_pos_w[:, body_indices[0]]


def get_command_pos_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Return command target position in world frame (pos_w if available)."""
    command_term = env.command_manager.get_term(command_name)
    if hasattr(command_term, "pos_w"):
        return command_term.pos_w
    return command_term.command[:, :3]


def _get_force_command(env: ManagerBasedRLEnv, command_name: str, use_applied: bool) -> torch.Tensor:
    if use_applied:
        cached = getattr(env, "_applied_force_commands", None)
        if isinstance(cached, dict):
            applied = cached.get(command_name)
            if applied is not None:
                return applied
    return env.command_manager.get_command(command_name)


def gripper_forces_local(env: ManagerBasedRLEnv, command_name: str, use_applied: bool = False) -> torch.Tensor:
    """Extract gripper forces in the local frame (yaw-only)."""
    base_yaw_quat = get_base_yaw_quat(env)
    gripper_force_w = _get_force_command(env, command_name, use_applied)
    return world_to_base_yaw(gripper_force_w, base_yaw_quat)


def base_forces_local(env: ManagerBasedRLEnv, command_name: str, use_applied: bool = False) -> torch.Tensor:
    """Extract base forces in the local frame (yaw-only)."""
    base_yaw_quat = get_base_yaw_quat(env)
    base_force_w = _get_force_command(env, command_name, use_applied)
    return world_to_base_yaw(base_force_w, base_yaw_quat)


def forces_local(env: ManagerBasedRLEnv, use_applied: bool = False) -> torch.Tensor:
    """Transform forces to local frame (yaw-only)."""
    base_yaw_quat = get_base_yaw_quat(env)
    gripper_force_w = _get_force_command(env, "gripper_force_ext", use_applied)
    base_force_w = _get_force_command(env, "base_force_ext", use_applied)
    gripper_force_l = world_to_base_yaw(gripper_force_w, base_yaw_quat)
    base_force_l = world_to_base_yaw(base_force_w, base_yaw_quat)

    return torch.cat([gripper_force_l, base_force_l], dim=-1)


def virtual_impedance_params(
    env: ManagerBasedRLEnv, command_name: str = "virtual_impedance_command"
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Return kp, kd, prop_kd from the command manager."""
    command = env.command_manager.get_command(command_name)
    return command[:, 0], command[:, 1], command[:, 2]
