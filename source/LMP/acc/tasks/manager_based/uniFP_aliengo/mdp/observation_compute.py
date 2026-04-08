# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script defines the Markov Decision Process (MDP) for the uniFP_aliengo environment.
"""

from __future__ import annotations

from typing import Sequence
import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_inv

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import transform_points
from .io_descriptor_compat import generic_io_descriptor, record_dtype, record_shape
from .utils import (
    _get_force_command,
    base_yaw_to_world,
    cartesian_to_spherical,
    get_base_yaw_quat,
    get_body_pos_w,
    get_command_pos_w,
    resolve_sphere_center_w,
    spherical_to_cartesian,
    world_to_base_yaw,
)


def _to_descriptor_list(value) -> list | None:
    if value is None:
        return None
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy().tolist()
    if isinstance(value, (tuple, list)):
        return list(value)
    return [value]


def _record_gait_command_name(output: torch.Tensor, descriptor, **kwargs) -> None:
    descriptor.command_name = kwargs.get("gait_command_name", "gait_phase")


def _record_command_aggregate_metadata(output: torch.Tensor, descriptor, **kwargs) -> None:
    descriptor.command_term_names = {
        "locomotion": kwargs.get("locomotion_command_name", "locomotion_commands"),
    }
    descriptor.command_scales = _to_descriptor_list(kwargs.get("scales"))


def ee_pos_in_base_frame(env, gripper_cfg: SceneEntityCfg) -> torch.Tensor:
    """Computes the position of the end-effector in the base frame."""
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get end-effector position in world frame
    ee_pos_w = get_body_pos_w(env, gripper_cfg)

    # 3. Transform ee position to robot base frame
    ee_pos_b = transform_points(ee_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)

    return ee_pos_b.view(env.num_envs, -1)


def ee_goal_pos_in_base_frame(env, command_name: str) -> torch.Tensor:
    """Computes the target position command in the base frame (Cartesian)."""
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get target position command
    target_pos_w = get_command_pos_w(env, command_name)

    # 3. Transform target position to robot base frame
    target_pos_b = transform_points(target_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)
    
    return target_pos_b.view(env.num_envs, -1)


def ee_goal_pos_in_sphere(env, command_name: str) -> torch.Tensor:
    """Computes the target position command in spherical coordinates."""
    command_term = env.command_manager.get_term(command_name)
    # The command is already in spherical coordinates [r, p, y]
    return command_term.command[:, :3].view(env.num_envs, -1)


def ee_pos_in_sphere(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """EE position in yaw-only, sphere-centered coordinates.

    Frame:
      - Origin: sphere center in world (base xy + yaw-rotated offset, fixed z from command cfg)
      - Axes: aligned with base yaw only (roll/pitch ignored)
    """
    if gripper_cfg is None:
        gripper_cfg = SceneEntityCfg("robot", body_names="gripperMover")

    ee_pos_w = get_body_pos_w(env, gripper_cfg)
    base_yaw_quat = get_base_yaw_quat(env)
    sphere_center_w, _ = resolve_sphere_center_w(
        env, command_name="target_position_commands", base_yaw_quat=base_yaw_quat
    )
    ee_rel_w = ee_pos_w - sphere_center_w
    ee_rel_b = world_to_base_yaw(ee_rel_w, base_yaw_quat)
    return cartesian_to_spherical(ee_rel_b)


def ee_goal_pos_in_sphere_radius(env, command_name: str) -> torch.Tensor:
    """Computes the radius component of the target position command in spherical coordinates."""
    command_term = env.command_manager.get_term(command_name)
    # The command is already in spherical coordinates [r, p, y]
    return command_term.command[:, 0].view(env.num_envs, -1)


def ee_goal_orient_in_sphere_pitch(env, command_name: str) -> torch.Tensor:
    """Computes the pitch component of the target orientation command in spherical coordinates."""
    command_term = env.command_manager.get_term(command_name)
    # The command is already in spherical coordinates [r, p, y]
    return command_term.command[:, 1].view(env.num_envs, -1)


def ee_goal_orient_in_sphere_yaw(env, command_name: str) -> torch.Tensor:
    """Computes the yaw component of the target orientation command in spherical coordinates."""
    command_term = env.command_manager.get_term(command_name)
    # The command is already in spherical coordinates [r, p, y]
    return command_term.command[:, 2].view(env.num_envs, -1)


def ee_tracking_error_in_base_frame(env, command_name: str, gripper_cfg: SceneEntityCfg) -> torch.Tensor:
    """Computes the tracking error of the end-effector in the base frame.

    The error is the difference between the target position command and the current
    end-effector position, both expressed in the robot's base frame.
    """
    # 1. Get robot base pose in world frame
    robot_base_pos_w = env.scene["robot"].data.root_pos_w
    robot_base_quat_w = env.scene["robot"].data.root_quat_w

    # 2. Get target position command - correct way to access command data
    target_pos_w = get_command_pos_w(env, command_name)

    # 3. Get end-effector position in world frame
    # We need to find the body index for the gripper
    ee_pos_w = get_body_pos_w(env, gripper_cfg)

    # 4. Transform target and ee positions to robot base frame
    # Use Isaac Lab's transform_points utility which handles coordinate transformations
    # Note: transform_points transforms FROM world frame TO base frame using inverse transform
    target_pos_b = transform_points(target_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)
    ee_pos_b = transform_points(ee_pos_w.unsqueeze(1), robot_base_pos_w, robot_base_quat_w).squeeze(1)

    # 5. Compute tracking error
    tracking_error = target_pos_b - ee_pos_b
    return tracking_error.view(env.num_envs, -1)


@generic_io_descriptor(
    observation_type="RootState",
    units="unit",
    axes=["projected_gravity_x", "projected_gravity_y"],
    on_inspect=[record_shape, record_dtype],
)
def base_orientation_2d(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """
    Get the 2D orientation of the base (roll and pitch from projected gravity).
    
    Args:
        env: The manager-based RL environment instance.
        asset_cfg: Configuration for the robot asset.
        
    Returns:
        Tensor of shape (num_envs, 2) containing x and y components of
        projected gravity.
    """
    # Get the robot asset
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Calculate projected gravity (roll and pitch information)
    # Get the rotation matrix from the robot's orientation
    quat_w = asset.data.root_quat_w
    
    # Transform gravity to body frame using quaternion
    gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=env.device)
    gravity_vec = gravity_vec.expand(env.num_envs, -1)
    
    # Use quat_apply to transform gravity vector to body frame
    proj_grav = quat_apply(quat_inv(quat_w), gravity_vec)
    
    return proj_grav[..., :2]


def foot_contact_binary(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """
    Get binary foot contact information based on contact force threshold.
    
    Args:
        env: The manager-based RL environment instance.
        sensor_cfg: Configuration for the contact sensor.
        threshold: Force threshold for determining contact.
        
    Returns:
        Tensor containing binary contact states for each foot.
    """
    # Get contact forces from the sensor
    contact_forces = env.scene[sensor_cfg.name].data.net_forces_w
    
    # Calculate the norm of contact forces and check against threshold
    contact_binary = (torch.norm(contact_forces, dim=-1) > threshold).float()
    
    return contact_binary


def ee_goal_orient_placeholder(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Placeholder for end-effector goal orientation (hardcoded to zeros).
    
    Args:
        env: The manager-based RL environment instance.
        
    Returns:
        Tensor of shape (num_envs, 3) filled with zeros.
    """
    return torch.zeros(env.num_envs, 3, device=env.device)


# ========== Privileged Observations ==========

def privileged_mass_params(env: ManagerBasedRLEnv) -> torch.Tensor:
    """返回缓存的质量随机化参数 (5维)。
    
    Returns:
        形状为 (num_envs, 5) 的张量，包含：
        - [0] base_mass_delta: 添加到基座的质量 (kg)
        - [1] com_x: 质心X轴偏移 (m)
        - [2] com_y: 质心Y轴偏移 (m)
        - [3] com_z: 质心Z轴偏移 (m)
        - [4] gripper_mass_delta: 添加到夹爪的质量 (kg)
    
    说明：这些值由 cache_base_mass_randomization 等事件函数按随机化分布采样并缓存，
    Isaac Lab 当前未提供从引擎回读已应用值的接口。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    return env._privileged_mass_params


def privileged_friction(env: ManagerBasedRLEnv) -> torch.Tensor:
    """返回缓存的摩擦随机化参数 (1维)。
    
    Returns:
        形状为 (num_envs, 1) 的张量，包含静摩擦系数值。
    
    说明：这个值由 cache_friction_randomization 事件函数按随机化分布采样并缓存。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_friction'):
        env._privileged_friction = torch.ones(env.num_envs, 1, device=env.device)
    
    return env._privileged_friction


def privileged_motor_strengths(env: ManagerBasedRLEnv) -> torch.Tensor:
    """从物理引擎读取随机化的电机刚度 (12维)。

    Returns:
        形状为 (num_envs, 12) 的张量，包含12个腿部关节的实际刚度值。
        关节顺序与配置中 randomize_leg_joint_gains 的 joint_names 一致。

    说明：这些值由 cache_motor_strength_randomization 事件函数从物理引擎读取。
    """
    # 初始化缓存
    if not hasattr(env, '_privileged_motor_strengths'):
        env._privileged_motor_strengths = torch.ones(env.num_envs, 12, device=env.device)
    
    return env._privileged_motor_strengths


# ========== Gait Clock ==========

def _get_gait_term_and_phase(env: ManagerBasedRLEnv, gait_command_name: str):
    """Return gait command term and its scalar phase.

    The gait phase command is expected to output a tensor of shape (num_envs, 1) where the first
    column is the phase in [0, 1).
    """
    gait_term = env.command_manager.get_term(gait_command_name)
    gait_cmd = gait_term.command
    if gait_cmd.ndim == 2 and gait_cmd.shape[1] >= 1:
        return gait_term, gait_cmd[:, 0]
    if gait_cmd.numel() == env.num_envs:
        return gait_term, gait_cmd.view(env.num_envs)
    raise ValueError(
        f"Gait command '{gait_command_name}' must output shape (num_envs, 1) or (num_envs,), got {tuple(gait_cmd.shape)}."
    )


def gait_clock_signal_per_foot(
    env: ManagerBasedRLEnv,
    gait_command_name: str = "gait_phase",
) -> torch.Tensor:
    """
    计算步态时钟信号（4维，每只脚一个正弦波）
    """
    # 1. 获取统一更新后的相位
    gait_term, gait_phase = _get_gait_term_and_phase(env, gait_command_name)
    phase_offset = float(getattr(gait_term.cfg, "phase_offset", 0.5))
    locomotion_command_name = getattr(gait_term.cfg, "locomotion_command_name", "locomotion_commands")
    
    # 2. 为四只脚分配相位（小跑步态：对角足同相）
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    
    # FL (左前) 和 RR (右后) 是一对对角足 - 相位 + offset
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    
    # FR (右前) 和 RL (左后) 是另一对对角足 - 相位 + 0
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL
    
    # 3. 转换为正弦信号
    clock_signals = torch.sin(2 * torch.pi * foot_phases)
    
    # 4. 处理站立状态输出
    # 虽然相位已重置为0，但为了明确物理含义，站立时输出 -1.0 (支撑相)
    try:
        locomotion_command_term = env.command_manager.get_term(locomotion_command_name)
        is_standing = locomotion_command_term.is_standing_env
    except (AttributeError, KeyError):
        is_standing = torch.zeros((env.num_envs,), device=env.device, dtype=torch.bool)
    
    standing_signal = torch.full((env.num_envs, 4), -1.0, device=env.device)
    
    final_signals = torch.where(
        is_standing.unsqueeze(1),
        standing_signal,
        clock_signals
    )
    
    return final_signals


@generic_io_descriptor(
    observation_type="Command",
    units="unit",
    axes=["sin_phase", "cos_phase"],
    on_inspect=[record_shape, record_dtype, _record_gait_command_name],
)
def gait_clock_signal(
    env: ManagerBasedRLEnv,
    gait_command_name: str = "gait_phase",
) -> torch.Tensor:
    """
    计算全局步态时钟信号（2维：sin, cos）
    """
    # 1. 获取统一更新后的相位
    _, gait_phase = _get_gait_term_and_phase(env, gait_command_name)
    
    # 2. 计算 2维信号: [sin(2πφ), cos(2πφ)]
    # 站立时 gait_phase=0 -> [0, 1]
    clock_signals = torch.stack([
        torch.sin(2 * torch.pi * gait_phase),
        torch.cos(2 * torch.pi * gait_phase)
    ], dim=1)
    
    return clock_signals


def stance_mask(
    env: ManagerBasedRLEnv,
    gait_command_name: str = "gait_phase",
) -> torch.Tensor:
    """
    计算步态接触计划（4维，0/1 二值信号）
    
    功能：明确告知哪只脚应该落地（支撑相）。
    基于 gait_clock_signal 的相位逻辑，将连续信号二值化。
    
    Args:
        env: 环境实例
        gait_command_name: 步态相位命令名（CommandManager 中的 term 名称）
        
    Returns:
        Tensor (num_envs, 4):
            - 1.0: 计划支撑相 (Stance / Ground)
            - 0.0: 计划摆动相 (Swing / Air)
    """
    # 1. 获取统一更新后的相位
    gait_term, gait_phase = _get_gait_term_and_phase(env, gait_command_name)
    phase_offset = float(getattr(gait_term.cfg, "phase_offset", 0.5))
    locomotion_command_name = getattr(gait_term.cfg, "locomotion_command_name", "locomotion_commands")
    
    # 2. 为四只脚分配相位
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL
    
    # 3. 计算正弦波
    # 根据当前约定：负值是支撑相，正值是摆动相
    sin_waves = torch.sin(2 * torch.pi * foot_phases)
    
    # 4. 生成接触掩码 (0/1)
    # sin_waves <= 0 表示支撑相 (1.0)
    # sin_waves > 0  表示摆动相 (0.0)
    contact_schedule = (sin_waves <= 0.0).float()
    
    # 5. 处理站立状态
    try:
        locomotion_command_term = env.command_manager.get_term(locomotion_command_name)
        is_standing = locomotion_command_term.is_standing_env
        # 站立时全为支撑相 (1.0)
        contact_schedule[is_standing] = 1.0
    except (AttributeError, KeyError):
        pass
    
    return contact_schedule


def compute_ref_dof(
    env: ManagerBasedRLEnv,
    target_joint_pos_thd: float = 0.1,
    target_joint_pos_scale: float = 0.35,
    calf_scale_factor: float = 2.0,
    gait_command_name: str = "gait_phase",
) -> torch.Tensor:
    """Compute open-loop reference leg joint positions for a trot gait.

    Returns:
        Tensor (num_envs, 12) in joint order:
            [FL_hip, FR_hip, RL_hip, RR_hip,
             FL_thigh, FR_thigh, RL_thigh, RR_thigh,
             FL_calf, FR_calf, RL_calf, RR_calf]
    """
    # Cache joint ids for the 12 leg joints (consistent with env config order).
    if not hasattr(env, "_unifp_leg_joint_ids"):
        leg_joint_names = [
            "FL_hip_joint",
            "FR_hip_joint",
            "RL_hip_joint",
            "RR_hip_joint",
            "FL_thigh_joint",
            "FR_thigh_joint",
            "RL_thigh_joint",
            "RR_thigh_joint",
            "FL_calf_joint",
            "FR_calf_joint",
            "RL_calf_joint",
            "RR_calf_joint",
        ]
        env._unifp_leg_joint_ids = env.scene["robot"].find_joints(leg_joint_names)[0]

    # Avoid recomputing multiple times within the same simulation step.
    if not hasattr(env, "_unifp_ref_dof_last_update_step"):
        env._unifp_ref_dof_last_update_step = -1
    gait_term = env.command_manager.get_term(gait_command_name)
    phase_offset = float(getattr(gait_term.cfg, "phase_offset", 0.5))
    locomotion_command_name = getattr(gait_term.cfg, "locomotion_command_name", "locomotion_commands")
    current_params = (
        float(phase_offset),
        float(target_joint_pos_thd),
        float(target_joint_pos_scale),
        float(calf_scale_factor),
        gait_command_name,
        locomotion_command_name,
    )
    if (
        getattr(env, "_unifp_ref_dof_last_update_step") == env.common_step_counter
        and getattr(env, "_unifp_ref_dof_last_params", None) == current_params
        and hasattr(env, "_unifp_ref_dof_pos")
    ):
        return env._unifp_ref_dof_pos

    robot: Articulation = env.scene["robot"]
    leg_default_pos = robot.data.default_joint_pos[:, env._unifp_leg_joint_ids]
    ref_dof_pos = leg_default_pos.clone()

    # Build per-foot phases for trot (FL/RR shifted by phase_offset).
    _, gait_phase = _get_gait_term_and_phase(env, gait_command_name)
    foot_phases = torch.zeros(env.num_envs, 4, device=env.device)
    foot_phases[:, 0] = (gait_phase + phase_offset) % 1.0  # FL
    foot_phases[:, 3] = (gait_phase + phase_offset) % 1.0  # RR
    foot_phases[:, 1] = gait_phase  # FR
    foot_phases[:, 2] = gait_phase  # RL

    # Convention in this repo: sin > 0 is swing, sin <= 0 is stance.
    sin_waves = torch.sin(2 * torch.pi * foot_phases)

    # Half-wave rectification with a threshold to tune duty factor:
    # lift in [0, 1], only active during (biased) swing.
    thd = float(target_joint_pos_thd)
    thd = max(-0.99, min(thd, 0.99))
    denom = 1.0 - thd
    lift = torch.clamp(sin_waves - thd, min=0.0) / denom

    # Standing: freeze to default pose (no swing).
    try:
        locomotion_command_term = env.command_manager.get_term(locomotion_command_name)
        is_standing = locomotion_command_term.is_standing_env
        lift[is_standing] = 0.0
    except (KeyError, AttributeError):
        pass

    thigh_amp = float(target_joint_pos_scale)
    calf_amp = float(target_joint_pos_scale) * float(calf_scale_factor)

    # AliengoZ1 URDF uses the same joint axis direction for thigh/calf across legs,
    # so we apply identical sign conventions for all feet.
    ref_dof_pos[:, 4:8] = ref_dof_pos[:, 4:8] + lift * thigh_amp
    ref_dof_pos[:, 8:12] = ref_dof_pos[:, 8:12] - lift * calf_amp

    env._unifp_ref_dof_pos = ref_dof_pos
    env._unifp_ref_dof_last_update_step = env.common_step_counter
    env._unifp_ref_dof_last_params = current_params
    return ref_dof_pos


def ref_dof_pos_error(env: ManagerBasedRLEnv, gait_command_name: str, target_joint_pos_thd: float, target_joint_pos_scale: float, calf_scale_factor: float) -> torch.Tensor:
    """Returns reference minus current for the 12 leg joints.

    This is typically used as a privileged observation to expose the tracking error
    against an open-loop gait reference in joint space.
    """
    ref = compute_ref_dof(
        env,
        target_joint_pos_thd=target_joint_pos_thd,
        target_joint_pos_scale=target_joint_pos_scale,
        calf_scale_factor=calf_scale_factor,
        gait_command_name=gait_command_name,
    )
    robot: Articulation = env.scene["robot"]
    current = robot.data.joint_pos[:, env._unifp_leg_joint_ids]
    return ref - current


@generic_io_descriptor(
    observation_type="Command",
    units="mixed",
    axes=[
        "base_lin_vel_x",
        "base_lin_vel_y",
        "base_ang_vel_z",
    ],
    on_inspect=[record_shape, record_dtype, _record_command_aggregate_metadata],
)
def locomotion_commands_with_scales(
    env: ManagerBasedRLEnv,
    scales: Sequence[float] | torch.Tensor,
    locomotion_command_name: str = "locomotion_commands",
) -> torch.Tensor:
    """Return scaled locomotion commands only."""
    locomotion_cmd = env.command_manager.get_command(locomotion_command_name)
    locomotion_cmd = locomotion_cmd[:, :3] if locomotion_cmd.shape[1] >= 3 else torch.nn.functional.pad(
        locomotion_cmd, (0, 3 - locomotion_cmd.shape[1])
    )

    if isinstance(scales, (tuple, list)):
        scales = torch.tensor(scales, device=env.device)
    elif isinstance(scales, torch.Tensor):
        scales = scales.to(env.device)

    return locomotion_cmd * scales


def ee_goal_offset_local_sphere(
    env: ManagerBasedRLEnv,
    ee_goal_command_name: str = "target_position_commands",
    gripper_force_ext_name: str = "gripper_force_ext",
    gripper_force_command_name: str = "gripper_force_command",
    gripper_impedance_command_name: str = "gripper_impedance_command",
) -> torch.Tensor:
    """EE goal offset (force/Kp) expressed in yaw-only, sphere-centered spherical coords.

    Steps: goal -> world, add force offset in world, subtract sphere center (fixed z),
    rotate by inverse base yaw, then Cartesian -> spherical.
    """
    # 1-3: Read forces and move the commanded force into world frame (yaw-only base frame).
    base_yaw_quat = get_base_yaw_quat(env)

    gripper_force_ext_w = _get_force_command(env, gripper_force_ext_name, use_applied=True)
    gripper_force_cmd_l = env.command_manager.get_command(gripper_force_command_name)
    gripper_force_cmd_w = base_yaw_to_world(gripper_force_cmd_l, base_yaw_quat)

    # 4: Combine external and commanded forces in world frame.
    forces_offset_w = gripper_force_cmd_w + gripper_force_ext_w

    # 5: Current EE goal in world frame (Cartesian).
    command_term = env.command_manager.get_term(ee_goal_command_name)
    sphere_center_w, has_sphere_center = resolve_sphere_center_w(
        env, command_term=command_term, base_yaw_quat=base_yaw_quat
    )
    if hasattr(command_term, "pos_w"):
        curr_goal_w = command_term.pos_w
    elif has_sphere_center:
        curr_goal_cmd = command_term.command[:, :3]
        curr_goal_b = spherical_to_cartesian(curr_goal_cmd)
        curr_goal_w = base_yaw_to_world(curr_goal_b, base_yaw_quat) + sphere_center_w
    else:
        curr_goal_w = command_term.command[:, :3]

    # 7: Offset the goal by virtual impedance (force / kp) in world frame.
    gripper_impedance_cmd = env.command_manager.get_command(gripper_impedance_command_name)
    kps = torch.clamp(gripper_impedance_cmd[:, 0], min=1e-6)
    curr_goal_w_offset = curr_goal_w + forces_offset_w / kps.unsqueeze(-1)

    # 8-9: World -> base-yaw frame, then Cartesian -> spherical.
    goal_rel_w = curr_goal_w_offset - sphere_center_w
    goal_rel_b = world_to_base_yaw(goal_rel_w, base_yaw_quat)

    return cartesian_to_spherical(goal_rel_b)


def base_lin_vel_offset(
    env: ManagerBasedRLEnv,
    locomotion_command_name: str = "locomotion_commands",
    base_force_ext_name: str = "base_force_ext",
    base_force_kd: float = 100.0,
) -> torch.Tensor:
    """Compute the force-modulated base XY velocity target in the base yaw frame."""
    base_yaw_quat = get_base_yaw_quat(env)

    base_force_ext_w = env.command_manager.get_command(base_force_ext_name)
    base_force_ext_xy = world_to_base_yaw(base_force_ext_w, base_yaw_quat)[:, :2]
    base_kd = max(float(base_force_kd), 1e-6)

    locomotion_cmd = env.command_manager.get_command(locomotion_command_name)
    base_lin_vel_cmd_xy = torch.zeros((env.num_envs, 2), device=env.device)
    if locomotion_cmd.shape[1] >= 2:
        base_lin_vel_cmd_xy[:, :2] = locomotion_cmd[:, :2]
    elif locomotion_cmd.shape[1] == 1:
        base_lin_vel_cmd_xy[:, 0] = locomotion_cmd[:, 0]

    return base_lin_vel_cmd_xy + base_force_ext_xy / base_kd
