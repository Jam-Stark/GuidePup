# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Event callbacks to cache privileged observations during domain randomization."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def cache_base_mass_randomization(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    mass_distribution_params: tuple[float, float],
    operation: str,
    distribution: str = "uniform",
):
    """Cache the randomized base mass delta after randomization event.
    
    This should be called AFTER mdp.randomize_rigid_body_mass to capture the
    randomized mass values.
    
    Args:
        env: The environment instance.
        env_ids: Environment indices to update. If None, applies to all environments.
        asset_cfg: Asset configuration.
        mass_distribution_params: Distribution parameters for mass randomization.
        operation: Operation type ("add", "scale", "abs").
        distribution: Distribution type ("uniform", "gaussian").
    """
    # Resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Initialize if not exists
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    # Sample mass delta and cache it
    if distribution == "uniform":
        mass_delta = torch.empty(len(env_ids), device=env.device).uniform_(*mass_distribution_params)
    elif distribution == "gaussian":
        mass_delta = torch.empty(len(env_ids), device=env.device).normal_(*mass_distribution_params)
    else:
        raise ValueError(f"Unsupported distribution: {distribution}")
    
    # Store base mass delta in column 0
    env._privileged_mass_params[env_ids, 0] = mass_delta


def cache_base_com_randomization(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    com_range: dict,
    operation: str = "add",
):
    """Cache the randomized base center of mass offset after randomization event.
    
    This should be called AFTER mdp.randomize_rigid_body_com to capture the
    randomized COM values.
    
    Args:
        env: The environment instance.
        env_ids: Environment indices to update. If None, applies to all environments.
        asset_cfg: Asset configuration.
        com_range: Distribution parameters with 'x', 'y', 'z' keys.
        operation: Operation type ("add", "scale", "abs").
    """
    # Resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Initialize if not exists
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    # Sample COM offsets and cache them
    com_x = torch.empty(len(env_ids), device=env.device).uniform_(*com_range["x"])
    com_y = torch.empty(len(env_ids), device=env.device).uniform_(*com_range["y"])
    com_z = torch.empty(len(env_ids), device=env.device).uniform_(*com_range["z"])
    
    # Store COM offsets in columns 1, 2, 3
    env._privileged_mass_params[env_ids, 1] = com_x
    env._privileged_mass_params[env_ids, 2] = com_y
    env._privileged_mass_params[env_ids, 3] = com_z


def cache_gripper_mass_randomization(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    mass_distribution_params: tuple[float, float],
    operation: str,
    distribution: str = "uniform",
):
    """Cache the randomized gripper mass delta after randomization event.
    
    This should be called AFTER mdp.randomize_rigid_body_mass to capture the
    randomized mass values.
    
    Args:
        env: The environment instance.
        env_ids: Environment indices to update. If None, applies to all environments.
        asset_cfg: Asset configuration.
        mass_distribution_params: Distribution parameters for mass randomization.
        operation: Operation type ("add", "scale", "abs").
        distribution: Distribution type ("uniform", "gaussian").
    """
    # Resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Initialize if not exists
    if not hasattr(env, '_privileged_mass_params'):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
    
    # Sample mass delta and cache it
    if distribution == "uniform":
        mass_delta = torch.empty(len(env_ids), device=env.device).uniform_(*mass_distribution_params)
    elif distribution == "gaussian":
        mass_delta = torch.empty(len(env_ids), device=env.device).normal_(*mass_distribution_params)
    else:
        raise ValueError(f"Unsupported distribution: {distribution}")
    
    # Store gripper mass delta in column 4
    env._privileged_mass_params[env_ids, 4] = mass_delta


def cache_friction_randomization(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    static_friction_range: tuple[float, float],
    dynamic_friction_range: tuple[float, float],
    restitution_range: tuple[float, float],
    num_buckets: int = 250,
):
    """Cache the randomized friction coefficient after randomization event.
    
    This should be called AFTER mdp.randomize_rigid_body_material to capture the
    randomized friction values.
    
    Args:
        env: The environment instance.
        env_ids: Environment indices to update. If None, applies to all environments.
        asset_cfg: Asset configuration.
        static_friction_range: Range for static friction.
        dynamic_friction_range: Range for dynamic friction.
        restitution_range: Range for restitution.
        num_buckets: Number of discretization buckets.
    """
    # Resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Initialize if not exists
    if not hasattr(env, '_privileged_friction'):
        env._privileged_friction = torch.ones(env.num_envs, 1, device=env.device)
    
    # Sample static friction uniformly from range
    static_friction = torch.empty(len(env_ids), device=env.device).uniform_(*static_friction_range)
    
    # Store static friction
    env._privileged_friction[env_ids, 0] = static_friction


def cache_motor_strength_randomization(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    stiffness_distribution_params: tuple[float, float],
    damping_distribution_params: tuple[float, float],
    operation: str = "scale",
    distribution: str = "uniform",
):
    """Cache the randomized motor strength scaling factors after randomization event.
    
    This should be called AFTER mdp.randomize_actuator_gains to capture the
    randomized gain values.
    
    Args:
        env: The environment instance.
        env_ids: Environment indices to update. If None, applies to all environments.
        asset_cfg: Asset configuration with joint names.
        stiffness_distribution_params: Distribution parameters for stiffness scaling.
        damping_distribution_params: Distribution parameters for damping scaling.
        operation: Operation type ("add", "scale", "abs").
        distribution: Distribution type ("uniform", "log_uniform", "gaussian").
    """
    # Resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    
    # Initialize if not exists
    if not hasattr(env, '_privileged_motor_strengths'):
        env._privileged_motor_strengths = torch.ones(env.num_envs, 12, device=env.device)
    
    # Sample stiffness scaling factors
    if distribution == "uniform":
        stiffness_scale = torch.empty(len(env_ids), 12, device=env.device).uniform_(*stiffness_distribution_params)
    elif distribution == "log_uniform":
        # Log-uniform sampling
        log_min = torch.log(torch.tensor(stiffness_distribution_params[0], device=env.device))
        log_max = torch.log(torch.tensor(stiffness_distribution_params[1], device=env.device))
        stiffness_scale = torch.exp(
            torch.empty(len(env_ids), 12, device=env.device).uniform_(log_min.item(), log_max.item())
        )
    elif distribution == "gaussian":
        stiffness_scale = torch.empty(len(env_ids), 12, device=env.device).normal_(*stiffness_distribution_params)
    else:
        raise ValueError(f"Unsupported distribution: {distribution}")
    
    # Store stiffness scaling factors
    env._privileged_motor_strengths[env_ids] = stiffness_scale
