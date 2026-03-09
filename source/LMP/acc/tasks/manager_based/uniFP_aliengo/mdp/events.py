# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to enable different events.

Events include anything related to altering the simulation state. This includes changing the physics
materials, applying external forces, and resetting the state of the asset.

The functions can be passed to the :class:`isaaclab.managers.EventTermCfg` object to enable
the event introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.actuators import ImplicitActuator
from isaaclab.assets import Articulation, DeformableObject, RigidObject
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv




def reset_gait_state(env: ManagerBasedEnv, env_ids: torch.Tensor | slice | None):
    """Reset cached gait phase buffers for the specified environments.

    This is used to keep episode-local open-loop gait clocks consistent across resets. In particular, the
    per-environment ``gait_last_update_step`` is set to the current ``common_step_counter`` so that the gait
    phase does not advance again within the same env-step in which the reset happens.
    """
    # Lazy-init buffers (kept on env to share across MDP terms).
    if not hasattr(env, "gait_indices") or not torch.is_tensor(getattr(env, "gait_indices")):
        env.gait_indices = torch.zeros(env.num_envs, device=env.device)
    if not hasattr(env, "gait_last_update_step") or not torch.is_tensor(getattr(env, "gait_last_update_step")):
        env.gait_last_update_step = torch.full((env.num_envs,), -1, device=env.device, dtype=torch.long)
    else:
        # Ensure dtype/device/shape are as expected.
        env.gait_last_update_step = env.gait_last_update_step.to(device=env.device, dtype=torch.long)
        if env.gait_last_update_step.shape != (env.num_envs,):
            fill_value = int(env.gait_last_update_step.flatten()[0].item()) if env.gait_last_update_step.numel() > 0 else -1
            env.gait_last_update_step = torch.full((env.num_envs,), fill_value, device=env.device, dtype=torch.long)

    current_step = int(getattr(env, "common_step_counter", 0))

    # Resolve environment ids.
    if env_ids is None or env_ids == slice(None):
        env.gait_indices.zero_()
        env.gait_last_update_step[:] = current_step
        return
    if not torch.is_tensor(env_ids):
        env_ids = torch.tensor(env_ids, device=env.device, dtype=torch.long)
    else:
        env_ids = env_ids.to(device=env.device)

    env.gait_indices[env_ids] = 0.0
    env.gait_last_update_step[env_ids] = current_step


def push_by_setting_velocity_standInclude(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | slice | None,
    velocity_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    command_name: str = "locomotion_commands",
    standing_velocity_scale: float = 1.0,
):
    """Push asset by setting its root velocity with optional standing scaling."""
    asset: Articulation | RigidObject = env.scene[asset_cfg.name]

    if env_ids is None or env_ids == slice(None):
        env_ids_tensor = torch.arange(env.num_envs, device=asset.device)
        env_ids_write = None
    else:
        if not torch.is_tensor(env_ids):
            env_ids_tensor = torch.tensor(env_ids, device=asset.device, dtype=torch.long)
        else:
            env_ids_tensor = env_ids.to(device=asset.device)
        env_ids_write = env_ids_tensor

    if env_ids_tensor.numel() == 0:
        return

    if hasattr(asset.data, "root_lin_vel_w") and hasattr(asset.data, "root_ang_vel_w"):
        root_vel_w = torch.cat((asset.data.root_lin_vel_w, asset.data.root_ang_vel_w), dim=-1)
    elif hasattr(asset.data, "root_vel_w"):
        root_vel_w = asset.data.root_vel_w
    else:
        raise AttributeError(f"Asset '{asset_cfg.name}' does not expose root velocity buffers.")

    env_root_vel = root_vel_w[env_ids_tensor].clone()

    standing_mask = None
    standing_velocity_scale = float(standing_velocity_scale)
    if standing_velocity_scale != 1.0 and command_name:
        try:
            command_term = env.command_manager.get_term(command_name)
            if hasattr(command_term, "is_standing_env"):
                standing_mask = command_term.is_standing_env.to(device=asset.device, dtype=torch.bool)
            else:
                command = env.command_manager.get_command(command_name)
                if command is not None:
                    command = command.to(device=asset.device)
                    if command.ndim == 2:
                        standing_mask = (command.abs() <= 1e-6).all(dim=-1)
        except KeyError:
            standing_mask = None

    if standing_mask is not None:
        if env_ids_write is not None:
            standing_mask = standing_mask[env_ids_tensor]
        if standing_mask.numel() != env_ids_tensor.numel():
            standing_mask = None

    scale = None
    if standing_mask is not None and standing_velocity_scale != 1.0:
        scale = torch.ones(env_ids_tensor.numel(), device=asset.device)
        if standing_mask.any():
            scale[standing_mask] = standing_velocity_scale

    vel_indices = {"x": 0, "y": 1, "z": 2, "roll": 3, "pitch": 4, "yaw": 5}
    for key, idx in vel_indices.items():
        if key not in velocity_range:
            continue
        low, high = velocity_range[key]
        samples = torch.empty(env_ids_tensor.numel(), device=asset.device).uniform_(low, high)
        if scale is not None:
            samples *= scale
        env_root_vel[:, idx] = samples

    asset.write_root_velocity_to_sim(env_root_vel, env_ids=env_ids_write)


class my_randomize_rigid_body_material(ManagerTermBase):
    """Randomize the physics materials on all geometries of the asset.

    This function creates a set of physics materials with random static friction, dynamic friction, and restitution
    values. The number of materials is specified by ``num_buckets``. The materials are generated by sampling
    uniform random values from the given ranges.

    The material properties are then assigned to the geometries of the asset. The assignment is done by
    creating a random integer tensor of shape  (num_instances, max_num_shapes) where ``num_instances``
    is the number of assets spawned and ``max_num_shapes`` is the maximum number of shapes in the asset (over
    all bodies). The integer values are used as indices to select the material properties from the
    material buckets.

    If the flag ``make_consistent`` is set to ``True``, the dynamic friction is set to be less than or equal to
    the static friction. This obeys the physics constraint on friction values. However, it may not always be
    essential for the application. Thus, the flag is set to ``False`` by default.
    
    This class also caches the applied friction values to env._privileged_friction.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        """Initialize the term.

        Args:
            cfg: The configuration of the event term.
            env: The environment instance.

        Raises:
            ValueError: If the asset is not a RigidObject or an Articulation.
        """
        super().__init__(cfg, env)

        # extract the used quantities (to enable type-hinting)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: RigidObject | Articulation = env.scene[self.asset_cfg.name]

        if not isinstance(self.asset, (RigidObject, Articulation)):
            raise ValueError(
                f"Randomization term 'randomize_rigid_body_material' not supported for asset: '{self.asset_cfg.name}'"
                f" with type: '{type(self.asset)}'."
            )

        # obtain number of shapes per body (needed for indexing the material properties correctly)
        if isinstance(self.asset, Articulation) and self.asset_cfg.body_ids != slice(None):
            self.num_shapes_per_body = []
            for link_path in self.asset.root_physx_view.link_paths[0]:
                link_physx_view = self.asset._physics_sim_view.create_rigid_body_view(link_path)
                self.num_shapes_per_body.append(link_physx_view.max_shapes)
            # ensure the parsing is correct
            num_shapes = sum(self.num_shapes_per_body)
            expected_shapes = self.asset.root_physx_view.max_shapes
            if num_shapes != expected_shapes:
                raise ValueError(
                    "Randomization term 'randomize_rigid_body_material' failed to parse the number of shapes per body."
                    f" Expected total shapes: {expected_shapes}, but got: {num_shapes}."
                )
        else:
            # in this case, we don't need to do special indexing
            self.num_shapes_per_body = None

        # obtain parameters for sampling friction and restitution values
        static_friction_range = cfg.params.get("static_friction_range", (1.0, 1.0))
        dynamic_friction_range = cfg.params.get("dynamic_friction_range", (1.0, 1.0))
        restitution_range = cfg.params.get("restitution_range", (0.0, 0.0))
        num_buckets = int(cfg.params.get("num_buckets", 1))

        # sample material properties from the given ranges
        range_list = [static_friction_range, dynamic_friction_range, restitution_range]
        ranges = torch.tensor(range_list, device="cpu")
        self.material_buckets = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], (num_buckets, 3), device="cpu")

        # ensure dynamic friction is always less than static friction
        make_consistent = cfg.params.get("make_consistent", False)
        if make_consistent:
            self.material_buckets[:, 1] = torch.min(self.material_buckets[:, 0], self.material_buckets[:, 1])

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        static_friction_range: tuple[float, float],
        dynamic_friction_range: tuple[float, float],
        restitution_range: tuple[float, float],
        num_buckets: int,
        asset_cfg: SceneEntityCfg,
        make_consistent: bool = False,
    ):
        # resolve environment ids
        if env_ids is None:
            env_ids = torch.arange(env.scene.num_envs, device="cpu")
        else:
            env_ids = env_ids.cpu()

        # randomly assign material IDs to the geometries
        total_num_shapes = self.asset.root_physx_view.max_shapes
        bucket_ids = torch.randint(0, num_buckets, (len(env_ids), total_num_shapes), device="cpu")
        material_samples = self.material_buckets[bucket_ids]

        # retrieve material buffer from the physics simulation
        materials = self.asset.root_physx_view.get_material_properties()

        # update material buffer with new samples
        if self.num_shapes_per_body is not None:
            # sample material properties from the given ranges
            for body_id in self.asset_cfg.body_ids:
                # obtain indices of shapes for the body
                start_idx = sum(self.num_shapes_per_body[:body_id])
                end_idx = start_idx + self.num_shapes_per_body[body_id]
                # assign the new materials
                materials[env_ids, start_idx:end_idx] = material_samples[:, start_idx:end_idx]
        else:
            # assign all the materials
            materials[env_ids] = material_samples[:]

        # apply to simulation
        self.asset.root_physx_view.set_material_properties(materials, env_ids)
        
        # --- Cache to env ---
        if not hasattr(env, "_privileged_friction"):
            env._privileged_friction = torch.ones(env.num_envs, 1, device=env.device)
            
        # Calculate mean static friction for each environment
        # material_samples: (len(env_ids), total_num_shapes, 3)
        # We want the mean of static friction (index 0) across all shapes (dim 1)
        mean_static_friction = material_samples[..., 0].mean(dim=1).to(env.device)
        
        # Update privileged friction for the affected envs
        env_ids_device = env_ids.to(env.device)
        env._privileged_friction[env_ids_device, 0] = mean_static_friction


def _validate_scale_range(params, name, allow_zero=False):
    if not isinstance(params, (tuple, list)) or len(params) != 2:
        raise TypeError(f"'{name}' must be a tuple of two numbers.")
    if params[0] > params[1]:
        raise ValueError(f"'{name}' lower bound must be less than or equal to upper bound.")
    if not allow_zero and params[0] <= 0:
        raise ValueError(f"'{name}' lower bound must be positive.")


def _randomize_prop_by_op(data, params, dim_0_ids, dim_1_ids, operation, distribution):
    if dim_0_ids is None:
        dim_0_ids = slice(None)
    if dim_1_ids is None:
        dim_1_ids = slice(None)
        
    target = data[dim_0_ids, dim_1_ids]
    low, high = params
    
    if distribution == "uniform":
        noise = torch.rand_like(target) * (high - low) + low
    elif distribution == "log_uniform":
        noise = torch.exp(torch.rand_like(target) * (torch.log(torch.tensor(high)) - torch.log(torch.tensor(low))) + torch.log(torch.tensor(low)))
    else:
        # Fallback to uniform if unknown
        noise = torch.rand_like(target) * (high - low) + low
        
    if operation == "add":
        target += noise
    elif operation == "scale":
        target *= noise
    elif operation == "abs":
        target = noise
        
    data[dim_0_ids, dim_1_ids] = target
    return data


class my_randomize_actuator_gains(ManagerTermBase):
    """Randomize the actuator gains in an articulation by adding, scaling, or setting random values.
    
    This class also caches the applied motor strengths (stiffness) to env._privileged_motor_strengths.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: RigidObject | Articulation = env.scene[self.asset_cfg.name]
        
        if cfg.params["operation"] == "scale":
            if "stiffness_distribution_params" in cfg.params:
                _validate_scale_range(
                    cfg.params["stiffness_distribution_params"], "stiffness_distribution_params", allow_zero=False
                )
            if "damping_distribution_params" in cfg.params:
                _validate_scale_range(cfg.params["damping_distribution_params"], "damping_distribution_params")
        elif cfg.params["operation"] not in ("abs", "add"):
            raise ValueError(
                "Randomization term 'randomize_actuator_gains' does not support operation:"
                f" '{cfg.params['operation']}'."
            )

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        stiffness_distribution_params: tuple[float, float] | None = None,
        damping_distribution_params: tuple[float, float] | None = None,
        operation: Literal["add", "scale", "abs"] = "abs",
        distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
    ):
        if env_ids is None:
            env_ids = torch.arange(env.scene.num_envs, device=self.asset.device)

        def randomize(data: torch.Tensor, params: tuple[float, float]) -> torch.Tensor:
            return _randomize_prop_by_op(
                data, params, dim_0_ids=None, dim_1_ids=actuator_indices, operation=operation, distribution=distribution
            )

        for actuator in self.asset.actuators.values():
            if isinstance(self.asset_cfg.joint_ids, slice):
                actuator_indices = slice(None)
                if isinstance(actuator.joint_indices, slice):
                    global_indices = slice(None)
                elif isinstance(actuator.joint_indices, torch.Tensor):
                    global_indices = actuator.joint_indices.to(self.asset.device)
                else:
                    raise TypeError("Actuator joint indices must be a slice or a torch.Tensor.")
            elif isinstance(actuator.joint_indices, slice):
                global_indices = actuator_indices = torch.tensor(self.asset_cfg.joint_ids, device=self.asset.device)
            else:
                actuator_joint_indices = actuator.joint_indices
                asset_joint_ids = torch.tensor(self.asset_cfg.joint_ids, device=self.asset.device)
                actuator_indices = torch.nonzero(torch.isin(actuator_joint_indices, asset_joint_ids)).view(-1)
                if len(actuator_indices) == 0:
                    continue
                global_indices = actuator_joint_indices[actuator_indices]
            
            if stiffness_distribution_params is not None:
                stiffness = actuator.stiffness[env_ids].clone()
                stiffness[:, actuator_indices] = self.asset.data.default_joint_stiffness[env_ids][
                    :, global_indices
                ].clone()
                randomize(stiffness, stiffness_distribution_params)
                actuator.stiffness[env_ids] = stiffness
                if isinstance(actuator, ImplicitActuator):
                    self.asset.write_joint_stiffness_to_sim(
                        stiffness, joint_ids=actuator.joint_indices, env_ids=env_ids
                    )
            
            if damping_distribution_params is not None:
                damping = actuator.damping[env_ids].clone()
                damping[:, actuator_indices] = self.asset.data.default_joint_damping[env_ids][:, global_indices].clone()
                randomize(damping, damping_distribution_params)
                actuator.damping[env_ids] = damping
                if isinstance(actuator, ImplicitActuator):
                    self.asset.write_joint_damping_to_sim(damping, joint_ids=actuator.joint_indices, env_ids=env_ids)

        # --- Cache to env ---
        if not hasattr(env, "_privileged_motor_strengths"):
            env._privileged_motor_strengths = torch.ones(env.num_envs, 12, device=env.device)

        joint_ids = asset_cfg.joint_ids
        if isinstance(joint_ids, slice):
            return
        joint_ids_tensor = torch.as_tensor(joint_ids, device=self.asset.device, dtype=torch.long)
        if joint_ids_tensor.numel() != env._privileged_motor_strengths.shape[1]:
            return
        env_ids_device = env_ids.to(env.device)
        actual_stiffness = self.asset.data.joint_stiffness[env_ids][:, joint_ids_tensor].to(env.device)
        env._privileged_motor_strengths[env_ids_device] = actual_stiffness


def my_randomize_rigid_body_com(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    com_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    """Randomize the center of mass (CoM) of rigid bodies by adding a random value sampled from the given ranges.

    .. note::
        This function uses CPU tensors to assign the CoM. It is recommended to use this function
        only during the initialization of the environment.
        
    This function also caches the applied CoM offsets to env._privileged_mass_params.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    # resolve body indices
    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    # sample random CoM values
    range_list = [com_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z"]]
    ranges = torch.tensor(range_list, device="cpu")
    rand_samples = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device="cpu").unsqueeze(1)

    # get the current com of the bodies (num_assets, num_bodies)
    coms = asset.root_physx_view.get_coms().clone()

    # Randomize the com in range
    coms[env_ids[:, None], body_ids, :3] += rand_samples

    # Set the new coms
    asset.root_physx_view.set_coms(coms, env_ids)
    
    # --- Cache to env ---
    if not hasattr(env, "_privileged_mass_params"):
        env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
        
    # rand_samples shape: (len(env_ids), 1, 3)
    # We need to store x, y, z offsets.
    # Assuming we are randomizing the base (or a single body), we take the first body's offset.
    # If multiple bodies are randomized, this caching logic might need adjustment, 
    # but based on cache_base_com_randomization, it seems to expect a single set of offsets per env.
    
    offsets = rand_samples.squeeze(1).to(env.device) # (len(env_ids), 3)
    
    env_ids_device = env_ids.to(env.device)
    env._privileged_mass_params[env_ids_device, 1] = offsets[:, 0] # x
    env._privileged_mass_params[env_ids_device, 2] = offsets[:, 1] # y
    env._privileged_mass_params[env_ids_device, 3] = offsets[:, 2] # z


class my_randomize_rigid_body_mass(ManagerTermBase):
    """随机化刚体质量并缓存采样值。
    
    这个类同时执行质量随机化（逻辑与 Isaac Lab 官方 randomize_rigid_body_mass 一致）
    和缓存采样参数到环境状态中，以便在特权观测中访问。
    
    Args:
        cfg: 事件配置
        env: 环境实例
    """
    
    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        """初始化事件项。"""
        super().__init__(cfg, env)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: RigidObject | Articulation = env.scene[self.asset_cfg.name]
        
        # 存储哪个身体要随机化（用于后续缓存索引）
        body_names = self.asset_cfg.body_names
        if isinstance(body_names, str):
            body_names = [body_names]
        self.body_names = body_names
        
        if not isinstance(self.asset, (RigidObject, Articulation)):
            raise ValueError(
                f"Randomization term 'my_randomize_rigid_body_mass' not supported for asset: "
                f"'{self.asset_cfg.name}' with type: '{type(self.asset)}'."
            )
            
        # 验证操作参数 (参考官方源码)
        if cfg.params["operation"] == "scale":
            if "mass_distribution_params" in cfg.params:
                _validate_scale_range(
                    cfg.params["mass_distribution_params"], "mass_distribution_params", allow_zero=False
                )
        elif cfg.params["operation"] not in ("abs", "add"):
            raise ValueError(
                "Randomization term 'my_randomize_rigid_body_mass' does not support operation:"
                f" '{cfg.params['operation']}'."
            )
    
    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        mass_distribution_params: tuple[float, float],
        operation: Literal["add", "scale", "abs"] = "add",
        distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
        recompute_inertia: bool = True,
        min_mass: float = 1e-6,
    ):
        """执行质量随机化和缓存。"""
        # 解析环境ID
        if env_ids is None:
            env_ids = torch.arange(env.scene.num_envs, device="cpu")
        else:
            env_ids = env_ids.cpu()
        
        # 初始化缓存张量
        if not hasattr(env, "_privileged_mass_params"):
            env._privileged_mass_params = torch.zeros(env.num_envs, 5, device=env.device)
        
        # 获取要随机化的身体索引
        if self.asset_cfg.body_ids == slice(None):
            body_ids = torch.arange(self.asset.num_bodies, dtype=torch.int, device="cpu")
        else:
            body_ids = torch.tensor(self.asset_cfg.body_ids, dtype=torch.int, device="cpu")
        
        # 1. 采样随机噪声 (Noise Sampling)
        low, high = mass_distribution_params
        if distribution == "uniform":
            noise = torch.empty(len(env_ids), device="cpu").uniform_(low, high)
        elif distribution == "log_uniform":
            log_low = torch.log(torch.tensor(low, dtype=torch.float32))
            log_high = torch.log(torch.tensor(high, dtype=torch.float32))
            noise = torch.exp(torch.empty(len(env_ids), device="cpu").uniform_(log_low.item(), log_high.item()))
        elif distribution == "gaussian":
            mean = (low + high) / 2
            std = (high - low) / 6
            noise = torch.normal(mean, std, size=(len(env_ids),), device="cpu")
            noise = torch.clamp(noise, low, high)
        else:
            raise ValueError(f"Unknown distribution: {distribution}")
            
        # 2. 应用随机化 (Apply Randomization)
        # 获取所有刚体的质量 (num_envs, num_bodies)
        masses = self.asset.root_physx_view.get_masses()
        
        # 提取要修改的部分 (基于 default_mass)
        masses_to_modify = self.asset.data.default_mass[env_ids[:, None], body_ids].clone()
        
        # 扩展噪声维度以匹配 (num_envs, num_bodies)
        noise_expanded = noise.unsqueeze(-1).expand(len(env_ids), len(body_ids))
        
        if operation == "add":
            masses_to_modify += noise_expanded
        elif operation == "scale":
            masses_to_modify *= noise_expanded
        elif operation == "abs":
            masses_to_modify = noise_expanded
            
        # 确保质量为正值
        masses_to_modify = torch.clamp(masses_to_modify, min=min_mass)
        
        # 更新完整质量张量
        masses[env_ids[:, None], body_ids] = masses_to_modify
        
        # 设置到物理仿真 (传入对应环境的完整刚体质量)
        self.asset.root_physx_view.set_masses(masses[env_ids], env_ids)
        
        # 3. 重新计算惯性张量 (Recompute Inertia)
        if recompute_inertia:
            # 计算新质量与默认质量的比率
            default_masses = self.asset.data.default_mass[env_ids[:, None], body_ids]
            # 避免除以零
            default_masses = torch.clamp(default_masses, min=1e-6)
            ratios = masses_to_modify / default_masses
            
            # 获取所有惯性张量
            inertias = self.asset.root_physx_view.get_inertias()
            
            # 获取默认惯性张量并缩放
            if isinstance(self.asset, Articulation):
                # Articulation inertia shape: (num_envs, num_bodies, 9)
                # 更新选定刚体的惯性张量
                inertias[env_ids[:, None], body_ids] = (
                    self.asset.data.default_inertia[env_ids[:, None], body_ids] * ratios[..., None]
                )
                self.asset.root_physx_view.set_inertias(inertias[env_ids], env_ids)
            else:
                # RigidObject inertia shape: (num_envs, 9)
                inertias[env_ids] = self.asset.data.default_inertia[env_ids] * ratios.squeeze(-1)
                self.asset.root_physx_view.set_inertias(inertias[env_ids], env_ids)
        
        # 4. 缓存采样参数 (Cache Parameters)
        # 缓存的是采样的噪声值 (noise)，而不是最终质量
        # 这对于网络学习 "变化量" 通常更有用
        env_ids_device = env_ids.to(env.device)
        noise_device = noise.to(env.device)
        
        if "base" in self.body_names or any("base" in name for name in self.body_names):
            env._privileged_mass_params[env_ids_device, 0] = noise_device
        elif "gripper" in self.body_names or any("gripper" in name.lower() for name in self.body_names):
            env._privileged_mass_params[env_ids_device, 4] = noise_device
        else:
            env._privileged_mass_params[env_ids_device, 0] = noise_device
