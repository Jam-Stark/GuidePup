from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import torch
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg as IsaacLabJointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction as IsaacLabJointPositionAction
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass
from isaaclab.utils.buffers import DelayBuffer

from .io_descriptor_compat import GenericActionIODescriptor


class DelayedJointPositionAction(IsaacLabJointPositionAction):
    """Joint position action term with an episode-fixed env-step delay."""

    cfg: "DelayedJointPositionActionCfg"

    def __init__(self, cfg: "DelayedJointPositionActionCfg", env: ManagerBasedEnv):
        super().__init__(cfg, env)
        if cfg.min_delay_steps < 0:
            raise ValueError(f"min_delay_steps must be non-negative, received: {cfg.min_delay_steps}")
        if cfg.max_delay_steps < cfg.min_delay_steps:
            raise ValueError(
                f"max_delay_steps must be >= min_delay_steps, received: {(cfg.min_delay_steps, cfg.max_delay_steps)}"
            )

        self._delay_buffer = DelayBuffer(cfg.max_delay_steps, self.num_envs, device=self.device)
        self._delayed_actions = torch.zeros_like(self.processed_actions)
        self.reset()

    def _resolve_delay_reset_ids(
        self, env_ids: Sequence[int] | slice | torch.Tensor | None
    ) -> tuple[Sequence[int] | slice, int]:
        if env_ids is None or env_ids == slice(None):
            return slice(None), self.num_envs
        if isinstance(env_ids, slice):
            start, stop, step = env_ids.indices(self.num_envs)
            return env_ids, len(range(start, stop, step))
        if isinstance(env_ids, torch.Tensor):
            env_ids = env_ids.to(device=self.device, dtype=torch.long)
            return env_ids, int(env_ids.numel())
        env_ids = torch.tensor(list(env_ids), device=self.device, dtype=torch.long)
        return env_ids, int(env_ids.numel())

    def _sample_time_lags(self, num_envs: int) -> int | torch.Tensor:
        if self.cfg.min_delay_steps == self.cfg.max_delay_steps:
            return self.cfg.min_delay_steps
        return torch.randint(
            low=self.cfg.min_delay_steps,
            high=self.cfg.max_delay_steps + 1,
            size=(num_envs,),
            dtype=torch.int,
            device=self.device,
        )

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        self._delayed_actions[:] = self._delay_buffer.compute(self.processed_actions)

    def apply_actions(self):
        self._asset.set_joint_position_target(self._delayed_actions, joint_ids=self._joint_ids)

    @property
    def IO_descriptor(self) -> GenericActionIODescriptor:
        descriptor = super().IO_descriptor
        descriptor.delay_range_steps = [int(self.cfg.min_delay_steps), int(self.cfg.max_delay_steps)]
        descriptor.use_default_offset = bool(self.cfg.use_default_offset)
        return descriptor

    def reset(self, env_ids: Sequence[int] | slice | torch.Tensor | None = None):
        super().reset(env_ids)
        batch_ids, num_envs = self._resolve_delay_reset_ids(env_ids)
        time_lags = self._sample_time_lags(num_envs)
        self._delay_buffer.set_time_lag(time_lag=time_lags, batch_ids=batch_ids)
        self._delay_buffer.reset(batch_ids)
        self._processed_actions[batch_ids] = 0.0
        self._delayed_actions[batch_ids] = 0.0


@configclass
class DelayedJointPositionActionCfg(IsaacLabJointPositionActionCfg):
    """Configuration for a delayed joint position action term."""

    class_type = DelayedJointPositionAction

    min_delay_steps: int = 0
    max_delay_steps: int = 0

class VariableForceAction(ActionTerm):
    """
    Action term that applies a variable external force (trapezoidal profile) to a rigid body.
    Executed at every physics step (application) and updated at every environment step (logic).
    """
    cfg: VariableForceActionCfg

    def __init__(self, cfg: VariableForceActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._asset: Articulation | RigidObject = env.scene[cfg.asset_name]
        self._step_dt = self._resolve_step_dt(env)
        self.push_interval_range_steps = self._seconds_range_to_steps(cfg.push_interval_range_s, cfg.push_interval_range)
        self.duration_range_steps = self._seconds_range_to_steps(cfg.duration_range_s, cfg.duration_range)
        self.settling_time_steps = self._seconds_to_steps(cfg.settling_time_s, cfg.settling_time_steps)
        
        # Resolve body index
        if cfg.body_name is None:
            self._body_idx = None 
        else:
            self._body_idx, _ = self._asset.find_bodies(cfg.body_name)
            
        # Initialize state buffers
        self.num_envs = env.num_envs
        self.device = env.device
        
        # State tensors
        self.push_interval = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.push_end_step = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.push_duration_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        
        self.selected_env_ids = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.freed_envs = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        
        self.force_target = torch.zeros((self.num_envs, 3), device=self.device)
        self.current_Fxyz = torch.zeros((self.num_envs, 3), device=self.device)
        self._raw_actions = torch.zeros((self.num_envs, self.action_dim), device=self.device)
        self._zero_torques = torch.zeros((self.num_envs, 3), device=self.device)
        
        # Initialize intervals
        self._sample_new_intervals(torch.arange(self.num_envs, device=self.device))

    @property
    def action_dim(self) -> int:
        return int(getattr(self.cfg, "dim", 0))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self.current_Fxyz

    def _resolve_step_dt(self, env: ManagerBasedEnv) -> float:
        """Resolve one env-step duration in seconds."""
        if hasattr(env, "step_dt") and env.step_dt is not None:
            return float(env.step_dt)
        physics_dt = getattr(env, "physics_dt", None)
        if physics_dt is None:
            sim_dt = getattr(getattr(env, "sim", None), "dt", None)
            if sim_dt is None:
                raise ValueError("Cannot infer sim.dt or physics_dt; please provide step_dt.")
            physics_dt = float(sim_dt)
        decimation = getattr(getattr(env, "cfg", None), "decimation", None)
        if decimation is None:
            decimation = getattr(env, "decimation", 1)
        decimation = 1 if decimation is None else decimation
        return float(physics_dt) * float(decimation)

    def _seconds_range_to_steps(self, seconds_range: tuple[float, float] | None, fallback: tuple[int, int]) -> tuple[int, int]:
        if seconds_range is None:
            return fallback
        low_s, high_s = seconds_range
        low = max(int(round(low_s / self._step_dt)), 1)
        high = max(int(round(high_s / self._step_dt)), low)
        return (low, high)

    def _seconds_to_steps(self, seconds_val: float | None, fallback: int) -> int:
        if seconds_val is None:
            return max(fallback, 1)
        return max(int(round(seconds_val / self._step_dt)), 1)

    def _resolve_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor | None) -> torch.Tensor:
        if env_ids is None:
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, slice):
            start, stop, step = env_ids.indices(self.num_envs)
            return torch.arange(start, stop, step, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.tensor(env_ids, device=self.device, dtype=torch.long)

    def _sample_new_intervals(self, env_ids):
        if len(env_ids) == 0:
            return
        # Random interval
        low, high = self.push_interval_range_steps
        self.push_interval[env_ids] = torch.randint(low, high + 1, (len(env_ids),), device=self.device)

    def reset(self, env_ids: Sequence[int] | slice | torch.Tensor | None = None):
        env_ids = self._resolve_env_ids(env_ids)
        
        self.selected_env_ids[env_ids] = False
        self.freed_envs[env_ids] = False
        self.force_target[env_ids] = 0.0
        self.current_Fxyz[env_ids] = 0.0
        self.push_end_step[env_ids] = 0
        self.push_duration_steps[env_ids] = 0
        if self.action_dim > 0:
            self._raw_actions[env_ids] = 0.0
        
        self._sample_new_intervals(env_ids)

    def process_actions(self, actions: torch.Tensor):
        # Logic from ForceAction.md
        # Executed once per environment step
        
        if self.action_dim > 0:
            self._raw_actions[:] = actions.to(self.device)

        current_time = self._env.episode_length_buf  # [num_envs]
        
        # --- Phase 1: Trigger & Init ---
        # Check trigger condition: episode_length_buf % push_interval == 0
        # Note: We need to handle the case where push_interval is 0 or uninitialized, but we init in __init__
        
        # Find envs that trigger a new push
        # We use modulo. To avoid division by zero, ensure push_interval > 0.
        trigger_mask = (current_time % self.push_interval == 0) & (current_time > 0)
        new_ids = trigger_mask.nonzero(as_tuple=False).squeeze(-1)
        
        if len(new_ids) > 0:
            # Probability Freed
            rand_vals = torch.rand(len(new_ids), device=self.device)
            freed_mask = rand_vals < self.cfg.prob_freed
            
            freed_ids = new_ids[freed_mask]
            active_ids = new_ids[~freed_mask]
            
            # Handle Freed Envs
            if len(freed_ids) > 0:
                self.freed_envs[freed_ids] = True
                self.selected_env_ids[freed_ids] = False
                self.force_target[freed_ids] = 0.0
                self.current_Fxyz[freed_ids] = 0.0
            
            # Handle Active Envs
            if len(active_ids) > 0:
                self.freed_envs[active_ids] = False
                self.selected_env_ids[active_ids] = True
                
                # Generate Target Force
                f_min, f_max = self.cfg.force_range
                # Random direction on sphere or just random 3D vector? 
                # ForceAction.md says: "在 [min_force, max_force] 范围内随机采样 Fx, Fy, Fz"
                # This implies uniform sampling in range for each component? Or magnitude?
                # Usually it means magnitude or component-wise. Let's assume component-wise for now or random direction * magnitude.
                # "ForceAction.md": "在 [min_force, max_force] 范围内随机采样 Fx, Fy, Fz" -> This sounds like component-wise.
                # But usually force is a vector. Let's assume random direction with magnitude in range.
                # Or maybe component wise. Let's stick to component wise if that's what it says.
                # Actually, "ForceAction.md" says: "在 [min_force, max_force] 范围内随机采样 Fx, Fy, Fz".
                # This could mean Fx ~ U(min, max), Fy ~ U(min, max)...
                # Let's implement component-wise uniform sampling.
                self.force_target[active_ids] = torch.rand((len(active_ids), 3), device=self.device) * (f_max - f_min) + f_min
                # Randomly flip signs? "ForceAction.md" doesn't say. But usually forces can be negative.
                # If min_force is negative, it's fine. If min_force is positive, it's always positive direction?
                # Let's assume the range can be negative or we multiply by random sign.
                # For now, I'll stick to the range provided. If user provides (-10, 10), it works.
                
                # Generate Duration
                d_min, d_max = self.duration_range_steps
                durations = torch.randint(d_min, d_max + 1, (len(active_ids),), device=self.device)
                
                # Safety Clip
                # duration = min(duration, (interval - settling_time) / 2)
                interval = self.push_interval[active_ids]
                settling = self.settling_time_steps
                max_dur = (interval - settling) // 2
                max_dur = torch.clamp(max_dur, min=1)
                durations = torch.min(durations, max_dur)
                # Ensure duration is at least 1 to avoid division by zero
                durations = torch.max(durations, torch.tensor(1, device=self.device))
                
                self.push_duration_steps[active_ids] = durations
                self.push_end_step[active_ids] = current_time[active_ids] + durations

        # --- Phase 2: Execution ---
        # Only for selected_env_ids
        active_mask = self.selected_env_ids
        if active_mask.any():
            active_idxs = active_mask.nonzero(as_tuple=False).squeeze(-1)
            
            curr_t = current_time[active_idxs]
            end_t = self.push_end_step[active_idxs]
            dur = self.push_duration_steps[active_idxs].float()
            settling = self.settling_time_steps
            target_f = self.force_target[active_idxs]
            
            # Step 2.1: Ramp Up
            # current_time < push_end_time
            ramp_up_mask = curr_t < end_t
            if ramp_up_mask.any():
                # F = F_target * (current_time - (end_time - duration)) / duration
                # Wait, end_time = start_time + duration.
                # So start_time = end_time - duration.
                # (current_time - start_time) / duration
                
                # Indices within active_idxs
                ru_idxs = active_idxs[ramp_up_mask]
                # Re-gather for these indices
                ru_curr = curr_t[ramp_up_mask]
                ru_end = end_t[ramp_up_mask]
                ru_dur = dur[ramp_up_mask]
                ru_target = target_f[ramp_up_mask]
                
                start_t = ru_end - ru_dur
                ratio = (ru_curr - start_t) / ru_dur
                ratio = torch.clamp(ratio, 0.0, 1.0)
                
                self.current_Fxyz[ru_idxs] = ru_target * ratio.unsqueeze(-1)

            # Step 2.2: Settling (Hold)
            # push_end_time <= current_time <= push_end_time + settling_time
            hold_mask = (curr_t >= end_t) & (curr_t <= end_t + settling)
            if hold_mask.any():
                h_idxs = active_idxs[hold_mask]
                self.current_Fxyz[h_idxs] = self.force_target[h_idxs]

            # Step 2.3: Ramp Down
            # current_time > push_end_time + settling_time
            ramp_down_mask = curr_t > end_t + settling
            if ramp_down_mask.any():
                rd_idxs = active_idxs[ramp_down_mask]
                rd_curr = curr_t[ramp_down_mask]
                rd_end = end_t[ramp_down_mask]
                rd_dur = dur[ramp_down_mask]
                rd_target = target_f[ramp_down_mask]
                
                # F = F_target - F_target * (current_time - (end_time + settling)) / duration
                start_decay = rd_end + settling
                ratio = (rd_curr - start_decay) / rd_dur
                ratio = torch.clamp(ratio, 0.0, 1.0)
                
                self.current_Fxyz[rd_idxs] = rd_target * (1.0 - ratio.unsqueeze(-1))

        # --- Phase 3: Reset & Cleanup ---
        # current_time >= push_end_time + settling_time + duration
        # Note: In Ramp Down, we calculate force. If ratio reaches 1, force is 0.
        # We should check if we are completely done.
        
        # We need to check for all envs, or just active ones?
        # The logic says "Reset" when task ends.
        # If we reset, we set selected_env_ids = 0.
        
        # Check for active envs that have finished
        if active_mask.any():
            active_idxs = active_mask.nonzero(as_tuple=False).squeeze(-1)
            curr_t = current_time[active_idxs]
            end_t = self.push_end_step[active_idxs]
            dur = self.push_duration_steps[active_idxs]
            settling = self.settling_time_steps
            
            done_mask = curr_t >= (end_t + settling + dur)
            if done_mask.any():
                done_idxs = active_idxs[done_mask]
                
                self.selected_env_ids[done_idxs] = False
                self.force_target[done_idxs] = 0.0
                self.current_Fxyz[done_idxs] = 0.0
                self.push_end_step[done_idxs] = 0
                self.push_duration_steps[done_idxs] = 0
                
                # Update interval
                self._sample_new_intervals(done_idxs)

        # --- Phase 4: Freed Handling ---
        # Ensure freed envs are zero
        if self.freed_envs.any():
            self.current_Fxyz[self.freed_envs] = 0.0

    def apply_actions(self):
        # Core: Apply the force at every physics step
        self._asset.set_external_force_and_torque(
            forces=self.current_Fxyz,
            torques=self._zero_torques,
            body_ids=self._body_idx
        )

@configclass
class VariableForceActionCfg(ActionTermCfg):
    """Configuration for the variable force action term."""
    class_type = VariableForceAction
    asset_name: str = "robot"
    body_name: str | None = "base"
    
    # Parameters from ForceAction.md (seconds preferred; *_steps kept for fallback)
    push_interval_range_s: tuple[float, float] | None = (3.5, 9.0)  # Trigger interval in seconds
    duration_range_s: tuple[float, float] | None = (1.0, 3.0)     # Ramp duration in seconds
    settling_time_s: float | None = 1.0                             # Hold time in seconds

    push_interval_range: tuple[int, int] = (175, 450) # Trigger interval in env steps (≈3.5–9.0s at 20ms/step)
    force_range: tuple[float, float] = (-50.0, 50.0) # Range for force magnitude
    duration_range: tuple[int, int] = (50, 150) # Ramp duration in env steps (≈1.0–3.0s at 20ms/step)
    settling_time_steps: int = 50 # Hold time in env steps (≈1.0s at 20ms/step)
    prob_freed: float = 0.2 # Probability of being freed
    
    dim: int = 0 # Not using agent actions



class ApplyForceCommandAction(ActionTerm):
    """Reads a command term (num_envs, 3) and applies it as an external force."""

    cfg: ApplyForceCommandActionCfg

    def __init__(self, cfg: ApplyForceCommandActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self._asset: Articulation | RigidObject = env.scene[cfg.asset_name]
        self._command = torch.zeros((env.num_envs, 3), device=env.device)
        self._is_global = bool(self.cfg.is_global)
        try:
            command_term = self._env.command_manager.get_term(self.cfg.command_name)
            if hasattr(command_term, "cfg") and hasattr(command_term.cfg, "is_global"):
                self._is_global = bool(command_term.cfg.is_global)
        except KeyError:
            pass

        if cfg.body_name is None:
            self._body_ids = None
            num_bodies = int(getattr(self._asset, "num_bodies", 1))
        else:
            self._body_ids, _ = self._asset.find_bodies(cfg.body_name)
            if len(self._body_ids) == 0:
                raise ValueError(f"Unable to find bodies matching '{cfg.body_name}' in asset '{cfg.asset_name}'.")
            num_bodies = len(self._body_ids)

        self._forces = torch.zeros((env.num_envs, num_bodies, 3), device=env.device)
        self._zero_torques = torch.zeros_like(self._forces)
        self._raw_actions = torch.zeros((env.num_envs, self.action_dim), device=env.device)

        # Cache applied force tensors so observations can access the actual applied values.
        if not hasattr(self._env, "_applied_force_commands"):
            self._env._applied_force_commands = {}
        self._env._applied_force_commands[self.cfg.command_name] = self._command

    @property
    def action_dim(self) -> int:
        return int(getattr(self.cfg, "dim", 0))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._command

    @property
    def IO_descriptor(self) -> GenericActionIODescriptor:
        descriptor = super().IO_descriptor
        descriptor.shape = (self.action_dim,)
        descriptor.dtype = str(self.raw_actions.dtype)
        descriptor.action_type = "ForceCommandAction"
        descriptor.asset_name = self.cfg.asset_name
        descriptor.body_name = self.cfg.body_name
        descriptor.command_name = self.cfg.command_name
        descriptor.is_global = bool(self._is_global)
        return descriptor

    @property
    def export_IO_descriptor(self) -> bool:
        return self.action_dim > 0

    def process_actions(self, actions: torch.Tensor):
        if self.action_dim > 0:
            self._raw_actions[:] = actions.to(self.device)

    def apply_actions(self):
        self._command[:] = self._env.command_manager.get_command(self.cfg.command_name)
        self._forces[:] = self._command.unsqueeze(1)
        self._asset.set_external_force_and_torque(
            forces=self._forces,
            torques=self._zero_torques,
            body_ids=self._body_ids,
            is_global=self._is_global,
        )

class FixedJointPositionAction(ActionTerm):
    """Holds specified joints at their default positions without adding action dimensions."""

    cfg: "FixedJointPositionActionCfg"

    def __init__(self, cfg: "FixedJointPositionActionCfg", env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._asset: Articulation = env.scene[cfg.asset_name]
        self._joint_ids, _ = self._asset.find_joints(cfg.joint_names)
        if len(self._joint_ids) == 0:
            raise ValueError(f"Unable to find joints matching '{cfg.joint_names}' in asset '{cfg.asset_name}'.")
        self._default_pos = self._asset.data.default_joint_pos[:, self._joint_ids].clone()
        self._raw_actions = torch.zeros((env.num_envs, self.action_dim), device=env.device)

    @property
    def action_dim(self) -> int:
        return int(getattr(self.cfg, "dim", 0))

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._default_pos

    @property
    def export_IO_descriptor(self) -> bool:
        return self.action_dim > 0

    def process_actions(self, actions: torch.Tensor):
        if self.action_dim > 0:
            self._raw_actions[:] = actions.to(self.device)

    def apply_actions(self):
        self._asset.set_joint_position_target(self._default_pos, joint_ids=self._joint_ids)

    def reset(self, env_ids: Sequence[int] | slice | torch.Tensor | None = None):
        if env_ids is None:
            env_ids = slice(None)
        self._default_pos[env_ids] = self._asset.data.default_joint_pos[env_ids][:, self._joint_ids]


@configclass
class ApplyForceCommandActionCfg(ActionTermCfg):
    """Apply a 3D force command to a body every physics step."""

    class_type = ApplyForceCommandAction

    asset_name: str = "robot"
    body_name: str | None = MISSING
    command_name: str = MISSING
    is_global: bool = False
    dim: int = 0

@configclass
class FixedJointPositionActionCfg(ActionTermCfg):
    """Fix joint position targets at default values."""

    class_type = FixedJointPositionAction

    asset_name: str = "robot"
    joint_names: Sequence[str] | str = MISSING
    dim: int = 0
