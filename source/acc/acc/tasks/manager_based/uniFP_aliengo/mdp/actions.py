from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.assets import Articulation, RigidObject
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

@configclass
class ConstantForceActionCfg(ActionTermCfg):
    """Configuration for the constant force action term."""
    class_type = None # Will be set to ConstantForceAction
    asset_name: str = "robot"
    body_name: str | None = "base" # Name of the body to apply force to
    force: tuple[float, float, float] = (0.0, 0.0, 0.0) # Force vector (x, y, z)
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0) # Torque vector (x, y, z)
    dim: int = 0 # No agent control


class ConstantForceAction(ActionTerm):
    """
    Action term that applies a constant external force to a rigid body.
    This action term has a dimension of 0, meaning it does not take any actions from the agent.
    It is used to apply constant forces like wind or gravity compensation.
    """
    cfg: ConstantForceActionCfg

    def __init__(self, cfg: ConstantForceActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        # resolve asset
        self._asset: Articulation | RigidObject = env.scene[cfg.asset_name]
        
        # resolve body index
        if cfg.body_name is None:
            self._body_idx = None 
        else:
            # find_bodies returns (indices, names)
            self._body_idx, _ = self._asset.find_bodies(cfg.body_name)
            
        # prepare force tensor
        # Shape: (num_envs, 3)
        self._force = torch.tensor(cfg.force, device=env.device).repeat(env.num_envs, 1)
        self._torque = torch.tensor(cfg.torque, device=env.device).repeat(env.num_envs, 1)

    def process_actions(self, actions: torch.Tensor):
        # No actions to process from the agent
        pass

    def apply_actions(self):
        # Apply external force and torque
        # This is called every physics step
        self._asset.set_external_force_and_torque(
            forces=self._force,
            torques=self._torque,
            body_ids=self._body_idx
        )


# Set the class type
ConstantForceActionCfg.class_type = ConstantForceAction
