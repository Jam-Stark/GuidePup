# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg as _BaseRslRlPpoActorCriticCfg


@configclass
class RslRlPpoActorCriticCfg(_BaseRslRlPpoActorCriticCfg):
    """Extends Isaac Lab's PPO actor-critic config with normalization flags."""

    actor_obs_normalization: bool = False
    critic_obs_normalization: bool = False
