# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for gait phase command generator."""

from __future__ import annotations

import math

from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass

from .gait_phase_command import GaitPhaseCommand


@configclass
class GaitPhaseCommandCfg(CommandTermCfg):
    """Configuration for the gait phase command generator."""

    class_type: type = GaitPhaseCommand

    # Disable the built-in time_left resampling; this term manages its own per-step update.
    resampling_time_range: tuple[float, float] = (9e9, 9e9)

    locomotion_command_name: str = "locomotion_commands"
    """Name of the locomotion command term used to query standing environments."""

    gait_frequency: float = 2.0
    """Gait frequency (Hz)."""

    phase_offset: float = 0.5
    """Phase offset between diagonal leg pairs for trot (FL/RR shifted by this amount)."""

    initial_phase: float = 0.0
    """Initial phase (wrapped to [0, 1)) on reset."""
