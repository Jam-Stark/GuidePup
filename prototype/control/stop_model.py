"""Empirical zero-command response, fitted only from estimated sensor states.

The residual at each sample covers the preceding time interval in six moving-to-
paused-stop cases. Its independently maximized position/rotation/sampling terms
are an empirical planning reserve, not observed obstacle occupancy or calibrated
stationary-restart error. The coefficients remain unchanged; candidate planning
owns how a deficient reserve can be restored under geometric constraints.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def stopping_pose(state, elapsed_s, response_times_s):
    """Mean stopping pose from [x,y,yaw,vx,vy,wz], body-frame twist."""
    gains = response_times_s * (1. - np.exp(-elapsed_s / response_times_s))
    half_yaw_gain = response_times_s[2] * (1. - math.exp(-.5 * elapsed_s / response_times_s[2]))
    heading = state[2] + state[5] * half_yaw_gain
    cos, sin = np.cos, np.sin
    forward, lateral = state[3] * gains[0], state[4] * gains[1]
    values = [state[0] + forward * cos(heading) - lateral * sin(heading),
              state[1] + forward * sin(heading) + lateral * cos(heading),
              state[2] + state[5] * gains[2]]
    return np.asarray(values)



class StopModel:
    def __init__(self, path: str | Path):
        config = json.loads(Path(path).read_text())
        self.model_id = config['model_id']
        self.times_s = np.asarray(config['times_s'], dtype=float)
        self.response_times_s = np.asarray(config['velocity_response_time_s'], dtype=float)
        self.position_residual_m = np.asarray(config['position_residual_m'], dtype=float)
        self.yaw_residual_rad = np.asarray(config['yaw_residual_rad'], dtype=float)
        self.domain = config['domain']
        if (self.response_times_s.shape != (3,) or np.any(self.response_times_s <= 0.)
                or not np.isfinite(self.response_times_s).all()
                or self.times_s.ndim != 1 or not len(self.times_s)
                or not np.isfinite(self.times_s).all()
                or np.any(np.diff(np.r_[0., self.times_s]) <= 0.)
                or self.position_residual_m.shape != self.times_s.shape
                or self.yaw_residual_rad.shape != self.times_s.shape
                or not np.isfinite(self.position_residual_m).all()
                or not np.isfinite(self.yaw_residual_rad).all()
                or np.any(self.position_residual_m < 0.) or np.any(self.yaw_residual_rad < 0.)):
            raise ValueError('Invalid calibrated stopping response or residual envelope')

    def pose(self, state, elapsed_s):
        return stopping_pose(state, elapsed_s, self.response_times_s)

    def poses(self, state):
        return np.asarray([self.pose(state, t) for t in self.times_s])

    def circle_residuals(self, centers):
        # Rotating a center by an angle error moves it by at most r*|angle|.
        return (self.position_residual_m[:, None]
                + self.yaw_residual_rad[:, None] * np.linalg.norm(centers, axis=1)[None, :])
