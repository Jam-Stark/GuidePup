"""A bounded map of observed RGB-D obstacle surfaces for the local controller.

Unknown pixels contribute no observations. The occupied-point map is not a free-space map.
World z=0 is the level-floor odometry convention in this prototype.
"""
from __future__ import annotations

import numpy as np


class DepthObstacleMap:
    def __init__(self, voxel_side_m=.04, radius_m=4.):
        self.voxel_side_m = voxel_side_m
        self.radius_m = radius_m
        self.grid_width = int(np.ceil(2 * radius_m / voxel_side_m)) + 3
        self.cells = np.empty((0, 2), dtype=np.int64)
        self.frame_id = None

    def update(self, frame, world_from_base):
        points = frame.points_base()
        # With this front-mounted D435i profile and Z>0.2m, valid rays begin
        # beyond base x=0.492m; the measured old-A2 front extent is 0.394m.
        # Do not erase nearby furniture as "self": the former 0.6m box
        # removed 25,132 real sofa pixels in the stopped controller_01 frame.
        world = points @ world_from_base[:3, :3].T + world_from_base[:3, 3]
        # Retain objects that can strike feet/legs/trunk; discard the level support floor.
        obstacle = world[(world[:, 2] > .06) & (world[:, 2] < 1.)][:, :2]
        cells = np.floor(obstacle / self.voxel_side_m).astype(np.int64)
        candidates = np.concatenate((self.cells, cells))
        centers = (candidates + .5) * self.voxel_side_m
        nearby = np.linalg.norm(centers - world_from_base[:2, 3], axis=1) <= self.radius_m
        # Structured axis=0 sorting held the GIL for ~90ms per frame in a real run.
        # Integer grid indices preserve every occupied cell without that sort path.
        origin = np.floor((world_from_base[:2, 3] - self.radius_m) / self.voxel_side_m).astype(np.int64) - 1
        local = candidates[nearby] - origin
        flat = np.ravel_multi_index(local.T, (self.grid_width, self.grid_width))
        unique = np.unique(flat)
        self.cells = np.column_stack(np.unravel_index(unique, (self.grid_width, self.grid_width))) + origin
        self.frame_id = frame.frame_id
        return (self.cells + .5) * self.voxel_side_m
