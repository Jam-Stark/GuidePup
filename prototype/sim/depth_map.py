"""Local RGB-D occupancy: ray-observed free, observed surfaces, and unknown.

The floor is fitted to measured points in gravity-aligned odom, never scene z=0.
Geometry, free-space queries and MPC use the capture's RobotState and epoch.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import open3d as o3d

UNKNOWN, FREE, OCCUPIED, SUPPORT = 0, 1, 2, 3
# A point on measured support is occupied by a physical surface; body-volume
# queries distinguish that support from an above-floor collision obstacle.
STATE_NAMES = np.array(['unknown', 'free', 'occupied', 'occupied'])


@dataclass(frozen=True)
class MapSnapshot:
    grid: np.ndarray
    origin: np.ndarray
    floor: np.ndarray
    state: object
    frame_id: str
    obstacles_odom: np.ndarray


class DepthObstacleMap:
    def __init__(self, voxel_side_m=.04, radius_m=4.):
        self.voxel_side_m = voxel_side_m
        self.radius_m = radius_m
        self.shape = np.array([int(2*radius_m/voxel_side_m)+3]*2 + [103])
        self.snapshot = None
        self.frame_id = None
        self.footprint = json.loads((Path(__file__).resolve().parents[1] / 'control/footprint.json').read_text())

    def _floor(self, points, base):
        # The lowest visible horizontal support is a measurement hypothesis.
        # A view with no measured support plane cannot authorize motion/completion.
        below = points[points[:, 2] < base[2]-.20]
        if len(below) < 100:
            raise ValueError('RGB-D has no measured floor support')
        low = np.quantile(below[:, 2], .1)
        candidates = below[below[:, 2] < low+.12]
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(candidates))
        # A low image band can contain more wall pixels than floor pixels.
        # Extract measured planes and select the gravity-aligned support; the
        # largest plane alone is not a floor classifier.
        while len(cloud.points) >= 100:
            plane, indices = cloud.segment_plane(distance_threshold=.02, ransac_n=3, num_iterations=80)
            plane = np.asarray(plane)
            if len(indices) < 100:
                break
            if plane[2] < 0:
                plane *= -1
            if plane[2] >= np.cos(np.deg2rad(15.)):
                return plane
            cloud = cloud.select_by_index(indices, invert=True)
        raise ValueError('RGB-D has no measured gravity-aligned floor plane')

    def _inside(self, indices):
        return np.all((indices >= 0) & (indices < self.shape), axis=1)

    def update(self, frame):
        state = frame.state
        if state.tracking != 'tracking':
            raise ValueError('Depth mapping requires tracked sensor state')
        transform = state.odom_from_base
        points = frame.points_base() @ transform[:3, :3].T + transform[:3, 3]
        floor = self._floor(points, transform[:3, 3])
        size = self.voxel_side_m
        origin = np.floor(transform[:3, 3]/size).astype(np.int64) - self.shape//2
        previous = self.snapshot
        grid = np.zeros(tuple(self.shape), dtype=np.uint8)
        if previous is not None and previous.state.epoch == state.epoch:
            shift = origin - previous.origin
            if np.all(np.abs(shift) < self.shape):
                old_slices = tuple(slice(max(0, int(d)), min(int(n), int(n+d))) for d, n in zip(shift, self.shape))
                new_slices = tuple(slice(max(0, int(-d)), min(int(n), int(n-d))) for d, n in zip(shift, self.shape))
                grid[new_slices] = previous.grid[old_slices]
        # Pixel sampling is explicit; cells missed by these rays remain unknown.
        vv, uu = np.nonzero(frame.valid_mask[::3, ::3])
        vv, uu = vv*3, uu*3
        z = frame.depth_z_m[vv, uu]
        k = frame.intrinsics
        optical = np.column_stack(((uu-k['cx'])*z/k['fx'], (vv-k['cy'])*z/k['fy'], z))
        camera = transform @ frame.camera_to_base
        endpoints = optical @ camera[:3, :3].T + camera[:3, 3]
        vectors = endpoints-camera[:3, 3]
        lengths = np.linalg.norm(vectors, axis=1)
        directions = vectors/lengths[:, None]
        # Half-voxel ray steps avoid jumping over cells on diagonal rays.
        max_length = min(float(lengths.max()), 2*self.radius_m)
        flat = grid.reshape(-1)
        for distance in np.arange(size*.5, max_length, size*.5):
            rays = lengths > distance+size*np.sqrt(3.)
            cells = np.floor((camera[:3, 3]+directions[rays]*distance)/size).astype(np.int64)-origin
            cells = cells[self._inside(cells)]
            flat[np.ravel_multi_index(cells.T, tuple(self.shape))] = FREE
        # Classify ALL measured endpoints against their capture's floor. Keeping
        # support distinct prevents later floor/odom drift from turning historical
        # floor surfaces into collision obstacles. New observations replace either
        # class; mixed cells retain the above-floor obstacle, including silhouettes.
        cells = np.floor(points/size).astype(np.int64)-origin
        inside = self._inside(cells)
        cells = cells[inside]
        heights = (points[inside] @ floor[:3]+floor[3])/floor[2]
        indices = np.ravel_multi_index(cells.T, tuple(self.shape))
        flat[indices] = SUPPORT
        flat[indices[heights > .06]] = OCCUPIED
        occupied = (np.column_stack(np.nonzero(grid == OCCUPIED))+origin+.5)*size
        obstacle = occupied[occupied[:, 2] < transform[2, 3]+.25, :2]
        cells = np.unique(np.floor(obstacle/size).astype(np.int64), axis=0)
        obstacles = (cells+.5)*size
        snapshot = MapSnapshot(grid, origin, floor, state, frame.frame_id, obstacles)
        self.snapshot = snapshot  # Publish state, geometry and obstacles together.
        self.frame_id = frame.frame_id
        return obstacles

    def _point_codes(self, snapshot, points):
        cells = np.floor(np.asarray(points).reshape(-1, 3)/self.voxel_side_m).astype(np.int64)-snapshot.origin
        inside = self._inside(cells)
        result = np.zeros(len(cells), dtype=np.uint8)
        result[inside] = snapshot.grid[tuple(cells[inside].T)]
        return result

    def _footprint_state(self, snapshot, pose):
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        centers = np.asarray(self.footprint['centers_base_m']) @ np.array([[c, s], [-s, c]]) + [x, y]
        radii = np.asarray(self.footprint['radii_m'])
        size = self.voxel_side_m
        lo = np.floor((centers-radii[:, None]).min(axis=0)/size).astype(int)
        hi = np.ceil((centers+radii[:, None]).max(axis=0)/size).astype(int)
        xx, yy = np.meshgrid(np.arange(lo[0], hi[0]+1), np.arange(lo[1], hi[1]+1), indexing='ij')
        xy = (np.column_stack((xx.ravel(), yy.ravel()))+.5)*size
        # Include any XY voxel intersecting the footprint, not just circle centres.
        inside = (np.linalg.norm(xy[:, None]-centers, axis=2) <= radii+size/np.sqrt(2.)).any(axis=1)
        xy = xy[inside]
        floor = snapshot.floor
        floor_z = -(xy @ floor[:2]+floor[3])/floor[2]
        top = snapshot.state.odom_from_base[2, 3]+.25
        zs = np.arange(np.floor((floor_z.min()+.06)/size), np.ceil(top/size)+1)*size+size*.5
        xyz = np.column_stack((np.repeat(xy, len(zs), axis=0), np.tile(zs, len(xy))))
        above_floor = xyz[:, 2] > np.repeat(floor_z+.06, len(zs))
        codes = self._point_codes(snapshot, xyz[above_floor])
        if np.any(codes == OCCUPIED):
            return 'occupied'
        # The support must also be observed. A fitted plane alone cannot fill a hole.
        support = np.stack([self._point_codes(snapshot, np.column_stack((xy, floor_z+dz)))
                            for dz in (-size, 0., size)])
        if np.any(codes == UNKNOWN) or not np.all((support == SUPPORT).any(axis=0)):
            return 'unknown'
        return 'free'

    def query(self, *, epoch, stamp_s, points_odom, footprint_poses_odom):
        snapshot = self.snapshot
        if snapshot is None:
            raise ValueError('No RGB-D map available')
        if epoch != snapshot.state.epoch:
            raise ValueError('Spatial evidence and query belong to different estimator epochs')
        if stamp_s > snapshot.state.stamp_s+1e-6:
            raise ValueError('Requested observation is newer than the integrated RGB-D map')
        return {
            'epoch': epoch, 'stamp_s': snapshot.state.stamp_s,
            'map_stamp_s': snapshot.state.stamp_s, 'frame_id': 'odom',
            'capture_frame_id': snapshot.frame_id,
            'point_states': STATE_NAMES[self._point_codes(snapshot, points_odom)].tolist(),
            'footprint_states': [self._footprint_state(snapshot, p) for p in footprint_poses_odom],
            'voxel_side_m': self.voxel_side_m, 'support_plane_odom': snapshot.floor.tolist(),
            'body_volume': {'bottom_above_measured_floor_m': .06, 'top_above_base_m': .25},
        }
