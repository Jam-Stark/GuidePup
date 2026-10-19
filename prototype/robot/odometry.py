"""Local depth/IMU/proprioceptive odometry, with explicit geometric observability.

Gyro and a measured body-velocity estimate predict motion. Depth point-to-plane
registration corrects translation only in directions constrained by the visible
surfaces. A blank wall cannot manufacture a visual tangential velocity.
"""
from __future__ import annotations

import uuid
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .state import RobotState


class RGBDLocalOdometry:
    source = 'depth_imu_proprioception'

    def __init__(self, camera_to_base: np.ndarray, valid_range_m: tuple[float, float]) -> None:
        self.camera_to_base = np.asarray(camera_to_base, dtype=np.float64).copy()
        if self.camera_to_base.shape != (4, 4):
            raise ValueError('camera_to_base must be 4x4')
        self.camera_from_base = np.linalg.inv(self.camera_to_base)
        self.near, self.far = map(float, valid_range_m)
        if not 0 < self.near < self.far:
            raise ValueError('valid_range_m must be ordered positive depths')
        self.epoch = str(uuid.uuid4())
        self._previous_points = None
        self._previous_stamp_s = None
        self._state = None
        self._gravity_base = None
        self._imu_stamp_s = None
        self._imu_gyro = None
        self._imu_velocity = None
        self._previous_from_current = np.eye(4)
        self._lost = False
        self._gravity_initialization_norm_m_s2 = None

    @property
    def state(self) -> RobotState:
        if self._state is None:
            raise RuntimeError('Odometry has not received an initial depth frame')
        return self._state

    @property
    def gravity_base(self) -> np.ndarray:
        if self._gravity_base is None:
            raise RuntimeError('Odometry requires stationary IMU initialization')
        return self._gravity_base.copy()

    def initialize_imu(self, packet, velocity_body_m_s) -> None:
        acceleration = self._vector(packet.accelerometer_m_s2, 'accelerometer_m_s2')
        magnitude = float(np.linalg.norm(acceleration))
        if magnitude == 0:
            raise RuntimeError('Stationary accelerometer initialization has zero magnitude')
        self._gravity_base = -acceleration/magnitude
        self._gravity_initialization_norm_m_s2 = magnitude
        self._imu_stamp_s = float(packet.stamp_s)
        self._imu_gyro = self._vector(packet.gyro_rad_s, 'gyro_rad_s')
        self._imu_velocity = self._vector(velocity_body_m_s, 'velocity_body_m_s')

    def propagate_imu(self, packet, velocity_body_m_s) -> None:
        """Consume every IMU sample; velocity is an estimate, never a command."""
        if self._gravity_base is None:
            raise RuntimeError('initialize_imu must precede propagation')
        stamp = float(packet.stamp_s)
        gyro = self._vector(packet.gyro_rad_s, 'gyro_rad_s')
        velocity = self._vector(velocity_body_m_s, 'velocity_body_m_s')
        dt = stamp-self._imu_stamp_s
        if dt < 0:
            raise RuntimeError('IMU timestamps moved backwards')
        increment = Rotation.from_rotvec((self._imu_gyro+gyro)*dt/2).as_matrix()
        self._gravity_base = increment.T @ self._gravity_base
        if self._previous_points is not None and not self._lost:
            rotation = self._previous_from_current[:3, :3]
            next_rotation = rotation @ increment
            self._previous_from_current[:3, 3] += (rotation @ self._imu_velocity + next_rotation @ velocity)*dt/2
            self._previous_from_current[:3, :3] = next_rotation
        self._imu_stamp_s, self._imu_gyro, self._imu_velocity = stamp, gyro, velocity

    def reset(self, packet, velocity_body_m_s) -> None:
        self.epoch = str(uuid.uuid4())
        self._previous_points = None
        self._previous_stamp_s = None
        self._state = None
        self._previous_from_current = np.eye(4)
        self._lost = False
        self.initialize_imu(packet, velocity_body_m_s)

    def _lost_state(self, stamp, reason, quality):
        self._lost = True
        self._state = RobotState(stamp_s=stamp, epoch=self.epoch,
            odom_from_base=self.state.odom_from_base, twist_base=np.zeros(6),
            tracking='lost', source=self.source,
            quality={**quality, 'velocity_estimated': False, 'failure': reason})
        return self._state

    def update(self, *, stamp_s: float, depth_z_m: np.ndarray, intrinsics: dict[str, float]) -> RobotState:
        if self._gravity_base is None:
            raise RuntimeError('initialize_imu must precede depth updates')
        if self._lost:
            raise RuntimeError('Tracking is lost; reset with a new sensor epoch before reuse')
        stamp = float(stamp_s)
        points, normals, valid_fraction = self._cloud(depth_z_m, intrinsics)
        quality = {'valid_depth_fraction': valid_fraction, 'depth_points': len(points),
                   'translation_source': 'proprioceptive_velocity_plus_observable_depth',
                   'rotation_source': 'integrated_gyro', 'normal_eigenvalue_ratio_min': .01}
        if self._previous_points is None:
            self._state = RobotState(stamp_s=stamp, epoch=self.epoch,
                odom_from_base=self._gravity_aligned_odom_from_base(), twist_base=np.zeros(6),
                tracking='initializing', source=self.source,
                quality={**quality, 'velocity_estimated': False,
                         'gravity_initialization_norm_m_s2': self._gravity_initialization_norm_m_s2})
        else:
            dt = stamp-self._previous_stamp_s
            if dt <= 0:
                raise RuntimeError('Depth timestamps must strictly increase')
            camera_current_from_previous = (self.camera_from_base @ np.linalg.inv(self._previous_from_current)
                                            @ self.camera_to_base)
            rotated = self._previous_points @ camera_current_from_previous[:3, :3].T
            tree = cKDTree(points)
            for _ in range(6):
                warped = rotated+camera_current_from_previous[:3, 3]
                distances, indices = tree.query(warped)
                good = distances < .10
                if good.sum() < 100:
                    return self._lost_state(stamp, 'insufficient_depth_overlap', quality)
                n = normals[indices[good]]
                residual = np.einsum('ij,ij->i', n, points[indices[good]]-warped[good])
                hessian = n.T @ n/len(n)
                gradient = n.T @ residual/len(n)
                eigenvalues, directions = np.linalg.eigh(hessian)
                observed = eigenvalues > eigenvalues[-1]*.01
                # Correct only measured normal directions; the orthogonal part
                # retains the proprioceptive prediction in this single estimator.
                correction = directions[:, observed] @ ((directions[:, observed].T @ gradient)/eigenvalues[observed])
                camera_current_from_previous[:3, 3] += correction
                if np.linalg.norm(correction) < 1e-5:
                    break
            prior_pose = self.state.odom_from_base
            pose = prior_pose @ self.camera_to_base @ np.linalg.inv(camera_current_from_previous) @ self.camera_from_base
            relative = np.linalg.inv(prior_pose) @ pose
            velocity = relative[:3, :3].T @ relative[:3, 3]/dt
            quality.update(depth_translation_rank=int(observed.sum()),
                depth_normal_eigenvalues=eigenvalues.tolist(),
                proprioceptive_translation_directions_camera=directions[:, ~observed].T.tolist(),
                depth_correspondences=int(good.sum()),
                point_to_plane_rmse_m=float(np.sqrt(np.mean(residual**2))),
                velocity_estimated=True)
            self._state = RobotState(stamp_s=stamp, epoch=self.epoch, odom_from_base=pose,
                twist_base=np.r_[velocity, self._imu_gyro], tracking='tracking', source=self.source, quality=quality)
        self._previous_points = points.copy()
        self._previous_stamp_s = stamp
        self._previous_from_current = np.eye(4)
        return self._state

    def _cloud(self, depth_z_m, intrinsics):
        depth = np.asarray(depth_z_m, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > self.near) & (depth < self.far)
        if not valid.any():
            raise RuntimeError('Odometry received no valid depth')
        height, width = depth.shape
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height,
            *(float(intrinsics[key]) for key in ('fx', 'fy', 'cx', 'cy')))
        cloud = o3d.geometry.PointCloud.create_from_depth_image(
            o3d.geometry.Image(np.where(valid, depth, 0).astype(np.float32)), intrinsic,
            depth_scale=1., depth_trunc=self.far, stride=3).voxel_down_sample(.04)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.12, max_nn=30))
        return np.asarray(cloud.points), np.asarray(cloud.normals), float(valid.mean())

    def _gravity_aligned_odom_from_base(self):
        up = -self._gravity_base
        forward = np.array([1., 0., 0.])
        forward -= up * float(up @ forward)
        norm = np.linalg.norm(forward)
        if norm == 0:
            raise RuntimeError('Gravity alignment has no base-forward yaw reference')
        forward /= norm
        transform = np.eye(4)
        transform[:3, :3] = np.stack((forward, np.cross(up, forward), up))
        return transform

    @staticmethod
    def _vector(value, name):
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (3,) or not np.isfinite(result).all():
            raise ValueError(f'{name} must be finite [x,y,z]')
        return result.copy()
