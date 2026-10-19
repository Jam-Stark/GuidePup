"""Gyro/accelerometer attitude and drifting WBC-velocity dead reckoning.

No depth, simulator pose/twist or command integration. Absolute heading and
translation remain unobserved. One attitude estimate also supplies WBC gravity.
"""
from __future__ import annotations

import uuid
import imufusion
import numpy as np
from scipy.spatial.transform import Rotation
from .state import RobotState


class IMUProprioOdometry:
    source = 'imu_fusion_and_wbc_auxiliary_velocity_dead_reckoning'

    def __init__(self, initial_packet, velocity_body, *, sensor_period_s):
        self.epoch = str(uuid.uuid4())
        acceleration = np.asarray(initial_packet.accelerometer_m_s2, dtype=float)
        if not np.linalg.norm(acceleration) > 0:
            raise ValueError('Stationary IMU gravity initialization is required')
        self.gravity_base = -acceleration/np.linalg.norm(acceleration)
        up = -self.gravity_base
        forward = np.array([1., 0., 0.])-up*up[0]
        if np.linalg.norm(forward) < 1e-6:
            raise ValueError('Initial orientation has no base-forward horizontal projection')
        forward /= np.linalg.norm(forward)
        self.transform = np.eye(4)
        self.transform[:3, :3] = np.stack((forward, np.cross(up, forward), up))
        if not np.isfinite(sensor_period_s) or sensor_period_s <= 0:
            raise ValueError('IMU sample period must be finite and positive')
        self.attitude = imufusion.Ahrs()
        # Published Fusion defaults for gain/rejection. No gyro range has been
        # supplied, so overrange recovery is explicitly disabled, not guessed.
        self.attitude.set_settings(imufusion.AhrsSettings(
            sample_rate=1./sensor_period_s, convention=imufusion.CONVENTION_NWU,
            gain=.5, gyroscope_range=0, acceleration_rejection=10,
            magnetic_rejection=0, rejection_timeout=5))
        quat = Rotation.from_matrix(self.transform[:3, :3]).as_quat()
        self.attitude.set_quaternion(quat[[3, 0, 1, 2]])
        self.stamp = float(initial_packet.stamp_s)
        self.gyro = initial_packet.gyro_rad_s.copy()
        self.velocity = self._velocity(velocity_body)

    @staticmethod
    def _velocity(value):
        v = np.asarray(value, dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError('Measured auxiliary velocity must be a finite 3-vector')
        return v.copy()

    def update(self, packet, velocity_body):
        velocity = self._velocity(velocity_body)
        dt = float(packet.stamp_s)-self.stamp
        if dt < 0:
            raise RuntimeError('IMU timestamp moved backwards')
        old_rotation = self.transform[:3, :3].copy()
        if dt > 0:
            self.attitude.set_sample_period(dt)
            self.attitude.update_no_magnetometer(
                np.degrees(packet.gyro_rad_s), packet.accelerometer_m_s2/9.81)
            quat = np.asarray(self.attitude.get_quaternion(), dtype=float)
            rotation = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
            self.transform[:3, 3] += (old_rotation@self.velocity+rotation@velocity)*dt/2
            self.transform[:3, :3] = rotation
            self.gravity_base = -rotation[2, :].copy()
        self.stamp, self.gyro, self.velocity = float(packet.stamp_s), packet.gyro_rad_s.copy(), velocity
        return self.state

    @property
    def state(self):
        internal = self.attitude.get_internal_states()
        return RobotState(stamp_s=self.stamp, epoch=self.epoch, odom_from_base=self.transform,
            twist_base=np.r_[self.velocity, self.gyro], tracking='tracking', source=self.source,
            quality={'velocity_source': 'a2_policy_aux_divided_by_2',
                'attitude_source': 'imufusion_1.3.3_gyro_accelerometer',
                'accelerometer_ignored': bool(internal.accelerometer_ignored),
                'acceleration_error_deg': float(internal.acceleration_error),
                'acceleration_recovery_trigger': float(internal.acceleration_recovery_trigger),
                'visual_or_depth_correction': False, 'global_localization': False,
                'limitation': 'Tilt is sensor-fused; heading and integrated WBC translation still drift and cannot prove metric arrival.'})
