"""Raw sensor access and policy-history construction without plant truth inputs."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SensorPacket:
    stamp_s: float
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    gyro_rad_s: np.ndarray
    accelerometer_m_s2: np.ndarray

    def __post_init__(self) -> None:
        for name, value, shape in (
            ("joint_position", self.joint_position, (12,)),
            ("joint_velocity", self.joint_velocity, (12,)),
            ("gyro_rad_s", self.gyro_rad_s, (3,)),
            ("accelerometer_m_s2", self.accelerometer_m_s2, (3,)),
        ):
            array = np.asarray(value, dtype=np.float64)
            if array.shape != shape or not np.isfinite(array).all():
                raise ValueError(f"{name} must be finite {shape}")
            object.__setattr__(self, name, array.copy())

    def payload(self) -> dict:
        return {
            "stamp_s": float(self.stamp_s),
            "joint_position": self.joint_position.tolist(),
            "joint_velocity": self.joint_velocity.tolist(),
            "gyro_rad_s": self.gyro_rad_s.tolist(),
            "accelerometer_m_s2": self.accelerometer_m_s2.tolist(),
        }


class MuJoCoSensorReader:
    """Read the model's joint encoders and IMU exclusively from sensordata."""

    def __init__(self, model, metadata) -> None:
        self.model = model
        # The selected A2 asset names joints FL_hip_joint and encoders FL_hip_pos.
        encoder_names = [name.removesuffix('_joint') for name in metadata.obs_joint_names]
        self.joint_position = tuple(self._slice(f"{name}_pos", 1) for name in encoder_names)
        self.joint_velocity = tuple(self._slice(f"{name}_vel", 1) for name in encoder_names)
        self.gyro = self._slice("imu_gyro", 3)
        self.accelerometer = self._slice("imu_acc", 3)

    def _slice(self, name: str, expected_dim: int) -> slice:
        sensor = self.model.sensor(name)
        dimension = int(self.model.sensor_dim[sensor.id])
        address = int(self.model.sensor_adr[sensor.id])
        if dimension != expected_dim:
            raise RuntimeError(f"sensor {name!r} dimension {dimension} != {expected_dim}")
        return slice(address, address + dimension)

    def read(self, data) -> SensorPacket:
        values = data.sensordata
        return SensorPacket(
            stamp_s=float(data.time),
            joint_position=np.array([values[item][0] for item in self.joint_position]),
            joint_velocity=np.array([values[item][0] for item in self.joint_velocity]),
            gyro_rad_s=values[self.gyro].copy(),
            accelerometer_m_s2=values[self.accelerometer].copy(),
        )


class SensorPolicyRuntime:
    """Reuse the old history layout while supplying only encoder/IMU observations."""

    def __init__(self, runtime_module, metadata, *, obs_delay_steps: int, leg_action_delay_steps: int) -> None:
        self.old = runtime_module
        self.metadata = metadata
        joint_dim = len(metadata.obs_joint_names)
        self.base_orientation_history = runtime_module.FrameStacker(2, metadata.history_length)
        self.base_ang_vel_history = runtime_module.FrameStacker(3, metadata.history_length)
        self.joint_pos_history = runtime_module.FrameStacker(joint_dim, metadata.history_length)
        self.joint_vel_history = runtime_module.FrameStacker(joint_dim, metadata.history_length)
        self.last_action_history = runtime_module.FrameStacker(metadata.action_dim, metadata.history_length)
        self.gait_history = runtime_module.FrameStacker(2, metadata.history_length)
        self.command_history = runtime_module.FrameStacker(len(metadata.command_scales), metadata.history_length)
        self.base_orientation_delay = runtime_module.FixedDelayBuffer(2, obs_delay_steps)
        self.base_ang_vel_delay = runtime_module.FixedDelayBuffer(3, obs_delay_steps)
        self.joint_pos_delay = runtime_module.FixedDelayBuffer(joint_dim, obs_delay_steps)
        self.joint_vel_delay = runtime_module.FixedDelayBuffer(joint_dim, obs_delay_steps)
        self.leg_action_delay = runtime_module.LegActionDelay(metadata.leg_action_dim, leg_action_delay_steps)
        self.last_action = np.zeros(metadata.action_dim, dtype=np.float32)

    def initialize(self, packet: SensorPacket, gravity_base: np.ndarray,
                   command_obs: np.ndarray, gait: np.ndarray) -> None:
        for delay in (self.base_orientation_delay, self.base_ang_vel_delay,
                      self.joint_pos_delay, self.joint_vel_delay, self.leg_action_delay):
            delay.reset()
        proprio = self._proprio(packet, gravity_base)
        terms = self._terms(self.last_action, command_obs, gait, proprio)
        for history, term in zip(self._histories(), terms, strict=True):
            history.reset(term)

    def update(self, packet: SensorPacket, gravity_base: np.ndarray, *, action: np.ndarray,
               command_obs: np.ndarray, gait: np.ndarray) -> None:
        self.last_action = np.asarray(action, dtype=np.float32).copy()
        terms = self._terms(self.last_action, command_obs, gait, self._proprio(packet, gravity_base))
        for history, term in zip(self._histories(), terms, strict=True):
            history.append(term)

    def policy_obs(self) -> np.ndarray:
        return np.concatenate(tuple(history.flattened() for history in self._histories()), axis=0).astype(np.float32)

    def _histories(self):
        return (self.base_orientation_history, self.base_ang_vel_history, self.joint_pos_history,
                self.joint_vel_history, self.last_action_history, self.gait_history, self.command_history)

    def _proprio(self, packet: SensorPacket, gravity_base: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        gravity = np.asarray(gravity_base, dtype=np.float64)
        if gravity.shape != (3,) or not np.isfinite(gravity).all():
            raise ValueError("gravity_base must be finite [x, y, z]")
        joint_default = np.asarray(self.metadata.obs_default_joint_pos, dtype=np.float64)
        return (
            gravity[:2].astype(np.float32),
            (packet.gyro_rad_s * self.metadata.base_ang_vel_scale).astype(np.float32),
            (packet.joint_position - joint_default).astype(np.float32),
            (packet.joint_velocity * self.metadata.joint_vel_scale).astype(np.float32),
        )

    def _terms(self, action: np.ndarray, command_obs: np.ndarray, gait: np.ndarray,
               proprio: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]):
        orientation, ang_vel, joint_pos, joint_vel = proprio
        return self.old.compute_policy_terms(
            data=None,
            last_action=action,
            command_obs=command_obs,
            delayed_base_orientation=self.base_orientation_delay.push(orientation),
            delayed_base_ang_vel=self.base_ang_vel_delay.push(ang_vel),
            delayed_joint_pos=self.joint_pos_delay.push(joint_pos),
            delayed_joint_vel=self.joint_vel_delay.push(joint_vel),
            metadata=self.metadata,
            gait_override=gait,
        )
