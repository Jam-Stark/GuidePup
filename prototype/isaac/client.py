"""Isaac policy adapter, with explicit RGB-only or historical RGB-D sensor paths."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from multiprocessing.connection import Client
from pathlib import Path

import numpy as np
import torch

from a2 import ROOT, old
from depth import DepthFrame, DepthImage
from camera.alignment import align_depth_to_rgb
from robot import RGBDLocalOdometry, SensorPacket, SensorPolicyRuntime
from robot.proprio_odometry import IMUProprioOdometry
from robot.force_stop import ForceStop


@dataclass(frozen=True)
class RGBFrame:
    frame_id: str
    state: object
    rgb: np.ndarray
    intrinsics: dict
    camera_to_base: np.ndarray


class IsaacA2Simulation:
    def __init__(self, socket_path: str, camera: str):
        if camera not in ('d435i_rgb_nominal', 'a2_head_isaaclab'):
            raise ValueError('Unsupported Isaac camera profile')
        self.rgb_only = camera == 'a2_head_isaaclab'
        torch.set_num_threads(1)
        self.connection = Client(socket_path, family='AF_UNIX')
        self.ready = self.connection.recv()
        if self.ready['type'] != 'ready':
            raise RuntimeError(f'Isaac plant did not initialize: {self.ready}')
        expected_mode = 'rgb' if self.rgb_only else 'rgbd'
        if self.ready['sensor_mode'] != expected_mode or self.ready['camera_profile_name'] != camera:
            raise RuntimeError('Selected navigation sensors do not match the actual plant camera')
        assets = ROOT / '.local/assets/a2-old'
        self.metadata = old.UniFPPolicyMetadata.from_json_path(assets / 'policy.json')
        self.force_scale = float(json.loads((assets/'policy.json').read_text())['base_force_scale'])
        self.policy = old.load_torchscript_policy(assets / 'policy.pt')
        self.profile = self.ready['camera_profile']
        self.camera_to_base = np.asarray(self.ready['camera_to_base'], dtype=np.float64)
        if not self.rgb_only:
            self.depth_camera_to_base = np.asarray(self.ready['depth_camera_to_base'], dtype=np.float64)
        self.spawn = tuple(self.ready['spawn_xyz_yaw_deg'][:3])
        self.spawn_yaw_deg = self.ready['spawn_xyz_yaw_deg'][3]
        self.stamp_s = float(self.ready['stamp_s'])
        self.latest_sensor_packet = SensorPacket(**self.ready['latest_sensor_packet'])
        initial = SensorPacket(**self.ready['imu_initialization'])
        self.imu_initialization = initial.payload()
        self.runtime = SensorPolicyRuntime(old, self.metadata, obs_delay_steps=0, leg_action_delay_steps=0)
        self.runtime.initialize(self.latest_sensor_packet,
            -initial.accelerometer_m_s2 / np.linalg.norm(initial.accelerometer_m_s2),
            np.zeros(3, dtype=np.float32), np.array([0., 1.], dtype=np.float32))
        self.action = np.zeros(12, dtype=np.float32)
        _, self.aux = old.infer_policy_outputs(self.policy, self.runtime.policy_obs(), action_dim=12)
        if self.aux is None or self.aux.shape != (6,):
            raise RuntimeError('The A2 policy must provide the original 6D auxiliary estimate')
        if self.rgb_only:
            self.odometry = IMUProprioOdometry(initial, self.velocity_estimate(), sensor_period_s=self.metadata.sim_dt)
        else:
            self.odometry = RGBDLocalOdometry(self.depth_camera_to_base, tuple(self.profile['depth_range_m']))
            self.odometry.initialize_imu(initial, self.velocity_estimate())
        self.robot_state = None
        self._propagate(self.latest_sensor_packet)
        self.sensor_log = None
        self.phase = 0.
        self.force_walking = False
        self.aux_stamp_s = self.latest_sensor_packet.stamp_s
        self.force_stop = None
        self.last_step = None

    def enable_force_stop(self):
        self.force_stop = ForceStop(self.force_scale)

    def resume_force_stop(self, reason):
        # Releases only the estimator stop. The caller's explicit paused=True
        # (including the historical human-owned brake path) remains authoritative.
        if self.force_stop is None:
            raise RuntimeError('External-force stop is not enabled')
        return self.force_stop.resume(reason)

    def request(self, **message):
        self.connection.send(message)
        return self.connection.recv()

    def velocity_estimate(self):
        return np.asarray(self.aux[:3], dtype=np.float64) / 2.

    def _propagate(self, packet):
        if self.rgb_only:
            self.robot_state = self.odometry.update(packet, self.velocity_estimate())
        else:
            self.odometry.propagate_imu(packet, self.velocity_estimate())

    def record_sensors(self, output):
        self.sensor_log = (output / 'sensors.jsonl').open('w', buffering=65536)
        (output / 'estimator_initialization.json').write_text(json.dumps({
            'stationary_imu': self.imu_initialization,
            'velocity_estimate_body_m_s': self.velocity_estimate().tolist(),
            'velocity_source': 'a2_policy_aux_divided_by_2',
            'force_aux_scale': self.force_scale}, indent=2))
        self._record_sensor_packet()

    def record_offline(self, output):
        # Only expose locations for the later evaluator; navigation never reads them.
        source = Path(self.ready['truth_output'])
        for name in ('ground_truth.jsonl', 'contacts.jsonl'):
            (output / name).symlink_to(source / name)

    def _record_sensor_packet(self):
        if self.sensor_log is not None:
            self.sensor_log.write(json.dumps({**self.latest_sensor_packet.payload(),
                'velocity_estimate_body_m_s': self.velocity_estimate().tolist(),
                'velocity_source': 'a2_policy_aux_divided_by_2',
                'aux_raw': self.aux.tolist(), 'aux_input_sensor_time_s': self.aux_stamp_s,
                'force_estimate_body_yaw_n': (self.aux[3:6]/self.force_scale).tolist()}) + '\n')

    def step(self, requested, paused=False, force_local=(0., 0., 0.), *, allow_force_walking=True, explicit_motion=False):
        if np.any(np.asarray(force_local) != 0):
            raise ValueError('The Isaac navigation experiment has no injected force intervention')
        if self.force_stop is not None:
            self.force_stop.observe(self.aux[3:6], self.aux_stamp_s, self.stamp_s)
            paused = paused or self.force_stop.latched
        command = np.zeros(3, dtype=np.float32) if paused else np.asarray(requested, dtype=np.float32)
        standing = np.linalg.norm(command[:2]) < .1 and abs(command[2]) < .1
        if paused or not allow_force_walking:
            self.force_walking = False
        elif standing:
            magnitude = np.linalg.norm(self.aux[3:5])
            self.force_walking = magnitude >= (.05 if self.force_walking else .2)
        else:
            self.force_walking = False
        # Bounded observation control can request a yaw below the legacy 0.1
        # deadband. Its nonzero command still needs gait; standing authority
        # disables this path as soon as the accepted command reaches zero.
        low_speed_motion = explicit_motion and np.max(np.abs(command)) > 1e-8
        walking = not paused and (not standing or self.force_walking or low_speed_motion)
        self.phase = (self.phase + self.metadata.control_dt * self.metadata.gait_frequency_hz) % 1. if walking else 0.
        gait = np.array([math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)], dtype=np.float32)
        self._propagate(self.latest_sensor_packet)
        self.runtime.update(self.latest_sensor_packet, self.odometry.gravity_base, action=self.action,
            command_obs=old.build_command_vector(locomotion_cmd=command, metadata=self.metadata), gait=gait)
        self.action, self.aux = old.infer_policy_outputs(self.policy, self.runtime.policy_obs(), action_dim=12)
        self.aux_stamp_s = self.latest_sensor_packet.stamp_s
        if self.aux is None or self.aux.shape != (6,) or not np.isfinite(self.aux).all():
            raise RuntimeError('Isaac deployment requires the original 6D A2 policy export')
        self._propagate(self.latest_sensor_packet)
        self.action = np.clip(self.action, -self.metadata.action_clip, self.metadata.action_clip)
        target = old.build_target_joint_positions(self.action, self.runtime, self.metadata)
        result = self.request(op='step', target_joint_positions=target.tolist())
        for packet in result['packets']:
            self.latest_sensor_packet = SensorPacket(**packet)
            self._propagate(self.latest_sensor_packet)
            self._record_sensor_packet()
        self.stamp_s = float(result['stamp_s'])
        self.last_step = dict(paused=bool(paused), walking=bool(walking), force_walking=bool(self.force_walking),
            force_walking_authorized=bool(allow_force_walking),
            explicit_motion_authorized=bool(explicit_motion),
            standing_hold=bool(not paused and not allow_force_walking and not walking),
            gait_phase=self.phase, gait=gait.tolist(), aux_input_sensor_time_s=self.aux_stamp_s,
            aux_raw=self.aux.tolist(), force_estimate_body_yaw_n=(self.aux[3:6]/self.force_scale).tolist(),
            force_stop=None if self.force_stop is None else self.force_stop.payload())
        return command

    def capture(self, frame_id):
        frame = self.request(op='camera')
        if abs(frame['stamp_s'] - self.latest_sensor_packet.stamp_s) > 1e-6:
            raise RuntimeError('Isaac camera and IMU timestamps do not match')
        if self.rgb_only:
            if frame['sensor_mode'] != 'rgb' or any('depth' in key for key in frame):
                raise RuntimeError('RGB-only IPC unexpectedly exposed depth')
            return RGBFrame(frame_id, self.robot_state, frame['rgb'], frame['intrinsics'], self.camera_to_base)
        self.robot_state = self.odometry.update(stamp_s=frame['stamp_s'],
            depth_z_m=frame['depth_z_m'], intrinsics=frame['depth_intrinsics'])
        valid_range = tuple(self.profile['depth_range_m'])
        geometry = DepthImage(frame_id, self.robot_state, frame['depth_z_m'],
            frame['depth_intrinsics'], frame['depth_camera_to_base'], valid_range)
        aligned_depth = align_depth_to_rgb(frame['depth_z_m'], frame['depth_intrinsics'],
            frame['depth_camera_to_base'], frame['intrinsics'], frame['camera_to_base'],
            frame['rgb'].shape[:2], valid_range)
        return DepthFrame(frame_id=frame_id, state=self.robot_state, rgb=frame['rgb'],
            depth_z_m=aligned_depth, intrinsics=frame['intrinsics'],
            camera_to_base=self.camera_to_base, valid_range_m=valid_range, geometry=geometry)

    def close(self):
        if self.sensor_log is not None:
            self.sensor_log.close()
        self.request(op='close')
        self.connection.close()
