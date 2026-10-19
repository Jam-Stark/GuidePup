"""Old A2 policy and a real MuJoCo dynamics/camera backend."""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import torch
from depth import DepthFrame, DepthImage

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/mujoco'))
sys.path.insert(0, str(ROOT / '.local/vendor/LightNav-0/mujoco_demo'))
sys.path.insert(0, str(ROOT / 'prototype'))
import runtime as old
from robot import MuJoCoSensorReader, RGBDLocalOdometry, SensorPacket, SensorPolicyRuntime


class A2Simulation:
    @property
    def stamp_s(self):
        return float(self.data.time)

    def __init__(self, scene: str, camera: str, render: bool = True,
                 spawn_pose: tuple[float, float, float] | None = None):
        torch.set_num_threads(1)
        assets = ROOT / '.local/assets/a2-old'
        self.metadata = old.UniFPPolicyMetadata.from_json_path(assets / 'policy.json')
        self.policy = old.load_torchscript_policy(assets / 'policy.pt')
        self.profile = json.loads((ROOT / 'prototype/camera/profiles.json').read_text())['profiles'][camera]
        root = ET.parse(assets / 'a2.xml').getroot()
        root.find('compiler').set('meshdir', str(assets / 'meshes'))
        root.find('option').set('timestep', str(self.metadata.sim_dt))
        root.find('option').set('integrator', 'implicitfast')
        world = root.find('worldbody')
        spawn_yaw = 0.
        if scene == 'procthor':
            from vln_mujoco.model import load_scene_xml, SPAWN
            environment = load_scene_xml()
            root.find('asset').extend(environment.find('asset'))
            root.find('default').extend(environment.find('default'))
            world.extend(environment.find('worldbody'))
            self.spawn = (*SPAWN[:2], 0.55)
        elif scene in ('elevator_lobby', 'room_elevator'):
            environment = ET.parse(ROOT / f'prototype/sim/scenes/{scene}.xml').getroot()
            root.find('asset').extend(environment.find('asset'))
            world.extend(environment.find('worldbody'))
            self.spawn = (2.3, -6.6, .55) if scene == 'room_elevator' else (2.8, -1.6, .55)
            spawn_yaw = math.radians(73. if scene == 'room_elevator' else 10.)
        elif scene == 'flat':
            self.spawn = (0., 0., 0.55)
            ET.SubElement(world, 'geom', name='floor', type='plane', size='20 20 .1',
                          rgba='.7 .7 .7 1', friction='1 .005 .0001')
            ET.SubElement(world, 'geom', name='red_goal', type='box', pos='5 0 .5',
                          size='.15 .5 .5', rgba='.8 .1 .1 1')
        else:
            raise ValueError(f'Unknown scene: {scene}')
        if spawn_pose is not None:
            spawn_x, spawn_y, spawn_yaw_deg = spawn_pose
            if not all(math.isfinite(value) for value in spawn_pose):
                raise ValueError('Spawn pose values must be finite.')
            self.spawn = (spawn_x, spawn_y, .55)
            spawn_yaw = math.radians(spawn_yaw_deg)
        self.spawn_yaw_deg = math.degrees(spawn_yaw)
        ET.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1', directional='true')
        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'global', offwidth=str(self.profile['width']), offheight=str(self.profile['height']))
        # Depth is deprojected along pixel-centre rays. MSAA depth resolves
        # silhouette subpixels into ghost geometry (observed at a doorway).
        ET.SubElement(visual, 'quality', shadowsize='2048', offsamples='0')
        pitch = math.radians(self.profile['pitch_down_deg'])
        ET.SubElement(world.find("body[@name='base_link']"), 'camera', name='nav_rgb',
                      pos=' '.join(map(str, self.profile['position_base_m'])),
                      xyaxes=f'0 -1 0 {math.sin(pitch)} 0 {math.cos(pitch)}',
                      fovy=str(self.profile['fovy_deg']))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        if 'clipping_range_m' in self.profile:
            near, far = self.profile['clipping_range_m']
            self.model.vis.map.znear = near / self.model.stat.extent
            self.model.vis.map.zfar = far / self.model.stat.extent
        self.data = mujoco.MjData(self.model)
        self.bindings = old.create_model_bindings(self.model, self.metadata)
        self.base_id = self.model.body('base_link').id
        self.contact_log = None
        self.sensor_log = None
        self.robot_geom = self.model.body_rootid[self.model.geom_bodyid] == self.base_id
        self.data.qpos[:3] = self.spawn
        self.data.qpos[3:7] = (math.cos(spawn_yaw / 2), 0., 0., math.sin(spawn_yaw / 2))
        self.data.qpos[self.bindings.qpos_indices] = self.metadata.default_joint_pos
        mujoco.mj_forward(self.model, self.data)
        self.sensor_reader = MuJoCoSensorReader(self.model, self.metadata)
        self.gains = np.asarray([self.metadata.joint_actuator_gains[n] for n in self.metadata.joint_names])
        gravity_initialization = self._hold_stationary_for_imu()
        self.imu_initialization = gravity_initialization.payload()
        self.latest_sensor_packet = self.sensor_reader.read(self.data)
        self.camera_to_base = self._profile_camera_to_base()
        self.odometry = RGBDLocalOdometry(self.camera_to_base, tuple(self.profile['depth_range_m']))
        self.runtime = SensorPolicyRuntime(old, self.metadata, obs_delay_steps=0, leg_action_delay_steps=0)
        self.runtime.initialize(
            self.latest_sensor_packet, -gravity_initialization.accelerometer_m_s2 / np.linalg.norm(gravity_initialization.accelerometer_m_s2),
            np.zeros(3, dtype=np.float32), np.array([0., 1.], dtype=np.float32))
        self.robot_state = None
        self.action = np.zeros(12, dtype=np.float32)
        _, self.aux = old.infer_policy_outputs(self.policy, self.runtime.policy_obs(), action_dim=12)
        self.odometry.initialize_imu(gravity_initialization, self.velocity_estimate())
        self.odometry.propagate_imu(self.latest_sensor_packet, self.velocity_estimate())
        self.phase = 0.
        self.force_walking = False
        self.renderer = mujoco.Renderer(self.model, height=self.profile['height'], width=self.profile['width']) if render else None
        self.render_options = mujoco.MjvOption()
        self.render_options.geomgroup[3:] = 0

    def record_contacts(self, output):
        """Diagnostic observer only; these data are never model or controller inputs."""
        self.contact_log = (output / 'contacts.jsonl').open('w', buffering=65536)
        geometry = []
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            mesh_id = int(self.model.geom_dataid[geom_id])
            geometry.append({
                'id': geom_id, 'name': self.model.geom(geom_id).name,
                'body': self.model.body(body_id).name, 'robot': bool(self.robot_geom[geom_id]),
                'type': int(self.model.geom_type[geom_id]),
                'mesh': self.model.mesh(mesh_id).name if self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH else None,
            })
        (output / 'contact_geometry.json').write_text(json.dumps(geometry, indent=2))

    def velocity_estimate(self):
        # Pinned old checkpoint: training-origin/params/env.yaml pred.base_lin_vel
        # uses scale=2.0; the exported decoder returns that scaled prediction.
        return np.asarray(self.aux[:3], dtype=np.float64) / 2.

    def record_sensors(self, output):
        self.sensor_log = (output / 'sensors.jsonl').open('w', buffering=65536)
        (output / 'estimator_initialization.json').write_text(json.dumps({
            'stationary_imu': self.imu_initialization,
            'velocity_estimate_body_m_s': self.velocity_estimate().tolist(),
            'velocity_source': 'a2_policy_aux_divided_by_2'}, indent=2))
        self._record_sensor_packet()

    def _record_sensor_packet(self):
        if self.sensor_log is not None:
            self.sensor_log.write(json.dumps({**self.latest_sensor_packet.payload(),
                'velocity_estimate_body_m_s': self.velocity_estimate().tolist(),
                'velocity_source': 'a2_policy_aux_divided_by_2'}) + '\n')

    def write_contacts(self, sample_time, qpos, qvel):
        contacts = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            first, second = int(contact.geom1), int(contact.geom2)
            if self.robot_geom[first] == self.robot_geom[second]:
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            contacts.append({'geoms': [first, second], 'position_world_m': contact.pos.tolist(),
                             'distance_m': float(contact.dist), 'frame': contact.frame.tolist(),
                             'force_torque_contact_frame': force.tolist()})
        if contacts:
            self.contact_log.write(json.dumps({'sim_time_s': sample_time, 'qpos': qpos.tolist(),
                                               'qvel': qvel.tolist(), 'contacts': contacts}) + '\n')

    def truth_pose(self):
        return np.array([*self.data.qpos[:2], old.yaw_from_quat(self.data.qpos[3:7])])

    def truth_velocity(self):
        return old.quat_inverse_rotate(self.data.qpos[3:7], self.data.qvel[:3])

    def truth_planar_twist(self):
        """Actual [vx,vy,yaw_rate] in the base-yaw plane, not XYZ linear velocity."""
        yaw = self.truth_pose()[2]
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = self.data.qvel[:2]
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        # MuJoCo free-joint angular velocity is body-local. Differentiate the
        # world heading vector so body roll/pitch do not masquerade as yaw.
        forward = rotation[:, 0]
        derivative = np.cross(rotation @ self.data.qvel[3:6], forward)
        yaw_rate = (forward[0] * derivative[1] - forward[1] * derivative[0]) / np.dot(forward[:2], forward[:2])
        return np.array([c * vx + s * vy, -s * vx + c * vy, yaw_rate])

    def _hold_stationary_for_imu(self):
        """Plant-only default-joint hold before gravity is initialized from the IMU."""
        target = np.asarray(self.metadata.default_joint_pos, dtype=np.float64)
        gyro_samples, acceleration_samples = [], []
        for index in range(int(round(1.5 / self.metadata.sim_dt))):
            joint_position = self.data.qpos[self.bindings.qpos_indices]
            joint_velocity = self.data.qvel[self.bindings.qvel_indices]
            self.data.ctrl[self.bindings.actuator_indices] = (
                self.gains[:, 0] * (target - joint_position) - self.gains[:, 1] * joint_velocity)
            mujoco.mj_step(self.model, self.data)
            packet = self.sensor_reader.read(self.data)
            # Use the final stationary half-second, excluding the initial drop.
            if index * self.metadata.sim_dt >= 1.0:
                gyro_samples.append(packet.gyro_rad_s)
                acceleration_samples.append(packet.accelerometer_m_s2)
        mujoco.mj_forward(self.model, self.data)
        latest = self.sensor_reader.read(self.data)
        return SensorPacket(
            stamp_s=latest.stamp_s, joint_position=latest.joint_position, joint_velocity=latest.joint_velocity,
            gyro_rad_s=np.mean(gyro_samples, axis=0), accelerometer_m_s2=np.mean(acceleration_samples, axis=0))

    def step(self, requested, paused=False, force_local=(0., 0., 0.)):
        command = np.zeros(3, dtype=np.float32) if paused else np.asarray(requested, dtype=np.float32)
        standing = np.linalg.norm(command[:2]) < .1 and abs(command[2]) < .1
        if paused:
            self.force_walking = False
        elif standing:
            magnitude = np.linalg.norm(self.aux[3:5])
            self.force_walking = magnitude >= (.05 if self.force_walking else .2)
        else:
            self.force_walking = False
        walking = not paused and (not standing or self.force_walking)
        self.phase = (self.phase + self.metadata.control_dt * self.metadata.gait_frequency_hz) % 1. if walking else 0.
        gait = np.array([math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)], dtype=np.float32)
        self.latest_sensor_packet = self.sensor_reader.read(self.data)
        self.odometry.propagate_imu(self.latest_sensor_packet, self.velocity_estimate())
        self.runtime.update(
            self.latest_sensor_packet, self.odometry.gravity_base, action=self.action,
            command_obs=old.build_command_vector(locomotion_cmd=command, metadata=self.metadata), gait=gait)
        self.action, self.aux = old.infer_policy_outputs(self.policy, self.runtime.policy_obs(), action_dim=12)
        if self.aux is None or self.aux.shape != (6,):
            raise RuntimeError('Prototype requires the selected old 6D A2 export (velocity + base force).')
        self.odometry.propagate_imu(self.latest_sensor_packet, self.velocity_estimate())
        self.action = np.clip(self.action, -self.metadata.action_clip, self.metadata.action_clip)
        target = old.build_target_joint_positions(self.action, self.runtime, self.metadata)
        for _ in range(self.metadata.control_decimation):
            q = self.data.qpos[self.bindings.qpos_indices]
            dq = self.data.qvel[self.bindings.qvel_indices]
            # A2 XML uses torque motors. Position targets are not motor controls.
            torque = self.gains[:, 0] * (target - q) - self.gains[:, 1] * dq
            self.data.ctrl[self.bindings.actuator_indices] = torque
            self.data.xfrc_applied[self.base_id, :3] = old.base_yaw_to_world(np.asarray(force_local), self.data.qpos[3:7])
            if self.contact_log is not None:
                # mj_step computes contact forces before integrating the state.
                sample_time, qpos, qvel = float(self.data.time), self.data.qpos.copy(), self.data.qvel.copy()
            mujoco.mj_step(self.model, self.data)
            if self.contact_log is not None:
                self.write_contacts(sample_time, qpos, qvel)
            # Refresh sensor values at the new timestamp and consume every IMU
            # sample, not just the last of four physics ticks in a policy step.
            mujoco.mj_forward(self.model, self.data)
            self.latest_sensor_packet = self.sensor_reader.read(self.data)
            self.odometry.propagate_imu(self.latest_sensor_packet, self.velocity_estimate())
            self._record_sensor_packet()
        return command

    def truth_world_from_base(self):
        transform = np.eye(4)
        transform[:3, :3] = self.data.xmat[self.base_id].reshape(3, 3)
        transform[:3, 3] = self.data.xpos[self.base_id]
        return transform

    def _profile_camera_to_base(self):
        """Fixed D435i optical-to-base calibration, independent of MuJoCo camera truth."""
        pitch = math.radians(self.profile['pitch_down_deg'])
        base_from_optical_nominal = np.array([
            [0., 0., 1.],
            [-1., 0., 0.],
            [0., -1., 0.],
        ])
        pitch_down = np.array([
            [math.cos(pitch), 0., math.sin(pitch)],
            [0., 1., 0.],
            [-math.sin(pitch), 0., math.cos(pitch)],
        ])
        transform = np.eye(4)
        transform[:3, :3] = pitch_down @ base_from_optical_nominal
        transform[:3, 3] = np.asarray(self.profile['position_base_m'], dtype=np.float64)
        return transform

    def capture(self, frame_id):
        """Ideal aligned RGB-D from one unchanged MuJoCo camera/physics state."""
        self.renderer.update_scene(self.data, camera='nav_rgb', scene_option=self.render_options)
        pixels = self.renderer.render()
        self.renderer.enable_depth_rendering()
        depth_z_m = self.renderer.render()
        self.renderer.disable_depth_rendering()
        height, width = pixels.shape[:2]
        focal = height / (2 * math.tan(math.radians(self.profile['fovy_deg']) / 2))
        self.robot_state = self.odometry.update(
            stamp_s=self.latest_sensor_packet.stamp_s, depth_z_m=depth_z_m,
            intrinsics={'fx': focal, 'fy': focal, 'cx': (width - 1) / 2, 'cy': (height - 1) / 2})
        geometry = DepthImage(frame_id, self.robot_state, depth_z_m,
            {'fx': focal, 'fy': focal, 'cx': (width-1)/2, 'cy': (height-1)/2},
            self.camera_to_base, tuple(self.profile['depth_range_m']))
        return DepthFrame(
            geometry=geometry, frame_id=frame_id, state=self.robot_state, rgb=pixels, depth_z_m=depth_z_m,
            intrinsics={'fx': focal, 'fy': focal, 'cx': (width - 1) / 2, 'cy': (height - 1) / 2},
            camera_to_base=self.camera_to_base,
            valid_range_m=tuple(self.profile['depth_range_m']))

    def close(self):
        if self.sensor_log is not None:
            self.sensor_log.close()
        if self.contact_log is not None:
            self.contact_log.close()
        if self.renderer:
            self.renderer.close()
