"""Native Isaac Lab A2 plant. IPC exposes only the selected camera and proprioception.

Run with the Isaac Lab Python environment; the navigation process owns the policy.
Ground truth and raw PhysX contacts go exclusively to offline output files.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import signal
import faulthandler
from multiprocessing.connection import Listener
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SIM_APP = None


def main():
    global SIM_APP
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', required=True)
    parser.add_argument('--usd-path')
    parser.add_argument('--disable-ujitso-cooking', action='store_true', help='Use the official GRScenes collision-cooking configuration.')
    parser.add_argument('--urdf', default='/home/baoquanc/workspace/LMP/resources/a2_description/urdf/a2.urdf')
    parser.add_argument('--spawn', type=float, nargs=4, default=[0., 0., .55, 0.], metavar=('X', 'Y', 'Z', 'YAW_DEG'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--sensor-mode', choices=('rgb', 'rgbd'), default='rgbd')
    parser.add_argument('--camera-profile', default='d435i_rgb_nominal')
    parser.add_argument('--camera-forward-m', type=float, help='Move the calibrated RGB/depth rig along base X for a mounting comparison.')
    parser.add_argument('--smoke', action='store_true', help='Bounded plant/camera bringup, no navigation.')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    startup_start = time.monotonic()

    from isaaclab.app import AppLauncher
    # Kit uses its Vulkan GPU table; CUDA_VISIBLE_DEVICES only remaps CUDA.
    # Set this from the UUID-matched Kit table when allocating another GPU.
    renderer_gpu = int(os.environ.get('ISAAC_RENDERER_GPU', '0'))
    launcher = AppLauncher(headless=True, enable_cameras=True, device='cuda:0', multi_gpu=False,
                           kit_args=f'--/renderer/activeGpu={renderer_gpu} --/renderer/multiGpu/enabled=false')
    app = launcher.app
    SIM_APP = app
    if args.disable_ujitso_cooking:
        import omni.physx.bindings._physx as physx_bindings
        app.set_setting(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, False)
        print('[startup] UJITSO collision cooking disabled per GRScenes loader', flush=True)
    import numpy as np
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.sensors import Camera, CameraCfg, Imu, ImuCfg, ContactSensor, ContactSensorCfg
    from isaaclab.sim.converters import UrdfConverterCfg
    from isaaclab.utils.math import matrix_from_quat

    torch.set_num_threads(1)
    metadata = json.loads((ROOT / '.local/assets/a2-old/policy.json').read_text())
    profile = json.loads((ROOT / 'prototype/camera/profiles.json').read_text())['profiles'][args.camera_profile]
    rgb_only = args.sensor_mode == 'rgb'
    if rgb_only and args.camera_profile != 'a2_head_isaaclab':
        raise ValueError('The RGB-only deployment path requires the A2 HEAD profile')
    if args.camera_forward_m is not None:
        profile['position_base_m'][0] = args.camera_forward_m
    native_depth_profile = None if rgb_only else profile['native_depth']
    action_names = metadata['joint_names']
    obs_names = metadata['obs_joint_names']
    limits = {j.attrib['name']: float(j.find('limit').attrib['effort'])
              for j in ET.parse(args.urdf).getroot().findall('joint') if j.attrib['type'] == 'revolute'}
    dt = metadata['sim_dt']
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device='cuda:0', render_interval=4))
    sim.carb_settings.set_bool('/physics/disableContactProcessing', False)
    if args.usd_path:
        cfg = sim_utils.UsdFileCfg(usd_path=args.usd_path)
        cfg.func('/World/Environment', cfg)
    else:
        cfg = sim_utils.CuboidCfg(size=(50., 50., .1), collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.6, .6, .6)))
        cfg.func('/World/Ground', cfg, translation=(0., 0., -.05))
    light = sim_utils.DomeLightCfg(intensity=1500.)
    light.func('/World/NavLight', light)
    yaw = math.radians(args.spawn[3])
    robot = Articulation(ArticulationCfg(
        prim_path='/World/A2',
        spawn=sim_utils.UrdfFileCfg(
            asset_path=args.urdf, usd_dir=str(output / 'converted_a2'), fix_base=False,
            merge_fixed_joints=False, self_collision=True, activate_contact_sensors=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(target_type='none',
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0., damping=0.)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(linear_damping=0., angular_damping=0.,
                max_linear_velocity=1000., max_angular_velocity=1000., max_depenetration_velocity=300.),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True,
                solver_position_iteration_count=4, solver_velocity_iteration_count=0)),
        init_state=ArticulationCfg.InitialStateCfg(pos=tuple(args.spawn[:3]),
            rot=(math.cos(yaw/2), 0., 0., math.sin(yaw/2)),
            joint_pos=dict(zip(action_names, metadata['default_joint_pos']))),
        actuators={'legs': ImplicitActuatorCfg(joint_names_expr=action_names,
            stiffness={n: metadata['joint_actuator_gains'][n][0] for n in action_names},
            damping={n: metadata['joint_actuator_gains'][n][1] for n in action_names},
            effort_limit_sim=limits, armature=.03,
            velocity_limit_sim={n: 14.6667 if 'calf' in n else 22. for n in action_names})},
    ))
    imu = Imu(ImuCfg(prim_path='/World/A2/base_link', update_period=dt, gravity_bias=(0., 0., 9.81)))
    contact_sensors = {link.attrib['name']: ContactSensor(ContactSensorCfg(
        prim_path='/World/A2/' + link.attrib['name'], update_period=dt))
        for link in ET.parse(args.urdf).getroot().findall('link')}
    pitch = math.radians(profile['pitch_down_deg'])
    focal = profile['height'] / (2 * math.tan(math.radians(profile['fovy_deg']) / 2))
    camera = Camera(CameraCfg(prim_path='/World/A2/base_link/NavCamera',
        width=profile['width'], height=profile['height'], update_period=0.,
        data_types=['rgb'] if rgb_only else ['rgb', 'distance_to_image_plane'],
        offset=CameraCfg.OffsetCfg(pos=tuple(profile['position_base_m']),
            rot=(math.cos(pitch/2), 0., math.sin(pitch/2), 0.), convention='world'),
        spawn=sim_utils.PinholeCameraCfg(focal_length=1.,
            horizontal_aperture=profile['width']/focal, vertical_aperture=profile['height']/focal,
            clipping_range=tuple(profile['clipping_range_m']))))
    camera_to_base = np.eye(4)
    camera_to_base[:3, :3] = np.array([[math.cos(pitch), 0., math.sin(pitch)], [0., 1., 0.],
        [-math.sin(pitch), 0., math.cos(pitch)]]) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    camera_to_base[:3, 3] = profile['position_base_m']
    # Separate native depth optics; zero offset is an explicit ideal simulation
    # calibration, not a measured D435i RGB-to-depth hardware extrinsic.
    depth_camera = None
    depth_camera_to_base = None
    if not rgb_only:
        depth_camera_to_base = camera_to_base.copy()
        depth_camera_to_base[:3, 3] += camera_to_base[:3, :3] @ np.asarray(native_depth_profile['optical_offset_m'])
        depth_camera = Camera(CameraCfg(prim_path='/World/A2/base_link/NavDepthCamera',
        width=native_depth_profile['width'], height=native_depth_profile['height'], update_period=0.,
        data_types=['distance_to_image_plane'],
        offset=CameraCfg.OffsetCfg(pos=tuple(float(v) for v in depth_camera_to_base[:3, 3]),
            rot=(math.cos(pitch/2), 0., math.sin(pitch/2), 0.), convention='world'),
        spawn=sim_utils.PinholeCameraCfg(focal_length=1.,
            horizontal_aperture=2 * math.tan(math.radians(native_depth_profile['hfov_deg']) / 2),
            clipping_range=tuple(profile['clipping_range_m']))))

    def depth_intrinsic_matrix():
        return depth_camera.data.intrinsic_matrices[0].cpu().numpy().copy()

    print(f'[startup] scene constructed; entering sim.reset at {time.monotonic() - startup_start:.1f}s', flush=True)
    sim.reset()
    print(f'[startup] sim.reset completed at {time.monotonic() - startup_start:.1f}s', flush=True)
    action_ids, _ = robot.find_joints(action_names, preserve_order=True)
    obs_ids, _ = robot.find_joints(obs_names, preserve_order=True)
    # sim.reset bootstraps PhysX before joint initialization; restore the setup
    # pose/velocity once here so that bootstrap contacts cannot launch the base.
    robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7])
    robot.write_root_velocity_to_sim(robot.data.default_root_state[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    robot.reset()
    imu.reset()
    stamp = 0.
    truth = (output / 'ground_truth.jsonl').open('w', buffering=65536)
    contacts = (output / 'contacts.jsonl').open('w', buffering=65536)

    def packet():
        return dict(stamp_s=stamp,
            joint_position=robot.data.joint_pos[0, obs_ids].cpu().numpy().copy(),
            joint_velocity=robot.data.joint_vel[0, obs_ids].cpu().numpy().copy(),
            gyro_rad_s=imu.data.ang_vel_b[0].cpu().numpy().copy(),
            accelerometer_m_s2=imu.data.lin_acc_b[0].cpu().numpy().copy())

    def offline():
        # This observer has no return value and never enters an IPC response.
        rotation = matrix_from_quat(robot.data.root_quat_w)[0].cpu().numpy()
        pos = robot.data.root_pos_w[0].cpu().numpy()
        velocity = robot.data.root_lin_vel_w[0].cpu().numpy()
        omega = robot.data.root_ang_vel_w[0].cpu().numpy()
        heading = math.atan2(rotation[1, 0], rotation[0, 0])
        c, s = math.cos(heading), math.sin(heading)
        forward = rotation[:, 0]
        derivative = np.cross(omega, forward)
        yaw_rate = (forward[0]*derivative[1]-forward[1]*derivative[0])/np.dot(forward[:2], forward[:2])
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, pos
        truth.write(json.dumps(dict(sim_time_s=stamp, pose_world=[float(pos[0]), float(pos[1]), heading],
            world_from_base=transform.tolist(), actual_velocity_body=(rotation.T @ velocity).tolist(),
            actual_planar_twist=[float(v) for v in (c*velocity[0]+s*velocity[1], -s*velocity[0]+c*velocity[1], yaw_rate)]))+'\n')

    def record_contacts():
        # GPU PhysX contacts are read through the tensor API. The event reporter
        # returned no events on this GPU scene, including known foot support.
        records = []
        for body, sensor in contact_sensors.items():
            sensor.update(dt, force_recompute=True)
            records.append(dict(body='/World/A2/' + body, is_foot=body.endswith('_foot'),
                net_normal_force_world_n=sensor.data.net_forces_w[0, 0].cpu().tolist()))
        contacts.write(json.dumps(dict(sim_time_s=stamp, dt_s=dt, observer='physx_tensor_contact_view',
            measurement='net_normal_force_all_contacts', bodies=records))+'\n')

    def tick(target):
        nonlocal stamp
        robot.set_joint_position_target(target, joint_ids=action_ids)
        robot.write_data_to_sim()
        sim.step(render=False)
        stamp += dt
        robot.update(dt)
        imu.update(dt, force_recompute=True)
        sample = packet()
        offline()
        record_contacts()
        return sample

    def capture(include_reference=False):
        # Both render products observe the same PhysX state. No physics step
        # occurs between their updates; stamp_s is their shared exposure time.
        for _ in range(2):
            sim.render()
        camera.update(dt, force_recompute=True)
        k = camera.data.intrinsic_matrices[0].cpu().numpy()
        frame = dict(rgb=camera.data.output['rgb'][0, :, :, :3].cpu().numpy().copy(),
            stamp_s=stamp, intrinsics=dict(fx=float(k[0,0]), fy=float(k[1,1]), cx=float(k[0,2]), cy=float(k[1,2])),
            camera_to_base=camera_to_base.copy(), sensor_mode=args.sensor_mode)
        if depth_camera is not None:
            depth_camera.update(dt, force_recompute=True)
            depth_k = depth_intrinsic_matrix()
            frame.update(depth_z_m=depth_camera.data.output['distance_to_image_plane'][0, :, :, 0].cpu().numpy().copy(),
                depth_intrinsics=dict(fx=float(depth_k[0,0]), fy=float(depth_k[1,1]), cx=float(depth_k[0,2]), cy=float(depth_k[1,2])),
                depth_camera_to_base=depth_camera_to_base.copy())
        if include_reference and not rgb_only:
            # Smoke archive only; production camera IPC never exposes this.
            frame['rgb_reference_depth_z_m'] = camera.data.output['distance_to_image_plane'][0, :, :, 0].cpu().numpy().copy()
        return frame

    def calibration(sensor, k):
        height, width = sensor.image_shape
        return dict(width=width, height=height, intrinsic_matrix=k.tolist(),
            hfov_deg=math.degrees(2 * math.atan(width / (2 * float(k[0, 0])))),
            fovy_deg=math.degrees(2 * math.atan(height / (2 * float(k[1, 1])))))

    target = torch.tensor(metadata['default_joint_pos'], device=sim.device, dtype=torch.float32).unsqueeze(0)
    samples = [tick(target) for _ in range(round(1.5/dt))]
    initialization = dict(samples[-1])
    for key in ('gyro_rad_s', 'accelerometer_m_s2'):
        initialization[key] = np.mean([p[key] for p in samples[-round(.5/dt):]], axis=0)
    camera_calibration = dict(rgb=calibration(camera, camera.data.intrinsic_matrices[0].cpu().numpy()),
        sensor_mode=args.sensor_mode, synchronization='Shared PhysX exposure; rendering does not advance physics')
    if not rgb_only:
        camera_calibration.update(depth=calibration(depth_camera, depth_intrinsic_matrix()),
            calibration_basis=native_depth_profile['calibration_basis'],
            renderer_model='Native RTX square-pixel pinhole; nominal full depth FOV without resampling',
            depth_intrinsics_source='IsaacLab CameraData.intrinsic_matrices',
            rgb_reference_depth_usage='Offline alignment verification only; never controller input')
    ready = dict(type='ready', latest_sensor_packet=samples[-1], imu_initialization=initialization,
        joint_names=action_names, obs_joint_names=obs_names, action_joint_ids=action_ids, obs_joint_ids=obs_ids,
        dt=dt, decimation=metadata['control_decimation'], stamp_s=stamp, camera_to_base=camera_to_base,
        sensor_mode=args.sensor_mode, camera_profile_name=args.camera_profile,
        camera_calibration=camera_calibration,
        camera_profile=profile, truth_output=str(output), spawn_xyz_yaw_deg=args.spawn,
        contact_observer=dict(measurement='net_normal_force_all_contacts',
            body_count=len(contact_sensors), frame='world', units='N',
            limitations='No counterpart collider identity, contact points, friction or individual normals; '
                        'includes self-contact; opposite normal forces may cancel. '
                        'Foot upward load is consistent with support, not proof of a named floor contact.'),
        backend='IsaacLab/PhysX/RTX', device=sim.device, renderer_gpu=renderer_gpu, multi_gpu=False, urdf=args.urdf,
        rendering=dict(backend='IsaacLab/PhysX/RTX', device=sim.device,
            cuda_device_name=torch.cuda.get_device_name(0),
            cuda_device_uuid=str(torch.cuda.get_device_properties(0).uuid),
            renderer_gpu=sim.carb_settings.get('/renderer/activeGpu'),
            multi_gpu=sim.carb_settings.get('/renderer/multiGpu/enabled')))
    if not rgb_only:
        ready.update(depth_camera_to_base=depth_camera_to_base, native_depth_profile=native_depth_profile)
    print('PLANT_READY', ready, flush=True)
    try:
        if args.smoke:
            frame = capture(include_reference=True)
            if rgb_only:
                np.savez(output / 'smoke_rgb.npz', rgb=frame['rgb'], stamp_s=stamp,
                    camera_to_base=camera_to_base, intrinsic_matrix=camera.data.intrinsic_matrices[0].cpu().numpy())
                (output / 'smoke_calibration.json').write_text(json.dumps(dict(
                    camera_profile=profile, camera_calibration=camera_calibration,
                    stamp_s=stamp, sensor_mode='rgb', depth_render_product_created=False,
                    intrinsics=frame['intrinsics'], camera_to_base=camera_to_base.tolist()), indent=2)+'\n')
                print('RGB_CAMERA_READY', frame['rgb'].shape, frame['intrinsics'], flush=True)
                return
            np.savez(output / 'smoke_rgbd.npz', **{k: frame[k] for k in
                ('rgb', 'depth_z_m', 'rgb_reference_depth_z_m', 'camera_to_base', 'depth_camera_to_base', 'stamp_s')},
                rgb_intrinsic_matrix=camera.data.intrinsic_matrices[0].cpu().numpy(),
                depth_intrinsic_matrix=depth_intrinsic_matrix())
            (output / 'smoke_calibration.json').write_text(json.dumps(dict(
                camera_profile=profile, native_depth_profile=native_depth_profile,
                camera_calibration=ready['camera_calibration'], stamp_s=frame['stamp_s'],
                intrinsics=frame['intrinsics'], depth_intrinsics=frame['depth_intrinsics'],
                camera_to_base=frame['camera_to_base'].tolist(),
                depth_camera_to_base=frame['depth_camera_to_base'].tolist()), indent=2) + '\n')
            print('CAMERA_READY', frame['rgb'].shape, frame['depth_z_m'].shape, frame['intrinsics'],
                  'depth_intrinsics', frame['depth_intrinsics'],
                  'finite_depth', int(np.isfinite(frame['depth_z_m']).sum()), flush=True)
        else:
            with Listener(args.socket, family='AF_UNIX') as listener, listener.accept() as connection:
                connection.send(ready)
                while app.is_running():
                    request = connection.recv()
                    if request['op'] == 'step':
                        target = torch.as_tensor(request['target_joint_positions'], dtype=torch.float32, device=sim.device).reshape(1, 12)
                        packets = [tick(target) for _ in range(metadata['control_decimation'])]
                        connection.send(dict(packets=packets, stamp_s=stamp))
                    elif request['op'] == 'camera':
                        connection.send(capture())
                    elif request['op'] == 'close':
                        # A close acknowledgement must mean the offline evidence
                        # is durable before the parent starts Kit shutdown.
                        truth.flush()
                        contacts.flush()
                        connection.send(dict(closed=True))
                        break
                    else:
                        raise ValueError(f"Unknown plant operation: {request['op']}")
    finally:
        truth.close()
        contacts.close()


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        # Kit's framework shutdown can terminate with exit 0 before Python
        # reports a pending exception. Preserve the original failure first.
        import os
        import sys
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    else:
        if SIM_APP is not None:
            SIM_APP.close(wait_for_replicator=False)
