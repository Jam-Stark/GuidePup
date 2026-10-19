"""Sensor-state A2 navigation over MuJoCo or the native Isaac plant."""
from __future__ import annotations

import argparse
import asyncio
import base64
from collections import OrderedDict
from copy import copy
import io
import json
import os
import sys
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

import aiohttp
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gpu import configure_cuda, configure_gpu
GPU_BINDING = configure_cuda()

from a2 import A2Simulation, ROOT
from depth_map import DepthObstacleMap
from motion_segment import MotionSegment
from semantic_depth import YOLOESemanticDepth
from truth import TruthRecorder

sys.path.insert(0, str(ROOT / 'prototype'))
from control import Nav2MPPI, LocalControlBlocked


def project_body_to_odom(waypoints, pose):
    """LightNav cumulative [forward,left,yaw] at the RGB exposure -> shared odom."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    result = waypoints.copy()
    result[:, :2] = waypoints[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
    result[:, 2] += pose[2]
    return result


def controller_diagnostics_for_status(diagnostics):
    """Keep Qwen/status concise; complete candidate evidence is in the native journal."""
    public = {key: value for key, value in diagnostics.items() if key != 'candidate_evaluations'}
    if 'candidate_summary' in public:
        public['candidate_summary'] = {key: value for key, value in public['candidate_summary'].items()
                                       if key != 'examples'}
    if public.get('final_check') is not None:
        public['final_check'] = {key: value for key, value in public['final_check'].items()
                                 if key not in ('controls', 'knots', 'samples', 'trace')}
    return public


class Prototype:
    def __init__(self, args):
        self.args = args
        if args.backend == 'isaac':
            from isaac.client import IsaacA2Simulation
            self.sim = IsaacA2Simulation(args.isaac_socket, args.camera)
            rendering = self.sim.ready['rendering']
            gpu_binding = GPU_BINDING
        else:
            from OpenGL import GL
            gpu_binding = configure_gpu()
            self.sim = A2Simulation(args.scene, args.camera,
                                    spawn_pose=None if args.spawn_pose is None else tuple(args.spawn_pose))
            rendering = {'vendor': GL.glGetString(GL.GL_VENDOR).decode(),
                         'renderer': GL.glGetString(GL.GL_RENDERER).decode(),
                         'version': GL.glGetString(GL.GL_VERSION).decode(),
                         'egl_device_id': os.environ['MUJOCO_EGL_DEVICE_ID'],
                         'offsamples': int(self.sim.model.vis.quality.offsamples)}
        self.state = 'PAUSED' if args.backend == 'isaac' else 'IDLE'
        self.task_id = None
        self.instruction = ''
        self.semantic_instruction = ''
        self.reference_source = 'lightnav'
        self.reason = ''
        self.generation = 0
        self.instance_id = str(uuid.uuid4())
        self.monitor_id = None
        self.monitor_deadline = 0.
        self.motion_owner = None
        self.requested = np.zeros(3)
        self.effective = np.zeros(3)
        self.path = None
        self.path_provenance = None
        self.force = np.zeros(3)
        self.force_until = 0.
        self.brake_ticks = 0
        self.started_wall = time.monotonic()
        self.frame_sequence = 0
        self.frames = OrderedDict()
        self.semantic_frames = OrderedDict()
        self.semantic_lock = asyncio.Lock()
        self.semantic_detector = YOLOESemanticDepth(args.semantic_checkpoint, 'cuda:0', classes=args.semantic_classes)
        self.capture_wakeup = asyncio.Event()
        self.capture()
        self.next_capture_time = self.capture_time + self.sim.profile['update_period_s']
        self.latency_ms = None
        self.control_wall_ms = 0.
        self.path_capture_time = None
        self.stop_candidate = False
        self.depth_map = DepthObstacleMap()
        self.obstacles_odom = np.empty((0, 2))
        self.map_capture_time = None
        self.controller_status = None
        self.controller_execution_delay_s = .20
        self.output = ROOT / '.local/outputs' / time.strftime(f'{args.backend}_sim_%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True)
        self.controller = Nav2MPPI(max_speed=args.max_speed, footprint_path=args.controller_footprint,
                                   log_path=self.output / 'nav2_mppi.log')
        self.frame_output = self.output / 'frames'
        self.frame_output.mkdir()
        self.navigation_frame_output = self.output / 'navigation_frames'
        if args.record_navigation_inputs:
            self.navigation_frame_output.mkdir()
        if args.backend == 'isaac':
            self.sim.record_offline(self.output)
        elif args.record_contacts:
            self.sim.record_contacts(self.output)
        self.log = (self.output / 'telemetry.jsonl').open('w')
        self.nav_log = (self.output / 'lightnav.jsonl').open('w', buffering=1)
        self.controller_log = (self.output / 'local_controller.jsonl').open('w', buffering=1)
        self.truth = TruthRecorder(self.sim, self.output) if args.backend == 'mujoco' else None
        self.sim.record_sensors(self.output)
        (self.output / 'configuration.json').write_text(json.dumps({
            **vars(args), 'camera_profile': self.sim.profile,
            'sensor_calibration': self.sim.ready['camera_calibration'] if args.backend == 'isaac' else {'type': 'coincident_mujoco_rgb_depth'},
            'rendering': rendering,
            'cuda_visible_devices': os.environ['CUDA_VISIBLE_DEVICES'],
            'gpu_binding': gpu_binding,
            'contact_observer': self.sim.ready['contact_observer'] if args.backend == 'isaac'
                                else {'measurement': 'MuJoCo per-contact force and geometry pair'},
            'semantic_depth': {'model': self.semantic_detector.checkpoint,
                               'classes': self.semantic_detector.classes,
                               'device': self.semantic_detector.device,
                               'confidence_threshold': self.semantic_detector.confidence},
            'spawn_pose': {
                'x_m': self.sim.spawn[0], 'y_m': self.sim.spawn[1], 'z_m': self.sim.spawn[2],
                'yaw_deg': self.sim.spawn_yaw_deg,
            },
            'observation': 'LightNav: raw RGB; Pi: RGB + YOLOE same-frame instance masks/labels and mask-restricted depth measurements, no scene target truth',
            'depth_source': ('Isaac RTX distance_to_image_plane' if args.backend == 'isaac'
                             else 'MuJoCo RGB-aligned optical-axis depth'),
            'controller_backend': 'nav2_mppi',
            'controller_native': self.controller.native_info,
            'controller_parameters': self.controller.config,
            'controller_footprint': json.loads(Path(args.controller_footprint).read_text()),
            'controller_stopping_model': json.loads((ROOT / 'prototype/control/stop_model.json').read_text()),
        }, indent=2))
        self.nav_task = None
        self.nav_wakeup = asyncio.Event()
        self.navigation_scope = None
        self.navigation_session_id = None
        self.navigation_session_scope = None
        self.navigation_input_frames = 0
        self.navigation_input_interval_s = None
        self.search_anchors = []
        self.search_reported_stamp = -np.inf
        self.search_progress = None
        self.lightnav_latest = None
        self.lightnav_hint = None
        self.maneuver = None
        self.turn_degrees = None
        self.advance_m = None
        self.goal_point_odom = None
        self.goal_heading_rad = None
        self.position_tolerance_m = .08
        self.heading_tolerance_deg = 8.
        self.goal_phase = None
        self.goal_phase_revision = 0
        self.goal_position_reached = False
        self.goal_position_reached_pose_odom = None
        self.motion_segment = None
        self.event_id = 0
        self.listeners = set()
        self.progress_pose = self.sim.robot_state.pose_odom
        self.progress_time = float(self.sim.stamp_s)

    def capture(self):
        self.frame_sequence += 1
        frame_id = f'{int(self.started_wall * 1000)}-{self.frame_sequence}'
        self.depth_frame = self.sim.capture(frame_id)
        self.capture_pose = np.asarray(self.depth_frame.pose_odom)
        self.capture_time = self.depth_frame.stamp_s
        output = io.BytesIO()
        Image.fromarray(self.depth_frame.rgb).save(output, format='JPEG', quality=90)
        self.jpeg = output.getvalue()
        self.capture_wakeup.set()

    def archive_frame(self, output, frame):
        np.savez_compressed(output / f'{frame.frame_id}.npz',
                            frame_id=frame.frame_id, stamp_s=frame.stamp_s,
                            robot_state_json=json.dumps(frame.state.payload()), rgb=frame.rgb, depth_z_m=frame.depth_z_m,
                            intrinsics=np.array([frame.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy')]),
                            camera_to_base=frame.camera_to_base,
                            native_depth_z_m=frame.geometry.depth_z_m,
                            native_depth_intrinsics=np.array([frame.geometry.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy')]),
                            native_depth_camera_to_base=frame.geometry.camera_to_base)

    async def scene(self):
        async with self.semantic_lock:
            frame = self.depth_frame
            hint = self.lightnav_hint
            archive = frame.frame_id not in self.frames
            if archive:
                semantic = await asyncio.to_thread(self.semantic_detector.predict, frame)
                self.frames[frame.frame_id] = frame
                self.semantic_frames[frame.frame_id] = semantic
                await asyncio.to_thread(self.archive_frame, self.frame_output, frame)
            self.frames.move_to_end(frame.frame_id)
            self.semantic_frames.move_to_end(frame.frame_id)
            if len(self.frames) > 8:
                evicted, _ = self.frames.popitem(last=False)
                del self.semantic_frames[evicted]
            semantic_payload = self.semantic_frames[frame.frame_id].payload()
            semantic_images = semantic_payload.pop('images')
            if archive:
                (self.frame_output / f'{frame.frame_id}.semantic.json').write_text(
                    json.dumps(semantic_payload, ensure_ascii=False, indent=2))
                self.semantic_frames[frame.frame_id].save_masks(
                    self.frame_output / f'{frame.frame_id}.semantic_masks.npz')
                for packet in semantic_images:
                    (self.frame_output / f'{frame.frame_id}.semantic.png').write_bytes(
                        base64.b64decode(packet['image']['data']))
            payload = frame.scene_payload(self.sim.profile)
            payload['semantic_depth'] = semantic_payload
            payload['images'].extend(semantic_images)
        payload['lightnav_hint'] = None if hint is None else {
            **hint, 'current_frame_id': frame.frame_id,
            'age_s': frame.stamp_s - hint['source_sim_time_s'],
            **frame.project_odom_point(hint['odom_point_m']),
        }
        return payload

    def measure_semantic_instance(self, frame_id, instance_id):
        if frame_id not in self.semantic_frames:
            raise HTTPException(409, 'Semantic RGB-D frame is not cached. Read scene, then select its exact frame_id and instance_id.')
        try:
            return self.semantic_frames[frame_id].measurement(instance_id)
        except KeyError as error:
            raise HTTPException(400, 'Unknown semantic instance in this frame.') from error

    def measure_region(self, frame_id, bbox_px):
        if frame_id not in self.frames:
            raise HTTPException(409, 'RGB-D frame is not cached. Read scene, then measure its exact frame_id.')
        try:
            frame = self.frames[frame_id]
            return frame.measurement_payload(bbox_px)
        except ValueError as error:
            raise HTTPException(400, str(error)) from error

    async def perception(self):
        try:
            while True:
                started = time.monotonic()
                frame = self.depth_frame
                if frame.state.tracking == 'initializing':
                    await asyncio.sleep(.02)
                    continue
                if frame.frame_id == self.depth_map.frame_id:
                    await asyncio.sleep(.02)
                    continue
                self.obstacles_odom = await asyncio.to_thread(self.depth_map.update, frame.geometry)
                self.map_capture_time = frame.stamp_s
                await asyncio.sleep(max(0., .1 - (time.monotonic() - started)))
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.archive_frame(self.frame_output, frame)
            self.fail(error)
            raise

    def publish(self):
        if self.motion_segment is not None:
            self.motion_segment.event(self.sim.stamp_s, self.state, self.reason)
        self.event_id += 1
        event = self.status()
        for queue in self.listeners:
            queue.put_nowait(event)

    async def events(self):
        queue = asyncio.Queue()
        self.listeners.add(queue)
        try:
            yield f'data: {json.dumps(self.status())}\n\n'
            while True:
                try:
                    event = await asyncio.wait_for(queue.get(), timeout=1.)
                except asyncio.TimeoutError:
                    # Telemetry keeps the stream live without inventing a transition.
                    event = self.status()
                yield f'data: {json.dumps(event)}\n\n'
        finally:
            self.listeners.remove(queue)

    def reset_progress(self):
        self.progress_pose = self.sim.robot_state.pose_odom
        self.progress_time = float(self.sim.stamp_s)

    def update_search_progress(self):
        """Report local odometry loops; this does not establish target absence."""
        state = self.sim.robot_state
        pose, stamp = state.pose_odom, state.stamp_s
        if self.search_anchors and stamp - self.search_anchors[-1]['stamp_s'] < .5:
            return
        previous = self.search_anchors[-1] if self.search_anchors else None
        path_m = 0. if previous is None else previous['path_m'] + float(
            np.linalg.norm(pose[:2] - np.asarray(previous['pose_odom'])[:2]))
        yaw_rad = 0. if previous is None else previous['yaw_rad'] + abs(float(
            np.arctan2(np.sin(pose[2] - previous['pose_odom'][2]),
                       np.cos(pose[2] - previous['pose_odom'][2]))))
        for anchor in self.search_anchors:
            if anchor['stamp_s'] <= self.search_reported_stamp:
                continue
            distance = float(np.linalg.norm(pose[:2] - np.asarray(anchor['pose_odom'])[:2]))
            anchor['excursion_m'] = max(anchor['excursion_m'], distance)
            yaw_error = abs(float(np.arctan2(np.sin(pose[2] - anchor['pose_odom'][2]),
                                            np.cos(pose[2] - anchor['pose_odom'][2]))))
            elapsed = stamp - anchor['stamp_s']
            traveled, turned = path_m - anchor['path_m'], yaw_rad - anchor['yaw_rad']
            spatial_loop = anchor['excursion_m'] >= .8 and traveled >= 2.
            angular_loop = turned >= 2. * np.pi and elapsed >= 8.
            if distance <= .30 and yaw_error <= np.deg2rad(30.) and (spatial_loop or angular_loop):
                self.search_progress = {
                    'event': 'search_repeated', 'kind': 'spatial_loop' if spatial_loop else 'angular_loop',
                    'navigation_scope': self.navigation_scope, 'source_epoch': state.epoch,
                    'anchor_stamp_s': anchor['stamp_s'], 'stamp_s': stamp,
                    'revisit_distance_m': distance, 'revisit_yaw_rad': yaw_error,
                    'elapsed_s': elapsed, 'traveled_m': traveled, 'turned_rad': turned,
                    'maximum_excursion_m': anchor['excursion_m'],
                }
                self.search_reported_stamp = stamp
                self.pause('search_repeated')
                return
        self.search_anchors.append({'stamp_s': stamp, 'pose_odom': pose.tolist(),
                                    'path_m': path_m, 'yaw_rad': yaw_rad, 'excursion_m': 0.})
        self.search_progress = {'event': 'searching', 'navigation_scope': self.navigation_scope,
                                'source_epoch': state.epoch, 'stamp_s': stamp,
                                'sampled_path_m': path_m, 'sampled_absolute_yaw_rad': yaw_rad}

    def invalidate_path(self):
        self.generation += 1
        self.path = None
        self.path_provenance = None
        self.requested[:] = 0.
        self.brake_ticks = 0
        self.stop_candidate = False
        # The single inference worker drains an outstanding response even after
        # invalidation. Cancelling receive would mispair it with the next request.
        # A running native MPPI solve cannot be interrupted safely. Its captured
        # generation is now invalid, so tracking must discard its eventual result.

    def navigate(self, instruction, *, owner='human', navigation_scope=None,
                 maneuver=None, turn_degrees=None, advance_m=None, semantic_instruction=None,
                 goal_point_odom=None, goal_heading_rad=None,
                 position_tolerance_m=.08, heading_tolerance_deg=8.):
        if self.sim.robot_state.tracking != 'tracking':
            raise HTTPException(409, 'Robot sensor odometry is not tracking.')
        if self.state == 'HUMAN_PAUSED':
            raise HTTPException(409, 'Human force brake is latched; manual resume is required.')
        if not instruction.strip():
            raise HTTPException(400, 'Navigation instruction must not be empty.')
        if self.state == 'ERROR' or (self.nav_task is not None and self.nav_task.done()):
            raise HTTPException(409, f'Prototype runtime failed: {self.reason}')
        previous_motion = self.motion()
        self.invalidate_path()
        self.state, self.reason = 'NAVIGATING', ''
        self.motion_owner = owner
        self.task_id = str(uuid.uuid4())
        new_scope = navigation_scope if owner == 'monitor' else f'human:{self.task_id}'
        if new_scope != self.navigation_scope:
            self.lightnav_hint = None
            self.search_anchors = []
            self.search_reported_stamp = -np.inf
            self.search_progress = None
        self.navigation_scope = new_scope
        self.lightnav_latest = None
        self.maneuver, self.turn_degrees, self.advance_m = maneuver, turn_degrees, advance_m
        self.reference_source = ('metric_goal' if maneuver == 'follow_target' else
                                 'metric_turn' if turn_degrees is not None else
                                 'metric_advance' if advance_m is not None else 'lightnav')
        self.goal_point_odom = goal_point_odom
        self.goal_heading_rad = goal_heading_rad
        self.position_tolerance_m, self.heading_tolerance_deg = position_tolerance_m, heading_tolerance_deg
        self.goal_phase = None
        self.goal_phase_revision = 0
        self.goal_position_reached = False
        self.goal_position_reached_pose_odom = None
        self.motion_segment = MotionSegment(self.task_id, self.sim.robot_state, {
            'telemetry': str(self.output / 'telemetry.jsonl'),
            'lightnav': str(self.output / 'lightnav.jsonl'),
        })
        self.instruction = instruction
        self.semantic_instruction = semantic_instruction if owner == 'monitor' else instruction
        if self.reference_source == 'lightnav' and self.args.lightnav_instruction_prefix:
            self.semantic_instruction = self.args.lightnav_instruction_prefix.strip() + ' ' + self.semantic_instruction
        self.stop_candidate = False
        self.latency_ms = None
        self.path_capture_time = None
        self.reset_progress()
        if self.reference_source == 'metric_goal':
            self.update_goal_phase()
        self.nav_wakeup.set()
        self.publish()
        self.log.write(json.dumps(self.status()) + '\n')
        return {'task_id': self.task_id, 'status': 'accepted', 'previous_motion': previous_motion}

    def update_goal_phase(self):
        """Evaluate the shared odometry state without changing task authority."""
        if self.state != 'NAVIGATING' or self.reference_source != 'metric_goal':
            return
        pose = self.sim.robot_state.pose_odom
        delta = np.asarray(self.goal_point_odom) - pose[:2]
        if not self.goal_position_reached and np.linalg.norm(delta) <= self.position_tolerance_m:
            self.goal_position_reached = True
            self.goal_position_reached_pose_odom = pose.tolist()
        if self.goal_position_reached:
            heading_error = self.goal_heading_rad - pose[2]
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
            phase = 'reached' if abs(heading_error) <= np.deg2rad(self.heading_tolerance_deg) else 'align_heading'
        else:
            bearing_error = np.arctan2(delta[1], delta[0]) - pose[2]
            bearing_error = np.arctan2(np.sin(bearing_error), np.cos(bearing_error))
            phase = 'align_target' if abs(bearing_error) > np.deg2rad(10.) else 'approach'
        if phase != self.goal_phase:
            self.goal_phase = phase
            self.goal_phase_revision += 1
            self.requested[:] = 0.
            # External generation and LightNav transaction remain valid. The
            # old native MPPI solve is discarded using this internal revision.
        if phase == 'reached':
            self.pause('target_checkpoint')

    def motion(self):
        if self.motion_segment is None:
            return None
        self.motion_segment.update(self.sim.robot_state)
        return self.motion_segment.payload()

    def pause(self, reason, *, human=False, owner=None):
        if self.state == 'ERROR':
            return
        if self.state == 'HUMAN_PAUSED':
            reason = self.reason
            human = True
        self.invalidate_path()
        self.state = 'HUMAN_PAUSED' if human else 'PAUSED'
        self.reason = reason
        if human:
            self.motion_owner = 'human'
        elif owner is not None:
            self.motion_owner = owner
        self.publish()

    def fail(self, error):
        # Expose actual runtime errors; no reconnect loop or replacement trajectory.
        self.invalidate_path()
        self.state, self.reason = 'ERROR', str(error)
        print(f'PROTOTYPE ERROR: {error}', flush=True)
        self.publish()

    def expire_monitor(self):
        if self.monitor_id is None or time.monotonic() < self.monitor_deadline:
            return
        self.monitor_id = None
        self.monitor_deadline = 0.
        if self.motion_owner == 'monitor' and self.state == 'NAVIGATING':
            self.pause('monitor_timeout')
        else:
            # Lease expiry must not stop an independent human MANUAL command or
            # overwrite the reason for an already latched human pause.
            self.publish()

    def claim_monitor(self, monitor_id):
        if self.state == 'ERROR':
            raise HTTPException(409, f'Prototype runtime failed: {self.reason}')
        self.expire_monitor()
        if not monitor_id.strip():
            raise HTTPException(400, 'monitor_id must not be empty.')
        if self.monitor_id is not None and self.monitor_id != monitor_id:
            raise HTTPException(409, 'Another monitor owns an unexpired lease.')
        self.monitor_id = monitor_id
        self.monitor_deadline = time.monotonic() + 3.
        self.instruction = ''
        self.task_id = None
        self.pause('monitor_claim', owner='monitor')
        return self.status()

    def require_monitor(self, monitor_id):
        self.expire_monitor()
        if self.monitor_id != monitor_id or self.monitor_id is None:
            raise HTTPException(409, 'Monitor lease is absent, expired, or owned by another monitor; claim is required.')

    def heartbeat_monitor(self, monitor_id):
        self.require_monitor(monitor_id)
        self.monitor_deadline = time.monotonic() + 3.
        return self.status()

    def monitor_action(self, body):
        # No await between validation and commit: all control mutations run on
        # the same event loop, including physics and human HTTP operations.
        self.require_monitor(body.monitor_id)
        if body.expected_generation != self.generation:
            raise HTTPException(409, f'Stale control generation; current generation is {self.generation}.')
        if body.action == 'navigate':
            if body.instruction is None:
                raise HTTPException(400, 'navigate requires instruction.')
            if body.semantic_instruction is None or not body.semantic_instruction.strip():
                raise HTTPException(400, 'navigate requires non-empty semantic_instruction.')
            if body.navigation_scope is None or not body.navigation_scope.strip():
                raise HTTPException(400, 'navigate requires non-empty navigation_scope.')
            if body.maneuver is None:
                raise HTTPException(400, 'navigate requires maneuver.')
            if body.turn_degrees is not None and (
                    body.maneuver not in ('turn_left', 'turn_right') or not np.isfinite(body.turn_degrees) or body.turn_degrees <= 0.):
                raise HTTPException(400, 'turn_degrees requires a turn maneuver and a positive finite angle.')
            if body.maneuver in ('turn_left', 'turn_right') and body.turn_degrees is None:
                raise HTTPException(400, 'Turn maneuver requires turn_degrees.')
            if body.maneuver == 'advance' and body.advance_m is None:
                raise HTTPException(400, 'Advance maneuver requires advance_m.')
            if body.advance_m is not None and (
                    body.maneuver != 'advance' or not np.isfinite(body.advance_m) or body.advance_m <= 0.):
                raise HTTPException(400, 'advance_m requires advance maneuver and a positive finite distance.')
            if body.maneuver == 'follow_target':
                if (body.goal_point_odom is None or body.goal_heading_rad is None
                        or not np.isfinite(body.goal_point_odom).all() or not np.isfinite(body.goal_heading_rad)):
                    raise HTTPException(400, 'follow_target requires finite goal_point_odom [x,y] and goal_heading_rad.')
                if (not np.isfinite(body.position_tolerance_m) or body.position_tolerance_m <= 0.
                        or not np.isfinite(body.heading_tolerance_deg) or body.heading_tolerance_deg <= 0.):
                    raise HTTPException(400, 'Goal position and heading tolerances must be positive and finite.')
            elif body.goal_point_odom is not None or body.goal_heading_rad is not None:
                raise HTTPException(400, 'Goal point and heading require follow_target maneuver.')
            accepted = self.navigate(body.instruction, owner='monitor', navigation_scope=body.navigation_scope,
                                     maneuver=body.maneuver, turn_degrees=body.turn_degrees, advance_m=body.advance_m,
                                     semantic_instruction=body.semantic_instruction,
                                     goal_point_odom=body.goal_point_odom, goal_heading_rad=body.goal_heading_rad,
                                     position_tolerance_m=body.position_tolerance_m, heading_tolerance_deg=body.heading_tolerance_deg)
            return {**self.status(), 'previous_motion': accepted['previous_motion']}
        elif body.action in ('pause', 'invalidate'):
            if body.action == 'invalidate':
                self.instruction = ''
                self.task_id = None
            self.pause(body.reason or f'monitor_{body.action}', owner='monitor')
        elif body.action == 'complete':
            if self.state in ('ERROR', 'HUMAN_PAUSED'):
                raise HTTPException(409, f'Completion cannot override {self.state}.')
            self.invalidate_path()
            self.state, self.reason = 'LOCAL_DONE', body.reason or 'local_complete'
            self.motion_owner = 'monitor'
            self.publish()
        return self.status()

    def status(self):
        return {
            'event_id': self.event_id,
            'generation': self.generation, 'instance_id': self.instance_id,
            'monitor_id': self.monitor_id, 'motion_owner': self.motion_owner,
            'monitor_lease_remaining_s': max(0., self.monitor_deadline - time.monotonic()),
            'task_id': self.task_id, 'state': self.state, 'instruction': self.instruction,
            'semantic_instruction': self.semantic_instruction, 'reference_source': self.reference_source,
            'maneuver': self.maneuver, 'turn_degrees': self.turn_degrees, 'advance_m': self.advance_m,
            'goal_point_odom': self.goal_point_odom, 'goal_heading_rad': self.goal_heading_rad,
            'position_tolerance_m': self.position_tolerance_m, 'heading_tolerance_deg': self.heading_tolerance_deg,
            'goal_phase': self.goal_phase, 'goal_phase_revision': self.goal_phase_revision,
            'goal_position_reached': self.goal_position_reached,
            'goal_position_reached_pose_odom': self.goal_position_reached_pose_odom,
            'navigation_start_pose_odom': None if self.motion_segment is None else self.motion_segment.start_pose.tolist(),
            'reason': self.reason, 'sim_time_s': float(self.sim.stamp_s),
            'elapsed_wall_s': time.monotonic() - self.started_wall,
            'control_step_wall_ms': self.control_wall_ms,
            'robot_state': self.sim.robot_state.payload(),
            'requested_cmd': self.requested.tolist(), 'effective_cmd': self.effective.tolist(),
            'estimator_aux_raw': self.sim.aux.tolist(),
            'force_brake_latched': self.state == 'HUMAN_PAUSED',
            'lightnav_latency_ms': self.latency_ms,
            'navigation_session_id': self.navigation_session_id,
            'navigation_session_scope': self.navigation_session_scope,
            'navigation_scope': self.navigation_scope,
            'navigation_input_frames': self.navigation_input_frames,
            'navigation_input_interval_s': self.navigation_input_interval_s,
            'lightnav_hint': self.lightnav_hint,
            'search_progress': self.search_progress,
            'lightnav_latest': self.lightnav_latest,
            'path_capture_time_s': self.path_capture_time,
            'scene_capture_time_s': self.capture_time,
            'scene_robot_state': self.depth_frame.state.payload(),
            'scene_frame_id': self.depth_frame.frame_id,
            'depth_valid_ratio': float(self.depth_frame.valid_mask.mean()),
            'native_depth_valid_ratio': float(self.depth_frame.geometry.valid_mask.mean()),
            'native_depth_shape': list(self.depth_frame.geometry.depth_z_m.shape),
            'obstacle_map_capture_time_s': self.map_capture_time,
            'obstacle_voxel_count': len(self.obstacles_odom),
            'local_controller': self.controller_status,
            'arrival_candidate': self.stop_candidate, 'camera': self.args.camera,
            'output': str(self.output),
        }

    async def physics(self):
        tick = 0
        setup_released = self.args.setup_release_file is None
        try:
            while True:
                if (not setup_released and self.map_capture_time is not None
                        and self.sim.robot_state.tracking == 'tracking'):
                    # Inspection/model startup is outside the episode. Hold the
                    # simulation clock rather than advancing a zero-command gait.
                    self.log.flush()
                    print(f'SETUP_HELD sim_time_s={self.sim.stamp_s}', flush=True)
                    while not Path(self.args.setup_release_file).exists():
                        await asyncio.sleep(.1)
                    setup_released = True
                    print(f'SETUP_RELEASED sim_time_s={self.sim.stamp_s}', flush=True)
                started = time.monotonic()
                self.expire_monitor()
                if self.sim.stamp_s >= self.force_until:
                    self.force[:] = 0.
                paused = self.state in ('PAUSED', 'HUMAN_PAUSED', 'ARRIVAL_CANDIDATE', 'LOCAL_DONE', 'ERROR')
                self.effective = self.sim.step(self.requested, paused=paused, force_local=self.force)
                if self.motion_segment is not None:
                    self.motion_segment.update(self.sim.robot_state)
                    self.motion_segment.record_command(self.sim.robot_state.stamp_s, self.effective)
                # Local control can increase yaw while the person pulls back. Navigation yaw must not
                # withdraw forward force-brake authority; keep the old rule for manual drive.
                eligible = not paused and self.requested[0] >= .2 and (
                    self.state == 'NAVIGATING' or abs(self.requested[2]) <= .1)
                self.brake_ticks = self.brake_ticks + 1 if eligible and self.sim.aux[3] <= -.6 else 0
                if self.brake_ticks >= 2:
                    # Commands/gait change on the next 50 Hz policy observation.
                    self.pause('force_brake', human=True)
                self.update_goal_phase()
                if self.state == 'NAVIGATING' and self.turn_degrees is not None:
                    direction = 1. if self.maneuver == 'turn_left' else -1.
                    if direction * self.motion_segment.yaw_change >= np.deg2rad(self.turn_degrees):
                        self.pause('turn_checkpoint')
                if self.state == 'NAVIGATING' and self.advance_m is not None:
                    start_pose = self.motion_segment.start_pose
                    heading = np.array([np.cos(start_pose[2]), np.sin(start_pose[2])])
                    forward_displacement = float((self.sim.robot_state.pose_odom[:2] - start_pose[:2]) @ heading)
                    if forward_displacement >= self.advance_m:
                        self.pause('advance_checkpoint')
                if self.state == 'NAVIGATING' and (self.path is not None or self.reference_source != 'lightnav'):
                    delta = self.sim.robot_state.pose_odom - self.progress_pose
                    yaw_delta = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
                    if np.linalg.norm(delta[:2]) >= .10 or abs(yaw_delta) >= .20:
                        self.reset_progress()
                    elif self.sim.stamp_s - self.progress_time >= self.args.no_progress_seconds:
                        self.pause('no_progress')
                if self.state == 'NAVIGATING' and self.reference_source == 'lightnav' and self.path is not None:
                    self.update_search_progress()
                if self.sim.stamp_s >= self.next_capture_time:
                    self.capture()
                    self.next_capture_time += self.sim.profile['update_period_s']
                self.control_wall_ms = (time.monotonic() - started) * 1000.
                status = self.status()
                self.log.write(json.dumps(status) + '\n')
                if self.truth is not None:
                    self.truth.write(status, self.force)
                if tick % 50 == 0:
                    self.log.flush()
                tick += 1
                await asyncio.sleep(max(0., self.sim.metadata.control_dt - (time.monotonic() - started)))
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.fail(exc)
            raise

    async def navigation(self):
        """One serial WebSocket transaction owner for the simulator lifetime."""
        try:
            await self.nav_wakeup.wait()
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(self.args.lightnav) as ws:
                    await ws.send_json({'action': 'login', 'data': {'clientId': f'a2-{self.instance_id}'}})
                    login = await ws.receive_json(timeout=10.)
                    if login['action'] != 'login' or login['data']['rc'] != 0:
                        raise RuntimeError(f'LightNav login failed: {login}')
                    active_scope = None
                    sequence = 0
                    last_capture_time = None
                    session_reset_pending = False
                    while True:
                        if self.state != 'NAVIGATING':
                            self.nav_wakeup.clear()
                            await self.nav_wakeup.wait()
                            continue
                        scope = self.navigation_scope
                        if scope != active_scope:
                            await ws.send_json({'action': 'reset', 'data': {}})
                            reset = await ws.receive_json(timeout=10.)
                            if reset['action'] != 'reset' or reset['data']['rc'] != 0:
                                raise RuntimeError(f'LightNav reset failed: {reset}')
                            active_scope = scope
                            self.navigation_session_id = str(uuid.uuid4())
                            self.navigation_session_scope = scope
                            self.navigation_input_frames = 0
                            session_reset_pending = True
                            # A new action can arrive while reset is being acknowledged.
                            if self.state != 'NAVIGATING' or self.navigation_scope != scope:
                                continue
                        # Released checkpoint: nominal 4 fps, genuine sensor exposures.
                        if last_capture_time is not None and self.capture_time - last_capture_time < .25 - 1e-9:
                            self.capture_wakeup.clear()
                            await self.capture_wakeup.wait()
                            continue
                        generation, task_id = self.generation, self.task_id
                        instruction, motion_instruction = self.semantic_instruction, self.instruction
                        reference_source = self.reference_source
                        jpeg, capture_pose, capture_time = self.jpeg, self.capture_pose.copy(), self.capture_time
                        capture_frame = self.depth_frame
                        frame_id = capture_frame.frame_id
                        input_archive = None
                        if self.args.record_navigation_inputs:
                            input_archive = self.navigation_frame_output / f'{frame_id}.npz'
                            await asyncio.gather(
                                asyncio.to_thread(self.archive_frame, self.navigation_frame_output,
                                                  capture_frame),
                                asyncio.to_thread((self.navigation_frame_output / f'{frame_id}.jpg').write_bytes,
                                                  jpeg),
                            )
                        request_sent_wall_time_s = time.time()
                        reset_before_input = session_reset_pending
                        session_input_index = self.navigation_input_frames
                        await ws.send_json({'action': 'next', 'data': {
                            'seq': sequence, 'image': base64.b64encode(jpeg).decode(),
                            'instruction': instruction}})
                        session_reset_pending = False
                        self.navigation_input_interval_s = (None if last_capture_time is None
                                                            else capture_time - last_capture_time)
                        last_capture_time = capture_time
                        self.navigation_input_frames += 1
                        started = time.monotonic()
                        response = await ws.receive_json(timeout=self.args.inference_timeout)
                        if response['action'] != 'next':
                            raise RuntimeError(f'Unexpected LightNav response: {response}')
                        result = response['data']
                        if result['rc'] != 0:
                            raise RuntimeError(f'LightNav inference failed: {result}')
                        if result['seq'] != sequence:
                            raise RuntimeError('LightNav response sequence mismatch')
                        latency_ms = (time.monotonic() - started) * 1000.
                        discarded = generation != self.generation or self.state != 'NAVIGATING'
                        metadata = {
                            'task_id': task_id, 'instruction': instruction,
                            'motion_instruction': motion_instruction, 'reference_source': reference_source,
                            'navigation_scope': scope, 'navigation_session_id': self.navigation_session_id,
                            'sequence': sequence, 'frame_id': frame_id,
                            'session_reset_before_input': reset_before_input,
                            'session_input_index': session_input_index,
                            'request_sent_wall_time_s': request_sent_wall_time_s,
                            'response_received_wall_time_s': time.time(),
                            'input_interval_s': self.navigation_input_interval_s,
                            'nominal_input_fps': 4, 'source_epoch': capture_frame.state.epoch,
                            'capture_time_s': capture_time, 'capture_pose_odom': capture_pose.tolist(),
                            'input_archive': None if input_archive is None else str(input_archive),
                            'received_stamp_s': self.sim.robot_state.stamp_s,
                        }
                        self.nav_log.write(json.dumps({
                            **metadata, 'latency_ms': latency_ms, 'result': result, 'discarded': discarded,
                        }) + '\n')
                        sequence += 1
                        if discarded:
                            continue
                        waypoints = np.asarray(result['actions']['actions'], dtype=np.float64)
                        if (waypoints.ndim != 2 or waypoints.shape[1] != 3 or not len(waypoints)
                                or not np.isfinite(waypoints).all()):
                            raise RuntimeError(f'Invalid LightNav path: {waypoints}')
                        path_odom = project_body_to_odom(waypoints, capture_pose)
                        self.latency_ms = latency_ms
                        self.path_capture_time = capture_time
                        self.stop_candidate = result['stop'] and reference_source == 'lightnav'
                        # pointing is legitimately absent for non-pointing checkpoints.
                        self.lightnav_latest = {**metadata, 'pointing': result.get('pointing'),
                                                'visible': result['visible'], 'stop': result['stop']}
                        pointing = result.get('pointing')
                        self.lightnav_hint = None
                        if pointing is not None and pointing['opos_state'] == 'point':
                            odom_point = capture_frame.odom_point_from_pixel(pointing['opos_px'])
                            if odom_point is not None:
                                self.lightnav_hint = {
                                    'source': 'LightNav_opos_plus_same_frame_depth',
                                    'claim': 'model_pointing_hypothesis_not_object_identity',
                                    'source_instruction': instruction,
                                    'navigation_scope': scope, 'source_frame_id': frame_id,
                                    'source_epoch': capture_frame.state.epoch,
                                    'source_sim_time_s': capture_time, 'odom_point_m': odom_point,
                                    'visible_claim': result['visible'],
                                }
                        self.motion_segment.add_plan({
                            **self.lightnav_latest, 'waypoints_odom': path_odom.tolist(),
                        })
                        if self.stop_candidate:
                            self.invalidate_path()
                            self.state = 'ARRIVAL_CANDIDATE'
                            self.stop_candidate = True
                            self.publish()
                            continue
                        if self.path is None and reference_source == 'lightnav':
                            self.reset_progress()
                        if reference_source == 'lightnav':
                            self.path = path_odom
                            self.path_provenance = metadata
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.fail(exc)
            raise

    async def tracking(self):
        try:
            while True:
                started = time.monotonic()
                if (self.state == 'NAVIGATING' and self.map_capture_time is not None
                        and (self.path is not None or self.reference_source != 'lightnav')):
                    generation = self.generation
                    goal_phase_revision, goal_phase = self.goal_phase_revision, self.goal_phase
                    input_time = float(self.sim.stamp_s)
                    map_view = copy(self.depth_map)
                    input_state = map_view.snapshot.state
                    pose = input_state.pose_odom
                    previous_command = (float(self.effective[0]), float(self.effective[2]))
                    # The optimizer starts from the measured frame time, so
                    # account for capture age as well as actuator/solver delay.
                    estimated_delay = self.controller_execution_delay_s + input_time - input_state.stamp_s
                    obstacles = map_view.snapshot.obstacles_odom
                    map_capture_time = input_state.stamp_s
                    reference_source = self.reference_source
                    path_provenance = self.path_provenance if reference_source == 'lightnav' else None
                    reference_times = (np.arange(self.controller.horizon) + 1) * self.controller.dt_s
                    if reference_source == 'metric_goal':
                        delta = np.asarray(self.goal_point_odom) - pose[:2]
                        bearing = np.arctan2(delta[1], delta[0])
                        reference = np.repeat(pose[None, :], self.controller.horizon, axis=0)
                        if goal_phase == 'approach':
                            heading = np.array([np.cos(bearing), np.sin(bearing)])
                            reference[:, :2] += (self.args.max_speed * reference_times)[:, None] * heading
                            reference[:, 2] = bearing
                        elif goal_phase in ('align_target', 'align_heading'):
                            target_yaw = self.goal_heading_rad if goal_phase == 'align_heading' else bearing
                            error = np.arctan2(np.sin(target_yaw - pose[2]), np.cos(target_yaw - pose[2]))
                            reference[:, 2] = pose[2] + np.sign(error) * .4 * reference_times
                        else:
                            raise RuntimeError(f'Invalid navigating goal phase: {goal_phase}')
                    elif reference_source == 'metric_turn':
                        direction = 1. if self.maneuver == 'turn_left' else -1.
                        reference = np.repeat(pose[None, :], self.controller.horizon, axis=0)
                        reference[:, 2] = pose[2] + direction * .4 * reference_times
                    elif reference_source == 'metric_advance':
                        start_pose = self.motion_segment.start_pose
                        heading = np.array([np.cos(start_pose[2]), np.sin(start_pose[2])])
                        progress = float((pose[:2] - start_pose[:2]) @ heading)
                        reference = np.repeat(start_pose[None, :], self.controller.horizon, axis=0)
                        reference[:, :2] += (progress + self.args.max_speed * reference_times)[:, None] * heading
                    else:
                        reference = self.path.copy()
                    translation_allowed = self.maneuver not in ('turn_left', 'turn_right')
                    if reference_source == 'metric_goal':
                        translation_allowed = goal_phase == 'approach'
                    if not translation_allowed:
                        reference[:, :2] = pose[:2]
                    if not np.any(self.depth_frame.geometry.valid_mask):
                        self.pause('depth_unknown')
                        continue
                    try:
                        result = await asyncio.to_thread(
                            self.controller.solve, input_state, reference, previous_command, obstacles,
                            execution_delay_s=estimated_delay,
                            speed_limit=self.args.max_speed,
                            translation_allowed=translation_allowed)
                    except LocalControlBlocked as error:
                        if (generation == self.generation and self.state == 'NAVIGATING'
                                and goal_phase_revision == self.goal_phase_revision):
                            self.controller_status = {'backend': 'nav2_mppi', 'solver_status': 'blocked', 'reason': str(error),
                                               'blocked_category': error.category,
                                               'native_diagnostics': controller_diagnostics_for_status(error.diagnostics),
                                               'reference_source': reference_source,
                                               'path_provenance': path_provenance, 'command_applied': False,
                                               'goal_phase': goal_phase, 'goal_phase_revision': goal_phase_revision,
                                               'translation_allowed': translation_allowed}
                            self.controller_log.write(json.dumps({
                                'control_task_id': self.task_id,
                                'sim_time_s': float(self.sim.stamp_s), 'capture_time_s': map_capture_time,
                                'input_sim_time_s': input_time, 'pose_odom': pose.tolist(),
                                'submitted_pose_odom': self.sim.robot_state.pose_odom.tolist(),
                                'robot_state': input_state.payload(),
                                'previous_command': previous_command, 'execution_delay_s': estimated_delay,
                                'reference_odom': reference.tolist(),
                                'obstacles_odom': obstacles.tolist(), **self.controller_status,
                            }) + '\n')
                            self.pause('local_controller_blocked')
                        continue
                    if (generation == self.generation and self.state == 'NAVIGATING'
                            and goal_phase_revision == self.goal_phase_revision):
                        self.controller_execution_delay_s = float(self.sim.stamp_s) - input_time
                        self.requested[:] = (result.command[0], 0., result.command[1])
                        self.controller_status = {
                            'backend': 'nav2_mppi',
                            'reference_source': reference_source,
                            'path_provenance': path_provenance, 'command_applied': True,
                            'goal_phase': goal_phase, 'goal_phase_revision': goal_phase_revision,
                            'translation_allowed': translation_allowed,
                            'solver_status': result.solver_status, 'solve_ms': result.solve_ms,
                            'native_diagnostics': controller_diagnostics_for_status(result.diagnostics),
                            'minimum_clearance_m': result.minimum_clearance_m,
                            'stop_minimum_clearance_m': result.stop_minimum_clearance_m,
                            'stop_reserve_m': result.stop_reserve_m,
                            'stop_model_id': result.stop_model_id,
                            'obstacle_count': result.obstacle_count,
                            'execution_delay_s': result.execution_delay_s,
                            'actual_execution_delay_s': self.controller_execution_delay_s,
                        }
                        self.controller_log.write(json.dumps({
                            'control_task_id': self.task_id,
                            'sim_time_s': float(self.sim.stamp_s), 'capture_time_s': map_capture_time,
                            'input_sim_time_s': input_time, 'pose_odom': pose.tolist(),
                            'submitted_pose_odom': self.sim.robot_state.pose_odom.tolist(),
                            'robot_state': input_state.payload(),
                            'previous_command': previous_command,
                            'reference_odom': reference.tolist(),
                            'obstacles_odom': obstacles.tolist(),
                            'predicted_odom': result.predicted_odom.tolist(),
                            'predicted_time_s': result.predicted_time_s.tolist(),
                            'command': list(result.command), **self.controller_status,
                        }) + '\n')
                await asyncio.sleep(max(0., .1 - (time.monotonic() - started)))
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.fail(exc)
            raise


class Navigate(BaseModel):
    instruction: str


class Reason(BaseModel):
    reason: str


class Resume(BaseModel):
    source: str
    expected_generation: int


class MonitorIdentity(BaseModel):
    monitor_id: str


class MonitorAction(MonitorIdentity):
    expected_generation: int
    action: Literal['navigate', 'pause', 'invalidate', 'complete']
    instruction: str | None = None
    semantic_instruction: str | None = None
    navigation_scope: str | None = None
    maneuver: Literal['lightnav', 'advance', 'turn_left', 'turn_right', 'follow_target', 'escape'] | None = None
    turn_degrees: float | None = None
    advance_m: float | None = None
    goal_point_odom: tuple[float, float] | None = None
    goal_heading_rad: float | None = None
    position_tolerance_m: float = .08
    heading_tolerance_deg: float = 8.
    reason: str | None = None


class Force(BaseModel):
    xyz: tuple[float, float, float]
    duration_s: float


class Region(BaseModel):
    frame_id: str
    bbox_px: tuple[int, int, int, int]


class SemanticInstance(BaseModel):
    frame_id: str
    instance_id: str


class SpatialEvidence(BaseModel):
    epoch: str
    stamp_s: float
    points_odom: list[tuple[float, float, float]]
    footprint_poses_odom: list[tuple[float, float, float]]


class Command(BaseModel):
    vx: float
    vy: float = 0.
    wz: float = 0.


def make_app(args):
    @asynccontextmanager
    async def lifespan(app):
        p = Prototype(args)
        app.state.prototype = p
        tasks = [asyncio.create_task(p.physics()), asyncio.create_task(p.perception()), asyncio.create_task(p.tracking())]
        p.nav_task = asyncio.create_task(p.navigation())
        tasks.append(p.nav_task)
        print(f'Artifacts: {p.output}', flush=True)
        try:
            yield
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            p.log.close()
            p.nav_log.close()
            p.controller_log.close()
            p.controller.close()
            if p.truth is not None:
                p.truth.close()
            p.sim.close()

    app = FastAPI(title='A2 LightNav prototype', lifespan=lifespan)

    @app.get('/status')
    async def status():
        return app.state.prototype.status()

    @app.get('/motion')
    async def motion():
        return app.state.prototype.motion()

    @app.get('/events')
    async def events():
        return StreamingResponse(app.state.prototype.events(), media_type='text/event-stream',
                                 headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

    @app.get('/scene.jpg')
    async def scene():
        p = app.state.prototype
        return Response(p.jpeg, media_type='image/jpeg', headers={
            'X-Sim-Time': str(p.capture_time),
            'X-Robot-State': json.dumps(p.depth_frame.state.payload()),
            'X-Scene-Camera': json.dumps(p.sim.profile),
            'X-Scene-Frame': p.depth_frame.frame_id,
        })

    @app.get('/scene')
    async def depth_scene():
        return await app.state.prototype.scene()

    @app.post('/measure_semantic_instance')
    async def measure_semantic_instance(body: SemanticInstance):
        return app.state.prototype.measure_semantic_instance(body.frame_id, body.instance_id)

    @app.post('/measure_region')
    async def measure_region(body: Region):
        return app.state.prototype.measure_region(body.frame_id, body.bbox_px)

    @app.post('/spatial_evidence')
    async def spatial_evidence(body: SpatialEvidence):
        try:
            return await asyncio.to_thread(app.state.prototype.depth_map.query, **body.model_dump())
        except ValueError as error:
            raise HTTPException(409, str(error)) from error

    @app.post('/navigate')
    async def navigate(body: Navigate):
        return app.state.prototype.navigate(body.instruction)

    @app.post('/pause')
    async def pause(body: Reason):
        p = app.state.prototype
        p.pause(body.reason, human=True)
        return p.status()

    @app.post('/resume')
    async def resume(body: Resume):
        if body.source != 'human':
            raise HTTPException(400, 'Only a human can resume; this endpoint is not a Pi tool.')
        p = app.state.prototype
        if body.expected_generation != p.generation:
            raise HTTPException(409, f'Stale human resume; current generation is {p.generation}.')
        if p.state not in ('HUMAN_PAUSED', 'PAUSED'):
            raise HTTPException(409, f'Cannot resume from {p.state}')
        p.invalidate_path()
        p.state, p.reason = 'IDLE', 'human_resume'
        p.motion_owner = 'human'
        p.instruction = ''
        p.task_id = None
        p.publish()
        return p.status()

    @app.post('/monitor/claim')
    async def monitor_claim(body: MonitorIdentity):
        return app.state.prototype.claim_monitor(body.monitor_id)

    @app.post('/monitor/heartbeat')
    async def monitor_heartbeat(body: MonitorIdentity):
        return app.state.prototype.heartbeat_monitor(body.monitor_id)

    @app.post('/monitor/action')
    async def monitor_action(body: MonitorAction):
        return app.state.prototype.monitor_action(body)

    @app.post('/force')
    async def force(body: Force):
        p = app.state.prototype
        p.force[:] = body.xyz
        p.force_until = float(p.sim.stamp_s) + body.duration_s
        return p.status()

    @app.post('/command')
    async def command(body: Command):
        p = app.state.prototype
        if p.state == 'HUMAN_PAUSED':
            raise HTTPException(409, 'Manual resume is required.')
        p.invalidate_path()
        p.state, p.reason = 'MANUAL', 'human_command'
        p.motion_owner = 'human'
        p.instruction = ''
        p.task_id = None
        p.requested[:] = (body.vx, body.vy, body.wz)
        p.publish()
        return p.status()

    @app.get('/', response_class=HTMLResponse)
    def index():
        return '''<!doctype html><meta charset="utf-8"><title>A2 prototype</title>
        <style>body{font:16px sans-serif;max-width:1000px;margin:30px auto}img{width:640px}pre{white-space:pre-wrap}
        button{padding:10px 16px;margin:4px;cursor:pointer}</style>
        <h1>A2 · LightNav · Pi</h1><img id="camera" src="/scene.jpg">
        <p>仿真人工操作（拉力持续 2 个仿真秒）：</p>
        <button onclick="operator('/pause',{reason:'human'})">人工暂停</button>
        <button onclick="operator('/resume',{source:'human'})">人工恢复</button>
        <button onclick="operator('/force',{xyz:[-40,0,0],duration_s:2})">向后拉 40 N</button>
        <button onclick="operator('/force',{xyz:[-80,0,0],duration_s:2})">向后拉 80 N</button>
        <p id="operation"></p><pre id="status"></pre>
        <script>async function operator(path,body){if(path==='/resume'){
        body.expected_generation=(await(await fetch('/status')).json()).generation;}
        const response=await fetch(path,{method:'POST',
        headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
        document.getElementById('operation').textContent=response.ok?'操作已执行':await response.text()}</script>
        <script>setInterval(async()=>{document.getElementById('camera').src='/scene.jpg?t='+Date.now();
        document.getElementById('status').textContent=JSON.stringify(await(await fetch('/status')).json(),null,2)},500)</script>'''

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['mujoco', 'isaac'], default='mujoco')
    parser.add_argument('--isaac-socket', help='AF_UNIX socket of the native Isaac plant')
    parser.add_argument('--setup-release-file', help='Hold simulation after sensor initialization until the experiment launcher releases setup.')
    parser.add_argument('--scene', default='procthor')
    profiles = json.loads((ROOT / 'prototype/camera/profiles.json').read_text())
    parser.add_argument('--camera', choices=[name for name, profile in profiles['profiles'].items() if 'depth_range_m' in profile],
                        default=profiles['default_profile'])
    parser.add_argument('--spawn-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--record-navigation-inputs', action='store_true')
    parser.add_argument('--semantic-checkpoint', default=str(ROOT / '.local/models/perception/yoloe-26l-seg.pt'))
    parser.add_argument('--semantic-classes', nargs='+', default=['door', 'door frame', 'elevator door', 'wall'])
    parser.add_argument('--record-contacts', action='store_true', help='Record 5 ms robot/environment contacts for offline diagnosis, not Agent input.')
    parser.add_argument('--lightnav', default='ws://127.0.0.1:8050')
    parser.add_argument('--inference-timeout', type=float, default=60.)
    parser.add_argument('--max-speed', type=float, default=.3)
    parser.add_argument('--controller-footprint', default=str(ROOT / 'prototype/control/footprint.json'))
    parser.add_argument('--lightnav-instruction-prefix', default='')
    parser.add_argument('--no-progress-seconds', type=float, default=8.)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    uvicorn.run(make_app(args), host='127.0.0.1', port=args.port, log_level='info')
