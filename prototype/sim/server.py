"""MuJoCo RGB-D + old A2 policy + body-aware MPC, exposed to Pi over HTTP."""
from __future__ import annotations

import argparse
import asyncio
import base64
from collections import OrderedDict
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
from gpu import configure_gpu
GPU_BINDING = configure_gpu()

from a2 import A2Simulation, ROOT
from OpenGL import GL
from depth_map import DepthObstacleMap
from motion_segment import MotionSegment
from semantic_depth import YOLOESemanticDepth
from vln_mujoco.mpc import build_pose_aligned_reference, project_body_to_world, Q_WEIGHTS

sys.path.insert(0, str(ROOT / 'prototype'))
from control import CollisionAwareMPC, MPCInfeasible


class Prototype:
    def __init__(self, args):
        self.args = args
        self.sim = A2Simulation(args.scene, args.camera,
                                spawn_pose=None if args.spawn_pose is None else tuple(args.spawn_pose))
        self.state = 'IDLE'
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
        self.force = np.zeros(3)
        self.force_until = 0.
        self.brake_ticks = 0
        self.started_wall = time.monotonic()
        self.frame_sequence = 0
        self.frames = OrderedDict()
        self.semantic_frames = OrderedDict()
        self.semantic_lock = asyncio.Lock()
        self.semantic_detector = YOLOESemanticDepth(args.semantic_checkpoint, 'cuda:0')
        self.capture()
        self.next_capture_time = self.capture_time + self.sim.profile['update_period_s']
        self.latency_ms = None
        self.control_wall_ms = 0.
        self.path_capture_time = None
        self.stop_candidate = False
        self.controller = CollisionAwareMPC(max_speed=args.max_speed)
        self.depth_map = DepthObstacleMap()
        self.obstacles_world = np.empty((0, 2))
        self.map_capture_time = None
        self.mpc_status = None
        self.mpc_execution_delay_s = .20
        self.output = ROOT / '.local/outputs' / time.strftime('sim_%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True)
        self.frame_output = self.output / 'frames'
        self.frame_output.mkdir()
        self.navigation_frame_output = self.output / 'navigation_frames'
        if args.record_navigation_inputs:
            self.navigation_frame_output.mkdir()
        if args.record_contacts:
            self.sim.record_contacts(self.output)
        self.log = (self.output / 'telemetry.jsonl').open('w')
        self.nav_log = (self.output / 'lightnav.jsonl').open('w', buffering=1)
        self.mpc_log = (self.output / 'mpc.jsonl').open('w', buffering=1)
        (self.output / 'configuration.json').write_text(json.dumps({
            **vars(args), 'camera_profile': self.sim.profile,
            'rendering': {'vendor': GL.glGetString(GL.GL_VENDOR).decode(),
                          'renderer': GL.glGetString(GL.GL_RENDERER).decode(),
                          'version': GL.glGetString(GL.GL_VERSION).decode(),
                          'egl_device_id': os.environ['MUJOCO_EGL_DEVICE_ID']},
            'cuda_visible_devices': os.environ['CUDA_VISIBLE_DEVICES'],
            'gpu_binding': GPU_BINDING,
            'semantic_depth': {'model': self.semantic_detector.checkpoint,
                               'classes': self.semantic_detector.classes,
                               'device': self.semantic_detector.device,
                               'confidence_threshold': self.semantic_detector.confidence},
            'spawn_pose': {
                'x_m': self.sim.spawn[0], 'y_m': self.sim.spawn[1], 'z_m': self.sim.spawn[2],
                'yaw_deg': self.sim.spawn_yaw_deg,
            },
            'observation': 'LightNav: raw RGB; Pi: RGB + YOLOE same-frame instance masks/labels and mask-restricted depth measurements, no scene target truth',
            'depth_source': 'ideal MuJoCo RGB-aligned optical-axis depth, not physical D435i noise',
            'render_offsamples': int(self.sim.model.vis.quality.offsamples),
            'mpc_footprint': json.loads((ROOT / 'prototype/control/footprint.json').read_text()),
        }, indent=2))
        self.nav_task = None
        self.nav_wakeup = asyncio.Event()
        self.navigation_scope = None
        self.navigation_session_id = None
        self.navigation_session_scope = None
        self.navigation_input_frames = 0
        self.lightnav_latest = None
        self.lightnav_hint = None
        self.maneuver = None
        self.turn_degrees = None
        self.advance_m = None
        self.goal_point_world = None
        self.goal_heading_rad = None
        self.position_tolerance_m = .08
        self.heading_tolerance_deg = 8.
        self.goal_phase = None
        self.goal_phase_revision = 0
        self.goal_position_reached = False
        self.goal_position_reached_pose_world = None
        self.motion_segment = None
        self.event_id = 0
        self.listeners = set()
        self.progress_pose = self.sim.pose()
        self.progress_time = float(self.sim.data.time)

    def capture(self):
        self.frame_sequence += 1
        frame_id = f'{int(self.started_wall * 1000)}-{self.frame_sequence}'
        self.depth_frame = self.sim.capture(frame_id)
        self.capture_world_from_base = self.sim.world_from_base()
        self.capture_pose = np.asarray(self.depth_frame.pose_world)
        self.capture_time = self.depth_frame.sim_time_s
        output = io.BytesIO()
        Image.fromarray(self.depth_frame.rgb).save(output, format='JPEG', quality=90)
        self.jpeg = output.getvalue()

    def archive_frame(self, output, frame, world_from_base):
        np.savez_compressed(output / f'{frame.frame_id}.npz',
                            frame_id=frame.frame_id, sim_time_s=frame.sim_time_s,
                            pose_world=frame.pose_world, rgb=frame.rgb, depth_z_m=frame.depth_z_m,
                            intrinsics=np.array([frame.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy')]),
                            camera_to_base=frame.camera_to_base, world_from_base=world_from_base)

    async def scene(self):
        async with self.semantic_lock:
            frame = self.depth_frame
            hint = self.lightnav_hint
            transform = self.capture_world_from_base.copy()
            archive = frame.frame_id not in self.frames
            if archive:
                semantic = await asyncio.to_thread(self.semantic_detector.predict, frame)
                self.frames[frame.frame_id] = frame
                self.semantic_frames[frame.frame_id] = semantic
                await asyncio.to_thread(self.archive_frame, self.frame_output, frame, transform)
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
            'age_s': frame.sim_time_s - hint['source_sim_time_s'],
            **frame.project_world_point(hint['world_point_m']),
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
                frame, transform = self.depth_frame, self.capture_world_from_base.copy()
                self.obstacles_world = await asyncio.to_thread(self.depth_map.update, frame, transform)
                self.map_capture_time = frame.sim_time_s
                await asyncio.sleep(max(0., .1 - (time.monotonic() - started)))
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.fail(error)
            raise

    def publish(self):
        if self.motion_segment is not None:
            self.motion_segment.event(self.sim.data.time, self.state, self.reason)
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
        self.progress_pose = self.sim.pose()
        self.progress_time = float(self.sim.data.time)

    def invalidate_path(self):
        self.generation += 1
        self.path = None
        self.requested[:] = 0.
        self.brake_ticks = 0
        self.stop_candidate = False
        # The single inference worker drains an outstanding response even after
        # invalidation. Cancelling receive would mispair it with the next request.
        # A running native MPC solve cannot be interrupted safely. Its captured
        # generation is now invalid, so tracking must discard its eventual result.

    def navigate(self, instruction, *, owner='human', navigation_scope=None,
                 maneuver=None, turn_degrees=None, advance_m=None, semantic_instruction=None,
                 goal_point_world=None, goal_heading_rad=None,
                 position_tolerance_m=.08, heading_tolerance_deg=8.):
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
        self.navigation_scope = new_scope
        self.lightnav_latest = None
        self.maneuver, self.turn_degrees, self.advance_m = maneuver, turn_degrees, advance_m
        self.reference_source = ('metric_goal' if maneuver == 'follow_target' else
                                 'metric_turn' if turn_degrees is not None else
                                 'metric_advance' if advance_m is not None else 'lightnav')
        self.goal_point_world = goal_point_world
        self.goal_heading_rad = goal_heading_rad
        self.position_tolerance_m, self.heading_tolerance_deg = position_tolerance_m, heading_tolerance_deg
        self.goal_phase = None
        self.goal_phase_revision = 0
        self.goal_position_reached = False
        self.goal_position_reached_pose_world = None
        self.motion_segment = MotionSegment(self.task_id, float(self.sim.data.time), self.sim.pose(), {
            'telemetry': str(self.output / 'telemetry.jsonl'),
            'lightnav': str(self.output / 'lightnav.jsonl'),
        })
        self.instruction = instruction
        self.semantic_instruction = semantic_instruction if owner == 'monitor' else instruction
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
        """Evaluate the measured pose at 50 Hz without changing task authority."""
        if self.state != 'NAVIGATING' or self.reference_source != 'metric_goal':
            return
        pose = self.sim.pose()
        delta = np.asarray(self.goal_point_world) - pose[:2]
        if not self.goal_position_reached and np.linalg.norm(delta) <= self.position_tolerance_m:
            self.goal_position_reached = True
            self.goal_position_reached_pose_world = pose.tolist()
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
            # old native MPC solve is discarded using this internal revision.
        if phase == 'reached':
            self.pause('target_checkpoint')

    def motion(self):
        if self.motion_segment is None:
            return None
        self.motion_segment.update(float(self.sim.data.time), self.sim.pose())
        return self.motion_segment.payload()

    def pause(self, reason, *, human=False, owner=None):
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
            if body.advance_m is not None and (
                    body.maneuver != 'advance' or not np.isfinite(body.advance_m) or body.advance_m <= 0.):
                raise HTTPException(400, 'advance_m requires advance maneuver and a positive finite distance.')
            if body.maneuver == 'follow_target':
                if (body.goal_point_world is None or body.goal_heading_rad is None
                        or not np.isfinite(body.goal_point_world).all() or not np.isfinite(body.goal_heading_rad)):
                    raise HTTPException(400, 'follow_target requires finite goal_point_world [x,y] and goal_heading_rad.')
                if (not np.isfinite(body.position_tolerance_m) or body.position_tolerance_m <= 0.
                        or not np.isfinite(body.heading_tolerance_deg) or body.heading_tolerance_deg <= 0.):
                    raise HTTPException(400, 'Goal position and heading tolerances must be positive and finite.')
            elif body.goal_point_world is not None or body.goal_heading_rad is not None:
                raise HTTPException(400, 'Goal point and heading require follow_target maneuver.')
            accepted = self.navigate(body.instruction, owner='monitor', navigation_scope=body.navigation_scope,
                                     maneuver=body.maneuver, turn_degrees=body.turn_degrees, advance_m=body.advance_m,
                                     semantic_instruction=body.semantic_instruction,
                                     goal_point_world=body.goal_point_world, goal_heading_rad=body.goal_heading_rad,
                                     position_tolerance_m=body.position_tolerance_m, heading_tolerance_deg=body.heading_tolerance_deg)
            return {**self.status(), 'previous_motion': accepted['previous_motion']}
        elif body.action in ('pause', 'invalidate'):
            if body.action == 'invalidate':
                self.instruction = ''
                self.task_id = None
            self.pause(body.reason or f'monitor_{body.action}', owner='monitor')
        elif body.action == 'complete':
            if self.state not in ('ARRIVAL_CANDIDATE', 'PAUSED'):
                raise HTTPException(409, f'Local completion requires ARRIVAL_CANDIDATE or PAUSED, not {self.state}.')
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
            'goal_point_world': self.goal_point_world, 'goal_heading_rad': self.goal_heading_rad,
            'position_tolerance_m': self.position_tolerance_m, 'heading_tolerance_deg': self.heading_tolerance_deg,
            'goal_phase': self.goal_phase, 'goal_phase_revision': self.goal_phase_revision,
            'goal_position_reached': self.goal_position_reached,
            'goal_position_reached_pose_world': self.goal_position_reached_pose_world,
            'navigation_start_pose_world': None if self.motion_segment is None else self.motion_segment.start_pose.tolist(),
            'reason': self.reason, 'sim_time_s': float(self.sim.data.time),
            'elapsed_wall_s': time.monotonic() - self.started_wall,
            'control_step_wall_ms': self.control_wall_ms,
            'pose_world': self.sim.pose().tolist(), 'base_z_m': float(self.sim.data.qpos[2]),
            'requested_cmd': self.requested.tolist(), 'effective_cmd': self.effective.tolist(),
            'actual_velocity_body': self.sim.velocity().tolist(),
            'actual_planar_twist': self.sim.planar_twist().tolist(),
            'estimator_aux_raw': self.sim.aux.tolist(), 'force_applied_local_N': self.force.tolist(),
            'force_brake_latched': self.state == 'HUMAN_PAUSED',
            'lightnav_latency_ms': self.latency_ms,
            'navigation_session_id': self.navigation_session_id,
            'navigation_session_scope': self.navigation_session_scope,
            'navigation_scope': self.navigation_scope,
            'navigation_input_frames': self.navigation_input_frames,
            'lightnav_latest': self.lightnav_latest,
            'path_capture_time_s': self.path_capture_time,
            'scene_capture_time_s': self.capture_time,
            'scene_capture_pose_world': self.capture_pose.tolist(),
            'scene_frame_id': self.depth_frame.frame_id,
            'depth_valid_ratio': float(self.depth_frame.valid_mask.mean()),
            'obstacle_map_capture_time_s': self.map_capture_time,
            'obstacle_voxel_count': len(self.obstacles_world),
            'mpc': self.mpc_status,
            'arrival_candidate': self.stop_candidate, 'camera': self.args.camera,
            'output': str(self.output),
        }

    async def physics(self):
        tick = 0
        try:
            while True:
                started = time.monotonic()
                self.expire_monitor()
                if self.sim.data.time >= self.force_until:
                    self.force[:] = 0.
                paused = self.state in ('PAUSED', 'HUMAN_PAUSED', 'ARRIVAL_CANDIDATE', 'LOCAL_DONE', 'ERROR')
                self.effective = self.sim.step(self.requested, paused=paused, force_local=self.force)
                if self.motion_segment is not None:
                    self.motion_segment.update(float(self.sim.data.time), self.sim.pose())
                # MPC can increase yaw while the person pulls back. Navigation yaw must not
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
                    forward_displacement = float((self.sim.pose()[:2] - start_pose[:2]) @ heading)
                    if forward_displacement >= self.advance_m:
                        self.pause('advance_checkpoint')
                if self.state == 'NAVIGATING' and (self.path is not None or self.reference_source != 'lightnav'):
                    delta = self.sim.pose() - self.progress_pose
                    yaw_delta = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
                    if np.linalg.norm(delta[:2]) >= .10 or abs(yaw_delta) >= .20:
                        self.reset_progress()
                    elif self.sim.data.time - self.progress_time >= self.args.no_progress_seconds:
                        self.pause('no_progress')
                if self.sim.data.time >= self.next_capture_time:
                    self.capture()
                    self.next_capture_time += self.sim.profile['update_period_s']
                self.control_wall_ms = (time.monotonic() - started) * 1000.
                self.log.write(json.dumps(self.status()) + '\n')
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
                            # A new action can arrive while reset is being acknowledged.
                            if self.state != 'NAVIGATING' or self.navigation_scope != scope:
                                continue
                        generation, task_id = self.generation, self.task_id
                        instruction, motion_instruction = self.semantic_instruction, self.instruction
                        reference_source = self.reference_source
                        jpeg, capture_pose, capture_time = self.jpeg, self.capture_pose.copy(), self.capture_time
                        capture_frame, capture_transform = self.depth_frame, self.capture_world_from_base.copy()
                        frame_id = capture_frame.frame_id
                        input_archive = None
                        if self.args.record_navigation_inputs:
                            input_archive = self.navigation_frame_output / f'{frame_id}.npz'
                            await asyncio.gather(
                                asyncio.to_thread(self.archive_frame, self.navigation_frame_output,
                                                  capture_frame, capture_transform),
                                asyncio.to_thread((self.navigation_frame_output / f'{frame_id}.jpg').write_bytes,
                                                  jpeg),
                            )
                        await ws.send_json({'action': 'next', 'data': {
                            'seq': sequence, 'image': base64.b64encode(jpeg).decode(),
                            'instruction': instruction}})
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
                            'capture_time_s': capture_time, 'capture_pose_world': capture_pose.tolist(),
                            'input_archive': None if input_archive is None else str(input_archive),
                            'received_sim_time_s': float(self.sim.data.time),
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
                        path_world = project_body_to_world(waypoints, capture_pose)
                        self.latency_ms = latency_ms
                        self.path_capture_time = capture_time
                        self.stop_candidate = result['stop'] and reference_source == 'lightnav'
                        # pointing is legitimately absent for non-pointing checkpoints.
                        self.lightnav_latest = {**metadata, 'pointing': result.get('pointing'),
                                                'visible': result['visible'], 'stop': result['stop']}
                        pointing = result.get('pointing')
                        self.lightnav_hint = None
                        if pointing is not None and pointing['opos_state'] == 'point':
                            world_point = capture_frame.world_point_from_pixel(pointing['opos_px'])
                            if world_point is not None:
                                self.lightnav_hint = {
                                    'source': 'LightNav_opos_plus_same_frame_depth',
                                    'claim': 'model_pointing_hypothesis_not_object_identity',
                                    'source_instruction': instruction,
                                    'navigation_scope': scope, 'source_frame_id': frame_id,
                                    'source_sim_time_s': capture_time, 'world_point_m': world_point,
                                    'visible_claim': result['visible'],
                                }
                        self.motion_segment.add_plan({
                            **self.lightnav_latest, 'waypoints_world': path_world.tolist(),
                        })
                        if self.stop_candidate:
                            self.invalidate_path()
                            self.state = 'ARRIVAL_CANDIDATE'
                            self.stop_candidate = True
                            self.publish()
                            continue
                        if self.path is None and reference_source == 'lightnav':
                            self.reset_progress()
                        self.path = path_world
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
                    input_time = float(self.sim.data.time)
                    pose = self.sim.pose()
                    actual_twist = self.sim.planar_twist()
                    previous_command = (float(self.effective[0]), float(self.effective[2]))
                    estimated_delay = self.mpc_execution_delay_s
                    obstacles, map_capture_time = self.obstacles_world, self.map_capture_time
                    reference_source = self.reference_source
                    reference_times = (np.arange(self.controller.horizon) + 1) * self.controller.dt_s
                    if reference_source == 'metric_goal':
                        delta = np.asarray(self.goal_point_world) - pose[:2]
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
                        reference = build_pose_aligned_reference(self.path, pose, horizon=self.controller.horizon, weights=Q_WEIGHTS)
                    translation_allowed = self.maneuver not in ('turn_left', 'turn_right')
                    if reference_source == 'metric_goal':
                        translation_allowed = goal_phase == 'approach'
                    if not translation_allowed:
                        reference[:, :2] = pose[:2]
                    if not np.any(self.depth_frame.valid_mask):
                        self.pause('depth_unknown')
                        continue
                    try:
                        result = await asyncio.to_thread(
                            self.controller.solve, pose, reference, previous_command, obstacles,
                            actual_planar_twist=actual_twist, execution_delay_s=estimated_delay,
                            speed_limit=self.args.max_speed, translation_allowed=translation_allowed)
                    except MPCInfeasible as error:
                        if (generation == self.generation and self.state == 'NAVIGATING'
                                and goal_phase_revision == self.goal_phase_revision):
                            self.mpc_status = {'solver_status': 'blocked', 'reason': str(error),
                                               'reference_source': reference_source,
                                               'goal_phase': goal_phase, 'goal_phase_revision': goal_phase_revision,
                                               'translation_allowed': translation_allowed}
                            self.mpc_log.write(json.dumps({
                                'sim_time_s': float(self.sim.data.time), 'capture_time_s': map_capture_time,
                                'input_sim_time_s': input_time, 'pose_world': pose.tolist(),
                                'submitted_pose_world': self.sim.pose().tolist(),
                                'actual_planar_twist': actual_twist.tolist(),
                                'previous_command': previous_command, 'execution_delay_s': estimated_delay,
                                'reference_world': reference.tolist(),
                                'obstacles_world': obstacles.tolist(), **self.mpc_status,
                            }) + '\n')
                            self.pause('mpc_blocked')
                        continue
                    if (generation == self.generation and self.state == 'NAVIGATING'
                            and goal_phase_revision == self.goal_phase_revision):
                        self.mpc_execution_delay_s = float(self.sim.data.time) - input_time
                        self.requested[:] = (result.command[0], 0., result.command[1])
                        self.mpc_status = {
                            'reference_source': reference_source,
                            'goal_phase': goal_phase, 'goal_phase_revision': goal_phase_revision,
                            'translation_allowed': translation_allowed,
                            'solver_status': result.solver_status, 'solve_ms': result.solve_ms,
                            'minimum_clearance_m': result.minimum_clearance_m,
                            'obstacle_count': result.obstacle_count,
                            'execution_delay_s': result.execution_delay_s,
                            'actual_execution_delay_s': self.mpc_execution_delay_s,
                        }
                        self.mpc_log.write(json.dumps({
                            'sim_time_s': float(self.sim.data.time), 'capture_time_s': map_capture_time,
                            'input_sim_time_s': input_time, 'pose_world': pose.tolist(),
                            'submitted_pose_world': self.sim.pose().tolist(),
                            'actual_planar_twist': actual_twist.tolist(),
                            'previous_command': previous_command,
                            'reference_world': reference.tolist(),
                            'obstacles_world': obstacles.tolist(),
                            'predicted_world': result.predicted_world.tolist(),
                            'predicted_time_s': result.predicted_time_s.tolist(),
                            'command': list(result.command), **self.mpc_status,
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
    maneuver: Literal['advance', 'turn_left', 'turn_right', 'follow_target'] | None = None
    turn_degrees: float | None = None
    advance_m: float | None = None
    goal_point_world: tuple[float, float] | None = None
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
            p.mpc_log.close()
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
            'X-Scene-Pose': json.dumps(p.capture_pose.tolist()),
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
        p.force_until = float(p.sim.data.time) + body.duration_s
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
    parser.add_argument('--scene', choices=['flat', 'procthor', 'elevator_lobby', 'room_elevator'], default='procthor')
    profiles = json.loads((ROOT / 'prototype/camera/profiles.json').read_text())
    parser.add_argument('--camera', choices=[name for name, profile in profiles['profiles'].items() if 'depth_range_m' in profile],
                        default=profiles['default_profile'])
    parser.add_argument('--spawn-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--record-navigation-inputs', action='store_true')
    parser.add_argument('--semantic-checkpoint', default=str(ROOT / '.local/models/perception/yoloe-26l-seg.pt'))
    parser.add_argument('--record-contacts', action='store_true', help='Record 5 ms robot/environment contacts for offline diagnosis, not Agent input.')
    parser.add_argument('--lightnav', default='ws://127.0.0.1:8050')
    parser.add_argument('--inference-timeout', type=float, default=60.)
    parser.add_argument('--max-speed', type=float, default=.3)
    parser.add_argument('--no-progress-seconds', type=float, default=8.)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    uvicorn.run(make_app(args), host='127.0.0.1', port=args.port, log_level='info')
