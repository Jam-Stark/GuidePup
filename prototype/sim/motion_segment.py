"""Bounded presentation of sensor-estimated poses, separate from model plans."""
from __future__ import annotations

import copy
import math

import numpy as np


class MotionSegment:
    def __init__(self, task_id, state, log_paths):
        stamp, pose = state.stamp_s, state.pose_odom
        self.epoch = state.epoch
        self.source = state.source
        self.task_id = task_id
        self.samples = []
        self.sample_count = 0
        self.path_length = 0.
        self.yaw_change = 0.
        self.start_pose = np.asarray(pose, dtype=np.float64).copy()
        self.last_pose = self.start_pose.copy()
        self.plans = []
        self.plan_count = 0
        self.events = []
        self.control_execution = {'control_ticks': 0, 'nonzero_ticks': 0,
            'first_stamp_s': None, 'last_stamp_s': None, 'last_effective_cmd': None,
            'command_min': None, 'command_max': None,
            'meaning': 'Velocity command actually submitted to locomotion; estimated motion is reported separately.'}
        self.log_paths = log_paths
        self.update(state)

    def update(self, state):
        if state.epoch != self.epoch:
            raise ValueError("Motion segment cannot cross estimator epochs")
        stamp, pose = state.stamp_s, state.pose_odom
        pose = np.asarray(pose, dtype=np.float64)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError(f'Invalid estimated pose: {pose}')
        if self.samples and stamp == self.samples[-1]['stamp_s']:
            return
        if self.samples and stamp < self.samples[-1]['stamp_s']:
            raise ValueError('Motion segment time moved backwards')
        delta = pose - self.last_pose
        self.path_length += float(np.linalg.norm(delta[:2]))
        self.yaw_change += math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        self.last_pose = pose.copy()
        self.sample_count += 1
        self.samples.append({'stamp_s': float(stamp), 'pose_odom': pose.tolist()})
        if len(self.samples) > 32:
            # Remove the least informative interior point, keeping both boundaries.
            # Position error is metres; yaw error uses a 0.3 m body-scale radius.
            errors = []
            for i in range(1, len(self.samples) - 1):
                a, b, c = self.samples[i - 1:i + 2]
                fraction = (b['stamp_s'] - a['stamp_s']) / (c['stamp_s'] - a['stamp_s'])
                pa, pb, pc = (np.asarray(s['pose_odom']) for s in (a, b, c))
                xy_error = np.linalg.norm(pb[:2] - (pa[:2] + fraction * (pc[:2] - pa[:2])))
                yaw_ac = math.atan2(math.sin(pc[2] - pa[2]), math.cos(pc[2] - pa[2]))
                yaw_error = pb[2] - pa[2] - fraction * yaw_ac
                errors.append(float(xy_error) + .3 * abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))))
            del self.samples[1 + int(np.argmin(errors))]

    def add_plan(self, plan):
        self.plan_count += 1
        self.plans.append(plan)
        if len(self.plans) > 3:
            del self.plans[1]

    def record_command(self, stamp, command):
        values = np.asarray(command, dtype=float)
        evidence = self.control_execution
        evidence['control_ticks'] += 1
        evidence['nonzero_ticks'] += int(np.any(np.abs(values) > 1e-6))
        if evidence['first_stamp_s'] is None:
            evidence['first_stamp_s'] = float(stamp)
            evidence['command_min'] = values.tolist()
            evidence['command_max'] = values.tolist()
        evidence['last_stamp_s'] = float(stamp)
        evidence['last_effective_cmd'] = values.tolist()
        evidence['command_min'] = np.minimum(evidence['command_min'], values).tolist()
        evidence['command_max'] = np.maximum(evidence['command_max'], values).tolist()

    def event(self, stamp, state, reason):
        value = {'stamp_s': float(stamp), 'state': state, 'reason': reason}
        if not self.events or (state, reason) != (self.events[-1]['state'], self.events[-1]['reason']):
            self.events.append(value)

    def payload(self):
        return copy.deepcopy({
            'control_task_id': self.task_id,
            'start_stamp_s': self.samples[0]['stamp_s'],
            'end_stamp_s': self.samples[-1]['stamp_s'],
            'source': self.source, 'epoch': self.epoch,
            'estimated': {'samples': self.samples, 'sample_count': self.sample_count,
                       'path_length_m': self.path_length,
                       'net_displacement_m': float(np.linalg.norm(self.last_pose[:2] - self.start_pose[:2])),
                       'yaw_change_rad': self.yaw_change},
            'plans': self.plans, 'plan_count': self.plan_count,
            'events': self.events, 'log_paths': self.log_paths,
            'control_execution': self.control_execution,
        })
