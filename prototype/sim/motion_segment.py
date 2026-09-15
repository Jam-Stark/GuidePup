"""Bounded presentation of actual control-tick poses, separate from model plans."""
from __future__ import annotations

import copy
import math

import numpy as np


class MotionSegment:
    def __init__(self, task_id, sim_time, pose, log_paths):
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
        self.log_paths = log_paths
        self.update(sim_time, pose)

    def update(self, sim_time, pose):
        pose = np.asarray(pose, dtype=np.float64)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError(f'Invalid simulator pose: {pose}')
        if self.samples and sim_time == self.samples[-1]['sim_time_s']:
            return
        if self.samples and sim_time < self.samples[-1]['sim_time_s']:
            raise ValueError('Motion segment time moved backwards')
        delta = pose - self.last_pose
        self.path_length += float(np.linalg.norm(delta[:2]))
        self.yaw_change += math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        self.last_pose = pose.copy()
        self.sample_count += 1
        self.samples.append({'sim_time_s': float(sim_time), 'pose_world': pose.tolist()})
        if len(self.samples) > 32:
            # Remove the least informative interior point, keeping both boundaries.
            # Position error is metres; yaw error uses a 0.3 m body-scale radius.
            errors = []
            for i in range(1, len(self.samples) - 1):
                a, b, c = self.samples[i - 1:i + 2]
                fraction = (b['sim_time_s'] - a['sim_time_s']) / (c['sim_time_s'] - a['sim_time_s'])
                pa, pb, pc = (np.asarray(s['pose_world']) for s in (a, b, c))
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

    def event(self, sim_time, state, reason):
        value = {'sim_time_s': float(sim_time), 'state': state, 'reason': reason}
        if not self.events or (state, reason) != (self.events[-1]['state'], self.events[-1]['reason']):
            self.events.append(value)

    def payload(self):
        return copy.deepcopy({
            'control_task_id': self.task_id,
            'start_sim_time_s': self.samples[0]['sim_time_s'],
            'end_sim_time_s': self.samples[-1]['sim_time_s'],
            'source': 'simulator_pose',
            'actual': {'samples': self.samples, 'sample_count': self.sample_count,
                       'path_length_m': self.path_length,
                       'net_displacement_m': float(np.linalg.norm(self.last_pose[:2] - self.start_pose[:2])),
                       'yaw_change_rad': self.yaw_change},
            'plans': self.plans, 'plan_count': self.plan_count,
            'events': self.events, 'log_paths': self.log_paths,
        })
