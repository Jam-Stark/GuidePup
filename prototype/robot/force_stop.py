"""Planner-recoverable STOP from the existing learned base-force estimate.

This detects a brake-level equivalent horizontal load, not a contact object.
The threshold and two-sample persistence come from the historical human brake;
direction and command eligibility are extended to cover turns and zero holds.
"""
from __future__ import annotations

import math

import numpy as np


class ForceStop:
    threshold_n = 60.
    consecutive_samples = 2

    def __init__(self, force_scale):
        self.force_scale = float(force_scale)
        if not np.isfinite(self.force_scale) or self.force_scale <= 0:
            raise ValueError('The exported policy must declare a positive base_force_scale')
        self.latched = False
        self.above_threshold_samples = 0
        self.estimate = None
        self.trigger = None

    def observe(self, raw_force, estimate_stamp_s, command_stamp_s):
        raw = np.asarray(raw_force, dtype=float)
        if raw.shape != (3,) or not np.isfinite(raw).all():
            raise ValueError('Base-force auxiliary output must be a finite 3-vector')
        stamp = float(estimate_stamp_s)
        if self.estimate is not None:
            if stamp < self.estimate['estimate_sensor_time_s']:
                raise RuntimeError('Force-estimate sensor time moved backwards')
            if stamp == self.estimate['estimate_sensor_time_s']:
                return  # One observation is not two independent control samples.
        force = raw/self.force_scale
        horizontal = float(np.linalg.norm(force[:2]))
        self.estimate = dict(estimate_sensor_time_s=stamp,
            raw_force_aux=raw.tolist(), force_estimate_body_yaw_n=force.tolist(),
            horizontal_magnitude_n=horizontal, magnitude_3d_n=float(np.linalg.norm(force)),
            force_direction_deg_from_forward_toward_left=None if horizontal == 0. else
                math.degrees(math.atan2(force[1], force[0])))
        self.above_threshold_samples = self.above_threshold_samples+1 if horizontal >= self.threshold_n else 0
        if not self.latched and self.above_threshold_samples >= self.consecutive_samples:
            self.latched = True
            self.trigger = dict(**self.estimate, command_stop_sensor_time_s=float(command_stamp_s),
                estimate_age_at_command_stop_s=float(command_stamp_s)-stamp,
                consecutive_samples=self.above_threshold_samples)

    def resume(self, reason):
        if not self.latched:
            raise RuntimeError('Cannot resume an external-force stop that is not active')
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('Resuming an external-force stop requires the planner decision reason')
        previous = self.payload()
        self.latched = False
        self.trigger = None
        self.above_threshold_samples = 0
        # Keep the latest estimate/time. Old samples cannot count again; two new
        # brake-level samples immediately re-stop even after a planner recovery.
        return dict(reason=reason, previous_stop=previous)

    def payload(self):
        return dict(latched=self.latched, threshold_horizontal_n=self.threshold_n,
            required_consecutive_samples=self.consecutive_samples,
            above_threshold_samples=self.above_threshold_samples,
            force_scale=self.force_scale, estimate=self.estimate, trigger=self.trigger,
            frame='Base yaw frame: +x forward, +y left, +z up. Vector is force ON the robot, not contact location.',
            source_kind='estimated_external_load', resume_authority='planner',
            authority='Stops all command sources until an explicit planner navigation/turn/advance decision. '
                'A hold keeps this stop active. This cannot clear a separate human brake.',
            source='Existing WBC 6D auxiliary estimator, force channels 3:6 / exported base_force_scale',
            limitation='Learned equivalent applied-base-load estimate. No calibrated collision recall, contact point, '
                'object identity or torque estimate. Vertical force is reported but does not trigger this planar brake.')
