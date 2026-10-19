"""Bounded observation motion and chronological RGB evidence, without scene truth.

These explicit planner actions send rate-limited twists to the existing WBC.
They are not a fallback navigation controller or an obstacle-clearance check.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from .planner_contract import LIMITS, SKILLS, validate_decision


class ObservationSkill:
    def __init__(self, decision, state, output, skill_id, acceleration_limits):
        validate_decision(decision)
        if decision['action'] not in SKILLS:
            raise ValueError('ObservationSkill requires an observation action')
        self.request, self.skill_id = dict(decision), skill_id
        self.output = Path(output)/f'skill_{skill_id:03d}'
        self.output.mkdir(parents=True)
        (self.output/'request.json').write_text(json.dumps(decision, indent=2)+'\n')
        self.epoch, self.start_s = state.epoch, float(state.stamp_s)
        self.start_pose = state.pose_odom.copy()
        self.last_pose = self.start_pose.copy()
        self.yaw_rad, self.travel_m = 0., 0.
        self.command = np.zeros(2)
        self.acceleration = np.asarray(acceleration_limits, dtype=float)
        self.stage = 'MOTION'
        self.motion_end_s = None
        self.motion_outcome = None
        self.stable_since = None
        self.finished = False
        self.status = None
        self.samples = []
        self.next_sample_s = self.start_s
        self.executed_ticks, self.nonzero_ticks = 0, 0
        self.command_integral = np.zeros(2)
        self.max_command = np.zeros(2)
        self.interruption = None
        self.alignment_corrections = 0
        self.turn_direction = (math.copysign(1., decision['angle_deg'])
            if decision['action'] == 'turn_observe' else None)

    def interrupt_force_stop(self, state, effective_command, dt, trigger):
        """Account for the preempting zero tick without claiming the skill finished."""
        self.command[:] = 0.
        self._settle(state, 'INTERRUPTED_EXTERNAL_FORCE')
        self.observe(state, effective_command, dt)
        self.status, self.finished = 'INTERRUPTED_EXTERNAL_FORCE', True
        self.interruption = trigger

    def _settle(self, state, outcome):
        self.stage, self.motion_outcome = 'SETTLING', outcome
        self.motion_end_s = float(state.stamp_s)

    def next_command(self, state, dt):
        if state.epoch != self.epoch or state.tracking != 'tracking':
            raise RuntimeError('Observation motion lost its sensor-estimated reference')
        elapsed = float(state.stamp_s)-self.start_s
        desired = np.zeros(2)
        action = self.request['action']
        if self.stage == 'MOTION':
            if action == 'turn_observe':
                remaining = math.radians(self.request['angle_deg'])-self.yaw_rad
                deadline = (abs(math.radians(self.request['angle_deg']))/
                    LIMITS['turn_timeout_nominal_rate_rad_s']+LIMITS['turn_timeout_extra_s'])
                # Aim at the requested angle, rather than stopping at the near
                # edge of its acceptance band. Switching to stance may change
                # yaw; only the settled outcome is compared with that band.
                if self.turn_direction*remaining <= 0.:
                    self._settle(state, 'ANGLE_REACHED_ESTIMATED')
                elif elapsed >= deadline:
                    outcome = ('ANGLE_REACHED_ESTIMATED' if abs(remaining) <=
                        math.radians(LIMITS['turn_tolerance_deg']) else 'ANGLE_NOT_REACHED')
                    self._settle(state, outcome)
                else:
                    desired[1] = np.clip(1.2*remaining, -LIMITS['turn_rate_rad_s'], LIMITS['turn_rate_rad_s'])
            elif elapsed >= self.request['duration_s']:
                self._settle(state, 'TIMED_COMMAND_FINISHED')
            elif action == 'advance_observe':
                desired[0] = self.request['speed_m_s']
        # Explicit bounded skill control; no post-processing of an MPPI command.
        self.command += np.clip(desired-self.command, -self.acceleration*dt, self.acceleration*dt)
        return self.command.copy()

    def observe(self, state, effective_command, dt):
        pose = state.pose_odom
        change = pose[2]-self.last_pose[2]
        self.yaw_rad += float(np.arctan2(np.sin(change), np.cos(change)))
        self.travel_m += float(np.linalg.norm(pose[:2]-self.last_pose[:2]))
        self.last_pose = pose.copy()
        command = np.asarray(effective_command)[[0, 2]]
        if not np.allclose(command, self.command, atol=1e-7, rtol=0.):
            raise RuntimeError('Executed observation command differs from accepted skill command')
        self.executed_ticks += 1
        self.nonzero_ticks += int(np.any(command != 0.))
        self.command_integral += command*dt
        self.max_command = np.maximum(self.max_command, np.abs(command))
        if self.stage == 'SETTLING':
            stopped = (np.linalg.norm(state.twist_base[:2]) < .08 and abs(state.planar_twist[2]) < .1
                and np.max(np.abs(self.command)) < 1e-8)
            if stopped:
                if self.stable_since is None:
                    self.stable_since = float(state.stamp_s)
                if float(state.stamp_s)-self.stable_since >= LIMITS['stable_stop_s']:
                    if self.request['action'] == 'turn_observe':
                        remaining = math.radians(self.request['angle_deg'])-self.yaw_rad
                        deadline = (abs(math.radians(self.request['angle_deg']))/
                            LIMITS['turn_timeout_nominal_rate_rad_s']+LIMITS['turn_timeout_extra_s'])
                        if abs(remaining) > math.radians(LIMITS['turn_tolerance_deg']):
                            if float(state.stamp_s)-self.start_s < deadline:
                                # Stance transition can change yaw after the first
                                # threshold crossing. Close the original angle loop
                                # through settling, within the SAME skill deadline.
                                self.stage = 'MOTION'
                                self.stable_since = None
                                self.motion_end_s = None
                                self.turn_direction = math.copysign(1., remaining)
                                self.alignment_corrections += 1
                                return
                            self.motion_outcome = 'ANGLE_NOT_REACHED'
                    self.status, self.finished = self.motion_outcome, True
            else:
                self.stable_since = None
            if not self.finished and float(state.stamp_s)-self.motion_end_s >= LIMITS['settle_timeout_s']:
                self.status, self.finished = 'STOP_NOT_CONFIRMED', True

    def sample(self, frame, *, force=False):
        stamp = float(frame['stamp_s'])
        if self.samples and self.samples[-1]['frame_id'] == frame['frame'].frame_id:
            return
        if not force and stamp < self.next_sample_s-1e-8:
            return
        pose = frame['frame'].state.pose_odom
        self.samples.append(dict(frame_id=frame['frame'].frame_id, path=str(frame['path']),
            sensor_time_s=stamp, elapsed_s=stamp-self.start_s,
            estimated_relative_yaw_deg=math.degrees(self.yaw_rad),
            estimated_displacement_xy_m=(pose[:2]-self.start_pose[:2]).tolist()))
        while self.next_sample_s <= stamp+1e-8:
            self.next_sample_s += LIMITS['sample_period_s']

    def finish(self, state, frame):
        if not self.finished:
            raise RuntimeError('Observation evidence cannot claim completion before execution ends')
        self.sample(frame, force=True)
        if not self.samples:
            raise RuntimeError('Observation produced no RGB evidence')
        count = min(len(self.samples), LIMITS['max_contact_sheet_frames'])
        selected = [self.samples[i] for i in np.rint(np.linspace(0, len(self.samples)-1, count)).astype(int)]
        sheet = self._contact_sheet(selected)
        angle_error_deg = None if self.request['action'] != 'turn_observe' else (
            self.request['angle_deg']-math.degrees(self.yaw_rad))
        if (angle_error_deg is not None and self.status == 'ANGLE_REACHED_ESTIMATED'
                and abs(angle_error_deg) > LIMITS['turn_tolerance_deg']):
            self.status = 'ANGLE_NOT_REACHED'
        result = dict(skill_id=self.skill_id, action=self.request['action'], status=self.status,
            motion_outcome=self.motion_outcome, request=self.request,
            force_stop_interruption=self.interruption,
            start_sensor_time_s=self.start_s, motion_end_sensor_time_s=self.motion_end_s,
            end_sensor_time_s=float(state.stamp_s),
            estimated_yaw_change_deg=math.degrees(self.yaw_rad), estimated_travel_m=self.travel_m,
            estimated_final_angle_error_deg=angle_error_deg,
            estimated_displacement_xy_m=(state.pose_odom[:2]-self.start_pose[:2]).tolist(),
            estimated_stopped=bool(self.stable_since is not None and
                float(state.stamp_s)-self.stable_since >= LIMITS['stable_stop_s']),
            executed_command_ticks=self.executed_ticks, nonzero_command_ticks=self.nonzero_ticks,
            alignment_corrections_within_original_deadline=self.alignment_corrections,
            command_integral_forward_m_yaw_rad=self.command_integral.tolist(),
            max_abs_command_vx_wz=self.max_command.tolist(), sample_period_s=LIMITS['sample_period_s'],
            captured_samples=len(self.samples), selected_frames=selected, contact_sheet=str(sheet),
            control_source='Explicit observation twist executor -> existing WBC; navigation MPPI suspended',
            navigation_history='Same session retained; no navigation inference during the observation',
            limitation='Yaw/translation are IMU/WBC estimates, not physical truth. Command integral is not odometry. '
                'A chronological contact sheet is not a calibrated panorama, map, or free-space certificate.')
        (self.output/'samples.json').write_text(json.dumps(self.samples, indent=2)+'\n')
        (self.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        return result

    def _contact_sheet(self, samples):
        width, height, label = 480, 170, 48
        columns = min(3, len(samples))
        rows = math.ceil(len(samples)/columns)
        sheet = Image.new('RGB', (columns*width, 62+rows*(height+label)), '#f1f4f8')
        draw = ImageDraw.Draw(sheet)
        font = ImageFont.truetype('DejaVuSans.ttf', 15)
        draw.text((12, 8), f'OBSERVATION {self.skill_id}: {self.request["action"]}', fill='#14324b', font=font)
        draw.text((12, 32), 'Time-ordered RGB views | estimated angles | not a calibrated panorama', fill='#4d6172', font=font)
        for index, sample in enumerate(samples):
            x, y = (index % columns)*width, 62+(index//columns)*(height+label)
            with Image.open(sample['path']) as image:
                tile = ImageOps.contain(image.convert('RGB'), (width-8, height))
            sheet.paste(tile, (x+(width-tile.width)//2, y))
            draw.text((x+8, y+height+2), f'{index+1:02d}  t={sample["sensor_time_s"]:.2f}s  '
                f'rel yaw={sample["estimated_relative_yaw_deg"]:+.1f} deg', fill='#14324b', font=font)
            draw.text((x+8, y+height+24), sample['frame_id'], fill='#4d6172', font=font)
        path = self.output/'contact_sheet.jpg'
        sheet.save(path, quality=95, subsampling=0)
        return path
