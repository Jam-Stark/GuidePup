"""Native Nav2 MPPI over JSON-lines; all inputs use the shared sensor odom epoch.

The library owns sampling, optimization and path critics. The custom native critic
uses A2's measured response and empirical stop envelope, as does final validation.
Only occupied depth voxel centres are collision evidence; unknown is not free.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.spatial import cKDTree
from .stop_model import StopModel


class LocalControlBlocked(RuntimeError):
    """A classified execution refusal, distinct from an interface/runtime error."""

    def __init__(self, reason, *, category, diagnostics):
        super().__init__(reason)
        self.category = category
        self.diagnostics = diagnostics


@dataclass(frozen=True)
class MPPIResult:
    command: tuple[float, float]
    predicted_odom: np.ndarray
    predicted_time_s: np.ndarray
    execution_delay_s: float
    minimum_clearance_m: float | None
    solver_status: str
    solve_ms: float
    obstacle_count: int
    stop_minimum_clearance_m: float | None
    stop_reserve_m: float | None
    stop_model_id: str
    diagnostics: dict



class Nav2MPPI:
    def __init__(self, *, max_speed=.3, footprint_path=None, config_path=None, log_path=None):
        directory = Path(__file__).resolve().parent
        self.config = json.loads(Path(config_path or directory / 'nav2/config.json').read_text())
        footprint = json.loads(Path(footprint_path or directory / 'footprint.json').read_text())
        self.centers = np.asarray(footprint['centers_base_m'], dtype=float)
        self.radii = np.asarray(footprint['radii_m'], dtype=float)
        self.cell_radius_m = footprint['obstacle_cell_side_m'] / math.sqrt(2.)
        self.minimum_swept_clearance_m = float(footprint['minimum_swept_clearance_m'])
        self.velocity_response_time_s = float(footprint['velocity_response_time_s'])
        self.yaw_response_time_s = float(footprint['yaw_response_time_s'])
        self.desired_clearance_m = float(footprint['desired_clearance_m'])
        self.horizon, self.dt_s = int(self.config['time_steps']), float(self.config['model_dt'])
        self.subsamples, self.control_dt_s = 2, .1
        if self.dt_s != .1:
            raise ValueError('Native bridge uses .1s controls with .05s collision samples')
        self.max_speed, self.w_max = float(max_speed), .6
        self.stop_model = StopModel(directory / 'stop_model.json')
        if not 0 < self.max_speed <= self.stop_model.domain['max_speed_m_s']:
            raise ValueError('Navigation max_speed exceeds the calibrated stopping domain')
        self.stop_circle_residuals = self.stop_model.circle_residuals(self.centers)
        self.maximum_between_sample_motion_m = (self.max_speed + np.linalg.norm(self.centers, axis=1).max() * self.w_max) * .025
        self.hard_clearance_m = self.minimum_swept_clearance_m + self.maximum_between_sample_motion_m
        log_path = Path(log_path or directory.parents[1] / '.local/outputs/nav2_mppi.log')
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = log_path.open('a', buffering=1)
        self._journal = log_path.with_suffix('.jsonl').open('a', buffering=1)
        env = os.environ.copy()
        env['RCUTILS_LOGGING_USE_STDOUT'] = '0'
        self._process = subprocess.Popen([str(directory / 'nav2/run.sh')], stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, stderr=self._log, text=True, bufsize=1, env=env)
        initialization = dict(self.config, footprint=footprint, max_speed=self.max_speed,
                              preferred_clearance_m=self.desired_clearance_m,
                              clearance_cost_weight=float(footprint['clearance_cost_weight']),
                              stop_model=json.loads((directory / 'stop_model.json').read_text()))
        self.native_info = self._exchange(initialization)
        if self.native_info['status'] != 'ready':
            raise RuntimeError(f'Native MPPI initialization failed: {self.native_info}')

    def _exchange(self, request):
        self._journal.write(json.dumps({'direction': 'request', 'record': request}, allow_nan=False) + '\n')
        self._process.stdin.write(json.dumps(request, allow_nan=False) + '\n')
        self._process.stdin.flush()
        line = self._process.stdout.readline()
        if not line:
            raise RuntimeError(f'Native MPPI exited ({self._process.poll()}); see {self._log.name}')
        reply = json.loads(line)
        self._journal.write(json.dumps({'direction': 'response', 'record': reply}, allow_nan=False) + '\n')
        return reply

    def close(self):
        if self._process.poll() is None:
            self._process.stdin.close()
            self._process.wait(timeout=10)
        self._log.close()
        self._journal.close()

    def _advance_numpy(self, state, control, dt):
        v, w = control
        ev, ew = math.exp(-dt / self.velocity_response_time_s), math.exp(-dt / self.yaw_response_time_s)
        forward = dt * v + (state[3] - v) * self.velocity_response_time_s * (1. - ev)
        lateral = state[4] * self.velocity_response_time_s * (1. - ev)
        turn = dt * w + (state[5] - w) * self.yaw_response_time_s * (1. - ew)
        heading = state[2] + .5 * dt * w + (state[5] - w) * self.yaw_response_time_s * (1. - math.exp(-.5 * dt / self.yaw_response_time_s))
        c, s = math.cos(heading), math.sin(heading)
        return np.array([state[0] + forward * c - lateral * s, state[1] + forward * s + lateral * c,
                         state[2] + turn, v + (state[3] - v) * ev, state[4] * ev, w + (state[5] - w) * ew])

    def _centers_at(self, poses):
        yaw = poses[:, 2, None]
        x = poses[:, 0, None] + np.cos(yaw) * self.centers[None, :, 0] - np.sin(yaw) * self.centers[None, :, 1]
        y = poses[:, 1, None] + np.sin(yaw) * self.centers[None, :, 0] + np.cos(yaw) * self.centers[None, :, 1]
        return np.stack([x, y], axis=-1)

    def solve(self, state, reference_odom, previous_command, obstacles_odom,
              speed_limit=.3, *, execution_delay_s, translation_allowed=True):
        started = time.monotonic()
        pose, twist = np.asarray(state.pose_odom), np.asarray(state.planar_twist)
        reference = np.asarray(reference_odom, dtype=float)
        obstacles = np.asarray(obstacles_odom, dtype=float).reshape(-1, 2)
        previous = np.asarray(previous_command, dtype=float)
        delay = float(execution_delay_s)
        if state.tracking != 'tracking':
            raise ValueError('MPPI requires a tracked sensor RobotState')
        if reference.ndim != 2 or reference.shape[1] != 3 or len(reference) < 2:
            raise ValueError('MPPI requires the odom reference path with shape (N>=2,3)')
        if previous.shape != (2,) or not all(np.isfinite(a).all() for a in (pose, twist, reference, obstacles, previous)):
            raise ValueError('MPPI requires finite sensor pose, twist, path, obstacles and previous command')
        if not np.isfinite(delay) or delay < 0 or not 0 <= speed_limit <= self.max_speed:
            raise ValueError('Invalid execution delay or authority speed limit')
        if not translation_allowed:
            speed_limit = 0.
            reference = reference.copy()
            reference[:, :2] = pose[:2]
        bound = max(self.max_speed, float(np.linalg.norm(twist[:2]))) + np.linalg.norm(self.centers, axis=1).max() * max(self.w_max, abs(twist[2]))
        hard = self.minimum_swept_clearance_m + bound * .025
        prefix_times = np.linspace(0., delay, math.ceil(delay / .05) + 1)
        prefix = np.asarray([self._advance_numpy(np.r_[pose, twist], previous, t) for t in prefix_times])
        minimum = None
        prefix_evidence = {'current_geometric_clearance_m': None,
                           'committed_prefix_minimum_clearance_m': None,
                           'required_geometry_clearance_m': hard,
                           'desired_clearance_m': self.desired_clearance_m,
                           'execution_delay_s': delay,
                           'scope': 'Shared estimated state and observed voxel geometry, not physical contact truth.'}
        if len(obstacles):
            tree = cKDTree(obstacles)
            distances = tree.query(self._centers_at(prefix[:, :3]).reshape(-1, 2))[0].reshape(-1, len(self.radii))
            clearances = distances - self.radii - self.cell_radius_m
            minimum = float(np.min(clearances))
            current = float(np.min(clearances[0]))
            stops = self.stop_model.poses(prefix[-1])
            distances = tree.query(self._centers_at(stops).reshape(-1, 2))[0].reshape(-1, len(self.radii))
            stop_min = float(np.min(distances - self.radii - self.cell_radius_m - self.stop_circle_residuals))
            prefix_evidence.update(current_geometric_clearance_m=current,
                committed_prefix_minimum_clearance_m=minimum,
                delayed_empirical_stop_clearance_m=stop_min,
                delayed_empirical_stop_deficit_m=max(0., hard-stop_min),
                desired_clearance_shortfall_m=max(0., self.desired_clearance_m-minimum))
            # This command has already been sent and remains active during the
            # measured delay. A future candidate cannot change that geometry.
            # Empirical stopping uncertainty is NOT a precondition for planning.
            if minimum < hard:
                category = ('current_observed_overlap' if current < 0. else
                            'committed_predicted_overlap' if minimum < 0. else
                            'committed_geometry_margin')
                diagnostics = {'category': category, 'optimization_attempts': 0,
                               'prefix': prefix_evidence}
                self._journal.write(json.dumps({'direction': 'python_geometry_rejection',
                    'stamp_s': float(state.stamp_s), 'record': diagnostics}, allow_nan=False)+'\n')
                raise LocalControlBlocked(
                    f'{category}: committed geometry clearance={minimum:.4f} m, required={hard:.4f} m',
                    category=category, diagnostics=diagnostics)
        reply = self._exchange({'epoch': state.epoch, 'stamp_s': float(state.stamp_s), 'initial': prefix[-1].tolist(),
                                'reference': reference.tolist(), 'obstacles': obstacles.tolist(),
                                'speed_limit': float(speed_limit), 'hard_clearance_m': float(hard),
                                'sensor_pose_odom': pose.tolist(), 'sensor_twist_base': twist.tolist(),
                                'previous_command': previous.tolist(), 'execution_delay_s': delay,
                                'translation_allowed': bool(translation_allowed),
                                'prefix_evidence': prefix_evidence})
        if reply['status'] == 'blocked':
            raise LocalControlBlocked(reply['reason'], category=reply['category'],
                diagnostics={**reply['diagnostics'], 'prefix': prefix_evidence,
                             'optimization_attempts': reply['optimization_attempts']})
        if reply['status'] != 'success':
            raise RuntimeError(f'Native MPPI protocol error: {reply}')
        controls = np.asarray(reply['controls'], dtype=float)
        command = np.asarray(reply['command'], dtype=float)
        if controls.shape != (self.horizon, 2) or command.shape != (2,) or not np.isfinite(controls).all() or not np.isfinite(command).all():
            raise RuntimeError('Native MPPI returned malformed controls')
        if abs(command[0] - controls[0, 0]) > 1e-6 or abs(command[1] - controls[0, 1]) > 1e-6:
            raise RuntimeError('Native command differs from clearance-checked control sequence')
        if not (-1e-6 <= command[0] <= speed_limit + 1e-6 and abs(command[1]) <= self.w_max + 1e-6):
            raise RuntimeError('Native command exceeds configured authority')
        predicted = np.vstack([prefix[:, :3], np.asarray(reply['predicted'])[1:]])
        times = np.r_[prefix_times, delay + np.arange(1, self.horizon + 1) * self.dt_s]
        if minimum is not None:
            minimum = min(minimum, reply['minimum_clearance_m'])
        stop_min = reply['stop_minimum_clearance_m']
        return MPPIResult(tuple(command.tolist()), predicted, times, delay, minimum, 'nav2_mppi_success',
                          (time.monotonic() - started) * 1000., len(obstacles), stop_min,
                          None if stop_min is None else stop_min - hard, self.stop_model.model_id,
                          dict(self.native_info, **reply['diagnostics'], prefix=prefix_evidence,
                               optimization_attempts=reply['optimization_attempts']))
