"""CUDA MPPI trajectory tracking for the explicitly RGB-only deployment path.

No map, depth, stop-clearance certificate, or simulator state is accepted here.
The navigation model owns RGB obstacle reasoning. MPPI optimizes acceleration
commands within the existing forward/yaw motion space. A CUDA graph executes
the upstream pytorch-mppi sampler, rollouts, costs, weighting and final rollout.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np
import torch
from pytorch_mppi import MPPI


class GPUTrackingMPPI:
    def __init__(self, config_path=None):
        path = Path(config_path or Path(__file__).with_name('rgb_tracking.json'))
        self.config = json.loads(path.read_text())
        c = self.config
        if c['backend'] != 'pytorch_mppi_cuda_tracking' or c['collision_checking'] != 'unavailable_rgb_only':
            raise ValueError('This controller is explicitly RGB-only tracking, without geometric avoidance')
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA MPPI requires CUDA; no CPU controller is substituted')
        self.device = torch.device('cuda:0')
        self.dt, self.horizon = float(c['model_dt_s']), int(c['time_steps'])
        self.vmax, self.wmax = float(c['max_speed_m_s']), float(c['max_yaw_rate_rad_s'])
        if not 0 < self.vmax <= 1.2 or not 0 < self.wmax <= .6:
            raise ValueError('Tracking authority exceeds the selected A2 motion domain')
        if self.dt != .1 or c['max_acceleration_m_s2'] != .4 or c['max_yaw_acceleration_rad_s2'] != .8:
            raise ValueError('Retain the existing 10 Hz control and acceleration limits')
        self.input_state = torch.zeros(8, device=self.device)
        self.reference = torch.zeros((self.horizon, 3), device=self.device)
        self.ev = math.exp(-self.dt/c['velocity_response_time_s'])
        self.ew = math.exp(-self.dt/c['yaw_response_time_s'])
        self.noise_std = torch.tensor(c['acceleration_noise_std'], device=self.device)
        bound = torch.tensor([c['max_acceleration_m_s2'], c['max_yaw_acceleration_rad_s2']], device=self.device)
        self.optimizer = MPPI(self._dynamics, self._cost, 8, torch.diag(self.noise_std**2),
            num_samples=c['batch_size'], horizon=self.horizon, device=self.device,
            terminal_state_cost=self._terminal_cost, lambda_=c['temperature'],
            u_min=-bound, u_max=bound, step_dependent_dynamics=True,
            U_init=torch.zeros((self.horizon, 2), device=self.device), sample_null_action=True)
        # Warm up the actual library on a side stream before graph capture.
        stream = torch.cuda.Stream()
        stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream), torch.no_grad():
            for _ in range(3):
                self.optimizer.command(self.input_state, shift_nominal_trajectory=False)
        torch.cuda.current_stream().wait_stream(stream)
        torch.cuda.synchronize()
        self.optimizer.U.zero_()
        self.nominal = self.optimizer.U
        self.graph = torch.cuda.CUDAGraph()
        with torch.no_grad(), torch.cuda.graph(self.graph):
            self.optimizer.command(self.input_state, shift_nominal_trajectory=False)
            state = self.input_state.unsqueeze(0)
            knots, commands = [], []
            for t in range(self.horizon):
                state = self._dynamics(state, self.optimizer.U[t].unsqueeze(0), t)
                knots.append(state[0, :6])
                commands.append(state[0, 6:8])
            self.prediction = torch.stack(knots)
            self.commands = torch.stack(commands)
            self.costs = self.optimizer.cost_total
            self.final_accelerations = self.optimizer.U
            # The graph always reads this stable storage on the next replay.
            self.nominal.copy_(torch.roll(self.optimizer.U, -1, dims=0))
        self.epoch = None
        self.start_event, self.end_event = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
        props = torch.cuda.get_device_properties(0)
        self.info = dict(backend=c['backend'], device=str(self.device), device_uuid=str(props.uuid),
            device_name=props.name, samples=c['batch_size'], horizon=self.horizon,
            cuda_graph=True, collision_checking='unavailable_rgb_only',
            state_source='IMU and A2 policy auxiliary velocity estimate; no simulator pose/twist')

    def _dynamics(self, x, a, t):
        c, dt = self.config, self.dt
        vc = torch.clamp(x[:, 6]+a[:, 0]*dt, 0., self.vmax)
        wc = torch.clamp(x[:, 7]+a[:, 1]*dt, -self.wmax, self.wmax)
        forward = dt*vc+(x[:, 3]-vc)*c['velocity_response_time_s']*(1-self.ev)
        lateral = x[:, 4]*c['velocity_response_time_s']*(1-self.ev)
        turn = dt*wc+(x[:, 5]-wc)*c['yaw_response_time_s']*(1-self.ew)
        angle = x[:, 2]+turn/2
        co, si = torch.cos(angle), torch.sin(angle)
        return torch.stack((x[:, 0]+co*forward-si*lateral, x[:, 1]+si*forward+co*lateral,
            x[:, 2]+turn, vc+(x[:, 3]-vc)*self.ev, x[:, 4]*self.ev,
            wc+(x[:, 5]-wc)*self.ew, vc, wc), dim=-1)

    def _cost(self, x, a, t):
        c = self.config
        error = x[:, :3]-self.reference[t]
        yaw = torch.atan2(torch.sin(error[:, 2]), torch.cos(error[:, 2]))
        return (c['position_weight']*torch.sum(error[:, :2]**2, dim=-1)
            +c['heading_weight']*yaw**2+c['acceleration_weight']*torch.sum(a**2, dim=-1))

    def _terminal_cost(self, states, actions):
        error = states[:, :, -1, :3]-self.reference[-1]
        yaw = torch.atan2(torch.sin(error[..., 2]), torch.cos(error[..., 2]))
        return self.config['terminal_weight']*(self.config['position_weight']*torch.sum(error[..., :2]**2, dim=-1)
            +self.config['heading_weight']*yaw**2)

    def _reference_at_times(self, path, pose):
        # Arc length includes angular progress so in-place turn references work.
        path = np.asarray(path, dtype=float).copy()
        if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all():
            raise ValueError('Tracking needs a finite (N>=2, 3) estimated-odom path')
        path[:, 2] = np.unwrap(path[:, 2])
        path[:, 2] += 2*math.pi*round((pose[2]-path[0, 2])/(2*math.pi))
        delta = np.diff(path, axis=0)
        increments = np.maximum(np.linalg.norm(delta[:, :2], axis=1), self.vmax/self.wmax*np.abs(delta[:, 2]))
        arc = np.r_[0., np.cumsum(increments)]
        keep = np.r_[True, increments > 1e-7]
        if keep.sum() < 2:
            return np.repeat(path[-1:], self.horizon, axis=0)
        path, arc = path[keep], arc[keep]
        # Project onto actual path segments rather than jumping to a distant knot.
        segments = path[1:, :2]-path[:-1, :2]
        norm = np.sum(segments**2, axis=1)
        fraction = np.divide(np.sum((pose[:2]-path[:-1, :2])*segments, axis=1), norm,
                             out=np.zeros_like(norm), where=norm > 1e-12).clip(0., 1.)
        projected = path[:-1, :2]+fraction[:, None]*segments
        angular = np.abs(np.arctan2(np.sin(path[:-1, 2]-pose[2]), np.cos(path[:-1, 2]-pose[2])))
        closest = np.argmin(np.linalg.norm(projected-pose[:2], axis=1)+.05*angular)
        start = arc[closest]+fraction[closest]*(arc[closest+1]-arc[closest])
        targets = np.minimum(start+self.vmax*self.dt*np.arange(1, self.horizon+1), arc[-1])
        return np.stack([np.interp(targets, arc, path[:, i]) for i in range(3)], axis=1)

    @torch.no_grad()
    def solve(self, state, path_odom, previous_command):
        started = time.monotonic()
        if state.tracking != 'tracking':
            raise RuntimeError('RGB tracking requires a valid sensor-estimated state')
        pose, twist = state.pose_odom, state.planar_twist
        previous = np.asarray(previous_command, dtype=float)
        if previous.shape != (2,) or not all(np.isfinite(x).all() for x in (pose, twist, previous)):
            raise ValueError('Nonfinite or malformed tracking state/command')
        if not 0 <= previous[0] <= self.vmax+1e-6 or abs(previous[1]) > self.wmax+1e-6:
            raise ValueError('Previous command exceeds configured authority')
        if state.epoch != self.epoch:
            self.nominal.zero_()
            self.epoch = state.epoch
        reference = self._reference_at_times(path_odom, pose)
        self.reference.copy_(torch.as_tensor(reference, dtype=torch.float32, device=self.device))
        self.input_state.copy_(torch.as_tensor(np.r_[pose, twist, previous], dtype=torch.float32, device=self.device))
        self.start_event.record()
        self.graph.replay()
        self.end_event.record()
        self.end_event.synchronize()
        commands, prediction, costs, accelerations = (x.cpu().numpy().copy() for x in
            (self.commands, self.prediction, self.costs, self.final_accelerations))
        if not all(np.isfinite(x).all() for x in (commands, prediction, costs, accelerations)):
            raise RuntimeError('CUDA MPPI returned nonfinite costs or trajectories')
        if (np.any(commands[:, 0] < -1e-6) or np.any(commands[:, 0] > self.vmax+1e-6)
                or np.any(np.abs(commands[:, 1]) > self.wmax+1e-6)):
            raise RuntimeError('CUDA MPPI command sequence violates authority')
        changes = np.diff(np.vstack([previous, commands]), axis=0)/self.dt
        if np.any(np.abs(changes) > np.array([.4, .8])+1e-5):
            raise RuntimeError('CUDA MPPI command sequence violates acceleration bounds')
        return dict(command=commands[0].tolist(), controls=commands.tolist(),
            accelerations=accelerations.tolist(), predicted_odom=prediction[:, :3].tolist(),
            reference_odom=reference.tolist(), candidate_costs=costs.tolist(),
            gpu_ms=float(self.start_event.elapsed_time(self.end_event)),
            solve_ms=(time.monotonic()-started)*1000, **self.info,
            minimum_clearance_m=None, clearance_evidence='unavailable_rgb_only',
            stopping_model='Nominal first-order tracking response only; no calibrated clearance envelope')
