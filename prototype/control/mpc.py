"""Depth constrained, actual-pose MPC for the old A2 policy.

All public poses and obstacle points are in the world XY frame, in metres/radians.
Obstacle points are occupied 4 cm voxel centres, not pre-inflated robot obstacles.
No simulator scene geometry or commanded-velocity odometry enters this module.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import casadi as ca
import numpy as np
from scipy.spatial import cKDTree


class MPCInfeasible(RuntimeError):
    """The caller must stop and report this; there is no unconstrained fallback."""


@dataclass(frozen=True)
class MPCResult:
    command: tuple[float, float]
    predicted_world: np.ndarray
    predicted_time_s: np.ndarray
    execution_delay_s: float
    minimum_clearance_m: float | None
    solver_status: str
    solve_ms: float
    obstacle_count: int


def _rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


class CollisionAwareMPC:
    """A velocity-response MPC whose A2 discs clear observed depth obstacles.

    Nearest-point constraints are updated from each solution until its complete
    sampled swept footprint clears every supplied occupied voxel. This keeps a
    fixed-size NLP without capping or discarding the local obstacle map.
    """

    def __init__(self, *, horizon=15, dt_s=0.15, control_dt_s=0.1, max_speed=0.3,
                 w_max=0.6, a_max_v=0.4, a_max_w=0.8,
                 footprint_path: str | Path | None = None):
        config = json.loads(Path(footprint_path or Path(__file__).with_name('footprint.json')).read_text())
        self.centers = np.asarray(config['centers_base_m'], dtype=float)
        self.radii = np.asarray(config['radii_m'], dtype=float)
        self.desired_clearance_m = float(config['desired_clearance_m'])
        self.minimum_swept_clearance_m = float(config['minimum_swept_clearance_m'])
        self.clearance_cost_weight = float(config['clearance_cost_weight'])
        self.velocity_response_time_s = float(config['velocity_response_time_s'])
        self.yaw_response_time_s = float(config['yaw_response_time_s'])
        self.cell_radius_m = float(config['obstacle_cell_side_m']) / math.sqrt(2.)
        self.horizon, self.dt_s = int(horizon), float(dt_s)
        self.control_dt_s = float(control_dt_s)
        self.max_speed, self.w_max = float(max_speed), float(w_max)
        self.a_max_v, self.a_max_w = float(a_max_v), float(a_max_w)
        self.subsamples = 3
        self.neighbors_per_circle = 4
        center_speed_bound = self.max_speed + np.linalg.norm(self.centers, axis=1).max() * self.w_max
        self.maximum_between_sample_motion_m = center_speed_bound * self.dt_s / (2. * self.subsamples)
        self.hard_clearance_m = self.minimum_swept_clearance_m + self.maximum_between_sample_motion_m
        if self.desired_clearance_m < self.hard_clearance_m:
            raise ValueError('Desired clearance must exceed the hard swept-footprint clearance.')
        self.required_distance = self.radii + self.hard_clearance_m + self.cell_radius_m
        self.desired_distance = self.radii + self.desired_clearance_m + self.cell_radius_m
        self._last_controls = None
        self._build_solver()

    def _advance(self, state, control, dt):
        """Integrate a held command with measured velocity as the initial state."""
        v, w = control[0], control[1]
        ev, ew = math.exp(-dt / self.velocity_response_time_s), math.exp(-dt / self.yaw_response_time_s)
        forward = dt * v + (state[3] - v) * self.velocity_response_time_s * (1. - ev)
        lateral = state[4] * self.velocity_response_time_s * (1. - ev)
        turn = dt * w + (state[5] - w) * self.yaw_response_time_s * (1. - ew)
        heading = state[2] + .5 * dt * w + (state[5] - w) * self.yaw_response_time_s * (
            1. - math.exp(-.5 * dt / self.yaw_response_time_s))
        return ca.horzcat(state[0] + forward * ca.cos(heading) - lateral * ca.sin(heading),
                          state[1] + forward * ca.sin(heading) + lateral * ca.cos(heading),
                          state[2] + turn, v + (state[3] - v) * ev, state[4] * ev,
                          w + (state[5] - w) * ew)

    def _advance_numpy(self, state, control, dt):
        v, w = control
        ev, ew = math.exp(-dt / self.velocity_response_time_s), math.exp(-dt / self.yaw_response_time_s)
        forward = dt * v + (state[3] - v) * self.velocity_response_time_s * (1. - ev)
        lateral = state[4] * self.velocity_response_time_s * (1. - ev)
        turn = dt * w + (state[5] - w) * self.yaw_response_time_s * (1. - ew)
        heading = state[2] + .5 * dt * w + (state[5] - w) * self.yaw_response_time_s * (
            1. - math.exp(-.5 * dt / self.yaw_response_time_s))
        c, s = math.cos(heading), math.sin(heading)
        return np.array([state[0] + forward * c - lateral * s,
                         state[1] + forward * s + lateral * c, state[2] + turn,
                         v + (state[3] - v) * ev, state[4] * ev, w + (state[5] - w) * ew])

    def _build_solver(self):
        opti = ca.Opti()
        controls = opti.variable(self.horizon, 2)
        states = opti.variable(self.horizon + 1, 6)
        initial = opti.parameter(6)
        reference = opti.parameter(self.horizon, 3)
        previous = opti.parameter(2)
        speed_limit = opti.parameter()
        query_count = (1 + self.horizon * self.subsamples) * len(self.centers)
        neighbors = opti.parameter(query_count * self.neighbors_per_circle, 2)
        have_obstacles = opti.parameter()
        hard_clearance = opti.parameter()
        opti.subject_to(states[0, :] == initial.T)
        opti.subject_to(opti.bounded(0., controls[:, 0], speed_limit))
        opti.subject_to(opti.bounded(-self.w_max, controls[:, 1], self.w_max))

        query_index = 0
        clearance_cost = 0.

        def constrain_footprint(pose, *, prefer_clearance=False):
            nonlocal query_index, clearance_cost
            c, s = ca.cos(pose[2]), ca.sin(pose[2])
            for center, radius, desired in zip(self.centers, self.radii, self.desired_distance):
                distance = float(radius) + self.cell_radius_m + hard_clearance
                xy = ca.vertcat(pose[0] + c * center[0] - s * center[1],
                                pose[1] + s * center[0] + c * center[1])
                for j in range(self.neighbors_per_circle):
                    point = neighbors[query_index * self.neighbors_per_circle + j, :].T
                    squared_distance = ca.sumsqr(xy - point)
                    constraint = squared_distance - distance ** 2
                    opti.subject_to(have_obstacles * constraint + (1. - have_obstacles) >= 0.)
                    if prefer_clearance:
                        # Policy tracking deviations may consume the desired
                        # margin. Keep it soft at prediction knots; all substeps
                        # still enforce the hard swept-footprint constraint.
                        # This shortfall has metre units and no square-root singularity.
                        shortfall = ca.fmax((float(desired) ** 2 - squared_distance) / (2. * float(desired)), 0.)
                        clearance_cost += (have_obstacles * self.clearance_cost_weight
                                           / self.neighbors_per_circle * shortfall ** 2)
                query_index += 1

        constrain_footprint(states[0, :])
        objective = 0.
        for k in range(self.horizon):
            opti.subject_to(states[k + 1, :] == self._advance(states[k, :], controls[k, :], self.dt_s))
            for fraction in range(1, self.subsamples + 1):
                pose = self._advance(states[k, :], controls[k, :], self.dt_s * fraction / self.subsamples)
                constrain_footprint(pose, prefer_clearance=fraction == self.subsamples)
            error = states[k + 1, :3] - reference[k, :]
            # Heading is a preference, so doorway clearance can override the VLN yaw.
            objective += 10. * (error[0] ** 2 + error[1] ** 2) + .4 * error[2] ** 2
            objective += .05 * ca.sumsqr(controls[k, :])
            prior = previous.T if k == 0 else controls[k - 1, :]
            delta = controls[k, :] - prior
            # Both commands may brake towards zero immediately. Measured
            # velocity response still limits how quickly the robot stops.
            slew_dt = self.control_dt_s if k == 0 else self.dt_s
            opti.subject_to(delta[0] <= self.a_max_v * slew_dt)
            opti.subject_to(opti.bounded(ca.fmin(prior[1], 0.) - self.a_max_w * slew_dt,
                                        controls[k, 1],
                                        ca.fmax(prior[1], 0.) + self.a_max_w * slew_dt))
            objective += .1 * ca.sumsqr(delta)
        objective += 5. * ca.sumsqr(states[-1, :2] - reference[-1, :2])
        opti.minimize(objective + clearance_cost)
        opti.solver('ipopt', {'expand': True, 'print_time': False}, {
            'max_iter': 100, 'print_level': 0, 'tol': 1e-5,
            'acceptable_tol': 1e-4, 'sb': 'yes',
        })
        self.opti, self.controls, self.states = opti, controls, states
        self.reference, self.previous, self.speed_limit = reference, previous, speed_limit
        self.neighbors, self.have_obstacles = neighbors, have_obstacles
        self.initial, self.hard_clearance = initial, hard_clearance

    def _rollout(self, controls, initial):
        states = np.empty((len(controls) + 1, 6))
        states[0] = initial
        for k, control in enumerate(controls):
            states[k + 1] = self._advance_numpy(states[k], control, self.dt_s)
        return states

    def _centers_at(self, poses):
        yaw = poses[:, 2, None]
        x = poses[:, 0, None] + np.cos(yaw) * self.centers[None, :, 0] - np.sin(yaw) * self.centers[None, :, 1]
        y = poses[:, 1, None] + np.sin(yaw) * self.centers[None, :, 0] + np.cos(yaw) * self.centers[None, :, 1]
        return np.stack([x, y], axis=-1)

    def _sample_poses(self, controls, initial):
        states = self._rollout(controls, initial)
        samples = [states[0, :3]]
        for k, control in enumerate(controls):
            for j in range(1, self.subsamples + 1):
                dt = self.dt_s * j / self.subsamples
                samples.append(self._advance_numpy(states[k], control, dt)[:3])
        return np.asarray(samples)

    def _nearest_constraints(self, controls, initial, tree):
        queries = self._centers_at(self._sample_poses(controls, initial)).reshape(-1, 2)
        count = min(self.neighbors_per_circle, tree.n)
        indices = tree.query(queries, k=count)[1].reshape(len(queries), count)
        if count < self.neighbors_per_circle:
            indices = np.pad(indices, ((0, 0), (0, self.neighbors_per_circle - count)), mode='edge')
        return tree.data[indices.ravel()]

    def _initial_guess(self, reference, previous, speed_limit, initial, tree, required_distance):
        """Choose a turning/forward seed; every output still uses the NLP."""
        if self._last_controls is not None:
            controls = np.vstack((self._last_controls[1:], self._last_controls[-1:]))
            prior = np.asarray(previous, dtype=float)
            for k in range(self.horizon):
                slew_dt = self.control_dt_s if k == 0 else self.dt_s
                controls[k, 0] = np.clip(controls[k, 0], 0., min(speed_limit, prior[0] + self.a_max_v * slew_dt))
                controls[k, 1] = np.clip(controls[k, 1], min(prior[1], 0.) - self.a_max_w * slew_dt,
                                       max(prior[1], 0.) + self.a_max_w * slew_dt)
                prior = controls[k]
            states = self._rollout(controls, initial)
            if tree is None:
                return controls, states
            distances = tree.query(self._centers_at(self._sample_poses(controls, initial)).reshape(-1, 2))[0].reshape(-1, len(self.radii))
            if np.all(distances >= required_distance):
                return controls, states
        best = None
        for target_v in (speed_limit, .5 * speed_limit, 0.):
            for target_w in (-self.w_max, -.5 * self.w_max, 0., .5 * self.w_max, self.w_max):
                controls = np.empty((self.horizon, 2))
                prior = np.asarray(previous, dtype=float)
                for k in range(self.horizon):
                    slew_dt = self.control_dt_s if k == 0 else self.dt_s
                    controls[k] = (min(target_v, prior[0] + self.a_max_v * slew_dt),
                                   np.clip(target_w, min(prior[1], 0.) - self.a_max_w * slew_dt,
                                           max(prior[1], 0.) + self.a_max_w * slew_dt))
                    prior = controls[k]
                poses = self._rollout(controls, initial)
                error = poses[1:, :3] - reference
                score = np.sum(10. * np.sum(error[:, :2] ** 2, axis=1) + .4 * error[:, 2] ** 2)
                score += 5. * np.sum((poses[-1, :2] - reference[-1, :2]) ** 2)
                if tree is not None:
                    distances = tree.query(self._centers_at(self._sample_poses(controls, initial)).reshape(-1, 2))[0].reshape(-1, len(self.radii))
                    score += 1e4 * np.sum(np.minimum(distances - required_distance, 0.) ** 2)
                    score += (self.clearance_cost_weight
                              * np.sum(np.minimum(distances[self.subsamples::self.subsamples]
                                                  - self.desired_distance, 0.) ** 2))
                if best is None or score < best[0]:
                    best = score, controls, poses
        return best[1], best[2]

    def solve(self, pose_world, reference_world, previous_command, obstacles_world,
              speed_limit=0.3, motion_allowed=True, *, actual_planar_twist,
              execution_delay_s, translation_allowed=True) -> MPCResult:
        started = time.monotonic()
        pose = np.asarray(pose_world, dtype=float)
        reference = np.asarray(reference_world, dtype=float)
        obstacles = np.asarray(obstacles_world, dtype=float).reshape(-1, 2)
        twist = np.asarray(actual_planar_twist, dtype=float)
        execution_delay_s = float(execution_delay_s)
        if pose.shape != (3,) or reference.shape != (self.horizon, 3):
            raise ValueError(f'Expected world pose (3,) and reference ({self.horizon},3).')
        if not np.all(np.isfinite(pose)) or not np.all(np.isfinite(reference)) or not np.all(np.isfinite(obstacles)):
            raise ValueError('MPC inputs must be finite world coordinates.')
        if twist.shape != (3,) or not np.all(np.isfinite(twist)) or not np.isfinite(execution_delay_s) or execution_delay_s < 0.:
            raise ValueError('Expected finite planar [vx,vy,wz] and nonnegative execution delay.')
        if not motion_allowed or (translation_allowed and speed_limit <= 0.):
            self._last_controls = None
            return MPCResult((0., 0.), np.repeat(pose[None, :], self.horizon + 1, axis=0),
                             np.arange(self.horizon + 1) * self.dt_s, 0.,
                             None, 'motion_paused', 0., len(obstacles))
        if speed_limit > self.max_speed:
            raise ValueError('Authority speed limit exceeds configured navigation max_speed.')
        if not translation_allowed:
            speed_limit = 0.
            reference = reference.copy()
            reference[:, :2] = pose[:2]
        rotation = _rotation(pose[2])
        local_reference = reference.copy()
        local_reference[:, :2] = (reference[:, :2] - pose[:2]) @ rotation
        local_reference[:, 2] = np.unwrap(_wrap(reference[:, 2] - pose[2]))
        local_obstacles = (obstacles - pose[:2]) @ rotation
        tree = cKDTree(local_obstacles) if len(obstacles) else None
        center_speed_bound = max(self.max_speed, float(np.linalg.norm(twist[:2]))) + (
            np.linalg.norm(self.centers, axis=1).max() * max(self.w_max, abs(twist[2])))
        hard_clearance = self.minimum_swept_clearance_m + center_speed_bound * self.dt_s / (2. * self.subsamples)
        required_distance = self.radii + self.cell_radius_m + hard_clearance
        # The already-issued command continues while the optimizer runs. This
        # prefix is not editable by the NLP and must itself clear the entire map.
        initial = np.r_[np.zeros(3), twist]
        prefix_steps = math.ceil(execution_delay_s / (self.dt_s / self.subsamples))
        prefix_times = np.linspace(0., execution_delay_s, prefix_steps + 1)
        prefix_states = np.array([self._advance_numpy(initial, previous_command, t) for t in prefix_times])
        initial = prefix_states[-1]
        prefix_minimum = None
        if tree is not None:
            distances = tree.query(self._centers_at(prefix_states[:, :3]).reshape(-1, 2))[0].reshape(-1, len(self.radii))
            prefix_minimum = float(np.min(distances - self.radii - self.cell_radius_m))
            if prefix_minimum < hard_clearance:
                self._last_controls = None
                raise MPCInfeasible(f'Current/delayed A2 footprint violates hard clearance: '
                                    f'{prefix_minimum:.3f} m < {hard_clearance:.3f} m.')
        self.opti.set_value(self.reference, local_reference)
        self.opti.set_value(self.previous, np.asarray(previous_command))
        self.opti.set_value(self.speed_limit, float(speed_limit))
        self.opti.set_value(self.initial, initial)
        self.opti.set_value(self.hard_clearance, hard_clearance)
        guess_controls, guess_states = self._initial_guess(local_reference, previous_command, speed_limit, initial, tree, required_distance)
        self.opti.set_value(self.have_obstacles, float(tree is not None))
        for constraint_round in range(3):
            if tree is None:
                neighbors = np.zeros(self.neighbors.shape)
            else:
                neighbors = self._nearest_constraints(guess_controls, initial, tree)
            self.opti.set_value(self.neighbors, neighbors)
            self.opti.set_initial(self.controls, guess_controls)
            self.opti.set_initial(self.states, guess_states)
            try:
                solution = self.opti.solve()
            except RuntimeError as exc:
                self._last_controls = None
                status = self.opti.stats().get('return_status', 'solver_error')
                raise MPCInfeasible(f'Depth/footprint MPC failed: {status}') from exc
            controls = np.asarray(solution.value(self.controls), dtype=float)
            predicted = np.asarray(solution.value(self.states), dtype=float)
            if tree is None:
                minimum_clearance = None
                break
            # Check every occupied point via the tree, including all substeps.
            distances = tree.query(self._centers_at(self._sample_poses(controls, initial)).reshape(-1, 2))[0].reshape(-1, len(self.radii))
            minimum_clearance = min(prefix_minimum, float(np.min(distances - self.radii - self.cell_radius_m)))
            if minimum_clearance >= hard_clearance - 1e-4:
                break
            guess_controls, guess_states = controls, predicted
        else:
            self._last_controls = None
            raise MPCInfeasible(f'Obstacle constraints did not converge: clearance {minimum_clearance:.3f} m.')
        self._last_controls = controls.copy()
        prediction_times = np.r_[prefix_times, execution_delay_s + np.arange(1, self.horizon + 1) * self.dt_s]
        predicted_world = np.vstack((prefix_states[:, :3], predicted[1:, :3]))
        predicted_world[:, :2] = predicted_world[:, :2] @ rotation.T + pose[:2]
        predicted_world[:, 2] = _wrap(predicted_world[:, 2] + pose[2])
        return MPCResult((float(controls[0, 0]), float(controls[0, 1])), predicted_world, prediction_times, execution_delay_s,
                         minimum_clearance, str(self.opti.stats()['return_status']),
                         (time.monotonic() - started) * 1000., len(obstacles))
