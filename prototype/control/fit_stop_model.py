"""Fit stopping response from captured estimated-state JSONL; never read truth."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from prototype.control.stop_model import stopping_pose

TIMES_S = np.array([.1, .2, .4, .8, 1.2, 2.])


def fit(cases_dir: Path, output: Path, report_path: Path):
    cases = []
    for metadata_path in sorted(cases_dir.glob('*/case.json')):
        metadata = json.loads(metadata_path.read_text())
        rows = [json.loads(line) for line in metadata_path.with_name('states.jsonl').read_text().splitlines()]
        stop_start = float(metadata['stop_start_stamp_s'])
        initial_index = min(range(len(rows)), key=lambda i: abs(rows[i]['stamp_s'] - stop_start))
        initial = rows[initial_index]
        if abs(initial['stamp_s'] - stop_start) > .02:
            raise ValueError(f'{metadata_path.parent.name}: missing state at zero-command onset')
        pose = np.asarray(initial['pose_odom'])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        selected = [row for row in rows[initial_index:] if row['stamp_s'] <= stop_start + TIMES_S[-1] + .02]
        times = np.array([row['stamp_s'] - stop_start for row in selected])
        observed = np.asarray([row['pose_odom'] for row in selected])
        observed[:, :2] = (observed[:, :2] - pose[:2]) @ rotation
        observed[:, 2] = np.unwrap(observed[:, 2] - pose[2])
        if times[-1] < TIMES_S[-1] - .02:
            raise ValueError(f'{metadata_path.parent.name}: incomplete 2s stopping capture')
        raw = [json.loads(line) for line in metadata_path.with_name('sensors.jsonl').read_text().splitlines()]
        raw = [row for row in raw if stop_start - 1e-8 <= row['stamp_s'] <= stop_start + TIMES_S[-1] + 1e-8]
        cases.append(dict(name=metadata_path.parent.name, times=times, observed=observed,
                          initial=np.r_[np.zeros(3), initial['planar_twist']],
                          raw_times=np.asarray([row['stamp_s'] - stop_start for row in raw]),
                          raw_linear_speed=np.linalg.norm([row['velocity_estimate_body_m_s'] for row in raw], axis=1),
                          raw_angular_speed=np.linalg.norm([row['gyro_rad_s'] for row in raw], axis=1)))
    if not cases:
        raise ValueError(f'No stopping cases at {cases_dir}')

    def predict(case, tau, times=None):
        return np.asarray([stopping_pose(case['initial'], t, tau)
                           for t in (case['times'] if times is None else times)])

    def residual(tau):
        # Convert yaw error to center displacement using the unchanged outer
        # footprint center radius. This balances metres and radians in fitting.
        return np.concatenate([((predict(case, tau) - case['observed'])
                                * np.array([1., 1., .3])).ravel() for case in cases])

    result = least_squares(residual, np.full(3, .15), bounds=(.01, 5.))
    if not result.success:
        raise RuntimeError(f'Stopping response fit failed: {result.message}')
    tau = result.x
    position_envelope = np.zeros(len(TIMES_S))
    yaw_envelope = np.zeros(len(TIMES_S))
    position_sampling_allowance = np.zeros(len(TIMES_S))
    yaw_sampling_allowance = np.zeros(len(TIMES_S))
    case_reports = []
    for case in cases:
        means = predict(case, tau, TIMES_S)
        sampling_intervals = []
        for i, end in enumerate(TIMES_S):
            start = 0. if i == 0 else TIMES_S[i - 1]
            mask = (case['times'] >= start - 1e-8) & (case['times'] <= end + 1e-8)
            if not np.any(mask):
                raise ValueError(f"{case['name']}: no sensor samples in stopping interval {start}..{end}")
            errors = case['observed'][mask] - means[i]
            position_envelope[i] = max(position_envelope[i], float(np.linalg.norm(errors[:, :2], axis=1).max()))
            yaw_envelope[i] = max(yaw_envelope[i], float(np.abs(errors[:, 2]).max()))
            raw_mask = (case['raw_times'] >= start - 1e-8) & (case['raw_times'] <= end + 1e-8)
            raw_times = case['raw_times'][raw_mask]
            if not len(raw_times):
                raise ValueError(f"{case['name']}: no raw sensors in stopping interval {start}..{end}")
            nearest_gap_s = float(np.min(np.abs(raw_times[:, None] - case['times'][None, mask]), axis=1).max())
            maximum_linear_speed = float(case['raw_linear_speed'][raw_mask].max())
            maximum_angular_speed = float(case['raw_angular_speed'][raw_mask].max())
            position_allowance = nearest_gap_s * maximum_linear_speed
            yaw_allowance = nearest_gap_s * maximum_angular_speed
            position_sampling_allowance[i] = max(position_sampling_allowance[i], position_allowance)
            yaw_sampling_allowance[i] = max(yaw_sampling_allowance[i], yaw_allowance)
            sampling_intervals.append(dict(begin_s=float(start), end_s=float(end),
                                           maximum_nearest_state_sample_gap_s=nearest_gap_s,
                                           maximum_raw_sensor_linear_3d_speed_m_s=maximum_linear_speed,
                                           maximum_raw_sensor_gyro_norm_rad_s=maximum_angular_speed,
                                           position_sampling_allowance_m=position_allowance,
                                           yaw_sampling_allowance_rad=yaw_allowance))
        models = {}
        for label, model_tau in [('calibrated', tau), ('previous_0.15s', np.full(3, .15))]:
            error = predict(case, model_tau) - case['observed']
            models[label] = dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1)))),
                                 position_max_error_m=float(np.linalg.norm(error[:, :2], axis=1).max()),
                                 yaw_rmse_rad=float(np.sqrt(np.mean(error[:, 2] ** 2))),
                                 yaw_max_error_rad=float(np.abs(error[:, 2]).max()))
        twist = case['initial'][3:]
        case_reports.append(dict(name=case['name'], initial_sensor_twist=twist.tolist(),
                                 measured_initial_twist_in_command_domain=bool(np.linalg.norm(twist[:2]) <= .3 and abs(twist[2]) <= .6),
                                 sample_count=len(case['times']), models=models,
                                 raw_sensor_sampling_intervals=sampling_intervals))
    config = dict(model_id='native_hospital_sensor_stop_20260916', times_s=TIMES_S.tolist(),
                  velocity_response_time_s=tau.tolist(),
                  position_residual_m=(position_envelope + position_sampling_allowance).tolist(),
                  yaw_residual_rad=(yaw_envelope + yaw_sampling_allowance).tolist(),
                  residual_components=dict(raw_position_residual_m=position_envelope.tolist(),
                                           raw_yaw_residual_rad=yaw_envelope.tolist(),
                                           position_sampling_allowance_m=position_sampling_allowance.tolist(),
                                           yaw_sampling_allowance_rad=yaw_sampling_allowance.tolist()),
                  domain=dict(max_speed_m_s=.3, max_yaw_rate_rad_s=.6),
                  basis=dict(source=str(cases_dir), case_count=len(cases), input='estimated RobotState only',
                             fitting_bounds_s=[.01, 5.],
                             residual='maximum observed position/yaw error to forecast endpoint over its preceding interval plus raw-sensor sampling allowance',
                             sampling_allowance='maximum nearest-state-sample time gap times interval maximum raw estimated 3D linear speed / gyro norm, independently maximized across cases; no truth inputs',
                             limitation='Empirical envelope of these cases only; no all-state or hardware guarantee.'))
    output.write_text(json.dumps(config, indent=2) + '\n')
    report_path.write_text(json.dumps(dict(config=config, cases=case_reports, optimizer=dict(
        cost=float(result.cost), evaluations=int(result.nfev), message=result.message)), indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('cases_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    fit(args.cases_dir, args.output, args.report)
