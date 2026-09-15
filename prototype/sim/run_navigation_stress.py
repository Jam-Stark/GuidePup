"""Prepare and execute paired random room starts with the existing recorders."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np

from record_task import ROOT, snapshot_sources


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def prepare(output, seed, pose_count, model):
    import mujoco
    from a2 import A2Simulation

    output.mkdir(parents=True, exist_ok=False)
    snapshot_sources(output)
    sim = A2Simulation('room_elevator', 'd435i_rgb_nominal', render=False)
    footprint = json.loads((ROOT / 'prototype/control/footprint.json').read_text())
    base_centers = np.asarray(footprint['centers_base_m'])
    radii = np.asarray(footprint['radii_m'])
    boxes = []
    for index in range(sim.model.ngeom):
        if sim.robot_geom[index] or sim.model.geom_type[index] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        if not (sim.model.geom_contype[index] or sim.model.geom_conaffinity[index]):
            continue
        center = sim.data.geom_xpos[index]
        size = sim.model.geom_size[index]
        if center[2] - size[2] >= .8 or center[2] + size[2] <= 0.:
            continue
        boxes.append({'name': sim.model.geom(index).name, 'center': center[:2].tolist(),
                      'halfsize': size[:2].tolist(),
                      'rotation': sim.data.geom_xmat[index].reshape(3, 3)[:2, :2].tolist()})

    rng = np.random.default_rng(seed)
    poses, rejected = [], []
    # One yaw sample per equal sector covers the entire circle. Positions are
    # sampled from the room interior; only static footprint overlap is excluded.
    for sector in rng.permutation(pose_count):
        yaw = rng.uniform(-180 + 360 * sector / pose_count, -180 + 360 * (sector + 1) / pose_count)
        angle = math.radians(yaw)
        rotation = np.array([[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]])
        for attempt in range(1000):
            xy = rng.uniform([1.65, -7.55], [4.35, -5.05])
            centers = base_centers @ rotation + xy
            clearances = []
            for box in boxes:
                local = (centers - box['center']) @ np.asarray(box['rotation'])
                q = np.abs(local) - box['halfsize']
                signed = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0) - radii
                clearances.append(float(np.min(signed)))
            clearance = min(clearances)
            pose = [*map(float, xy), float(yaw)]
            if clearance >= footprint['desired_clearance_m']:
                poses.append({'id': f'p{len(poses) + 1:02d}', 'pose_xy_yaw_deg': pose,
                              'yaw_sector': int(sector), 'initial_footprint_clearance_m': clearance})
                break
            rejected.append({'pose_xy_yaw_deg': pose, 'initial_footprint_clearance_m': clearance})
        else:
            raise RuntimeError(f'No nonoverlapping initial pose sampled for yaw sector {sector}')

    trials = []
    for pose in poses:
        for arm in rng.permutation(['primary', 'force_resume']):
            trials.append({'id': f"{pose['id']}_{arm}", 'pose_id': pose['id'], 'arm': str(arm),
                           'pose_xy_yaw_deg': pose['pose_xy_yaw_deg']})
    manifest = {
        'version': 'navigation-stress-20260915', 'created_at': time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'seed': seed, 'goal': '出门走到电梯门口', 'scene': 'room_elevator',
        'model_profile': model,
        'sampling': {'xy_bounds_m': [[1.65, 4.35], [-7.55, -5.05]], 'yaw_bounds_deg': [-180, 180],
                     'yaw_strata': pose_count, 'minimum_initial_footprint_clearance_m': footprint['desired_clearance_m'],
                     'definition': 'Random room interior positions with the fixed three-circle footprint clear of collision-enabled scene boxes below 0.8m; all yaw sectors retained.'},
        'timeout_s': 360, 'max_recoveries': 3, 'decision_timeout_s': 30,
        'renderer_environment': {key: os.environ[key] for key in (
            'MUJOCO_GL', 'MUJOCO_EGL_DEVICE_ID', '__EGL_VENDOR_LIBRARY_FILENAMES',
            'LIBGL_ALWAYS_SOFTWARE', 'GALLIUM_DRIVER', 'LP_NUM_THREADS') if key in os.environ},
        'force_intervention': 'Existing recorder: local -80N for 2sim seconds after stage0 forward command>=0.2m/s and net movement>=0.5m; withdraw, ask, explicit human resume.',
        'pairing_limit': 'Same requested initial pose; independent full runs, Qwen planning and actual trajectories are not restored identical prefixes. Report realized instruction and intervention exposure.',
        'admission': 'All predeclared admitted poses run in both arms; early model/navigation failures remain outcomes. Force not reached is unexposed, not a successful pause test.',
        'stop_condition': 'Finish all trials; missing recorder summary or launch failure aborts the batch without retries. Completed ERROR/NEEDS_USER/LIMIT_REACHED episodes remain results.',
        'source_snapshot': str(output / 'source_snapshot'), 'poses': poses, 'trials': trials,
        'rejected_initial_overlap_candidates': rejected, 'static_admission_boxes': boxes,
    }
    save(output / 'manifest.json', manifest)
    print(json.dumps({'manifest': str(output / 'manifest.json'), 'poses': len(poses), 'trials': len(trials),
                      'rejected_initial_overlap_candidates': len(rejected)}, ensure_ascii=False), flush=True)


def run(output):
    manifest = json.loads((output / 'manifest.json').read_text())
    environment = os.environ.copy()
    for key in ('http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY'):
        environment.pop(key, None)
    environment.update(NO_PROXY='*', no_proxy='*')
    environment.update(manifest['renderer_environment'])
    python = ROOT / '.local/envs/sim/bin/python'
    started = time.time()
    completed = []
    for index, trial in enumerate(manifest['trials']):
        episode = output / trial['id']
        script = 'record_task.py' if trial['arm'] == 'primary' else 'record_monitor.py'
        command = [str(python), str(ROOT / 'prototype/sim' / script), '--output', str(episode),
                   '--model', manifest['model_profile'],
                   '--record-navigation-inputs',
                   '--spawn-pose', *map(str, trial['pose_xy_yaw_deg']),
                   '--timeout-s', str(manifest['timeout_s']), '--max-recoveries', str(manifest['max_recoveries']),
                   '--decision-timeout-s', str(manifest['decision_timeout_s'])]
        if trial['arm'] == 'primary':
            command += ['--scene', manifest['scene'], '--goal', manifest['goal']]
        save(output / 'batch_state.json', {'state': 'RUNNING', 'started_epoch': started, 'current': trial,
                                          'completed': completed, 'remaining_including_current': len(manifest['trials']) - index})
        print(json.dumps({'event': 'trial_started', **trial}, ensure_ascii=False), flush=True)
        with (output / f"{trial['id']}.log").open('w') as log:
            process = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
        summary_path = episode / 'summary.json'
        if not summary_path.exists():
            save(output / 'batch_state.json', {'state': 'FAILED', 'trial': trial, 'recorder_return_code': process.returncode,
                                              'completed': completed, 'reason': 'Recorder did not produce summary.json; see trial log.'})
            raise RuntimeError(f"{trial['id']} recorder failed without summary; no retries")
        summary = json.loads(summary_path.read_text())
        for evaluator in ('evaluate_monitor.py', 'evaluate_navigation_v2.py'):
            subprocess.run([str(python), str(ROOT / 'prototype/sim' / evaluator), '--episode', str(episode)],
                           cwd=ROOT, env=environment, check=True, stdout=subprocess.DEVNULL)
        item = {**trial, 'episode': str(episode), 'recorder_return_code': process.returncode,
                'outcome_code': summary['exit_code'], 'elapsed_wall_s': summary['elapsed_wall_s'],
                'recorder_error': summary.get('recorder_error')}
        completed.append(item)
        print(json.dumps({'event': 'trial_completed', 'index': index + 1, 'total': len(manifest['trials']), **item},
                         ensure_ascii=False), flush=True)
    save(output / 'batch_state.json', {'state': 'COMPLETED', 'started_epoch': started,
                                      'elapsed_wall_s': time.time() - started, 'completed': completed})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['prepare', 'run'])
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seed', type=int, default=20260915)
    parser.add_argument('--pose-count', type=int, default=12)
    parser.add_argument('--model', choices=['qwen', 'gemini'], default='qwen')
    args = parser.parse_args()
    if args.command == 'prepare':
        prepare(args.output.resolve(), args.seed, args.pose_count, args.model)
    else:
        run(args.output.resolve())
