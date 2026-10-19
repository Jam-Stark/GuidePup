"""Offline physical and navigation evidence for a recorded Isaac multi-room task."""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np
from matplotlib.path import Path as PolygonPath

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'prototype/sim'))
from evaluate_navigation_v2 import decision_evidence, image_and_qwen_evidence


def read(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def circles(pose, footprint):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    centers = np.asarray(footprint['centers_base_m']) @ np.array([[c, s], [-s, c]]) + [x, y]
    return centers, np.asarray(footprint['radii_m'])


def footprint_room_clearance(centers, radii, room):
    """Signed disc-to-room-boundary clearance, including concave structural corners."""
    if 'interior_polygon_xy_m' in room:
        polygon = np.asarray(room['interior_polygon_xy_m'], dtype=float)
        edges = np.roll(polygon, -1, axis=0) - polygon
        offsets = centers[:, None, :] - polygon[None, :, :]
        along = np.sum(offsets * edges, axis=2) / np.sum(edges * edges, axis=1)
        closest = polygon[None, :, :] + np.clip(along, 0., 1.)[:, :, None] * edges
        distances = np.sqrt(np.min(np.sum((centers[:, None, :] - closest) ** 2, axis=2), axis=1))
        inside = PolygonPath(np.vstack([polygon, polygon[0]]), closed=True).contains_points(centers)
        return float(np.min(np.where(inside, distances, -distances) - radii))
    xmin, xmax, ymin, ymax = room['interior_xy_bounds_m']
    return float(min(np.min(centers[:, 0] - radii) - xmin,
        xmax - np.max(centers[:, 0] + radii), np.min(centers[:, 1] - radii) - ymin,
        ymax - np.max(centers[:, 1] + radii)))


def portal_state(pose, footprint, portal):
    centers, radii = circles(pose, footprint)
    near, far = portal['x_slab_m']
    low, high = portal['structural_clear_y_interval_at_z_0_5_m']
    # Exact horizontal circle section over the jamb thickness, not full circle
    # width when its center has not reached the plane yet.
    dx = np.maximum(np.maximum(near - centers[:, 0], centers[:, 0] - far), 0.)
    intersects = dx <= radii
    lateral = None
    if np.any(intersects):
        half = np.sqrt(radii[intersects] ** 2 - dx[intersects] ** 2)
        ys = centers[intersects, 1]
        lateral = float(min(np.min(ys - half) - low, high - np.max(ys + half)))
    return {'trailing_clearance_m': float(np.min(centers[:, 0] - radii) - far),
            'leading_clearance_m': float(np.max(centers[:, 0] + radii) - near),
            'aperture_clearance_m': lateral}


def crossing(trajectory, footprint, portal, after):
    active = False
    eligible = False
    minimum = None
    for row in trajectory:
        if row['sim_time_s'] < after:
            continue
        state = portal_state(row['pose_world'], footprint, portal)
        if state['leading_clearance_m'] <= 0:
            eligible, active, minimum = True, False, None
        if eligible and state['aperture_clearance_m'] is not None:
            active = True
            value = state['aperture_clearance_m']
            minimum = value if minimum is None else min(minimum, value)
        if active and state['trailing_clearance_m'] >= 0:
            return {'id': portal['id'], 'crossed_in_order': minimum >= 0,
                    'first_full_crossing_s': row['sim_time_s'],
                    'minimum_aperture_clearance_m': minimum, 'pose_world': row['pose_world'],
                    'final': portal_state(trajectory[-1]['pose_world'], footprint, portal)}
    return {'id': portal['id'], 'crossed_in_order': False, 'first_full_crossing_s': None,
            'minimum_aperture_clearance_m': minimum,
            'final': portal_state(trajectory[-1]['pose_world'], footprint, portal)}


def target_waiting_conditions(pose, footprint, target, waiting):
    """Offline footprint-to-plane score using the existing waiting convention."""
    centers, radii = circles(pose, footprint)
    point = np.asarray(target['point_world_m'][:2], dtype=float)
    normal = np.asarray(target['approach_normal_xy'], dtype=float)
    clearance = float(np.min((centers-point) @ normal-radii))
    tangent = np.array([-normal[1], normal[0]])
    lateral = abs(float((np.asarray(pose[:2])-point) @ tangent))
    desired_yaw = math.atan2(-normal[1], -normal[0])
    heading = abs(math.degrees(math.atan2(math.sin(pose[2]-desired_yaw), math.cos(pose[2]-desired_yaw))))
    lo, hi = waiting['front_clearance_m']
    return {'front_clearance_m': clearance, 'lateral_offset_m': lateral,
        'heading_error_deg': heading, 'front_range_m': [lo, hi],
        'lateral_tolerance_m': waiting['lateral_tolerance_m'],
        'heading_tolerance_deg': waiting['heading_tolerance_deg'],
        'satisfied': lo <= clearance <= hi and lateral <= waiting['lateral_tolerance_m']
            and heading <= waiting['heading_tolerance_deg']}


def evaluate(run):
    episode = run / 'episode'
    summary = read(episode / 'summary.json')
    scenario = read(run / 'scenario.json')
    geometry = scenario['offline_evaluation']
    raw = Path(summary['final']['output'])
    footprint = read(raw / 'configuration.json')['controller_footprint']
    contract = read(episode / 'source_snapshot/navigation-contract.json')
    start, finish = summary['before']['sim_time_s'], summary['final']['sim_time_s']
    all_truth = rows(raw / 'ground_truth.jsonl')
    truth_at = {round(r['sim_time_s'], 6): r for r in all_truth}
    trajectory = [r for r in all_truth if start - 1e-6 <= r['sim_time_s'] <= finish + 1e-6]
    telemetry = [r for r in rows(raw / 'telemetry.jsonl') if start - 1e-6 <= r['sim_time_s'] <= finish + 1e-6]
    events = rows(episode / 'agent.jsonl')
    monitor = read(episode / 'monitor_state.json')
    lightnav = rows(raw / 'lightnav.jsonl')
    controller = rows(raw / 'local_controller.jsonl')
    accepted = [r for r in lightnav if not r['discarded']]
    applied = [r for r in controller if r['command_applied']]
    crossings, after = [], start
    approaching = 'target_plane' in geometry
    for portal in ([] if approaching else geometry['portals']):
        result = crossing(trajectory, footprint, portal, after)
        crossings.append(result)
        after = result['first_full_crossing_s'] if result['crossed_in_order'] else math.inf

    final = trajectory[-1]
    centers, radii = circles(final['pose_world'], footprint)
    waiting = (target_waiting_conditions(final['pose_world'], footprint, geometry['target_plane'], contract['waiting_zone'])
               if approaching else None)
    room_clearance = None if approaching else footprint_room_clearance(centers, radii, geometry['destination_room'])
    stop_duration = contract['stopped_duration_s']
    stop_window = [r for r in trajectory if r['sim_time_s'] >= finish - stop_duration - 1e-6]
    max_speed = max(math.hypot(*r['actual_planar_twist'][:2]) for r in stop_window)
    max_yaw = max(abs(r['actual_planar_twist'][2]) for r in stop_window)
    stopped = (stop_window[-1]['sim_time_s'] - stop_window[0]['sim_time_s'] >= stop_duration - .006
        and max_speed < .05 and max_yaw < .05 and np.allclose(summary['final']['effective_cmd'], 0))
    path = sum(math.dist(a['pose_world'][:2], b['pose_world'][:2]) for a, b in zip(trajectory, trajectory[1:]))
    declarations = [event for event in events if event['kind'] == 'qwen_completion_declared']
    max_nonfoot, max_foot, max_foot_up, nonfoot_ticks, contact_count = 0., 0., 0., 0, 0
    peak_nonfoot = None
    for row in rows(raw / 'contacts.jsonl'):
        if not start - 1e-6 <= row['sim_time_s'] <= finish + 1e-6:
            continue
        contact_count += 1
        tick_nonfoot, upward = 0., 0.
        for body in row['bodies']:
            force = body['net_normal_force_world_n']
            magnitude = math.sqrt(sum(v*v for v in force))
            if body['is_foot']:
                max_foot = max(max_foot, magnitude)
                upward += max(0., force[2])
            else:
                tick_nonfoot = max(tick_nonfoot, magnitude)
                if magnitude > max_nonfoot:
                    max_nonfoot = magnitude
                    peak_nonfoot = {'sim_time_s': row['sim_time_s'], **body}
        max_foot_up = max(max_foot_up, upward)
        nonfoot_ticks += tick_nonfoot > 1.

    unique_states = {round(r['robot_state']['stamp_s'], 6): r['robot_state'] for r in telemetry}
    paired = [(s, truth_at[t]) for t, s in sorted(unique_states.items())]
    odom_initial, truth_initial = paired[0]
    align = np.asarray(truth_initial['world_from_base']) @ np.linalg.inv(np.asarray(odom_initial['odom_from_base']))
    errors, yaw_errors = [], []
    for state, truth in paired:
        estimate = align @ np.asarray(state['odom_from_base'])
        errors.append(float(np.linalg.norm(estimate[:2, 3] - np.asarray(truth['pose_world'][:2]))))
        delta = math.atan2(estimate[1, 0], estimate[0, 0]) - truth['pose_world'][2]
        yaw_errors.append(math.atan2(math.sin(delta), math.cos(delta)))

    physical = {'definition': 'Independent native PhysX ground truth; never navigation observations.',
        'window_sim_s': [start, finish], 'physics_samples': len(trajectory), 'actual_path_m': path,
        'initial_pose_world': trajectory[0]['pose_world'], 'final_pose_world': final['pose_world'],
        'ordered_portals': crossings,
        'final_destination_room_clearance_m': room_clearance,
        'stopped': stopped, 'stop_window_duration_s': stop_duration,
        'stop_window_max_speed_m_s': max_speed, 'stop_window_max_yaw_rate_rad_s': max_yaw,
        'physical_goal_reached': (waiting['satisfied'] if approaching else all(r['crossed_in_order'] for r in crossings) and room_clearance >= 0) and stopped,
        'target_waiting_conditions': waiting,
        'geometry_limits': geometry['geometry_limits'], 'footprint': footprint,
        'contacts': {'observer': 'physx_tensor_contact_view', 'samples': contact_count,
            'max_nonfoot_net_normal_force_n': max_nonfoot, 'peak_nonfoot': peak_nonfoot,
            'nonfoot_samples_above_1n': nonfoot_ticks, 'max_foot_net_normal_force_n': max_foot,
            'max_total_upward_foot_normal_force_n': max_foot_up,
            'limitations': 'Body net normal forces only; no counterpart identity, points or friction. Includes self-contact and possible cancellation. Not proof of collision-free travel.'},
        'odometry': {'alignment': 'Single first-paired-frame SE3 alignment, scoring only', 'samples': len(paired),
            'position_rmse_m': float(np.sqrt(np.mean(np.square(errors)))), 'position_max_m': max(errors),
            'position_final_m': errors[-1], 'yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(np.square(yaw_errors)))))} }
    decision = decision_evidence(events)
    actions = [e for e in events if e['kind'] == 'action_executed']
    presentation = image_and_qwen_evidence(events, actions)
    navigation = {'monitor_outcome': summary['monitor_outcome'], 'task': monitor['task'],
        'recorder_error': summary['recorder_error'], 'event_counts': dict(Counter(e['kind'] for e in events)),
        'qwen_requests': sum(e['kind'] == 'decision_request' for e in events),
        'decision_context': decision, 'images_and_qwen': presentation,
        'lightnav': {'responses': len(lightnav), 'accepted': len(accepted),
            'stops': [{'sequence': r['sequence'], 'capture_time_s': r['capture_time_s'],
                'received_stamp_s': r['received_stamp_s'], 'instruction': r['instruction']} for r in accepted if r['result']['stop']],
            'visible_true': sum(r['result']['visible'] for r in accepted),
            'input_intervals_s': [r['input_interval_s'] for r in accepted if r['input_interval_s'] is not None]},
        'local_controller': {'backend': 'nav2_mppi', 'solves': len(controller), 'applied': len(applied),
            'applied_reference_sources': dict(Counter(r['reference_source'] for r in applied)),
            'solver_statuses': dict(Counter(r['solver_status'] for r in controller))},
        'intervention': {'protocol': summary['interaction_protocol'], 'force_injection_available': False,
            'human_paused_samples': sum(r['state'] == 'HUMAN_PAUSED' for r in telemetry),
            'force_brake_observed': summary['force_brake_observed']},
        'runtime_error_events': [e for e in events if 'error' in e['kind'] or e['kind'] == 'decision_failed'],
        'control_error_reasons': list(dict.fromkeys(r['reason'] for r in telemetry if r['state'] == 'ERROR')),
        'contract': contract}
    model_declared_completion = bool(declarations)
    false_completion = model_declared_completion and not physical['physical_goal_reached']
    navigation['completion_declarations'] = declarations
    results = {'scene': scenario['name'], 'goal': scenario['goal'], 'episode': str(episode),
        'raw_output': str(raw), 'model': summary['model_profile'], 'elapsed_wall_s': summary['elapsed_wall_s'],
        'runtime_outcome': summary['monitor_outcome'],
        'model_declared_completion': model_declared_completion, 'false_completion': false_completion,
        'physical_goal_reached': physical['physical_goal_reached'],
        'verified_success': bool(physical['physical_goal_reached'] and summary['monitor_outcome'] == 'COMPLETED'
                                 and model_declared_completion and not summary['recorder_error']
                                 and not navigation['control_error_reasons'] and nonfoot_ticks == 0),
        'needs_user_reason': monitor['task'].get('pauseReason'),
        'recorder_error': summary['recorder_error'], 'runtime_error_events': navigation['runtime_error_events'],
        'control_error_reasons': navigation['control_error_reasons'],
        'contact_validation_limit': physical['contacts']['limitations'],
        'evaluations': ['physical_evaluation.json', 'navigation_evidence.json']}
    save(run / 'physical_evaluation.json', physical)
    save(run / 'navigation_evidence.json', navigation)
    save(run / 'results.json', results)
    print(json.dumps(results, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    evaluate(parser.parse_args().run.resolve())
