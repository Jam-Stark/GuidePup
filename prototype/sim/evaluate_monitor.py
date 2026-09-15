"""Offline physical evidence for a completed room_elevator monitor episode.

Reads record_task.py's summary.json, server logs and optional agent.jsonl.
No MuJoCo, service, model or simulator execution is performed.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PHYSICS_DT = .005
CONTROL_DT = .02
TIME_EPS = 1e-8


def read_json(path):
    return json.loads(path.read_text())


def window_rows(path, start, finish, *, physics_steps=False):
    """Contact timestamps start their 5 ms interval; telemetry is a state sample."""
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            t = float(row['sim_time_s'])
            if t < start - TIME_EPS:
                continue
            if (physics_steps and t >= finish - TIME_EPS) or t > finish + TIME_EPS:
                break
            yield line_number, row


def scene_geometry(path):
    scene = ET.parse(path).getroot()

    def box(body_name, geom_name):
        body = scene.find(f"./worldbody/body[@name='{body_name}']")
        geom = body.find(f"./geom[@name='{geom_name}']")
        center = np.fromstring(body.get('pos', '0 0 0'), sep=' ') + np.fromstring(geom.attrib['pos'], sep=' ')
        halfsize = np.fromstring(geom.attrib['size'], sep=' ')
        return {'geom': geom_name, 'center_m': center.tolist(), 'halfsize_m': halfsize.tolist()}

    west = box('room_exit_surround', 'room_exit_west_jamb')
    east = box('room_exit_surround', 'room_exit_east_jamb')
    leaves = [box('elevator_doors', name) for name in ('elevator_left_leaf', 'elevator_right_leaf')]
    faces = [{'geom': leaf['geom'],
              'front_plane_x_m': leaf['center_m'][0] - leaf['halfsize_m'][0],
              'y_interval_m': [leaf['center_m'][1] - leaf['halfsize_m'][1],
                               leaf['center_m'][1] + leaf['halfsize_m'][1]],
              'z_interval_m': [leaf['center_m'][2] - leaf['halfsize_m'][2],
                               leaf['center_m'][2] + leaf['halfsize_m'][2]]}
             for leaf in leaves]
    return {
        'source_xml': str(path),
        'doorway': {'plane_y_m': west['center_m'][1], 'forward_direction': '+world_y',
                    'aperture_x_m': [west['center_m'][0] + west['halfsize_m'][0],
                                     east['center_m'][0] - east['halfsize_m'][0]],
                    'jamb_y_interval_m': [west['center_m'][1] - west['halfsize_m'][1],
                                          west['center_m'][1] + west['halfsize_m'][1]],
                    'jambs': [west, east]},
        'elevator_faces': faces,
        'definition': 'Native room_elevator MJCF axis-aligned boxes, including body offsets; offline only.',
    }


def footprint_at(pose, footprint, doorway):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    centers = np.asarray(footprint['centers_base_m']) @ np.array([[c, s], [-s, c]])
    centers += [x, y]
    radii = np.asarray(footprint['radii_m'])
    plane = doorway['plane_y_m']
    lower, upper = doorway['aperture_x_m']
    trailing = float(np.min(centers[:, 1] - radii) - plane)
    leading = float(np.max(centers[:, 1] + radii) - plane)
    intersects = np.abs(centers[:, 1] - plane) <= radii
    section = None
    if np.any(intersects):
        selected = centers[intersects]
        halfwidth = np.sqrt(np.maximum(0., radii[intersects] ** 2 - (selected[:, 1] - plane) ** 2))
        section_left = float(np.min(selected[:, 0] - halfwidth))
        section_right = float(np.max(selected[:, 0] + halfwidth))
        section = {'x_interval_m': [section_left, section_right],
                   'aperture_clearance_m': min(section_left - lower, upper - section_right)}
    jamb_clearances = []
    for jamb in doorway['jambs']:
        q = np.abs(centers - np.asarray(jamb['center_m'][:2])) - jamb['halfsize_m'][:2]
        distances = np.linalg.norm(np.maximum(q, 0.), axis=1) + np.minimum(np.max(q, axis=1), 0.)
        jamb_clearances.extend((distances - radii).tolist())
    return {'centers_world_xy_m': centers.tolist(),
            'trailing_edge_beyond_plane_m': trailing,
            'leading_edge_beyond_plane_m': leading,
            'full_body_beyond_plane': trailing >= 0.,
            'full_body_on_room_side': leading <= 0.,
            'plane_section': section,
            'minimum_signed_jamb_clearance_m': min(jamb_clearances)}


def crossing_evidence(rows, footprint, doorway):
    states = [footprint_at(row['pose_world'], footprint, doorway) for row in rows]
    first_index = next((i for i in range(1, len(rows))
                        if states[i - 1]['trailing_edge_beyond_plane_m'] < 0.
                        and states[i]['full_body_beyond_plane']), None)
    first = None
    if first_index is not None:
        first = {'first_observed_sim_time_s': rows[first_index]['sim_time_s'],
                 'crossing_time_observation_bracket_s': [rows[first_index - 1]['sim_time_s'],
                                                        rows[first_index]['sim_time_s']],
                 'previous_pose_world': rows[first_index - 1]['pose_world'],
                 'first_observed_pose_world': rows[first_index]['pose_world'],
                 'previous_trailing_margin_m': states[first_index - 1]['trailing_edge_beyond_plane_m'],
                 'observed_trailing_margin_m': states[first_index]['trailing_edge_beyond_plane_m']}
    groups = []
    for i, state in enumerate(states):
        if state['plane_section'] is None:
            continue
        if not groups or i != groups[-1][-1] + 1:
            groups.append([i])
        else:
            groups[-1].append(i)
    windows = []
    for group in groups:
        minimum = min(group, key=lambda i: states[i]['plane_section']['aperture_clearance_m'])
        windows.append({'first_observed_sim_time_s': rows[group[0]]['sim_time_s'],
                        'last_observed_sim_time_s': rows[group[-1]]['sim_time_s'],
                        'observed_sample_count': len(group),
                        'minimum_plane_aperture_clearance_m': states[minimum]['plane_section']['aperture_clearance_m'],
                        'minimum_clearance_sim_time_s': rows[minimum]['sim_time_s'],
                        'minimum_clearance_pose_world': rows[minimum]['pose_world'],
                        'minimum_signed_jamb_clearance_m': min(states[i]['minimum_signed_jamb_clearance_m'] for i in group)})
    return {'definition': 'All three rotated disks must satisfy center_y - radius >= doorway plane_y. This is the first observed full-footprint plane crossing, not a continuous-time interpolation.',
            'clearance_definition': 'At each observed plane intersection, use each disk chord sqrt(r^2 - dy^2); report the closest chord edge to the two aperture x limits. No extra MPC clearance is added.',
            'initial': states[0], 'final': states[-1],
            'first_full_body_crossing': first,
            'initially_full_body_beyond_plane': states[0]['full_body_beyond_plane'],
            'plane_intersection_windows': windows,
            'minimum_crossing_aperture_clearance_m': min((w['minimum_plane_aperture_clearance_m'] for w in windows), default=None),
            'limits': ['Crossing and clearance are sampled observations; motion between telemetry samples is not reconstructed.',
                       'The footprint covers the documented walking poses, not every possible joint configuration.',
                       'Passing the center plane is distinct from clearing the entire 0.30 m jamb depth. Contacts are evaluated separately.']}


class ContactStats:
    def __init__(self):
        self.count = 0
        self.ticks = set()
        self.peak = None

    def add(self, tick, observation):
        self.count += 1
        self.ticks.add(tick)
        if self.peak is None or observation['normal_force_N'] > self.peak['normal_force_N']:
            self.peak = observation

    def result(self):
        groups = []
        for tick in sorted(self.ticks):
            if not groups or tick != groups[-1][-1] + 1:
                groups.append([tick])
            else:
                groups[-1].append(tick)
        return {'contact_point_records': self.count, 'distinct_physics_steps': len(self.ticks),
                'union_duration_s': len(self.ticks) * PHYSICS_DT,
                'windows': [{'first_sample_sim_time_s': g[0] * PHYSICS_DT,
                             'last_sample_sim_time_s': g[-1] * PHYSICS_DT,
                             'union_duration_s': len(g) * PHYSICS_DT} for g in groups],
                'maximum_single_point_normal_force_N': self.peak['normal_force_N'] if self.peak else None,
                'peak': self.peak}


def contact_evidence(source, start, finish):
    geometry = {geom['id']: geom for geom in read_json(source / 'contact_geometry.json')}
    feet = {key for key, geom in geometry.items() if geom['robot'] and geom['type'] == 2
            and geom['body'] in ('FL_calf', 'FR_calf', 'RL_calf', 'RR_calf')}
    floors = {key for key, geom in geometry.items() if not geom['robot'] and geom['type'] == 0 and geom['name'] == 'floor'}
    all_non_support, positive, non_ground, abnormal_floor = (ContactStats() for _ in range(4))
    pairs = {}
    excluded = excluded_positive = row_count = 0
    for line_number, row in window_rows(source / 'contacts.jsonl', start, finish, physics_steps=True):
        row_count += 1
        tick = round(row['sim_time_s'] / PHYSICS_DT)
        for contact in row['contacts']:
            a, b = (geometry[key] for key in contact['geoms'])
            robot, environment = (a, b) if a['robot'] else (b, a)
            fn = float(contact['force_torque_contact_frame'][0])
            if robot['id'] in feet and environment['id'] in floors:
                excluded += 1
                excluded_positive += fn > 0.
                continue
            observation = {'sim_time_s': row['sim_time_s'], 'source_line': line_number,
                           'robot_geom': robot, 'environment_geom': environment,
                           'normal_force_N': fn, 'position_world_m': contact['position_world_m'],
                           'base_position_world_m': row['qpos'][:3]}
            all_non_support.add(tick, observation)
            if fn <= 0.:
                continue
            positive.add(tick, observation)
            (abnormal_floor if environment['id'] in floors else non_ground).add(tick, observation)
            key = (robot['id'], environment['id'])
            if key not in pairs:
                pairs[key] = ContactStats()
            pairs[key].add(tick, observation)
    return {'source': str(source / 'contacts.jsonl'), 'physics_step_s': PHYSICS_DT,
            'window_convention': 'Contact steps start at sim_time_s; include [before, final), excluding a next step after the recorded final state.',
            'recorded_nonempty_contact_rows_in_window': row_count,
            'support_exclusion': {'rule': 'Only floor plane versus A2 calf-attached foot sphere; trunk/leg contact with floor is retained.',
                                  'floor_geom_ids': sorted(floors), 'foot_geom_ids': sorted(feet),
                                  'excluded_contact_point_records': excluded,
                                  'excluded_positive_contact_point_records': excluded_positive},
            'all_non_support': all_non_support.result(),
            'positive_non_support': positive.result(), 'positive_non_ground': non_ground.result(),
            'positive_abnormal_floor': abnormal_floor.result(),
            'positive_geom_pairs': [{'robot_geom': geometry[a], 'environment_geom': geometry[b], **stats.result()}
                                    for (a, b), stats in sorted(pairs.items())],
            'definition': 'Positive means contact-frame normal force > 0, irrespective of contact.distance. Duration is the union of 5 ms steps, not the sum over contact points.'}


def door_face_distance(pose, faces):
    candidates = []
    for face in faces:
        point = [face['front_plane_x_m'], float(np.clip(pose[1], *face['y_interval_m']))]
        candidates.append({'geom': face['geom'], 'nearest_face_point_world_xy_m': point,
                           'base_horizontal_distance_m': math.hypot(point[0] - pose[0], point[1] - pose[1])})
    return min(candidates, key=lambda item: item['base_horizontal_distance_m'])


def stage_evidence(path, start, finish, footprint, doorway, crossing):
    if not path.exists():
        return {'available': False, 'source': str(path),
                'limitation': 'No monitor journal: stage planning, omission and completion times are unknown.'}
    plans, advances, completions, chronology = [], [], [], []
    first = crossing['first_full_body_crossing']
    if first is not None:
        chronology.append({'kind': 'full_body_plane_crossing_observed',
                           'sim_time_s': first['first_observed_sim_time_s'],
                           'source': 'telemetry', 'pose_world': first['first_observed_pose_world']})
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            event = json.loads(line)
            kind = event['kind']
            if kind not in ('task_planned', 'stage_completed', 'task_completed'):
                continue
            data = event['data']
            control = data['control']
            t = float(control['sim_time_s'])
            if not start - TIME_EPS <= t <= finish + TIME_EPS:
                continue
            geometry = footprint_at(control['pose_world'], footprint, doorway)
            if first is None:
                relative = 'no_full_body_crossing_observed_in_window'
            elif t < first['crossing_time_observation_bracket_s'][0]:
                relative = 'before_first_crossing_observation_bracket'
            elif t < first['first_observed_sim_time_s']:
                relative = 'inside_first_crossing_observation_bracket'
            else:
                relative = 'at_or_after_first_observed_crossing'
            entry = {'source_line': line_number, 'seq': event['seq'], 'at': event['at'],
                     'task_id': event['task_id'], 'revision': event['revision'], 'kind': kind,
                     'sim_time_s': t, 'pose_world': control['pose_world'],
                     'full_body_beyond_doorway_plane': geometry['full_body_beyond_plane'],
                     'trailing_edge_beyond_plane_m': geometry['trailing_edge_beyond_plane_m'],
                     'relative_to_first_observed_crossing': relative}
            if kind == 'task_planned':
                stages = data['stages']
                exits = [i for i, stage in enumerate(stages) if stage['kind'] == 'pass_doorway']
                approaches = [i for i, stage in enumerate(stages) if stage['kind'] == 'approach_target']
                entry.update(original_goal=data['originalGoal'], stages=stages,
                             pass_doorway_stage_indices=exits, pass_doorway_omitted=not exits,
                             pass_doorway_precedes_each_approach=all(any(i < j for i in exits) for j in approaches) if approaches else None)
                plans.append(entry)
            elif kind == 'stage_completed':
                entry.update(stage_index=data['stageIndex'], stage=data['stage'], model_reason=data['modelReason'],
                             pass_doorway_advanced_before_full_body_exit=data['stage']['kind'] == 'pass_doorway' and not geometry['full_body_beyond_plane'])
                advances.append(entry)
            else:
                entry['prior_pass_doorway_completion_recorded'] = any(
                    advance['task_id'] == entry['task_id'] and advance['stage']['kind'] == 'pass_doorway'
                    and advance['seq'] < entry['seq'] for advance in advances)
                completions.append(entry)
            chronology.append(entry)
    return {'available': True, 'source': str(path), 'plans': plans, 'stage_advances': advances,
            'task_completion_events': completions,
            'chronology': sorted(chronology, key=lambda item: item['sim_time_s']),
            'any_plan_omitted_pass_doorway': any(plan['pass_doorway_omitted'] for plan in plans) if plans else None,
            'any_recorded_pass_doorway_advance_before_full_body_exit': any(
                advance['pass_doorway_advanced_before_full_body_exit'] for advance in advances),
            'limits': ['No task_planned event means plan omission is unknown, not false.',
                       'Stage geometry uses each event control.pose_world at control.sim_time_s; a completion label is not a physical PASS.',
                       'Task completion without a prior pass_doorway completion event is reported separately from omission in a recorded plan.']}


def velocity_mismatch(rows):
    moving = [row for row in rows if row['state'] == 'NAVIGATING' and any(row['effective_cmd'])]
    if not moving:
        return {'samples': 0}
    actual = np.asarray([row['actual_planar_twist'] for row in moving])
    commanded = np.asarray([row['effective_cmd'] for row in moving])
    difference = actual - commanded
    return {'samples': len(moving), 'mean_actual_vx_vy_wz': actual.mean(axis=0).tolist(),
            'mean_effective_vx_vy_wz': commanded.mean(axis=0).tolist(),
            'mean_actual_minus_effective': difference.mean(axis=0).tolist(),
            'mae_actual_minus_effective': np.abs(difference).mean(axis=0).tolist()}


def evaluate(episode):
    summary = read_json(episode / 'summary.json')
    if summary['scene'] != 'room_elevator':
        raise ValueError('evaluate_monitor.py evaluates the native room_elevator scene only.')
    before, final = summary['before'], summary['final']
    source = Path(final['output']).resolve()
    if Path(before['output']).resolve() != source:
        raise ValueError('Episode before/final refer to different simulator runs.')
    start, finish = float(before['sim_time_s']), float(final['sim_time_s'])
    configuration = read_json(source / 'configuration.json')
    footprint = configuration['mpc_footprint']
    geometry = scene_geometry(ROOT / 'prototype/sim/scenes/room_elevator.xml')
    telemetry = [row for _, row in window_rows(source / 'telemetry.jsonl', start, finish)]
    rows = []
    for row in [before, *telemetry, final]:
        if rows and abs(row['sim_time_s'] - rows[-1]['sim_time_s']) < TIME_EPS:
            rows[-1] = row
        else:
            rows.append(row)
    poses = np.asarray([row['pose_world'] for row in rows])
    times = np.asarray([row['sim_time_s'] for row in rows])
    crossing = crossing_evidence(rows, footprint, geometry['doorway'])
    final_second = [row for row in rows if row['sim_time_s'] >= finish - 1.]
    final_speeds = [math.hypot(*row['actual_planar_twist'][:2]) for row in final_second]
    mpc_rows = [row for _, row in window_rows(source / 'mpc.jsonl', start, finish)]
    mpc_counts = Counter(row['solver_status'] for row in mpc_rows)
    return {
        'episode': str(episode), 'raw_server_output': str(source), 'scene': summary['scene'],
        'goal': summary['goal'], 'sim_time_window_s': [start, finish],
        'geometry': geometry, 'runtime_recorded_footprint': footprint,
        'contacts': contact_evidence(source, start, finish),
        'trajectory': {'columns': ['sim_time_s', 'base_x_m', 'base_y_m', 'base_yaw_rad'],
                       'samples': [[row['sim_time_s'], *row['pose_world']] for row in rows],
                       'nominal_telemetry_period_s': CONTROL_DT,
                       'largest_sample_gap_s': float(np.max(np.diff(times))) if len(times) > 1 else None,
                       'initial_pose_world': before['pose_world'], 'final_pose_world': final['pose_world'],
                       'initial_base_z_m': before['base_z_m'], 'final_base_z_m': final['base_z_m'],
                       'minimum_base_z_m': min(row['base_z_m'] for row in rows),
                       'path_length_m': float(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum()),
                       'net_xy_displacement_m': float(np.linalg.norm(poses[-1, :2] - poses[0, :2]))},
        'doorway_crossing': crossing,
        'elevator_distance': {'definition': 'Base-origin horizontal distance to the finite elevator leaf front-face segments from scene XML; excludes surround/button/handle. No arrival threshold is imposed.',
                              'initial': door_face_distance(before['pose_world'], geometry['elevator_faces']),
                              'final': door_face_distance(final['pose_world'], geometry['elevator_faces'])},
        'stopping_observation': {'final_actual_planar_twist_vx_vy_wz': final['actual_planar_twist'],
                                 'final_actual_velocity_body_xyz': final['actual_velocity_body'],
                                 'final_planar_speed_m_s': math.hypot(*final['actual_planar_twist'][:2]),
                                 'final_requested_cmd': final['requested_cmd'], 'final_effective_cmd': final['effective_cmd'],
                                 'last_one_second_samples': len(final_second),
                                 'last_one_second_planar_speed_mean_m_s': float(np.mean(final_speeds)),
                                 'last_one_second_planar_speed_max_m_s': max(final_speeds),
                                 'definition': 'Actual velocities are reported directly; zero requested command is not treated as proof of stopping.'},
        'mpc': {'status_counts': dict(mpc_counts),
                'blocked_or_failed_calls': [{key: row[key] for key in ('sim_time_s', 'pose_world', 'solver_status', 'reason')}
                                            for row in mpc_rows if row['solver_status'] != 'Solve_Succeeded']},
        'velocity_mismatch_navigating_nonzero_effective': velocity_mismatch(telemetry),
        'monitor_stages': stage_evidence(episode / 'agent.jsonl', start, finish, footprint, geometry['doorway'], crossing),
        'reported_final_server_state': final['state'],
        'interpretation': 'Independent geometric, contact and kinematic observations only; no model/monitor semantic outcome is converted into physical PASS.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', required=True, type=Path)
    args = parser.parse_args()
    episode = args.episode.resolve()
    result = evaluate(episode)
    output = episode / 'physical_evaluation.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(output)


if __name__ == '__main__':
    main()
