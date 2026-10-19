"""Run the fixed 36-pose/arm suite with Qwen, then conditional Gemini comparison."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from record_task import ROOT, snapshot_sources
from run_navigation_stress import run as run_stress, save
from evaluate_navigation_stress import target_anchor
sys.path.insert(0, str(ROOT / 'prototype'))
from gpu import configure_gpu


BASELINES = (
    ('broad', ROOT / '.local/outputs/navigation_stress/cohort_01/manifest.json'),
    ('door_facing', ROOT / '.local/outputs/navigation_stress/door_facing_01/manifest.json'),
)
def prepare(output):
    output.mkdir(parents=True, exist_ok=True)
    binding = configure_gpu()
    gpu_environment = {
        'CUDA_VISIBLE_DEVICES': binding['device_uuid'], 'MUJOCO_GL': 'egl',
        'MUJOCO_EGL_DEVICE_ID': str(binding['egl_device_index']),
        '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
    }
    sources = [(cohort, path, json.loads(path.read_text())) for cohort, path in BASELINES]
    for model in ('qwen', 'gemini'):
        destination = output / model
        destination.mkdir()
        snapshot_sources(destination)
        poses, trials, cohorts = [], [], []
        for cohort, path, previous in sources:
            poses.extend({**pose, 'cohort': cohort} for pose in previous['poses'])
            trials.extend({**trial, 'cohort': cohort} for trial in previous['trials'])
            cohorts.append({'id': cohort, 'source_manifest': str(path), 'seed': previous['seed'],
                            'sampling': previous['sampling'], 'pose_count': len(previous['poses']),
                            'trial_count': len(previous['trials'])})
        manifest = {
            'version': 'navigation-yolo-observation-v1', 'model_profile': model,
            'created_at': time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'seed': [entry['seed'] for entry in cohorts],
            'goal': '出门走到电梯门口', 'scene': 'room_elevator',
            'cohorts': cohorts, 'poses': poses, 'trials': trials,
            'sampling': {'definition': 'Exact previous admitted poses and arm ordering; two populations reported separately.',
                         'cohorts': cohorts},
            'timeout_s': None, 'decision_timeout_s': 30, 'max_recoveries': 3,
            'renderer_environment': gpu_environment,
            'physical_gpu_binding': binding,
            'source_snapshot': str(destination / 'source_snapshot'),
            'pairing_limit': 'Same initial pose and runtime, independent model calls and dynamics; no restored identical prefix.',
            'admission': 'All 36 predeclared slots retained, including errors and force-unexposed cases.',
            'force_intervention': sources[0][2]['force_intervention'],
            'stop_condition': 'All 36 completed outcomes; startup/recorder failure without summary aborts, no automatic retry.',
        }
        save(destination / 'manifest.json', manifest)
    save(output / 'protocol.json', {
        'planned_per_model': 36, 'qwen_minimum_full_passes': 29,
        'gemini_trigger': 'Qwen full_passes<29 OR false_completion_count>0 OR contact_episode_count>0',
        'model_options': {'qwen': {'thinking': 'off', 'output_cap': 2048},
                          'gemini': {'thinking': 'low', 'output_cap': 4096}},
        'truth_access': 'Scene truth used only by offline evaluator, never perception/monitor/controller.',
        'gpu_environment': gpu_environment,
        'physical_gpu_binding': binding,
    })


def episode_result(trial, directory):
    episode = directory / trial['id']
    summary = json.loads((episode / 'summary.json').read_text())
    physical = json.loads((episode / 'physical_evaluation.json').read_text())
    evidence = json.loads((episode / 'v2_evidence.json').read_text())
    events = [json.loads(line) for line in (episode / 'agent.jsonl').read_text().splitlines()]
    terminals = [row for row in events if row['kind'] in ('task_completed', 'task_stopped')]
    terminal = terminals[-1]['data']
    completed = terminal['status'] == 'COMPLETED'
    actual = evidence['actual_elevator_stop']
    comparison = actual['contract_comparison_geometry_only']
    clearance, clearance_range = comparison['actual_front_clearance_vs_contract_range']
    heading, heading_limit = comparison['actual_heading_error_vs_contract_deg']
    lateral, lateral_limit = comparison['actual_leaf_projection_lateral_offset_vs_observed_patch_m']
    stage0 = evidence['stage0_actual_far_jamb']
    stop = physical['stopping_observation']
    contacts = physical['contacts']['positive_non_support']['contact_point_records']
    anchor = target_anchor(events, physical, evidence)
    checks = {
        'true_whole_body_exit_at_stage_completion': stage0['status'] == 'OBSERVED'
            and bool(stage0['stage0_completed']) and all(
                row['already_fully_clear_of_actual_far_jamb'] for row in stage0['stage0_completed']),
        'true_elevator_clearance': clearance_range[0] <= clearance <= clearance_range[1],
        'true_elevator_heading': abs(heading) <= heading_limit,
        'true_elevator_lateral': lateral <= lateral_limit,
        'stopped': stop['final_planar_speed_m_s'] < .05 and all(
            abs(value) < 1e-6 for key in ('final_requested_cmd', 'final_effective_cmd') for value in stop[key]),
        'no_positive_non_support_contact': contacts == 0,
    }
    geometry_pass = all(checks.values())
    requests = [row['data'] for row in events if row['kind'] == 'decision_request']
    return {
        **trial, 'episode': str(episode), 'monitor_status': terminal['status'],
        'terminal_reason': terminal['reason'], 'full_pass': completed and geometry_pass,
        'geometry_and_contact_pass': geometry_pass, 'false_completion': completed and not geometry_pass,
        'checks': checks, 'true_elevator_front_clearance_m': clearance,
        'target_roi_point_on_true_elevator_leaf': anchor.get('point_on_actual_leaf'),
        'target_anchor_geometry': anchor,
        'heading_error_deg': heading, 'lateral_offset_m': lateral,
        'positive_non_support_contact_records': contacts,
        'true_exit_observed': evidence['doorway_actual_far_jamb'].get('first_observed_sim_time_s') is not None,
        'force_brake_observed': summary.get('force_brake_observed', False),
        'interaction_phase': summary.get('interaction_phase'),
        'model_calls': len(requests), 'elapsed_wall_s': summary['elapsed_wall_s'],
        'model_configuration': {key: requests[0][key] for key in
                                ('model', 'provider', 'baseUrl', 'reasoningEffort', 'maxTokens')},
    }


def reduce_suite(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    episodes = [episode_result(trial, directory) for trial in manifest['trials']]
    groups = []
    for cohort in ('broad', 'door_facing'):
        for arm in ('primary', 'force_resume'):
            selected = [row for row in episodes if row['cohort'] == cohort and row['arm'] == arm]
            groups.append({'cohort': cohort, 'arm': arm, 'count': len(selected),
                           'full_passes': sum(row['full_pass'] for row in selected),
                           'monitor_completed': sum(row['monitor_status'] == 'COMPLETED' for row in selected),
                           'true_exits': sum(row['true_exit_observed'] for row in selected),
                           'force_brakes': sum(row['force_brake_observed'] for row in selected)})
    result = {
        'model_profile': manifest['model_profile'], 'count': len(episodes),
        'full_passes': sum(row['full_pass'] for row in episodes),
        'false_completion_count': sum(row['false_completion'] for row in episodes),
        'contact_episode_count': sum(row['positive_non_support_contact_records'] > 0 for row in episodes),
        'groups': groups, 'episodes': episodes,
    }
    save(directory / 'comparison_summary.json', result)
    return result


def run(output):
    python = ROOT / '.local/envs/sim/bin/python'
    summaries = []
    for model in ('qwen', 'gemini'):
        directory = output / model
        save(output / 'comparison_state.json', {'state': 'RUNNING', 'current_model': model, 'completed': summaries})
        run_stress(directory)
        subprocess.run([str(python), str(ROOT / 'prototype/sim/evaluate_navigation_stress.py'),
                        '--batch', str(directory)], cwd=ROOT, check=True)
        result = reduce_suite(directory)
        summaries.append({key: value for key, value in result.items() if key != 'episodes'})
        if model == 'qwen':
            trigger = result['full_passes'] < 29 or result['false_completion_count'] > 0 or result['contact_episode_count'] > 0
            save(output / 'gemini_decision.json', {'run_gemini': trigger, 'qwen_result': summaries[-1],
                                                  'criterion': json.loads((output / 'protocol.json').read_text())['gemini_trigger']})
            if not trigger:
                break
    save(output / 'comparison_state.json', {'state': 'COMPLETED', 'completed': summaries})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['prepare', 'run', 'reduce'])
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    {'prepare': prepare, 'run': run, 'reduce': reduce_suite}[args.command](args.output.resolve())
