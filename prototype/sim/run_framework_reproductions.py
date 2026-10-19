"""Run fixed framework poses, with the force interaction or its no-intervention control."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
CASES = (
    ('qwen', 'p05_force_resume', (2.7662063165748876, -7.173436056562866, 164.29460352541886)),
    ('gemini', 't01_force_resume', (2.548203902100475, -6.900806258145334, 80.58823003115508)),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='A new episode-group directory.')
    parser.add_argument('--model', choices=('both', 'qwen', 'gemini'), default='both')
    parser.add_argument('--interaction', choices=('force_resume', 'none'), default='force_resume')
    args = parser.parse_args()
    from run_navigation_comparison import episode_result

    base = args.output.resolve()
    base.mkdir(parents=True)
    selected = [case for case in CASES if args.model in ('both', case[0])]
    manifest = {
        'scene': 'room_elevator', 'goal': '出门走到电梯门口',
        'timeout_s': None, 'decision_timeout_s': 30, 'max_recoveries': 3,
        'interaction_protocol': args.interaction,
        'cases': [{'model': model, 'original_case': case, 'spawn_pose_xy_yaw_deg': pose}
                  for model, case, pose in selected],
        'scope': 'Only selected fixed poses and the explicitly chosen interaction; no new batch.',
    }
    (base / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    results = []
    for model, case, pose in selected:
        run_case = case if args.interaction == 'force_resume' else case.removesuffix('_force_resume') + '_no_interaction'
        episode = base / f'{model}_{run_case}'
        command = [sys.executable, str(ROOT / 'prototype/sim/record_monitor.py'),
                   '--model', model, '--output', str(episode), '--spawn-pose', *map(str, pose),
                   '--record-navigation-inputs',
                   '--decision-timeout-s', '30', '--max-recoveries', '3',
                   '--interaction', args.interaction]
        print(json.dumps({'running': str(episode), 'command': command}), flush=True)
        environment = os.environ.copy()
        environment['PATH'] = str(ROOT / '.local/bin') + os.pathsep + environment['PATH']
        recording = subprocess.run(command, cwd=ROOT, env=environment)
        # Exit 2 is a recorded non-success outcome, including NEEDS_USER.
        if recording.returncode not in (0, 2) and not (episode / 'summary.json').exists():
            recording.check_returncode()
        # A recorded early failure still has evidence to evaluate. Preserve the
        # process error after both evaluators; never start another episode for it.
        for evaluator in ('evaluate_monitor.py', 'evaluate_navigation_v2.py'):
            subprocess.run([sys.executable, str(ROOT / 'prototype/sim' / evaluator),
                            '--episode', str(episode)], cwd=ROOT, check=True)
        result = episode_result({'id': episode.name, 'arm': args.interaction,
                                 'cohort': 'targeted_reproduction'}, base)
        result['recording_returncode'] = recording.returncode
        results.append(result)
        (base / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({key: result[key] for key in (
            'id', 'monitor_status', 'full_pass', 'geometry_and_contact_pass', 'terminal_reason')},
            ensure_ascii=False), flush=True)
        if recording.returncode not in (0, 2):
            recording.check_returncode()


if __name__ == '__main__':
    main()
