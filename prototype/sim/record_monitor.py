"""Record the fixed resident-monitor force/pause/question/resume trial."""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import time

import aiohttp
from record_task import snapshot_sources, model_environment

ROOT = Path(__file__).resolve().parents[2]
SIM = 'http://127.0.0.1:8766'
MONITOR = 'http://127.0.0.1:8770'
SCENE = 'room_elevator'
GOAL = '出门走到电梯门口'
QUESTION = '我们已经出门了吗？现在看到什么，为什么停了？'
TERMINAL = {'COMPLETED', 'NEEDS_USER', 'ERROR', 'LIMIT_REACHED', 'CANCELLED'}


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def decision_requests(path):
    with path.open() as source:
        return sum(json.loads(line)['kind'] == 'decision_request' for line in source)


async def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True)
    snapshot_sources(output)
    environment = os.environ.copy()
    for key in ('http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY'):
        environment.pop(key, None)
    environment.update(NO_PROXY='*', no_proxy='*')
    environment.update(model_environment(args.model))
    with (output / 'setup.log').open('wb') as setup_log:
        setup = await asyncio.create_subprocess_exec(
            'bash', str(ROOT / 'prototype/run.sh'), 'reset', '--scene', SCENE,
            '--record-contacts', *(['--record-navigation-inputs'] if args.record_navigation_inputs else []),
            *([] if args.spawn_pose is None else ['--spawn-pose', *map(str, args.spawn_pose)]),
            cwd=ROOT, env=environment,
            stdout=setup_log, stderr=asyncio.subprocess.STDOUT)
        if await setup.wait() != 0:
            raise RuntimeError('Simulator setup failed; task was not started. See setup.log.')

    async with aiohttp.ClientSession(trust_env=False) as client:
        async def request(base, path, body=None, timeout_s=15):
            async with client.request('GET' if body is None else 'POST', base + path,
                                      json=body, timeout=aiohttp.ClientTimeout(total=timeout_s)) as response:
                response.raise_for_status()
                return await response.json()

        async def rgb():
            async with client.get(SIM + '/scene.jpg', timeout=aiohttp.ClientTimeout(total=15)) as response:
                response.raise_for_status()
                return await response.read()

        before = await request(SIM, '/status')
        save_json(output / 'before.json', before)
        (output / 'initial.jpg').write_bytes(await rgb())
        video = await asyncio.create_subprocess_exec(
            'ffmpeg', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg',
            '-framerate', '5', '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'fast',
            '-crf', '23', '-pix_fmt', 'yuv420p', str(output / 'camera.mp4'),
            stdin=asyncio.subprocess.PIPE)
        started = time.monotonic()
        pi_command = (['bash', str(ROOT / 'prototype/pi/zenmux.sh'), 'gemini'] if args.model == 'gemini'
                      else ['bash', str(ROOT / 'prototype/pi/run.sh')])
        process = await asyncio.create_subprocess_exec(
            *pi_command, 'monitor', '--port', '8770',
            '--log', str(output / 'agent.jsonl'), '--db', str(output / 'tasks.sqlite'),
            '--timeout-s', str(args.timeout_s), '--max-recoveries', str(args.max_recoveries),
            '--decision-timeout-s', str(args.decision_timeout_s), cwd=ROOT, env=environment,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)

        async def collect():
            with (output / 'console.log').open('wb') as console:
                async for line in process.stdout:
                    console.write(line)
                    console.flush()

        console_task = asyncio.create_task(collect())
        interactions = []

        def event(kind, **data):
            interactions.append({'elapsed_wall_s': time.monotonic() - started, 'kind': kind, **data})
            save_json(output / 'interactionlog.json', interactions)

        task_id = None
        task_started = None
        monitor_state = None
        phase = 'WAITING_FOR_MOTION'
        force_until = None
        brake_seen = False
        paused_pose = None
        pause_max_displacement_m = 0.
        question_task = None
        question_started = None
        terminal_state = None
        terminal_started = None
        terminal_request_count = None
        idle_evidence = None
        recorder_error = None

        paused_at_sim_time = None

        def require_pause(status, snapshot):
            nonlocal pause_max_displacement_m
            displacement = math.hypot(status['pose_world'][0] - paused_pose[0],
                                      status['pose_world'][1] - paused_pose[1])
            pause_max_displacement_m = max(pause_max_displacement_m, displacement)
            if status['state'] != 'HUMAN_PAUSED' or snapshot['task']['currentStage'] != 0:
                raise RuntimeError('Human pause or original stage changed before explicit resume.')
            if any(abs(value) > 1e-6 for value in status['requested_cmd']):
                raise RuntimeError('A new navigation request was observed during HUMAN_PAUSED.')
            # The brake is detected after a policy step. effective_cmd at that
            # exact timestamp describes the step just executed; next step must be zero.
            if (status['sim_time_s'] > paused_at_sim_time
                    and any(abs(value) > 1e-6 for value in status['effective_cmd'])):
                raise RuntimeError('Nonzero effective command persisted after the brake observation tick.')

        try:
            with (output / 'samples.jsonl').open('w', buffering=1) as samples:
                while True:
                    tick = time.monotonic()
                    status = await request(SIM, '/status')
                    samples.write(json.dumps(status) + '\n')
                    video.stdin.write(await rgb())
                    await video.stdin.drain()
                    if process.returncode is not None:
                        raise RuntimeError(f'Resident monitor exited unexpectedly: {process.returncode}')
                    try:
                        monitor_state = await request(MONITOR, '/state')
                    except aiohttp.ClientConnectorError:
                        if task_id is not None or time.monotonic() - started >= 30:
                            raise
                        await asyncio.sleep(max(0., .2 - (time.monotonic() - tick)))
                        continue

                    if task_id is None:
                        accepted = await request(MONITOR, '/task', {'goal': GOAL})
                        task_id = accepted['taskId']
                        task_started = time.monotonic()
                        event('task_submitted', goal=GOAL, response=accepted, status=status)
                    else:
                        task = monitor_state['task']
                        if task['status'] in TERMINAL:
                            if terminal_state is None:
                                terminal_state = monitor_state
                                terminal_started = time.monotonic()
                                terminal_request_count = decision_requests(output / 'agent.jsonl')
                                event('monitor_terminal', monitor=monitor_state, status=status,
                                      decision_requests=terminal_request_count)
                                if phase != 'RESUMED':
                                    recorder_error = f'Task terminated before the fixed interaction finished (phase={phase}).'
                                    event('interaction_incomplete', error=recorder_error)
                            if time.monotonic() - terminal_started >= 3:
                                after_count = decision_requests(output / 'agent.jsonl')
                                idle_evidence = {
                                    'elapsed_s': time.monotonic() - terminal_started,
                                    'decision_requests_before': terminal_request_count,
                                    'decision_requests_after': after_count,
                                    'new_decision_requests': after_count - terminal_request_count,
                                    'resident_alive': process.returncode is None,
                                    'monitor': monitor_state,
                                }
                                event('resident_after_terminal', **idle_evidence)
                                break
                        elif time.monotonic() - task_started >= args.timeout_s:
                            raise TimeoutError('Recorder wall-clock episode limit reached before a monitor terminal result.')
                        elif phase == 'WAITING_FOR_MOTION':
                            travelled = math.hypot(status['pose_world'][0] - before['pose_world'][0],
                                                   status['pose_world'][1] - before['pose_world'][1])
                            if task['currentStage'] > 0:
                                raise RuntimeError('Stage 0 ended without reaching the fixed force trigger.')
                            if (task['currentStage'] == 0 and status['state'] == 'NAVIGATING'
                                    and status['requested_cmd'][0] >= .2 and travelled >= .5):
                                applied = await request(SIM, '/force', {'xyz': [-80, 0, 0], 'duration_s': 2})
                                force_until = applied['sim_time_s'] + 2
                                phase = 'WAITING_FOR_FORCE_CLEAR'
                                event('force_applied', xyz_N=[-80, 0, 0], duration_s=2,
                                      force_until_sim_time_s=force_until, travelled_m=travelled,
                                      status=applied, monitor=monitor_state)
                        elif phase == 'WAITING_FOR_FORCE_CLEAR':
                            if status['state'] == 'HUMAN_PAUSED' and status['reason'] == 'force_brake':
                                if not brake_seen:
                                    brake_seen = True
                                    paused_pose = status['pose_world']
                                    paused_at_sim_time = status['sim_time_s']
                                    event('force_brake_observed', status=status, monitor=monitor_state)
                                require_pause(status, monitor_state)
                            elif brake_seen:
                                raise RuntimeError('Force brake was released before the explicit human resume.')
                            cleared = (status['sim_time_s'] >= force_until
                                       and all(value == 0 for value in status['force_applied_local_N']))
                            if cleared:
                                if not brake_seen:
                                    raise RuntimeError('The real force ended without a force-brake latch; no artificial pause was applied.')
                                require_pause(status, monitor_state)
                                event('force_cleared_pause_held', status=status, monitor=monitor_state,
                                      pause_max_displacement_m=pause_max_displacement_m)
                                question_started = time.monotonic()
                                question_task = asyncio.create_task(request(
                                    MONITOR, '/question', {'text': QUESTION}, args.decision_timeout_s + 30))
                                phase = 'ASKING_WHILE_PAUSED'
                                event('question_submitted', text=QUESTION, status=status, monitor=monitor_state)
                        elif phase == 'ASKING_WHILE_PAUSED':
                            require_pause(status, monitor_state)
                            if question_task.done():
                                answer = question_task.result()
                                event('question_answer', response=answer,
                                      elapsed_s=time.monotonic() - question_started,
                                      status=status, monitor=monitor_state,
                                      pause_max_displacement_m=pause_max_displacement_m)
                                event('before_explicit_resume', status=status, monitor=monitor_state)
                                resumed = await request(MONITOR, '/resume', {})
                                phase = 'RESUMED'
                                event('explicit_human_resume', response=resumed)
                    await asyncio.sleep(max(0., .2 - (time.monotonic() - tick)))
        except Exception as error:
            recorder_error = f'{type(error).__name__}: {error}'
            event('recorder_failed', error=recorder_error, phase=phase)
        finally:
            if monitor_state is not None:
                save_json(output / 'monitor_state.json', monitor_state)
            if question_task is not None and not question_task.done():
                question_task.cancel()
                await asyncio.gather(question_task, return_exceptions=True)
            if process.returncode is None:
                process.terminate()
            await process.wait()
            await console_task
            video.stdin.close()
            await video.wait()

        if video.returncode != 0:
            recorder_error = recorder_error or f'Video encoder failed: {video.returncode}'
        final = await request(SIM, '/status')
        (output / 'final.jpg').write_bytes(await rgb())
        # The evaluator may use this archive; it was not supplied to Qwen.
        observation = await request(SIM, '/scene')
        save_json(output / 'terminal_frame.json',
                  {key: observation[key] for key in ('frame_id', 'sim_time_s', 'pose_world')})
        outcome = terminal_state['task']['status'] if terminal_state else None
        code = 1 if recorder_error or outcome == 'ERROR' else 0 if outcome == 'COMPLETED' else 2
        summary = {
            'scene': SCENE, 'goal': GOAL, 'exit_code': code,
            'model_profile': args.model,
            'requested_spawn_pose_xy_yaw_deg': args.spawn_pose,
            'record_navigation_inputs': args.record_navigation_inputs,
            'elapsed_wall_s': time.monotonic() - started,
            'timeout_s': args.timeout_s, 'max_recoveries': args.max_recoveries,
            'decision_timeout_s': args.decision_timeout_s,
            'before': before, 'final': final, 'task_id': task_id,
            'monitor_outcome': outcome, 'monitor_process_exit_code': process.returncode,
            'interaction_phase': phase, 'force_brake_observed': brake_seen,
            'pause_max_displacement_m': pause_max_displacement_m,
            'resident_idle_evidence': idle_evidence, 'recorder_error': recorder_error,
        }
        save_json(output / 'summary.json', summary)
        print(json.dumps({'exit_code': code, 'monitor_outcome': outcome,
                          'interaction_phase': phase, 'output': str(output)}, ensure_ascii=False))
        return code


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--spawn-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--record-navigation-inputs', action='store_true')
    parser.add_argument('--timeout-s', default=360, type=int)
    parser.add_argument('--decision-timeout-s', default=30, type=int)
    parser.add_argument('--max-recoveries', default=3, type=int)
    parser.add_argument('--model', choices=['qwen', 'gemini'], default='qwen')
    raise SystemExit(asyncio.run(run(parser.parse_args())))
