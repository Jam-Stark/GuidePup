"""Record one real Pi/LightNav/A2 task in a selected scene, including failures."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import subprocess
import shutil
import time

import aiohttp

ROOT = Path(__file__).resolve().parents[2]


def model_environment(profile):
    if profile == 'qwen':
        return {'PI_PROVIDER': 'openai-compatible', 'PI_MODEL': 'Qwen3.6-35B-A3B-FP8/',
                'PI_BASE_URL': 'http://vic.precognition.team:8888/v1',
                'PI_CONTEXT_WINDOW': '262144', 'PI_MODEL_MAX_TOKENS': '2048',
                'PI_MAX_TOKENS': '2048', 'PI_VISION': '1', 'PI_API_KEY': ''}
    return {}


def snapshot_sources(output):
    destination = output / 'source_snapshot'
    destination.mkdir()
    shutil.copytree(ROOT / 'prototype/pi/src', destination / 'pi_src')
    shutil.copytree(ROOT / 'prototype/control', destination / 'control')
    shutil.copytree(ROOT / 'prototype/robot', destination / 'robot')
    shutil.copytree(ROOT / 'prototype/isaac', destination / 'isaac', ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(ROOT / 'prototype/camera', destination / 'camera', ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(ROOT / 'prototype/sim/scenes', destination / 'scenes')
    shutil.copytree(ROOT / 'prototype/perception', destination / 'perception')
    for name in ('server.py', 'a2.py', 'depth.py', 'depth_map.py', 'truth.py', 'semantic_depth.py', 'motion_segment.py', 'record_task.py', 'record_monitor.py', 'evaluate_monitor.py', 'evaluate_navigation_v2.py', 'evaluate_navigation_stress.py', 'run_navigation_stress.py', 'run_navigation_comparison.py', 'run_framework_reproductions.py'):
        shutil.copy2(ROOT / 'prototype/sim' / name, destination / name)
    for name in ('gpu.sh', 'gpu.py', 'stack.py'):
        shutil.copy2(ROOT / 'prototype' / name, destination / name)
    shutil.copy2(ROOT / 'prototype/sim/run.sh', destination / 'sim_run.sh')
    shutil.copy2(ROOT / 'prototype/lightnav/serve.sh', destination / 'lightnav_serve.sh')
    shutil.copy2(ROOT / 'prototype/lightnav/environment.sh', destination / 'lightnav_environment.sh')
    for name in ('run.sh', 'zenmux.sh', 'zenmux.mjs'):
        shutil.copy2(ROOT / 'prototype/pi' / name, destination / f'pi_{name}')
    shutil.copy2(ROOT / 'prototype/camera/profiles.json', destination / 'camera_profiles.json')
    shutil.copy2(ROOT / 'prototype/pi/navigation-contract.json', destination / 'navigation-contract.json')


async def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True)
    snapshot_sources(output)
    environment = os.environ.copy()
    for key in ('http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY'):
        environment.pop(key, None)
    environment.update(NO_PROXY='*', no_proxy='*')
    environment.update(model_environment(args.model))
    setup = await asyncio.create_subprocess_exec(
        'bash', str(ROOT / 'prototype/run.sh'), 'reset', '--scene', args.scene,
        '--record-contacts', *(['--record-navigation-inputs'] if args.record_navigation_inputs else []),
        *([] if args.spawn_pose is None else ['--spawn-pose', *map(str, args.spawn_pose)]),
        cwd=ROOT, env=environment)
    if await setup.wait() != 0:
        raise RuntimeError('Simulator setup failed; task was not started.')

    async with aiohttp.ClientSession() as client:
        async def get_json(path):
            async with client.get('http://127.0.0.1:8766' + path) as response:
                response.raise_for_status()
                return await response.json()

        async def get_rgb():
            async with client.get('http://127.0.0.1:8766/scene.jpg') as response:
                response.raise_for_status()
                return await response.read()

        before = await get_json('/status')
        (output / 'before.json').write_text(json.dumps(before, indent=2))
        (output / 'initial.jpg').write_bytes(await get_rgb())
        video = subprocess.Popen(
            ['ffmpeg', '-loglevel', 'error', '-f', 'image2pipe', '-vcodec', 'mjpeg',
             '-framerate', '5', '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'fast',
             '-crf', '23', '-pix_fmt', 'yuv420p', str(output / 'camera.mp4')],
            stdin=subprocess.PIPE)
        finished = asyncio.Event()

        async def record():
            with (output / 'samples.jsonl').open('w', buffering=1) as log:
                while not finished.is_set():
                    tick = time.monotonic()
                    log.write(json.dumps(await get_json('/status')) + '\n')
                    video.stdin.write(await get_rgb())
                    await asyncio.sleep(max(0., .2 - (time.monotonic() - tick)))

        started = time.monotonic()
        pi_command = (['bash', str(ROOT / 'prototype/pi/zenmux.sh'), 'gemini'] if args.model == 'gemini'
                      else ['bash', str(ROOT / 'prototype/pi/run.sh')])
        process = await asyncio.create_subprocess_exec(
            *pi_command, 'run', args.goal,
            '--max-recoveries', str(args.max_recoveries),
            '--decision-timeout-s', str(args.decision_timeout_s),
            '--log', str(output / 'agent.jsonl'), cwd=ROOT, env=environment,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)

        async def collect():
            try:
                with (output / 'console.log').open('wb') as console:
                    async for line in process.stdout:
                        console.write(line)
                        console.flush()
                        print(line.decode(), end='', flush=True)
                return await process.wait()
            finally:
                finished.set()

        try:
            code, _ = await asyncio.gather(collect(), record())
        finally:
            finished.set()
            if process.returncode is None:
                process.terminate()
                await process.wait()
            video.stdin.close()
            video.wait()
        if video.returncode != 0:
            raise RuntimeError(f'Video encoder failed: {video.returncode}')
        final = await get_json('/status')
        (output / 'final.jpg').write_bytes(await get_rgb())
        # Archive the terminal observation for offline geometry evaluation only.
        observation = await get_json('/scene')
        (output / 'terminal_frame.json').write_text(json.dumps(
            {k: observation[k] for k in ('frame_id', 'stamp_s', 'robot_state')}, indent=2))
        summary = {'scene': args.scene, 'goal': args.goal, 'exit_code': code,
                   'model_profile': args.model, 'decision_timeout_s': args.decision_timeout_s,
                   'requested_spawn_pose_xy_yaw_deg': args.spawn_pose,
                   'record_navigation_inputs': args.record_navigation_inputs,
                   'elapsed_wall_s': time.monotonic() - started,
                   'timeout_s': None, 'max_recoveries': args.max_recoveries,
                   'before': before, 'final': final}
        (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({'exit_code': code, 'final_state': final['state'], 'output': str(output)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True, choices=['flat', 'procthor', 'elevator_lobby', 'room_elevator'])
    parser.add_argument('--goal', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--spawn-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--record-navigation-inputs', action='store_true')
    parser.add_argument('--max-recoveries', default=3, type=int)
    parser.add_argument('--model', choices=['qwen', 'gemini'], default='qwen')
    parser.add_argument('--decision-timeout-s', default=30, type=int)
    asyncio.run(run(parser.parse_args()))
