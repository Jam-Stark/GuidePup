"""Run one Isaac multi-room navigation; scene annotations stay in setup/evaluation."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import aiohttp

ROOT = Path(__file__).resolve().parents[2]
SIM_PYTHON = ROOT / '.local/envs/sim/bin/python'


async def run(args):
    scenario = json.loads(args.scenario.read_text())
    output = args.output.resolve()
    output.mkdir(parents=True)
    shutil.copy2(args.scenario, output / 'scenario.json')
    socket_path = output / 'plant.sock'
    if len(str(socket_path)) >= 104:
        socket_path = Path('/tmp') / f'guidepup-isaac-{os.getpid()}.sock'
    environment = os.environ.copy()
    for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
        environment.pop(key, None)
    environment.update(LIGHTNAV_GPU=str(args.sim_gpu), ISAAC_RENDERER_GPU=str(args.sim_gpu), NO_PROXY='*', no_proxy='*', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    environment['PATH'] = str(ROOT / '.local/bin') + os.pathsep + environment['PATH']
    sim_url = f'http://127.0.0.1:{args.navigation_port}'
    environment['SIM_URL'] = sim_url
    status = {'state': 'STARTING', 'started_epoch': time.time(), 'scenario': str(args.scenario),
              'output': str(output), 'processes': {},
              'gpu_allocation': {'navigation_model': args.lightnav_gpu, 'simulation_and_local_gpu_tasks': args.sim_gpu}, 'scope': 'scene loading validation' if args.preview_only else 'one fresh no-intervention Isaac navigation'}
    processes, streams = {}, []

    def save(**values):
        status.update(values)
        (output / 'run_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2) + '\n')

    def launch(name, command, cwd=ROOT, gpu=None):
        log = (output / f'{name}.log').open('w')
        streams.append(log)
        child_environment = environment.copy()
        if gpu is not None:
            child_environment['LIGHTNAV_GPU'] = str(gpu)
        process = subprocess.Popen([str(value) for value in command], cwd=cwd, env=child_environment,
                                   stdout=log, stderr=subprocess.STDOUT)
        processes[name] = process
        status['processes'][name] = {'pid': process.pid, 'command': list(map(str, command)),
                                     'physical_gpu': args.sim_gpu if gpu is None else gpu}
        save()
        return process

    async def wait_ready(name, check, timeout_s=600):
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while not await check():
            if processes[name].poll() is not None:
                raise RuntimeError(f'{name} exited before readiness; see {name}.log')
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(f'{name} startup exceeded {timeout_s}s')
            await asyncio.sleep(1)

    async def stop_process(name):
        process = processes[name]
        if process.poll() is None:
            process.terminate()
        try:
            await asyncio.to_thread(process.wait, timeout=15)
        except subprocess.TimeoutExpired:
            # Kit shutdown can hang after logs are closed; never leave an owned GPU process behind.
            process.kill()
            await asyncio.to_thread(process.wait)
            status['processes'][name]['shutdown_required_kill'] = True
        status['processes'][name]['exit_code'] = process.returncode

    recorded = False
    try:
        asset = (ROOT / scenario['asset']).resolve()
        plant_command = ['bash', ROOT / 'prototype/isaac/run.sh', '--socket', socket_path,
            '--usd-path', asset, '--spawn', *scenario['spawn_xyz_yaw_deg'], '--output', output / 'physics']
        if args.camera_forward_m is not None:
            plant_command += ['--camera-forward-m', args.camera_forward_m]
        if scenario.get('disable_ujitso_cooking', False):
            plant_command += ['--disable-ujitso-cooking']
        launch('physics', plant_command)

        async def socket_ready():
            return socket_path.exists()
        await wait_ready('physics', socket_ready, timeout_s=None)
        launch('navigation', [SIM_PYTHON, ROOT / 'prototype/sim/server.py', '--backend', 'isaac',
            '--isaac-socket', socket_path, '--scene', scenario['name'], '--record-contacts',
            '--record-navigation-inputs', '--controller-footprint', args.controller_footprint.resolve(),
            '--setup-release-file', output / 'release_setup', '--port', args.navigation_port,
            '--lightnav-instruction-prefix', args.lightnav_instruction_prefix,
            '--semantic-classes', *scenario['semantic_classes']],
            cwd=ROOT / '.local/models/perception')
        async with aiohttp.ClientSession(trust_env=False, timeout=aiohttp.ClientTimeout(total=3)) as session:
            async def navigation_ready():
                try:
                    async with session.get(sim_url + '/status') as response:
                        response.raise_for_status()
                        current = await response.json()
                        if current['state'] == 'ERROR':
                            raise RuntimeError(f"Navigation startup failed: {current['reason']}")
                        return (current['robot_state']['tracking'] == 'tracking'
                                and current['obstacle_map_capture_time_s'] is not None)
                except (aiohttp.ClientConnectorError, asyncio.TimeoutError):
                    return False
            await wait_ready('navigation', navigation_ready)
            async with session.get(sim_url + '/scene.jpg') as response:
                response.raise_for_status()
                (output / 'preview.jpg').write_bytes(await response.read())
            async with session.get(sim_url + '/status') as response:
                initial = await response.json()
                (output / 'preview_status.json').write_text(json.dumps(initial, indent=2)+'\n')
            save(state='PREVIEW_READY', raw_output=initial['output'])
            if args.preview_only:
                save(state='PREVIEW_COMPLETE', full_navigation_tasks=0)
                return 0
            if args.wait_for_start:
                while not (output / 'start_task').exists():
                    if processes['navigation'].poll() is not None or processes['physics'].poll() is not None:
                        raise RuntimeError('Backend exited while the rendered preview was being inspected')
                    await asyncio.sleep(1)
            if not args.reuse_lightnav:
                launch('lightnav', ['bash', ROOT / 'prototype/lightnav/serve.sh'], gpu=args.lightnav_gpu)

            async def lightnav_ready():
                try:
                    async with session.ws_connect('http://127.0.0.1:8050') as ws:
                        await ws.send_json({'action': 'login', 'data': {'clientId': 'isaac-launcher'}})
                        result = await ws.receive_json()
                        if result['action'] != 'login' or result['data']['rc'] != 0:
                            raise RuntimeError(f'LightNav login failed: {result}')
                        return True
                except (aiohttp.ClientConnectorError, asyncio.TimeoutError):
                    return False
            if args.reuse_lightnav:
                if not await lightnav_ready():
                    raise RuntimeError('The requested resident LightNav service is not ready on port8050; no replacement model was started.')
                save(model_service={'ownership': 'external', 'reused': True,
                                    'physical_gpu': args.lightnav_gpu, 'url': 'ws://127.0.0.1:8050',
                                    'shutdown_with_experiment': False})
            else:
                await wait_ready('lightnav', lightnav_ready, 180)
        (output / 'release_setup').write_text('Models ready; begin episode from inspected sensor state.\n')
        save(state='NAVIGATION_RUNNING')
        episode = output / 'episode'
        process = launch('recording', [SIM_PYTHON, ROOT / 'prototype/sim/record_monitor.py',
            '--connect-existing', '--scene', scenario['name'], '--goal', scenario['goal'],
            '--model', 'qwen', '--interaction', 'none', '--output', episode,
            '--decision-timeout-s', '30', '--max-recoveries', '3',
            '--record-navigation-inputs', '--spawn-pose', scenario['spawn_xyz_yaw_deg'][0],
            scenario['spawn_xyz_yaw_deg'][1], scenario['spawn_xyz_yaw_deg'][3]])
        code = await asyncio.to_thread(process.wait)
        recorded = (episode / 'summary.json').exists()
        save(state='RECORDING_ENDED', recording_exit_code=code)
        if not recorded:
            raise RuntimeError(f'Recorder exited {code} without an episode summary')
    except Exception as error:
        save(state='ERROR', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        for name in ('recording', 'navigation', 'physics', 'lightnav'):
            if name in processes:
                await stop_process(name)
        save(elapsed_wall_s=time.time()-status['started_epoch'])
        for stream in streams:
            stream.close()
    if recorded:
        subprocess.run([str(SIM_PYTHON), str(ROOT / 'prototype/isaac/evaluate.py'),
                        '--run', str(output)], cwd=ROOT, check=True)
        save(state='ERROR' if code == 1 else 'FINISHED',
             elapsed_wall_s=time.time()-status['started_epoch'])
        return code


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, default=ROOT / 'prototype/isaac/scenarios/campus_mf_r1_elevator.json')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--sim-gpu', type=int, default=int(os.environ.get('LIGHTNAV_GPU', '0')), help='Physical GPU for Isaac rendering, physics and other local GPU work.')
    parser.add_argument('--lightnav-gpu', type=int, default=int(os.environ.get('LIGHTNAV_GPU', '0')), help='Physical GPU dedicated to navigation-model inference.')
    parser.add_argument('--reuse-lightnav', action='store_true', help='Require the existing resident model on port8050; leave it loaded when this experiment ends. Never launch a fallback model.')
    parser.add_argument('--navigation-port', type=int, default=8768, help='Isolated navigation service port; also passed to the Pi recorder.')
    parser.add_argument('--preview-only', action='store_true', help='Load A2, validate RGB-D/state/map, save a preview and exit without model calls or a task.')
    parser.add_argument('--controller-footprint', type=Path, default=ROOT / 'prototype/control/footprint.json')
    parser.add_argument('--lightnav-instruction-prefix', default='')
    parser.add_argument('--camera-forward-m', type=float)
    parser.add_argument('--wait-for-start', action='store_true', help='Save rendered preview and wait for local start_task file before submission.')
    raise SystemExit(asyncio.run(run(parser.parse_args())))
