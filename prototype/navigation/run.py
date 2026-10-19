"""Default simplified A2 HEAD RGB deployment entry; no task until selection."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import shutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'prototype'))
sys.path.insert(0, str(ROOT/'prototype/sim'))


async def run(args):
    task = json.loads(args.task.read_text())
    config = json.loads(args.config.read_text())
    spawn = args.spawn if args.preview_only and args.spawn is not None else task['spawn_xyz_yaw_deg']
    if spawn is None or (not args.preview_only and not task['goal']):
        raise ValueError('New default task is pending START / GOAL / heading selection; old task will not run')
    if len(spawn) != 4 or not all(isinstance(x, (float, int)) for x in spawn):
        raise ValueError('Spawn must be [x, y, z, yaw_deg]')
    if config['sensor_mode'] != 'rgb' or config['camera_profile'] != 'a2_head_isaaclab':
        raise ValueError('The simplified deployment uses A2 HEAD RGB only')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    # Refuse a busy physical GPU; never terminate foreign jobs or choose a CPU fallback.
    physical_uuid = subprocess.check_output(['nvidia-smi', '-i', str(args.sim_gpu),
        '--query-gpu=uuid', '--format=csv,noheader'], text=True).strip()
    apps = subprocess.check_output(['nvidia-smi', '--query-compute-apps=pid,gpu_uuid',
        '--format=csv,noheader'], text=True)
    busy = [line for line in apps.splitlines() if physical_uuid in line]
    if busy:
        raise RuntimeError(f'Physical GPU{args.sim_gpu} is busy: {busy}; select an idle GPU explicitly')
    os.environ['LIGHTNAV_GPU'] = str(args.sim_gpu)
    os.environ['PATH'] = str(ROOT/'.local/bin')+os.pathsep+os.environ['PATH']
    if shutil.which('ffmpeg') is None:
        raise RuntimeError('The project ffmpeg binary must be installed under .local/bin before plant launch')
    from gpu import configure_cuda
    binding = configure_cuda()
    from isaac.client import IsaacA2Simulation
    from navigation.engine import RGBNavigation
    environment = os.environ.copy()
    for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
        environment.pop(key, None)
    environment.update(NO_PROXY='*', no_proxy='*', ISAAC_RENDERER_GPU=str(args.sim_gpu))
    sock = Path('/tmp')/f'guidepup-rgb-{os.getpid()}.sock'
    command = ['bash', str(ROOT/'prototype/isaac/run.sh'), '--socket', str(sock),
        '--usd-path', str((ROOT/task['asset']).resolve()), '--spawn', *map(str, spawn),
        '--output', str(output/'physics'), '--sensor-mode', 'rgb', '--camera-profile', 'a2_head_isaaclab']
    receipt = dict(config=config, task=task, actual_spawn=spawn, camera='a2_head_isaaclab',
        mode='stationary preview' if args.preview_only else 'one RGB-only navigation task',
        gpu=binding, command=command, lightnav_service_ownership='external; never unloaded by this entry',
        started_epoch=time.time(), source='Current working tree; existing uncommitted work retained')
    (output/'manifest.json').write_text(json.dumps(receipt, indent=2)+'\n')
    plant = None
    log = (output/'physics.log').open('w')
    process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
    try:
        while not sock.exists():
            if process.poll() is not None:
                raise RuntimeError(f'RGB plant exited {process.returncode}; see {output/"physics.log"}')
            await asyncio.sleep(.5)
        plant = IsaacA2Simulation(str(sock), 'a2_head_isaaclab')
        rendering = plant.ready['rendering']
        if (rendering['cuda_device_uuid'].removeprefix('GPU-') != physical_uuid.removeprefix('GPU-')
                or int(rendering['renderer_gpu']) != args.sim_gpu):
            raise RuntimeError(f'CUDA/RTX allocation differs from requested physical GPU: {rendering}')
        receipt.update(rendering=rendering, camera_calibration=plant.ready['camera_calibration'])
        (output/'manifest.json').write_text(json.dumps(receipt, indent=2)+'\n')
        engine = RGBNavigation(plant, config, ROOT/config['controller_config'], output/'episode', task['goal'])
        result = await engine.run(preview_only=args.preview_only)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['status'] in ('COMPLETED', 'PREVIEW_COMPLETE') else 2
    finally:
        if plant is not None:
            plant.close()
        elif process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        sock.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'prototype/navigation/config.json')
    parser.add_argument('--task', type=Path, default=ROOT/'prototype/navigation/task.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sim-gpu', type=int, default=4, help='Physical GPU for Isaac and CUDA MPPI; GPU3 model remains resident')
    parser.add_argument('--preview-only', action='store_true', help='Stationary HEAD/WBC/IMU bringup; no planner or navigation inference')
    parser.add_argument('--spawn', nargs=4, type=float, help='Preview-only view pose; never sets the default task')
    args = parser.parse_args()
    if args.spawn is not None and not args.preview_only:
        parser.error('--spawn is a preview-only override; set the selected task explicitly')
    return asyncio.run(run(args))


if __name__ == '__main__':
    raise SystemExit(main())
