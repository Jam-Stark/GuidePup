"""Start the installed prototype and hand a task to the persistent Pi runtime."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

import aiohttp

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / '.local'
SERVICES = ('forcecontrol-lightnav-serve', 'forcecontrol-lightnav-sim')


def session_exists(name):
    return subprocess.run(['tmux', 'has-session', '-t', name], capture_output=True).returncode == 0


async def ready(kind):
    try:
        async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=2)) as client:
            if kind == 'sim':
                async with client.get('http://127.0.0.1:8766/status') as response:
                    response.raise_for_status()
                    return await response.json()
            async with client.ws_connect('http://127.0.0.1:8050') as ws:
                await ws.send_json({'action': 'login', 'data': {'clientId': 'prototype-launcher'}})
                result = await ws.receive_json()
                if result['action'] != 'login' or result['data']['rc'] != 0:
                    raise RuntimeError(f'LightNav login failed: {result}')
                return {'ready': True}
    except (aiohttp.ClientConnectorError, TimeoutError):
        return None


async def start(name, script, args, kind):
    if session_exists(name):
        if await ready(kind):
            print(f'{name}: ready (existing)', flush=True)
            return
        raise RuntimeError(f'{name} exists but is not ready; inspect its log, or run down then up.')
    log = DATA / 'outputs' / f'{name}.log'
    # Both service launchers source prototype/gpu.sh inside tmux.
    command = shlex.join(['env', f"LIGHTNAV_GPU={os.environ.get('LIGHTNAV_GPU', '0')}",
                          'bash', str(ROOT / script), *args])
    subprocess.run(['tmux', 'new-session', '-d', '-s', name, '-c', str(ROOT),
                    f'exec {command} > {shlex.quote(str(log))} 2>&1'], check=True)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        if not session_exists(name):
            raise RuntimeError(f'{name} exited; read {log}')
        if await ready(kind):
            print(f'{name}: ready; log={log}', flush=True)
            return
        await asyncio.sleep(1)
    raise TimeoutError(f'{name} did not become ready in 120 seconds; read {log}')


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['build', 'up', 'down', 'reset', 'status', 'chat', 'run', 'monitor'])
    parser.add_argument('pi_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command == 'build':
        subprocess.run([str(DATA / 'envs/node-v22.23.2-linux-x64/bin/node'),
                        str(ROOT / 'prototype/pi/node_modules/typescript/bin/tsc')],
                       cwd=ROOT / 'prototype/pi', check=True)
        print('Pi TypeScript build completed.')
        return
    if args.command == 'down':
        for name in reversed(SERVICES):
            if session_exists(name):
                subprocess.run(['tmux', 'kill-session', '-t', name], check=True)
                print(f'{name}: stopped')
        return
    if args.command == 'status':
        print(json.dumps({'lightnav': await ready('lightnav'), 'sim': await ready('sim')}, indent=2))
        return
    if args.command == 'reset' and session_exists(SERVICES[1]):
        subprocess.run(['tmux', 'kill-session', '-t', SERVICES[1]], check=True)
    (DATA / 'outputs').mkdir(exist_ok=True)
    await start(SERVICES[0], 'prototype/lightnav/serve.sh', [], 'lightnav')
    sim_args = args.pi_args if args.command in ('up', 'reset') else []
    await start(SERVICES[1], 'prototype/sim/run.sh', sim_args, 'sim')
    print('Camera/status: http://127.0.0.1:8766/', flush=True)
    if args.command in ('chat', 'run', 'monitor'):
        os.execv(str(ROOT / 'prototype/pi/run.sh'),
                 [str(ROOT / 'prototype/pi/run.sh'), args.command, *args.pi_args])


if __name__ == '__main__':
    asyncio.run(main())
