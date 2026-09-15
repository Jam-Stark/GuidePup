"""Concrete control integration checks for pause races and monitor loss, no LLM."""
import asyncio
import json
import time
from pathlib import Path
import aiohttp

OUT = Path('.local/outputs/task_monitor/control_contract.json')

async def main():
    events = []
    async with aiohttp.ClientSession() as client:
        async def request(path, body=None, expected=200):
            method = client.get if body is None else client.post
            kw = {} if body is None else {'json': body}
            async with method('http://127.0.0.1:8766' + path, **kw) as response:
                data = await response.json()
                events.append({'at': time.time(), 'path': path, 'body': body, 'http': response.status, 'result': data})
                assert response.status == expected, (path, response.status, data)
                return data
        owner = 'control-contract-check'
        status = await request('/monitor/claim', {'monitor_id': owner})
        before = status['generation']
        await request('/pause', {'reason': 'human'})
        await request('/monitor/action', {'monitor_id': owner, 'expected_generation': before, 'action': 'navigate', 'instruction': 'Go forward'}, 409)
        paused = await request('/status')
        await request('/monitor/action', {'monitor_id': owner, 'expected_generation': paused['generation'], 'action': 'navigate', 'instruction': 'Go forward'}, 409)
        await request('/resume', {'source': 'human', 'expected_generation': before}, 409)
        await request('/resume', {'source': 'human', 'expected_generation': paused['generation']})
        resumed = await request('/status')
        assert resumed['state'] == 'IDLE' and not resumed['instruction'] and resumed['task_id'] is None
        await request('/monitor/action', {'monitor_id': owner, 'expected_generation': before, 'action': 'navigate', 'instruction': 'Go forward'}, 409)
        moving = await request('/monitor/action', {'monitor_id': owner, 'expected_generation': resumed['generation'], 'action': 'navigate', 'instruction': 'Walk forward to the red box and stop in front of it.'})
        # Deliberately omit heartbeat; Python, not the monitor, must stop the path.
        await asyncio.sleep(3.5)
        expired = await request('/status')
        assert expired['state'] == 'PAUSED' and expired['reason'] == 'monitor_timeout', expired
        assert expired['requested_cmd'] == [0, 0, 0] and expired['monitor_id'] is None
        await request('/monitor/heartbeat', {'monitor_id': owner}, 409)
        await request('/monitor/action', {'monitor_id': owner, 'expected_generation': expired['generation'], 'action': 'navigate', 'instruction': 'Go forward'}, 409)
        status = await request('/monitor/claim', {'monitor_id': owner})
        running = True
        async def heartbeat():
            while running:
                await request('/monitor/heartbeat', {'monitor_id': owner})
                await asyncio.sleep(1)
        beat = asyncio.create_task(heartbeat())
        await request('/monitor/action', {'monitor_id': owner, 'expected_generation': status['generation'], 'action': 'navigate', 'instruction': 'Walk forward to the red box and stop in front of it.'})
        try:
            async with asyncio.timeout(15):
                async with client.get('http://127.0.0.1:8766/events') as stream:
                    async for line in stream.content:
                        if line.startswith(b'data: '):
                            status = json.loads(line[6:])
                            if status['state'] == 'NAVIGATING' and status['requested_cmd'][0] >= .2:
                                break
            force = await request('/force', {'xyz': [-80, 0, 0], 'duration_s': 2})
            async with asyncio.timeout(5):
                async with client.get('http://127.0.0.1:8766/events') as stream:
                    async for line in stream.content:
                        if line.startswith(b'data: '):
                            status = json.loads(line[6:])
                            if status['state'] == 'HUMAN_PAUSED':
                                break
            assert status['reason'] == 'force_brake' and status['requested_cmd'] == [0, 0, 0]
            events.append({'case': 'direct_force_brake', 'sim_latency_s': status['sim_time_s'] - force['sim_time_s'], 'control': status})
            await request('/monitor/action', {'monitor_id': owner, 'expected_generation': status['generation'], 'action': 'complete', 'reason': 'fake model finished'}, 409)
        finally:
            running = False
            await beat
            await request('/force', {'xyz': [0, 0, 0], 'duration_s': 0})
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({'result': 'PASS', 'scope': 'Real Python service, old policy, no model decisions', 'events': events}, indent=2))
    print(str(OUT))

asyncio.run(main())
