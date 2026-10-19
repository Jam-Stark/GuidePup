"""Explicit Pi/Qwen binding for the shared robot action contract; no fallback."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import time

from .planner_contract import validate_decision

ROOT = Path(__file__).resolve().parents[2]


class PiPlanner:
    def __init__(self, config, output):
        if config['backend'] != 'pi_tools':
            raise ValueError('PiPlanner requires explicit backend=pi_tools')
        self.config, self.output = dict(config), Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.sequence = 0

    def decide(self, context, images):
        self.sequence += 1
        out = self.output/f'decision_{self.sequence:03d}'
        out.mkdir()
        archived = []
        for i, (label, path) in enumerate(images):
            target = out/f'image_{i:02d}.jpg'
            target.write_bytes(Path(path).read_bytes())
            archived.append(dict(label=label, path=str(target.resolve())))
        request = dict(context=context, images=archived, timeout_s=self.config['decision_timeout_s'],
            events_path=str((out/'events.jsonl').resolve()), response_path=str((out/'response.json').resolve()),
            receipt_path=str((out/'receipt.json').resolve()))
        (out/'request.json').write_text(json.dumps(request, indent=2)+'\n')
        env = os.environ.copy()
        for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
            env.pop(key, None)
        env.update(NO_PROXY='*', no_proxy='*')
        command = [str(ROOT/'.local/envs/node-v22.23.2-linux-x64/bin/node'),
            str(ROOT/'prototype/pi/dist/rgb-planner.js'), str((out/'request.json').resolve())]
        started = time.monotonic()
        with (out/'stdout.log').open('w') as stdout, (out/'stderr.log').open('w') as stderr:
            completed = subprocess.run(command, env=env, stdout=stdout, stderr=stderr,
                timeout=self.config['decision_timeout_s']+5)
        if completed.returncode != 0:
            raise RuntimeError(f'Pi robot-action decision failed (exit {completed.returncode}); see {out}')
        result = validate_decision(json.loads((out/'response.json').read_text()))
        receipt = json.loads((out/'receipt.json').read_text())
        receipt.update(elapsed_ms=(time.monotonic()-started)*1000, result=result)
        (out/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
        return result
