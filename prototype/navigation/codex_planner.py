"""Codex CLI as a vision decision backend, without coding/execution tools.

Uses the existing ChatGPT login, per-call model settings and the existing Codex
proxy. Does not edit CODEX_HOME, auth, proxy service, Qwen profiles or global config.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from .planner_contract import CONTRACT, SCHEMA, SYSTEM, validate_decision


class CodexPlanner:
    def __init__(self, config, output):
        self.config, self.output = dict(config), Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.sequence = 0
        self.process = None
        self.closed = False
        if config['backend'] != 'codex_exec' or config['model'] not in ('gpt-6.1-sol', 'gpt-6-astra'):
            raise ValueError('The navigation deployment supports the explicitly selected Sol or Astra model')
        if config['service_tier'] not in ('default', 'fast'):
            raise ValueError('Select an explicit default or fast service tier; no automatic tier selection')

    def close(self):
        """Cancel only this runtime's in-flight CLI child when the task ends."""
        self.closed = True
        process = self.process
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def decide(self, context, images):
        self.sequence += 1
        out = self.output/f'decision_{self.sequence:03d}'
        out.mkdir()
        (out/'schema.json').write_text(json.dumps(SCHEMA, indent=2)+'\n')
        prompt = (SYSTEM+'\nAVAILABLE ROBOT ACTIONS:\n'+json.dumps(CONTRACT['actions'])+
            '\nINPUT:\n'+json.dumps(context, ensure_ascii=False, allow_nan=False))
        prompt += '\nIMAGES IN ORDER: '+json.dumps([label for label, _ in images])
        (out/'prompt.txt').write_text(prompt)
        fast_mode = self.config['service_tier'] == 'fast'
        with tempfile.TemporaryDirectory(prefix='guidepup-codex-planner-') as workdir:
            command = ['codex', 'exec', '-', '--ignore-user-config', '--ignore-rules', '--ephemeral',
                '--skip-git-repo-check', '--sandbox', 'read-only', '-C', workdir,
                '--model', self.config['model'], '-c', 'model_reasoning_effort='+json.dumps(self.config['reasoning_effort']),
                '-c', 'service_tier='+json.dumps(self.config['service_tier']),
                '-c', 'developer_instructions='+json.dumps(SYSTEM), '-c', 'project_doc_max_bytes=0',
                '--strict-config', '--enable' if fast_mode else '--disable', 'fast_mode',
                '--json', '--output-schema', str((out/'schema.json').resolve()),
                '--output-last-message', str((out/'response.json').resolve())]
            for feature in ('shell_tool', 'unified_exec', 'multi_agent', 'apps', 'plugins',
                            'browser_use', 'browser_use_external', 'computer_use', 'image_generation',
                            'hooks', 'goals', 'shell_snapshot', 'remote_plugin', 'skill_search',
                            'workspace_dependencies', 'view_image'):
                command += ['--disable', feature]
            for index, (_, path) in enumerate(images):
                source = Path(path).resolve()
                # Archive the exact image bytes; do not rescale or annotate model inputs.
                archived = out/f'image_{index:02d}{source.suffix}'
                archived.write_bytes(source.read_bytes())
                command += ['--image='+str(archived.resolve())]
            env = os.environ.copy()
            # Scoped to this Codex child. No server-wide proxy is enabled.
            if self.config['codex_proxy']:
                for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
                    env[key] = self.config['codex_proxy']
                env.update(NO_PROXY='127.0.0.1,localhost', no_proxy='127.0.0.1,localhost')
            started = time.monotonic()
            receipt = dict(model=self.config['model'], reasoning_effort=self.config['reasoning_effort'],
                service_tier=self.config['service_tier'], fast_mode=fast_mode, ultrafast_requested=False,
                command=command,
                decision_timeout_s=None,
                scope='Read-only vision proposal; no robot execution tools', started_epoch=time.time())
            (out/'request.json').write_text(json.dumps(receipt, indent=2)+'\n')
            with (out/'events.jsonl').open('w') as events, (out/'stderr.log').open('w') as errors:
                if self.closed:
                    raise RuntimeError('Planner has been closed by the task runtime')
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=events, stderr=errors,
                    text=True, env=env, start_new_session=True)
                self.process = process
                try:
                    if self.closed:
                        self.close()
                    process.communicate(prompt)
                finally:
                    self.process = None
            records = [json.loads(line) for line in (out/'events.jsonl').read_text().splitlines() if line]
            completed = any(r.get('type') == 'turn.completed' for r in records)
            items = [r['item'] for r in records if r.get('type') in ('item.started', 'item.completed')]
            diagnostics = [r for r in items if r['type'] == 'error']
            if diagnostics:
                raise RuntimeError(f'Codex reported a configuration/runtime error: {diagnostics}; see {out}')
            forbidden = [r for r in items if r['type'] not in ('agent_message', 'reasoning')]
            if forbidden:
                raise RuntimeError(f'Planner attempted an execution tool; no proposal accepted: {out}')
            if process.returncode != 0 or not completed:
                raise RuntimeError(f'Codex decision failed (exit {process.returncode}); see {out}')
            result = json.loads((out/'response.json').read_text())
            validate_decision(result)
            receipt.update(elapsed_ms=(time.monotonic()-started)*1000, exit_code=process.returncode,
                           result=result, usage=[r['usage'] for r in records if r.get('type') == 'turn.completed'])
            (out/'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
            return result
