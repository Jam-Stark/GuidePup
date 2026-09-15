#!/usr/bin/env python3
"""Offline v1.4.0 project-config audit. No inference calls; NOT an effective-runtime proof."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import tomllib


def audit(root: Path, schema: Path | None = None) -> dict:
    issues = []; roles = []
    cfg_path = root / '.codex/config.toml'
    cfg = tomllib.loads(cfg_path.read_text(encoding='utf-8'))
    for key in ('model','model_reasoning_effort'):
        if key in cfg: issues.append(f'Main override present: {key}; App/user ownership expected.')
    if cfg.get('agents', {}).get('default_subagent_reasoning_effort') not in ('low','medium','high'):
        issues.append('Default subagent effort missing or above high.')
    if cfg.get('agents', {}).get('default_subagent_model') != 'gpt-6-astra':
        issues.append('Default fallback must be Astra unless its inherited context is independently constrained.')
    timeout = cfg.get('background_terminal_max_timeout')
    if not isinstance(timeout, int) or timeout < 36_000_000:
        issues.append('Long empty-write wait is not configured for at least ten hours.')
    if not (root / '.codex/MODEL_TEAM_VERSION').is_file():
        issues.append('Missing explicit model-team activation marker.')
    hook_path = root / '.codex/hooks.json'
    if not hook_path.is_file() or 'codex_team_hook.py' not in hook_path.read_text(encoding='utf-8'):
        issues.append('Missing project spawn/continuity hook registration; runtime hook trust still needs local verification.')
    names = set()
    for p in sorted((root / '.codex/agents').glob('*.toml')):
        d = tomllib.loads(p.read_text(encoding='utf-8'))
        name = d.get('name')
        if not name or name in names: issues.append(f'{p.name}: missing/duplicate role name.')
        names.add(name)
        for k in ('description','developer_instructions','model','model_reasoning_effort','sandbox_mode',
                  'model_context_window','model_auto_compact_token_limit','model_auto_compact_token_limit_scope'):
            if k not in d: issues.append(f'{p.name}: missing explicit {k}.')
        w = d.get('model_context_window', 0); c = d.get('model_auto_compact_token_limit', 0)
        if not isinstance(w, int) or not isinstance(c, int) or not (0 < c < w <= 262144):
            issues.append(f'{p.name}: invalid/unbounded child context {w}/{c}.')
        if str(d.get('model','')).startswith('gpt-5.6') and (not isinstance(w, int) or not isinstance(c, int) or w > 196608 or c > 163840):
            issues.append(f'{p.name}: 5.6 budget exceeds the conservative v1.4.0 profile.')
        if d.get('model_auto_compact_token_limit_scope') != 'total': issues.append(f'{p.name}: compact scope must be total.')
        if d.get('model_reasoning_effort') not in ('low','medium','high'): issues.append(f'{p.name}: effort not authorized.')
        roles.append({k:d.get(k) for k in ('name','model','model_reasoning_effort','model_context_window','model_auto_compact_token_limit')})
    for builtin in ('default','worker','explorer'):
        if builtin not in names: issues.append(f'Missing budgeted built-in override: {builtin}.')
    if schema:
        s = json.loads(schema.read_text(encoding='utf-8'))
        if not isinstance(s, dict): raise ValueError('Schema must be a JSON object.')
        properties = s.get('properties', {})
        for key in ('background_terminal_max_timeout','model_context_window','model_auto_compact_token_limit','model_auto_compact_token_limit_scope'):
            if key not in properties:
                issues.append(f'Provided local schema does not declare {key}; do not assume support.')
    return {'project_config': 'STATIC_PASS' if not issues else 'FAIL', 'issues': issues, 'roles': roles,
            'native_tool_schema': 'UNVERIFIED', 'effective_child_config': 'UNVERIFIED',
            'ten_hour_wait': 'NOT_RUN', 'idle_session_auto_wake': 'UNVERIFIED',
            'billing_hard_cap': 'NOT_PROVIDED',
            'note': 'Local user/admin/CLI overrides and first-request context inheritance still need runtime verification.'}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, default=Path('.'))
    p.add_argument('--schema', type=Path); p.add_argument('--probe-version', action='store_true'); a = p.parse_args()
    try:
        result = audit(a.root.resolve(), a.schema)
        if a.probe_version:
            v = subprocess.run(['codex','--version'], capture_output=True, text=True, timeout=5)
            result['codex_version_probe'] = {'exit_code': v.returncode, 'stdout': v.stdout.strip(), 'stderr': v.stderr.strip()}
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        result = {'project_config':'FAIL','issues':[str(exc)]}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['project_config'] == 'STATIC_PASS' else 2

if __name__ == '__main__':
    raise SystemExit(main())
