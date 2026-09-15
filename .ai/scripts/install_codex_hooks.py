#!/usr/bin/env python3
"""Register this worktree's hooks in Codex's user hook source (0.153.0).

The project JSON remains authoritative. Each handler runs only in this worktree.
Unrelated user hooks are preserved. Hook trust is left to Codex's normal UI.
"""
from pathlib import Path
import copy
import json
import shlex
import subprocess


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    actual = subprocess.check_output(
        ['git', '-C', str(root), 'rev-parse', '--show-toplevel'], text=True
    ).strip()
    if Path(actual).resolve() != root:
        raise ValueError('Installer must belong to the target worktree root.')
    source = json.loads((root / '.codex/hooks.json').read_text())
    target = Path.home() / '.codex/hooks.json'
    data = json.loads(target.read_text()) if target.exists() else {'hooks': {}}
    marker = f'Jam worktree {root}: '
    for event, groups in list(data['hooks'].items()):
        kept = []
        for group in groups:
            group = copy.deepcopy(group)
            group['hooks'] = [h for h in group['hooks']
                              if not h.get('statusMessage', '').startswith(marker)]
            if group['hooks']:
                kept.append(group)
        data['hooks'][event] = kept
    for event, groups in source['hooks'].items():
        for group in groups:
            group = copy.deepcopy(group)
            for handler in group['hooks']:
                if handler['type'] != 'command':
                    raise ValueError('Worktree registration supports command handlers only.')
                handler['command'] = (
                    'if [ "$(git rev-parse --show-toplevel 2>/dev/null)" = '
                    + shlex.quote(str(root)) + ' ]; then '
                    + handler['command'] + '; fi'
                )
                handler['statusMessage'] = marker + event
            data['hooks'].setdefault(event, []).append(group)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    print(f'Registered {sum(len(g["hooks"]) for gs in source["hooks"].values() for g in gs)} handlers: {target}')
    print('Review and trust the listed handlers through Codex /hooks before execution.')


if __name__ == '__main__':
    main()
