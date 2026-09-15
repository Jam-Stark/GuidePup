#!/usr/bin/env python3
"""v1.4.0 candidate inbox and bounded deterministic memory search.

Does not write canonical entries, auto-resolve contradictions, commit or sync.
Exact normalized candidates deduplicate; semantic rephrases require a curator.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import tempfile
import subprocess
from uuid import uuid4
from typing import Any, Iterator

INBOX = Path('.ai/runtime/memory-inbox')
INDEX = Path('memory/_index.json')
LEVELS = ['INSPECTED','STATIC_PASS','TEST_PASS','RUNTIME_PASS','EXPERIMENT_PASS','HARDWARE_PASS','INFERENCE']


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def save(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            json.dump(data, out, ensure_ascii=False, indent=2, sort_keys=True)
            out.write('\n'); out.flush(); os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def load(path: Path) -> dict[str, Any]:
    d = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(d, dict): raise ValueError(f'Expected object: {path}')
    return d


@contextmanager
def lock() -> Iterator[None]:
    INBOX.mkdir(parents=True, exist_ok=True)
    with (INBOX / '.lock').open('a') as out:
        fcntl.flock(out, fcntl.LOCK_EX); yield


def candidate_path(cid: str) -> Path:
    if not re.fullmatch(r'mem-[A-Za-z0-9-]{1,80}', cid):
        raise ValueError('Invalid candidate id.')
    return INBOX / (cid + '.json')


def norm(text: str) -> str:
    return ' '.join(text.split()).strip()


def add(a: argparse.Namespace) -> int:
    if not all(norm(v) for v in [a.title, a.scope, a.source, a.target_route, a.body]):
        raise ValueError('title/scope/source/target-route/body must be nonempty.')
    route = Path(a.target_route)
    if route.is_absolute() or '..' in route.parts:
        raise ValueError('target-route must be a project-relative route without ..')
    if a.kind != 'hypothesis' and a.evidence == 'INFERENCE':
        raise ValueError('Inference must remain a hypothesis candidate, not a validated fact.')
    key = [norm(a.scope), route.as_posix(), a.kind, a.action, norm(a.body)]
    evidence = {'source': a.source, 'level': a.evidence}
    with lock():
        path = None
        for existing in sorted(INBOX.glob('*.json')):
            d = load(existing)
            if d.get('dedupe_key') == key:
                path = existing
                break
        if path is not None:
            if evidence not in d['provenance']:
                d['provenance'].append(evidence); d['updated_at'] = now(); save(path, d)
        else:
            cid = 'mem-' + uuid4().hex
            path = candidate_path(cid)
            save(path, {'id': cid, 'created_at': now(), 'title': a.title, 'scope': a.scope,
                        'kind': a.kind, 'status': 'candidate', 'source': a.source,
                        'evidence': a.evidence, 'provenance': [evidence], 'dedupe_key': key,
                        'suggested_action': a.action, 'target_route': route.as_posix(),
                        'body': a.body, 'conflicts_with': a.conflicts_with})
    print(path); return 0


def list_candidates(_: argparse.Namespace) -> int:
    for p in sorted(INBOX.glob('*.json')):
        d = load(p)
        print(json.dumps({k:d.get(k) for k in ['id','status','title','kind','target_route']}, ensure_ascii=False))
    return 0


def show(a: argparse.Namespace) -> int:
    print(candidate_path(a.candidate).read_text(encoding='utf-8')); return 0


def mark(a: argparse.Namespace) -> int:
    if not norm(a.resolution): raise ValueError('Resolution is required.')
    p = candidate_path(a.candidate)
    with lock():
        d = load(p)
        d.update(status=a.status, resolution=a.resolution, resolved_at=now()); save(p, d)
    print(p); return 0


def field(text: str, name: str, default: str = '') -> str:
    m = re.search(r'^' + re.escape(name) + r':\s*(.*)$', text, re.M)
    return m.group(1).strip().strip('\"\'') if m else default


def entries() -> list[dict[str, Any]]:
    out = []
    for p in sorted(Path('memory').rglob('description.md')):
        if p.is_symlink(): raise ValueError(f'Memory entry symlink is not indexed: {p}')
        text = p.read_text(encoding='utf-8')
        out.append({'path': p.as_posix(), 'route': p.parent.as_posix(),
                    'title': field(text, 'name', p.parent.name), 'status': field(text, 'status', 'active'),
                    'scope': field(text, 'scope'), 'read_when': field(text, 'read_when')})
    return out


def reindex(_: argparse.Namespace) -> int:
    data = {'schema_version': 2, 'entries': entries()}
    if INDEX.exists() and load(INDEX) == data:
        print(f'UNCHANGED {INDEX}'); return 0
    save(INDEX, data); print(INDEX); return 0


def search(a: argparse.Namespace) -> int:
    terms = [x.casefold() for x in re.split(r'\s+', a.query.strip()) if x]
    if not terms: raise ValueError('Query must not be blank.')
    hits = []
    for d in entries():  # fresh source-derived metadata; a stale on-disk index is not authoritative
        if not a.include_inactive and d['status'] in ('retired','superseded'): continue
        text = Path(d['path']).read_text(encoding='utf-8')
        title = (d['title'] + ' ' + d['path'] + ' ' + d['scope'] + ' ' + d['read_when']).casefold()
        body = text.casefold()
        score = sum((5 if term in title else 0) + (1 if term in body else 0) for term in terms)
        if not score: continue
        lines = [line.strip() for line in text.splitlines() if any(t in line.casefold() for t in terms)]
        hits.append((score, {**d, 'snippet': ' '.join(lines[:2])[:320]}))
    for score,d in sorted(hits, key=lambda x: (-x[0], x[1]['path']))[:a.limit]:
        print(json.dumps({'score': score, **d}, ensure_ascii=False))
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project-root', type=Path)
    sub = p.add_subparsers(dest='action', required=True)
    a = sub.add_parser('candidate-add'); a.set_defaults(func=add)
    for k in ['title','scope','source','target-route','body']: a.add_argument('--' + k, required=True)
    a.add_argument('--evidence', required=True, choices=LEVELS)
    a.add_argument('--kind', choices=['fact','decision','procedure','hypothesis'], default='fact')
    a.add_argument('--action', choices=['create','append','move','split','merge','supersede','retire'], required=True)
    a.add_argument('--conflicts-with', action='append', default=[])
    a = sub.add_parser('candidate-list'); a.set_defaults(func=list_candidates)
    a = sub.add_parser('candidate-show'); a.set_defaults(func=show); a.add_argument('--candidate', required=True)
    a = sub.add_parser('candidate-mark'); a.set_defaults(func=mark); a.add_argument('--candidate', required=True)
    a.add_argument('--status', choices=['resolved','rejected','deferred'], required=True); a.add_argument('--resolution', required=True)
    a = sub.add_parser('reindex'); a.set_defaults(func=reindex)
    a = sub.add_parser('search'); a.set_defaults(func=search); a.add_argument('--query', required=True)
    a.add_argument('--limit', type=int, choices=[1,2], default=2); a.add_argument('--include-inactive', action='store_true')
    return p

if __name__ == '__main__':
    import sys
    try:
        args = parser().parse_args()
        if args.project_root is not None:
            root = args.project_root.expanduser().resolve()
        else:
            result = subprocess.run(['git','rev-parse','--show-toplevel'], capture_output=True, text=True)
            if result.returncode: raise ValueError('Run in a repository or supply --project-root.')
            root = Path(result.stdout.strip()).resolve()
        os.chdir(root)
        raise SystemExit(args.func(args))
    except (OSError, ValueError) as exc:
        print(f'memory_curator: {exc}', file=sys.stderr); raise SystemExit(2)
