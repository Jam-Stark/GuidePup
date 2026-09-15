#!/usr/bin/env python3
"""Optional, lazily activated coordination ledger for ForceControl v1.3."""

from __future__ import annotations

import argparse
import json
import re
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError as exc:
    raise SystemExit("Python 3.11+ is required") from exc

CONFIG = Path('.ai/team-state.toml')
DEFAULT_DIR = Path('.ai/runtime/team')
DEFAULT_DB = DEFAULT_DIR / 'team-state.sqlite3'
DEFAULT_SNAPSHOT = DEFAULT_DIR / 'team-snapshot.json'
DEFAULT_MARKER = DEFAULT_DIR / 'coordination.json'
TASK_RE = re.compile(r'^[a-z0-9]+(?:_[a-z0-9]+)*$')


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def split_values(values: list[str] | None) -> list[str]:
    out: list[str] = []
    for value in values or []:
        out.extend(item.strip() for item in value.split(',') if item.strip())
    return sorted(set(out))


def load_config(path: Path = CONFIG) -> dict:
    if not path.is_file():
        return {}
    with path.open('rb') as handle:
        return tomllib.load(handle)


def paths(cfg: dict) -> tuple[Path, Path, Path]:
    return (
        Path(cfg.get('database', DEFAULT_DB)),
        Path(cfg.get('snapshot', DEFAULT_SNAPSHOT)),
        Path(cfg.get('marker', DEFAULT_MARKER)),
    )


def activation(marker: Path) -> dict | None:
    if not marker.is_file():
        return None
    try:
        data = json.loads(marker.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {'mode': 'invalid', 'reason': 'unreadable marker'}
    return data if data.get('active') else None


def connect(db: Path) -> sqlite3.Connection:
    db.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(db)
    con.row_factory = sqlite3.Row
    con.executescript("""
    PRAGMA journal_mode=WAL;
    CREATE TABLE IF NOT EXISTS tasks (
      task_name TEXT PRIMARY KEY,
      canonical_path TEXT,
      agent_id TEXT,
      role TEXT NOT NULL,
      revision TEXT,
      outcome TEXT,
      state TEXT NOT NULL DEFAULT 'declared',
      dependencies TEXT NOT NULL DEFAULT '[]',
      consumers TEXT NOT NULL DEFAULT '[]',
      read_set TEXT NOT NULL DEFAULT '[]',
      write_set TEXT NOT NULL DEFAULT '[]',
      resources TEXT NOT NULL DEFAULT '[]',
      acceptance TEXT,
      evidence TEXT,
      non_goals TEXT,
      latest_verdict TEXT,
      artifacts TEXT NOT NULL DEFAULT '[]',
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );
    CREATE TABLE IF NOT EXISTS leases (
      resource TEXT PRIMARY KEY,
      task_name TEXT NOT NULL,
      kind TEXT NOT NULL,
      acquired_at TEXT NOT NULL,
      released_at TEXT
    );
    CREATE TABLE IF NOT EXISTS candidates (
      revision TEXT PRIMARY KEY,
      source_commit TEXT,
      paths TEXT NOT NULL,
      contracts TEXT NOT NULL,
      topology TEXT NOT NULL,
      status TEXT NOT NULL DEFAULT 'frozen',
      created_at TEXT NOT NULL
    );
    CREATE TABLE IF NOT EXISTS verdicts (
      verdict_id TEXT PRIMARY KEY,
      task_name TEXT NOT NULL,
      revision TEXT NOT NULL,
      reviewer_role TEXT NOT NULL,
      status TEXT NOT NULL,
      paths TEXT NOT NULL,
      contracts TEXT NOT NULL,
      topology TEXT NOT NULL,
      evidence TEXT NOT NULL,
      validity TEXT NOT NULL DEFAULT 'VALID',
      reason TEXT,
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );
    CREATE TABLE IF NOT EXISTS messages (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      timestamp TEXT NOT NULL,
      sender TEXT NOT NULL,
      receiver TEXT NOT NULL,
      message_type TEXT NOT NULL,
      revision TEXT,
      summary TEXT,
      material INTEGER NOT NULL DEFAULT 0,
      source_call_id TEXT
    );
    """)
    return con


def decode_fields(item: dict) -> dict:
    for key in ('dependencies','consumers','read_set','write_set','resources','artifacts','paths','contracts','topology','evidence'):
        if key in item and isinstance(item[key], str):
            try:
                item[key] = json.loads(item[key])
            except json.JSONDecodeError:
                pass
    return item


def snapshot(con: sqlite3.Connection, path: Path, active: dict | None) -> None:
    data: dict[str, object] = {'activation': active or {'active': False}}
    for table in ('tasks','leases','candidates','verdicts','messages'):
        data[table] = [decode_fields(dict(row)) for row in con.execute(f'SELECT * FROM {table} ORDER BY rowid')]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def role_group(role: str | None) -> str:
    value = (role or '').lower()
    if 'research' in value or value in {'explore','librarian','context_researcher','deep_researcher'}:
        return 'researcher'
    if 'planner' in value or value in {'prometheus','metis','scope_planner'}:
        return 'planner'
    if 'review' in value or value in {'oracle','momus'}:
        return 'reviewer'
    if 'runtime' in value or 'qa' in value or 'runner' in value:
        return 'runtime_qa'
    if 'memory' in value or 'curator' in value:
        return 'memory_curator'
    return 'writer'


def required_fields(cfg: dict, role: str | None) -> list[str]:
    group = role_group(role)
    configured = cfg.get('roles', {}).get(group, {}).get('required')
    if isinstance(configured, list):
        return [str(x) for x in configured]
    defaults = {
        'researcher': ['outcome'],
        'planner': ['outcome'],
        'writer': ['outcome','revision','write_set','acceptance'],
        'reviewer': ['outcome','revision','read_set','acceptance'],
        'runtime_qa': ['outcome','revision','acceptance'],
        'memory_curator': ['outcome','read_set'],
    }
    return defaults[group]


def validate_row(cfg: dict, row: sqlite3.Row | None) -> list[str]:
    if row is None:
        return ['task contract not found']
    missing: list[str] = []
    for key in required_fields(cfg, row['role']):
        value = row[key]
        if key in {'read_set','write_set','resources'}:
            try:
                value = json.loads(value)
            except Exception:
                value = []
        if not value:
            missing.append(key)
    return missing


def require_active(cfg: dict) -> tuple[Path, Path, Path, dict]:
    db, snap, marker = paths(cfg)
    active = activation(marker)
    if not active:
        raise SystemExit('coordination state is inactive; activate it explicitly first')
    return db, snap, marker, active


def ensure_task_name(name: str) -> None:
    if not TASK_RE.fullmatch(name):
        raise SystemExit('task_name must use lowercase letters, digits and underscores')


def cmd_status(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, marker = paths(cfg)
    active = activation(marker)
    data = {'active': bool(active), 'mode': active.get('mode') if active else 'inactive', 'reason': active.get('reason') if active else None, 'database_exists': db.exists(), 'snapshot_exists': snap.exists()}
    if args.json:
        print(json.dumps(data, ensure_ascii=False))
    else:
        print(('ACTIVE ' + str(data['mode'])) if data['active'] else 'INACTIVE')
    return 0


def cmd_activate(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, marker = paths(cfg)
    existing = activation(marker)
    if existing and not args.force:
        raise SystemExit(f"coordination already active in {existing.get('mode')} mode")
    allowed = cfg.get('activation', {}).get('allowed_modes', ['adaptive','strict'])
    if args.mode not in allowed:
        raise SystemExit(f'unsupported mode: {args.mode}')
    marker.parent.mkdir(parents=True, exist_ok=True)
    active = {'active': True, 'mode': args.mode, 'reason': args.reason, 'activated_at': now()}
    marker.write_text(json.dumps(active, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    con = connect(db)
    snapshot(con, snap, active)
    print(f'ACTIVE {args.mode}: {args.reason}')
    return 0


def cmd_deactivate(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, marker = paths(cfg)
    active = activation(marker)
    if not active:
        print('ALREADY INACTIVE')
        return 0
    if db.exists():
        con = connect(db)
        open_leases = con.execute('SELECT COUNT(*) FROM leases WHERE released_at IS NULL').fetchone()[0]
        active_tasks = con.execute("SELECT COUNT(*) FROM tasks WHERE state IN ('declared','spawned','running','waiting')").fetchone()[0]
        if (open_leases or active_tasks) and not args.force:
            raise SystemExit(f'cannot deactivate with {active_tasks} active tracked task(s) and {open_leases} open lease(s); pass --force only after Main adjudicates them')
        snapshot(con, snap, active)
    if args.archive:
        archive = marker.parent / f"coordination-{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
        archive.write_text(json.dumps({**active, 'active': False, 'deactivated_at': now()}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'ARCHIVED {archive}')
    marker.unlink(missing_ok=True)
    print('INACTIVE')
    return 0


def cmd_hook_check(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, _snap, marker = paths(cfg)
    active = activation(marker)
    if not active:
        print(json.dumps({'allow': True, 'managed': False, 'reason': 'coordination inactive'}))
        return 0
    if not args.task_name or not TASK_RE.fullmatch(args.task_name):
        if active.get('mode') == 'strict' and role_group(args.role) in set(cfg.get('activation', {}).get('strict_role_groups', [])):
            print(json.dumps({'allow': False, 'managed': True, 'reason': 'strict coordination requires a semantic task_name'}))
            return 2
        print(json.dumps({'allow': True, 'managed': True, 'reason': 'unmanaged spawn allowed'}))
        return 0
    con = connect(db)
    row = con.execute('SELECT * FROM tasks WHERE task_name=?', (args.task_name,)).fetchone()
    if row is None:
        strict_groups = set(cfg.get('activation', {}).get('strict_role_groups', ['writer','reviewer','runtime_qa']))
        if active.get('mode') == 'strict' and role_group(args.role) in strict_groups:
            print(json.dumps({'allow': False, 'managed': True, 'reason': f'contract required for {args.task_name}'}))
            return 2
        print(json.dumps({'allow': True, 'managed': True, 'reason': 'no contract registered; adaptive spawn allowed'}))
        return 0
    missing = validate_row(cfg, row)
    if missing:
        print(json.dumps({'allow': False, 'managed': True, 'reason': f"invalid contract {args.task_name}: {', '.join(missing)}"}))
        return 2
    print(json.dumps({'allow': True, 'managed': True, 'reason': f'contract validated: {args.task_name}'}))
    return 0


def cmd_contract_create(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    ensure_task_name(args.task_name)
    con = connect(db)
    stamp = now()
    payload = (
        args.task_name, args.canonical_path, None, args.role, args.revision, args.outcome,
        'declared', json.dumps(split_values(args.dependency)), json.dumps(split_values(args.consumer)),
        json.dumps(split_values(args.read_set)), json.dumps(split_values(args.write_set)),
        json.dumps(split_values(args.resource)), args.acceptance, args.evidence, args.non_goals,
        None, '[]', stamp, stamp,
    )
    con.execute("""INSERT INTO tasks VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
      ON CONFLICT(task_name) DO UPDATE SET
        canonical_path=excluded.canonical_path, role=excluded.role, revision=excluded.revision,
        outcome=excluded.outcome, dependencies=excluded.dependencies, consumers=excluded.consumers,
        read_set=excluded.read_set, write_set=excluded.write_set, resources=excluded.resources,
        acceptance=excluded.acceptance, evidence=excluded.evidence, non_goals=excluded.non_goals,
        updated_at=excluded.updated_at""", payload)
    con.commit(); snapshot(con, snap, active)
    print(f'CONTRACT SAVED {args.task_name}')
    return 0


def cmd_contract_validate(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, _snap, _marker, _active = require_active(cfg)
    con = connect(db)
    row = con.execute('SELECT * FROM tasks WHERE task_name=?', (args.task_name,)).fetchone()
    missing = validate_row(cfg, row)
    if missing:
        print(f'INVALID {args.task_name}: {", ".join(missing)}', file=sys.stderr)
        return 2
    print(f'VALID {args.task_name}')
    return 0


def cmd_task_state(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    con = connect(db)
    cur = con.execute('UPDATE tasks SET state=?, agent_id=COALESCE(?,agent_id), canonical_path=COALESCE(?,canonical_path), updated_at=? WHERE task_name=?', (args.state,args.agent_id,args.canonical_path,now(),args.task_name))
    if cur.rowcount != 1:
        raise SystemExit(f'unknown task: {args.task_name}')
    con.commit(); snapshot(con, snap, active)
    print(f'STATE {args.task_name}={args.state}')
    return 0


def cmd_lease_acquire(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    resources = split_values(args.resource)
    if not resources:
        raise SystemExit('at least one actual exclusive resource is required')
    con = connect(db)
    if con.execute('SELECT 1 FROM tasks WHERE task_name=?', (args.task_name,)).fetchone() is None:
        raise SystemExit(f'unknown task: {args.task_name}')
    for resource in resources:
        row = con.execute('SELECT * FROM leases WHERE resource=? AND released_at IS NULL', (resource,)).fetchone()
        if row and row['task_name'] != args.task_name:
            print(f'LEASE CONFLICT {resource}: owned by {row["task_name"]}', file=sys.stderr)
            return 3
    stamp = now()
    for resource in resources:
        con.execute('INSERT INTO leases(resource,task_name,kind,acquired_at,released_at) VALUES(?,?,?,?,NULL) ON CONFLICT(resource) DO UPDATE SET task_name=excluded.task_name,kind=excluded.kind,acquired_at=excluded.acquired_at,released_at=NULL', (resource,args.task_name,args.kind,stamp))
    con.commit(); snapshot(con, snap, active)
    print(f'LEASED {args.task_name}: {", ".join(resources)}')
    return 0


def cmd_lease_release(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    con = connect(db)
    con.execute('UPDATE leases SET released_at=? WHERE task_name=? AND released_at IS NULL', (now(),args.task_name))
    con.commit(); snapshot(con, snap, active)
    print(f'RELEASED {args.task_name}')
    return 0


def cmd_freeze(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    paths_bound = split_values(args.path)
    if not paths_bound:
        raise SystemExit('formal freeze requires at least one bound path')
    con = connect(db)
    con.execute('INSERT OR REPLACE INTO candidates VALUES(?,?,?,?,?,?,?)', (args.revision,args.source_commit,json.dumps(paths_bound),json.dumps(split_values(args.contract)),json.dumps(split_values(args.topology)),'frozen',now()))
    con.commit(); snapshot(con, snap, active)
    print(f'FROZEN {args.revision} ({args.purpose})')
    return 0


def cmd_verdict(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    con = connect(db); stamp = now()
    if con.execute('SELECT 1 FROM candidates WHERE revision=?', (args.revision,)).fetchone() is None:
        raise SystemExit(f'candidate revision is not frozen: {args.revision}')
    con.execute('INSERT OR REPLACE INTO verdicts VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)', (args.verdict_id,args.task_name,args.revision,args.reviewer_role,args.status,json.dumps(split_values(args.path)),json.dumps(split_values(args.contract)),json.dumps(split_values(args.topology)),json.dumps(split_values(args.evidence_item)),'VALID',None,stamp,stamp))
    con.execute('UPDATE tasks SET latest_verdict=?,updated_at=? WHERE task_name=?', (args.status,stamp,args.task_name))
    con.commit(); snapshot(con, snap, active)
    print(f'VERDICT {args.verdict_id}={args.status}')
    return 0


def intersects(bound: list[str], changed: list[str]) -> bool:
    return any(a == b or a.startswith(b.rstrip('/') + '/') or b.startswith(a.rstrip('/') + '/') for a in bound for b in changed)


def cmd_invalidate(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    con = connect(db)
    cp, cc, ct = split_values(args.changed_path), split_values(args.changed_contract), split_values(args.changed_topology)
    for row in con.execute('SELECT * FROM verdicts WHERE revision=?', (args.revision,)):
        bp, bc, bt = json.loads(row['paths']), json.loads(row['contracts']), json.loads(row['topology'])
        if intersects(bp, cp) or set(bc) & set(cc) or set(bt) & set(ct):
            validity, reason = 'INVALID', 'bound dependency changed'
        elif args.explicitly_disjoint:
            validity, reason = 'RETAINED', 'explicitly disjoint change'
        else:
            validity, reason = 'REVIEW_REQUIRED', 'relationship not proven disjoint'
        con.execute('UPDATE verdicts SET validity=?,reason=?,updated_at=? WHERE verdict_id=?', (validity,reason,now(),row['verdict_id']))
        print(f'{row["verdict_id"]}: {validity}')
    con.commit(); snapshot(con, snap, active)
    return 0


def cmd_message(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    db, snap, _marker, active = require_active(cfg)
    con = connect(db)
    con.execute('INSERT INTO messages(timestamp,sender,receiver,message_type,revision,summary,material,source_call_id) VALUES(?,?,?,?,?,?,?,?)', (now(),args.sender,args.receiver,args.message_type,args.revision,args.summary,int(args.material),args.source_call_id))
    con.commit(); snapshot(con, snap, active)
    print('MESSAGE RECORDED')
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=CONFIG)
    sub = p.add_subparsers(dest='cmd', required=True)
    st = sub.add_parser('status'); st.add_argument('--json', action='store_true'); st.set_defaults(func=cmd_status)
    ac = sub.add_parser('activate'); ac.add_argument('--mode', choices=['adaptive','strict'], default='adaptive'); ac.add_argument('--reason', required=True); ac.add_argument('--force', action='store_true'); ac.set_defaults(func=cmd_activate)
    de = sub.add_parser('deactivate'); de.add_argument('--archive', action='store_true'); de.add_argument('--force', action='store_true'); de.set_defaults(func=cmd_deactivate)
    hc = sub.add_parser('hook-check-spawn'); hc.add_argument('--task-name'); hc.add_argument('--role'); hc.set_defaults(func=cmd_hook_check)
    c = sub.add_parser('contract-create'); c.add_argument('--task-name', required=True); c.add_argument('--role', required=True); c.add_argument('--canonical-path'); c.add_argument('--revision'); c.add_argument('--outcome', required=True); c.add_argument('--dependency', action='append'); c.add_argument('--consumer', action='append'); c.add_argument('--read-set', action='append'); c.add_argument('--write-set', action='append'); c.add_argument('--resource', action='append'); c.add_argument('--acceptance'); c.add_argument('--evidence'); c.add_argument('--non-goals'); c.set_defaults(func=cmd_contract_create)
    cv = sub.add_parser('contract-validate'); cv.add_argument('--task-name', required=True); cv.set_defaults(func=cmd_contract_validate)
    ts = sub.add_parser('task-state'); ts.add_argument('--task-name', required=True); ts.add_argument('--state', required=True); ts.add_argument('--agent-id'); ts.add_argument('--canonical-path'); ts.set_defaults(func=cmd_task_state)
    la = sub.add_parser('lease-acquire'); la.add_argument('--task-name', required=True); la.add_argument('--resource', action='append', required=True); la.add_argument('--kind', choices=['exclusive','write_path','gpu','isaacsim','display','port','hardware','output_root'], default='exclusive'); la.set_defaults(func=cmd_lease_acquire)
    lr = sub.add_parser('lease-release'); lr.add_argument('--task-name', required=True); lr.set_defaults(func=cmd_lease_release)
    fr = sub.add_parser('freeze-create'); fr.add_argument('--revision', required=True); fr.add_argument('--purpose', choices=['formal_code_review','formal_isaaclab_review','formal_runtime_qa'], required=True); fr.add_argument('--source-commit'); fr.add_argument('--path', action='append', required=True); fr.add_argument('--contract', action='append'); fr.add_argument('--topology', action='append'); fr.set_defaults(func=cmd_freeze)
    ve = sub.add_parser('verdict-add'); ve.add_argument('--verdict-id', required=True); ve.add_argument('--task-name', required=True); ve.add_argument('--revision', required=True); ve.add_argument('--reviewer-role', required=True); ve.add_argument('--status', required=True); ve.add_argument('--path', action='append'); ve.add_argument('--contract', action='append'); ve.add_argument('--topology', action='append'); ve.add_argument('--evidence-item', action='append'); ve.set_defaults(func=cmd_verdict)
    inv = sub.add_parser('invalidate'); inv.add_argument('--revision', required=True); inv.add_argument('--changed-path', action='append'); inv.add_argument('--changed-contract', action='append'); inv.add_argument('--changed-topology', action='append'); inv.add_argument('--explicitly-disjoint', action='store_true'); inv.set_defaults(func=cmd_invalidate)
    msg = sub.add_parser('message-record'); msg.add_argument('--sender', required=True); msg.add_argument('--receiver', required=True); msg.add_argument('--message-type', choices=['PEER_FINDING','PEER_REQUEST','AUTHORITY_REQUEST'], required=True); msg.add_argument('--revision'); msg.add_argument('--summary'); msg.add_argument('--material', action='store_true'); msg.add_argument('--source-call-id'); msg.set_defaults(func=cmd_message)
    return p


def main() -> int:
    args = parser().parse_args()
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
