#!/usr/bin/env python3
"""v1.4.0 POSIX durable runner. No LLM, network, Git writes or automatic retry.

RUN_RECEIPT.json is immutable intent; STATUS.json is atomic observed state.
ETA expires a wait, not the job. Evaluator PASS applies only to its named claim.
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
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterator

VERSION = '1.4.0'
TERMINAL = {'PROCESS_COMPLETED', 'PROCESS_FAILED', 'CANCELLED', 'LAUNCH_FAILED'}

def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')

def load(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'Expected JSON object: {path}')
    return data

def save(path: Path, obj: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            json.dump(obj, out, ensure_ascii=False, indent=2, sort_keys=True)
            out.write('\n'); out.flush(); os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)

def report(obj: dict[str, Any]) -> int:
    print(json.dumps(obj, ensure_ascii=False, sort_keys=True))
    return 0

def project_root(raw: str | None = None) -> Path:
    if raw:
        root = Path(raw).expanduser().resolve()
    else:
        result = subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True, text=True)
        if result.returncode:
            raise ValueError('Run inside a repository or supply --project-root.')
        root = Path(result.stdout.strip()).resolve()
    if not root.is_dir():
        raise ValueError(f'No project directory: {root}')
    return root

def anchored(root: Path, raw: str) -> Path:
    p = Path(raw).expanduser()
    return (p if p.is_absolute() else root / p).resolve()

def checked_receipt(raw: str | Path) -> tuple[Path, dict[str, Any]]:
    p = Path(raw).expanduser().resolve()
    d = load(p)
    if d.get('schema_version') != 2 or d.get('supervisor_version') != VERSION:
        raise ValueError('Not a v1.4 receipt. Preserve/finish legacy runs with their original supervisor.')
    expected = Path(d['project_root']) / '.ai/runtime/runs' / d['name'] / 'RUN_RECEIPT.json'
    if p != expected.resolve():
        raise ValueError('Receipt moved from its declared run directory; manual migration required.')
    return p, d

@contextmanager
def locked(path: Path) -> Iterator[None]:
    with path.open('a', encoding='utf-8') as out:
        fcntl.flock(out, fcntl.LOCK_EX)
        yield

def state_path(p: Path) -> Path:
    return p.with_name('STATUS.json')

def event(p: Path, receipt: dict[str, Any], state: dict[str, Any], kind: str) -> None:
    event_id = f"{receipt['name']}--{kind}"
    root = Path(receipt['project_root']) / '.ai/runtime/run-events'
    if (root / 'archive' / f'{event_id}.json').exists():
        return
    save(root / 'pending' / f'{event_id}.json', {
        'id': event_id, 'kind': kind, 'created_at': now(),
        'receipt': str(p), 'status_path': str(state_path(p)),
        'owner_session': receipt.get('owner_session', ''),
        'process_state': state['process_state'],
        'acceptance_state': state['acceptance_state'],
        'summary': f"{receipt['name']}: process={state['process_state']}; acceptance={state['acceptance_state']}",
    })

def prepare(args: argparse.Namespace) -> int:
    root = project_root(args.project_root)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', args.name):
        raise ValueError('Name must be 1..80 safe ASCII filename characters.')
    if args.expected_seconds <= 0:
        raise ValueError('expected-seconds must be positive; estimate a meaningful decision time.')
    cwd = anchored(root, args.cwd)
    if not cwd.is_dir():
        raise ValueError(f'Invalid command cwd: {cwd}')
    if anchored(cwd, args.output).exists():
        raise ValueError('Output already exists; choose a fresh owned log path, never mix old/new runs.')
    if args.auto_eval and not args.authorize_eval:
        raise ValueError('--auto-eval requires explicit --authorize-eval.')
    if args.authorize_eval and not (args.eval_command and args.acceptance_claim):
        raise ValueError('Authorized eval requires an exact command and acceptance claim.')
    rev = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True)
    revision = args.source_revision or (rev.stdout.strip() if not rev.returncode else 'UNVERSIONED')
    run = root / '.ai/runtime/runs' / args.name
    run.mkdir(parents=True, exist_ok=False)
    p = run / 'RUN_RECEIPT.json'
    receipt = {
        'schema_version': 2, 'supervisor_version': VERSION, 'name': args.name,
        'project_root': str(root), 'cwd': str(cwd), 'source_revision': revision,
        'config_ref': args.config_ref, 'checkpoint_lineage': args.checkpoint_lineage,
        'owner_session': args.owner_session, 'command': args.command,
        'output': str(anchored(cwd, args.output)),
        'checkpoint': str(anchored(cwd, args.checkpoint)) if args.checkpoint else None,
        'resources': args.resource, 'session': args.session or 'jam-' + args.name,
        'expected_seconds': args.expected_seconds, 'eta_source': args.eta_source,
        'stop_condition': args.stop_condition,
        'eval_command': args.eval_command, 'acceptance_claim': args.acceptance_claim,
        'eval_authorized': args.authorize_eval, 'auto_eval': args.auto_eval,
        'created_at': now(),
    }
    save(p, receipt)
    save(state_path(p), {'process_state': 'DECLARED', 'acceptance_state': 'UNASSESSED', 'updated_at': now()})
    return report({'receipt': str(p), 'state': 'DECLARED'})

def launch(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    if args.backend == 'tmux' and not shutil.which('tmux'):
        raise ValueError('tmux missing; no implicit fallback. Use an explicitly accepted backend.')
    if not shutil.which('bash'):
        raise ValueError('bash required for the declared command; no shell substitution.')
    with locked(p.with_name('launch.lock')):
        s = load(state_path(p))
        if s['process_state'] != 'DECLARED':
            raise ValueError(f"Already launched or launch uncertain: {s['process_state']}; never auto-retry.")
        start = time.time()
        s.update(process_state='LAUNCHING', launched_at=now(), launched_epoch=start,
                 wait_until_epoch=start + r['expected_seconds'], backend=args.backend, updated_at=now())
        save(state_path(p), s)
        cmd = [sys.executable, str(Path(__file__).resolve()), '_run', '--receipt', str(p)]
        try:
            if args.backend == 'tmux':
                subprocess.run(['tmux', 'new-session', '-d', '-s', r['session'], shlex.join(cmd)],
                               cwd=r['cwd'], check=True, capture_output=True, text=True)
            else:
                with p.with_name('supervisor.log').open('ab') as out:
                    subprocess.Popen(cmd, cwd=r['cwd'], stdin=subprocess.DEVNULL,
                                     stdout=out, stderr=out, start_new_session=True, close_fds=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            s.update(process_state='LAUNCH_FAILED', reason=str(exc), updated_at=now())
            save(state_path(p), s); event(p, r, s, 'completion')
            raise
    return report({'receipt': str(p), 'backend': args.backend, 'session': r['session'],
                   'wait_until_epoch': s['wait_until_epoch']})

def terminate_group(proc: subprocess.Popen[Any]) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()

def run_command(command: str, cwd: str, output: Path, cancel_file: Path,
                signal_cancel: list[bool]) -> tuple[int, bool]:
    output.parent.mkdir(parents=True, exist_ok=True)
    cancelled = False
    with output.open('xb') as out:
        proc = subprocess.Popen(['bash', '-c', command], cwd=cwd, stdin=subprocess.DEVNULL,
                                stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                if signal_cancel[0] or cancel_file.exists():
                    cancelled = True; terminate_group(proc); break
                try:
                    proc.wait(timeout=1.0); break  # deterministic OS wait, not a model invocation
                except subprocess.TimeoutExpired:
                    continue
        finally:
            if proc.poll() is None:
                terminate_group(proc)
    return int(proc.returncode), cancelled

def internal_run(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    with locked(p.with_name('runner.lock')):
        s = load(state_path(p))
        if s['process_state'] != 'LAUNCHING':
            raise ValueError('Runner may only claim a LAUNCHING record once.')
        s.update(process_state='RUNNING', supervisor_pid=os.getpid(), updated_at=now())
        save(state_path(p), s)
        stop = [False]
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda _s, _f: stop.__setitem__(0, True))
        try:
            rc, cancelled = run_command(r['command'], r['cwd'], Path(r['output']),
                                        p.with_name('CANCEL_REQUEST.json'), stop)
            s.update(exit_code=rc, process_state='CANCELLED' if cancelled else
                     ('PROCESS_COMPLETED' if rc == 0 else 'PROCESS_FAILED'),
                     finished_at=now(), finished_epoch=time.time(), updated_at=now())
            cp = Path(r['checkpoint']) if r.get('checkpoint') else None
            s['checkpoint_state'] = 'NOT_DECLARED' if cp is None else ('PRESENT_NOT_VALIDATED' if cp.is_file() else 'MISSING')
            if cp is not None and not cp.is_file():
                s['acceptance_state'] = 'BLOCKED_MISSING_CHECKPOINT'
            save(state_path(p), s)
        except (OSError, ValueError) as exc:
            s.update(process_state='PROCESS_FAILED', reason=str(exc), updated_at=now())
            save(state_path(p), s)
            event(p, r, s, 'completion')
            raise
        if r['auto_eval'] and s['process_state'] == 'PROCESS_COMPLETED' and s['acceptance_state'] == 'UNASSESSED':
            evaluate(p, r, stop)
        else:
            event(p, r, s, 'completion')
    return 0

def evaluate(p: Path, r: dict[str, Any], stop: list[bool] | None = None) -> dict[str, Any]:
    if not r.get('eval_authorized') or not r.get('eval_command') or not r.get('acceptance_claim'):
        raise ValueError('No pre-authorized evaluator and named claim in receipt.')
    with locked(p.with_name('eval.lock')):
        s = load(state_path(p))
        if s.get('eval_started_at'):
            return s  # includes EVAL_RUNNING after a crash: report uncertainty, do not retry
        if s['process_state'] != 'PROCESS_COMPLETED':
            raise ValueError('Evaluation requires a successfully completed process.')
        if s['acceptance_state'] != 'UNASSESSED':
            raise ValueError(f"Evaluation blocked: {s['acceptance_state']}")
        s.update(acceptance_state='EVAL_RUNNING', eval_started_at=now(), updated_at=now())
        save(state_path(p), s)
        try:
            rc, cancelled = run_command(r['eval_command'], r['cwd'], p.with_name('eval.log'),
                                        p.with_name('CANCEL_REQUEST.json'), stop or [False])
            s.update(eval_exit_code=rc, acceptance_state='CANCELLED' if cancelled else ('PASS' if rc == 0 else 'FAIL'),
                     accepted_claim=r['acceptance_claim'], eval_finished_at=now(), updated_at=now())
        except OSError as exc:
            s.update(acceptance_state='EVAL_ERROR', eval_error=str(exc), updated_at=now())
        save(state_path(p), s); event(p, r, s, 'acceptance')
        return s

def get_status(p: Path, r: dict[str, Any]) -> dict[str, Any]:
    s = load(state_path(p))
    s = {**s, 'receipt': str(p), 'acceptance_claim': r.get('acceptance_claim', '')}
    plan = p.with_name('WAIT_PLAN.json')
    if plan.exists():
        s['wait_until_epoch'] = load(plan)['wait_until_epoch']
    deadline = s.get('wait_until_epoch')
    if deadline is not None:
        s['remaining_seconds'] = max(0, round(deadline - time.time(), 3))
    # No PID-only liveness claim: PID reuse, remote hosts and reboot require explicit inspection.
    return s

def status(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    return report(get_status(p, r))

def wait(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    initial = load(state_path(p))
    if 'wait_until_epoch' not in initial:
        raise ValueError('Not launched; wait needs a persisted launch-time ETA.')
    if args.until_epoch is not None:
        if args.until_epoch <= time.time() or not args.reason:
            raise ValueError('A revised future wait deadline requires --reason; do not reset it on every read.')
        save(p.with_name('WAIT_PLAN.json'), {'wait_until_epoch': args.until_epoch, 'reason': args.reason, 'updated_at': now()})
    deadline = get_status(p, r)['wait_until_epoch']
    while True:
        s = get_status(p, r)
        # During auto eval the original pipeline is still active; no model poll is needed.
        done = s['process_state'] in TERMINAL and s['acceptance_state'] != 'EVAL_RUNNING'
        if r.get('auto_eval') and s['process_state'] == 'PROCESS_COMPLETED' and s['acceptance_state'] == 'UNASSESSED':
            done = False
        if done:
            return report({**s, 'wait_reason': 'COMPLETION'})
        if time.time() >= deadline:
            return report({**s, 'wait_reason': 'DECISION_TIME_REACHED',
                           'note': 'No automatic kill/restart or PASS. Re-estimate once from actual evidence.'})
        time.sleep(min(1.0, max(0.01, deadline - time.time())))

def finalize(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    return report(evaluate(p, r) if args.run_eval else get_status(p, r))

def cancel(args: argparse.Namespace) -> int:
    p, r = checked_receipt(args.receipt)
    s = load(state_path(p))
    awaiting_auto_eval = r.get('auto_eval') and s['process_state'] == 'PROCESS_COMPLETED' and s['acceptance_state'] == 'UNASSESSED'
    if s['process_state'] in TERMINAL and s['acceptance_state'] != 'EVAL_RUNNING' and not awaiting_auto_eval:
        return report({'state': 'ALREADY_FINISHED', 'receipt': str(p)})
    if s['process_state'] == 'DECLARED':
        raise ValueError('Not launched; no process to cancel.')
    save(p.with_name('CANCEL_REQUEST.json'), {'requested_at': now(), 'reason': args.reason})
    return report({'state': 'CANCEL_REQUESTED', 'receipt': str(p), 'not_yet_confirmed': True})

def events(args: argparse.Namespace) -> int:
    root = project_root(args.project_root) / '.ai/runtime/run-events/pending'
    for p in sorted(root.glob('*.json')):
        report(load(p))
    return 0

def ack(args: argparse.Namespace) -> int:
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,120}', args.event):
        raise ValueError('Invalid event id.')
    root = project_root(args.project_root) / '.ai/runtime/run-events'
    root.mkdir(parents=True, exist_ok=True)
    with locked(root / 'ack.lock'):
        p = root / 'pending' / (args.event + '.json')
        dest = root / 'archive' / p.name
        if dest.exists():
            return report({'id': args.event, 'state': 'ALREADY_ACKNOWLEDGED'})
        d = load(p)
        d.update(acknowledged_at=now(), resolution=args.resolution)
        save(dest, d); p.unlink()
    return report({'id': args.event, 'state': 'ACKNOWLEDGED'})

def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    a = sub.add_parser('prepare'); a.set_defaults(func=prepare)
    a.add_argument('--project-root'); a.add_argument('--name', required=True)
    a.add_argument('--command', required=True); a.add_argument('--cwd', default='.')
    a.add_argument('--output', required=True); a.add_argument('--expected-seconds', required=True, type=float)
    a.add_argument('--eta-source', required=True); a.add_argument('--stop-condition', required=True)
    for k in ('checkpoint','session','source-revision','config-ref','checkpoint-lineage','owner-session','eval-command','acceptance-claim'):
        a.add_argument('--' + k, default='')
    a.add_argument('--resource', action='append', default=[])
    a.add_argument('--authorize-eval', action='store_true'); a.add_argument('--auto-eval', action='store_true')
    for name, func in [('launch', launch), ('_run', internal_run), ('status', status), ('wait', wait), ('finalize', finalize), ('cancel', cancel)]:
        a = sub.add_parser(name); a.set_defaults(func=func); a.add_argument('--receipt', required=True)
        if name == 'launch': a.add_argument('--backend', choices=['tmux','process'], default='tmux')
        if name == 'wait':
            a.add_argument('--until-epoch', type=float); a.add_argument('--reason', default='')
        if name == 'finalize': a.add_argument('--run-eval', action='store_true')
        if name == 'cancel': a.add_argument('--reason', required=True)
    for name, func in [('events', events), ('ack', ack)]:
        a = sub.add_parser(name); a.set_defaults(func=func); a.add_argument('--project-root')
        if name == 'ack':
            a.add_argument('--event', required=True); a.add_argument('--resolution', required=True)
    return p

if __name__ == '__main__':
    try:
        args = parser().parse_args()
        raise SystemExit(args.func(args))
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'run_supervisor: {exc}', file=sys.stderr)
        raise SystemExit(2)
