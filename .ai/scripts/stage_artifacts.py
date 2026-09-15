#!/usr/bin/env python3
"""Explicit stage-artifact inspector, semantic ZIP packer, and rclone uploader.

No command runs automatically at task closure. Cloud-facing ZIP files are
ordinary, independently readable archives capped at 95 MiB by default. The
tool never creates split-volume .z01/.z02 archives or binary-slices a model.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import shutil
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Any, Iterable
from zoneinfo import ZoneInfo

try:
    import tomllib
except ModuleNotFoundError as exc:
    raise SystemExit('Python 3.11+ is required') from exc

MIB = 1024 * 1024
DEFAULT_LIMIT = 95 * MIB
DEFAULT_CONFIG = Path('.ai/artifact-sync.toml')
DEFAULT_WORKER_PREFIX = 'worker_delivery__'
DEFAULT_PRO_PREFIX = 'pro_delivery__'
DEFAULT_PRO_ZIP = 'pro_delivery__full_review.zip'
CHECKPOINT_SUFFIXES = {'.pt','.pth','.ckpt','.onnx','.safetensors','.bin'}
MEDIA_SUFFIXES = {'.png','.jpg','.jpeg','.webp','.gif','.svg','.pdf','.mp4','.mov','.avi','.mkv','.webm','.html'}
SOURCE_SUFFIXES = {'.py','.pyi','.ipynb','.toml','.yaml','.yml','.json','.jsonc','.ini','.cfg','.conf','.xml','.urdf','.md'}
LOG_SUFFIXES = {'.log','.json','.jsonl','.csv','.tsv','.txt','.out','.err'}
SPLIT_RE = re.compile(r'(?:\.z\d{2,}|\.zip\.\d+)$', re.I)
SECRET_NAME_RE = re.compile(r'(^|[._-])(secret|token|credential|private[_-]?key)([._-]|$)', re.I)
SECRET_PATTERNS = [
    re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
    re.compile(r'\bsk-[A-Za-z0-9_-]{20,}\b'),
    re.compile(r'\bgh[pousr]_[A-Za-z0-9]{20,}\b'),
    re.compile(r'\bAKIA[0-9A-Z]{16}\b'),
]
PURPOSE = {
    'source_and_configs': '源文件、resolved config、运行配置与说明文档。',
    'logs_and_metrics': '训练/评估日志、指标、summary、receipt 与 ledger。',
    'plots_and_evidence': '图表、render、视频、PDF 与其他可视化证据。',
    'checkpoints': '经显式 opt-in 选择且可独立读取的模型/checkpoint。',
    'other_evidence': '未落入以上类别的其他阶段证据。',
}
ORDER = tuple(PURPOSE)


@dataclass(frozen=True)
class Candidate:
    path: Path
    relative: str
    size: int
    source: str


@dataclass(frozen=True)
class Exclusion:
    relative: str
    reason: str


@dataclass(frozen=True)
class Archive:
    name: str
    group: str
    files: tuple[Candidate, ...]
    size: int


def run(cmd: list[str], cwd: Path, check: bool=True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(cmd, cwd=cwd, capture_output=True, check=check)


def git_root(start: Path) -> Path:
    return Path(run(['git','rev-parse','--show-toplevel'], start).stdout.decode().strip()).resolve()


def git_text(root: Path, *args: str) -> str:
    return run(['git',*args], root).stdout.decode(errors='replace').strip()


def load_config(root: Path, raw: Path) -> dict[str, Any]:
    path = raw if raw.is_absolute() else root / raw
    with path.open('rb') as handle:
        return tomllib.load(handle)


def slug(text: str) -> str:
    return re.sub(r'-+', '-', re.sub(r'[^A-Za-z0-9._-]+','-',text.strip())).strip('-._') or 'unknown'


def match(path: str, patterns: Iterable[str]) -> bool:
    return any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns)


def naming(cfg: dict[str,Any]) -> tuple[str,str,str]:
    values = cfg.get('naming', {})
    worker = str(values.get('worker_prefix', DEFAULT_WORKER_PREFIX))
    pro = str(values.get('pro_prefix', DEFAULT_PRO_PREFIX))
    pro_zip = str(values.get('pro_full_review_zip', DEFAULT_PRO_ZIP))
    for label, value in (('worker_prefix', worker), ('pro_prefix', pro), ('pro_full_review_zip', pro_zip)):
        if not value or '/' in value or '\\' in value:
            raise SystemExit(f'invalid naming.{label}: {value!r}')
    if not worker.endswith('__') or not pro.endswith('__'):
        raise SystemExit('worker_prefix and pro_prefix must end with double underscore')
    if not pro_zip.startswith(pro) or not pro_zip.endswith('.zip'):
        raise SystemExit('pro_full_review_zip must use pro_prefix and end with .zip')
    return worker, pro, pro_zip


def discover(root: Path) -> dict[str,str]:
    result: dict[str,str] = {}
    for source, args in (
        ('untracked',['git','ls-files','--others','--exclude-standard','-z']),
        ('ignored',['git','ls-files','--others','--ignored','--exclude-standard','-z']),
    ):
        for raw in run(args, root).stdout.split(b'\0'):
            if raw:
                result[PurePosixPath(raw.decode(errors='surrogateescape')).as_posix()] = source
    return result


def sensitive_name(relative: str) -> bool:
    parts = PurePosixPath(relative).parts
    return any(part in {'.git','.venv','venv','node_modules'} or part.startswith('.env') or SECRET_NAME_RE.search(part) for part in parts)


def sensitive_content(path: Path) -> bool:
    if path.stat().st_size > 2*MIB:
        return False
    data = path.read_bytes()
    if b'\0' in data[:4096]:
        return False
    text = data.decode('utf-8', errors='ignore')
    return any(pattern.search(text) for pattern in SECRET_PATTERNS)


def select(root: Path, cfg: dict[str,Any], args: argparse.Namespace) -> tuple[list[Candidate],list[Exclusion]]:
    scfg = cfg.get('selection', {})
    includes = list(scfg.get('include', [])); excludes = list(scfg.get('exclude', []))
    checkpoints = list(scfg.get('checkpoint_patterns', [])); max_file = int(scfg.get('max_file_bytes', 2*1024**3)); max_bundle = int(scfg.get('max_bundle_bytes',10*1024**3))
    if not includes:
        raise SystemExit('selection.include must not be empty')
    stage_tokens = {args.stage.lower(), slug(args.stage).lower()}; selectors = [x.lower() for x in args.selector]
    accepted: list[Candidate] = []; rejected: list[Exclusion] = []; total = 0
    for relative, source in sorted(discover(root).items()):
        path = root / relative
        if not path.is_file() or path.is_symlink() or not match(relative, includes):
            continue
        reason = None
        if match(relative, excludes) or sensitive_name(relative): reason = 'excluded-or-sensitive-name'
        elif checkpoints and match(relative, checkpoints) and not args.include_checkpoints: reason = 'checkpoint-opt-in-required'
        elif not args.all_matching and not selectors and not any(x in relative.lower() for x in stage_tokens): reason = 'stage-not-in-path'
        elif selectors and not any(x in relative.lower() for x in selectors): reason = 'selector-mismatch'
        elif path.stat().st_size > max_file: reason = f'file-too-large:{path.stat().st_size}'
        elif sensitive_content(path): reason = 'secret-content'
        elif total + path.stat().st_size > max_bundle: reason = 'bundle-size-limit'
        if reason:
            rejected.append(Exclusion(relative, reason)); continue
        item = Candidate(path, relative, path.stat().st_size, source); accepted.append(item); total += item.size
    return accepted, rejected


def group(item: Candidate) -> str:
    suffix = item.path.suffix.lower(); lower = item.relative.lower()
    if suffix in CHECKPOINT_SUFFIXES: return 'checkpoints'
    if suffix in MEDIA_SUFFIXES or any(x in lower for x in ('plot','render','video','figure','evidence')): return 'plots_and_evidence'
    if suffix in SOURCE_SUFFIXES or any(x in lower for x in ('config','source','script')): return 'source_and_configs'
    if suffix in LOG_SUFFIXES or any(x in lower for x in ('log','metric','summary','receipt','ledger','eval')): return 'logs_and_metrics'
    return 'other_evidence'


def zip_once(destination: Path, files: list[Candidate]) -> int:
    with zipfile.ZipFile(destination, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for item in files:
            zf.write(item.path, arcname=item.relative)
    return destination.stat().st_size


def split_to_limit(out: Path, base: str, files: list[Candidate], limit: int, checkpoint_group: bool, worker_prefix: str) -> tuple[list[Archive],list[Exclusion]]:
    if not files:
        return [], []
    probe = out / f'.{base}-{os.getpid()}-probe.zip'
    size = zip_once(probe, files); probe.unlink()
    if size <= limit:
        name = f'{base}.zip'; final = out / name; size = zip_once(final, files)
        group_name = base.removeprefix(worker_prefix).split('_part')[0]
        return [Archive(name, group_name, tuple(files), size)], []
    if len(files) == 1:
        reason = 'checkpoint-over-cloud-limit-use-rclone-exception' if checkpoint_group else 'single-file-over-cloud-limit'
        return [], [Exclusion(files[0].relative, reason)]
    files = sorted(files, key=lambda x: x.size, reverse=True)
    left: list[Candidate] = []; right: list[Candidate] = []; lsize = rsize = 0
    for item in files:
        if lsize <= rsize: left.append(item); lsize += item.size
        else: right.append(item); rsize += item.size
    records: list[Archive] = []; excluded: list[Exclusion] = []
    for idx, subset in enumerate((left,right), 1):
        sub_records, sub_excluded = split_to_limit(out, f'{base}_part{idx:02d}', subset, limit, checkpoint_group, worker_prefix)
        records.extend(sub_records); excluded.extend(sub_excluded)
    return records, excluded


def metadata(root: Path, args: argparse.Namespace, accepted: list[Candidate], rejected: list[Exclusion], worker_prefix: str, pro_prefix: str, pro_zip: str) -> dict[str,Any]:
    current = datetime.now(ZoneInfo('Asia/Hong_Kong'))
    return {
        'schema_version': 4, 'project': args.project or root.name, 'worktree': args.worktree or root.name,
        'stage': args.stage, 'timestamp_hkt': current.isoformat(timespec='seconds'),
        'release': f"{current.strftime('%Y%m%d-%H%M%S-HKT')}__{git_text(root,'rev-parse','--short=12','HEAD')}",
        'branch': git_text(root,'branch','--show-current') or 'detached', 'git_revision': git_text(root,'rev-parse','HEAD'),
        'handoff_trigger': args.trigger, 'included': [{'path':x.relative,'bytes':x.size,'source':x.source} for x in accepted],
        'excluded': [{'path':x.relative,'reason':x.reason} for x in rejected],
        'questions': args.question or [],
        'delivery_naming': {'worker_prefix': worker_prefix, 'pro_prefix': pro_prefix, 'pro_full_review_zip': pro_zip},
        'evidence_boundary': 'Selected artifact presence does not by itself prove runtime, experiment, or hardware success.',
    }


def index_text(meta: dict[str,Any], archives: list[Archive], excluded: list[Exclusion], limit: int) -> str:
    lines = [f"# Worker Bundle Index — {meta['project']} / {meta['stage']}",'',f"Release: `{meta['release']}`",f"Single-ZIP ceiling: `{limit}` bytes (95 MiB default, final compressed size)",'','## Worker delivery archives']
    for idx, record in enumerate(archives,1):
        lines += ['',f"### {idx}. `{record.name}`",'',f"Purpose: {PURPOSE.get(record.group,'阶段证据。')}",f"Compressed bytes: `{record.size}`",'','Contents:']
        lines += [f"- `{item.relative}`" for item in record.files]
    lines += ['','## Not packaged']
    lines += [f"- `{item.relative}` — {item.reason}" for item in excluded] or ['- None.']
    lines += ['',f"The cloud Pro full-review archive will be uploaded later into this same task folder as `{meta['delivery_naming']['pro_full_review_zip']}`.",'Each ZIP above is a normal independent archive. No `.z01/.z02` or binary reconstruction is required.','']
    return '\n'.join(lines)


def handoff_text(meta: dict[str,Any]) -> str:
    questions = meta['questions'] or ['请独立分析本阶段结果、失败模式、替代解释和下一阶段候选。']
    return '\n'.join([f"# Worker-to-Pro handoff — {meta['project']} / {meta['stage']}",'',f"Git revision: `{meta['git_revision']}`",f"Branch: `{meta['branch']}`",'', '## Questions', *[f'- {q}' for q in questions], '', '## Evidence boundary', meta['evidence_boundary'], '', '## Expected Pro delivery', f"Upload `{meta['delivery_naming']['pro_full_review_zip']}` into this same release folder after review.", ''])


def pack(args: argparse.Namespace) -> int:
    if not args.confirm_stage_handoff:
        raise SystemExit('pass --confirm-stage-handoff after Owner/stage authorization')
    root = git_root(args.repo); cfg = load_config(root, args.config); accepted, rejected = select(root,cfg,args)
    if not accepted and not args.allow_empty:
        raise SystemExit('no eligible artifacts')
    worker_prefix, pro_prefix, pro_zip = naming(cfg)
    meta = metadata(root,args,accepted,rejected,worker_prefix,pro_prefix,pro_zip); base = args.output or root / '.ai/outgoing-artifacts'
    release = base / meta['project'] / meta['worktree'] / slug(args.stage) / meta['release']
    release.mkdir(parents=True, exist_ok=False)
    limit = int(cfg.get('packaging',{}).get('max_single_zip_bytes', DEFAULT_LIMIT))
    if limit > DEFAULT_LIMIT:
        raise SystemExit('max_single_zip_bytes must not exceed 95 MiB for cloud-facing bundles')
    records: list[Archive] = []; oversized: list[Exclusion] = []
    try:
        grouped = {name: [] for name in ORDER}
        for item in accepted: grouped[group(item)].append(item)
        for name in ORDER:
            rec, exc = split_to_limit(release, f'{worker_prefix}{name}', grouped[name], limit, name=='checkpoints', worker_prefix)
            records.extend(rec); oversized.extend(exc)
        delivered = {item.relative for rec in records for item in rec.files}
        meta['included'] = [x for x in meta['included'] if x['path'] in delivered]
        all_excluded = rejected + oversized; meta['excluded'] = [{'path':x.relative,'reason':x.reason} for x in all_excluded]
        meta['packaging'] = {'max_single_zip_bytes':limit,'strategy':'semantic-independent-standard-zips','split_volume_archives':False,'archives':[{'name':r.name,'group':r.group,'compressed_bytes':r.size,'files':[x.relative for x in r.files]} for r in records]}
        (release/f'{worker_prefix}BUNDLE_MANIFEST.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        (release/f'{worker_prefix}BUNDLE_INDEX.md').write_text(index_text(meta,records,all_excluded,limit),encoding='utf-8')
        (release/f'{worker_prefix}PRO_HANDOFF.md').write_text(handoff_text(meta),encoding='utf-8')
        if not records and not args.allow_empty: raise SystemExit('no cloud-readable archive produced')
    except BaseException:
        shutil.rmtree(release, ignore_errors=True); raise
    print(release)
    for record in records: print(f'{record.name}\t{record.size}')
    return 0


def inspect(args: argparse.Namespace) -> int:
    root=git_root(args.repo); cfg=load_config(root,args.config); accepted,rejected=select(root,cfg,args)
    for item in accepted: print(f'ELIGIBLE\t{item.relative}\t{item.size}\t{group(item)}')
    for item in rejected: print(f'EXCLUDED\t{item.relative}\t{item.reason}')
    return 0


def validate_upload(source: Path, limit: int, oversize_exception: bool) -> None:
    files = [source] if source.is_file() else [x for x in source.rglob('*') if x.is_file()]
    if not files: raise SystemExit('upload source is empty')
    bad = [x for x in files if SPLIT_RE.search(x.name)]
    if bad: raise SystemExit('split-volume archives are prohibited: ' + ', '.join(map(str,bad)))
    large = [x for x in files if x.stat().st_size > limit]
    if large and not oversize_exception: raise SystemExit('file exceeds cloud limit: ' + ', '.join(map(str,large)))


def upload(args: argparse.Namespace) -> int:
    if not args.confirm_stage_handoff: raise SystemExit('pass --confirm-stage-handoff')
    root=git_root(args.repo); cfg=load_config(root,args.config); source=args.bundle.resolve()
    if not source.exists(): raise SystemExit(f'not found: {source}')
    limit=int(cfg.get('packaging',{}).get('max_single_zip_bytes',DEFAULT_LIMIT)); validate_upload(source,limit,args.allow_oversize_rclone_exception)
    if shutil.which('rclone') is None: raise SystemExit('rclone unavailable; use a connected Drive/browser runtime or configure rclone')
    up=cfg.get('upload',{}); remote=str(up.get('rclone_remote',up.get('remote',''))).strip(); folder=str(up.get('root_folder_id','')).strip()
    if not remote or not folder: raise SystemExit('upload remote/root_folder_id missing')
    destination='/'.join(map(slug,[args.project,args.worktree,args.stage,args.release]))
    cmd=['rclone','copy',str(source),f'{remote}:{destination}','--drive-root-folder-id',folder,'--no-traverse','--progress']
    if args.dry_run: cmd.append('--dry-run')
    return subprocess.run(cmd,cwd=root,check=False).returncode


def selection_args(p: argparse.ArgumentParser) -> None:
    p.add_argument('--repo',type=Path,default=Path.cwd()); p.add_argument('--config',type=Path,default=DEFAULT_CONFIG); p.add_argument('--stage',required=True); p.add_argument('--selector',action='append',default=[]); p.add_argument('--all-matching',action='store_true'); p.add_argument('--include-checkpoints',action='store_true')


def parser() -> argparse.ArgumentParser:
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest='cmd',required=True)
    i=sub.add_parser('inspect'); selection_args(i); i.set_defaults(func=inspect)
    k=sub.add_parser('pack'); selection_args(k); k.add_argument('--project'); k.add_argument('--worktree'); k.add_argument('--output',type=Path); k.add_argument('--question',action='append',default=[]); k.add_argument('--allow-empty',action='store_true'); k.add_argument('--confirm-stage-handoff',action='store_true'); k.add_argument('--trigger',choices=['owner-request','stage-closure'],required=True); k.set_defaults(func=pack)
    u=sub.add_parser('upload'); u.add_argument('--repo',type=Path,default=Path.cwd()); u.add_argument('--config',type=Path,default=DEFAULT_CONFIG); u.add_argument('--bundle',type=Path,required=True); u.add_argument('--project',required=True); u.add_argument('--worktree',required=True); u.add_argument('--stage',required=True); u.add_argument('--release',required=True); u.add_argument('--confirm-stage-handoff',action='store_true'); u.add_argument('--allow-oversize-rclone-exception',action='store_true'); u.add_argument('--dry-run',action='store_true'); u.set_defaults(func=upload)
    return p


if __name__ == '__main__':
    args=parser().parse_args(); raise SystemExit(args.func(args))
