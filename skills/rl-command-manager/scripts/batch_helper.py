#!/usr/bin/env python3
"""Manage RL train/play command batches with dual analysis (previous + base).

This tool maintains a markdown manager file with:
- base registry
- batch sections
- auto-generated play command
- diff vs previous and vs base
- impact notes
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

BASE_REGISTRY_START = "<!-- RLCM:BASE_REGISTRY_START -->"
BASE_REGISTRY_END = "<!-- RLCM:BASE_REGISTRY_END -->"
BATCHES_START = "<!-- RLCM:BATCHES_START -->"
BATCHES_END = "<!-- RLCM:BATCHES_END -->"

GROUP_ORDER = {
    "Actuator": 0,
    "Command": 1,
    "Reward": 2,
    "Curriculum": 3,
    "Runner/Device": 4,
    "Other": 5,
}

BOOL_FLAGS = {
    "--distributed",
    "--resume",
    "--video",
    "--disable_fabric",
    "--use_pretrained_checkpoint",
    "--real-time",
    "--headless",
    "--export_io_descriptors",
    "--enable_cameras",
}

FLAGS_WITH_VALUES = {
    "--video_length",
    "--video_interval",
    "--num_envs",
    "--task",
    "--agent",
    "--seed",
    "--max_iterations",
    "--experiment_name",
    "--run_name",
    "--load_run",
    "--checkpoint",
    "--logger",
    "--log_project_name",
    "--device",
}

CLI_ANALYSIS_FLAGS = {
    "--num_envs",
    "--device",
    "--distributed",
    "--seed",
    "--max_iterations",
    "--run_name",
    "--logger",
    "--log_project_name",
    "--headless",
}

RUN_DIR_NAME_RE = re.compile(r"^(?P<ts>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})_(?P<suffix>.+)$")
MODEL_FILE_RE = re.compile(r"^model_(\d+)\.pt$")
CHECKPOINT_AUTO_TOKENS = {"", "auto", "<checkpoint_path>", "<CHECKPOINT_PATH>"}
PUBLIC_IP_RE = re.compile(r"\bPUBLIC_IP=([^\s]+)")

DEFAULT_PUBLIC_IP = "10.13.11.197"
DEFAULT_PLAY_DEVICE = "cuda:1"
DEFAULT_PLAY_NUM_ENVS = 128
BASE_DEFAULT_TOKEN = "base"
PUBLIC_IP_PLACEHOLDERS = {"", "<PUBLIC_IP>", "<public_ip>"}


@dataclass
class CommandSpec:
    raw_command: str
    prefix_env: OrderedDict[str, str]
    launcher: str
    launcher_args: list[str]
    entry_script: str
    cli_flags: OrderedDict[str, Any]
    positionals: list[str]
    hydra_overrides: OrderedDict[str, str]


@dataclass
class DiffItem:
    key: str
    group: str
    change: str  # added | removed | changed
    old_value: str
    new_value: str


@dataclass
class BatchRecord:
    batch_id: str
    full_block: str
    metadata: dict[str, str]
    train_command: str
    play_command: str
    manual_notes: str


@dataclass
class ManagerDoc:
    text: str
    registry: dict[str, Any]
    batches_region: str
    batches: OrderedDict[str, BatchRecord]
    batch_order: list[str]


def now_iso() -> str:
    return datetime.now().replace(microsecond=0).isoformat()


def parse_run_timestamp(name: str) -> datetime | None:
    match = RUN_DIR_NAME_RE.match(name)
    if not match:
        return None
    try:
        return datetime.strptime(match.group("ts"), "%Y-%m-%d_%H-%M-%S")
    except ValueError:
        return None


def find_model_checkpoint(run_dir: Path) -> Path | None:
    model_files = [path for path in run_dir.glob("model_*.pt") if path.is_file()]
    if not model_files:
        return None

    numbered: list[tuple[int, Path]] = []
    for model_file in model_files:
        match = MODEL_FILE_RE.match(model_file.name)
        if match:
            numbered.append((int(match.group(1)), model_file))

    if numbered:
        numbered.sort(key=lambda item: item[0], reverse=True)
        return numbered[0][1]

    model_files.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return model_files[0]


def relative_posix(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def auto_checkpoint_from_logs(
    train_spec: CommandSpec,
    fallback_run_name: str,
    logs_root: Path,
    repo_root: Path,
) -> tuple[str | None, str | None]:
    run_name = str(train_spec.cli_flags.get("--run_name", fallback_run_name)).strip()
    if not run_name:
        return None, "missing run_name for auto checkpoint lookup"

    experiment_name = str(train_spec.cli_flags.get("--experiment_name", "")).strip()
    if logs_root.is_absolute():
        root = logs_root
    else:
        root = (repo_root / logs_root).resolve()
    if not root.exists():
        return None, f"logs root not found: {root}"

    experiment_dirs: list[Path]
    if experiment_name:
        experiment_dir = root / experiment_name
        experiment_dirs = [experiment_dir] if experiment_dir.is_dir() else []
    else:
        experiment_dirs = [path for path in root.iterdir() if path.is_dir()]

    run_dirs: list[Path] = []
    for exp_dir in experiment_dirs:
        for run_dir in exp_dir.iterdir():
            if not run_dir.is_dir():
                continue
            if run_dir.name.endswith(f"_{run_name}"):
                run_dirs.append(run_dir)

    if not run_dirs:
        scope = f"{root}/{experiment_name}" if experiment_name else str(root)
        return None, f"no run directory found for run_name='{run_name}' under {scope}"

    def run_dir_sort_key(path: Path):
        ts = parse_run_timestamp(path.name)
        mtime = path.stat().st_mtime
        return (ts is not None, ts or datetime.min, mtime)

    run_dirs.sort(key=run_dir_sort_key, reverse=True)
    for run_dir in run_dirs:
        checkpoint = find_model_checkpoint(run_dir)
        if checkpoint is not None:
            return relative_posix(checkpoint, repo_root), None

    return None, f"run directories found for '{run_name}', but no model_*.pt exists"


def resolve_checkpoint_path(
    checkpoint_arg: str,
    train_spec: CommandSpec,
    run_name: str,
    logs_root: Path,
    repo_root: Path,
) -> tuple[str, str | None]:
    raw = (checkpoint_arg or "").strip()
    if raw not in CHECKPOINT_AUTO_TOKENS:
        return raw, None

    resolved, reason = auto_checkpoint_from_logs(
        train_spec=train_spec,
        fallback_run_name=run_name,
        logs_root=logs_root,
        repo_root=repo_root,
    )
    if resolved is not None:
        return resolved, None
    return "<CHECKPOINT_PATH>", f"[WARN] Auto checkpoint unresolved: {reason}"


def is_env_assignment(token: str) -> bool:
    return re.match(r"^[A-Za-z_][A-Za-z0-9_]*=.+$", token) is not None


def extract_first_code_block(text: str) -> str:
    match = re.search(r"```(?:bash|sh|shell|text)?\n(.*?)\n```", text, flags=re.S)
    return match.group(1).strip() if match else text.strip()


def normalize_command_text(text: str) -> str:
    text = extract_first_code_block(text)
    text = text.replace("\\\r\n", " ").replace("\\\n", " ")
    text = re.sub(r"\s+", " ", text).strip()
    return text


def read_command_file(path: str) -> str:
    raw = Path(path).read_text(encoding="utf-8")
    return normalize_command_text(raw)


def should_take_flag_value(flag: str, next_token: str) -> bool:
    if next_token.startswith("--"):
        return False
    if flag in BOOL_FLAGS:
        return False
    if flag in FLAGS_WITH_VALUES:
        return True
    if "=" in next_token:
        return False
    return True


def parse_token_flags(tokens: list[str]) -> OrderedDict[str, Any]:
    out: OrderedDict[str, Any] = OrderedDict()
    idx = 0
    while idx < len(tokens):
        tok = tokens[idx]
        if tok.startswith("--"):
            if "=" in tok:
                flag, value = tok.split("=", 1)
                out[flag] = value
            else:
                if idx + 1 < len(tokens) and should_take_flag_value(tok, tokens[idx + 1]):
                    out[tok] = tokens[idx + 1]
                    idx += 1
                else:
                    out[tok] = True
        idx += 1
    return out


def parse_command(command: str) -> CommandSpec:
    normalized = normalize_command_text(command)
    if not normalized:
        raise ValueError("Empty command content.")

    tokens = shlex.split(normalized, posix=True)
    if not tokens:
        raise ValueError("Unable to tokenize command.")

    prefix_env: OrderedDict[str, str] = OrderedDict()
    idx = 0
    while idx < len(tokens) and is_env_assignment(tokens[idx]):
        key, value = tokens[idx].split("=", 1)
        prefix_env[key] = value
        idx += 1

    if idx >= len(tokens):
        raise ValueError("Command does not include a launcher.")

    launcher = tokens[idx]
    idx += 1
    launcher_args: list[str] = []
    entry_script = ""

    if launcher == "torchrun":
        while idx < len(tokens):
            tok = tokens[idx]
            if tok.endswith(".py"):
                entry_script = tok
                idx += 1
                break
            launcher_args.append(tok)
            idx += 1
        if not entry_script:
            raise ValueError("torchrun command missing python entry script.")
    elif launcher in {"python", "python3"}:
        if idx >= len(tokens):
            raise ValueError("python command missing entry script.")
        entry_script = tokens[idx]
        idx += 1
    elif launcher.endswith(".py"):
        entry_script = launcher
        launcher = "python"
    else:
        raise ValueError(f"Unsupported launcher: {launcher}")

    cli_flags: OrderedDict[str, Any] = OrderedDict()
    positionals: list[str] = []
    hydra_overrides: OrderedDict[str, str] = OrderedDict()

    while idx < len(tokens):
        tok = tokens[idx]
        if tok.startswith("--"):
            if "=" in tok:
                flag, value = tok.split("=", 1)
                cli_flags[flag] = value
            else:
                if idx + 1 < len(tokens) and should_take_flag_value(tok, tokens[idx + 1]):
                    cli_flags[tok] = tokens[idx + 1]
                    idx += 1
                else:
                    cli_flags[tok] = True
        else:
            if "=" in tok and not tok.startswith("="):
                key, value = tok.split("=", 1)
                hydra_overrides[key] = value
            else:
                positionals.append(tok)
        idx += 1

    return CommandSpec(
        raw_command=normalized,
        prefix_env=prefix_env,
        launcher=launcher,
        launcher_args=launcher_args,
        entry_script=entry_script,
        cli_flags=cli_flags,
        positionals=positionals,
        hydra_overrides=hydra_overrides,
    )


def group_key(key: str) -> str:
    if key.startswith("env.scene.robot.actuators."):
        return "Actuator"
    if key.startswith("env.commands."):
        return "Command"
    if key.startswith("env.rewards."):
        return "Reward"
    if key.startswith("env.curriculum."):
        return "Curriculum"
    if key.startswith("agent.") or key.startswith("cli.") or key.startswith("launcher.") or key.startswith("prefix_env."):
        return "Runner/Device"
    return "Other"


def analysis_params(spec: CommandSpec) -> OrderedDict[str, str]:
    params: OrderedDict[str, str] = OrderedDict()

    for key, value in spec.hydra_overrides.items():
        params[key] = str(value)

    for flag in CLI_ANALYSIS_FLAGS:
        if flag in spec.cli_flags:
            value = spec.cli_flags[flag]
            params[f"cli.{flag}"] = "true" if value is True else str(value)

    launcher_flags = parse_token_flags(spec.launcher_args)
    for flag, value in launcher_flags.items():
        params[f"launcher.{flag}"] = "true" if value is True else str(value)

    for key, value in spec.prefix_env.items():
        params[f"prefix_env.{key}"] = str(value)

    return params


def diff_params(old: OrderedDict[str, str], new: OrderedDict[str, str]) -> list[DiffItem]:
    out: list[DiffItem] = []
    all_keys = sorted(set(old.keys()) | set(new.keys()))
    for key in all_keys:
        in_old = key in old
        in_new = key in new
        if not in_old and in_new:
            out.append(DiffItem(key=key, group=group_key(key), change="added", old_value="", new_value=new[key]))
            continue
        if in_old and not in_new:
            out.append(DiffItem(key=key, group=group_key(key), change="removed", old_value=old[key], new_value=""))
            continue
        old_value = old[key]
        new_value = new[key]
        if old_value != new_value:
            out.append(
                DiffItem(
                    key=key,
                    group=group_key(key),
                    change="changed",
                    old_value=old_value,
                    new_value=new_value,
                )
            )
    out.sort(key=lambda item: (GROUP_ORDER.get(item.group, 99), item.key))
    return out


def change_text(item: DiffItem) -> str:
    if item.change == "added":
        return f"set to {item.new_value}"
    if item.change == "removed":
        return f"removed (was {item.old_value})"
    return f"{item.old_value} -> {item.new_value}"


def escape_md(value: str) -> str:
    return value.replace("|", "\\|")


def render_diff_markdown(diff_items: list[DiffItem] | None, missing_reason: str | None = None) -> str:
    if missing_reason:
        return f"Reference unavailable: {missing_reason}"
    if diff_items is None:
        return "Reference unavailable."
    if not diff_items:
        return "No parameter changes."

    lines = ["| group | key | change | from | to |", "|---|---|---|---|---|"]
    for item in diff_items:
        from_value = "(missing)" if item.change == "added" else item.old_value
        to_value = "(missing)" if item.change == "removed" else item.new_value
        lines.append(
            "| {group} | {key} | {change} | {old} | {new} |".format(
                group=escape_md(item.group),
                key=escape_md(item.key),
                change=escape_md(item.change),
                old=escape_md(str(from_value)),
                new=escape_md(str(to_value)),
            )
        )
    return "\n".join(lines)


def heuristic_for_key(key: str) -> tuple[str, str, str]:
    if key.endswith(".stiffness"):
        return (
            "Increases tracking authority and response speed.",
            "Can increase oscillation and torque spikes.",
            "Monitor torque limits, action_rate, and contact forces.",
        )
    if key.endswith(".damping"):
        return (
            "Adds damping and can improve motion smoothness.",
            "Too much damping may reduce agility and tracking quality.",
            "Monitor tracking error, settling time, and velocity lag.",
        )
    if "env.commands.gait_phase.gait_frequency" in key:
        return (
            "Changes gait cadence and contact timing.",
            "High cadence may destabilize stance transitions.",
            "Monitor stance/contact masks, slip rate, and body tilt.",
        )
    if "env.rewards.feet_air_time.params.target_air_time" in key:
        return (
            "Shifts preferred swing duration.",
            "May induce hopping or foot drag if mismatched.",
            "Monitor feet_air_time, feet_height, and feet_drag terms.",
        )
    if key.startswith("env.curriculum.") and key.endswith(".start_steps"):
        return (
            "Moves curriculum activation earlier or later.",
            "Too early can destabilize; too late can slow adaptation.",
            "Monitor reward progression and episode failure rate.",
        )
    if key.startswith("env.rewards."):
        return (
            "Re-weights reward objectives.",
            "Policy may exploit changed incentives.",
            "Monitor reward decomposition and qualitative play behavior.",
        )
    if key.startswith("env.commands."):
        return (
            "Changes command distribution or command semantics.",
            "Can narrow or shift learned behavior envelope.",
            "Monitor command tracking metrics and coverage.",
        )
    if key.startswith("env.scene.robot.actuators."):
        return (
            "Changes low-level joint behavior.",
            "May alter contact stability and torque usage.",
            "Monitor torque metrics and contact smoothness.",
        )
    if key.startswith("cli.") or key.startswith("launcher.") or key.startswith("prefix_env."):
        return (
            "Changes runtime/training execution context.",
            "Can alter throughput and reproducibility.",
            "Monitor fps, wall-time, and seed consistency.",
        )
    return (
        "Potential behavior change due to parameter update.",
        "Unclear side-effects without targeted checks.",
        "Monitor key task metrics and play stability.",
    )


def render_impact_notes(diff_prev: list[DiffItem] | None, diff_base: list[DiffItem] | None) -> str:
    if diff_prev is None and diff_base is None:
        return "Impact notes blocked: previous and base references unavailable."

    primary = diff_prev if diff_prev is not None else diff_base
    if primary is None:
        primary = []
    if not primary:
        if diff_base:
            primary = diff_base
        else:
            return "No impact notes generated because no parameter deltas were detected."

    base_by_key = {item.key: item for item in (diff_base or [])}

    lines = [
        "| group | key | change_vs_prev | change_vs_base | expected_impact | risk | monitor |",
        "|---|---|---|---|---|---|---|",
    ]
    for item in primary:
        expected, risk, monitor = heuristic_for_key(item.key)
        base_item = base_by_key.get(item.key)
        if base_item is None:
            change_vs_base = "no change"
        else:
            change_vs_base = change_text(base_item)
        change_vs_prev = change_text(item) if diff_prev is not None else "N/A"

        lines.append(
            "| {group} | {key} | {prev} | {base} | {impact} | {risk} | {monitor} |".format(
                group=escape_md(item.group),
                key=escape_md(item.key),
                prev=escape_md(change_vs_prev),
                base=escape_md(change_vs_base),
                impact=escape_md(expected),
                risk=escape_md(risk),
                monitor=escape_md(monitor),
            )
        )

    return "\n".join(lines)


def yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    text = "" if value is None else str(value)
    if text == "":
        return '""'
    if re.match(r"^[A-Za-z0-9._:/+\-<>]+$", text) and text.lower() not in {"null", "true", "false"}:
        return text
    return json.dumps(text)


def render_metadata_yaml(metadata: dict[str, Any]) -> str:
    ordered_keys = [
        "batch_id",
        "task",
        "run_name",
        "status",
        "previous_batch_id",
        "base_id",
        "created_at",
        "analysis_mode",
        "public_ip",
        "play_device",
        "play_num_envs",
        "checkpoint",
    ]
    lines = []
    for key in ordered_keys:
        if key in metadata:
            lines.append(f"{key}: {yaml_scalar(metadata[key])}")
    return "\n".join(lines)


def parse_simple_yaml(block: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for raw_line in block.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if value.startswith('"') and value.endswith('"') and len(value) >= 2:
            value = value[1:-1]
        elif value.startswith("'") and value.endswith("'") and len(value) >= 2:
            value = value[1:-1]
        if value in {"null", "None"}:
            value = ""
        out[key] = value
    return out


def extract_between_markers(text: str, start_marker: str, end_marker: str) -> str:
    start_idx = text.find(start_marker)
    if start_idx == -1:
        raise ValueError(f"Missing marker: {start_marker}")
    start_idx += len(start_marker)
    end_idx = text.find(end_marker, start_idx)
    if end_idx == -1:
        raise ValueError(f"Missing marker: {end_marker}")
    return text[start_idx:end_idx]


def replace_between_markers(text: str, start_marker: str, end_marker: str, new_content: str) -> str:
    start_idx = text.find(start_marker)
    if start_idx == -1:
        raise ValueError(f"Missing marker: {start_marker}")
    content_start = start_idx + len(start_marker)
    end_idx = text.find(end_marker, content_start)
    if end_idx == -1:
        raise ValueError(f"Missing marker: {end_marker}")
    return text[:content_start] + new_content + text[end_idx:]


def parse_registry(text: str) -> dict[str, Any]:
    segment = extract_between_markers(text, BASE_REGISTRY_START, BASE_REGISTRY_END)
    match = re.search(r"```json\n(.*?)\n```", segment, flags=re.S)
    if not match:
        raise ValueError("Base registry block must contain a json code block.")
    return json.loads(match.group(1))


def render_registry_block(registry: dict[str, Any]) -> str:
    return "\n```json\n" + json.dumps(registry, indent=2) + "\n```\n"


def extract_named_code(block: str, name: str) -> str:
    segment = extract_between_markers(
        block,
        f"<!-- RLCM:{name}_START -->",
        f"<!-- RLCM:{name}_END -->",
    )
    match = re.search(r"```[A-Za-z0-9_-]*\n(.*?)\n```", segment, flags=re.S)
    return match.group(1).strip() if match else ""


def extract_manual_notes(block: str) -> str:
    match = re.search(r"### Manual Result Notes\n(.*?)\n<!-- RLCM:BATCH_END", block, flags=re.S)
    if not match:
        return "- Fill training metrics and play observations."
    notes = match.group(1).rstrip()
    return notes if notes.strip() else "- Fill training metrics and play observations."


def parse_batches(region: str) -> tuple[OrderedDict[str, BatchRecord], list[str]]:
    batches: OrderedDict[str, BatchRecord] = OrderedDict()
    order: list[str] = []
    pattern = re.compile(
        r"<!-- RLCM:BATCH_START ([^\s]+) -->\n(.*?)\n<!-- RLCM:BATCH_END \1 -->",
        flags=re.S,
    )
    for match in pattern.finditer(region):
        batch_id = match.group(1)
        full_block = match.group(0)
        metadata_yaml = extract_named_code(full_block, "METADATA")
        metadata = parse_simple_yaml(metadata_yaml)
        train_command = extract_named_code(full_block, "TRAIN_COMMAND")
        play_command = extract_named_code(full_block, "PLAY_COMMAND")
        manual_notes = extract_manual_notes(full_block)
        batches[batch_id] = BatchRecord(
            batch_id=batch_id,
            full_block=full_block,
            metadata=metadata,
            train_command=train_command,
            play_command=play_command,
            manual_notes=manual_notes,
        )
        order.append(batch_id)
    return batches, order


def load_manager(path: Path) -> ManagerDoc:
    text = path.read_text(encoding="utf-8")
    registry = parse_registry(text)
    batches_region = extract_between_markers(text, BATCHES_START, BATCHES_END)
    batches, order = parse_batches(batches_region)
    return ManagerDoc(text=text, registry=registry, batches_region=batches_region, batches=batches, batch_order=order)


def build_template_file(registry: dict[str, Any]) -> str:
    return f"""# RL Command Batches

Single source of truth for train/play command batches.

## Usage

1. Keep each experiment as one batch section.
2. Use helper script to append/update batches and auto-generate play + analysis.
3. Analysis mode is fixed to `prev+base`.
4. Play defaults can inherit from `base` batch metadata.

## Field conventions

- `batch_id`: unique experiment id
- `task`: gym task id
- `run_name`: training run name
- `status`: `planned` | `running` | `done`
- `previous_batch_id`: previous batch used for incremental diff
- `base_id`: baseline batch id for stable comparison
- `created_at`: ISO timestamp
- `public_ip`: play livestream public ip (default `{DEFAULT_PUBLIC_IP}`)

## Base Registry

{BASE_REGISTRY_START}
```json
{json.dumps(registry, indent=2)}
```
{BASE_REGISTRY_END}

## Batches

{BATCHES_START}

<!-- RLCM:BATCH_TEMPLATE_START -->
## Batch <batch_id>

### Metadata
```yaml
batch_id: <batch_id>
task: Template-UniFPAliengo-v0
run_name: <run_name>
status: planned
previous_batch_id: <previous_batch_id>
base_id: <base_id>
created_at: <ISO_TIMESTAMP>
analysis_mode: prev+base
public_ip: {DEFAULT_PUBLIC_IP}
play_device: {DEFAULT_PLAY_DEVICE}
play_num_envs: {DEFAULT_PLAY_NUM_ENVS}
checkpoint: <CHECKPOINT_PATH>
```

### Train Command
```bash
# fill by helper
```

### Play Command (Auto)
```bash
# fill by helper
```

### Diff vs Previous (Auto)
```markdown
# fill by helper
```

### Diff vs Base (Auto)
```markdown
# fill by helper
```

### Impact Notes (Auto)
```markdown
# fill by helper
```

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_TEMPLATE_END -->

{BATCHES_END}
"""


def render_markdown_code(content: str) -> str:
    payload = content.strip() if content.strip() else "No content."
    return f"```markdown\n{payload}\n```"


def render_batch_block(
    batch_id: str,
    metadata: dict[str, Any],
    train_command: str,
    play_command: str,
    diff_prev_md: str,
    diff_base_md: str,
    impact_md: str,
    manual_notes: str,
) -> str:
    metadata_yaml = render_metadata_yaml(metadata)
    manual_notes = manual_notes.rstrip() if manual_notes.strip() else "- Fill training metrics and play observations."

    return f"""<!-- RLCM:BATCH_START {batch_id} -->
## Batch {batch_id}

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
{metadata_yaml}
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
{train_command}
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
{play_command}
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
{render_markdown_code(diff_prev_md)}
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
{render_markdown_code(diff_base_md)}
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
{render_markdown_code(impact_md)}
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
{manual_notes}
<!-- RLCM:BATCH_END {batch_id} -->"""


def insert_or_replace_batch(doc: ManagerDoc, batch_id: str, new_block: str) -> str:
    region = doc.batches_region
    if batch_id in doc.batches:
        old_block = doc.batches[batch_id].full_block
        region = region.replace(old_block, new_block)
    else:
        tail = region.rstrip()
        if tail:
            region = tail + "\n\n" + new_block + "\n"
        else:
            region = "\n" + new_block + "\n"

    text = doc.text
    text = replace_between_markers(text, BATCHES_START, BATCHES_END, region)
    return text


def update_registry(doc: ManagerDoc, registry: dict[str, Any]) -> str:
    registry_block = render_registry_block(registry)
    return replace_between_markers(doc.text, BASE_REGISTRY_START, BASE_REGISTRY_END, registry_block)


def to_command_lines(base_line: str, overrides: list[str]) -> str:
    if not overrides:
        return base_line
    lines = [base_line + " \\"]
    for idx, override in enumerate(overrides):
        suffix = " \\" if idx < len(overrides) - 1 else ""
        lines.append(f"    {override}{suffix}")
    return "\n".join(lines)


def parse_int_or_none(raw: Any) -> int | None:
    text = "" if raw is None else str(raw).strip()
    if not text:
        return None
    try:
        value = int(text)
    except ValueError:
        return None
    if value <= 0:
        return None
    return value


def normalize_public_ip(raw: str) -> str:
    text = (raw or "").strip()
    if text in PUBLIC_IP_PLACEHOLDERS:
        return DEFAULT_PUBLIC_IP
    return text


def extract_public_ip_from_play_command(play_command: str) -> str | None:
    match = PUBLIC_IP_RE.search(play_command or "")
    if not match:
        return None
    value = normalize_public_ip(match.group(1))
    return value if value else None


def get_base_play_defaults(doc: ManagerDoc, base_id: str) -> tuple[str | None, str | None, int | None]:
    if not base_id:
        return None, None, None
    base_batch = doc.batches.get(base_id)
    if base_batch is None:
        return None, None, None

    metadata = base_batch.metadata
    public_ip = normalize_public_ip(metadata.get("public_ip", ""))
    if not public_ip:
        public_ip = extract_public_ip_from_play_command(base_batch.play_command) or ""

    play_device = str(metadata.get("play_device", "")).strip() or None
    play_num_envs = parse_int_or_none(metadata.get("play_num_envs", ""))
    return (public_ip or None), play_device, play_num_envs


def resolve_play_runtime_defaults(
    doc: ManagerDoc,
    base_id: str,
    public_ip_arg: str,
    play_device_arg: str,
    play_num_envs_arg: str | int,
) -> tuple[str, str, int]:
    base_public_ip, base_play_device, base_play_num_envs = get_base_play_defaults(doc, base_id)

    raw_public_ip = (public_ip_arg or "").strip()
    if raw_public_ip.lower() == BASE_DEFAULT_TOKEN:
        public_ip = base_public_ip or DEFAULT_PUBLIC_IP
    else:
        public_ip = normalize_public_ip(raw_public_ip)
        if not public_ip:
            public_ip = DEFAULT_PUBLIC_IP

    raw_play_device = (play_device_arg or "").strip()
    if raw_play_device.lower() == BASE_DEFAULT_TOKEN:
        play_device = base_play_device or DEFAULT_PLAY_DEVICE
    else:
        play_device = raw_play_device or DEFAULT_PLAY_DEVICE

    raw_play_num_envs = (str(play_num_envs_arg) if play_num_envs_arg is not None else "").strip()
    if raw_play_num_envs.lower() == BASE_DEFAULT_TOKEN:
        play_num_envs = base_play_num_envs or DEFAULT_PLAY_NUM_ENVS
    else:
        parsed_play_num_envs = parse_int_or_none(raw_play_num_envs)
        if parsed_play_num_envs is None:
            raise ValueError(
                f"Invalid --play-num-envs value: {play_num_envs_arg}. "
                f"Use positive integer or '{BASE_DEFAULT_TOKEN}'."
            )
        play_num_envs = parsed_play_num_envs

    return public_ip, play_device, play_num_envs


def generate_play_command(
    train_spec: CommandSpec,
    task: str,
    public_ip: str,
    play_device: str,
    play_num_envs: int,
    checkpoint: str,
) -> str:
    overrides: OrderedDict[str, str] = OrderedDict()
    for key, value in train_spec.hydra_overrides.items():
        if key.startswith("env.") or key.startswith("agent."):
            overrides[key] = value

    if "agent.device" in overrides:
        del overrides["agent.device"]
    overrides["agent.device"] = play_device

    base_line = (
        f"PUBLIC_IP={public_ip} LIVESTREAM=1 ENABLE_CAMERAS=1 "
        f"python scripts/rsl_rl/play.py --task {task} --resume "
        f"--checkpoint {checkpoint} --num_envs {play_num_envs} --device {play_device}"
    )
    override_items = [f"{key}={value}" for key, value in overrides.items()]
    return to_command_lines(base_line, override_items)


def resolve_previous_id(doc: ManagerDoc, requested: str, current_batch_id: str) -> str:
    if requested != "auto":
        return "" if requested in {"", "none", "null"} else requested

    for batch_id in reversed(doc.batch_order):
        if batch_id != current_batch_id:
            return batch_id
    return ""


def build_analysis(
    current_spec: CommandSpec,
    prev_spec: CommandSpec | None,
    base_spec: CommandSpec | None,
    prev_missing_reason: str | None = None,
    base_missing_reason: str | None = None,
) -> tuple[str, str, str]:
    current = analysis_params(current_spec)

    prev_diff: list[DiffItem] | None
    base_diff: list[DiffItem] | None

    if prev_spec is None:
        prev_diff = None
    else:
        prev_diff = diff_params(analysis_params(prev_spec), current)

    if base_spec is None:
        base_diff = None
    else:
        base_diff = diff_params(analysis_params(base_spec), current)

    diff_prev_md = render_diff_markdown(prev_diff, prev_missing_reason)
    diff_base_md = render_diff_markdown(base_diff, base_missing_reason)
    impact_md = render_impact_notes(prev_diff, base_diff)

    return diff_prev_md, diff_base_md, impact_md


def cmd_init(args: argparse.Namespace) -> int:
    manager_path = Path(args.manager_file)
    manager_path.parent.mkdir(parents=True, exist_ok=True)

    base_spec = parse_command(read_command_file(args.base_train_cmd_file))
    registry = {
        "active_base": args.active_base_id,
        "bases": {
            args.active_base_id: {
                "created_at": now_iso(),
                "reason": "Initial base via init",
                "train_command": base_spec.raw_command,
            }
        },
    }

    if manager_path.exists():
        doc = load_manager(manager_path)
        doc.registry["active_base"] = args.active_base_id
        doc.registry.setdefault("bases", {})[args.active_base_id] = {
            "created_at": now_iso(),
            "reason": "Initial base via init",
            "train_command": base_spec.raw_command,
        }
        text = update_registry(doc, doc.registry)
    else:
        text = build_template_file(registry)

    manager_path.write_text(text, encoding="utf-8")
    print(f"Initialized manager: {manager_path}")
    print(f"Active base: {args.active_base_id}")
    return 0


def cmd_set_base(args: argparse.Namespace) -> int:
    manager_path = Path(args.manager_file)
    doc = load_manager(manager_path)

    base_spec = parse_command(read_command_file(args.base_train_cmd_file))
    registry = doc.registry
    registry.setdefault("bases", {})[args.base_id] = {
        "created_at": now_iso(),
        "reason": args.reason,
        "train_command": base_spec.raw_command,
    }
    registry["active_base"] = args.base_id

    text = update_registry(doc, registry)
    manager_path.write_text(text, encoding="utf-8")
    print(f"Set active base to: {args.base_id}")
    return 0


def cmd_add_batch(args: argparse.Namespace) -> int:
    manager_path = Path(args.manager_file)
    doc = load_manager(manager_path)

    train_spec = parse_command(read_command_file(args.train_cmd_file))

    previous_id = resolve_previous_id(doc, args.previous_batch_id, args.batch_id)

    if args.base_id == "active":
        base_id = doc.registry.get("active_base", "")
    else:
        base_id = args.base_id

    prev_spec: CommandSpec | None = None
    prev_missing_reason: str | None = None
    if previous_id:
        prev_batch = doc.batches.get(previous_id)
        if prev_batch is None:
            prev_missing_reason = f"previous batch '{previous_id}' not found"
        else:
            prev_spec = parse_command(prev_batch.train_command)
    else:
        prev_missing_reason = "no previous batch specified"

    base_spec: CommandSpec | None = None
    base_missing_reason: str | None = None
    base_entry = doc.registry.get("bases", {}).get(base_id)
    if base_entry is None:
        base_missing_reason = f"base id '{base_id}' not found in registry"
    else:
        base_spec = parse_command(base_entry["train_command"])

    diff_prev_md, diff_base_md, impact_md = build_analysis(
        train_spec,
        prev_spec,
        base_spec,
        prev_missing_reason=prev_missing_reason,
        base_missing_reason=base_missing_reason,
    )

    run_name = str(train_spec.cli_flags.get("--run_name", args.batch_id))
    checkpoint_path, checkpoint_warning = resolve_checkpoint_path(
        checkpoint_arg=args.checkpoint,
        train_spec=train_spec,
        run_name=run_name,
        logs_root=Path(args.log_root),
        repo_root=Path.cwd(),
    )
    if checkpoint_warning:
        print(checkpoint_warning)

    public_ip, play_device, play_num_envs = resolve_play_runtime_defaults(
        doc=doc,
        base_id=base_id,
        public_ip_arg=args.public_ip,
        play_device_arg=args.play_device,
        play_num_envs_arg=args.play_num_envs,
    )

    play_command = generate_play_command(
        train_spec=train_spec,
        task=args.task,
        public_ip=public_ip,
        play_device=play_device,
        play_num_envs=play_num_envs,
        checkpoint=checkpoint_path,
    )

    metadata = {
        "batch_id": args.batch_id,
        "task": args.task,
        "run_name": run_name,
        "status": "planned",
        "previous_batch_id": previous_id,
        "base_id": base_id,
        "created_at": now_iso(),
        "analysis_mode": "prev+base",
        "public_ip": public_ip,
        "play_device": play_device,
        "play_num_envs": str(play_num_envs),
        "checkpoint": checkpoint_path,
    }

    manual_notes = "- Fill training metrics and play observations."
    if args.batch_id in doc.batches:
        manual_notes = doc.batches[args.batch_id].manual_notes

    new_block = render_batch_block(
        batch_id=args.batch_id,
        metadata=metadata,
        train_command=train_spec.raw_command,
        play_command=play_command,
        diff_prev_md=diff_prev_md,
        diff_base_md=diff_base_md,
        impact_md=impact_md,
        manual_notes=manual_notes,
    )

    updated_text = insert_or_replace_batch(doc, args.batch_id, new_block)
    manager_path.write_text(updated_text, encoding="utf-8")

    print(f"Batch upserted: {args.batch_id}")
    print(f"Previous batch: {previous_id or '(none)'}")
    print(f"Base batch: {base_id or '(none)'}")
    return 0


def cmd_rebuild_analysis(args: argparse.Namespace) -> int:
    manager_path = Path(args.manager_file)
    doc = load_manager(manager_path)

    target = doc.batches.get(args.batch_id)
    if target is None:
        raise ValueError(f"Batch not found: {args.batch_id}")

    metadata = dict(target.metadata)
    train_spec = parse_command(target.train_command)

    previous_id = metadata.get("previous_batch_id", "")
    base_id = metadata.get("base_id", "")

    prev_spec: CommandSpec | None = None
    prev_missing_reason: str | None = None
    if previous_id:
        prev_batch = doc.batches.get(previous_id)
        if prev_batch is None:
            prev_missing_reason = f"previous batch '{previous_id}' not found"
        else:
            prev_spec = parse_command(prev_batch.train_command)
    else:
        prev_missing_reason = "no previous batch specified"

    base_spec: CommandSpec | None = None
    base_missing_reason: str | None = None
    base_entry = doc.registry.get("bases", {}).get(base_id)
    if base_entry is None:
        base_missing_reason = f"base id '{base_id}' not found in registry"
    else:
        base_spec = parse_command(base_entry["train_command"])

    diff_prev_md, diff_base_md, impact_md = build_analysis(
        train_spec,
        prev_spec,
        base_spec,
        prev_missing_reason=prev_missing_reason,
        base_missing_reason=base_missing_reason,
    )

    public_ip_hint = metadata.get("public_ip", BASE_DEFAULT_TOKEN)
    play_device_hint = metadata.get("play_device", BASE_DEFAULT_TOKEN)
    play_num_envs_hint = metadata.get("play_num_envs", BASE_DEFAULT_TOKEN)
    run_name = metadata.get("run_name", "") or str(train_spec.cli_flags.get("--run_name", ""))
    checkpoint_hint = metadata.get("checkpoint", "auto")
    checkpoint, checkpoint_warning = resolve_checkpoint_path(
        checkpoint_arg=checkpoint_hint,
        train_spec=train_spec,
        run_name=run_name,
        logs_root=Path(args.log_root),
        repo_root=Path.cwd(),
    )
    if checkpoint_warning:
        print(checkpoint_warning)
    metadata["checkpoint"] = checkpoint
    public_ip, play_device, play_num_envs = resolve_play_runtime_defaults(
        doc=doc,
        base_id=base_id,
        public_ip_arg=public_ip_hint,
        play_device_arg=play_device_hint,
        play_num_envs_arg=play_num_envs_hint,
    )
    metadata["public_ip"] = public_ip
    metadata["play_device"] = play_device
    metadata["play_num_envs"] = str(play_num_envs)
    task = metadata.get("task", "Template-UniFPAliengo-v0")

    play_command = generate_play_command(
        train_spec=train_spec,
        task=task,
        public_ip=public_ip,
        play_device=play_device,
        play_num_envs=play_num_envs,
        checkpoint=checkpoint,
    )

    metadata.setdefault("analysis_mode", "prev+base")

    new_block = render_batch_block(
        batch_id=args.batch_id,
        metadata=metadata,
        train_command=train_spec.raw_command,
        play_command=play_command,
        diff_prev_md=diff_prev_md,
        diff_base_md=diff_base_md,
        impact_md=impact_md,
        manual_notes=target.manual_notes,
    )

    updated_text = insert_or_replace_batch(doc, args.batch_id, new_block)
    manager_path.write_text(updated_text, encoding="utf-8")
    print(f"Rebuilt analysis for batch: {args.batch_id}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage RL train/play command batches.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    parser_init = subparsers.add_parser("init", help="Initialize manager file with active base.")
    parser_init.add_argument("--manager-file", required=True)
    parser_init.add_argument("--active-base-id", required=True)
    parser_init.add_argument("--base-train-cmd-file", required=True)
    parser_init.set_defaults(func=cmd_init)

    parser_add = subparsers.add_parser("add-batch", help="Add or update a batch section.")
    parser_add.add_argument("--manager-file", required=True)
    parser_add.add_argument("--batch-id", required=True)
    parser_add.add_argument("--train-cmd-file", required=True)
    parser_add.add_argument("--task", required=True)
    parser_add.add_argument("--previous-batch-id", default="auto")
    parser_add.add_argument("--base-id", default="active")
    parser_add.add_argument("--public-ip", default=BASE_DEFAULT_TOKEN)
    parser_add.add_argument("--play-device", default=BASE_DEFAULT_TOKEN)
    parser_add.add_argument("--play-num-envs", default=BASE_DEFAULT_TOKEN)
    parser_add.add_argument("--checkpoint", default="auto")
    parser_add.add_argument("--log-root", default="logs/rsl_rl")
    parser_add.set_defaults(func=cmd_add_batch)

    parser_set_base = subparsers.add_parser("set-base", help="Set active base and store its train command snapshot.")
    parser_set_base.add_argument("--manager-file", required=True)
    parser_set_base.add_argument("--base-id", required=True)
    parser_set_base.add_argument("--base-train-cmd-file", required=True)
    parser_set_base.add_argument("--reason", required=True)
    parser_set_base.set_defaults(func=cmd_set_base)

    parser_rebuild = subparsers.add_parser("rebuild-analysis", help="Rebuild auto sections for one existing batch.")
    parser_rebuild.add_argument("--manager-file", required=True)
    parser_rebuild.add_argument("--batch-id", required=True)
    parser_rebuild.add_argument("--log-root", default="logs/rsl_rl")
    parser_rebuild.set_defaults(func=cmd_rebuild_analysis)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
