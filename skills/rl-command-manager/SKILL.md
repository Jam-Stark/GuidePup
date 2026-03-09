---
name: rl-command-manager
description: Manage RL train/play command batches in one markdown file with automatic play command generation and dual comparison analysis (vs previous batch and vs active base batch).
---

# RL Command Manager

Use this skill when the user wants to:
- add a new RL training command batch entry;
- auto-generate a matching play command;
- analyze changes against both previous batch and base batch;
- update the active base for future comparisons.

## Inputs

- Manager file: `experiments/COMMAND_BATCHES.md`
- Train command file: plain text or markdown code block containing a full train command
- Optional: target batch id, previous batch id, base id, public ip, play device, play num envs, checkpoint path

## Workflow

1. Ensure manager file exists and base is initialized.
2. Parse train command into structured fields:
- `prefix_env`
- `launcher`
- `entry_script`
- `cli_flags`
- `hydra_overrides`
3. Generate play command template from train command.
4. Compute diffs:
- current vs previous batch
- current vs active/specified base
5. Generate grouped impact notes.
6. Append or update only target batch section in manager file.

## Commands

Run helper from repo root:

```bash
python skills/rl-command-manager/scripts/batch_helper.py init \
  --manager-file experiments/COMMAND_BATCHES.md \
  --active-base-id cfg30_zScale \
  --base-train-cmd-file /path/to/base_train_cmd.txt
```

```bash
python skills/rl-command-manager/scripts/batch_helper.py add-batch \
  --manager-file experiments/COMMAND_BATCHES.md \
  --batch-id cfg32-base \
  --train-cmd-file /path/to/train_cmd.txt \
  --task Template-UniFPAliengo-v0 \
  --previous-batch-id auto \
  --base-id active \
  --public-ip base \
  --play-device base \
  --play-num-envs base \
  --checkpoint auto \
  --log-root logs/rsl_rl
```

```bash
python skills/rl-command-manager/scripts/batch_helper.py set-base \
  --manager-file experiments/COMMAND_BATCHES.md \
  --base-id cfg33_ref \
  --base-train-cmd-file /path/to/base_train_cmd.txt \
  --reason "phase-2 baseline"
```

```bash
python skills/rl-command-manager/scripts/batch_helper.py rebuild-analysis \
  --manager-file experiments/COMMAND_BATCHES.md \
  --batch-id cfg32-base \
  --log-root logs/rsl_rl
```

## References

- Group mapping rules: `references/override_groups.md`
- Impact heuristics: `references/impact_heuristics.md`

## Output policy

- Do not rewrite historical batch sections unless user explicitly asks.
- For missing references (previous/base), keep explicit blocked notes in diff sections.
- Keep `analysis_mode` fixed to `prev+base`.
- `PUBLIC_IP` defaults to `10.13.11.197`; `add-batch` can inherit `public_ip/play_device/play_num_envs` from base via `base`.
