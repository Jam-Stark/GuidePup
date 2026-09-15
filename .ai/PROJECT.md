# Project overlay

## Identity

- Repository: `Jam-Stark/GuidePup` working tree at `ForceControl`.
- Prototype branch/worktree: `codex/a2-lightnav-prototype` at `/home/baoquanc/workspace/ForceControl-lightnav`, created from `4605004` (A2_v2). The original `ForceControl` worktree contains newer uncommitted work and is not this task's write target.
- Prototype entrypoint: `prototype/README.md`; dependencies, models and outputs live under `.local`, linked to `/mnt/ssd1/baoquanc/forcecontrol-lightnav`.
- Domain: Isaac Lab extension for legged/mobile-manipulation reinforcement learning, evaluation, and MuJoCo sim-to-sim workflows.
- Source-truth order: executed source and resolved configuration -> runtime telemetry/artifacts -> official dependency documentation -> plans and durable memory. Plans and memory route work but do not override current code or runtime evidence.

## Real paths

```text
entrypoints: scripts/rsl_rl/{train,play}.py, scripts/skrl/{train,play}.py, scripts/mujoco/play_policy.py, scripts/list_envs.py
configs: source/LMP/acc/tasks/manager_based/**, especially */*_env_cfg.py and */agents/*
runtime/evaluation: scripts/experiments/uniFP_aliengo/**, scripts/mujoco/**
robot/model assets: resources/**, with MuJoCo XML under resources/*/mujoco/
artifacts: logs/**, outputs/**, io_descriptors/**, experiments/COMMAND_BATCHES.md
memory routes: MEMORY.md -> memory/MEMORY.md -> memory/<entry>/{description,TODO,DONE}.md
```

## Protected paths

Generic role refresh/migration must not overwrite `MEMORY.md`, `memory/**`, `.ai/PROJECT.md`, `source/LMP/**`, `scripts/**`, `resources/**`, `experiments/**`, `io_descriptors/**`, runtime outputs, checkpoints, or local credentials/configuration.

## Validation map

- Python syntax/import claim: targeted compile/import using the Isaac Lab Python environment.
- Task registration/config wiring claim: `python scripts/list_envs.py` plus targeted config/source inspection.
- Isaac Lab runtime claim: the matching `scripts/rsl_rl/*`, `scripts/skrl/*`, or evaluation entrypoint with the resolved task/config.
- MuJoCo sim-to-sim claim: the matching `scripts/mujoco/*` entrypoint, model XML, exported policy, and recorded telemetry.
- Training/experiment claim: exact command, resolved config, checkpoint lineage, seed/device, logs, and output path; static configuration is not evidence of run completion.
- Hardware claim: requires separate real-hardware evidence; simulation results do not establish hardware behavior or safety.

## Resource and safety boundaries

- Treat GPU, simulator display/process, ports, robot hardware, checkpoints, and experiment output directories as exclusive when a task actually uses them.
- Runs expected to exceed 30 minutes belong in a dedicated `tmux` session with the command and output location recorded; do not start expensive training or hardware motion without current explicit Owner authorization.
- Git commit/push, uploads, cloud actions, and other external writes require current explicit Owner authorization.
- Preserve fail-fast behavior in Isaac Lab code: do not hide invalid state with fallback data, broad exception handling, or defensive continuation.

## Cloud Pro handoff

- Git remote used by cloud reviewer: `origin`.
- Branch or review branch: current task branch; verify it from Git before each handoff.
- Whether Owner request authorizes in-scope commit+push: yes, only when the Owner explicitly requests a Cloud Pro handoff and subject to branch protection.
- Drive task folder for Worker artifacts: `Pro_Space/ForceControl/<worktree>/<stage>/<release>/`.
- Pro delivery transfer: Owner uploads `pro_delivery__full_review.zip` in the local Worker conversation; the cloud Pro does not upload it to Drive.
- Pro review document root: `docs/pro-reviews`.
- Placement rule: `<Pro review document root>/<stage-or-release>/<commit-short>/`.
- Local AI decisions reserved from cloud gate-setting: IsaacLab/GPU/runtime feasibility, exact commands, resource constraints, scientific admission thresholds, hardware safety, and acceptance based on local evidence.

## Workflow v1.4.0

- Imported the workflow files from the current `/home/baoquanc/workspace/ForceControl` working tree on `feature/a2-migration`; the workflow is included in the Owner-authorized navigation migration commit/push. No branch merge.
- Main retains scope, resources, Git and integration authority. Roles are configured in `.codex/agents/`; team routing is `.codex/TEAM.md`.
- Keep this prototype’s project overlay, navigation memory, old policy and local model/service configuration.
- This is a linked worktree. Hook files and role configuration are installed locally; effective host discovery/trust remains unverified.
- Migration evidence is in `.ai/WORKFLOW_SYNC.md`.

## Navigation migration

Owner 2026-09-15 authorized publishing the current prototype and required checkpoint/log artifacts. See `prototype/MIGRATION.md`, release `20260915_215130_HKT`. Drive upload uses the proxy; source/model downloads remain direct. New-machine data is repository-relative `.local`; select its physical GPU with `LIGHTNAV_GPU`. The two pending framework trials have not run because both old-machine GPU allocations blocked during NVIDIA renderer initialization.
