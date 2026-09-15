# Optional coordination state

## 1. 默认状态

Team state is **inactive by default**. FAST and ordinary STANDARD tasks do not initialize `.ai/runtime/team/`, do not require disk-backed contracts, and do not freeze a candidate.

Use it only for:

- multiple writers or overlapping ownership risk;
- exclusive GPU/IsaacSim/display/port/hardware/output resources;
- cross-session DAG or long-lived coordinated work;
- formal review/runtime QA that needs exact candidate identity;
- existing verdicts whose validity must survive narrow fixes.

## 2. 激活模式

```bash
python .ai/scripts/team_state.py status
python .ai/scripts/team_state.py activate --mode adaptive --reason "two writers + GPU leases"
```

- `adaptive`: registered tasks are validated; unregistered lean/read-only spawns remain allowed.
- `strict`: roles configured as controlled must have a valid contract before spawn. Use only for formal coordinated work.

结束后：

```bash
python .ai/scripts/team_state.py deactivate --archive
```

Ledger runtime files are Git-ignored and are not durable project memory.

## 3. Task contract

Do not register every spawn. Register only tasks whose ownership、dependency、formal verdict or cross-session state needs persistence.

```text
task_name / role / revision
outcome / stopping condition
dependencies / consumers
read_set / write_set
actual exclusive resources
acceptance / evidence / non-goals
```

Task names use lowercase letters、digits 和 underscores. Nicknames are display-only.

## 4. Lease

Lease only actual exclusive resources:

```text
path:<repo-relative-path>
gpu:<id>
isaacsim:<instance>
display:<id>
port:<number>
hardware:<name>
output:<absolute-or-repo-relative-root>
```

Read-only agents do not receive leases. A single writer with no competing owner does not need a file lease unless work must persist across sessions.

## 5. Candidate freeze and verdict

Freeze is required only before formal review/QA or when the candidate would otherwise be ambiguous:

```bash
python .ai/scripts/team_state.py freeze-create ...
python .ai/scripts/team_state.py verdict-add ...
```

After a narrow change:

- bound path/contract/topology changed -> `INVALID`;
- explicitly disjoint -> `RETAINED`;
- uncertain -> `REVIEW_REQUIRED`.

The tool is conservative and does not claim arbitrary semantic dependency inference.

## 6. P2P metadata

Only when coordination state is active, material P2P edges may be recorded. Full message bodies do not need to be mirrored into Main context. Inactive mode leaves ordinary communication unlogged.
