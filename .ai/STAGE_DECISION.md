<!-- managed-by: jam-coding-role; file: STAGE_DECISION.md -->
# Optional multi-planner stage decision workflow

This file is read only when Owner explicitly selects a cross-stage planning workflow. It is not required for ordinary implementation or every training completion.

## Roles

- **Owner**: starts the stage, selects planners, controls budget and approves the final plan.
- **Local planner**: sees local worktree、memory、resolved config、untracked artifacts and runtime constraints; audits feasibility.
- **Cloud insight planner**: reads remote code and an explicitly prepared artifact bundle; proposes novelty、alternatives and experimental design without assuming unbundled local facts.
- **Worker**: implements/runs the approved plan; does not change scientific or release claims unilaterally.

## Optional roster

Owner may choose: Owner only、local only、cloud only、local+cloud independent proposals followed by local synthesis，or another roster.

## Recommended dual-planner flow

```text
stage close explicitly requested
-> code/evidence state fixed
-> artifact handoff only if enabled
-> local independent plan
-> cloud independent plan
-> local feasibility audit and comparison
-> synthesis
-> Owner approval
```

Cloud gates are classified as safety、product release、scientific admission or exploratory diagnostics. A scientific gate blocks only the corresponding claim unless Owner promotes it to a product gate.
