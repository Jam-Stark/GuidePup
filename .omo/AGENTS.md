# ForceControl OpenCode/OMO adapter v1.3

Root `AGENTS.md` and core `.ai/ROLE.md`、`.ai/PROJECT.md`、`.ai/WORKFLOW.md` are canonical.

## Default

- FAST: lead handles directly.
- STANDARD: ordinary `task()` delegation, usually one writer and optional read-only specialists.
- OMO Team Mode is OFF by default and is not required for simple QA、temporary implementation or normal focused work.

## Team Mode trigger

Enable Team Mode only for a real shared task graph、multiple writers with disjoint `WRITE_SET`，or persistent member communication. Retain official task list、mailbox、claim/update and shutdown lifecycle. Do not force `oracle`、`librarian`、`explore`、`metis`、`momus` or `prometheus` into direct team membership when ordinary delegation is the supported route.

## Optional facilities

Read `.ai/TEAM_STATE.md` only if the work also needs repository-level leases/freeze/verdict outside OMO's own team state. Read long-run、memory、scientific、stage-decision and artifact documents only on their root routing triggers.

Lead owns scope、resources、Git、external writes and final verification. Child agents do not commit or push.
