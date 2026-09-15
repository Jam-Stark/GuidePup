<!-- managed-by: jam-coding-role; file: RUNTIME_ADAPTERS.md -->
# Runtime adapters

## Codex MultiAgentV2

- Root `AGENTS.md` is the repository entrypoint; roles remain in `.codex/agents/*.toml`, model/effort/concurrency remain in `.codex/config.toml`.
- Main owns scope、acceptance、write/resource ownership、Git 和 final integration.
- `send_message` / `followup_task` may carry P2P facts and bounded requests by task path.
- `.codex/hooks.json` is adaptive: when coordination state is inactive it permits ordinary spawns and does not create a ledger. When Main explicitly activates coordination, hooks validate only the applicable controlled contracts and record metadata.
- Do not require a persistent contract for every read-only researcher、planner or small implementation spawn.

## OpenCode / OMO

- `opencode.json.instructions` loads only the core routing files and `.omo/AGENTS.md`; optional `.ai/*` documents are read when their trigger applies.
- Ordinary delegation and OMO Team Mode remain separate.
- Team Mode is OFF by default and is enabled only for real multi-member coordination, especially multiple writers or persistent shared tasks.
- `task(category=...)` and `task(subagent_type=...)` remain distinct routes. Do not force ineligible specialists into Team Mode.

## Standalone Claude Code

- `CLAUDE.md` imports the root/core policy.
- Standalone Claude remains a single-agent local planner/implementer; `.claude/settings.json` denies the Agent tool.
- Claude-family models used inside OMO are governed by OMO and do not change the standalone lane.

## Shared rules

- Universal behavior and Chinese expression rules live in `.ai/ROLE.md`.
- Project facts live in `.ai/PROJECT.md`.
- Optional facilities are route-triggered, not globally preloaded.
- If a runtime lacks P2P、upload、wake 或 hardware tools, use the available path and state the limitation rather than imitating unsupported capability.
