# Planner instruction policy and STOP evidence

Implemented 2026-10-15 18:40 HKT. This update does not add a full navigation trial.

Subsequently, the Owner authorized one new full trial. It continued after STOP but
reached the wrong corridor-end door, with incorrect GPT completion and unconfirmed
physical stopping. See the separate [full-run report](NAVIGATION_CAMPUS_PROMPT_RULES_20261015.md).

## Changes

The actual GPT planner and the existing Pi decision entry explicitly load one English
policy file: `prototype/pi/prompts/lightnav-instruction-rules.txt`. It combines the
official [Prompt Guide](https://github.com/lightorigins/LightNav-0#%EF%B8%8F-prompt-guide)
with our evidence-handling requirements. Prefer a direct action, an identifiable visual
target and one primary cue. Preserve the user's constraints; optional stopping remains
valid. Do not insert hidden coordinates, appearance templates or operator map labels.

The official [protocol](https://github.com/lightorigins/LightNav-0/blob/main/docs/PROTOCOL.md)
defines STOP as a zero predicted trajectory. It does not certify arrival. A contributor
also acknowledged wrong-object grounding in [issue 7](https://github.com/lightorigins/LightNav-0/issues/7#issuecomment-5711723082)
and suggested a distinguishing visible attribute. Neither source proves that prompt
changes alone will resolve this campus trial.

The GPT STOP request now includes:

- The previous navigation action's starting RGB, the exact model-input RGB and current
  HEAD RGB, with separate labels. Model/current frames carry their sensor times.
- Original pointing coordinates, image dimensions, boundary flags, model tokens and
  predicted waypoints, associated with the source frame and navigation session.
- Per-action counts of MPPI controls actually sent and nonzero WBC command ticks.
- Separately labelled estimated travel, displacement and yaw from the existing
  IMU/WBC dead reckoning. These are not physical ground truth or target-relative distance.
- Earlier attempt instructions and progress, bounded by the existing recovery budget.

GPT decides whether to continue, change the description, select an observation goal,
request help or finish. The coordinator forwards the chosen instruction unchanged.
Source-frame pointing is not silently transferred to a newer RGB. Missing evidence,
uncertain target identity and observed incompletion remain different states.

## Pi implementation choice

`PiDecision` uses the installed Pi 0.85.1 `models.complete()` API. It only sees explicitly
supplied prompts; it does not automatically load skills. TypeScript reads the shared
file relative to `import.meta.url`, so both `src/` and compiled `dist/` resolve it.
`CodexPlanner` loads that same file relative to `__file__`. Missing files fail immediately.

This does not claim that Pi owns the current GPT transport. The actual GPT path keeps
the working Codex CLI ChatGPT login and `gpt-6.1-sol`, medium reasoning, fast mode.
The installed native Pi Codex catalog does not contain this exact model or automatically
import the CLI login. Registering a new model/auth path or wrapping the CLI in a Pi
Agent would add unrelated transport work. No credentials or model profiles were changed.

The original LightNav training prompt, visual history/session, nominal frame-time prefix,
30-second decision limit, 3-recovery limit, controller, camera and speed permissions
are unchanged. Monitor remains read-only. No full-task time budget was reinstated.

## Verification

Evidence: `.local/outputs/planner_instruction_feedback_20261015_01/`.

Python import/syntax checks passed. Pi TypeScript build and compiled-module/shared-file
loading passed. The recorded-input script calls the production evidence builders on
the four STOP observations from `campus_rgb_e2_to_entrance_20261015_01`.
It reads only online RGB/model/estimated-state/execution/MPPI records, not simulator truth.

| Recorded STOP sequence | Distinct MPPI commands sent | Nonzero WBC ticks | Estimated travel in this action |
|---|---:|---:|---:|
| 106 | 291 | 1455 | 13.381 m |
| 109 | 7 | 34 | 0.064 m |
| 110 | 0 | 0 | 0.011 m |
| 111 | 0 | 0 | 0.012 m |

Command counts match the original execution records; pointing and source image are
preserved exactly, with one retained navigation session. Small odom changes in the last
two cases are estimates, not evidence of executed navigation or certified physical motion.

Two real GPT calls used the first and second recorded STOP cases. For these probes,
current RGB was deliberately frozen to the STOP source, with its true capture time and
age explicitly disclosed. Action-start RGB was the first recorded model input after
action acceptance. These are frozen-observation probes, not exact replays of the old
planner requests or predictions of future execution.

| Probe | Wall time | Actual GPT action |
|---|---:|---|
| First STOP | 15.75 s | `navigate`; approach the glass entrance straight ahead and stop indoors |
| Second STOP | 17.20 s | `navigate`; approach the glass entrance doors straight ahead and stop indoors |

Both calls explicitly used the bottom-edge/clamped target point and remaining visible
lobby floor as evidence. The second also identified seven sent controls and approximately
0.064 m estimated travel. Neither declared completion. Full English instructions,
evidence, uncertainty, original image bytes, CLI events and receipts are archived in
`replay_contexts/stop_01` and `stop_02`.

The initial verification-script launch had an incorrect repository-parent path; it
failed before any model call. The path was corrected before the checks above.

## Limits and resources

New full navigation tasks: **0**. New navigation-model calls: **0**. New GPT calls: **2**.
No proposed instruction was executed, so this does not demonstrate elimination of
repeated STOP, physical progress, obstacle avoidance or arrival at Entrance A.
No simulator or hardware was started. The pre-existing GPU3 model service remained
loaded (PID 2454462); no other project's process or proxy configuration was changed.

Primary implementation: `navigation/codex_planner.py`, `navigation/engine.py`,
`navigation/decision_evidence.py`, `pi/src/decision.ts` and the shared policy file.
