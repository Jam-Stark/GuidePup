# Planner recovery authority and removal of planner count/time limits

2026-10-15. The owner corrected the earlier implementation: automatic estimated
external-force STOP must be planner-recoverable. Only explicit human brake keeps
human-only recovery authority. The owner then requested removal of observation
and correction count limits and GPT's local thinking-time deadline, followed by
one new default task.

## Authority correction

`robot/force_stop.py:ForceStop` replaces the earlier conflated `ForceBrake` path.
The inherited threshold, persistence, scale, body-yaw frame and sensor source are
unchanged. The new event is `EXTERNAL_FORCE_STOP`. It invalidates old commands,
interrupts the current skill, preserves partial views and reports the load vector,
magnitude/direction, input time and interrupted activity to the planner.

An accepted `navigate`, `turn_observe` or `advance_observe` explicitly releases the
automatic stop and starts that new action. `hold_observe` retains it while collecting
views and returns to the planner normally. `complete` retains it while checking
the original completion/stopping criteria. `needs_user` does not release it.
The runtime no longer restricts this event to `needs_user` or automatically ends
the task as `FORCE_BRAKED`. Release and actual motion remain separate evidence.

The last input timestamp is retained across recovery; old force samples cannot be
counted twice. Two fresh brake-level samples can stop the robot again immediately.
There is no suppression window or forced-forward override. Force episodes and
recovery proposals are retained in `force_stop_history`. Generation checks reject
old queued/in-flight decisions and navigation responses.

Historical `sim/server.py` human-brake authority is unchanged: `HUMAN_PAUSED`,
human-only `/resume`, and rejection of planner motion/completion. The RGB batch
entry adds no manual-brake UI. Releasing the automatic gate cannot clear a caller's
explicit `paused=True`. A force estimate cannot identify its human/obstacle source.
Semantic references remain read-only.

## Removed limits

- Removed `max_calls_per_cycle` and `max_calls_per_task` from the shared contract
  and all runtime count-rejection paths.
- Removed `max_recoveries` from both runtime configs and the correction rejection
  path. Counters remain visible as history, with no maximum.
- Removed default GPT `decision_timeout_s` and the CLI `communicate` deadline.
  The model remains GPT-6.1 Sol / medium / fast. Task shutdown cancels only its
  owned pending CLI child; this is not a normal decision-time cutoff.
- The separate Pi/Qwen API deadline stays unchanged; no new Qwen or Gemini task
  is requested. There is still no whole-task time budget.

Individual observation angle/speed/duration/settling bounds, control freedoms,
camera, scene, WBC, MPPI configuration and 60 N/two-sample force threshold remain.
No additional semantic controller, collision truth input or fallback is added.
API/configuration/schema/runtime errors still fail explicitly.

## Narrow verification

Evidence: `.local/outputs/force_stop_authority_20261015_01/validation/`.

Actual runtime dispatch and the pinned WBC on frozen recorded IMU/joints verified:

- Planner navigation, turn and advance decisions release the automatic gate and
  emit nonzero commands. Hold/complete/help retain its zero-command pause.
- A fourth recovery and seventh skill are accepted; a third observation in the
  same cycle is also accepted.
- A stale queued decision is discarded, and two fresh high-force samples re-stop
  after release without recounting the old sample.
- An explicit caller pause still forces zero command and stationary gait after
  automatic release.

These checks use synthetic trigger/decision inputs, not physical navigation.
Python parsing and shared JSON loading passed. No provider transport dependency or
TypeScript code was changed. The actual automatic-force recovery branch still
requires physical trial coverage; the preceding task peaked below its threshold.

## Original-decision retrospective

The original ephemeral GPT conversations cannot be resumed. Their original online
contexts, exact RGB bytes and declared choices were recovered. A new same-model
read-only review found both choices justified the bright panels as an unconfirmed
observation subgoal after using two cycle observations. Neither recorded rationale
compared that choice against another direction. Both tasks later requested left
turns after STOP. Salience of the panels is a retrospective hypothesis, not a known
internal cause. No scene map/truth was provided to the review, and no robot action
executed. The original 30 s review timeout is preserved; a 90 s offline review
returned successfully before the owner removed the online deadline.

Full review: `.local/outputs/force_stop_authority_20261015_01/RETROSPECTIVE.md` and
`retrospective_response.json`.

## One newly authorized task

Output `.local/outputs/campus_unlimited_recovery_20261015_01/`.
Default E2 elevator area to Entrance A, HEAD RGB only; GPU3 resident navigation
model preserved, GPU4 selected for Isaac and CUDA MPPI. Independent tmux, fresh
output, no automatic retry. The 1200 s ETA is an inspection point, not a task cutoff.
The task finished STOP_NOT_CONFIRMED with a false GPT arrival claim, no runtime
exception, 12.819 m physical travel and no arrival at A. It used 8 skills, including
5 in its initial cycle. Its final report is
`prototype/NAVIGATION_UNLIMITED_RECOVERY_20261015.md`. The prior active-task note
is superseded; the owner's subsequent goal-description run is a separate trial.
