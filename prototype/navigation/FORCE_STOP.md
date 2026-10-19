# External-force STOP and human brake authority

| Signal | Motion response | Recovery authority |
|---|---|---|
| Learned equivalent horizontal base load >=60 N for two distinct 50 Hz samples | Zero locomotion, disable force-driven walking, stationary gait; invalidate reference and interrupt skill | Planner may explicitly choose navigation, turn or advance |
| Explicit human brake / operator pause | Keep operator-owned pause | Human only; planner cannot release it |

An estimated force does not identify a human, obstacle, contact location or source.
The automatic event is `EXTERNAL_FORCE_STOP`, not a human brake. The frame and
scale remain unchanged: aux[3:6] / exported 0.01, yaw-only base coordinates with
+x forward, +y left, +z up. Vertical force is reported but does not trigger this
planar gate. The estimator is not a calibrated contact detector.

## Planner recovery

The plant-owned `ForceStop` keeps all command sources stopped until a new planner
decision. It does not restart when force falls. The runtime sends the trigger's
Newton vector, magnitude, direction/time, interrupted instruction/skill, current
RGB and partial observation views to GPT or Pi/Qwen through the shared contract.

- `navigate`, `turn_observe`, `advance_observe`: an accepted decision releases
  only the automatic stop and starts the new action. Instructions remain the
  planner's own complete text; navigation history is retained.
- `hold_observe`: keeps the automatic stop active while collecting views, then
  returns normally to the planner. It cannot silently end the task.
- `complete`: keeps the stop active and uses the unchanged arrival/stop checks.
- `needs_user`: reports the problem without releasing the stop.

Old queued events and in-flight planner/model responses are invalidated by control
generation. A recovery clears the consecutive-sample counter but retains the last
estimate timestamp. Two NEW brake-level samples can immediately stop the robot
again; there is no grace interval suppressing the signal or automatic forward command.
The stop does not freeze physics or guarantee physical stillness.

The old operator brake remains in `sim/server.py`: `HUMAN_PAUSED`, human-only
`/resume`, and rejection of planner navigation/completion while paused. Its source
and authority are not changed. The RGB batch entry adds no manual-brake UI.
At the adapter boundary, releasing `ForceStop` never clears a caller's explicit
`paused=True`; the operator pause still forces zero command and stationary gait.

## Limits after the owner's update

Observation-call and correction-cycle counts are recorded but have no maximum.
The default GPT CLI has no local thinking-time deadline. The task has no global
time budget. GPT still decides whether to continue, complete or request help.
Each observation retains its existing angle, speed, duration and settling bounds;
the robot gains no new motion freedom. API/runtime/schema errors remain explicit.
Task shutdown cancels only its own pending Codex child process, without a normal
decision deadline. Qwen's separate configured API deadline is unchanged.

`execution.jsonl` records `external_force_stop` ticks, requested/effective command,
gait state and estimator input time. `events.jsonl` records interruption, actual
planner choice and release. `force_stop_history` retains every episode and its
recovery proposal. A release, predicted path or nonzero command is not physical
movement; physical success/contact remain independently evaluated offline.

The preceding 2026-10-15 trial did not trigger the inherited 60 N threshold
(peak estimate 42.409 N). This authority correction does not change that result
or claim that obstacle-contact stopping is solved.
