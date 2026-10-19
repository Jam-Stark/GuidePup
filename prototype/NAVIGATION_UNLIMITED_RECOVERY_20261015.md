# One task without skill/correction count or GPT decision-time limits

2026-10-15. Output `.local/outputs/campus_unlimited_recovery_20261015_01/`.

## Result

One new full task ended **STOP_NOT_CONFIRMED**, with no runtime exception.
GPT declared arrival at glass doors, but the independent review found the selected
Entrance A was not reached. Physical stopping also failed. This is **0/1 physical
success**, with a false planner completion claim; it is not a timeout or limit
exhaustion. No automatic retry was made.

| Measurement | Result |
|---|---:|
| Wall / sensor duration | 473.071 s / 169.36 s |
| Physical travel | 12.819 m |
| Final physical XY / yaw | [5.2614, -2.1731] m / -2.2729 rad |
| Distance to selected A, offline only | 17.242 m |
| Final 0.6 s peak speed / yaw rate | 0.1626 m/s / 0.1253 rad/s |
| Observation calls / maximum in one cycle | 8 / 5 |
| GPT calls / longest call | 12 / 22.583 s |
| Correction cycles | 2 |
| Navigation inputs / STOP | 51 / 3 |
| Navigation sessions / initial resets | 1 / 1 |
| MPPI solves / actually executed controls | 143 / 143, no mismatch |
| Force-stop events / maximum estimated horizontal load | 0 / 48.598 N |

## Authorized changes and coverage

Automatic `EXTERNAL_FORCE_STOP` is planner-recoverable through navigation, turn
or advance; hold retains the stop. Explicit human brake remains human-owned.
The shared skill-call limits, correction limit and default GPT's local 30 s
deadline were removed. Counters remain, and individual skill angle/speed/time
bounds remain. Details and narrow checks:
[authority report](NAVIGATION_STOP_AUTHORITY_20261015.md).

The actual task exercised **five observations before its first navigation** and
**eight skills overall**, exceeding both old 2-per-cycle and 6-per-task limits.
It did not exercise a GPT call longer than 30 s, more than three corrections,
or a physical estimated-force STOP/recovery. Those branches cannot be claimed
physically validated. Interface checks accepted a fourth recovery/seventh skill,
third same-cycle observation, automatic release with nonzero emitted command,
and retained explicit caller pause. Synthetic checks are not navigation success.

Task, spawn, scene, camera calibration and controller configuration match the
preceding capped task. The default model remains GPT-6.1 Sol / medium / fast,
HEAD RGB only, GPU3 resident navigation model and GPU4 Isaac/CUDA MPPI.
This one trial changes both authority and limits, so it does not isolate their
individual causal effects. No scene truth enters online planning or control.

## Actual planner and movement flow

GPT first requested turns **+75, -140, -90, -90, -90 degrees**, returning for
more views instead of being stopped at two observations. Actual turns were
approximately +70.55, -138.22, -89.28, -89.68 and -88.89 degrees. Each returned
physically stopped. It then moved toward the open corridor, observed another
right 45 degrees, moved along the corridor, observed left 90 degrees, requested
a two-second hold and approached a glass-door candidate.

This task went west toward the lobby rather than east toward the previous
bright-panel dead end. It still did not cross the open lobby to Entrance A.
The final image shows the corridor-side dark glass facade and the adjoining
lobby. GPT's final stated evidence was near glass double doors/handles and indoor
floor; that did not establish the identity of the selected main entrance.

Both navigation continuations actually moved:

| Navigation segment | Controls sent | Physical travel during navigation |
|---|---:|---:|
| Initial open-corridor approach | 27 | 0.729 m |
| First recovery, bright corridor end | 56 | 2.753 m |
| Second recovery, glass-door candidate | 60 | 1.951 m |

These are actual navigation-phase measurements, not instruction counts or predicted
paths. Overall travel also includes observation and zero-command movement.

## Independent stopping and contact evidence

The seventh skill, a left-90 observation, physically turned 84.86 degrees and
travelled 1.227 m; stopping was unconfirmed. The eighth skill, hold, still moved
0.490 m and changed yaw 5.72 degrees with zero locomotion commands.

After GPT's final completion claim, the existing 10 s stop observation executed
**501 zero-command ticks**, yet physically travelled **1.173 m**. The original
force-driven walking flag remained active during all 501 ticks. This association
does not establish the entire cause of residual movement. The stop window is an
execution outcome check, not a restored task/GPT thinking-time limit.

The offline non-foot net normal-force peak was **1307.72 N on RR_thigh at
127.65 s**. Its observer lacks counterpart identity, contact points and friction,
and may include self-contact/cancellation. It cannot identify a particular wall
or certify clearance. The learned applied-base-force estimate peaked at only
48.598 N and did not activate the unchanged 60 N/two-sample automatic gate.
The gate is not validated as a reliable obstacle-contact detector by this trial.

## Evidence and closure

- Complete HEAD video: `navigation/episode/camera.mp4`, 2541 frames, 15 fps,
  **169.4 s**, 768×272; full ffmpeg decode passed.
- Two primary evaluations: `navigation/analysis/navigation_evaluation.json` and
  `control_evaluation.json`; additional observation, force-stop and diagnostic files.
- English measured trajectory and force timeline, all RGB, raw IMU/joints,
  estimated load/gait, complete planner/model I/O, MPPI candidate/command logs,
  independent physical/contact traces, snapshot and root `results.json`.
- All online/execution/estimated/physical/contact tails match **170.88 s**.

The task tmux, simulator and encoder exited. GPU4 was released; GPU3/PID2454462
remains loaded under the owner's standing request. No training, hardware operation,
driver reset, fallback, Git commit/push or unrequested extra task occurred.
The owner's subsequent request authorizes a separate task-description-only run;
it must not be merged into this trial's result.
