# Campus navigation after the official instruction-policy update

2026-10-15. **One newly authorized full navigation trial. Physical success: 0/1.**
The run ended as `STOP_NOT_CONFIRMED`; GPT declared completion at a different door.
There was no navigation runtime exception, automatic retry or additional full trial.

## Outcome and comparison

| Measure | Previous trial | New trial |
|---|---:|---:|
| Physical arrival at Entrance A | No | **No** |
| Online status | NEEDS_USER | **STOP_NOT_CONFIRMED** |
| GPT completion declaration | No | **Yes, incorrect target** |
| Recorded physical travel | 13.389 m | **48.137 m** |
| Final body-center distance to A, offline | 13.290 m | **34.762 m** |
| Independent physical stop | Yes | **No** |
| Model inputs / STOP outputs | 112 / 4 | **263 / 3** |
| Recovery decisions used | 3 | **2** |
| MPPI solves / commands actually sent | 298 / 298 | **832 / 832** |
| Navigation runtime error | None | **None** |

The new trajectory continued west along the corridor, passed the turn into the main
lobby, and ended at a different glass door at the west end. The requested Entrance A
remains north of the lobby. The final physical pose was
`[-35.1461, -1.3599, -3.1329 rad]`; A's offline reference is `[-3.376, 12.75] m`.
These coordinates were never supplied to online planning or control.

The new rules improved continuation in this instance: the second recovery executed
substantial motion. They did **not** improve task success. GPT's last decision confused
the observation subgoal with the original destination and accepted the wrong door.
The model's internal recognition and STOP causes remain unverified.

## Fixed task and realized setup

- Default E2 elevator-front start to the glass entrance in the main lobby, stopping
  approximately nearby on the indoor side without passing through the doors.
- Same spawn `[12.51, -2.45, 0.55, 180 deg]` and campus USD asset; no scene changes.
- Existing HEAD RGB at 768 x 272 / 132 degrees, existing mount; no depth or online truth.
- GPT `gpt-6.1-sol`, medium, fast, 30-second decisions; maximum three recoveries and
  no whole-task time budget. Existing Codex login and scoped proxy retained.
- Existing navigation model stayed loaded on GPU3. A new client episode used one
  session and one initial reset; both continuations retained history.
- Isaac/MPPI used the then-free physical GPU4. Its CUDA UUID
  `GPU-20093912-98d6-3c89-9517-3ac344e38fc3` matched RTX renderer index 4.
- RGB-only CUDA MPPI trajectory tracking, command ceiling 0.50 m/s; no geometric
  obstacle checking, new movement freedoms or controller tuning.

The wall duration was **358.99 seconds**. Online and independent records span
1.52 to 120.56 seconds, or **119.04 seconds** of task sensor time.

The intervention is the shared instruction policy **plus** richer STOP evidence and
earlier-attempt summaries. This is one trial per condition, not an isolated wording
experiment. GPU4 replaces baseline GPU7 and the log-flush repair is retained. No model
weights were trained or modified. This run cannot establish general success rates.

## Decisions, model response and actual execution

| Sensor time | Event | Subsequent observed result |
|---|---|---|
| 7.08 s | Initial navigation | 383 controls; 18.107 m within NAVIGATING |
| 45.56 s | STOP 1, seq136 | Source at 45.24 s; still 14.492 m from A, facing west |
| 52.32 s | Recovery 1 accepted | 7 controls; 0.095 m, then STOP again |
| 53.34 s | STOP 2, seq139 | Target pointing absent, `visible=false` |
| 59.16 s | Recovery 2 accepted | **442 controls; 27.002 m**, toward the west-end door |
| 103.72 s | STOP 3, seq262 | Interior image point on the other glass door, not A |
| 110.54 s | GPT declares complete | Original destination identity remains unverified |
| 120.56 s | STOP_NOT_CONFIRMED | Estimated stable stop not confirmed in the existing 10-second window |

The actual first recovery instruction was:

> Walk toward the bright opening at the end of the corridor ahead and stop there to observe the lobby entrance. Do not pass through any glass entrance doors.

The second was:

> Approach the bright rectangular opening straight ahead at the end of the corridor and stop before it. Do not pass through any glass entrance doors.

These instructions reached the navigation model unchanged. The first two STOP decisions
explicitly acknowledged uncertain entrance identity and chose an observation goal.
The final decision treated the now-nearby corridor-end glass door as fulfillment of the
original task. A correctly grounded pixel on a visible door did not prove it was A.

All four real GPT requests contained the shared policy. All three STOP requests preserved
the original pointing values and source-frame IDs; the archived model-source JPEGs were
byte-identical to the images passed to GPT. Current RGB was supplied separately with its
own timestamp. Executed-command and estimated-motion summaries were present. Thus the
new evidence and rephrasing path actually ran; the failure is not a missing instruction
or missing pointing handoff. Monitor remained read-only.

## Control, stopping and contact

All **832** MPPI solves produced commands actually sent; none had an accepted/executed
command mismatch. Median CUDA solve time was **3.057 ms**, total **4.607 ms**.
The measured physical peak speed was **0.672 m/s**, despite the 0.50 m/s command ceiling.
The command ceiling is not an assertion that the physical plant cannot overshoot it.

After the last model STOP, all **842** subsequent WBC command ticks were zero. The
independent body trajectory nevertheless accumulated **2.754 m** of movement, including
**1.479 m** after GPT's completion declaration. No nonzero recovery command explains
this final movement. The respective contributions of WBC response, contacts and dynamics
have not been isolated in this run.

In the last 0.6-second physical window, peak planar speed was **0.0986 m/s** and peak yaw
rate **0.2250 rad/s**. Both exceed the retained stop criteria of 0.08 m/s and 0.10 rad/s.
Physical stop therefore failed too; this is not established as an estimator-only false alarm.

The non-foot net normal-force peak was **755.67 N on base_link at 117.78 s**, during
zero-command holding. There were 256 contact samples with a non-foot peak above 10 N.
The observer gives per-body net normal force, without counterpart identity, contact
points or friction. It includes self-contact and possible cancellation. This does not
identify the contact object or certify collision-free travel. The last RGB is largely
occluded by a nearby pale surface.

## Evidence completeness, errors and artifacts

Online events, command execution, estimated state, physical trajectory and contacts
all end at **120.56 s**. The old missing-tail problem did not recur. The full camera
video decoded without errors. The two evaluations and root `results.json` distinguish
GPT's incorrect completion declaration, physical failure and controlled stop failure;
the navigation runtime error is `null`.

Initial offline plotting commands assumed USD Python bindings were available in the
selected environments and failed. The final plot directly draws measured physical XY
traces with Matplotlib, with no scene rendering or fabricated geometry. These offline
tool errors did not affect the navigation or physical metrics.

Evidence root: `.local/outputs/campus_prompt_rules_20261015_01/`.

- `RUN_RECEIPT.json`, `PROCESS_DONE.json`, `console.log`, `run_task.py`, `source_snapshot/`.
- `navigation/episode/camera.mp4` and original RGB, sensor, full model, GPT, MPPI and
  executed-command records.
- `navigation/physics/{ground_truth,contacts}.jsonl`, used only after the run.
- `navigation/analysis/{navigation_evaluation,control_evaluation,stop_response,summary}.json`.
- `navigation/analysis/{trajectory.png,trajectory.svg,physical_trajectory.csv}`.
- `navigation/results.json` and reproducible offline `analyze.py`.

The dedicated task tmux and simulator exited; GPU4 was released. The pre-existing
GPU3 model service remains loaded. No other project's process, proxy, hardware,
training job or Git history was changed. No further trial was started.
