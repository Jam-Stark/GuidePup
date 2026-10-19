# Planner-controlled observation skills and campus navigation

2026-10-15. The Owner requested executable observation skills first, then a full
default navigation task, followed by answers about MPPI inputs and learned odometry.

## Implementation

The default planner remains Codex GPT-6.1 Sol / medium / fast. GPT structured
actions and Pi/Qwen named tool calls share `navigation/planner-contract.json`,
`planner-system.txt` and the same runtime parameter validation and executor.
The new optional Pi backend reuses the existing lab configuration; no default
Qwen profile, Codex login or proxy service was changed.

| Observation skill | Parameters | Runtime behavior |
|---|---|---|
| `turn_observe` | Signed 10–180°; positive left, negative right | IMU-relative yaw feedback, forward command zero, yaw command at most 0.30 rad/s |
| `advance_observe` | 0.10–0.20 m/s, 0.5–4.0 seconds | Short straight command with bounded ramp-in/ramp-out |
| `hold_observe` | 0.5–4.0 seconds | Zero command and time-ordered observation |

The skill executor owns WBC commands exclusively while active; navigation inference
and MPPI tracking are suspended. It is an explicit observation action, not a
fallback navigation controller. Existing forward/yaw freedoms and acceleration
limits remain. There is no lateral or reverse command and no new geometric
collision check. An in-place-turn request means **zero commanded translation**;
it does not guarantee that the physical body center stays fixed.

HEAD RGB remains captured at 15 Hz; observation samples are selected every 0.5
sensor seconds and at the end. Up to 12 chronological views form a labelled
contact sheet. All raw RGB and sample references are preserved. The next planner
request receives the last two sheets, current RGB, the correctly labelled old
navigation-source image/pointing and execution/estimated-motion reports.
These are contact sheets, not calibrated panoramic stitches or metric maps.

Skills have finite execution/settling limits. Angle failure and unconfirmed
stopping are returned explicitly; zero command is never called physical stillness.
Two calls are allowed per decision cycle and six per task. A recovery cycle can
contain observations followed by navigation; it consumes one of the existing three
recoveries. The 30-second decision deadline remains, with no full-task time limit.
The semantic reference/Monitor remains read-only and chooses no actions.

Details: [Observation skill interface](/home/baoquanc/workspace/GuidePup/prototype/navigation/OBSERVATION_SKILLS.md).

## Narrow validation

- Python parsing and Pi TypeScript build passed.
- Actual GPT and actual lab Qwen each proposed the requested `turn_observe +30°`
  through their real configured interface. Both proposals were archived without
  robot execution; valid and invalid motion-parameter contracts were checked.
- The Qwen request recorded exact ID `Qwen3.6-35B-A3B-FP8/`, the lab endpoint,
  thinking off and 2048 output tokens. No new local language model was loaded.
- Evidence: `.local/outputs/campus_observation_skills_20261015_01/validation/`.

## Interrupted attempt 01

Output: `.local/outputs/campus_observation_skills_20261015_01/`.
This was a real task attempt and is retained in the accounting.

GPT chose two right-90° observation turns, then an approach to the visible corridor
end as an explicitly unconfirmed observation subgoal. Both turns actually executed.

| Skill | IMU-estimated yaw | Independent physical yaw | Physical travel during skill | Physical stop at return |
|---|---:|---:|---:|---|
| Right 90°, first | −89.40° | −89.39° | 0.296 m | Yes |
| Right 90°, second | −88.42° | −88.35° | 0.436 m | Yes |

The total physical path was 0.803 m, remaining in the elevator area. The task then
failed with `ClientConnectionResetError: Cannot write to closing transport` before
its first navigation-model image or MPPI control. It was not a model STOP, MPPI
refusal, semantic completion, or navigation success. Both independent evaluations,
results, the 15-Hz video, contact sheets, physical/contact records and source snapshot
are saved. Trace tails are complete.

The resident WebSocket service closed the connection about 40 seconds after login.
Its installed server uses default 20-second ping interval + 20-second timeout;
aiohttp answers PING while receiving, and the old client did not receive during
planner/observation idle. The client now maintains one continuous reader, queuing
JSON replies while processing heartbeat traffic. A TaskGroup propagates actual
closure/errors. No reconnect, fake frame, inference retry or session reset was added.
A real-service idle check passed after **45.007 seconds on the same connection**,
with no model inference or robot execution. The model service was never reloaded.

Diagnosis: attempt 01 `TRANSPORT_DIAGNOSIS.md` and `validation/socket_idle_check.json`.

## Full navigation after the transport repair

Output: `.local/outputs/campus_observation_skills_20261015_02/`.
The run starts separately from the original selected E2 spawn and original Entrance A
task after the repair. It completed as **NEEDS_USER**, with no runtime exception and
no GPT completion claim. **The requested entrance was not reached and physical
stopping was not achieved.** Both actual task attempts are reported; neither succeeded.

| Measure | Interrupted attempt 01 | Navigation after repair, attempt 02 |
|---|---:|---:|
| Online outcome | ERROR | **NEEDS_USER** |
| Runtime error | WebSocket closing transport | **None** |
| GPT completion claim | No | **No** |
| Independent arrival at Entrance A | No | **No** |
| Independent physical stop | Yes | **No** |
| Physical path length | 0.803 m | **12.218 m** |
| Final distance to A, offline only | 22.287 m | **30.079 m** |
| Observation skills actually executed | 2 | **4** |
| Navigation-model inputs / STOP | 0 / 0 | **53 / 1** |
| MPPI solves / commands sent | 0 / 0 | **150 / 150** |
| Navigation session / initial reset | 1 / 1, no image sent | **1 / 1** |

Attempt 02 ran for 304.78 wall seconds. Its recorded sensor interval is 1.52–102.80 s.
The full native camera video contains 1520 frames at 15 Hz, approximately 101.33 s.
Both attempts' complete videos decoded without errors. Independent physical,
contact, estimated-state and executed-command traces end at the online end time.

### Realized planner and execution flow

1. GPT requested right 90°, then right 180° to inspect additional directions.
   The runtime executed both and returned chronological contact sheets.
2. GPT selected a **leftward corridor approach toward bright panels**, explicitly
   as an observation subgoal, with target identity still unconfirmed. The navigation
   instruction reached the model unchanged. MPPI executed 150 commands; physical
   travel during NAVIGATING was 7.546 m.
3. The navigation model issued its only STOP at sensor time 54.90 s, seq52, pointing
   into the bright panels in its original source image. The independent distance
   to A at that exposure was 28.409 m. STOP did not identify the requested entrance.
4. GPT rejected completion and requested left 90°, then another left 90° to inspect
   the surroundings. It actually received each applicable contact sheet; archived
   image bytes match those produced by the skill executor.
5. The last skill reported `motion_outcome=ANGLE_NOT_REACHED` and
   `status=STOP_NOT_CONFIRMED`. GPT requested help at 102.80 s, stating that neither
   the entrance identity nor successful turn/stopping was established.

This ended by GPT choice, **not a recovery-limit rejection**: one of three recovery
cycles and four of six observation calls had been used. Both observation calls
within the current recovery cycle were used. There was no automatic instruction, semantic Monitor
veto, forced forward command, fallback controller or additional trial afterward.

| Skill request | Estimated final yaw change | Independent physical yaw change | Physical travel | Physical stop at return |
|---|---:|---:|---:|---|
| Right 90° | −89.53° | −89.51° | 0.286 m | Yes |
| Right 180° | −178.03° | −177.99° | 0.738 m | Yes |
| Left 90° | +83.14° | +83.11° | 1.022 m | No |
| Left 90° | +15.27° | +4.98° | 1.048 m | No |

All requested skills produced actual command ticks. Their commands stayed within
the existing forward/yaw freedoms and observation limits; no translation command
was sent for a turn. The resulting physical movement shows that zero translation
command did not hold the body center fixed. The last skill also shows that the
uncorrected orientation estimate can be wrong: its final relative yaw differs by
about 10.3° from the independent measurement. This run does not isolate that
estimation error's cause or establish a general estimator accuracy.

The returned sheets contain 14, 24, 20 and 36 sampled raw frames, respectively;
each displayed sheet uses at most 12 chronological views. No artificial RGB frame,
depth or truth pose was fed online. The single WebSocket/session remained alive
through observation and planning; no transport closure was recorded after repair.
The navigation model was not resumed after the last two observation calls because
GPT chose help, so that particular post-STOP resume branch was not exercised here.

### Why did it not return along the corridor?

Once an observed corridor end does not identify the requested entrance, returning
to inspect the other branch is a reasonable search strategy. The two successive
left-90° observation requests started looking toward that return direction, but
their physical turns were only about 83° and 5°; the robot did not complete the
intended turn-around. GPT then chose help based on unachieved turning and unconfirmed
stopping, rather than issuing a return-route instruction to the navigation model.
Two recovery cycles remained available. No coordinator or Monitor prohibited
return navigation, and no evidence established that both corridor directions
had been searched.

This exposes two separate limitations: reliable stopping/turn execution, and an
explicit planner-owned search record that distinguishes checked dead ends from
unvisited branches and chooses a return route. Such a record should inform GPT's
decisions, not become another automatic semantic stage supervisor. No new return
policy or additional experiment was silently added after the recorded run.

### Stopping and contact

The final 0.6-second physical window has peak planar speed **0.1473 m/s** and peak
yaw rate **0.2988 rad/s**, exceeding the retained stop criteria. There is no claim
of physical stopping merely because GPT chose help or commands were zero.

After observation 3, all 373 command ticks while GPT was deciding were zero,
yet the body traveled 0.770 m. After observation 4, all 355 decision-wait ticks
were zero, yet travel accumulated another 0.569 m. These are measured physical
path lengths, including motion in place and oscillation, not net displacement.
They do not establish the respective contributions of WBC gait logic, contact
or dynamics. The existing WBC and its zero-command behavior were not repaired
as part of this observation-interface change.

The non-foot net normal-force peak was **1386.58 N on RL_thigh at 98.02 s**, during
the final zero-command planner wait. There were 2106 contact samples with non-foot
peak above 10 N. The observer reports per-body net normal forces, without contact
counterpart identity, points or friction; it includes self-contact and possible
normal-force cancellation. It cannot identify a particular wall/door collider or
certify collision-free travel. Close-wall RGB, unachieved turning and contact signals
co-occur, but their detailed physical causal chain remains unresolved.

GPT only saw the online estimated +15.27°, unachieved-angle and unconfirmed-stop
reports. The independent +4.98° and contact forces were never fed into its decision.
An instantaneous low-speed reference and a sustained 0.6-second stable-stop result
also have different meanings; one does not contradict the other simply because
their Boolean values differ. This distinction remains material to interpretation.

Task, scene, camera, navigation model/weights, MPPI configuration and WBC export
remain unchanged. The model stays resident on GPU3, while Isaac and MPPI use GPU4
with checked CUDA UUID / RTX renderer correspondence. No simulator truth, depth or
operator target coordinates are supplied to planning or control. No training or
hardware operation is authorized or performed.

The simulator and task tmux exited, GPU4 is free, and the existing GPU3 model
process (PID 2454462, port 8050) remains loaded. Other projects and the Codex-only
proxy were untouched. No Git commit/push was made, and no further full experiment
was started after attempt 02.

### Coverage and evidence

`turn_observe` and observation-to-planner image delivery ran physically. Both GPT
and Pi/Qwen interfaces made real model calls. `advance_observe` and `hold_observe`
passed parameter validation/build but were **not requested in the physical task**;
their movement/settling branches are not claimed as closed-loop validated.
The new task does not establish general task success, robust stopping or a hardware
deployment result. The interrupted attempt is retained, not excluded from reporting.

Attempt 02 preserves:

- `RUN_RECEIPT.json`, `PROCESS_DONE.json`, `run_task.py`, `source_snapshot/`.
- `navigation/episode/camera.mp4`, all raw RGB/sensors, full planner/model input and
  output, MPPI candidate costs/accepted controls and executed commands.
- `navigation/episode/observation_skills/skill_NNN/{request,result,samples}.json`
  and the actual contact sheets supplied to GPT.
- `navigation/physics/{ground_truth,contacts}.jsonl`, for offline evaluation only.
- `navigation/analysis/{navigation_evaluation,control_evaluation,observation_evaluation,planner_wait_motion}.json`.
- `navigation/analysis/{trajectory.png,trajectory.svg,physical_trajectory.csv}`
  and root `navigation/results.json`.

## MPPI input and learned odometry

Source-backed notes are saved in attempt 01 `DESIGN_NOTES.md`.
MPPI does not inherently require depth; obstacle-aware optimization requires an
obstacle representation and corresponding cost/constraint. The current GPU MPPI
only tracks paths and has no geometric obstacle input.

Learned leg-inertial odometry and RGB+IMU VIO are both established research routes.
The current A2 export already supplies an auxiliary velocity estimate, but the RGB
path simply integrates it with gyro and has no visual correction or bounded drift.
Improved motion history could aid planner judgment, but path length, displacement
and landmark identity must remain distinct. No new odometry model was trained here.

Primary references:

- [Nav2 MPPI mechanism and obstacle critics](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md).
- [Legolas: Deep Leg-Inertial Odometry](https://proceedings.mlr.press/v270/wasserman25a.html).
- [VINS-Mono](https://arxiv.org/abs/1708.03852).
