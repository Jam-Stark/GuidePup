# Astra medium with standard processing — 2026-10-16

## Result

**Astra naturally declared COMPLETE and the robot physically stopped. A genuine
building-exit arrival remains UNVERIFIED.** It ended at the same signed glass
double doors G as the previous Sol task, not the independently verified Entrance A.
No additional full task was launched to obtain a preferred outcome.

| Measurement | This full task |
|---|---:|
| New full navigation tasks | 1 |
| Wall time | 953.611 s |
| Original RGB video | 354.200 s / 5,313 frames / 15 fps |
| Independent physical travel | 13.370 m |
| Astra decisions / completed responses | 28 / 28 |
| Observation skills | 13: 9 turns, 4 forward observations |
| Navigation-model inputs / target probes | 81 / 12 |
| Continuous-navigation STOP / target-probe STOP | 0 / 1 |
| CUDA MPPI solves / executed control bundles | 216 / 215 |
| Command mismatch / executed probe references | 0 / 0 |
| Sessions / initial resets | 1 / 1 |
| Task runtime error / controlled NEEDS_USER | None / No |
| Planner completion / actual stable stop | Yes / Yes |
| Genuine exit success / false completion | Unknown / Unknown |

All 28 actual full-task requests passed the no-Fast audit: explicit
`gpt-6-astra`, `medium`, `service_tier="default"`, `--disable fast_mode` and
`--ignore-user-config`. All returned usable responses. The separate successful
connection check is excluded from the full-task and success counts.

## Owner authorization and question

The Owner authorized trying **Astra medium** and explicitly prohibited Fast mode.
This is a new full navigation task using the previous task and physical parameters.
It is not authorization for training, hardware, a batch sweep or Git publication.

The question is whether the new planner naturally completes the closed-door
building-exit task, and whether independent evidence establishes actual arrival
and stable stopping. A planner completion claim is scored separately from physical
success. The task remains **“Find the building exit and stop near it.”**

## Exact model and processing configuration

The independent profile `prototype/navigation/astra-config.json` selects:

```
model = gpt-6-astra
reasoning_effort = medium
service_tier = default
```

Every actual Codex command explicitly sets the model, effort and service tier,
uses `--disable fast_mode` and `--ignore-user-config`, and has no Fast enable flag.
The shared adapter derives Fast mode only from an explicit `service_tier=fast`;
the task launcher requires the Astra/medium/default tuple before launching Isaac.
No automatic model or controller fallback exists.

Official references: [Astra model](https://developers.openai.com/api/docs/models/gpt-6-astra),
[Codex configuration](https://learn.chatgpt.com/docs/config-file/config-reference),
[Fast mode](https://developers.openai.com/api/docs/guides/fast-mode).
The local Codex binary is 0.160.0. It supports `fast_mode` but has no
`ultrafast_mode` feature. An initial local check rejected that unsupported flag
before inference (empty event log); the error is retained. The actual configuration
requests standard/default, not priority, fast or ultrafast.

The CLI JSON does not expose the server service-tier echo or billing record.
Recorded proof therefore establishes the exact executed command and explicit
standard request, without claiming an independently observed billing tier.

## Admission and fixed parameters

One real Astra vision/action-schema call used the exact initial RGB and online
context from the previous Sol run. It completed in **20.732 s**, chose a right
60-degree observation, and did not execute motion. The same-image Sol/fast call
took 19.144 s. This is a connection check, not a navigation success or a latency
benchmark. See `PREFLIGHT.json` and `preflight_02/`.

The new config differs from the previous manifest only in planner model and
service tier. E2 start, closed MF-R1 scene, HEAD RGB, shared Fusion state, complete
decision/handoff history, action contract, observation authority, navigation model,
GPU MPPI, WBC checkpoint, speed bounds and 60 N stop threshold are retained.
There is no task/GPT thinking deadline or skill/recovery count limit. No depth,
map, target coordinates or scene semantics are sent online.

The changed model and service tier are evaluated together in one task. This is
not a model-only causal comparison, and one run cannot establish general superiority.

## Full task — completed

Output: `.local/outputs/campus_astra_medium_standard_20261016_01/`.

Command:

```bash
bash prototype/run.sh \
  --config prototype/navigation/astra-config.json \
  --sim-gpu 4 \
  --output .local/outputs/campus_astra_medium_standard_20261016_01/navigation
```

Independent tmux: `guidepup-astra-medium-standard-20261016-01`.
GPU3's existing navigation model is reused; GPU4 is allocated to Isaac/MPPI.
ETA is 60–75 minutes based on the previous 52-minute task and the initial measured
connection latency. The registered ETA was a review point, not a kill limit.
The durable waiter returned early on the normal completion event after 953.611 s.

## Actual decision and motion sequence

1. Astra used four left-turn observations, read exit-direction signs, and chose a
   recessed yellow-framed doorway as an observation subgoal. It explicitly did
   not claim that subgoal was the building exit.
2. One proposed target was rejected by Astra after reviewing the original JPEG
   and the model's pointing. A revised yellow-door proposal was confirmed.
   The only continuous-navigation segment then physically travelled **7.997 m**,
   executing 215 MPPI control bundles. There was no MPPI infeasibility or solver
   exception; bundle 216 was preempted by the force-stop authority.
3. At sensor time **125.22 s**, the learned horizontal applied-base-load estimate
   triggered the inherited 60 N stop. Astra received the vector, direction and
   interrupted navigation. It chose a left-turn observation to inspect the hall.
4. Astra then selected the signed glass doors G as an exit candidate. Several
   target handoffs were rejected because the model pointed at adjacent glazing
   or returned no object point. Rewording and small observation turns did not
   consistently fix that alignment. These proposed probe trajectories were
   **never executed**.
5. The only model STOP was target probe **seq73**, captured at 192.64 s and
   received at 200.84 s. Its actual capture pose was approximately
   `[4.984, -0.492, -2.052]` in offline world XY/yaw. Astra confirmed the target
   identity but did not accept STOP as arrival. The framework reported the
   confirmed probe's STOP back as MODEL_STOP; it did not run a new continuous
   trajectory or force that STOP into forward motion.
6. Astra requested a closer threshold target, rejected further mismatched probes,
   and used four existing 0.15 m/s, 3 s forward-observation skills. Their actual
   travelled distances were **0.492, 0.497, 0.494 and 0.490 m**. These were real
   WBC-executed observation motions, with navigation MPPI suspended, not model
   predictions or successful resumed continuous navigation.
7. Decision 28 naturally declared COMPLETE after the final new view. It cited
   the enlarged double doors, previous exit sign, executed progress and estimated
   stopping, while noting that the outside remained hidden.

All 13 skills returned physically stopped under the existing independent stop
criterion. One requested 60-degree recovery turn reported ANGLE_NOT_REACHED;
it actually turned 56.797 degrees and was not silently marked as fully reached.
The largest per-skill IMU yaw-change error was 0.240 degrees in this task. This
does not validate long-term translation or replace a calibrated odometry system.

## Physical scoring and exit identity

Both evaluators ran on the original physical trace and online execution records.
Final actual pose was **`[4.233, -1.952, -1.871]`** in offline world XY/yaw.
The base was about **2.298 m** from the G door plane. This distance is descriptive;
no new metric arrival threshold was introduced. The last 0.6 s peak physical speed
was 0.0000645 m/s and peak absolute yaw rate was 0.000292 rad/s: **actually stopped**.
All five online/execution/estimate/physical/contact tails end at 355.64 s.

Final RGB `rgb_005187.jpg`, the physical endpoint and the prior source-USD audit
identify G, not A. G genuinely has a green EXIT sign and handles. Its source
`OfficeGlazing/Foyer_DoorLeafGlass0–1` is closed static glass; the existing audit
did not establish an exterior connection behind it. That makes its genuine
building-exit role **unverified**, rather than proven true or proven false.
The final distance to A was 16.555 m, an offline reference only. The current task
does not require outdoor crossing and is not automatically failed for staying
indoors. See the prior `EXIT_IDENTITY_AUDIT.md` and current `POST_RUN_REVIEW.json`.

Consequently `physical_success=null` and `false_completion=null`. A verified stop
and an Astra completion declaration are reported separately from genuine exit
success. No scene coordinates, USD identities or offline measurements selected
online actions.

## Observed force-stop and recovery

The trigger estimate was **[-59.607, 11.739, 0.137] N** in the base-yaw frame,
horizontal magnitude **60.752 N**, direction **168.859 degrees** from forward
toward left. The task's horizontal estimate peak was 62.227 N. No artificial
pull or human brake was injected.

From command stop at 125.22 s to explicit Astra resume at 134.56 s, all **467**
control ticks had zero commands, no walking and no force-walking. Actual initial
planar speed was about 0.292 m/s. The first qualifying 0.6 s physical stable-stop
window began 0.790 s after the zero command. During the entire latched interval
the actual travelled path was 0.149 m, net displacement 0.0525 m. These measurements
do not assume zero command freezes physics.

Astra resumed with a left 60-degree observation. That first recovery actually
executed **552 nonzero ticks**, turned **56.797 degrees**, travelled 0.396 m and
returned stopped. This task therefore exercises the dynamic threshold stop and
explicit planner-resume branch in simulation. It does not establish calibrated
collision detection or prove which object caused the load.

The offline non-foot contact observer peaked at **2358.716 N on FR_thigh** at
124.725–124.730 s, before the learned-force latch. This is per-body net normal
force, without counterpart identity, contact point or friction. It may include
self-contact and cancellation. It is not interchangeable with the learned
applied-base-force estimate and cannot certify collision-free motion. RGB-only
MPPI remains a trajectory tracker with speed constraints, without depth-based
geometric obstacle avoidance. Human-brake recovery, a second force interruption,
and failed-solver branches were not exercised in this task.

## Comparison scope

The preceding Sol medium / Fast task took 3120.557 s, travelled 35.615 m and used
87 planner calls / 65 observation skills. This Astra medium / standard task took
953.611 s, travelled 13.370 m and used 28 calls / 13 skills. Both naturally
declared completion, physically stopped and ended at G, whose exit identity is
unverified. This one task did **not** resolve the endpoint-identity question.
The model and service tier changed together; different closed-loop observations
followed. These timings and counts do not isolate model capability or establish
general superiority.

## Evidence and closure

Under `.local/outputs/campus_astra_medium_standard_20261016_01/`:

- `NO_FAST_AUDIT.json`: all 28 actual commands and completed responses.
- `results.json`, `ATTEMPT_ACCOUNTING.json`, `POST_RUN_REVIEW.json`.
- `navigation/analysis/navigation_evaluation.json` and `control_evaluation.json`.
- `navigation/analysis/handoff_evaluation.json`: original-JPEG association,
  standing during review, no probe execution, all invariants passed.
- `navigation/analysis/trajectory.{png,svg}`, `physical_trajectory.csv`,
  `force_timeline.{png,svg}` and full physics/contact traces.
- `navigation/episode/planner/`: actual prompts, original images, commands and
  complete responses. Navigation inputs/outputs and MPPI/execution logs are kept.
- `navigation/episode/camera.mp4`: unmodified full video; full decode passed.
- `navigation_review_10x.mp4`: 35.433 s English review, with outcome labelled
  COMPLETE / Actual stop YES / Exit UNVERIFIED.
- `source_snapshot/`, `STARTUP_CHECK.json`, `PREFLIGHT.json`, process and wait receipts.

Owned simulation, planner, experiment tmux and rendering processes ended; GPU4 is
released. GPU3's external resident navigation-model PID2454462 / port8050 remains
loaded. Other projects and the scoped Codex proxy were not changed. Memory records
the result and limitations. There was no extra full task, training, hardware or
Git publication.
