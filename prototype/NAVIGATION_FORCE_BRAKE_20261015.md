# Learned base-force STOP and one campus navigation trial

2026-10-15. Output: `.local/outputs/campus_force_stop_20261015_01/`.

## Outcome

The requested force-brake authority and planner feedback are implemented. The
single new default navigation task ended **NEEDS_USER**, with no navigation
runtime exception and no false completion. It travelled **11.713 m**, did not
reach Entrance A, and was not physically stopped at the end.

**The new force brake did not trigger.** The largest evaluated horizontal force
estimate was **42.409 N**, below the inherited 60 N threshold. There were therefore
zero actual force-interruption planner calls or physical braking observations.
Synthetic authority checks passed, but this trial does not validate force-triggered
stopping during real obstacle contact. The collision/stop problem is not solved.

## Implemented responsibility boundary

The planner still owns task intent, observation actions, navigation instructions
and semantic completion. The semantic Monitor/reference remains read-only.
The new gate is plant motion authority, using only the existing WBC estimator.
It does not select a semantic route, infer task completion or read contact truth.

1. The pinned WBC produces six auxiliary values. Force channels 3:6 are divided
   by exported `base_force_scale = 0.01`, yielding the learned equivalent applied
   base-force vector in Newtons. Its frame is yaw-only base: +x forward, +y left,
   +z up. This is the training-label frame, not a contact position.
2. The historical human brake required x <= -60 N for two consecutive control
   samples while commanded forward velocity was at least 0.2 m/s. The new gate
   keeps 60 N and two distinct 50 Hz samples, extends direction to horizontal
   magnitude, and removes movement-type eligibility. Navigation, turns, advances,
   holds, planner waiting and completion stopping all remain subordinate to it.
   Vertical force is reported; vertical-only loads and torques do not trigger it.
3. The adapter checks the last available estimate **before** constructing WBC
   command/gait input. It latches zero locomotion, clears `force_walking`, resets
   phase and uses stationary gait `[0, 1]`. This is the old pause path, not a
   physics freeze, zero motor torque or guaranteed instantaneous physical stop.
   The estimate's input time and age at command STOP are retained.
4. The coordinator invalidates the path/control generation and interrupts an
   active observation with `INTERRUPTED_FORCE_BRAKE`, preserving partial views
   and executed-command counts. Stale model/decision responses cannot revive it.
5. A new `FORCE_BRAKE` planner request contains the estimated vector, magnitude,
   angle, coordinate convention, interrupted phase/instruction/skill, current RGB
   and recent observation sheets. The latch follows historical human-only resume
   authority. No planner action clears it. Other action proposals are logged as
   denied. This batch entry ends after reporting and bounded stop observation;
   it adds no automatic resume or hardware interface.
6. Normal 0.08 m/s / 0.1 rad/s thresholds and a 0.6 s stable window are retained.
   The force-stop observation is bounded at 10 sensor seconds, while the pause
   stays active until its planner report finishes. Runtime/model errors propagate.

Implementation: `prototype/robot/force_brake.py`, `prototype/isaac/client.py`,
`prototype/navigation/{engine,observation_skills}.py` and shared `planner-system.txt`.
Protocol: [FORCE_BRAKE.md](/home/baoquanc/workspace/GuidePup/prototype/navigation/FORCE_BRAKE.md).
GPT and Pi/Qwen share the rule and enforcement. The actual new trial uses the
unchanged default GPT-6.1 Sol / medium / fast backend, not a new Qwen run.

## Estimator contract and narrow validation

The export returns predictions verbatim. Training targets are scaled base linear
velocity followed by `base_forces_local(... use_applied=true)`. The force target
is the applied base-force command transformed by inverse yaw, not per-link
obstacle contact force, contact point or object identity. Relevant sources:

- `scripts/mujoco/runtime.py:622` and `.local/assets/a2-old/policy.json`;
- `.local/assets/a2-old/training-origin/params/env.yaml`, prediction observations
  around 1067–1119; the configured force curriculum does not establish a measured
  training distribution or obstacle-contact calibration;
- `prototype/sim/server.py:631` and its human-only `/resume` authority.

The estimator consumes a 32-frame sensor/history stack. There is no new neural
training, low-pass filter, collision truth input or threshold retuning.

The prior failed observation-skills task was replayed through the exact pinned
WBC using only saved IMU, joints, commands and initialization. All **5065 control
ticks / 20261 sensor packets** aligned, and the reconstructed auxiliary velocity
matched the recorded values with **zero error**. Its horizontal estimate peaked
at **50.586 N**; no hypothetical brake occurred. Thus even the preceding failed
turn would not be intercepted by this inherited threshold on its recorded inputs.
Frozen replay does not predict the trajectory after changing control.

Narrow synthetic checks established two distinct samples, all four horizontal
directions/diagonal, no duplicate-sample counting, persistence after load release,
and no vertical-only/nonconsecutive trigger. Fixed recorded sensor inputs plus
the actual WBC export verified effective zero command, force-walking off and
stationary gait for navigation, turn, advance and decision hold. A synthetic
planner scheduling check verified stale-proposal discard and denial of an explicit
latch-bypass proposal. These are interface checks, not physical braking tests.

Evidence: `validation/{replay_summary,authority_checks}.json`,
`validation/prior_force_trace.csv`, `replay_force.py`, `validate_authority.py`.
Python parsing passed. No provider transport or TypeScript source was changed.

## Single actual task

Task, spawn, scene, camera calibration and configuration match the preceding run.
The scene is MF-R1, E2 elevator area to the main-lobby glass entrance, stopping
nearby indoors. HEAD RGB only; no depth, online map or simulator truth.
GPU3/PID2454462 remained the resident navigation model. GPU4 CUDA UUID
`20093912-98d6-3c89-9517-3ac344e38fc3` matched renderer 4.
No global proxy was used; the existing Codex child proxy remained unchanged.

| Measurement | Result |
|---|---:|
| New full tasks / retries | 1 / 0 |
| Wall time / sensor interval | 284.598 s / 1.52–95.40 s |
| Online status / runtime error | NEEDS_USER / none |
| Planner completion claim | No |
| Actual task arrival / stable stop | No / no |
| Physical path / final distance to A | 11.713 m / 30.086 m |
| Final physical XY / yaw | [22.3616, -2.8294] m / 1.5557 rad |
| Final 0.6 s speed / yaw-rate peaks | 0.0741 m/s / 0.3194 rad/s |
| MPPI solves / controls executed | 127 / 127; no mismatch or rejection |
| Navigation-model inputs / STOP | 45 / 1 |
| Sessions / initial resets | 1 / 1 |
| Planner calls / recovery cycles | 6 / 1 of 3 |
| Observation calls | 4 of 6; last cycle 2 of 2 |
| Force-brake triggers / actual report calls | 0 / 0 |

Actual GPT flow:

| Sensor time | Decision / result |
|---|---|
| 7.14 s | Left 90° observation; actual +88.47°, stopped at return |
| 22.16 s | Left 90° observation; actual +84.84°, stopped at return; angle not fully met |
| 35.98 s | Navigate toward bright panels as an explicitly unconfirmed observation subgoal |
| 48.92 s | Model STOP seq44; source image at 48.58 s |
| 57.18 s | GPT requested 1 s `hold_observe`; zero commands plus settling, stop not confirmed |
| 71.32 s | GPT requested left 90°; actual +86.82°, stop not confirmed |
| 95.40 s | GPT requested user help: target unidentified, two observation stops unconfirmed |

The actual full navigation instruction was:

> Walk toward the bright panels at the end of the corridor ahead and stop before them. Stay indoors and do not pass through any entrance doors.

The final help decision came from GPT. Neither a Monitor stage gate, recovery
exhaustion, MPPI refusal nor a force latch caused it. No post-STOP navigation
continuation executed. All six real GPT inputs contained the new force reference,
coordinate convention and brake policy; none was a real `FORCE_BRAKE` event.

## What the force and contact evidence means

The peak evaluated estimate occurred at sensor input **42.32 s**, while MPPI
commanded vx=0.50 m/s. Vector **[-41.973, +6.070, +0.100] N**, horizontal magnitude
**42.409 N**, angle **+171.77°**: force toward the rear and slightly left. This is
an estimated force direction on the robot, not a known contact side or location.
The threshold was never reached, even once; the latch correctly remained false.

Offline non-foot contact observer peaks were **3535.78 N on FR_thigh at 41.825 s**
and **1283.65 N on base_link at 75.44 s**. These are per-body net normal forces
from PhysX, without counterpart identity, contact points or friction, and may
include self-contact/cancellation. They are not the same quantity as the learned
applied-base-load estimate and cannot supply its calibration. They were never
available to the online gate or planner.

Zero command remained distinct from stopping: `hold_observe` executed **200 zero
ticks**, yet physically travelled **0.365 m** and changed yaw **7.31°**. Across
the task, 940 zero-command ticks still had the original force-driven walking gait
active below the brake threshold. The final tick had stationary gait, but the
independent last-0.6-second stop test still failed. No claim is made that all
residual motion comes from that gait rule alone.

This is evidence that the existing estimator plus the inherited 60 N trigger
does **not** provide a reliable obstacle-contact STOP in this task. Lowering the
threshold after observing a failure would require separate calibration/false-trigger
evidence; it was not silently changed. Obstacle-contact coverage, vertical/torque
braking, actual force-triggered skill interruption and human resume remain untested
in this new physical trial. No causal success-rate claim follows from one run.

## Deliverables and closure

- Complete 15 fps HEAD video: `navigation/episode/camera.mp4`, 768×272,
  **1409 frames / 93.933 s**, complete ffmpeg decode passed.
- Two primary independent evaluations:
  `navigation/analysis/{navigation_evaluation,control_evaluation}.json`.
- Additional observation and force coverage:
  `navigation/analysis/{observation_evaluation,force_brake_evaluation,diagnostic_facts}.json`.
- English figures: `navigation/analysis/trajectory.{png,svg}` and
  `navigation/analysis/force_timeline.{png,svg}`. World geometry/contact is offline only.
- Complete RGB, raw sensors, force estimates, all model/planner inputs and outputs,
  executed commands/gait, MPPI candidates, independent physical and contact traces,
  source snapshot, receipts and `results.json` at experiment root and navigation root.

Trace tails match at 95.40 s across online, execution, estimate, physical and contact
logs. Simulation/encoder/task tmux exited and GPU4 was released. The user-owned
GPU3 navigation model remains loaded. No additional task, training, hardware action,
driver reset, fallback controller, Git commit/push or modification to scene/MPPI
limits/default planner profile was performed.
