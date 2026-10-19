# Natural building-exit instruction: one full navigation task

2026-10-15 21:03 HKT. Output: `/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01`.

## Result

The task ended **NEEDS_USER**, with **no runtime exception**, **no planner completion claim**, and **0/1 independent physical success**. The robot remained indoors and did not stop stably. This was a planner-requested controlled termination, not a skill/correction cap or a thinking/task timeout.

The exact user-level instruction was:

> Find the building exit and go outside.

No trees, hedges, mat, target image, route, scene map or target coordinates were supplied with this task. The new endpoint is **outdoors through a genuine building exit**. It is not the previous indoor stopping point at selected Entrance A; A's offline distance is diagnostic only.

| Measurement | Result |
|---|---:|
| Wall / sensor duration | 494.803 s / 176.76 s |
| Physical travel | 15.241 m |
| Final physical XY / yaw | [3.3403, -3.8387] m / -0.8561 rad |
| Final 0.6 s peak planar speed / yaw rate | 0.2470 m/s / 0.5944 rad/s |
| GPT calls / longest call | 12 / 21.717 s |
| Observation calls / maximum in one cycle | 8 / 3 |
| Runtime correction-cycle counter | 3 |
| Navigation inputs / accepted STOP proposals | 60 / 3 |
| Navigation sessions / initial resets | 1 / 1 |
| MPPI solves / actually executed controls | 171 / 171; no command mismatch |
| MPPI median GPU / total solve time | 3.054 / 4.552 ms |
| Automatic estimated-force STOP events | 0 |
| Full video | 2652 frames, 176.8 s, 768×272 at 15 fps; complete decode passed |

## What changed

Only `prototype/navigation/task.json`'s `goal` field changed relative to the completed uncapped task. All other captured runtime sources, resolved configuration and camera calibration match. The current task keeps E2 as its starting position, HEAD RGB only, GPU3's resident navigation model, GPU4 Isaac/CUDA MPPI, the existing WBC, no lateral/reverse command, and the original individual skill bounds. No skill-count, correction-count, GPT thinking-time or whole-task deadline was restored. The semantic Monitor remains read-only.

The earlier detailed-description attempt was stopped while running when the Owner changed the task. It is retained separately in `.local/outputs/campus_goal_description_20261015_01/`, marked **CANCELLED_BY_OWNER_TASK_CHANGE**, with its partial 88.467 s video and raw SIGINT/CancelledError evidence. It is neither a completed trial nor an autonomous runtime failure. This report covers exactly one subsequent full task; no automatic retry followed it.

## What GPT and the navigation model actually did

GPT maintained that an observation subgoal or a model STOP was not proof of reaching outdoors. It first surveyed left 60° and right 100°, then approached the bright corridor end as an explicitly unconfirmed observation subgoal. After STOP 1 it observed the glass-door area with left 65° and left 50°, then approached a front-right glass-door candidate.

After STOP 2 it asked for left 45°, a four-second hold and right 90°. At sensor time **141.30 s**, the actual RGB showed an open lobby on the right and the distant exterior glass facade. GPT described a new local target using what it saw: “Approach the glass doors with trees visible beyond them and stop.” This was GPT's online visual description, not a visual detail supplied by the user-level task.

The navigation model again moved toward nearby dark glazing. Its last accepted STOP, sequence 59, came with a target point clamped at the upper image edge; this did not establish target identity. GPT rejected completion, requested another right-90° observation, then requested assistance after the observed turn/stop failure. No module automatically advanced a semantic stage or fabricated an outdoor-arrival certificate.

| Actual navigation phase | MPPI controls executed | Physical travel during that phase |
|---|---:|---:|
| Initial corridor subgoal | 70 | 3.311 m |
| First navigation continuation: glass double-door candidate | 50 | 1.563 m |
| Second navigation continuation: exterior-door description | 51 | 1.114 m |

These are executed commands and independent physical motion, not predicted trajectory lengths. The runtime counter reached three correction cycles, but only two navigation continuations occurred; the last cycle ended after its observation skill failed. Overall travel includes observation and zero-command movement.

## Physical turn and stopping failure

The final right-90° skill emitted **742 nonzero yaw-command ticks** across **875 execution ticks**, with zero forward/lateral command. Estimated net yaw was **−6.28°**; independent physical net yaw was **−6.80°**, with **0.944 m** physical travel. The turn failure therefore cannot be explained solely as an odometry reporting error.

The four-second hold, including its settling interval, emitted **352 all-zero command ticks**, yet physically travelled **0.481 m** and changed yaw **+7.32°**. Stopping also failed at task end. The original force-driven walking logic was active on 2484 zero-command ticks across the task. That association does not separate contact, gait response and body dynamics into a complete causal account. No stop-policy fix was made during this description-only task.

The 171 MPPI results were all issued with matching final commands; there was no local-controller refusal or optimizer/interface error. This RGB-only MPPI configuration tracks trajectories and bounds motion. It has no depth occupancy map or geometric obstacle-rejection guarantee. Observation turns use their existing exclusive skill motion path rather than MPPI tracking.

## Contact and estimated-force limits

The offline contact observer recorded a **2901.78 N per-body net normal-force peak on base_link at 146.015 s**, with additional strong non-foot signals during the final turn. The final RGB shows close dark glazing. These observations support concern about contact and constrained motion, but the observer does not record counterpart identity, contact points or friction; it may include self-contact and force cancellation. It cannot prove the exact obstacle surface or certify collision-free movement.

The online learned horizontal base-force estimate peaked at **47.756 N**, below the inherited 60 N/two-distinct-sample trigger. Thus **no automatic force STOP or planner release occurred**. Learned applied-base-force estimates and offline contact forces are different quantities. This task does not validate contact-triggered stopping/recovery, and the threshold was not lowered.

## Interpretation and limits

- Fact: GPT did not mistake an indoor subgoal for successful outdoor arrival in this task.
- Fact: the system still did not reach outdoors; the final turn was physically ineffective and stopping failed.
- Inference: destination wording alone is insufficient to explain the remaining navigation failure. Local target grounding and execution remain relevant.
- Limit: both wording and the requested endpoint changed. One stochastic full task does not isolate a causal wording effect or establish general reliability.
- Coverage: more than two observations in one cycle and more than six overall were exercised. A GPT call above 30 s, more than three corrections, automatic force STOP/release, and a real exterior crossing were not exercised. The outdoor doors' traversability was not established.

## Evidence and closure

- [Full video](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/episode/camera.mp4)
- [Physical trajectory and offline floor projection](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/trajectory.png)
- [Navigation evaluation](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/navigation_evaluation.json)
- [Control evaluation](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/control_evaluation.json)
- [Observation execution](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/observation_evaluation.json)
- [Force-stop evaluation](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/force_stop_evaluation.json)
- [Results](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/results.json)
- [Intervention facts](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/diagnostic_facts.json)

RGB, model inputs/outputs, full GPT prompts/responses, observation samples, MPPI commands, IMU/joint estimates, independent physical/contact traces and source snapshots are retained. Online, execution, estimate, physical and contact traces all end at **178.28 s**. Simulator truth is used only by these offline evaluations.

The task's tmux, simulator and video encoder exited. GPU4 was released; GPU3's navigation model remains loaded as requested. No additional experiment, training, hardware action, driver reset or Git commit/push was performed.


## Follow-up diagnosis: visual target handoff and waiting motion

2026-10-15 21:26 UTC+8. This is a read-only analysis of the same completed task, with **no additional model call or navigation** and **no production implementation change**.

The outdoor-door instruction is **decision_010**, not decision_011. Its actual current source image is `rgb_001988` at **134.040 s**, byte-identical to `decision_010/image_02.jpg`; the lobby and vegetation beyond the glass are already visible on the right. The later decision_011 follows STOP and asks for another right turn. A temporary review mistake associated these adjacent directories; it is corrected here. The images do **not** establish that GPT mixed old/current views.

At 141.300 s, the navigation model's first response for the outdoor-door description (seq43) places `opos_px=[280.0,35.26]` on nearby dark glazing on the **left**, with `apos_state=rot_left`. The exterior facade described by GPT is on the **right** in that same input image. This is direct evidence that the language handoff did not preserve the intended visible object in that step. It does not identify the model's internal reason or prove all earlier GPT candidate choices were correct.

Between the d10 current capture and its accepted action, all **363 execution ticks** commanded zero locomotion, yet independent physics recorded **−12.421° yaw**, **0.55490 m net translation**, and **0.79055 m travel**. `walking` and `force_walking` were both true on **318 ticks**. Across all 12 decisions, image-to-action age was **6.12–9.48 sensor seconds**. These are image-age windows, not all exact GPT request intervals. The audit preserves their labels and measurements explicitly.

The current `engine.py` accepts a returned decision if its generation is unchanged. It does not check view/motion freshness. In `isaac/client.py`, zero locomotion can retain the existing force-driven gait: nominal load entry is 20 N and retention 5 N under the exported .01 scale. These are different from the 60 N automatic STOP threshold. Force-driven gait is associated with waiting motion here; it has not been isolated as its sole cause.

### Recommended next change — not implemented by this audit

1. Give decision waiting and skill settling explicit standing authority. Verify sensor-estimated stillness; if the view changes materially during inference, present the fresh view for a new decision. A normal standing wait should remain planner-resumable and separate from human brake.
2. Have the planner identify a target in a named current image, with a point/region as evidence and a full English instruction. For a side target, use an observation turn and verify the new view before approaching.
3. Return the navigation model's first target pointing to GPT on that **exact input JPEG**. GPT decides whether it refers to its intended object and chooses navigation, realignment or a new description. The semantic Monitor stays read-only. Missing pointing/uncertain identity stays unknown.
4. First validate the existing failed frames and a short actual standing/turning segment. Full navigation should follow after those narrow checks; this audit starts none of them.

The navigation protocol currently accepts RGB and instruction text, with target pointing as an **output**. A planner target box would be evidence for handoff checking, not a supported native point-goal input. See the [official protocol](https://github.com/lightorigins/LightNav-0/blob/main/docs/PROTOCOL.md). Explicit visual reference marks have prior research, e.g. [Set-of-Mark](https://arxiv.org/abs/2310.11441); that work does not validate this proposed A2 interface or guarantee correct exit recognition.

- [Original frames and recorded pointing comparison](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/planner_handoff_comparison.png)
- [Computed image-to-action windows](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/planner_handoff_audit.json)
- [Reproducible offline audit](/home/baoquanc/workspace/GuidePup/.local/outputs/campus_building_exit_20261015_01/navigation/analysis/audit_handoff.py)
