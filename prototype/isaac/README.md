# Native Isaac navigation

## Current deployment path: HEAD RGB only

The current default is [the simplified RGB entry](../navigation/README.md), launched
through `prototype/run.sh`. It uses `worker.py --sensor-mode rgb --camera-profile a2_head_isaaclab`,
Codex GPT-6.1 Sol, CUDA MPPI tracking and the existing WBC. It creates no depth camera/render product.
The selected default route is elevator E2 → Entrance A, approximately nearby and indoors.
Its first full RGB-only trial ended NEEDS_USER before reaching Entrance A; see the RGB README.

The RGB-D launcher and examples below are historical reproductions, selected explicitly.

## Default scene: campus MF-R1

The default Isaac test scene is now the Owner-provided campus package, referenced through
`.local/assets/campus_mf_r1/integration/environment.usda`. The original first/third-floor
geometry, textures and collisions are preserved. Source units are metres and Z-up.
Only the embedded PhysicsScene is deactivated so the A2 plant owns the single simulation.

The default scenario is `scenarios/campus_mf_r1_elevator.json`: start at the supplied
first-floor start point and approach the middle elevator (number 2), face its doors,
and stop. The target panel geometry is used only by the offline evaluator; online
navigation receives the natural-language goal and sensor-derived data.

The first-to-third-floor separation of 11.10m is a modelling placement, not a measured
height. This first-floor task leaves the elevator in its source closed state. The
provided elevator controller is retained with the source package, but is not run;
boarding, riding and multi-floor task scheduling are outside this scene swap.

The current host has another project's service on port8766. The launcher uses8768
by default and passes `SIM_URL` to the recorder and Pi process.

```bash
cd /home/baoquanc/workspace/GuidePup
# Asset preparation after extracting the supplied package:
bash prototype/isaac/asset_python.sh prototype/isaac/prepare_campus.py
# Bounded scene/A2/RGB-D/state/map loading validation; no navigation/model calls:
LIGHTNAV_GPU=7 ISAAC_RENDERER_GPU=7 .local/envs/sim/bin/python \
  prototype/isaac/run_experiment.py --preview-only --output .local/outputs/campus_preview_NEW
# Full first-floor task entry when navigation is requested again; reuse the retained GPU3 model:
.local/envs/sim/bin/python prototype/isaac/run_experiment.py \
  --sim-gpu 4 --lightnav-gpu 3 --reuse-lightnav \
  --output .local/outputs/campus_navigation_NEW \
  --controller-footprint .local/outputs/campus_mf_r1_integration_20261015_01/footprint_10cm.json
```

No full task was submitted during the 2026-10-15 integration. Native RTX on physical
GPU7, RGB-D, sensor-only odometry and depth-map readiness passed; see
[the integration report](../NAVIGATION_CAMPUS_MF_R1_20261015.md).

## Earlier environment scenarios

This backend runs the existing A2 policy, sensor odometry, LightNav, Nav2 MPPI and the Pi
task runtime against native Isaac Lab / PhysX dynamics and RTX RGB-D rendering.
`scenarios/hospital_rooms.json` selects two rooms connected by the official
Hospital corridor. It keeps the shipped open doors, meshes and collisions.

`scenarios/hospital_open_room.json` starts a fresh task at the previous corridor
endpoint and directly targets the adjacent fully open room. It resets the local
map, estimator and model sessions; the door geometry is unchanged. The result
is recorded in [the direct-room report](../NAVIGATION_ISAAC_OPEN_ROOM_20260916.md).

`scenarios/hospital_room_to_open_room.json` returns to the original source-room
spawn and targets that same fully open room across the corridor. Its offline
score requires `room_a_exit` followed by `room_b_entry`; their coordinates stay
outside online navigation. This task uses the current navigation implementation
and a separate 10 cm desired-clearance footprint configuration.

## Process boundary

`worker.py` runs in `/home/baoquanc/anaconda3/envs/isaaclab/bin/python`
(Isaac Sim 5.1, Isaac Lab 0.54.4). `client.py` and the shared navigation server
run in `.local/envs/sim`; no packages are added to the shared Isaac environment.
They exchange native joint targets and sensor packets through a local Unix socket.

The worker exports joint encoders, body IMU angular velocity and acceleration,
and synchronized RGB plus optical-axis depth. It uses the actual camera intrinsic
matrix and the configured fixed camera-to-body extrinsics. It does not export
root position, orientation or velocity to the controller. The current estimator
uses depth, integrated gyro and the original policy's learned body-velocity
estimate. All online geometry, local control and the read-only semantic observer consumers share this estimate.

RGB remains 640×360 and approximately 69°×42°. An independent native RTX depth
camera covers approximately 87°×58° on a square-pixel 848×496 simulation raster.
This raster is not a claimed hardware stream mode. Full depth feeds odometry
and mapping; calibrated reprojection supplies the RGB-aligned semantic depth.
See [native depth calibration](../camera/NATIVE_DEPTH.md) for actual K/T, exposure
alignment evidence and the nominal co-located-optics limitation.

The original A2 URDF is read from
`/home/baoquanc/workspace/LMP/resources/a2_description/urdf/a2.urdf`; converted USD
is written inside each experiment output. Initial placement is applied once,
before the stationary IMU initialization. Runtime motion uses native PhysX and
the original training actuator gains, implicit PD, armature and joint limits.
The existing policy export remains unchanged.

Ground truth is written separately for offline scoring. Per-body native PhysX
contact sensors record net normal forces at 200 Hz. They do not identify the
other collider or contact point, and do not measure friction. These records can
expose non-foot loading but cannot prove collision-free travel.

## Run one task

The official Hospital asset and all its USD/MDL/texture dependencies must already
be local under `.local/assets/isaac/Isaac/Environments/Hospital/` and the referenced
shared directories. The asset download receipt is retained with preparation
evidence in `.local/outputs/isaac_assets_20260916/`. Downloads clear both-case
HTTP/HTTPS/ALL_PROXY and explicitly disable library proxy discovery. Official S3
endpoints are accessed directly; the Codex dedicated proxy is unrelated.

```bash
LIGHTNAV_GPU=0 .local/envs/sim/bin/python prototype/isaac/run_experiment.py \
  --scenario prototype/isaac/scenarios/hospital_rooms.json \
  --output .local/outputs/isaac_hospital_<new-name> \
  --wait-for-start
```

Run the command in an independent tmux session. It saves `preview.jpg` before
submitting any task. After inspecting that actual RTX image, create `start_task`
inside the new output directory. The launcher starts LightNav and one Qwen task
with no whole-task wall-clock deadline, a 30-second decision budget and at most three
recoveries. This experiment has no injected force or manual pause/resume.

GPU0 is resolved to this machine's UUID for CUDA; Isaac RTX uses the corresponding
single visible GPU. Qwen uses the existing lab profile, including the model ID's
trailing slash. The launcher does not start Gemini or overwrite any Pi profile.

Only `goal` and `semantic_classes` from the scenario enter navigation. Asset path
and spawn pose initialize physics. `offline_evaluation` supplies native geometry
to the post-run evaluator only. YOLO classes remain task configuration.

Individual comparisons use explicit launcher arguments:

- `--controller-footprint PATH`: use a separate footprint/preferences JSON; keep body
  dimensions and hard clearances fixed when comparing desired clearance.
- `--lightnav-instruction-prefix TEXT`: prepend a local-navigation preference
  to the Qwen stage intent without changing the Qwen model profile.
- `--camera-forward-m 0.0`: move the RGB/depth rig together to base X=0 and
  export the changed extrinsics. It does not modify the default camera profile.

Use a fresh output directory for each run and change one comparison argument
at a time. The recorded configuration includes all resolved values.

## Evidence

`episode/` retains Pi requests and tool responses, monitor state, camera video,
source snapshot and summary. The raw server output preserves RGB-D frames,
sensor estimates, LightNav inputs/outputs and native MPPI commands actually applied.
`physics/` holds independent world-state and contact-force records.

After stopping the services, `evaluate.py` writes `physical_evaluation.json`,
`navigation_evidence.json` and `results.json`. Physical scoring requires all
configured ordered door crossings by the whole body envelope, followed by a
stop inside the destination room. Stage completion events are checked against
the physical trajectory. NEEDS_USER and runtime errors are retained separately.

This is a new scene and physics backend as well as a new renderer. Comparison
with an earlier MuJoCo episode cannot isolate rendering as the cause of a change.

Official environment catalog:
[Isaac Sim 5.1 environments](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_environments.html).

## Current decision/controller boundary (September 17)

Qwen decides navigation instructions and task completion. Semantic observations
are read-only references: no distance-triggered review pause, automatic stage
advance, or all-stage-certificate completion gate. STOP and actual execution
failure return to Qwen. Recovery budget is at most three for the whole task.

The local controller is native Nav2 MPPI, with the retained A2 envelope and
empirical stopping model. `complete_task` means a Qwen declaration; the two offline
evaluators independently score actual ordered passage, destination entry and stop.
See [the implementation/run report](../NAVIGATION_MPPI_20260917.md).

## Task duration

The launcher and recorder impose no whole-task time budget. Per-decision timeout remains 30 seconds and the task retains at most 3 recoveries. A supervisor ETA is a waiting estimate, not an automatic termination deadline.

## Retained navigation model (2026-10-15)

Owner stopped further navigation retries and requested retaining loaded model weights.
The current model is on physical GPU3 in tmux `guidepup-lightnav-gpu3`, WebSocket
`ws://127.0.0.1:8050`. Simulation and task services are stopped. A future explicitly
requested task should use `--sim-gpu 4 --lightnav-gpu 3 --reuse-lightnav`.
This option requires a working resident service, never starts a replacement model,
and leaves the external service running on experiment exit. Without it, the launcher
owns its model process and shuts it down with the experiment. Only model loading is
retained; each navigation task still creates its own visual session.

## GRScenes import

The official dataset is [InternRobotics/GRScenes](https://huggingface.co/datasets/InternRobotics/GRScenes).
Task downloads explicitly unset both cases of HTTP/HTTPS/ALL_PROXY, set NO_PROXY/no_proxy to `*`, and disable downloader proxy settings. On this host the Hugging Face endpoint times out; the same repository is available through a direct connection to `hf-mirror.com`. Download source and resolved files are recorded with the experiment.

The current commercial archive extracts to `commercial_scenes/` (the dataset card's older `target_30_new` name does not match this archive). Preserve its per-scene `Materials` and `models` links and set the **process-local** `MDL_SYSTEM_PATH` to the commercial root's `Materials` directory.

Use the shipped `start_result_navigation.usd`. Its navigation preprocessing disables internal door objects and supplies static mesh collisions; do not silently replace it with the interaction or raw scene. Inspect actual geometry dimensions: these layers are Z-up with no authored `metersPerUnit`, and their geometry is centimetre scale. An unauthored USD default alone is not sufficient evidence of scale.

`asset_python.sh` exposes this host's bundled OpenUSD to CPU-only asset tools, without starting Isaac or allocating a GPU. `prepare_grscenes.py` creates a separate metre-scale reference wrapper, preserving source meshes/materials/collisions. It disables only the imported PhysicsScene prim so the existing A2 SimulationContext remains the owner of timestep, gravity and simulation device. The original dataset files are unchanged.

```bash
bash prototype/isaac/asset_python.sh prototype/isaac/prepare_grscenes.py \
  --scene .local/assets/GRScenes/scenes/GRScenes-100/commercial_scenes/scenes/SCENE_ID/start_result_navigation.usd \
  --output .local/assets/GRScenes/integrations/SCENE_ID/environment.usda \
  --meters-per-unit 0.01
```

Room/door annotations are for experiment placement and offline evaluation only. Online navigation continues to receive sensor-derived state and the natural-language task, not asset transforms or a scene map.

GRScenes实测与结果：`../NAVIGATION_GRSCENES_20260920.md`。场景需显式`disable_ujitso_cooking: true`，对应官方加载要求。启动器用`--setup-release-file`在地图/跟踪就绪后固定仿真时钟，等待预览和模型加载；释放后才开始任务，不能用持续零指令步态替代启动等待。原资产眼部与Ground_Leaves纹理缺陷详见报告。
