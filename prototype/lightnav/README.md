# LightNav service

Uses the official [LightNav-0](https://github.com/lightorigins/LightNav-0) HF
backend and [LightOriginsHQ/LightNav-0](https://huggingface.co/LightOriginsHQ/LightNav-0)
checkpoint, including its RVQ action decoder. The server is separate from the
A2 policy and the simulator.

## Install

Run from this worktree. `.local` points to the data disk; Python packages,
downloaded weights, pip/Hugging Face caches, and temporary files live there.
The venv reads the selected Python's existing packages, reusing this machine's
torch 2.10.0 CUDA installation. New packages are installed into the data-disk
venv. The shared `mujoco` and `isaaclab` conda environments are not modified.

```bash
LIGHTNAV_PYTHON=/home/baoquanc/anaconda3/envs/mujoco/bin/python \
  bash prototype/lightnav/setup.sh
```

The environment uses Python 3.11, torch 2.10.0, torchvision 0.25.0 and the
upstream-required transformers 5.8.0. Torch/torchvision use the official
PyTorch cu128 wheel index. `lightnav[video]` supplies the official
HF server, image/video client and renderer. vLLM and FlashAttention are not
required for this backend. Long installations/downloads can be run in a
dedicated tmux session.

```bash
tmux new-session -s forcecontrol-lightnav-install \
  'bash prototype/lightnav/setup.sh > .local/cache/lightnav/install.log 2>&1'
```

To resume only the official model download:

```bash
bash prototype/lightnav/download_model.sh
```

Downloads use this server's direct connection. `environment.sh` clears the
six HTTP/HTTPS/ALL proxy variables, sets both `NO_PROXY` spellings to `*`,
clears `PIP_PROXY`, and disables pip config files. The clone command also
disables system/global Git config and explicitly clears Git's HTTP proxy.
This matters for tmux, whose server retained proxy variables even while the
interactive exec environment had none. The venv's pip is 26.2.1.

## Run

```bash
LIGHTNAV_GPU=0 bash prototype/lightnav/serve.sh
```

Defaults: `hf`, BF16/SDPA, navigation prompt (`vln`), one session per batch,
`ws://127.0.0.1:8050`. `LIGHTNAV_PORT` changes the port; extra arguments are
passed to the official server. The server warms up before opening the port
and touches `.local/cache/lightnav/server/ready` once listening.

Send a captured PNG/JPEG through the real service:

```bash
.local/envs/lightnav/bin/python prototype/lightnav/infer_image.py \
  --image /absolute/path/to/camera.png \
  --instruction 'Go to the door in front and stop.' \
  --output .local/cache/lightnav/image_prediction.json
```

## Simulator interface

The official [WebSocket protocol](https://github.com/lightorigins/LightNav-0/blob/main/docs/PROTOCOL.md)
uses `login`, `reset`, then one `next` request per frame:

```json
{"action":"next","data":{"seq":0,"image":"BASE64_JPEG","instruction":"Go to the door in front and stop."}}
```

`data.actions.actions` contains ten cumulative robot-local poses
`[forward_m, left_m, yaw_rad]`, relative to the pose when that input image was
captured. Use the corresponding capture-time odometry to transform the path
before tracking it. The waypoints do not encode a wall-clock deadline or
velocity command. `data.stop` means a zero trajectory; task completion is
decided by the task manager.

The released model reads `eval_config.json`: 256×448 model frames, 4 Hz
training video rate, SlowFast history and a 10-pose horizon. Leave those
processing settings at their checkpoint values for the first prototype.
Client frames use RGB; their native JPEG/PNG size is accepted by the server.

Useful upstream modules under `.local/vendor/LightNav-0/mujoco_demo/vln_mujoco/`:

- `mpc.py`: ROS-independent CasADi/IPOPT path tracker.
- `vln_client.py`: asynchronous WebSocket client and capture-time timestamp.
- `robots/base.py`: backend interface for MuJoCo model/data, command stepping,
  pose/velocity and cameras.
- `robots/microduck.py`: example of a 50 Hz walking-policy backend.

The upstream demo inference receive timeout is 3 seconds. Measure HF latency
before reusing that value; `infer_image.py` allows 120 seconds so that a slow
prediction is reported with its actual latency.

## Local bring-up evidence (2026-09-14)

Environment imports and CUDA discovery succeeded: LightNav 0.1.0,
transformers 5.8.0, torch 2.10.0+cu128, torchvision 0.25.0+cu128,
huggingface-hub 1.31.0, NumPy 2.4.3, Pillow 12.3.0, websockets 17.1.

The official 9,695,744,034-byte weight file downloaded successfully. The HF
server then loaded the model, ran its startup warmup, and opened port 8050.

One actual A2 MuJoCo camera image (`.local/outputs/a2_camera.jpg`, 640×360) was
sent with `Turn right and go through the doorway.`. The response had `rc=0`,
ten waypoints, `stop=false` and `apos_state=rot_right`; the first waypoint's
yaw was −0.314 radians. Server latency was 353.84 ms and client wall time
408.63 ms. This establishes the real image-to-trajectory path, not closed-loop
navigation performance.

On GPU0 (RTX 3090), the service occupied 8,972 MiB after that call. Its 200 ms
process-memory sampling peak during startup/warmup/the one image call was
also 8,972 MiB; this is a sampled device-memory peak, not an allocator peak.

Evidence lives under `.local/cache/lightnav/`:

- `a2_image_prediction.json`: complete response, image path and instruction.
- `server.log`: official model load, warmup and WebSocket serving logs.
- `gpu_memory_summary.json`, `gpu-memory.csv`: observed process memory.
- `download.log`: official checkpoint download completion.
