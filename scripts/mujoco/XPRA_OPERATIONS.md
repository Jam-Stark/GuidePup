# MuJoCo Xpra Operations

This document records the working Xpra-based remote visualization workflow for the A2 MuJoCo sim2sim runner.

Assumptions:

- current working directory is the repository root
- conda environment name is `mujoco`
- Xpra display is fixed to `:100`
- Xpra HTML5 port is fixed to `14500`
- the MuJoCo model is `resources/a2_description/a2.xml`
- the A2 root body is `base_link`; body-force/camera overlays use metadata from `policy.json`
- Ubuntu package `xpra` version is `3.1.5`

Important compatibility note:

- on this machine, `xpra 3.1.5` accepts `--bind-tcp=127.0.0.1:14500 --tcp-auth=none`
- it does not accept `--bind-tcp=127.0.0.1:14500,auth=none`
- the live `xpra info :100` output reports `env.XAUTHORITY=/home/baoquanc/.Xauthority`

## 1. Environment Checks

Check the basic runtime:

```bash
python scripts/mujoco/check_setup.py --model-path resources/a2_description/a2.xml
```

Check a TorchScript policy file before launching visualization:

```bash
python scripts/mujoco/check_setup.py \
  --model-path resources/a2_description/a2.xml \
  --policy-path /absolute/path/to/policy.pt
```

Run a short headless smoke test:

```bash
python scripts/mujoco/play_policy.py \
  --model-path resources/a2_description/a2.xml \
  --headless \
  --duration-s 0.1
```

## 2. Start A Persistent Xpra Session

Activate the environment first:

```bash
source /home/baoquanc/anaconda3/etc/profile.d/conda.sh
conda activate mujoco
```

Recommended helper:

```bash
bash scripts/mujoco/start_xpra_server.sh
```

Equivalent raw command:

```bash
xpra start :100 \
  --bind-tcp=127.0.0.1:14500 \
  --tcp-auth=none \
  --html=on \
  --mdns=no \
  --daemon=yes \
  --exit-with-children=no \
  --pulseaudio=no \
  --notifications=no \
  --printing=no \
  --file-transfer=no \
  --webcam=no
```

Important behavior change versus the old one-shot pattern:

- do not pass `--start-child`
- do not tie the session lifetime to a MuJoCo process
- keep the browser pointed at the same HTML5 page and launch MuJoCo jobs into `DISPLAY=:100`

You can now leave the Xpra server running in the background and reuse the same page for multiple runs.

## 3. Launch MuJoCo Into The Existing Xpra Session

Recommended helper:

```bash
bash scripts/mujoco/run_in_xpra.sh \
  python scripts/mujoco/play_policy.py \
  --model-path resources/a2_description/a2.xml \
  --policy-path /absolute/path/to/policy.pt \
  --duration-s 3600 \
  --real-time
```

Equivalent raw command:

```bash
cd /home/baoquanc/workspace/ForceControl
export DISPLAY=:100
export XAUTHORITY=/home/baoquanc/.Xauthority
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export PYTHONUNBUFFERED=1
conda run -n mujoco python scripts/mujoco/play_policy.py \
  --model-path resources/a2_description/a2.xml \
  --policy-path /absolute/path/to/policy.pt \
  --duration-s 3600 \
  --real-time
```

Staged-direction-force evaluation works the same way:

```bash
bash scripts/mujoco/run_in_xpra.sh \
  python scripts/mujoco/staged_direction_force_eval.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/.../mujoco_exported/policy.pt \
  --duration-s 60.0
```

Notes:

- the MuJoCo process exits normally when the run finishes
- the Xpra server keeps running after the MuJoCo process exits
- refreshing or reconnecting the browser page keeps you on the same `:100` session
- any later MuJoCo process launched with `DISPLAY=:100` and `XAUTHORITY=/home/baoquanc/.Xauthority` will appear in the same web session

## 4. Connect From Local Machine

Use either method:

- VS Code Remote SSH port forwarding for remote port `14500`
- or manual SSH local forwarding:

```bash
ssh -N -L 14500:127.0.0.1:14500 baoquanc@ai-precog-machine5
```

Then open:

```text
http://127.0.0.1:14500/
```

If the Xpra page is already open when you launch a new MuJoCo job into `:100`, the window should appear there directly.

## 5. Session Monitoring

Check whether the Xpra session exists:

```bash
xpra list
```

Inspect the running Xpra session:

```bash
xpra info :100
```

Check that the HTML5 port is listening:

```bash
ss -ltn | rg ":14500\\b"
```

Check that the local Xpra web endpoint responds:

```bash
curl -I --max-time 5 http://127.0.0.1:14500/
```

Check the MuJoCo and Xpra processes:

```bash
pgrep -af "scripts/mujoco/play_policy.py|scripts/mujoco/staged_direction_force_eval.py|xpra.*:100"
```

Tail the Xpra server log:

```bash
tail -n 200 /run/user/1005/xpra/:100.log
```

Useful log signals:

- `xpra is ready.` means the Xpra server started correctly
- `serving html content from '/usr/share/xpra/www'` means the HTML5 client is enabled
- `OpenGL is supported on display ':100'` means rendering is available
- `using 'llvmpipe ...' renderer` means software rendering is active

## 6. Stop And Restart

Stop a MuJoCo job without stopping Xpra:

```bash
pkill -f "scripts/mujoco/play_policy.py"
pkill -f "scripts/mujoco/staged_direction_force_eval.py"
```

Stop the current Xpra session cleanly when you no longer need the browser endpoint:

```bash
xpra stop :100
```

Confirm that it has exited:

```bash
xpra list
pgrep -af "scripts/mujoco/play_policy.py|scripts/mujoco/staged_direction_force_eval.py|xpra.*:100" || true
```

If the session is stuck, stop it first and then start the same `:100` display again.
Do not change the display number unless there is a strong reason.

## 7. Common Failure Checks

If the browser page does not open:

```bash
xpra list
ss -ltn | rg ":14500\\b"
curl -I --max-time 5 http://127.0.0.1:14500/
```

If the browser page opens but there is no MuJoCo window:

```bash
xpra info :100
tail -n 200 /run/user/1005/xpra/:100.log
pgrep -af "scripts/mujoco/play_policy.py|scripts/mujoco/staged_direction_force_eval.py|xpra.*:100"
```

If MuJoCo crashes on startup, re-run the non-visual checks:

```bash
python scripts/mujoco/check_setup.py --model-path resources/a2_description/a2.xml
python scripts/mujoco/play_policy.py --model-path resources/a2_description/a2.xml --headless --duration-s 0.1
```

If the model opens but performance is poor, this is expected for the current software-rendering baseline.
The next upgrade path is `VirtualGL + Xpra` or `TurboVNC + VirtualGL`.
