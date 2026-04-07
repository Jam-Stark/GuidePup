# RL Command Batches

Single source of truth for train/play command batches.

## Usage

1. Keep each experiment as one batch section.
2. Use helper script to append/update batches and auto-generate play + analysis.
3. Analysis mode is fixed to `prev+base`.
4. `checkpoint` can be auto-resolved from `logs/rsl_rl` by helper.
5. `public_ip/play_device/play_num_envs` can inherit from `base` by helper defaults.

## Field conventions

- `batch_id`: unique experiment id
- `task`: gym task id
- `run_name`: training run name
- `status`: `planned` | `running` | `done`
- `previous_batch_id`: previous batch used for incremental diff
- `base_id`: baseline batch id for stable comparison
- `created_at`: ISO timestamp
- `public_ip`: play livestream public ip (default `10.13.11.197`)

## Base Registry

<!-- RLCM:BASE_REGISTRY_START -->
```json
{
  "active_base": "cfg30_zScale",
  "bases": {
    "cfg30_zScale": {
      "created_at": "2026-02-27T00:00:00",
      "reason": "Initial user baseline",
      "train_command": "CUDA_VISIBLE_DEVICES=0,1 torchrun --nproc_per_node=2 --master_port=29501 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg30_zScale --distributed --num_envs 2048 env.scene.robot.actuators.hips.stiffness=100.0 env.scene.robot.actuators.hips.damping=3.5 env.scene.robot.actuators.thighs.stiffness=100.0 env.scene.robot.actuators.thighs.damping=3.5 env.scene.robot.actuators.calfs.stiffness=160.0 env.scene.robot.actuators.calfs.damping=7.0 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=1.5 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667"
    }
  }
}
```
<!-- RLCM:BASE_REGISTRY_END -->

## Batches

<!-- RLCM:BATCHES_START -->

<!-- RLCM:BATCH_TEMPLATE_START -->
## Batch <batch_id>

### Metadata
```yaml
batch_id: <batch_id>
task: Template-UniFPAliengo-v0
run_name: <run_name>
status: planned
previous_batch_id: <previous_batch_id>
base_id: <base_id>
created_at: <ISO_TIMESTAMP>
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: <CHECKPOINT_PATH>
```

### Train Command
```bash
# fill by helper
```

### Play Command (Auto)
```bash
# fill by helper
```

### Diff vs Previous (Auto)
```markdown
# fill by helper
```

### Diff vs Base (Auto)
```markdown
# fill by helper
```

### Impact Notes (Auto)
```markdown
# fill by helper
```

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_TEMPLATE_END -->

<!-- RLCM:BATCH_START cfg30_zScale -->
## Batch cfg30_zScale

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
batch_id: cfg30_zScale
task: Template-UniFPAliengo-v0
run_name: cfg30_zScale
status: planned
previous_batch_id: ""
base_id: cfg30_zScale
created_at: 2026-02-27T19:56:34
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: logs/rsl_rl/uniFP_aliengo_position_force/2026-01-28_21-08-27_cfg30_zScale/model_16000.pt
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
CUDA_VISIBLE_DEVICES=0,1 torchrun --nproc_per_node=2 --master_port=29501 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg30_zScale --distributed --num_envs 2048 env.scene.robot.actuators.hips.stiffness=100.0 env.scene.robot.actuators.hips.damping=3.5 env.scene.robot.actuators.thighs.stiffness=100.0 env.scene.robot.actuators.thighs.damping=3.5 env.scene.robot.actuators.calfs.stiffness=160.0 env.scene.robot.actuators.calfs.damping=7.0 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=1.5 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
PUBLIC_IP=10.13.11.197 LIVESTREAM=1 ENABLE_CAMERAS=1 python scripts/rsl_rl/play.py --task Template-UniFPAliengo-v0 --resume --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-01-28_21-08-27_cfg30_zScale/model_16000.pt --num_envs 128 --device cuda:1 \
    env.scene.robot.actuators.hips.stiffness=100.0 \
    env.scene.robot.actuators.hips.damping=3.5 \
    env.scene.robot.actuators.thighs.stiffness=100.0 \
    env.scene.robot.actuators.thighs.damping=3.5 \
    env.scene.robot.actuators.calfs.stiffness=160.0 \
    env.scene.robot.actuators.calfs.damping=7.0 \
    env.scene.robot.actuators.arm.stiffness=64.0 \
    env.scene.robot.actuators.arm.damping=1.5 \
    env.scene.robot.actuators.arm_shoulder.stiffness=128.0 \
    env.scene.robot.actuators.arm_shoulder.damping=3.0 \
    env.commands.gait_phase.gait_frequency=3.0 \
    env.rewards.feet_air_time.params.target_air_time=0.1667 \
    agent.device=cuda:1
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
```markdown
Reference unavailable: no previous batch specified
```
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
```markdown
No parameter changes.
```
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
```markdown
No impact notes generated because no parameter deltas were detected.
```
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_END cfg30_zScale -->

<!-- RLCM:BATCH_START cfg31 -->
## Batch cfg31

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
batch_id: cfg31
task: Template-UniFPAliengo-v0
run_name: cfg31
status: planned
previous_batch_id: cfg30_zScale
base_id: cfg30_zScale
created_at: 2026-02-27T19:56:34
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: logs/rsl_rl/uniFP_aliengo_position_force/2026-02-01_18-21-09_cfg31/model_21600.pt
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
CUDA_VISIBLE_DEVICES=2,3 torchrun --nproc_per_node=2 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg31 --distributed --num_envs 2048 env.scene.robot.actuators.hips.stiffness=180.0 env.scene.robot.actuators.hips.damping=4.0 env.scene.robot.actuators.thighs.stiffness=180.0 env.scene.robot.actuators.thighs.damping=4.0 env.scene.robot.actuators.calfs.stiffness=300.0 env.scene.robot.actuators.calfs.damping=6.5 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=2.0 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
PUBLIC_IP=10.13.11.197 LIVESTREAM=1 ENABLE_CAMERAS=1 python scripts/rsl_rl/play.py --task Template-UniFPAliengo-v0 --resume --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-02-01_18-21-09_cfg31/model_21600.pt --num_envs 128 --device cuda:1 \
    env.scene.robot.actuators.hips.stiffness=180.0 \
    env.scene.robot.actuators.hips.damping=4.0 \
    env.scene.robot.actuators.thighs.stiffness=180.0 \
    env.scene.robot.actuators.thighs.damping=4.0 \
    env.scene.robot.actuators.calfs.stiffness=300.0 \
    env.scene.robot.actuators.calfs.damping=6.5 \
    env.scene.robot.actuators.arm.stiffness=64.0 \
    env.scene.robot.actuators.arm.damping=2.0 \
    env.scene.robot.actuators.arm_shoulder.stiffness=128.0 \
    env.scene.robot.actuators.arm_shoulder.damping=3.0 \
    env.commands.gait_phase.gait_frequency=3.0 \
    env.rewards.feet_air_time.params.target_air_time=0.1667 \
    agent.device=cuda:1
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | changed | 1.5 | 2.0 |
| Actuator | env.scene.robot.actuators.calfs.damping | changed | 7.0 | 6.5 |
| Actuator | env.scene.robot.actuators.calfs.stiffness | changed | 160.0 | 300.0 |
| Actuator | env.scene.robot.actuators.hips.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.hips.stiffness | changed | 100.0 | 180.0 |
| Actuator | env.scene.robot.actuators.thighs.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.thighs.stiffness | changed | 100.0 | 180.0 |
| Runner/Device | cli.--run_name | changed | cfg30_zScale | cfg31 |
| Runner/Device | launcher.--master_port | removed | 29501 | (missing) |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1 | 2,3 |
```
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | changed | 1.5 | 2.0 |
| Actuator | env.scene.robot.actuators.calfs.damping | changed | 7.0 | 6.5 |
| Actuator | env.scene.robot.actuators.calfs.stiffness | changed | 160.0 | 300.0 |
| Actuator | env.scene.robot.actuators.hips.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.hips.stiffness | changed | 100.0 | 180.0 |
| Actuator | env.scene.robot.actuators.thighs.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.thighs.stiffness | changed | 100.0 | 180.0 |
| Runner/Device | cli.--run_name | changed | cfg30_zScale | cfg31 |
| Runner/Device | launcher.--master_port | removed | 29501 | (missing) |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1 | 2,3 |
```
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
```markdown
| group | key | change_vs_prev | change_vs_base | expected_impact | risk | monitor |
|---|---|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | 1.5 -> 2.0 | 1.5 -> 2.0 | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.calfs.damping | 7.0 -> 6.5 | 7.0 -> 6.5 | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.calfs.stiffness | 160.0 -> 300.0 | 160.0 -> 300.0 | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Actuator | env.scene.robot.actuators.hips.damping | 3.5 -> 4.0 | 3.5 -> 4.0 | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.hips.stiffness | 100.0 -> 180.0 | 100.0 -> 180.0 | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Actuator | env.scene.robot.actuators.thighs.damping | 3.5 -> 4.0 | 3.5 -> 4.0 | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.thighs.stiffness | 100.0 -> 180.0 | 100.0 -> 180.0 | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Runner/Device | cli.--run_name | cfg30_zScale -> cfg31 | cfg30_zScale -> cfg31 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | launcher.--master_port | removed (was 29501) | removed (was 29501) | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | 0,1 -> 2,3 | 0,1 -> 2,3 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
```
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_END cfg31 -->

<!-- RLCM:BATCH_START cfg32-base -->
## Batch cfg32-base

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
batch_id: cfg32-base
task: Template-UniFPAliengo-v0
run_name: cfg32-base
status: planned
previous_batch_id: cfg31
base_id: cfg30_zScale
created_at: 2026-02-27T19:56:34
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: logs/rsl_rl/uniFP_aliengo_position_force/2026-02-27_16-55-28_cfg32-base/model_3600.pt
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
CUDA_VISIBLE_DEVICES=0,1,2,3 torchrun --nproc_per_node=4 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg32-base --distributed --num_envs 1024 env.scene.robot.actuators.hips.stiffness=100.0 env.scene.robot.actuators.hips.damping=3.5 env.scene.robot.actuators.thighs.stiffness=100.0 env.scene.robot.actuators.thighs.damping=3.5 env.scene.robot.actuators.calfs.stiffness=160.0 env.scene.robot.actuators.calfs.damping=7.0 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=1.5 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
PUBLIC_IP=10.13.11.197 LIVESTREAM=1 ENABLE_CAMERAS=1 python scripts/rsl_rl/play.py --task Template-UniFPAliengo-v0 --resume --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-02-27_16-55-28_cfg32-base/model_3600.pt --num_envs 128 --device cuda:1 \
    env.scene.robot.actuators.hips.stiffness=100.0 \
    env.scene.robot.actuators.hips.damping=3.5 \
    env.scene.robot.actuators.thighs.stiffness=100.0 \
    env.scene.robot.actuators.thighs.damping=3.5 \
    env.scene.robot.actuators.calfs.stiffness=160.0 \
    env.scene.robot.actuators.calfs.damping=7.0 \
    env.scene.robot.actuators.arm.stiffness=64.0 \
    env.scene.robot.actuators.arm.damping=1.5 \
    env.scene.robot.actuators.arm_shoulder.stiffness=128.0 \
    env.scene.robot.actuators.arm_shoulder.damping=3.0 \
    env.commands.gait_phase.gait_frequency=3.0 \
    env.rewards.feet_air_time.params.target_air_time=0.1667 \
    agent.device=cuda:1
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | changed | 2.0 | 1.5 |
| Actuator | env.scene.robot.actuators.calfs.damping | changed | 6.5 | 7.0 |
| Actuator | env.scene.robot.actuators.calfs.stiffness | changed | 300.0 | 160.0 |
| Actuator | env.scene.robot.actuators.hips.damping | changed | 4.0 | 3.5 |
| Actuator | env.scene.robot.actuators.hips.stiffness | changed | 180.0 | 100.0 |
| Actuator | env.scene.robot.actuators.thighs.damping | changed | 4.0 | 3.5 |
| Actuator | env.scene.robot.actuators.thighs.stiffness | changed | 180.0 | 100.0 |
| Runner/Device | cli.--num_envs | changed | 2048 | 1024 |
| Runner/Device | cli.--run_name | changed | cfg31 | cfg32-base |
| Runner/Device | launcher.--nproc_per_node | changed | 2 | 4 |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 2,3 | 0,1,2,3 |
```
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Runner/Device | cli.--num_envs | changed | 2048 | 1024 |
| Runner/Device | cli.--run_name | changed | cfg30_zScale | cfg32-base |
| Runner/Device | launcher.--master_port | removed | 29501 | (missing) |
| Runner/Device | launcher.--nproc_per_node | changed | 2 | 4 |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1 | 0,1,2,3 |
```
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
```markdown
| group | key | change_vs_prev | change_vs_base | expected_impact | risk | monitor |
|---|---|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | 2.0 -> 1.5 | no change | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.calfs.damping | 6.5 -> 7.0 | no change | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.calfs.stiffness | 300.0 -> 160.0 | no change | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Actuator | env.scene.robot.actuators.hips.damping | 4.0 -> 3.5 | no change | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.hips.stiffness | 180.0 -> 100.0 | no change | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Actuator | env.scene.robot.actuators.thighs.damping | 4.0 -> 3.5 | no change | Adds damping and can improve motion smoothness. | Too much damping may reduce agility and tracking quality. | Monitor tracking error, settling time, and velocity lag. |
| Actuator | env.scene.robot.actuators.thighs.stiffness | 180.0 -> 100.0 | no change | Increases tracking authority and response speed. | Can increase oscillation and torque spikes. | Monitor torque limits, action_rate, and contact forces. |
| Runner/Device | cli.--num_envs | 2048 -> 1024 | 2048 -> 1024 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | cli.--run_name | cfg31 -> cfg32-base | cfg30_zScale -> cfg32-base | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | launcher.--nproc_per_node | 2 -> 4 | 2 -> 4 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | 2,3 -> 0,1,2,3 | 0,1 -> 0,1,2,3 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
```
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_END cfg32-base -->

<!-- RLCM:BATCH_START cfg33 -->
## Batch cfg33

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
batch_id: cfg33
task: Template-UniFPAliengo-v0
run_name: cfg33
status: planned
previous_batch_id: cfg31
base_id: cfg30_zScale
created_at: 2026-02-28T16:40:56
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: <CHECKPOINT_PATH>
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
CUDA_VISIBLE_DEVICES=0,1 torchrun --nproc_per_node=2 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg33 --distributed --num_envs 2048 env.scene.robot.actuators.hips.stiffness=180.0 env.scene.robot.actuators.hips.damping=4.0 env.scene.robot.actuators.thighs.stiffness=180.0 env.scene.robot.actuators.thighs.damping=4.0 env.scene.robot.actuators.calfs.stiffness=300.0 env.scene.robot.actuators.calfs.damping=6.5 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=2.0 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667 env.actions.arm_actions.scale=0.0 env.commands.target_position_commands.ranges.radius=[0.60,0.60] env.commands.target_position_commands.ranges.pitch_angle=[0.0,0.0] env.commands.target_position_commands.ranges.yaw_angle=[0.0,0.0] env.rewards.tracking_ee_force_world.weight=0.0
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
PUBLIC_IP=10.13.11.197 LIVESTREAM=1 ENABLE_CAMERAS=1 python scripts/rsl_rl/play.py --task Template-UniFPAliengo-v0 --resume --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-03-17_22-31-52_cfg33-Random-upSpeed/model_19400.pt --num_envs 128 --device cuda:1 \
    env.scene.robot.actuators.hips.stiffness=180.0 \
    env.scene.robot.actuators.hips.damping=4.0 \
    env.scene.robot.actuators.thighs.stiffness=180.0 \
    env.scene.robot.actuators.thighs.damping=4.0 \
    env.scene.robot.actuators.calfs.stiffness=300.0 \
    env.scene.robot.actuators.calfs.damping=6.5 \
    env.scene.robot.actuators.arm.stiffness=64.0 \
    env.scene.robot.actuators.arm.damping=2.0 \
    env.scene.robot.actuators.arm_shoulder.stiffness=128.0 \
    env.scene.robot.actuators.arm_shoulder.damping=3.0 \
    env.commands.gait_phase.gait_frequency=3.0 \
    env.rewards.feet_air_time.params.target_air_time=0.1667 \
    env.actions.arm_actions.scale=0.0 \
    env.commands.target_position_commands.ranges.radius=[0.60,0.60] \
    env.commands.target_position_commands.ranges.pitch_angle=[0.0,0.0] \
    env.commands.target_position_commands.ranges.yaw_angle=[0.0,0.0] \
    env.rewards.tracking_ee_force_world.weight=0.0 \
    agent.device=cuda:1
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Command | env.commands.target_position_commands.ranges.pitch_angle | added | (missing) | [0.0,0.0] |
| Command | env.commands.target_position_commands.ranges.radius | added | (missing) | [0.60,0.60] |
| Command | env.commands.target_position_commands.ranges.yaw_angle | added | (missing) | [0.0,0.0] |
| Reward | env.rewards.tracking_ee_force_world.weight | added | (missing) | 0.0 |
| Runner/Device | cli.--run_name | changed | cfg31 | cfg33 |
| Other | env.actions.arm_actions.scale | added | (missing) | 0.0 |
```
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Actuator | env.scene.robot.actuators.arm.damping | changed | 1.5 | 2.0 |
| Actuator | env.scene.robot.actuators.calfs.damping | changed | 7.0 | 6.5 |
| Actuator | env.scene.robot.actuators.calfs.stiffness | changed | 160.0 | 300.0 |
| Actuator | env.scene.robot.actuators.hips.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.hips.stiffness | changed | 100.0 | 180.0 |
| Actuator | env.scene.robot.actuators.thighs.damping | changed | 3.5 | 4.0 |
| Actuator | env.scene.robot.actuators.thighs.stiffness | changed | 100.0 | 180.0 |
| Command | env.commands.target_position_commands.ranges.pitch_angle | added | (missing) | [0.0,0.0] |
| Command | env.commands.target_position_commands.ranges.radius | added | (missing) | [0.60,0.60] |
| Command | env.commands.target_position_commands.ranges.yaw_angle | added | (missing) | [0.0,0.0] |
| Reward | env.rewards.tracking_ee_force_world.weight | added | (missing) | 0.0 |
| Runner/Device | cli.--run_name | changed | cfg30_zScale | cfg33 |
| Runner/Device | launcher.--master_port | removed | 29501 | (missing) |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1 | 2,3 |
| Other | env.actions.arm_actions.scale | added | (missing) | 0.0 |
```
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
```markdown
| group | key | change_vs_prev | change_vs_base | expected_impact | risk | monitor |
|---|---|---|---|---|---|---|
| Command | env.commands.target_position_commands.ranges.pitch_angle | set to [0.0,0.0] | set to [0.0,0.0] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Command | env.commands.target_position_commands.ranges.radius | set to [0.60,0.60] | set to [0.60,0.60] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Command | env.commands.target_position_commands.ranges.yaw_angle | set to [0.0,0.0] | set to [0.0,0.0] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Reward | env.rewards.tracking_ee_force_world.weight | set to 0.0 | set to 0.0 | Re-weights reward objectives. | Policy may exploit changed incentives. | Monitor reward decomposition and qualitative play behavior. |
| Runner/Device | cli.--run_name | cfg31 -> cfg33 | cfg30_zScale -> cfg33 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Other | env.actions.arm_actions.scale | set to 0.0 | set to 0.0 | Potential behavior change due to parameter update. | Unclear side-effects without targeted checks. | Monitor key task metrics and play stability. |
```
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_END cfg33 -->

<!-- RLCM:BATCH_START cfg34 -->
## Batch cfg34

### Metadata
<!-- RLCM:METADATA_START -->
```yaml
batch_id: cfg34
task: Template-UniFPAliengo-v0
run_name: cfg34
status: planned
previous_batch_id: cfg32-base
base_id: cfg30_zScale
created_at: 2026-02-28T16:41:06
analysis_mode: prev+base
public_ip: 10.13.11.197
play_device: cuda:1
play_num_envs: 128
checkpoint: logs/rsl_rl/uniFP_aliengo_position_force/2026-02-28_20-56-37_cfg34/model_22800.pt
```
<!-- RLCM:METADATA_END -->

### Train Command
<!-- RLCM:TRAIN_COMMAND_START -->
```bash
CUDA_VISIBLE_DEVICES=2,3 torchrun --nproc_per_node=2 scripts/rsl_rl/train.py --headless --task Template-UniFPAliengo-v0 --run_name cfg34 --distributed --num_envs 2048 env.scene.robot.actuators.hips.stiffness=100.0 env.scene.robot.actuators.hips.damping=3.5 env.scene.robot.actuators.thighs.stiffness=100.0 env.scene.robot.actuators.thighs.damping=3.5 env.scene.robot.actuators.calfs.stiffness=160.0 env.scene.robot.actuators.calfs.damping=7.0 env.scene.robot.actuators.arm.stiffness=64.0 env.scene.robot.actuators.arm.damping=1.5 env.scene.robot.actuators.arm_shoulder.stiffness=128.0 env.scene.robot.actuators.arm_shoulder.damping=3.0 env.commands.gait_phase.gait_frequency=3.0 env.rewards.feet_air_time.params.target_air_time=0.1667 env.actions.arm_actions.scale=0.0 env.commands.target_position_commands.ranges.radius=[0.60,0.60] env.commands.target_position_commands.ranges.pitch_angle=[0.0,0.0] env.commands.target_position_commands.ranges.yaw_angle=[0.0,0.0] env.rewards.tracking_ee_force_world.weight=0.0
```
<!-- RLCM:TRAIN_COMMAND_END -->

### Play Command (Auto)
<!-- RLCM:PLAY_COMMAND_START -->
```bash
PUBLIC_IP=10.13.11.197 LIVESTREAM=1 ENABLE_CAMERAS=1 python scripts/rsl_rl/play.py --task Template-UniFPAliengo-v0 --resume --checkpoint logs/rsl_rl/uniFP_aliengo_position_force/2026-02-28_20-56-35_cfg33/model_19800.pt --num_envs 128 --device cuda:1 \
    env.scene.robot.actuators.hips.stiffness=100.0 \
    env.scene.robot.actuators.hips.damping=3.5 \
    env.scene.robot.actuators.thighs.stiffness=100.0 \
    env.scene.robot.actuators.thighs.damping=3.5 \
    env.scene.robot.actuators.calfs.stiffness=160.0 \
    env.scene.robot.actuators.calfs.damping=7.0 \
    env.scene.robot.actuators.arm.stiffness=64.0 \
    env.scene.robot.actuators.arm.damping=1.5 \
    env.scene.robot.actuators.arm_shoulder.stiffness=128.0 \
    env.scene.robot.actuators.arm_shoulder.damping=3.0 \
    env.commands.gait_phase.gait_frequency=3.0 \
    env.rewards.feet_air_time.params.target_air_time=0.1667 \
    env.actions.arm_actions.scale=0.0 \
    env.commands.target_position_commands.ranges.radius=[0.60,0.60] \
    env.commands.target_position_commands.ranges.pitch_angle=[0.0,0.0] \
    env.commands.target_position_commands.ranges.yaw_angle=[0.0,0.0] \
    env.rewards.tracking_ee_force_world.weight=0.0 \
    agent.device=cuda:1
```
<!-- RLCM:PLAY_COMMAND_END -->

### Diff vs Previous (Auto)
<!-- RLCM:DIFF_PREV_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Command | env.commands.target_position_commands.ranges.pitch_angle | added | (missing) | [0.0,0.0] |
| Command | env.commands.target_position_commands.ranges.radius | added | (missing) | [0.60,0.60] |
| Command | env.commands.target_position_commands.ranges.yaw_angle | added | (missing) | [0.0,0.0] |
| Reward | env.rewards.tracking_ee_force_world.weight | added | (missing) | 0.0 |
| Runner/Device | cli.--num_envs | changed | 1024 | 2048 |
| Runner/Device | cli.--run_name | changed | cfg32-base | cfg34 |
| Runner/Device | launcher.--nproc_per_node | changed | 4 | 2 |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1,2,3 | 2,3 |
| Other | env.actions.arm_actions.scale | added | (missing) | 0.0 |
```
<!-- RLCM:DIFF_PREV_END -->

### Diff vs Base (Auto)
<!-- RLCM:DIFF_BASE_START -->
```markdown
| group | key | change | from | to |
|---|---|---|---|---|
| Command | env.commands.target_position_commands.ranges.pitch_angle | added | (missing) | [0.0,0.0] |
| Command | env.commands.target_position_commands.ranges.radius | added | (missing) | [0.60,0.60] |
| Command | env.commands.target_position_commands.ranges.yaw_angle | added | (missing) | [0.0,0.0] |
| Reward | env.rewards.tracking_ee_force_world.weight | added | (missing) | 0.0 |
| Runner/Device | cli.--run_name | changed | cfg30_zScale | cfg34 |
| Runner/Device | launcher.--master_port | removed | 29501 | (missing) |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | changed | 0,1 | 2,3 |
| Other | env.actions.arm_actions.scale | added | (missing) | 0.0 |
```
<!-- RLCM:DIFF_BASE_END -->

### Impact Notes (Auto)
<!-- RLCM:IMPACT_START -->
```markdown
| group | key | change_vs_prev | change_vs_base | expected_impact | risk | monitor |
|---|---|---|---|---|---|---|
| Command | env.commands.target_position_commands.ranges.pitch_angle | set to [0.0,0.0] | set to [0.0,0.0] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Command | env.commands.target_position_commands.ranges.radius | set to [0.60,0.60] | set to [0.60,0.60] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Command | env.commands.target_position_commands.ranges.yaw_angle | set to [0.0,0.0] | set to [0.0,0.0] | Changes command distribution or command semantics. | Can narrow or shift learned behavior envelope. | Monitor command tracking metrics and coverage. |
| Reward | env.rewards.tracking_ee_force_world.weight | set to 0.0 | set to 0.0 | Re-weights reward objectives. | Policy may exploit changed incentives. | Monitor reward decomposition and qualitative play behavior. |
| Runner/Device | cli.--num_envs | 1024 -> 2048 | no change | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | cli.--run_name | cfg32-base -> cfg34 | cfg30_zScale -> cfg34 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | launcher.--nproc_per_node | 4 -> 2 | no change | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Runner/Device | prefix_env.CUDA_VISIBLE_DEVICES | 0,1,2,3 -> 2,3 | 0,1 -> 2,3 | Changes runtime/training execution context. | Can alter throughput and reproducibility. | Monitor fps, wall-time, and seed consistency. |
| Other | env.actions.arm_actions.scale | set to 0.0 | set to 0.0 | Potential behavior change due to parameter update. | Unclear side-effects without targeted checks. | Monitor key task metrics and play stability. |
```
<!-- RLCM:IMPACT_END -->

### Manual Result Notes
- Fill training metrics and play observations.
<!-- RLCM:BATCH_END cfg34 -->
<!-- RLCM:BATCHES_END -->
