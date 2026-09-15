# MuJoCo Experiment Memory

更新时间：2026-06-05 14:43 HKT

## 职责

alan 的长期职责是专精当前 MuJoCo sim2sim 实验的配置核查/解释，重点比较 IsaacLab train/play 与 MuJoCo replay/sim2sim pipeline 的配置、模型、observation/action mapping、delay、friction、policy export 与 runtime 参数。

## 回答约束

- alan 之后回答任何 MuJoCo experiment 相关问题前，必须先读取本 entry：`memory/mujoco-experiment/description.md`。
- 如果问题涉及当前排查状态、已完成事项或下一步，必须同时读取 `memory/mujoco-experiment/TODO.md` 和 `memory/mujoco-experiment/DONE.md`。
- 回答中必须给出 memory 位置证据和来源，例如引用本文件路径、对应 TODO/DONE 条目，以及用户提供信息的 timestamp/source。

## 当前用户给出的 failing command / 触发入口

- 来源：用户 2026-06-03 调研请求。
- 触发入口：`scripts/mujoco/play_policy.py`。
- 完整 command：

```bash
DISPLAY=:101 XAUTHORITY=$HOME/.Xauthority PYTHONUNBUFFERED=1 python scripts/mujoco/play_policy.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/2026-05-23_14-25-38_A2_base_only_v2/mujoco_exported/policy.pt \
  --model-path resources/a2_description/a2.xml \
  --locomotion-cmd 0.5 0.0 0.0 \
  --obs-delay-steps 0 \
  --leg-action-delay-steps 0 \
  --ground-friction 1.5 0.02 0.02 \
  --foot-friction 1.5 0.02 0.02 \
  --duration-s 3600 \
  --real-time
```

## 已知现象

- IsaacLab train/play 表现正常。
- sim2sim 到 MuJoCo 时，在 `scripts/mujoco/play_policy.py` 运行后机器人直接摔倒。
- 当前尚未确认 root cause；初始假设集中在 play_policy 参数、action delay、obs delay、friction、MuJoCo model、policy export/loading 与 IsaacLab/MuJoCo runtime 配置对齐。
- 来源：用户 2026-06-03 23:20 HKT 补充说明。当前 A2 XML 不是按本 policy/runtime 新写的 sim2sim XML；之前 MuJoCo experiment 是测试 Aliengo，Aliengo XML 由用户自己写过，而现在 A2 XML 完全使用原官方版本、尚未做迁移适配。

## 当前调研结论摘要

- 2026-06-03 23:04 HKT：首轮静态/短命令调研显示最高优先级风险是 MuJoCo `model.opt.timestep=0.002` 与 exported metadata `sim_dt=0.005` 不一致；`play_policy.py` 仍按 `control_decimation=4` 调 `mj_step`，导致实际 control step 是 `0.008s`，而训练/metadata 是 `0.020s`。
- observation layout 初查显示当前 MuJoCo runtime 的 per-term history flatten 方式与本机 IsaacLab `ObservationManager` 代码一致；repo 内“按时间维展平/last 46 dims current frame”的文字说明可能过时，需要 john 后续确认 cfg/docs 语义。
- `policy.pt` 可由 `conda run -n mujoco` 正常加载，export wrapper 使用 Identity normalizer；policy export 本身暂未见 running stats 缺失问题。
- 2026-06-03 23:20 HKT：主控完成三组 10s headless timing 对比。当前 `0.002,nstep=4` 实际 action hold `0.008s`，10s 内未触发 fall 判定但 torque saturation 很多；`0.005,nstep=4` 和 `0.002,nstep=10` 都满足 `0.020s` action hold，但分别在约 1.06s 和 0.38s 触发 fall 判定。结论：timing mismatch 是真实问题，但单独修正 action hold 不能稳定 A2 MuJoCo，反而暴露更深的 A2 XML / dynamics / PD / delay-noise 对齐问题。
- 2026-06-03 23:28 HKT：已对 `resources/a2_description/a2.xml` 做最小 A2 MJCF patch：显式 `<option timestep="0.005" gravity="0 0 -9.81"/>`，并把 hip/thigh/calf joint default passive damping 改为 active cfg nominal `4.5/4.5/9.0`，armature 保持 `0.03`。当前 XML 使用 torque `motor` actuator，cfg stiffness 走 `scripts/mujoco/runtime.py` 显式 PD torque 路径，未写成 MuJoCo passive joint stiffness，避免生成固定 `springref` spring；exported `policy.json` 里 hip/thigh `kv=5.0` 与 active cfg `4.5` 的 metadata drift 仍留给后续 PD metadata TODO。
- 2026-06-04 10:57 HKT：`scripts/mujoco/staged_direction_force_eval.py` 已扩展为纯 MuJoCo force estimator benchmark：默认保留 legacy staged 10-40N profile，新增 `smooth_0_20` profile、`--cmd-vx/--cmd-vy/--cmd-wz` velocity CLI、0-20N smoothstep ramp/hold/release/idle 参数和 `<output-csv>.summary.json` per-direction metrics。3s headless smoke 使用 A2 policy/XML 与 `--force-profile smooth_0_20 --cmd-vx 0.0` 通过，runtime 输出 `model_timestep=0.0050 control_dt=0.0200`。
- 2026-06-04 11:03 HKT：主控完成 MuJoCo force estimator full matrix 和 plots。`cmd_vx=0.0/0.3/0.5` 三组 `smooth_0_20` 45s headless horizon 均通过，runtime 均为 `model_timestep=0.0050 control_dt=0.0200`；summary active L2 mean/peak 分别为 `8.014/18.892`、`8.490/18.972`、`8.967/25.601`。CSV/summary/plot 产物位于 `logs/experiments/unifp_force_estimator_mujoco/`；plot command 打印 `Invalid MIT-MAGIC-COOKIE-1 key` warning，但 exit code 0 且 PNG saved。
- 2026-06-05 14:43 HKT：旧 run `policy.json` 与 `params/env.yaml` / `params/agent.yaml` 对齐，不是明显坏 export；但 current source 的 hip/thigh damping 已变为 `4.5`，而旧 run logged cfg、eval cfg 与 export metadata 是 `5.0`。更长期的 export risk 是 `export_policy_wrapper.py` 仍从 `runtime.default_smart_gait_metadata()` 写 `joint_actuator_gains`，不是从 instantiated env/articulation 动态读取。

## TODO Summary

- 剩余 TODO 已改写在 `TODO.md`：显式 timestep 与 nominal leg passive damping/armature patch 已完成；Force estimator sweep runner、full matrix、plots 和旧 export cfg drift 核查已完成。当前 open work 回到 A2 XML migration、MuJoCo official semantics、`policy_vec[:, -46:]` current-slice 风险和 Isaac/MuJoCo first divergence trace。

## DONE Summary

- 2026-06-03 22:51 HKT：创建 `mujoco-experiment` memory entry，记录 alan 职责、回答约束、已知 failing path/现象与初始排查 TODO。
- 2026-06-03 23:04 HKT：完成首轮 failing command、runtime timing、observation history、delay、friction/model、policy export 静态与短命令调研，并记录最高优先级风险。
- 2026-06-03 23:20 HKT：记录用户确认 A2 XML 沿用官方版本且未针对当前 policy/runtime 做迁移适配；记录三组 timing 最小验证结果。
- 2026-06-03 23:28 HKT：完成 A2 XML 最小 patch：`timestep=0.005`、hip/thigh/calf passive damping `4.5/4.5/9.0`、armature `0.03` 保持；`check_setup.py` 确认 `model_timestep=0.0050`。
- 2026-06-04 10:57 HKT：完成 MuJoCo Force Estimator Sweep Plan implementation：`staged_direction_force_eval.py` 支持 command velocity CLI、legacy/smooth profile、0-20N nonlinear force schedule 和 summary JSON metrics，并完成 3s `conda run -n mujoco` headless smoke。
- 2026-06-04 11:03 HKT：完成 `smooth_0_20` force estimator full matrix：`cmd_vx=0.0/0.3/0.5` 三组 CSV、summary JSON 和 force timeseries PNG 均生成。
- 2026-06-05 14:43 HKT：完成旧 run export metadata 核查：`policy.json` 与 logged training cfg 对齐，但与 current active source 有 hip/thigh damping drift；记录 export wrapper 的 hard-coded PD metadata 风险。
