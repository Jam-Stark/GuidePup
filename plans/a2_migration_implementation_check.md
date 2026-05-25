# A2 Migration Implementation Check

## Scope

本检查针对当前分支在 `source/LMP/acc/tasks/manager_based/uniFP_aliengo` 内做的 A2 原地迁移，重点核对 observation、reward/termination、asset/randomization/event、actuator/goal/gait 四个方向是否按计划落实。

检查方式：
- Hooke: observations
- Zeno: rewards / terminations
- James: asset / randomization / events
- Huygens: actuator / goal / gait
- 主线程额外做静态 grep、diff、URDF 名称核对和 Python 编译检查

结论：训练用 active path 的硬迁移项基本正确落实，没有发现会直接导致 A2 body/joint lookup 失败、policy I/O 维度变化、或足端顺序错位的 blocker。仍有若干低风险项和训练基线参数需要后续 rollout/训练结果确认。

## Version History

- v1, recorded `2026-05-22 19:49:13 HKT`: A2 初始迁移基线。该版本完成 A2 asset/body/joint/foot 顺序迁移，并保留基础训练设置：`BASE_EXT_FULL_RANGE=(-65.0, 65.0)`、`BASE_FORCE_KD=160.0`、A2 actuator limits、PD v1 `hip/thigh=140.0/5.0`、`calf=220.0/9.0`、terrain `noise_range=(0.00, 0.03)`、leg action delay `(0, 1)`、feet height target `0.08`、feet high threshold `0.15`。
- v2, recorded `2026-05-22 19:49:13 HKT`: 在 v1 上增加当前增强项：`armature=0.03`、terrain `noise_range=(0.00, 0.05)`、leg action delay `(0, 2)`、feet height target `0.13`、feet high threshold `0.22`。其余未列参数继承 v1。

## Implemented Correctly

### Shared Robot Contract

- 已新增共享常量文件：`source/LMP/acc/tasks/manager_based/uniFP_aliengo/robot_constants.py`
  - `BASE_BODY_NAME = "base_link"`
  - `FOOT_BODY_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")`
  - `LEG_JOINT_NAMES` 保持 12DoF leg-only 顺序：hips FL/FR/RL/RR，thighs FL/FR/RL/RR，calfs FL/FR/RL/RR。
- A2 URDF 的实际 link/joint 名称和上述 contract 对得上；URDF body/link 顺序不是 policy contract 顺序，所以显式 preserve/order helper 是必要的。

### Asset / Body References

- `unifpaliengo_env_cfg.py` 的 robot asset 已指向 `resources/a2_description/urdf/a2.urdf`。
- active 训练配置里的 locomotion command body、base force action、base mass randomization、base COM randomization 都已改为 `base_link`。
- active Python 配置里未发现旧 Aliengo URDF path、`trunk`、`body_name="base"` 作为训练路径 body 引用。
- `penalize_contacts` 已从 trunk 改到 `base_link`，并保留 thigh/calf illegal contact 检测。

### Observations

- policy observation 仍保持单帧 46 维，history 32，flatten 后 1472：
  - `2 + 3 + 12 + 12 + 12 + 2 + 3 = 46`
- privileged observation 仍保持单帧 93 维，history 3，flatten 后 279：
  - `3 + 3 + 12 + 5 + 1 + 12 + 4 + 4 + 3 + 3 + 12 + 12 + 12 + 2 + 3 + 2 = 93`
- policy/privileged 的 `dof_pos`、`joint_vel` 都使用 `LEG_JOINT_NAMES` 并设置 `preserve_order=True`。
- `foot_contact_binary()` 通过 ordered contact force helper 输出 FL/FR/RL/RR 顺序。
- `compute_ref_dof()` 使用 `LEG_JOINT_NAMES` + `preserve_order=True`，ref dof 切片和 gait phase 顺序与 policy contract 对齐。

### Rewards / Terminations

- reward 中 joint torque、stand still、joint acc、joint vel、joint pos limit、torque limit、hip deviation 等顺序敏感 `SceneEntityCfg` 已用 `LEG_JOINT_NAMES` 或显式 hip list，并设置 `preserve_order=True`。
- feet contact、feet air time、feet drag、feet height、feet pos xy 等足端相关 reward 已改为 ordered foot/body/contact helper。
- `feet_height` active 配置已改为 `all_feet=True`。
- `feet_contact_forces` 的 `max_contact_force` 已改为 `240.0`。
- `base_height` reward target 已改为 `0.50`。
- base height termination minimum 已改为 `0.20`。

### Events / Randomization

- reset joints 和 randomize leg joint gains 已使用 `LEG_JOINT_NAMES` + `preserve_order=True`。
- eval reset joints 也已补上 `preserve_order=True`。
- base mass randomization 已改为 `base_link` 上 `add uniform (0.0, 8.0)` 并保持 recompute inertia。
- base COM randomization 已作用于 `base_link`。

### Actuator / Goal / Gait

- A2 actuator nominal limits 已按 URDF 落实：
  - hip/thigh: `effort_limit_sim=120.0`, `velocity_limit_sim=22.0`
  - calf: `effort_limit_sim=180.0`, `velocity_limit_sim=14.6667`
- PD v1 baseline 已更新为：
  - hip/thigh: `stiffness=140.0`, `damping=5.0`
  - calf: `stiffness=220.0`, `damping=9.0`
- v2 已同步 MuJoCo A2 XML 的 joint armature：hip/thigh/calf 均为 `armature=0.03`。
- `BASE_EXT_FULL_RANGE = (-65.0, 65.0)`，`BASE_FORCE_KD = 160.0` 已落实，并被 locomotion command、tracking reward、privileged base velocity offset 使用。
- training gait 主配置保持 open-loop trot：
  - `gait_frequency=2.0`
  - `phase_offset=0.5`
  - `target_air_time=0.25`

## Issues And Risks

### No Runtime Smoke Test Yet

- 已做静态检查和 `py_compile`，但没有完成 IsaacLab runtime smoke test。
- 当前 shell 里普通 `python` 无法 import `isaaclab`；尝试 `IsaacLab/isaaclab.sh -p` 也没有得到可用的 `isaaclab` import 环境。
- 因此还没有实际断言：
  - `robot.body_names` 包含 `base_link`
  - resolved leg joints 精确等于 `LEG_JOINT_NAMES`
  - contact sensor body order 精确等于 `FOOT_BODY_NAMES`
  - action dim / obs dim 在 IsaacLab manager 初始化后仍匹配预期

### Eval Config Is Not The Same Gait Baseline

- `unifpaliengo_eval_cfg.py` 仍把 reverse-push eval 的 gait frequency 覆盖为 `3.0`，并把 `target_air_time` 改为 `0.1667`。
- 如果该 eval 是专项反推测试，这可以作为例外；如果要用它验证 A2 迁移基线，应改回 `2.0 / 0.25` 或另建 A2 baseline eval。

### Base Force Full Range Is Curriculum-Gated

- `BaseForceExtCfg.force_range` 初始仍是 `(0.0, 0.0)`。
- `(-65.0, 65.0)` 通过 curriculum 在 `8000 * 24` step 后切入。
- 如果计划语义是“训练一开始就 full range”，当前实现不满足；如果保留原课程逻辑，则当前是合理基线。

### Base Force Z Axis Is Scaled

- `force_z_scale=0.1` 仍在，因此 Z 轴扰动力实际不是完整 `(-65, 65)`，而是采样后乘 0.1。
- 如果目标是 XY 主扰动，这合理；如果目标是 XYZ 都按 A2 质量比例放大，需要调整。

### PD Gain Is Nominal Baseline, Not Fixed Runtime Gain

- nominal PD 已保留，但 reset event 会对 stiffness/damping 乘 `0.9~1.1` 做域随机化。
- 如果“PD gain 保留”指 nominal baseline，这已满足；如果指运行时完全固定，应关闭或收窄该 randomization。

### Legacy / Inactive Defaults

- `mdp/actions.py` 中未启用的 `VariableForceActionCfg.body_name` 默认仍是 `"base"`。当前 active `ActionsCfg` 使用的是 `ApplyForceCommandActionCfg(body_name=BASE_BODY_NAME)`，不受影响；但以后若重新启用 legacy force action 会在 A2 上有 body lookup 风险。
- 部分 legacy shaped contact reward 函数未接入 active `RewardsCfg`，里面仍有硬编码 gait phase 或旧注释。当前不影响训练，但如果恢复启用，需要重新审查。

### Naming Still Says Aliengo

- 任务目录、class name、gym id、experiment name 仍包含 `aliengo` / `UniFPAliengo`。
- 这不影响物理 asset，但会影响外部脚本语义、日志目录和实验可读性。因为本轮计划明确“不新建任务包，原地迁移”，所以暂时可接受。

### Observation Robustness

- `foot_contact_binary()` 只读取 `contact_sensor.data.net_forces_w`，没有像某些 reward 函数一样 fallback 到 `force_matrix_w`。
- 当前 `ContactSensorCfg` 应提供 `net_forces_w`，但若后续 sensor 配置变化，privileged `foot_contacts` 会先受影响。
- `last_action` 没显式切 `leg_actions`，当前维度正确依赖于 `base_force_ext.dim=0`。如果以后让 base force 成为 policy action，policy/privileged obs 维度会自动变大。
- `compute_ref_dof()` 的 joint lookup 使用了 `preserve_order=True`，但没有像 foot body helper 一样校验 resolved name/count；当前常量和 URDF 对得上，属于健壮性增强项。

### Reward Semantics

- `feet_height(all_feet=True)` 目前对四足取 max height，只保证至少一只脚达到高度，不保证每个 swing foot 都满足高度目标。
- `feet_height_high` 同样基于 max height 聚合。
- base height reward/termination 使用 world root z，不是 terrain-relative height。当前 rough heightfield 高度很小，风险有限；若后续加大地形高度，可能需要 terrain-relative base height。
- contact mask 阈值仍是 sensor `force_threshold=1.5`；`240.0` 是过大接触力 penalty threshold，不是接触二值化阈值。这是合理分工，但需要避免误读。

### Randomization Implementation Caveat

- COM randomizer 当前在 startup mode 下安全；实现是对当前 COM 做 `+= rand_samples`。
- 如果未来改成 reset mode，可能产生累计 COM 偏移，需要改为基于 default COM 重新采样。

## Baseline Parameters Requiring Training Feedback

以下项目不是迁移正确性 blocker，但当前只是 A2 初始基线，需要通过 rollout、训练曲线和指标再调：

- actuator PD / inertia:
  - v1 hip/thigh `140.0 / 5.0`
  - v1 calf `220.0 / 9.0`
  - v2 armature `0.03`
  - gain randomization `0.9~1.1`
- torque utilization：
  - A2 `120 / 180 Nm` limit 是否导致 torque_limit penalty 过强或过弱
  - `torques`、`torque_limit`、`dof_acc_legs`、`dof_vel_legs` 权重
- base force：
  - `BASE_EXT_FULL_RANGE=(-65,65)`
  - `BASE_FORCE_KD=160`
  - curriculum start `8000*24`
  - `force_z_scale=0.1`
- mass / COM randomization：
  - base mass add `(0.0, 8.0)`
  - COM xyz `(-0.1, 0.1)`
- terrain:
  - v1 random uniform `noise_range=(0.00, 0.03)`
  - v2 random uniform `noise_range=(0.00, 0.05)`
- gait / ref dof：
  - `2 Hz`
  - `phase_offset=0.5`
  - `target_air_time=0.25`
  - `target_joint_pos_scale=0.35`
  - `calf_scale_factor=2.0`
  - `target_joint_pos_thd=0.1`
- height / contact rewards:
  - base height target `0.50`
  - termination min height `0.20`
  - v1 feet height target `0.08`
  - v1 feet high threshold `0.15`
  - v2 feet height target `0.13`
  - v2 feet high threshold `0.22`
  - max contact force `240.0`
  - binary contact threshold `1.5`
- observations:
  - policy obs delay `(0,2)`
  - v1 leg action delay `(0,1)`
  - v2 leg action delay `(0,2)`
  - noise levels on base orientation / angular velocity / joint pos / joint vel

## Recommended Next Checks

1. 在可 import IsaacLab 的环境里跑最小 env smoke test，断言 body/joint/contact 顺序和 action/obs dims。
2. 用 16 到 64 env 做短 rollout，检查：
   - base force action 不再 lookup 失败
   - `foot_contacts` 与 `stance_mask` 对角小跑顺序一致
   - `ref_dof_legs` 不出现腿序反相
   - torque utilization 是否大量贴近 1.0
   - contact force penalty 是否常态爆高
   - base height termination 是否过早触发
3. 决定 reverse-push eval 是否应该跟训练基线保持 `2 Hz / 0.25`，还是保留 `3 Hz / 0.1667` 作为专项测试。
4. 若 A2 初训站立/慢走不稳，优先调 PD、ref_dof scale/calf factor、feet height/contact force、base force curriculum，而不是再改 observation/action contract。

## Static Verification Already Completed

- `python -m py_compile` passed for changed A2 migration files.
- `git diff --check` passed.
- active Python search did not find old Aliengo URDF path, active `trunk`, or active `body_name="base"` references in the training cfg.
