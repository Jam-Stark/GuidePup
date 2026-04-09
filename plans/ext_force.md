# Ext Force Disturbance-Only Environment Plan

## 目标

将 `uniFP_aliengo` 裁剪成“只学 base external force 扰动下抗扰”的任务。

本轮讨论先固化需求，不直接修改环境代码。以下内容是当前确认的裁剪方向和它们对应的代码落点。

默认假设：

- 保留腿部 locomotion 控制。
- 保留 `policy.history_length = 32`。
- 保留 `CommandsCfg.locomotion_commands.rel_standing_envs`。
- 保留 `RewardsCfg.stand_still`，不把当前任务进一步裁成与 gait 解耦的 pure-standing variant。
- 当前轮不把任务进一步裁成“纯 standing-only”；若后续确认只做 standing，再继续关掉 gait / locomotion command。
- 当前仓库已经切到 ext-force 专门分支；后续实现允许直接原地重构当前 task-local helper / cfg，不以兼容旧 manipulation 任务为目标。

## IsaacLab Manager-Based Compliance Constraint

强约束：

- 最终实现必须严格停留在 IsaacLab 的 **manager-based RL workflow** 内。
- 允许的主要改动入口应仅限于：
  - `CommandsCfg`
  - `ActionsCfg`
  - `ObservationsCfg`
  - `RewardsCfg`
  - `TerminationsCfg`
  - `EventCfg`
  - `CurriculumCfg`
  - 以及这些 manager term 所调用的 task-local `mdp/*.py` helper / `ManagerTermBase` 子类

明确禁止的实现方向：

- 不通过覆写 `ManagerBasedRLEnv.step()` 去手搓任务主逻辑
- 不把 reward / observation / command 语义偷偷搬到训练脚本或评估脚本里
- 不在脚本侧做“外力推断后再回写环境状态”的旁路逻辑
- 不引入 direct-workflow 风格的 `_get_rewards()` / `_get_observations()` 集中实现

官方依据：

- Manager-based workflow 要求任务被拆成 observations / actions / rewards / commands / events / curriculum 等独立 manager 组件
- Commands 在官方语义中用于 goal-conditioned tasks 的目标/命令；如果任务不需要某类 command，可以将该 manager 或 term 设为 `None`

文档依据：

- `/home/baoquanc/workspace/IsaacLab/docs/source/overview/core-concepts/task_workflows.rst`
- `/home/baoquanc/workspace/IsaacLab/docs/source/tutorials/03_envs/create_manager_base_env.rst`
- `/home/baoquanc/workspace/IsaacLab/docs/source/tutorials/03_envs/create_manager_rl_env.rst`
- Context7 `/isaac-sim/isaaclab` topic: `manager based workflow actions observations rewards commands events curriculums`

本次复核结论：

- `arm_actions -> FixedJointPositionActionCfg` 是符合 manager-based workflow 的
- `predObs` 作为单独 observation group 提供 estimator supervision 是符合 manager-based workflow 的
- 用 task-local `mdp` helper 重写 observation / reward 逻辑是符合 manager-based workflow 的
- 删除不用的 command terms 也是符合 manager-based workflow 的
- 唯一需要收紧表述的地方是：在 ext-force-only 任务里，`base_impedance_command` 不应再被当作“command”长期保留

## 任务本质澄清

用户当前明确的目标不是“让 robot 抵抗外力后仍尽量保持原始速度命令不变”，而是训练一个 **能够感知并顺应 base external force 的 compliant locomotion policy**。

目标行为定义：

- 从静止开始，如果 base 持续受到外力牵引，robot 应能够被牵引启动并跟随该外力方向运动。
- 在运动过程中，如果 base 受到反向外力，robot 应能够被拉慢，甚至被拉停。
- 这种速度调整应是实时的、连续的，不是 episode 级别的模式切换。
- 全过程中，force estimator 应持续预测外力；当前确定的 estimator supervision 目标为 `6D = base_lin_vel + base_forces_local`。

结论：

- 该任务更准确的表述是：`external force modulates locomotion behavior online`
- 因此 ext force 在任务里不是纯噪声源，而是实时影响 robot 目标行为的外部交互信号

与现有实现最接近的代码语义：

- `mdp.base_lin_vel_offset()` 已经把 force 映射为速度偏移：`v_eff = v_cmd + F / Kd`
- `mdp.tracking_lin_vel_force_world()` 已经按 force-offset velocity 在做 tracking reward

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py:657-714`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/rewards.py:189-254`

因此，后续实现时的原则不是“删除所有与 force-offset velocity 有关的逻辑”，而是：

- 保留“外力实时调速”这一任务语义
- 但把现有公式里对 `base_force_command` 的依赖去掉，只保留 `base_force_ext`
- 同时把 `Kd` 从 `base_impedance_command` 中解耦出来，改成 observation / reward term 的固定参数，而不是继续通过 `CommandManager` 挂一个伪 command

## estimator 与 policy 的关系

当前 actor / estimator 架构是：

- history encoder 消化 `32` 帧历史，输出 latent
- actor 使用 `current obs + latent`
- state estimator 使用 latent 预测 `predObs` 监督目标

直接代码位置：

- `source/LMP/acc/algorithms/rsl_rl/modules.py:16-126`
- `source/LMP/acc/algorithms/rsl_rl/algorithm.py:25-107`

基于该结构，本任务里“policy 能 sense 外界 force”应解释为：

- actor 不必直接接收 oracle external force
- actor 可以通过 history encoder 的 latent 去在线推断 external force
- estimator 对 `predObs = base_lin_vel + base_forces_local` 的监督，目的是迫使 latent 编码外力及其对 base 运动的影响

这也意味着：

- `predObs` 中保留 `base_lin_vel` 是有意义的，因为它不是偏离目标，而是 force 作用结果的一部分
- 当前 6D supervision 与用户目标是一致的

## 当前确认的需求

### 1. 只保留 `base_force_ext`，关闭 gripper force 分支、`base_force_command`，并移除 `base_impedance_command` 的 command 角色

需求：

- 保留 `CommandsCfg.base_force_ext`，因为这是唯一需要训练 policy 去应对的外扰源。
- 保留 `ActionsCfg.base_force_ext`，因为真正把 command 施加到 base 的动作项在这里。
- 关闭 `CommandsCfg.gripper_force_ext`、`CommandsCfg.gripper_force_command`、`CommandsCfg.base_force_command`。
- 移除 `CommandsCfg.base_impedance_command` 作为 command term 的角色。
- `policy` 不再接收任何 gripper force / base command-force 输入。

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:275-321`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:603-612`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/actions.py:404-480`

必须同步处理的耦合项：

- 当前 `policy.commands` 通过 `mdp.all_commands_with_scales()` 聚合了 locomotion、EE goal、gripper force、base force command。
- 当前 `base_lin_vel_offset()` / `tracking_lin_vel_force_world()` 还通过 `base_impedance_command` 读取 `Kd`。
- 如果 `gripper_force_command` / `base_force_command` 被关闭，当前 helper 不能原样保留。
- 如果 `base_impedance_command` 被移除，`Kd` 必须转为 reward / observation term 参数。

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:706-739`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py:553-606`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py:657-714`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/rewards.py:189-254`

结论：

- 不能只把 force range 设为零了事；要么删掉 `policy.commands` 里的相关维度，要么改写 helper 让它只输出保留项。
- 当前推荐是把 `policy.commands` 改成“只保留 locomotion command”的轻量版本，不把任何 force command 喂给 actor。
- 从 manager-based 规范上看，`base_impedance_command` 作为固定任务参数并不适合继续走 `CommandManager`；后续实现应把它下沉为 `ObsTerm/RewTerm` 的普通 `params`

### 2. force estimator / adaptation target 只关注 base disturbance

需求：

- `predObs` 不再包含任何机械臂/末端/夹爪扰动信息。
- 至少要删除 `ee_goal_pos` 和 `gripper_forces_local`。
- estimator 的目标应收缩到 base 相关信息。

当前实现：

- `predObs = base_lin_vel + ee_goal_pos + gripper_forces_local + base_forces_local`
- 总维度为 `12`

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:946-987`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py:15-31`

最终确认的裁剪方案：

- `predObs = base_lin_vel + base_forces_local`
- 对应 `prediction_dim = 6`
- `obs_pred_weights` 必须同步改成长度 `6`

本轮记录的结论：

- 已确认 estimator 不再关心 gripper / EE 分支。
- 已确认保留 `base_lin_vel` 作为辅助预测目标。
- 不再保留 `3D = base_forces_local` 方案作为当前实现选项。

### 3. 冻结 arm 的方式改为“固定默认关节位置”，而不是“固定 EE goal”

需求：

- 不再通过固定 `target_position_commands` 去间接冻结机械臂。
- 直接关闭机械臂控制，把 arm 关节固定在默认位置。

当前实现：

- `arm_actions` 仍是可学习的 `JointPositionActionCfg`
- 环境还保留 `target_position_commands`
- EE 跟踪 reward `tracking_ee_force_world` 仍在使用

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:267-269`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:591-597`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1486-1497`

可直接复用的固定动作实现：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/actions.py:482-545`

实现要求：

- 将 `ActionsCfg.arm_actions` 从 `mdp.JointPositionActionCfg` 改为 `mdp.FixedJointPositionActionCfg`
- 固定 joint 至少包括 `arm_joint1..arm_joint5`
- `arm_joint6` 也应一并固定在默认位置；如果现有 `arm_actions` term 不覆盖它，则需要额外增加一个 fixed action term，避免 wrist 悬空

关联影响：

- 一旦 arm 被固定，`target_position_commands`、`gripper_impedance_command`、`tracking_ee_force_world`、相关 gripper force 分支都应一起退出任务定义

### 4. 删除 policy / reward / privileged / pred 中所有 arm / manipulation 相关内容

需求：

- `policy` 不再携带 arm joint 状态、EE command、gripper force command。
- reward 不再为 arm motion 或 EE 跟踪服务。
- `privileged` 和 `pred` 也要同步缩成 base/legs-only，避免 critic 继续依赖 manipulation 状态。

#### 4.1 Policy observation

当前 actor 单帧输入是：

- `base_orientation` 2
- `base_ang_vel` 3
- `dof_pos` 17
- `joint_vel` 17
- `last_action` 17
- `gait_clock` 2
- `commands` 15

总计：

- `obs_curr_dim = 73`

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:648-745`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py:17-19`

裁剪要求：

- `dof_pos` 只保留 12 个 leg joints，删掉 `arm_joint1..arm_joint5`
- `joint_vel` 只保留 12 个 leg joints
- `last_action` 应从 17 维降到 12 维
- `commands` 改成 locomotion-only，不再包含 EE goal / EE orient / gripper force / base force command

如果保留 `gait_clock` 与 locomotion command，则新的 actor 单帧输入维度应为：

- `2 + 3 + 12 + 12 + 12 + 2 + 3 = 46`

即：

- `obs_curr_dim: 73 -> 46`

#### 4.2 Privileged observation

当前 `privilegedObs` 仍包含：

- `ee_pos`
- `gripper_forces_local`
- arm joints in `dof_pos` / `joint_vel`
- full `commands`

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:747-944`

裁剪要求：

- 删除所有 EE / gripper / arm-only 项
- `privileged.commands` 也要同步缩成 locomotion-only 或 base-only 版本
- `base_lin_vel_offset` 目前依赖 `base_force_command` 与 `base_impedance_command`，如果这两个被移除，则对应 privileged term 也必须重写或删除

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:916-927`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py:657-714`

#### 4.3 Reward

应删除或收缩的 arm / manipulation reward 项：

- `dof_acc_arm`
- `dof_vel_arm`
- `action_rate_arm`
- `tracking_ee_force_world`

`torque_limit` 当前 joint list 也包含 arm joints，应收缩到 legs-only。

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1395-1409`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1421-1424`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1435-1446`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1486-1497`

另外，`tracking_lin_vel_force_world` 当前仍使用：

- `base_force_ext`
- `base_force_command`
- `base_impedance_command`

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:1303-1314`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/rewards.py:189-254`

结论：

- 如果确定删除 `base_force_command`，这个 reward helper 必须同步简化成只看 `base_force_ext`
- 如果 `base_impedance_command` 也移除，则该 reward helper 必须改成从 term params 读取固定 `Kd`
- 否则 reward 仍会对已经“关闭”的 command 分支有硬依赖
- `stand_still` 继续保留，不作为本轮裁剪对象

### 5. `policy.history_length` 保持不变

需求：

- 保持 `PolicyCfg.history_length = 32`
- 保持 `history_num_frames = 32`

直接代码位置：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py:741-745`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py:16-19`

同步要求：

- 虽然 history 长度不变，但 `obs_curr_dim` 必须从 `73` 改到新的单帧维度
- 若按当前计划保留 gait + locomotion command，则应改为 `46`
- 若后续再裁成 standing-only，则该维度还会进一步下降

## 建议的最小实现顺序

1. 先把 arm 改成 fixed joint action，确认 action dim 从 `17` 缩到 `12`
2. 删掉 gripper / EE / base command-force 分支，并改写 actor `commands`
3. 将 `base_impedance_command` 从 command 角色改为固定 term 参数，并同步重写 `base_lin_vel_offset` / `tracking_lin_vel_force_world`
4. 同步裁剪 `predObs` 为 `6D = base_lin_vel + base_forces_local`，并更新 `prediction_dim` / `obs_pred_weights`
5. 同步裁剪 `privilegedObs`
6. 清理 arm/manipulation rewards
7. 最后更新 `obs_curr_dim`

## 已确认项补充

- `predObs` 固定采用 `6D`：`base_lin_vel + base_forces_local`
- `prediction_dim` 固定改为 `6`
- `arm` 不再由 policy 直接控制，改为固定默认姿势
- 当前分支允许直接重构现有 shared helper，不再额外保留 manipulation 兼容层

## Implement Plan

以下 checklist 按文件分组组织。实施时建议按分组顺序推进；每完成一组，就对照该组的验收点自检，再继续下一组。

### A. 环境主配置

目标文件：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`

Checklist：

- [ ] `CommandsCfg` 仅保留 ext-force 任务还需要的 command terms：`locomotion_commands`、`gait_phase`、`base_force_ext`、metrics
- [ ] 删除或禁用 `target_position_commands`
- [ ] 删除或禁用 `gripper_impedance_command`
- [ ] 删除或禁用 `base_impedance_command`
- [ ] 删除或禁用 `gripper_force_ext`
- [ ] 删除或禁用 `gripper_force_command`
- [ ] 删除或禁用 `base_force_command`
- [ ] `ActionsCfg.arm_actions` 改为 `mdp.FixedJointPositionActionCfg`
- [ ] arm 默认固定姿势覆盖 `arm_joint1..arm_joint6`
- [ ] `ActionsCfg.leg_actions` 保持可学习
- [ ] `ActionsCfg.base_force_ext` 保持有效
- [ ] `ObservationsCfg.policy.dof_pos` 缩到 12 个 leg joints
- [ ] `ObservationsCfg.policy.joint_vel` 缩到 12 个 leg joints
- [ ] `ObservationsCfg.policy.last_action` 对应总 action dim 收缩到 12
- [ ] `ObservationsCfg.policy.commands` 改成 locomotion-only
- [ ] `ObservationsCfg.privileged` 删除 EE / gripper / arm-only 项
- [ ] `ObservationsCfg.pred` 收缩为 `base_lin_vel + base_forces_local`
- [ ] `RewardsCfg` 删除 `dof_acc_arm`
- [ ] `RewardsCfg` 删除 `dof_vel_arm`
- [ ] `RewardsCfg` 删除 `action_rate_arm`
- [ ] `RewardsCfg` 删除 `tracking_ee_force_world`
- [ ] `RewardsCfg.torque_limit` joint list 收缩到 legs-only
- [ ] `RewardsCfg.stand_still` 保留
- [ ] `CommandsCfg.locomotion_commands.rel_standing_envs` 保留

分组验收点：

- [ ] 环境 cfg 中不再有 ext-force-only 任务不需要的 manipulation / gripper / base command-force runtime term 引用
- [ ] `policy` 单帧维度目标收敛到 `46`
- [ ] action 总维度目标收敛到 `12`

### B. 观测 helper

目标文件：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py`

Checklist：

- [ ] 直接重写 `all_commands_with_scales()`，使其仅聚合 locomotion command
- [ ] `all_commands_with_scales()` 的输出维度改为 3
- [ ] `base_lin_vel_offset()` 删除对 `base_force_command` 的依赖
- [ ] `base_lin_vel_offset()` 删除对 `base_impedance_command` 的依赖
- [ ] `base_lin_vel_offset()` 改为只使用 `base_force_ext + fixed Kd`
- [ ] 若 `ee_goal_offset_local_sphere()` 等 manipulation helper 已完全无引用，可一并清理

分组验收点：

- [ ] 观测 helper 不再访问已删除的 command names
- [ ] `base_lin_vel_offset()` 的 term 参数接口与新的 `ObsTerm.params` 保持一致

### C. Reward helper

目标文件：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/rewards.py`

Checklist：

- [ ] 直接重写 `tracking_lin_vel_force_world()`
- [ ] `tracking_lin_vel_force_world()` 删除对 `base_force_command` 的依赖
- [ ] `tracking_lin_vel_force_world()` 删除对 `base_impedance_command` 的依赖
- [ ] `tracking_lin_vel_force_world()` 改为只使用 `base_force_ext + fixed Kd`
- [ ] 若 `tracking_ee_force_world()` 已无任何引用，可一并删除

分组验收点：

- [ ] reward helper 不再访问已删除的 command names
- [ ] `tracking_lin_vel_force_world()` 的 term 参数接口与新的 `RewTerm.params` 保持一致

### D. 训练配置

目标文件：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py`

Checklist：

- [ ] `prediction_dim` 从 `12` 改为 `6`
- [ ] `obs_curr_dim` 从 `73` 改为 `46`
- [ ] `history_num_frames` 保持 `32`
- [ ] `obs_pred_weights` 改为长度 `6`
- [ ] `obs_groups` 继续指向 `policy / privileged / pred`

分组验收点：

- [ ] actor / estimator 配置维度与环境观测真实维度一致
- [ ] 没有遗留针对旧 12D `predObs` 的权重配置

### E. 评估环境配置

目标文件：

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_eval_cfg.py`

Checklist：

- [ ] 删除对 `target_position_commands` 的评估时修改
- [ ] 删除对 `gripper_force_ext` 的评估时修改
- [ ] 删除对 `gripper_force_command` 的评估时修改
- [ ] 删除对 `base_force_command` 的评估时修改
- [ ] 删除对 `tracking_ee_force_world` 的评估时修改
- [ ] 不再通过 `arm_actions.scale = 0.0` 冻结机械臂，而是复用训练环境里的 fixed action 定义
- [ ] 保留 deterministic `base_force_ext` scripted evaluation 逻辑

分组验收点：

- [ ] eval cfg 不再覆盖已经删除的 term
- [ ] eval cfg 在 ext-force-only 环境上可独立实例化

### F. 评估脚本与日志脚本

目标文件：

- `scripts/experiments/uniFP_aliengo/staged_direction_force_eval_ref_standing.py`
- `logs/experiments/unifp_staged_direction_force_ref_standing/plot_staged_direction_force_csv.py`

Checklist：

- [ ] `staged_direction_force_eval_ref_standing.py` 中 estimator 输出切片从旧 12D 逻辑改为新 6D 逻辑
- [ ] 评估 CSV 不再输出 gripper force estimation columns
- [ ] 评估脚本只记录 base disturbance estimation 相关字段
- [ ] plot 脚本删除或停用 gripper force figure 逻辑
- [ ] plot 脚本仅保留 base force estimation 可视化

分组验收点：

- [ ] CSV header 与 ext-force-only estimator target 一致
- [ ] 画图脚本不再依赖 gripper force 列

## Checklist 验收方式

以下验收按“静态检查 -> 构建检查 -> 运行时检查 -> 行为检查”四层执行。只有前一层通过后，才进入下一层。

### 1. 静态检查

目标：

- 确认代码中不再存在 ext-force-only 任务已经移除的 runtime term 依赖
- 确认关键维度配置已经同步完成

建议检查命令：

```bash
rg -n "target_position_commands|gripper_impedance_command|gripper_force_ext|gripper_force_command|base_force_command|base_impedance_command|tracking_ee_force_world" \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo \
  scripts/experiments/uniFP_aliengo \
  logs/experiments/unifp_staged_direction_force_ref_standing
```

```bash
rg -n "prediction_dim|obs_curr_dim|history_num_frames|obs_pred_weights|history_length" \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo
```

```bash
rg -n "rel_standing_envs|stand_still" \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo
```

静态验收标准：

- [ ] 移除项仅允许出现在文档、注释或明确保留的 dead-code 清理待办中；不能再出现在有效 runtime 路径
- [ ] `prediction_dim = 6`
- [ ] `obs_curr_dim = 46`
- [ ] `history_length = 32`
- [ ] `history_num_frames = 32`
- [ ] `rel_standing_envs` 仍在
- [ ] `stand_still` 仍在

### 2. 构建检查

目标：

- 先确认改动后的 Python 文件能通过最基础的语法检查

建议检查命令：

```bash
python -m py_compile \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_eval_cfg.py \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/rewards.py \
  source/LMP/acc/tasks/manager_based/uniFP_aliengo/agents/rsl_rl_ppo_cfg.py \
  scripts/experiments/uniFP_aliengo/staged_direction_force_eval_ref_standing.py \
  logs/experiments/unifp_staged_direction_force_ref_standing/plot_staged_direction_force_csv.py
```

构建验收标准：

- [ ] 所有目标文件都能通过 `py_compile`

### 3. 运行时检查

目标：

- 确认 manager-based 环境仍能正确组装
- 确认不存在 `CommandManager` / `ObservationManager` / `RewardManager` 的缺项引用

运行时验收标准：

- [ ] 训练环境可以实例化
- [ ] 评估环境可以实例化
- [ ] 不出现 `get_command()` / `get_term()` 查找已删除 term 的报错
- [ ] `policy` 观测、`privileged` 观测、`pred` 观测维度与训练配置一致
- [ ] action 维度为 `12`

### 4. 行为检查

目标：

- 确认 ext-force-only 任务语义成立
- 确认 estimator 只围绕 base disturbance 工作

行为验收标准：

- [ ] 在 staged-direction-force eval 中，CSV 只保留 base force estimation 字段
- [ ] plot 脚本可以直接消费新的 CSV 结构
- [ ] estimator 输出只对应 `base_lin_vel + base_forces_local`
- [ ] policy 在外力作用下表现为在线调速，而不是固定追踪原始速度命令

## 实施顺序建议

推荐执行顺序：

1. 先完成 A 组中的 `ActionsCfg`、`CommandsCfg`、`ObservationsCfg.pred`
2. 再完成 B / C 两组 helper 重写，去掉已删除 command 的硬依赖
3. 回到 A 组完成 `policy` / `privileged` / `reward` 收尾
4. 完成 D 组训练配置同步
5. 完成 E / F 组评估与日志链路同步
6. 最后按“Checklist 验收方式”从静态检查一路跑到行为检查
