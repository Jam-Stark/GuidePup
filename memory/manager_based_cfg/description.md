# Manager-Based CFG Memory

更新时间：2026-06-05 14:43 HKT

## 职责

john 的长期职责是专精当前 manager-based 版本 cfg 的核查、解释和修改方案，当前主入口是 `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`。重点关注 IsaacLab train/play 与 MuJoCo `play_policy` 的 sim2sim 对齐，包括 observation/action scaling、HistoryWrapper、command routing、force/position control、delay、noise 和 domain randomization。

## 回答约束

- john 之后回答 manager-based cfg 相关问题时，回答开头必须先给 code 引用，方便用户核查。
- code 引用优先指向 `unifpaliengo_env_cfg.py` 的具体行；涉及 action/command/delay/utility 实现时，同时引用对应 supporting source。
- 开始实现、调试、review 或文档更新前，先读根目录 `MEMORY.md`；进入本职责时先读本文件。需要判断当前状态或更新 TODO 时，再读 `TODO.md` 和 `DONE.md`。

## 本次实际读取范围

- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/robot_constants.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/actions.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/modifiers.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/commands/force_command.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/commands/velocity_command.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/commands/gait_phase_command.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/commands/gait_phase_command_cfg.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/observation_compute.py`
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/mdp/utils.py`

## 当前 cfg 关键字段摘要

- Environment：`UniFPAliengoEnvCfg` 继承 `ManagerBasedRLEnvCfg`，`scene` 为 `UniFPSceneCfg(num_envs=4096, env_spacing=3.0)`；`decimation=4`、`episode_length_s=20`、`sim.dt=1/200`，所以 env step 是 0.02s / 50Hz；gravity 为 `(0, 0, -9.81)`，PhysX 使用 TGS solver，position iterations 4，velocity iterations 0。
- Robot / joint order：base body 是 `base_link`；12 个 leg joints 顺序为 FL/FR/RL/RR hip，然后 thigh，然后 calf。cfg 中 leg observation/action/reset/reward 多处使用 `preserve_order=True`，sim2sim 必须按这个顺序映射。
- Terrain / scene：terrain importer 使用 generator；sub terrain 为 40% flat、60% rough_flat，rough height noise range `(0.00, 0.05)`；ground physics material friction 为 1.0。`replicate_physics=False`，`clone_in_fabric=False`，用于支持 articulation/domain randomization。
- Commands：`locomotion_commands` 使用 `ExtendedUniformVelocityCommand`，范围为 `lin_vel_x=(-1.2, 1.2)`、`lin_vel_y=(-0.6, 0.6)`、`ang_vel_z=(-1.0, 1.0)`，`heading_command=False`，`rel_standing_envs=0.3`，小命令 clip 为 x/y/yaw `(0.1, 0.1, 0.2)`。`gait_phase` 是 2.0Hz open-loop gait clock，phase offset 0.5。`base_force_ext` 使用 `VariableForceCommandCfg`，初始 `force_range=(0, 0)`，interval `(6.0, 12.0)s`，duration `(1.0, 3.0)s`，settling `3.0s`，`prob_freed=0.2`，`is_global=True`，`force_z_scale=0.1`；curriculum 在 `8000*24` steps 后切到 `(-65, 65)`。
- Actions：policy-facing leg action 是 `DelayedJointPositionActionCfg`，12 DoF，`scale=0.25`，`use_default_offset=True`，`clip=(-100, 100)`，action delay range 为 `(0, 2)` env steps。`base_force_ext` action 是 `ApplyForceCommandActionCfg`，`dim=0`，从 command manager 读取 force command 并施加到 `base_link`，不是 policy action dimension。
- Observation policy：单帧 46 dim，顺序为 `base_orientation(2)`、`base_ang_vel(3)`、`dof_pos(12)`、`joint_vel(12)`、`last_action(12)`、`gait_clock(2)`、`commands(3)`；`history_length=32` 且 `flatten_history_dim=True`。policy obs corruption enabled；base orientation/angular velocity/dof pos/joint vel 有 episode-fixed obs delay `(0, 2)` env steps 和 Gaussian noise，其中 scales 分别是 base_ang_vel 0.25、dof_pos 1.0、joint_vel 0.05、commands `(2.0, 2.0, 0.25)`。
- Observation privileged/pred：privileged 单帧 docstring 标注 93 dim，`history_length=3`，corruption disabled，包含 base lin vel、base force local、ref dof error、mass/friction/motor strengths、stance/contacts、projected gravity、base ang vel、dof pos/vel、last action、gait clock、scaled commands、base lin vel offset。pred 包含 base lin vel 与 `base_forces_local(use_applied=True)`。
- Delay：policy obs delay 由 `RandomizedDelayModifier` 使用 IsaacLab `DelayBuffer`，reset 时按 env 采样固定 lag；leg action delay 由 `DelayedJointPositionAction` 使用 `DelayBuffer`，reset 时采样 lag 并把 processed/delayed action 清零。
- Force / position control：URDF import 的 joint drive 是 `drive_type="force"`、`target_type="none"`、zero stiffness/damping；实际 leg control 通过 IsaacLab `ImplicitActuatorCfg` 和 delayed joint position target 走 position target + PD，hips/thighs stiffness/damping 为 140/4.5，calfs 为 220/9.0，armature 0.03。外力由 `ApplyForceCommandAction` 每 physics step 写 external force/zero torque，并继承 command term 的 `is_global`。
- Domain randomization / reset：reset 包含 gait state、root pose/velocity、joint offset；reset-mode randomization 包含 rigid body material friction `(0.3, 3.0)`、actuator gains scale `(0.7, 1.3)`；startup-mode randomization 包含 base mass add `(0, 8)` 和 base COM xyz `(-0.1, 0.1)`。
- Rewards：当前 reward surface 是 no-arm locomotion + base-force disturbance，包含 feet contact number、tracking lin vel with force、tracking yaw angular velocity、torque/velocity/acc/action-rate/limit penalties、stand still、ref dof legs、alive、vertical velocity、feet air time/height/drag/contact forces、base height、feet xy/high-height 等。
- Termination：启用 `time_out` 与 `bad_orientation(limit_angle=0.9)`；illegal contact 和 base height termination 当前注释关闭。

## TODO Summary

- 已完成 cfg-side static audit：IsaacLab policy obs corruption/noise/delay、command routing、HistoryWrapper / ObservationManager flatten、leg action delay、MuJoCo `play_policy.py` runtime 对齐风险。
- 剩余 TODO 保留在 `TODO.md`：采集 IsaacLab play/eval 与 MuJoCo `play_policy` 的 paired first-step logs，确认 first divergence；同时重点验证 MuJoCo timestep/control_dt 和 `policy_history_only=True` 的 `curr = policy_vec[:, -46:]` current-slice 语义是否影响 rollout。

## DONE Summary

- 2026-06-03 22:52 HKT：初始化 `manager_based_cfg` memory entry，记录 john 职责、回答 code 引用约束、当前 cfg 关键字段摘要和初始 sim2sim 对齐 TODO。
- 2026-06-03 23:15 HKT：完成 IsaacLab cfg/play/eval 与 MuJoCo `play_policy.py` 的 static audit。结论：train 默认 policy corruption/noise 和 obs/action random delay `(0,2)` 开启；`ReversePushEvalCfg` / export validation 关闭 policy noise 但保留 obs/action delay cfg；`play_policy.py --obs-delay-steps 0 --leg-action-delay-steps 0` 不是随机 delay 训练分布的等价复现，只是 zero-lag eval variant。IsaacLab ObservationManager 实际是 term-major history flatten，MuJoCo runtime 当前也按 term-major 拼接，因此 cfg docstring “按时间维展平”有歧义但不是当前首要 mismatch；更高风险是 MuJoCo timestep/control_dt、current obs slice、fixed zero delay 与 actual exported metadata/cfg drift。
- 2026-06-05 14:43 HKT：完成旧 run `policy.json` 对比：该 export 与 `2026-05-23_14-25-38_A2_base_only_v2/params/env.yaml` / `params/agent.yaml` 对齐，但与 current active cfg 的 hip/thigh damping `4.5` 存在 drift；export wrapper 对 `joint_actuator_gains` 仍依赖 `runtime.default_smart_gait_metadata()`，不是全量 env_cfg-derived metadata。
