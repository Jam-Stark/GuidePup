# MuJoCo No-Arm Aliengo 适配检查与操作计划

## 检查结论总览

| 组件 | 状态 | 说明 |
|---|---|---|
| `scripts/mujoco/runtime.py` | ✅ 已适配 | 默认 metadata 已是 12-joint leg-only / 46 obs / 12 action |
| `scripts/mujoco/export_policy_wrapper.py` | ✅ 已适配 | 依赖 `UniFPAliengoReversePushEvalCfg`，自动跟随 no-arm env cfg |
| `scripts/mujoco/smart_gait.py` | ⚠️ 默认路径需更新 | `--model-path` 默认指向 URDF，无 MuJoCo actuator 会导致 nu==0 退出 |
| `scripts/mujoco/play_policy.py` | ⚠️ 默认路径需更新 | 同上 |
| MuJoCo no-arm MJCF | ❌ 缺失 | 需要从 `aliengoZ1.xml` 派生一个 leg-only `aliengo.xml` |
| `scripts/export_io_descriptors.py` | ✅ 已适配 | 上一轮确认，URDF 正确即导出正确 |

---

## 1. `runtime.py` — 已完全 no-arm ✅

- `_default_joint_names()` (line 25): 仅 12 个腿关节
- `_default_obs_joint_names()` (line 42): 仅 12 个腿关节
- `_default_joint_positions()` (line 59): 仅 12 个默认位置
- `UniFPPolicyMetadata` (line 96): `action_dim=12`, `leg_action_dim=12`, `per_frame_obs_dim=46`
- `build_command_vector()` (line 442): 仅处理 3 维 locomotion 命令
- `build_target_joint_positions()` (line 839): 仅操作 `leg_action_dim` 维度
- `default_smart_gait_metadata()` (line 231) 和 `default_staged_eval_metadata()` (line 205): PD gains 只定义 12 个腿关节
- 无 arm 相关引用

**结论**：无需修改。

---

## 2. `export_policy_wrapper.py` — 已兼容 ✅

- 使用 `UniFPAliengoReversePushEvalCfg` (line 55) 构建 env，该 cfg 继承自已转为 no-arm 的 `UniFPAliengoEnvCfg`
- metadata 导出 (line 168-174) 从实际 policy 输出推导 `action_dim`、`per_frame_obs_dim`、`history_length`
- 基础 metadata 使用 `default_smart_gait_metadata()` 已是 leg-only
- 注意：导出依赖 Isaac sim 环境能正常构建；只要 env cfg + URDF 正确，导出就正确

**结论**：无需修改。

---

## 3. `smart_gait.py` — 默认模型路径需更新 ⚠️

- line 1234: `--model-path` 默认为 `Path("resources/aliengo_description/urdf/aliengo.urdf")`
- `resources/aliengo_description/urdf/aliengo.urdf` 是原始 Unitree aliengo URDF，虽然不含 arm，但 URDF 没有 MuJoCo position actuator 定义
- MuJoCo 加载 URDF 时只创建 joint，不创建 actuator → `model.nu == 0` → line 1352-1354 早退
- 当前实际使用时用户手动传 `--model-path resources/aliengo/mujoco/aliengoZ1.xml`，但该 XML 包含 arm

**需要的改动**：
1. 创建 no-arm MJCF `resources/aliengo/mujoco/aliengo.xml`
2. 更新 `--model-path` 默认值为 `resources/aliengo/mujoco/aliengo.xml`

---

## 4. `play_policy.py` — 默认模型路径需更新 ⚠️

- line 49: 同样默认 `Path("resources/aliengo_description/urdf/aliengo.urdf")`
- 与 smart_gait.py 同理

**需要的改动**：更新 `--model-path` 默认值为 `resources/aliengo/mujoco/aliengo.xml`

---

## 5. 缺失的 MuJoCo MJCF：`aliengo.xml` ❌

### 为什么 URDF 不够

MuJoCo 可以加载 URDF，但结果缺少：
- Position actuator 定义及 PD 增益（MuJoCo actuator section）
- Ground plane 和灯光（worldbody scene）
- Contact class 分层体系（default class）
- 初始姿态 keyframe
- 仿真时间步和积分器设置（option section）
- 调优过的碰撞参数（solref/solimp/friction）

### 从 `aliengoZ1.xml` 派生 `aliengo.xml` 的具体操作

现有 `resources/aliengo/mujoco/aliengoZ1.xml` 是一个功能完整的 MJCF，包含 arm + leg。需要删除的内容清单：

#### asset 部分 — 删除 arm mesh 资源
- 删除 `z1_GripperStator`、`z1_GripperMover`（collision mesh）
- 删除 `z1_Link00_visual_mesh` 到 `z1_Link06_visual_mesh`
- 删除 `z1_GripperStator_visual_mesh`、`z1_GripperMover_visual_mesh`

#### default 部分 — 删除 arm contact class
- 删除 `<default class="arm_contact">` 块

#### worldbody/trunk 部分 — 删除 arm 链
- 删除 `arm_mount_geom`（line 89）
- 删除 `arm_link00_visual_geom`（line 90）
- 删除整个 `<body name="arm_link01">` ... `</body>` 子树（line 92-140）
- 更新 trunk inertial：需要用纯 aliengo 底盘的惯性参数（来自 `resources/aliengo/urdf/aliengo.urdf` 中 trunk link 的 inertial）

#### contact 部分 — 删除 arm 排除
- 删除所有 arm body 的 `<exclude>` 元素（line 225-231）
- 整个 `<contact>` section 可能变为空，可以整段删除

#### actuator 部分 — 删除 arm actuator
- 删除 `arm_joint1_pos` 到 `arm_joint6_hold`（line 250-255）
- 保留 12 个腿 actuator 不变

#### keyframe 部分 — 更新 qpos 和 ctrl
- 原始 qpos 为 25 维：`[x y z qw qx qy qz arm1..arm6 FR_hip FR_thigh FR_calf FL_hip ... RL_calf]`
- no-arm qpos 为 19 维：`[x y z qw qx qy qz FR_hip FR_thigh FR_calf FL_hip ... RL_calf]`
- 具体值：`qpos="0 0 0.5 1 0 0 0 -0.1 0.5 -1.0 0.1 0.5 -1.0 -0.1 0.5 -1.0 0.1 0.5 -1.0"`
- ctrl 从 18 维变为 12 维：`ctrl="0.1 -0.1 0.1 -0.1 0.5 0.5 0.5 0.5 -1.0 -1.0 -1.0 -1.0"`
- 注意 joint 顺序：MuJoCo MJCF 中腿关节的物理定义顺序是 FR→FL→RR→RL，但 actuator 顺序是 FL/FR/RL/RR-hip → FL/FR/RL/RR-thigh → FL/FR/RL/RR-calf。keyframe ctrl 要与 actuator 顺序对齐

#### model name — 更新
- `<mujoco model="aliengoZ1">` → `<mujoco model="aliengo">`

### visual_meshes 目录
- `resources/aliengo/mujoco/visual_meshes/` 已包含所有腿部 obj 文件（trunk.obj, hip.obj, thigh.obj, thigh_mirror.obj, calf.obj）
- arm 相关 obj 可以保留（不会被引用）或清理

---

## 操作计划（实施步骤）

1. **创建 `resources/aliengo/mujoco/aliengo.xml`**
   - 从 `aliengoZ1.xml` 复制并删除全部 arm 内容
   - 更新 trunk inertial 为纯底盘值
   - 更新 keyframe 为 19 维 qpos + 12 维 ctrl
   - 更新 model name

2. **更新 `scripts/mujoco/smart_gait.py` 默认 model-path**
   - line 1234: `default=Path("resources/aliengo/mujoco/aliengo.xml")`

3. **更新 `scripts/mujoco/play_policy.py` 默认 model-path**
   - line 49: `default=Path("resources/aliengo/mujoco/aliengo.xml")`

4. **可选：更新 `scripts/mujoco/staged_direction_force_eval.py` 默认 model-path**
   - 该脚本按 plan 已排除在 no-arm 维护范围外，但如果仍在使用，也需要更新

5. **smoke test 验证**
   - `python scripts/mujoco/play_policy.py --model-path resources/aliengo/mujoco/aliengo.xml --headless --duration-s 2`
   - 确认 `model.nu == 12`，`model.nq == 19`（7 freejoint + 12 joints），无报错
   - `python scripts/mujoco/smart_gait.py --model-path resources/aliengo/mujoco/aliengo.xml --policy-path <path> --headless --duration-s 2`

---

## 风险和注意事项

- **trunk inertial**：aliengoZ1.xml 的 trunk inertial (line 82-83) 包含了 arm mount 质量。需要用 `resources/aliengo/urdf/aliengo.urdf` 中 trunk 的纯底盘惯性数据替换：`mass="11.644"`, `origin xyz="0.008811 0.003839 0.000273"`, inertia 张量来自该 URDF
- **keyframe qpos 顺序**：MuJoCo 按 body 定义顺序分配 joint index，aliengoZ1.xml body 顺序是 trunk→arm→FR→FL→RR→RL，去掉 arm 后变为 trunk→FR→FL→RR→RL，qpos 顺序随之改变
- **MuJoCo 与 Isaac PD 增益差异**：已有的 actuator kp/kv 值（如 hip kp=80, calf kp=160）是为 aliengoZ1 调参的初始值，`default_smart_gait_metadata()` 在 runtime.py 中会用 `apply_joint_actuator_gains()` 覆盖
