# 步态控制机制设计文档

## 概述

本文档详细说明了基于 Isaac Lab Manager-Based 环境的四足机器人步态控制机制实现，该机制通过**显式步态时钟 (Explicit Gait Clock)** 和**步态奖励函数 (Gait Reward Functions)** 引导机器人学习稳定的对角小跑步态 (Diagonal Trotting Gait)。

设计灵感来源于 VBC (Versatile Behavior Control) 论文，实现了自适应的站立/行走状态切换和精确的步态约束。

---

## 1. 步态参数设计

### 1.1 基本步态参数

| 参数 | 值 | 说明 |
|------|-----|------|
| **步态类型** | Trotting (对角小跑) | 对角足同相运动 |
| **步态频率** | 2.0 Hz | 每秒2个完整步态周期 |
| **步态周期** | 0.5 秒 | 单个步态周期持续时间 |
| **支撑相占比** | 50% (0.25秒) | 足部接触地面的时间 |
| **摆动相占比** | 50% (0.25秒) | 足部离地摆动的时间 |
| **相位偏移** | 0.5 (180°) | 两组对角足之间的相位差 |

### 1.2 足部相位分配

采用对角小跑步态，四足分为两组：

```
组1（相位 + 0.5）：
  - FL (Front Left, 左前足)
  - RR (Rear Right, 右后足)

组2（相位 + 0.0）：
  - FR (Front Right, 右前足)
  - RL (Rear Left, 左后足)
```

**运动规律**：
- 当 FL 和 RR 处于支撑相时，FR 和 RL 处于摆动相
- 当 FL 和 RR 处于摆动相时，FR 和 RL 处于支撑相
- 两组对角足交替支撑，形成稳定的对角小跑步态

---

## 2. 步态时钟观测 (Gait Clock Observation)

### 2.1 功能说明

**函数名称**: `gait_clock_signal()`  
**文件路径**: `source/acc/acc/tasks/manager_based/vbc/observation_compute.py`  
**观测维度**: 4 维（每只脚一个信号）

### 2.2 工作原理

#### 2.2.1 行走状态 (Walking)

```python
# 主时钟相位计算
gait_phase = (current_time * gait_frequency) % 1.0  # 0.0 到 1.0 循环

# 足部相位分配
foot_phases = [
    (gait_phase + 0.5) % 1.0,  # FL (左前)
    gait_phase,                 # FR (右前)
    gait_phase,                 # RL (左后)
    (gait_phase + 0.5) % 1.0,  # RR (右后)
]

# 转换为正弦信号
clock_signals = sin(2π × foot_phases)  # 范围: [-1, 1]
```

**信号含义**：
- `clock_signal ≈ +1.0`: 摆动相最高点（足部抬起最高）
- `clock_signal ≈ 0.0`: 相位转换点（支撑相→摆动相 或 摆动相→支撑相）
- `clock_signal ≈ -1.0`: 支撑相中期（足部稳定接触地面）

#### 2.2.2 站立状态 (Standing)

```python
# 检测站立状态
is_standing = locomotion_command_term.is_standing_env

# 站立时冻结时钟信号
standing_signal = torch.full((num_envs, 4), -1.0)  # 所有足部固定为支撑相

# 根据状态选择信号
final_signals = torch.where(
    is_standing.unsqueeze(1),
    standing_signal,    # 站立 → 所有信号 = -1.0
    clock_signals       # 行走 → 动态正弦波
)
```

**设计意图**：
- 站立时，所有足部时钟固定为 `-1.0`，表示"所有足部应保持接触地面"
- 行走时，时钟动态变化，指导策略网络何时抬腿、何时落地

### 2.3 配置示例

```python
# vbc_env_cfg.py - ObservationsCfg.PolicyCfg
gait_clock = ObsTerm(
    func=mdp.gait_clock_signal,
    params={
        "command_name": "locomotion_commands",  # 用于判断站立/行走状态
        "gait_frequency": 2.0,                  # 2 Hz 步态频率
        "phase_offset": 0.5,                    # 180° 相位偏移
    }
)
```

---

## 3. 步态奖励函数 (Gait Reward Functions)

### 3.1 设计原则

#### 3.1.1 符号约定

- **函数返回值**: 正值表示违反程度（惩罚程度）
- **配置权重**: 负值（如 `-2.0`）
- **最终奖励**: `reward = weight × function_return`
  - 违反越严重 → 函数返回值越大 → 最终奖励越负（惩罚越重）

#### 3.1.2 自适应站立/行走

由于步态时钟已经处理了站立/行走状态区分：
- 站立时：`desired_contact` 全为高值 → 自然不惩罚接触力
- 行走时：`desired_contact` 周期变化 → 正常惩罚步态违规

**因此奖励函数无需额外的 walking mask，代码更简洁！**

---

### 3.2 奖励函数 1: 接触力跟踪 (Contact Force Tracking)

**函数名称**: `reward_tracking_contacts_shaped_force()`  
**文件路径**: `source/acc/acc/tasks/manager_based/vbc/mdp/rewards.py`

#### 3.2.1 目标

- **摆动相**: 足部应该离地（接触力 ≈ 0）
- **支撑相**: 足部应该接触地面（接触力 > 0）

#### 3.2.2 核心机制

使用 **冯·米塞斯分布 (von Mises Distribution)** 计算期望接触概率：

```python
# 冯·米塞斯分布
von_mises = exp(κ × (cos(2π × phase) - 1))
desired_contact = von_mises / (von_mises + ε)

# κ (kappa) = 0.07: 控制分布尖锐度
#   - κ 越大 → 转换越尖锐（支撑相/摆动相边界更明确）
#   - κ 越小 → 转换越平滑（允许更长的过渡期）
```

**概率分布特性**：
- `phase ≈ 0.0` 或 `1.0` (支撑相中期): `desired_contact ≈ 1.0`
- `phase ≈ 0.5` (摆动相中期): `desired_contact ≈ 0.0`
- 过渡期: 平滑的概率梯度

#### 3.2.3 惩罚计算

```python
# 对每只脚计算惩罚
penalty = 0
for each foot:
    # 摆动相时有接触力的惩罚
    penalty += (1 - desired_contact) × (1 - exp(-force² / σ_force))
    
    # 解读:
    # - (1 - desired_contact): 摆动相权重（摆动相时接近1，支撑相时接近0）
    # - (1 - exp(-force² / σ)): 力的大小惩罚（力越大，惩罚越接近1）
    
return penalty / 4.0  # 平均四只脚的惩罚
```

#### 3.2.4 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gait_force_sigma` | 0.5 | 力的标准差，控制惩罚敏感度 |
| `kappa_gait_probs` | 0.07 | 冯·米塞斯分布尖锐度参数 |

#### 3.2.5 配置示例

```python
# vbc_env_cfg.py - RewardsCfg
tracking_contacts_force = RewTerm(
    func=mdp.reward_tracking_contacts_shaped_force,
    weight=-2.0,  # 负权重：惩罚违规
    params={
        "asset_cfg": SceneEntityCfg("robot"),
        "gait_force_sigma": 0.5,
        "kappa_gait_probs": 0.07,
    }
)
```

---

### 3.3 奖励函数 2: 接触速度跟踪 (Contact Velocity Tracking)

**函数名称**: `reward_tracking_contacts_shaped_vel()`  
**文件路径**: `source/acc/acc/tasks/manager_based/vbc/mdp/rewards.py`

#### 3.3.1 目标

- **支撑相**: 足部应该静止（速度 ≈ 0，避免打滑）
- **摆动相**: 允许足部运动（不惩罚速度）

#### 3.3.2 惩罚计算

```python
# 对每只脚计算惩罚
penalty = 0
for each foot:
    # 支撑相时有速度的惩罚（打滑）
    penalty += desired_contact × (1 - exp(-velocity² / σ_vel))
    
    # 解读:
    # - desired_contact: 支撑相权重（支撑相时接近1，摆动相时接近0）
    # - (1 - exp(-velocity² / σ)): 速度大小惩罚（速度越大，惩罚越接近1）
    
return penalty / 4.0  # 平均四只脚的惩罚
```

#### 3.3.3 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gait_vel_sigma` | 0.5 | 速度的标准差，控制惩罚敏感度 |
| `kappa_gait_probs` | 0.07 | 冯·米塞斯分布尖锐度参数 |

#### 3.3.4 配置示例

```python
# vbc_env_cfg.py - RewardsCfg
tracking_contacts_vel = RewTerm(
    func=mdp.reward_tracking_contacts_shaped_vel,
    weight=-2.0,  # 负权重：惩罚打滑
    params={
        "asset_cfg": SceneEntityCfg("robot"),
        "gait_vel_sigma": 0.5,
        "kappa_gait_probs": 0.07,
    }
)
```

---

### 3.4 奖励函数 3: 摆动足高度 (Swing Foot Height)

**函数名称**: `reward_feet_height()`  
**文件路径**: `source/acc/acc/tasks/manager_based/vbc/mdp/rewards.py`

#### 3.4.1 目标

- **摆动相**: 足部应抬高到目标高度（避免拖地）
- **支撑相**: 不检查高度
- **站立时**: 不检查高度（通过 walking mask 控制）

#### 3.4.2 设计差异

**为什么不用冯·米塞斯分布？**

与前两个奖励函数不同，`feet_height` 使用**简单阈值判断**：

```python
# 简单判断摆动相（接触力 < 1N）
in_swing = (foot_forces < 1.0)

# 理由：
# - feet_height 只关心"摆动相中期"的足部高度
# - 不关心"过渡期"（刚离地/即将落地时高度可以低）
# - 简单阈值足够，无需平滑的概率分布
```

而前两个函数需要冯·米塞斯分布是因为：
- 接触力/速度在过渡期需要平滑变化
- 需要精确控制接触/离地的时机
- 过渡期的容忍度很重要

#### 3.4.3 惩罚计算

```python
# 只在摆动相且低于目标高度时惩罚
height_error = torch.where(
    in_swing,
    torch.clamp(target_height - foot_heights, min=0.0),  # 只惩罚低于目标
    torch.zeros_like(foot_heights)
)

penalty = torch.sum(height_error, dim=1)

# 只在行走状态下应用（站立时返回0）
return penalty * walking_mask.float()
```

#### 3.4.4 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_height` | 0.3 米 | 摆动相目标足部高度 |
| `command_name` | "locomotion_commands" | 用于获取 walking mask |

#### 3.4.5 配置示例

```python
# vbc_env_cfg.py - RewardsCfg
feet_height = RewTerm(
    func=mdp.reward_feet_height,
    weight=-1.0,  # 负权重：惩罚足部抬得不够高
    params={
        "asset_cfg": SceneEntityCfg("robot"),
        "command_name": "locomotion_commands",  # 仅在行走时生效
        "target_height": 0.3,  # 目标高度 0.3m
    }
)
```

---

## 4. 完整奖励权重配置

### 4.1 推荐权重

基于 VBC 论文和实验调优：

```python
# RewardsCfg 中的步态相关奖励
class RewardsCfg:
    # 1. 接触力跟踪（重要性: 高）
    tracking_contacts_force = RewTerm(
        func=mdp.reward_tracking_contacts_shaped_force,
        weight=-2.0,  # 强惩罚摆动相接触
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "gait_force_sigma": 0.5,
            "kappa_gait_probs": 0.07,
        }
    )
    
    # 2. 接触速度跟踪（重要性: 高）
    tracking_contacts_vel = RewTerm(
        func=mdp.reward_tracking_contacts_shaped_vel,
        weight=-2.0,  # 强惩罚支撑相打滑
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "gait_vel_sigma": 0.5,
            "kappa_gait_probs": 0.07,
        }
    )
    
    # 3. 摆动足高度（重要性: 中）
    feet_height = RewTerm(
        func=mdp.reward_feet_height,
        weight=-1.0,  # 中等惩罚低足部高度
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "target_height": 0.3,  # 30cm 足部高度
        }
    )
```

### 4.2 权重调优指南

| 问题 | 可能原因 | 调整建议 |
|------|---------|---------|
| 摆动相仍有接触 | 惩罚不足 | 增大 `tracking_contacts_force.weight`（如 -3.0） |
| 支撑相打滑严重 | 惩罚不足 | 增大 `tracking_contacts_vel.weight`（如 -3.0） |
| 足部抬得不够高 | 惩罚不足或目标太高 | 增大 `feet_height.weight` 或降低 `target_height` |
| 步态转换不够平滑 | kappa 太大 | 减小 `kappa_gait_probs`（如 0.05） |
| 步态约束过于宽松 | kappa 太小 | 增大 `kappa_gait_probs`（如 0.1） |
| 力/速度惩罚过于敏感 | sigma 太小 | 增大 `gait_force_sigma` 或 `gait_vel_sigma` |

---

## 5. 观测空间和奖励空间总结

### 5.1 步态相关观测（4 维）

```python
# 添加到 ObservationsCfg.PolicyCfg
gait_clock: 4 维  # [FL_clock, FR_clock, RL_clock, RR_clock]
    - 范围: [-1, 1]
    - 行走时: 动态正弦波
    - 站立时: 固定为 -1.0
```

### 5.2 步态相关奖励（3 项）

```python
# 添加到 RewardsCfg
tracking_contacts_force: weight = -2.0  # 惩罚摆动相接触
tracking_contacts_vel:   weight = -2.0  # 惩罚支撑相打滑
feet_height:             weight = -1.0  # 惩罚低足部高度（仅行走时）
```

---

## 6. 训练监控指标

### 6.1 TensorBoard 监控

在训练过程中，监控以下奖励曲线：

```bash
# 步态奖励
rewards/tracking_contacts_force  # 应该趋向于 0（接近0说明步态正确）
rewards/tracking_contacts_vel    # 应该趋向于 0
rewards/feet_height              # 应该趋向于 0

# 总奖励
rewards/total                    # 应该持续上升

# 其他验证
episode_length                   # 站立/行走都能维持长 episode
```

### 6.2 步态质量验证

**理想的训练结果**：

1. **行走状态**：
   - 对角足清晰的交替运动（FL+RR vs FR+RL）
   - 摆动相时足部明显离地（高度 > 0.3m）
   - 支撑相时足部静止（速度 ≈ 0）
   - 步态频率稳定在 2 Hz

2. **站立状态**：
   - 所有四足保持接触地面
   - 足部速度接近 0
   - 身体稳定不晃动

3. **状态切换**：
   - 从站立切换到行走：平滑启动步态
   - 从行走切换到站立：快速停止并稳定

---

## 7. 实现文件清单

### 7.1 核心文件

```
source/acc/acc/tasks/manager_based/vbc/
├── observation_compute.py          # 步态时钟观测函数
│   └── gait_clock_signal()        # 4维步态时钟信号
│
├── mdp/
│   ├── __init__.py                # 导出 gait_clock_signal
│   └── rewards.py                 # 步态奖励函数
│       ├── reward_tracking_contacts_shaped_force()
│       ├── reward_tracking_contacts_shaped_vel()
│       └── reward_feet_height()
│
└── vbc_env_cfg.py                 # 环境配置
    ├── ObservationsCfg.PolicyCfg  # 添加 gait_clock
    └── RewardsCfg                 # 添加 3 个步态奖励
```

### 7.2 配置修改点

1. **观测配置** (`vbc_env_cfg.py` - ObservationsCfg.PolicyCfg):
   - 添加 `gait_clock` 观测项（4 维）

2. **奖励配置** (`vbc_env_cfg.py` - RewardsCfg):
   - 添加 `tracking_contacts_force` 奖励项
   - 添加 `tracking_contacts_vel` 奖励项
   - 添加 `feet_height` 奖励项

3. **导出配置** (`mdp/__init__.py`):
   - 导出 `gait_clock_signal` 函数

---

## 8. 设计优势

### 8.1 显式步态引导

- ✅ 策略网络直接接收步态时钟信号
- ✅ 无需从零学习步态相位
- ✅ 加速训练收敛

### 8.2 状态自适应

- ✅ 自动区分站立/行走状态
- ✅ 站立时冻结时钟（所有足支撑相）
- ✅ 行走时动态步态（对角小跑）

### 8.3 奖励函数简洁性

- ✅ 无需在奖励函数中重复添加 walking mask
- ✅ 步态时钟统一处理状态逻辑
- ✅ 代码更易维护

### 8.4 可调参数丰富

- ✅ 步态频率可调（`gait_frequency`）
- ✅ 相位偏移可调（`phase_offset`）
- ✅ 分布尖锐度可调（`kappa_gait_probs`）
- ✅ 惩罚敏感度可调（`sigma` 参数）

---

## 9. 常见问题 (FAQ)

### Q1: 为什么选择 2 Hz 步态频率？

**A**: 基于 VBC 论文实验：
- 2 Hz 对四足机器人是自然频率
- 支撑相/摆动相各 0.25 秒，足够完成动作
- 频率过高（>3 Hz）：执行器难以跟随
- 频率过低（<1 Hz）：运动效率低

### Q2: 为什么 kappa = 0.07 这么小？

**A**: 
- κ 控制冯·米塞斯分布的尖锐度
- κ = 0.07 提供**平滑的过渡期**
- 过大的 κ → 过于尖锐的转换 → 策略难以学习
- 过小的 κ → 过于宽松的约束 → 步态不清晰

### Q3: 为什么 feet_height 只在行走时生效？

**A**: 
- 站立时不需要抬脚，检查高度没意义
- 避免站立时的无效惩罚
- 保持奖励函数的物理意义清晰

### Q4: 为什么 feet_height 不用冯·米塞斯分布？

**A**: 
- `feet_height` 只关心摆动相中期的足部高度
- 过渡期（刚离地/即将落地）允许高度较低
- 简单阈值判断（force < 1N）足够
- 而 force/velocity 需要精确控制过渡期

### Q5: 如何验证步态是否正确？

**A**: 可视化验证：
1. 录制机器人行走视频
2. 观察 FL+RR 是否同时抬起/落地
3. 观察 FR+RL 是否同时抬起/落地
4. 检查两组是否交替运动（180° 相位差）

数据验证：
1. TensorBoard 查看 `tracking_contacts_force` 趋向 0
2. 打印 `gait_clock_signal` 输出，验证相位关系
3. 记录足部接触状态，验证对角小跑模式

---

## 10. 未来改进方向

### 10.1 多步态支持

- [ ] 实现 Walk、Trot、Pace、Bound 等多种步态
- [ ] 通过命令切换步态类型
- [ ] 自适应步态频率（根据速度命令调整）

### 10.2 地形自适应

- [ ] 崎岖地形下的步态调整
- [ ] 斜坡/楼梯的专门步态
- [ ] 基于高度图的步态规划

### 10.3 课程学习

- [ ] 从简单步态（原地踏步）到复杂步态（快速奔跑）
- [ ] 逐步增加步态约束强度
- [ ] 动态调整奖励权重

---

## 11. 参考资料

### 11.1 论文

- **VBC (Versatile Behavior Control)**: 显式步态时钟的主要灵感来源
- **ANYmal 系列论文**: 四足机器人步态控制的经典实现

### 11.2 代码参考

- **Isaac Lab 官方文档**: Manager-Based RL 环境设计
- **Legged Gym**: 四足机器人训练环境的先驱实现

### 11.3 数学工具

- **冯·米塞斯分布 (von Mises Distribution)**: 圆周数据的概率分布
- **指数奖励函数**: `exp(-x²/σ)` 的平滑惩罚特性

---

## 12. 版本历史

| 版本 | 日期 | 变更说明 |
|------|------|---------|
| 1.0 | 2025-01-11 | 初始版本：完整步态控制机制实现 |
|     |            | - 添加 gait_clock_signal 观测 |
|     |            | - 添加 3 个步态奖励函数 |
|     |            | - 实现站立/行走状态自适应 |

---

## 13. 联系与贡献

如有问题或改进建议，欢迎提交 Issue 或 Pull Request。

**文档作者**: Copilot (GitHub)  
**创建日期**: 2025-01-11  
**最后更新**: 2025-01-11
