# 动态机械臂操作命令生成器

## 概述

`DynamicManipulationCommand` 是一个自定义的 Isaac Lab 命令生成器，采用**球坐标系采样 + 轨迹插值**方法生成平滑的机械臂操作轨迹。它通过在球坐标空间中进行线性插值(LERP)和四元数球面插值(SLERP)，生成从起点到目标的连续轨迹。

## 核心思想

### 方法论
系统采用基于轨迹插值的方式生成操作命令，而非简单的目标跟踪。每个轨迹周期包含：
- **起点和终点**（球坐标系）
- **目标姿态**（四元数表示）
- **轨迹时长**（运动时间 + 保持时间）
- **实时插值**（每步更新命令）

### 具体实现

#### 1. 采样触发机制

**初始化采样**
- 环境重置时触发第一次采样
- 使用固定起始位置: 球坐标 `[0.5, π/8, 0.0]` (半径, 俯仰角, 方位角)
- 采样第一个目标位置

**周期性重采样**
- 触发条件: 当前步数 ≥ 总时间步数 (运动步数 + 保持步数)
- 重采样计时器在每个控制步更新
- 当前目标自动成为下一轨迹的起点

#### 2. 球坐标系采样范围

**位置参数 (球坐标)**
- **径向距离** l: `[0.4, 0.95]` 米
- **俯仰角** p: `[-π/2.5, π/3]` 弧度 (约 -72° 到 60°)
- **方位角** y: `[-1.2, 1.2]` 弧度 (约 -69° 到 69°)

**姿态参数 (欧拉角增量)**
- **基础姿态**: `(π/2, 0, 0)` - 末端执行器朝下
- **Roll 增量**: `[-0.5, 0.5]` 弧度 (约 ±29°)
- **Pitch 增量**: `[-0.5, 0.5]` 弧度 (约 ±29°)
- **Yaw 增量**: `[-0.5, 0.5]` 弧度 (约 ±29°)
- **最终姿态** = 基础姿态 + 随机增量

#### 3. 球心坐标系统

**固定参数**
- **前向偏移**: `0.3` 米 (机器人本体X轴)
- **侧向偏移**: `0.0` 米 (机器人本体Y轴)
- **绝对高度**: `0.7` 米 (世界坐标系Z轴，高度不随机器人俯仰/翻滚变化)

**坐标转换链**
1. 提取机器人基座的**仅偏航角**旋转四元数
2. 将球坐标转为笛卡尔坐标 (相对球心)
3. 应用偏航旋转到世界坐标系
4. 加上球心世界坐标得到最终目标位置

#### 4. 时间参数

**轨迹时间**
- **运动时间**: `[1.0, 3.0]` 秒，均匀采样
- **保持时间**: `[0.5, 2.0]` 秒，均匀采样
- **控制周期**: `0.02` 秒 (仿真步长 `0.005` × 降频因子 `4`)
- **总时间步数** = (运动时间 + 保持时间) / 控制周期

#### 5. 碰撞避免策略

**检测范围 (世界坐标系)**
- **X轴**: `[-0.8, 0.1]` 米
- **Y轴**: `[-0.2, 0.2]` 米
- **Z轴**: `[-0.7, -0.05]` 米 (地面以下限制: -0.7米)

**采样策略**
- 最大尝试次数: `10` 次
- 对每个环境独立采样直到找到无碰撞目标
- 超过最大次数后使用最后一次采样结果 (带警告)

#### 6. 轨迹插值方法

**位置插值**
- **方法**: 线性插值 (LERP)
- **坐标系**: 球坐标系 (先插值球坐标，再转笛卡尔)
- **插值参数**: `t = clamp(当前步数 / 运动步数, 0, 1)`
- **公式**: `current_sphere = start_sphere * (1-t) + target_sphere * t`

**姿态插值**
- **方法**: 球面线性插值 (SLERP)
- **坐标系**: 四元数
- **确保**: 平滑的姿态过渡，避免万向锁
- **短路径**: 自动选择最短旋转路径

**阶段划分**
- **运动阶段** (0 ≤ t ≤ 1): 从起点插值到目标
- **保持阶段** (t = 1): 固定在目标位置和姿态

#### 7. 初始化特殊处理

**固定起始位置 (球坐标)**
- **起点**: `[0.5, π/8, 0.0]` - (半径0.5m, 俯仰π/8, 方位0)
- **首次采样**: 随机目标位置
- **后续轨迹**: 上一轨迹的终点成为新起点

## 使用方法

### 在环境配置中使用

```python
from acc.tasks.manager_based.acc.commands import DynamicManipulationCommandCfg
import math

@configclass
class CommandsCfg:
    """命令配置"""
    
    @configclass
    class TargetPositionCommandCfg(DynamicManipulationCommandCfg):
        """动态机械臂操作命令配置 - 球坐标采样 + 轨迹插值"""
        
        # 资产配置
        asset_name: str = "robot"
        body_name: str = "gripperMover"
        
        # 球心配置
        sphere_center: Tuple[float, float] = (0.3, 0.0)  # 机身坐标系XY
        sphere_center_z: float = 0.7  # 世界坐标系固定Z高度
        
        # 时间配置
        move_time_range: Tuple[float, float] = (1.0, 3.0)  # 运动时长
        hold_time_range: Tuple[float, float] = (0.5, 2.0)  # 保持时长
        
        max_collision_attempts: int = 10
        
        # 球坐标采样范围
        @configclass
        class Ranges:
            radius: Tuple[float, float] = (0.4, 0.95)  # 径向距离 l
            pitch_angle: Tuple[float, float] = (-math.pi/2.5, math.pi/3)  # 俯仰角 p
            yaw_angle: Tuple[float, float] = (-1.2, 1.2)  # 方位角 y
        
        ranges: Ranges = Ranges()
        
        # 碰撞检测边界框 (世界坐标系)
        @configclass
        class CollisionBox:
            x_min: float = -0.8
            x_max: float = 0.1
            y_min: float = -0.2
            y_max: float = 0.2
            z_min: float = -0.7
            z_max: float = -0.05
        
        collision_box: CollisionBox = CollisionBox()
        debug_vis: bool = False
    
    target_position_commands: TargetPositionCommandCfg = TargetPositionCommandCfg()
```

### 命令格式

命令张量维度: **7维**
```
[pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
```

- `pos_x, pos_y, pos_z`: 当前插值位置（世界坐标系）
- `quat_w, quat_x, quat_y, quat_z`: 当前插值姿态（四元数）

**注意**: 这是经过插值后的**当前命令**，而非最终目标。命令每个控制步都会更新。

### 访问命令数据

在观测函数或奖励函数中访问命令：

```python
def my_observation_fn(env, command_name: str):
    # 通过 command manager 访问命令
    command_term = env.command_manager._terms[command_name]
    
    # 提取当前目标位置 (前3维)
    current_target_pos = command_term.command[:, :3]
    
    # 提取当前目标姿态 (后4维，四元数)
    current_target_quat = command_term.command[:, 3:7]  # [w, x, y, z]
    
    return current_target_pos
```

## 轨迹执行流程

```
1. 环境重置
   └─> 设置固定起点球坐标 [0.5, π/8, 0.0]
   └─> 采样第一个目标 (球坐标 + 姿态 + 时间)
   └─> 初始化计数器: current_step = 0

2. 每个控制步 (0.02秒)
   ├─> current_step += 1
   ├─> 计算插值参数: t = current_step / move_steps
   ├─> 球坐标LERP: current_sphere = start * (1-t) + target * t
   ├─> 转换为笛卡尔坐标并变换到世界坐标系
   ├─> 四元数SLERP: current_quat = slerp(start_quat, target_quat, t)
   └─> 更新命令: [pos(3), quat(4)]

3. 运动阶段完成 (current_step >= move_steps)
   └─> 进入保持阶段 (t = 1，固定在目标)
   └─> 继续计数直到 total_steps

4. 轨迹完成 (current_step >= total_steps)
   ├─> 当前目标 → 新起点
   ├─> 采样新目标
   ├─> 重置计数器: current_step = 0
   └─> 返回步骤2

5. 循环执行 2-4
```

## 核心优势

1. **平滑轨迹**: 通过插值生成连续平滑的运动轨迹，避免突变
2. **球坐标采样**: 在球坐标系采样确保均匀的空间覆盖
3. **物理可达性**: 通过碰撞检测和工作空间约束确保目标可达
4. **姿态稳定**: SLERP保证姿态平滑过渡，避免万向锁
5. **自适应跟随**: 球心跟随机器人位置和偏航角，适应移动基座
6. **固定世界高度**: 球心Z坐标固定，不受机器人俯仰/翻滚影响
7. **连续任务流**: 自动衔接的轨迹序列，无学习间隙

## 调试与可视化

启用调试可视化：
```python
debug_vis: bool = True
```

这将在仿真环境中显示目标位姿的可视化标记（坐标系），帮助理解任务目标。

## 技术细节

### 球坐标转换

球坐标 `(l, p, y)` 到笛卡尔坐标的转换：
```python
# 球坐标 (l, p, y) → 笛卡尔坐标 (x, y, z)
x = l * cos(p) * cos(y)
y = l * cos(p) * sin(y)  
z = l * sin(p)

# 相对于球心 (center_x, center_y) 在机器人坐标系
body_x = center_x + x
body_y = center_y + y
body_z = z

# 变换到世界坐标系（仅应用偏航角）
world_x = robot_x + body_x * cos(robot_yaw) - body_y * sin(robot_yaw)
world_y = robot_y + body_x * sin(robot_yaw) + body_y * cos(robot_yaw)
world_z = sphere_center_z + body_z  # 固定世界高度0.7m
```

其中:
- `l`: 半径 ∈ [0.4, 0.95]m
- `p`: 俯仰角 ∈ [-π/2.5, π/3]rad（约-72°到60°）
- `y`: 偏航角 ∈ [-1.2, 1.2]rad（约±69°）

### SLERP 四元数插值

球面线性插值确保姿态平滑过渡：
```python
# 球面线性插值
dot = q1·q2
if dot < 0:  # 确保最短路径
    q2 = -q2
    dot = -dot

if dot > 0.9995:  # 接近平行，使用线性插值
    return normalize((1-t)*q1 + t*q2)
else:  # 标准SLERP
    theta = arccos(dot)
    return (sin((1-t)*theta)*q1 + sin(t*theta)*q2) / sin(theta)
```

### LERP 位置插值

在球坐标空间进行线性插值：
```python
# 球坐标线性插值
current_l = start_l * (1 - t) + target_l * t
current_p = start_p * (1 - t) + target_p * t
current_y = start_y * (1 - t) + target_y * t

# 然后转换为笛卡尔坐标和世界坐标
```

### 碰撞检测

目标位置需**不在**碰撞框内才有效：
```python
# 检查是否在碰撞区域（世界坐标系）
in_collision = (
    (collision_box.x_min <= world_x <= collision_box.x_max) &
    (collision_box.y_min <= world_y <= collision_box.y_max) &
    (collision_box.z_min <= world_z <= collision_box.z_max)
)

# 如果在碰撞区域，重新采样（最多10次）
```

默认碰撞框（桌面区域）：
- X: [-0.8, 0.1]m
- Y: [-0.2, 0.2]m  
- Z: [-0.7, -0.05]m

## 性能考虑

### 计算开销
- **每步更新**: O(batch_size) 的插值计算（LERP + SLERP）
  - 球坐标LERP: 简单线性组合，极低开销
  - 四元数SLERP: 三角函数计算，中等开销
  - 坐标转换: 矩阵乘法，低开销
- **重采样频率**: 平均每 `(move_time + hold_time) / 0.02` 步触发一次
  - 典型值: (1.5 + 1.0) / 0.02 = 125步一次
- **碰撞检测**: 仅在重采样时执行
  - 平均尝试次数: 1-3次
  - 最大尝试次数: 10次

### GPU 并行化
- 所有张量运算使用 PyTorch GPU 后端
- 批量处理多个环境的命令（支持数千并行环境）
- 避免 CPU-GPU 数据传输，所有计算在GPU完成
- 向量化碰撞检测，无循环

### 数值稳定性
- SLERP 使用 `dot > 0.9995` 阈值避免除零
- 四元数归一化确保单位长度
- 球坐标范围限制避免奇异点（极点附近）
- 三角函数使用 PyTorch 高精度实现

## 未来扩展方向

1. **动态障碍物**: 支持运行时更新的碰撞检测区域
2. **逆运动学验证**: 在采样时检查IK可解性，确保关节限制内可达
3. **自适应难度**: 根据成功率动态调整球体半径和时间范围
4. **多模态采样**: 支持不同采样分布（均匀、高斯、基于先验等）
5. **预定义轨迹**: 支持脚本化任务序列与随机采样混合
6. **速度感知**: 考虑末端执行器速度限制，自适应调整插值时间
7. **姿态约束**: 支持保持特定轴对齐等姿态约束
