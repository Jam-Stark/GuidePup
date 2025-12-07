# VBC环境域随机化事件实现说明

本文档说明了在VBC环境中实现的7个域随机化事件，这些事件基于Isaac Lab官方EventManager API。

## 事件列表

### 1. 物理材质随机化 (`randomize_physics_material`)
- **功能**: 在每次环境重置时，随机化机器人所有部件的物理材质属性
- **模式**: `reset`
- **随机范围**:
  - 静摩擦系数 (static_friction): [0.3, 3.0]
  - 动摩擦系数 (dynamic_friction): [0.3, 3.0]
  - 恢复系数 (restitution): 保持为0.0（无弹性碰撞）
- **使用API**: `mdp.randomize_rigid_body_material`
- **目标**: 机器人所有刚体 (`body_names=".*"`)
- **采样效率**: 使用250个离散化桶 (`num_buckets=250`)

### 2. 腿部关节增益随机化 (`randomize_leg_joint_gains`)
- **功能**: 随机化12个腿部关节的驱动器增益（刚度和阻尼）
- **模式**: `reset`
- **随机范围**: 缩放因子在 [0.7, 1.3] 之间（均匀分布）
- **使用API**: `mdp.randomize_actuator_gains`
- **目标关节**: 
  - FL_hip_joint, FR_hip_joint, RL_hip_joint, RR_hip_joint
  - FL_thigh_joint, FR_thigh_joint, RL_thigh_joint, RR_thigh_joint
  - FL_calf_joint, FR_calf_joint, RL_calf_joint, RR_calf_joint
- **操作类型**: `scale` (乘法缩放)

### 3. 机械臂关节增益随机化 (`randomize_arm_joint_gains`)
- **功能**: 随机化6个机械臂关节的驱动器增益
- **模式**: `reset`
- **随机范围**: 缩放因子在 [0.7, 1.3] 之间（均匀分布）
- **使用API**: `mdp.randomize_actuator_gains`
- **目标关节**: joint1, joint2, joint3, joint4, joint5, joint6
- **操作类型**: `scale`

### 4. 基座质量随机化 (`randomize_base_mass`)
- **功能**: 在环境创建时，为机器人基座增加随机质量
- **模式**: `startup` (仅在环境创建时生效一次)
- **随机范围**: 增加 [0.0, 15.0] 公斤的质量
- **使用API**: `mdp.randomize_rigid_body_mass`
- **目标**: 机器人基座 (`body_names="base"`)
- **操作类型**: `add` (加法操作)
- **分布**: 均匀分布

### 5. 基座质心随机化 (`randomize_base_com`)
- **功能**: 在环境创建时，随机化机器人基座的质心位置
- **模式**: `startup`
- **随机范围**: x, y, z轴各在 [-0.15, 0.15] 米范围内偏移
- **使用API**: `mdp.randomize_rigid_body_com`
- **目标**: 机器人基座 (`body_names="base"`)
- **操作类型**: `add`

### 6. 夹爪质量随机化 (`randomize_gripper_mass`)
- **功能**: 在环境创建时，为夹爪增加随机质量
- **模式**: `startup`
- **随机范围**: 增加 [0.0, 0.1] 公斤的质量
- **使用API**: `mdp.randomize_rigid_body_mass`
- **目标**: 夹爪 (`body_names="gripperMover"`)
- **操作类型**: `add`
- **分布**: 均匀分布

### 7. 推动机器人 (`push_robot`)
- **功能**: 在训练过程中，以固定时间间隔对机器人施加瞬时速度变化
- **模式**: `interval` (按时间间隔触发)
- **触发间隔**: 每 8.0 秒推一次
- **随机范围**: 
  - x方向速度: [-0.5, 0.5] m/s
  - y方向速度: [-0.5, 0.5] m/s
- **使用API**: 官方函数 `mdp.push_by_setting_velocity`
- **目标**: 机器人根部

## 技术实现细节

### 使用官方API
所有事件均使用Isaac Lab官方提供的MDP事件函数，无需自定义实现：

- **物理材质随机化**: `mdp.randomize_rigid_body_material`
- **执行器增益随机化**: `mdp.randomize_actuator_gains`
- **刚体质量随机化**: `mdp.randomize_rigid_body_mass`
- **刚体质心随机化**: `mdp.randomize_rigid_body_com`
- **推动机器人**: `mdp.push_by_setting_velocity`

## 事件触发时机

- **reset模式事件**: 在每次环境重置时触发，包括：
  - 物理材质随机化
  - 腿部关节增益随机化
  - 机械臂关节增益随机化

- **startup模式事件**: 仅在环境初始化时触发一次，包括：
  - 基座质量随机化
  - 基座质心随机化
  - 夹爪质量随机化

- **interval模式事件**: 按固定时间间隔触发，包括：
  - 推动机器人（每8秒）

## 配置文件位置

所有事件配置位于：
- `source/acc/acc/tasks/manager_based/vbc/vbc_env_cfg.py` 中的 `EventCfg` 类

所有事件函数均使用Isaac Lab官方API，位于：
- `isaaclab.envs.mdp.events` 模块

## 参考文献

所有实现基于Isaac Lab官方文档：
- Event Manager API: https://github.com/isaac-sim/isaaclab
- 物理材质随机化: `mdp.randomize_rigid_body_material`
- 执行器增益随机化: `mdp.randomize_actuator_gains`
- 刚体质量随机化: `mdp.randomize_rigid_body_mass`
- 刚体质心随机化: `mdp.randomize_rigid_body_com`
