# uniFP 可视化配置摘要

本文档总结 uniFP Aliengo 任务中当前启用的可视化元素，以及它们对应的观测/物理量含义。

## 开关位置

- 速度命令可视化：`source/acc/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py` 中 `CommandsCfg.LocomotionCommandCfg.debug_vis = True`。
- 末端目标/力控可视化：`source/acc/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py` 中 `CommandsCfg.TargetPositionCommandCfg.debug_vis = True`。
- 接触传感器 `debug_vis` 均为 `False`，不会显示足端接触或非法接触的几何体。

## 可视化元素与含义

- `/Visuals/Command/velocity_goal`（红色箭头）
  - 含义：力偏置后的目标线速度方向（xy 平面）。
  - 计算：`locomotion_commands` 的线速度 + `(base_force_ext + base_force_command) / base_kd`。
  - 参考：`source/acc/acc/tasks/manager_based/uniFP_aliengo/commands/velocity_command.py`。

- `/Visuals/Command/velocity_current`（青色箭头）
  - 含义：当前基座线速度方向（xy 平面）。
  - 数据：`robot.data.root_lin_vel_b[:, :2]`。

- `/Visuals/Command/EE/goal`（黄色球）
  - 含义：末端执行器目标位置（世界坐标）。
  - 数据：`target_position_commands`（内部以球坐标采样后转换）。
  - 参考：`source/acc/acc/tasks/manager_based/uniFP_aliengo/commands/dynamic_manipulation_command.py`。

- `/Visuals/Command/EE/actual`（蓝色球）
  - 含义：末端执行器真实位置（世界坐标）。
  - 数据：机器人实际 EE 位姿。

- `/Visuals/Command/EE/goal_offset`（紫色球）
  - 含义：目标位置 + 力/阻抗偏置后的目标（世界坐标）。
  - 计算：`goal_w + (gripper_force_ext + gripper_force_command) / kp`。

- `/Visuals/Command/Force/gripper_ext`（蓝色箭头）
  - 含义：夹爪外力扰动（世界坐标）。
  - 数据：`gripper_force_ext`。

- `/Visuals/Command/Force/gripper_cmd`（绿色箭头）
  - 含义：夹爪力指令（世界坐标，由本地力指令转换）。
  - 数据：`gripper_force_command`。

- `/Visuals/Command/Force/base_ext`（蓝色箭头）
  - 含义：基座外力扰动（世界坐标）。
  - 数据：`base_force_ext`。

- `/Visuals/Command/Force/base_cmd`（绿色箭头）
  - 含义：基座力指令（世界坐标，由本地力指令转换）。
  - 数据：`base_force_command`。

## 与观测项的对应关系（简要）

- 速度相关：`commands` 中 Base Linear Velocity（索引 0-1），`base_lin_vel`（privileged/pred）。
- 末端目标/偏置：`commands` 的 EE Sphere Pos（索引 3-5），`ee_pos_in_sphere`，`ee_goal_offset`（privileged）。
- 力相关：`commands` 中 EE/Base Force（索引 9-14），`gripper_forces_local`、`base_forces_local`（privileged/pred）。

