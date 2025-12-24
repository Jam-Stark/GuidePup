UniFP 复现指南：力控任务生成逻辑详解

本文档详细拆解了 _push_gripper 函数中的力控指令生成逻辑。该逻辑用于在训练过程中随机生成变化的力控目标（Force Command），迫使 Policy 学习如何输出指定的末端力。

1. 数据结构准备 (Per-Environment State)

在环境类中，你需要为每一个环境实例维护以下状态张量（Tensor），长度等于 num_envs：

计时器与触发器:

episode_length_buf: 当前环境运行的绝对时间步数（长整型）。

push_interval: 下一次触发任务的间隔步数（例如每 200 步触发一次）。

push_end_time: 当前任务预计结束的绝对时间步。

push_duration: 当前任务设定的力变化持续时间（决定力变化的快慢）。

状态标志:

selected_env_ids (Active Flag): bool 或 int，标记当前是否正在执行推力任务。

freed_envs: bool，标记当前周期是否被“豁免”（轮空，不执行任务）。

物理量存储:

force_target: [N, 3]，当前任务的目标力向量 $(F_x, F_y, F_z)$。

current_Fxyz: [N, 3]，当前这一帧应该输出的指令力（随时间变化的瞬时值）。

commands: [N, D], 最终输入给神经网络的观测向量，包含上述计算出的力。

2. 时间单位与换算

- 行为逻辑按“环境步”计时：`episode_length_buf` 每环境步 +1。
- 一个环境步时长 `step_dt = env.step_dt`；若缺省则 `step_dt = sim.dt * decimation`。
- 支持秒级配置并自动换算为步：
  - `push_interval_range_s` → 触发间隔（秒）
  - `duration_range_s` → 上升/下降段持续（秒）
  - `settling_time_s` → 保持段（秒）
- 若秒级字段为空，则使用旧的步数字段 `push_interval_range`、`duration_range`、`settling_time_steps` 作为回退。

3. 逻辑流程详解 (Step-by-Step)

该逻辑在每一个仿真步（Step）中执行。

第一阶段：触发与初始化 (Trigger & Init)

目标：决定哪些环境在这一帧开始新的任务循环，并生成任务参数。

判断触发条件:

检查 episode_length_buf % push_interval == 0。

筛选出满足条件的 env_ids (记为 new_ids)。

生成任务参数 (针对 new_ids):

概率豁免 (Freed Check):

生成随机数 rand(0, 1)。

如果 rand > prob (0.8)，标记 freed_envs = True，否则为 False。

生成目标力 (Target Force):

在 [min_force, max_force] 范围内随机采样 $F_x, F_y, F_z$。

生成持续时间 (Duration):

在 [min_duration, max_duration] 范围内随机采样。

时间安全裁剪 (Safety Clip):

核心约束：任务的总时长（上升+稳定+下降）不能超过两次触发之间的间隔。

公式：duration = min(duration, (interval - settling_time) / 2)。

解释：除以2是因为主要由上升段和下降段组成，需确保两者加起来不超过 (总间隔 - 稳定时间)。

计算结束时间:

push_end_time = current_time + duration (这是上升段结束的时间点)。

激活标记:

设置 selected_env_ids[new_ids] = 1。

第二阶段：任务执行 (Execution)

目标：针对被标记为 Active (selected_env_ids == 1) 的环境，根据当前时间计算瞬时力。

此阶段分为三个子步骤，构成了 梯形/三角形 力控曲线。

Step 2.1: 上升期 (Ramp Up)

条件: current_time < push_end_time

逻辑: 力从 0 线性增加到 force_target。

公式:


$$F_{current} = F_{target} \times \frac{current\_time - (end\_time - duration)}{duration}$$


(代码中使用了 clamp 确保比值在 0~1 之间)

操作: 将计算出的 F_current 写入 self.commands。

Step 2.2: 稳定期 (Settling / Hold)

条件: push_end_time <= current_time <= push_end_time + settling_time

逻辑: 代码中实际上没有显式处理这个区间的 if 分支，这意味着在这个区间内：

既不满足上升条件，也不满足下降条件。

隐含行为：self.current_Fxyz 保持在 Step 2.1 结束时的最后状态（即 F_target），或者如果逻辑严密，应该显式维持 F_target。

注：原代码逻辑主要依靠 Step 1 和 Step 2 的互斥区间。在中间的 gap 期，如果不更新 commands，它将保持上一帧的值（即满幅 Target 力）。

Step 2.3: 下降期 (Ramp Down)

条件: current_time > push_end_time + settling_time

逻辑: 力从 force_target 线性衰减回 0。

公式:


$$F_{current} = F_{target} - F_{target} \times \frac{current\_time - (end\_time + settling\_time)}{duration}$$

操作: 将计算出的 F_current 写入 self.commands。

第三阶段：重置与清理 (Reset)

目标：当任务彻底结束时，清理状态，准备下一次循环。

判断结束条件:

current_time >= push_end_time + settling_time + duration

筛选出满足条件的 reset_ids。

执行重置 (针对 reset_ids):

selected_env_ids = 0 (取消激活)。

force_target = 0, current_Fxyz = 0。

push_end_time = 0, push_duration = 0。

commands = 0 (关键：观测值归零)。

更新间隔: 随机采样新的 push_interval (防止 Agent 记忆固定周期)。

第四阶段：豁免处理 (Freed Handling)

目标：确保被豁免的环境完全静默。

针对所有标记为 freed_envs 的环境：

强制将 commands 设为 0。

强制将内部状态（Target, Duration等）设为 0。

确保 Active Flag 为 0。

3. 关键数学逻辑总结

整个过程实质上是在生成一个含平顶的脉冲信号：

Force Magnitude
    ^
    |          _______ (Settling Time)
    |         /       \
Target Force |        /         \
    |       /           \
    |      /             \
    |_____/               \__________
          |   |       |   |
        Start End   End+Settling  Reset
          <--->       <--->
         Duration    Duration


Duration: 定义了斜坡的陡峭程度。Duration 越短，力变化越剧烈。

Settling Time: 定义了力保持在最高点的时间。

Interval: 定义了整个波形的出现频率。