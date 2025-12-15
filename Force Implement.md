# Isaac Lab Continuous Force Implementation Guide

在 Isaac Lab 的 Manager-Based Workflow 中，施加持续力（如风力、重力补偿、持续推力）的最佳实践与 Isaac Gym 或 Direct Workflow 有所不同。

## 核心问题

1.  **Event API 的局限性**：
    `EventTerm` (mode="interval") 是基于环境步 (Environment Step) 触发的。如果 `decimation > 1`，它只能在每 N 个物理步后触发一次，无法实现“每个物理仿真步 (Physics Step) 都施加力”的效果，导致力是脉冲式的而不是持续的。

2.  **`_apply_action` 的误区**：
    覆写 `_apply_action()` 是 Direct Workflow (`DirectRLEnv`) 的做法。在 Manager-Based Workflow (`ManagerBasedRLEnv`) 中，环境逻辑被解耦为 Managers (ActionManager, EventManager 等)，直接修改环境类会破坏架构的模块化。

## 最佳实践：自定义 ActionTerm

在 Manager-Based 架构中，`ActionManager` 会在每个物理步 (Physics Step) 调用所有 Action Terms 的 `apply_actions()` 方法。

我们可以利用这一机制，创建一个 **维度为 0 (`dim=0`)** 的自定义 Action Term。它不消耗 RL Agent 的动作空间，但会在每一帧物理仿真中被执行。

### 1. 实现自定义 Action 类

创建 `actions.py`：

```python
from __future__ import annotations
import torch
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.assets import Articulation, RigidObject
from isaaclab.utils import configclass

@configclass
class ConstantForceActionCfg(ActionTermCfg):
    """Configuration for the constant force action term."""
    class_type = None # 将被设置为 ConstantForceAction
    asset_name: str = "robot"
    body_name: str | None = "base" # 施加力的刚体名称
    force: tuple[float, float, float] = (0.0, 0.0, 0.0) # 力向量 (x, y, z)
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0) # 扭矩向量 (x, y, z)
    dim: int = 0 # 关键：维度为0，不占用 Agent 动作空间

class ConstantForceAction(ActionTerm):
    """
    Action term that applies a constant external force to a rigid body.
    Executed at every physics step.
    """
    cfg: ConstantForceActionCfg

    def __init__(self, cfg: ConstantForceActionCfg, env):
        super().__init__(cfg, env)
        self._asset: Articulation | RigidObject = env.scene[cfg.asset_name]
        
        # 解析 Body Index
        if cfg.body_name is None:
            self._body_idx = None 
        else:
            self._body_idx, _ = self._asset.find_bodies(cfg.body_name)
            
        # 预处理力张量
        self._force = torch.tensor(cfg.force, device=env.device).repeat(env.num_envs, 1)
        self._torque = torch.tensor(cfg.torque, device=env.device).repeat(env.num_envs, 1)

    def process_actions(self, actions: torch.Tensor):
        # dim=0, 没有来自 Agent 的动作需要处理
        # 如果需要动态变化的力（如随机风），可以在这里更新 self._force
        pass

    def apply_actions(self):
        # 核心：此方法会在 decimation 循环内的每个物理步被调用
        self._asset.set_external_force_and_torque(
            forces=self._force,
            torques=self._torque,
            body_ids=self._body_idx
        )

# 注册实现类
ConstantForceActionCfg.class_type = ConstantForceAction
```

### 2. 在环境配置中启用

在 `env_cfg.py` 的 `ActionsCfg` 中添加该 Action：

```python
from . import mdp # 假设 actions.py 已在 mdp 包中导出

@configclass
class ActionsCfg:
    # 原有的关节控制动作
    joint_pos = mdp.JointPositionActionCfg(...)

    # 新增：持续外力 (例如持续风阻)
    # 因为 dim=0，它完全是“背景”效果，不会影响 RL 策略网络的输入/输出维度
    constant_wind = mdp.ConstantForceActionCfg(
        asset_name="robot",
        body_name="base",
        force=(1.0, 0.5, 0.0), # 持续施加的力向量
        dim=0
    )
```

## 优势

1.  **高频率**：真正做到了在每个 Physics Step (dt) 施加力，物理仿真更准确。
2.  **模块化**：不需要修改环境核心代码，只需插拔 Config。
3.  **灵活性**：可以在 `process_actions` 中编写逻辑来实现时变力（如正弦波风力、随机扰动），甚至可以读取环境观测来计算力（如空气动力学阻力）。
