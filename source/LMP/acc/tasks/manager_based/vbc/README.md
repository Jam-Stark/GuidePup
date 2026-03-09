# VBC Task - 新任务配置

## 概述

`vbc` 是一个独立于 `acc` 的新任务，位于同一个项目中。两个任务完全独立，通过不同的任务 ID 来区分。

## 文件结构

```
source/LMP/acc/tasks/manager_based/
├── __init__.py           # 导入所有任务
├── acc/                  # ACC 任务
│   ├── __init__.py
│   ├── acc_env_cfg.py
│   ├── agents/
│   ├── commands/
│   └── mdp/
└── vbc/                  # VBC 任务（新建）
    ├── __init__.py
    ├── vbc_env_cfg.py
    ├── agents/
    │   ├── __init__.py
    │   ├── rsl_rl_ppo_cfg.py
    │   └── skrl_ppo_cfg.yaml
    ├── commands/
    └── mdp/
```

## 任务注册

- **ACC 任务**: `Template-Acc-v0`
- **VBC 任务**: `Template-Vbc-v0`

## 使用方法

### 训练 ACC 任务
```bash
python scripts/rsl_rl/train.py --task=Template-Acc-v0 --num_envs=4096
```

### 训练 VBC 任务
```bash
python scripts/rsl_rl/train.py --task=Template-Vbc-v0 --num_envs=4096
```

### 播放训练好的模型
```bash
# ACC
python scripts/rsl_rl/play.py --task=Template-Acc-v0 --num_envs=64

# VBC
python scripts/rsl_rl/play.py --task=Template-Vbc-v0 --num_envs=64
```

## 配置说明

### VBC 环境配置 (`vbc_env_cfg.py`)

当前 VBC 配置包含：
- **场景**: 简单的地面和机器人
- **观测**: 基础运动学观测（速度、姿态、关节状态）
- **动作**: 关节位置控制
- **命令**: 速度跟踪命令
- **奖励**: 基础的运动跟踪和稳定性奖励
- **终止条件**: 超时和机身高度检查

### 自定义修改

你可以根据实际需求修改 `vbc_env_cfg.py`:

1. **修改机器人模型**: 更新 `VbcSceneCfg.robot` 中的 URDF 路径
2. **添加观测**: 在 `ObservationsCfg.PolicyCfg` 中添加新的观测项
3. **修改奖励**: 在 `RewardsCfg` 中调整权重或添加新奖励
4. **添加传感器**: 在 `VbcSceneCfg` 中添加相机、接触传感器等

## 下一步

1. ✅ 基础结构已创建
2. ⚠️ 需要根据你的具体任务修改 `vbc_env_cfg.py`
3. ⚠️ 如需自定义 MDP 函数，在 `vbc/mdp/` 中添加
4. ⚠️ 如需自定义命令生成器，在 `vbc/commands/` 中添加

## 注意事项

- 两个任务**完全独立**，互不影响
- 可以同时训练两个任务
- 日志和检查点会分别保存在不同的实验目录中
- 只要任务 ID 不同，就是独立的任务
