# Memory Index

本目录保存 ForceControl 的 file-based memory entries。读取时按根目录 `MEMORY.md` 路由进入，避免一次性读取无关 subsystem。

## Entries

- 当前 workflow v1.4.0 与同步证据：`memory/workflow/description.md`；角色按 `.codex/TEAM.md`，历史 roster 不代表存活 agent。

- `navigation-prototype/`：旧A2隔离worktree、GPU3绑定、通用工具/空间证据/自动完成框架、YOLO RGB-D、单段buffer与force brake；最新记录回放及NVIDIA运行限制、此前72轮对照和v2历史按description路由。

- `mujoco-experiment/`：alan 长期职责；当前 MuJoCo sim2sim experiment 配置核查/解释；维护 failing command、runtime 参数、policy/model/export、delay/friction/contact/first divergence TODO。
- `mujoco_official_api/`：bob 长期职责；MuJoCo 官方 docs-first 约束；IsaacLab 到 MuJoCo sim2sim 的 XML/API/solver/contact/control stepping 规范证据。
- `manager_based_cfg/`：john 长期职责；当前 manager-based 版本 cfg 的核查、解释和修改方案；重点维护 `unifpaliengo_env_cfg.py` 的 observation/action/command/reward/termination/noise/delay/domain randomization 契约与 sim2sim 对齐 TODO。
- `agent_roster/`：长期子agent roster；记录 alan/bob/john 的 agent id、职责 memory 入口、回答前证据约束与后续任务分配规则。

## Build History

- 2026-06-03 22:51 HKT：初始化 `mujoco-experiment` entry，记录 alan 职责、回答 memory 证据约束、failing path 和初始 sim2sim experiment TODO。
- 2026-06-03 22:52 HKT：初始化 memory system 最小路由，并新增 `mujoco_official_api` entry。
- 2026-06-03 22:52 HKT：新增 `manager_based_cfg` entry，记录 john 职责、回答 code 引用约束、当前 cfg 摘要和 sim2sim 对齐 TODO。
- 2026-06-03 22:55 HKT：主控复核并补齐 `mujoco-experiment` 路由；新增 `agent_roster` entry，记录三个长期子agent 的运行时 id 与分工。
