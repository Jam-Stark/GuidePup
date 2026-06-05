# ForceControl Memory

本文件是项目根目录的 file-based memory system 顶层入口。开始实现、调试、review 或文档更新前，先读取本文件，再按任务类型读取最小必要 memory entry。

## 路由

- MuJoCo sim2sim experiment 配置核查/解释（alan）：见 `memory/mujoco-experiment/description.md`
- MuJoCo 官方 API / XML / stepping 规范核查：见 `memory/mujoco_official_api/description.md`
- Manager-based uniFP Aliengo cfg 核查/解释/修改方案（john）：见 `memory/manager_based_cfg/description.md`
- 长期子agent roster 与任务分配规则：见 `memory/agent_roster/description.md`

## 使用约束

- memory 只记录可复用的项目事实、施工状态、设计决策、当前 blocker 和下一步 TODO，不作为聊天历史。
- 每个 entry 先读 `description.md`；需要施工或判断当前状态时，再读 `TODO.md` 和 `DONE.md`。
- 完成 TODO 时，同步更新对应 `TODO.md`、`DONE.md`、`description.md` summary；如改变路由，同步更新本文件和 `memory/MEMORY.md`。
- Timestamp 格式使用 `YYYY-MM-DD HH:MM HKT`。
