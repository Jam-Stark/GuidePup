# ForceControl Memory

本文件是项目根目录的 file-based memory system 顶层入口。开始实现、调试、review 或文档更新前，先读取本文件，再按任务类型读取最小必要 memory entry。

## 路由

- 当前 workflow v1.4.0 与同步证据：`memory/workflow/description.md`；角色按 `.codex/TEAM.md`，历史 roster 不代表存活 agent。

- 跨机器继续当前导航测试：`prototype/MIGRATION.md`，发布分支 `codex/a2-lightnav-prototype`，迁移标记 `20260915_215130_HKT`。

- 旧 A2 + LightNav + Pi navigation：见 `memory/navigation-prototype/description.md`，入口 `prototype/README.md`；最新通用框架修正/上下文回放/运行限制 `prototype/NAVIGATION_FRAMEWORK.md`，此前YOLO与两模型72轮 `prototype/NAVIGATION_YOLO.md`，压力及80帧异常 `prototype/NAVIGATION_STRESS.md`，v2 `prototype/NAVIGATION_V2.md`。

- MuJoCo sim2sim experiment 配置核查/解释（alan）：见 `memory/mujoco-experiment/description.md`
- MuJoCo 官方 API / XML / stepping 规范核查：见 `memory/mujoco_official_api/description.md`
- Manager-based uniFP Aliengo cfg 核查/解释/修改方案（john）：见 `memory/manager_based_cfg/description.md`
- 领域角色历史与任务分配路由：见 `memory/agent_roster/description.md`

## 使用约束

- Owner 2026-09-14：后续所有下载使用服务器自身网络直连，不走代理。下载命令须显式清除大小写 HTTP/HTTPS/ALL_PROXY，设置 NO_PROXY/no_proxy=`*`；tmux 可能保留旧代理环境，不能只看当前 shell。安装工具的显式代理配置也需覆盖。
- Owner 2026-09-15：迁移所需 Git commit/push 和 checkpoint/log 的 Google Drive 上传已授权；本次 Drive 上传必须走代理。下载与模型服务继续按直连入口执行。
- Qwen 不在本机下载：使用 Owner 提供的实验室服务 `http://vic.precognition.team:8888/v1`，模型 `Qwen3.6-35B-A3B-FP8/`，无需鉴权。

- memory 只记录可复用的项目事实、施工状态、设计决策、当前 blocker 和下一步 TODO，不作为聊天历史。
- 每个 entry 先读 `description.md`；需要施工或判断当前状态时，再读 `TODO.md` 和 `DONE.md`。
- 完成 TODO 时，同步更新对应 `TODO.md`、`DONE.md`、`description.md` summary；如改变路由，同步更新本文件和 `memory/MEMORY.md`。
- Timestamp 格式使用 `YYYY-MM-DD HH:MM HKT`。
