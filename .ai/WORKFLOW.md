<!-- managed-by: jam-coding-role; file: WORKFLOW.md -->
# Adaptive workflow v1.4.0

FAST：明确的小改动、简单问答或临时检查，由 Main 直接完成。
STANDARD：普通实现、调试和设计，按目标选择最小充分执行路径。
HIGH_RISK：涉及受保护副作用的授权覆盖层，不是“复杂”的同义词。

## 主动委托

进入深度工作前，判断独立工作线、specialist context、独立审阅或并行隔离是否有实质收益。有则立即委托最少必要的 1–3 个 focused agents，不等 Owner 说“team”，也不先把应委托的工作全部做完。任务紧耦合且 Main 更便宜时直接做；不再强制输出 NO_DELEGATION_REASON 模板。更高层或 runtime 禁止子 agent 时服从限制。

Main 管 scope、acceptance、write/resource authority、Git 和整合；P2P 传技术事实，不传权限。一个路径/排他资源同一时间只有一个 writer/owner。高风险副作用未获授权前，只开展安全的只读工作。

## 按需设施

| 条件 | 读取/启用 |
|---|---|
| 多 writer、排他资源、跨 session DAG、正式 candidate review/QA | .ai/TEAM_STATE.md；按实际需求 ledger/lease/freeze |
| 预计超过 30 分钟，或必须抗会话中断 | .ai/LONG_RUNNING_TASKS.md；先持久化 ETA 与 receipt，再长等待 |
| 已验证且可复用的新事实、纠错或重复发现 | .ai/MEMORY_GOVERNANCE.md；candidate/retrieval/curation |
| RL、simulation、benchmark、causal claim 或实机 | .ai/SCIENTIFIC_ENGINEERING.md |
| Owner 指定跨阶段多 planner | .ai/STAGE_DECISION.md |
| Owner 要求或 stage 明确声明交付 | .ai/ARTIFACT_HANDOFF.md |

只读小团队不默认启用 ledger。普通改动不默认 freeze、curator、review wave 或 artifact。只关闭实际启用的设施。

训练、评估、渲染、部署采用 .ai/PROJECT.md 的已验证环境/命令入口；过期时追踪实际代码，不凭历史猜命令。
