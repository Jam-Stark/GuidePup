<!-- managed-by: jam-coding-role; file: MEMORY_GOVERNANCE.md -->
# Memory governance v1.4.0

保留现有 MEMORY.md → subsystem → entry 路由和 description.md / TODO.md / DONE.md；不批量改写历史。Memory 是可复用事实与决策，不是日志、聊天、运行 heartbeat 或授权来源。

## 归类与提交

事实/决定/已验证纠错放对应 entry；可复用操作步骤放已有 skill/runbook 并从 entry 链接；活动任务/等待截止放 .ai/runtime；未经验证的猜想留 candidate，不冒充当前事实。一条事实只有一个 canonical home。

触发新 candidate 的条件：新 durable fact、反复重查、已确认旧结论失效、混合主题、漂移副本或入口太大。没有 candidate 就不启动 curator，也不按固定 25 步或 30 分钟唤醒模型做 reflection/dream。

Main/curator 拥有 canonical write authority；worker 只提议。candidate 必须含 source、evidence level、scope 和 target route。同一事实去重，不把重述当新增学习。相互矛盾的事实保留 conflict/provenance，不能按“最新时间”自动覆盖。

Main 批准后可 create/move/split/merge/supersede/retire，并同步必要 router。维护项目既有时间标准（ForceControl 用同一 HKT 时间写 description/TODO/DONE），不为更新元数据而无意义改动三份文件。长期 entry 保存 source_of_truth、read_when、last_verified、status 和 supersedes；原始实验数据只链接。

## 检索与压缩

先走已知精确 route；未知时 search 只返回最相关的最多两个路径和简短摘要，再定向阅读。index 是从 canonical 文件生成的派生视图，不是第二套事实；内容未变就不重写 generated_at，不向稳定系统提示注入每轮计数。

压缩前若已形成 durable learning，提交 candidate；未完成工作仅把 run receipt、待决问题、下一动作和授权边界写到 .ai/runtime/handoff.md。机器 hook 不总结聊天、不调用 LLM、不把摘要自动提升为 truth。PostCompact 从 runtime 恢复连续性，不批量重读全部 memory。

```bash
python .ai/scripts/memory_curator.py candidate-add --title ... --scope ... \
 --source 'commit/path/symbol or evidence artifact' --evidence RUNTIME_PASS \
 --action append --target-route memory/SUBSYSTEM/ENTRY --body ...
python .ai/scripts/memory_curator.py candidate-list
python .ai/scripts/memory_curator.py candidate-mark --candidate ID --status resolved --resolution ...
python .ai/scripts/memory_curator.py reindex
python .ai/scripts/memory_curator.py search --query '关键词'
```

不导入 OMO 的 people/soul、人际关系推断、自动 Git commit/global sync；不在两个系统中建立重复 canonical memory。现有 OMO memory 可提供检索/提议，但 ForceControl 项目事实仍以当前项目 source/evidence 为准。
