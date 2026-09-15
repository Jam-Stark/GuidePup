<!-- managed-by: jam-coding-role; file: MODEL_ROUTING.md -->
# Codex model routing v1.4.0

Main 的 model / model_reasoning_effort 由 App/用户设置决定；不要在项目 config 或 bootstrap 写入默认 Main 模型/effort。子 agent 则必须显式选择已配置角色，最高 high；本规则已经授权范围内的 high，不需要每次重新申请“Ultra”许可。昂贵实验/硬件等副作用仍按 ROLE 授权。

## 分工

Astra/low：scope_planner、default；Astra/medium：code_reviewer、semantic/IsaacLab reviewer、语义密集的 implementation/IsaacLab worker；Astra/high：一个真正未解决的 deep_researcher 问题。
Terra/high：worker 的普通、范围准确的实现；Terra/medium：runtime_qa 的执行与失败解释。
Luna/high：context_researcher、explorer 的窄检索；Luna/medium：memory_curator 的机械整理。

复杂实现直接派语义 worker，不先让 Terra 试错再串行升级。reviewer 按具体 concern 触发，不组成默认审批队列。通常最多一个 high-effort Astra lane；并发上限不是必须开满的数量。

## 上下文与成本

每个角色 TOML 都显式设定 model_context_window / model_auto_compact_token_limit / model_auto_compact_token_limit_scope="total"。本包 Astra 子角色为 262144/229376；Terra/Luna 为 196608/163840。不得继承 Main 的 516000/464400 后只修改 model 字段。

给子 agent 新鲜且有界的 brief：问题、必要事实、精确路径、写入边界、完成条件；当前 MultiAgentV2 必须显式传 `fork_turns="none"`（默认值是 `all`），且不能传 `fork_context`。不要复制 Main 全历史或 fork 一个已膨胀的上下文。工具输出只返回与判断相关的片段，原始大日志保留文件引用。角色上下文上限是配置意图，auto compact 不是计费硬闸；首次请求、工具大输出、prefix 与本机继承行为仍需要 runtime 核对。

禁止用 spawn 参数临时改成更小模型却保留大角色窗口。未知角色/自定义角色也须通过 .ai/scripts/codex_preflight.py 的审计。没有实际 effective-config 或工具 schema 证据，不宣称“已保证不会长上下文加价”。

Codex 的 Astra 长上下文计费例外不等于 Astra API 免费长上下文。通过 API key 使用时，以当前 API 规则为准；本包不会替用户改账户计费设置。Main 临时切到 5.6 时，先将 Main context/compact 调低，或改用小上下文的新会话。

角色的 read-only/workspace-write 是配置意图；权限仍受宿主和 Main 的 runtime snapshot 约束。不能把一行 sandbox_mode 当成已经证明的 OS 隔离。
