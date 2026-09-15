<!-- managed-by: jam-coding-role; file: LONG_RUNNING_TASKS.md -->
# Long jobs v1.4.0 — one logical wait, durable completion

授权不变：先确认精确命令、资源、终止条件和允许的后续 eval。超过 30 分钟或需断线连续性时，用 supervisor；单个无竞争任务不需要全量 team ledger。有资源竞争再启用 lease。

## 等待策略

完成一次必要的启动/稳定性检查后，基于预计剩余时间和真正的下一决策点，登记一个绝对 wait_until。预计还有 5h 就按 5h 等，10h 就按 10h 等；不要机械拆成每 30min 唤醒 Main。ETA 不明确时，只做一次有依据的估计；不能用“先看看”作为永久短轮询理由。

等待结束条件是：进程完成/失败、显式取消、已约定的截止或实际决策点。不能为了等足十小时而忽视两分钟后已经发生的失败。下例的 expected-seconds 是 ETA/复查边界，不是自动 kill 的时间。

```bash
# 从项目根执行；这里只展示接口，不授权下面命令对应的真实训练。
python .ai/scripts/run_supervisor.py prepare --name RUN_NAME \
  --command 'EXACT_AUTHORIZED_COMMAND' --cwd . --output logs/RUN_NAME.log \
  --expected-seconds 36000 --eta-source '已验证同类 run 时长/当前吞吐量' \
  --stop-condition 'Owner 定义的终止条件'
python .ai/scripts/run_supervisor.py launch --receipt .ai/runtime/runs/RUN_NAME/RUN_RECEIPT.json
python .ai/scripts/run_supervisor.py wait --receipt .ai/runtime/runs/RUN_NAME/RUN_RECEIPT.json
```

默认 backend 是 named tmux；缺失时明确失败，不能偷偷改执行环境。显式 process backend 用于隔离 smoke test 或已接受其生命周期边界的本机任务。命令日志不输出到等待 PTY；wait 只在完成、错误或 ETA 到达时输出一个小结果。OS 端读取状态不是 LLM 轮询。

## Codex 原生等待适配

项目可设 `background_terminal_max_timeout = 86400000`（毫秒；24h 上限，不是默认等待时长）。它只放宽 empty write_stdin 的等待上限，不改变 exec_command 初始 yield 上限。启动安静的 `wait` 命令拿到 session_id 后，用 `chars=""` 和预计剩余毫秒数调用 write_stdin；不要不断启动新的 sleep。

本机必须确认该键被有效加载、工具 schema 接受长 yield、实际等待没有被 App/代理层截短。先做一次 >5min 安静命令的原生 transport 验证，不运行真实训练。若宿主有更小硬上限，应明确记录限制；复用同一个 waiter 和剩余截止时间，不把 transport 返回当成必须分析日志的业务事件。

如果已验证本机 native background completion 能自动续接，用它接收 waiter 的完成事件，免去手工 write_stdin。后台 terminal 的 UI 退出事件不必然触发新的模型 turn。普通 background hook 不会在空闲 session 中自行开启 turn；不要用 Stop hook 循环或十小时 hook timeout 假装实现调度。

## Continuity 与验收

RUN_RECEIPT.json 固定记录 cwd、命令、source revision、资源、输出、ETA、checkpoint/eval 合同。STATUS.json 是运行事实；wait_until_epoch 不随每次 read 重置。若新证据要求调整等待，可用 `wait --until-epoch UNIX_SECONDS --reason ...`，新截止持久化到 WAIT_PLAN.json，而不是每次轮询重新估计。运行者自动原子写出 .ai/runtime/run-events/pending 的完成事件，不依赖 Main 手工 finalize 才记账。

process_state 与 acceptance_state 分离。exit 0 / checkpoint 存在只支持 PROCESS_COMPLETED / UNASSESSED。只有明确授权且事先写明 claim 的 evaluator 返回结果，才记录该 claim 的 PASS/FAIL。checkpoint 存在不代表可加载，evaluator 退出码也不替代其未实现的科研验收。

```bash
python .ai/scripts/run_supervisor.py status --receipt PATH
python .ai/scripts/run_supervisor.py finalize --receipt PATH --run-eval
python .ai/scripts/run_supervisor.py cancel --receipt PATH --reason 'Owner 授权取消'
python .ai/scripts/run_supervisor.py events
python .ai/scripts/run_supervisor.py ack --event EVENT_ID --resolution '已阅读并完成指定后续处理'
```

只有 prepare 明确 `--authorize-eval --eval-command ... --acceptance-claim ...` 后，自动或显式 eval 才可运行；`--auto-eval` 另行选择。finalize 幂等，不自动重跑已经开始过的 eval。收到事件不等于完成处理；hook 不自动 ack 或删除。并发会话只有已指定的 Main 能接管后续资源。

断线/关 App/额度耗尽时，tmux job 与事件可继续存在；**这不等于 session 已被唤醒**。下次 SessionStart 或按需 events 恢复。主机休眠、断电、SIGKILL 与资源安全 watchdog 不由 tmux 保证；实机仍需独立硬件安全系统。不要用日志静默自动判定训练挂死。

v1.3 活动任务不要迁移：先保留旧脚本结束/人工接管。v1.4 拒绝把旧 receipt 的 PASS 自动升级为实验通过。
