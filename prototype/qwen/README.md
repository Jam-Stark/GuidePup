# 实验室 Qwen 任务模型

Owner 指定复用已部署的实验室 OpenAI-compatible 服务，为 Pi 提供视觉理解、用户交流与工具调用。

| 配置 | 值 |
| --- | --- |
| API base URL | `http://vic.precognition.team:8888/v1` |
| 精确模型 ID | `Qwen3.6-35B-A3B-FP8/`，保留末尾 `/` |
| 服务认证 | 无需 API key |
| Pi 接入 | `openai-compatible` provider，启用视觉输入 |

主线已成功读取 `/v1/models` 并确认上述模型 ID。当前真实模型的图片与工具回合由 Pi 接入任务执行，运行结果统一记录在 `../RUN_STATUS.md`。本目录已删除废弃的本地 Qwen 安装、下载和 GPU 服务配置；不需要在本机部署第二份模型。

## 只读图片与工具演示

先按 [Pi 说明](../pi/README.md) 配置 `prototype/pi/.env`，并保持 MuJoCo 服务运行。从 worktree 根目录执行：

```bash
.local/envs/node-v22.23.2-linux-x64/bin/node --env-file=prototype/pi/.env prototype/qwen/pi_readonly_demo.mjs
```

`pi_readonly_demo.mjs` 复用已有 Pi Agent，只注册 `status` 和 `scene` 两个只读工具。它要求模型读取真实状态及前向 RGB，判断原目标 `Turn right and go through the doorway` 是否已经完成，并说明图像依据。此次演示不提交导航、暂停或恢复操作。

LightNav 的 stop 是到达候选，不能单凭这一信号认定已经穿门。演示用于检查模型是否能够结合图像、原始需求和执行状态完成这一判断。

输出为 `.local/outputs/qwen_pi_readonly.json`，包含工具调用、模型回答和延迟。`firstModelDeltaMs` 是客户端观测到首个流式内容增量的时间，`firstTextMs` 是首个文本增量的时间；`elapsedMs` 覆盖整个回合，包括工具执行及后续模型请求。
