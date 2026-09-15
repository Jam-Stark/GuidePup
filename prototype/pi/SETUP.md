# 新机器配置 Pi、Qwen 和 ZenMux Gemini

在仓库根目录运行 `prototype/pi/setup.sh`。它安装项目自己的 Node 22.23.2、Pi 0.85.1 及锁定依赖，
构建 `prototype/pi/dist/`，并在缺少配置时将 `.env.example` 复制为 `prototype/pi/.env`。
不用在全局 Pi 或其他项目的配置中注册这些模型。本原型通过 `src/model.ts` 建立自己的 provider。

## 默认 Qwen

新机器的 `prototype/pi/.env` 使用以下完整配置：

```dotenv
SIM_URL=http://127.0.0.1:8766
PI_PROVIDER=openai-compatible
PI_MODEL=Qwen3.6-35B-A3B-FP8/
PI_BASE_URL=http://vic.precognition.team:8888/v1
PI_CONTEXT_WINDOW=262144
PI_MODEL_MAX_TOKENS=2048
PI_VISION=1
PI_MAX_TOKENS=2048
PI_API_KEY=
```

精确模型 ID 包含末尾 `/`。实验室服务无需鉴权，不下载本地 Qwen。
`src/model.ts` 为 Qwen 设置 `enable_thinking=false`；不改成 Gemini 的 provider 或上下文大小。
新机器必须能访问实验室的 `vic.precognition.team:8888`。
已有 shell 中的 `PI_*` 环境变量优先于 `.env`，配置不一致时先清理具体覆盖项。

## ZenMux 调用 Gemini

在新机器私下设置 `ZENMUX_API_KEY` 环境变量，或创建 `.local/secrets/zenmux.env`：

```dotenv
ZENMUX_API_KEY=<由Owner在新机器配置的真实ZenMux密钥>
```

该文件不进入 Git/Drive；新机器 AI 不应要求把密钥贴进对话或日志。若凭证尚未提供，继续完成其他准备和 Qwen 测试，
明确告知 Owner 需要在新机器配置这个文件。旧机器的私有文件不会随迁移附件复制。

始终用 `prototype/pi/zenmux.sh gemini ...` 选择 Gemini。launcher 为这个进程设置：

| 参数 | 值 |
| --- | --- |
| provider / API | `openai-compatible` / `openai-completions` |
| endpoint | `https://zenmux.ai/api/v1` |
| model | `google/gemini-3.8-flash` |
| context window | `1048576` |
| model/output cap | `4096` / `4096` |
| vision | 开启 |
| reasoning effort | `low` |
| credential | `ZENMUX_API_KEY` |

不要把 Gemini 配置写回默认 Qwen `.env`，也不要换成 Google 直连接口。
当前 launcher 会通过 `https://1.1.1.1/dns-query` 解析 `zenmux.ai`，再直接请求其 HTTPS API；
新机器需要允许这条实际网络路径。DNS、TLS、认证错误应显式报告，不自动替换 provider 或模型。

## 确认配置，再执行任务

```bash
prototype/pi/run.sh model
prototype/pi/zenmux.sh gemini model
```

这两个命令只打印模型配置和凭证是否存在，不显示密钥，也不证明模型已成功响应。
在启动长时间仿真前，使用安装后的 Pi `configuredModel()` / `models.complete()` 接口，
按上述两个 profile 分别发起一次简短文本请求，确认实际 endpoint、认证和模型 ID 可用。
沿用当前入口的直连方式与 30 秒请求上限，记录模型 ID、结果/错误和耗时即可，不记录认证头。
Pi API 调用形状可直接参照 `src/decision.ts`，不要使用伪造响应代替连接验证。

随后按 [迁移说明](../MIGRATION.md) 运行 `prototype/sim/run_framework_reproductions.py`。
runner 的 `--model qwen` 与 `--model gemini` 会分别使用对应 launcher；两者共用同一仿真、场景和评价协议。
