# 固定 RGB-D 帧对照

研究的是目标实体与像素 ROI、距离层的对应关系。每个模型使用相同 case、相同用户文本和同一套 Pi 工具说明；每个 case 创建独立 Agent，只开放 `status`、`scene`、`measure_region`，不提交导航或完成声明。

先由统一入口编译 Pi，并启动 Main 提供的静态 replay 服务，再运行：

```bash
bash prototype/pi/experiments/compare.sh \
  --cases far,doorway,near \
  --prompt-file /absolute/prompt.txt \
  --output /absolute/new-output-directory
```

case ID 以 replay 实际配置为准；`--replay-url` 默认 `http://127.0.0.1:8767`。3 帧 × 2 种表达 × 3 模型共 18 个 case，顺序为 Qwen、Gemini、DeepSeek，每帧先 baseline 后 enhanced。输出目录中的模型子目录必须未存在，避免覆盖失败记录。最坏总时长约 45 分钟，应由 Main 在独立 tmux 中运行。

当前冻结输入清单为 `.local/outputs/depth_translator_compare/reference/case_manifest.json`。启动只读服务：

```bash
.local/envs/sim/bin/python prototype/sim/depth_replay.py \
  --manifest .local/outputs/depth_translator_compare/reference/case_manifest.json
```

- **baseline**：scene 只有编号图；ROI 只有原始裁图及旧分层统计。
- **enhanced**：scene 提供原图与编号图；ROI 提供原图选框、裁图、距离层像素归属图，以及每层的像素位置。
- 每个 case 的 `status` 和 `scene` 成功后固定，不重复获取；最多请求 4 次 ROI，150 秒后中止。ROI 可重新选择。用 Pi 原生工具过滤及 `beforeToolCall` 阻止同批重复请求，不添加模型重试。
- 三模型均 `tool_choice=auto`，每次请求输出上限 4096 tokens。Qwen 沿用 `enable_thinking=false`；ZenMux Gemini/DeepSeek 沿用 `reasoning_effort=low`。这些差异写入输入记录，不把不同模型的 thinking 开销当成同等计算预算。实验进程覆盖输出上限，日常 Qwen 配置不变。
- Qwen 复用实验室端点；Gemini/DeepSeek 复用现有 `zenmux.sh --script` 的直连、官方模型 ID 和本地凭证读取。runner 不记录凭证，没有模型替换或失败后重跑。

replay HTTP 接口：

```text
GET  /cases/{case_id}/status
GET  /cases/{case_id}/scene?presentation=baseline|enhanced
POST /cases/{case_id}/measure_region?presentation=baseline|enhanced
     {"frame_id":"...", "bbox_px":[x0,y0,x1,y1]}
```

每个 case 保存 `input.json`（完整提示、模型参数、工具定义）、`events.jsonl`（可见回复和工具事件）、`result.json`、`answer.txt` 及真实工具图片。日志只保留图片路径和标签，不保存 base64 或完整 thinking。`result.json` 包含逐次工具参数/结果、模型请求用量、墙钟耗时、首次可见文本时间和错误；token 总计直接累加 Pi 的 usage 字段，不估算未知价格。

`FINISHED` 只表示模型正常结束并输出了文本；它不代表选中了目标、距离解释正确或导航成功。统计是距离带中的像素集合，`p10_m`–`p90_m` 为中央 80% 像素分布，不是置信区间；未知深度不代表空旷。目标选区与几何真值由 Main 的离线对照另行核验。所有超时、工具失败和不完整回答均保留，不筛除后重跑。

## Monitor 暂停问答回放

`monitor-dialogue-replay.mjs` 对已存档的真实暂停问答执行一次实际 Qwen 调用，复用同一组图片、原问题和任务状态，仅补入生产 `controlEventFact`。没有仿真写入、导航或新拍摄，专门复核把后拉制动错解释成障碍检测的问题。

```bash
env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy \
  NO_PROXY='*' no_proxy='*' .local/envs/node-v22.23.2-linux-x64/bin/node \
  prototype/pi/experiments/monitor-dialogue-replay.mjs \
  --episode .local/outputs/task_monitor/human_pause_question_resume_03 \
  --output .local/outputs/task_monitor/new_dialogue_replay
```

输入、实际请求、模型回复写入新目录。单次正确回放不等于现场或跨问法交流可靠；完整原回合的不正确回答保留不变。
