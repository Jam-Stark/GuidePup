# ZenMux 接入与实跑记录

2026-09-14 HKT。两个指定模型均已通过现有 Pi → LightNav → MPC → 旧 A2 policy
完成一次语义导航运行。两次均由模型报告 COMPLETED，离线 base 轨迹证实穿过门洞；
这不是无碰撞、精确距离或跨场景可靠性的验证。

## 接入

- Base URL：`https://zenmux.ai/api/v1`，OpenAI Chat Completions 协议。
- 精确 ID：`google/gemini-3.8-flash`、`deepseek/deepseek-v4.1-flash`。
- 两者实际完成 RGB、流式文字、工具调用及工具结果回传；服务目录也报告支持 image。
- 两者均为 `reasoning_effort: low`，每次回复预算4096 token，没有自动换模型。
- Gemini 自动任务使用 `tool_choice: required`；DeepSeek 使用 `auto`，见实际故障。
- key 位于未跟踪的 `.local/secrets/zenmux.env`，权限0600；不进入报告或代码。
- 本机系统 DNS 返回不可达地址。启动器每次通过 HTTPS DNS 解析 ZenMux，只改当前
  Node 进程对此域名的解析；保持域名/TLS校验，不经过代理，不固定IP，不修改系统DNS。

接入依据为 [ZenMux 快速开始](https://zenmux.ai/docs/guide/quickstart)、
[工具调用](https://zenmux.ai/docs/guide/advanced/tool-calls.html)、
[多模态](https://zenmux.ai/docs/guide/advanced/multimodal.html) 和
[推理配置](https://zenmux.ai/docs/guide/advanced/reasoning.html)。
服务目录保存在 `.local/outputs/zenmux_models.json`；模型页面分别为
[Gemini](https://zenmux.ai/google/gemini-3.8-flash) 和
[DeepSeek](https://zenmux.ai/deepseek/deepseek-v4.1-flash)。

## 条件与结果

同一个 ProcTHOR 场景、旧 A2 6D 导出、头部相机 `a2_head_isaaclab`（768×272，
Owner 的仿真视野/安装参数）。LightNav 仅收 RGB；Pi 收到 RGB、相机配置和机器人
状态，没有深度、目标坐标或真实剩余距离。机器人位姿仍由仿真提供，作为未来里程计
的替代。没有注入外力，没有修改模型提示词或控制算法。

只读调用并行，运动任务串行且每项前重置仿真。每种配置各一次，语义导航最多120秒、
3次恢复，数值任务最多30秒。调用前记录了
[计划](../../.local/outputs/zenmux_comparison/plan.json)，后来只追加了一项协议修正。

| 实际运行 | Gemini | DeepSeek |
| --- | --- | --- |
| 只读 status + scene | 成功，整轮6.97秒，首文字5.18秒 | 成功，整轮7.48秒，首文字4.65秒 |
| 只读距离判断 | 明确不能确认约1米 | 明确不能确认约1米，并提出用户选择 |
| “距离白色置物架前表面约1米处停下” | 5.76秒后 pause / NEEDS_USER；无 navigate 或 complete | 修正协议后7.84秒 pause / NEEDS_USER；无 navigate 或 complete |
| “穿过右前方的敞开门，走到门内白色置物架前，然后停下” | 41.35秒，COMPLETED | 46.31秒，COMPLETED |
| 语义任务工具 | status×2、scene×2、navigate×1、complete×1 | 同左 |
| 语义任务局部恢复 | 0次 | 0次 |
| 离线门平面交点 | x=11.033m，y=11.222m，在门洞内 | x=11.033m，y=11.151m，在门洞内 |
| 终点 base [x,y,yaw] | [11.942,10.634,−0.437] | [11.774,10.585,−0.111] |

导航耗时是墙钟时间，包含模型调用和物理执行，不是模型延迟。只读整轮包含工具执行
与后续模型请求。重置相同场景后仍有站立漂移、不同生成指令和轨迹；不能从一次运行
的耗时差异推导模型排名或成功率。

## 实际故障与判定边界

**DeepSeek 初次自动任务 HTTP 400。** low thinking + required 被拒绝：
`Thinking mode does not support this tool_choice`。错误发生在工具执行之前，保留于
[deepseek_metric](../../.local/outputs/zenmux_comparison/deepseek_metric/summary.json)。
只对该精确模型改为 auto，保留 low thinking；重跑数值任务和之后的语义任务均调用
成功。没有自动重试或模型替换。DeepSeek 官方
[Oh My Pi 集成文档](https://api-docs.deepseek.com/quick_start/agent_integrations/oh_my_pi/)
记录了 V4 thinking 对 tool_choice 的限制；本次 V4.1 的限制由真实400确认。

**数值任务暂停不等于测距成功。** 两者均没有把数值目标标记完成。DeepSeek 在暂停前
解释缺少测距，并要求用户决定是否接受纯视觉条件；Gemini 只调用 pause，任务循环
随后终止，仅显示通用“需要人工补充信息”消息。不能将静态问答中的解释当作自动
任务已向用户说明原因。两者可靠拒绝的能力均尚未经过重复或跨场景验证。

**两者完成理由都包含不充分的推断。** 它们结合前后图像判断进入房间，但还把
“base总位移远大于相机前伸0.34m”作为已过门的证明。没有门位置或轨迹关系，
总位移不能证明穿门。本次过门由另行记录的轨迹与场景门平面交点证实，场景几何
没有提供给模型。

终点图像均显示接近白色置物架，但没有校准距离。此次未启用5ms接触记录，不能报告
无碰撞。更换高层模型没有补上已有 MPC 缺少机器人尺寸/障碍约束的问题。
两次都没有触发局部恢复，故不能宣称新模型的错误恢复已验证。force brake 与人工
恢复权限沿用已有实现，本次没有重做施力实验。

## 产物与复现

- [精简证据与可见模型回复](../../.local/outputs/zenmux_comparison/evidence.json)
- [Gemini 视频](../../.local/outputs/zenmux_comparison/gemini_navigation/camera.mp4)、[终点图](../../.local/outputs/zenmux_comparison/gemini_navigation/final.jpg)
- [DeepSeek 视频](../../.local/outputs/zenmux_comparison/deepseek_navigation/camera.mp4)、[终点图](../../.local/outputs/zenmux_comparison/deepseek_navigation/final.jpg)
- 各运行目录有 agent.jsonl、console.log、samples.jsonl、summary.json 和相机配置。
- 记录器 `.local/outputs/run_zenmux_demo.py`；汇总脚本 `.local/outputs/zenmux_comparison/summarize.py`。

```bash
cd /home/baoquanc/workspace/ForceControl-lightnav
prototype/run.sh up
prototype/pi/zenmux.sh gemini chat
# 或退出该会话后：
prototype/pi/zenmux.sh deepseek chat
```

TypeScript 编译通过。默认 `prototype/run.sh chat/run` 继续使用实验室 Qwen；
ZenMux 用独立入口选择模型，未改 checkpoint，未操作硬件，未提交或推送 Git。
