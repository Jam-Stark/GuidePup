# 新机器迁移进度：Qwen 回合尚未启动

本页保留首次准备阶段的阻塞记录。后续已完成环境恢复和一个新的 Qwen 回合，当前结果见 [2026-09-16 运行报告](NAVIGATION_MIGRATION_20260916.md)。

2026-09-15 HKT。迁移版本 `20260915_215130_HKT`，分支 `codex/a2-lightnav-prototype`，新工作目录 `/home/baoquanc/workspace/GuidePup`。Owner 后续限定使用物理 GPU0，并暂缓 Gemini；本次剩余目标仅为 Qwen `p05_force_resume`。

## 已完成的恢复

- 在新目录克隆指定分支，没有覆盖已有未提交工作。
- 从指定 Drive 文件夹下载 BUNDLE_INDEX、BUNDLE_MANIFEST、UPLOAD_RECEIPT 和三个 ZIP。三个 ZIP 分别为 35,804,940、198,148、7,749,432 字节，共 447 个文件，解压到仓库根目录并保留 `.local/`。A2 旧 policy、资产、LightNav vendor 源码和历史证据已恢复；历史记录不计为新实验。
- `.local` 使用本机根 NVMe 文件系统。原始下载在 `/home/baoquanc/workspace/guidepup_migration_20260915_215130_HKT`，清单与回执也保存在 `.local/migration/20260915_215130_HKT/`。
- 使用本机 Python 3.11.7 建立独立 `sim`、`lightnav` venv；依赖尚未安装完成。FFmpeg 7.0.2 通过 `.local/bin/ffmpeg` 使用本机已有 imageio 附带二进制。
- 按 Ultralytics 8.4.152 源码中的实际下载地址，直连官方 GitHub `ultralytics/assets` 的 `v8.4.0` release，下载 `yoloe-26l-seg.pt` 和 `mobileclip2_b.ts`。大小分别为 79,417,413 和 253,794,476 字节，与迁移清单相同；尚未执行模型加载和任务词表初始化，文件大小相同不证明内容或运行一致。
- 本机 `configure_gpu()` 已返回 GPU0 UUID `GPU-f593e489-014b-eed5-4331-b01447615b6e`、EGL 索引 2，CUDA 环境绑定相同 UUID。这是设备选择证据；MuJoCo renderer 尚未启动，不能断言旧 `nvkms_open_common` 阻塞已解除。
- `prototype/pi/.env` 已按 SETUP.md 配置默认 Qwen：`http://vic.precognition.team:8888/v1`，精确 ID `Qwen3.6-35B-A3B-FP8/`，无鉴权，2048/off。直连 `/v1/models` 返回 200 并列出该 ID；实际 Pi `configuredModel()` / `models.complete()` 连接验证尚未执行。

## 当前阻塞与真实结果

本机对 `nodejs.org`、`download.pytorch.org`、`pypi.org`、`registry.npmjs.org`、`huggingface.co` 的官方直连请求发生 TCP 443 超时；`1.1.1.1` DoH 也不通。GitHub、Google Drive 和实验室 Qwen 可达。没有更换代理、镜像、锁定版本、provider 或 renderer。

Node 22.23.2 没有安装，本机指定缓存位置没有可用归档；系统 Node 18 没有被用于替代。Python 安装停在 PyTorch 官方源，LightNav 公开权重尚未下载。两个安装 supervisor 已在确认网络错误后取消，保留原日志，不反复重启。

| 项目 | 本次结果 |
| --- | --- |
| Qwen `p05_force_resume` | **NOT_RUN**：依赖恢复受阻，尚未启动仿真或任务 |
| Gemini `t01_force_resume` | Owner 后续要求暂缓，不属于当前执行范围 |
| 新真实成功 | 0；新导航回合分母也是 0 |
| 任务 `NEEDS_USER` | 无任务运行，不能赋予导航求助原因 |
| 导航运行错误 | 未执行，无法评价；已发生的是环境准备网络错误 |
| 搜索上下文、mask/物理边界、Monitor 自动完成 | 尚无新闭环证据 |
| 真实几何、接触、−80N 制动、暂停问答、人工恢复 | 尚无本次执行数据 |
| 两项 evaluator / runner `results.json` | 未运行／未生成；未用准备状态文件冒充结果 |

## 证据与继续入口

- [准备状态](../.local/outputs/navigation_framework/new_machine_restore_20260915/preparation_status.json)
- [附件恢复记录](../.local/outputs/navigation_framework/new_machine_restore_20260915/restoration.json)
- [本机 GPU UUID 匹配](../.local/outputs/navigation_framework/new_machine_restore_20260915/gpu_selection.json)
- [直连网络错误](../.local/outputs/navigation_framework/new_machine_restore_20260915/direct_network.json)、[sim 安装日志](../.local/outputs/navigation_framework/new_machine_restore_20260915/sim_install.log)、[LightNav 安装日志](../.local/outputs/navigation_framework/new_machine_restore_20260915/lightnav_install.log)
- [公开感知权重下载](../.local/outputs/navigation_framework/new_machine_restore_20260915/perception_downloads.json)
- [Qwen 配置及 Pi 未运行原因](../.local/outputs/navigation_framework/new_machine_pi_20260915/summary.json)

恢复上述官方源直连后，从仓库根目录 source `.local/migration/20260915_215130_HKT/new_machine_environment.sh`，继续三个 setup 入口、实际 Pi 短连接验证和 MIGRATION.md 的 YOLO 任务词表初始化。随后仅运行：

```bash
.local/envs/sim/bin/python prototype/sim/run_framework_reproductions.py \
  --model qwen --output <新的输出目录>
```

保持原 360 秒任务、30 秒决策、最多 3 次恢复和 −80N/暂停问答/人工恢复协议，runner 自动执行原两项 evaluator。继续使用 GPU0，并为安装和回合使用新的日志与独立 tmux；不重复当前网络失败尝试。代理下载需要 Owner 对“直连入口”要求的明确例外。

本次未修改导航框架、YOLO 任务词表或验收条件；未添加测试、训练、操作硬件、commit 或 push。仿真与 LightNav 服务均未启动，安装和下载会话已结束。GPU0 的渲染与完整 Qwen 回合仍待实际执行。
