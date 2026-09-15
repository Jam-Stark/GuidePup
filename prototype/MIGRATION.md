# 2026-09-15 导航框架迁移

本次交付代码仓库是 `https://github.com/Jam-Stark/GuidePup`，分支为 `codex/a2-lightnav-prototype`。
迁移版本标记为 `20260915_215130_HKT`。当前修正与证据边界先读 [NAVIGATION_FRAMEWORK.md](NAVIGATION_FRAMEWORK.md)。

Drive 目录：[20260915_215130_HKT](https://drive.google.com/drive/folders/1AA-_hWHKX6erT_-Iwxt4Iqh_ZUsMz2hr)。
目录中的 `worker_delivery__BUNDLE_INDEX.md` 和 `worker_delivery__BUNDLE_MANIFEST.json` 是附件清单；
实际上传状态以交付回执为准。

## 恢复代码和附件

新机器可选择任意工作目录。尚未有仓库时：

```bash
git clone --branch codex/a2-lightnav-prototype --single-branch git@github.com:Jam-Stark/GuidePup.git ForceControl-lightnav
cd ForceControl-lightnav
```

已有同仓库时，先处理当前未提交工作，再 `git fetch origin`、切换该分支、`git pull --ff-only origin codex/a2-lightnav-prototype`。
不要覆盖另一项任务的工作树。

下载以下普通 ZIP，全部以**仓库根目录**为解压目标，不额外嵌套文件夹：

| 附件 | 恢复位置和用途 |
| --- | --- |
| `worker_delivery__checkpoints_part01.zip` | `.local/assets/a2-old/`：实际使用的 `policy.pt`、`policy.json`、`a2.xml` 和 `meshes/`；`training-origin/` 保存原 `model_59999.pt` 及训练配置，仅作来源记录 |
| `worker_delivery__source_and_configs.zip` | `.local/vendor/LightNav-0/`：当前使用的 Python 源码和 MuJoCo helper；`.local/migration/20260915_215130_HKT/`：原环境版本、公开模型文件尺寸与来源 |
| `worker_delivery__logs_and_metrics.zip` | `.local/outputs/` 下保持原结构的 framework 证据、72 轮汇总、两个旧目标 case、少量原始帧和相关观察图 |

可以先把 `.local` 建成指向本机数据盘的符号链接，再解压。不要照搬旧机器的 `/mnt/ssd1/baoquanc/...` 路径。
代码统一从仓库根目录定位 `.local`。附件不包含 Python/Node 环境、缓存或 API 凭证。
本次上传按 Owner 要求走代理；软件/公开模型下载及运行请求沿用本任务的直连约定。

## 重建运行环境

目标环境是 Linux x86_64、Python 3.11、可用的 NVIDIA CUDA/EGL、`tmux` 和 `ffmpeg`。
原机器 PyTorch 为 2.10.0/CUDA 12.8，MuJoCo 3.6.0，LightNav transformers 5.8.0，YOLOE Ultralytics 8.4.152。
按新机器 GPU 和驱动实际支持情况确认 CUDA wheel；不要复制旧 GPU UUID 或 EGL 序号。

```bash
# 位于仓库根目录；附件已经解压。
prototype/sim/setup.sh
prototype/lightnav/setup.sh
prototype/pi/setup.sh
```

`sim/setup.sh` 创建独立 venv，并安装 MuJoCo、Torch、MPC、HTTP、YOLOE 和 CLIP 依赖。
只有显式传 `--source <旧仓库>` 才会执行旧资产导入；迁移附件已提供资产，不需要这个参数。
`lightnav/setup.sh` 使用已恢复的 vendor 源码，创建独立环境，再从公开 Hugging Face 模型库下载 LightNav。
`pi/setup.sh` 安装 Node 22.23.2、按 `package-lock.json` 安装 Pi 并构建 TypeScript。

公开模型不重复上传到 Drive。在仓库根目录运行以下命令准备 YOLOE 和文本编码器：

```bash
mkdir -p .local/models/perception
env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy NO_PROXY='*' no_proxy='*' \
  .local/envs/sim/bin/python - <<'PY'
import os
from pathlib import Path
from ultralytics import YOLOE

os.chdir(Path('.local/models/perception'))
model = YOLOE('yoloe-26l-seg.pt')
model.set_classes(['door', 'door frame', 'elevator door', 'wall'])
print('YOLOE and its text encoder are ready.')
PY
```

LightNav 来自 `LightOriginsHQ/LightNav-0`，主权重文件为 `model-00001-of-00001.safetensors`，原文件 9,695,744,034 字节。
YOLOE 主模型原文件 79,417,413 字节，`mobileclip2_b.ts` 原文件 253,794,476 字节。
完整公开文件尺寸/配置见附件中的模型清单；这不是内容一致性证明。若公开模型已有更新，记录差异再解释结果，不能声称严格复现原权重。

新机器须重新配置 Pi 和两个远程模型，完整配置、环境变量优先级及实际连接确认步骤见 [pi/SETUP.md](pi/SETUP.md)。
Qwen 使用 `http://vic.precognition.team:8888/v1`，模型 ID 必须保留末尾斜杠：`Qwen3.6-35B-A3B-FP8/`。
不要在新机器下载本地 Qwen。Gemini 使用 `prototype/pi/zenmux.sh gemini`，配置为
`google/gemini-3.8-flash`、`reasoning_effort=low`、输出上限 4096；Qwen thinking off、上限 2048。
Gemini 需要在新机器重新设置 `ZENMUX_API_KEY`，或私下配置 `.local/secrets/zenmux.env`。
该凭证未提交 Git、未上传 Drive。缺失时先完成 Qwen 和环境准备，再请 Owner 在新机器安全配置；不要把密钥写入日志或迁移文件。

## 继续未能运行的测试

之前 GPU3、随后 GPU0 均能加载 LightNav CUDA，但 MuJoCo 启动卡在 `nvkms_open_common`，120 秒超时，
**修正后没有开始任何新的完整导航回合**。GPU1 掉卡是线索，尚未证明因果。
先在新机器选择可用物理 GPU，通过 `LIGHTNAV_GPU` 显式传给服务；launcher 会按该 GPU UUID 匹配 EGL。

```bash
export LIGHTNAV_GPU=0  # 按新机器实际可用 GPU 修改。
prototype/run.sh build
.local/envs/sim/bin/python prototype/sim/run_framework_reproductions.py \
  --output ".local/outputs/navigation_framework/migration_$(date +%Y%m%d_%H%M%S)"
```

runner 启动服务并顺序执行两个已有声明的真实 `room_elevator` 回合，每回合任务 360 秒、单次决策 30 秒、最多 3 次恢复，
均保留 −80 N 后拉、制动、暂停问答、人工恢复协议与原始导航输入记录：

| 模型与原 case | 初始 `(x, y, yaw_deg)` |
| --- | --- |
| Qwen `p05_force_resume` | `(2.7662063165748876, -7.173436056562866, 164.29460352541886)` |
| Gemini `t01_force_resume` | `(2.548203902100475, -6.900806258145334, 80.58823003115508)` |

每个回合自动执行 `evaluate_monitor.py`、`evaluate_navigation_v2.py`，汇总写入新输出目录的 `results.json`。
可用 `--model qwen` 或 `--model gemini` 分开执行，每次必须用新的输出目录。
长安装/运行需要跨会话持续时使用独立 tmux，记录实际 ETA 后等待结果；不反复重启或轮询。

核验重点是当前上下文是否允许缺少目标时搜索、重叠 mask 是否被错误当成两个物理门框、满足几何/停止条件时 Monitor 是否正确自动完成。
保留 YOLO 词表为任务配置，框架判断保持通用；不要添加特定目标外观模板、放宽门洞/碰撞条件或把场景真值给模型。
分别报告 Monitor 状态、真实几何和接触结果、候梯是否正确、暂停/恢复/问答、运行错误及 `NEEDS_USER` 原因。
框架回放的三次真实模型调用、记录数据的路径证明和旧 72 轮结果属于此前证据，不能计为新完整任务成功。

若服务启动再次失败，保存两项服务日志和实际进程/GPU状态，停止本任务服务并报告；不要重复同样启动，也不要擅自重置驱动。
完成两回合后先报告结果；本次不自动扩成新 36/72 轮，不训练、不操作硬件。
最后执行 `prototype/run.sh down`，确认本任务服务已结束，并更新导航 memory 与结果文档。
