# 学校 MF-R1 场景接入

2026-10-15。按Owner要求，GuidePup默认Isaac测试场景已替换为所提供的学校USD；默认任务为“一层起点走到中间2号电梯门前，朝门停下”。GPU7原生加载、A2、RTX画面、RGB-D、传感器里程计和局部地图已实际接通。本次只做加载验证，完整导航任务数0；没有新增导航成功或跨层验证结论。

## 场景与源码

- 原始附件：`多楼层场景包_MF-R1.zip`，126,907,761字节，89个文件，解压内容130,550,309字节。
- 原包原样保留于`.local/assets/campus_mf_r1/多楼层场景包_MF-R1/`，包括一层/三层USD、MDL和贴图、附带电梯控制器/说明/历史报告。附件中的操作说明作为参考读取，未直接执行其启动/验证脚本。
- 实际源入口：`多楼层场景.usd`；默认prim `/World`，单位米，Z向上。USD依赖16层/39资产，未解析依赖0；源碰撞API1602个。
- 新准备入口：`prototype/isaac/prepare_campus.py`，使用既有`asset_python.sh`内的OpenUSD；生成独立引用层`.local/assets/campus_mf_r1/integration/environment.usda`。不缩放/平移/重建几何，不改材质或碰撞，只在引用层停用导入的`/World/Physics`，使现有A2 SimulationContext成为唯一物理循环所有者。
- 新默认配置：`prototype/isaac/scenarios/campus_mf_r1_elevator.json`。起点沿用包内Start1F的XY=[0,2]、A2初始高度0.55m，初始yaw−20°；该朝向用于放置，不下传真实位姿给在线模块。
- `run_experiment.py`默认选择校园配置。新增`--preview-only`，实际执行现有plant→RGB-D→估计→局部地图，然后退出；不启动导航模型或Pi，不创建任务。
- 当前其他项目占用8766，故启动器使用独立导航端口8768，并将同一`SIM_URL`传给记录器和Pi。没有终止或修改原8766服务。

一层到三层Z差11.10m是模型堆叠摆放值，不是实测层高。仅2号电梯具有附带运行控制器；原控制器、运行参数和导航拓扑保留在原包。本次第一层“走到门前”任务不调用电梯控制器，门保持原始关闭状态，不进行升降/绑定/传送机器人。分层定位、自动进出梯和跨层任务调度未实施。

## GPU7实际加载验证

证据根：`.local/outputs/campus_mf_r1_integration_20261015_01/`。

实际命令：

```bash
LIGHTNAV_GPU=7 ISAAC_RENDERER_GPU=7 .local/envs/sim/bin/python \
  prototype/isaac/run_experiment.py --preview-only --navigation-port 8768 \
  --output .local/outputs/campus_mf_r1_integration_20261015_01/preview
```

独立tmux `guidepup-campus-mf-r1-20261015`，实际48.12秒完成，退出0。CPU资产准备未分配GPU。

| 项目 | 实际结果 |
|---|---|
| CUDA物理GPU | 7，UUID `7c8cb1d2-4ebf-e2e3-35ad-fa0f6f72924d` |
| Kit Vulkan renderer | active GPU7、同UUID/PCI表对应，非多GPU |
| 最终加载状态 | PREVIEW_COMPLETE；控制状态PAUSED，曝光1.82仿真秒 |
| 里程计 | tracking，source=depth_imu_proprioception，depth rank3 |
| 原生深度 | 848×496，有效率92.110% |
| RGB对应深度 | 有效率85.242% |
| 局部地图 | 已建立，36占用体素，map capture1.62s |
| 资源/材质/运行Error | physics/navigation日志未发现Error或未解析依赖 |
| 完整导航任务/导航模型调用 | 0/0，未启动Pi/Qwen |

实际D435i布局和完整原生深度配置沿用当前实现。原始传感器/物理/接触与导航状态日志在`.local/outputs/isaac_sim_20261015_163025/`；画面为`preview/preview.jpg`，配置和状态为`preview_status.json`及原始`configuration.json`。

日志包含已有headless显示初始化提示、非选中CUDA设备跳过提示以及shader缓存接近上限提示；GPU7 RTX实际成功渲染，未重置驱动、清理共享缓存或使用renderer fallback。

## 默认任务的独立评分

新增电梯接近评分分支，房间穿越评分路径保留。目标平面取源Landing1L/R的朝大厅一侧真实几何表面：Y=−3.722500005m，中心X=12.51m，接近侧法向+Y。它区别于门的刚体中心Y=−3.740m。

采用既有`navigation-contract.json`等待约定：机身包络至门平面的有符号净距0.6–0.9m、朝向误差≤25°、横向偏差≤0.65m，以及原0.3秒停稳窗口。未修改在线语义Monitor、Qwen完成权限、控制器或阈值。

只做一个解析几何检查：朝门的三圆包络前缘距门平面0.75m时评分通过，数值误差小于1e−9；见`target_scoring_check.json`。这是计算接线验证，不是机器人已到门口的运行证据。真实目标坐标、场景拓扑和原包航点仅用于放置/离线评分，没有进入在线地图、模型或动作选择。

本次预览使用启动器原默认footprint；因为没有导航，不代表运动可行性已验证。保留上一轮独立10cm footprint副本于本次证据根，正式导航可沿用该配置而不覆盖默认footprint。具体任务命令见`prototype/isaac/README.md`。

## 收尾

新的场景配置和入口、预览图及说明已保存。GPU7验证前空闲，验证后约3MiB/0%；本任务physics/navigation/native bridge与tmux已退出，8768关闭。原8766服务PID292116继续存活；GPU0/6上的其他项目进程未操作。没有模型下载、训练、硬件操作或Git提交。

附带历史报告的8kg方块乘梯结果不是本项目A2乘梯验证。本次未运行完整导航、未验证A2跨20mm门槛或乘梯能力；后续结论必须来自相应的新运行证据。
