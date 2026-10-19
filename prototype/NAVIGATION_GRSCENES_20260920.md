# GRScenes 医院跨房间导航：2026-09-20

## 结果

**全量下载完成；唯一实际创建的完整导航任务真实成功 0/1。** GPU7 / Qwen 宣布 `COMPLETED`，但独立物理轨迹显示机器人没有走出源房间，未进入目标房间，最终停稳。没有追加实验。

| 项目 | 本轮证据 |
|---|---|
| 正式回合 | `.local/outputs/grscenes_room_to_room_20260920_01/navigation_04/` |
| 原始记录 | `.local/outputs/isaac_sim_20260920_180412/` |
| 任务时长 | 墙钟175.265秒；物理窗口1.90–23.26秒；无全任务时间限制 |
| 实际路径长度 | 1.212027米，初末平移约0.787米 |
| 源门完整穿越 / 目标房间进入 | 均未达成 |
| 最终源门前缘距离 | 仍在门前0.656242米 |
| 最终停稳 | 是，最后0.3秒最大速度0.00546m/s、最大yaw速度0.00483rad/s |
| Qwen完成声明 | 有；独立评分为误报 |
| NEEDS_USER / 恢复额度终止 | 均无 |
| 正式任务运行错误 | 无；受控局部受阻1次，随后恢复执行 |

`physical_evaluation.json`、`navigation_evidence.json`、`results.json` 已生成。旧回合、离线预测及初始化失败不计新导航成功。

## 下载与场景接入

数据源：[InternRobotics/GRScenes](https://huggingface.co/datasets/InternRobotics/GRScenes)，保留原README和CC-BY-NC-SA4.0归属。**236个文件，76,615,735,832字节全部完成**，存于`.local/assets/GRScenes/`；商业场景压缩包已解压，其余源文件/分卷保留。

服务器直连原Hugging Face端点失败，使用直连`hf-mirror.com`镜像；下载进程移除大小写HTTP/HTTPS/ALL代理，NO_PROXY为`*`，aria2显式空代理。没有改动Codex专用代理。下载完成记录见`download/status.json`，无内容哈希。

选择官方商业导航场景`MWF4WLIKTIFZIAABAAAAABY8_usd/start_result_navigation.usd`，实际RTX画面为医院病房/走廊，保留人物、家具及官方导航资产门状态。源出口约1.2m宽、走廊约2m宽、目标入口约1.8m宽，目标房间约6×8m。建筑空间较宽，但并非空房：门口存在原始静态人物。

任务：**走出当前房间后向右沿走廊走，进入走廊对侧第一间宽门洞敞开的房间，直到整个机身进入后停下。**

- `prepare_grscenes.py`生成单独USD引用包装层：经几何核实的厘米单位缩放0.01、整体Z轴转90°；原stage未显式声明单位。保留全部原生碰撞几何，仅停用导入资产自己的PhysicsScene，由现有A2 SimulationContext管理。
- 官方脚本和[v2.1.0发布说明](https://github.com/InternRobotics/InternUtopia/releases/tag/v2.1.0)要求关闭UJITSO collision cooking；场景显式设置，其他场景不改变。
- 实际商业ZIP根目录为`commercial_scenes`。共享Materials/模型引用保持；CPU工具使用已安装Isaac OpenUSD，不另装USD。
- 独立评分使用真实房间多边形，排除两个结构柱及隔壁独立房间；保持原机身圆包络、顺序通门和停稳标准。早期只看wall分类得到的“东侧相通附间”推测已由完整Base/other墙体纠正，最终以`research/geometry/whole_room.json`为准。
- 场景几何仅用于初始化和离线评分；在线仍只有RGB-D、IMU、关节及估计状态。

GPU7 CUDA UUID与Kit/Vulkan渲染表核对一致。保留默认相机、完整原生深度及RGB标定、独立10cm期望净距footprint、原速度自由度/停车模型、Qwen实验室配置、30秒决策及最多3次恢复。无居中前缀、外力、人工暂停/恢复、Gemini、训练或硬件。

## 初始化错误与已完成的最小修正

1. `navigation/`：旧600秒物理加载上限超时，未创建任务。物理资产初始化改为等待完成/显式进程错误；未改renderer或物理参数。
2. `navigation_02/`：为补齐官方UJITSO设置而中止的初始化，未创建任务。首次调用堆栈位于`sim.reset→play→app.update`，不能单凭此断言物理死锁。
3. `navigation_03/`：加载成功，`sim.reset`耗时2034.9秒。等待全量下载时，预览后的服务器仍推进零指令步态，从约2秒运行到159秒；状态估计漂移后先报`RGB-D has no measured floor support`。之后任务接管409，task_id为空，Qwen/LightNav导航帧/MPPI均为0。记录器4.762秒不是完整导航。常规评估因不存在monitor_state.json显式失败，错误保留，未伪造评估结果。
4. 修正预览生命周期：启动器传入`--setup-release-file`，地图/传感器就绪后**暂停仿真时钟**，预览确认及模型就绪后才释放。没有改变在线地面判据、估计器或控制器，没有假帧或真值纠偏。
5. `navigation_04/`：实际验证`SETUP_HELD`和`SETUP_RELEASED`均为1.74秒，随后创建唯一正式任务。初始化2341.4秒与导航任务时长分开记录；不能宣称UJITSO选项单独解决了耗时。

原资产存在人物`eyes03A.jpg`缺件；原远程Ground_Leaves纹理还有解码/上传错误。没有替换纹理或启用渲染fallback。病房门口/地面画面和深度可用，但这些资产完整性缺陷限制视觉实验的解释，不能声称无渲染错误的基准。

## LightNav、MPPI与Qwen

实际发送LightNav的任务文本：

> Exit the room, turn right, and enter the first wide open doorway on the opposite side of the corridor, then stop inside.

- LightNav54次返回、52次接受，单会话仅seq0重置；seq30转向期间的STOP被丢弃，不计目标到达。唯一接受STOP为seq53，输入时刻21.30秒，收到时刻21.52秒。接受输出的visible全部false。
- STOP时真实位姿约`[-0.2532,9.6938,-0.1112]`，机身前缘仍距源门0.6377米，尚未出源房间。没有证据把STOP解释为真实到达。
- Qwen5次调用，期间发生1次局部受阻恢复；最后根据RGB和约0.93米估计位移声称已经穿过走廊进入目标房间。只读参考当时仍为`unverified`、通门`not_observed`、身份`not_reviewed`。Qwen错误解读了当前场景；这不是Monitor完成认证。
- 原生MPPI86次优化：85成功、1受阻；控制记录85条，84实际下发（67 LightNav参考、17显式转向参考）。额外成功求解不等于已执行。
- 唯一受阻发生于9.4秒：当前几何净距约13.82cm、承诺前缀末端约11.21cm、经验停车净距约3.64cm，原生实际评估512候选，全部因停车包络拒绝，`no_valid_control`。不是Python统一前置否决、当前机身几何重叠或优化/接口异常。
- 84条下发命令逐条与原生接受的第一条控制匹配；所有成功响应与最终检查一致。恢复后有真实命令执行，随后LightNav再次导航至STOP；不能把本轮归为“一次MPC拒绝后始终不动”。左转25°恢复在checkpoint实际转25.507°，17条转向命令、161个非零tick；净平移3.80cm。详情见`analysis/navigation_04_QWEN_FLOW.md`。
- 本轮原生全部normal模式，未覆盖初始停车包络不足的restoration分支。最小已执行轨迹几何净距约10.49cm、经验停车净距约3.52cm，均属估计地图/经验模型值，并非真实网格净距。

## 物理与接触限制

4273条5ms物理样本；非足部净法向力最大0N，未见该observer记录的非足接触。它不能提供接触对端、接触点或摩擦，包含自接触且相反法向可能抵消；**不能据此证明全程无碰撞**。没有计算全场景真实网格最小净距。定位离线XY RMSE约9.76cm、末端误差17.11cm，不能直接归因全部语义误判。

## 证据与收尾

证据根目录：`.local/outputs/grscenes_room_to_room_20260920_01/`。

- `navigation_04/{results,physical_evaluation,navigation_evidence}.json`
- `navigation_04/episode/camera.mp4`、initial/final图、agent.jsonl与模型观测
- raw目录中的RGB-D、LightNav完整输入输出、local_controller.jsonl、nav2_mppi.jsonl、ground_truth.jsonl、contacts.jsonl
- `analysis/{controller_flow,stop_physical}.json`、`physical_route.png`、QWEN_FLOW报告
- `download/status.json`、`research/startup_notes.md`、`resources_final.json`

所有本任务服务、端口和tmux已结束，GPU7已释放，未终止其他项目进程。Kit退出未及时结束，由现有清理逻辑关闭其自有进程；与任务内运行错误分开。完整任务仅此一轮，不追加调参/训练/导航。
