# 电梯大厅接入与首轮运行

2026-09-14 HKT。已新增原生 MuJoCo `elevator_lobby` 并实际运行“**找到电梯，走到电梯门口并停下。**”本回合正确走向金属电梯双门，在一次追加接近后停下；接触记录为零非支撑接触。Qwen 的最终帧号和测量引用仍有错误，不能把此次物理完成扩大成可靠的自主验收能力。

## 已配置的场景

大厅约10×8 m、高3 m，中间保留至少2.5 m宽的开阔路线。电梯是静态关闭的双扇金属门，宽1.7 m、高2.3 m，配有楼层显示和呼梯按钮；同一面墙上放普通木门作为干扰，侧边有长凳和盆栽。起点配置为base `[2.8,-1.6,0.55]` m、yaw=10°，距电梯门中心约7.76 m。起点相机能同时看到电梯和普通房门。

![首轮实际起点RGB](../.local/outputs/elevator_scene/elevator_01/initial.jpg)

这是项目自建的MJCF场景，**没有把它称为Open-RMF Hotel World转换结果**。[官方Hotel World](https://github.com/open-rmf/rmf_demos#hotel-world)是包含大厅、两层客房和两部电梯的Gazebo示例；其[楼层YAML](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_maps/maps/hotel/hotel.building.yaml)通过[生成器](https://github.com/open-rmf/rmf_traffic_editor/blob/main/rmf_building_map_tools/building_map/lift.py)创建建筑和电梯。当前方案只参考官方1.7 m门宽，没有复制其家具或纹理。[资源核查](../.local/outputs/elevator_scene/resource_research/RESOURCE.md)保存了真实路径和转换工作量判断。

新增[场景XML](sim/scenes/elevator_lobby.xml)、A2场景装配分支、`--scene elevator_lobby`选项和[任务录像入口](sim/record_task.py)。旧A2 `model_59999.pt`的6D导出、D435i名义640×360 RGB-D、LightNav、MPC参数、Qwen/Pi提示与工具、force brake权限均保持本轮运行前的实现。未启动Gazebo、Isaac Lab或实机。

## 一次真实任务的结果

运行前登记了一次主回合 `elevator_01`，180墙钟秒上限、最多3次恢复，无外力。实际用时**40.12墙钟秒**，仿真窗口0.92–37.04 s，没有重新运行以挑选更好的结果。场景预览阶段只调整了地砖尺寸和室内照明；没有使用模型答案挑选场景。

| 核对项 | 实际结果 |
|---|---|
| 目标选择 | 初始ROI包含金属电梯门，排除了右侧普通木门；后续实际轨迹走向电梯 |
| LightNav指令 | `Approach the elevator doors and stop.`；第一次候选后追加 `Approach closer to the elevator doors and stop.` |
| 实际行进 | 20 ms轨迹累计6.574 m，净位移6.422 m；追加接近约0.675 m |
| 终点base | `[8.640,1.072,yaw=0.162]`；横向位于两侧电梯门框之间 |
| 实际停止 | 命令归零；模型最后读取状态的平面速度约0.0033 m/s，录像终态采样约0.0148 m/s |
| 物理接触 | 5 ms日志中全部非支撑contact为0，非地面正法向接触为0；正常足底支撑单独排除 |
| MPC | 256次求解均为 `Solve_Succeeded`，没有 `mpc_blocked` |
| 终态目标距离 | 独立终帧可见门板距base的水平距离最小1.229 m、中位1.264 m；门叶线段最近几何距离1.230 m |
| Pi结果 | `COMPLETED`，1次恢复；4次ROI请求中2次成功、2次帧引用错误 |

上述距离作为观察结果报告，没有给用户原句增加“必须1米”或其它默认数值条件。起点目标已在视野内，这次覆盖了可见目标识别、接近和停车，没有覆盖电梯不在视野内时的全局搜索。门保持关闭，没有进入电梯或跨楼层导航。

[实际视频](../.local/outputs/elevator_scene/elevator_01/camera.mp4)、[运行summary](../.local/outputs/elevator_scene/elevator_01/summary.json)、[物理报告](../.local/outputs/elevator_scene/analysis/elevator_01/PHYSICAL.md)、[模型语义核验](../.local/outputs/elevator_scene/semantic/elevator_01/REVIEW.md)。

![实际俯视轨迹与两次候选到达](../.local/outputs/elevator_scene/analysis/elevator_01/trajectory_topdown.png)

## Qwen实际做对和做错的部分

第一次候选到达时，Qwen使用SSE中的帧尾号267去量，而scene实际返回283，得到409。它随后改用283成功重测；ROI几乎全为金属门板，距离带中位数分别1.966/2.123 m。随后提交的closer指令真实执行，使机器人继续靠近约0.675 m。这次有实际局部纠正，不是只改写一段说明。

这次重测与navigate仍在同一批消息中预先生成；模型已有scene图像和概览，但没有等新的ROI返回再决定导航，不能宣称这条动作是读取重测结果后作出的决定。

第二次候选时，再次把SSE帧336与scene350混用，ROI请求失败。这次Qwen没有重新测量就完成，并声称“根据frame351测量，中位数1.269米”。**1.269米确有数据来源**：它实际读过的frame350网格概览中，门板对应的cell3/9/15的D2中位数就是1.269米。351来自随后status里的拍摄帧号；没有350/351的成功ROI测量。因此错误在于帧与测量类型的引用链，不应写成数值完全凭空生成。

最终图像和实际低速支持接近与停车，但“紧贴电梯门”的表述夸大；这与约1.23 m的base距离不是一回事。已知的scene返回值依赖问题本轮未修改，两个409完整保留。模型与工具的后续改动仍应解决先读scene、再生成依赖该帧的测量，而不是扩大缓存或把完成声明当证据。

## 独立终点参考

录像脚本在任务结束后另外归档了一帧，只供离线分析，**不是Qwen看过的画面**。本次为 `2190203243-371`、sim37.00 s。根据同一时刻的qpos/qvel离线forward并渲染，RGB与光轴深度逐像素完全一致；可见几何分割只选两块电梯门叶，排除门框、按钮、普通木门、墙和地板。214,952个有效门板像素独立反投影后得到前述距离。

![终帧门板参考，青色仅用于离线核对](../.local/outputs/elevator_scene/reference/elevator_01/door_overlay.png)

[参考数据](../.local/outputs/elevator_scene/reference/elevator_01/reference.json)、[终点俯视图](../.local/outputs/elevator_scene/reference/elevator_01/overview.png)。场景坐标、目标标签和上述参考均没有传给LightNav、Qwen或MPC。当前仍使用理想MuJoCo深度，这不证明实机D435i测距或跨场景可靠性。

## 再次运行

当前服务已重置到电梯大厅起点、IDLE，保留实验室Qwen。正常运行：

```bash
cd /home/baoquanc/workspace/ForceControl-lightnav
prototype/run.sh reset --scene elevator_lobby
prototype/run.sh run "找到电梯，走到电梯门口并停下。" --timeout-s 180 --max-recoveries 3
```

一次命令重置、执行并录像：

```bash
.local/envs/sim/bin/python prototype/sim/record_task.py \
  --scene elevator_lobby --goal "找到电梯，走到电梯门口并停下。" \
  --output .local/outputs/elevator_scene/elevator_02 \
  --timeout-s 180 --max-recoveries 3
```

使用新的输出目录。首轮的[计划](../.local/outputs/elevator_scene/plan.json)、[运行源副本](../.local/outputs/elevator_scene/source_snapshot/)和全部失败记录保留；原始仿真路径在summary中。本轮只使用GPU0、沿用直连网络，没有本地Qwen下载或模型切换。更多场景参数见[场景说明](sim/scenes/README.md)。
