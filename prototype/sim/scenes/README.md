# 场景配置

`elevator_lobby.xml` 是本项目原生静态 MuJoCo 大厅。它没有转换或复制 Open-RMF Hotel World 的建筑、家具或纹理；10×8 m 布局为本项目设计，只参考[官方 Hotel World 布局](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_maps/maps/hotel/hotel.building.yaml)的 1.7 m 双滑门宽度。Hotel World 的建筑由 YAML 生成 Gazebo SDF，不能直接作为 MJCF 加载；[资源核查](../../../.local/outputs/elevator_scene/resource_research/RESOURCE.md)记录了原始路径。

| 项目 | 当前配置 |
|---|---|
| 大厅 | x∈[0,10] m、y∈[−4,4] m，高3 m；中间通道开阔 |
| 起点 | base xyz=[2.8,−1.6,0.55] m，初始yaw=10°；站立后的实际姿态另记 |
| 电梯 | 东侧墙、闭合金属双扇门；宽1.7 m、高2.3 m，带楼层显示、上下呼梯按钮、门框和门槛 |
| 起点视野 | 距门中心约7.76 m，电梯与普通门同时可见 |
| 干扰 | 东侧普通木门；侧边长凳及盆栽，不占据主通道 |
| 相机和控制 | 继续使用D435i名义640×360 RGB-D、当前MPC和旧A2导出 |

门保持关闭，视觉和碰撞几何来自相同静态场景；只测试找到目标、接近门口、停下。没有开门、乘梯或换楼层。模型只获得相机观测和已有状态，不获得场景目标坐标或剩余距离。

加载大厅并交给当前实验室 Qwen/Pi：

```bash
cd /home/baoquanc/workspace/ForceControl-lightnav
prototype/run.sh reset --scene elevator_lobby
prototype/run.sh run "找到电梯，走到电梯门口并停下。" --timeout-s 180 --max-recoveries 3
```

一次命令完成重置、任务执行和录像/接触记录：

```bash
.local/envs/sim/bin/python prototype/sim/record_task.py \
  --scene elevator_lobby \
  --goal "找到电梯，走到电梯门口并停下。" \
  --output .local/outputs/elevator_scene/elevator_02 \
  --timeout-s 180 --max-recoveries 3
```

每次使用新的输出目录，避免覆盖失败。`record_task.py` 保存初末RGB、5FPS视频、任务工具事件、5Hz状态采样、实际仿真目录和一个仅供离线评价的终态RGB-D缓存；原始仿真同时记录5ms接触、20ms控制状态及LightNav/MPC日志。精确时序以日志为准。它不添加默认1米条件，也不把 `COMPLETED` 当作独立语义验收。

普通 `reset` 仍按现有CLI默认加载 ProcTHOR；要加载电梯大厅需保留 `--scene elevator_lobby`。当前两个服务分别在本机8050/8766，网络仍显式直连，未改变Qwen、policy或控制权限。首轮真实结果见[电梯运行报告](../../ELEVATOR_TEST.md)。
