# Prototype 相机配置

2026-09-14 当前默认恢复为 `d435i_rgb_nominal`，开启同帧 RGB-D。LightNav 接收原始 RGB；Pi 接收 Depth 翻译器的编号图、分层距离和按需 ROI；MPC 使用独立的局部深度障碍地图。深度接口与实机对齐要求见 [DEPTH_TRANSLATOR.md](DEPTH_TRANSLATOR.md)。

## 当前 D435i RGB-D 配置

| 参数 | 仿真配置 |
| --- | --- |
| RGB / 深度光学网格 | 同一针孔视角，640×360 |
| 水平×垂直视野 | 69°×42.2726° |
| 相对 base 安装位置 | `[0.30, 0, 0.10]` m |
| 姿态 | 前向，下俯5°；随机身完整姿态变化 |
| 目标采样 | 10 Hz 仿真时间，实际帧时间戳随输出保存 |
| 有效光轴深度 | 0.2–8 m；无效像素明确为未知 |
| 提供模型的距离 | 从 base 原点到观测表面的水平距离，单位 m |

这组安装参数用于原型仿真，实机应重新标定。当前理想渲染没有模拟 D435i 原生深度相机的更宽视野、测距噪声或材质失效。相机看不见的空间也没有被证明为空旷。深度帧保存 RGB、光轴深度、内参、相机到 base 的完整外参和拍摄位姿，保证测量可追溯。

`profiles.json` 仍保存旧头部和112°参数作为历史记录；RGB-D服务只接受含 `depth_range_m` 的配置。下面的头部资料是前一轮调查结果，不是本轮默认相机。

## 历史 A2 头部相机

| 参数 | 当前配置 | 来源与含义 |
| --- | --- | --- |
| 渲染宽×高 | 768×272 | 原 384×136 的两倍线性分辨率，比例均为 48:17 |
| 安装平移 | `[0.3381, 0.0336, 0.0525]` m | 当前 MuJoCo `base_link` 局部坐标；与官方 A2 URDF 的 `camera_joint` 一致 |
| 安装姿态 | WXYZ `[1, 0, 0, 0]`，`world` convention | Owner 的 IsaacLab 配置；光轴沿父坐标系 +X，上方为 +Z，无额外下俯 |
| 水平×垂直视野 | 132°×77.00248735° | 从 Owner 的 focal/aperture 计算，属于仿真针孔参数 |
| `update_period_s` | `0.06666666666666667` | 15 Hz 的请求更新周期；实际渲染时间以帧时间戳为准 |
| `clipping_range_m` | `[0.1, 20.0]` | Owner 的仿真近、远裁剪面 |

原始参数完整保存在 `profiles.json` 的 `source_*` 字段：`source_parent_frame="trunk"`、原分辨率 `[384,136]`（宽、高）、`focal_length=1`、`horizontal_aperture=4.492073547808433`、`vertical_aperture=1.59094271484882`。这些字段记录来源；运行时采用表中的安装平移、俯角、视野、分辨率和更新周期。

针孔视野按 `FOV = 2 atan(aperture / (2 focal_length))` 计算。宽高均翻倍且保持 aperture/focal 比值不变，视野不变，768×272 下的名义 `fx=fy=170.967815px`；原 384×136 下为 `85.483908px`。未给出真实镜头畸变或标定主点，因此当前仿真仍是无畸变、居中主点模型。

IsaacLab 的 `world` convention 是 **局部安装姿态的轴约定**，不是让相机始终保持世界水平；机身俯仰和侧倾仍会传给相机。官方定义为 +X 前方、+Z 上方，WXYZ 是相对父坐标系的四元数。对应 MuJoCo 相机轴为 `xyaxes="0 -1 0 0 0 1"`。[IsaacLab CameraCfg 官方定义](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/sensors/camera/camera_cfg.py)

坐标原点核对：当前旧 IsaacLab 任务加载的 A2 URDF 和旧 MuJoCo 导出都使用 `base_link`，四个髋关节原点与机身惯性原点一致，没有 `trunk` link。Owner 另一份配置中的 `trunk` 命名无法在这些资产里直接验证。不过，2026-09-14 直连取得的官方 A2 URDF 明确将 `camera_joint` 接在 `base_link`，其平移恰为 `[0.3381,0.0336,0.0525]`、RPY 为 `[0,0,0]`，因此当前安装位置有独立的官方资产依据；并非通过质心位置补偿或猜测父坐标系偏移得到。[官方 A2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/a2_description/urdf/a2.urdf)

若机身接近水平、base 原点离地约 0.51 m，光心高度约为 0.5625 m；这是几何估算，运行时高度会随姿态变化。相机前伸 0.3381 m，因此相机已经看到门内，不代表机器人 base 已穿门。

## 官方实机规格的核实边界

Unitree A2 产品页确认 A2 和 A2-Pro 均配一台前向 HD camera；最新公开 URDF 给出上述安装位置。所查产品页与公开 SDK 没有给出可核实的 A2 相机原生分辨率、FOV、帧率、镜头畸变、视频流地址或曝光时间戳协议。**这里的 132°、15 Hz、384×136，以及提高后的 768×272，都不能写成已经确认的 A2 实机视频规格。** [A2 官方产品页](https://www.unitree.com/A2/)

公开 SDK 中能找到的是 Go2 的 `VideoClient.GetImageSample`；不能据此认定 A2 支持同一视频服务。实际接机时，应读取 A2 已提供的视频流配置和标定资料，再确定解码尺寸、是否已去畸变及帧时间戳。当前 profile 足以复现给定的仿真几何，尚未验证真实 A2 取流。[Unitree SDK 的 A2 模块](https://github.com/unitreerobotics/unitree_sdk2/tree/main/include/unitree/robot/a2)、[Go2 VideoClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/video/video_client.hpp)

## LightNav 对相机的实际要求

LightNav 协议输入为单路 RGB 图像、自然语言指令和序号，没有内参矩阵、畸变参数、相机高度或安装外参字段。路径点定义在该帧拍摄时刻的机器人局部坐标系：前、左、逆时针为正。客户端应缓存同帧实际 base pose，把返回路径点转到世界坐标；不能把路径点再当作相机光学坐标做一次外参变换。[LightNav 官方协议](https://github.com/lightorigins/LightNav-0/blob/main/docs/PROTOCOL.md)

这表示接入无需向模型发送某一套固定标定，并不表示任意安装姿态都具有相同效果。前向、刚性安装和可见地面/通道是本原型的工程选择；模型没有一个输入参数可用于显式补偿倒装镜头、大幅侧视或不同畸变。服务的 `--cam_hfov_deg`、`--cam_height` 只影响录制视频的轨迹叠加，不会校准导航模型。[LightNav 配置说明](https://github.com/lightorigins/LightNav-0/blob/main/docs/CONFIGURATION.md)

官方训练介绍描述了 FOV 90–130°、相机高度 0.5–1.5 m、俯仰角 −15° 到 15° 的采样变化；公开 Habitat 评估配置则使用 480×270、水平 FOV 120°、相机高 0.88 m。这些是训练/评估的参考分布，没有构成接入协议的固定安装要求。历史头部132°比公开训练 FOV 范围上沿略宽；本轮D435i的69°则窄于该范围。两者效果都需要实跑判断，深度约束也不能补回LightNav不可见的RGB内容。[LightNav 官方训练介绍](https://www.lightorigins.com/en/blog/lightnav-0)、[官方 Habitat 相机配置](https://github.com/lightorigins/LightNav-0/blob/main/habitat_server/configs/vlnce_r2r.yaml)

本机下载的 checkpoint `eval_config.json` 使用 `video_size=[256,448]`（高、宽）。默认 `aspect_mode=stretch` 对整帧做双线性缩放到 448×256，**没有中心裁剪或 letterbox**，与官方说明的训练预处理一致。因此 48:17 的 A2 全视野被保留，但图像宽高比例会改变。`aspect_mode=keep` 可按近似相同像素预算选择保留比例的网格；对本 profile，本地官方尺寸函数返回 544×192（宽、高）。这个选项改变训练时的 token 网格，本轮保持默认 `stretch`。[官方预处理代码](https://github.com/lightorigins/LightNav-0/blob/main/src/lightnav/inference/frame_preprocessing.py)、[官方宽高比说明](https://github.com/lightorigins/LightNav-0/blob/main/docs/CONFIGURATION.md#input-aspect-ratio---aspect_mode)

768×272 的用途是提供更细的原始 RGB 给场景观察，也避免原 136 像素高的画面直接放大到模型的 256 像素高。LightNav 最终输入仍是 448×256，模型图像 token 数不会随渲染像素翻四倍；尚不能声称增加像素已提高导航成功率。该改变还会增加渲染、JPEG 和传输开销，本轮不继续提高分辨率。

该配置已实际运行 `head_rgb_only_01`：base穿门、模型报告视觉完成，但物理记录确认沙发和门框接触。当前结果不能说明相机更换解决了碰撞，也没有隔离像素、视野、Pi观测等因素做因果比较。[实跑记录](../RUN_STATUS.md)

官方 Go2 实机 launch 用 Orbbec RGB 640×360@30fps，MuJoCo demo 使用约 112° 水平视野；它们是不同平台的部署参考，并非 A2 相机规格。相机 15 Hz、异步 LightNav 推理和旧 policy 的 50 Hz 分别运行；force 交互后的路径跟踪反馈仍使用实际 pose，不能由运动命令积分代替。[官方 Go2 launch](https://github.com/lightorigins/LightNav-0/blob/main/robot_deploy/src/vln_bringup/launch/go2.launch.py)、[官方 MuJoCo 相机说明](https://github.com/lightorigins/LightNav-0/blob/main/mujoco_demo/README.md)

## 保留的历史对比配置

| Profile | 分辨率 | 水平×垂直视野 | 安装平移 / 下俯 | 更新周期 |
| --- | --- | --- | --- | --- |
| `d435i_rgb_nominal` | 640×360 | 69°×42.2726° | `[0.30,0,0.10]` m / 5° | 0.1 s |
| `lightnav_112_reference` | 640×360 | 112°×79.6522° | `[0.30,0,0.10]` m / 5° | 0.1 s |

这两项保留此前模拟镜头与临时支架几何；`0.1 s` 是旧仿真更新周期，不是相机硬件最高帧率。D435i 的名义 RGB 视野不等于深度双目的视野，也不是用户设备的实测内参。旧运行曾向 Pi 提供同帧理想深度网格来校验距离；这些结果属于历史 RGB-D 原型，不能用来证明当前 A2 RGB 方案具有米制距离测量能力。当前 Pi 只能依据 RGB 与真实运动状态判断进度，不能把画面占比换算成已经测得的精确距离。
