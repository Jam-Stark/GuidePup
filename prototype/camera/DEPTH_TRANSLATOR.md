# 同帧 RGB-D 深度翻译器

`prototype/sim/depth.py` 使用现有 NumPy/Pillow，把同一相机帧的米制深度转成可核对的普通数据，不包含物体识别、导航决策或物体距离猜测。当前数据来源始终标记为 `synthetic_mujoco_rgbd` / `synthetic: true`；这是理想渲染结果，不代表 D435i 的实测噪声、孔洞或反光表面表现。

```python
frame = DepthFrame(
    frame_id, sim_time_s, pose_world,          # 唯一帧号、拍摄仿真时间、拍摄时 SE2
    rgb, depth_z_m,                           # uint8[H,W,3] RGB；float[H,W] 光轴 Z，米
    intrinsics={'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy},
    camera_to_base=T_base_optical,            # 4×4，同一 optical frame → robot base
    valid_range_m=(0.2, 8.0),                 # 光轴 Z 的有效开区间，不是水平距离上限
)
overview = frame.overview()
jpeg = frame.annotated_jpeg()
roi = frame.measure_region([x0, y0, x1, y1])
roi_jpeg = frame.region_jpeg([x0, y0, x1, y1]) # 同一ROI的原始RGB裁剪，供模型核对测量对象
scene_payload = frame.scene_payload(camera_profile)  # 原始RGB + 编号RGB + 统计
roi_payload = frame.measurement_payload([x0, y0, x1, y1]) # 全图选框 + 裁图 + 距离层像素图
points = frame.points_base()                  # 所有有效像素的 Nx3 base 坐标，米
```

RGB、深度、内外参和拍摄位姿必须来自同一帧。实例拥有输入图像的副本；HTTP 层按 `frame_id` 缓存实际 scene 请求的 8 帧，ROI 读取指定实例。帧号不存在时由 HTTP 返回明确错误，不能改测当前画面。编号图只供 Pi；LightNav 和原始视频继续使用未标注 RGB。

- 图像采用原分辨率，整数 `(u,v)` 指像素中心，左上角中心为 `(0,0)`。`bbox_px=[x0,y0,x1,y1]` 使用半开区域，即包含 `x0/y0`，不包含 `x1/y1`；越界或空 ROI 直接报错，不裁切。
- 光学坐标为 `+X` 向右、`+Y` 向下、`+Z` 向前；base 为前、左、上。先计算 `p_optical=[(u-cx)*Z/fx, (v-cy)*Z/fy, Z]`，再乘外参。报告距离一律是 `hypot(x_base,y_base)`，不是相机光轴深度、相机到目标的射线长度或机器人外壳净空；模型无需自行加减相机前伸量。
- 只接受无畸变、已像素对齐的 pinhole RGB-D。光轴深度必须对应 `intrinsics` 的同一 optical frame。模块不会把原始畸变图自动变成 pinhole 图。
- 零值、非有限值，以及不在配置有效深度开区间内的像素均为 unknown。unknown 不代表可通行，也不以远平面或零坐标代替。调用方需保证渲染背景远裁剪值不落入有效区间。

`overview()` 返回帧号、拍摄时间/SE2、宽高、来源、距离定义和 `cells`。固定 6 列 × 3 行，格号 1–18 按从左到右、从上到下排列。每格包含像素 bbox、总像素数、有效数、`valid_ratio`、`unknown_ratio` 和距离层 `layers`；完全无测量的格子是 `status: unknown, layers: []`。

每层的 `range_m=[low,high]` 表示宽 0.5 米的半开水平距离区间，`median_m` 是该区间内实际测量的中位数，`fraction_of_valid` 是该层占区域有效像素的比例。不会把前侧柱子和背景的距离合成一个全区域均值。距离层只是距离分布，不是物体分割：同一实体可能跨层，同一层也可能包含多个实体、墙或地面。选择目标 ROI 仍需结合对应 RGB。

`measure_region()` 使用同样的帧元数据与距离分层，并在每层增加 `count`、`p10_m`、`p90_m`。每个固定距离带有 `layer_id`，例如 `D4` 表示 `[2.0,2.5)` m。ROI 还返回 `color_rgb`、该层像素的原图包围框 `pixel_bbox_px`，以及属于该层的实际代表像素 `sample_pixel_px`。这些都由实际深度像素计算，不包含物体标签；包围框内不保证每个像素都属于该层。

细查时先在同帧 RGB 中圈定目标可见前表面；如果 ROI 混入地板、前侧柱子或背景，应保留这些层并解释混合，不能任选最接近期望距离的一层声称到达。稀疏有效像素也不能代表整个物体表面。

`region_jpeg(bbox_px)` 返回完全相同缓存帧、相同半开像素区域的原始 RGB 裁剪图，不带网格、不画物体、不做语义识别。短边不足 256 像素时等比放大，放大后的长边限制为 1024 像素；已有大图不缩小。HTTP/Pi 将该图和结构化测量一起交给模型，使其能核对测到的是家具、地面还是背景。放大只用于观看，不新增图像细节；`frame_id`、测量统计及 `bbox_px` 始终引用原图像素，不能用裁剪图的坐标直接发起下一次原图 ROI 查询。

## 像素对应的图像输出

当前 HTTP 的 scene/measure_region 都使用 `images: [{kind, label, image}]`，其中 `image` 是 Pi 的 `ImageContent`。Pi 按说明文字和图像交错传给模型，结构化统计不重复携带 base64。旧单张 `image` 字段已移除。

| 工具 | 图像 | 作用 |
| --- | --- | --- |
| scene | `rgb` 原始图、`grid` 编号图 | 先识别物体，再定位像素；粗格子不等于物体 |
| measure_region | `selection` 全图青色选框 | 检查像素坐标是否落在目标位置 |
| measure_region | `crop` 原始裁图 | 检查实际被测表面与背景 |
| measure_region | `layers` RGB/距离层并排图 | 同一裁剪几何，逐像素颜色和 D 编号对应距离统计 |

`layers_png()` 使用无损PNG保存像素分层图；距离层图缩放采用最近邻，灰色表示unknown。标注点选自真实层内像素，图例给出半开距离区间、中位数和有效像素比例。原始RGB与彩色分层图使用同一裁剪范围和显示比例。目标与背景在相近距离时仍可能同属一个距离带；该输出没有自动完成语义分割。

旧表达只在 `prototype/sim/depth_replay.py` 的实验 baseline 中重建，作为同帧对照条件；生产接口只输出增强表达。重放服务从冻结NPZ读取RGB、深度和标定，不读取离线目标mask，不提供运动接口。

`points_base()` 不降采样、不膨胀、不删地面，返回所有有效像素对应的 Nx3 点。控制适配层负责用拍摄时完整 base 姿态/高度去地面、去自身、变换到世界系和维护已见占用；此模块不读取场景几何真值。已约定建议 0.04 米 XY 体素，控制器只对单元外接半径 `0.04 / sqrt(2)` 膨胀一次，避免两侧重复膨胀。缺失点不能清空历史障碍或证明空间空闲。

实机适配依据：RealSense 官方 SDK 定义了像素中心、光学坐标、内外参、畸变和米制反投影。需要使用实际流的标定和 `depth_scale`，不能把 `Z16` 原整数当米，也不能用普通 resize 代替几何对齐。[SDK 投影与坐标说明](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)、[官方 RGB/depth 对齐示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)。

实现实机输入时尤其要核对 Z 所属坐标系：官方 `align_z_to_other` 会几何映射像素并保留源 depth 的深度数值；对齐到 RGB 网格不等于已经重新计算 RGB optical frame 的 Z。若使用 RGB 网格，需要通过标定外参把深度点转到 RGB optical frame 后生成相应 Z；也可把 RGB 对齐到已校正的 depth 网格并使用 depth 内参/外参。该模块的输入合同要求坐标系一致。[官方对齐实现](https://github.com/realsenseai/librealsense/blob/master/src/proc/align.cpp)、[官方投影与遮挡说明](https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/)。

本模块已用一次合成算例验算前景/背景分层、相机平移到 base 距离、无效像素、ROI 和同尺寸编号 JPEG。真实 MuJoCo 场景的接入与导航效果由主线运行记录验证；未做实机 D435i 标定或测距实验。

## 仿真像素中心与离屏采样

当前MuJoCo RGB-D关闭`quality.offsamples`。真实闭环曾在门框边缘形成7个深度像素，投影成真实表面前约19–21cm的假障碍；同姿态禁用多重采样后该体素消失，5个像素与独立射线交点误差降至约1–4mm。MPC未读场景标签，射线仅供离线诊断。详见[运行结果](../RGBD_MPC_RESULTS.md)。这不替代实机D435i的标定、有效性判定或边缘质量处理。
