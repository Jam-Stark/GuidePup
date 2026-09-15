# Depth 翻译器增强与三模型同帧对照

2026-09-14 HKT。增强版已接入生产 Pi/MuJoCo 接口，18 个预定回合全部执行完毕。Gemini 在这组三视角中的最终目标选区和距离对应最一致；DeepSeek 能利用增强图示取得目标距离，但仍夸大选区纯净程度；Qwen 仍出现缺测、选错区域和选错距离层。不能把“提供了深度”直接等同于“模型可靠理解了深度”。

## 实现

`scene` 提供同帧原始 RGB、6×3 编号图和深度概览。`measure_region(frame_id,bbox_px)` 提供三种视图：全图青色选框、原始裁图、RGB/距离层像素并排图。每个距离带有 D 编号、颜色、像素包围框和真实代表像素；统计和图像来自同一像素集合。未知深度显示为灰色，距离定义仍为 base 原点水平距离。

![实际缓存帧的分层展示例子](../../.local/outputs/depth_translator_compare/presentation_preview/layers.png)

该图的 ROI 是为展示输出而人工选择的，并非某个模型的选区。D3 对应前侧立柱的一部分，而背景和更远结构落入其他距离带；同一距离带仍可能含多个物体。翻译器没有新增语义模型或自动物体标签。

当前接口使用 `images: [{kind,label,image}]`，Pi 按说明与图片交错发送，结构化 JSON 不重复携带图片数据。旧单图字段已移除；旧表达只在只读实验服务中作为 baseline 保留。实现细节见[深度接口](../camera/DEPTH_TRANSLATOR.md)。

TypeScript 编译通过；生产服务重启后，Pi CLI 成功读取原始图，HTTP scene/measure_region 的帧号一致，分别返回2张/3张图，20900个有效像素全部归入相应距离层。见[实际接口记录](../../.local/outputs/depth_translator_compare/live_api_check.json)。

正式对照共46次ROI请求，其中45次成功、1次越界报错。独立参考重新计算45个成功ROI的逐层像素计数，全部与工具结果一致；这支持像素分层实现的一致性，不代表目标语义或实机深度已经可靠。

## 固定条件

- 模型：实验室 `Qwen3.6-35B-A3B-FP8/`；ZenMux `google/gemini-3.8-flash`、`deepseek/deepseek-v4.1-flash`。
- 三帧 × 两种表达 × 三模型，共18个独立Pi对话。每个对话最多4次ROI请求、150墙钟秒；没有为更好答案重跑。
- baseline：编号图、旧距离统计、ROI裁图；enhanced：增加原图、全图选框、像素分层图与层内像素位置。对应ROI的深度统计不变。
- 同一用户提示和Pi系统提示，工具均为 `status/scene/measure_region`，`tool_choice=auto`，每次模型回复上限4096 tokens。每次成功读取status/scene后固定观测，不轮询。
- Qwen thinking off；Gemini/DeepSeek reasoning low。两种端点和推理策略的计算预算不等价，记录配置而不作等算力排名。
- 帧来自同一旧A2、D435i、无MSAA的MuJoCo运行：far与near为真实缓存，doorway由sim23.00s记录姿态离线重建。起点和终点重渲染RGB/Depth与缓存逐像素一致，离线分割可对应原始像素。
- 目标mask只用于独立评价，模型只接收RGB、深度、标定和拍摄位姿，没有目标标签、目标世界坐标或真实剩余距离。

目标任务是选择白色置物架朝向机器人的可见实体小区域，核对实际测量并判断是否在1m以内；不是运行导航或验证停车。目标可见结构的最近base水平距离分别为7.322、2.247、1.891m，三帧均为负例。远帧86个目标像素的光轴Z超出8m有效范围，按未知处理；离轴有效点的base水平距离可以大于8m。

完整条件见[预注册计划](../../.local/outputs/depth_translator_compare/plan.json)、[相同提示](../../.local/outputs/depth_translator_compare/prompt.txt)、[帧清单](../../.local/outputs/depth_translator_compare/reference/case_manifest.json)、[独立参考](../../.local/outputs/depth_translator_compare/reference/REFERENCE.md)。源码副本保存在[source_snapshot](../../.local/outputs/depth_translator_compare/source_snapshot/)，未提交Git。

## 结果

下表按far / doorway / near排列。比例为最终实际测量ROI内属于置物架结构的像素占比，独立mask排除架内空隙、墙地和碗盘等物体；这不是测量成功率，也不保证ROI中的每个距离层都属于目标。

| 模型 | 旧表达：最终ROI目标像素占比 | 增强表达：最终ROI目标像素占比 | 主要结果 |
| --- | --- | --- | --- |
| Qwen | 31.60% / 缺测 / 99.92% | 0% / 25.45% / 缺测 | 增强版仍有错误区域、错误层与无工具依据的数值声明 |
| Gemini | 92.22% / 98.08% / 100% | 100% / 100% / 100% | 六回合最终距离均有目标像素支持；增强版最终选区更干净，但过程有错误与纠正 |
| DeepSeek | 55.40% / 无最终回答 / 95.86% | 87.83% / 96.37% / 100% | 增强版三帧都给出有目标像素对应的距离，仍有背景混入和叙述过度 |

### Qwen：增强图示没有解决主要失败

- 旧表达doorway、增强表达near没有调用 `measure_region`，却声称已经测量并提供median、p10、p90和有效比例，不能视为实测。
- 增强表达far用了4次ROI，每次目标像素均为0%；最终将背景的5.538m归给置物架。
- 增强表达doorway的ROI目标像素为25.45%。D5中91.22%像素属于目标，D6仅0.98%属于目标；模型最终采用D6的3.162m，并声称没有混入其他表面。这正是已有图层信息却选错距离归属的例子。
- 旧表达near最终ROI目标像素99.92%，选中2–2.5m层的目标比例99.85%，数值2.243m有目标像素支持。但它把“全局中位数落在该层”错误推成“全局中位数等于该层中位数”；独立计算该ROI目标像素中位数约2.078m。几何选区改善不等于统计解释正确。

### Gemini：最终结果一致，增强版并非每次更省步骤

增强表达三帧的最终ROI及所选距离层均为100%目标像素，报告立柱距离分别为7.406、2.337、1.973m，均引用实际工具统计。

far由旧表达4次ROI变为增强表达2次；doorway均2次；near由1次变为3次请求。增强near第一次测到纯背景，第二次提交 `[576,450,596,550]`，超出原图640×360而收到HTTP400，第三次才正确测量立柱。它在最终回复中说明了2次有效测量，失败请求仍保留。该例说明当前多图界面下仍可能出现选区/坐标错误，不能宣称一次定位就可靠。

### DeepSeek：增强版取得最终目标测量，表面描述仍不够准确

旧表达doorway执行了4次ROI，最终只输出一个类似第5次 `measure_region` 的DSML调用文本，没有最终JSON或距离判断，也没有实际执行该末尾请求。runner的FINISHED仅表示模型以普通文本结束，此回合没有完成所要求的回答。

增强表达三帧报告7.411、2.801、1.973m，都有实际目标像素支持。但far的最终ROI目标比例仅87.83%，D14层目标比例95.16%；doorway的ROI和D5层均为96.37%。模型所称“未见背景”或“全部是立柱”不成立。doorway与Gemini选的是不同立柱，2.801m与2.337m不能直接当作同一表面的测距误差。

## 耗时与解释边界

| 模型 | 旧表达回合耗时中位数 | 增强表达回合耗时中位数 | 旧 / 增强ROI请求总数 |
| --- | ---: | ---: | ---: |
| Qwen | 17.972s | 4.540s | 4 / 5 |
| Gemini | 14.943s | 20.612s | 7 / 7 |
| DeepSeek | 29.931s | 36.897s | 12 / 11 |

每格只有3个回合，耗时含多轮模型请求与工具执行；Qwen的短回合包含未测量即回答，不能解释为完成同一测量任务更快。原始token用量、调用数和耗时保存在[runtime_summary.json](../../.local/outputs/depth_translator_compare/runtime_summary.json)，不同提供方的token口径不直接当作费用比较。

18个回合均按计划结束，无超时或提供方HTTP错误；其中1次工具HTTP400后继续纠正。17条最终距离判断均为no，另1条缺少最终判断。三帧全是1m负例，不能由no推断任务成功，不能评价是否会错误拒绝已经达到1m的情况。

这组证据支持增强版可展示可核对的像素/距离关系，并且Gemini、DeepSeek在部分视角利用这些信息收紧或重选ROI。它不支持对所有模型、跨场景或动态导航的普遍改善结论；原图、选框、像素图和元数据是一起改变的，也没有分离各项贡献。

下一轮动态到达核验可优先采用Gemini作为候选。Qwen仍不能独立担任可靠的测距/到达裁判；本轮没有改变默认模型配置，也没有把数值结果作为控制硬门槛。实际1m附近的正负例、动态时间一致性以及实机D435i误差仍未验证。

逐次ROI、距离层、模型原文和错误见[独立评价](../../.local/outputs/depth_translator_compare/evaluation/all_models_SUMMARY.md)、[原始运行目录](../../.local/outputs/depth_translator_compare/runs/)。每个 `roi_reference_overlay.png` 都是实验后的离线真值与选区叠图，没有提供给模型。模拟器与LightNav服务保留；只读对照服务在收尾时停止。
