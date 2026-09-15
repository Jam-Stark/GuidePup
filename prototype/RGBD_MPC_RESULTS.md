# Qwen、D435i RGB-D 与 A2 尺寸约束 MPC

本轮实现使用旧 `model_59999.pt` 的既有 6D 导出、实验室 Qwen、真实 LightNav HF 推理和 MuJoCo。没有重新训练 policy、操作实机或引入人体包络。所有运行使用 GPU0；下载与模型调用沿用直连设置。

当前链路已实际运行：最终无外力回合行驶 5.618 m，基座穿门，未记录到非地面接触；独立平地回合验证了 force brake 锁存与人工恢复。Qwen 仍把墙面/地板误当成目标置物架，因此尚未可靠核验语义目标到达。受力的完整场景回合中途被 MPC 暂停，结果也保留在下文。

## 实现边界

- LightNav 只接收原始 RGB；Pi 接收 6×3 编号 RGB、各区域独立距离带和有效比例。
- `measure_region(frame_id,bbox_px)` 测量原图半开像素矩形，返回同帧统计及实际裁剪图。距离为 base 原点到可见表面的水平距离，单位米，不是路线剩余距离或机身前缘净空。
- 前景、背景按距离带分别返回；无效深度明确为未知。翻译器不识别家具，Qwen 负责目标语义与像素选择。
- MPC 使用 RGB-D 与拍摄位姿构建的 4 cm 局部占用体素，以及覆盖已观测 A2 身体/摆腿范围的三个圆。求解后的轨迹对全部局部障碍核验。
- 控制器不读取场景碰撞标签或目标坐标；它们只用于以下离线评价。人的锁存制动仍高于导航，只有人工能恢复。

接口和几何依据见 [Depth 翻译器](camera/DEPTH_TRANSLATOR.md)、[控制器](control/README.md)、[Pi](pi/README.md)。仿真 RGB-D 是理想针孔输出；当前同视角深度没有模拟 D435i 原生深度视野、噪声和材质失效，实机还需要正确对齐和标定。

## 运行条件与已保留的失败

目标固定为“穿过右前方的敞开门，走到门内白色置物架前，然后停下”。普通集成回合最多 120 墙钟秒、3 次恢复，没有施加外力。事先约定及后续独立控制诊断见 [plan.json](../.local/outputs/rgbd_body_mpc/plan.json)。这些是修复期间的描述性运行，不是固定版本的成功率或因果排名。

| 运行 | 运动与任务结果 | 非地面正力接触 |
| --- | --- | --- |
| baseline：旧无障碍 MPC、D435i、Qwen | 路程 5.801 m，base 穿门，Qwen 报告 COMPLETED | 沙发 273 个不同 5 ms 步，合计 1.365 s；单点峰值 741.77 N |
| integrated_01：初版深度与尺寸约束 | 路程 0.528 m，提前停止，未穿门 | 0；早停不能证明全路线改善 |
| integrated_02：首次控制修复 | 未发出导航，重复读图导致 Qwen 上下文溢出 | 0；站立期间累计路程不是导航进展 |
| controller_01：独立 LightNav+MPC | 路程 0.339 m，净空约束阻止继续，未穿门 | 0；早停不能证明全路线改善 |
| integrated_03：延迟与速度响应修复 | 路程 4.398 m，已绕过先前沙发区域，门前暂停，未穿门 | 0；已行驶部分无接触，整条路线尚未通过 |
| integrated_04：关闭离屏多重采样 | 另一条LightNav转向路线仅行驶0.394 m即不可行；Qwen两次恢复后暂停 | 0；短段早停，不能据此认为优于03 |
| integrated_05：允许转向指令立即制动 | 行驶5.618m，基座过门；Qwen报告COMPLETED，但目标ROI选错 | 0个非地面接触点，0个异常落地接触 |

接触统计排除了正常足底支撑；时长按有正接触力的 5 ms 物理步取并集，不把同一步多个接触点重复计时。数值来自 MuJoCo 求解器，不代表实机冲击力。完整原始结果位于 [运行目录](../.local/outputs/rgbd_body_mpc/)；每个运动回合保存视频、端点、样本、模型事件和接触分类。

`integrated_05` 用时32.921秒墙钟时间，episode为sim0.74–30.84秒。基座穿门时sim23.48505s、y=11.249289m在门洞内；终点基座约[11.221,11.124]m。正常足底支撑14,460个记录被排除，没有任何非支撑接触记录。MPC求解中位55.8ms、P90 77.2ms、P99 101.7ms，首个冷求解约328ms；10Hz是控制更新目标，不能据此宣称严格实时。

[最终无外力运行视频](../.local/outputs/rgbd_body_mpc/integrated_05/camera.mp4)、[同一reducer接触统计](../.local/outputs/rgbd_body_mpc/integrated_05/contact_summary.md)、[原始任务结果](../.local/outputs/rgbd_body_mpc/integrated_05/summary.json)。当前结果支持这次路径实际执行和碰撞约束的效果，不是跨路线保证，也没有独立验证目标已接近。

实际故障及对应修改：

1. 初版把额外 30 mm 净空也作为起点硬门槛，剩余 28.28 mm 时拒绝规划。身体半径与体素膨胀保持不变，额外期望净空改为软代价；预测扫掠仍保留硬约束。
2. 深度地图的二维结构排序占用 Python GIL，中位更新约 106 ms。改用整数栅格索引后，同三帧的占用集合完全相同，中位约 21 ms。帧归档也移入工作线程。
3. Qwen 的 6 个成功消息输出了 182 个读工具，其中一批含 170 次 `scene`，随后超过 262144 上下文。Pi 改用原生上下文准备接口：每个事件一次 status/scene，读取后移除已读工具，同批重复不再执行 I/O；测量和任务决策仍由模型选择。
4. `controller_01` 的求解期间仍在执行旧 0.30 m/s 指令，模型却假设新的 0.048 m/s 立即生效。实际提交位置比预测偏约 36 mm；停止后还移动约 37 mm、约 0.28 s 才接近静止。控制侧因此加入实际平面速度、执行延迟前缀和一阶响应近似。
5. 原“自身过滤”方盒误删近处沙发：暂停后实际 RGB-D 中有 25,132 个像素被删，其 base x 为 0.548–0.600 m，已超过记录的机器人前端范围 0.394 m。当前前装 D435i 与有效近距条件下移除此方盒，不将近身环境当机器人。
6. `integrated_04` 发现转向指令的对称变化限速阻止及时制动。仅改为允许向零立即减速、继续限制加速和反向起步，原失败输入恢复可解；身体与硬净空不变。`integrated_05` 随后实际穿门，未受阻。

`integrated_03` 的实际 MPC 求解耗时中位 52.8 ms、P90 72.3 ms、P99 109.9 ms，首次冷求解约351 ms。任务仅使用两轮观察：第一次发出导航，第二次在 `mpc_blocked` 后暂停；没有重复读图风暴。最后一次预测与对应实际位置相差约3 mm，剩余停止原因已由精确姿态重渲染确认为仿真深度边缘异常，不能归因于上一轮的36 mm执行延迟。

[第三轮门前暂停视频](../.local/outputs/rgbd_body_mpc/integrated_03/camera.mp4)、[接触统计](../.local/outputs/rgbd_body_mpc/integrated_03/contact_summary.md)、[任务结果](../.local/outputs/rgbd_body_mpc/integrated_03/summary.json)。

## 受力速度偏差与制动权限

`force_01` 使用最终版本在同一 ProcTHOR 场景运行 Pi → LightNav → MPC → 旧 policy。它行驶 1.054 m、净位移 0.913 m，没有非地面接触，也没有穿门。−40 N 后拉期间请求 vx 均值为 0.300 m/s，实际均值为 0.0744 m/s，末 0.5 s 均值为 0.0824 m/s；旧 policy 辅助力估计最低 −0.4519，未达到制动阈值。随后 MPC 因几何约束暂停，计划中的 −80 N、松手和人工恢复环节没有执行。此结果验证了受力时明显的指令/实速偏差，但不能证明整个导航回路在受力情况下完成目标。见[受力路线统计](../.local/outputs/rgbd_body_mpc/force_01/contact_summary.md)。

为单独执行剩余权限流程，`force_authority_flat` 在空旷场景运行同一 LightNav、新 MPC 与旧 policy，没有运行 Pi。施加 −80 N 后，注力响应快照 sim 3.08 s 至首次 `HUMAN_PAUSED` 锁存 sim 3.68 s，相隔 0.60 s；下一 20 ms tick（3.70 s）有效运动命令全部归零。放力后的首条 telemetry 为 5.12 s，此后直到人工恢复的 6.14 s 仍保持锁存和零命令。重新 `navigate` 返回 HTTP 409，人工 `resume` 返回 200；7.12 s 实际 vx 恢复到 0.148 m/s，有效指令 vx 为 0.240 m/s。

该平地回合 `summary.error=null`，无非地面接触或异常落地；结束后已暂停并重置到默认 ProcTHOR/D435i 初始场景。施力时机器人仍能被向后拉动，零命令不等于位置锁定。0.60 s 是本次注力到估计器触发的观测值，不是制动时延保证。见[权限证据](../.local/outputs/rgbd_body_mpc/force_authority_flat/contact_summary.md)和[原始结果](../.local/outputs/rgbd_body_mpc/force_authority_flat/summary.json)。

## 门前新增障碍点的来源

利用 `contacts.jsonl` 记录的同一 25.22 s qpos/qvel 重建静态场景相机，不推进物理步，复现了触发体素 `[10.82,11.70]` 的7个像素。它们沿门框图像边缘排列，但投影点落在真实门框前方。离线 `mj_ray`（排除位于相机近裁剪面内的 base 自身）确认对应真实表面比这些投影点远 0.188–0.213 m。

同一姿态、同一内外参，仅将 MuJoCo `quality.offsamples` 从默认多重采样改为0，该体素没有像素落入；同5个像素反投影相对射线命中的误差降为1.1–4.45 mm。这支持当前多重采样深度与像素中心反投影不一致的诊断。当前 RGB-D 统一关闭离屏多重采样，没有使用场景 geom ID 在线删点或调小身体包络。

[重建帧](../.local/outputs/rgbd_body_mpc/depth_point_diagnosis/frame.jpg)、[射线对照](../.local/outputs/rgbd_body_mpc/depth_point_diagnosis/offsamples_comparison.json)。MuJoCo参数含义见[官方 XML 文档](https://mujoco.readthedocs.io/en/latest/XMLreference.html#visual-quality)。它修正的是仿真观测，不代表实机 D435i 已没有边缘误差。

## 测距正确与目标选区是两件事

独立投影核对用缓存帧的真实像素、内参和外参，计算 base 点 `[2.043353,-1.024675,-0.513748]` m，水平距离 2.285880 m；HTTP 工具返回 2.286 m，仅有输出取整差异。见 [投影核对](../.local/outputs/rgbd_body_mpc/qwen_readonly/projection_check.json)。

但 Qwen 两次只读探针都没选对家具：第一次把地板的 1.85–2.22 m 距离带解释成置物架；增加真实裁剪图后，第二次正确说出所测是墙，却没有按“选择家具”的要求重选。第二次还把 p10–p90 误称置信区间，接口已明确其含义是像素距离的中央 80% 分位范围。

因此，目前不能把 `complete_navigation` 当可靠独立验收：它仍记录模型判断，没有服务端目标语义或数值距离硬门槛。深度工具已经提供真实可对应的几何信息，但不会自动解决 VLM 的物体定位错误。探针原始图、ROI、回复和事件见 [首次](../.local/outputs/rgbd_body_mpc/qwen_readonly/RESULT.md) 与 [裁剪图版本](../.local/outputs/rgbd_body_mpc/qwen_readonly_crop/RESULT.md)。

停在门前后的单次只读数值测量为6.733秒，Qwen调用status/scene/measure_region，前后都保持PAUSED。它选取的ROI确有置物架，但混合立柱、层板、空隙与背景，工具返回3.0–5.5m的多个距离带；模型据此否定已在约1m处。没有执行数值停车任务，也没有证明一个确定的前表面距离。见[端点回复](../.local/outputs/rgbd_body_mpc/qwen_endpoint_measure/reply.md)及[实际ROI裁图](../.local/outputs/rgbd_body_mpc/qwen_endpoint_measure/crop_1.jpg)。这次脚本输出目录曾误用旧探针目录，归档已分离，原始结构化日志缺失与图像重建情况已在旧探针RESULT.md中明确记录。

`integrated_05`进一步保留了完成判定失败：第一次ROI使用了status中的旧帧号而非刚返回的scene帧号，被409拒绝，模型随后改用同帧编号。其3个成功ROI都是最右列，主要为墙/地板，却在完成理由中称为置物架上中下部。服务器确实测量了指定的像素，但没有识别物体。ROI图保存为该run的`measured_roi_1/2/3.jpg`，不能用Qwen给出的1.85–2.55m替代目标前表面距离。

Owner追问是否为Depth翻译器的问题后，复核同帧编号图、工具参数和裁图：Qwen文字能描述画面中前部存在白色置物架，但实际选的是最右列6/12/18格；中间裁图主要为墙，只在左边缘有少量家具像素。直接故障是目标与测量像素的绑定错误，不能仅凭此认定模型完全认不出置物架，也未发现本次编号和ROI坐标错位。独立单像素投影核对支持距离换算实现，但不证明所有像素或实机传感准确。

当前翻译器的表达仍有明确局限：粗网格跨越家具与背景，距离层只给统计，没有显示每层对应的像素位置；即使ROI包含家具，也可能无法确定哪个距离带属于所需前表面。因此不能把系统问题全部归给Qwen。下一步应先在原图上明确展示选区及各距离层对应像素，使模型确认实际所测表面，再使用其距离；仅重复提示或换算数值不能解决目标绑定问题。此项为后续改进方向，本轮尚未实现或验证。
