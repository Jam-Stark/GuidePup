# MuJoCo Official API Memory

## Summary

- 创建时间：2026-06-03 22:52 HKT
- 责任人：bob
- 长期职责：专精 MuJoCo official functions/API/cfg/XML 方法，支持 IsaacLab 到 MuJoCo sim2sim 的官方规范核查。
- 回答约束：bob 之后在回答 MuJoCo 相关问题前，必须先查 MuJoCo official docs，并在回答中给出官方文档证据和来源；文档获取优先使用 context7。
- context7 library id：`/google-deepmind/mujoco`
- 初始化查询 topics：
  - `XML Reference option solver timestep integrator iterations tolerance cone impratio friction cone contact`
  - `XML Reference default class geom friction solref solimp contact condim actuator ctrlrange forcerange joint default class`
  - `API reference mj_step mj_step1 mj_step2 control callback data ctrl qpos qvel act timestep simulation state control stepping`
  - `XML Reference actuator ctrlrange ctrllimited forcerange force limited actuator shortcut motor position velocity general control range`

## 初始官方证据摘要

- XML `option` / solver：MuJoCo official `XMLreference.rst` 记录 `cone` 控制 contact friction cone 类型；`impratio` 控制 elliptic friction cones 下 frictional-to-normal constraint impedance ratio；`noslip_iterations` 和 `noslip_tolerance` 是 Noslip solver 后处理参数。
- Solver / friction modeling：MuJoCo official `modeling.rst` 建议物理精度和抑制 contact slip 时优先考虑 elliptic friction cones，尤其配合 Newton algorithm 和 small tolerance；pyramidal cones 可能更快但物理保证较弱。
- Contact / friction parameters：MuJoCo official `computation/index.rst` 说明 contact 参数来自 model definition，包含 `condim`、`friction`、`margin`、`gap`、`solref`、`solimp`；`XMLreference.rst` 说明 `contact/pair` 可显式定义 geom pair，并支持 `condim`、`friction`、`solref`、`solimp` 等属性。
- XML `default` / `class`：MuJoCo official `XMLreference.rst` 说明 `default/geom`、`default/joint`、`default/general`、`default/motor` 等 defaults class 会设置 dummy element 的默认属性；actuator shortcuts 在同一个 defaults class 中会写入同一个 underlying `general` actuator，后写 shortcut 可能覆盖先前设置。
- Actuator / control range：MuJoCo official `XMLreference.rst` 说明 actuator `ctrlrange` 定义 control input clamp range，`ctrllimited` 控制运行时是否 clamp；official `modeling.rst` 区分 `ctrlrange` control clamping、`forcerange` actuator output force clamping、`joint/actuatorfrcrange` joint input force clamping。
- Simulation stepping / control：MuJoCo official `APIreference/functions.rst` 与 `programming/simulation.rst` 说明 `mj_step` 推进一个 timestep；可在 `mj_step` 前直接设置 `mjData.ctrl`、`qfrc_applied`、`xfrc_applied`，或安装 `mjcb_control`；`mj_step1`/`mj_step2` 可拆分 pipeline 以便基于中间结果设置 control，但不兼容 RK4。`mjModel.opt.timestep` 定义积分时间长度。

## 官方来源

- https://github.com/google-deepmind/mujoco/blob/main/doc/XMLreference.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/modeling.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/computation/index.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/programming/simulation.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/APIreference/functions.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/APIreference/APIglobals.rst
- https://github.com/google-deepmind/mujoco/blob/main/doc/APIreference/functions_override.rst

## TODO Summary

- 待核查：XML `default`/`class` 继承与 actuator shortcut 覆盖规则、actuator `ctrlrange`/`ctrllimited`/`forcerange` 与 IsaacLab action scaling 对齐、geom/contact `friction`/`condim`/`solref`/`solimp` 参数映射、solver/integrator/iterations/tolerance/cone/impratio/noslip 设置、`mj_step1`/`mj_step2`/`mj_forward`/`mjcb_control` advanced control timing。

## DONE Summary

- 2026-06-03 22:52 HKT：已初始化 bob 的 `mujoco_official_api` memory entry，并记录 context7 官方文档证据入口。
- 2026-06-03 23:10 HKT：完成 MuJoCo official `option timestep` 与 Python `mujoco.mj_step(model, data, nstep=...)` semantics 核查。官方 docs 结论：XML `<option timestep>` 默认 `0.002` seconds；`mj_step` 每次推进 `mjModel.opt.timestep`；Python `nstep=N` 表示调用 underlying `mj_step` N 次；直接控制时应先写 `mjData.ctrl` 再 `mj_step`。项目结论：`resources/a2_description/a2.xml` 未设置 timestep，compiled A2 `model.opt.timestep=0.0020`；`metadata.control_decimation=4` 时 `play_policy.py` 实际 action hold 为 `0.002 * 4 = 0.008s`，不等于 metadata/training `control_dt=0.005 * 4 = 0.020s`。
