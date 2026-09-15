# TODO

- 2026-06-03 22:52 HKT：核查 MuJoCo XML `default` / `class` 继承规则，特别是 `default/geom`、`default/joint`、`default/general`、`default/motor` 以及 actuator shortcuts 在同一 defaults class 中的覆盖行为。
- 2026-06-03 22:52 HKT：核查 actuator `ctrlrange`、`ctrllimited`、`forcerange`、`joint/actuatorfrcrange` 与 IsaacLab action scaling / clamp / torque limit 的对应关系。
- 2026-06-03 22:52 HKT：核查 geom/contact `friction`、`condim`、`solref`、`solimp`、`margin`、`gap`、`priority`、`solmix` 的 official semantics，并整理 IsaacLab 到 MuJoCo sim2sim 参数映射 checklist。
- 2026-06-03 23:10 HKT：继续核查 `option` 中 `integrator`、`solver`、`iterations`、`tolerance`、`cone`、`impratio`、`noslip_iterations`、`noslip_tolerance` 对 sim2sim 稳定性和 contact slip 的影响；`timestep` 默认值/单步含义已完成。
- 2026-06-03 23:10 HKT：继续核查 MuJoCo advanced state/control stepping：`mj_step1`、`mj_step2`、`mj_forward`、`mjcb_control` 在 callback、RK4、intermediate results 场景下的官方执行时序；`mj_step`、Python `nstep`、`mjData.ctrl` 直接写入与 `play_policy` action hold / decimation 对齐已完成。
