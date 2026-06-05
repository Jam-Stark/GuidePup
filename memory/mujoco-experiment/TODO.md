# MuJoCo Experiment TODO

## Open

- 2026-06-03 23:28 HKT：继续把 A2 XML 当作尚未完全迁移适配的 official baseline，补齐剩余 A2 MJCF migration checklist：solver/integrator、motor actuator model/control range、joint passive `frictionloss` 与 stiffness/passive spring semantics、mass/inertia/COM、foot/ground contact geom、`condim/friction/priority/solref/solimp` 与 IsaacLab cfg/export metadata 对齐。显式 `<option timestep="0.005">`、leg joint passive damping nominal `4.5/4.5/9.0`、armature `0.03` 已在 2026-06-03 23:28 HKT 完成。
- 2026-06-03 23:20 HKT：继续补齐 IsaacLab play trace 与 MuJoCo trace 对齐：同一 command、同一 initial state，记录前 50 control steps 的 observation slices、action、target joint position、base pose、contact state，定位 first divergence。
- 2026-06-03 23:20 HKT：在 A2 XML migration 后重复最小实验矩阵：`timestep/nstep` 正确对齐、obs/action delay `0/1/2`、friction `1.0/1.5`、passive joint dynamics nominal vs zeroed/Isaac-equivalent、PD gains/current checkpoint metadata，每组只跑短 headless metrics。
- 2026-06-03 23:20 HKT：让 bob 继续核查 MuJoCo official motor actuator torque control、contact `condim/friction/priority/solref/solimp`、solver/integrator 与 passive joint loss 的官方语义。
- 2026-06-03 23:20 HKT：让 john 继续核查 `policy_vec[:, -46:]` term-major current-slice 风险；checkpoint 对应 cfg/logs 与 active cfg drift 已在 2026-06-05 14:43 HKT 完成静态核查。
