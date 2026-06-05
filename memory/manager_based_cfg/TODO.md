# Manager-Based CFG TODO

## Open

- 2026-06-03 22:52 HKT：用 IsaacLab play 与 MuJoCo `play_policy` 的前几步 logs 对比 observation、history buffer、raw action、processed action、target joint position、external force、base pose/velocity/contact state，定位 first divergence。
- 2026-06-03 23:15 HKT：在 MuJoCo `play_policy.py` 前先确认 loaded `model.opt.timestep` 是否等于 IsaacLab `sim.dt=0.005`；若仍是 A2 XML 默认 `0.002`，需要先修正 timestep/control_decimation，否则 gait clock、control_dt、delay unit 和 action hold 都不等价。
- 2026-06-03 23:15 HKT：核查 `AsymmetricActorCritic(policy_history_only=True)` 的 `curr = policy_vec[:, -46:]` 是否与 term-major ObservationManager layout 的 “current frame” 语义一致；若训练和 MuJoCo export 都使用同一错误语义则 sim2sim 一致，但这可能降低策略可解释性。
