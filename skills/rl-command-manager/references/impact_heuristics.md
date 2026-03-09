# Impact Heuristics

These rules guide auto notes for changed parameters.

| Pattern | Expected Impact | Risk | Monitor |
|---|---|---|---|
| `*.stiffness` | stronger tracking / faster response | oscillation, torque spikes | `torque_limit`, `action_rate`, contact forces |
| `*.damping` | more damping / smoother motion | over-damping, slow response | tracking error, phase lag |
| `env.commands.gait_phase.gait_frequency` | gait cadence changes | instability at high frequency | contact pattern, slip, body pitch/roll |
| `env.rewards.feet_air_time.params.target_air_time` | step timing preference shifts | hop or drag behavior | air-time stats, feet height, drag |
| `env.curriculum.*.start_steps` | curriculum activation timing changes | too-early destabilization or late adaptation | reward progression, failure rate |
| `env.rewards.*` | objective weighting changes | reward hacking / forgetting behavior | reward decomposition + play qualitative checks |
| `env.commands.*` | command distribution changes | policy overfit to narrow command space | command tracking metrics |

Task anchor:
- `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py`
