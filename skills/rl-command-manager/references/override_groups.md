# Override Group Mapping

Use these prefix rules to map parameters into analysis groups.

| Prefix | Group |
|---|---|
| `env.scene.robot.actuators.` | Actuator |
| `env.commands.` | Command |
| `env.rewards.` | Reward |
| `env.curriculum.` | Curriculum |
| `agent.` | Runner/Device |
| `cli.` | Runner/Device |
| `launcher.` | Runner/Device |
| `prefix_env.` | Runner/Device |
| (fallback) | Other |

Notes:
- `cli.*` keys are synthetic keys extracted from command-line flags.
- `launcher.*` keys are extracted from launcher arguments such as `torchrun --nproc_per_node`.
