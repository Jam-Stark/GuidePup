# Agent Roster Memory

更新时间：2026-06-03 22:55 HKT

## 职责

本 entry 记录本项目长期保留的子agent roster、职责边界、对应 memory 入口和后续任务分配规则。主控 agent 在分派 MuJoCo sim2sim、MuJoCo official API/cfg、manager-based cfg 相关任务前，先读取根目录 `MEMORY.md`，再读取本文件与对应职责 entry。

## 长期子agent

- alan：runtime agent id `019e8df7-31de-7ca1-b25b-31e190ae53ec`，tool nickname `James`。职责是当前 MuJoCo sim2sim experiment 配置核查/解释。职责 memory：`memory/mujoco-experiment/description.md`。回答前必须查对应 MuJoCo experiment Memory，并给出 memory 位置证据和来源。
- bob：runtime agent id `019e8df7-549a-75c1-b5e9-feef667ab159`，tool nickname `Volta`。职责是 MuJoCo official functions/API/cfg/XML 规范核查，支持 IsaacLab 到 MuJoCo sim2sim。职责 memory：`memory/mujoco_official_api/description.md`。回答前必须查 MuJoCo official docs，优先 context7，并给出官方文档证据和来源。
- john：runtime agent id `019e8df7-7f87-7601-804e-d6856c44d302`，tool nickname `Archimedes`。职责是当前 manager-based 版本 cfg 的核查、解释和修改方案。职责 memory：`memory/manager_based_cfg/description.md`。回答前必须带 code 引用，优先引用 `source/LMP/acc/tasks/manager_based/uniFP_aliengo/unifpaliengo_env_cfg.py` 和必要 supporting source。

## 分配规则

- MuJoCo `play_policy.py` CLI、policy export、runtime 参数、delay/friction/model/first divergence 相关任务优先交给 alan。
- MuJoCo XML、actuator、contact/friction、solver、timestep、`mj_step` / `mj_step1` / `mj_step2` / control callback 官方规范相关任务优先交给 bob，并要求使用 context7 查官方 docs。
- IsaacLab manager-based cfg、observation/action scaling、HistoryWrapper、command routing、force/position control、noise/domain randomization、reward/termination 相关任务优先交给 john，并要求提供 code 引用。
- 需要 code implement 时，主控 agent 只做需求拆解和最终复核；implementation 交给 code worker，并另启 implement reviewer double check，同时同步相关 memory TODO/DONE。
- 如果 runtime agent id 不可 resume，则用相同职责和 memory 约束重新 spawn 对应角色，并在本 entry 更新新的 id。

## TODO Summary

- 维护长期 roster，若子agent id 替换、职责改变或新增专项 agent，需要更新本文件和 `memory/MEMORY.md`。

## DONE Summary

- 2026-06-03 22:55 HKT：记录 alan/bob/john runtime agent id、职责入口、回答前证据约束和后续任务分配规则。
