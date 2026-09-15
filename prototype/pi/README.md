# Pi resident navigation monitor

The runtime uses one resident event loop, one model proposal at a time, one preceding movement-decision buffer, and SQLite/JSONL task archives. Python owns LightNav/MPC/old A2 motion and the human force brake. Current framework changes and evidence: [NAVIGATION_FRAMEWORK.md](../NAVIGATION_FRAMEWORK.md). Design: [MONITOR_DESIGN.md](MONITOR_DESIGN.md).

## Run

From `/home/baoquanc/workspace/ForceControl-lightnav`:

```bash
bash prototype/run.sh build
bash prototype/run.sh reset --scene room_elevator --record-contacts
bash prototype/run.sh monitor --port 8770
```

Only one monitor owns a simulator. Stop the resident monitor before using a recorder, which resets the scene and starts its own monitor. HTTP on `127.0.0.1:8770`: `GET /state`; `POST /task {"goal":"出门走到电梯门口"}`, `/question {"text":"现在走到哪里了？"}`, `/pause`, `/resume`. Only the operator can resume human pause. The `run` and `chat` commands use the same TaskMonitor. During an active task, plain chat text and `/question` request an answer; replace the goal explicitly through `/run` or `/task`. Questions cannot replace the task.

```bash
bash prototype/run.sh run '出门走到电梯门口' --timeout-s 360 --max-recoveries 3
bash prototype/run.sh chat
```

## Observation, memory and completion

Every visual request includes current RGB and its YOLO overlay, with the previous accepted movement's RGB when present. The model selects current mask IDs or normalized ROI coordinates; the monitor binds the actual frame, measures, and supplies crops/distance membership for semantic review. YOLO vocabulary is task configuration. Common prompts do not prescribe an object's appearance. Mask overlap and measured world support describe relations without proving entity identity; nearby observations cannot be counted as opposite passage boundaries solely because their IDs differ.

`currentStep` and the actual tool list describe what can be done now. Missing target evidence permits observation; completion requirements are not observation prerequisites. The monitor assigns measurement roles and infers ordinary association state. Remaining enums use Pi's provider-compatible `StringEnum`. A parameter error executes nothing, returns an error tool result and allows one correction within the same decision deadline.

The buffer contains only the previous accepted maneuver and its observation, followed by actual motion and separately identified LightNav predictions. Intermediate model calls and questions do not replace it. A `follow_target` decision keeps the same buffer through its internal turning, approach and final heading; the next accepted movement decision archives it. `recall_memory` retrieves a bounded earlier segment only for the current decision. Plain `models.complete` does not automatically read skills or accumulate an Agent transcript.

Unspecified approach goals use the explicit [navigation-contract.json](navigation-contract.json): footprint clearance0.6–0.9m, facing the observed plane within25 degrees, and lateral alignment with the measured patch. These are prototype conventions. Explicit numeric user distance overrides them. After current semantic confirmation, Monitor checks the latest control state and automatically completes/advances stages; no model completion tool remains. A completion event still does not replace independent target/contact verification.

## Model and artifacts

The lab Qwen service and exact `Qwen3.6-35B-A3B-FP8/` model ID are configured locally. Pi 0.85.1 uses `models.complete`, required single tool proposals, non-parallel tool calls, thinking off and explicit failures. Do not print or copy `.env` credentials into reports. Network commands clear proxy variables and use direct access.

SQLite stores task documents and full events. JSONL mirrors requests/responses, proposals, ROI results, target associations, `decision_buffer_updated`, `decision_segment_archived`, control transitions and stage completion. Images are separate files. `/motion` gives the current compact execution segment; raw telemetry and all LightNav predictions remain under the simulator output directory. `navigation_session_id` and frame counts expose whether same-stage history persists.

Primary modules: `src/task-monitor.ts`, `src/decision-memory.ts`, `src/monitor-evidence.ts`, `src/monitor-tools.ts`, `src/monitor-store.ts`, `src/decision.ts`, `src/sim-client.ts`. Earlier results are retained in [MONITOR_TEST.md](../MONITOR_TEST.md), [DEPTH_COMPARISON.md](DEPTH_COMPARISON.md), and [ZENMUX_RESULTS.md](ZENMUX_RESULTS.md). They do not establish v2 performance.
