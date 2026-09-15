# Resident task monitor

Current evidence and limitations: [NAVIGATION_FRAMEWORK.md](../NAVIGATION_FRAMEWORK.md). Earlier frozen experiments remain in NAVIGATION_YOLO.md, NAVIGATION_STRESS.md and NAVIGATION_V2.md.

## Decision context

Each request is rebuilt from the user goal, current task/stage, measured observation, actual available tools and one preceding accepted movement decision. currentStep describes the present operation. Requirements for completion do not restrict search/observation; target geometry and follow_target instructions appear only when a target exists. The common prompt does not encode class-specific appearance templates. YOLO vocabulary belongs to task configuration, and semantics come from the request and actual RGB.

Ordinary requests include current RGB, current semantic overlay and the previous decision RGB when present. Measurement review adds relevant crop/membership images and retained boundary crops. Source images are archived individually. The sensor imagePaths array includes images not sent to the model, so request attachment order must be reconstructed from actual kinds and buffer references.

The previous-movement buffer contains actual50Hz motion separately from LightNav predictions. Questions and intermediate perception calls do not replace it. One follow_target command retains one control task and buffer across internal alignment/approach/final-heading phases. Explicit recall retrieves at most two earlier task segments; failed-attempt filtering is a separate boolean from the natural-language reason.

## Tool protocol

The model submits one proposal. Monitor performs its operation; normal operations do not require another user approval. Measurement role is derived from action type. Review selections follow the measurement array order. Existing identity is replaced only through explicit replacePreviousIdentification; first/same observation bookkeeping is inferred by the framework.

Pi StringEnum supplies actual enums rather than anyOf/const. Current mask IDs are enumerated in the tool schema. Validation errors are returned as native toolResult/isError, with one correction inside the same30-second decision deadline. Invalid proposals never execute. Other control/provider/runtime errors remain explicit failures. Requests, responses, full context, validation errors and executed effects remain separate log events.

## Measured support and semantics

Depth layers and semantic masks retain exact frame/depth membership, world representative points, axis-aligned measured support bounds and fitted surface residuals. Mask relations report actual pixel overlap and world-support distances. These facts do not determine object identity, full object extent or semantic relationships.

A passage stage holds spatial boundary evidence groups with B identifiers, source crops and measurement counts. Nearby samples that cannot form opposite boundaries of a usable opening are grouped as one navigation-boundary hypothesis; this does not prove they are the same physical object. Spatially separated supports still require visual confirmation that the intervening opening is the requested one. Left/right roles are derived when constructing the confirmed geometry. The existing minimum opening span is an action-geometry requirement, not a recognition template.

The current metric follower uses approximately vertical surface geometry. Geometry support requirements describe what this action can execute; they must not be presented as object-category recognition rules. Target anchors retain a world reference point across new views, with explicit correction when the retained identification was wrong.

## Execution and completion

search_target and change_view rotate the body to a measured angular checkpoint, then acquire a new view. The camera has no separate head joint. A confirmed metric goal runs continuously through alignment, approach and final heading under the existing MPC footprint/obstacle/latency constraints. LightNav receives the stage semantic instruction and remains a separate prediction/hint source. Internal goal-phase changes discard late MPC responses without starting a new external control generation.

Monitor automatically checks completion after observation and semantic review. Passage requires the complete footprint beyond the confirmed opening and configured exit margin. Approach requires an executed and ended action in the current stage, reviewed current target ROI, associated surface, configured distance/heading/lateral conditions and measured low-speed stop.

Before complete, Monitor reads fresh control status, checks generation and recomputes conditions from current pose/velocity. Normal yaw drift does not invalidate a static world target by itself. If conditions have changed, observation/adjustment resumes. Explicit numeric ROI range is remeasured after translation when necessary. The control service must accept complete before Monitor marks a stage complete. The removed propose_stage_complete tool is not a supported path.

Python owns force-brake latching, motion lease and control generation. HUMAN_PAUSED cancels model work and rejects autonomous complete; only the operator resumes. Questions expose answer and recall tools, and cannot replace the task. These authority paths were retained.

## Evidence boundary

The current build, three real model-context probes and recorded boundary/completion code paths are documented in NAVIGATION_FRAMEWORK.md. A focused source review found no new completion bypass of current evidence or human-pause authority. The attempted fresh simulation stopped during NVIDIA initialization before task start, so these results do not establish a new full-task success rate or hardware behavior.
