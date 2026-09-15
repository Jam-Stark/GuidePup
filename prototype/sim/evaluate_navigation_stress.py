"""Reduce a navigation-stress batch from existing recorder and evaluator artifacts.

This is deliberately an offline reducer.  It neither starts a simulator nor reruns an
episode evaluator: a missing recorder/evaluator artifact remains a missing datum.
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def read_json(path: Path):
    with path.open() as stream:
        return json.load(stream)


def read_jsonl(path: Path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def value_at(value, *path):
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def observed(value, reason=None):
    return {"status": "OBSERVED", "value": value} if reason is None else {
        "status": "NOT_OBSERVED", "reason": reason,
    }


def short_json(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")) if value is not None else ""


def range_contains(value, bounds):
    return isinstance(value, (int, float)) and isinstance(bounds, list) and len(bounds) == 2 and bounds[0] <= value <= bounds[1]


def terminal_event(events):
    terminals = [event for event in events if event.get("kind") in {"task_completed", "task_stopped"}]
    if not terminals:
        return None
    event = terminals[-1]
    return {
        "event_kind": event["kind"],
        "status": value_at(event, "data", "status"),
        "reason": value_at(event, "data", "reason"),
        "seq": event.get("seq"),
        "control_state": value_at(event, "data", "control", "state"),
        "control_reason": value_at(event, "data", "control", "reason"),
    }


def qwen_from_events(events, available):
    if not available:
        return observed(None, "agent.jsonl is missing; Qwen calls and usage are unavailable.")
    requests = [event for event in events if event.get("kind") == "decision_request"]
    proposals = [event for event in events if event.get("kind") == "proposal"]
    usage = Counter()
    for event in proposals:
        item = value_at(event, "data", "result", "usage")
        if isinstance(item, dict):
            for key in ("input", "output", "cacheRead", "cacheWrite", "reasoning", "totalTokens"):
                if isinstance(item.get(key), (int, float)):
                    usage[key] += item[key]
    return {"call_count": len(requests), "usage_sum": dict(usage), "source": "agent.jsonl"}


def target_anchor(events, physical, evidence):
    anchors = [value_at(event, "data", "current") for event in events if event.get("kind") == "target_associated"]
    anchors = [anchor for anchor in anchors if isinstance(anchor, dict)]
    if not anchors:
        return observed(None, "No target_associated event was saved.")
    faces = value_at(physical, "geometry", "elevator_faces")
    if not isinstance(faces, list):
        return observed(None, "Actual elevator-leaf geometry is unavailable for target-anchor comparison.")
    plane_tolerance = value_at(evidence, "actual_elevator_stop", "contract_comparison_geometry_only", "source", "target_plane_max_rms_m")
    if not isinstance(plane_tolerance, (int, float)):
        return observed(None, "Saved target-plane tolerance is unavailable for target-anchor comparison.")
    anchor = anchors[-1]
    point = anchor.get("pointWorldM")
    retained = anchor.get("referencePointWorldM")
    if not isinstance(point, list) or len(point) < 3:
        return {"status": "NOT_OBSERVED", "reason": "The saved current target ROI has no pointWorldM.",
                "anchor_id": anchor.get("id"), "retained_reference_point_world_m": retained}
    residuals = [abs(point[0] - face["front_plane_x_m"]) for face in faces
                 if isinstance(face, dict)
                 and isinstance(face.get("front_plane_x_m"), (int, float))]
    on_actual_leaf = any(isinstance(face, dict) and isinstance(face.get("front_plane_x_m"), (int, float))
                         and isinstance(face.get("y_interval_m"), list) and len(face["y_interval_m"]) == 2
                         and isinstance(face.get("z_interval_m"), list) and len(face["z_interval_m"]) == 2
                         and abs(point[0] - face["front_plane_x_m"]) <= plane_tolerance
                         and face["y_interval_m"][0] <= point[1] <= face["y_interval_m"][1]
                         and face["z_interval_m"][0] <= point[2] <= face["z_interval_m"][1]
                         for face in faces)
    return {
        "status": "OBSERVED",
        "anchor_id": anchor.get("id"),
        "semantic_claim": anchor.get("semanticClaim"),
        "roi_representative_point_world_m": point,
        "retained_reference_point_world_m": retained,
        "nearest_actual_leaf_plane_residual_m": min(residuals) if residuals else None,
        "plane_membership_tolerance_m": plane_tolerance,
        "point_on_actual_leaf": on_actual_leaf,
        "wrong_target_anchor_geometry": not on_actual_leaf,
        "definition": "Geometric comparison of current pointWorldM from the target ROI with either finite actual elevator leaf in x/y/z; retained referencePointWorldM is reported separately. It does not prove visual target identity.",
    }


def stage2_rows(rows):
    result = []
    for row in rows:
        scope = row.get("navigation_scope")
        if isinstance(scope, str) and scope.endswith(":2:1"):
            result.append(row)
    return result


def lightnav_stage2(rows, events):
    raw_rows = stage2_rows(rows)
    if not raw_rows:
        return observed(None, "No raw LightNav response for navigation scope :2:1 was saved.")
    first_raw = raw_rows[0]
    accepted_rows = [row for row in raw_rows if row.get("discarded") is False]
    first = accepted_rows[0] if accepted_rows else None
    session_ids = [row.get("navigation_session_id") for row in accepted_rows if row.get("navigation_session_id") is not None]
    first_result = first.get("result") if isinstance(first, dict) and isinstance(first.get("result"), dict) else None
    point_rows = []
    for row in accepted_rows:
        result = row.get("result") if isinstance(row.get("result"), dict) else {}
        p = result.get("pointing") if isinstance(result.get("pointing"), dict) else {}
        if result.get("visible") is True and p.get("opos_state") == "point" and p.get("opos_px") is not None:
            point_rows.append(row)
    stage1_seqs = [event.get("seq") for event in events if event.get("kind") == "stage_advanced"
                   and value_at(event, "data", "currentStage") == 1 and isinstance(event.get("seq"), int)]
    hint_inspected = any(event.get("kind") == "target_hint_inspected" and stage1_seqs
                         and isinstance(event.get("seq"), int) and event["seq"] > stage1_seqs[0] for event in events)
    return {
        "status": "OBSERVED",
        "instruction": first.get("instruction") if first else None,
        "first_capture_pose_world": first.get("capture_pose_world") if first else None,
        "first_capture_time_s": first.get("capture_time_s") if first else None,
        "first_actual_result": first_result,
        "first_result_schema": "Accepted raw lightnav.jsonl result.actions = {step, actions}; no predicted path is substituted for it.",
        "session_id": session_ids[0] if session_ids else None,
        "raw_response_count": len(raw_rows),
        "accepted_response_count": len(accepted_rows),
        "first_raw_reset_step": {"sequence": first_raw.get("sequence"), "capture_pose_world": first_raw.get("capture_pose_world"),
                                 "result": first_raw.get("result"), "discarded": first_raw.get("discarded")},
        "first_raw_response_discarded": first_raw.get("discarded") is True,
        "session_steps": len({row.get("sequence") for row in accepted_rows if row.get("sequence") is not None}),
        "stop_while_invisible": any(value_at(row, "result", "stop") is True and value_at(row, "result", "visible") is False for row in accepted_rows),
        "visible_target_point_without_recorded_hint_review": bool(point_rows) and not hint_inspected,
        "visible_point_row_count": len(point_rows),
        "recorded_target_hint_review_after_stage1": hint_inspected,
        "accepted_only_definition": "Behavioral counts exclude discarded raw LightNav responses; first_raw_reset_step is retained solely as reset provenance.",
    }


def interaction(events, interaction_rows, interaction_available, arm):
    if arm == "force_resume" and not interaction_available:
        return {
            "applicable": True, "force_applied": None, "force_latched": None,
            "question_submitted": None, "question_answered": None, "explicit_resume": None,
            "monitor_task_ids": sorted({event.get("task_id") for event in events if event.get("task_id")}),
            "task_changed": None, "exposure_status": "NOT_OBSERVED",
            "source": "interactionlog.json is missing; force exposure cannot be inferred.",
        }
    force_applied = [row for row in interaction_rows if row.get("kind") == "force_applied"]
    brake = [row for row in interaction_rows if row.get("kind") == "force_brake_observed"]
    questions = [row for row in interaction_rows if row.get("kind") == "question_submitted"]
    answers = [row for row in interaction_rows if row.get("kind") == "question_answer"]
    resumes = [row for row in interaction_rows if row.get("kind") == "explicit_human_resume"]
    agent_pauses = [event for event in events if event.get("kind") == "human_paused"]
    agent_resumes = [event for event in events if event.get("kind") == "human_resumed"]
    task_ids = {event.get("task_id") for event in events if event.get("task_id")}
    latched = bool(brake) or any(value_at(event, "data", "control", "force_brake_latched") is True for event in agent_pauses)
    exposed = bool(force_applied)
    return {
        "applicable": arm == "force_resume",
        "force_applied": exposed,
        "force_latched": latched,
        "question_submitted": bool(questions),
        "question_answered": bool(answers),
        "explicit_resume": bool(resumes) or bool(agent_resumes),
        "monitor_task_ids": sorted(task_ids),
        "task_changed": len(task_ids) > 1,
        "exposure_status": "UNEXPOSED" if arm == "force_resume" and not exposed else (
            "EXPOSED_LATCHED" if exposed and latched else "EXPOSED_NO_LATCH" if exposed else "NOT_APPLICABLE"),
        "source": "interactionlog.json plus agent human_paused/human_resumed events",
    }


def geometry_contact(physical, evidence):
    crossing = value_at(evidence, "doorway_actual_far_jamb")
    stop = value_at(evidence, "actual_elevator_stop")
    contacts = value_at(physical, "contacts", "positive_non_support")
    crossing_ok = isinstance(crossing, dict) and crossing.get("status") == "OBSERVED"
    contact_free = isinstance(contacts, dict) and contacts.get("distinct_physics_steps") == 0
    clearance = value_at(stop, "three_circle_front_edge_to_front_plane_signed_m")
    heading = value_at(stop, "heading_error_to_door_normal_deg")
    lateral = value_at(stop, "lateral_offset_to_actual_leaf_patch_m")
    waiting = value_at(stop, "contract_comparison_geometry_only", "source", "waiting_zone") or {}
    stop_ok = (range_contains(clearance, waiting.get("front_clearance_m"))
               and isinstance(heading, (int, float)) and heading <= waiting.get("heading_tolerance_deg", float("-inf"))
               and isinstance(lateral, (int, float)) and lateral <= waiting.get("lateral_tolerance_m", float("-inf")))
    status = "OBSERVED" if isinstance(physical, dict) and isinstance(evidence, dict) else "NOT_OBSERVED"
    return {
        "status": status,
        "full_body_out_actual_far_jamb": crossing_ok if status == "OBSERVED" else None,
        "positive_non_support_contact": (not contact_free) if isinstance(contacts, dict) else None,
        "contact_free": contact_free if isinstance(contacts, dict) else None,
        "terminal_clearance_m": clearance,
        "terminal_heading_error_deg": heading,
        "terminal_lateral_offset_actual_leaf_m": lateral,
        "terminal_waiting_geometry": stop_ok if isinstance(stop, dict) else None,
        "independent_geometry_and_contact_conditions": crossing_ok and contact_free and stop_ok if status == "OBSERVED" else None,
        "definition": "Physical doorway crossing, non-support contact absence, and actual-leaf waiting geometry are independent recorded conditions; this is not a semantic success label.",
    }


def failure_phase(terminal, stage1_entered, inter):
    if terminal is None:
        return "RECORDER_OR_AGENT_TERMINAL_MISSING"
    if terminal.get("status") == "COMPLETED":
        return None
    if inter["applicable"] and inter["exposure_status"] == "UNEXPOSED":
        return "PRE_FORCE_UNEXPOSED"
    return "STAGE2_OR_LATER" if stage1_entered else "STAGE0"


def summarize_episode(batch: Path, trial: dict):
    episode = batch / trial["id"]
    summary_path = episode / "summary.json"
    base = {
        "trial_id": trial["id"], "pose_id": trial["pose_id"], "arm": trial["arm"],
        "requested_spawn_pose_xy_yaw_deg": trial["pose_xy_yaw_deg"], "episode": str(episode),
    }
    if not summary_path.exists():
        return {**base, "record_status": "PENDING", "missing_artifacts": ["summary.json"]}
    summary = read_json(summary_path)
    events_path = episode / "agent.jsonl"
    events = read_jsonl(events_path) if events_path.exists() else []
    interaction_path = episode / "interactionlog.json"
    interaction_rows = read_json(interaction_path) if interaction_path.exists() else []
    physical_path = episode / "physical_evaluation.json"
    evidence_path = episode / "v2_evidence.json"
    physical = read_json(physical_path) if physical_path.exists() else None
    evidence = read_json(evidence_path) if evidence_path.exists() else None
    raw_output = value_at(summary, "final", "output")
    raw_log = Path(raw_output) / "lightnav.jsonl" if isinstance(raw_output, str) else None
    raw_rows = read_jsonl(raw_log) if raw_log is not None and raw_log.exists() else []
    missing_artifacts = [name for name, present in (("agent.jsonl", bool(events_path.exists())),
                                                     ("physical_evaluation.json", physical is not None),
                                                     ("v2_evidence.json", evidence is not None),
                                                     ("summary.final.output/lightnav.jsonl", raw_log is not None and raw_log.exists())) if not present]
    terminal = terminal_event(events)
    stage1_entered = any(value_at(event, "data", "currentStage") == 1 for event in events if event.get("kind") == "stage_advanced")
    qwen = value_at(evidence, "qwen") if isinstance(evidence, dict) else None
    qwen = qwen if isinstance(qwen, dict) and qwen.get("status") == "OBSERVED" else qwen_from_events(events, events_path.exists())
    inter = interaction(events, interaction_rows, interaction_path.exists(), trial["arm"])
    geom = geometry_contact(physical, evidence)
    target = target_anchor(events, physical, evidence)
    stage2 = lightnav_stage2(raw_rows, events)
    return {
        **base,
        "record_status": "RECORDED" if not missing_artifacts else "RECORDED_INCOMPLETE",
        "missing_artifacts": missing_artifacts,
        "before_actual_pose_world": value_at(summary, "before", "pose_world"),
        "recorder_exit_code": summary.get("exit_code"),
        "recorder_error": summary.get("recorder_error"),
        "walltime_s": summary.get("elapsed_wall_s"),
        "server_final_state": value_at(summary, "final", "state"),
        "server_final_reason": value_at(summary, "final", "reason"),
        "agent_terminal": terminal,
        "agent_task_outcome": terminal.get("status") if terminal else None,
        "stage1_entered": stage1_entered,
        "geometry_contact": geom,
        "target_roi_actual_leaf": target,
        "qwen": qwen,
        "interaction": inter,
        "stage2": stage2,
        "failure_phase": failure_phase(terminal, stage1_entered, inter),
        "termination_reason": terminal.get("reason") if terminal else summary.get("recorder_error"),
    }


def count_arm(records):
    total = len(records)
    recorded = [record for record in records if record["record_status"] != "PENDING"]
    complete = [record for record in recorded if record.get("agent_task_outcome") == "COMPLETED"]
    geometry = [record for record in recorded if value_at(record, "geometry_contact", "independent_geometry_and_contact_conditions") is True]
    stage2 = [record for record in recorded if record.get("stage1_entered")]
    exposed = [record for record in recorded if value_at(record, "interaction", "force_applied") is True]
    return {"planned": total, "recorded": len(recorded), "pending": total - len(recorded),
            "task_completed": len(complete), "independent_geometry_and_contact_conditions": len(geometry),
            "stage2_entered": len(stage2), "force_exposed": len(exposed)}


def csv_rows(records):
    rows = []
    for record in records:
        terminal = record.get("agent_terminal") or {}
        geometry = record.get("geometry_contact") or {}
        interaction_data = record.get("interaction") or {}
        stage2 = record.get("stage2") or {}
        target = record.get("target_roi_actual_leaf") or {}
        qwen = record.get("qwen") or {}
        rows.append({
            "trial_id": record["trial_id"], "pose_id": record["pose_id"], "arm": record["arm"],
            "record_status": record["record_status"], "requested_spawn_pose_xy_yaw_deg": short_json(record["requested_spawn_pose_xy_yaw_deg"]),
            "before_actual_pose_world": short_json(record.get("before_actual_pose_world")),
            "agent_task_outcome": record.get("agent_task_outcome"), "agent_terminal_event": terminal.get("event_kind"),
            "server_final_state": record.get("server_final_state"), "stage1_entered": record.get("stage1_entered"),
            "full_body_out_actual_far_jamb": geometry.get("full_body_out_actual_far_jamb"),
            "positive_non_support_contact": geometry.get("positive_non_support_contact"),
            "terminal_clearance_m": geometry.get("terminal_clearance_m"),
            "terminal_heading_error_deg": geometry.get("terminal_heading_error_deg"),
            "target_roi_point_on_actual_leaf": target.get("point_on_actual_leaf"),
            "target_roi_representative_point_world_m": short_json(target.get("roi_representative_point_world_m")),
            "retained_target_reference_point_world_m": short_json(target.get("retained_reference_point_world_m")),
            "qwen_calls": qwen.get("call_count"), "qwen_usage": short_json(qwen.get("usage_sum")),
            "walltime_s": record.get("walltime_s"), "force_exposure": interaction_data.get("exposure_status"),
            "force_latched": interaction_data.get("force_latched"), "question_submitted": interaction_data.get("question_submitted"),
            "explicit_resume": interaction_data.get("explicit_resume"), "task_changed": interaction_data.get("task_changed"),
            "stage2_instruction": stage2.get("instruction"), "stage2_first_capture_pose_world": short_json(stage2.get("first_capture_pose_world")),
            "stage2_first_actual_result": short_json(stage2.get("first_actual_result")), "stage2_session_steps": stage2.get("session_steps"),
            "stage2_raw_response_count": stage2.get("raw_response_count"), "stage2_accepted_response_count": stage2.get("accepted_response_count"),
            "stage2_first_raw_response_discarded": stage2.get("first_raw_response_discarded"),
            "stage2_stop_while_invisible": stage2.get("stop_while_invisible"),
            "stage2_visible_target_point_without_recorded_hint_review": stage2.get("visible_target_point_without_recorded_hint_review"),
            "failure_phase": record.get("failure_phase"), "termination_reason": record.get("termination_reason"),
            "missing_artifacts": "; ".join(record.get("missing_artifacts", [])),
        })
    return rows


def markdown(summary):
    lines = ["# Navigation stress batch summary", "", f"- Batch state: `{summary['batch_state']}`.",
             f"- Execution admission: `{summary['execution_admission']}`.",
             f"- Planned `{summary['planned']}`; recorded `{summary['recorded']}`; pending `{summary['pending']}`.",
             "- Task terminal labels come from agent `task_completed`/`task_stopped`; the server final state is reported separately.",
             "- Geometry/contact conditions and target-ROI-to-actual-leaf comparison are physical/geometric observations, not semantic reliability proof.", "",
             "## Per arm", "", "| Arm | Planned | Recorded | Pending | Task completed | Geometry + contact | Stage 2 | Force exposed |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for arm, counts in summary["per_arm"].items():
        lines.append(f"| {arm} | {counts['planned']} | {counts['recorded']} | {counts['pending']} | {counts['task_completed']} | {counts['independent_geometry_and_contact_conditions']} | {counts['stage2_entered']} | {counts['force_exposed']} |")
    lines.extend(["", "## Conditional observations", "",
                  f"- Force-resume exposure states: `{short_json(summary['force_resume_exposure'])}`.",
                  f"- Recorded stage-2 instructions: `{short_json(summary['stage2_instruction_groups'])}`.",
                  "- Paired rows only describe independently generated runs from the same requested pose; Qwen instructions and physical prefixes were not restored identically, so no pause-causality or paired win claim is made.", "",
                  "## Data availability", ""])
    if summary["execution_admission"] == "NOT_RUN":
        lines.append(f"- No recorder summary exists. The batch failed before an admitted navigation episode; all planned trials remain pending. Recorder reason: `{summary['batch_state_reason']}`.")
    for record in summary["episodes"]:
        if record["record_status"] != "RECORDED":
            lines.append(f"- `{record['trial_id']}`: `{record['record_status']}`; missing `{', '.join(record.get('missing_artifacts', []))}`.")
    if all(record["record_status"] == "RECORDED" for record in summary["episodes"]):
        lines.append("- All recorder and requested offline evaluator artifacts were present.")
    lines.extend(["", "Missing artifacts are never counted as success, zero contacts, absent Qwen calls, or a completed force-resume interaction.", ""])
    return "\n".join(lines)


def write_outputs(batch, summary):
    (batch / "batch_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    rows = csv_rows(summary["episodes"])
    with (batch / "episodes.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["trial_id"])
        writer.writeheader()
        writer.writerows(rows)
    (batch / "batch_summary.md").write_text(markdown(summary), encoding="utf-8")


def evaluate(batch: Path):
    manifest_path = batch / "manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing batch manifest: {manifest_path}")
    manifest = read_json(manifest_path)
    trials = manifest.get("trials")
    if not isinstance(trials, list) or not trials:
        raise ValueError("manifest.json has no trials.")
    batch_state_path = batch / "batch_state.json"
    batch_state = read_json(batch_state_path) if batch_state_path.exists() else {"state": "NOT_STARTED"}
    records = [summarize_episode(batch, trial) for trial in trials]
    no_recorder_summary = not any(record["record_status"] != "PENDING" for record in records)
    execution_admission = "NOT_RUN" if batch_state.get("state") == "FAILED" and no_recorder_summary else "RECORDED_OR_PENDING"
    per_arm = {arm: count_arm([record for record in records if record["arm"] == arm]) for arm in ("primary", "force_resume")}
    exposure = Counter(record.get("interaction", {}).get("exposure_status") for record in records
                       if record.get("interaction", {}).get("applicable"))
    instructions = Counter(record["stage2"].get("instruction") for record in records
                           if record.get("stage2", {}).get("status") == "OBSERVED")
    pairs = []
    by_pose = defaultdict(list)
    for record in records:
        by_pose[record["pose_id"]].append(record)
    for pose_id, members in sorted(by_pose.items()):
        pairs.append({"pose_id": pose_id, "requested_spawn_pose_xy_yaw_deg": members[0]["requested_spawn_pose_xy_yaw_deg"],
                      "arms": [{"arm": item["arm"], "record_status": item["record_status"],
                                "agent_task_outcome": item.get("agent_task_outcome"), "failure_phase": item.get("failure_phase")}
                               for item in sorted(members, key=lambda item: item["arm"])],
                      "interpretation": "Same requested pose only; independent Qwen decisions and no exact physical-prefix restoration."})
    summary = {
        "reducer": "prototype/sim/evaluate_navigation_stress.py", "batch": str(batch),
        "manifest_version": manifest.get("version"), "batch_state": batch_state.get("state", "NOT_OBSERVED"),
        "batch_state_reason": batch_state.get("reason"), "execution_admission": execution_admission,
        "planned": len(records), "recorded": sum(record["record_status"] != "PENDING" for record in records),
        "pending": sum(record["record_status"] == "PENDING" for record in records), "per_arm": per_arm,
        "force_resume_exposure": dict(exposure), "stage2_instruction_groups": dict(instructions),
        "pairing_limit": manifest.get("pairing_limit"), "episodes": records, "paired_same_requested_pose": pairs,
        "limitations": ["The reducer reports explicit saved events and offline geometry only; it does not infer causal failure from model explanations.",
                        "A server PAUSED/LOCAL_DONE state is not substituted for an agent task terminal label.",
                        "Unavailable metrics remain unknown; planned and recorded denominators stay visible and unknowns are never counted as successes."],
    }
    write_outputs(batch, summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", required=True, type=Path, help="Batch directory containing manifest.json")
    args = parser.parse_args()
    summary = evaluate(args.batch.resolve())
    print(json.dumps({"batch": summary["batch"], "planned": summary["planned"], "recorded": summary["recorded"],
                      "pending": summary["pending"], "outputs": ["batch_summary.json", "episodes.csv", "batch_summary.md"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
