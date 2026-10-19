"""Bounded offline evidence summary for a room_elevator navigation-v2 episode.

The evaluator reads saved episode artifacts only.  It neither starts MuJoCo/Pi/
LightNav nor calls a model.  Geometry is offline truth and is never evidence
that the model semantically identified the target.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
EPS = 1e-8


def read_json(path):
    return json.loads(path.read_text())


def read_jsonl(path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def missing(reason):
    return {"status": "NOT_OBSERVED", "reason": reason}


def source_path(episode, snapshot_name, live_name):
    for candidate in (episode / "source_snapshot" / snapshot_name,
                      episode / "source_snapshot" / "pi" / snapshot_name,
                      ROOT / live_name):
        if candidate.exists():
            return candidate, "source_snapshot" if candidate.is_relative_to(episode / "source_snapshot") else "live_fallback"
    return None, "missing"


def scene_geometry(path):
    scene = ET.parse(path).getroot()

    def box(body_name, geom_name):
        body = scene.find(f"./worldbody/body[@name='{body_name}']")
        geom = body.find(f"./geom[@name='{geom_name}']")
        body_pos = [float(value) for value in body.get("pos", "0 0 0").split()]
        geom_pos = [float(value) for value in geom.get("pos", "0 0 0").split()]
        center = [a + b for a, b in zip(body_pos, geom_pos)]
        halfsize = [float(value) for value in geom.attrib["size"].split()]
        return {"geom": geom_name, "center_m": center, "halfsize_m": halfsize}

    west = box("room_exit_surround", "room_exit_west_jamb")
    east = box("room_exit_surround", "room_exit_east_jamb")
    leaves = [box("elevator_doors", name) for name in ("elevator_left_leaf", "elevator_right_leaf")]
    faces = [{"geom": leaf["geom"], "front_plane_x_m": leaf["center_m"][0] - leaf["halfsize_m"][0],
              "y_interval_m": [leaf["center_m"][1] - leaf["halfsize_m"][1],
                               leaf["center_m"][1] + leaf["halfsize_m"][1]]}
             for leaf in leaves]
    return {"doorway": {"far_jamb_plane_y_m": max(west["center_m"][1] + west["halfsize_m"][1],
                                                     east["center_m"][1] + east["halfsize_m"][1]),
                         "jambs": [west, east]}, "elevator_faces": faces}


def rotate_centers(pose, footprint):
    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [(x + cx * cos_yaw - cy * sin_yaw, y + cx * sin_yaw + cy * cos_yaw, radius)
            for (cx, cy), radius in zip(footprint["centers_base_m"], footprint["radii_m"])]


def far_jamb_clearance(pose, footprint, far_plane_y):
    return min(y - radius for _, y, radius in rotate_centers(pose, footprint)) - far_plane_y


def full_far_jamb_crossing(rows, footprint, far_plane_y):
    if not rows:
        return missing("No saved physical trajectory samples.")
    clearances = [far_jamb_clearance(row["pose_world"], footprint, far_plane_y) for row in rows]
    index = next((i for i in range(1, len(rows)) if clearances[i - 1] < 0 <= clearances[i]), None)
    if index is None:
        if clearances[0] >= 0:
            return {"status": "OBSERVED", "already_clear_at_window_start": True,
                    "first_observed_sim_time_s": rows[0]["sim_time_s"], "clearance_m": clearances[0]}
        return {"status": "NOT_OBSERVED", "reason": "No saved sample has all three circles beyond the actual far jamb plane.",
                "final_clearance_m": clearances[-1]}
    return {"status": "OBSERVED", "definition": "All three recorded footprint circles have their trailing y edge beyond the far (+world_y) jamb face.",
            "first_observed_sim_time_s": rows[index]["sim_time_s"],
            "observation_bracket_s": [rows[index - 1]["sim_time_s"], rows[index]["sim_time_s"]],
            "previous_clearance_m": clearances[index - 1], "observed_clearance_m": clearances[index]}


def stage_zero_evidence(events, footprint, far_plane_y, physical_rows):
    truth_at = {round(row['sim_time_s'], 8): row for row in physical_rows}
    completed = []
    for event in events:
        if event.get("kind") != "stage_completed" or event.get("data", {}).get("stageIndex") != 0:
            continue
        control = event["data"].get("control")
        if not control:
            completed.append({"seq": event.get("seq"), "status": "NOT_OBSERVED", "reason": "stage_completed has no saved control pose."})
            continue
        control = truth_at[round(control['sim_time_s'], 8)]
        clearance = far_jamb_clearance(control["pose_world"], footprint, far_plane_y)
        completed.append({"seq": event.get("seq"), "sim_time_s": control["sim_time_s"],
                          "pose_world": control["pose_world"], "far_jamb_clearance_m": clearance,
                          "already_fully_clear_of_actual_far_jamb": clearance >= 0})
    return {"status": "OBSERVED", "stage0_completed": completed} if completed else missing("No stage-0 completion event was saved.")


def wrap_deg(angle):
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))


def elevator_stop(final_pose, footprint, geometry, contract):
    faces = geometry["elevator_faces"]
    plane_x = faces[0]["front_plane_x_m"]
    base_x, base_y, yaw = final_pose
    choices = []
    for face in faces:
        low, high = face["y_interval_m"]
        projected_y = min(max(base_y, low), high)
        choices.append((abs(base_y - projected_y), face, projected_y))
    _, face, projection_y = min(choices, key=lambda item: item[0])
    circles = rotate_centers(final_pose, footprint)
    leading_index, leading = max(enumerate(circles), key=lambda item: item[1][0] + item[1][2])
    base_clearance = plane_x - base_x
    footprint_clearance = plane_x - (leading[0] + leading[2])
    heading_error = abs(wrap_deg(yaw))  # The true elevator face is approached along +world_x.
    lateral_offset = abs(base_y - projection_y)
    waiting = contract.get("waiting_zone", {})
    return {"status": "OBSERVED", "definition": "Actual room_elevator leaf front face from saved scene XML; approaching direction is +world_x.",
            "final_pose_world": final_pose, "actual_front_plane_x_m": plane_x,
            "actual_door_face_projection_world_xy_m": [plane_x, projection_y], "projected_leaf_geom": face["geom"],
            "projected_leaf_y_interval_m": face["y_interval_m"], "base_to_front_plane_signed_m": base_clearance,
            "three_circle_front_edge_to_front_plane_signed_m": footprint_clearance,
            "leading_circle_index": leading_index, "leading_circle_center_world_xy_m": leading[:2],
            "heading_error_to_door_normal_deg": heading_error, "lateral_offset_to_actual_leaf_patch_m": lateral_offset,
            "contract_comparison_geometry_only": {"source": contract,
                "actual_front_clearance_vs_contract_range": [footprint_clearance, waiting.get("front_clearance_m")],
                "actual_heading_error_vs_contract_deg": [heading_error, waiting.get("heading_tolerance_deg")],
                "actual_leaf_projection_lateral_offset_vs_observed_patch_m": [lateral_offset, waiting.get("lateral_tolerance_m")],
                "limitation": "The online lateral value is relative to the selected observed target-patch plane centroid. This offline value is relative to the actual finite leaf range; neither establishes visual target semantics."}}


def latest_before(items, seq):
    prior = [item for item in items if item["seq"] < seq]
    return prior[-1] if prior else None


def recursive_buffer(value):
    if not isinstance(value, dict):
        return False
    return "buffer" in value or any(recursive_buffer(child) for child in value.values() if isinstance(child, dict))


def decision_evidence(events):
    actions = []
    motion_by_task = {}
    archives = []
    for event in events:
        data = event.get("data", {})
        if event.get("kind") == "action_executed" and isinstance(data.get("buffer"), dict):
            decision = data["buffer"].get("decision", {})
            if decision.get("controlTaskId"):
                actions.append({"seq": event["seq"], "buffer": data["buffer"], "control_task_id": decision["controlTaskId"]})
        if event.get("kind") == "decision_buffer_updated" and isinstance(data.get("motion"), dict):
            motion_by_task[data["motion"].get("control_task_id")] = data["motion"]
        if event.get("kind") == "decision_segment_archived" and isinstance(data, dict):
            decision = data.get("decision", {})
            if decision.get("controlTaskId"):
                motion_by_task[decision["controlTaskId"]] = data.get("motion")
                archives.append({"seq": event["seq"], "control_task_id": decision["controlTaskId"], "end_reason": data.get("endReason")})
    contexts = []
    buffer_contexts = []
    for event in events:
        if event.get("kind") != "decision_context":
            continue
        context = event.get("data", {}).get("context", {})
        buffer = context.get("buffer") if isinstance(context, dict) else None
        if not isinstance(buffer, dict):
            contexts.append({"seq": event["seq"], "purpose": event.get("data", {}).get("purpose"),
                             "buffer_present": False})
            continue
        decision = buffer.get("decision", {})
        current = decision.get("controlTaskId")
        expected = latest_before(actions, event["seq"])
        entry = {"seq": event["seq"], "purpose": event.get("data", {}).get("purpose"), "buffer_present": True,
                 "control_task_id": current, "previous_accepted_control_task_id": expected and expected["control_task_id"],
                 "matches_most_recent_accepted_action": bool(expected and current == expected["control_task_id"]),
                 "contains_recursive_buffer": recursive_buffer(buffer)}
        contexts.append(entry)
        buffer_contexts.append(entry)
    if not actions and not buffer_contexts:
        return missing("No v2 action_executed.buffer or decision_context.context.buffer was saved.")
    non_motion = {"planning": {"plan_task"}, "measurement": {"review_measurements"}, "dialogue": {"answer_user", "dialogue"}}
    return {"status": "OBSERVED", "accepted_action_buffers": [{"seq": item["seq"], "control_task_id": item["control_task_id"],
                                                                       "buffer_id": item["buffer"].get("id"), "motion_saved": item["control_task_id"] in motion_by_task}
                                                                      for item in actions],
            "decision_contexts": contexts, "contexts_with_buffer": buffer_contexts, "archived_old_buffers": archives,
            "planning_measurement_dialogue_buffer_references": {name: [item for item in contexts if item["purpose"] in purposes]
                                                                  for name, purposes in non_motion.items()},
            "motion_by_control_task_id": motion_by_task}


def image_and_qwen_evidence(events, action_buffers):
    requests = [event for event in events if event.get("kind") == "decision_request"]
    proposals = [event for event in events if event.get("kind") == "proposal"]
    if not requests:
        return {"images": missing("No decision_request records."), "qwen": missing("No decision_request records.")}
    regular = []
    roi_counter = Counter()
    for event in requests:
        data = event.get("data", {})
        kinds = [image.get("kind") for image in data.get("images", [])]
        for kind in kinds:
            if kind in ("crop", "layers", "selection"):
                roi_counter[kind] += 1
        purpose = data.get("purpose")
        if purpose in {"stage_decision", "verify_arrival", "recover"}:
            prior = latest_before(action_buffers, event["seq"])
            regular.append({"seq": event["seq"], "purpose": purpose, "image_kinds": kinds,
                            "has_current_rgb": "rgb" in kinds, "has_stage_entry": "stage_entry" in kinds,
                            "prior_action_buffer_exists": prior is not None,
                            "has_previous_decision_when_prior_exists": prior is None or "previous_decision" in kinds})
    recalls = [event["seq"] for event in proposals if event.get("data", {}).get("result", {}).get("name") == "recall_memory"]
    usage_records = [event.get("data", {}).get("result", {}).get("usage") for event in proposals]
    usage_records = [usage for usage in usage_records if isinstance(usage, dict)]
    numeric_usage = defaultdict(float)
    for usage in usage_records:
        for key in ("input", "output", "cacheRead", "cacheWrite", "reasoning", "totalTokens"):
            if isinstance(usage.get(key), (int, float)):
                numeric_usage[key] += usage[key]
    peak = max(requests, key=lambda event: event.get("data", {}).get("textChars", -1))
    return {"images": {"status": "OBSERVED", "regular_decision_image_sets": regular,
                        "roi_extra_image_kind_counts": dict(roi_counter), "recall_memory_proposal_seqs": recalls,
                        "definition": "Image kinds come from the raw decision_request; ROI images and recall_memory calls are counted separately."},
            "qwen": {"status": "OBSERVED", "call_count": len(requests),
                     "usage_sum": dict(numeric_usage) if usage_records else missing("proposal.result.usage was not recorded."),
                     "input_context_peak": {"seq": peak["seq"], "text_chars": peak.get("data", {}).get("textChars"),
                                            "tools_json_chars": peak.get("data", {}).get("toolsJsonChars"),
                                            "image_count": peak.get("data", {}).get("imageCount")}}}


def raw_motion_evidence(raw_output, actions, motion_by_task, finish):
    telemetry = read_jsonl(raw_output / "ground_truth.jsonl") if raw_output.exists() else []
    if not telemetry:
        return missing("Offline ground_truth.jsonl is unavailable.")
    result = []
    for index, action in enumerate(actions):
        control_task_id = action["control_task_id"]
        decision = action["buffer"].get("decision", {})
        start = decision.get("acceptedStampS")
        motion = motion_by_task.get(control_task_id)
        end = (motion or {}).get("end_stamp_s")
        if end is None:
            next_action = actions[index + 1] if index + 1 < len(actions) else None
            end = next_action["buffer"].get("decision", {}).get("acceptedStampS") if next_action else finish
        samples = [row for row in telemetry if start is not None and start - EPS <= row["sim_time_s"] <= end + EPS]
        if len(samples) < 2:
            result.append({"control_task_id": control_task_id, **missing("Fewer than two raw 50 Hz samples in the motion interval.")})
            continue
        raw_length = sum(math.hypot(b["pose_world"][0] - a["pose_world"][0], b["pose_world"][1] - a["pose_world"][1])
                         for a, b in zip(samples, samples[1:]))
        actual = (motion or {}).get("estimated", {})
        result.append({"status": "OBSERVED", "control_task_id": control_task_id, "interval_s": [start, end],
                       "raw_50hz_sample_count": len(samples), "raw_50hz_path_length_m": raw_length,
                       "motion_summary_sample_count": actual.get("sample_count"),
                       "motion_summary_path_length_m": actual.get("path_length_m"),
                       "summary_minus_raw_path_length_m": (actual.get("path_length_m") - raw_length
                                                             if isinstance(actual.get("path_length_m"), (int, float)) else None),
                       "lightnav_plan_count": (motion or {}).get("plan_count"),
                       "limitation": "Summary is sensor-estimated odometry; raw length is independent offline truth. Their difference is estimation error, not a truth input. LightNav plans are not traveled distance."})
    return {"status": "OBSERVED", "per_action": result}


def lightnav_evidence(raw_output):
    rows = read_jsonl(raw_output / "lightnav.jsonl") if raw_output.exists() else []
    rows = [row for row in rows if row.get("navigation_scope") is not None]
    if not rows:
        return missing("No raw LightNav rows with navigation_scope were saved.")
    scopes = defaultdict(list)
    for row in rows:
        scopes[row["navigation_scope"]].append(row)
    scope_rows = [{"navigation_scope": scope, "session_ids": sorted({row.get("navigation_session_id") for row in values}),
                   "raw_response_count": len(values), "discarded_response_count": sum(bool(row.get("discarded")) for row in values)}
                  for scope, values in scopes.items()]
    transitions = []
    previous_scope = rows[0]["navigation_scope"]
    previous_session = rows[0].get("navigation_session_id")
    for row in rows[1:]:
        if row["navigation_scope"] != previous_scope:
            transitions.append({"from_scope": previous_scope, "to_scope": row["navigation_scope"],
                                "previous_session_id": previous_session, "new_scope_session_id": row.get("navigation_session_id"),
                                "session_changed_after_scope_change": previous_session != row.get("navigation_session_id"),
                                "raw_row_discarded": bool(row.get("discarded"))})
        previous_scope, previous_session = row["navigation_scope"], row.get("navigation_session_id")
    return {"status": "OBSERVED", "per_scope": scope_rows, "scope_change_observations": transitions,
            "discarded_rows_total": sum(bool(row.get("discarded")) for row in rows),
            "definition": "Raw server lightnav.jsonl metadata. A transition can only establish reset when a response for the new scope was recorded."}


def contact_reference(physical):
    contacts = physical.get("contacts", {})
    if not contacts:
        return missing("physical_evaluation.json has no contact section.")
    return {"status": "OBSERVED", "source": "physical_evaluation.json (referenced; not recomputed here)",
            "positive_non_support": contacts.get("positive_non_support"),
            "positive_non_ground": contacts.get("positive_non_ground")}


def load_physical(episode):
    path = episode / "physical_evaluation.json"
    if path.exists():
        return read_json(path), "existing"
    from evaluate_monitor import evaluate
    physical = evaluate(episode)
    path.write_text(json.dumps(physical, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return physical, "generated_by_evaluate_monitor"


def markdown(result):
    crossing = result["doorway_actual_far_jamb"]
    stop = result["actual_elevator_stop"]
    qwen = result["qwen"]
    lines = ["# Navigation v2 offline evidence", "",
             f"- Physical report: `{result['physical_report']['path']}` ({result['physical_report']['provenance']}).",
             f"- Actual far-jamb clearance: `{crossing['status']}`; first observed time: `{crossing.get('first_observed_sim_time_s', 'NOT_OBSERVED')}`.",
             f"- Actual elevator stop geometry: `{stop['status']}`; three-circle front clearance: `{stop.get('three_circle_front_edge_to_front_plane_signed_m', 'NOT_OBSERVED')}` m; heading error: `{stop.get('heading_error_to_door_normal_deg', 'NOT_OBSERVED')}` degrees.",
             f"- Qwen calls: `{qwen.get('call_count', 'NOT_OBSERVED')}`. Contact result is referenced from physical_evaluation.json.", "",
             "These are saved-log and offline-geometry observations. They do not establish that the model semantically recognized the doorway or elevator, and `COMPLETED` is not converted into a semantic PASS."]
    return "\n".join(lines) + "\n"


def evaluate(episode):
    physical, provenance = load_physical(episode)
    if physical.get("scene") != "room_elevator":
        raise ValueError("evaluate_navigation_v2.py evaluates room_elevator episodes only.")
    scene_path, scene_source = source_path(episode, "scenes/room_elevator.xml", "prototype/sim/scenes/room_elevator.xml")
    contract_path, contract_source = source_path(episode, "navigation-contract.json", "prototype/pi/navigation-contract.json")
    if scene_path is None or contract_path is None:
        raise FileNotFoundError("Saved scene or navigation-contract source is required for v2 offline evidence.")
    geometry = scene_geometry(scene_path)
    contract = read_json(contract_path)
    footprint = physical["runtime_recorded_footprint"]
    trajectory = physical["trajectory"]["samples"]
    rows = [{"sim_time_s": sample[0], "pose_world": sample[1:]} for sample in trajectory]
    events = read_jsonl(episode / "agent.jsonl")
    decisions = decision_evidence(events)
    actions = [] if decisions.get("status") == "NOT_OBSERVED" else [
        {"seq": entry["seq"], "buffer": next(item["buffer"] for item in [
            {"seq": event["seq"], "buffer": event["data"]["buffer"]} for event in events
            if event.get("kind") == "action_executed" and event["seq"] == entry["seq"]]) , "control_task_id": entry["control_task_id"]}
        for entry in decisions["accepted_action_buffers"]]
    raw_output = Path(physical["raw_server_output"])
    return {"episode": str(episode), "physical_report": {"path": str(episode / "physical_evaluation.json"), "provenance": provenance},
            "source_selection": {"scene": {"path": str(scene_path), "provenance": scene_source},
                                 "navigation_contract": {"path": str(contract_path), "provenance": contract_source}},
            "doorway_actual_far_jamb": full_far_jamb_crossing(rows, footprint, geometry["doorway"]["far_jamb_plane_y_m"]),
            "stage0_actual_far_jamb": stage_zero_evidence(events, footprint, geometry["doorway"]["far_jamb_plane_y_m"], rows),
            "actual_elevator_stop": elevator_stop(rows[-1]["pose_world"], footprint, geometry, contract),
            "decision_buffer": {key: value for key, value in decisions.items() if key != "motion_by_control_task_id"},
            "motion_50hz_vs_summary": raw_motion_evidence(raw_output, actions, decisions.get("motion_by_control_task_id", {}), rows[-1]["sim_time_s"]),
            "lightnav_sessions": lightnav_evidence(raw_output),
            "contacts_reference": contact_reference(physical),
            **image_and_qwen_evidence(events, actions),
            "semantic_limit": "Offline geometry, action logs, and monitor terminal labels do not prove visual target semantics. No COMPLETED or geometric match is reported as a semantic PASS."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    args = parser.parse_args()
    episode = args.episode.resolve()
    result = evaluate(episode)
    output = episode / "v2_evidence.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    report = episode / "v2_evidence.md"
    report.write_text(markdown(result))
    print(output)
    print(report)


if __name__ == "__main__":
    main()
