#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from datetime import datetime
import math
import sys
import time
from pathlib import Path

import numpy as np

from model_loader import load_mujoco_model_from_path
from runtime import (
    apply_control_targets,
    apply_joint_actuator_gains,
    base_forward_axis_xy,
    build_command_vector,
    build_target_joint_positions,
    clear_body_forces,
    configure_tracking_viewer_camera,
    create_model_bindings,
    default_smart_gait_metadata,
    draw_velocity_arrows,
    infer_policy_outputs,
    initialize_runtime_state,
    load_policy_metadata,
    load_torchscript_policy,
    parse_triplet,
    reset_model_state,
    resolve_body_id,
    set_body_force,
    summarize_joint_actuator_gains,
    update_runtime_state_after_step,
    world_to_base_yaw,
    yaw_from_quat,
)

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import mujoco.viewer as mujoco_viewer
except ImportError:
    mujoco_viewer = None


DEFAULT_DURATION_S = 60.0
BASE_FORCE_SCALE = 0.01
BASE_FORCE_KD = 160.0
LIN_VEL_X_CLIP = 0.1
LIN_VEL_Y_CLIP = 0.1
ANG_VEL_YAW_CLIP = 0.2
BRAKE_GATE_FORCE_X_THRESHOLD_N = -32.0
BRAKE_GATE_MIN_CMD_VX = 0.2
BRAKE_GATE_MAX_CMD_WZ = 0.10
BRAKE_GATE_HOLD_STEPS = 2

FORCE_SCHEDULE_NONE = "none"
FORCE_SCHEDULE_DEMO = "demo"
FORCE_SCHEDULE_STOP_GO = "stop_go"
FORCE_SCHEDULE_HUMAN_GUIDE = "human_guide"

FORCE_INTERP_LINEAR = "linear"
FORCE_INTERP_QUADRATIC = "quadratic"

DIR_NONE = "none"
DIR_FORWARD = "forward"
DIR_LEFT = "left"
DIR_BACKWARD = "backward"

MODE_STANDING = 0
MODE_FORCE_WALKING = 1
MODE_COMMAND_WALKING = 2

MODE_NAME_BY_ID = {
    MODE_STANDING: "standing",
    MODE_FORCE_WALKING: "force_walking",
    MODE_COMMAND_WALKING: "command_walking",
}

BRAKE_GATE_ELIGIBLE_PHASES = frozenset(
    {
        "walk_backward_pull",
        "walk_backward_ramp",
    }
)

STOP_GO_PHASE_IDLE = "idle"
STOP_GO_PHASE_PRE_TENSION = "pre_tension"
STOP_GO_PHASE_START_PULL = "start_pull"
STOP_GO_PHASE_STEADY_PULL = "steady_pull"
STOP_GO_PHASE_STOP_CUE = "stop_cue"
STOP_GO_PHASE_SLACK_RECOVERY = "slack_recovery"
STOP_GO_PHASE_POST_IDLE = "post_idle"

HUMAN_GUIDE_PHASE_FORWARD_PULL = "forward_pull"
HUMAN_GUIDE_PHASE_RIGHT_TURN = "right_turn"
HUMAN_GUIDE_PHASE_RIGHT_PULL = "right_pull"
HUMAN_GUIDE_PHASE_GAP = "gap"
HUMAN_GUIDE_PHASE_BACKWARD_PULL = "backward_pull"
HUMAN_GUIDE_PHASE_LEFT_TURN = "left_turn"
HUMAN_GUIDE_PHASE_LEFT_PULL = "left_pull"
HUMAN_GUIDE_PHASE_TAIL = "tail_idle"
HUMAN_GUIDE_PHASE_WALK_CMD_FORWARD = "walk_cmd_forward"
HUMAN_GUIDE_PHASE_WALK_CMD_GAP = "walk_cmd_gap"
HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL = "walk_backward_pull"
HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP = "walk_backward_ramp"
HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN = "walk_right_turn"
HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH = "walk_right_pull_high"
HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE = "walk_right_pull_settle"
HUMAN_GUIDE_PHASE_WALK_LEFT_TURN = "walk_left_turn"
HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH = "walk_left_pull_high"
HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE = "walk_left_pull_settle"

STOP_GO_HEADING_TOKEN_TO_DEG = {
    "forward": 0.0,
    "front": 0.0,
    "f": 0.0,
    "left": 90.0,
    "l": 90.0,
    "backward": 180.0,
    "back": 180.0,
    "b": 180.0,
    "right": -90.0,
    "r": -90.0,
    "forward_left": 45.0,
    "front_left": 45.0,
    "fl": 45.0,
    "forward_right": -45.0,
    "front_right": -45.0,
    "fr": -45.0,
    "back_left": 135.0,
    "bl": 135.0,
    "back_right": -135.0,
    "br": -135.0,
}

GATE_PRESET_LEGACY = "legacy"
GATE_PRESET_BALANCED = "balanced"
GATE_PRESET_ROBUST = "robust"
GATE_PRESET_V1 = "v1"
GATE_PRESET_V1_ROBUST = "v1_robust"
GATE_PRESET_V1_TURNING = "v1_turning"
GATE_PRESET_V1_ROBUST_TURNING = "v1_robust_turning"
GATE_PRESET_V2 = "v2"
GATE_PRESET_V2_ROBUST = "v2_robust"

GATE_PRESET_OVERRIDES = {
    GATE_PRESET_LEGACY: {},
    GATE_PRESET_BALANCED: {
        "baseline_tau_s": 1.5,
        "baseline_margin_n": 0.25,
        "baseline_update_max_n": 0.8,
        "enter_threshold_n": 2.0,
        "exit_threshold_n": 0.25,
        "enter_tau_on_s": 0.16,
        "enter_tau_off_s": 0.25,
        "exit_tau_on_s": 0.06,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.80,
        "exit_score_threshold": 0.50,
        "dir_consistency_min_force_n": 0.5,
        "dir_consistency_tau_s": 0.10,
        "dir_consistency_threshold": 0.65,
        "switch_cooldown_s": 1.0,
        "enter_hold_s": 0.08,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_ROBUST: {
        "baseline_tau_s": 1.0,
        "baseline_margin_n": 0.35,
        "baseline_update_max_n": 1.2,
        "enter_threshold_n": 2.4,
        "exit_threshold_n": 0.15,
        "enter_tau_on_s": 0.08,
        "enter_tau_off_s": 0.25,
        "exit_tau_on_s": 0.04,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.90,
        "exit_score_threshold": 0.60,
        "dir_consistency_min_force_n": 1.0,
        "dir_consistency_tau_s": 0.20,
        "dir_consistency_threshold": 0.65,
        "switch_cooldown_s": 1.2,
        "enter_hold_s": 0.08,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_V1: {
        "baseline_tau_s": 1.0,
        "baseline_margin_n": 0.34,
        "baseline_update_max_n": 1.00,
        "enter_threshold_n": 2.30,
        "exit_threshold_n": 0.15,
        "enter_tau_on_s": 0.08,
        "enter_tau_off_s": 0.22,
        "exit_tau_on_s": 0.04,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.90,
        "exit_score_threshold": 0.60,
        "dir_consistency_min_force_n": 1.00,
        "dir_consistency_tau_s": 0.16,
        "dir_consistency_threshold": 0.72,
        "switch_cooldown_s": 1.00,
        "enter_hold_s": 0.08,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_V1_ROBUST: {
        "baseline_tau_s": 1.0,
        "baseline_margin_n": 0.35,
        "baseline_update_max_n": 1.15,
        "enter_threshold_n": 2.40,
        "exit_threshold_n": 0.15,
        "enter_tau_on_s": 0.08,
        "enter_tau_off_s": 0.25,
        "exit_tau_on_s": 0.04,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.92,
        "exit_score_threshold": 0.60,
        "dir_consistency_min_force_n": 1.05,
        "dir_consistency_tau_s": 0.18,
        "dir_consistency_threshold": 0.74,
        "switch_cooldown_s": 1.10,
        "enter_hold_s": 0.08,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_V1_TURNING: {
        "baseline_tau_s": 1.2,
        "baseline_margin_n": 0.22,
        "baseline_update_max_n": 0.90,
        "enter_threshold_n": 1.90,
        "exit_threshold_n": 0.18,
        "enter_tau_on_s": 0.10,
        "enter_tau_off_s": 0.22,
        "exit_tau_on_s": 0.05,
        "exit_tau_off_s": 0.09,
        "enter_score_threshold": 0.78,
        "exit_score_threshold": 0.55,
        "dir_consistency_min_force_n": 0.75,
        "dir_consistency_tau_s": 0.14,
        "dir_consistency_threshold": 0.60,
        "switch_cooldown_s": 0.90,
        "enter_hold_s": 0.06,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_V1_ROBUST_TURNING: {
        "baseline_tau_s": 1.3,
        "baseline_margin_n": 0.25,
        "baseline_update_max_n": 1.00,
        "enter_threshold_n": 2.10,
        "exit_threshold_n": 0.18,
        "enter_tau_on_s": 0.12,
        "enter_tau_off_s": 0.24,
        "exit_tau_on_s": 0.05,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.82,
        "exit_score_threshold": 0.58,
        "dir_consistency_min_force_n": 0.80,
        "dir_consistency_tau_s": 0.16,
        "dir_consistency_threshold": 0.62,
        "switch_cooldown_s": 1.00,
        "enter_hold_s": 0.06,
        "exit_hold_s": 0.04,
    },
    # --- v2 presets: tuned for base-only policy (noisier force estimator) ---
    # Strategy: raise evidence_cap to widen dynamic range; real force (20-40N)
    # separates clearly from estimator drift noise (≤6N) in the scoring
    # pipeline, preserving v1-class entry latency.
    GATE_PRESET_V2: {
        "evidence_cap_n": 10.0,
        "baseline_tau_s": 1.0,
        "baseline_margin_n": 0.34,
        "baseline_update_max_n": 1.00,
        "enter_threshold_n": 8.00,
        "exit_threshold_n": 0.15,
        "enter_tau_on_s": 0.08,
        "enter_tau_off_s": 0.22,
        "exit_tau_on_s": 0.04,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.90,
        "exit_score_threshold": 0.60,
        "dir_consistency_min_force_n": 1.00,
        "dir_consistency_tau_s": 0.16,
        "dir_consistency_threshold": 0.72,
        "switch_cooldown_s": 1.00,
        "enter_hold_s": 0.12,
        "exit_hold_s": 0.04,
    },
    GATE_PRESET_V2_ROBUST: {
        "evidence_cap_n": 10.0,
        "baseline_tau_s": 1.2,
        "baseline_margin_n": 0.50,
        "baseline_update_max_n": 1.20,
        "enter_threshold_n": 8.50,
        "exit_threshold_n": 0.15,
        "enter_tau_on_s": 0.12,
        "enter_tau_off_s": 0.22,
        "exit_tau_on_s": 0.04,
        "exit_tau_off_s": 0.10,
        "enter_score_threshold": 0.90,
        "exit_score_threshold": 0.60,
        "dir_consistency_min_force_n": 1.00,
        "dir_consistency_tau_s": 0.18,
        "dir_consistency_threshold": 0.72,
        "switch_cooldown_s": 1.20,
        "enter_hold_s": 0.20,
        "exit_hold_s": 0.04,
    },
}


@dataclass(frozen=True)
class ForceSegment:
    segment_name: str
    direction_name: str
    phase_name: str
    start_s: float
    end_s: float
    force_start_n: float = 0.0
    force_end_n: float = 0.0
    capture_axis: bool = False
    force_interp: str = FORCE_INTERP_LINEAR

    @property
    def is_active(self) -> bool:
        return self.direction_name != DIR_NONE

    def force_target_at(self, time_s: float) -> float:
        if not self.is_active:
            return 0.0
        if self.end_s <= self.start_s or math.isclose(self.force_start_n, self.force_end_n):
            return float(self.force_end_n)
        ratio = (float(time_s) - self.start_s) / (self.end_s - self.start_s)
        ratio = max(0.0, min(ratio, 1.0))
        if self.force_interp == FORCE_INTERP_QUADRATIC:
            ratio = ratio * ratio
        return float(self.force_start_n + ratio * (self.force_end_n - self.force_start_n))


@dataclass
class ForceModeSwitchState:
    mode: int
    command_zero: bool
    force_xy_raw: float
    force_xy_cap: float
    force_baseline: float
    force_excess: float
    dir_consistency: float
    enter_score: float
    exit_score: float
    enter_timer: float
    exit_timer: float
    cooldown_timer: float
    standing_mask: bool
    walking_mask: bool


@dataclass
class ForceGuidanceState:
    enabled: bool
    active: bool
    effective_cmd_vel_b: np.ndarray
    heading_error_rad: float
    estimated_force_heading_rad: float
    estimated_force_mag: float
    motion_sign: float


@dataclass
class BrakeGateState:
    enabled: bool
    eligible: bool
    active: bool
    hold_counter: int
    est_force_x_n: float
    est_force_y_n: float
    threshold_x_n: float
    blocked_phase_name: str | None


@dataclass(frozen=True)
class HumanGuideLegProfile:
    force_freq_hz: tuple[float, float]
    force_phase_rad: tuple[float, float]
    force_weight: float
    heading_freq_hz: tuple[float, float]
    heading_phase_rad: tuple[float, float]
    heading_weight: float
    dropout_start_s: float
    dropout_duration_s: float


@dataclass(frozen=True)
class HumanGuideTurnProfile:
    force_freq_hz: float
    force_phase_rad: float
    heading_freq_hz: float
    heading_phase_rad: float


def parse_heading_deg_sequence(seq_text: str) -> tuple[float, ...]:
    tokens = [token.strip().lower() for token in str(seq_text).split(",") if token.strip()]
    if not tokens:
        raise ValueError("stop-go heading sequence must contain at least one entry.")
    headings_deg: list[float] = []
    for token in tokens:
        if token in STOP_GO_HEADING_TOKEN_TO_DEG:
            headings_deg.append(float(STOP_GO_HEADING_TOKEN_TO_DEG[token]))
            continue
        headings_deg.append(float(token))
    return tuple(headings_deg)


def format_heading_label(heading_deg: float) -> str:
    wrapped_deg = ((float(heading_deg) + 180.0) % 360.0) - 180.0
    if math.isclose(abs(wrapped_deg), 180.0, abs_tol=1.0e-6):
        wrapped_deg = 180.0
    return f"heading_{wrapped_deg:+06.1f}deg"


def mode_name(mode_id: int) -> str:
    return MODE_NAME_BY_ID.get(int(mode_id), f"unknown_{int(mode_id)}")


def smoothstep01(value: float) -> float:
    clamped = max(0.0, min(float(value), 1.0))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def smooth_bump01(value: float) -> float:
    clamped = max(0.0, min(float(value), 1.0))
    if clamped <= 0.5:
        return smoothstep01(clamped / 0.5)
    return smoothstep01((1.0 - clamped) / 0.5)


def lerp_scalar(start: float, end: float, ratio: float) -> float:
    return float(start + (end - start) * float(ratio))


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


def angle_progress(abs_angle_rad: float, *, full_speed_angle_rad: float, turn_only_angle_rad: float) -> float:
    denom = max(float(turn_only_angle_rad) - float(full_speed_angle_rad), 1.0e-6)
    progress = (float(turn_only_angle_rad) - float(abs_angle_rad)) / denom
    return max(0.0, min(progress, 1.0))


def shaped_dropout_scale(elapsed_s: float, start_s: float, duration_s: float) -> float:
    if duration_s <= 0.0:
        return 1.0
    local_time_s = float(elapsed_s) - float(start_s)
    if local_time_s <= 0.0 or local_time_s >= duration_s:
        return 1.0
    half_duration_s = 0.5 * duration_s
    if local_time_s <= half_duration_s:
        return 1.0 - smoothstep01(local_time_s / max(half_duration_s, 1.0e-6))
    return smoothstep01((local_time_s - half_duration_s) / max(half_duration_s, 1.0e-6))


def normalize_xy(vector_xy: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    norm = float(np.linalg.norm(vector_xy))
    if norm <= eps:
        return np.array([1.0, 0.0], dtype=np.float32)
    return (vector_xy / norm).astype(np.float32)


def rotate_xy(vector_xy: np.ndarray, angle_rad: float) -> np.ndarray:
    cos_a = math.cos(float(angle_rad))
    sin_a = math.sin(float(angle_rad))
    x = float(vector_xy[0])
    y = float(vector_xy[1])
    return np.array([cos_a * x - sin_a * y, sin_a * x + cos_a * y], dtype=np.float32)


def direction_axis_from_forward(forward_axis_xy: np.ndarray, direction_name: str) -> np.ndarray:
    if direction_name == DIR_FORWARD:
        return forward_axis_xy.astype(np.float32, copy=True)
    if direction_name == DIR_LEFT:
        return np.array([-forward_axis_xy[1], forward_axis_xy[0]], dtype=np.float32)
    if direction_name == DIR_BACKWARD:
        return (-forward_axis_xy).astype(np.float32)
    return np.zeros(2, dtype=np.float32)


def build_demo_force_segments(duration_s: float) -> tuple[ForceSegment, ...]:
    tail_end = max(float(duration_s), 60.0)
    return (
        ForceSegment("idle_0", DIR_NONE, "idle", 0.0, 5.0),
        ForceSegment("forward_low_10", DIR_FORWARD, "low", 5.0, 6.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("forward_low_15", DIR_FORWARD, "low", 6.0, 7.0, 15.0, 15.0),
        ForceSegment("forward_low_5", DIR_FORWARD, "low", 7.0, 8.0, 5.0, 5.0),
        ForceSegment(
            "forward_ramp_quad",
            DIR_FORWARD,
            "ramp",
            8.0,
            11.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("forward_high", DIR_FORWARD, "high", 11.0, 13.0, 40.0, 40.0),
        ForceSegment("idle_1", DIR_NONE, "idle", 13.0, 20.0),
        ForceSegment("left_low_10", DIR_LEFT, "low", 20.0, 21.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("left_low_15", DIR_LEFT, "low", 21.0, 22.0, 15.0, 15.0),
        ForceSegment("left_low_5", DIR_LEFT, "low", 22.0, 23.0, 5.0, 5.0),
        ForceSegment(
            "left_ramp_quad",
            DIR_LEFT,
            "ramp",
            23.0,
            26.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("left_high", DIR_LEFT, "high", 26.0, 28.0, 40.0, 40.0),
        ForceSegment("idle_2", DIR_NONE, "idle", 28.0, 40.0),
        ForceSegment("backward_low_10", DIR_BACKWARD, "low", 40.0, 41.0, 10.0, 10.0, capture_axis=True),
        ForceSegment("backward_low_15", DIR_BACKWARD, "low", 41.0, 42.0, 15.0, 15.0),
        ForceSegment("backward_low_5", DIR_BACKWARD, "low", 42.0, 43.0, 5.0, 5.0),
        ForceSegment(
            "backward_ramp_quad",
            DIR_BACKWARD,
            "ramp",
            43.0,
            46.0,
            10.0,
            40.0,
            force_interp=FORCE_INTERP_QUADRATIC,
        ),
        ForceSegment("backward_high", DIR_BACKWARD, "high", 46.0, 48.0, 40.0, 40.0),
        ForceSegment("idle_3", DIR_NONE, "idle", 48.0, tail_end),
    )


class ScriptedForceScheduler:
    def __init__(self, segments: tuple[ForceSegment, ...]):
        self.segments = segments
        self.current_segment_name: str | None = None
        self.current_axis_xy = np.full(2, np.nan, dtype=np.float32)

    def segment_for_time(self, time_s: float) -> ForceSegment:
        if time_s >= self.segments[-1].start_s:
            return self.segments[-1]
        for segment in self.segments:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return self.segments[-1]

    def update(self, base_quat_wxyz: np.ndarray, time_s: float) -> dict[str, object]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name
        if segment_changed and segment.capture_axis and segment.is_active:
            forward_axis_xy = base_forward_axis_xy(base_quat_wxyz)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)

        force_target_n = segment.force_target_at(time_s)
        force_w = np.zeros(3, dtype=np.float32)
        axis_xy = np.full(2, np.nan, dtype=np.float32)
        if segment.is_active:
            axis_xy = self.current_axis_xy.copy()
            force_w[:2] = axis_xy * force_target_n

        self.current_segment_name = segment.segment_name
        return {
            "segment_name": segment.segment_name,
            "direction_name": segment.direction_name,
            "phase_name": segment.phase_name,
            "force_schedule_target_n": float(force_target_n),
            "force_target_n": float(force_target_n),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": segment.is_active,
        }


class StopGoTuningForceScheduler:
    def __init__(self, args: argparse.Namespace, *, control_dt: float, headings_deg: tuple[float, ...]):
        self.args = args
        self.control_dt = float(control_dt)
        self.headings_deg = tuple(float(v) for v in headings_deg)
        if not self.headings_deg:
            raise ValueError("stop-go schedule requires at least one heading.")

        self.idle_s = max(float(args.stop_go_idle_s), 0.0)
        self.pre_tension_s = max(float(args.stop_go_pre_tension_s), 0.0)
        self.start_pull_s = max(float(args.stop_go_start_pull_s), 0.0)
        self.steady_s = max(float(args.stop_go_steady_s), 0.0)
        self.stop_cue_s = max(float(args.stop_go_stop_cue_s), 0.0)
        self.slack_s = max(float(args.stop_go_slack_s), 0.0)
        self.post_idle_s = max(float(args.stop_go_post_idle_s), 0.0)
        self.cycle_duration_s = (
            self.idle_s
            + self.pre_tension_s
            + self.start_pull_s
            + self.steady_s
            + self.stop_cue_s
            + self.slack_s
            + self.post_idle_s
        )
        if self.cycle_duration_s <= 1.0e-6:
            raise ValueError("stop-go schedule requires a positive total cycle duration.")

        self.pre_tension_n = max(float(args.stop_go_pre_tension_n), 0.0)
        self.start_pull_n = max(float(args.stop_go_start_pull_n), self.pre_tension_n)
        self.steady_force_n = max(float(args.stop_go_steady_force_n), 0.0)
        self.steady_force_osc_n = max(float(args.stop_go_steady_force_osc_n), 0.0)
        self.stop_cue_n = max(float(args.stop_go_stop_cue_n), 0.0)

        self.pre_tension_heading_drift_rad = math.radians(float(args.stop_go_pre_tension_heading_drift_deg))
        self.start_pull_heading_drift_rad = math.radians(float(args.stop_go_start_pull_heading_drift_deg))
        self.steady_heading_drift_rad = math.radians(float(args.stop_go_steady_heading_drift_deg))

        self.steady_osc_period_s = max(float(args.stop_go_steady_osc_period_s), max(self.control_dt, 1.0e-3))
        self.mag_alpha = self._time_constant_to_alpha(float(args.stop_go_mag_tau_s))
        self.angle_alpha = self._time_constant_to_alpha(float(args.stop_go_angle_tau_s))

        self.mag_state = 0.0
        self.angle_state = 0.0
        self.base_heading_rad = 0.0
        self.reference_forward_xy = np.array([1.0, 0.0], dtype=np.float32)
        self.captured_cycle_idx = -1

    def update(self, base_quat_wxyz: np.ndarray, time_s: float) -> dict[str, object]:
        cycle_idx, heading_deg, phase_name, phase_elapsed_s, phase_duration_s = self._phase_info(time_s)
        phase_active = phase_name not in {STOP_GO_PHASE_IDLE, STOP_GO_PHASE_POST_IDLE}

        if phase_active and cycle_idx != self.captured_cycle_idx:
            self.reference_forward_xy = base_forward_axis_xy(base_quat_wxyz)
            self.base_heading_rad = math.radians(heading_deg)
            self.captured_cycle_idx = cycle_idx

        scheduled_force_target_n = self._force_target_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
        drift_target_rad = self._heading_drift_for_phase(phase_name, phase_elapsed_s, phase_duration_s)

        self.mag_state = self.mag_state + self.mag_alpha * (scheduled_force_target_n - self.mag_state)
        self.angle_state = self.angle_state + self.angle_alpha * (drift_target_rad - self.angle_state)

        axis_xy = rotate_xy(self.reference_forward_xy, self.base_heading_rad + self.angle_state)
        force_w = np.zeros(3, dtype=np.float32)
        force_w[:2] = axis_xy * self.mag_state

        is_active = phase_active or self.mag_state > 1.0e-4
        if not is_active:
            axis_xy = np.full(2, np.nan, dtype=np.float32)

        return {
            "segment_name": f"cycle_{cycle_idx:02d}",
            "direction_name": format_heading_label(heading_deg),
            "phase_name": phase_name,
            "force_schedule_target_n": float(scheduled_force_target_n),
            "force_target_n": float(self.mag_state),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": bool(is_active),
        }

    def _phase_info(self, time_s: float) -> tuple[int, float, str, float, float]:
        cycle_idx = max(int(math.floor(float(time_s) / self.cycle_duration_s)), 0)
        cycle_time_s = float(time_s) - cycle_idx * self.cycle_duration_s
        heading_deg = self.headings_deg[cycle_idx % len(self.headings_deg)]

        boundaries = (
            (STOP_GO_PHASE_IDLE, self.idle_s),
            (STOP_GO_PHASE_PRE_TENSION, self.pre_tension_s),
            (STOP_GO_PHASE_START_PULL, self.start_pull_s),
            (STOP_GO_PHASE_STEADY_PULL, self.steady_s),
            (STOP_GO_PHASE_STOP_CUE, self.stop_cue_s),
            (STOP_GO_PHASE_SLACK_RECOVERY, self.slack_s),
            (STOP_GO_PHASE_POST_IDLE, self.post_idle_s),
        )

        elapsed_before = 0.0
        for phase_name, phase_duration_s in boundaries:
            phase_end = elapsed_before + phase_duration_s
            if cycle_time_s < phase_end or math.isclose(cycle_time_s, phase_end):
                return cycle_idx, heading_deg, phase_name, max(cycle_time_s - elapsed_before, 0.0), phase_duration_s
            elapsed_before = phase_end
        return cycle_idx, heading_deg, STOP_GO_PHASE_POST_IDLE, 0.0, self.post_idle_s

    def _force_target_for_phase(self, phase_name: str, phase_elapsed_s: float, phase_duration_s: float) -> float:
        progress = phase_elapsed_s / max(phase_duration_s, 1.0e-6)
        smooth = smoothstep01(progress)
        if phase_name == STOP_GO_PHASE_IDLE:
            return 0.0
        if phase_name == STOP_GO_PHASE_PRE_TENSION:
            return self.pre_tension_n * smooth
        if phase_name == STOP_GO_PHASE_START_PULL:
            return self.pre_tension_n + (self.start_pull_n - self.pre_tension_n) * smooth
        if phase_name == STOP_GO_PHASE_STEADY_PULL:
            settle_ratio = phase_elapsed_s / max(min(self.steady_s * 0.35, self.steady_s), 1.0e-6)
            settle = smoothstep01(settle_ratio)
            osc = math.sin((2.0 * math.pi * phase_elapsed_s) / self.steady_osc_period_s)
            base_target = (1.0 - settle) * self.start_pull_n + settle * self.steady_force_n
            return max(base_target + settle * self.steady_force_osc_n * osc, 0.0)
        if phase_name == STOP_GO_PHASE_STOP_CUE:
            steady_end = self._steady_force_target_at(self.steady_s)
            return steady_end + (self.stop_cue_n - steady_end) * smooth
        if phase_name == STOP_GO_PHASE_SLACK_RECOVERY:
            return self.stop_cue_n * (1.0 - smooth)
        return 0.0

    def _heading_drift_for_phase(self, phase_name: str, phase_elapsed_s: float, phase_duration_s: float) -> float:
        progress = phase_elapsed_s / max(phase_duration_s, 1.0e-6)
        if phase_name == STOP_GO_PHASE_PRE_TENSION:
            return self.pre_tension_heading_drift_rad * math.sin(math.pi * progress)
        if phase_name == STOP_GO_PHASE_START_PULL:
            return self.start_pull_heading_drift_rad * math.sin(math.pi * progress)
        if phase_name == STOP_GO_PHASE_STEADY_PULL:
            return self.steady_heading_drift_rad * math.sin((2.0 * math.pi * phase_elapsed_s) / self.steady_osc_period_s)
        if phase_name == STOP_GO_PHASE_STOP_CUE:
            steady_end = self._steady_heading_target_at(self.steady_s)
            return steady_end * (1.0 - smoothstep01(progress))
        return 0.0

    def _steady_force_target_at(self, elapsed_s: float) -> float:
        settle_ratio = elapsed_s / max(min(self.steady_s * 0.35, self.steady_s), 1.0e-6)
        settle = smoothstep01(settle_ratio)
        osc = math.sin((2.0 * math.pi * elapsed_s) / self.steady_osc_period_s)
        base_target = (1.0 - settle) * self.start_pull_n + settle * self.steady_force_n
        return max(base_target + settle * self.steady_force_osc_n * osc, 0.0)

    def _steady_heading_target_at(self, elapsed_s: float) -> float:
        return self.steady_heading_drift_rad * math.sin((2.0 * math.pi * elapsed_s) / self.steady_osc_period_s)

    def _time_constant_to_alpha(self, tau_seconds: float) -> float:
        tau = max(float(tau_seconds), 0.0)
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-self.control_dt / tau)


class HumanGuideForceScheduler:
    _ACTIVE_PHASES = {
        HUMAN_GUIDE_PHASE_FORWARD_PULL,
        HUMAN_GUIDE_PHASE_RIGHT_TURN,
        HUMAN_GUIDE_PHASE_RIGHT_PULL,
        HUMAN_GUIDE_PHASE_BACKWARD_PULL,
        HUMAN_GUIDE_PHASE_LEFT_TURN,
        HUMAN_GUIDE_PHASE_LEFT_PULL,
        HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL,
        HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE,
        HUMAN_GUIDE_PHASE_WALK_LEFT_TURN,
        HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH,
        HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE,
    }
    _PHASE_TO_BLOCK_IDX = {
        HUMAN_GUIDE_PHASE_FORWARD_PULL: 0,
        HUMAN_GUIDE_PHASE_RIGHT_TURN: 0,
        HUMAN_GUIDE_PHASE_RIGHT_PULL: 0,
        HUMAN_GUIDE_PHASE_BACKWARD_PULL: 1,
        HUMAN_GUIDE_PHASE_LEFT_TURN: 1,
        HUMAN_GUIDE_PHASE_LEFT_PULL: 1,
        HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL: 2,
        HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP: 2,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN: 2,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH: 2,
        HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE: 2,
        HUMAN_GUIDE_PHASE_WALK_LEFT_TURN: 2,
        HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH: 2,
        HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE: 2,
    }
    _BLOCK_SEGMENT_NAMES = {
        0: "human_guide_block_00",
        1: "human_guide_block_01",
        2: "human_guide_block_02",
    }
    _TAIL_FORWARD_CMD_VX = 0.80
    _TAIL_TURN_RATE_RAD_S = math.radians(18.0)
    _TAIL_FORCE_HEADING_LAG_S = 0.45
    _TAIL_FORCE_LOW_N = 15.0
    _TAIL_FORCE_MID_N = 30.0
    _TAIL_FORCE_HIGH_N = 40.0

    def __init__(self, args: argparse.Namespace, *, control_dt: float):
        self.args = args
        self.control_dt = float(control_dt)
        self.default_cmd_vel_b = np.array([float(args.cmd_vx), float(args.cmd_vy), float(args.cmd_wz)], dtype=np.float32)

        self.straight_s = max(float(args.human_guide_straight_s), 0.0)
        self.turn_s = max(float(args.human_guide_turn_s), 0.0)
        self.gap_s = max(float(args.human_guide_gap_s), 0.0)
        if self.straight_s <= 0.0:
            raise ValueError("human-guide schedule requires --human-guide-straight-s > 0.")
        if self.turn_s <= 0.0:
            raise ValueError("human-guide schedule requires --human-guide-turn-s > 0.")

        self.base_force_n = max(float(args.human_guide_base_force_n), 0.0)
        self.force_jitter_n = max(float(args.human_guide_force_jitter_n), 0.0)
        self.turn_force_jitter_n = 0.35 * self.force_jitter_n
        self.heading_jitter_rad = math.radians(max(float(args.human_guide_heading_jitter_deg), 0.0))
        self.turn_heading_jitter_rad = 0.35 * self.heading_jitter_rad
        self.dropout_max_s = max(float(args.human_guide_dropout_max_s), 0.0)

        self.mag_alpha = self._time_constant_to_alpha(float(args.stop_go_mag_tau_s))
        self.angle_alpha = self._time_constant_to_alpha(float(args.stop_go_angle_tau_s))

        self.mag_state = 0.0
        self.heading_state = 0.0
        self.reference_forward_xy = np.array([1.0, 0.0], dtype=np.float32)
        self.captured_block_idx = -1

        core_phase_durations = [
            (HUMAN_GUIDE_PHASE_FORWARD_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_RIGHT_TURN, self.turn_s),
            (HUMAN_GUIDE_PHASE_RIGHT_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_GAP, self.gap_s),
            (HUMAN_GUIDE_PHASE_BACKWARD_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_LEFT_TURN, self.turn_s),
            (HUMAN_GUIDE_PHASE_LEFT_PULL, self.straight_s),
        ]
        self.core_duration_s = sum(duration_s for _, duration_s in core_phase_durations)

        self.walk_tail_cmd_start_s = max(self.core_duration_s + 5.0, 33.0)
        self.walk_tail_force_start_s = max(self.core_duration_s + 7.0, 35.0)
        self.walk_tail_pre_idle_s = max(self.walk_tail_force_start_s - self.core_duration_s, 0.0)
        self.walk_tail_backward_pull_s = 3.0
        self.walk_tail_backward_ramp_s = 2.0
        self.walk_tail_turn_s = 5.0
        self.walk_tail_high_force_s = 3.0
        self.walk_tail_final_bump_s = 5.0
        self.walk_tail_settle_s = 0.0
        tail_force_end_s = (
            self.walk_tail_force_start_s
            + self.walk_tail_backward_pull_s
            + self.walk_tail_backward_ramp_s
            + self.walk_tail_turn_s
            + self.walk_tail_high_force_s
            + self.walk_tail_settle_s
            + self.walk_tail_turn_s
            + self.walk_tail_final_bump_s
        )
        self.walk_tail_final_hold_s = max(float(args.duration_s) - tail_force_end_s, 0.0)

        self.walk_tail_ramp_start_s = self.walk_tail_force_start_s + self.walk_tail_backward_pull_s
        self.walk_tail_right_turn_start_s = self.walk_tail_ramp_start_s + self.walk_tail_backward_ramp_s
        self.walk_tail_right_turn_end_s = self.walk_tail_right_turn_start_s + self.walk_tail_turn_s
        self.walk_tail_right_high_start_s = self.walk_tail_right_turn_end_s
        self.walk_tail_right_high_end_s = self.walk_tail_right_high_start_s + self.walk_tail_high_force_s
        self.walk_tail_left_turn_start_s = self.walk_tail_right_high_end_s + self.walk_tail_settle_s
        self.walk_tail_left_turn_end_s = self.walk_tail_left_turn_start_s + self.walk_tail_turn_s
        self.walk_tail_left_high_start_s = self.walk_tail_left_turn_end_s
        self.walk_tail_left_high_end_s = self.walk_tail_left_high_start_s + self.walk_tail_final_bump_s

        phase_durations: list[tuple[str, float]] = list(core_phase_durations)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_TAIL, self.walk_tail_pre_idle_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL, self.walk_tail_backward_pull_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP, self.walk_tail_backward_ramp_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN, self.walk_tail_turn_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH, self.walk_tail_high_force_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE, self.walk_tail_settle_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_LEFT_TURN, self.walk_tail_turn_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH, self.walk_tail_final_bump_s)
        self._append_phase(phase_durations, HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE, self.walk_tail_final_hold_s)
        self.phase_durations = tuple(phase_durations)
        self.total_duration_s = sum(duration_s for _, duration_s in self.phase_durations)

        self.leg_profiles = {
            HUMAN_GUIDE_PHASE_FORWARD_PULL: self._build_leg_profile(seed_offset=11),
            HUMAN_GUIDE_PHASE_RIGHT_PULL: self._build_leg_profile(seed_offset=23),
            HUMAN_GUIDE_PHASE_BACKWARD_PULL: self._build_leg_profile(seed_offset=37),
            HUMAN_GUIDE_PHASE_LEFT_PULL: self._build_leg_profile(seed_offset=53),
        }
        self.turn_profiles = {
            HUMAN_GUIDE_PHASE_RIGHT_TURN: self._build_turn_profile(seed_offset=71),
            HUMAN_GUIDE_PHASE_LEFT_TURN: self._build_turn_profile(seed_offset=89),
        }
        self.walk_backward_profile = self._build_leg_profile(
            seed_offset=101,
            phase_duration_s=self.walk_tail_backward_pull_s + self.walk_tail_backward_ramp_s,
        )
        self.walk_right_profile = self._build_leg_profile(
            seed_offset=131,
            phase_duration_s=self.walk_tail_turn_s + self.walk_tail_high_force_s + self.walk_tail_settle_s,
        )
        self.walk_left_profile = self._build_leg_profile(
            seed_offset=149,
            phase_duration_s=self.walk_tail_turn_s + self.walk_tail_final_bump_s + max(self.walk_tail_final_hold_s, 2.0),
        )

    @staticmethod
    def _append_phase(phase_durations: list[tuple[str, float]], phase_name: str, duration_s: float) -> None:
        if duration_s > 1.0e-6:
            phase_durations.append((phase_name, float(duration_s)))

    def update(self, base_quat_wxyz: np.ndarray, time_s: float) -> dict[str, object]:
        phase_name, phase_elapsed_s, phase_duration_s = self._phase_info(time_s)
        block_idx = self._PHASE_TO_BLOCK_IDX.get(phase_name, None)

        if block_idx is not None and block_idx != self.captured_block_idx:
            self.reference_forward_xy = base_forward_axis_xy(base_quat_wxyz)
            heading_start_deg = self._nominal_heading_deg_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
            if phase_name in {
                HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL,
                HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP,
                HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN,
                HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH,
                HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE,
                HUMAN_GUIDE_PHASE_WALK_LEFT_TURN,
                HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH,
                HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE,
            }:
                heading_start_deg = self._walk_tail_heading_deg(time_s)
            self.heading_state = math.radians(heading_start_deg)
            self.captured_block_idx = block_idx

        scheduled_force_target_n, heading_target_rad = self._targets_for_phase(
            phase_name,
            phase_elapsed_s,
            phase_duration_s,
            time_s=time_s,
        )
        self.mag_state = self.mag_state + self.mag_alpha * (scheduled_force_target_n - self.mag_state)
        if heading_target_rad is not None:
            self.heading_state = self.heading_state + self.angle_alpha * (heading_target_rad - self.heading_state)

        phase_active = phase_name in self._ACTIVE_PHASES
        is_active = phase_active or self.mag_state > 1.0e-4

        force_w = np.zeros(3, dtype=np.float32)
        axis_xy = np.full(2, np.nan, dtype=np.float32)
        if is_active:
            axis_xy = rotate_xy(self.reference_forward_xy, self.heading_state)
            force_w[:2] = axis_xy * self.mag_state

        direction_name = DIR_NONE
        if phase_active:
            direction_name = format_heading_label(math.degrees(self.heading_state))

        return {
            "segment_name": self._segment_name_for_phase(phase_name),
            "direction_name": direction_name,
            "phase_name": phase_name,
            "force_schedule_target_n": float(scheduled_force_target_n),
            "force_target_n": float(self.mag_state),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": bool(is_active),
            "nominal_cmd_vel_b": self._nominal_command_for_time(time_s),
        }

    def _phase_info(self, time_s: float) -> tuple[str, float, float]:
        elapsed_before_s = 0.0
        for phase_name, phase_duration_s in self.phase_durations:
            phase_end_s = elapsed_before_s + phase_duration_s
            if float(time_s) < phase_end_s or math.isclose(float(time_s), phase_end_s):
                return phase_name, max(float(time_s) - elapsed_before_s, 0.0), phase_duration_s
            elapsed_before_s = phase_end_s
        return HUMAN_GUIDE_PHASE_TAIL, max(float(time_s) - self.total_duration_s, 0.0), 0.0

    def _segment_name_for_phase(self, phase_name: str) -> str:
        block_idx = self._PHASE_TO_BLOCK_IDX.get(phase_name, None)
        if block_idx is not None:
            return self._BLOCK_SEGMENT_NAMES[block_idx]
        if phase_name in {HUMAN_GUIDE_PHASE_WALK_CMD_FORWARD, HUMAN_GUIDE_PHASE_WALK_CMD_GAP}:
            return "human_guide_walk_cmd"
        if phase_name == HUMAN_GUIDE_PHASE_GAP:
            return "human_guide_gap"
        return "human_guide_tail"

    def _targets_for_phase(
        self,
        phase_name: str,
        phase_elapsed_s: float,
        phase_duration_s: float,
        *,
        time_s: float,
    ) -> tuple[float, float | None]:
        if phase_name in self.leg_profiles:
            profile = self.leg_profiles[phase_name]
            force_target_n = self._straight_force_target(profile, phase_elapsed_s)
            heading_target_deg = self._nominal_heading_deg_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
            heading_target_deg += math.degrees(self.heading_jitter_rad) * self._profile_wave(
                phase_elapsed_s,
                profile.heading_freq_hz,
                profile.heading_phase_rad,
                profile.heading_weight,
            )
            return force_target_n, math.radians(heading_target_deg)

        if phase_name in self.turn_profiles:
            profile = self.turn_profiles[phase_name]
            min_force_n = 0.55 * self.base_force_n
            force_target_n = self.base_force_n + self.turn_force_jitter_n * math.sin(
                (2.0 * math.pi * profile.force_freq_hz * phase_elapsed_s) + profile.force_phase_rad
            )
            force_target_n = max(force_target_n, min_force_n)
            heading_target_deg = self._nominal_heading_deg_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
            heading_target_deg += math.degrees(self.turn_heading_jitter_rad) * math.sin(
                (2.0 * math.pi * profile.heading_freq_hz * phase_elapsed_s) + profile.heading_phase_rad
            )
            return force_target_n, math.radians(heading_target_deg)

        if phase_name in {
            HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL,
            HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP,
            HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN,
            HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH,
            HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE,
            HUMAN_GUIDE_PHASE_WALK_LEFT_TURN,
            HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_HIGH,
            HUMAN_GUIDE_PHASE_WALK_LEFT_PULL_SETTLE,
        }:
            profile, stage_elapsed_s = self._walk_tail_profile_and_elapsed(phase_name, time_s)
            base_force_n = self._walk_tail_base_force_n(time_s)
            force_target_n = self._profile_force_target(profile, stage_elapsed_s, base_force_n=base_force_n)
            heading_target_deg = self._walk_tail_heading_deg(time_s)
            heading_target_deg += math.degrees(self.heading_jitter_rad) * self._profile_wave(
                stage_elapsed_s,
                profile.heading_freq_hz,
                profile.heading_phase_rad,
                profile.heading_weight,
            )
            return force_target_n, math.radians(heading_target_deg)

        return 0.0, None

    def _straight_force_target(self, profile: HumanGuideLegProfile, elapsed_s: float) -> float:
        return self._profile_force_target(profile, elapsed_s, base_force_n=self.base_force_n)

    def _profile_force_target(self, profile: HumanGuideLegProfile, elapsed_s: float, *, base_force_n: float) -> float:
        force_target_n = base_force_n + self.force_jitter_n * self._profile_wave(
            elapsed_s,
            profile.force_freq_hz,
            profile.force_phase_rad,
            profile.force_weight,
        )
        force_target_n = max(force_target_n, 0.0)
        return force_target_n * shaped_dropout_scale(
            elapsed_s,
            profile.dropout_start_s,
            profile.dropout_duration_s,
        )

    def _nominal_command_for_time(self, time_s: float) -> np.ndarray:
        if float(time_s) < self.walk_tail_cmd_start_s:
            return self.default_cmd_vel_b.astype(np.float32, copy=True)
        if float(time_s) >= self.walk_tail_left_high_end_s:
            return np.zeros(3, dtype=np.float32)

        command = np.array([self._TAIL_FORWARD_CMD_VX, 0.0, 0.0], dtype=np.float32)
        if self.walk_tail_right_turn_start_s <= float(time_s) < self.walk_tail_right_turn_end_s:
            command[2] = -self._TAIL_TURN_RATE_RAD_S
        elif self.walk_tail_left_turn_start_s <= float(time_s) < self.walk_tail_left_turn_end_s:
            command[2] = self._TAIL_TURN_RATE_RAD_S
        return command

    def _nominal_heading_deg_for_phase(self, phase_name: str, phase_elapsed_s: float, phase_duration_s: float) -> float:
        progress = phase_elapsed_s / max(phase_duration_s, 1.0e-6)
        if phase_name == HUMAN_GUIDE_PHASE_FORWARD_PULL:
            return 0.0
        if phase_name == HUMAN_GUIDE_PHASE_RIGHT_TURN:
            return lerp_scalar(0.0, -90.0, smoothstep01(progress))
        if phase_name == HUMAN_GUIDE_PHASE_RIGHT_PULL:
            return -90.0
        if phase_name == HUMAN_GUIDE_PHASE_BACKWARD_PULL:
            return 180.0
        if phase_name == HUMAN_GUIDE_PHASE_LEFT_TURN:
            return lerp_scalar(180.0, 90.0, smoothstep01(progress))
        if phase_name == HUMAN_GUIDE_PHASE_LEFT_PULL:
            return 90.0
        return 0.0

    @staticmethod
    def _profile_wave(
        elapsed_s: float,
        freqs_hz: tuple[float, float],
        phases_rad: tuple[float, float],
        primary_weight: float,
    ) -> float:
        secondary_weight = 1.0 - primary_weight
        value = (
            primary_weight * math.sin((2.0 * math.pi * freqs_hz[0] * elapsed_s) + phases_rad[0])
            + secondary_weight * math.sin((2.0 * math.pi * freqs_hz[1] * elapsed_s) + phases_rad[1])
        )
        return max(-1.0, min(1.0, value))

    def _walk_tail_profile_and_elapsed(self, phase_name: str, time_s: float) -> tuple[HumanGuideLegProfile, float]:
        if phase_name in {HUMAN_GUIDE_PHASE_WALK_BACKWARD_PULL, HUMAN_GUIDE_PHASE_WALK_BACKWARD_RAMP}:
            return self.walk_backward_profile, max(float(time_s) - self.walk_tail_force_start_s, 0.0)
        if phase_name in {
            HUMAN_GUIDE_PHASE_WALK_RIGHT_TURN,
            HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_HIGH,
            HUMAN_GUIDE_PHASE_WALK_RIGHT_PULL_SETTLE,
        }:
            return self.walk_right_profile, max(float(time_s) - self.walk_tail_right_turn_start_s, 0.0)
        return self.walk_left_profile, max(float(time_s) - self.walk_tail_left_turn_start_s, 0.0)

    def _walk_tail_base_force_n(self, time_s: float) -> float:
        if float(time_s) < self.walk_tail_force_start_s:
            return 0.0
        if float(time_s) < self.walk_tail_ramp_start_s:
            return self._TAIL_FORCE_LOW_N
        if float(time_s) < self.walk_tail_right_turn_start_s:
            progress = (float(time_s) - self.walk_tail_ramp_start_s) / max(self.walk_tail_backward_ramp_s, 1.0e-6)
            return lerp_scalar(self._TAIL_FORCE_LOW_N, self._TAIL_FORCE_MID_N, smoothstep01(progress))
        if float(time_s) < self.walk_tail_right_high_start_s:
            return self._TAIL_FORCE_MID_N
        if float(time_s) < self.walk_tail_right_high_end_s:
            progress = (float(time_s) - self.walk_tail_right_high_start_s) / max(self.walk_tail_high_force_s, 1.0e-6)
            return self._TAIL_FORCE_MID_N + (self._TAIL_FORCE_HIGH_N - self._TAIL_FORCE_MID_N) * smooth_bump01(progress)
        if float(time_s) < self.walk_tail_left_turn_start_s:
            return self._TAIL_FORCE_MID_N
        if float(time_s) < self.walk_tail_left_high_start_s:
            return self._TAIL_FORCE_MID_N
        if float(time_s) < self.walk_tail_left_high_end_s:
            bump_duration_s = min(self.walk_tail_high_force_s, self.walk_tail_final_bump_s)
            bump_end_s = self.walk_tail_left_high_start_s + bump_duration_s
            if float(time_s) < bump_end_s:
                progress = (float(time_s) - self.walk_tail_left_high_start_s) / max(bump_duration_s, 1.0e-6)
                return self._TAIL_FORCE_MID_N + (self._TAIL_FORCE_HIGH_N - self._TAIL_FORCE_MID_N) * smooth_bump01(progress)
            decay_duration_s = max(self.walk_tail_final_bump_s - bump_duration_s, 1.0e-6)
            progress = (float(time_s) - bump_end_s) / decay_duration_s
            return lerp_scalar(self._TAIL_FORCE_MID_N, 0.0, smoothstep01(progress))
        return 0.0

    def _walk_tail_heading_deg(self, time_s: float) -> float:
        delayed_time_s = float(time_s) - self._TAIL_FORCE_HEADING_LAG_S
        if delayed_time_s < self.walk_tail_right_turn_start_s:
            return 180.0
        if delayed_time_s < self.walk_tail_right_turn_end_s:
            progress = (delayed_time_s - self.walk_tail_right_turn_start_s) / max(self.walk_tail_turn_s, 1.0e-6)
            return lerp_scalar(180.0, 90.0, smoothstep01(progress))
        if delayed_time_s < self.walk_tail_left_turn_start_s:
            return 90.0
        if delayed_time_s < self.walk_tail_left_turn_end_s:
            progress = (delayed_time_s - self.walk_tail_left_turn_start_s) / max(self.walk_tail_turn_s, 1.0e-6)
            return lerp_scalar(90.0, 180.0, smoothstep01(progress))
        return 180.0

    def _build_leg_profile(self, seed_offset: int, *, phase_duration_s: float | None = None) -> HumanGuideLegProfile:
        rng = np.random.default_rng(int(self.args.seed) + int(seed_offset))
        duration_s = self.straight_s if phase_duration_s is None else max(float(phase_duration_s), 0.0)
        boundary_margin_s = min(max(0.65, 0.16 * duration_s), 0.30 * duration_s)
        max_dropout_s = min(self.dropout_max_s, max(duration_s - (2.0 * boundary_margin_s), 0.0))
        if max_dropout_s <= 1.0e-6:
            dropout_start_s = 0.0
            dropout_duration_s = 0.0
        else:
            min_dropout_s = min(0.12, max_dropout_s)
            dropout_duration_s = self._sample_uniform(rng, min_dropout_s, max_dropout_s)
            start_hi_s = duration_s - boundary_margin_s - dropout_duration_s
            if start_hi_s <= boundary_margin_s:
                dropout_start_s = 0.0
                dropout_duration_s = 0.0
            else:
                dropout_start_s = self._sample_uniform(rng, boundary_margin_s, start_hi_s)

        return HumanGuideLegProfile(
            force_freq_hz=(
                self._sample_uniform(rng, 0.15, 0.32),
                self._sample_uniform(rng, 0.34, 0.58),
            ),
            force_phase_rad=(
                self._sample_uniform(rng, 0.0, 2.0 * math.pi),
                self._sample_uniform(rng, 0.0, 2.0 * math.pi),
            ),
            force_weight=self._sample_uniform(rng, 0.35, 0.65),
            heading_freq_hz=(
                self._sample_uniform(rng, 0.18, 0.38),
                self._sample_uniform(rng, 0.42, 0.72),
            ),
            heading_phase_rad=(
                self._sample_uniform(rng, 0.0, 2.0 * math.pi),
                self._sample_uniform(rng, 0.0, 2.0 * math.pi),
            ),
            heading_weight=self._sample_uniform(rng, 0.35, 0.65),
            dropout_start_s=dropout_start_s,
            dropout_duration_s=dropout_duration_s,
        )

    def _build_turn_profile(self, seed_offset: int) -> HumanGuideTurnProfile:
        rng = np.random.default_rng(int(self.args.seed) + int(seed_offset))
        return HumanGuideTurnProfile(
            force_freq_hz=self._sample_uniform(rng, 0.20, 0.45),
            force_phase_rad=self._sample_uniform(rng, 0.0, 2.0 * math.pi),
            heading_freq_hz=self._sample_uniform(rng, 0.25, 0.55),
            heading_phase_rad=self._sample_uniform(rng, 0.0, 2.0 * math.pi),
        )

    @staticmethod
    def _sample_uniform(rng: np.random.Generator, lower: float, upper: float) -> float:
        if upper <= lower:
            return float(lower)
        return float(rng.uniform(lower, upper))

    def _time_constant_to_alpha(self, tau_seconds: float) -> float:
        tau = max(float(tau_seconds), 0.0)
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-self.control_dt / tau)


class ForceModeSwitcher:
    def __init__(self, args: argparse.Namespace):
        self.lin_vel_x_clip = float(args.lin_vel_x_clip)
        self.lin_vel_y_clip = float(args.lin_vel_y_clip)
        self.ang_vel_yaw_clip = float(args.ang_vel_yaw_clip)
        self.baseline_tau_s = float(args.baseline_tau_s)
        self.baseline_margin_n = float(args.baseline_margin_n)
        self.baseline_update_max_n = float(args.baseline_update_max_n)
        self.evidence_cap_n = float(args.evidence_cap_n)
        self.enter_th_n = float(args.enter_threshold_n)
        self.exit_th_n = float(args.exit_threshold_n)
        self.enter_tau_on_s = float(args.enter_tau_on_s)
        self.enter_tau_off_s = float(args.enter_tau_off_s)
        self.exit_tau_on_s = float(args.exit_tau_on_s)
        self.exit_tau_off_s = float(args.exit_tau_off_s)
        self.enter_score_threshold = float(args.enter_score_threshold)
        self.exit_score_threshold = float(args.exit_score_threshold)
        self.dir_consistency_min_force_n = float(args.dir_consistency_min_force_n)
        self.dir_consistency_tau_s = float(args.dir_consistency_tau_s)
        self.dir_consistency_threshold = float(args.dir_consistency_threshold)
        self.switch_cooldown_s = float(args.switch_cooldown_s)
        self.enter_hold_s = float(args.enter_hold_s)
        self.exit_hold_s = float(args.exit_hold_s)

        self.mode = MODE_STANDING
        self.force_state = MODE_STANDING
        self.force_baseline = 0.0
        self.dir_consistency = 1.0
        self.enter_score = 0.0
        self.exit_score = 0.0
        self.enter_timer = 0.0
        self.exit_timer = 0.0
        self.cooldown_timer = 0.0
        self.prev_force_dir_xy = np.array([1.0, 0.0], dtype=np.float32)
        self.prev_force_dir_valid = False

    def classify_command_zero(self, cmd_vel_b: np.ndarray) -> bool:
        return (
            abs(float(cmd_vel_b[0])) < self.lin_vel_x_clip
            and abs(float(cmd_vel_b[1])) < self.lin_vel_y_clip
            and abs(float(cmd_vel_b[2])) < self.ang_vel_yaw_clip
        )

    @staticmethod
    def _alpha(dt: float, tau_s: float) -> float:
        return float(dt) / max(float(tau_s) + float(dt), 1.0e-8)

    @staticmethod
    def _update_score(score: float, target: float, *, dt: float, tau_on_s: float, tau_off_s: float) -> float:
        alpha_on = ForceModeSwitcher._alpha(dt, tau_on_s)
        alpha_off = ForceModeSwitcher._alpha(dt, tau_off_s)
        alpha = alpha_on if target >= score else alpha_off
        updated = float(score + alpha * (target - score))
        return max(0.0, min(updated, 1.0))

    def update(self, cmd_vel_b: np.ndarray, base_force_est_local: np.ndarray, *, dt: float) -> ForceModeSwitchState:
        dt = float(dt)
        command_zero = self.classify_command_zero(cmd_vel_b)
        self.cooldown_timer = max(self.cooldown_timer - dt, 0.0)

        force_xy_raw = float(np.linalg.norm(base_force_est_local[:2]))
        if not math.isfinite(force_xy_raw):
            force_xy_raw = 0.0
        force_xy_cap = max(0.0, min(force_xy_raw, self.evidence_cap_n))

        force_dir_xy = normalize_xy(base_force_est_local[:2])
        valid_force_dir = force_xy_raw >= self.dir_consistency_min_force_n
        dir_alignment = 1.0
        if valid_force_dir and self.prev_force_dir_valid:
            dir_alignment = max(0.0, min(float(np.dot(force_dir_xy, self.prev_force_dir_xy)), 1.0))
        dir_target = dir_alignment if valid_force_dir else 1.0
        dir_alpha = self._alpha(dt, self.dir_consistency_tau_s)
        self.dir_consistency = self.dir_consistency + dir_alpha * (dir_target - self.dir_consistency)
        if valid_force_dir:
            self.prev_force_dir_xy = force_dir_xy
        self.prev_force_dir_valid = valid_force_dir

        if not command_zero:
            self.mode = MODE_COMMAND_WALKING
            self.force_state = MODE_STANDING
            self.dir_consistency = 1.0
            self.enter_score = 0.0
            self.exit_score = 0.0
            self.enter_timer = 0.0
            self.exit_timer = 0.0
            self.cooldown_timer = 0.0
            self.prev_force_dir_valid = False
        else:
            if (
                self.force_state == MODE_STANDING
                and self.cooldown_timer <= 0.0
                and force_xy_cap <= (self.force_baseline + self.baseline_update_max_n)
            ):
                baseline_alpha = self._alpha(dt, self.baseline_tau_s)
                self.force_baseline = self.force_baseline + baseline_alpha * (force_xy_cap - self.force_baseline)

            force_excess = max(force_xy_cap - self.force_baseline - self.baseline_margin_n, 0.0)
            enter_force_target = max(0.0, min(force_excess / max(self.enter_th_n, 1.0e-6), 1.0))
            dir_term = max(
                0.0,
                min(
                    (self.dir_consistency - self.dir_consistency_threshold)
                    / max(1.0 - self.dir_consistency_threshold, 1.0e-6),
                    1.0,
                ),
            )
            enter_target = enter_force_target * dir_term
            exit_target = 1.0 - max(0.0, min(force_excess / max(self.exit_th_n, 1.0e-6), 1.0))

            self.enter_score = self._update_score(
                self.enter_score,
                enter_target,
                dt=dt,
                tau_on_s=self.enter_tau_on_s,
                tau_off_s=self.enter_tau_off_s,
            )
            self.exit_score = self._update_score(
                self.exit_score,
                exit_target,
                dt=dt,
                tau_on_s=self.exit_tau_on_s,
                tau_off_s=self.exit_tau_off_s,
            )

            if self.force_state == MODE_STANDING:
                self.exit_timer = 0.0
                if self.cooldown_timer > 0.0:
                    self.enter_timer = 0.0
                else:
                    if self.enter_score >= self.enter_score_threshold:
                        self.enter_timer += dt
                    else:
                        self.enter_timer = 0.0
                if self.enter_timer >= self.enter_hold_s:
                    self.force_state = MODE_FORCE_WALKING
                    self.enter_score = 1.0
                    self.exit_score = 0.0
                    self.enter_timer = 0.0
                    self.exit_timer = 0.0
                    self.cooldown_timer = self.switch_cooldown_s
            else:
                self.enter_timer = 0.0
                if self.cooldown_timer > 0.0:
                    self.exit_timer = 0.0
                else:
                    if self.exit_score >= self.exit_score_threshold:
                        self.exit_timer += dt
                    else:
                        self.exit_timer = 0.0
                if self.exit_timer >= self.exit_hold_s:
                    self.force_state = MODE_STANDING
                    self.enter_score = 0.0
                    self.exit_score = 1.0
                    self.enter_timer = 0.0
                    self.exit_timer = 0.0
                    self.cooldown_timer = self.switch_cooldown_s

        force_excess = max(force_xy_cap - self.force_baseline - self.baseline_margin_n, 0.0)
        force_walking_mask = command_zero and self.force_state == MODE_FORCE_WALKING
        standing_mask = command_zero and not force_walking_mask
        walking_mask = not standing_mask

        if standing_mask:
            self.mode = MODE_STANDING
        elif force_walking_mask:
            self.mode = MODE_FORCE_WALKING

        return ForceModeSwitchState(
            mode=self.mode,
            command_zero=command_zero,
            force_xy_raw=force_xy_raw,
            force_xy_cap=force_xy_cap,
            force_baseline=self.force_baseline,
            force_excess=force_excess,
            dir_consistency=self.dir_consistency,
            enter_score=self.enter_score,
            exit_score=self.exit_score,
            enter_timer=self.enter_timer,
            exit_timer=self.exit_timer,
            cooldown_timer=self.cooldown_timer,
            standing_mask=standing_mask,
            walking_mask=walking_mask,
        )


class ForceGuidedTurningController:
    def __init__(self, args: argparse.Namespace):
        self.enabled = bool(args.force_guided_turning)
        self.base_force_kd = max(float(args.base_force_kd), 1.0e-6)
        self.min_force_n = max(float(args.force_guided_min_force_n), 0.0)
        self.yaw_kp = max(float(args.force_guided_yaw_kp), 0.0)
        self.yaw_rate_max = max(float(args.force_guided_yaw_rate_max), 0.0)
        self.turn_only_angle_rad = math.radians(max(float(args.force_guided_turn_only_angle_deg), 0.0))
        self.full_speed_angle_rad = math.radians(
            max(min(float(args.force_guided_full_speed_angle_deg), float(args.force_guided_turn_only_angle_deg)), 0.0)
        )
        self.speed_scale = max(float(args.force_guided_speed_scale), 0.0)
        self.speed_max = max(float(args.force_guided_speed_max), 0.0)

    def compute(
        self,
        nominal_cmd_vel_b: np.ndarray,
        base_force_est_local: np.ndarray,
        mode_state: ForceModeSwitchState,
    ) -> ForceGuidanceState:
        effective_cmd_vel_b = nominal_cmd_vel_b.astype(np.float32, copy=True)
        if not self.enabled:
            return ForceGuidanceState(
                enabled=False,
                active=False,
                effective_cmd_vel_b=effective_cmd_vel_b,
                heading_error_rad=0.0,
                estimated_force_heading_rad=0.0,
                estimated_force_mag=0.0,
                motion_sign=1.0,
            )

        force_xy = np.asarray(base_force_est_local[:2], dtype=np.float32)
        estimated_force_mag = float(np.linalg.norm(force_xy))
        if not math.isfinite(estimated_force_mag):
            estimated_force_mag = 0.0
        estimated_force_heading_rad = math.atan2(float(force_xy[1]), float(force_xy[0]))

        reverse_mask = float(force_xy[0]) < 0.0
        motion_sign = -1.0 if reverse_mask else 1.0
        alignment_xy = (-force_xy) if reverse_mask else force_xy
        heading_error_rad = wrap_to_pi(math.atan2(float(alignment_xy[1]), float(alignment_xy[0])))

        active = bool(
            mode_state.command_zero
            and mode_state.mode == MODE_FORCE_WALKING
            and estimated_force_mag >= self.min_force_n
        )
        if active:
            speed_cmd = (estimated_force_mag / self.base_force_kd) * self.speed_scale
            speed_cmd = max(0.0, min(speed_cmd, self.speed_max))
            speed_progress = angle_progress(
                abs(heading_error_rad),
                full_speed_angle_rad=self.full_speed_angle_rad,
                turn_only_angle_rad=self.turn_only_angle_rad,
            )
            yaw_rate_cmd = max(-self.yaw_rate_max, min(self.yaw_kp * heading_error_rad, self.yaw_rate_max))
            effective_cmd_vel_b[0] = motion_sign * speed_cmd * speed_progress
            effective_cmd_vel_b[1] = 0.0
            effective_cmd_vel_b[2] = yaw_rate_cmd

        return ForceGuidanceState(
            enabled=True,
            active=active,
            effective_cmd_vel_b=effective_cmd_vel_b,
            heading_error_rad=heading_error_rad,
            estimated_force_heading_rad=estimated_force_heading_rad,
            estimated_force_mag=estimated_force_mag,
            motion_sign=motion_sign,
        )


class BrakeCommandGate:
    def __init__(self, args: argparse.Namespace):
        self.enabled = bool(args.brake_gate)
        self.force_x_threshold_n = float(args.brake_gate_force_x_threshold_n)
        self.min_cmd_vx = max(float(args.brake_gate_min_cmd_vx), 0.0)
        self.max_cmd_wz = max(float(args.brake_gate_max_cmd_wz), 0.0)
        self.lin_vel_x_clip = float(args.lin_vel_x_clip)
        self.lin_vel_y_clip = float(args.lin_vel_y_clip)
        self.ang_vel_yaw_clip = float(args.ang_vel_yaw_clip)
        self.hold_steps = max(int(args.brake_gate_hold_steps), 1)
        self.hold_counter = 0
        self.blocked_phase_name: str | None = None

    def _command_nonzero(self, cmd_vel_b: np.ndarray) -> bool:
        return not (
            abs(float(cmd_vel_b[0])) < self.lin_vel_x_clip
            and abs(float(cmd_vel_b[1])) < self.lin_vel_y_clip
            and abs(float(cmd_vel_b[2])) < self.ang_vel_yaw_clip
        )

    def update(
        self,
        cmd_vel_b: np.ndarray,
        base_force_est_local: np.ndarray,
        *,
        phase_name: str,
    ) -> BrakeGateState:
        if self.blocked_phase_name is not None and phase_name != self.blocked_phase_name:
            self.blocked_phase_name = None
            self.hold_counter = 0

        est_force_x_n = float(base_force_est_local[0])
        est_force_y_n = float(base_force_est_local[1])
        if not math.isfinite(est_force_x_n):
            est_force_x_n = 0.0
        if not math.isfinite(est_force_y_n):
            est_force_y_n = 0.0

        latched_phase_active = self.blocked_phase_name == phase_name
        eligible = bool(
            self.enabled
            and self._command_nonzero(cmd_vel_b)
            and float(cmd_vel_b[0]) >= self.min_cmd_vx
            and abs(float(cmd_vel_b[2])) <= self.max_cmd_wz
            and phase_name in BRAKE_GATE_ELIGIBLE_PHASES
        )
        trigger_sample = bool((not latched_phase_active) and eligible and est_force_x_n <= self.force_x_threshold_n)
        if latched_phase_active:
            active = True
        else:
            if trigger_sample:
                self.hold_counter = min(self.hold_counter + 1, self.hold_steps)
            else:
                self.hold_counter = 0
            active = bool(eligible and self.hold_counter >= self.hold_steps)
            if active:
                self.blocked_phase_name = phase_name

        return BrakeGateState(
            enabled=self.enabled,
            eligible=eligible,
            active=active,
            hold_counter=self.hold_counter,
            est_force_x_n=est_force_x_n,
            est_force_y_n=est_force_y_n,
            threshold_x_n=self.force_x_threshold_n,
            blocked_phase_name=self.blocked_phase_name,
        )


class SmartGaitPhaseClock:
    def __init__(self, gait_frequency_hz: float):
        self.gait_frequency_hz = float(gait_frequency_hz)
        self.phase = 0.0

    def current_vector(self) -> np.ndarray:
        return np.array(
            [
                math.sin(2.0 * math.pi * self.phase),
                math.cos(2.0 * math.pi * self.phase),
            ],
            dtype=np.float32,
        )

    def step(self, mode_id: int, dt: float) -> np.ndarray:
        if int(mode_id) == MODE_STANDING:
            self.phase = 0.0
        else:
            self.phase = (self.phase + float(dt) * self.gait_frequency_hz) % 1.0
        return self.current_vector()


def resolve_output_csv(path_str: str | None) -> Path:
    if path_str:
        output_path = Path(path_str).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = (
            Path("logs")
            / "experiments"
            / "unifp_smart_gait_mujoco"
            / f"smart_gait_{stamp}.csv"
        ).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def extract_force_estimator_outputs(
    pred_est: np.ndarray,
    base_force_tgt_local: np.ndarray,
    *,
    base_force_scale: float,
) -> dict[str, np.ndarray | float]:
    pred_est = np.asarray(pred_est, dtype=np.float32).reshape(-1)
    scale = max(float(base_force_scale), 1.0e-6)
    if pred_est.shape == (6,):
        base_force_est_local = pred_est[3:6] / scale
    elif pred_est.shape == (12,):
        base_force_est_local = pred_est[9:12] / scale
    else:
        raise RuntimeError(f"Expected pred_est shape (6,) or (12,), received {pred_est.shape}")

    base_force_err_l2 = float(np.linalg.norm(base_force_est_local - base_force_tgt_local))
    return {
        "base_force_est_local": base_force_est_local.astype(np.float32),
        "base_force_tgt_local": np.asarray(base_force_tgt_local, dtype=np.float32),
        "base_force_err_l2": base_force_err_l2,
    }


def write_csv_row(
    writer: csv.writer,
    *,
    step: int,
    time_s: float,
    nominal_cmd_vel_b: np.ndarray,
    pre_brake_cmd_vel_b: np.ndarray,
    effective_cmd_vel_b: np.ndarray,
    applied_force_target_n: float,
    base_lin_vel_local: np.ndarray,
    estimator_outputs: dict[str, np.ndarray | float],
    mode_state: ForceModeSwitchState,
    guidance_state: ForceGuidanceState,
    brake_state: BrakeGateState,
    gait_phase: float,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
    force_schedule_target_n: float,
    force_target_n: float,
    base_yaw_deg: float,
    base_yaw_rate_z: float,
) -> None:
    base_force_est_local = np.asarray(estimator_outputs["base_force_est_local"], dtype=np.float32)
    base_force_tgt_local = np.asarray(estimator_outputs["base_force_tgt_local"], dtype=np.float32)
    base_force_err_l2 = float(estimator_outputs["base_force_err_l2"])
    writer.writerow(
        [
            0,
            step,
            f"{time_s:.6f}",
            mode_state.mode,
            mode_name(mode_state.mode),
            int(mode_state.command_zero),
            int(mode_state.standing_mask),
            int(mode_state.walking_mask),
            f"{nominal_cmd_vel_b[0]:.6f}",
            f"{nominal_cmd_vel_b[1]:.6f}",
            f"{nominal_cmd_vel_b[2]:.6f}",
            force_segment_name,
            force_direction_name,
            force_phase_name,
            f"{force_schedule_target_n:.6f}",
            f"{force_target_n:.6f}",
            f"{applied_force_target_n:.6f}",
            f"{base_force_est_local[0]:.6f}",
            f"{base_force_est_local[1]:.6f}",
            f"{base_force_est_local[2]:.6f}",
            f"{base_force_tgt_local[0]:.6f}",
            f"{base_force_tgt_local[1]:.6f}",
            f"{base_force_tgt_local[2]:.6f}",
            f"{base_force_err_l2:.6f}",
            f"{base_lin_vel_local[0]:.6f}",
            f"{base_lin_vel_local[1]:.6f}",
            f"{base_lin_vel_local[2]:.6f}",
            f"{mode_state.force_xy_raw:.6f}",
            f"{mode_state.force_xy_cap:.6f}",
            f"{mode_state.force_baseline:.6f}",
            f"{mode_state.force_excess:.6f}",
            f"{mode_state.dir_consistency:.6f}",
            f"{mode_state.enter_score:.6f}",
            f"{mode_state.exit_score:.6f}",
            f"{mode_state.enter_timer:.6f}",
            f"{mode_state.exit_timer:.6f}",
            f"{mode_state.cooldown_timer:.6f}",
            f"{gait_phase:.6f}",
            int(guidance_state.enabled),
            int(guidance_state.active),
            int(brake_state.enabled),
            int(brake_state.eligible),
            int(brake_state.active),
            brake_state.hold_counter,
            f"{brake_state.threshold_x_n:.6f}",
            f"{brake_state.est_force_x_n:.6f}",
            f"{brake_state.est_force_y_n:.6f}",
            brake_state.blocked_phase_name or "",
            f"{pre_brake_cmd_vel_b[0]:.6f}",
            f"{pre_brake_cmd_vel_b[1]:.6f}",
            f"{pre_brake_cmd_vel_b[2]:.6f}",
            f"{effective_cmd_vel_b[0]:.6f}",
            f"{effective_cmd_vel_b[1]:.6f}",
            f"{effective_cmd_vel_b[2]:.6f}",
            f"{math.degrees(guidance_state.heading_error_rad):.6f}",
            f"{math.degrees(guidance_state.estimated_force_heading_rad):.6f}",
            f"{guidance_state.estimated_force_mag:.6f}",
            f"{guidance_state.motion_sign:.0f}",
            f"{base_yaw_deg:.6f}",
            f"{base_yaw_rate_z:.6f}",
        ]
    )


def maybe_log_step(
    *,
    args: argparse.Namespace,
    step: int,
    time_s: float,
    nominal_cmd_vel_b: np.ndarray,
    pre_brake_cmd_vel_b: np.ndarray,
    effective_cmd_vel_b: np.ndarray,
    applied_force_target_n: float,
    mode_state: ForceModeSwitchState,
    guidance_state: ForceGuidanceState,
    brake_state: BrakeGateState,
    gait_phase: float,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
) -> None:
    if args.log_interval <= 0 or step % args.log_interval != 0:
        return
    print(
        f"[SMART_GAIT][step={step:05d}] "
        f"t={time_s:6.2f}s "
        f"segment={force_segment_name}/{force_direction_name}/{force_phase_name} "
        f"mode={mode_name(mode_state.mode)} "
        f"command_zero={mode_state.command_zero} "
        f"standing_mask={mode_state.standing_mask} "
        f"cmd=({nominal_cmd_vel_b[0]:+.3f}, {nominal_cmd_vel_b[1]:+.3f}, {nominal_cmd_vel_b[2]:+.3f}) "
        f"pre_brake=({pre_brake_cmd_vel_b[0]:+.3f}, {pre_brake_cmd_vel_b[1]:+.3f}, {pre_brake_cmd_vel_b[2]:+.3f}) "
        f"eff_cmd=({effective_cmd_vel_b[0]:+.3f}, {effective_cmd_vel_b[1]:+.3f}, {effective_cmd_vel_b[2]:+.3f}) "
        f"brake_active={brake_state.active} "
        f"brake_phase={brake_state.blocked_phase_name or '-'} "
        f"brake_hold={brake_state.hold_counter}/{max(int(args.brake_gate_hold_steps), 1)} "
        f"brake_fx={brake_state.est_force_x_n:+.3f}N "
        f"F_applied={applied_force_target_n:.3f}N "
        f"F_raw={mode_state.force_xy_raw:.3f}N "
        f"F_cap={mode_state.force_xy_cap:.3f}N "
        f"F_base={mode_state.force_baseline:.3f}N "
        f"F_ex={mode_state.force_excess:.3f}N "
        f"dir={mode_state.dir_consistency:.3f} "
        f"enter_s={mode_state.enter_score:.3f} "
        f"exit_s={mode_state.exit_score:.3f} "
        f"enter_t={mode_state.enter_timer:.3f}s "
        f"exit_t={mode_state.exit_timer:.3f}s "
        f"cooldown={mode_state.cooldown_timer:.3f}s "
        f"guide_active={guidance_state.active} "
        f"guide_err={math.degrees(guidance_state.heading_error_rad):+.1f}deg "
        f"gait_phase={gait_phase:.3f}"
    )


def apply_gate_preset(args: argparse.Namespace) -> argparse.Namespace:
    preset_name = str(getattr(args, "gate_preset", GATE_PRESET_LEGACY))
    if preset_name not in GATE_PRESET_OVERRIDES:
        raise ValueError(f"Unsupported gate preset '{preset_name}'.")
    for field_name, value in GATE_PRESET_OVERRIDES[preset_name].items():
        setattr(args, field_name, value)
    return args


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate the smart-gait UniFP policy in MuJoCo.")
    parser.add_argument(
        "--model-path",
        type=Path,
        default=Path("resources/a2_description/a2.xml"),
        help="Path to a MuJoCo-loadable URDF or MJCF model.",
    )
    parser.add_argument("--policy-path", type=Path, required=True, help="Path to the MuJoCo-ready TorchScript policy.")
    parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S, help="Evaluation horizon in seconds.")
    parser.add_argument(
        "--policy-history-length",
        type=int,
        default=None,
        help="Optional override for the flattened policy history length used by the MuJoCo runtime.",
    )
    parser.add_argument("--cmd-vx", type=float, default=0.0, help="Fixed body-frame x velocity command.")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Fixed body-frame y velocity command.")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="Fixed yaw-rate command.")
    parser.add_argument(
        "--force-schedule",
        type=str,
        default=FORCE_SCHEDULE_DEMO,
        choices=[
            FORCE_SCHEDULE_NONE,
            FORCE_SCHEDULE_DEMO,
            FORCE_SCHEDULE_STOP_GO,
            FORCE_SCHEDULE_HUMAN_GUIDE,
        ],
    )
    parser.add_argument("--stop-go-heading-seq", type=str, default="forward,left,backward")
    parser.add_argument("--stop-go-idle-s", type=float, default=3.0)
    parser.add_argument("--stop-go-pre-tension-s", type=float, default=1.0)
    parser.add_argument("--stop-go-start-pull-s", type=float, default=1.2)
    parser.add_argument("--stop-go-steady-s", type=float, default=3.5)
    parser.add_argument("--stop-go-stop-cue-s", type=float, default=0.8)
    parser.add_argument("--stop-go-slack-s", type=float, default=1.6)
    parser.add_argument("--stop-go-post-idle-s", type=float, default=1.4)
    parser.add_argument("--stop-go-pre-tension-n", type=float, default=3.0)
    parser.add_argument("--stop-go-start-pull-n", type=float, default=10.0)
    parser.add_argument("--stop-go-steady-force-n", type=float, default=15.0)
    parser.add_argument("--stop-go-steady-force-osc-n", type=float, default=1.5)
    parser.add_argument("--stop-go-stop-cue-n", type=float, default=1.5)
    parser.add_argument("--stop-go-pre-tension-heading-drift-deg", type=float, default=2.0)
    parser.add_argument("--stop-go-start-pull-heading-drift-deg", type=float, default=4.0)
    parser.add_argument("--stop-go-steady-heading-drift-deg", type=float, default=8.0)
    parser.add_argument("--stop-go-mag-tau-s", type=float, default=0.08)
    parser.add_argument("--stop-go-angle-tau-s", type=float, default=0.12)
    parser.add_argument("--stop-go-steady-osc-period-s", type=float, default=1.4)
    parser.add_argument("--human-guide-straight-s", type=float, default=5.0)
    parser.add_argument("--human-guide-turn-s", type=float, default=2.5)
    parser.add_argument("--human-guide-gap-s", type=float, default=3.0)
    parser.add_argument("--human-guide-base-force-n", type=float, default=15.0)
    parser.add_argument("--human-guide-force-jitter-n", type=float, default=2.0)
    parser.add_argument("--human-guide-heading-jitter-deg", type=float, default=12.0)
    parser.add_argument("--human-guide-dropout-max-s", type=float, default=0.35)
    parser.add_argument("--walking-gait-frequency", type=float, default=2.0)
    parser.add_argument(
        "--gate-preset",
        type=str,
        default=GATE_PRESET_LEGACY,
        choices=(
            GATE_PRESET_LEGACY,
            GATE_PRESET_BALANCED,
            GATE_PRESET_ROBUST,
            GATE_PRESET_V1,
            GATE_PRESET_V1_ROBUST,
            GATE_PRESET_V1_TURNING,
            GATE_PRESET_V1_ROBUST_TURNING,
            GATE_PRESET_V2,
            GATE_PRESET_V2_ROBUST,
        ),
        help="Optional standing/walking gate preset. Presets override the individual gate tuning flags.",
    )
    parser.add_argument("--baseline-tau-s", type=float, default=2.0)
    parser.add_argument("--baseline-margin-n", type=float, default=0.15)
    parser.add_argument("--baseline-update-max-n", type=float, default=0.25)
    parser.add_argument("--evidence-cap-n", type=float, default=4.0)
    parser.add_argument("--enter-threshold-n", type=float, default=0.5)
    parser.add_argument("--exit-threshold-n", type=float, default=0.40)
    parser.add_argument("--enter-tau-on-s", type=float, default=0.08)
    parser.add_argument("--enter-tau-off-s", type=float, default=0.20)
    parser.add_argument("--exit-tau-on-s", type=float, default=0.08)
    parser.add_argument("--exit-tau-off-s", type=float, default=0.08)
    parser.add_argument("--enter-score-threshold", type=float, default=0.50)
    parser.add_argument("--exit-score-threshold", type=float, default=0.60)
    parser.add_argument("--dir-consistency-min-force-n", type=float, default=0.35)
    parser.add_argument("--dir-consistency-tau-s", type=float, default=0.10)
    parser.add_argument("--dir-consistency-threshold", type=float, default=0.45)
    parser.add_argument("--switch-cooldown-s", type=float, default=0.20)
    parser.add_argument("--enter-hold-s", type=float, default=0.06)
    parser.add_argument("--exit-hold-s", type=float, default=0.08)
    parser.add_argument("--lin-vel-x-clip", type=float, default=LIN_VEL_X_CLIP)
    parser.add_argument("--lin-vel-y-clip", type=float, default=LIN_VEL_Y_CLIP)
    parser.add_argument("--ang-vel-yaw-clip", type=float, default=ANG_VEL_YAW_CLIP)
    parser.add_argument("--base-force-kd", type=float, default=BASE_FORCE_KD)
    parser.add_argument("--force-guided-turning", action="store_true")
    parser.add_argument("--force-guided-min-force-n", type=float, default=1.0)
    parser.add_argument("--force-guided-yaw-kp", type=float, default=1.8)
    parser.add_argument("--force-guided-yaw-rate-max", type=float, default=0.8)
    parser.add_argument("--force-guided-turn-only-angle-deg", type=float, default=25.0)
    parser.add_argument("--force-guided-full-speed-angle-deg", type=float, default=8.0)
    parser.add_argument("--force-guided-speed-scale", type=float, default=1.0)
    parser.add_argument("--force-guided-speed-max", type=float, default=0.35)
    parser.add_argument("--brake-gate", action="store_true")
    parser.add_argument("--brake-gate-force-x-threshold-n", type=float, default=BRAKE_GATE_FORCE_X_THRESHOLD_N)
    parser.add_argument("--brake-gate-min-cmd-vx", type=float, default=BRAKE_GATE_MIN_CMD_VX)
    parser.add_argument("--brake-gate-max-cmd-wz", type=float, default=BRAKE_GATE_MAX_CMD_WZ)
    parser.add_argument("--brake-gate-hold-steps", type=int, default=BRAKE_GATE_HOLD_STEPS)
    parser.add_argument("--base-force-scale", type=float, default=BASE_FORCE_SCALE)
    parser.add_argument("--obs-delay-steps", type=int, default=0, choices=(0, 1, 2))
    parser.add_argument("--leg-action-delay-steps", type=int, default=0, choices=(0, 1, 2))
    parser.add_argument("--seed", type=int, default=0, help="Seed used by the human-guide schedule randomization.")
    parser.add_argument("--log-interval", type=int, default=60)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--real-time", action="store_true")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    args = apply_gate_preset(args)

    if args.policy_history_length is not None and int(args.policy_history_length) <= 0:
        parser.error(f"--policy-history-length must be positive, got {args.policy_history_length}.")

    if mujoco is None:
        print("MuJoCo is not installed. Install dependencies from scripts/mujoco/requirements.txt first.", file=sys.stderr)
        return 1

    output_csv = resolve_output_csv(args.output_csv)
    metadata = load_policy_metadata(args.policy_path, fallback=default_smart_gait_metadata())
    if args.policy_history_length is not None:
        metadata = replace(metadata, history_length=int(args.policy_history_length))
    metadata = replace(metadata, gait_frequency_hz=float(args.walking_gait_frequency))

    base_nominal_cmd_vel_b = np.array([float(args.cmd_vx), float(args.cmd_vy), float(args.cmd_wz)], dtype=np.float32)
    initial_command_obs = build_command_vector(
        locomotion_cmd=base_nominal_cmd_vel_b,
        metadata=metadata,
    )

    model = load_mujoco_model_from_path(mujoco, args.model_path)
    data = mujoco.MjData(model)
    if model.nu == 0:
        print("The MuJoCo model has no actuators; smart-gait evaluation cannot run.", file=sys.stderr)
        return 2

    bindings = create_model_bindings(model, metadata)
    base_body_id = resolve_body_id(model, metadata.base_body_name)
    apply_joint_actuator_gains(model, bindings, metadata)
    policy = load_torchscript_policy(args.policy_path)

    reset_model_state(mujoco, model, data, metadata, bindings.qpos_indices)
    clear_body_forces(data)

    gait_clock_state = SmartGaitPhaseClock(metadata.gait_frequency_hz)
    runtime_state = initialize_runtime_state(
        data=data,
        bindings=bindings,
        metadata=metadata,
        obs_delay_steps=args.obs_delay_steps,
        leg_action_delay_steps=args.leg_action_delay_steps,
        command_obs=initial_command_obs,
        gait_override=gait_clock_state.current_vector(),
    )

    scheduler = None
    if args.force_schedule == FORCE_SCHEDULE_DEMO:
        scheduler = ScriptedForceScheduler(build_demo_force_segments(args.duration_s))
    elif args.force_schedule == FORCE_SCHEDULE_STOP_GO:
        scheduler = StopGoTuningForceScheduler(
            args,
            control_dt=metadata.control_dt,
            headings_deg=parse_heading_deg_sequence(args.stop_go_heading_seq),
        )
    elif args.force_schedule == FORCE_SCHEDULE_HUMAN_GUIDE:
        scheduler = HumanGuideForceScheduler(args, control_dt=metadata.control_dt)

    switcher = ForceModeSwitcher(args)
    guidance_controller = ForceGuidedTurningController(args)
    brake_gate = BrakeCommandGate(args)
    mode_counts = np.zeros(3, dtype=np.int64)
    last_nominal_cmd_vel_b = base_nominal_cmd_vel_b.astype(np.float32, copy=True)
    last_external_force_w = np.zeros(3, dtype=np.float32)

    print(
        "[INFO] Runtime config: "
        f"gate_preset={args.gate_preset} "
        f"history_length={metadata.history_length} "
        f"per_frame_obs_dim={metadata.per_frame_obs_dim} "
        f"action_dim={metadata.action_dim} "
        f"gait_frequency_hz={metadata.gait_frequency_hz:.3f} "
        f"model_timestep={model.opt.timestep:.4f} "
        f"control_dt={metadata.control_dt:.4f} "
        f"obs_delay_steps={args.obs_delay_steps} "
        f"leg_action_delay_steps={args.leg_action_delay_steps} "
        f"actuator_gains={summarize_joint_actuator_gains(metadata)}"
    )
    print(f"[INFO] CSV output: {output_csv}")

    csv_header = [
        "env_id",
        "step",
        "time_s",
        "mode_id",
        "mode_name",
        "command_zero",
        "standing_mask",
        "walking_mask",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "force_segment_name",
        "force_direction_name",
        "force_phase_name",
        "scheduled_force_target_n",
        "force_target_n",
        "applied_force_target_n",
        "base_force_est_local_x",
        "base_force_est_local_y",
        "base_force_est_local_z",
        "base_force_tgt_local_x",
        "base_force_tgt_local_y",
        "base_force_tgt_local_z",
        "base_force_err_l2",
        "base_lin_vel_local_x",
        "base_lin_vel_local_y",
        "base_lin_vel_local_z",
        "base_force_est_xy_raw",
        "force_xy_cap",
        "force_baseline",
        "force_excess",
        "dir_consistency",
        "enter_score",
        "exit_score",
        "enter_timer",
        "exit_timer",
        "cooldown_timer",
        "gait_phase",
        "force_guided_turning_enabled",
        "force_guided_turning_active",
        "brake_gate_enabled",
        "brake_gate_eligible",
        "brake_gate_active",
        "brake_gate_hold_counter",
        "brake_gate_force_x_threshold_n",
        "brake_gate_est_force_x_n",
        "brake_gate_est_force_y_n",
        "brake_gate_blocked_phase_name",
        "pre_brake_cmd_vx",
        "pre_brake_cmd_vy",
        "pre_brake_cmd_wz",
        "effective_cmd_vx",
        "effective_cmd_vy",
        "effective_cmd_wz",
        "guided_heading_error_deg",
        "guided_est_force_heading_deg",
        "guided_est_force_mag",
        "guided_motion_sign",
        "base_yaw_deg",
        "base_yaw_rate_z",
    ]

    def run_one_control_step(step: int, writer: csv.writer) -> None:
        nonlocal last_external_force_w, last_nominal_cmd_vel_b
        time_s = step * float(metadata.control_dt)
        base_quat = data.qpos[3:7].astype(np.float64, copy=True)
        base_lin_vel_local = world_to_base_yaw(data.qvel[:3].astype(np.float64, copy=True), base_quat)
        base_yaw_deg = math.degrees(yaw_from_quat(base_quat))
        base_yaw_rate_z = float(data.qvel[5])

        if scheduler is None:
            scheduler_outputs = {
                "segment_name": FORCE_SCHEDULE_NONE,
                "direction_name": DIR_NONE,
                "phase_name": "idle",
                "force_schedule_target_n": 0.0,
                "force_target_n": 0.0,
                "force_w": np.zeros(3, dtype=np.float32),
                "axis_xy": np.full(2, np.nan, dtype=np.float32),
                "is_active": False,
            }
        else:
            scheduler_outputs = scheduler.update(base_quat, time_s)

        nominal_cmd_vel_b = np.asarray(
            scheduler_outputs.get("nominal_cmd_vel_b", base_nominal_cmd_vel_b),
            dtype=np.float32,
        )
        scheduled_force_w = np.asarray(scheduler_outputs["force_w"], dtype=np.float32)

        obs = runtime_state.policy_obs()
        action, pred_est = infer_policy_outputs(policy, obs, action_dim=metadata.action_dim)
        action = np.clip(action, -metadata.action_clip, metadata.action_clip)
        if pred_est is None:
            raise RuntimeError(
                "smart_gait MuJoCo evaluation requires a MuJoCo export that returns (action, pred_est). "
                "Re-export the policy with scripts/mujoco/export_policy_wrapper.py."
            )

        scheduled_force_tgt_local = world_to_base_yaw(
            scheduled_force_w,
            base_quat,
        )
        gate_estimator_outputs = extract_force_estimator_outputs(
            pred_est.astype(np.float32),
            scheduled_force_tgt_local.astype(np.float32),
            base_force_scale=args.base_force_scale,
        )

        mode_state = switcher.update(
            nominal_cmd_vel_b,
            np.asarray(gate_estimator_outputs["base_force_est_local"], dtype=np.float32),
            dt=metadata.control_dt,
        )
        guidance_state = guidance_controller.compute(
            nominal_cmd_vel_b,
            np.asarray(gate_estimator_outputs["base_force_est_local"], dtype=np.float32),
            mode_state,
        )
        pre_brake_cmd_vel_b = guidance_state.effective_cmd_vel_b.astype(np.float32, copy=True)
        brake_state = brake_gate.update(
            pre_brake_cmd_vel_b,
            np.asarray(gate_estimator_outputs["base_force_est_local"], dtype=np.float32),
            phase_name=str(scheduler_outputs["phase_name"]),
        )

        applied_force_w = scheduled_force_w.astype(np.float32, copy=True)
        if brake_state.active:
            guidance_state.effective_cmd_vel_b = np.zeros(3, dtype=np.float32)
            applied_force_w = np.zeros(3, dtype=np.float32)

        last_nominal_cmd_vel_b = guidance_state.effective_cmd_vel_b.astype(np.float32, copy=True)
        last_external_force_w = applied_force_w.astype(np.float32, copy=True)
        if float(np.linalg.norm(applied_force_w)) > 1.0e-6:
            set_body_force(data, base_body_id, applied_force_w)
        else:
            clear_body_forces(data)

        applied_force_target_n = float(np.linalg.norm(applied_force_w[:2]))
        base_force_tgt_local = world_to_base_yaw(
            applied_force_w,
            base_quat,
        )
        estimator_outputs = extract_force_estimator_outputs(
            pred_est.astype(np.float32),
            base_force_tgt_local.astype(np.float32),
            base_force_scale=args.base_force_scale,
        )
        mode_counts[mode_state.mode] += 1

        target_joint_pos = build_target_joint_positions(action, runtime_state, metadata)
        apply_control_targets(model, data, bindings, target_joint_pos, metadata)

        wall_start = time.perf_counter()
        try:
            mujoco.mj_step(model, data, nstep=metadata.control_decimation)
        except TypeError:
            for _ in range(metadata.control_decimation):
                mujoco.mj_step(model, data)

        next_command_obs = build_command_vector(
            locomotion_cmd=guidance_state.effective_cmd_vel_b,
            metadata=metadata,
        )
        gait_override = gait_clock_state.step(mode_state.mode, metadata.control_dt)
        update_runtime_state_after_step(
            data=data,
            bindings=bindings,
            metadata=metadata,
            runtime_state=runtime_state,
            action=action,
            command_obs=next_command_obs,
            gait_override=gait_override,
        )

        if args.real_time:
            sleep_time = metadata.control_dt - (time.perf_counter() - wall_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        write_csv_row(
            writer,
            step=step,
            time_s=time_s,
            nominal_cmd_vel_b=nominal_cmd_vel_b,
            pre_brake_cmd_vel_b=pre_brake_cmd_vel_b,
            effective_cmd_vel_b=guidance_state.effective_cmd_vel_b,
            applied_force_target_n=applied_force_target_n,
            base_lin_vel_local=base_lin_vel_local.astype(np.float32),
            estimator_outputs=estimator_outputs,
            mode_state=mode_state,
            guidance_state=guidance_state,
            brake_state=brake_state,
            gait_phase=gait_clock_state.phase,
            force_segment_name=str(scheduler_outputs["segment_name"]),
            force_direction_name=str(scheduler_outputs["direction_name"]),
            force_phase_name=str(scheduler_outputs["phase_name"]),
            force_schedule_target_n=float(scheduler_outputs["force_schedule_target_n"]),
            force_target_n=float(scheduler_outputs["force_target_n"]),
            base_yaw_deg=base_yaw_deg,
            base_yaw_rate_z=base_yaw_rate_z,
        )
        maybe_log_step(
            args=args,
            step=step,
            time_s=time_s,
            nominal_cmd_vel_b=nominal_cmd_vel_b,
            pre_brake_cmd_vel_b=pre_brake_cmd_vel_b,
            effective_cmd_vel_b=guidance_state.effective_cmd_vel_b,
            applied_force_target_n=applied_force_target_n,
            mode_state=mode_state,
            guidance_state=guidance_state,
            brake_state=brake_state,
            gait_phase=gait_clock_state.phase,
            force_segment_name=str(scheduler_outputs["segment_name"]),
            force_direction_name=str(scheduler_outputs["direction_name"]),
            force_phase_name=str(scheduler_outputs["phase_name"]),
        )

    duration_s = float(args.duration_s)
    max_steps = max(int(round(duration_s / metadata.control_dt)), 1)

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(csv_header)

        if args.headless:
            for step in range(max_steps):
                run_one_control_step(step, writer)
        else:
            if mujoco_viewer is None:
                print("MuJoCo viewer is unavailable in this environment. Re-run with --headless.", file=sys.stderr)
                return 3
            with mujoco_viewer.launch_passive(model, data) as viewer:
                configure_tracking_viewer_camera(mujoco, viewer, data, track_body_id=base_body_id)
                for step in range(max_steps):
                    if not viewer.is_running():
                        break
                    run_one_control_step(step, writer)
                    draw_velocity_arrows(
                        mujoco,
                        viewer,
                        data,
                        base_body_id=base_body_id,
                        command_vel_xy=last_nominal_cmd_vel_b[:2],
                        external_force_w=last_external_force_w,
                        force_arrow_z_extra_offset=0.05,
                    )
                    viewer.sync()

    clear_body_forces(data)
    print(f"[INFO] CSV written to: {output_csv}")
    print(
        "[SUMMARY] "
        f"standing={mode_counts[MODE_STANDING] * metadata.control_dt:.2f}s "
        f"force_walking={mode_counts[MODE_FORCE_WALKING] * metadata.control_dt:.2f}s "
        f"command_walking={mode_counts[MODE_COMMAND_WALKING] * metadata.control_dt:.2f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
