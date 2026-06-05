#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import json
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
    default_staged_eval_metadata,
    draw_velocity_arrows,
    finite_or_nan,
    format_optional,
    infer_policy_outputs,
    initialize_runtime_state,
    load_policy_metadata,
    load_torchscript_policy,
    reset_model_state,
    resolve_body_id,
    set_body_force,
    summarize_joint_actuator_gains,
    update_runtime_state_after_step,
    world_to_base_yaw,
)

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import mujoco.viewer as mujoco_viewer
except ImportError:
    mujoco_viewer = None


EPISODE_LENGTH_S = 60.0

FORCE_PROFILE_LEGACY_STAGED = "legacy_staged"
FORCE_PROFILE_SMOOTH_0_20 = "smooth_0_20"
FORCE_INTERP_LINEAR = "linear"
FORCE_INTERP_SMOOTHSTEP = "smoothstep"

DIR_NONE = "none"
DIR_FORWARD = "forward"
DIR_LEFT = "left"
DIR_BACKWARD = "backward"
ACTIVE_DIRECTIONS = (DIR_FORWARD, DIR_LEFT, DIR_BACKWARD)
DIRECTION_INDEX = {name: idx for idx, name in enumerate(ACTIVE_DIRECTIONS)}


@dataclass(frozen=True)
class StageSegment:
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
        if self.force_interp == FORCE_INTERP_SMOOTHSTEP:
            ratio = smoothstep01(ratio)
        return float(self.force_start_n + ratio * (self.force_end_n - self.force_start_n))


STAGE_SEGMENTS = (
    StageSegment("idle_0", DIR_NONE, "idle", 0.0, 5.0),
    StageSegment("forward_low", DIR_FORWARD, "low", 5.0, 8.0, 10.0, 10.0, capture_axis=True),
    StageSegment("forward_ramp", DIR_FORWARD, "ramp", 8.0, 9.0, 10.0, 40.0),
    StageSegment("forward_high", DIR_FORWARD, "high", 9.0, 13.0, 40.0, 40.0),
    StageSegment("idle_1", DIR_NONE, "idle", 13.0, 20.0),
    StageSegment("left_low", DIR_LEFT, "low", 20.0, 23.0, 10.0, 10.0, capture_axis=True),
    StageSegment("left_ramp", DIR_LEFT, "ramp", 23.0, 24.0, 10.0, 40.0),
    StageSegment("left_high", DIR_LEFT, "high", 24.0, 28.0, 40.0, 40.0),
    StageSegment("idle_2", DIR_NONE, "idle", 28.0, 40.0),
    StageSegment("backward_low", DIR_BACKWARD, "low", 40.0, 43.0, 10.0, 10.0, capture_axis=True),
    StageSegment("backward_ramp", DIR_BACKWARD, "ramp", 43.0, 44.0, 10.0, 40.0),
    StageSegment("backward_high", DIR_BACKWARD, "high", 44.0, 48.0, 40.0, 40.0),
    StageSegment("idle_3", DIR_NONE, "idle", 48.0, EPISODE_LENGTH_S),
)


def smoothstep01(value: float) -> float:
    clamped = max(0.0, min(float(value), 1.0))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def build_smooth_0_20_segments(
    *,
    peak_force_n: float,
    ramp_s: float,
    hold_s: float,
    release_s: float,
    idle_s: float,
) -> tuple[StageSegment, ...]:
    peak = max(float(peak_force_n), 0.0)
    ramp = max(float(ramp_s), 0.0)
    hold = max(float(hold_s), 0.0)
    release = max(float(release_s), 0.0)
    idle = max(float(idle_s), 0.0)

    segments: list[StageSegment] = []
    cursor_s = 0.0
    for direction_name in ACTIVE_DIRECTIONS:
        if idle > 0.0:
            segments.append(StageSegment(f"{direction_name}_idle_pre", DIR_NONE, "idle", cursor_s, cursor_s + idle))
            cursor_s += idle
        segments.append(
            StageSegment(
                f"{direction_name}_ramp_smooth",
                direction_name,
                "ramp",
                cursor_s,
                cursor_s + ramp,
                0.0,
                peak,
                capture_axis=True,
                force_interp=FORCE_INTERP_SMOOTHSTEP,
            )
        )
        cursor_s += ramp
        segments.append(
            StageSegment(
                f"{direction_name}_hold",
                direction_name,
                "hold",
                cursor_s,
                cursor_s + hold,
                peak,
                peak,
            )
        )
        cursor_s += hold
        segments.append(
            StageSegment(
                f"{direction_name}_release_smooth",
                direction_name,
                "release",
                cursor_s,
                cursor_s + release,
                peak,
                0.0,
                force_interp=FORCE_INTERP_SMOOTHSTEP,
            )
        )
        cursor_s += release
        if idle > 0.0:
            segments.append(StageSegment(f"{direction_name}_idle_post", DIR_NONE, "idle", cursor_s, cursor_s + idle))
            cursor_s += idle

    if not segments:
        return (StageSegment("idle_0", DIR_NONE, "idle", 0.0, EPISODE_LENGTH_S),)
    return tuple(segments)


def build_force_profile_segments(args: argparse.Namespace) -> tuple[StageSegment, ...]:
    if args.force_profile == FORCE_PROFILE_LEGACY_STAGED:
        return STAGE_SEGMENTS
    if args.force_profile == FORCE_PROFILE_SMOOTH_0_20:
        return build_smooth_0_20_segments(
            peak_force_n=args.peak_force_n,
            ramp_s=args.ramp_s,
            hold_s=args.hold_s,
            release_s=args.release_s,
            idle_s=args.idle_s,
        )
    raise ValueError(f"Unsupported force profile: {args.force_profile}")


def direction_axis_from_forward(forward_axis_xy: np.ndarray, direction_name: str) -> np.ndarray:
    if direction_name == DIR_FORWARD:
        return forward_axis_xy.astype(np.float32)
    if direction_name == DIR_LEFT:
        return np.array([-forward_axis_xy[1], forward_axis_xy[0]], dtype=np.float32)
    if direction_name == DIR_BACKWARD:
        return (-forward_axis_xy).astype(np.float32)
    return np.zeros(2, dtype=np.float32)


class StagedDirectionForceScheduler:
    def __init__(self, segments: tuple[StageSegment, ...]):
        if not segments:
            raise ValueError("StagedDirectionForceScheduler requires at least one segment.")
        self.segments = segments
        self.current_segment_name: str | None = None
        self.current_axis_direction_name: str | None = None
        self.current_axis_xy = np.full(2, np.nan, dtype=np.float32)
        self.current_block_start_root_xy = np.full(2, np.nan, dtype=np.float32)

    def segment_for_time(self, time_s: float) -> StageSegment:
        if time_s >= self.segments[-1].start_s:
            return self.segments[-1]
        for segment in self.segments:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return self.segments[-1]

    def update(self, base_quat_wxyz: np.ndarray, root_pos_xy: np.ndarray, time_s: float) -> dict[str, object]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name

        should_capture_axis = segment_changed and (
            segment.capture_axis
            or (segment.is_active and segment.direction_name != self.current_axis_direction_name)
            or (segment.is_active and not np.all(np.isfinite(self.current_axis_xy)))
        )
        if should_capture_axis:
            forward_axis_xy = base_forward_axis_xy(base_quat_wxyz)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)
            self.current_axis_direction_name = segment.direction_name
            self.current_block_start_root_xy = root_pos_xy.astype(np.float32, copy=True)

        force_target_n = segment.force_target_at(time_s)
        force_w = np.zeros(3, dtype=np.float32)
        if segment.is_active:
            axis_xy = self.current_axis_xy.copy()
            block_start_root_xy = self.current_block_start_root_xy.copy()
            force_w[:2] = axis_xy * force_target_n
        else:
            axis_xy = np.full(2, np.nan, dtype=np.float32)
            block_start_root_xy = np.full(2, np.nan, dtype=np.float32)

        self.current_segment_name = segment.segment_name
        return {
            "segment_name": segment.segment_name,
            "direction_name": segment.direction_name,
            "phase_name": segment.phase_name,
            "force_target_n": float(force_target_n),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "block_start_root_xy": block_start_root_xy,
            "is_active": segment.is_active,
        }


def resolve_output_csv(path_str: str | None) -> Path:
    if path_str:
        output_path = Path(path_str).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = (
            Path("logs")
            / "experiments"
            / "unifp_staged_direction_force_mujoco"
            / f"staged_direction_force_{stamp}.csv"
        ).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def resolve_summary_json(output_csv: Path) -> Path:
    return output_csv.with_suffix(output_csv.suffix + ".summary.json")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate the A2 UniFP policy in MuJoCo under the staged-direction-force schedule."
    )
    parser.add_argument(
        "--model-path",
        type=Path,
        default=Path("resources/a2_description/a2.xml"),
        help="Path to a MuJoCo-loadable URDF or MJCF model.",
    )
    parser.add_argument("--policy-path", type=Path, required=True, help="Path to the MuJoCo-ready TorchScript policy.")
    parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Evaluation horizon in seconds. Defaults to the full selected force profile.",
    )
    parser.add_argument("--cmd-vx", type=float, default=0.0, help="Fixed body-frame x velocity command.")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Fixed body-frame y velocity command.")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="Fixed yaw-rate command.")
    parser.add_argument(
        "--force-profile",
        type=str,
        default=FORCE_PROFILE_LEGACY_STAGED,
        choices=(FORCE_PROFILE_LEGACY_STAGED, FORCE_PROFILE_SMOOTH_0_20),
        help="Force schedule profile. The default preserves the previous staged 10-40N benchmark.",
    )
    parser.add_argument("--peak-force-n", type=float, default=20.0, help="Peak force for smooth_0_20.")
    parser.add_argument("--ramp-s", type=float, default=4.0, help="Smooth ramp duration per direction.")
    parser.add_argument("--hold-s", type=float, default=4.0, help="Peak hold duration per direction.")
    parser.add_argument("--release-s", type=float, default=3.0, help="Smooth release duration per direction.")
    parser.add_argument("--idle-s", type=float, default=2.0, help="Idle duration before and after each direction.")
    parser.add_argument(
        "--obs-delay-steps",
        type=int,
        default=0,
        choices=(0, 1, 2),
        help="Fixed proprioception delay in control steps.",
    )
    parser.add_argument(
        "--leg-action-delay-steps",
        type=int,
        default=0,
        choices=(0, 1, 2),
        help="Fixed leg action delay in control steps.",
    )
    parser.add_argument("--headless", action="store_true", help="Run without the MuJoCo viewer.")
    parser.add_argument("--real-time", action="store_true", help="Sleep to match wall-clock control rate.")
    return parser


def write_csv_row(
    writer: csv.writer,
    *,
    step: int,
    time_s: float,
    segment_name: str,
    direction_name: str,
    phase_name: str,
    cmd_vel_b: np.ndarray,
    force_target_n: float,
    force_w: np.ndarray,
    axis_xy: np.ndarray,
    proj_speed: float,
    cross_speed: float,
    proj_disp: float,
    cross_disp: float,
    root_pos_xy: np.ndarray,
    gripper_force_est_local: np.ndarray,
    gripper_force_tgt_local: np.ndarray,
    gripper_force_err_l2: float,
    base_force_est_local: np.ndarray,
    base_force_tgt_local: np.ndarray,
    base_force_err_l2: float,
) -> None:
    writer.writerow(
        [
            0,
            step,
            f"{time_s:.6f}",
            segment_name,
            direction_name,
            phase_name,
            f"{cmd_vel_b[0]:.6f}",
            f"{cmd_vel_b[1]:.6f}",
            f"{cmd_vel_b[2]:.6f}",
            f"{force_target_n:.6f}",
            f"{force_w[0]:.6f}",
            f"{force_w[1]:.6f}",
            f"{axis_xy[0]:.6f}",
            f"{axis_xy[1]:.6f}",
            f"{proj_speed:.6f}",
            f"{cross_speed:.6f}",
            f"{proj_disp:.6f}",
            f"{cross_disp:.6f}",
            f"{root_pos_xy[0]:.6f}",
            f"{root_pos_xy[1]:.6f}",
            f"{gripper_force_est_local[0]:.6f}",
            f"{gripper_force_est_local[1]:.6f}",
            f"{gripper_force_est_local[2]:.6f}",
            f"{gripper_force_tgt_local[0]:.6f}",
            f"{gripper_force_tgt_local[1]:.6f}",
            f"{gripper_force_tgt_local[2]:.6f}",
            f"{gripper_force_err_l2:.6f}",
            f"{base_force_est_local[0]:.6f}",
            f"{base_force_est_local[1]:.6f}",
            f"{base_force_est_local[2]:.6f}",
            f"{base_force_tgt_local[0]:.6f}",
            f"{base_force_tgt_local[1]:.6f}",
            f"{base_force_tgt_local[2]:.6f}",
            f"{base_force_err_l2:.6f}",
        ]
    )


def extract_force_estimator_outputs(
    pred_est: np.ndarray,
    base_force_tgt_local: np.ndarray,
    *,
    base_force_scale: float,
) -> dict[str, np.ndarray | float]:
    scale = max(float(base_force_scale), 1.0e-6)
    if pred_est.shape == (12,):
        gripper_force_est_local = np.zeros(3, dtype=np.float32)
        base_force_est_local = pred_est[9:12] / scale
    elif pred_est.shape == (6,):
        gripper_force_est_local = np.zeros(3, dtype=np.float32)
        base_force_est_local = pred_est[3:6] / scale
    elif pred_est.shape == (3,):
        gripper_force_est_local = np.zeros(3, dtype=np.float32)
        base_force_est_local = pred_est / scale
    else:
        raise RuntimeError(f"Expected pred_est shape (3,), (6,), or (12,), received {pred_est.shape}")

    gripper_force_tgt_local = np.zeros(3, dtype=np.float32)
    base_force_err_l2 = float(np.linalg.norm(base_force_est_local - base_force_tgt_local))
    gripper_force_err_l2 = float(np.linalg.norm(gripper_force_est_local - gripper_force_tgt_local))
    return {
        "gripper_force_est_local": gripper_force_est_local.astype(np.float32),
        "gripper_force_tgt_local": gripper_force_tgt_local,
        "gripper_force_err_l2": gripper_force_err_l2,
        "base_force_est_local": base_force_est_local.astype(np.float32),
        "base_force_tgt_local": base_force_tgt_local.astype(np.float32),
        "base_force_err_l2": base_force_err_l2,
    }


def json_safe_float(value: float | np.floating | None) -> float | None:
    if value is None:
        return None
    value_f = float(value)
    return value_f if math.isfinite(value_f) else None


def empty_direction_metrics(sample_count_total: int = 0) -> dict[str, object]:
    return {
        "sample_count_total": int(sample_count_total),
        "sample_count_active": 0,
        "active_mean_l2": None,
        "rmse_l2": None,
        "peak_l2": None,
        "per_axis_mae": {"x": None, "y": None, "z": None},
        "primary_axis_mae": None,
        "lag_seconds": None,
        "correlation": None,
    }


def best_lag_correlation(
    target_primary: np.ndarray,
    estimate_primary: np.ndarray,
    *,
    step_dt: float,
    max_lag_s: float = 1.0,
) -> tuple[float | None, float | None]:
    target = np.asarray(target_primary, dtype=np.float64)
    estimate = np.asarray(estimate_primary, dtype=np.float64)
    finite_mask = np.isfinite(target) & np.isfinite(estimate)
    target = target[finite_mask]
    estimate = estimate[finite_mask]
    if target.size < 3 or estimate.size < 3:
        return None, None

    max_lag_steps = max(int(round(max_lag_s / max(float(step_dt), 1.0e-9))), 0)
    best_lag_steps: int | None = None
    best_corr: float | None = None
    for lag_steps in range(-max_lag_steps, max_lag_steps + 1):
        if lag_steps > 0:
            target_slice = target[:-lag_steps]
            estimate_slice = estimate[lag_steps:]
        elif lag_steps < 0:
            target_slice = target[-lag_steps:]
            estimate_slice = estimate[:lag_steps]
        else:
            target_slice = target
            estimate_slice = estimate

        if target_slice.size < 3:
            continue
        target_std = float(np.std(target_slice))
        estimate_std = float(np.std(estimate_slice))
        if target_std <= 1.0e-9 or estimate_std <= 1.0e-9:
            continue
        corr = float(np.corrcoef(target_slice, estimate_slice)[0, 1])
        if not math.isfinite(corr):
            continue
        if best_corr is None or corr > best_corr:
            best_corr = corr
            best_lag_steps = lag_steps

    if best_corr is None or best_lag_steps is None:
        return None, None
    return float(best_lag_steps) * float(step_dt), best_corr


def compute_direction_metrics(samples: list[dict[str, object]], *, step_dt: float) -> dict[str, object]:
    if not samples:
        return empty_direction_metrics()

    est = np.asarray([sample["base_force_est_local"] for sample in samples], dtype=np.float64)
    tgt = np.asarray([sample["base_force_tgt_local"] for sample in samples], dtype=np.float64)
    err_l2 = np.asarray([sample["base_force_err_l2"] for sample in samples], dtype=np.float64)
    target_norm = np.linalg.norm(tgt, axis=1)
    active_mask = np.isfinite(target_norm) & (target_norm > 1.0)
    if not np.any(active_mask):
        return empty_direction_metrics(sample_count_total=len(samples))

    est_active = est[active_mask]
    tgt_active = tgt[active_mask]
    err_l2_active = err_l2[active_mask]
    err_active = est_active - tgt_active
    axis_mae = np.mean(np.abs(err_active), axis=0)

    target_norm_active = np.linalg.norm(tgt_active, axis=1)
    unit_target = tgt_active / np.maximum(target_norm_active[:, None], 1.0e-9)
    target_primary = np.sum(tgt_active * unit_target, axis=1)
    estimate_primary = np.sum(est_active * unit_target, axis=1)
    primary_axis_mae = float(np.mean(np.abs(estimate_primary - target_primary)))
    lag_seconds, correlation = best_lag_correlation(
        target_primary,
        estimate_primary,
        step_dt=step_dt,
        max_lag_s=1.0,
    )

    return {
        "sample_count_total": int(len(samples)),
        "sample_count_active": int(np.count_nonzero(active_mask)),
        "active_mean_l2": json_safe_float(np.mean(err_l2_active)),
        "rmse_l2": json_safe_float(math.sqrt(float(np.mean(np.square(err_l2_active))))),
        "peak_l2": json_safe_float(np.max(err_l2_active)),
        "per_axis_mae": {
            "x": json_safe_float(axis_mae[0]),
            "y": json_safe_float(axis_mae[1]),
            "z": json_safe_float(axis_mae[2]),
        },
        "primary_axis_mae": json_safe_float(primary_axis_mae),
        "lag_seconds": json_safe_float(lag_seconds),
        "correlation": json_safe_float(correlation),
    }


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if mujoco is None:
        print("MuJoCo is not installed. Install dependencies from scripts/mujoco/requirements.txt first.", file=sys.stderr)
        return 1

    output_csv = resolve_output_csv(args.output_csv)
    summary_json = resolve_summary_json(output_csv)
    force_segments = build_force_profile_segments(args)
    profile_duration_s = float(force_segments[-1].end_s)
    duration_s = float(args.duration_s) if args.duration_s is not None else profile_duration_s
    duration_s = max(duration_s, 0.0)
    metadata = load_policy_metadata(args.policy_path, fallback=default_staged_eval_metadata())
    metadata = default_staged_eval_metadata() if metadata is None else metadata
    locomotion_cmd = np.array([float(args.cmd_vx), float(args.cmd_vy), float(args.cmd_wz)], dtype=np.float32)
    initial_command_obs = build_command_vector(
        locomotion_cmd=locomotion_cmd,
        metadata=metadata,
    )

    model = load_mujoco_model_from_path(mujoco, args.model_path)
    data = mujoco.MjData(model)
    if model.nu == 0:
        print("The MuJoCo model has no actuators; staged policy evaluation cannot run.", file=sys.stderr)
        return 2

    bindings = create_model_bindings(model, metadata)
    base_body_id = resolve_body_id(model, metadata.base_body_name)
    apply_joint_actuator_gains(model, bindings, metadata)
    policy = load_torchscript_policy(args.policy_path)

    reset_model_state(mujoco, model, data, metadata, bindings.qpos_indices)
    clear_body_forces(data)
    runtime_state = initialize_runtime_state(
        data=data,
        bindings=bindings,
        metadata=metadata,
        obs_delay_steps=args.obs_delay_steps,
        leg_action_delay_steps=args.leg_action_delay_steps,
        command_obs=initial_command_obs,
    )

    print(
        "[INFO] Runtime config: "
        f"gait_frequency_hz={metadata.gait_frequency_hz:.3f} "
        f"model_timestep={model.opt.timestep:.4f} "
        f"control_dt={metadata.control_dt:.4f} "
        f"obs_delay_steps={args.obs_delay_steps} "
        f"leg_action_delay_steps={args.leg_action_delay_steps} "
        f"base_body_name={metadata.base_body_name} "
        f"actuator_gains={summarize_joint_actuator_gains(metadata)}"
    )
    print(
        "[INFO] Force benchmark: "
        f"force_profile={args.force_profile} "
        f"command=({locomotion_cmd[0]:.3f}, {locomotion_cmd[1]:.3f}, {locomotion_cmd[2]:.3f}) "
        f"profile_duration_s={profile_duration_s:.3f} "
        f"duration_s={duration_s:.3f}"
    )
    print(f"[INFO] CSV output: {output_csv}")
    print(f"[INFO] Summary JSON output: {summary_json}")

    step_dt = float(metadata.control_dt)
    max_steps = max(int(round(duration_s / step_dt)), 1)
    scheduler = StagedDirectionForceScheduler(force_segments)

    block_last_proj_disp = np.full(len(ACTIVE_DIRECTIONS), np.nan, dtype=np.float64)
    block_last_cross_disp = np.full(len(ACTIVE_DIRECTIONS), np.nan, dtype=np.float64)
    block_peak_abs_cross_speed = np.full(len(ACTIVE_DIRECTIONS), -np.inf, dtype=np.float64)
    active_force_sample_count = 0
    base_force_err_active_sum = 0.0
    base_force_err_active_max = 0.0
    gripper_force_err_active_sum = 0.0
    gripper_force_err_active_max = 0.0

    ended_early = False
    end_step = max_steps
    last_logged_payload: dict[str, object] | None = None
    direction_metric_samples: dict[str, list[dict[str, object]]] = {name: [] for name in ACTIVE_DIRECTIONS}

    csv_header = [
        "env_id",
        "step",
        "time_s",
        "segment_name",
        "direction_name",
        "phase_name",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "force_target_n",
        "force_x",
        "force_y",
        "axis_x",
        "axis_y",
        "proj_speed",
        "cross_speed",
        "proj_disp",
        "cross_disp",
        "root_x",
        "root_y",
        "gripper_force_est_local_x",
        "gripper_force_est_local_y",
        "gripper_force_est_local_z",
        "gripper_force_tgt_local_x",
        "gripper_force_tgt_local_y",
        "gripper_force_tgt_local_z",
        "gripper_force_err_l2",
        "base_force_est_local_x",
        "base_force_est_local_y",
        "base_force_est_local_z",
        "base_force_tgt_local_x",
        "base_force_tgt_local_y",
        "base_force_tgt_local_z",
        "base_force_err_l2",
    ]

    def run_one_control_step(step: int, writer: csv.writer) -> None:
        nonlocal active_force_sample_count
        nonlocal base_force_err_active_sum
        nonlocal base_force_err_active_max
        nonlocal gripper_force_err_active_sum
        nonlocal gripper_force_err_active_max
        nonlocal last_logged_payload

        time_s = step * step_dt
        root_pos_xy = data.qpos[:2].astype(np.float32, copy=True)
        root_vel_xy = data.qvel[:2].astype(np.float32, copy=True)
        base_quat = data.qpos[3:7].astype(np.float64, copy=True)

        scheduler_outputs = scheduler.update(base_quat, root_pos_xy, time_s)
        set_body_force(data, base_body_id, scheduler_outputs["force_w"])

        axis_xy = np.asarray(scheduler_outputs["axis_xy"], dtype=np.float32)
        block_start_root_xy = np.asarray(scheduler_outputs["block_start_root_xy"], dtype=np.float32)
        proj_speed = float("nan")
        cross_speed = float("nan")
        proj_disp = float("nan")
        cross_disp = float("nan")
        if bool(scheduler_outputs["is_active"]):
            cross_axis_xy = np.array([-axis_xy[1], axis_xy[0]], dtype=np.float32)
            proj_speed = float(np.dot(root_vel_xy, axis_xy))
            cross_speed = float(np.dot(root_vel_xy, cross_axis_xy))
            rel_pos_xy = root_pos_xy - block_start_root_xy
            proj_disp = float(np.dot(rel_pos_xy, axis_xy))
            cross_disp = float(np.dot(rel_pos_xy, cross_axis_xy))

        obs = runtime_state.policy_obs()
        action, pred_est = infer_policy_outputs(policy, obs, action_dim=metadata.action_dim)
        action = np.clip(action, -metadata.action_clip, metadata.action_clip)
        if pred_est is None:
            raise RuntimeError(
                "The staged evaluator requires a MuJoCo export that returns (action, pred_est). "
                "Re-export the policy with scripts/mujoco/export_policy_wrapper.py."
            )

        estimator_outputs = extract_force_estimator_outputs(
            pred_est.astype(np.float32),
            world_to_base_yaw(np.asarray(scheduler_outputs["force_w"], dtype=np.float32), base_quat),
            base_force_scale=metadata.base_force_scale,
        )
        direction_name = str(scheduler_outputs["direction_name"])
        if direction_name in direction_metric_samples:
            direction_metric_samples[direction_name].append(
                {
                    "time_s": time_s,
                    "base_force_est_local": np.asarray(
                        estimator_outputs["base_force_est_local"],
                        dtype=np.float32,
                    ).copy(),
                    "base_force_tgt_local": np.asarray(
                        estimator_outputs["base_force_tgt_local"],
                        dtype=np.float32,
                    ).copy(),
                    "base_force_err_l2": float(estimator_outputs["base_force_err_l2"]),
                }
            )

        write_csv_row(
            writer,
            step=step,
            time_s=time_s,
            segment_name=str(scheduler_outputs["segment_name"]),
            direction_name=direction_name,
            phase_name=str(scheduler_outputs["phase_name"]),
            cmd_vel_b=locomotion_cmd,
            force_target_n=float(scheduler_outputs["force_target_n"]),
            force_w=np.asarray(scheduler_outputs["force_w"], dtype=np.float32),
            axis_xy=axis_xy,
            proj_speed=proj_speed,
            cross_speed=cross_speed,
            proj_disp=proj_disp,
            cross_disp=cross_disp,
            root_pos_xy=root_pos_xy,
            gripper_force_est_local=np.asarray(estimator_outputs["gripper_force_est_local"], dtype=np.float32),
            gripper_force_tgt_local=np.asarray(estimator_outputs["gripper_force_tgt_local"], dtype=np.float32),
            gripper_force_err_l2=float(estimator_outputs["gripper_force_err_l2"]),
            base_force_est_local=np.asarray(estimator_outputs["base_force_est_local"], dtype=np.float32),
            base_force_tgt_local=np.asarray(estimator_outputs["base_force_tgt_local"], dtype=np.float32),
            base_force_err_l2=float(estimator_outputs["base_force_err_l2"]),
        )

        last_logged_payload = {
            "step": step,
            "time_s": time_s,
            "root_pos_xy": root_pos_xy.copy(),
            "estimator_outputs": estimator_outputs,
        }

        if bool(scheduler_outputs["is_active"]):
            direction_idx = DIRECTION_INDEX[str(scheduler_outputs["direction_name"])]
            block_last_proj_disp[direction_idx] = proj_disp
            block_last_cross_disp[direction_idx] = cross_disp
            block_peak_abs_cross_speed[direction_idx] = max(block_peak_abs_cross_speed[direction_idx], abs(cross_speed))
            active_force_sample_count += 1
            base_force_err_active_sum += float(estimator_outputs["base_force_err_l2"])
            base_force_err_active_max = max(base_force_err_active_max, float(estimator_outputs["base_force_err_l2"]))
            gripper_force_err_active_sum += float(estimator_outputs["gripper_force_err_l2"])
            gripper_force_err_active_max = max(
                gripper_force_err_active_max,
                float(estimator_outputs["gripper_force_err_l2"]),
            )

        target_joint_pos = build_target_joint_positions(action, runtime_state, metadata)
        apply_control_targets(model, data, bindings, target_joint_pos, metadata)

        wall_start = time.perf_counter()
        try:
            mujoco.mj_step(model, data, nstep=metadata.control_decimation)
        except TypeError:
            for _ in range(metadata.control_decimation):
                mujoco.mj_step(model, data)

        next_command_obs = build_command_vector(
            locomotion_cmd=locomotion_cmd,
            metadata=metadata,
        )
        update_runtime_state_after_step(
            data=data,
            bindings=bindings,
            metadata=metadata,
            runtime_state=runtime_state,
            action=action,
            command_obs=next_command_obs,
        )
        if args.real_time:
            sleep_time = metadata.control_dt - (time.perf_counter() - wall_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(csv_header)

        if args.headless:
            for step in range(max_steps):
                run_one_control_step(step, writer)
            end_step = max_steps
        else:
            if mujoco_viewer is None:
                print("MuJoCo viewer is unavailable in this environment. Re-run with --headless.", file=sys.stderr)
                return 3
            with mujoco_viewer.launch_passive(model, data) as viewer:
                with viewer.lock():
                    configure_tracking_viewer_camera(mujoco, viewer, data, track_body_id=base_body_id)
                viewer.sync()
                for step in range(max_steps):
                    if not viewer.is_running():
                        ended_early = True
                        end_step = step
                        break
                    run_one_control_step(step, writer)
                    # Rebuild custom user_scn geoms before sync so MuJoCo incorporates them this frame.
                    draw_velocity_arrows(
                        mujoco,
                        viewer,
                        data,
                        base_body_id=base_body_id,
                        command_vel_xy=locomotion_cmd[:2],
                        external_force_w=data.xfrc_applied[base_body_id, :3],
                        force_arrow_z_extra_offset=0.05,
                    )
                    viewer.sync()
                else:
                    end_step = max_steps

        final_time_s = duration_s
        if last_logged_payload is not None and float(last_logged_payload["time_s"]) < final_time_s:
            final_segment = scheduler.segment_for_time(final_time_s)
            nan_axis_xy = np.full(2, np.nan, dtype=np.float32)
            nan_scalar = float("nan")
            zero_force = np.zeros(3, dtype=np.float32)
            estimator_outputs = last_logged_payload["estimator_outputs"]
            write_csv_row(
                writer,
                step=max_steps,
                time_s=final_time_s,
                segment_name=final_segment.segment_name,
                direction_name=final_segment.direction_name,
                phase_name=final_segment.phase_name,
                cmd_vel_b=locomotion_cmd,
                force_target_n=0.0,
                force_w=zero_force,
                axis_xy=nan_axis_xy,
                proj_speed=nan_scalar,
                cross_speed=nan_scalar,
                proj_disp=nan_scalar,
                cross_disp=nan_scalar,
                root_pos_xy=np.asarray(last_logged_payload["root_pos_xy"], dtype=np.float32),
                gripper_force_est_local=np.asarray(estimator_outputs["gripper_force_est_local"], dtype=np.float32),
                gripper_force_tgt_local=np.asarray(estimator_outputs["gripper_force_tgt_local"], dtype=np.float32),
                gripper_force_err_l2=float(estimator_outputs["gripper_force_err_l2"]),
                base_force_est_local=np.asarray(estimator_outputs["base_force_est_local"], dtype=np.float32),
                base_force_tgt_local=np.asarray(estimator_outputs["base_force_tgt_local"], dtype=np.float32),
                base_force_err_l2=float(estimator_outputs["base_force_err_l2"]),
            )

    clear_body_forces(data)

    print(f"[INFO] CSV written to: {output_csv}")
    if ended_early:
        print(f"[WARN] Evaluation ended before the planned timeout-only horizon at step={end_step}.")
    else:
        print(f"[INFO] Evaluation reached the planned timeout horizon at step={end_step}.")

    base_force_err_active_mean = (
        base_force_err_active_sum / active_force_sample_count if active_force_sample_count > 0 else float("nan")
    )
    gripper_force_err_active_mean = (
        gripper_force_err_active_sum / active_force_sample_count if active_force_sample_count > 0 else float("nan")
    )
    base_force_err_peak = base_force_err_active_max if active_force_sample_count > 0 else float("nan")
    gripper_force_err_peak = gripper_force_err_active_max if active_force_sample_count > 0 else float("nan")

    forward_proj_disp = finite_or_nan(block_last_proj_disp[DIRECTION_INDEX[DIR_FORWARD]])
    left_proj_disp = finite_or_nan(block_last_proj_disp[DIRECTION_INDEX[DIR_LEFT]])
    backward_proj_disp = finite_or_nan(block_last_proj_disp[DIRECTION_INDEX[DIR_BACKWARD]])

    forward_cross_disp = finite_or_nan(block_last_cross_disp[DIRECTION_INDEX[DIR_FORWARD]])
    left_cross_disp = finite_or_nan(block_last_cross_disp[DIRECTION_INDEX[DIR_LEFT]])
    backward_cross_disp = finite_or_nan(block_last_cross_disp[DIRECTION_INDEX[DIR_BACKWARD]])

    forward_cross_speed_peak = finite_or_nan(block_peak_abs_cross_speed[DIRECTION_INDEX[DIR_FORWARD]])
    left_cross_speed_peak = finite_or_nan(block_peak_abs_cross_speed[DIRECTION_INDEX[DIR_LEFT]])
    backward_cross_speed_peak = finite_or_nan(block_peak_abs_cross_speed[DIRECTION_INDEX[DIR_BACKWARD]])

    if forward_cross_speed_peak < 0.0:
        forward_cross_speed_peak = float("nan")
    if left_cross_speed_peak < 0.0:
        left_cross_speed_peak = float("nan")
    if backward_cross_speed_peak < 0.0:
        backward_cross_speed_peak = float("nan")

    print(
        "[SUMMARY][env 0] "
        f"forward_proj_disp={format_optional(forward_proj_disp)} "
        f"forward_cross_disp={format_optional(forward_cross_disp)} "
        f"forward_cross_speed_peak={format_optional(forward_cross_speed_peak)} "
        f"left_proj_disp={format_optional(left_proj_disp)} "
        f"left_cross_disp={format_optional(left_cross_disp)} "
        f"left_cross_speed_peak={format_optional(left_cross_speed_peak)} "
        f"backward_proj_disp={format_optional(backward_proj_disp)} "
        f"backward_cross_disp={format_optional(backward_cross_disp)} "
        f"backward_cross_speed_peak={format_optional(backward_cross_speed_peak)} "
        f"base_force_err_active_mean={format_optional(base_force_err_active_mean)} "
        f"base_force_err_peak={format_optional(base_force_err_peak)} "
        f"gripper_force_err_active_mean={format_optional(gripper_force_err_active_mean)} "
        f"gripper_force_err_peak={format_optional(gripper_force_err_peak)}"
    )
    per_direction_metrics = {
        direction_name: compute_direction_metrics(direction_metric_samples[direction_name], step_dt=step_dt)
        for direction_name in ACTIVE_DIRECTIONS
    }
    summary_payload = {
        "policy_path": str(Path(args.policy_path).expanduser().resolve()),
        "model_path": str(Path(args.model_path).expanduser().resolve()),
        "output_csv": str(output_csv),
        "summary_json": str(summary_json),
        "command": {
            "cmd_vx": float(locomotion_cmd[0]),
            "cmd_vy": float(locomotion_cmd[1]),
            "cmd_wz": float(locomotion_cmd[2]),
        },
        "force_profile": args.force_profile,
        "force_profile_duration_s": json_safe_float(profile_duration_s),
        "peak_force_n": json_safe_float(args.peak_force_n),
        "ramp_s": json_safe_float(args.ramp_s),
        "hold_s": json_safe_float(args.hold_s),
        "release_s": json_safe_float(args.release_s),
        "idle_s": json_safe_float(args.idle_s),
        "model_timestep": json_safe_float(model.opt.timestep),
        "control_dt": json_safe_float(metadata.control_dt),
        "obs_delay_steps": int(args.obs_delay_steps),
        "leg_action_delay_steps": int(args.leg_action_delay_steps),
        "duration_s": json_safe_float(duration_s),
        "max_steps": int(max_steps),
        "end_step": int(end_step),
        "ended_early": bool(ended_early),
        "active_force_sample_count": int(active_force_sample_count),
        "base_force_err_active_mean": json_safe_float(base_force_err_active_mean),
        "base_force_err_peak": json_safe_float(base_force_err_peak),
        "per_direction_metrics": per_direction_metrics,
    }
    with summary_json.open("w") as summary_file:
        json.dump(summary_payload, summary_file, indent=2, sort_keys=True, allow_nan=False)
        summary_file.write("\n")
    print(f"[INFO] Summary JSON written to: {summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
