#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
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
    default_staged_eval_metadata,
    draw_velocity_arrows,
    finite_or_nan,
    format_optional,
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


def direction_axis_from_forward(forward_axis_xy: np.ndarray, direction_name: str) -> np.ndarray:
    if direction_name == DIR_FORWARD:
        return forward_axis_xy.astype(np.float32)
    if direction_name == DIR_LEFT:
        return np.array([-forward_axis_xy[1], forward_axis_xy[0]], dtype=np.float32)
    if direction_name == DIR_BACKWARD:
        return (-forward_axis_xy).astype(np.float32)
    return np.zeros(2, dtype=np.float32)


class StagedDirectionForceScheduler:
    def __init__(self):
        self.current_segment_name: str | None = None
        self.current_axis_xy = np.full(2, np.nan, dtype=np.float32)
        self.current_block_start_root_xy = np.full(2, np.nan, dtype=np.float32)

    def segment_for_time(self, time_s: float) -> StageSegment:
        if time_s >= STAGE_SEGMENTS[-1].start_s:
            return STAGE_SEGMENTS[-1]
        for segment in STAGE_SEGMENTS:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return STAGE_SEGMENTS[-1]

    def update(self, base_quat_wxyz: np.ndarray, root_pos_xy: np.ndarray, time_s: float) -> dict[str, object]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name

        if segment_changed and segment.capture_axis:
            forward_axis_xy = base_forward_axis_xy(base_quat_wxyz)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)
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
    parser.add_argument("--duration-s", type=float, default=EPISODE_LENGTH_S, help="Evaluation horizon in seconds.")
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


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if mujoco is None:
        print("MuJoCo is not installed. Install dependencies from scripts/mujoco/requirements.txt first.", file=sys.stderr)
        return 1

    output_csv = resolve_output_csv(args.output_csv)
    metadata = load_policy_metadata(args.policy_path, fallback=default_staged_eval_metadata())
    metadata = default_staged_eval_metadata() if metadata is None else metadata
    locomotion_cmd = parse_triplet(metadata.default_locomotion_cmd)
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
    print(f"[INFO] CSV output: {output_csv}")

    step_dt = float(metadata.control_dt)
    duration_s = float(args.duration_s)
    max_steps = max(int(round(duration_s / step_dt)), 1)
    scheduler = StagedDirectionForceScheduler()

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

        write_csv_row(
            writer,
            step=step,
            time_s=time_s,
            segment_name=str(scheduler_outputs["segment_name"]),
            direction_name=str(scheduler_outputs["direction_name"]),
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
