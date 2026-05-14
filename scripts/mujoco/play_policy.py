#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from model_loader import load_mujoco_model_from_path
from runtime import (
    apply_control_targets,
    apply_joint_actuator_gains,
    build_command_vector,
    build_target_joint_positions,
    clear_body_forces,
    configure_tracking_viewer_camera,
    create_model_bindings,
    default_runtime_metadata,
    infer_policy_outputs,
    initialize_runtime_state,
    load_policy_metadata,
    load_torchscript_policy,
    parse_triplet,
    reset_model_state,
    resolve_body_id,
    summarize_joint_actuator_gains,
    update_runtime_state_after_step,
    draw_velocity_arrows,
)

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import mujoco.viewer as mujoco_viewer
except ImportError:
    mujoco_viewer = None


DEFAULT_STANDING_LIN_VEL_X_CLIP = 0.1
DEFAULT_STANDING_LIN_VEL_Y_CLIP = 0.1
DEFAULT_STANDING_ANG_VEL_YAW_CLIP = 0.2
VIEWER_IMPORT_DELAY_S = 5.0
VIEWER_DELAY_HIDDEN_BASE_POS = np.array([0.0, 0.0, -20.0], dtype=np.float64)

IMPORT_INIT_HIP_POS = 0.0
IMPORT_INIT_THIGH_POS = 1.45
IMPORT_INIT_CALF_POS = -2.78
IMPORT_INIT_LEG_QPOS = np.array(
    [
        IMPORT_INIT_HIP_POS,
        IMPORT_INIT_HIP_POS,
        IMPORT_INIT_HIP_POS,
        IMPORT_INIT_HIP_POS,
        IMPORT_INIT_THIGH_POS,
        IMPORT_INIT_THIGH_POS,
        IMPORT_INIT_THIGH_POS,
        IMPORT_INIT_THIGH_POS,
        IMPORT_INIT_CALF_POS,
        IMPORT_INIT_CALF_POS,
        IMPORT_INIT_CALF_POS,
        IMPORT_INIT_CALF_POS,
    ],
    dtype=np.float64,
)


def clip_standing_command(
    locomotion_cmd: np.ndarray,
    *,
    lin_vel_x_clip: float,
    lin_vel_y_clip: float,
    ang_vel_yaw_clip: float,
) -> tuple[np.ndarray, bool]:
    standing_command = (
        abs(float(locomotion_cmd[0])) < float(lin_vel_x_clip)
        and abs(float(locomotion_cmd[1])) < float(lin_vel_y_clip)
        and abs(float(locomotion_cmd[2])) < float(ang_vel_yaw_clip)
    )
    if not standing_command:
        return locomotion_cmd.astype(np.float32, copy=False), False
    return np.zeros(3, dtype=np.float32), True


def standing_gait_vector() -> np.ndarray:
    return np.array([0.0, 1.0], dtype=np.float32)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Minimal MuJoCo sim2sim runner for the UniFP Aliengo policy.")
    parser.add_argument(
        "--model-path",
        type=Path,
        default=Path("resources/aliengo/mujoco/aliengo.xml"),
        help="Path to a MuJoCo-loadable URDF or MJCF model.",
    )
    parser.add_argument(
        "--policy-path",
        type=Path,
        default=None,
        help="Path to a TorchScript policy. If omitted, zero actions are sent.",
    )
    parser.add_argument("--duration-s", type=float, default=20.0, help="Simulation duration in seconds.")
    parser.add_argument(
        "--locomotion-cmd",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "WZ"),
        default=None,
        help="Base velocity command before Isaac scaling. Defaults to metadata or built-in values.",
    )
    parser.add_argument(
        "--obs-delay-steps",
        type=int,
        default=0,
        choices=(0, 1, 2),
        help="Fixed proprioception delay in control steps. Training used a random 0-2 range.",
    )
    parser.add_argument(
        "--leg-action-delay-steps",
        type=int,
        default=0,
        choices=(0, 1),
        help="Fixed leg action delay in control steps. Training used a random 0-1 range.",
    )
    parser.add_argument(
        "--standing-lin-vel-x-clip",
        type=float,
        default=DEFAULT_STANDING_LIN_VEL_X_CLIP,
        help="Treat |vx| below this threshold as standing, matching Isaac training command clipping.",
    )
    parser.add_argument(
        "--standing-lin-vel-y-clip",
        type=float,
        default=DEFAULT_STANDING_LIN_VEL_Y_CLIP,
        help="Treat |vy| below this threshold as standing, matching Isaac training command clipping.",
    )
    parser.add_argument(
        "--standing-ang-vel-yaw-clip",
        type=float,
        default=DEFAULT_STANDING_ANG_VEL_YAW_CLIP,
        help="Treat |wz| below this threshold as standing, matching Isaac training command clipping.",
    )
    parser.add_argument("--headless", action="store_true", help="Run without the MuJoCo viewer.")
    parser.add_argument("--real-time", action="store_true", help="Sleep to match wall-clock control rate.")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if mujoco is None:
        print("MuJoCo is not installed. Install dependencies from scripts/mujoco/requirements.txt first.", file=sys.stderr)
        return 1

    metadata = default_runtime_metadata()
    if args.policy_path is not None:
        metadata = load_policy_metadata(args.policy_path, fallback=metadata)

    locomotion_cmd = parse_triplet(args.locomotion_cmd or metadata.default_locomotion_cmd)
    locomotion_cmd, standing_command = clip_standing_command(
        locomotion_cmd,
        lin_vel_x_clip=args.standing_lin_vel_x_clip,
        lin_vel_y_clip=args.standing_lin_vel_y_clip,
        ang_vel_yaw_clip=args.standing_ang_vel_yaw_clip,
    )
    gait_override = standing_gait_vector() if standing_command else None
    command_obs = build_command_vector(
        locomotion_cmd=locomotion_cmd,
        metadata=metadata,
    )

    model = load_mujoco_model_from_path(mujoco, args.model_path)
    data = mujoco.MjData(model)
    if model.nu == 0:
        print(
            "The model loaded successfully but exposes no actuators. "
            "URDF is useful for geometry import, but you still need MuJoCo actuators for policy playback.",
            file=sys.stderr,
        )
        return 2

    bindings = create_model_bindings(model, metadata)
    trunk_body_id = resolve_body_id(model, "trunk")
    apply_joint_actuator_gains(model, bindings, metadata)
    policy = load_torchscript_policy(args.policy_path) if args.policy_path else None

    def initialize_robot_runtime_state():
        reset_model_state(mujoco, model, data, metadata, bindings.qpos_indices)
        data.qpos[bindings.qpos_indices] = IMPORT_INIT_LEG_QPOS
        mujoco.mj_forward(model, data)
        clear_body_forces(data)
        return initialize_runtime_state(
            data=data,
            bindings=bindings,
            metadata=metadata,
            obs_delay_steps=args.obs_delay_steps,
            leg_action_delay_steps=args.leg_action_delay_steps,
            command_obs=command_obs,
            gait_override=gait_override,
        )

    def hide_robot_during_viewer_delay() -> None:
        mujoco.mj_resetData(model, data)
        if model.nq >= 7:
            data.qpos[0:3] = VIEWER_DELAY_HIDDEN_BASE_POS
            data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        clear_body_forces(data)
        mujoco.mj_forward(model, data)

    def print_runtime_config() -> None:
        print(
            "[INFO] Runtime config: "
            f"gait_frequency_hz={metadata.gait_frequency_hz:.3f} "
            f"obs_delay_steps={args.obs_delay_steps} "
            f"leg_action_delay_steps={args.leg_action_delay_steps} "
            f"standing_command={standing_command} "
            f"actuator_gains={summarize_joint_actuator_gains(metadata)}"
        )

    def run_control_step(runtime_state) -> None:
        obs = runtime_state.policy_obs()
        action, _ = infer_policy_outputs(policy, obs, action_dim=metadata.action_dim)
        action = np.clip(action, -metadata.action_clip, metadata.action_clip)
        target_joint_pos = build_target_joint_positions(action, runtime_state, metadata)
        apply_control_targets(data, bindings, target_joint_pos)

        wall_start = time.perf_counter()
        try:
            mujoco.mj_step(model, data, nstep=metadata.control_decimation)
        except TypeError:
            for _ in range(metadata.control_decimation):
                mujoco.mj_step(model, data)

        update_runtime_state_after_step(
            data=data,
            bindings=bindings,
            metadata=metadata,
            runtime_state=runtime_state,
            action=action,
            command_obs=command_obs,
            gait_override=gait_override,
        )

        if args.real_time:
            sleep_time = metadata.control_dt - (time.perf_counter() - wall_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    if args.headless:
        runtime_state = initialize_robot_runtime_state()
        print_runtime_config()
        while data.time < args.duration_s:
            run_control_step(runtime_state)
    else:
        if mujoco_viewer is None:
            print("MuJoCo viewer is unavailable in this environment. Re-run with --headless.", file=sys.stderr)
            return 3
        with mujoco_viewer.launch_passive(model, data) as viewer:
            hide_robot_during_viewer_delay()
            delay_start = time.perf_counter()
            while viewer.is_running() and (time.perf_counter() - delay_start) < VIEWER_IMPORT_DELAY_S:
                viewer.sync()
                time.sleep(0.01)
            if not viewer.is_running():
                return 0

            runtime_state = initialize_robot_runtime_state()
            print_runtime_config()
            configure_tracking_viewer_camera(mujoco, viewer, data, track_body_id=trunk_body_id)
            while viewer.is_running() and data.time < args.duration_s:
                run_control_step(runtime_state)
                draw_velocity_arrows(
                    mujoco, viewer, data,
                    trunk_body_id=trunk_body_id,
                    command_vel_xy=locomotion_cmd[:2],
                )
                viewer.sync()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
