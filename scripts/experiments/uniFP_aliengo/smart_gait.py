#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import sys

import torch

SCRIPTS_DIR = Path(__file__).resolve().parents[2]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir
from isaaclab.app import AppLauncher


LONG_RESAMPLING_S = 1.0e9
DEFAULT_EPISODE_LENGTH_S = 60.0
EVAL_VARIANT_TAG = "base_force_only"
EVAL_VARIANT_NOTE = "base-force-only"

FORCE_INTERP_LINEAR = "linear"
FORCE_INTERP_QUADRATIC = "quadratic"

FORCE_SCHEDULE_NONE = "none"
FORCE_SCHEDULE_DEMO = "demo"
FORCE_SCHEDULE_STOP_GO = "stop_go"
FORCE_SCHEDULE_HUMAN_GUIDE = "human_guide"

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
    mode: torch.Tensor
    command_zero: torch.Tensor
    force_xy_raw: torch.Tensor
    force_xy_cap: torch.Tensor
    force_baseline: torch.Tensor
    force_excess: torch.Tensor
    dir_consistency: torch.Tensor
    enter_score: torch.Tensor
    exit_score: torch.Tensor
    enter_timer: torch.Tensor
    exit_timer: torch.Tensor
    cooldown_timer: torch.Tensor
    standing_mask: torch.Tensor
    walking_mask: torch.Tensor


@dataclass
class ForceGuidanceState:
    enabled: bool
    active_mask: torch.Tensor
    effective_cmd_vel_b: torch.Tensor
    heading_error_rad: torch.Tensor
    estimated_force_heading_rad: torch.Tensor
    estimated_force_mag: torch.Tensor
    motion_sign: torch.Tensor


parser = argparse.ArgumentParser(
    description=(
        "Play a UniFP Aliengo checkpoint with a runtime standing/walking arbiter for the "
        f"{EVAL_VARIANT_NOTE} variant. The outer loop rewrites gait standing semantics based on "
        "command clip rules plus the base force estimator."
    )
)
policy_group = parser.add_mutually_exclusive_group(required=True)
policy_group.add_argument("--checkpoint", type=str, help="Path to an RSL-RL checkpoint.")
policy_group.add_argument(
    "--policy-jit",
    type=str,
    help=(
        "Path to an exported JIT policy. Note: the stock IsaacLab exporter is not compatible with this task's "
        "AsymmetricActorCritic adaptation policy."
    ),
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
parser.add_argument("--episode-length-s", type=float, default=DEFAULT_EPISODE_LENGTH_S, help="Episode horizon.")
parser.add_argument("--output-csv", type=str, default=None, help="Optional CSV output path.")
parser.add_argument(
    "--policy-history-length",
    type=int,
    default=None,
    help=(
        "Optional override for the flattened policy history length used by eval observation stacking and "
        "raw checkpoint reconstruction. Use this when the checkpoint was trained with a non-default history length."
    ),
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
    help=(
        "External base-force schedule. 'demo' keeps the legacy staged zero-command demo, "
        "'stop_go' runs a smooth start-stop-slack tuning schedule closer to guide-tension behavior, "
        "and 'human_guide' scripts a more human-like pull-turn-pull scenario with deterministic irregularity."
    ),
)
parser.add_argument(
    "--stop-go-heading-seq",
    type=str,
    default="forward,left,backward",
    help=(
        "Comma-separated nominal headings for each stop-go cycle. Tokens may be forward/left/right/backward/"
        "forward_left/forward_right/back_left/back_right, or explicit numeric degrees."
    ),
)
parser.add_argument("--stop-go-idle-s", type=float, default=3.0, help="Idle duration before each stop-go cycle.")
parser.add_argument(
    "--stop-go-pre-tension-s",
    type=float,
    default=1.0,
    help="Smooth pre-tension duration before the robot is asked to start moving.",
)
parser.add_argument(
    "--stop-go-start-pull-s",
    type=float,
    default=1.2,
    help="Smooth rise duration from pre-tension to the start-pull peak.",
)
parser.add_argument(
    "--stop-go-steady-s",
    type=float,
    default=3.5,
    help="Steady pull duration with smooth low-frequency force variation.",
)
parser.add_argument(
    "--stop-go-stop-cue-s",
    type=float,
    default=0.8,
    help="Smooth decrease duration for stop cue before slack recovery.",
)
parser.add_argument(
    "--stop-go-slack-s",
    type=float,
    default=1.6,
    help="Duration to smoothly release from stop cue to near-zero slack.",
)
parser.add_argument(
    "--stop-go-post-idle-s",
    type=float,
    default=1.4,
    help="Idle duration after each stop-go cycle finishes slack recovery.",
)
parser.add_argument("--stop-go-pre-tension-n", type=float, default=3.0, help="Nominal pre-tension force magnitude.")
parser.add_argument("--stop-go-start-pull-n", type=float, default=10.0, help="Nominal start-pull peak force magnitude.")
parser.add_argument("--stop-go-steady-force-n", type=float, default=15.0, help="Mean steady-pull force magnitude.")
parser.add_argument(
    "--stop-go-steady-force-osc-n",
    type=float,
    default=1.5,
    help="Sinusoidal force oscillation amplitude during steady pull.",
)
parser.add_argument(
    "--stop-go-stop-cue-n",
    type=float,
    default=1.5,
    help="Residual force magnitude kept at the end of the stop cue before slack release.",
)
parser.add_argument(
    "--stop-go-pre-tension-heading-drift-deg",
    type=float,
    default=2.0,
    help="Peak heading drift during the pre-tension phase.",
)
parser.add_argument(
    "--stop-go-start-pull-heading-drift-deg",
    type=float,
    default=4.0,
    help="Peak heading drift during the start-pull phase.",
)
parser.add_argument(
    "--stop-go-steady-heading-drift-deg",
    type=float,
    default=8.0,
    help="Peak heading drift during the steady-pull phase.",
)
parser.add_argument(
    "--stop-go-mag-tau-s",
    type=float,
    default=0.08,
    help="Magnitude smoothing time constant for the stop-go schedule.",
)
parser.add_argument(
    "--stop-go-angle-tau-s",
    type=float,
    default=0.12,
    help="Heading smoothing time constant for the stop-go schedule.",
)
parser.add_argument(
    "--stop-go-steady-osc-period-s",
    type=float,
    default=1.4,
    help="Oscillation period for steady-pull force and heading modulation.",
)
parser.add_argument(
    "--human-guide-straight-s",
    type=float,
    default=5.0,
    help="Duration of each straight pull segment in the human-guide scenario.",
)
parser.add_argument(
    "--human-guide-turn-s",
    type=float,
    default=2.5,
    help="Duration of each 90-degree turn segment in the human-guide scenario.",
)
parser.add_argument(
    "--human-guide-gap-s",
    type=float,
    default=3.0,
    help="Idle gap between the forward/right block and the backward/left block in the human-guide scenario.",
)
parser.add_argument(
    "--human-guide-base-force-n",
    type=float,
    default=15.0,
    help="Nominal guide-tension magnitude used by the human-guide scenario.",
)
parser.add_argument(
    "--human-guide-force-jitter-n",
    type=float,
    default=2.0,
    help="Peak low-frequency force jitter magnitude added on top of the human-guide base force.",
)
parser.add_argument(
    "--human-guide-heading-jitter-deg",
    type=float,
    default=12.0,
    help="Peak low-frequency heading sway around the nominal pull direction in the human-guide scenario.",
)
parser.add_argument(
    "--human-guide-dropout-max-s",
    type=float,
    default=0.35,
    help="Maximum duration of a short full-drop tension window inside each straight human-guide pull segment.",
)
parser.add_argument(
    "--walking-gait-frequency",
    type=float,
    default=3.0,
    help="Walking gait frequency used when mode is command_walking or force_walking.",
)
parser.add_argument(
    "--force-lp-tau-s",
    type=float,
    default=0.15,
    help="Legacy low-pass time constant from the previous switcher; retained for CLI compatibility.",
)
parser.add_argument("--baseline-tau-s", type=float, default=2.0, help="Adaptive standing-baseline update time constant.")
parser.add_argument(
    "--baseline-margin-n",
    type=float,
    default=0.15,
    help="Force margin above the adaptive baseline before excess evidence becomes positive.",
)
parser.add_argument(
    "--baseline-update-max-n",
    type=float,
    default=0.25,
    help="Only update the adaptive baseline when capped force stays within this band above the current baseline.",
)
parser.add_argument(
    "--evidence-cap-n",
    type=float,
    default=4.0,
    help="Internal cap applied to force evidence so large pushes do not create excessive release lag.",
)
parser.add_argument(
    "--enter-threshold-n",
    type=float,
    default=0.5,
    help="Force-excess threshold used to saturate enter evidence.",
)
parser.add_argument(
    "--exit-threshold-n",
    type=float,
    default=0.40,
    help="Force-excess threshold below which exit evidence saturates toward standing.",
)
parser.add_argument("--enter-tau-on-s", type=float, default=0.08, help="Rise time constant for enter evidence.")
parser.add_argument("--enter-tau-off-s", type=float, default=0.20, help="Decay time constant for enter evidence.")
parser.add_argument("--exit-tau-on-s", type=float, default=0.08, help="Rise time constant for exit evidence.")
parser.add_argument("--exit-tau-off-s", type=float, default=0.08, help="Decay time constant for exit evidence.")
parser.add_argument(
    "--enter-score-threshold",
    type=float,
    default=0.50,
    help="Enter score threshold that must be exceeded before enter hold starts accumulating.",
)
parser.add_argument(
    "--exit-score-threshold",
    type=float,
    default=0.60,
    help="Exit score threshold that must be exceeded before exit hold starts accumulating.",
)
parser.add_argument(
    "--dir-consistency-min-force-n",
    type=float,
    default=0.35,
    help="Minimum raw force needed before direction consistency starts tracking successive force directions.",
)
parser.add_argument(
    "--dir-consistency-tau-s",
    type=float,
    default=0.10,
    help="Time constant for smoothing the direction-consistency signal.",
)
parser.add_argument(
    "--dir-consistency-threshold",
    type=float,
    default=0.45,
    help="Direction-consistency threshold used to attenuate enter evidence under unstable directions.",
)
parser.add_argument("--switch-cooldown-s", type=float, default=0.20, help="Cooldown after each force-triggered gait transition.")
parser.add_argument("--enter-hold-s", type=float, default=0.06, help="Additional hold required to enter force_walking once enter score is high enough.")
parser.add_argument("--exit-hold-s", type=float, default=0.08, help="Additional hold required to return to standing once exit score is high enough.")
parser.add_argument(
    "--force-guided-turning",
    action="store_true",
    help=(
        "Enable an outer loop that converts the estimated base-force direction into an effective yaw-rate / "
        "forward-speed command during force_walking. Disabled by default."
    ),
)
parser.add_argument(
    "--force-guided-min-force-n",
    type=float,
    default=1.0,
    help="Minimum estimated XY force before force-guided turning becomes active.",
)
parser.add_argument(
    "--force-guided-yaw-kp",
    type=float,
    default=1.8,
    help="Proportional gain mapping force-direction heading error to yaw-rate command.",
)
parser.add_argument(
    "--force-guided-yaw-rate-max",
    type=float,
    default=0.8,
    help="Maximum absolute yaw-rate command produced by the force-guided turning outer loop.",
)
parser.add_argument(
    "--force-guided-turn-only-angle-deg",
    type=float,
    default=25.0,
    help="Above this heading error, the force-guided outer loop suppresses forward/backward speed to prioritize turning.",
)
parser.add_argument(
    "--force-guided-full-speed-angle-deg",
    type=float,
    default=8.0,
    help="Below this heading error, the force-guided outer loop allows full commanded forward/backward speed.",
)
parser.add_argument(
    "--force-guided-speed-scale",
    type=float,
    default=1.0,
    help="Scale factor applied to force-magnitude-derived forward/backward speed in the force-guided outer loop.",
)
parser.add_argument(
    "--force-guided-speed-max",
    type=float,
    default=0.35,
    help="Maximum absolute forward/backward speed command emitted by the force-guided outer loop.",
)
parser.add_argument("--log-interval", type=int, default=60, help="Console logging period in simulation steps.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

livestream_enabled = str(os.environ.get("LIVESTREAM", "0")) not in ("", "0")
if getattr(args_cli, "livestream", -1) not in (-1, 0):
    livestream_enabled = True
if livestream_enabled and not getattr(args_cli, "enable_cameras", False):
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import acc  # noqa: F401

import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_angle_axis, quat_from_euler_xyz

from acc.tasks.manager_based.uniFP_aliengo.mdp.utils import base_yaw_to_world, get_base_yaw_quat
from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg


def resolve_output_csv(path_str: str | None) -> Path:
    if path_str:
        output_path = Path(path_str).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = (
            Path("logs")
            / "experiments"
            / "unifp_smart_gait"
            / f"smart_gait_{EVAL_VARIANT_TAG}_{stamp}.csv"
        ).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env_cfg.commands.locomotion_commands.resampling_time_range = (LONG_RESAMPLING_S, LONG_RESAMPLING_S)
    env_cfg.commands.locomotion_commands.rel_standing_envs = 0.0
    env_cfg.commands.locomotion_commands.ranges.lin_vel_x = (float(args_cli.cmd_vx), float(args_cli.cmd_vx))
    env_cfg.commands.locomotion_commands.ranges.lin_vel_y = (float(args_cli.cmd_vy), float(args_cli.cmd_vy))
    env_cfg.commands.locomotion_commands.ranges.ang_vel_z = (float(args_cli.cmd_wz), float(args_cli.cmd_wz))
    env_cfg.commands.gait_phase.gait_frequency = float(args_cli.walking_gait_frequency)
    env_cfg.commands.locomotion_commands.debug_vis = False

    env_cfg.commands.gait_phase.gait_frequency = 2.0

    """env_cfg.scene.robot.actuators["hips"].stiffness = 180.0
    env_cfg.scene.robot.actuators["hips"].damping = 4.0
    env_cfg.scene.robot.actuators["thighs"].stiffness = 180.0
    env_cfg.scene.robot.actuators["thighs"].damping = 4.0
    env_cfg.scene.robot.actuators["calfs"].stiffness = 300.0
    env_cfg.scene.robot.actuators["calfs"].damping = 6.5"""

    target_position_term = getattr(env_cfg.commands, "target_position_commands", None)
    if target_position_term is not None:
        target_position_term.debug_vis = False

    if args_cli.policy_history_length is not None:
        if int(args_cli.policy_history_length) <= 0:
            raise ValueError(
                f"--policy-history-length must be positive, got {args_cli.policy_history_length}."
            )
        env_cfg.observations.policy.history_length = int(args_cli.policy_history_length)

    env_cfg.terminations.base_height = None
    return env_cfg


def load_raw_checkpoint_policy(checkpoint_path: str, env: ManagerBasedRLEnv):
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    from acc.algorithms.rsl_rl.runner import OnPolicyRunnerAdaptation
    from acc.tasks.manager_based.uniFP_aliengo.agents.rsl_rl_ppo_cfg import PPORunnerCfg

    agent_cfg = PPORunnerCfg()
    agent_cfg.device = env.device
    if args_cli.policy_history_length is not None:
        agent_cfg.policy.history_num_frames = int(args_cli.policy_history_length)

    wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunnerAdaptation(wrapped_env, agent_cfg.to_dict(), log_dir=None, device=env.device)
    runner.load(checkpoint_path)
    policy = runner.get_inference_policy(device=env.device)
    estimator_model = runner.alg.policy
    estimator_model.eval()
    return policy, wrapped_env, estimator_model


def normalize_xy(vectors_xy: torch.Tensor, eps: float = 1.0e-6) -> torch.Tensor:
    norms = torch.linalg.norm(vectors_xy, dim=-1, keepdim=True)
    fallback = torch.zeros_like(vectors_xy)
    fallback[:, 0] = 1.0
    return torch.where(norms > eps, vectors_xy / torch.clamp(norms, min=eps), fallback)


def base_forward_axis_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    yaw_quat = get_base_yaw_quat(env)
    local_forward = torch.zeros((env.num_envs, 3), device=env.device)
    local_forward[:, 0] = 1.0
    forward_w = base_yaw_to_world(local_forward, yaw_quat)
    return normalize_xy(forward_w[:, :2])


def direction_axis_from_forward(forward_axis_xy: torch.Tensor, direction_name: str) -> torch.Tensor:
    if direction_name == DIR_FORWARD:
        return forward_axis_xy
    if direction_name == DIR_LEFT:
        return torch.stack([-forward_axis_xy[:, 1], forward_axis_xy[:, 0]], dim=-1)
    if direction_name == DIR_BACKWARD:
        return -forward_axis_xy
    return torch.zeros_like(forward_axis_xy)


def extract_force_estimator_outputs(
    obs,
    estimator_model,
    *,
    base_force_scale: float,
):
    num_envs = obs.batch_size[0]
    device = obs.device
    nan_force = torch.full((num_envs, 3), float("nan"), device=device)
    nan_err = torch.full((num_envs,), float("nan"), device=device)

    if estimator_model is None or not hasattr(estimator_model, "predict") or "pred" not in obs:
        return {
            "base_force_est_local": nan_force.clone(),
            "base_force_tgt_local": nan_force.clone(),
            "base_force_err_l2": nan_err.clone(),
        }

    pred_target_scaled = obs["pred"]
    pred_est_scaled = estimator_model.predict(obs)

    base_force_est_local = pred_est_scaled[:, 3:6] / base_force_scale
    base_force_tgt_local = pred_target_scaled[:, 3:6] / base_force_scale

    return {
        "base_force_est_local": base_force_est_local,
        "base_force_tgt_local": base_force_tgt_local,
        "base_force_err_l2": torch.linalg.norm(base_force_est_local - base_force_tgt_local, dim=-1),
    }


def build_demo_force_segments(episode_length_s: float) -> tuple[ForceSegment, ...]:
    tail_end = max(float(episode_length_s), 60.0)
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


def rotate_xy(vectors_xy: torch.Tensor, angles_rad: torch.Tensor) -> torch.Tensor:
    cos_a = torch.cos(angles_rad)
    sin_a = torch.sin(angles_rad)
    x = vectors_xy[:, 0]
    y = vectors_xy[:, 1]
    return torch.stack([cos_a * x - sin_a * y, sin_a * x + cos_a * y], dim=-1)


def smoothstep01(value: float) -> float:
    clamped = max(0.0, min(float(value), 1.0))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def lerp_scalar(start: float, end: float, ratio: float) -> float:
    return float(start + (end - start) * float(ratio))


def wrap_to_pi(angles_rad: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angles_rad), torch.cos(angles_rad))


def angle_progress(abs_angle_rad: torch.Tensor, *, full_speed_angle_rad: float, turn_only_angle_rad: float) -> torch.Tensor:
    denom = max(float(turn_only_angle_rad) - float(full_speed_angle_rad), 1.0e-6)
    progress = (float(turn_only_angle_rad) - abs_angle_rad) / denom
    return torch.clamp(progress, min=0.0, max=1.0)


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


class ScriptedForceScheduler:
    """Deterministic base-force schedule used for the default zero-command demo."""

    def __init__(self, num_envs: int, device: torch.device | str, segments: tuple[ForceSegment, ...]):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.segments = segments
        self.current_segment_name: str | None = None
        self.current_axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

    def segment_for_time(self, time_s: float) -> ForceSegment:
        if time_s >= self.segments[-1].start_s:
            return self.segments[-1]
        for segment in self.segments:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return self.segments[-1]

    def update(self, env: ManagerBasedRLEnv, time_s: float) -> dict[str, torch.Tensor | float | str | bool]:
        segment = self.segment_for_time(time_s)
        segment_changed = segment.segment_name != self.current_segment_name

        if segment_changed and segment.capture_axis and segment.is_active:
            forward_axis_xy = base_forward_axis_xy(env)
            self.current_axis_xy = direction_axis_from_forward(forward_axis_xy, segment.direction_name)

        force_target_n = torch.full((self.num_envs,), segment.force_target_at(time_s), device=self.device)
        force_w = torch.zeros((self.num_envs, 3), device=self.device)
        axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

        if segment.is_active:
            axis_xy = self.current_axis_xy.clone()
            force_w[:, :2] = axis_xy * force_target_n.unsqueeze(-1)

        self.current_segment_name = segment.segment_name
        return {
            "segment_name": segment.segment_name,
            "direction_name": segment.direction_name,
            "phase_name": segment.phase_name,
            "force_target_n": force_target_n,
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": segment.is_active,
        }


class StopGoTuningForceScheduler:
    """Smooth stop-go guide-tension schedule for gait-switch tuning.

    This schedule focuses on tuning start / stop / slack-recovery transitions instead of
    stress-testing with abrupt piecewise jumps. Each cycle follows:
    idle -> pre_tension -> start_pull -> steady_pull -> stop_cue -> slack_recovery -> post_idle
    """

    def __init__(self, num_envs: int, device: torch.device | str, step_dt: float, headings_deg: tuple[float, ...]):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.step_dt = float(step_dt)
        self.headings_deg = tuple(float(v) for v in headings_deg)
        if len(self.headings_deg) == 0:
            raise ValueError("stop-go schedule requires at least one heading.")

        self.idle_s = max(float(args_cli.stop_go_idle_s), 0.0)
        self.pre_tension_s = max(float(args_cli.stop_go_pre_tension_s), 0.0)
        self.start_pull_s = max(float(args_cli.stop_go_start_pull_s), 0.0)
        self.steady_s = max(float(args_cli.stop_go_steady_s), 0.0)
        self.stop_cue_s = max(float(args_cli.stop_go_stop_cue_s), 0.0)
        self.slack_s = max(float(args_cli.stop_go_slack_s), 0.0)
        self.post_idle_s = max(float(args_cli.stop_go_post_idle_s), 0.0)
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

        self.pre_tension_n = max(float(args_cli.stop_go_pre_tension_n), 0.0)
        self.start_pull_n = max(float(args_cli.stop_go_start_pull_n), self.pre_tension_n)
        self.steady_force_n = max(float(args_cli.stop_go_steady_force_n), 0.0)
        self.steady_force_osc_n = max(float(args_cli.stop_go_steady_force_osc_n), 0.0)
        self.stop_cue_n = max(float(args_cli.stop_go_stop_cue_n), 0.0)

        self.pre_tension_heading_drift_rad = math.radians(float(args_cli.stop_go_pre_tension_heading_drift_deg))
        self.start_pull_heading_drift_rad = math.radians(float(args_cli.stop_go_start_pull_heading_drift_deg))
        self.steady_heading_drift_rad = math.radians(float(args_cli.stop_go_steady_heading_drift_deg))

        self.steady_osc_period_s = max(float(args_cli.stop_go_steady_osc_period_s), max(self.step_dt, 1.0e-3))
        self.mag_alpha = self._time_constant_to_alpha(float(args_cli.stop_go_mag_tau_s))
        self.angle_alpha = self._time_constant_to_alpha(float(args_cli.stop_go_angle_tau_s))

        self.mag_state = torch.zeros((self.num_envs,), device=self.device)
        self.angle_state = torch.zeros((self.num_envs,), device=self.device)
        self.base_heading_rad = torch.zeros((self.num_envs,), device=self.device)
        self.reference_forward_xy = torch.zeros((self.num_envs, 2), device=self.device)
        self.reference_forward_xy[:, 0] = 1.0
        self.captured_cycle_idx = -1

    def update(self, env: ManagerBasedRLEnv, time_s: float) -> dict[str, torch.Tensor | float | str | bool]:
        cycle_idx, heading_deg, phase_name, phase_elapsed_s, phase_duration_s = self._phase_info(time_s)
        phase_active = phase_name not in {STOP_GO_PHASE_IDLE, STOP_GO_PHASE_POST_IDLE}

        if phase_active and cycle_idx != self.captured_cycle_idx:
            self.reference_forward_xy = base_forward_axis_xy(env)
            self.base_heading_rad.fill_(math.radians(heading_deg))
            self.captured_cycle_idx = cycle_idx

        force_target_n_scalar = self._force_target_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
        drift_target_rad_scalar = self._heading_drift_for_phase(phase_name, phase_elapsed_s, phase_duration_s)

        force_target_n = torch.full((self.num_envs,), force_target_n_scalar, device=self.device)
        drift_target_rad = torch.full((self.num_envs,), drift_target_rad_scalar, device=self.device)
        self.mag_state = self.mag_state + self.mag_alpha * (force_target_n - self.mag_state)
        self.angle_state = self.angle_state + self.angle_alpha * (drift_target_rad - self.angle_state)

        axis_xy = rotate_xy(self.reference_forward_xy, self.base_heading_rad + self.angle_state)
        force_w = torch.zeros((self.num_envs, 3), device=self.device)
        force_w[:, :2] = axis_xy * self.mag_state.unsqueeze(-1)

        is_active = phase_active or bool(torch.any(self.mag_state > 1.0e-4).item())
        if not is_active:
            axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)

        return {
            "segment_name": f"cycle_{cycle_idx:02d}",
            "direction_name": format_heading_label(heading_deg),
            "phase_name": phase_name,
            "force_target_n": self.mag_state.clone(),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": is_active,
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
        if phase_name == STOP_GO_PHASE_SLACK_RECOVERY:
            return 0.0
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
        return 1.0 - math.exp(-self.step_dt / tau)


# Follow-up schedule ideas, intentionally not implemented in this patch:
# - threshold_hesitation
# - rapid_regrasp
# - continuous_arc
class HumanGuideForceScheduler:
    """Script a human-like guide-tension scenario with deterministic irregularity."""

    _ACTIVE_PHASES = {
        HUMAN_GUIDE_PHASE_FORWARD_PULL,
        HUMAN_GUIDE_PHASE_RIGHT_TURN,
        HUMAN_GUIDE_PHASE_RIGHT_PULL,
        HUMAN_GUIDE_PHASE_BACKWARD_PULL,
        HUMAN_GUIDE_PHASE_LEFT_TURN,
        HUMAN_GUIDE_PHASE_LEFT_PULL,
    }
    _PHASE_TO_BLOCK_IDX = {
        HUMAN_GUIDE_PHASE_FORWARD_PULL: 0,
        HUMAN_GUIDE_PHASE_RIGHT_TURN: 0,
        HUMAN_GUIDE_PHASE_RIGHT_PULL: 0,
        HUMAN_GUIDE_PHASE_BACKWARD_PULL: 1,
        HUMAN_GUIDE_PHASE_LEFT_TURN: 1,
        HUMAN_GUIDE_PHASE_LEFT_PULL: 1,
    }
    _BLOCK_SEGMENT_NAMES = {
        0: "human_guide_block_00",
        1: "human_guide_block_01",
    }

    def __init__(self, num_envs: int, device: torch.device | str, step_dt: float):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.step_dt = float(step_dt)

        self.straight_s = max(float(args_cli.human_guide_straight_s), 0.0)
        self.turn_s = max(float(args_cli.human_guide_turn_s), 0.0)
        self.gap_s = max(float(args_cli.human_guide_gap_s), 0.0)
        if self.straight_s <= 0.0:
            raise ValueError("human-guide schedule requires --human-guide-straight-s > 0.")
        if self.turn_s <= 0.0:
            raise ValueError("human-guide schedule requires --human-guide-turn-s > 0.")

        self.base_force_n = max(float(args_cli.human_guide_base_force_n), 0.0)
        self.force_jitter_n = max(float(args_cli.human_guide_force_jitter_n), 0.0)
        self.turn_force_jitter_n = 0.35 * self.force_jitter_n
        self.heading_jitter_rad = math.radians(max(float(args_cli.human_guide_heading_jitter_deg), 0.0))
        self.turn_heading_jitter_rad = 0.35 * self.heading_jitter_rad
        self.dropout_max_s = max(float(args_cli.human_guide_dropout_max_s), 0.0)

        self.mag_alpha = self._time_constant_to_alpha(float(args_cli.stop_go_mag_tau_s))
        self.angle_alpha = self._time_constant_to_alpha(float(args_cli.stop_go_angle_tau_s))

        self.mag_state = torch.zeros((self.num_envs,), device=self.device)
        self.heading_state = torch.zeros((self.num_envs,), device=self.device)
        self.reference_forward_xy = torch.zeros((self.num_envs, 2), device=self.device)
        self.reference_forward_xy[:, 0] = 1.0
        self.captured_block_idx = -1

        self.phase_durations = (
            (HUMAN_GUIDE_PHASE_FORWARD_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_RIGHT_TURN, self.turn_s),
            (HUMAN_GUIDE_PHASE_RIGHT_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_GAP, self.gap_s),
            (HUMAN_GUIDE_PHASE_BACKWARD_PULL, self.straight_s),
            (HUMAN_GUIDE_PHASE_LEFT_TURN, self.turn_s),
            (HUMAN_GUIDE_PHASE_LEFT_PULL, self.straight_s),
        )
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

    def update(self, env: ManagerBasedRLEnv, time_s: float) -> dict[str, torch.Tensor | float | str | bool]:
        phase_name, phase_elapsed_s, phase_duration_s = self._phase_info(time_s)
        block_idx = self._PHASE_TO_BLOCK_IDX.get(phase_name, None)

        if block_idx is not None and block_idx != self.captured_block_idx:
            self.reference_forward_xy = base_forward_axis_xy(env)
            heading_start_deg = self._nominal_heading_deg_for_phase(phase_name, phase_elapsed_s, phase_duration_s)
            self.heading_state.fill_(math.radians(heading_start_deg))
            self.captured_block_idx = block_idx

        force_schedule_target_n_scalar, heading_target_rad_scalar = self._targets_for_phase(
            phase_name,
            phase_elapsed_s,
            phase_duration_s,
        )

        force_schedule_target_n = torch.full(
            (self.num_envs,),
            force_schedule_target_n_scalar,
            device=self.device,
        )
        self.mag_state = self.mag_state + self.mag_alpha * (force_schedule_target_n - self.mag_state)

        if heading_target_rad_scalar is not None:
            heading_target_rad = torch.full((self.num_envs,), heading_target_rad_scalar, device=self.device)
            self.heading_state = self.heading_state + self.angle_alpha * (heading_target_rad - self.heading_state)

        phase_active = phase_name in self._ACTIVE_PHASES
        is_active = phase_active or bool(torch.any(self.mag_state > 1.0e-4).item())

        force_w = torch.zeros((self.num_envs, 3), device=self.device)
        axis_xy = torch.full((self.num_envs, 2), float("nan"), device=self.device)
        if is_active:
            axis_xy = rotate_xy(self.reference_forward_xy, self.heading_state)
            force_w[:, :2] = axis_xy * self.mag_state.unsqueeze(-1)

        direction_name = DIR_NONE
        if phase_active:
            direction_name = format_heading_label(math.degrees(float(self.heading_state[0].item())))

        return {
            "segment_name": self._segment_name_for_phase(phase_name),
            "direction_name": direction_name,
            "phase_name": phase_name,
            "force_schedule_target_n": force_schedule_target_n.clone(),
            "force_target_n": self.mag_state.clone(),
            "force_w": force_w,
            "axis_xy": axis_xy,
            "is_active": is_active,
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
        if phase_name == HUMAN_GUIDE_PHASE_GAP:
            return "human_guide_gap"
        return "human_guide_tail"

    def _targets_for_phase(self, phase_name: str, phase_elapsed_s: float, phase_duration_s: float) -> tuple[float, float | None]:
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

        return 0.0, None

    def _straight_force_target(self, profile: HumanGuideLegProfile, elapsed_s: float) -> float:
        force_target_n = self.base_force_n + self.force_jitter_n * self._profile_wave(
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

    def _build_leg_profile(self, seed_offset: int) -> HumanGuideLegProfile:
        generator = torch.Generator(device="cpu")
        generator.manual_seed(int(args_cli.seed) + int(seed_offset))

        boundary_margin_s = min(max(0.65, 0.16 * self.straight_s), 0.30 * self.straight_s)
        max_dropout_s = min(self.dropout_max_s, max(self.straight_s - (2.0 * boundary_margin_s), 0.0))
        if max_dropout_s <= 1.0e-6:
            dropout_start_s = 0.0
            dropout_duration_s = 0.0
        else:
            min_dropout_s = min(0.12, max_dropout_s)
            dropout_duration_s = self._sample_uniform(generator, min_dropout_s, max_dropout_s)
            start_hi_s = self.straight_s - boundary_margin_s - dropout_duration_s
            if start_hi_s <= boundary_margin_s:
                dropout_start_s = 0.0
                dropout_duration_s = 0.0
            else:
                dropout_start_s = self._sample_uniform(generator, boundary_margin_s, start_hi_s)

        return HumanGuideLegProfile(
            force_freq_hz=(
                self._sample_uniform(generator, 0.15, 0.32),
                self._sample_uniform(generator, 0.34, 0.58),
            ),
            force_phase_rad=(
                self._sample_uniform(generator, 0.0, 2.0 * math.pi),
                self._sample_uniform(generator, 0.0, 2.0 * math.pi),
            ),
            force_weight=self._sample_uniform(generator, 0.35, 0.65),
            heading_freq_hz=(
                self._sample_uniform(generator, 0.18, 0.38),
                self._sample_uniform(generator, 0.42, 0.72),
            ),
            heading_phase_rad=(
                self._sample_uniform(generator, 0.0, 2.0 * math.pi),
                self._sample_uniform(generator, 0.0, 2.0 * math.pi),
            ),
            heading_weight=self._sample_uniform(generator, 0.35, 0.65),
            dropout_start_s=dropout_start_s,
            dropout_duration_s=dropout_duration_s,
        )

    def _build_turn_profile(self, seed_offset: int) -> HumanGuideTurnProfile:
        generator = torch.Generator(device="cpu")
        generator.manual_seed(int(args_cli.seed) + int(seed_offset))
        return HumanGuideTurnProfile(
            force_freq_hz=self._sample_uniform(generator, 0.20, 0.45),
            force_phase_rad=self._sample_uniform(generator, 0.0, 2.0 * math.pi),
            heading_freq_hz=self._sample_uniform(generator, 0.25, 0.55),
            heading_phase_rad=self._sample_uniform(generator, 0.0, 2.0 * math.pi),
        )

    @staticmethod
    def _sample_uniform(generator: torch.Generator, lower: float, upper: float) -> float:
        if upper <= lower:
            return float(lower)
        sample = torch.rand((), generator=generator)
        return lerp_scalar(lower, upper, float(sample.item()))

    def _time_constant_to_alpha(self, tau_seconds: float) -> float:
        tau = max(float(tau_seconds), 0.0)
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-self.step_dt / tau)


class ForceGuidedTurningController:
    """Optional outer loop that maps estimated force direction to yaw-rate / fore-aft commands."""

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        base_force_kd: float,
        max_speed: float,
        max_yaw_rate: float,
    ):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.enabled = bool(args_cli.force_guided_turning)
        self.base_force_kd = max(float(base_force_kd), 1.0e-6)
        self.min_force_n = max(float(args_cli.force_guided_min_force_n), 0.0)
        self.yaw_kp = max(float(args_cli.force_guided_yaw_kp), 0.0)
        self.yaw_rate_max = max(min(float(args_cli.force_guided_yaw_rate_max), float(max_yaw_rate)), 0.0)
        self.turn_only_angle_rad = math.radians(max(float(args_cli.force_guided_turn_only_angle_deg), 0.0))
        self.full_speed_angle_rad = math.radians(
            max(min(float(args_cli.force_guided_full_speed_angle_deg), float(args_cli.force_guided_turn_only_angle_deg)), 0.0)
        )
        self.speed_scale = max(float(args_cli.force_guided_speed_scale), 0.0)
        self.speed_max = max(min(float(args_cli.force_guided_speed_max), float(max_speed)), 0.0)

    def compute(
        self,
        nominal_cmd_vel_b: torch.Tensor,
        base_force_est_local: torch.Tensor,
        mode_state: ForceModeSwitchState,
    ) -> ForceGuidanceState:
        effective_cmd_vel_b = nominal_cmd_vel_b.clone()
        active_mask = torch.zeros((self.num_envs,), device=self.device, dtype=torch.bool)
        heading_error_rad = torch.zeros((self.num_envs,), device=self.device)
        estimated_force_heading_rad = torch.zeros((self.num_envs,), device=self.device)
        estimated_force_mag = torch.zeros((self.num_envs,), device=self.device)
        motion_sign = torch.ones((self.num_envs,), device=self.device)

        if not self.enabled:
            return ForceGuidanceState(
                enabled=False,
                active_mask=active_mask,
                effective_cmd_vel_b=effective_cmd_vel_b,
                heading_error_rad=heading_error_rad,
                estimated_force_heading_rad=estimated_force_heading_rad,
                estimated_force_mag=estimated_force_mag,
                motion_sign=motion_sign,
            )

        force_xy = base_force_est_local[:, :2]
        estimated_force_mag = torch.linalg.norm(force_xy, dim=-1)
        estimated_force_mag = torch.where(
            torch.isfinite(estimated_force_mag),
            estimated_force_mag,
            torch.zeros_like(estimated_force_mag),
        )
        estimated_force_heading_rad = torch.atan2(force_xy[:, 1], force_xy[:, 0])

        reverse_mask = force_xy[:, 0] < 0.0
        motion_sign = torch.where(reverse_mask, -torch.ones_like(estimated_force_mag), torch.ones_like(estimated_force_mag))

        alignment_xy = force_xy.clone()
        alignment_xy[reverse_mask] = -alignment_xy[reverse_mask]
        heading_error_rad = torch.atan2(alignment_xy[:, 1], alignment_xy[:, 0])
        heading_error_rad = wrap_to_pi(heading_error_rad)

        force_mode_mask = mode_state.command_zero & (mode_state.mode == MODE_FORCE_WALKING)
        active_mask = force_mode_mask & (estimated_force_mag >= self.min_force_n)

        if active_mask.any():
            speed_cmd = (estimated_force_mag / self.base_force_kd) * self.speed_scale
            speed_cmd = torch.clamp(speed_cmd, min=0.0, max=self.speed_max)

            abs_heading_error_rad = torch.abs(heading_error_rad)
            speed_progress = angle_progress(
                abs_heading_error_rad,
                full_speed_angle_rad=self.full_speed_angle_rad,
                turn_only_angle_rad=self.turn_only_angle_rad,
            )
            yaw_rate_cmd = torch.clamp(self.yaw_kp * heading_error_rad, min=-self.yaw_rate_max, max=self.yaw_rate_max)

            effective_cmd_vel_b[active_mask, 0] = motion_sign[active_mask] * speed_cmd[active_mask] * speed_progress[active_mask]
            effective_cmd_vel_b[active_mask, 1] = 0.0
            effective_cmd_vel_b[active_mask, 2] = yaw_rate_cmd[active_mask]

        return ForceGuidanceState(
            enabled=True,
            active_mask=active_mask,
            effective_cmd_vel_b=effective_cmd_vel_b,
            heading_error_rad=heading_error_rad,
            estimated_force_heading_rad=estimated_force_heading_rad,
            estimated_force_mag=estimated_force_mag,
            motion_sign=motion_sign,
        )


class SmartGaitMarkers:
    """Visualization markers for command velocity, current velocity, and external base force."""

    def __init__(self, env: ManagerBasedRLEnv):
        self.env = env
        self.device = env.device
        self.cmd_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/cmd_velocity",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(0.6, 0.6, 0.6),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                    ),
                },
            )
        )
        self.curr_vel_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/current_velocity",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(0.6, 0.6, 0.6),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.9, 0.9)),
                    ),
                },
            )
        )
        self.base_force_markers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/SmartGait/base_force_ext",
                markers={
                    "arrow": sim_utils.UsdFileCfg(
                        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                        scale=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.4, 1.0)),
                    ),
                },
            )
        )
        self.cmd_vel_markers.set_visibility(True)
        self.curr_vel_markers.set_visibility(True)
        self.base_force_markers.set_visibility(True)
        self._cmd_base_scale = torch.tensor((0.6, 0.6, 0.6), device=self.device, dtype=torch.float)
        self._curr_base_scale = torch.tensor((0.6, 0.6, 0.6), device=self.device, dtype=torch.float)

    def update(self):
        robot = self.env.scene["robot"]
        base_pos_w = robot.data.root_pos_w.clone()
        vel_marker_pos_w = base_pos_w.clone()
        vel_marker_pos_w[:, 2] += 0.5
        force_marker_pos_w = base_pos_w.clone()
        force_marker_pos_w[:, 2] += 0.2

        base_yaw_quat = get_base_yaw_quat(self.env)
        cmd_vel_b = self.env.command_manager.get_command("locomotion_commands")
        cmd_vel_w = torch.zeros((self.env.num_envs, 3), device=self.device)
        cmd_vel_w[:, :2] = base_yaw_to_world(
            torch.nn.functional.pad(cmd_vel_b[:, :2], (0, 1)),
            base_yaw_quat,
        )[:, :2]

        base_lin_vel_w = robot.data.root_lin_vel_w
        base_force_ext_w = self.env.command_manager.get_command("base_force_ext")

        self.cmd_vel_markers.visualize(
            vel_marker_pos_w,
            self._vectors_to_yaw_quat(cmd_vel_w),
            scales=self._apply_speed_scale(cmd_vel_w, self._cmd_base_scale),
        )
        self.curr_vel_markers.visualize(
            vel_marker_pos_w,
            self._vectors_to_yaw_quat(base_lin_vel_w),
            scales=self._apply_speed_scale(base_lin_vel_w, self._curr_base_scale),
        )
        self.base_force_markers.visualize(
            force_marker_pos_w,
            self._forces_to_quat(base_force_ext_w),
            scales=self._forces_to_scales(base_force_ext_w),
        )

    def _vectors_to_yaw_quat(self, vectors_w: torch.Tensor) -> torch.Tensor:
        yaw = torch.atan2(vectors_w[:, 1], vectors_w[:, 0])
        zeros = torch.zeros_like(yaw)
        return quat_from_euler_xyz(zeros, zeros, yaw)

    def _apply_speed_scale(self, vel_w: torch.Tensor, base_scale: torch.Tensor) -> torch.Tensor:
        speed = torch.norm(vel_w[:, :2], dim=-1)
        speed_ref = 0.3
        scale_factor = torch.clamp(speed / max(speed_ref, 1.0e-3), min=1.2, max=3.0)
        scales = base_scale.unsqueeze(0).repeat(vel_w.shape[0], 1)
        scales[:, 0] = scales[:, 0] * scale_factor
        return scales

    def _forces_to_quat(self, forces_w: torch.Tensor) -> torch.Tensor:
        eps = 1.0e-6
        mag = torch.linalg.norm(forces_w, dim=1)
        direction = forces_w / (mag.unsqueeze(-1) + eps)
        dot = torch.clamp(direction[:, 0], -1.0, 1.0)
        angle = torch.acos(dot)
        axis = torch.stack([torch.zeros_like(dot), -direction[:, 2], direction[:, 1]], dim=1)
        axis_norm = torch.linalg.norm(axis, dim=1, keepdim=True)
        default_axis = torch.zeros_like(axis)
        default_axis[:, 2] = 1.0
        axis = torch.where(axis_norm > eps, axis, default_axis)
        axis = axis / torch.clamp(torch.linalg.norm(axis, dim=1, keepdim=True), min=eps)
        quat = quat_from_angle_axis(angle, axis)
        identity = torch.zeros((forces_w.shape[0], 4), device=forces_w.device)
        identity[:, 0] = 1.0
        return torch.where(mag.unsqueeze(-1) > eps, quat, identity)

    def _forces_to_scales(self, forces_w: torch.Tensor) -> torch.Tensor:
        mag = torch.linalg.norm(forces_w, dim=1)
        length = torch.clamp(mag * 0.02, max=0.6)
        thickness = torch.full_like(length, 0.05)
        return torch.stack([length, thickness, thickness], dim=1)


class ForceModeSwitcher:
    """Runtime outer-loop arbiter for standing vs. walking gait semantics.

    The zero-command branch uses only force-estimator outputs. It maintains an adaptive
    standing baseline, caps the force evidence used for switching, tracks the short-horizon
    direction consistency of the estimated force vector, and runs separate enter/exit
    evidence accumulators with independent dynamics.
    """

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        lin_vel_x_clip: float,
        lin_vel_y_clip: float,
        ang_vel_yaw_clip: float,
        baseline_tau_s: float,
        baseline_margin_n: float,
        baseline_update_max_n: float,
        evidence_cap_n: float,
        enter_th_n: float,
        exit_th_n: float,
        enter_tau_on_s: float,
        enter_tau_off_s: float,
        exit_tau_on_s: float,
        exit_tau_off_s: float,
        enter_score_threshold: float,
        exit_score_threshold: float,
        dir_consistency_min_force_n: float,
        dir_consistency_tau_s: float,
        dir_consistency_threshold: float,
        switch_cooldown_s: float,
        enter_hold_s: float,
        exit_hold_s: float,
    ):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.lin_vel_x_clip = float(lin_vel_x_clip)
        self.lin_vel_y_clip = float(lin_vel_y_clip)
        self.ang_vel_yaw_clip = float(ang_vel_yaw_clip)
        self.baseline_tau_s = float(baseline_tau_s)
        self.baseline_margin_n = float(baseline_margin_n)
        self.baseline_update_max_n = float(baseline_update_max_n)
        self.evidence_cap_n = float(evidence_cap_n)
        self.enter_th_n = float(enter_th_n)
        self.exit_th_n = float(exit_th_n)
        self.enter_tau_on_s = float(enter_tau_on_s)
        self.enter_tau_off_s = float(enter_tau_off_s)
        self.exit_tau_on_s = float(exit_tau_on_s)
        self.exit_tau_off_s = float(exit_tau_off_s)
        self.enter_score_threshold = float(enter_score_threshold)
        self.exit_score_threshold = float(exit_score_threshold)
        self.dir_consistency_min_force_n = float(dir_consistency_min_force_n)
        self.dir_consistency_tau_s = float(dir_consistency_tau_s)
        self.dir_consistency_threshold = float(dir_consistency_threshold)
        self.switch_cooldown_s = float(switch_cooldown_s)
        self.enter_hold_s = float(enter_hold_s)
        self.exit_hold_s = float(exit_hold_s)

        self.mode = torch.full((self.num_envs,), MODE_STANDING, device=self.device, dtype=torch.long)
        self.force_state = torch.full((self.num_envs,), MODE_STANDING, device=self.device, dtype=torch.long)
        self.force_baseline = torch.zeros((self.num_envs,), device=self.device)
        self.dir_consistency = torch.ones((self.num_envs,), device=self.device)
        self.enter_score = torch.zeros((self.num_envs,), device=self.device)
        self.exit_score = torch.zeros((self.num_envs,), device=self.device)
        self.enter_timer = torch.zeros((self.num_envs,), device=self.device)
        self.exit_timer = torch.zeros((self.num_envs,), device=self.device)
        self.cooldown_timer = torch.zeros((self.num_envs,), device=self.device)
        self.prev_force_dir_xy = torch.zeros((self.num_envs, 2), device=self.device)
        self.prev_force_dir_xy[:, 0] = 1.0
        self.prev_force_dir_valid = torch.zeros((self.num_envs,), device=self.device, dtype=torch.bool)

    def classify_command_zero(self, cmd_vel_b: torch.Tensor) -> torch.Tensor:
        is_small_x = torch.abs(cmd_vel_b[:, 0]) < self.lin_vel_x_clip
        is_small_y = torch.abs(cmd_vel_b[:, 1]) < self.lin_vel_y_clip
        is_small_yaw = torch.abs(cmd_vel_b[:, 2]) < self.ang_vel_yaw_clip
        return is_small_x & is_small_y & is_small_yaw

    @staticmethod
    def _alpha(dt: float, tau_s: float) -> float:
        return float(dt) / max(float(tau_s) + float(dt), 1.0e-8)

    @staticmethod
    def _update_score(
        score: torch.Tensor,
        target: torch.Tensor,
        *,
        dt: float,
        tau_on_s: float,
        tau_off_s: float,
    ) -> torch.Tensor:
        alpha_on = ForceModeSwitcher._alpha(dt, tau_on_s)
        alpha_off = ForceModeSwitcher._alpha(dt, tau_off_s)
        alpha_on_tensor = torch.full_like(score, alpha_on)
        alpha_off_tensor = torch.full_like(score, alpha_off)
        alpha = torch.where(target >= score, alpha_on_tensor, alpha_off_tensor)
        updated = score + alpha * (target - score)
        return torch.clamp(updated, min=0.0, max=1.0)

    def update(self, cmd_vel_b: torch.Tensor, base_force_est_local: torch.Tensor, *, dt: float) -> ForceModeSwitchState:
        dt = float(dt)
        command_zero = self.classify_command_zero(cmd_vel_b)
        self.cooldown_timer = torch.clamp(self.cooldown_timer - dt, min=0.0)

        force_xy_raw = torch.linalg.norm(base_force_est_local[:, :2], dim=-1)
        force_xy_raw = torch.where(torch.isfinite(force_xy_raw), force_xy_raw, torch.zeros_like(force_xy_raw))
        force_xy_cap = torch.clamp(force_xy_raw, min=0.0, max=self.evidence_cap_n)

        force_dir_xy = normalize_xy(base_force_est_local[:, :2])
        valid_force_dir = force_xy_raw >= self.dir_consistency_min_force_n
        dir_alignment = torch.ones((self.num_envs,), device=self.device)
        both_valid = valid_force_dir & self.prev_force_dir_valid
        if both_valid.any():
            dir_alignment[both_valid] = torch.clamp(
                torch.sum(force_dir_xy[both_valid] * self.prev_force_dir_xy[both_valid], dim=-1),
                min=0.0,
                max=1.0,
            )
        dir_target = torch.ones_like(force_xy_raw)
        dir_target[valid_force_dir] = dir_alignment[valid_force_dir]
        dir_alpha = self._alpha(dt, self.dir_consistency_tau_s)
        self.dir_consistency = self.dir_consistency + dir_alpha * (dir_target - self.dir_consistency)
        self.prev_force_dir_xy[valid_force_dir] = force_dir_xy[valid_force_dir]
        self.prev_force_dir_valid = valid_force_dir.clone()

        command_walking_mask = ~command_zero
        if command_walking_mask.any():
            self.mode[command_walking_mask] = MODE_COMMAND_WALKING
            self.force_state[command_walking_mask] = MODE_STANDING
            self.dir_consistency[command_walking_mask] = 1.0
            self.enter_score[command_walking_mask] = 0.0
            self.exit_score[command_walking_mask] = 0.0
            self.enter_timer[command_walking_mask] = 0.0
            self.exit_timer[command_walking_mask] = 0.0
            self.cooldown_timer[command_walking_mask] = 0.0
            self.prev_force_dir_valid[command_walking_mask] = False

        zero_ids = torch.nonzero(command_zero, as_tuple=False).squeeze(-1)
        if zero_ids.numel() > 0:
            baseline_update_mask = (
                command_zero
                & (self.force_state == MODE_STANDING)
                & (self.cooldown_timer <= 0.0)
                & (force_xy_cap <= (self.force_baseline + self.baseline_update_max_n))
            )
            if baseline_update_mask.any():
                baseline_alpha = self._alpha(dt, self.baseline_tau_s)
                self.force_baseline[baseline_update_mask] = self.force_baseline[baseline_update_mask] + baseline_alpha * (
                    force_xy_cap[baseline_update_mask] - self.force_baseline[baseline_update_mask]
                )

            force_excess = torch.relu(force_xy_cap - self.force_baseline - self.baseline_margin_n)
            enter_force_target = torch.clamp(force_excess / max(self.enter_th_n, 1.0e-6), min=0.0, max=1.0)
            dir_term = torch.clamp(
                (self.dir_consistency - self.dir_consistency_threshold)
                / max(1.0 - self.dir_consistency_threshold, 1.0e-6),
                min=0.0,
                max=1.0,
            )
            enter_target = enter_force_target * dir_term
            exit_target = 1.0 - torch.clamp(force_excess / max(self.exit_th_n, 1.0e-6), min=0.0, max=1.0)

            self.enter_score[zero_ids] = self._update_score(
                self.enter_score[zero_ids],
                enter_target[zero_ids],
                dt=dt,
                tau_on_s=self.enter_tau_on_s,
                tau_off_s=self.enter_tau_off_s,
            )
            self.exit_score[zero_ids] = self._update_score(
                self.exit_score[zero_ids],
                exit_target[zero_ids],
                dt=dt,
                tau_on_s=self.exit_tau_on_s,
                tau_off_s=self.exit_tau_off_s,
            )

            standing_ids = zero_ids[self.force_state[zero_ids] == MODE_STANDING]
            if standing_ids.numel() > 0:
                self.exit_timer[standing_ids] = 0.0
                blocked_ids = standing_ids[self.cooldown_timer[standing_ids] > 0.0]
                if blocked_ids.numel() > 0:
                    self.enter_timer[blocked_ids] = 0.0
                ready_standing_ids = standing_ids[self.cooldown_timer[standing_ids] <= 0.0]
                if ready_standing_ids.numel() > 0:
                    enter_condition = self.enter_score[ready_standing_ids] >= self.enter_score_threshold
                    self.enter_timer[ready_standing_ids] = torch.where(
                        enter_condition,
                        self.enter_timer[ready_standing_ids] + dt,
                        torch.zeros_like(self.enter_timer[ready_standing_ids]),
                    )
                enter_ids = standing_ids[self.enter_timer[standing_ids] >= self.enter_hold_s]
                if enter_ids.numel() > 0:
                    self.force_state[enter_ids] = MODE_FORCE_WALKING
                    self.enter_score[enter_ids] = 1.0
                    self.exit_score[enter_ids] = 0.0
                    self.enter_timer[enter_ids] = 0.0
                    self.exit_timer[enter_ids] = 0.0
                    self.cooldown_timer[enter_ids] = self.switch_cooldown_s

            walking_ids = zero_ids[self.force_state[zero_ids] == MODE_FORCE_WALKING]
            if walking_ids.numel() > 0:
                self.enter_timer[walking_ids] = 0.0
                blocked_ids = walking_ids[self.cooldown_timer[walking_ids] > 0.0]
                if blocked_ids.numel() > 0:
                    self.exit_timer[blocked_ids] = 0.0
                ready_walking_ids = walking_ids[self.cooldown_timer[walking_ids] <= 0.0]
                if ready_walking_ids.numel() > 0:
                    exit_condition = self.exit_score[ready_walking_ids] >= self.exit_score_threshold
                    self.exit_timer[ready_walking_ids] = torch.where(
                        exit_condition,
                        self.exit_timer[ready_walking_ids] + dt,
                        torch.zeros_like(self.exit_timer[ready_walking_ids]),
                    )
                exit_ids = walking_ids[self.exit_timer[walking_ids] >= self.exit_hold_s]
                if exit_ids.numel() > 0:
                    self.force_state[exit_ids] = MODE_STANDING
                    self.enter_score[exit_ids] = 0.0
                    self.exit_score[exit_ids] = 1.0
                    self.enter_timer[exit_ids] = 0.0
                    self.exit_timer[exit_ids] = 0.0
                    self.cooldown_timer[exit_ids] = self.switch_cooldown_s

        force_excess = torch.relu(force_xy_cap - self.force_baseline - self.baseline_margin_n)

        force_walking_mask = command_zero & (self.force_state == MODE_FORCE_WALKING)
        standing_mask = command_zero & ~force_walking_mask
        walking_mask = ~standing_mask

        self.mode[standing_mask] = MODE_STANDING
        self.mode[force_walking_mask] = MODE_FORCE_WALKING

        return ForceModeSwitchState(
            mode=self.mode.clone(),
            command_zero=command_zero.clone(),
            force_xy_raw=force_xy_raw.clone(),
            force_xy_cap=force_xy_cap.clone(),
            force_baseline=self.force_baseline.clone(),
            force_excess=force_excess.clone(),
            dir_consistency=self.dir_consistency.clone(),
            enter_score=self.enter_score.clone(),
            exit_score=self.exit_score.clone(),
            enter_timer=self.enter_timer.clone(),
            exit_timer=self.exit_timer.clone(),
            cooldown_timer=self.cooldown_timer.clone(),
            standing_mask=standing_mask.clone(),
            walking_mask=walking_mask.clone(),
        )


def mode_name(mode_id: int) -> str:
    return MODE_NAME_BY_ID.get(int(mode_id), f"unknown_{int(mode_id)}")


def override_standing_mask(env: ManagerBasedRLEnv, standing_mask: torch.Tensor) -> None:
    locomotion_term = env.command_manager.get_term("locomotion_commands")
    if not hasattr(locomotion_term, "is_standing_env"):
        raise AttributeError("locomotion_commands term does not expose is_standing_env.")
    locomotion_term.is_standing_env[:] = standing_mask.to(device=env.device, dtype=torch.bool)


def set_locomotion_command(term, cmd_vel_b: torch.Tensor) -> None:
    if not hasattr(term, "vel_command_b"):
        raise AttributeError("locomotion_commands term does not expose vel_command_b for runtime overrides.")
    term.vel_command_b[:] = cmd_vel_b.to(device=term.vel_command_b.device, dtype=term.vel_command_b.dtype)


def extract_base_yaw_deg(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    _, _, yaw_rad = euler_xyz_from_quat(robot.data.root_quat_w)
    return torch.rad2deg(yaw_rad)


def extract_base_yaw_rate_z(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    if hasattr(robot.data, "root_ang_vel_b"):
        return robot.data.root_ang_vel_b[:, 2].clone()
    if hasattr(robot.data, "root_ang_vel_w"):
        return robot.data.root_ang_vel_w[:, 2].clone()
    return torch.full((env.num_envs,), float("nan"), device=env.device)


def write_csv_rows(
    writer: csv.writer,
    *,
    step: int,
    time_s: float,
    num_envs: int,
    cmd_vel_b: torch.Tensor,
    base_lin_vel_local: torch.Tensor,
    estimator_outputs: dict[str, torch.Tensor],
    mode_state: ForceModeSwitchState,
    guidance_state: ForceGuidanceState,
    gait_phase: torch.Tensor,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
    force_target_n: torch.Tensor,
    force_schedule_target_n: torch.Tensor | None = None,
    effective_cmd_vel_b: torch.Tensor | None = None,
    base_yaw_deg: torch.Tensor | None = None,
    base_yaw_rate_z: torch.Tensor | None = None,
):
    base_force_est_local = estimator_outputs["base_force_est_local"]
    base_force_tgt_local = estimator_outputs["base_force_tgt_local"]
    base_force_err_l2 = estimator_outputs["base_force_err_l2"]
    if force_schedule_target_n is None:
        force_schedule_target_n = force_target_n
    if effective_cmd_vel_b is None:
        effective_cmd_vel_b = cmd_vel_b
    if base_yaw_deg is None:
        base_yaw_deg = torch.full((num_envs,), float("nan"), device=cmd_vel_b.device)
    if base_yaw_rate_z is None:
        base_yaw_rate_z = torch.full((num_envs,), float("nan"), device=cmd_vel_b.device)
    for env_id in range(num_envs):
        mode_id = int(mode_state.mode[env_id].item())
        writer.writerow(
            [
                env_id,
                step,
                f"{time_s:.6f}",
                mode_id,
                mode_name(mode_id),
                int(mode_state.command_zero[env_id].item()),
                int(mode_state.standing_mask[env_id].item()),
                int(mode_state.walking_mask[env_id].item()),
                f"{cmd_vel_b[env_id, 0].item():.6f}",
                f"{cmd_vel_b[env_id, 1].item():.6f}",
                f"{cmd_vel_b[env_id, 2].item():.6f}",
                force_segment_name,
                force_direction_name,
                force_phase_name,
                f"{force_schedule_target_n[env_id].item():.6f}",
                f"{force_target_n[env_id].item():.6f}",
                f"{base_force_est_local[env_id, 0].item():.6f}",
                f"{base_force_est_local[env_id, 1].item():.6f}",
                f"{base_force_est_local[env_id, 2].item():.6f}",
                f"{base_force_tgt_local[env_id, 0].item():.6f}",
                f"{base_force_tgt_local[env_id, 1].item():.6f}",
                f"{base_force_tgt_local[env_id, 2].item():.6f}",
                f"{base_force_err_l2[env_id].item():.6f}",
                f"{base_lin_vel_local[env_id, 0].item():.6f}",
                f"{base_lin_vel_local[env_id, 1].item():.6f}",
                f"{base_lin_vel_local[env_id, 2].item():.6f}",
                f"{mode_state.force_xy_raw[env_id].item():.6f}",
                f"{mode_state.force_xy_cap[env_id].item():.6f}",
                f"{mode_state.force_baseline[env_id].item():.6f}",
                f"{mode_state.force_excess[env_id].item():.6f}",
                f"{mode_state.dir_consistency[env_id].item():.6f}",
                f"{mode_state.enter_score[env_id].item():.6f}",
                f"{mode_state.exit_score[env_id].item():.6f}",
                f"{mode_state.enter_timer[env_id].item():.6f}",
                f"{mode_state.exit_timer[env_id].item():.6f}",
                f"{mode_state.cooldown_timer[env_id].item():.6f}",
                f"{gait_phase[env_id, 0].item():.6f}",
                int(guidance_state.enabled),
                int(guidance_state.active_mask[env_id].item()),
                f"{effective_cmd_vel_b[env_id, 0].item():.6f}",
                f"{effective_cmd_vel_b[env_id, 1].item():.6f}",
                f"{effective_cmd_vel_b[env_id, 2].item():.6f}",
                f"{math.degrees(guidance_state.heading_error_rad[env_id].item()):.6f}",
                f"{math.degrees(guidance_state.estimated_force_heading_rad[env_id].item()):.6f}",
                f"{guidance_state.estimated_force_mag[env_id].item():.6f}",
                f"{guidance_state.motion_sign[env_id].item():.0f}",
                f"{base_yaw_deg[env_id].item():.6f}",
                f"{base_yaw_rate_z[env_id].item():.6f}",
            ]
        )


def maybe_log_step(
    *,
    step: int,
    time_s: float,
    cmd_vel_b: torch.Tensor,
    effective_cmd_vel_b: torch.Tensor,
    mode_state: ForceModeSwitchState,
    guidance_state: ForceGuidanceState,
    gait_phase: torch.Tensor,
    force_segment_name: str,
    force_direction_name: str,
    force_phase_name: str,
) -> None:
    if args_cli.log_interval <= 0 or step % args_cli.log_interval != 0:
        return
    env_id = 0
    mode_id = int(mode_state.mode[env_id].item())
    print(
        f"[SMART_GAIT][step={step:05d}] "
        f"t={time_s:6.2f}s "
        f"segment={force_segment_name}/{force_direction_name}/{force_phase_name} "
        f"mode={mode_name(mode_id)} "
        f"command_zero={bool(mode_state.command_zero[env_id].item())} "
        f"standing_mask={bool(mode_state.standing_mask[env_id].item())} "
        f"cmd=({cmd_vel_b[env_id, 0].item():+.3f}, {cmd_vel_b[env_id, 1].item():+.3f}, {cmd_vel_b[env_id, 2].item():+.3f}) "
        f"eff_cmd=({effective_cmd_vel_b[env_id, 0].item():+.3f}, {effective_cmd_vel_b[env_id, 1].item():+.3f}, {effective_cmd_vel_b[env_id, 2].item():+.3f}) "
        f"F_raw={mode_state.force_xy_raw[env_id].item():.3f}N "
        f"F_cap={mode_state.force_xy_cap[env_id].item():.3f}N "
        f"F_base={mode_state.force_baseline[env_id].item():.3f}N "
        f"F_ex={mode_state.force_excess[env_id].item():.3f}N "
        f"dir={mode_state.dir_consistency[env_id].item():.3f} "
        f"enter_s={mode_state.enter_score[env_id].item():.3f} "
        f"exit_s={mode_state.exit_score[env_id].item():.3f} "
        f"enter_t={mode_state.enter_timer[env_id].item():.3f}s "
        f"exit_t={mode_state.exit_timer[env_id].item():.3f}s "
        f"cooldown={mode_state.cooldown_timer[env_id].item():.3f}s "
        f"guide_active={bool(guidance_state.active_mask[env_id].item())} "
        f"guide_err={math.degrees(guidance_state.heading_error_rad[env_id].item()):+.1f}deg "
        f"gait_phase={gait_phase[env_id, 0].item():.3f}"
    )


def main():
    output_csv = resolve_output_csv(args_cli.output_csv)
    env_cfg = build_env_cfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    base_force_obs_scale = float(env_cfg.observations.pred.base_forces_local.scale)

    checkpoint_path = retrieve_file_path(args_cli.checkpoint) if args_cli.checkpoint else None
    jit_path = retrieve_file_path(args_cli.policy_jit) if args_cli.policy_jit else None

    wrapped_env = None
    estimator_model = None
    if checkpoint_path:
        print(f"[INFO] Loading raw checkpoint: {checkpoint_path}")
        policy, wrapped_env, estimator_model = load_raw_checkpoint_policy(checkpoint_path, env)
        env.reset()
        obs = wrapped_env.get_observations()
    elif jit_path:
        raise RuntimeError(
            "This task uses AsymmetricActorCritic with a history encoder, but IsaacLab's stock "
            "export_policy_as_jit() only exports policy.actor and drops the encoder/adaptation path. "
            "As a result, the exported policy.pt is not a faithful inference artifact for this task. "
            "Use --checkpoint for smart_gait, or add a custom exporter that wraps the full act_inference() path."
        )
    else:
        raise RuntimeError("Either --checkpoint or --policy-jit must be provided.")

    force_term = env.command_manager.get_term("base_force_ext")
    if not hasattr(force_term, "set_force") or not hasattr(force_term, "clear_force"):
        raise TypeError(
            f"commands.base_force_ext must be a scripted force term exposing set_force()/clear_force() for "
            f"{EVAL_VARIANT_NOTE} smart-gait evaluation."
        )
    locomotion_term = env.command_manager.get_term("locomotion_commands")
    markers = SmartGaitMarkers(env)

    scheduler = None
    if args_cli.force_schedule == FORCE_SCHEDULE_DEMO:
        scheduler = ScriptedForceScheduler(
            num_envs=env.num_envs,
            device=env.device,
            segments=build_demo_force_segments(float(args_cli.episode_length_s)),
        )
    elif args_cli.force_schedule == FORCE_SCHEDULE_STOP_GO:
        scheduler = StopGoTuningForceScheduler(
            num_envs=env.num_envs,
            device=env.device,
            step_dt=float(env.step_dt),
            headings_deg=parse_heading_deg_sequence(args_cli.stop_go_heading_seq),
        )
    elif args_cli.force_schedule == FORCE_SCHEDULE_HUMAN_GUIDE:
        scheduler = HumanGuideForceScheduler(
            num_envs=env.num_envs,
            device=env.device,
            step_dt=float(env.step_dt),
        )

    locomotion_cfg = env_cfg.commands.locomotion_commands
    switcher = ForceModeSwitcher(
        num_envs=env.num_envs,
        device=env.device,
        lin_vel_x_clip=float(locomotion_cfg.lin_vel_x_clip),
        lin_vel_y_clip=float(locomotion_cfg.lin_vel_y_clip),
        ang_vel_yaw_clip=float(locomotion_cfg.ang_vel_yaw_clip),
        baseline_tau_s=float(args_cli.baseline_tau_s),
        baseline_margin_n=float(args_cli.baseline_margin_n),
        baseline_update_max_n=float(args_cli.baseline_update_max_n),
        evidence_cap_n=float(args_cli.evidence_cap_n),
        enter_th_n=float(args_cli.enter_threshold_n),
        exit_th_n=float(args_cli.exit_threshold_n),
        enter_tau_on_s=float(args_cli.enter_tau_on_s),
        enter_tau_off_s=float(args_cli.enter_tau_off_s),
        exit_tau_on_s=float(args_cli.exit_tau_on_s),
        exit_tau_off_s=float(args_cli.exit_tau_off_s),
        enter_score_threshold=float(args_cli.enter_score_threshold),
        exit_score_threshold=float(args_cli.exit_score_threshold),
        dir_consistency_min_force_n=float(args_cli.dir_consistency_min_force_n),
        dir_consistency_tau_s=float(args_cli.dir_consistency_tau_s),
        dir_consistency_threshold=float(args_cli.dir_consistency_threshold),
        switch_cooldown_s=float(args_cli.switch_cooldown_s),
        enter_hold_s=float(args_cli.enter_hold_s),
        exit_hold_s=float(args_cli.exit_hold_s),
    )
    guidance_controller = ForceGuidedTurningController(
        num_envs=env.num_envs,
        device=env.device,
        base_force_kd=float(locomotion_cfg.base_force_kd),
        max_speed=max(abs(float(locomotion_cfg.ranges.lin_vel_x[0])), abs(float(locomotion_cfg.ranges.lin_vel_x[1]))),
        max_yaw_rate=max(abs(float(locomotion_cfg.ranges.ang_vel_z[0])), abs(float(locomotion_cfg.ranges.ang_vel_z[1]))),
    )

    if livestream_enabled:
        print("[INFO] LIVESTREAM enabled; cameras forced on for WebRTC compatibility.")

    num_envs = env.num_envs
    device = env.device
    step_dt = float(env.step_dt)
    max_steps = max(int(round(float(env_cfg.episode_length_s) / step_dt)), 1)
    mode_counts = torch.zeros((num_envs, 3), device=device, dtype=torch.long)
    nominal_cmd_vel_b = torch.zeros((num_envs, 3), device=device)
    nominal_cmd_vel_b[:, 0] = float(args_cli.cmd_vx)
    nominal_cmd_vel_b[:, 1] = float(args_cli.cmd_vy)
    nominal_cmd_vel_b[:, 2] = float(args_cli.cmd_wz)
    set_locomotion_command(locomotion_term, nominal_cmd_vel_b)

    with output_csv.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
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
                "scheduled_force_target_n",
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
        )

        for step in range(max_steps):
            if not simulation_app.is_running():
                break

            time_s = step * step_dt
            robot = env.scene["robot"]
            cmd_vel_b = nominal_cmd_vel_b.clone()
            base_lin_vel_local = robot.data.root_lin_vel_b.clone()
            base_yaw_deg = extract_base_yaw_deg(env)
            base_yaw_rate_z = extract_base_yaw_rate_z(env)

            if scheduler is None:
                scheduler_outputs = {
                    "segment_name": FORCE_SCHEDULE_NONE,
                    "direction_name": DIR_NONE,
                    "phase_name": "idle",
                    "force_schedule_target_n": torch.zeros((num_envs,), device=device),
                    "force_target_n": torch.zeros((num_envs,), device=device),
                    "force_w": torch.zeros((num_envs, 3), device=device),
                    "axis_xy": torch.full((num_envs, 2), float("nan"), device=device),
                    "is_active": False,
                }
            else:
                scheduler_outputs = scheduler.update(env, time_s)
                if "force_schedule_target_n" not in scheduler_outputs:
                    scheduler_outputs["force_schedule_target_n"] = scheduler_outputs["force_target_n"]

            force_term.clear_force()
            if scheduler_outputs["is_active"]:
                force_term.set_force(scheduler_outputs["force_w"])

            with torch.inference_mode():
                estimator_outputs = extract_force_estimator_outputs(
                    obs,
                    estimator_model,
                    base_force_scale=base_force_obs_scale,
                )

            mode_state = switcher.update(
                cmd_vel_b,
                estimator_outputs["base_force_est_local"],
                dt=step_dt,
            )
            guidance_state = guidance_controller.compute(
                cmd_vel_b,
                estimator_outputs["base_force_est_local"],
                mode_state,
            )
            set_locomotion_command(locomotion_term, guidance_state.effective_cmd_vel_b)
            override_standing_mask(env, mode_state.standing_mask)

            for mode_id in range(3):
                mode_counts[:, mode_id] += (mode_state.mode == mode_id).to(dtype=torch.long)

            with torch.inference_mode():
                actions = policy(obs)
                obs, _, dones, _ = wrapped_env.step(actions)

            gait_phase = env.command_manager.get_command("gait_phase").clone()
            write_csv_rows(
                writer,
                step=step,
                time_s=time_s,
                num_envs=num_envs,
                cmd_vel_b=cmd_vel_b,
                base_lin_vel_local=base_lin_vel_local,
                estimator_outputs=estimator_outputs,
                mode_state=mode_state,
                guidance_state=guidance_state,
                gait_phase=gait_phase,
                force_segment_name=str(scheduler_outputs["segment_name"]),
                force_direction_name=str(scheduler_outputs["direction_name"]),
                force_phase_name=str(scheduler_outputs["phase_name"]),
                force_target_n=scheduler_outputs["force_target_n"],
                force_schedule_target_n=scheduler_outputs["force_schedule_target_n"],
                effective_cmd_vel_b=guidance_state.effective_cmd_vel_b,
                base_yaw_deg=base_yaw_deg,
                base_yaw_rate_z=base_yaw_rate_z,
            )
            maybe_log_step(
                step=step,
                time_s=time_s,
                cmd_vel_b=cmd_vel_b,
                effective_cmd_vel_b=guidance_state.effective_cmd_vel_b,
                mode_state=mode_state,
                guidance_state=guidance_state,
                gait_phase=gait_phase,
                force_segment_name=str(scheduler_outputs["segment_name"]),
                force_direction_name=str(scheduler_outputs["direction_name"]),
                force_phase_name=str(scheduler_outputs["phase_name"]),
            )

            markers.update()

            done_mask = dones.to(dtype=torch.bool, device=device)
            if done_mask.any():
                break

    force_term.clear_force()
    print(f"[INFO] CSV written to: {output_csv}")
    for env_id in range(num_envs):
        standing_s = mode_counts[env_id, MODE_STANDING].item() * step_dt
        force_walking_s = mode_counts[env_id, MODE_FORCE_WALKING].item() * step_dt
        command_walking_s = mode_counts[env_id, MODE_COMMAND_WALKING].item() * step_dt
        print(
            f"[SUMMARY][env {env_id}] "
            f"standing={standing_s:.2f}s "
            f"force_walking={force_walking_s:.2f}s "
            f"command_walking={command_walking_s:.2f}s"
        )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
