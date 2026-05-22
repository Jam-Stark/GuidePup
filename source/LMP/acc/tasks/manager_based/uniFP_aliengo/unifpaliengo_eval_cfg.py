from __future__ import annotations

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from . import mdp
from .commands import ScriptedForceCommandCfg
from .unifpaliengo_env_cfg import CommandsCfg as TrainCommandsCfg
from .unifpaliengo_env_cfg import LEG_JOINT_NAMES
from .unifpaliengo_env_cfg import UniFPAliengoEnvCfg

RESET_JOINT_NAMES = list(LEG_JOINT_NAMES)


@configclass
class ReversePushEvalCommandsCfg(TrainCommandsCfg):
    """Command configuration for deterministic reverse-push evaluation."""

    @configclass
    class BaseForceExtCfg(ScriptedForceCommandCfg):
        command_name: str = "base_force_ext"
        is_global: bool = True

    base_force_ext: BaseForceExtCfg = BaseForceExtCfg()


@configclass
class ReversePushEvalEventCfg:
    """Minimal deterministic events for evaluation."""

    reset_gait_state = EventTerm(
        func=mdp.reset_gait_state,
        mode="reset",
    )

    reset_robot_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    reset_joints_by_offset = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot", joint_names=RESET_JOINT_NAMES, preserve_order=True),
        },
    )


@configclass
class ReversePushEvalCurriculumCfg:
    """Disable curriculum updates for deterministic evaluation."""


@configclass
class UniFPAliengoReversePushEvalCfg(UniFPAliengoEnvCfg):
    """Test-only cfg for scripted reverse-push locomotion evaluation."""

    commands: ReversePushEvalCommandsCfg = ReversePushEvalCommandsCfg()
    events: ReversePushEvalEventCfg = ReversePushEvalEventCfg()
    curriculum: ReversePushEvalCurriculumCfg = ReversePushEvalCurriculumCfg()

    def __post_init__(self) -> None:
        super().__post_init__()

        # Keep the robot framed in the viewport/WebRTC stream by tracking the base root.
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
        self.viewer.env_index = 0
        self.viewer.eye = (4.0, -2.5, 2.2)
        self.viewer.lookat = (0.0, 0.0, 0.5)

        self.scene.robot.actuators["hips"].stiffness = 140.0
        self.scene.robot.actuators["hips"].damping = 5.0
        self.scene.robot.actuators["thighs"].stiffness = 140.0
        self.scene.robot.actuators["thighs"].damping = 5.0
        self.scene.robot.actuators["calfs"].stiffness = 220.0
        self.scene.robot.actuators["calfs"].damping = 9.0

        self.commands.gait_phase.gait_frequency = 3.0
        self.commands.locomotion_commands.debug_vis = False

        self.commands.locomotion_commands.resampling_time_range = (1.0e9, 1.0e9)
        self.commands.locomotion_commands.rel_standing_envs = 0.0
        self.commands.locomotion_commands.ranges.lin_vel_x = (0.4, 0.4)
        self.commands.locomotion_commands.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.locomotion_commands.ranges.ang_vel_z = (0.0, 0.0)

        self.rewards.feet_air_time.params["target_air_time"] = 0.1667

        self.observations.policy.enable_corruption = False
