# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from typing import Tuple
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers.recorder_manager import RecorderManagerBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils import modifiers as modifier_utils
from isaaclab.utils.noise import GaussianNoiseCfg

from .robot_constants import BASE_BODY_NAME, FOOT_BODY_NAMES, LEG_JOINT_NAMES  # noqa: F401
from . import mdp
from .mdp.recorders import FrontBackLegTorquesRecorderCfg
from isaaclab.managers import CommandTermCfg, CurriculumTermCfg
from isaaclab.sensors import ContactSensorCfg

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.converters import UrdfConverterCfg
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG
from isaaclab.markers import VisualizationMarkersCfg

from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
import isaaclab.terrains.height_field.hf_terrains_cfg as hf_gen_cfg

from .commands import (
    ExtendedUniformVelocityCommand,
    GaitPhaseCommandCfg,
    VariableForceCommandCfg,
)
from .metrics import (
    ActionDistributionMetricsCommandCfg,
    FeetContactForceMetricsCommandCfg,
    LegTorqueMetricsCommandCfg,
)

##
# Global configuration constants
##
FORCE_COMMAND_RESUME_STEPS = 0 * 24  # 恢复力命令的训练步数阈值
FORCE_COMMAND_START_STEPS = 8000 * 24  # 启用力命令的训练步数阈值
BASE_EXT_FULL_RANGE = (-65.0, 65.0)  # A2 base力的完整范围
BASE_FORCE_KD = 160.0
POLICY_OBS_DELAY_RANGE_STEPS = (0, 2)  # policy观测延迟范围（env steps）
LEG_ACTION_DELAY_RANGE_STEPS = (0, 1)  # 腿部动作延迟范围（env steps）


def make_policy_obs_delay_modifier_cfg() -> modifier_utils.ModifierCfg:
    """Create a fresh delay modifier config for a policy observation term."""
    return modifier_utils.ModifierCfg(
        func=mdp.RandomizedDelayModifier,
        params={
            "min_delay_steps": POLICY_OBS_DELAY_RANGE_STEPS[0],
            "max_delay_steps": POLICY_OBS_DELAY_RANGE_STEPS[1],
        },
    )


##
# Pre-defined configs
##
@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    @configclass
    class LocomotionCommandCfg(CommandTermCfg):
        """Configuration for base locomotion commands."""

        class_type = ExtendedUniformVelocityCommand
        resampling_time_range: Tuple[float, float] = (5.0, 5.0)
        heading_command: bool = False
        rel_standing_envs: float = 0.3

        ang_vel_yaw_clip: float = 0.2
        lin_vel_x_clip: float = 0.1
        lin_vel_y_clip: float = 0.1
        base_force_ext_name: str = "base_force_ext"
        base_force_kd: float = BASE_FORCE_KD
        debug_vis_speed_ref: float = 0.3
        debug_vis_min_scale: float = 1.2
        debug_vis_max_scale: float = 3.0

        command_name: str = "locomotion_commands"
        dim: int = 3

        @configclass
        class Ranges:
            """Ranges for the commands."""

            lin_vel_x: Tuple[float, float] = (-1.2, 1.2)
            lin_vel_y: Tuple[float, float] = (-0.6, 0.6)
            ang_vel_z: Tuple[float, float] = (-1.0, 1.0)
            heading: Tuple[float, float] = (-math.pi, math.pi)

        ranges: Ranges = Ranges()

        strategy: str = "uniform"
        asset_name: str = "robot"
        body_name: str = BASE_BODY_NAME
        debug_vis: bool = True

        goal_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
            prim_path="/Visuals/Command/velocity_goal",
            markers={
                "arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.6, 0.6, 0.6),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
            },
        )
        current_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
            prim_path="/Visuals/Command/velocity_current",
            markers={
                "arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.6, 0.6, 0.6),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.9, 0.9)),
                ),
            },
        )

    @configclass
    class GaitPhaseCfg(GaitPhaseCommandCfg):
        """Configuration for open-loop gait phase clock."""

        locomotion_command_name: str = "locomotion_commands"
        gait_frequency: float = 2.0
        phase_offset: float = 0.5
        initial_phase: float = 0.0

    locomotion_commands: LocomotionCommandCfg = LocomotionCommandCfg()
    gait_phase: GaitPhaseCfg = GaitPhaseCfg()
    leg_torque_metrics: LegTorqueMetricsCommandCfg = LegTorqueMetricsCommandCfg()
    feet_contact_force_metrics: FeetContactForceMetricsCommandCfg = FeetContactForceMetricsCommandCfg()
    action_distribution_metrics: ActionDistributionMetricsCommandCfg = ActionDistributionMetricsCommandCfg()

    @configclass
    class BaseForceExtCfg(VariableForceCommandCfg):
        force_range: Tuple[float, float] = (0.0, 0.0)
        push_interval_range_s: Tuple[float, float] | None = (6.0, 12.0)
        duration_range_s: Tuple[float, float] | None = (1.0, 3.0)
        settling_time_s: float | None = 3.0
        prob_freed: float = 0.2
        is_global: bool = True

        force_z_scale: float = 0.1

    base_force_ext: BaseForceExtCfg = BaseForceExtCfg()


##
# Scene definition
##


@configclass
class UniFPTerrainCfg(TerrainGeneratorCfg):
    """
    - 实现"rough flat"粗糙平地
    
    - 地形类型为trimesh（三角网格）
    - 100%使用rough flat地形
    - 高度噪声范围[0.00, 0.1]米
    - 地形块大小8m x 8m
    - 10行20列网格布局
    """
    # 地形生成器设置
    terrain_type = "generator"
    
    # 网格布局
    num_rows = 10
    num_cols = 20
    
    # 地形块大小 (米)
    size = (8.0, 8.0)
    
    # 课程学习设置 (关闭)
    curriculum = False
    
    # 子地形配置
    sub_terrains = {
        "rough_flat": hf_gen_cfg.HfRandomUniformTerrainCfg(
            proportion=1.0,
            noise_range=(0.00, 0.03),  # 高度噪声范围 (米)
            noise_step=0.005,
            border_width=0.0,
        )
    }


@configclass
class UniFPSceneCfg(InteractiveSceneCfg):
    # 关键：禁用物理属性复制，以支持域随机化
    replicate_physics = False
    # 开启 Fabric 优化：使用 USD Fabric 进行环境克隆，提高实例化速度
    # 注意：开启此选项会导致 Articulation 随机化（如质量）出错，因此暂时关闭
    clone_in_fabric = False

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=UniFPTerrainCfg(),
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
        debug_vis=False,
    )
    """ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,  # 明确定义静摩擦
                dynamic_friction=1.0,  # 明确定义动摩擦
                restitution=0.0       # 明确定义恢复系数
            )
        )
    )"""

    # 机器人资产配置
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="resources/a2_description/urdf/a2.urdf",
            fix_base=False,
            activate_contact_sensors=True,  # 启用接触传感器
            merge_fixed_joints=False,  # 保留足部等固定关节链接
            self_collision=True,
            
            # 刚体物理属性配置
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,      # VBC: max_linear_velocity
                max_angular_velocity=1000.0,     # VBC: max_angular_velocity
                max_depenetration_velocity=300.0,
                disable_gravity=False,           # VBC: disable_gravity = False
                retain_accelerations=False,
                linear_damping=0.0,              # VBC: linear_damping = 0.0
                angular_damping=0.0,             # VBC: angular_damping = 0.0
            ),
            
            # 关节根属性配置
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,   # 启用自碰撞（与self_collision配合使用）
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
                articulation_enabled=True,
            ),
            
            # 关键：为 URDF 导入定义关节驱动，并关闭目标
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                drive_type="force",          # DOF_MODE_EFFORT
                target_type="none",
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=0.0,
                    damping=0.0,
                ),
            ),
            
        ),
        
        # 初始状态配置
        init_state=ArticulationCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.55],
            rot=[1.0, 0.0, 0.0, 0.0],
            joint_pos={
                "FL_hip_joint": 0.0,  # [rad]
                "FL_thigh_joint": 0.5,  # [rad]
                "FL_calf_joint": -1.0,  # [rad]

                "RL_hip_joint": 0.0,  # [rad]
                "RL_thigh_joint": 0.5,  # [rad]
                "RL_calf_joint": -1.0,  # [rad]

                "FR_hip_joint": 0.0,  # [rad]
                "FR_thigh_joint": 0.5,  # [rad]
                "FR_calf_joint": -1.0,  # [rad]

                "RR_hip_joint": 0.0,  # [rad]
                "RR_thigh_joint": 0.5,  # [rad]
                "RR_calf_joint": -1.0,  # [rad]
            },
        ),
        
        # 软关节限制因子
        soft_joint_pos_limit_factor=0.9,
        # 执行器配置
        actuators={
            "hips": ImplicitActuatorCfg(
                joint_names_expr=[
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",          # hip joints
                ],
                effort_limit_sim=120.0,
                velocity_limit_sim=22.0,
                stiffness=140.0,
                damping=5.0,
                armature=0.0,
            ),
            "thighs": ImplicitActuatorCfg(
                joint_names_expr=[
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",  # thigh joints
                ],
                effort_limit_sim=120.0,
                velocity_limit_sim=22.0,
                stiffness=140.0,
                damping=5.0,
                armature=0.0,
            ),
            "calfs": ImplicitActuatorCfg(
                joint_names_expr=[
                    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",      # calf joints
                ],
                effort_limit_sim=180.0,
                velocity_limit_sim=14.6667,
                stiffness=220.0,
                damping=9.0,
                armature=0.0,
            ),
        },

    )
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    # sensors
    # 足部接触传感器
    contact_feet_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*_foot",
        update_period=0.0,
        force_threshold=1.5,
        history_length=2,
        debug_vis=False,
        track_air_time=True,
        # 关键：必须启用contact processing才能获取接触力数据
        track_pose=False,  # 不需要跟踪姿态（性能优化）
    )
    
    # 非法接触检测 penalize_contacts_on = ["thigh", "calf", "base_link"]
    penalize_contacts: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*(_thigh|_calf|base_link)",
        update_period=0.0,
        force_threshold=1.0,
        history_length=1,
        debug_vis=False,
    )

##
# MDP settings
##

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    leg_actions = mdp.DelayedJointPositionActionCfg(
        asset_name="robot",
        joint_names=list(LEG_JOINT_NAMES),
        scale=0.25,
        use_default_offset=True,
        preserve_order=True,
        clip={".*": (-100.0, 100.0)},
        min_delay_steps=LEG_ACTION_DELAY_RANGE_STEPS[0],
        max_delay_steps=LEG_ACTION_DELAY_RANGE_STEPS[1],
    )
    base_force_ext = mdp.ApplyForceCommandActionCfg(
        asset_name="robot",
        body_name=BASE_BODY_NAME,
        command_name="base_force_ext",
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for the active locomotion policy.

        单帧观测为 12DoF leg-only 契约：
        - base_orientation: 2
        - base_ang_vel: 3
        - dof_pos: 12
        - joint_vel: 12
        - last_action: 12
        - gait_clock: 2
        - commands: 3

        单帧总计 46 维；`history_length=32` 后按时间维展平。
        """

        # 1. 机身方向 (2维) - 提取projected_gravity的x,y分量获取roll和pitch信息
        base_orientation = ObsTerm(
            func=mdp.base_orientation_2d,
            params={"asset_cfg": SceneEntityCfg("robot")},
            modifiers=[make_policy_obs_delay_modifier_cfg()],
            noise=GaussianNoiseCfg(mean=0.0, std=0.05, operation="add"),
            clip=(-100.0, 100.0)
        )
        # 2,base_ang_vel,基座角速度,3
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            modifiers=[make_policy_obs_delay_modifier_cfg()],
            noise=GaussianNoiseCfg(mean=0.0, std=0.2, operation="add"),
            scale=0.25,
            clip=(-100.0, 100.0)
        )
        # 3. 腿部关节位置 (12维)
        dof_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)},
            modifiers=[make_policy_obs_delay_modifier_cfg()],
            noise=GaussianNoiseCfg(mean=0.0, std=0.01, operation="add"),
            scale=1.0,
            clip=(-100.0, 100.0)
        )
        
        # 4. 腿部关节速度 (12维)
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)},
            modifiers=[make_policy_obs_delay_modifier_cfg()],
            noise=GaussianNoiseCfg(mean=0.0, std=1.5, operation="add"),
            scale=0.05,
            clip=(-100.0, 100.0)
        )

        # 5. 上一时刻的动作 (12维)
        last_action = ObsTerm(func=mdp.last_action,
                              clip=(-100.0, 100.0))
        # 6. 步态时钟信号 (2维) - 全局时钟正弦余弦
        gait_clock = ObsTerm(
            func=mdp.gait_clock_signal,
            clip=(-100.0, 100.0)
        )
        # 7. locomotion 指令 (3维)
        commands = ObsTerm(
            func=mdp.locomotion_commands_with_scales,
            params={"scales": (2.0, 2.0, 0.25)},
            clip=(-100.0, 100.0)
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 32
            self.flatten_history_dim = True
    
    @configclass
    class privilegedObs(ObsGroup):
        """Privileged observations for critic/debug on the leg-only branch.

        单帧布局仅包含 locomotion 与 base-force 相关信息：
        1. base_lin_vel (3)
        2. base_forces_local (3)
        3. diff / ref_dof_pos_error (12)
        4. mass_params (5)
        5. friction (1)
        6. motor_strengths (12)
        7. stance_mask (4)
        8. foot_contacts (4)
        9. projected_gravity (3)
        10. base_ang_vel (3)
        11. dof_pos (12)
        12. joint_vel (12)
        13. last_action (12)
        14. gait_clock (2)
        15. commands (3)
        16. base_lin_vel_offset (2)

        单帧总计 93 维；`history_length=3` 后按时间维展平。
        """

        # 1. 基座线速度 (3维)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            scale=2.0,
            clip=(-100.0, 100.0)
        )

        # 4. 基座受力 (3维)
        base_forces_local = ObsTerm(
            func=mdp.base_forces_local,
            params={"command_name": "base_force_ext"},
            scale=0.01,
            clip=(-100.0, 100.0)
        )

        # 5. diff,关节位置误差 (Ref - Curr),12 (4腿x3),跟踪性能
        diff = ObsTerm(
            func=mdp.ref_dof_pos_error,
            params={
                "gait_command_name": "gait_phase",
                "target_joint_pos_thd": 0.1,
                "target_joint_pos_scale": 0.35,
                "calf_scale_factor": 2.0,
            },
            clip=(-100.0, 100.0)
        )

        # 6. 随机化质量参数 (5维)
        mass_params = ObsTerm(
            func=mdp.privileged_mass_params,
            clip=(-100.0, 100.0)
        )
        
        # 7. 随机化摩擦系数 (1维)
        friction = ObsTerm(
            func=mdp.privileged_friction,
            clip=(-100.0, 100.0)
        )
        
        # 8. 随机化电机强度 (12维)
        motor_strengths = ObsTerm(
            func=mdp.privileged_motor_strengths,
            clip=(-100.0, 100.0)
        )

        # 9. 步态计划 (4维) - 四个足端的计划着地状态
        stance_mask = ObsTerm(
            func=mdp.stance_mask,
            params={
                "gait_command_name": "gait_phase",
            },
            clip=(-100.0, 100.0)
        )

        # 10. 足端实际接触状态 (4维) - 四个足端的接触状态
        foot_contacts = ObsTerm(
            func=mdp.foot_contact_binary,
            params={
                "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
                "threshold": 1.5
            },
            clip=(-100.0, 100.0)
        )
        # 11,projected_gravity,重力在基座坐标系投影,3
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            clip=(-100.0, 100.0)
        )
        #  12,base_ang_vel,基座角速度,3
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            clip=(-100.0, 100.0)
        )
        # 13. 腿部关节位置 (12维)
        dof_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)},
            scale=1.0,
            clip=(-100.0, 100.0)
        )
        
        # 14. 腿部关节速度 (12维)
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)},
            scale=0.05,
            clip=(-100.0, 100.0)
        )

        # 15. 上一时刻的动作 (12维)
        last_action = ObsTerm(func=mdp.last_action)

        # 16. 步态时钟信号 (2维)
        gait_clock = ObsTerm(
            func=mdp.gait_clock_signal,
            params={
                "gait_command_name": "gait_phase",
            },
            clip=(-100.0, 100.0)
        )

        # 17. locomotion 指令 (3维)
        commands = ObsTerm(
            func=mdp.locomotion_commands_with_scales,
            params={"scales": (2.0, 2.0, 0.25)},
            clip=(-100.0, 100.0)
        )

        # 添加force offset后的base xy速度 (2维)
        base_lin_vel_offset = ObsTerm(
            func=mdp.base_lin_vel_offset,
            params={
                "locomotion_command_name": "locomotion_commands",
                "base_force_ext_name": "base_force_ext",
                "base_force_kd": BASE_FORCE_KD,
            },
            scale=2.0,
            clip=(-100.0, 100.0)
        )
        
        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 3
            self.flatten_history_dim = True
    
    @configclass
    class predObs(ObsGroup):
        """Predictor observation group used by the auxiliary estimator path."""
        # 1. 基座线速度 (3维)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            scale=2.0,
            clip=(-100.0, 100.0)
        )

        # 4. 基座受力 (3维)
        base_forces_local = ObsTerm(
            func=mdp.base_forces_local,
            params={"command_name": "base_force_ext", "use_applied": True},
            scale=0.01,
            clip=(-100.0, 100.0)
        )
        
        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True
    # observation groups
    policy: PolicyCfg = PolicyCfg()
    privileged: privilegedObs = privilegedObs()
    pred: predObs = predObs()


# 课程学习辅助函数
def modify_lin_vel_x_range(env, env_ids, old_value, new_value, num_steps):
    """在指定步数后修改lin_vel_x范围
    
    Args:
        env: 环境实例
        env_ids: 环境ID列表
        old_value: 当前值
        new_value: 新值
        num_steps: 触发变更的步数阈值
    
    Returns:
        新值或NO_CHANGE标记
    """
    if env.common_step_counter > num_steps:
        return new_value
    return mdp.modify_term_cfg.NO_CHANGE


def modify_force_range(
    env,
    env_ids,
    old_value,
    zero_range,
    full_range,
    start_steps=FORCE_COMMAND_START_STEPS,
    resume_steps=0,
):
    """Clamp force command range to zero until the curriculum threshold."""
    if env.common_step_counter + resume_steps > start_steps:
        return full_range
    return zero_range


@configclass
class CurriculumCfg:
    """Curriculum learning configuration.
    
    课程学习配置：在训练120000步后，将x方向速度范围从(0.0, 0.8)扩展到(-0.8, 0.8)，
    使机器人学习后退能力。
    """

    # 课程学习：120000步后扩展x方向速度范围从(0.0, 0.8)到(-0.8, 0.8)
    """lin_vel_x_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.locomotion_commands.ranges.lin_vel_x",
            "modify_fn": modify_lin_vel_x_range,
            "modify_params": {
                "new_value": (-0.8, 0.8),
                "num_steps": 100_000,
            }
        }
    )"""

    base_force_ext_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.base_force_ext.force_range",
            "modify_fn": modify_force_range,
            "modify_params": {
                "zero_range": (0.0, 0.0),
                "full_range": BASE_EXT_FULL_RANGE,
                "start_steps": FORCE_COMMAND_START_STEPS,
                "resume_steps": FORCE_COMMAND_RESUME_STEPS,
            },
        },
    )


@configclass
class RecordersCfg(RecorderManagerBaseCfg):
    """Recorder terms for logging environment data."""

    leg_torques = FrontBackLegTorquesRecorderCfg()
            

@configclass
class EventCfg:
    """Configuration for events."""

    # ========== 重置事件 (reset mode) ==========
    # 重置步态时钟状态（相位缓存），避免跨 episode 漂移
    reset_gait_state = EventTerm(
        func=mdp.reset_gait_state,
        mode="reset",
    )
    
    # 重置机器人根部状态 (位置、姿态、速度)
    reset_robot_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.05, 0.05)},
            "velocity_range": {
                "x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.1, 0.1),
                "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-0.1, 0.1)
            },
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    
    # 重置关节位置和速度
    reset_joints_by_offset = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True),
        },
    )
    
    # ========== 域随机化事件 (reset mode) ==========
    
    # 1. 物理材质随机化 - 静摩擦和动摩擦
    randomize_physics_material = EventTerm(
        func=mdp.my_randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 3.0),
            "dynamic_friction_range": (0.3, 3.0),
            "restitution_range": (0.0, 0.0),  # 恢复系数保持为0
            "num_buckets": 1000,  # 离散化桶数，提高采样效率
        },
    )

    
    # 2. 腿部关节刚度和阻尼随机化
    randomize_leg_joint_gains = EventTerm(
        func=mdp.my_randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True),
            "stiffness_distribution_params": (0.9, 1.1),  # 缩放因子范围
            "damping_distribution_params": (0.9, 1.1),
            "operation": "scale",  # 缩放操作
            "distribution": "uniform",  # 均匀分布
        },
    )

    # ========== 启动时随机化事件 (startup mode) ==========
    
    # 4. 基座质量随机化 - 增加0到8公斤（使用缓存版本以保存采样值）
    randomize_base_mass = EventTerm(
        func=mdp.my_randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_BODY_NAME),
            "mass_distribution_params": (0.0, 8.0),
            "operation": "add",  # 添加操作
            "distribution": "uniform",
            "recompute_inertia": True,  # 重新计算惯性张量
        },
    )
    
    # 5. 基座质心随机化 - xyz方向各偏移±0.1米
    randomize_base_com = EventTerm(
        func=mdp.my_randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_BODY_NAME),
            "com_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
            },
        },
    )
    # ========== 间隔触发事件 (interval mode) ==========
    
    # 7. 推动机器人 - 每8秒施加一次随机速度
    push_robot = None


@configclass
class RewardsCfg:
    """Reward terms for the active no-arm locomotion task.

    该分支只保留 locomotion 与 base-force disturbance 相关奖励，以及
    12 个腿关节上的动作/力矩/速度/位置约束。其余 legacy manipulation
    reward surface 已停用。
    """

    # feet contact number 奖励足部接触数量
    feet_contact_number = RewTerm(
        func=mdp.feet_contact_number,
        weight=2.0,
        params={"sensor_cfg": SceneEntityCfg("contact_feet_forces")},
    )
    # tracking lin vel with force in world coordinates 追踪世界坐标系下的线速度（包含力的考量）
    tracking_lin_vel_force_world = RewTerm(
        func=mdp.tracking_lin_vel_force_world,
        weight=2.0,
        params={
            "locomotion_command_name": "locomotion_commands",
            "base_force_ext_name": "base_force_ext",
            "base_force_kd": BASE_FORCE_KD,
            "tracking_sigma": 0.25,
        },
    )
    # tracking angular velocity 追踪角速度
    tracking_ang_vel = RewTerm(
        func=mdp.track_ang_vel_z,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "std": 0.25,
        },
    )
    # torques 惩罚关节力矩
    torques = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-5.0e-6,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)}
    )
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)}
    )
    ref_dof_legs = RewTerm(
        func=mdp.ref_dof_leg,
        weight=1.0,
        params={
            "sigma": 0.1,
            "gait_command_name": "gait_phase",
            "target_joint_pos_thd": 0.1,
            "target_joint_pos_scale": 0.35,
            "calf_scale_factor": 2.0,
        }
    )
    alive = RewTerm(func=mdp.is_alive, weight=1.5)
    z_vel_penalty = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.5)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=1.0,
        params={
            "command_name": "locomotion_commands",
            "target_air_time": 0.25,
        },
    )
    feet_height = RewTerm(
        func=mdp.reward_feet_height,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "target_height": 0.08,
            "all_feet": True,
        }
    )
    xy_ang_vel = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.02)
    dof_acc_legs = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)}
    )
    dof_vel_legs = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-8.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)}
    )
    collision_penalty = RewTerm(
        func=mdp.undesired_contacts,
        weight=-5.0,
        params={"threshold": 1.0, "sensor_cfg": SceneEntityCfg("penalize_contacts")},
    )
    action_rate_legs = RewTerm(
        func=mdp.joint_processed_action_rate_l2,
        weight=-0.02,
        params={"action_term_name": "leg_actions"}
    )
    dof_pos_limit = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True)}
    )
    torque_limit = RewTerm(
        func=mdp.soft_torque_limits,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=list(LEG_JOINT_NAMES), preserve_order=True),
            "soft_limit": 0.9
        }
    )
    hip_pos = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg(
            "robot",
            joint_names=["FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint"],
            preserve_order=True,
        )}
    )
    feet_drag = RewTerm(
        func=mdp.feet_drag,
        weight=-0.0008,
        params={
            "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
            "asset_cfg": SceneEntityCfg("robot")
        }
    )
    feet_contact_forces = RewTerm(
        func=mdp.feet_contact_forces_l1,
        weight=-0.001,
        params={
            "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
            "max_contact_force": 240.0,
        }
    )
    base_height = RewTerm(
        func=mdp.base_height_l2,
        weight=-2.0,
        params={"target_height": 0.50}
    )
    feet_pos_xy = RewTerm(
        func=mdp.feet_pos_xy,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot")}
    )
    feet_height_high = RewTerm(
        func=mdp.feet_height_high,
        weight=-15.0,
        params={"asset_cfg": SceneEntityCfg("robot"), "height_threshold": 0.15}
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Illegal contacts on selected bodiesfe
    """illegal_contacts = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "threshold": 1.0,
            "sensor_cfg": SceneEntityCfg("terminate_after_contacts"),
        },
    )"""
    # (3) Base_height below minimum
    base_height = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "minimum_height": 0.20,
        },
    )
    # (4) Robot orientation out of bounds (roll/pitch limits)
    bad_orientation = DoneTerm(
        func=mdp.bad_orientation,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "limit_angle": 0.8,
        },
    )

##
# Environment configuration
##


@configclass
class UniFPAliengoEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: UniFPSceneCfg = UniFPSceneCfg(num_envs=4096, env_spacing=3.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # 课程学习设置
    curriculum: CurriculumCfg = CurriculumCfg()
    # Recorder settings
    # recorders: RecordersCfg = RecordersCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.sim.dt = 1 / 200
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -9.81)
        # self.sim.use_fabric = False  # 禁用NVIDIA Fabric (需要支持的硬件)
        
        # 开启 Stage in Memory 优化：直接在内存中创建舞台，加快启动速度
        #self.sim.create_stage_in_memory = True
        
        # 🔥 关键修复：启用contact reporting以获取接触力数据
        # Isaac Lab中ContactSensor需要物理引擎报告contact forces
        self.sim.physx.enable_scene_query_support = True  # 启用场景查询支持
        
        # 🚀 PhysX GPU性能优化配置（基于Isaac Lab官方最佳实践）
        # 核心GPU缓冲区配置（大幅提升）
        self.sim.physx.gpu_max_rigid_contact_count = 2097152        # ✅ 增加到2M接触对（原524K）
        self.sim.physx.gpu_max_rigid_patch_count = 655360           # ✅ 增加到640K补丁（原163K）
        self.sim.physx.gpu_found_lost_pairs_capacity = 32768        # ✅ 增加到32K（满足错误要求）
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 524288  # ✅ 增加到512K聚合对（原131K）
        
        # 高级GPU内存配置
        self.sim.physx.gpu_heap_capacity = 536870912               # ✅ 增加到512MB GPU堆（原128MB）
        self.sim.physx.gpu_temp_buffer_capacity = 134217728        # ✅ 增加到128MB临时缓冲区（原32MB）
        self.sim.physx.gpu_max_num_partitions = 32                 # ✅ 增加到32个分区（原16）
        
        # 核心求解器设置（GPU会自动启用）
        self.sim.physx.solver_type = 1                             # 🎯 TGS求解器（机器人优化）
        self.sim.physx.max_position_iteration_count = 4            # ✅ 增加位置迭代次数提升精度
        self.sim.physx.max_velocity_iteration_count = 0            # ✅ 增加速度迭代次数
