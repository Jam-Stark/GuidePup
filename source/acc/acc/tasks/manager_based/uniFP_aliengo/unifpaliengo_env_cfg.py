# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from typing import Dict, Tuple
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
from isaaclab.utils.noise import GaussianNoiseCfg

from . import mdp
from .mdp.recorders import FrontBackLegTorquesRecorderCfg
from isaaclab.envs.mdp.commands import UniformPoseCommand
from isaaclab.managers import CommandTermCfg, CurriculumTermCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.controllers import DifferentialIKControllerCfg

from isaaclab.sim import SimulationCfg, PhysxCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.converters import UrdfConverterCfg
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG
from isaaclab.markers import VisualizationMarkersCfg

from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
import isaaclab.terrains.height_field.hf_terrains_cfg as hf_gen_cfg

# 导入自定义命令生成器
from .commands import (
    DynamicManipulationCommandCfg,
    ExtendedUniformVelocityCommand,
    GaitPhaseCommandCfg,
    VariableForceCommandCfg,
    VirtualImpedanceCommand,
)
from .metrics import (
    ActionDistributionMetricsCommandCfg,
    FeetContactForceMetricsCommandCfg,
    LegTorqueMetricsCommandCfg,
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

        # 必需字段：指定命令生成器的类类型
        class_type = ExtendedUniformVelocityCommand
        
        # 必需字段：重采样时间范围（秒）
        resampling_time_range: Tuple[float, float] = (5.0, 5.0)
        heading_command: bool = False
        rel_standing_envs: float = 0.3

        ang_vel_yaw_clip: float = 0.2
        lin_vel_x_clip: float = 0.1
        lin_vel_y_clip: float = 0.1

        # 命令名称
        command_name: str = "locomotion_commands"

        # 命令维度: lin_vel_x, lin_vel_y, ang_vel_yaw, heading
        dim: int = 3

        @configclass
        class Ranges:
            """Ranges for the commands."""

            lin_vel_x: Tuple[float, float] = (-0.6, 0.6)
            lin_vel_y: Tuple[float, float] = (-0.4, 0.4)
            ang_vel_z: Tuple[float, float] = (-0.6, 0.6)
            heading: Tuple[float, float] = (-math.pi, math.pi)

        # 命令范围
        ranges: Ranges = Ranges()

        strategy: str = "uniform"
        asset_name: str = "robot"
        body_name: str = "base"
        
        # 启用可视化
        debug_vis: bool = True

        # 可视化配置 - 目标速度箭头 (红色)
        goal_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
            prim_path="/Visuals/Command/velocity_goal",
            markers={
                "arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.3, 0.3, 0.3),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
            },
        )
        # 可视化配置 - 当前速度箭头 (青色)
        current_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
            prim_path="/Visuals/Command/velocity_current",
            markers={
                "arrow": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.3, 0.3, 0.3),
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

    @configclass
    class TargetPositionCommandCfg(DynamicManipulationCommandCfg):
        """动态机械臂操作命令配置 - 球坐标采样 + 轨迹插值。
        
        采样规范：
        1. 球坐标: l∈[0.4,0.95]m, p∈[-π/2.5,π/3]rad, y∈[-1.2,1.2]rad
        2. 球心: 世界Z=0.7m, 机身XY=(0.3,0.0)m, 仅偏航旋转
        3. 姿态: 基础(π/2,0,0) + 增量±0.5rad
        4. 插值: 位置LERP(球坐标) + 姿态SLERP(四元数)
        5. 时间: 运动1-3s + 保持0.5-2s
        6. 碰撞检测: 采样10个轨迹点，检查禁飞区（机器人躯干）
        """
        
        # 资产配置
        asset_name: str = "robot"
        body_name: str = "gripperMover"
        
        # 球心配置
        sphere_center: Tuple[float, float] = (0.2535, 0.0)  # 机身坐标系XY
        sphere_center_z: float = 0.5  # 世界坐标系固定Z高度
        
        # 时间配置
        move_time_range: Tuple[float, float] = (1.5, 3.0)  # 运动时长
        hold_time_range: Tuple[float, float] = (0.5, 1.2)  # 保持时长
        
        max_collision_attempts: int = 20
        
        # 球坐标采样范围
        @configclass
        class Ranges:
            """球坐标采样范围"""
            radius: Tuple[float, float] = (0.4, 0.9)  # 径向距离 l (米)
            pitch_angle: Tuple[float, float] = (-1.0, 1.0)  # 俯仰角 p (弧度)
            yaw_angle: Tuple[float, float] = (-1.2, 1.2)  # 方位角 y (弧度)
            
        ranges: Ranges = Ranges()
        
        # 碰撞检测边界框 (相对于机器人根部的坐标系)
        @configclass
        class CollisionBox:
            """碰撞检测边界框 - 定义机械臂不应进入的区域（机器人躯干）
            
            坐标系：相对于机器人根部（base_link）
            - 原点：机器人基座中心 (Aliengo Trunk Center)
            - X轴：前方为正
            - Y轴：左侧为正
            - Z轴：上方为正
            
            根据 Aliengo + Z1 配置（基于URDF分析）：
            - 机械臂基座位置：X ≈ 0.25m, Z ≈ 0.06m
            - 躯干范围：X ∈ [-0.33, 0.33], Y ∈ [-0.08, 0.08], Z ∈ [-0.06, 0.06]
            
            设置边界：
            - X ∈ [-0.6, 0.15]: 覆盖机身及后腿，截止于机械臂基座后方10cm处
            - Y ∈ [-0.2, 0.2]:  左右各20cm，覆盖躯干及大腿根部
            - Z ∈ [-0.6, 0.05]: 从地面/脚底 (-0.6) 到 躯干顶部 (0.05)
            
            物理意义：防止机械臂碰撞机器人躯干、腿部及地面
            """
            x_min: float = -0.6   # 后方60cm（覆盖后腿区域）
            x_max: float = 0.15   # 前方15cm（机械臂基座在0.25，留出空间防止自碰）
            y_min: float = -0.2   # 左侧20cm（覆盖躯干宽度及部分腿部）
            y_max: float = 0.2    # 右侧20cm
            z_min: float = -0.4   # 下方40cm（Aliengo站高约40cm，此值覆盖地面）
            z_max: float = 0.05   # 上方5cm（覆盖躯干顶部，防止机械臂向下砸背）

        collision_box: CollisionBox = CollisionBox()
        
        # 可视化
        debug_vis: bool = True

    @configclass
    class GripperImpedanceCommandCfg(CommandTermCfg):
        """Configuration for virtual impedance parameters."""

        class_type = VirtualImpedanceCommand

        resampling_time_range: Tuple[float, float] = (9e9, 9e9)
        command_name: str = "gripper_impedance_command"
        dim: int = 3

        kp_range: Tuple[float, float] = (200.0, 200.0)
        kd_range: Tuple[float, float] = (3.0, 3.0)
        prop_kd: float = 0.1

    @configclass
    class BaseImpedanceCommandCfg(CommandTermCfg):
        """Configuration for virtual impedance parameters for the base."""

        class_type = VirtualImpedanceCommand

        resampling_time_range: Tuple[float, float] = (9e9, 9e9)
        command_name: str = "base_impedance_command"
        dim: int = 3

        kp_range: Tuple[float, float] = (200.0, 200.0)
        kd_range: Tuple[float, float] = (200.0, 200.0)
        prop_kd: float = 0.1

    locomotion_commands: LocomotionCommandCfg = LocomotionCommandCfg()
    gait_phase: GaitPhaseCfg = GaitPhaseCfg()
    target_position_commands: TargetPositionCommandCfg = TargetPositionCommandCfg()
    gripper_impedance_command: GripperImpedanceCommandCfg = GripperImpedanceCommandCfg()
    base_impedance_command: BaseImpedanceCommandCfg = BaseImpedanceCommandCfg()
    leg_torque_metrics: LegTorqueMetricsCommandCfg = LegTorqueMetricsCommandCfg()
    feet_contact_force_metrics: FeetContactForceMetricsCommandCfg = FeetContactForceMetricsCommandCfg()
    action_distribution_metrics: ActionDistributionMetricsCommandCfg = ActionDistributionMetricsCommandCfg()

    @configclass
    class GripperForceExtCfg(VariableForceCommandCfg):
        force_range: Tuple[float, float] = (-50.0, 50.0)
        push_interval_range_s: Tuple[float, float] | None = (3.5, 9.0)
        duration_range_s: Tuple[float, float] | None = (1.0, 3.0)
        settling_time_s: float | None = 1.0
        prob_freed: float = 0.2
        is_global: bool = True

    @configclass
    class BaseForceExtCfg(VariableForceCommandCfg):
        force_range: Tuple[float, float] = (-30.0, 30.0)
        push_interval_range_s: Tuple[float, float] | None = (3.5, 9.0)
        duration_range_s: Tuple[float, float] | None = (1.0, 3.0)
        settling_time_s: float | None = 3.0
        prob_freed: float = 0.2
        is_global: bool = True
    
    @configclass
    class GripperForceCommandCfg(VariableForceCommandCfg):
        force_range: Tuple[float, float] = (-50.0, 50.0)
        push_interval_range_s: Tuple[float, float] | None = (3.5, 9.0)
        duration_range_s: Tuple[float, float] | None = (1.0, 3.0)
        settling_time_s: float | None = 1.0
        prob_freed: float = 0.2
        is_global: bool = False
    
    @configclass
    class BaseForceCommandCfg(VariableForceCommandCfg):
        force_range: Tuple[float, float] = (-30.0, 30.0)
        push_interval_range_s: Tuple[float, float] | None = (3.5, 9.0)
        duration_range_s: Tuple[float, float] | None = (1.0, 3.0)
        settling_time_s: float | None = 3.0
        prob_freed: float = 0.2
        is_global: bool = False

    gripper_force_ext: GripperForceExtCfg = GripperForceExtCfg()
    base_force_ext: BaseForceExtCfg = BaseForceExtCfg()
    gripper_force_command: GripperForceCommandCfg = GripperForceCommandCfg()
    base_force_command: BaseForceCommandCfg = BaseForceCommandCfg()


##
# Scene definition
##


@configclass
class UniFPTerrainCfg(TerrainGeneratorCfg):
    """
    VBC专用地形配置 - 实现"rough flat"粗糙平地
    
    按照VBC原始配置要求：
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
            noise_range=(0.00, 0.05),  # 高度噪声范围 (米)
            noise_step=0.005,
            border_width=0.0,
        )
    }


@configclass
class UniFPSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""
    # 关键：禁用物理属性复制，以支持域随机化
    replicate_physics = False
    # 开启 Fabric 优化：使用 USD Fabric 进行环境克隆，提高实例化速度
    # 注意：开启此选项可能会导致 Articulation 随机化（如质量）出错，因此暂时关闭
    clone_in_fabric = False

    """terrain = TerrainImporterCfg(
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
    )"""
    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,  # 明确定义静摩擦
                dynamic_friction=1.0,  # 明确定义动摩擦
                restitution=0.0       # 明确定义恢复系数
            )
        )
    )

    # 机器人资产配置
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="resources/aliengoZ1/urdf/aliengoZ1.urdf",
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
            
            # 关节根属性配置 - 按照VBC配置
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,   # 启用自碰撞（与self_collision配合使用）
                solver_position_iteration_count=16,  # 提高求解精度以改善碰撞检测
                solver_velocity_iteration_count=8,   # 提高速度求解精度
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
                articulation_enabled=True,
            ),
            
            # 关键：为 URDF 导入定义关节驱动，并关闭目标
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                drive_type="force",          # VBC: DOF_MODE_EFFORT
                target_type="none",          # 关掉驱动，避免与 actuators 叠加
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=0.0,
                    damping=0.0,
                ),
            ),
            
        ),
        
        # 初始状态配置
        init_state=ArticulationCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.5],
            rot=[1.0, 0.0, 0.0, 0.0],
            joint_pos={
                "FL_hip_joint": 0.1,  # [rad]
                "FL_thigh_joint": 0.5,  # [rad]
                "FL_calf_joint": -1.0,  # [rad]

                "RL_hip_joint": 0.1,  # [rad]
                "RL_thigh_joint": 0.5,  # [rad]
                "RL_calf_joint": -1.0,  # [rad]

                "FR_hip_joint": -0.1,  # [rad]
                "FR_thigh_joint": 0.5,  # [rad]
                "FR_calf_joint": -1.0,  # [rad]

                "RR_hip_joint": -0.1,  # [rad]
                "RR_thigh_joint": 0.5,  # [rad]
                "RR_calf_joint": -1.0,  # [rad]

                "arm_joint1": 0.0,  # 原 z1_waist
                "arm_joint2": 1.48,  # 原 z1_shoulder (抬起)
                "arm_joint3": -0.63,  # 原 z1_elbow (折叠)
                "arm_joint4": -0.84,  # 原 z1_wrist_angle
                "arm_joint5": 0.0,  # 原 z1_forearm_roll
                "arm_joint6": 1.57,  # 原 z1_wrist_rotate (90度，利于向下抓取)
                # "gripperMover": -0.785,  # 原 z1_gripperMover (张开/闭合状态)
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
                effort_limit_sim=35.0,
                velocity_limit_sim=20.0,
                stiffness=80.0,
                damping=2.5,
                armature=0.0,
            ),
            "thighs": ImplicitActuatorCfg(
                joint_names_expr=[
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",  # thigh joints
                ],
                effort_limit_sim=35.0,
                velocity_limit_sim=20.0,
                stiffness=80.0,
                damping=2.5,
                armature=0.0,
            ),
            "calfs": ImplicitActuatorCfg(
                joint_names_expr=[
                    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",      # calf joints
                ],
                # Aliengo URDF: Calf max 44.4 Nm, others 35.3 Nm
                effort_limit_sim=45.0,
                # Aliengo URDF: Hip/Thigh max 20 rad/s, Calf 15.9 rad/s
                velocity_limit_sim=15.0,
                stiffness=80.0,
                damping=2.0,
                armature=0.0,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[
                    "arm_joint1", "arm_joint3",
                    "arm_joint4", "arm_joint5",
                ],
                # Z1 URDF: Joint2 max 60 Nm, others 30 Nm
                effort_limit_sim=30.0,
                # Z1 URDF: All joints max 3.1415 rad/s
                velocity_limit_sim=3.15,
                stiffness=50.0,
                damping=2.5,
                armature=0.0,
            ),
            "arm_shoulder": ImplicitActuatorCfg(
                joint_names_expr=[
                    "arm_joint2",
                ],
                # Z1 URDF: Joint2 max 60 Nm, others 30 Nm
                effort_limit_sim=60.0,
                # Z1 URDF: All joints max 3.1415 rad/s
                velocity_limit_sim=3.15,
                stiffness=60.0,
                damping=3.5,
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
    
    # 非法接触检测 penalize_contacts_on = ["thigh", "calf","trunk"]
    penalize_contacts: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*(_thigh|_calf|trunk)",
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

    leg_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ],
        scale=0.25,
        use_default_offset=True,
        clip={".*": (-100.0, 100.0)}
    )
    arm_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"],
        scale=0.25,
        use_default_offset=True,
        clip={".*": (-100.0, 100.0)}
    )

    gripper_force_ext = mdp.ApplyForceCommandActionCfg(
        asset_name="robot",
        body_name="gripperMover",
        command_name="gripper_force_ext",
    )
    base_force_ext = mdp.ApplyForceCommandActionCfg(
        asset_name="robot",
        body_name="base",
        command_name="base_force_ext",
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group.
        本体感知 (Proprioception):

            get_body_orientation() (通常是重力向量或欧拉角，维度 2)

            base_ang_vel (IMU 陀螺仪数据)

            dof_pos & dof_vel (电机编码器数据)

        动作历史:

            actions (上一时刻的输出，用于让网络学会平滑控制)

        任务指令:

            sin_pos, cos_pos (步态时钟)

            commands (包括期望速度、期望末端位置等)

        噪声处理 (Noise Injection):

            代码包含 if self.add_noise: 逻辑。

        历史堆叠 (Frame Stacking):

            obs_buf_all = torch.stack(..., dim=1)
        """

        # 1. 机身方向 (2维) - 提取projected_gravity的x,y分量获取roll和pitch信息
        base_orientation = ObsTerm(
            func=mdp.base_orientation_2d,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=GaussianNoiseCfg(mean=0.0, std=0.05, operation="add"),
            clip=(-100.0, 100.0)
        )
        # 2,base_ang_vel,基座角速度,3
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=GaussianNoiseCfg(mean=0.0, std=0.2, operation="add"),
            scale=0.25,
            clip=(-100.0, 100.0)
        )
        # 3. 关节位置 (17维) - 当前位置相对默认位置的偏移，排除夹爪关节
        dof_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                "arm_joint3", "arm_joint4", "arm_joint5"
            ])},
            noise=GaussianNoiseCfg(mean=0.0, std=0.01, operation="add"),
            scale=1.0,
            clip=(-100.0, 100.0)
        )
        
        # 4. 关节速度 (17维) - 排除夹爪关节
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                    "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                    "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                    "arm_joint3", "arm_joint4", "arm_joint5"
                ])},
            noise=GaussianNoiseCfg(mean=0.0, std=1.5, operation="add"),
            scale=0.05,
            clip=(-100.0, 100.0)
        )

        # 5. 上一时刻的动作 (17维)
        last_action = ObsTerm(func=mdp.last_action,
                              clip=(-100.0, 100.0))
        # 6. 步态时钟信号 (2维) - 全局时钟正弦余弦
        gait_clock = ObsTerm(
            func=mdp.gait_clock_signal,
            clip=(-100.0, 100.0)
        )
        # 7. 指令 (15维) - 给定的机器人运动指令
        """commands:
            Base Velocity Commands) - [3维]
                Index 0: Base Linear Velocity X
                Index 1: Base Linear Velocity Y
                Index 2: Base Angular Velocity Yaw
            End-Effector Position Commands - Spherical - [3维]
                Index 3: EE Spherical Radius
                Index 4: EE Spherical Pitch
                Index 5: EE Spherical Yaw
            End-Effector Orientation Commands - [3维]
                Index 6: EE Roll
                Index 7: EE Pitch
                Index 8: EE Yaw
            End-Effector Force Commands - [3维]
                Index 9: EE Force X
                Index 10: EE Force Y
                Index 11: EE Force Z
            Base Force Commands - [3维]
                Index 12: Base Force X
                Index 13: Base Force Y
                Index 14: Base Force Z
        """
        commands = ObsTerm(
            func=mdp.all_commands_with_scales,
            params={"scales": (
                2.0, 2.0, 0.25,  # Base Velocity
                0.5, 1.0, 1.3,   # EE Sphere Pos
                0.5, 0.5, 0.5,   # EE Orientation
                0.01, 0.01, 0.01,  # EE Force
                0.01, 0.01, 0.01  # Base Force
            )},
            clip=(-100.0, 100.0)
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 32
            self.flatten_history_dim = True
    
    @configclass
    class privilegedObs(ObsGroup):
        """
        privilieged观测（total 80维）：
            索引/切片,变量名,物理含义,维度估算,作用
            1,base_lin_vel,基座线速度,3,核心状态
            2,ee_pos_sphe_arm,末端执行器球坐标位置,3,操作任务目标
            3,forces_local (Gripper),夹爪受力,3,感知交互力
            4,forces_local (Base),基座受力,3,感知碰撞/负载
            5,diff,关节位置误差 (Ref - Curr),12 (4腿x3),跟踪性能
            6,mass_params_tensor,物理参数：质量,22,域随机化信息
            7,friction_coeffs_tensor,物理参数：摩擦系数,1,域随机化信息
            8,motor_strength,物理参数：电机强度,17,域随机化信息
            9,stance_mask,步态计划（该哪条腿着地）,4,步态时序
            10,contact_mask,真实接触状态,4,接触反馈
            11,projected_gravity,重力在基座坐标系投影,3,姿态感知
            12,base_ang_vel,基座角速度,3,核心状态
            13,dof_pos (error),关节位置（减去默认位）,17,本体感知
            14,dof_vel,关节速度,17,本体感知
            15,actions,上一帧动作,17,动作平滑/延迟
            16,sin/cos_pos,步态相位时钟,2,周期信号
            17,commands,用户指令,15,任务目标
            18,ee_goal_offset...,末端目标(增加力作用下的offset),3,操作误差
        """

        # 1. 基座线速度 (3维)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            scale=2.0,
            clip=(-100.0, 100.0)
        )
        
        # 2. 机械臂末端执行器球坐标位置 (3维)   base坐标系下
        ee_goal_pos = ObsTerm(
            func=mdp.ee_pos_in_sphere,
            scale=(0.5, 1.0, 1.3),
            clip=(-100.0, 100.0)
        )

        # 3. 夹爪受力 (3维)
        gripper_forces_local = ObsTerm(
            func=mdp.gripper_forces_local,
            params={"command_name": "gripper_force_ext"},
            scale=0.01,
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
        # 13. 关节位置 (17维) - 当前位置相对默认位置的偏移
        dof_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                "arm_joint3", "arm_joint4", "arm_joint5"
            ])},
            scale=1.0,
            clip=(-100.0, 100.0)
        )
        
        # 14. 关节速度 (17维) - 排除夹爪关节
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                    "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                    "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                    "arm_joint3", "arm_joint4", "arm_joint5"
                ])},
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

        # 17. 指令 (15维) - 给定的机器人运动指令
        commands = ObsTerm(
            func=mdp.all_commands_with_scales,
            params={"scales": (
                2.0, 2.0, 0.25,  # Base Velocity
                0.5, 1.0, 1.3,   # EE goal Sphere Pos
                0.5, 0.5, 0.5,   # EE goal Orientation
                0.01, 0.01, 0.01,  # EE Force
                0.01, 0.01, 0.01  # Base Force
            )},
            clip=(-100.0, 100.0)
        )

        # 18.   末端目标 with force offset (3维) - 机械臂末端执行器目标位置与实际位置的偏差
        ee_goal_offset = ObsTerm(
            func=mdp.ee_goal_offset_local_sphere,
            params={"ee_goal_command_name": "target_position_commands",
                    "gripper_force_ext_name": "gripper_force_ext",
                    "gripper_force_command_name": "gripper_force_command",
                    "gripper_impedance_command_name": "gripper_impedance_command"},
            scale=(0.5, 1.0, 1.3),
            clip=(-100.0, 100.0)
        )
        
        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 3
            self.flatten_history_dim = True
    
    @configclass
    class predObs(ObsGroup):
        """
        base_lin_vel,基座线速度,3,核心状态
        ee_pos_sphe_arm,末端执行器球坐标位置,3,操作
        forces_local (Gripper),夹爪受力,3,感知交互力
        forces_local (Base),基座受力,3,感知碰撞/负
        """
        # 1. 基座线速度 (3维)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            scale=2.0,
            clip=(-100.0, 100.0)
        )
        
        # 2. 机械臂末端执行器球坐标位置 (3维)   base坐标系下
        ee_goal_pos = ObsTerm(
            func=mdp.ee_pos_in_sphere,
            scale=(0.5, 1.0, 1.3),
            clip=(-100.0, 100.0)
        )

        # 3. 夹爪受力 (3维)
        gripper_forces_local = ObsTerm(
            func=mdp.gripper_forces_local,
            params={"command_name": "gripper_force_ext"},
            scale=0.01,
            clip=(-100.0, 100.0)
        )
        # 4. 基座受力 (3维)
        base_forces_local = ObsTerm(
            func=mdp.base_forces_local,
            params={"command_name": "base_force_ext"},
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


def modify_force_range(env, env_ids, old_value, zero_range, full_range, resume_steps=0):
    """Clamp force command range to zero until the curriculum threshold."""
    start_steps = 8000 * 24
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

    # First force_start_iterations iterations: disable force cmd/ext, then restore ranges.
    gripper_force_ext_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.gripper_force_ext.force_range",
            "modify_fn": modify_force_range,
            "modify_params": {
                "zero_range": (0.0, 0.0),
                "full_range": (-50.0, 50.0),
                "resume_steps": 0,
            },
        },
    )

    base_force_ext_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.base_force_ext.force_range",
            "modify_fn": modify_force_range,
            "modify_params": {
                "zero_range": (0.0, 0.0),
                "full_range": (-30.0, 30.0),
                "resume_steps": 0,
            },
        },
    )

    gripper_force_command_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.gripper_force_command.force_range",
            "modify_fn": modify_force_range,
            "modify_params": {
                "zero_range": (0.0, 0.0),
                "full_range": (-50.0, 50.0),
                "resume_steps": 0,
            },
        },
    )

    base_force_command_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.base_force_command.force_range",
            "modify_fn": modify_force_range,
            "modify_params": {
                "zero_range": (0.0, 0.0),
                "full_range": (-30.0, 30.0),
                "resume_steps": 0,
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
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"
            ]),
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
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
            ]),
            "stiffness_distribution_params": (0.7, 1.3),  # 缩放因子范围
            "damping_distribution_params": (0.7, 1.3),
            "operation": "scale",  # 缩放操作
            "distribution": "uniform",  # 均匀分布
        },
    )

    
    # 3. 机械臂关节刚度和阻尼随机化
    randomize_arm_joint_gains = EventTerm(
        func=mdp.my_randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"
            ]),
            "stiffness_distribution_params": (0.7, 1.3),
            "damping_distribution_params": (0.7, 1.3),
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    
    # ========== 启动时随机化事件 (startup mode) ==========
    
    # 4. 基座质量随机化 - 增加0到10公斤（使用缓存版本以保存采样值）
    randomize_base_mass = EventTerm(
        func=mdp.my_randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (0.0, 5.0),
            "operation": "add",  # 添加操作
            "distribution": "uniform",
            "recompute_inertia": True,  # 重新计算惯性张量
        },
    )
    
    # 5. 基座质心随机化 - xyz方向各偏移±0.15米
    randomize_base_com = EventTerm(
        func=mdp.my_randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "com_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
            },
        },
    )
    
    # 6. 夹爪质量随机化 - 增加0到0.1公斤（使用缓存版本以保存采样值）
    randomize_gripper_mass = EventTerm(
        func=mdp.my_randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
            "mass_distribution_params": (0.0, 0.1),
            "operation": "add",
            "distribution": "uniform",
            "recompute_inertia": True,  # 重新计算惯性张量
        },
    )
    
    # ========== 间隔触发事件 (interval mode) ==========
    
    # 7. 推动机器人 - 每8秒施加一次随机速度
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity_standInclude,
        mode="interval",
        interval_range_s=(8.0, 8.0),  # 每8秒触发一次
        params={
            "velocity_range": {
                "x": (-0.5, 0.5),  # x方向速度范围 (m/s)
                "y": (-0.5, 0.5),  # y方向速度范围 (m/s)
            },
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "standing_velocity_scale": 1.5,
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP.
    Reward 名称,权重 (Scale),说明 (推测)
        feet_contact_number,2.0,奖励足部接触数量（可能用于鼓励特定的支撑模式）
        tracking_lin_vel_force_world,2.0,追踪世界坐标系下的线速度（包含力的考量）
        tracking_ang_vel,1.0,追踪角速度
        torques,-5.e-6,惩罚关节力矩（减少能耗/平滑控制）
        stand_still,0.5,奖励静止站立（当指令为0时）
        ref_dof_leg,1.0,保持腿部关节在参考位置
        alive,1.5,存活奖励（只要机器人不倒下就给予奖励）
        lin_vel_z,-1.5,惩罚 Z 轴线速度（减少上下跳动，保持平稳）
        feet_air_time,1.0,奖励足部滞空时间（鼓励迈步）
        feet_height,1.0,奖励抬脚高度（避免拖地）
        ang_vel_xy,-0.02,惩罚 XY 平面的角速度（保持躯干水平稳定）
        dof_acc,-2.5e-7,惩罚关节加速度（平滑运动）
        dof_vel,-8.e-4,惩罚关节速度（减少剧烈运动）
        dof_acc_arm,-4.5e-7,惩罚机械臂关节加速度
        dof_vel_arm,-2.e-4,惩罚机械臂关节速度
        collision,-5.,碰撞惩罚（如膝盖撞地等）
        action_rate,-0.02,惩罚动作变化率（鼓励动作平滑）
        action_rate_arm,-0.045,惩罚机械臂动作变化率
        dof_pos_limits,-10.0,惩罚关节超出位置限制
        torque_limits,-0.005,惩罚力矩超出限制
        hip_pos,-0.5,惩罚髋关节位置偏差
        feet_drag,-0.0008,惩罚拖脚行为
        feet_contact_forces,-0.001,惩罚过大的足部接触力
        base_height,-2.0,惩罚基座高度偏差（保持目标高度）
        feet_pos_xy,-0.5,惩罚足部水平位置偏差
        feet_height_high,-15,惩罚抬脚过高
        tracking_ee_force_world,2.0,追踪末端执行器（EE）在世界坐标系下的力
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
            "base_force_cmd_name": "base_force_command",
            "base_impedance_command_name": "base_impedance_command",
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
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint",
            "RR_hip_joint", "FL_thigh_joint", "FR_thigh_joint",
            "RL_thigh_joint", "RR_thigh_joint", "FL_calf_joint",
            "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
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
    feet_air_time = RewTerm(func=mdp.feet_air_time, weight=1.0, params={"command_name": "locomotion_commands"})
    feet_height = RewTerm(
        func=mdp.reward_feet_height,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "target_height": 0.08,
            "all_feet": False,
        }
    )
    xy_ang_vel = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.02)
    dof_acc_legs = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ])}
    )
    dof_vel_legs = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-8.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ])}
    )
    dof_acc_arm = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-4.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5"
        ])}
    )
    dof_vel_arm = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-2.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5"
        ])}
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
    action_rate_arm = RewTerm(
        func=mdp.joint_processed_action_rate_l2,
        weight=-0.045,
        params={"action_term_name": "arm_actions"}
    )
    dof_pos_limit = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    torque_limit = RewTerm(
        func=mdp.soft_torque_limits,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5",
        ]),
            "soft_limit": 0.9
        }
    )
    hip_pos = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint"
        ])}
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
            "max_contact_force": 150.0,
        }
    )
    base_height = RewTerm(
        func=mdp.base_height_l2,
        weight=-2.0,
        params={"target_height": 0.40}
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
    tracking_ee_force_world = RewTerm(
        func=mdp.tracking_ee_force_world,
        weight=2.0,
        params={
            "command_name": "target_position_commands",
            "gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
            "force_command_name": "gripper_force_ext",
            "gripper_force_command_name": "gripper_force_command",
            "gripper_impedance_command_name": "gripper_impedance_command",
            "tracking_ee_sigma": 1.0,
        },
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
            "minimum_height": 0.05,
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
    scene: UniFPSceneCfg = UniFPSceneCfg(num_envs=4096, env_spacing=4.0)
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
        self.sim.physx.max_position_iteration_count = 8            # ✅ 增加位置迭代次数提升精度
        self.sim.physx.max_velocity_iteration_count = 4            # ✅ 增加速度迭代次数
