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
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp
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

from .observation_compute import (
    ee_goal_pos_in_base_frame,
    ee_tracking_error_in_base_frame,
    base_orientation_2d,
    foot_contact_binary,
    ee_goal_orient_placeholder,
    privileged_mass_params,
    privileged_friction,
    privileged_motor_strengths,
    joint_pos_rel_scaled,
    joint_vel_scaled,
)

# 导入自定义命令生成器
from .commands import DynamicManipulationCommandCfg, ExtendedUniformVelocityCommand
##
# Pre-defined configs
##
@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    @configclass
    class LocomotionCommandCfg(CommandTermCfg):
        """Configuration for base locomotion commands."""

        # 必需字段：指定命令生成器的类类型 - 使用扩展版本支持站立时角速度也为0
        class_type = ExtendedUniformVelocityCommand
        
        # 必需字段：重采样时间范围（秒）
        resampling_time_range: Tuple[float, float] = (10.0, 10.0)
        heading_command: bool = False
        rel_standing_envs: float = 0.2

        # 命令名称
        command_name: str = "locomotion_commands"

        # 命令维度: lin_vel_x, lin_vel_y, ang_vel_yaw, heading
        dim: int = 3

        @configclass
        class Ranges:
            """Ranges for the commands."""

            lin_vel_x: Tuple[float, float] = (0.0, 0.8)
            lin_vel_y: Tuple[float, float] = (0.0, 0.0)
            ang_vel_z: Tuple[float, float] = (-1.0, 1.0)
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
        sphere_center_z: float = 0.6  # 世界坐标系固定Z高度
        
        # 时间配置
        move_time_range: Tuple[float, float] = (1.0, 2.0)  # 运动时长
        hold_time_range: Tuple[float, float] = (0.5, 1.2)  # 保持时长
        
        max_collision_attempts: int = 20
        
        # 球坐标采样范围
        @configclass
        class Ranges:
            """球坐标采样范围"""
            radius: Tuple[float, float] = (0.4, 0.7)  # 径向距离 l (米)
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

    locomotion_commands: LocomotionCommandCfg = LocomotionCommandCfg()
    target_position_commands: TargetPositionCommandCfg = TargetPositionCommandCfg()


##
# Scene definition
##


@configclass
class VbcTerrainCfg(TerrainGeneratorCfg):
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
class VbcSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""
    # 关键：禁用物理属性复制，以支持域随机化
    replicate_physics = False
    # 开启 Fabric 优化：使用 USD Fabric 进行环境克隆，提高实例化速度
    # 注意：开启此选项可能会导致 Articulation 随机化（如质量）出错，因此暂时关闭
    clone_in_fabric = False

    """terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=VbcTerrainCfg(),
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

    # 机器人资产配置 - 按照VBC标准配置，排除夹爪关节
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="resources/aliengoZ1/urdf/aliengoZ1.urdf",
            fix_base=False,
            activate_contact_sensors=True,  # 启用接触传感器
            merge_fixed_joints=False,  # 保留足部等固定关节链接
            self_collision=True,
            
            # 刚体物理属性配置 - 按照VBC asset配置
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
            'FL_hip_joint': 0.1,    # [rad]
            'FL_thigh_joint': 0.8,  # [rad]
            'FL_calf_joint': -1.5,  # [rad]

            'RL_hip_joint': 0.1,    # [rad]
            'RL_thigh_joint': 0.8,  # [rad]
            'RL_calf_joint': -1.5,  # [rad]

            'FR_hip_joint': -0.1,   # [rad]
            'FR_thigh_joint': 0.8,  # [rad]
            'FR_calf_joint': -1.5,  # [rad]

            'RR_hip_joint': -0.1,   # [rad]
            'RR_thigh_joint': 0.8,  # [rad]
            'RR_calf_joint': -1.5,  # [rad]
            
            'arm_joint1': 0.0,      # 原 z1_waist
            'arm_joint2': 1.48,     # 原 z1_shoulder (抬起)
            'arm_joint3': -0.63,    # 原 z1_elbow (折叠)
            'arm_joint4': -0.84,    # 原 z1_wrist_angle
            'arm_joint5': 0.0,      # 原 z1_forearm_roll
            'arm_joint6': 1.57,     # 原 z1_wrist_rotate (90度，利于向下抓取)
            #  'gripperMover': -0.785, # 原 z1_gripperMover (张开/闭合状态)
        }
        ),
        
        # 软关节限制因子
        soft_joint_pos_limit_factor=1.0,
        # 执行器配置
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[
                    # 按照实际机器人关节顺序排列
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",          # hip joints
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",  # thigh joints
                    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",      # calf joints
                ],
                # Aliengo URDF: Calf max 44.4 Nm, others 35.3 Nm
                effort_limit_sim=45.0,        # 修改：适配 Aliengo 的真实力矩限制
                # Aliengo URDF: Hip/Thigh max 20 rad/s, Calf 15.9 rad/s
                velocity_limit_sim=20.0,      # 修改：适配 Aliengo 的真实速度限制
                # 建议刚度：Aliengo 较轻，建议 40.0 左右 (原 B1 为 80.0)
                stiffness=40.0,
                damping=1.0,
                armature=0.0,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[
                    "arm_joint1", "arm_joint2", "arm_joint3",
                    "arm_joint4", "arm_joint5", "arm_joint6",
                ],
                # Z1 URDF: Joint2 max 60 Nm, others 30 Nm
                effort_limit_sim=60.0,        # 修改：Z1 机械臂最大力矩
                # Z1 URDF: All joints max 3.1415 rad/s
                velocity_limit_sim=3.15,      # 修改：Z1 机械臂最大速度
                # 机械臂位置控制通常需要较高刚度，400.0 可能是为了高精度跟踪，
                # 但如果发生抖动，可尝试降低至 200.0 或 100.0
                stiffness=400.0,
                damping=40.0,  
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
    
    # 非法接触检测 penalize_contacts_on = ["thigh", "calf"]
    penalize_contacts: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*_(thigh|calf)",
        update_period=0.0,
        force_threshold=1.0,
        history_length=1,
        debug_vis=False,
    )
    
    # 非法接触检测 terminate_after_contacts_on = ["hip"]
    terminate_after_contacts: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*_(hip)",
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

    # 腿部动作空间 - 12维, 使用关节位置控制
    leg_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ],
        scale=0.25,
        use_default_offset=True,
    )

    # 机械臂动作空间 - 使用从 Command 读取目标的 IK 控制
    #
    # 关键区别:
    # 1. 使用 CommandBasedIKActionCfg (自定义) 而非 DifferentialInverseKinematicsActionCfg
    # 2. 动作维度 = 0 (不需要策略网络输出机械臂目标)
    # 3. 目标来源: commands.target_position_commands (7维: pos+quat)
    # 4. command_type="position" 表示只使用位置目标 (忽略姿态)
    #
    # 浮动基座 Jacobian 处理 (自动):
    # - Jacobian 列索引: [0-5: 基座, 6-17: 腿部, 18-23: 机械臂]
    # - joint1-6 自动映射到 Jacobian[18-23]列 (内部自动 +6)
    arm_actions = mdp.CommandBasedIKActionCfg(
        asset_name="robot",
        body_name="gripperMover",
        joint_names=["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"],
        command_name="target_position_commands",
        controller=DifferentialIKControllerCfg(
            command_type="pose",  # "position" 只控制位置, "pose" 控制位置+姿态
            ik_method="dls",
            use_relative_mode=False,
        )
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group.
        
        基础观测 (70维总计):
        - 机身方向 (Body Orientation) (维度: 2): 机身的 roll 和 pitch 角
        - 机身角速度 (Base Angular Velocity) (维度: 3): 机身在本地坐标系下的 x, y, z 轴角速度
        - 关节位置 (DOF Positions) (维度: 18): 所有18个关节相对默认位置的差值
        - 关节速度 (DOF Velocities) (维度: 18): 所有18个关节的当前速度
        - 上一时刻的动作 (Last Actions) (维度: 12): 腿部动作历史
        - 足端接触状态 (Foot Contacts) (维度: 4): 四个足端接触布尔值
        - 指令 (Commands) (维度: 3): 运动指令 (lin_vel_x, lin_vel_y, ang_vel_yaw)
        - 末端执行器目标位置 (End-Effector Goal Position) (维度: 3): 机身本地坐标系下的目标位置
        - 末端执行器目标姿态 (维度: 3): 硬编码为0的占位符
        - 步态时钟信号 (Gait Clock) (维度: 4): 每只脚的相位正弦波 (2Hz Trotting)
        """

        # 1. 机身方向 (2维) - 提取projected_gravity的x,y分量获取roll和pitch信息
        base_orientation = ObsTerm(
            func=base_orientation_2d,
            params={"asset_cfg": SceneEntityCfg("robot")}
        )

        # 2. 机身角速度 (3维) - 机身在本地坐标系下的角速度
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("robot")}
        )

        # 3. 关节位置 (18维) - 当前位置相对默认位置的偏移，排除夹爪关节
        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
            ])}
        )

        # 4. 关节速度 (18维) - 排除夹爪关节
        joint_vel = ObsTerm(
            func=joint_vel_scaled,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                    "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint",
                    "RL_calf_joint", "RR_calf_joint", "arm_joint1", "arm_joint2",
                    "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
                ]),
                "scale": 0.05  # 速度通常需要更小的缩放系数，例如 0.05
            }
        )

        # 5. 上一时刻的动作 (12维) - 腿部动作历史
        last_action = ObsTerm(func=mdp.last_action)

        # 6. 足端接触状态 (4维) - 四个足端的接触状态
        foot_contacts = ObsTerm(
            func=foot_contact_binary,
            params={
                "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
                "threshold": 1.5
            }
        )

        # 7. 指令 (3维) - 给定的机器人运动指令
        commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "locomotion_commands"}
        )

        # 8. 末端执行器目标位置 (3维) - 在机身本地坐标系下
        ee_goal_pos = ObsTerm(
            func=ee_goal_pos_in_base_frame,
            params={"command_name": "target_position_commands"}
        )

        # 9. 末端执行器目标姿态 (3维) - 硬编码为0的占位符
        ee_goal_orient = ObsTerm(func=ee_goal_orient_placeholder)

        # 10. 步态时钟信号 (4维) - 每只脚的相位正弦波
        # 用于引导机器人学习稳定的对角小跑步态
        # 频率: 2 Hz, 步态模式: Trotting (对角小跑)
        # 站立时所有足部时钟固定为支撑相 (-1)，行走时动态变化
        gait_clock = ObsTerm(
            func=mdp.gait_clock_signal,
            params={
                "command_name": "locomotion_commands",  # 用于判断站立/行走状态
                "gait_frequency": 2.0,  # 步态频率 (Hz)
                "phase_offset": 0.5,    # 对角足相位偏移 (180度)
            }
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True
    
    @configclass
    class privilegedObs(ObsGroup):
        """
        privilieged观测 (18维):
            - random_mass_params_tensor(5): [base_mass_delta, com_x, com_y,
              com_z, gripper_mass_delta]
            - random_friction(1): [static_friction]
            - random_motor_strengths(12): [腿部12个关节的电机强度]
              FL/FR/RL/RR每条腿3个关节
        """
        
        # 1. 随机化质量参数 (5维)
        mass_params = ObsTerm(
            func=privileged_mass_params,
        )
        
        # 2. 随机化摩擦系数 (1维)
        friction = ObsTerm(
            func=privileged_friction,
        )
        
        # 3. 随机化电机强度 (12维)
        motor_strengths = ObsTerm(
            func=privileged_motor_strengths,
        )
        
        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    privileged: privilegedObs = privilegedObs()


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


@configclass
class CurriculumCfg:
    """Curriculum learning configuration.
    
    课程学习配置：在训练120000步后，将x方向速度范围从(0.0, 0.8)扩展到(-0.8, 0.8)，
    使机器人学习后退能力。
    """
    
    # 课程学习：120000步后扩展x方向速度范围从(0.0, 0.8)到(-0.8, 0.8)
    lin_vel_x_curriculum = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.locomotion_commands.ranges.lin_vel_x",
            "modify_fn": modify_lin_vel_x_range,
            "modify_params": {
                "new_value": (-0.8, 0.8),
                "num_steps": 100_000,
            }
        }
    )


@configclass
class EventCfg:
    """Configuration for events."""

    # ========== 重置事件 (reset mode) ==========
    
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
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
            ]),
        },
    )
    
    # ========== 域随机化事件 (reset mode) ==========
    
    # 1. 物理材质随机化 - 静摩擦和动摩擦
    randomize_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 3.0),
            "dynamic_friction_range": (0.3, 3.0),
            "restitution_range": (0.0, 0.0),  # 恢复系数保持为0
            "num_buckets": 1000,  # 离散化桶数，提高采样效率
        },
    )
    
    # 1.1 缓存物理材质随机化参数（用于 privileged observations）
    cache_physics_material = EventTerm(
        func=mdp.cache_friction_randomization,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 3.0),
            "dynamic_friction_range": (0.3, 3.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 1000,
        },
    )
    
    # 2. 腿部关节刚度和阻尼随机化
    randomize_leg_joint_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
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
    
    # 2.1 缓存腿部关节刚度随机化参数（用于 privileged observations）
    cache_leg_joint_gains = EventTerm(
        func=mdp.cache_motor_strength_randomization,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
            ]),
            "stiffness_distribution_params": (0.7, 1.3),
            "damping_distribution_params": (0.7, 1.3),
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    
    # 3. 机械臂关节刚度和阻尼随机化
    randomize_arm_joint_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
            ]),
            "stiffness_distribution_params": (0.7, 1.3),
            "damping_distribution_params": (0.7, 1.3),
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    
    # ========== 启动时随机化事件 (startup mode) ==========
    
    # 4. 基座质量随机化 - 增加0到15公斤
    randomize_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (0.0, 10.0),
            "operation": "add",  # 添加操作
            "distribution": "uniform",
        },
    )
    
    # 4.1 缓存基座质量随机化参数（用于 privileged observations）
    cache_base_mass = EventTerm(
        func=mdp.cache_base_mass_randomization,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (0.0, 15.0),
            "operation": "add",
            "distribution": "uniform",
        },
    )
    
    # 5. 基座质心随机化 - xyz方向各偏移±0.15米
    randomize_base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "com_range": {
                "x": (-0.15, 0.15),
                "y": (-0.15, 0.15),
                "z": (-0.15, 0.15),
            },
        },
    )
    
    # 5.1 缓存基座质心随机化参数（用于 privileged observations）
    cache_base_com = EventTerm(
        func=mdp.cache_base_com_randomization,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "com_range": {
                "x": (-0.15, 0.15),
                "y": (-0.15, 0.15),
                "z": (-0.15, 0.15),
            },
            "operation": "add",
        },
    )
    
    # 6. 夹爪质量随机化 - 增加0到0.1公斤
    randomize_gripper_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
            "mass_distribution_params": (0.0, 0.1),
            "operation": "add",
            "distribution": "uniform",
        },
    )
    
    # 6.1 缓存夹爪质量随机化参数（用于 privileged observations）
    cache_gripper_mass = EventTerm(
        func=mdp.cache_gripper_mass_randomization,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
            "mass_distribution_params": (0.0, 0.1),
            "operation": "add",
            "distribution": "uniform",
        },
    )
    
    # ========== 间隔触发事件 (interval mode) ==========
    
    # 7. 推动机器人 - 每8秒施加一次随机速度
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(8.0, 8.0),  # 每8秒触发一次
        params={
            "velocity_range": {
                "x": (-0.5, 0.5),  # x方向速度范围 (m/s)
                "y": (-0.5, 0.5),  # y方向速度范围 (m/s)
            },
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward checked
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-1.0)
    # (3) Penalize z axis base linear velocity l2 checked
    z_vel_penalty = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.5)
    # (4) Penalize xy axes base angular velocity l2 checked
    xy_ang_vel_penalty = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.2)
    # (5) Penalize only roll orientation l2 (allows pitch movement) checked
    roll_penalty = RewTerm(func=mdp.roll_orientation, weight=-2.0)
    # (6) Penalize deviation from target base height checked
    base_height_penalty = RewTerm(
        func=mdp.base_height,
        weight=-5.0,
        params={"target_height": 0.4}  # 设置目标高度
    )
    # (7) Penalize torques applied l2 checked
    torque_penalty = RewTerm(func=mdp.joint_torques_l2,
        weight=-2.5e-5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    # dof acceleration penalty	惩罚关节加速度	减少关节震动
    dof_acceleration_penalty = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-7.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ])}
    )
    # (21) Penalize torque rate of change (delta torques) l2
    delta_torques = RewTerm(
        func=mdp.delta_torques_l2,
        weight=-1.0e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ])}
    )
    # stand_still 仅站立时有效，鼓励保持默认姿态
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    walking_dof = RewTerm(
        func=mdp.walking_dof,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    # (10) Penalize changes in actions checked
    action_rate_penalty = RewTerm(func=mdp.action_rate_l2, weight=-0.015)  # l2
    # (11) Penalize collisions on selected bodies checked
    collision_penalty = RewTerm(
        func=mdp.undesired_contacts,
        weight=-10.0,
        params={"threshold": 1.0, "sensor_cfg": SceneEntityCfg("penalize_contacts")},
    )
    # (12) Penalize dof positions too close to the limit checked
    dof_pos_limit_penalty = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ])}
    )
    # (15) X轴线速度跟踪，区分前进/后退/站立
    tracking_lin_vel_max = RewTerm(
        func=mdp.tracking_lin_vel_max,
        weight=2.0,
        params={
            "command_name": "locomotion_commands",
            "epsilon": 1e-5,
            "asset_cfg": SceneEntityCfg("robot")
        }
    )
    # (16) Tracking of angular velocity command (yaw)
    ang_vel_yaw_tracking = RewTerm(
        func=mdp.track_ang_vel_z,
        weight=0.5,
        params={"std": 0.2, "command_name": "locomotion_commands"}
    )
    # (17) Reward for feet_air_time 
    feet_air_time = RewTerm(func=mdp.feet_air_time, weight=1.0, params={"command_name": "locomotion_commands"})
    # feet_jerk	-0.0002	基于接触力变化率计算冲击	惩罚足部冲击
    feet_jerk = RewTerm(
        func=mdp.feet_jerk,
        weight=-0.0002,
        params={"sensor_cfg": SceneEntityCfg("contact_feet_forces")}
    )
    # feet_drag	-0.08	足部拖拽惩罚	惩罚足部在接触地面时移动
    feet_drag = RewTerm(
        func=mdp.feet_drag,
        weight=-0.08,
        params={
            "sensor_cfg": SceneEntityCfg("contact_feet_forces"),
            "asset_cfg": SceneEntityCfg("robot")
        }
    )
    # (18) Reward for end-effector position tracking
    object_distance = RewTerm(
        func=mdp.end_effector_pos_tracking,
        weight=0.8,
        params={"command_name": "target_position_commands",
                "gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
                "sigma": 0.1}
    )
    # (20) Penalize hip joints deviation from default position l2 checked
    hip_pos_penalty = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-0.3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint"
        ])}
    )
    # 步态跟踪奖励
    """tracking_contacts_force = RewTerm(
        func=mdp.reward_tracking_contacts_shaped_force,
        weight=-2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "gait_force_sigma": 0.5,
            "kappa_gait_probs": 0.07,
        }
    )"""
    tracking_contacts_vel = RewTerm(
        func=mdp.reward_tracking_contacts_shaped_vel,
        weight=-2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "gait_vel_sigma": 0.5,
            "kappa_gait_probs": 0.07,
        }
    )
    
    # 步态足部高度奖励（仅在行走时生效）
    feet_height = RewTerm(
        func=mdp.reward_feet_height,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "locomotion_commands",
            "target_height": 0.2,
        }
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Illegal contacts on selected bodiesfe
    illegal_contacts = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "threshold": 1.0,
            "sensor_cfg": SceneEntityCfg("terminate_after_contacts"),
        },
    )
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
class VbcAliengoEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: VbcSceneCfg = VbcSceneCfg(num_envs=6144, env_spacing=4.0)
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
