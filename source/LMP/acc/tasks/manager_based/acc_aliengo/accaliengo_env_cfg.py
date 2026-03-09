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

from . import mdp
from isaaclab.envs.mdp.commands import UniformVelocityCommand
from .commands import DynamicManipulationCommandCfg
from isaaclab.managers import CommandTermCfg
from isaaclab.sensors import ContactSensorCfg

from isaaclab.sim import SimulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.converters import UrdfConverterCfg

from .observation_compute import (
    ee_pos_in_base_frame,
    ee_goal_pos_in_base_frame,
    ee_tracking_error_in_base_frame,
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
        class_type = UniformVelocityCommand
        
        # 必需字段：重采样时间范围（秒）
        resampling_time_range: Tuple[float, float] = (10.0, 10.0)
        heading_command: bool = False
        rel_standing_envs: float = 1.0

        # 命令名称
        command_name: str = "locomotion_commands"

        # 命令维度: lin_vel_x, lin_vel_y, ang_vel_yaw, heading
        dim: int = 3

        @configclass
        class Ranges:
            """Ranges for the commands."""

            lin_vel_x: Tuple[float, float] = (-1.0, 1.0)
            lin_vel_y: Tuple[float, float] = (0.0, 0.0)
            ang_vel_z: Tuple[float, float] = (0.0, 1.0)
            heading: Tuple[float, float] = (-math.pi, math.pi)

        # 命令范围
        ranges: Ranges = Ranges()

        strategy: str = "uniform"  # or "curriculum", "fixed", etc.
        asset_name: str = "robot"
        body_name: str = "base"

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
        sphere_center: Tuple[float, float] = (0.2535, 0.0)  # VBC: 机身坐标系XY
        sphere_center_z: float = 0.6  # VBC: 世界坐标系固定Z高度
        
        # 时间配置
        move_time_range: Tuple[float, float] = (2.0, 3.0)  # VBC: 运动时长
        hold_time_range: Tuple[float, float] = (0.5, 1.2)  # VBC: 保持时长
        
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
            x_min: float = -0.6   # VBC: 后方60cm（覆盖后腿区域）
            x_max: float = 0.15   # VBC: 前方15cm（机械臂基座在0.25，留出空间防止自碰）
            y_min: float = -0.2   # VBC: 左侧20cm（覆盖躯干宽度及部分腿部）
            y_max: float = 0.2    # VBC: 右侧20cm
            z_min: float = -0.4   # VBC: 下方40cm（Aliengo站高约40cm，此值覆盖地面）
            z_max: float = 0.05   # VBC: 上方5cm（覆盖躯干顶部，防止机械臂向下砸背）

        collision_box: CollisionBox = CollisionBox()
        
        # 可视化
        debug_vis: bool = True

    # 命令组
    target_position_commands: TargetPositionCommandCfg = TargetPositionCommandCfg()
    locomotion_commands: LocomotionCommandCfg = LocomotionCommandCfg()

##
# Scene definition
##


@configclass
class AccSceneCfg(InteractiveSceneCfg):
    """Configuration for ACC scene."""
    
    # 关键：禁用物理属性复制，以支持域随机化（VBC配置）
    replicate_physics = False

    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,  # VBC配置
                dynamic_friction=1.0,  # VBC配置
                restitution=0.0       # VBC配置
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
            self_collision=True,  # VBC新增：启用自碰撞检测
            
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
                solver_position_iteration_count=16,  # VBC: 提高求解精度以改善碰撞检测
                solver_velocity_iteration_count=8,   # VBC: 提高速度求解精度
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
            pos=[0.0, 0.0, 0.45],
            rot=[1.0, 0.0, 0.0, 0.0],
            joint_pos={ 
            'FL_hip_joint': 0.1,
            'FL_thigh_joint': 0.8,
            'FL_calf_joint': -1.5,

            'RL_hip_joint': 0.1,
            'RL_thigh_joint': 0.8,
            'RL_calf_joint': -1.5,

            'FR_hip_joint': -0.1,
            'FR_thigh_joint': 0.8,
            'FR_calf_joint': -1.5,

            'RR_hip_joint': -0.1,
            'RR_thigh_joint': 0.8,
            'RR_calf_joint': -1.5,
            
            'arm_joint1': 0.0,
            'arm_joint2': 1.48,
            'arm_joint3': -0.63,
            'arm_joint4': -0.84,
            'arm_joint5': 0.0,
            'arm_joint6': 1.57,
        }
        ),
        
        # 软关节限制因子
        soft_joint_pos_limit_factor=1.0,  # VBC: 使用1.0不限制
        
        # 执行器配置 - 按照VBC control配置
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[
                    # 按照实际机器人关节顺序排列
                    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",          # hip joints
                    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",  # thigh joints
                    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",      # calf joints
                ],
                # Aliengo URDF: Calf max 44.4 Nm, others 35.3 Nm
                effort_limit_sim=45.0,        # VBC: 适配 Aliengo 的真实力矩限制
                # Aliengo URDF: Hip/Thigh max 20 rad/s, Calf 15.9 rad/s
                velocity_limit_sim=20.0,      # VBC: 适配 Aliengo 的真实速度限制
                # VBC: Aliengo 较轻，使用较小刚度
                stiffness=40.0,               # VBC建议值（原B1为80.0）
                damping=1.0,                  # VBC建议值
                armature=0.0,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[
                    "arm_joint1", "arm_joint2", "arm_joint3",
                    "arm_joint4", "arm_joint5", "arm_joint6",
                ],
                # Z1 URDF: Joint2 max 60 Nm, others 30 Nm
                effort_limit_sim=60.0,        # VBC: Z1 机械臂最大力矩
                # Z1 URDF: All joints max 3.1415 rad/s
                velocity_limit_sim=3.15,      # VBC: Z1 机械臂最大速度
                stiffness=400.0,              # VBC: 高精度位置控制
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
        scale=0.25,  # 减小缩放因子以限制腿部运动幅度
        use_default_offset=True,  # 关键：让动作为与默认姿态的偏移量
    )

    # 机械臂动作空间 - 使用关节位置控制（与观测匹配）
    arm_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"],
        scale=0.25,  # 关节位置缩放因子
        use_default_offset=True,
    )





@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        
        # 基座的高度
        # base_height = ObsTerm(func=mdp.base_pos_z, params={"asset_cfg": SceneEntityCfg("robot")})
        # 基座的线速度
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        # 基座的角速度
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        # projected gravity in base frame
        project_gravity = ObsTerm(func=mdp.projected_gravity, params={"asset_cfg": SceneEntityCfg("robot")})
        # 控制指令（目标线速度x, y和目标角速度yaw）
        command = ObsTerm(func=mdp.generated_commands, params={"command_name": "locomotion_commands"})

        # 关节位置误差（当前位置 - 默认位置）- 排除夹爪关节
        dof_err = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
            ])})  # 明确列出要包含的关节，排除夹爪关节
        # 关节速度 - 排除夹爪关节
        dof_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
        ])})
        # 夹爪在机器人本体坐标系下的位置
        curr_ee_pos_cart = ObsTerm(
            func=ee_pos_in_base_frame,
            params={"gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover")},
        )
        # 当前的末端执行器目标位置(本体坐标系下)
        curr_ee_goal_pos_cart = ObsTerm(
            func=ee_goal_pos_in_base_frame,
            params={"command_name": "target_position_commands"},
        )
        # 夹爪在机器人本体坐标系下的跟踪误差
        tracking_error = ObsTerm(
            func=ee_tracking_error_in_base_frame,
            params={
                "command_name": "target_position_commands",
                "gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
            },
        )
        # 上一步的动作
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # 重置机器人根部状态 (位置、姿态、速度)
    reset_robot_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.05, 0.05)},  # VBC: z范围为(-0.05, 0.05)
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
            "position_range": (0.0, 0.0),  # 相对于默认位置的比例
            "velocity_range": (0.0, 0.0),   # 重置时速度为0
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
                "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
                "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
                "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
            ]),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-1.0)
    # (3) Penalize z axis base linear velocity l2
    z_vel_penalty = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.0)
    # (4) Penalize xy axes base angular velocity l2
    xy_ang_vel_penalty = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.1)
    # (5) Penalize non flat base orientation l2
    orientation_penalty = RewTerm(func=mdp.flat_orientation_l2, weight=-0.5)
    base_height_penalty = RewTerm(  # l2
        func=mdp.base_height_l2, 
        weight=-0.2,
        params={"target_height": 0.36}  # VBC: Aliengo目标高度0.36米
    )
    # (7) Penalize torques
    torque_penalty = RewTerm(func=mdp.joint_torques_l2, weight=-0.0001)  # l2
    # (8) Penalize dof velocities
    dof_vel_penalty = RewTerm(func=mdp.joint_vel_l2, weight=-0.)  # l2
    # (9) Penalize dof accelerations
    dof_acc_penalty = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-9)  # l2
    # (10) Penalize changes in actions
    action_rate_penalty = RewTerm(func=mdp.action_rate_l2, weight=-0.001)  # l2
    # (11) Penalize collisions on selected bodies
    collision_penalty = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"threshold": 1.0, "sensor_cfg": SceneEntityCfg("penalize_contacts")},
    )  
    # (12) Penalize dof positions too close to the limit - 使用 Isaac Lab 官方 API
    dof_pos_limit_penalty = RewTerm(
        func=mdp.joint_pos_limits,  # 使用官方 API，自动读取 URDF 中的关节限制
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"
        ])}  # 明确列出要检查的关节，排除夹爪关节
    )
    # (13) Penalize dof velocities too close to the limit, clip to max error = 1 rad/s per joint to avoid huge penalties
    # dof_vel_limit_penalty = RewTerm(func=mdp.dof_vel_limit_penalty, weight=-0.)
    # (14) Penalize torques too close to the limit
    # torque_limit_penalty = RewTerm(func=mdp.torque_limit_penalty, weight=-0.0001)

    # (15) Tracking of linear velocity commands (xy axes)
    lin_vel_xy_tracking = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=2.0,
        params={"std": 0.5, "command_name": "locomotion_commands"}
    )
    # (16) Tracking of angular velocity command (yaw) - 使用官方API
    ang_vel_yaw_tracking = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"std": 0.25, "command_name": "locomotion_commands"}
    )
    # (17) Reward for feet_air_time 自定义函数实现于 compute_rewards.py
    feet_air_time = RewTerm(func=mdp.feet_air_time, weight=1.0, params={"command_name": "locomotion_commands"})
    # (18) Reward for end-effector position tracking
    object_distance = RewTerm(
        func=mdp.end_effector_pos_tracking,
        weight=2.0,
        params={"command_name": "target_position_commands",
                "gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover"),
                "sigma": 0.1}
    )
    # (19) Reward for end-effector position tracking l2
    object_distance_l2 = RewTerm(
        func=mdp.end_effector_pos_tracking_l2,
        weight=-1.0,
        params={"command_name": "target_position_commands",
                "gripper_cfg": SceneEntityCfg("robot", body_names="gripperMover")}
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Illegal contacts on selected bodies
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
    # (4) Base_height above maximum
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
class AccAliengoEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: AccSceneCfg = AccSceneCfg(num_envs=6144, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

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