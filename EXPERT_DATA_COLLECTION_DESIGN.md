# Expert Data Collection Design for ACC

## 1. Overview
This document outlines the strategy used to collect expert demonstrations from a trained VBC (Velocity-Based Control) policy to initialize the ACC (Adaptive Compliance Control) network via Behavior Cloning.

The goal is to create a dataset of state-action pairs $(s, a)$ where the ACC network learns to mimic the locomotion and manipulation behavior of the VBC system.

## 2. Observation Space (75-dim)
The ACC student policy requires a specific 75-dimensional observation vector. Since the VBC environment provides different observations, we manually construct the ACC observation vector during recording:

| Component | Dims | Source in VBC |
|-----------|------|---------------|
| Base Linear Velocity | 3 | `robot.data.root_lin_vel_b` |
| Base Angular Velocity | 3 | `robot.data.root_ang_vel_b` |
| Projected Gravity | 3 | `robot.data.projected_gravity_b` |
| Locomotion Command | 3 | `env.command_manager` (lin_x, lin_y, ang_yaw) |
| DOF Position Error | 18 | $q - q_{default}$ (12 legs + 6 arm) |
| DOF Velocity | 18 | `robot.data.joint_vel` |
| EE Position (Base Frame) | 3 | Computed via FK (`body_pos_w` transformed to base) |
| EE Goal (Base Frame) | 3 | `env.command_manager` (target_position) |
| Tracking Error | 3 | $EE_{goal} - EE_{curr}$ |
| Last Action | 18 | Buffer of previous step's expert action |
| **Total** | **75** | |

## 3. Expert Action Construction (The Hybrid Strategy)

The ACC policy controls all 18 joints (12 legs + 6 arm) by outputting **normalized joint position targets**.
$$ a = \frac{q_{target} - q_{default}}{\text{scale}} $$
where $\text{scale} = 0.25$.

However, the VBC "expert" is a hybrid system:
1.  **Legs**: Controlled by an RL Neural Network.
2.  **Arm**: Controlled by a mathematical Inverse Kinematics (IK) solver.

### The Challenge: IK Instability
During data analysis, we observed that `robot.data.joint_pos_target` for the Arm joints contained numerical artifacts (values > 3000 rad). This occurs because the IK solver interacts directly with the physics engine, and the high-level target buffer in Isaac Lab was not reliably reflecting the IK solution in the Python layer.

### The Solution: Hybrid Sampling
To generate stable, high-quality labels for Behavior Cloning, we implemented a hybrid sampling strategy:

#### A. For Legs (Joints 0-11)
We use the **Policy Output**.
-   **Source**: `robot.data.joint_pos_target`
-   **Logic**: The VBC policy directly outputs these targets. They are the "ground truth" expert intent.

#### B. For Arm (Joints 12-17)
We use the **Current Joint Position**.
-   **Source**: `robot.data.joint_pos`
-   **Logic**: Since the IK solver is a high-gain position controller, the robot's actual position closely tracks the valid IK solution. By using the current position as the target, we teach the student network to "maintain the configuration achieved by the expert."
-   **Benefit**: This completely eliminates the numerical instability from the IK target buffer while preserving the correct manipulation trajectory.

### Final Action Computation
```python
# 1. Legs: Trust the VBC Policy targets
leg_targets = robot.data.joint_pos_target[:, 0:12]

# 2. Arm: Trust the Physics State (Stable Proxy)
arm_targets = robot.data.joint_pos[:, 12:18]

# 3. Combine
final_targets = torch.cat([leg_targets, arm_targets], dim=1)

# 4. Normalize
expert_action = (final_targets - default_joint_pos) / 0.25
```

## 4. Data Storage
The data is saved in HDF5 format for efficient training:
-   `obs`: (N, 75) - Float32
-   `actions`: (N, 18) - Float32
-   `dones`: (N, 1) - Uint8

## 5. Validation
Statistical analysis of the collected data confirms the fix:
-   **Leg Actions**: Range [-21, +12] (Normalized), consistent with walking.
-   **Arm Actions**: Range [-17, +9] (Normalized), consistent with manipulation.
-   **Stability**: No outliers or numerical explosions.
