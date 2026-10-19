"""Read-only progress from estimated state and commands actually sent to the plant."""
from __future__ import annotations

import numpy as np


def model_observation(record):
    """Keep model pointing tied to the original source image, not the latest camera frame."""
    result = record['result']
    stamp = record['capture_time_s']
    frame_id = record['frame_id']
    return dict(stop=bool(result['stop']), visible=result['visible'],
        sequence=record['sequence'], capture_time_s=stamp, session_id=record['session_id'],
        source_frame_id=frame_id,
        source_image_label=f'Navigation model input RGB; frame {frame_id}; '
            f'sensor time {stamp:.3f} s; pointing refers to this image',
        response_sensor_time_s=record['stamp_s'], instruction=record['instruction'],
        pointing=result.get('pointing'), raw_text=result['raw_text'],
        predicted_waypoints=result['actions']['actions'])


def handoff_history_evidence(records):
    """Retain every target review without duplicating raw waypoint arrays/images."""
    history = []
    for record in records:
        entry = {key: value for key, value in record.items()
            if key not in ('first_result', 'comparison', 'overlay', 'source_image')}
        if 'first_result' in record:
            result = record['first_result']
            entry['probe'] = dict(stop=result['stop'], visible=result['visible'],
                pointing=result.get('pointing'), raw_text=result['raw_text'],
                comparison=record['comparison'], candidate_motion_executed=False)
        history.append(entry)
    return history


class NavigationProgress:
    def __init__(self, instruction, state, applied_controls, solver_calls):
        self.instruction = instruction
        self.start_s = float(state.stamp_s)
        self.start_pose = np.asarray(state.pose_odom, dtype=float).copy()
        self.last_pose = self.start_pose.copy()
        self.travel_m = 0.
        self.yaw_change_rad = 0.
        self.start_applied_controls = applied_controls
        self.start_solver_calls = solver_calls
        self.nonzero_command_ticks = 0
        self.last_nonzero_command = None
        self.last_nonzero_command_s = None

    def observe(self, state, effective_command):
        pose = np.asarray(state.pose_odom, dtype=float)
        self.travel_m += float(np.linalg.norm(pose[:2]-self.last_pose[:2]))
        yaw_delta = pose[2]-self.last_pose[2]
        self.yaw_change_rad += float(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))
        self.last_pose = pose.copy()
        command = np.asarray(effective_command, dtype=float)
        if np.any(command != 0.):
            self.nonzero_command_ticks += 1
            self.last_nonzero_command = command.tolist()
            self.last_nonzero_command_s = float(state.stamp_s)

    def snapshot(self, state, applied_controls, solver_calls):
        delta = np.asarray(state.pose_odom, dtype=float)[:2]-self.start_pose[:2]
        return dict(instruction=self.instruction, start_s=self.start_s,
            observed_until_s=float(state.stamp_s),
            elapsed_s=float(state.stamp_s)-self.start_s,
            executed_commands=dict(
                distinct_mppi_controls_sent=applied_controls-self.start_applied_controls,
                nonzero_wbc_command_ticks=self.nonzero_command_ticks,
                last_nonzero_vx_vy_wz=self.last_nonzero_command,
                last_nonzero_sensor_time_s=self.last_nonzero_command_s),
            controller_solver_calls=solver_calls-self.start_solver_calls,
            estimated_motion=dict(travel_m=self.travel_m, net_displacement_xy_m=delta.tolist(),
                net_displacement_m=float(np.linalg.norm(delta)), yaw_change_rad=self.yaw_change_rad),
            limitation='Commands sent are not proof of movement. Motion is drifting IMU/WBC '
                'dead reckoning, not physical ground truth or target-relative distance. '
                'The observation interval includes coasting and waiting after STOP.')
