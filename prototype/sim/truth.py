"""Offline-only simulation evaluator stream. Never served by the robot API."""
from __future__ import annotations

import json


class TruthRecorder:
    def __init__(self, simulation, output):
        self.simulation = simulation
        self.stream = (output / 'ground_truth.jsonl').open('w', buffering=1)

    def write(self, control, applied_force):
        sim = self.simulation
        self.stream.write(json.dumps({
            'sim_time_s': float(sim.data.time),
            'pose_world': sim.truth_pose().tolist(),
            'base_z_m': float(sim.data.qpos[2]),
            'world_from_base': sim.truth_world_from_base().tolist(),
            'actual_velocity_body': sim.truth_velocity().tolist(),
            'actual_planar_twist': sim.truth_planar_twist().tolist(),
            'force_applied_local_N': applied_force.tolist(),
            'state': control['state'], 'task_id': control['task_id'],
            'requested_cmd': control['requested_cmd'], 'effective_cmd': control['effective_cmd'],
            'estimator_stamp_s': control['robot_state']['stamp_s'],
        }) + '\n')

    def close(self):
        self.stream.close()
