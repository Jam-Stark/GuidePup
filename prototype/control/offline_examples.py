"""Small executable geometry examples, without a simulator or policy inference."""
import json

import numpy as np

from .mpc import CollisionAwareMPC, MPCInfeasible


def main():
    controller = CollisionAwareMPC()
    x = np.arange(1, controller.horizon + 1) * .07
    doorway = np.column_stack([
        np.full(54, .65),
        np.r_[np.linspace(-1.5, -.46, 27), np.linspace(.46, 1.5, 27)],
    ])
    cases = [
        ('open_straight', np.zeros(3),
         np.column_stack([x, x * 0., x * 0.]), np.empty((0, 2))),
        ('near_left_obstacle', np.zeros(3),
         np.column_stack([x, -.3 * x, np.full_like(x, -.3)]), np.array([[0., .365]])),
        ('diagonal_doorway', np.array([0., .15, -.75]),
         np.column_stack([x, .15 - .2 * x, np.full_like(x, -.75)]), doorway),
    ]
    for name, pose, reference, obstacles in cases:
        result = controller.solve(pose, reference, (0., 0.), obstacles,
                                  actual_planar_twist=(0., 0., 0.), execution_delay_s=.2)
        print(json.dumps({
            'case': name, 'command': result.command,
            'minimum_clearance_m': result.minimum_clearance_m,
            'final_predicted_pose': result.predicted_world[-1].tolist(),
            'solver_status': result.solver_status, 'solve_ms': result.solve_ms,
        }))
    result = controller.solve(pose, reference, (.3, .2), obstacles, motion_allowed=False,
                              actual_planar_twist=(.3, 0., .2), execution_delay_s=.2)
    print(json.dumps({'case': 'human_pause', 'command': result.command, 'status': result.solver_status}))
    try:
        controller.solve(np.zeros(3), reference, (0., 0.), np.array([[0., .2]]),
                         actual_planar_twist=(0., 0., 0.), execution_delay_s=.2)
    except MPCInfeasible as exc:
        print(json.dumps({'case': 'occupied_footprint', 'stopped_reason': str(exc)}))


if __name__ == '__main__':
    main()
