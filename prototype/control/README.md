# A2 depth-constrained MPC

The controller accepts a depth-derived local obstacle map and constrains the
robot's body and leg footprint while following LightNav's geometric path.
It never reads scene assets or uses commanded velocity as odometry.

```python
from prototype.control import CollisionAwareMPC, MPCInfeasible

controller = CollisionAwareMPC()  # horizon=15, prediction dt=0.15 s, control dt=0.1 s
result = controller.solve(
    pose_world,          # (3,) actual [x_m, y_m, yaw_rad]
    reference_world,     # (controller.horizon, 3), selected by actual path progress
    previous_command,    # currently effective [forward_m_s, yaw_rad_s]
    obstacles_world,     # (N,2) occupied 4 cm voxel centres, not inflated
    actual_planar_twist=actual_planar_twist,  # measured [vx,vy,yaw_rate], base-yaw plane
    execution_delay_s=execution_delay_s,     # recent solver duration in SIM seconds
    speed_limit=0.3,     # limit after operator/navigation authority arbitration
    motion_allowed=True,
)
```

Use `result.command` as `[v,0,w]`. Other result fields are `predicted_world`,
`predicted_time_s`, `execution_delay_s`, `minimum_clearance_m`, `solver_status`,
`solve_ms`, and `obstacle_count`. Prediction times are relative to input capture;
the returned trajectory includes the held-command delay prefix, so its length
is not fixed to `horizon+1`. Each time has exactly one corresponding world pose.
Clearance is distance from the modelled robot discs to occupied voxel surfaces;
it is `None` when no obstacles were supplied or motion is paused, so status is
JSON serializable. On `MPCInfeasible`, clear the navigation command and report
the stopped state. There is no unconstrained controller fallback.

The external force-brake state remains authoritative. `motion_allowed=False`
returns zero immediately. The controller has no velocity tracking error gain,
integral compensation, or clock-based path progress; it cannot raise the input
speed limit to catch up with a person-induced slowdown. The first command's
acceleration limit uses the actual 0.1 s control interval, separately from the
0.15 s prediction interval. Braking is allowed immediately.

Yaw commands use the interval
`[min(previous_w,0)-a_max_w*dt, max(previous_w,0)+a_max_w*dt]`, also clipped by
the configured yaw speed limit. This permits immediate command reduction to
zero while limiting growth in either direction. The velocity-response model
still predicts finite physical braking time; requesting zero is not treated
as zero actual yaw rate.

## Velocity response and execution delay

The predicted state is `[x,y,yaw,vx,vy,yaw_rate]`. Translation and yaw velocity
follow the requested command with a first-order response time of 0.15 s;
uncommanded lateral velocity decays with the same translation response time.
Measured velocity initializes this state on every solve. Thus requesting zero
does not instantly stop the predicted robot. This approximation is grounded in
`controller_01`: after a zero command, about 0.28 s and 37 mm of travel remained.
It is a prototype response model, not a fitted identification of all A2 gaits.

The currently effective command is held for `execution_delay_s` before any new
optimized command takes effect. Use 0.20 s initially while standing, then the
previous solve's elapsed simulation time. All prefix poses are checked against
the complete obstacle map. The solver rejects a prefix that is already unable
to clear the map; it cannot undo past motion. An estimate based on the previous
duration does not guarantee that the next solve will finish within that time.

Record input/submit times and poses, actual planar twist, effective prior
command, and the delay used. Do not pass the third component of XYZ linear
velocity as yaw rate. IPOPT uses an expanded graph; a feasible shifted previous
solution supplies the next initial guess without rerunning all 15 seed paths.

## Robot geometry

`footprint.json` stores three 0.30 m discs at base x = -0.30, 0, +0.30 m.
They cover the A2 collision primitives in 259 recorded walking poses from
`sim_20260914_164607`. A2-only forward kinematics, without simulation stepping,
gave horizontal bounds x=[-0.487,+0.394], y=[-0.261,+0.265] m and a maximum
required disc radius of 0.29392 m. This includes the observed leg swing, not
every joint-limit pose. These parameters do not describe the human user.

The 4 cm obstacle voxel is expanded once by its half diagonal, 0.02828 m.
At the configured velocity/rotation limits the nearest-sample centre motion is
bounded by about 0.012 m. The hard sampled clearance is this motion allowance
plus `minimum_swept_clearance_m=0.003`, giving 0.015 m at all three poses per
prediction step and the current pose. If measured speed or yaw rate exceeds
configured command limits, the motion allowance grows to cover that initial
velocity. The body radii and voxel inflation are unchanged. The remaining
3 mm gap describes the predicted motion sweep;
policy tracking error and unseen geometry still require real execution
evidence, so this is not a collision-free guarantee for all robot motions.

`desired_clearance_m=0.03` is a soft objective at prediction knots, with squared
shortfall weight 10000, averaged over the four nearby voxel constraints per
disc. It encourages extra clearance without rejecting a physically clear
initial pose just because tracking error consumed a few millimetres of that
desired margin. The hard swept clearance cannot be relaxed by this objective.

## Constraint construction

Each predicted disc initially constrains its four closest observed voxels with
exact squared-distance inequalities. After the NLP solves, a cKDTree query
checks the whole sampled swept footprint against **all** local voxels. If the
trajectory changed the relevant closest obstacles, constraints are updated and
the NLP solves again, up to three rounds; unresolved clearance stops execution.
This does not truncate the map to a fixed obstacle-count budget.

The local depth map, ground/self filtering, and persistence of observed geometry
outside the current camera view belong to the caller. Missing depth is not a
request to call the controller with an empty, supposedly free map.

## Offline evidence

Run from the prototype worktree with the existing SciPy 1.17.1 / CasADi 3.8.0
simulation environment:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .local/envs/sim/bin/python -m prototype.control.offline_examples
```

The examples cover open paths, a nearby obstacle, an oblique 0.92 m doorway,
human pause, and an occupied footprint. Initial geometry-only exercises passed
before the first integration; they are not evidence of physical door traversal.

The first integration's line 38 in
`.local/outputs/sim_20260914_180200/mpc.jsonl` demonstrated why desired clearance
must stay soft: its 28.28 mm current gap was rejected by a 30 mm hard rule.

The subsequent `controller_01` run exposed a separate execution mismatch:
line 13 of `.local/outputs/sim_20260914_181835/mpc.jsonl` assumed immediate
braking to 0.0478 m/s, but the old command continued during its 180.8 ms solve.
Actual position was 36 mm ahead of that prediction at submission. New depth
voxels compounded the loss of clearance. CPU replay with the velocity/delay
model used recorded actual pose/twist, effective command, map and prior solve
duration:

| Log line | Delay | Original forward command | New `[v,w]` | Predicted minimum | Solve time |
| --- | ---: | ---: | --- | ---: | ---: |
| 10 | 0.12 s | 0.3000 m/s | `[0.3000,-0.6000]` | 0.018564 m | 299 ms, cold |
| 11 | 0.14 s | 0.3000 m/s | `[0.1181,-0.6000]` | 0.015000 m | 67 ms, warm |
| 12 | 0.20 s | 0.3000 m/s | `MPCInfeasible` | cannot clear observed map with existing motion | — |

For line 11, direct pairwise distances against all **175** voxels at 479 poses,
including a 5 ms delay-and-motion sweep, matched the controller's 0.0150001 m
minimum. The numerical and symbolic state rollouts agreed within 1.1e-12.
The changed command brakes earlier; replay cannot establish that an entire
physical route now succeeds. The caller must measure actual cadence rather
than assume a stable 10 Hz.

After the RGB-D MSAA correction, `integrated_04` exposed excessive yaw-command
braking limits. A bounded A/B replay used only lines 15–16 of
`.local/outputs/sim_20260914_184522/mpc.jsonl`, retaining identical measured
state, delay, reference and complete map. The original line 15 command
`[0.285752,-0.439999]` and line 16 infeasibility reproduced exactly. With only
the yaw braking interval changed, line 15 returned `[0.077591,+0.080000]` and
line 16 returned approximately `[0,+0.080000]`, both `Solve_Succeeded`.
For line 16, all 176 voxels and 463 delay/motion poses at 5 ms sampling gave
15.392 mm minimum clearance. Solver sample clearance was 15.454 mm; the
sample threshold grows slightly because the recorded actual speed/yaw rate
exceeded command limits. Body radii, voxel inflation and minimum swept gap
were unchanged. Warm solve time was 182.5 ms for this constrained case, so
execution-delay estimates still need comparison with actual submission times.
