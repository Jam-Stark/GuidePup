# A2 local control with Nav2 MPPI

The production controller is `Nav2MPPI` in `mppi.py`, backed by the real
`nav2_mppi_controller::Optimizer` C++ library. CasADi/IPOPT and the old geometric
recovery optimizer have been removed from the execution path; their prior source
is preserved with the September 17 integration evidence and old experiment snapshots.

Qwen selects semantic instructions. LightNav produces a soft odom-frame reference
path. Native MPPI samples and optimizes velocity sequences using path/goal critics
and an A2 footprint/stopping critic. Its output is `[forward, 0, yaw]`, bounded by
0.3 m/s and 0.6 rad/s. There is no lateral-motion expansion or second path optimizer.

## Inputs and geometry

All poses, measured body twists and occupied voxel centres come from the same
RGB-D/IMU/joint-state estimate and map snapshot. No simulator truth enters control.
The three-disc envelope, 4 cm voxel inflation and hard swept clearance remain.
`footprint.json` is the single source for desired clearance; the experiment uses
a separate 10 cm copy without changing the default.

The already-issued command is propagated over capture age and estimated execution
delay using measured twist and the existing 0.15-second response model. Python
rejects only geometric overlap or insufficient basic geometric margin in that
committed prefix. An empirical stopping-reserve deficit is passed to native MPPI.

The calibrated `stop_model.json` is unchanged. Its six moving-to-paused stop cases
do not establish a low-speed scaling law or static-restart guarantee. The custom
critic keeps geometric clearance and nominal stopped geometry hard, while scoring
the full empirical reserve deficit. If that reserve is already deficient, candidates
must stay within the zero-command prediction's deficit ceiling and restore the
full reserve by a fixed 2.4-second deadline. Replanning cannot postpone the deadline
or enlarge the ceiling. This is a bounded engineering restoration rule, not a
formal safety guarantee; see [the stopping-planning report](../NAVIGATION_STOP_PLANNING_20260917.md).

Sampled candidates and the final weighted, smoothed, constrained command sequence
use the same evaluator. The actual first command comes from that accepted sequence.
Native path critics retain Nav2's DiffDrive prediction. This difference is explicit;
neither prediction is physical truth.

The collision check covers observed occupied depth voxels. Unobserved space is
not certified free; the costmap retains unknown cells. This local obstacle control
is not a global route planner or proof of collision-free travel.

## Failure and evidence

`LocalControlBlocked` distinguishes current/committed geometric overlap, basic margin,
native candidate infeasibility, empirical reserve restoration failure, and final
trajectory rejection. These are estimated-map findings, not contact observations.
Runtime/API/dependency errors
raise errors. Native retry attempts are zero; no alternate controller or fabricated
forward command is used. Actual execution failure returns to Qwen within the task
recovery budget. Model STOP is separately returned as a decision event.

The server records `local_controller.jsonl` (input state, reference, obstacles,
predicted motion, commands and disposition), `nav2_mppi.jsonl` (full native requests
and responses, all candidate metrics and final checked controls/knots), and
`nav2_mppi.log` (native diagnostic stderr). The public status retains compact summaries.
The motion buffer
separates effective velocity-command summaries from sensor-estimated movement.

## Native dependency and build

Native sources/configuration and build/run scripts are in `nav2/`. This host uses
rootless official ROS 2 Jazzy binary packages under `.local/nav2_sysroot`, installed
through direct server networking; shared Isaac/Python environments are untouched.
The final setup receipt and the one-run report are linked from
[the integration report](../NAVIGATION_MPPI_20260917.md).

The Python adapter owns the native subprocess; call `close()` after use. Production
server shutdown closes it after cancelling tracking. No hardware is controlled by
these simulation launchers.
