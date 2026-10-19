# Native Nav2 MPPI transport

`mppi.py` launches `run.sh` and exchanges one JSON record per line. The executable
links the official `nav2_mppi_controller` optimizer. `GuidePupEnvelopeCritic` is a
plugin loaded by its normal critic manager; it does not implement MPPI sampling
or weighted optimization. No behavior tree, global planner, ROS sensor subscriber,
scene geometry, perfect pose, or alternate controller is used.

## Build and run

On Ubuntu 24.04, `setup.sh` downloads official ROS Jazzy/Ubuntu packages directly
and extracts them rootlessly under `.local/nav2_sysroot`. It relocates exported
CMake metadata from `/opt/ros/jazzy` to that prefix. No system packages are installed.
Build tools (CMake and a C++17 compiler) come from the host. Then run:

```bash
prototype/control/nav2/setup.sh
prototype/control/nav2/build.sh
```

The verified installation is Nav2 **1.3.13**, official package
`ros-jazzy-nav2-mppi-controller` version `1.3.13-1noble.20260903.032340`.
The installation receipt is
[setup_nav2_jazzy_packages.tsv](../../../.local/outputs/mppi_integration_20260917_01/setup_nav2_jazzy_packages.tsv).
Build/run scripts set the rootless ROS, Ubuntu-library, LLVM and Python metadata
paths. `NAV2_SYSROOT` and `NAV2_PREFIX` may select an existing installation.
The bridge installs inside `.local/nav2_bridge`. Missing libraries, unsupported
APIs, malformed data, and bridge failure are errors. They never select another
controller. A native `NoValidControl` or rejected final trajectory is an explicit
`LocalControlBlocked`. `retry_attempt_limit=0` prevents native optimization retry.

## Data and geometry

Each request carries the exact sensor epoch, timestamp, delay-propagated measured
pose/twist, observed depth voxel centres and LightNav path, all in odom XY. ROS
stamps identify that sensor snapshot; no TF pose lookup supplies controller state.
The direct costmap context is 4 cm resolution, its origin and size are explicit,
and all cells start unknown. Observed occupied centres become lethal cells; no
unknown cell is asserted observed free. The exact point index includes every
supplied occupied centre, even outside the context grid. This preserves the
previous normal controller's **observed-obstacle collision scope**. It does not
certify unknown space as traversable.

The shared trajectory evaluator is the sole authority for both sampled candidates
and the exact filtered/constrained sequence. It retains the complete three-circle
body, voxel half-diagonal and per-request hard clearance. Actual commanded motion
checks geometric clearance at .05 s samples. At the initial state and each .1 s
knot, nominal calibrated stopped-body geometry is checked every .05 s through
2 s without uncertainty padding. All these geometric checks must satisfy the
original hard clearance. Python checks the observed initial geometry and the
already committed execution-delay prefix before optimization; empirical stop
uncertainty alone no longer prevents a native solve.

The empirical stop clearance retains every original response coefficient and
position/yaw residual in `stop_model.json`. Its deficit is a squared planning cost
with the existing clearance weight, separately from the preferred-clearance cost.
In normal mode (initial empirical stop clearance at least the hard clearance),
every future knot must still satisfy that empirical envelope.

When the initial empirical envelope is deficient, restoration uses the maximum
empirical deficit along a zero-command NAV-response rollout over the configured
2.4 s horizon as a transient ceiling. This rollout is a comparison, never an
issued fallback. A persistent episode records the sensor epoch, initial execution
time (`stamp_s + execution_delay_s`), fixed deadline one horizon later and ceiling.
Replanning can tighten the ceiling using the new zero-command comparison, but
cannot increase it or move the deadline. An epoch change or actual initial
empirical clearance recovery ends the episode. Optimizer resets, Qwen pauses and
strategy changes do not reset it. Candidates must remain under the ceiling,
satisfy the full original empirical envelope at every knot at/after the deadline,
and recover by their horizon endpoint. Expired episodes still evaluate every
sample and return an explicit block. This is bounded feasibility restoration;
it does not weaken body geometry or scale the uncertainty with speed.

After filtering, Nav2's own sequence constraints are reapplied (history may cross
a newly reduced speed limit), then the same evaluator checks the actual controls.
An invalid mean is rejected without retries or replacement candidates.

The interval residual reflects cross-case paused-stop calibration uncertainty,
not occupied space; calibration contains no static restart case. Soft-constraint
feasibility restoration, as discussed by
[Wabersich and Zeilinger](https://arxiv.org/abs/2105.10241), is conceptual motivation
only. This prototype map/model and finite horizon establish no safety theorem.

`config.json` owns the MPPI horizon and sampling parameters. The independently
selected footprint file owns physical circles, hard-clearance parameters,
`desired_clearance_m` and `clearance_cost_weight`. The requested experiment supplies
.10 m as the preference; the repository default is not silently overridden.
The preference remains a cost, not a 10 cm hard ban.
Commands remain nonholonomic: vx from zero to the authority limit, |wz| <= .6,
no commanded vy. Measured lateral velocity remains in the empirical response and
stopping model.

## Prediction distinction

A bounded DiffDrive extension projects native command samples to the authorized
forward/yaw domain before Nav2's normal acceleration propagation. Every native
critic then sees the same candidate commands; measured initial velocity remains
intact. Because native propagation also overwrites command samples, command
bounds are reapplied afterwards while retaining propagated actual velocities.
Thus a zero forward authority emits zero even while measured forward motion is
still decaying; it never erases that motion from either model's initial state.
This prevents scoring reverse/overspeed Gaussian noise as a valid stopping
trajectory without rejecting nearly every sample at a zero forward limit.
Nav2's native DiffDrive model supplies its path/goal critics. Its sampled command
sequences are also evaluated by the custom critic using A2's prior empirical
first-order response model and calibrated stop envelope. These are distinct models:
this integration does not claim that native DiffDrive dynamics identify the A2
plant. The reported `predicted_odom` uses the empirical response for the actual
filtered native sequence, including execution latency.

The adapter records stderr separately and journals initialization, native input
and native output beside it (`.jsonl`). Replies identify the native backend and
single optimization attempt. This source is only runtime verified after a native
build and execution; writing the bridge is not evidence of successful navigation.

## Upstream implementation

- [Jazzy Optimizer API](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/include/nav2_mppi_controller/optimizer.hpp)
- [Jazzy optimization and control filtering](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/optimizer.cpp)
- [Native critic plugin interface](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp)

The subclass only exposes the optimized sequence, sets absolute forward authority
without coupling it to yaw authority, and moves warm-start shifting to the next
request. `evalControl` remains the official library implementation. Its native
noise generator, weighting, acceleration propagation and filter are retained; the
DiffDrive extension additionally bounds samples to the command domain before
propagation.

## Per-solve evidence

Success and blocked replies include `category`, `reason`, `optimization_attempts`
and `diagnostics`:

- `planning_context`: initial geometric, nominal-stop and empirical-stop
  clearances, normal/restoration mode, current zero-command baseline deficit,
  persistent ceiling, episode start, absolute deadline and remaining time.
- `candidate_evaluations`: every actual sampled comparison, with index, first
  command, acceptance/reason, envelope cost, minimum geometric/nominal-stop/
  empirical-stop clearances and terminal empirical clearance. Evaluation continues
  through the whole horizon even after a rejection to retain complete minima.
- `candidate_summary`: accepted/rejected counts, counts by rejection reason,
  complete controls for the feasible sample with best envelope cost and one sample
  per rejection reason. The reported cost is the envelope critic contribution;
  Nav2 independently adds its unchanged path, goal and control costs.
- `final_check`: the exact filtered/constrained controls, all knot states and
  clearance metrics, including rejected averages. If native `NoValidControl`
  throws before weighting/filtering, it explicitly reports unavailable with null
  controls/knots; no nonexistent final sequence is represented as evaluated.

Unbounded clearances with no observed obstacles are JSON null. Rejection reasons
separate predicted geometry overlap, geometric margin, nominal stopped geometry,
empirical ceiling, restoration deadline/expiry and normal empirical envelope.
