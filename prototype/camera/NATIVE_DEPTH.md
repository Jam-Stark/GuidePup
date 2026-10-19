# Separate RGB and full depth field of view

The Isaac backend renders two synchronized optical streams from one PhysX state:

| Stream | Raster | Actual rendered field of view | Consumer |
|---|---|---|---|
| RGB | 640×360 | 69.000° × 42.273° | LightNav and Qwen images |
| Depth | 848×496 | 87.000° × 58.065° | Depth/IMU odometry, occupancy, MPC geometry |

The depth raster is a **simulation render size**, not a claimed D435i hardware
stream mode. It covers the nominal 87°×58° D435i depth optics using RTX's native
square-pixel pinhole rays. The RGB stream is not widened or stretched.

## Calibration and data flow

`prototype/isaac/worker.py` exports actual per-camera intrinsic matrices and fixed
optical-to-body transforms. Both optical centers are explicitly co-located in
this nominal simulation; this is not measured device extrinsic calibration.
The real sensor adapter must supply the selected hardware stream's intrinsic
matrix, depth scale, rigid extrinsics and timestamps.

`DepthImage` retains native axial-Z depth, its own pixel grid and calibration.
`RGBDLocalOdometry` consumes this full native depth plus the existing IMU and
learned proprioceptive velocity; its estimation algorithm is unchanged.
`DepthObstacleMap` integrates the same native rays using the resulting shared
`RobotState`.

`camera/alignment.py` deprojects native depth pixels, applies the depth-to-RGB
rigid transform, projects their pixel footprints and uses a nearest-depth buffer
on the RGB grid. Missing samples remain unknown. RGB masks and pointing are
measured on this calibrated RGB-grid depth. Native pixels outside RGB contribute
to geometric obstacles/free-space evidence but do not receive RGB semantics.

`DepthFrame.geometry` attaches the full native image to the RGB observation;
both share one state and exposure time. Frame archives include both depth grids,
both intrinsic matrices, both fixed transforms and the estimated state. RGB-axis
reference depth is rendered only for the offline calibration smoke artifact and
is excluded from production camera IPC.

## Why the render raster differs from the hardware mode

In this installed Isaac Sim 5.1 / Isaac Lab 0.54.4 environment, the OpenCV lens
schema and `UsdRender.Product.pixelAspectRatio` accepted separate fx/fy metadata,
but the `distance_to_image_plane` output still followed square-pixel rays.
Using the declared 848×480 / 87°×58° matrix produced a 31.4 mm median floor
reprojection error. Those candidates were rejected rather than used online.

The explicit native 848×496 raster uses the actual `CameraData` matrix with
fx=fy=446.8028 pixels. No raw-depth resampling or alternate renderer is used.
The realized vertical FOV exceeds the nominal 58° by approximately 0.065°.

## Actual calibration evidence

`.local/outputs/native_depth_sequence_20260916/calibration_06/` contains the native
capture and independent RGB-axis reference. At the same stationary scene state:

- 420608 valid native depth pixels; 208433 project outside the RGB image.
- Native-depth-to-RGB interior discrepancy: median 0.241 mm, 95th percentile
  3.42 mm. Known floor region: median 3.00 mm, 95th percentile 9.91 mm.
- Maximum discrepancy is 8.05 cm near sampled boundaries; these are not perfect
  per-pixel correspondence or hardware accuracy guarantees.
- Earlier failed initialization and rejected projection candidates are retained
  as `calibration_01` through `calibration_05`; none counted as navigation runs.

Only after this calibration passed was the FOV-only navigation baseline started
at `.local/outputs/native_depth_sequence_20260916/depth_only_01/`. Stopping,
clearance preference, recovery, language and mount-position changes are later
separate phases of the Owner-authorized work.

## Primary references

- [Intel D435i nominal specifications](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
- [D400 datasheet: stream modes and mode-dependent FOV](https://www.intelrealsense.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf)
- [RealSense projection, intrinsics and extrinsics](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)
- [RealSense alignment sampling and occlusion limitations](https://github.com/realsenseai/librealsense/blob/master/examples/align/rs-align.cpp)
- [Isaac Sim 5.1 camera calibration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html)
