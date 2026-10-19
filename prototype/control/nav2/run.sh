#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
sysroot=${NAV2_SYSROOT:-$repo/.local/nav2_sysroot}
prefix=${NAV2_PREFIX:-$sysroot/opt/ros/jazzy}
export AMENT_PREFIX_PATH="$repo/.local/nav2_bridge:$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$repo/.local/nav2_bridge/lib:$prefix/lib:$sysroot/usr/lib/x86_64-linux-gnu:$sysroot/usr/lib/llvm-18/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RCUTILS_LOGGING_USE_STDOUT=0
export ROS_LOCALHOST_ONLY=1
exec "$repo/.local/nav2_bridge/lib/guidepup_nav2_bridge/guidepup_mppi_bridge"
