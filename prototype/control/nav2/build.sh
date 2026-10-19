#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
sysroot=${NAV2_SYSROOT:-$repo/.local/nav2_sysroot}
prefix=${NAV2_PREFIX:-$sysroot/opt/ros/jazzy}
export NAV2_SYSROOT="$sysroot" NAV2_PREFIX="$prefix"
# The isolated environment must contain the actual Nav2 controller library.
export CMAKE_PREFIX_PATH="$prefix:$sysroot/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export CMAKE_INCLUDE_PATH="$sysroot/usr/include"
export CMAKE_LIBRARY_PATH="$prefix/lib:$sysroot/usr/lib/x86_64-linux-gnu:$sysroot/usr/lib/llvm-18/lib"
export LD_LIBRARY_PATH="$prefix/lib:$sysroot/usr/lib/x86_64-linux-gnu:$sysroot/usr/lib/llvm-18/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PATH="$prefix/bin:$PATH"
export PYTHONPATH="$prefix/lib/python3.12/site-packages:$sysroot/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
cmake -S "$repo/prototype/control/nav2" -B "$repo/.local/build/nav2_bridge" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$repo/.local/nav2_bridge"
cmake --build "$repo/.local/build/nav2_bridge" -j 4
cmake --install "$repo/.local/build/nav2_bridge"
