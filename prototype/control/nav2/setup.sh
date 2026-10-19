#!/usr/bin/env bash
# Install the official ROS 2 Jazzy binary packages into a relocatable, rootless tree.
set -euo pipefail

project_root=$(cd "$(dirname "$0")/../../.." && pwd)
sysroot="$project_root/.local/nav2_sysroot"
ros_prefix="$sysroot/opt/ros/jazzy"
sources="$sysroot/meta/sources.list"
lists="$sysroot/meta/apt-lists"
archives="$sysroot/debs"

unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY='*'
export no_proxy='*'
install -d "$lists/partial" "$archives/partial" "$sysroot/staging"
cat > "$sources" <<'SOURCES'
deb [trusted=yes] http://packages.ros.org/ros2/ubuntu noble main
deb [trusted=yes] http://archive.ubuntu.com/ubuntu noble main universe
deb [trusted=yes] http://archive.ubuntu.com/ubuntu noble-updates main universe
SOURCES

apt_common=(
  -o "Dir::Etc::sourcelist=$sources"
  -o 'Dir::Etc::sourceparts=-'
  -o "Dir::State::lists=$lists"
  -o "Dir::Cache::archives=$archives"
  -o 'Dir::State::status=/var/lib/dpkg/status'
  -o 'Acquire::http::Proxy=false'
  -o 'Acquire::https::Proxy=false'
  -o 'Debug::NoLocking=true'
)

apt-get "${apt_common[@]}" update
apt-get "${apt_common[@]}" --download-only --yes --no-install-recommends install \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-rclcpp \
  libboost-dev \
  nlohmann-json3-dev

while IFS= read -r -d '' deb; do
  dpkg-deb --extract "$deb" "$sysroot"
done < <(find "$archives" -maxdepth 1 -type f -name '*.deb' -print0 | sort -z)

while IFS= read -r -d '' cmake_file; do
  NAV2_RELOCATED_PREFIX="$ros_prefix" perl -0pi -e \
    's{(?<![A-Za-z0-9_./-])/opt/ros/jazzy}{$ENV{NAV2_RELOCATED_PREFIX}}g' \
    "$cmake_file"
done < <(find "$ros_prefix" -type f -name '*.cmake' -print0)

test -f "$ros_prefix/include/nav2_mppi_controller/optimizer.hpp"
test -f "$ros_prefix/include/nav2_mppi_controller/critic_function.hpp"
test -e "$ros_prefix/lib/libmppi_controller.so"
test -f "$ros_prefix/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp"
test -e "$ros_prefix/lib/libnav2_costmap_2d_core.so"
test -e "$ros_prefix/lib/librclcpp.so"
test -f "$sysroot/usr/include/nlohmann/json.hpp"

printf 'ROS_PREFIX=%s\nSYSROOT=%s\n' "$ros_prefix" "$sysroot"
