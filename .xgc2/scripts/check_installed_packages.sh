#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"
test -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_vrpn_server_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_vrpn_system_plugin.so"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/mocap_noise.h"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/measurement_delay.h"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/wire_timestamp.h"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_delay_simple.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_delay_complex.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_hybrid.yaml"
grep -A3 '^delay:' "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_hybrid.yaml" \
  | grep -q 'timestamp_policy: sample_time'

test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/native_vrpn_extension.h"
test ! -e "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/launch/vrpn_server.launch"
roslaunch --files gazebo_sim_worlds native_world.launch world:=/explicit/prepared.world paused:=true gui:=false ros_data:=false vrpn_config:=/explicit/vrpn.yaml >/dev/null

echo "Installed package check passed"
