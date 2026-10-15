#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
# source ~/microros_ws/install/setup.bash   # uncomment if you built the agent from source

PORT=${1:-/dev/ttyUSB0}; BAUD=${2:-460800}
cd "$(dirname "$0")"

trap 'kill 0' EXIT

ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b "$BAUD" &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run twist_mux twist_mux --ros-args --params-file twist_mux_config.yaml -r cmd_vel_out:=/cmd_vel_raw &
python3 -m http.server 8000 &

wait