#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
# source ~/microros_ws/install/setup.bash   # only if you built the agent from source
PORT=${1:-/dev/ttyUSB0}; BAUD=${2:-115200}
cd "$(dirname "$0")"
trap 'kill 0' EXIT
ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b "$BAUD" &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
python3 -m http.server 8000 &
wait