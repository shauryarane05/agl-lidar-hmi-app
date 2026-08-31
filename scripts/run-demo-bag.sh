#!/bin/sh
# Self-contained AGL LiDAR Demo.
# Plays the shipped LiDAR rosbag in a loop and exposes it to the Flutter app via
# a local rosbridge WebSocket server on :9090. No live CARLA / no network needed.
#
# If the perception node (lidar_detector) is present on the image it is started
# too, so the app also receives /carla/detections and draws the 3D boxes. When
# the node is not packaged yet the demo still runs as a point-cloud view.
set -e

BAG_DIR="${BAG_DIR:-/usr/share/agl-lidar-demo/rosbag/carla_lidar}"
PLAY_RATE="${PLAY_RATE:-5}"   # bag captured ~0.35 Hz; speed up for a livelier view

# TODO(verify-on-image): source the ROS 2 environment as installed by meta-ros
# on the AGL image. ros-core installs to the system prefix, so `ros2` is usually
# already on PATH; adjust the candidates below if your image differs.
for f in /usr/bin/ros_setup.sh /opt/ros/*/setup.sh /etc/profile.d/ros2.sh; do
    [ -f "$f" ] && . "$f" && break
done

# 1) local rosbridge on :9090 (backgrounded)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 3

# 2) perception node, if it is on the image: /carla/lidar -> /carla/detections
if ros2 pkg executables lidar_detector 2>/dev/null | grep -q lidar_detector; then
    echo "run-demo-bag: starting lidar_detector (clustering -> /carla/detections)"
    ros2 run lidar_detector lidar_detector &
    sleep 2
else
    echo "run-demo-bag: lidar_detector not installed - point-cloud view only"
fi

# 3) loop the recorded cloud into /carla/lidar
exec ros2 bag play --loop --rate "${PLAY_RATE}" "${BAG_DIR}"
