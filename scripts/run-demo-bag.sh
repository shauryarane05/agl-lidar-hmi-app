#!/bin/sh
# Self-contained AGL LiDAR HMI demo.
# Plays the shipped LiDAR rosbag in a loop and exposes it to the Flutter app via
# a local rosbridge WebSocket server on :9090. No live CARLA / no network needed.
set -e

BAG_DIR="${BAG_DIR:-/usr/share/agl-lidar-hmi/rosbag/carla_lidar}"
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

# 2) loop the recorded cloud into /carla/lidar
exec ros2 bag play --loop --rate "${PLAY_RATE}" "${BAG_DIR}"
