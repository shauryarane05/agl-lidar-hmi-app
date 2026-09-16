#!/bin/sh
# Self-contained AGL LiDAR Demo.
# Plays the shipped rosbag in a loop and exposes it to the Flutter app via a
# local rosbridge WebSocket server on :9090. No live CARLA, no GPU, no network.
#
# The shipped clip already contains everything the app renders:
#   /carla/lidar                              PointCloud2   (the raw cloud)
#   /carla/detections                         Detection3DArray (pre-computed boxes)
#   /carla/hero/rgb_front/image/compressed    CompressedImage  (front camera, JPEG)
#   /carla/hero/rgb_aerial/image/compressed   CompressedImage  (aerial camera, JPEG)
# so no perception node and no image compressor need to run on the target -- the
# demo is a pure replay. (Embedding the detector on-image is future work; until
# then the detections are replayed from the recording.)
set -e

BAG_DIR="${BAG_DIR:-/usr/share/agl-lidar-demo/rosbag/carla_lidar}"
PLAY_RATE="${PLAY_RATE:-1}"   # clip is recorded in real time; play it back 1:1

# Source the ROS 2 environment as installed by meta-ros on the AGL image.
# ros-core installs to the system prefix, so `ros2` is usually already on PATH;
# adjust the candidates below if your image differs.
for f in /usr/bin/ros_setup.sh /opt/ros/*/setup.sh /etc/profile.d/ros2.sh; do
    [ -f "$f" ] && . "$f" && break
done

# 1) local rosbridge on :9090 (backgrounded)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 3

# 2) loop the recorded clip: cloud + detections + both compressed cameras
exec ros2 bag play --loop --rate "${PLAY_RATE}" "${BAG_DIR}"
