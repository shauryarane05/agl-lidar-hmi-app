#!/bin/sh
# Self-contained AGL LiDAR Demo.
# Runs the on-target perception pipeline against the shipped rosbag and exposes
# it to the Flutter app via a local rosbridge WebSocket server on :9090.
# No live CARLA, no GPU, no network.
#
# Default (DEMO_MODE=live): the detector runs ON the image.
#   The clip replays only the raw sensor streams
#     /carla/lidar                              PointCloud2      (the raw cloud)
#     /carla/hero/rgb_front/image/compressed    CompressedImage  (front camera, JPEG)
#     /carla/hero/rgb_aerial/image/compressed   CompressedImage  (aerial camera, JPEG)
#   and the lidar_detector node consumes /carla/lidar and publishes
#     /carla/detections                         Detection3DArray (live boxes)
#     /carla/detections_markers                 MarkerArray
#   so the boxes the app renders are computed on the target in real time.
#
# DEMO_MODE=replay: pure replay of the whole clip, including the pre-recorded
#   /carla/detections. No perception node runs. Kept as a fallback for images
#   built without the lidar-detector package.
set -e

BAG_DIR="${BAG_DIR:-/usr/share/agl-lidar-demo/rosbag/carla_lidar}"
PLAY_RATE="${PLAY_RATE:-1}"      # clip is recorded in real time; play it back 1:1
DEMO_MODE="${DEMO_MODE:-live}"   # live = on-target detector; replay = pre-recorded boxes

# Source the ROS 2 environment as installed by meta-ros on the AGL image.
# ros-core installs to the system prefix, so `ros2` is usually already on PATH;
# adjust the candidates below if your image differs.
for f in /usr/bin/ros_setup.sh /opt/ros/*/setup.sh /etc/profile.d/ros2.sh; do
    [ -f "$f" ] && . "$f" && break
done

# 1) local rosbridge on :9090 (backgrounded)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 3

if [ "${DEMO_MODE}" = "replay" ]; then
    # Pure replay: cloud + pre-recorded detections + both compressed cameras.
    exec ros2 bag play --loop --rate "${PLAY_RATE}" "${BAG_DIR}"
fi

# 2) on-target perception: detect boxes live from the raw cloud (backgrounded)
ros2 run lidar_detector lidar_detector &
sleep 2

# 3) loop only the RAW sensor streams; detections are produced live above.
#    NOTE: the bag path must come BEFORE --topics, otherwise ros2 bag play's
#    nargs='+' greedily swallows it as a topic name.
exec ros2 bag play --loop --rate "${PLAY_RATE}" "${BAG_DIR}" --topics \
    /carla/lidar \
    /carla/hero/rgb_front/image/compressed \
    /carla/hero/rgb_aerial/image/compressed
