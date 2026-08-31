# AGL LiDAR Demo

A Flutter human machine interface (HMI) that renders a live top-down LiDAR /
perception view for Automotive Grade Linux (AGL). It subscribes to a ROS 2
`sensor_msgs/msg/PointCloud2` stream over a rosbridge WebSocket, draws the point
cloud in real time, and overlays the 3D bounding boxes produced by a perception
node.

The repository ships a recorded CARLA LiDAR rosbag and a helper script so the
whole thing runs fully offline, with no live CARLA instance and no network.

## Architecture

```
CARLA LiDAR ->  ROS 2 PointCloud2  ->  lidar_detector  ->  Detection3DArray  -.
  (recorded    (/carla/lidar)          (PCL clustering)    (/carla/detections)  \
   into the                                                                       rosbridge :9090 -> Flutter HMI
   shipped bag) --------------------------------------------------------------- /                    (this app)
```

The HMI subscribes to two topics and draws both layers:

- `/carla/lidar`, `PointCloud2` (`x, y, z, intensity`) - the raw cloud.
- `/carla/detections`, `vision_msgs/msg/Detection3DArray` - per-object 3D boxes.

`/carla/detections` is the perception contract: the classical clustering node
publishes it today, and a learned detector (for example PointPillars) can
publish the same message later, so the HMI does not change when the perception
backend is upgraded.

- App name / bundle id: `agl_lidar_demo`
- rosbridge endpoint: `ws://127.0.0.1:9090` (see `kRosbridgeUrl` in `lib/main.dart`)
- Range shown: 50 m (matches the CARLA sensor configuration)

## Perception node

`perception/lidar_detector/` is a C++ `ament_cmake` ROS 2 package. It subscribes
to `/carla/lidar` and runs a classic PCL pipeline - `VoxelGrid` downsample ->
`PassThrough` z-crop -> RANSAC ground-plane removal -> Euclidean clustering ->
axis-aligned 3D boxes - publishing `vision_msgs/msg/Detection3DArray` on
`/carla/detections` (and CUBE markers on `/carla/detections_markers`).

Run it on any ROS 2 host feeding the same rosbridge:

```sh
colcon build --packages-select lidar_detector
. install/setup.bash
ros2 run lidar_detector lidar_detector
```

## Offline demo

`scripts/run-demo-bag.sh` starts a local `rosbridge_server` on port 9090, plays
the bundled rosbag in a loop (publishing `/carla/lidar`), and - if the
`lidar_detector` node is installed on the image - starts it as well so
`/carla/detections` is published too. Launch the Flutter app and it connects to
the local rosbridge automatically.

```sh
# On the AGL image (installed by the recipe):
agl-lidar-demo.sh

# Or from a checkout, pointing at the local bag:
BAG_DIR=./rosbag/carla_lidar ./scripts/run-demo-bag.sh
```

Environment overrides:

- `BAG_DIR` path to the rosbag directory (default
  `/usr/share/agl-lidar-demo/rosbag/carla_lidar`)
- `PLAY_RATE` playback rate multiplier (default `5`; the bag was captured at a
  low frame rate, so playing it faster gives a livelier view)

## Contents

- `lib/` Flutter application source
- `perception/lidar_detector/` PCL clustering node (point cloud -> 3D boxes)
- `rosbag/carla_lidar/` recorded LiDAR bag (`metadata.yaml`, `carla_lidar_0.db3`)
- `scripts/run-demo-bag.sh` offline demo launcher (loop play plus local rosbridge)

## Packaging for AGL

This app is packaged for AGL with a BitBake recipe that uses
`inherit flutter-app agl-app`. The recipe installs the Flutter bundle, copies
the rosbag to `/usr/share/agl-lidar-demo/rosbag`, and installs the demo launcher
as `/usr/bin/agl-lidar-demo.sh`. Runtime dependencies include `flutter-auto`,
`rosbridge-server`, `ros2bag`, and the ROS 2 CLI tools.

## License

Apache-2.0. See [LICENSE](LICENSE).

## Author

Shaurya Rane <ssrane_b23@ee.vjti.ac.in>
