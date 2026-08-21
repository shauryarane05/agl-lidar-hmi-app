# AGL LiDAR HMI

A Flutter human machine interface (HMI) that renders a live top-down LiDAR /
perception view for Automotive Grade Linux (AGL). It subscribes to a ROS 2
`sensor_msgs/msg/PointCloud2` stream over a rosbridge WebSocket and draws the
point cloud in real time.

The repository ships a recorded CARLA LiDAR rosbag and a helper script so the
whole thing runs fully offline, with no live CARLA instance and no network.

## Architecture

```
CARLA LiDAR  ->  ROS 2 PointCloud2  ->  rosbridge (WebSocket :9090)  ->  Flutter HMI
   (recorded into the shipped rosbag)                                    (this app)
```

- App name / bundle id: `agl_hmi_demo`
- rosbridge endpoint: `ws://127.0.0.1:9090` (see `kRosbridgeUrl` in `lib/main.dart`)
- Topic: `/carla/lidar`, `PointCloud2` with `x, y, z, intensity`
- Range shown: 50 m (matches the CARLA sensor configuration)

## Offline demo

`scripts/run-demo-bag.sh` starts a local `rosbridge_server` on port 9090 and
plays the bundled rosbag in a loop, publishing `/carla/lidar`. Launch the
Flutter app and it connects to the local rosbridge automatically.

```sh
# On the AGL image (installed by the recipe):
agl-lidar-hmi-demo.sh

# Or from a checkout, pointing at the local bag:
BAG_DIR=./rosbag/carla_lidar ./scripts/run-demo-bag.sh
```

Environment overrides:

- `BAG_DIR` path to the rosbag directory (default
  `/usr/share/agl-lidar-hmi/rosbag/carla_lidar`)
- `PLAY_RATE` playback rate multiplier (default `5`; the bag was captured at a
  low frame rate, so playing it faster gives a livelier view)

## Contents

- `lib/` Flutter application source
- `rosbag/carla_lidar/` recorded LiDAR bag (`metadata.yaml`, `carla_lidar_0.db3`)
- `scripts/run-demo-bag.sh` offline demo launcher (loop play plus local rosbridge)

## Packaging for AGL

This app is packaged for AGL with a BitBake recipe that uses
`inherit flutter-app agl-app`. The recipe installs the Flutter bundle, copies
the rosbag to `/usr/share/agl-lidar-hmi/rosbag`, and installs the demo launcher
as `/usr/bin/agl-lidar-hmi-demo.sh`. Runtime dependencies include
`flutter-auto`, `rosbridge-server`, `ros2bag`, and the ROS 2 CLI tools.

## License

Apache-2.0. See [LICENSE](LICENSE).

## Author

Shaurya Rane <ssrane_b23@ee.vjti.ac.in>
