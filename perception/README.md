# Perception

A ROS 2 perception node for the LiDAR HMI pipeline. It turns the raw CARLA
LiDAR point cloud into 3D bounding boxes, so the dashboard can show detected
objects instead of only raw points.

## lidar_detector

A small C++ node built on PCL. For each `sensor_msgs/PointCloud2` on
`/carla/lidar` it runs a classical, training-free pipeline:

1. **VoxelGrid** downsample (0.2 m leaf)
2. **PassThrough** crop in z (−2.0 to 3.0 m)
3. **RANSAC** ground-plane segmentation and removal (0.3 m threshold)
4. **Euclidean clustering** (0.6 m tolerance, min 15 points)

Each surviving cluster becomes an axis-aligned 3D box. The node publishes:

- `/carla/detections_markers` — `visualization_msgs/MarkerArray` (for viewers)
- `/carla/detections` — `vision_msgs/Detection3DArray` (for downstream code)

### Build

```sh
# in a ROS 2 Humble workspace, with this folder as a package
colcon build --packages-select lidar_detector
source install/setup.bash
```

### Run (offline, against the recorded bag)

```sh
# terminal 1: the detector
ros2 run lidar_detector lidar_detector

# terminal 2: replay the recorded CARLA LiDAR bag onto /carla/lidar
ros2 bag play rosbag/carla_lidar
```

### Scope

The node detects **clusters**, not classified objects. A box means points
grouped together above the ground, so vehicles, pedestrians, poles, and walls
all come back as boxes. Size filtering and classification are the next steps.

## License

Apache-2.0, matching the rest of the repository.
