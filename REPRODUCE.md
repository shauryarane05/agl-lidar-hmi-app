# Reproduction guide: AGL LiDAR Perception Demo

This guide takes you from nothing to the demo running on a Jetson Orin Nano:
a recorded CARLA LiDAR clip turned into 3D object detections on the board and
drawn in the AGL Flutter dashboard, fully offline. Follow it top to bottom.

If you only want to try the perception pipeline on a Linux desktop (no board),
jump to [Run it without a board](#appendix-run-it-without-a-board).

## 1. What you need

Hardware:

- An **NVIDIA Jetson Orin Nano Developer Kit** with an **NVMe** SSD installed.
- A **USB-C cable** with data lines (not charge-only) for flashing.
- A **DisplayPort** monitor and an **Ethernet** cable.
- A USB-C PD power supply (the Orin Nano needs a negotiating PD supply or a
  barrel-jack supply, not a dumb 5 V brick).

Host (the machine you build on):

- A **Linux** workstation, x86_64, with about **100 GB free disk** and plenty of
  RAM. A full image build takes several hours the first time.
- Standard Yocto host packages (`git`, `python3`, `gawk`, `wget`, `diffstat`,
  `chrpath`, `cpio`, `zstd`, `lz4`, etc.) and Google's `repo` tool.

Note on host Python: if your host Python is newer than the Yocto release
supports, `bitbake` will refuse to run. Source the Yocto buildtools tarball
(a self-contained toolchain that runs from your home directory, no root needed)
before building, and the sanity check passes.

## 2. Get the AGL source

```sh
mkdir -p ~/AGL && cd ~/AGL
repo init -b master -u https://gerrit.automotivelinux.org/gerrit/AGL/AGL-repo
repo sync -j"$(nproc)"
```

This pulls `meta-agl`, `meta-agl-devel`, `meta-ros`, `meta-tegra`, and the rest.

## 3. Apply the demo changes

The application and the on-target perception are already **merged** into
`meta-agl-devel`:

- agl-lidar-demo application: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/31990
- offline rosbag autostart: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32069
- lidar-detector recipe: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32137
- detect on-target: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32138

The **Jetson board support** and a few **build fixes** are still under review. A
clean `master` tree needs them until they merge. Apply each one with the
`Cherry Pick` command from its Gerrit page (open the change, the three-dot menu,
`Download`, `Cherry Pick`), in the right layer:

In `meta-agl`:

- jetson-orin-nano-devkit-nvme machine template: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl/+/32153
- static GID for the debug group (build unblock): https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl/+/32152

In `meta-agl-devel`:

- packagegroup line continuation: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32155
- meta-python layer dependency: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32156
- rosbridge-suite SPDX license: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32177
- rosbag2-py SOABI fix: https://gerrit.automotivelinux.org/gerrit/c/AGL/meta-agl-devel/+/32178

Once these are merged, this whole step goes away and a plain `repo sync` is
enough.

## 4. Configure and build the image

From the top of the tree:

```sh
source meta-agl/scripts/aglsetup.sh \
    -m jetson-orin-nano-devkit-nvme -b build-jetson-ivi \
    agl-demo agl-flutter agl-ros2
bitbake agl-ivi-demo-flutter
```

- `agl-demo` pulls the IVI demo platform, `agl-flutter` the Flutter stack, and
  `agl-ros2` the ROS 2 layers (ROS 2 Humble through meta-ros).
- Run the build inside `tmux` or `screen` so it survives a dropped SSH session.

The output lands in
`build-jetson-ivi/tmp/deploy/images/jetson-orin-nano-devkit-nvme/`.

## 5. Flash the board

Put the Jetson into **Force Recovery Mode** (jumper the recovery pins on the
button header, then apply power; it enumerates on the host as
`0955 NVIDIA Corp. APX`). Then, from the image deploy directory:

```sh
cd build-jetson-ivi/tmp/deploy/images/jetson-orin-nano-devkit-nvme/
sudo ./initrd-flash --erase-nvme
```

Wait for `Final status: SUCCESS`, then let the board cold-boot from NVMe.

## 6. First boot and running the demo

The backend starts itself on boot under one systemd service,
`agl-lidar-demo.service`: a local `rosbridge_server` on port 9090, the
`lidar_detector` node, and the bag replay. No shell step is needed for the
backend.

To put the dashboard on screen, open the **AGL LiDAR Demo** tile on the home
screen. If there is no visible pointer, launch it from a shell over SSH or the
serial console:

```sh
agl-shell-activator -a activate -p agl_lidar_demo -o DP-1
```

(`DP-1` is the connected DisplayPort output on this board; `-o` is required.)

## 7. Verify it is live

The detections are computed on the device, not replayed from the clip:

```sh
# source the ROS 2 environment first, e.g. . /opt/ros/humble/setup.sh

ros2 topic hz /carla/detections                # expect ~30 Hz
ros2 topic info /carla/detections --verbose     # Publisher node name: lidar_detector
```

If the publisher of `/carla/detections` is `lidar_detector` (not the bag), the
perception is running on the target.

## 8. Configuration

The demo launcher reads these environment variables (set them in the service or
the environment before it starts):

| Variable    | Default                                        | Meaning |
| ----------- | ---------------------------------------------- | ------- |
| `DEMO_MODE` | `live`                                         | `live` runs the detector on the raw streams; `replay` plays the clip's pre-recorded detections and runs no detector |
| `BAG_DIR`   | `/usr/share/agl-lidar-demo/rosbag/carla_lidar` | path to the recorded clip |
| `PLAY_RATE` | `1`                                            | playback rate (the clip is recorded in real time) |

The Flutter app reads these build-time defines (`--dart-define`):

| Define          | Default               | Meaning |
| --------------- | --------------------- | ------- |
| `ROSBRIDGE_URL` | `ws://127.0.0.1:9090` | rosbridge endpoint |
| `SHOW_CAMERAS`  | `true`                | show or hide the camera column |

## 9. Troubleshooting

- **Backend restarts in a loop, `ros2 bag play` says `No module named
  'rosbag2_py._reader'`.** The rosbag2 Python extensions were named with the
  build host architecture. The fix is change 32178 (step 3); if you are on an
  image without it, symlink the extensions to the target SOABI as a stopgap:
  ```sh
  cd /opt/ros/humble/lib/python3.*/site-packages/rosbag2_py
  for f in _info _reader _reindexer _storage _transport _writer; do
    ln -sf ${f}.cpython-*-x86_64-linux-gnu.so ${f}.cpython-*-aarch64-linux-gnu.so
  done
  ```
- **The image build stops around 77 percent on an SPDX error.** The
  rosbridge-suite recipes declared an invalid license; apply change 32177.
- **The HMI dashboard draws in a narrow window with the home screen beside it.**
  The home screen owns the AGL shell background on this image; drive the demo
  through the shell activator (step 6). A full-screen kiosk layout is a board
  configuration change, not a runtime one.
- **Verifying topics prints nothing / `ros2` not found.** Source the ROS 2
  environment first (`. /opt/ros/humble/setup.sh`).

## 10. Repository map

- `lib/main.dart` - the Flutter HMI.
- `perception/lidar_detector/` - the PCL perception node.
- `rosbag/carla_lidar/` - the recorded clip (`metadata.yaml`, `demo_clip_0.db3`).
- `scripts/run-demo-bag.sh` - the offline launcher (rosbridge + bag + detector).

In the AGL tree, the recipes live in
`meta-agl-devel/meta-agl-ros2/recipes-ros2/` (`lidar-detector/` and
`agl_lidar_demo/`).

## Appendix: run it without a board

You can run the perception and the offline replay on any Linux desktop with ROS
2 Humble, without building an image or touching the Jetson.

```sh
# build the perception node
colcon build --packages-select lidar_detector
. install/setup.bash

# start rosbridge + loop the clip + run the detector
BAG_DIR=./rosbag/carla_lidar ./scripts/run-demo-bag.sh
```

To iterate on the app itself, hot-reload it onto a running AGL target (QEMU or
the board) and point it at the host rosbridge (the QEMU guest reaches the host
at `10.0.2.2`):

```sh
flutter run -d agl-qemu --dart-define=ROSBRIDGE_URL=ws://10.0.2.2:9090
```

## Links

- Application repository: https://github.com/shauryarane05/agl-lidar-hmi-app
- All Gerrit changes: https://gerrit.automotivelinux.org/gerrit/q/owner:ssranevjti@gmail.com
- Project blog: https://shauryarane05.github.io/agl-gsoc-2026-blog/
- Final report: https://shauryarane05.github.io/agl-gsoc-2026-blog/final-report.html
