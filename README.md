# ArmPi-FPV ROS 2

This repository is an incremental ROS 2 Humble port of the Hiwonder
[ArmPi-FPV](https://github.com/Hiwonder/ArmPi-FPV), using Hiwonder's
[JetArm ROS 2 branch](https://github.com/Hiwonder/JetArm/tree/Jetson_nano_ros2)
as the driver and application baseline.

The first migration slice provides:

- ArmPi-FPV servo mapping (`joint1..joint5` use bus IDs `6..2`; the gripper uses ID `1`)
- a configurable Raspberry Pi/STM32 serial driver (default `/dev/ttyAMA0`)
- ROS 2 `JointState` and `FollowJointTrajectory` interfaces
- the original ArmPi-FPV URDF and meshes
- ArmPi-FPV link dimensions and joint pulse conversion
- a USB camera path compatible with the existing ROS 2 vision topic name
- a MoveIt 2 configuration for the arm and gripper controllers
- one hardware launch entry point with no machine-specific home-directory paths

## Target platform

- Raspberry Pi 4 or Raspberry Pi 5, 64-bit
- Ubuntu 22.04
- ROS 2 Humble
- Hiwonder STM32 robot controller on `/dev/ttyAMA0`
- ArmPi-FPV USB camera on `/dev/video0`

The included analytic-kinematics extensions are AArch64 binaries. The workspace
can be inspected and most packages can be built on x86_64, but the `kinematics`
node must run on the 64-bit Raspberry Pi target.

## Build

Install ROS 2 Humble, MoveIt 2, `usb_cam`, and the normal colcon tooling, then:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to \
  armpi_fpv_bringup armpi_fpv_moveit_config
source install/setup.bash
```

The optional `xf_mic_asr_offline` package contains vendor AArch64 libraries and
will not link on x86_64. It is not required for the base arm.

## Run

Before enabling motion, place the arm in a clear workspace and be ready to
disconnect servo power.

Start the base arm and camera:

```bash
ros2 launch armpi_fpv_bringup hardware.launch.py
```

Override devices when needed:

```bash
ros2 launch armpi_fpv_bringup hardware.launch.py \
  serial_port:=/dev/ttyAMA0 camera_device:=/dev/video0
```

Start MoveIt 2 against the physical arm:

```bash
ros2 launch armpi_fpv_moveit_config moveit.launch.py
```

Start MoveIt 2 without opening the serial port, using a mock joint-state source:

```bash
ros2 launch armpi_fpv_moveit_config moveit.launch.py use_hardware:=false
```

Useful launch options:

| Argument | Default | Purpose |
| --- | --- | --- |
| `serial_port` | `/dev/ttyAMA0` | STM32 serial device |
| `camera_device` | `/dev/video0` | USB camera device |
| `use_hardware` | `true` | Open the serial controller and start servo control |
| `use_camera` | `true` | Start `usb_cam` |
| `use_kinematics` | `true` | Start Hiwonder analytic kinematics |
| `use_web_video` | `false` | Start `web_video_server` |

## Raspberry Pi serial setup

The UART must be enabled and the Linux serial console must not own the selected
device. The user running ROS also needs permission to access it (normally by
membership in `dialout`). Verify the resolved device before powering the servos:

```bash
ls -l /dev/ttyAMA0
groups
```

## Camera compatibility

ArmPi-FPV has a 2D USB camera, not JetArm's depth camera. To minimize churn in
the already-ported ROS 2 vision nodes, bringup publishes the RGB image as
`/depth_cam/rgb/image_raw` and camera calibration as
`/depth_cam/depth/camera_info`. No depth image is produced. Applications that
require `/depth_cam/depth/image_raw` are not compatible with stock ArmPi-FPV
hardware.

## Migration status

See [docs/MIGRATION.md](docs/MIGRATION.md) for package-by-package status and the
next safe porting steps. The legacy JetArm application tree remains available as
reference code, but it is intentionally excluded from default bringup until its
raw servo commands and depth-camera assumptions are removed.
