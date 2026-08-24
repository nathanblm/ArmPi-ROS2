# ROS 2 Humble container on the Hiwonder Raspberry Pi image

The official Hiwonder image contains a vendor ROS 1 container. Keep that
container intact and run this repository in the separate ROS 2 Humble
container defined by [`compose.yaml`](../compose.yaml).

The ROS 2 container is deliberately idle after startup. It does not launch the
hardware stack or move a servo automatically.

## Prerequisites

The Raspberry Pi host and Docker engine must be 64-bit ARM:

```bash
uname -m
getconf LONG_BIT
docker info --format '{{.Architecture}}'
```

The expected results are `aarch64`, `64`, and `aarch64`. A 32-bit host cannot
load the included analytic-kinematics extensions.

Check which devices represent the STM32 UART and USB camera:

```bash
ls -l /dev/ttyAMA0 /dev/serial0 /dev/video0
readlink -f /dev/serial0
v4l2-ctl --list-devices
```

Copy the tracked environment template and edit it if the resolved devices are
different:

```bash
cp .env.example .env
```

The local `.env` file is ignored by Git. `.env.example` remains the documented
default.

## Keep ROS 1 away from the hardware

Before starting ROS 2, stop Hiwonder's ROS 1 startup service and vendor
container. Substitute the actual container name reported by `docker ps -a`:

```bash
sudo systemctl stop start_node.service
docker stop <hiwonder-container>
sudo fuser -v /dev/ttyAMA0 /dev/video0
```

Do not continue if another process still owns either device. Do not delete the
vendor container; it can be restarted if the original software is needed.

## Build and enter the ROS 2 container

From the repository root:

```bash
docker compose build
docker compose up -d
docker compose exec armpi_ros2 ./docker/bootstrap.sh
docker compose exec armpi_ros2 bash
```

The bootstrap script installs dependencies for only the base ArmPi-FPV
migration and builds through `armpi_fpv_bringup` and
`armpi_fpv_moveit_config`. It intentionally excludes the optional vendor voice
package.

In every new container shell:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

Verify the runtime before opening the serial device:

```bash
uname -m
python3 --version
echo "$ROS_DISTRO"
ls -l "$ARMPI_SERIAL_DEVICE" "$ARMPI_CAMERA_DEVICE"
```

The expected platform is `aarch64`, Python 3.10, and ROS distribution
`humble`.

## First hardware launch

Keep the camera and analytic kinematics disabled for the initial serial and
single-servo tests:

```bash
ros2 launch armpi_fpv_bringup hardware.launch.py \
  serial_port:="$ARMPI_SERIAL_DEVICE" \
  use_camera:=false \
  use_kinematics:=false
```

Follow the staged physical test procedure before enabling more components.
Keep a physical servo-power disconnect within reach and do not place hands in
the arm's workspace.

After the UART and individual servo mappings have been validated:

```bash
ros2 launch armpi_fpv_bringup hardware.launch.py \
  serial_port:="$ARMPI_SERIAL_DEVICE" \
  camera_device:="$ARMPI_CAMERA_DEVICE" \
  use_camera:=true \
  use_kinematics:=true
```

## Keyboard joint commissioning

With the hardware launch running in one terminal, open a second container
shell and start the keyboard jogger:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 run servo_controller keyboard_jog
```

The jogger talks directly to the bus-servo driver, so do not run MoveIt or an
ArmPi application at the same time. Press `h` to move slowly to the stock
ArmPi-FPV home pose and unlock jogging. Select a servo with `1` through `6`,
then use `a` and `d` to move it. Use `-` and `+` to change the step size.
Press Space to stop motion, and `q` to stop and exit.

Servo position feedback is unavailable on the stock ArmPi-FPV configuration.
The displayed positions are commanded estimates, not measurements. Keep the
workspace clear and a physical servo-power disconnect within reach.

## Container lifecycle

Stop the ROS 2 container without deleting its persistent build volumes:

```bash
docker compose down
```

Start it again with:

```bash
docker compose up -d
```

Do not use `docker compose down --volumes` unless the build, install, log, and
ROS log volumes should be erased. Automatic hardware bringup and automatic
container restart should remain disabled during commissioning.
