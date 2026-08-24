#!/usr/bin/env bash
set -eo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble was not found in /opt/ros/humble" >&2
    exit 1
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "This deployment requires a 64-bit ARM host (aarch64)." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
set -u
cd /workspace

package_paths=(
    src/armpi_fpv_description
    src/armpi_fpv_bringup
    src/armpi_fpv_moveit_config
    src/driver/ros_robot_controller
    src/driver/ros_robot_controller_msgs
    src/driver/servo_controller
    src/driver/servo_controller_msgs
    src/driver/kinematics
    src/driver/kinematics_msgs
)

for package_path in "${package_paths[@]}"; do
    if [[ ! -e "$package_path/package.xml" ]]; then
        echo "Missing package manifest: $package_path/package.xml" >&2
        exit 1
    fi
done

rosdep update
rosdep install \
    --from-paths "${package_paths[@]}" \
    --ignore-src \
    --rosdistro humble \
    --yes

colcon build \
    --symlink-install \
    --packages-up-to armpi_fpv_bringup armpi_fpv_moveit_config

echo
echo "ArmPi-FPV ROS 2 build completed. In each new shell, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspace/install/setup.bash"
