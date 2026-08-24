# ArmPi-FPV ROS 2 migration

## Source-to-target map

| ArmPi-FPV ROS 1 | ROS 2 target | Status |
| --- | --- | --- |
| `ros_robot_controller` | `ros_robot_controller` | Ported; serial device is a launch parameter |
| `hiwonder_servo_*` | `servo_controller*` and `ros_robot_controller_msgs` | ArmPi servo map applied; trajectory action repaired |
| `armpi_fpv_descrption` | `armpi_fpv_description` | Ported to `ament_cmake`; original URDF/meshes retained |
| `armpi_fpv_kinematics` | `kinematics*` | ArmPi geometry and servo order applied; AArch64 runtime validation remains |
| `armpi_fpv_bringup` | `armpi_fpv_bringup` | Initial hardware/model/USB-camera launch complete |
| `armpi_fpv_moveit_config` | `armpi_fpv_moveit_config` | Initial MoveIt 2/OMPL/controller configuration complete |
| `lab_config` | `app/lab_manager` | JetArm ROS 2 code exists; ArmPi calibration workflow pending |
| `object_tracking` | `app/object_tracking` | ROS 2 code exists; raw servo IDs and calibration still JetArm-specific |
| `object_sorting` | `app/object_sortting` | ROS 2 code exists; rename, servo abstraction, and ArmPi calibration pending |
| `object_pallezting` | no safe default node | Pending |
| `warehouse` | no safe default node | Pending |
| `face_detect` | `app/face_tracker` | ROS 2 baseline exists; ArmPi validation pending |
| `asr_control` | `xf_mic_asr_offline` | Optional vendor/AArch64 component; excluded from base build |

## Raspberry Pi container deployment

The official Hiwonder ROS 1 container is retained unchanged. A separate ARM64
Ubuntu 22.04/ROS 2 Humble image, Compose configuration, and bootstrap script
are provided for this migration. See [DOCKER.md](DOCKER.md) for device
selection, ROS 1 shutdown, build, and commissioning instructions.

## Hardware differences that must remain explicit

1. ArmPi bus IDs are `joint1=6`, `joint2=5`, `joint3=4`, `joint4=3`,
   `joint5=2`, `r_joint=1`. JetArm code commonly assumes `1..5` and gripper
   `10`.
2. ArmPi link lengths are `0.064605`, `0.10048`, `0.094714`, `0.05071`, and
   `0.1126` metres for the base/link/tool model used by analytic kinematics.
3. ArmPi joint 3 uses the opposite pulse orientation from the imported JetArm
   controller configuration.
4. ArmPi supplies RGB only. JetArm point-cloud, ranging, and 3D grasp nodes need
   a real depth camera or a separate redesign.

## Next implementation slices

1. Validate servo direction, limits, home pose, and emergency-stop procedure on
   an unladen ArmPi at reduced velocity.
2. Calibrate the USB camera and commit the resulting ROS 2 camera-info YAML.
3. Replace raw servo-ID tuples in 2D applications with logical joint or
   `FollowJointTrajectory` commands.
4. Port and validate color tracking, sorting, palletizing, then warehouse flows
   one at a time with recorded image tests before hardware motion tests.
5. Replace the vendor-only analytic kinematics binaries with a source-buildable
   implementation or use MoveIt 2 as the only kinematics backend.
6. Treat voice control and JetArm depth applications as optional packages so
   they never break the base ArmPi build.
