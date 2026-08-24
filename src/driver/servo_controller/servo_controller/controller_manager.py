#!/usr/bin/env python3
"""ROS 2 controllers for the ArmPi-FPV serial bus servos."""

import math

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from servo_controller.joint_position_controller import JointPositionController
from servo_controller.joint_trajectory_action_controller import (
    JointTrajectoryActionController,
)
from servo_controller.servo_controller import ServoManager, ServoState as TrackedServoState
from servo_controller_msgs.msg import (
    ServoPosition,
    ServosPosition,
    ServoState as ServoStateMessage,
    ServoStateList,
)


class ControllerManager(Node):
    def __init__(self, name):
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'r_joint',
        ]
        self.base_frame = self.get_parameter('base_frame').value

        self.controllers = {}
        servos = {}
        for joint_name in self.joints:
            joint_config = self.get_parameters_by_prefix(joint_name)
            controller = JointPositionController(joint_config, joint_name)
            self.controllers[joint_name] = controller
            servos[str(controller.servo_id)] = TrackedServoState(
                joint_name,
                controller.initial_position_raw,
            )

        self.servo_manager = ServoManager(self, servos)

        for controller_name in ['arm_controller', 'gripper_controller']:
            controller_config = self.get_parameters_by_prefix(controller_name)
            joint_controllers = [
                self.controllers[joint_name]
                for joint_name in controller_config['joint_controllers'].value
            ]
            self.controllers[controller_name] = JointTrajectoryActionController(
                self,
                self.servo_manager,
                controller_name,
                joint_controllers,
            )

        # /joint_states is the standard input for robot_state_publisher and MoveIt 2.
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.servo_states_pub = self.create_publisher(ServoStateList, '~/servo_states', 10)
        self.create_subscription(
            ServosPosition,
            'servo_controller',
            self.servo_controller_callback,
            10,
        )
        self.create_subscription(
            JointState,
            'joint_controller',
            self.joint_controller_callback,
            10,
        )

        namespace = '' if self.get_namespace() == '/' else self.get_namespace()
        self.hardware_client = self.create_client(
            Trigger,
            namespace + '/ros_robot_controller/init_finish',
        )
        if not self.hardware_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning(
                'ros_robot_controller is not ready; servo commands will be queued by DDS'
            )

        self.create_timer(0.02, self.publish_joint_states)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('ArmPi-FPV servo controller started')

    def get_node_state(self, request, response):
        del request
        response.success = True
        return response

    def servo_controller_callback(self, msg):
        commands = []
        positions = self.servo_manager.get_position()

        for incoming in msg.position:
            servo_id = str(incoming.id)
            if servo_id not in positions:
                continue

            command = ServoPosition()
            command.id = incoming.id
            controller = self.controllers[positions[servo_id].name]

            if msg.position_unit == 'pulse':
                command.position = incoming.position
            elif msg.position_unit == 'rad':
                command.position = float(controller.pos_rad_to_pulse(incoming.position))
            elif msg.position_unit == 'deg':
                command.position = float(
                    controller.pos_rad_to_pulse(math.radians(incoming.position))
                )
            else:
                self.get_logger().warning(
                    f'Unsupported servo position unit: {msg.position_unit!r}'
                )
                return
            commands.append(command)

        self.servo_manager.set_position(msg.duration, commands)

    def joint_controller_callback(self, msg):
        commands = []
        for joint_name, position in zip(msg.name, msg.position):
            controller = self.controllers.get(joint_name)
            if not isinstance(controller, JointPositionController):
                continue

            command = ServoPosition()
            command.id = controller.servo_id
            command.position = float(controller.pos_rad_to_pulse(position))
            commands.append(command)

        self.servo_manager.set_position(0.02, commands)

    def publish_joint_states(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = self.base_frame

        servo_msg = ServoStateList()
        servo_msg.header = joint_msg.header
        positions = self.servo_manager.get_position()

        for joint_name in self.joints:
            controller = self.controllers[joint_name]
            state = positions[str(controller.servo_id)]
            joint_msg.name.append(joint_name)
            joint_msg.position.append(controller.pos_pulse_to_rad(state.position))

            state_msg = ServoStateMessage()
            state_msg.id = controller.servo_id
            state_msg.goal = int(state.position)
            state_msg.position = int(state.position)
            servo_msg.servo_state.append(state_msg)

        self.joint_states_pub.publish(joint_msg)
        self.servo_states_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManager('controller_manager')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
