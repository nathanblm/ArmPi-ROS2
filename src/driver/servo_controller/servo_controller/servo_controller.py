#!/usr/bin/env python3
"""Raw bus-servo state and command forwarding."""

from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition


class ServoState:
    def __init__(self, name='', position=0):
        self.name = name
        self.position = position


class ServoManager:
    """Track logical joint state and forward raw positions to the STM32 node."""

    def __init__(self, node, servos):
        self.node = node
        self.servos = servos
        self.servo_position_pub = node.create_publisher(
            ServosPosition,
            'ros_robot_controller/bus_servo/set_position',
            1,
        )

    def get_position(self):
        return self.servos

    def set_position(self, duration, commands):
        duration = max(0.02, min(30.0, float(duration)))
        msg = ServosPosition()
        msg.duration = duration

        for command in commands:
            servo_id = str(command.id)
            if servo_id not in self.servos:
                self.node.get_logger().warning(
                    f'Ignoring command for unconfigured servo ID {command.id}'
                )
                continue

            pulse = max(0, min(1000, int(command.position)))
            self.servos[servo_id].position = pulse

            servo_msg = ServoPosition()
            servo_msg.id = command.id
            servo_msg.position = float(pulse)
            msg.position.append(servo_msg)

        if msg.position:
            self.servo_position_pub.publish(msg)
