#!/usr/bin/env python3
"""Interactive keyboard commissioning tool for ArmPi-FPV bus servos."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from ros_robot_controller_msgs.msg import (
    BusServoState,
    ServoPosition,
    ServosPosition,
    SetBusServoState,
)


# This is the neutral pose used by the original ArmPi-FPV applications.
HOME = {
    1: 200,  # gripper open
    2: 500,  # wrist rotation
    3: 80,   # wrist pitch
    4: 825,  # elbow
    5: 625,  # shoulder
    6: 500,  # base
}

# Commissioning limits are intentionally narrower than the servo's 0..1000
# electrical range. They are not a substitute for calibrated joint limits.
LIMITS = {
    1: (180, 570),
    2: (200, 800),
    3: (50, 950),
    4: (50, 950),
    5: (100, 900),
    6: (50, 950),
}

NAMES = {
    1: 'gripper',
    2: 'wrist rotation',
    3: 'wrist pitch',
    4: 'elbow',
    5: 'shoulder',
    6: 'base',
}

HELP = """
ArmPi-FPV keyboard jogger

  h       move slowly to the stock home pose and enable jogging
  1..6    select servo: 1 gripper, 2 wrist rotation, 3 wrist pitch,
                         4 elbow, 5 shoulder, 6 base
  a / d   decrease / increase selected servo by the current step
  - / +   decrease / increase step (5, 10, 25 or 50 pulses)
  Space   stop all servo motion immediately
  p       print the locally tracked commanded positions
  ?       show this help
  q       stop motion and quit

The robot has no usable position feedback. Do not run MoveIt or another arm
controller at the same time, and keep a physical power disconnect in reach.
"""


class KeyboardJogger(Node):
    def __init__(self):
        super().__init__('armpi_keyboard_jog')
        self.position_pub = self.create_publisher(
            ServosPosition,
            '/ros_robot_controller/bus_servo/set_position',
            1,
        )
        self.state_pub = self.create_publisher(
            SetBusServoState,
            '/ros_robot_controller/bus_servo/set_state',
            1,
        )
        self.positions = dict(HOME)
        self.selected = 6
        self.steps = (5, 10, 25, 50)
        self.step_index = 1
        self.homed = False

    @property
    def step(self):
        return self.steps[self.step_index]

    def hardware_is_available(self):
        return self.position_pub.get_subscription_count() > 0

    def publish_positions(self, positions, duration):
        message = ServosPosition()
        message.duration = float(duration)
        for servo_id, pulse in positions.items():
            command = ServoPosition()
            command.id = servo_id
            command.position = pulse
            message.position.append(command)
        self.position_pub.publish(message)

    def home(self):
        if not self.hardware_is_available():
            print('\nNo hardware-driver subscriber; is hardware.launch.py running?')
            return
        self.positions = dict(HOME)
        self.publish_positions(self.positions, 3.0)
        self.homed = True
        print('\nMoving to stock home over 3 seconds. Jogging enabled.')
        self.print_positions()

    def jog(self, direction):
        if not self.homed:
            print('\nJogging is locked. Press h to establish the stock home pose first.')
            return
        low, high = LIMITS[self.selected]
        requested = self.positions[self.selected] + direction * self.step
        pulse = max(low, min(high, requested))
        if pulse == self.positions[self.selected]:
            print(f'\n{NAMES[self.selected]} is at commissioning limit {pulse}.')
            return
        self.positions[self.selected] = pulse
        self.publish_positions({self.selected: pulse}, 0.25)
        print(
            f'\rServo {self.selected} ({NAMES[self.selected]}): {pulse:4d}  '
            f'step: {self.step:2d}   ',
            end='',
            flush=True,
        )

    def stop(self):
        message = SetBusServoState()
        for servo_id in HOME:
            state = BusServoState()
            state.present_id = [1, servo_id]
            state.stop = [1, 1]
            message.state.append(state)
        self.state_pub.publish(message)
        print('\nSTOP sent. Servos remain torque-enabled and hold their positions.')

    def select(self, servo_id):
        self.selected = servo_id
        print(
            f'\nSelected servo {servo_id}: {NAMES[servo_id]} '
            f'(tracked pulse {self.positions[servo_id]})'
        )

    def change_step(self, direction):
        self.step_index = max(
            0,
            min(len(self.steps) - 1, self.step_index + direction),
        )
        print(f'\nJog step: {self.step} pulses')

    def print_positions(self):
        values = ', '.join(
            f'{servo_id}:{self.positions[servo_id]}' for servo_id in sorted(HOME)
        )
        print(f'Commanded pulses: {values}')


def wait_for_hardware(node, timeout=5.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.hardware_is_available():
            return True
    return False


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_jog must be run in an interactive terminal.', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = KeyboardJogger()
    old_terminal = termios.tcgetattr(sys.stdin)

    try:
        if not wait_for_hardware(node):
            print(
                'No subscriber on /ros_robot_controller/bus_servo/set_position.\n'
                'Start the hardware launch first.',
                file=sys.stderr,
            )
            return 2

        print(HELP)
        print('Selected servo 6: base. Press h before jogging.')
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            key = sys.stdin.read(1)
            if key in ('q', '\x03'):
                node.stop()
                break
            if key == ' ':
                node.stop()
            elif key == 'h':
                node.home()
            elif key in '123456':
                node.select(int(key))
            elif key == 'a':
                node.jog(-1)
            elif key == 'd':
                node.jog(1)
            elif key in ('-', '_'):
                node.change_step(-1)
            elif key in ('+', '='):
                node.change_step(1)
            elif key == 'p':
                print()
                node.print_positions()
            elif key == '?':
                print(HELP)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
