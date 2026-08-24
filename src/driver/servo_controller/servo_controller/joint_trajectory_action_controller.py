#!/usr/bin/env python3
"""FollowJointTrajectory adapter for Hiwonder serial bus servos."""

import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse
from rclpy.duration import Duration
from rclpy.time import Time

from servo_controller_msgs.msg import ServoPosition


class JointTrajectoryActionController:
    def __init__(self, node, servo_manager, controller_name, controllers):
        self.node = node
        self.servo_manager = servo_manager
        self.joint_names = [controller.joint_name for controller in controllers]
        self.joint_to_controller = {
            controller.joint_name: controller for controller in controllers
        }
        self.action_server = ActionServer(
            node,
            FollowJointTrajectory,
            controller_name + '/follow_joint_trajectory',
            execute_callback=self.follow_trajectory_callback,
            cancel_callback=self.cancel_callback,
        )

    @staticmethod
    def cancel_callback(cancel_request):
        del cancel_request
        return CancelResponse.ACCEPT

    @staticmethod
    def result(error_code, message=''):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = message
        return result

    def _publish_feedback(self, goal_handle, desired_positions):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.node.get_clock().now().to_msg()
        feedback.joint_names = self.joint_names
        feedback.desired.positions = list(desired_positions)

        states = self.servo_manager.get_position()
        for joint_name in self.joint_names:
            controller = self.joint_to_controller[joint_name]
            pulse = states[str(controller.servo_id)].position
            feedback.actual.positions.append(controller.pos_pulse_to_rad(pulse))

        feedback.error.positions = [
            desired - actual
            for desired, actual in zip(
                feedback.desired.positions,
                feedback.actual.positions,
            )
        ]
        goal_handle.publish_feedback(feedback)

    def follow_trajectory_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            message = 'Incoming trajectory is empty'
            self.node.get_logger().error(message)
            goal_handle.abort()
            return self.result(FollowJointTrajectory.Result.INVALID_GOAL, message)

        missing_joints = [
            joint for joint in self.joint_names if joint not in trajectory.joint_names
        ]
        if missing_joints:
            message = f'Trajectory is missing joints: {missing_joints}'
            self.node.get_logger().error(message)
            goal_handle.abort()
            return self.result(FollowJointTrajectory.Result.INVALID_JOINTS, message)

        lookup = [trajectory.joint_names.index(name) for name in self.joint_names]
        for point in trajectory.points:
            if not point.positions or max(lookup) >= len(point.positions):
                message = 'A trajectory point does not contain all joint positions'
                self.node.get_logger().error(message)
                goal_handle.abort()
                return self.result(FollowJointTrajectory.Result.INVALID_GOAL, message)

        if trajectory.header.stamp.sec == 0 and trajectory.header.stamp.nanosec == 0:
            start_time = self.node.get_clock().now()
        else:
            start_time = Time.from_msg(trajectory.header.stamp)

        while self.node.get_clock().now() < start_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self.result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    'Trajectory canceled before start',
                )
            time.sleep(0.001)

        previous_offset = 0.0
        for point in trajectory.points:
            offset = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            segment_duration = max(0.02, offset - previous_offset)
            previous_offset = offset

            desired_positions = [point.positions[index] for index in lookup]
            commands = []
            for joint_name, desired_position in zip(
                self.joint_names,
                desired_positions,
            ):
                controller = self.joint_to_controller[joint_name]
                command = ServoPosition()
                command.id = controller.servo_id
                command.position = float(controller.pos_rad_to_pulse(desired_position))
                commands.append(command)

            self.servo_manager.set_position(segment_duration, commands)
            target_time = start_time + Duration(seconds=offset)

            while self.node.get_clock().now() < target_time:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self.result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'Trajectory canceled',
                    )
                time.sleep(0.001)

            self._publish_feedback(goal_handle, desired_positions)

        goal_handle.succeed()
        self.node.get_logger().info('Trajectory execution completed')
        return self.result(FollowJointTrajectory.Result.SUCCESSFUL)
