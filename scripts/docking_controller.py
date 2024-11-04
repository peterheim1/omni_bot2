#! /usr/bin/env python3
# Copyright ...
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from opennav_docking_msgs.action import DockRobot, UndockRobot
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import the NavigateToPose action (adjust based on your navigation stack)
from nav2_msgs.action import NavigateToPose  # Ensure nav2_msgs is installed


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class DockingController(Node):

    def __init__(self):
        super().__init__('docking_controller')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        # Action Clients
        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self, UndockRobot, 'undock_robot')
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service Client for '/cal' service
        self.cal_service_client = self.create_client(Empty, '/cal')
        while not self.cal_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'/cal' service not available, waiting...")

        # Publisher to stop the robot by publishing zero velocities
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize previous button states for edge detection
        self.prev_buttons = []

        # Subscribe to the 'joy' topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('DockingController node has been initialized.')

    def destroy_node(self):
        self.docking_client.destroy()
        self.undocking_client.destroy()
        self.navigation_client.destroy()
        self.cal_service_client.destroy()
        self.cmd_vel_publisher.destroy()
        super().destroy_node()

    def dock_robot(self):
        """Send a DockRobot action request."""
        dock_pose = PoseStamped()
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        dock_pose.header.frame_id = "base_link"
        dock_pose.pose.position.x = 0.7  # 70 cm in front of the robot
        dock_pose.pose.position.y = 0.0
        dock_pose.pose.orientation.w = 1.0  # No rotation

        self.get_logger().info(f'Docking at pose: {dock_pose}...')

        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False
        goal_msg.dock_pose = dock_pose
        goal_msg.dock_type = ""  # Specify if needed

        send_goal_future = self.docking_client.send_goal_async(goal_msg, self.feedback_callback)
        send_goal_future.add_done_callback(self.dock_goal_response_callback)

    def undock_robot(self):
        """Send an UndockRobot action request."""
        dock_type = "nova_carter_dock"  # Specify the dock type as needed
        self.get_logger().info(f'Undocking from dock type: {dock_type}...')

        while not self.undocking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('"UndockRobot" action server not available, waiting...')

        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type

        send_goal_future = self.undocking_client.send_goal_async(goal_msg, self.feedback_callback)
        send_goal_future.add_done_callback(self.undock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Docking request was rejected!')
            return

        self.get_logger().info('Docking request accepted.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.dock_result_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undocking request was rejected!')
            return

        self.get_logger().info('Undocking request accepted.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.undock_result_callback)

    def dock_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Docking succeeded!')
        else:
            self.get_logger().error(f'Docking failed with status: {status}')

    def undock_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Undocking succeeded!')
        else:
            self.get_logger().error(f'Undocking failed with status: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Process feedback if needed
        self.feedback = feedback

    def call_cal_service(self):
        """Call the '/cal' service."""
        if not self.cal_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("'/cal' service not available!")
            return

        request = Empty.Request()
        self.get_logger().info("Calling '/cal' service...")

        future = self.cal_service_client.call_async(request)
        future.add_done_callback(self.cal_service_callback)

    def cal_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("'/cal' service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"'/cal' service call failed: {e}")

    def cancel_navigation(self):
        """Cancel any ongoing navigation tasks."""
        if not self.navigation_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('"navigate_to_pose" action server not available, cannot cancel navigation.')
            return

        # Cancel all goals
        self.get_logger().info('Cancelling all navigation goals...')
        cancel_future = self.navigation_client.cancel_all_goals_async()
        cancel_future.add_done_callback(self.cancel_navigation_callback)

    def cancel_navigation_callback(self, future):
        try:
            result = future.result()
            if result.goals_canceling > 0:
                self.get_logger().info(f'Cancelled {result.goals_canceling} navigation goal(s).')
            else:
                self.get_logger().info('No navigation goals were active to cancel.')
        except Exception as e:
            self.get_logger().error(f'Failed to cancel navigation goals: {e}')

    def stop_robot(self):
        """Publish zero velocities to stop the robot."""
        self.get_logger().info('Stopping the robot by publishing zero velocities.')
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Publish the Twist message multiple times to ensure the robot receives it
        for _ in range(10):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

    def joy_callback(self, msg):
        """Callback function for joystick messages."""
        # Initialize previous buttons state if empty
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)

        # Iterate through buttons and detect rising edge
        for i, button in enumerate(msg.buttons):
            if i >= len(self.prev_buttons):
                self.prev_buttons.append(0)

            if button == 1 and self.prev_buttons[i] == 0:
                self.get_logger().info(f'Button {i} pressed.')
                if i == 0:
                    self.undock_robot()
                elif i == 1:
                    self.call_cal_service()
                elif i == 2:
                    self.cancel_navigation()
                    self.stop_robot()
                elif i == 3:
                    self.dock_robot()

            # Update previous button state
            self.prev_buttons[i] = button


def main(args=None):
    rclpy.init(args=args)

    docking_controller = DockingController()

    try:
        rclpy.spin(docking_controller)
    except KeyboardInterrupt:
        docking_controller.get_logger().info('Shutting down DockingController node.')
    finally:
        docking_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
