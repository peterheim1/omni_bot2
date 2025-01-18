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
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')



        # Service Client for '/cal' service
        self.cal_service_client = self.create_client(Empty, '/cal')
        while not self.cal_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'/cal' service not available, waiting...")

        # Service Client for '/dock ' service
        self.dock_service_client = self.create_client(Empty, 'start_docking')
        while not self.dock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'start_docking' service not available, waiting...")

        # Service Client for '/undock ' service
        self.undock_service_client = self.create_client(Empty, 'undock')
        while not self.undock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("'undock' service not available, waiting...") 
    



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
        self.dock_service_client.destroy()
        self.undock_service_client.destroy()
        self.navigation_client.destroy()
        self.cal_service_client.destroy()
        self.cmd_vel_publisher.destroy()
        super().destroy_node()

    def dock_robot(self):
        """Call the '/start dock' service."""
        if not self.dock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("'/dock' service not available!")
            return

        request = Empty.Request()
        self.get_logger().info("Calling '/start dock' service...")

        future = self.dock_service_client.call_async(request)
        future.add_done_callback(self.dock_service_callback)

    def undock_robot(self):
        """Call the '/undock' service."""
        if not self.undock_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("'/undock' service not available!")
            return

        request = Empty.Request()
        self.get_logger().info("Calling '/undock' service...")

        future = self.undock_service_client.call_async(request)
        future.add_done_callback(self.undock_service_callback)

    def dock_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("'/start dock' service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"'/start dock' service call failed: {e}")

    def undock_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("'/undock' service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"'/undock' service call failed: {e}")

    

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
