#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3, Twist
from tf2_ros import TransformBroadcaster
from math import sin, cos, radians, pi
import numpy as np

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info('Starting odometry node')

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # Rotation in radians

        self.last_time = self.get_clock().now()

        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.wheel_track = 0.34  # Distance between left and right wheels
        self.ticks_per_meter = 11236

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()

        # Parse joint state message
        try:
            wheel_positions = {
                'joint_chassis_to_steering_left_front': 0,
                'joint_chassis_to_steering_left_rear': 0,
                'joint_chassis_to_steering_right_rear': 0,
                'joint_chassis_to_steering_right_front': 0,
                'joint_steering_to_wheel_left_front': 0,
                'joint_steering_to_wheel_left_rear': 0,
                'joint_steering_to_wheel_right_rear': 0,
                'joint_steering_to_wheel_right_front': 0
            }

            for i, name in enumerate(msg.name):
                if name in wheel_positions:
                    wheel_positions[name] = msg.position[i]

            # Calculate velocities based on wheel positions
            vx = (wheel_positions['joint_steering_to_wheel_left_front'] +
                  wheel_positions['joint_steering_to_wheel_left_rear'] +
                  wheel_positions['joint_steering_to_wheel_right_rear'] +
                  wheel_positions['joint_steering_to_wheel_right_front']) / 4

            vy = (wheel_positions['joint_steering_to_wheel_left_front'] -
                  wheel_positions['joint_steering_to_wheel_left_rear'] +
                  wheel_positions['joint_steering_to_wheel_right_rear'] -
                  wheel_positions['joint_steering_to_wheel_right_front']) / 4

            vth = (wheel_positions['joint_steering_to_wheel_left_front'] -
                   wheel_positions['joint_steering_to_wheel_left_rear'] -
                   wheel_positions['joint_steering_to_wheel_right_rear'] +
                   wheel_positions['joint_steering_to_wheel_right_front']) / (4 * self.wheel_track)

            # Update odometry
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
            delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
            delta_th = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # Create odometry quaternion
            odom_quat = Quaternion()
            odom_quat.z = sin(self.th / 2)
            odom_quat.w = cos(self.th / 2)

            # Publish odometry transform
            odom_trans = TransformStamped()
            odom_trans.header.stamp = current_time.to_msg()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'base_footprint'
            odom_trans.transform.translation.x = self.x
            odom_trans.transform.translation.y = self.y
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat

            self.odom_broadcaster.sendTransform(odom_trans)

            # Publish odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = odom_quat
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            self.odom_publisher.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")

def main(args=None):
    rclpy.init(args=args)

    odometry_node = OdometryNode()

    rclpy.spin(odometry_node)

    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
