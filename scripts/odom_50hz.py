#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, atan2, sqrt, pi, radians, degrees

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info('Starting odometry node')

        self.use_sim_time = self.get_parameter_or('use_sim_time', rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)).value
        if self.use_sim_time:
            self.get_logger().info('Using simulated time')

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # Rotation in radians

        self.last_time = self.get_clock().now()

        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.wheel_track = 0.2375  # Distance between left and right wheels
        self.wheel_base = 0.2375  # Distance between front and rear wheels (modify as needed)

        # Create a timer to publish odometry at 50 Hz
        self.timer = self.create_timer(1.0 / 50.0, self.publish_odometry)

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.current_time = self.last_time

    def joint_state_callback(self, msg):
        #self.get_logger().info("Received JointState message")

        try:
            wheel_positions = msg.position[:4]
            wheel_speeds = msg.velocity[-4:]

            #self.get_logger().info(f"Wheel positions: {wheel_positions}")
            #self.get_logger().info(f"Wheel speeds: {wheel_speeds}")

            # Extract wheel positions and speeds
            fl_angle, fr_angle, rl_angle, rr_angle = [float(angle) for angle in wheel_positions]
            fl_speed, fr_speed, rl_speed, rr_speed = [float(speed) for speed in wheel_speeds]

            # Define the position vectors for each wheel relative to the robot center
            r_fl = [self.wheel_base / 2.0, self.wheel_track / 2.0]
            r_fr = [self.wheel_base / 2.0, -self.wheel_track / 2.0]
            r_rl = [-self.wheel_base / 2.0, self.wheel_track / 2.0]
            r_rr = [-self.wheel_base / 2.0, -self.wheel_track / 2.0]

            # Calculate velocity vectors for each wheel
            v_fl = [fl_speed * cos(fl_angle), fl_speed * sin(fl_angle)]
            v_fr = [fr_speed * cos(fr_angle), fr_speed * sin(fr_angle)]
            v_rl = [rl_speed * cos(rl_angle), rl_speed * sin(rl_angle)]
            v_rr = [rr_speed * cos(rr_angle), rr_speed * sin(rr_angle)]

            # Calculate the average linear velocity in the x and y directions
            self.vx = (v_fl[0] + v_fr[0] + v_rl[0] + v_rr[0]) / 4.0
            self.vy = (v_fl[1] + v_fr[1] + v_rl[1] + v_rr[1]) / 4.0

            # Calculate the rotational velocity vth (with sign inversion)
            self.vth = -((v_fr[1] - v_rr[1]) - (v_fl[1] - v_rl[1])) / (2.0 * self.wheel_track)

            #self.get_logger().info(f"Calculated velocities: vx={self.vx:.2f}, vy={self.vy:.2f}, vth={self.vth:.2f}")

            self.current_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")

    def publish_odometry(self):
        # Update odometry
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        self.last_time = self.current_time

        # Apply coordinate transformation
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Debug message for current position
        #self.get_logger().info(f"Current position: x={self.x:.2f}, y={self.y:.2f}, th={degrees(self.th):.2f}°")

        # Create odometry quaternion
        odom_quat = Quaternion()
        odom_quat.z = sin(self.th / 2.0)
        odom_quat.w = cos(self.th / 2.0)

        # Publish odometry transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.transform.translation.x = float(self.x)
        odom_trans.transform.translation.y = float(self.y)
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = float(self.vy)
        odom.twist.twist.angular.z = float(self.vth)

        self.odom_publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)

    odometry_node = OdometryNode()

    rclpy.spin(odometry_node)

    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
