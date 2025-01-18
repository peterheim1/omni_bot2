#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState  # New import for BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty
from math import atan2, sqrt

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # Hard-coded debug variable
        self.debug = True  # Set to True to enable debug logs, False to disable

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'detected_dock_pose',
            self.pose_callback,
            10
        )
        # Removed subscription to '/docked' and its callback
        # Added subscription to '/battery_state'
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # Services
        self.start_docking_service = self.create_service(
            Empty,
            '/start_docking',
            self.start_docking_handler
        )
        self.undock_service = self.create_service(
            Empty,
            '/undock',
            self.undock_handler
        )

        # State variables
        self.state = 'IDLE'  # Possible states: IDLE, DOCKING, UNDOCKING_FORWARD, UNDOCKING_ROTATE
        self.dock_pose = None
        self.docked = False  # Now set based on battery_state

        # Undocking parameters
        self.forward_speed = 0.17  # meters per second
        self.rotate_speed = 0.1  # radians per second (90 degrees)
        self.forward_duration = 4.0  # seconds to move forward 1 meter at 0.2 m/s
        self.rotate_duration = 1.0  # seconds to rotate 90 degrees at 1.57 rad/s

        # Timers for undocking
        self.state_start_time = None

        # Rotation attempt parameters to prevent infinite spinning
        self.max_rotation_attempts = 5
        self.rotation_attempts = 0
        self.rotation_start_time = None
        self.max_rotation_time = 30.0  # seconds

        self.get_logger().info("Docking node initialized and ready.")

        # Create a timer to run the main loop at 10 Hz
        self.timer = self.create_timer(0.1, self.main_loop)

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function to handle incoming PoseStamped messages.
        Updates the docking pose based on detections.

        Args:
            msg (PoseStamped): Incoming PoseStamped message.
        """
        self.dock_pose = msg.pose
        if self.debug:
            yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
            self.get_logger().debug(
                f"Received dock pose: position=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}), "
                f"orientation (yaw)=({yaw:.2f} rad)"
            )
        else:
            self.get_logger().info("Dock pose received.")

    def battery_callback(self, msg: BatteryState):
        """
        Callback function to handle incoming BatteryState messages.
        Updates the docked status based on power_supply_status.

        Args:
            msg (BatteryState): Incoming BatteryState message.
        """
        # Assuming power_supply_status: 1 indicates docked
        previous_status = self.docked
        self.docked = (msg.power_supply_status == 1)
        if self.debug:
            self.get_logger().debug(f"Battery power_supply_status: {msg.power_supply_status}, Docked: {self.docked}")
        else:
            self.get_logger().info(f"Docked status updated: {self.docked}")

    def start_docking_handler(self, request, response):
        """
        Service handler to initiate docking.

        Args:
            request: Service request (Empty).
            response: Service response (Empty).
        """
        if self.state != 'DOCKING':
            if self.dock_pose is not None:
                self.state = 'DOCKING'
                self.get_logger().info("Docking initiated.")
                if self.debug:
                    self.get_logger().debug(f"Current state set to DOCKING.")
            else:
                self.get_logger().warn("Cannot start docking: No dock pose available.")
        else:
            self.get_logger().info("Already in docking state.")
            if self.debug:
                self.get_logger().debug(f"Docking state unchanged.")
        return response

    def undock_handler(self, request, response):
        """
        Service handler to initiate undocking.

        Args:
            request: Service request (Empty).
            response: Service response (Empty).
        """
        if self.state not in ['UNDOCKING_FORWARD', 'UNDOCKING_ROTATE']:
            self.state = 'UNDOCKING_FORWARD'
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("Undocking initiated: Moving forward.")
            if self.debug:
                self.get_logger().debug(f"Current state set to UNDOCKING_FORWARD.")
        else:
            self.get_logger().info("Already in undocking process.")
            if self.debug:
                self.get_logger().debug(f"Undocking state unchanged.")
        return response

    def main_loop(self):
        """
        Main loop executed at 10 Hz to handle docking and undocking states.
        """
        twist = Twist()

        if self.state == 'DOCKING':
            if self.docked:
                self.get_logger().info("Docked successfully. Stopping docking.")
                self.state = 'IDLE'
                # Publish zero velocities to ensure the robot stops
                self.publish_twist(twist)
                if self.debug:
                    self.get_logger().debug(f"Transitioned to IDLE state.")
                # Reset rotation attempts
                self.rotation_attempts = 0
                self.rotation_start_time = None
                return

            if self.dock_pose:
                # Simple proportional controller to move towards the dock pose
                Kx = 0.07
                Ky = 0.1  # Proportional gain for y-axis
                Kz = 0.07
                # Calculate the error between current position and dock position
                error_x = self.dock_pose.position.x
                error_y = self.dock_pose.position.y
                yaw = self.get_yaw_from_quaternion(self.dock_pose.orientation)
                error_z = yaw

                # Compute distance to dock
                distance = sqrt(error_x**2 + error_y**2)

                # Log current state and distance
                if not self.debug:
                    self.get_logger().info(f"Current State: {self.state}, Distance to Dock: {distance:.2f} m")
                else:
                    self.get_logger().debug(f"Current State: {self.state}, Distance to Dock: {distance:.2f} m")

                if distance < 0.1:
                    self.get_logger().info("Robot is very close to dock. Stopping movement.")
                    self.state = 'IDLE'
                    twist = Twist()
                    self.publish_twist(twist)
                    if self.debug:
                        self.get_logger().debug(f"Transitioned to IDLE state.")
                    return

                # Set velocities based on proportional control
                twist.linear.x = Kx * error_x
                twist.linear.y = Ky * error_y  # Y movement reversed
                twist.angular.z = Kz * error_z

                # Limit the velocities
                max_linear = 0.07  # m/s
                max_angular = 0.2  # rad/s
                twist.linear.x = max(-max_linear, min(twist.linear.x, max_linear))
                twist.linear.y = max(-max_linear, min(twist.linear.y, max_linear))
                twist.angular.z = max(-max_angular, min(twist.angular.z, max_angular))

                if self.debug:
                    self.get_logger().debug(
                        f"Docking: Dock Position -> x: {error_x:.2f}, y: {error_y:.2f}, yaw: {error_z:.2f} rad"
                    )
                    self.get_logger().debug(
                        f"Docking: cmd_vel -> linear.x: {twist.linear.x:.2f}, "
                        f"linear.y: {twist.linear.y:.2f}, angular.z: {twist.angular.z:.2f}"
                    )
                else:
                    self.get_logger().info(
                        f"Docking: Moving towards dock. Distance: {distance:.2f} m."
                    )

                # Publish the Twist message
                self.publish_twist(twist)
            else:
                # No dock pose detected, initiate rotation to search
                self.get_logger().warn("No dock pose detected. Initiating search rotation.")
                twist.angular.z = 0.2  # Rotate at a fixed speed to search

                if self.debug:
                    self.get_logger().debug("Publishing fixed rotation velocity for searching.")
                else:
                    self.get_logger().info("Rotating to search for dock pose.")

                # Publish the rotation command
                self.publish_twist(twist)

                # Increment rotation attempts
                self.rotation_attempts += 1

                # Record rotation start time if first rotation attempt
                if self.rotation_attempts == 1:
                    self.rotation_start_time = self.get_clock().now()

                # Check for maximum rotation attempts or time
                if self.rotation_attempts > self.max_rotation_attempts:
                    self.get_logger().warn("Maximum rotation attempts reached. Stopping robot.")
                    self.state = 'IDLE'
                    twist = Twist()  # Stop movement
                    self.publish_twist(twist)
                    self.rotation_attempts = 0
                    self.rotation_start_time = None
                elif self.rotation_start_time:
                    elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds * 1e-9
                    if elapsed > self.max_rotation_time:
                        self.get_logger().warn("Rotation time exceeded. Stopping robot.")
                        self.state = 'IDLE'
                        twist = Twist()
                        self.publish_twist(twist)
                        self.rotation_attempts = 0
                        self.rotation_start_time = None

        elif self.state == 'UNDOCKING_FORWARD':
            current_time = self.get_clock().now()
            elapsed = (current_time - self.state_start_time).nanoseconds * 1e-9
            if elapsed < self.forward_duration:
                twist.linear.x = self.forward_speed
                if self.debug:
                    self.get_logger().debug(f"Undocking: Moving forward. Elapsed time: {elapsed:.2f}s")
                else:
                    self.get_logger().info(f"Undocking: Moving forward. Elapsed time: {elapsed:.2f}s")
                self.publish_twist(twist)
            else:
                # Transition to rotation
                self.state = 'UNDOCKING_ROTATE'
                self.state_start_time = self.get_clock().now()
                self.get_logger().info("Undocking: Forward movement complete. Starting rotation.")
                if self.debug:
                    self.get_logger().debug(f"Transitioned to UNDOCKING_ROTATE state.")

        elif self.state == 'UNDOCKING_ROTATE':
            current_time = self.get_clock().now()
            elapsed = (current_time - self.state_start_time).nanoseconds * 1e-9
            if elapsed < self.rotate_duration:
                twist.angular.z = self.rotate_speed
                if self.debug:
                    self.get_logger().debug(f"Undocking: Rotating. Elapsed time: {elapsed:.2f}s")
                else:
                    self.get_logger().info(f"Undocking: Rotating. Elapsed time: {elapsed:.2f}s")
                self.publish_twist(twist)
            else:
                # Transition to IDLE
                self.state = 'IDLE'
                self.get_logger().info("Undocking complete. Stopping robot.")
                twist = Twist()  # Stop movement
                if self.debug:
                    self.get_logger().debug(f"Transitioned to IDLE state.")
                self.publish_twist(twist)

        else:  # IDLE
            # Do not publish any Twist messages when in IDLE
            if self.debug:
                self.get_logger().debug("IDLE state: No cmd_vel published. Robot is stationary.")
            else:
                self.get_logger().info("IDLE state: Robot is stationary.")
            # Optionally, ensure the robot stops by sending a zero Twist once
            # Uncomment the following lines if the robot requires at least one stop command
            # twist = Twist()
            # self.publish_twist(twist)

    def publish_twist(self, twist):
        """
        Publish a Twist message to the /cmd_vel topic.

        Args:
            twist (geometry_msgs.msg.Twist): The Twist message to publish.
        """
        self.cmd_pub.publish(twist)

    def get_yaw_from_quaternion(self, orientation):
        """
        Convert quaternion to yaw angle.

        Args:
            orientation (geometry_msgs.msg.Quaternion): Orientation in quaternion.

        Returns:
            float: Yaw angle in radians.
        """
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Docking node stopped cleanly.")
    finally:
        # Ensure the robot stops moving when the node is shut down
        twist = Twist()
        node.publish_twist(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
