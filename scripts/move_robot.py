import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math
import sys

class MoveRobot(Node):

    def __init__(self, operation, distance_or_angle):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.initial_position = None
        self.initial_yaw = None
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.speed = 0.2  # meters per second
        self.angular_speed = 0.5  # radians per second
        self.operation = operation
        self.distance_to_travel = distance_or_angle if operation in ['forward', 'left_y', 'right_y'] else 0.0
        self.angle_to_turn = math.radians(distance_or_angle) if operation in ['left', 'right'] else 0.0
        self.direction = 1 if operation == 'left' else -1 if operation == 'right' else 0
        self.y_direction = 1 if operation == 'left_y' else -1 if operation == 'right_y' else 0
        self.get_logger().info(f'Starting to {operation}')

    def euler_from_quaternion(self, x, y, z, w):
        """Helper function to convert quaternion to euler angles."""
        quaternion = [x, y, z, w]
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=False)
        return euler  # returns (roll, pitch, yaw)

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, current_yaw = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        if self.initial_position is None and self.initial_yaw is None:
            self.initial_position = current_position
            self.initial_yaw = current_yaw
            return

        if self.operation == 'forward':
            self.distance_traveled = math.sqrt(
                (current_position.x - self.initial_position.x) ** 2 +
                (current_position.y - self.initial_position.y) ** 2
            )
            if self.distance_traveled < self.distance_to_travel:
                msg = Twist()
                msg.linear.x = self.speed
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'Reached the target distance. Actual distance traveled: {self.distance_traveled:.2f} meters')
                self.destroy_node()

        elif self.operation in ['left', 'right']:
            self.angle_turned = abs(current_yaw - self.initial_yaw)
            if self.angle_turned > math.pi:
                self.angle_turned = 2 * math.pi - self.angle_turned

            if self.angle_turned < self.angle_to_turn:
                msg = Twist()
                msg.angular.z = self.angular_speed * self.direction
                self.publisher_.publish(msg)
            else:
                msg = Twist()
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'Reached the target angle. Actual angle turned: {math.degrees(self.angle_turned):.2f} degrees')
                self.destroy_node()

        elif self.operation in ['left_y', 'right_y']:
            self.distance_traveled = abs(current_position.y - self.initial_position.y)
            if self.distance_traveled < self.distance_to_travel:
                msg = Twist()
                msg.linear.y = self.speed * self.y_direction
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            else:
                msg = Twist()
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'Reached the target lateral distance. Actual distance traveled: {self.distance_traveled:.2f} meters')
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    if args is None or len(args) < 2:
        print("Usage: ros2 run omni_bot2 move_robot.py <forward/left/right/left_y/right_y> <distance_or_angle>")
        return

    operation = args[0]
    distance_or_angle = float(args[1])
    
    move_robot = MoveRobot(operation, distance_or_angle)
    rclpy.spin(move_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
