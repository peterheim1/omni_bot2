#!/usr/bin/env python3

from math import atan2, degrees, sqrt, radians, cos, sin
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
from SerialDataGateway3 import SerialDataGateway  # Ensure this module is available
# Acceleration constraints (m/s²)
MAX_ACCELERATION = 0.5  # Maximum acceleration
MAX_DECELERATION = 0.5  # Maximum deceleration

# Time step for velocity updates (seconds)
TIME_STEP = 0.1  # 100 ms

class CmdVelSubscriber(Node):
    """ROS2 Node that subscribes to the /cmd_vel topic and publishes wheel commands."""
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize velocities
        self.lock = threading.Lock()
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Define wheel positions (x_pos, y_pos) relative to robot center
        self.wheels = {
            'S0': (1, 1),    # Front-left
            'S3': (1, -1),   # Front-right
            'S1': (-1, 1),   # Rear-left
            'S2': (-1, -1)   # Rear-right
        }

        # Create a publisher for wheel commands
        self.wheel_command_pub = self.create_publisher(String, 'wheel_commands', 10)

    def cmd_vel_callback(self, msg):
        """Callback function that gets called when a new Twist message is received."""
        with self.lock:
            self.linear_x = msg.linear.x
            self.linear_y = msg.linear.y
            self.angular_z = msg.angular.z
        self.get_logger().debug(f"Received cmd_vel: linear_x={self.linear_x}, linear_y={self.linear_y}, angular_z={self.angular_z}")

        # Calculate wheel data
        wheel_data = self.calculate_wheel_data(self.linear_x, self.linear_y, self.angular_z)
        #self.print_wheel_data(wheel_data)

        # Format the message as 'a S0 S1 S2 S3 V_F_L V_R_L V_R_R V_F_R\r'
        S0_angle = int(round(wheel_data['S0']['angle']))
        S1_angle = int(round(wheel_data['S1']['angle']))
        S2_angle = int(round(wheel_data['S2']['angle']))
        S3_angle = int(round(wheel_data['S3']['angle']))

        # Assuming velocity scaling: m/s to cm/s by multiplying by 100
        # Adjust scaling as per Arduino's expected input
        V_F_L = int(round(wheel_data['S0']['velocity'] * 1000))
        V_R_L = int(round(wheel_data['S1']['velocity'] * 1000))
        V_R_R = int(round(wheel_data['S2']['velocity'] * 1000))
        V_F_R = int(round(wheel_data['S3']['velocity'] * 1000))

        # Create the message string
        message = 'a %d %d %d %d %d %d %d %d\r' % (S0_angle, S1_angle, S2_angle, S3_angle, V_F_L, V_R_L, V_R_R, V_F_R)
        self.get_logger().info(f"Wheel Command: {message.strip()}")  # Log without carriage return

        # Publish the wheel command
        self.wheel_command_pub.publish(String(data=message))

    def calculate_wheel_data(self, X, Y, Z):
        """
        Calculate the angles and velocities for a 4-wheel swerve steering robot.

        Parameters:
        X (float): Forward velocity (+X is forward) in m/s
        Y (float): Left velocity (+Y is left) in m/s
        Z (float): Rotation velocity (-Z is clockwise) in rad/s

        Returns:
        dict: Wheel angles normalized to ±180 degrees, velocities in m/s, and reversal flags
        """
        data = {}
        for key, (x_pos, y_pos) in self.wheels.items():
            # Calculate the rotational component (linear velocity due to rotation)
            rot_x = -Z * y_pos
            rot_y = Z * x_pos

            # Total velocity components for the wheel
            total_x = X + rot_x
            total_y = Y + rot_y

            # Calculate the angle in degrees
            angle = degrees(atan2(total_y, total_x))
            angle = self.normalize_angle(angle)

            # Calculate the velocity magnitude
            velocity = sqrt(total_x**2 + total_y**2)

            # Initialize reversal flag
            reversed_flag = False

            # Condition: If angle > 128 degrees or angle < -128 degrees, reverse angle and velocity
            if abs(angle) > 128:
                angle = self.normalize_angle(angle - 180)
                velocity = -velocity  # Invert velocity to indicate reversal
                reversed_flag = True

            data[key] = {'angle': angle, 'velocity': velocity, 'reversed': reversed_flag}

        return data  

    def normalize_angle(self, angle):
        """Normalize angle to be within -180 to 180 degrees."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def print_wheel_data(self, wheel_data):
        """Print the wheel angles and velocities."""
        print("\n--- Wheel Data ---")
        for key in sorted(wheel_data.keys()):
            angle = wheel_data[key]['angle']
            velocity = wheel_data[key]['velocity']
            reversed_flag = wheel_data[key]['reversed']
            direction = "Reversed" if reversed_flag else "Normal"
            print(f"{key}: Angle = {angle:.1f}°, Velocity = {velocity:.2f} m/s ({direction})")
        print("------------------\n")


class MinimalPublisher(Node):
    """ROS2 Node that subscribes to wheel commands and sends them to Arduino via serial."""
    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Starting Arduino control')                      
        self.publisher_ = self.create_publisher(String, 'base_serial', 10)
        self._JointPublisher = self.create_publisher(JointState, 'joint_states', 5) 
        self._batteryPublisher = self.create_publisher(Float32,'battery_state', 10)                   
        self._SerialDataGateway = SerialDataGateway("/dev/ttyACM0", 115200,  self._HandleReceivedLine) 
        self.srv = self.create_service(Empty, 'cal', self.Calibrate_callback)  
        self.Start()

        # Subscribe to wheel commands
        self.wheel_sub = self.create_subscription(
            String,
            'wheel_commands',
            self.wheel_command_callback,
            10)
        self.wheel_sub  # prevent unused variable warning

    def _HandleReceivedLine(self, line):
        """Handles lines received from the serial port."""
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)        
        if len(line) > 0:
            lineParts = line.strip().split('\t')                 
            if lineParts[0] == 'a':
                self._joint_state(lineParts)
            if (lineParts[0] == 'b'):
                self._Broadcast_battery(lineParts)
                                 

    def _joint_state(self, lineParts):
        """Parses joint state data from serial input and publishes it."""
        partsCount = len(lineParts)
        if partsCount < 9:
            self.get_logger().warning("Incomplete joint state data received.")
            return
        try:
            # Parse positions and velocities
            s1 = radians(float(lineParts[1]))
            s2 = radians(float(lineParts[2]))
            s3 = radians(float(lineParts[3]))
            s4 = radians(float(lineParts[4])) 
            v1 = (float(lineParts[5]) * 0.001) * 1.01
            v2 = (float(lineParts[6]) * 0.001) * 1.01
            v3 = (float(lineParts[7]) * 0.001) * 1.01
            v4 = (float(lineParts[8]) * 0.001) * 1.01

            # Create and publish JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'joint_chassis_to_steering_left_front', 'joint_chassis_to_steering_left_rear', 
                'joint_chassis_to_steering_right_rear', 'joint_chassis_to_steering_right_front',
                'joint_steering_to_wheel_left_front', 'joint_steering_to_wheel_left_rear', 
                'joint_steering_to_wheel_right_rear', 'joint_steering_to_wheel_right_front'
            ]
            msg.position = [
                s1, s2, s3, s4,
                0.0, 0.0, 0.0, 0.0
            ]
            msg.velocity = [0.0, 0.0, 0.0, 0.0, v1, v2, v3, v4]
            msg.effort = [0.0] * 8
            self._JointPublisher.publish(msg)
            self.get_logger().debug(f'Published JointState: {msg}')
            
        except Exception as e:
            self.get_logger().error(f"Error parsing joint state: {e}")

    def wheel_command_callback(self, msg):
        """Callback function to handle wheel commands and send to Arduino."""
        message = msg.data
        self.get_logger().info(f"Sending to Arduino: {message.strip()}")  # Remove carriage return in log
        self._WriteSerial(message)

    def _Broadcast_battery(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        volt = float(lineParts[1])/20.7
        per = int((volt / 20.7) * 100)
        msg = Float32()
        msg.data = volt
        #msg.header.stamp = Node.get_clock(self).now().to_msg()
        #msg.voltage = float(lineParts[1])* 0.015867159
        #msg.percentage = float(per)
        self._batteryPublisher.publish(msg)
        #self.get_logger().info(str(msg))

    def Start(self):
        """Starts the serial data gateway and sends initial command."""
        self._SerialDataGateway.Start()
        message = 'c \r'
        self._WriteSerial(message)
        self.get_logger().info("Serial communication started.")

    def Stop(self):
        """Stops the serial data gateway and sends stop command."""
        self.get_logger().info("Stopping serial communication.")
        message = 'c \r'
        self._WriteSerial(message)
        self._SerialDataGateway.Stop()
        self.get_logger().info("Serial communication stopped.")
        
    def _WriteSerial(self, message):
        """Writes a message to the serial port."""
        self._SerialDataGateway.Write(message)
        self.get_logger().debug(f"Sent to serial: {message.strip()}")

    def Calibrate_callback(self, request, responce):
        responce
        message = 'c \r' 
        self.get_logger().info("Sending auto dock message: " + message)
        self._WriteSerial(message)
        return responce

def ros_spin(executor):
    """Function to spin the ROS2 executor."""
    executor.spin()


def main():
    # Initialize ROS2
    rclpy.init()

    # Create the ROS nodes
    cmd_vel_node = CmdVelSubscriber()
    minimal_publisher = MinimalPublisher()

    # Create a single executor and add both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(cmd_vel_node)
    executor.add_node(minimal_publisher)

    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    ros_thread.start()

    # Keep the main thread alive to allow ROS and other threads to operate
    try:
        while True:
            time.sleep(1)  # Sleep for 1 second to reduce CPU usage
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS2
        executor.shutdown()
        cmd_vel_node.destroy_node()
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
