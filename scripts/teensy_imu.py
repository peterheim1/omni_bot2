import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.orientation = None
        self.linear_acceleration = None
        self.angular_velocity = None

        # Define covariance matrices
        self.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]  # Example values
        self.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]  # Example values
        self.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]  # Example values

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii').strip()
            #self.get_logger().info(f'Received line: {line}')
            if line.startswith('<') and line.endswith('>'):
                line = line[1:-1]
                parts = line.split(',')
                if parts[0] == 'ACC' and len(parts) == 4:
                    #self.get_logger().info(f'Parsed accelerometer data: {parts[1:]}')
                    accel_data = [float(x) for x in parts[1:]]
                    self.linear_acceleration = accel_data
                    self.publish_imu_data()
                elif parts[0] == 'GYRO' and len(parts) == 4:
                    #self.get_logger().info(f'Parsed gyroscope data: {parts[1:]}')
                    gyro_data = [float(x) for x in parts[1:]]
                    self.angular_velocity = gyro_data
                    self.publish_imu_data()
                elif parts[0] == 'ROT' and len(parts) == 5:
                    #self.get_logger().info(f'Parsed quaternion data: {parts[1:]}')
                    quat_data = [float(x) for x in parts[1:]]
                    self.orientation = Quaternion(w=quat_data[0], x=quat_data[1], y=quat_data[2], z=quat_data[3])
                    self.publish_imu_data()

    def publish_imu_data(self):
        imu_msg = Imu()
        current_time = self.get_clock().now().to_msg()

        if self.orientation:
            imu_msg.orientation = self.orientation
            imu_msg.orientation_covariance = self.orientation_covariance
            #self.get_logger().info(f'Publishing orientation: {self.orientation}')
        
        if self.linear_acceleration:
            imu_msg.linear_acceleration.x = self.linear_acceleration[0]
            imu_msg.linear_acceleration.y = self.linear_acceleration[1]
            imu_msg.linear_acceleration.z = self.linear_acceleration[2]
            imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
            #self.get_logger().info(f'Publishing linear acceleration: {self.linear_acceleration}')
        
        if self.angular_velocity:
            imu_msg.angular_velocity.x = self.angular_velocity[0]
            imu_msg.angular_velocity.y = self.angular_velocity[1]
            imu_msg.angular_velocity.z = self.angular_velocity[2]
            imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
            #self.get_logger().info(f'Publishing angular velocity: {self.angular_velocity}')

        imu_msg.header.stamp = current_time
        self.publisher_.publish(imu_msg)
        #self.get_logger().info('Published IMU data')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
