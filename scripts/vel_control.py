import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import threading

class DriveModuleController(Node):
    def __init__(self):
        super().__init__('drive_velocity_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'drive_module_velocity_controller/commands',
            self.listener_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/linorobot', 115200, timeout=1)
        self.serial_lock = threading.Lock()
        self.received_values = [0.0, 0.0, 0.0, 0.0]

        # Start a thread to continuously read from the Arduino
        #self.read_thread = threading.Thread(target=self.read_from_serial)
        #self.read_thread.daemon = True
        #self.read_thread.start()

    def listener_callback(self, msg):
        if len(msg.data) >= 4:
            data_to_send = [int(x * 20) for x in msg.data[:4]]
            formatted_data = ' '.join(map(str, data_to_send)) + '\n'
            self.send_to_serial(formatted_data)
            self.get_logger().info(f'Sent to Arduino: {formatted_data.strip()}')

    def send_to_serial(self, data):
        with self.serial_lock:
            self.serial_port.write(data.encode())

    def read_from_serial(self):
        while rclpy.ok():
            with self.serial_lock:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    try:
                        values = list(map(int, line.split()))
                        if len(values) == 4:
                            self.received_values = [float(x) for x in values]
                            self.get_logger().info(f'Received from Arduino: {self.received_values}')
                    except ValueError:
                        self.get_logger().warn(f'Invalid data received: {line}')

def main(args=None):
    rclpy.init(args=args)
    drive_module_controller = DriveModuleController()
    rclpy.spin(drive_module_controller)
    drive_module_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
