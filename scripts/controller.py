#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# you may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from SerialDataGateway3 import SerialDataGateway
from math import degrees, radians
import threading

class OmniController(Node):
    def __init__(self):
        super().__init__("omni_controller")
        self._JointPublisher = self.create_publisher(JointState, 'joint_states', 5)
        self.publisher_ = self.create_publisher(String, 'base_serial', 10)
        self._SerialDataGateway = SerialDataGateway("/dev/ttyACM0", 115200, self._HandleReceivedLine)
        self.subscription_steering = self.create_subscription(Float64MultiArray, '/drive_module_steering_angle_controller/commands', self._HandleSteering_Command, 10)
        self.subscription_velocity = self.create_subscription(Float64MultiArray, '/drive_module_velocity_controller/commands', self._HandleVelocity_Command, 10)
        self.front_left_steering = 0.0   
        self.front_right_steering = 0.0
        self.rear_left_steering = 0.0    
        self.rear_right_steering = 0.0
        self.front_left_velocity = 0.0    
        self.front_right_velocity = 0.0
        self.rear_left_velocity = 0.0    
        self.rear_right_velocity = 0.0
        self.V0 = 0.0
        self.V3 = 0.0 
        self.V1 = 0.0
        self.V2 = 0.0 
        self.msg_send = JointState()
        self.Start()
        self.count = 0
        self.lock = threading.Lock()

        # Start the background thread to handle incoming serial data
        self.serial_thread = threading.Thread(target=self._serial_read_thread)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def _serial_read_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
    def constrain(self,value, min_value, max_value):
        return max(min_value, min(value, max_value))        

    def _HandleReceivedLine(self, line):
        msg = String()
        msg.data = line
        #self.publisher_.publish(msg)
        if len(line) > 0:
            lineParts = line.split('\t')                 
            if lineParts[0] == 'a':
                self._Broadcast_Odom(lineParts)

    def _Broadcast_Odom(self, lineParts):
        partsCount = len(lineParts)
        if partsCount < 4:
            pass
        try:
            s1 = radians(int(lineParts[1])) 
            s2 = radians(int(lineParts[2])) 
            s3 = radians(int(lineParts[3]))
            s4 = radians(int(lineParts[4])) 
            v1 = float(int(lineParts[5]) *0.001)
            v2 = float(int(lineParts[6]) *0.001)
            v3 = float(int(lineParts[7]) *0.001)
            v4 = float(int(lineParts[8]) *0.001)

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
            #self.get_logger().info(f'Publishing: {msg}')
            #self._WriteSerial(msg)

        except Exception as e:
            self.get_logger().info(f"Unexpected error odom from base serial.py: {e}")
            #pass

    def _HandleSteering_Command(self, msg):
        with self.lock:
            self.S0 = msg.data[0]    
            self.S3 = msg.data[3]
            self.S1 = msg.data[1]   
            self.S2 = msg.data[2]

            #self.front_left_steering = self.constrain(msg.data[0], -2.35619, 2.35619)  # -135 to 135 degrees in radians
            #self.front_right_steering = self.constrain(msg.data[3], -2.35619, 2.35619)
            #self.rear_left_steering = self.constrain(msg.data[1], -2.35619, 2.35619)
            #self.rear_right_steering = self.constrain(msg.data[2], -2.35619, 2.35619)

        j1 = degrees(self.S0)   
        j2 = degrees(self.S1)
        j3 = degrees(self.S2) 
        j4 = degrees(self.S3)

        with self.lock:
            v1 = self.V0 * 30
            v2 = self.V1 * 30
            v3 = self.V2 * 30
            v4 = self.V3 * 30

        message = 'a %d %d %d %d %d %d %d %d\r' % (j1, j2, j3, j4, v1, v2, v3, v4)

        self.get_logger().info(f'Sending: {message}')
        self._WriteSerial(message)

    def _HandleVelocity_Command(self, msg): 
        with self.lock:
            self.V0 = msg.data[0]
            self.V3 = msg.data[3]
            self.V1 = msg.data[1]   
            self.V2 = msg.data[2]

    

    def Enc_reset(self):       
        message = 'x \r'
        self._WriteSerial(message)
        
    def Start(self):
        self._SerialDataGateway.Start()
        message = 'x \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'x \r'
        self._WriteSerial(message)
        self._SerialDataGateway.stop()
        
    def _WriteSerial(self, message):
        self._SerialDataGateway.Write(message)

def main(args=None):
    rclpy.init(args=args)
    pub = OmniController()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
