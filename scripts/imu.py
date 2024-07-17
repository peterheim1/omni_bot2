#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.clock import Clock, Time
from rclpy.duration import Duration as TimeDuration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_geometry_msgs import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import sys
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState,Imu
from std_msgs.msg import Float64MultiArray,String,Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from SerialDataGateway3 import SerialDataGateway
from math import sin, cos, pi, radians, degrees
from tf_transformations import quaternion_from_euler
from robbie_msg.msg import HeadJoint
from SerialDataGateway3 import SerialDataGateway



class ImuPub(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        #self._JointPublisher = self.create_publisher(JointState, 'joint_states',  5)
        self.publisher_ = self.create_publisher(String, 'base_serial', 10)
        self._anglePublisher = self.create_publisher(Float32,'angle', 10)
        self._SerialDataGateway = SerialDataGateway("/dev/ttyUSB1", 115200,  self._HandleReceivedLine)
        
        # Declare all parameters
        #self.msg_send  = JointState()
        # Set the timer to publish messages at 10 Hz (0.1 seconds interval)
        #self.timer = self.create_timer(0.2, self.timer_callback)
        self.Start()
        self.count = 0

    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(msg))
        #if (self._Counter % 50 == 0):
        
        if (len(line) > 0):
                        lineParts = line.split('\t')                 
                        if (lineParts[0] == 'o'):
                                self._Broadcast_Odom(lineParts)
                                return

    def _Broadcast_Odom(self, lineParts):
        partsCount = len(lineParts)
        #if (partsCount  < 2):
            #pass
        #try:
        msg = Float32()
        v1 = (float(lineParts[1]))
        msg.data = v1
        
                      


            
            #self._JointPublisher.publish(v1) 
        self.get_logger().info(str(v1))
        
        self._anglePublisher.publish(msg)

            

        #except:
            #self.get_logger().info("Unexpected error imu.py   :" + str(sys.exc_info()[0]))   
                              
    

    

       

    def timer_callback(self):
        #self._JointPublisher.publish(self.msg_send)
        self.count += 1

    def Enc_reset(self):       
        message = 'c \r'
        #self._WriteSerial(message)
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        
        self._SerialDataGateway.Start()
        message = 'c \r'
        #self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'c \r'
        #self._WriteSerial(message)
        #sleep(5)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
         self._SerialDataGateway.Write(message)    
        
        
def main(args=None):
    rclpy.init(args=args)

    pub = ImuPub()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

