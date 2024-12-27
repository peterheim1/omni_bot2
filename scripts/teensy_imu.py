#! /usr/bin/env python3
import rclpy
import sys
from time import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from robbie_msg.msg import HeadJoint
from SerialDataGateway3 import SerialDataGateway

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('angle_publisher')
        self.get_logger().info('starting angle publisher')
         #         Internal data        
        self._anglePublisher = self.create_publisher(Float32,'heading', 10)
        self._SerialDataGateway = SerialDataGateway("/dev/ttyACM0", 115200,  self._HandleReceivedLine)
        
        self.Start()
        

    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        #self.publisher_.publish(msg)
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(x))
        #if (self._Counter % 50 == 0):
        
        if (len(line) > 0):
                        lineParts = line.split('\t')                 
                        
                        if (lineParts[0] == 'h'):
                                self._Broadcast_angle(lineParts)
                                return
                                            
                                
    def _Broadcast_angle(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        if (partsCount  < 2):
                pass
        msg = Float32()
        
        msg.data = float(lineParts[1])
        self._anglePublisher.publish(msg)
        #self.get_logger().info(str(msg))                    
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        
        self._SerialDataGateway.Start()
        message = 'c \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'c \r'
        self._WriteSerial(message)
        #sleep(5)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
         self._SerialDataGateway.Write(message)
         
    
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()