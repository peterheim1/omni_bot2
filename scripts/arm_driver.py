#!/usr/bin/env python3
'''

Created March, 2017

@author: Peter Heim

  arm_driver.py - gateway to Arduino based arm controller
  Copyright (c) 2011 Peter Heim.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rclpy
from rclpy.node import Node
import sys
from time import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from robbie_msg.msg import HeadJoint, ArmJoint

from std_msgs.msg import String
from std_msgs.msg import Float64, Float32

from trajectory_msgs.msg import JointTrajectory

from sensor_msgs.msg import JointState
from SerialDataGateway3 import SerialDataGateway

class ArmDriver(Node):

    def __init__(self):
        super().__init__('robbie_arm_driver')
        self.get_logger().info('starting robbie_arm driver control')
        self.publisher_ = self.create_publisher(String, 'arm_driver_serial', 10)
        self._arm_JointPublisher = self.create_publisher(JointState, 'joint_states',  5)
        self._SerialDataGateway = SerialDataGateway("/dev/torso_driver", 115200,  self._HandleReceivedLine)
        self.subscription = self.create_subscription(ArmJoint,'right_arm_controller/command', self._HandleJoint_1_Command,10)
        self.subscription = self.create_subscription(HeadJoint,'head_controller/command', self._HandleJoint_2_Command,10)
        self.subscription = self.create_subscription(Float32,'gripper_controller/command', self._HandleJoint_3_Command,10)
        self.subscription
        self.Start()

    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)
        #self.get_logger().info(str(msg))
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(x))
        #if (self._Counter % 50 == 0):
        
        
    
    def _HandleJoint_1_Command(self, Command):
                """ Handle movement requests.
                    for right arm joints
                    send message in degrees 0 -180
                """
                
                
                j1 = degrees(Command.j0) 
                j2 = degrees(Command.j1) + 90
                j3 = degrees(Command.j2) 
                j4 = degrees(Command.j3) 
                j5 = degrees(Command.j4) 
                j6 = degrees(Command.j5) 
                
                message = 'r %d %d %d %d %d %d  \r' % (j1, j2, j3, j4, j5, j6)  
                self.get_logger().info(str(message))
                self._WriteSerial(message)
                
    def _HandleJoint_2_Command(self, Command):
                """ Handle movement requests.
                    head 
                    send message in degrees 0 -180
                """
                
                j0 = 90 #degrees(Command.j0) +90 # roll
                j1 = degrees(Command.j1) +90 # pitch
                j2 = degrees(Command.j2) +90 # yaw
                
                message = 'h %d %d %d  \r' % (j0, j1, j2)  
                #rospy.loginfo(message)
                self._WriteSerial(message)
                  
    def _HandleJoint_3_Command(self, Command):
                """ Handle movement requests.
                    right gripper
                    send message in degrees 0 -180
                """
                #msg =Float32()
                
                j0 = degrees(Command.data) +90
                
                message = 'k %d  \r' % (j0)  
                self.get_logger().info(str(j0))
                self._WriteSerial(message)          
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        
        self._SerialDataGateway.Start()
        message = 's \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'r \r'
        self._WriteSerial(message)
        sleep(5)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
         self._SerialDataGateway.Write(message)
         


def main(args=None):
    rclpy.init(args=args)

    arm_driver = ArmDriver()

    rclpy.spin(arm_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
