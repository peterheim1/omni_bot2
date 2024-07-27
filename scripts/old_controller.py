#! /usr/bin/env python3
import rclpy
import sys
from time import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees,sqrt
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState, JointState
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from robbie_msg.msg import HeadJoint
from SerialDataGateway3 import SerialDataGateway


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('starting arduino control')
         #         Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.enc_center = 0
        #self.left_enc_r = 0
        #self.right_enc_r = 0
        #self.center_enc_r = 0

        
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.theta = 0
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = 0
        self.ticksPerMeter = int(11236) #11236
        self.wheel_track = float(0.34)
        self.lastTime = Node.get_clock(self).now()
        self.publisher_ = self.create_publisher(String, 'base_serial', 10)
        self._JointPublisher = self.create_publisher(JointState, 'joint_states', 5)
        #self._enc_publisher = self.create_publisher(HeadJoint, 'encoder', 10)
        self._OdometryPublisher = self.create_publisher(Odometry,'odom', 10)
        self._OdometryTransformBroadcaster = tf2_ros.TransformBroadcaster(self)
        self._batteryPublisher = self.create_publisher(Float32,'battery_state', 10)
        self._anglePublisher = self.create_publisher(Float32,'angle', 10)
        self._SerialDataGateway = SerialDataGateway("/dev/ttyACM0", 115200,  self._HandleReceivedLine)
        
        self.Start()
        self.subscription = self.create_subscription(Twist,'cmd_vel', self._HandleVelocityCommand,10)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Empty, 'autodock', self.AutoDock_callback)
        self.srv = self.create_service(Empty, 'cal', self.Calibrate_callback)
        
        now = Node.get_clock(self).now()   
        #self.then = self.now # time for determining dx/dy
        self.last = int(time() * 1000)
        #self.t_next = now + self.t_delta
        self.odom_linear_scale_correction = 1.0
        self.odom_angular_scale_correction = 1.0
        
        

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
                        if (lineParts[0] == 'a'):
                                self._Broadcast_Odom(lineParts)
                                return
                        if (lineParts[0] == 'c'):
                                self._Broadcast_angle(lineParts)
                                return
                        if (lineParts[0] == 'b'):
                                self._Broadcast_battery(lineParts)
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
       
            
    def _Broadcast_battery(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        volt = float(lineParts[1])
        per = int((volt / 13.4) * 100)
        msg = Float32()
        msg.data = volt
        #msg.header.stamp = Node.get_clock(self).now().to_msg()
        #msg.voltage = float(lineParts[1])* 0.015867159
        #msg.percentage = float(per)
        self._batteryPublisher.publish(msg)
        #self.get_logger().info(str(msg))
        
            
    def _Broadcast_Odom(self, lineParts):
        partsCount = len(lineParts)
        if (partsCount  < 4):
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

            v1 =v1 * 1.01
            v2 =v2 * 1.01
            v3 =v3 * 1.01
            v4 =v4 * 1.01

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
        
        
        


        
    def Enc_reset(self):       
        message = 'c \r'
        self._WriteSerial(message)
        
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
         
    def _HandleVelocityCommand(self, twistCommand):
        """ Handle movement requests. """
        scale_factor = 1
        #R = 42.43
        L = 30
        W = 30
        R = sqrt((L*L)+(W*W))

        RevD = 1 # used to set forwards or backwards
        x = twistCommand.linear.x        # m/s
        y = twistCommand.linear.y        # m/s
        omega = twistCommand.angular.z      # rad/s
        if omega > 0.2:
             omega = 0.2

        if omega < -0.2:
             omega = -0.2     
        omega = omega 
        self.th = omega

        y = y * -1
        if x < 0:
            RevD =-1
        
        A = y - omega*(30/R)
        B = y + omega*(30/R)
        C = abs(x) - omega*(30/R)
        D = abs(x) + omega*(30/R)
                
        #S1 = math.atan2(A, D)*180 / math.pi #REAR left in degrees        
        #S2 = math.atan2(A, C)*180 / math.pi #REAR right
        #S0 = math.atan2(B, D)*180 / math.pi # FRONT left
        #S3 = math.atan2(B, C)*180 / math.pi # FRONT right

        S3 = math.atan2(A, D)*180 / math.pi # fr       
        S0 = math.atan2(A, C)*180 / math.pi #fl
        S2 = math.atan2(B, D)*180 / math.pi # rr
        S1 = math.atan2(B, C)*180 / math.pi # rl
        

        V_R_R = (math.sqrt((B*B)+(D*D))*1000) * RevD
        V_R_L = (math.sqrt((B*B)+(C*C))*1000) * RevD
        V_F_L = (math.sqrt((A*A)+(C*C))*1000) * RevD
        V_F_R = (math.sqrt((A*A)+(D*D))*1000) * RevD


        
        #scale velocities below 270 
        #max_vel =max(V_R_R,V_R_L,V_F_L,V_F_R)
        
        #if max_vel > 156.0:
             #scale_factor = float(155.0/max_vel)
        #else: 
            #scale_factor = 1.0

        #V_R_R=V_R_R*scale_factor
        #V_R_L=V_R_L*scale_factor
        #V_F_L=V_F_L*scale_factor
        #V_F_R=V_F_R*scale_factor
        
        #REAR RIGHT
        if  S2 > 128:
            S2 = S2 - 180
            V_R_R  = V_R_R * -1
        if  S2 < -128:
            S2 = S2 + 180
            V_R_R  = V_R_R * -1
        # REAR LEFT    
        if S1 > 128:
            S1 = S1 - 180
            V_R_L  = V_R_L * -1
        if S1 < -128:
            S1 = S1 + 180
            V_R_L  = V_R_L * -1
        # FRONT RIGHT   
        if S3 > 128:
            S3 = S3 - 180
            V_F_R  = V_F_R * -1
        if S3 < -128:
            S3 = S3 + 180
            V_F_R  = V_F_R * -1
            
        #FRONT LEFT    
        if S0 > 128:
            S0 = S0 - 180
            V_F_L  = V_F_L * -1
        if S0 < -128:
            S0 = S0 + 180
            V_F_L  = V_F_L * -1

            
        
        #convert angles to encoder ticks
        #Front_left = (j4) #int(self.translate(j1, -150, 150, 0, 1024))
        #Front_right = (j3)#+13 5#int(self.translate(j2, -150, 150, 0, 1024))
        #Rear_left = (j2) #+135#int(self.translate(j3, -150, 150, 0, 1024))
        #Rear_right = (j1)#+135#int(self.translate(j4, -150, 150, 0, 1024))
        S0 = S0 * -1
        S1 = S1 * -1
        S2 = S2 * -1
        S3 = S3 * -1

        
        #a =[Front_left,Front_right,Rear_left,Rear_right]
        #b = [j1, j2, j3, j4, V_F_L, V_F_R, V_R_L, V_R_R]
        
        message = 'a %d %d %d %d %d %d %d %d\r' % (S0, S1, S2, S3,V_F_L, V_R_L, V_R_R, V_F_R )
        self.get_logger().info(str(message))
        self._WriteSerial(message)
        
    def AutoDock_callback(self, request, responce):
        responce
        message = 'a \r' 
        self.get_logger().info("Sending auto dock message: " + message)
        self._WriteSerial(message)
        return responce
    
    def Calibrate_callback(self, request, responce):
        responce
        message = 'c \r' 
        self.get_logger().info("Sending auto dock message: " + message)
        self._WriteSerial(message)
        return responce
        

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
