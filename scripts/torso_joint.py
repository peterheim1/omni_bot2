#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from SerialDataGateway3 import SerialDataGateway

class ArmDriver(Node):

    def __init__(self):
        super().__init__('robbie_arm_driver')
        self.get_logger().info('Starting robbie arm driver control (using /joint_states)')

        # Publisher for sending messages to the Arduino over serial
        self.publisher_ = self.create_publisher(String, 'arm_driver_serial', 10)
        
        # Set up the serial connection
        self._SerialDataGateway = SerialDataGateway("/dev/head", 115200, self._HandleReceivedLine)
        
        # Subscribe to /joint_states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._HandleJointStates,
            10
        )
        
        # Define joint groups with the names (the order defines the message order)
        self.groups = {
    "right": [
        "right_joint_1",
        "right_joint_2",
        "right_joint_3",
        "right_joint_4",
        "right_joint_5",
        "right_joint_6"
    ],
    "left": [
        "left_joint_1",
        "left_joint_2",
        "left_joint_3",
        "left_joint_4",
        "left_joint_5",
        "left_joint_6"
    ],
    "head": [
        "head_pan_joint",
        "head_tilt_joint",
        "ear_left_joint",
        "ear_right_joint",
    ],

    "grippers": [
        "right_gripper_joint", 
        "left_gripper_joint"]
        }
        
        # Dictionary to keep track of the last published value for each joint
        self.last_joint_values = {}

        self.Start()

    def _HandleReceivedLine(self, line):
        """ Callback when a line is received from the serial port. """
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)
        
    def _HandleJointStates(self, msg):
        """
        Callback for /joint_states.
        For each joint in our defined groups:
          - Convert the value from radians to degrees.
          - Compare with the last published value.
          - If any joint in a group has changed, publish a new message for that group.
        """
        # Build a dictionary for the current joint values (only for joints of interest)
        current_values = {}
        # Combine all joint names from the groups for quick lookup
        all_interested_joints = self.groups["right"] + self.groups["left"] + self.groups["head"] + self.groups["grippers"]
        for name, pos in zip(msg.name, msg.position):
            if name in all_interested_joints:
                # Convert from radians to degrees (without multiplying by 10)
                value = int(round(math.degrees(pos)))
                current_values[name] = value

        # Process each group and publish if any value has changed
        for group, joints in self.groups.items():
            changed = False
            group_values = []
            for joint in joints:
                # If the joint is not reported in the current message, use the last known value (or 0 if not available)
                if joint in current_values:
                    new_val = current_values[joint]
                else:
                    new_val = self.last_joint_values.get(joint, 0)
                group_values.append(new_val)
                # Compare with the stored value; if different, mark as changed.
                if self.last_joint_values.get(joint) != new_val:
                    changed = True

            # If any joint in the group changed, update stored values and publish message.
            if changed:
                # Update the stored last values for the group
                for joint, val in zip(joints, group_values):
                    self.last_joint_values[joint] = val

                # Define a prefix letter for each group
                if group == "right":
                    prefix = "r"
                elif group == "left":
                    prefix = "l"
                    
                elif group == "head":
                    prefix = "h"
                elif group == "grippers":
                    prefix = "g"
                else:
                    prefix = "u"  # unknown

                # Construct the message string (values separated by space, terminated with carriage return/newline)
                message = prefix + " " + " ".join(str(v) for v in group_values) + " \r\n"
                self.get_logger().info(f"Publishing {group} group: {message.strip()}")
                self._WriteSerial(message)

    def Start(self):
        """Starts the serial gateway and sends the start message."""
        self._SerialDataGateway.Start()
        start_message = 's \r'
        self._WriteSerial(start_message)
        
    def Stop(self):
        """Stops the serial gateway."""
        self.get_logger().info("Stopping")
        stop_message = 'r \r'
        self._WriteSerial(stop_message)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        """Write a message to the serial gateway."""
        self._SerialDataGateway.Write(message)


def main(args=None):
    rclpy.init(args=args)
    arm_driver = ArmDriver()
    try:
        rclpy.spin(arm_driver)
    except KeyboardInterrupt:
        pass
    arm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()