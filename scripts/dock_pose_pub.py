#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import cv2
from tf_transformations import quaternion_from_matrix

class FPDockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        self.get_logger().debug(f"docking pose publisher started")
        # Publisher to publish the detected dock pose
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'detected_dock_pose',
            10  # Queue size
        )
        
        # Subscriber to AprilTag detections
        self.subscription_ = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detection_callback,
            10  # Queue size
        )
        
        # Timer to periodically publish the saved pose
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.saved_pose = None

        # Parameters (These should be set according to your setup)
        self.tag_size = 0.162  # Size of the AprilTag (meters)
        # Example camera intrinsic parameters
        self.camera_matrix = np.array([
            [600, 0, 320],
            [0, 600, 240],
            [0, 0, 1]
        ])  # Replace with your camera's intrinsic matrix
        self.dist_coeffs = np.zeros((4,1))  # Assuming no lens distortion

    def not_within_range(self, x, z, range_limit):
        """
        Calculate the Euclidean distance in the XZ-plane and determine if it's beyond the range limit.
        
        Args:
            x (float): X-coordinate.
            z (float): Z-coordinate.
            range_limit (float): Maximum allowable distance.
        
        Returns:
            bool: True if distance > range_limit, else False.
        """
        distance = math.sqrt(x ** 2 + z ** 2)
        return distance > range_limit

    def detection_callback(self, msg):
        """
        Callback function that processes AprilTag detections, computes the pose, and saves it if within range.
        
        Args:
            msg (AprilTagDetectionArray): The incoming AprilTag detections.
        """
        if not msg.detections:
            self.get_logger().debug("No detections received.")
            return  # Exit if there are no detections

        for detection in msg.detections:
            # Extract necessary data from detection
            tag_id = detection.id
            family = detection.family
            centre = detection.centre
            corners = detection.corners

            if len(corners) != 4:
                self.get_logger().warn(f"Detection ID {tag_id} does not have 4 corners.")
                continue  # Skip if not a quadrilateral

            # Define real-world coordinates of the tag corners
            # Assuming the tag is placed on the XY plane with Z=0
            half_size = self.tag_size / 2.0
            object_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=float)

            # Extract image points from detection
            image_points = np.array([
                [corner.x, corner.y] for corner in corners
            ], dtype=float)

            # Compute pose using solvePnP
            success, rotation_vector, translation_vector = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs
            )

            if not success:
                self.get_logger().warn(f"Pose estimation failed for tag ID {tag_id}.")
                continue

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Create a 4x4 transformation matrix
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix
            transformation_matrix[:3, 3] = translation_vector.flatten()

            # Convert rotation matrix to quaternion
            quaternion = quaternion_from_matrix(transformation_matrix)

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header = msg.header  # Use the same header as the detection message
            pose_msg.pose.position.x = translation_vector[0][0]
            pose_msg.pose.position.y = translation_vector[1][0]
            pose_msg.pose.position.z = translation_vector[2][0]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            # Debugging information
            self.get_logger().debug(f"Detected pose for tag ID {tag_id}: {pose_msg.pose}")

            # Check if the detected pose is beyond the specified range
            if self.not_within_range(pose_msg.pose.position.x, pose_msg.pose.position.z, 0.1):
                self.saved_pose = pose_msg
                #self.get_logger().info(f"Pose saved for tag ID {tag_id} for publishing.")
            else:
                self.get_logger().info(f"Detected pose for tag ID {tag_id} is within range; not saving.")

    def timer_callback(self):
        """
        Timer callback that publishes the saved pose at regular intervals.
        """
        if self.saved_pose is not None:
            self.publisher_.publish(self.saved_pose)
            #self.get_logger().info(f"Published pose: {self.saved_pose.pose}")
            self.saved_pose = None  # Reset after publishing to avoid repeated publishing
        else:
            self.get_logger().debug("No pose to publish.")

def main(args=None):
    """
    The main entry point for the node.
    """
    rclpy.init(args=args)
    node = FPDockPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
