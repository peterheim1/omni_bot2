#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import tf_transformations
from threading import Lock

class AprilTagPosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')

        # Publisher for PoseStamped messages
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'detected_dock_pose',
            10
        )

        # Subscriber to AprilTag detections
        self.detection_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detection_callback,
            10
        )

        # Subscriber to CameraInfo
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/rear_camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Initialize camera intrinsic parameters
        self.K = None
        self.dist_coeffs = None
        self.camera_info_received = False
        self.camera_info_lock = Lock()

        # Define the real-world size of the AprilTag (meters)
        self.tag_size = 0.162  # Adjust based on your tag

        self.get_logger().info('AprilTagPosePublisher node has been started.')

    def multiply_quaternions(self, q1, q2):
        """
        Multiply two quaternions.
        
        Args:
            q1 (list or tuple): First quaternion [x, y, z, w].
            q2 (list or tuple): Second quaternion [x, y, z, w].
        
        Returns:
            list: Resulting quaternion [x, y, z, w].
        """
        return tf_transformations.quaternion_multiply(q1, q2)

    def camera_info_callback(self, msg: CameraInfo):
        with self.camera_info_lock:
            if not self.camera_info_received:
                # Extract camera matrix
                self.K = np.array(msg.k).reshape((3, 3))
                # Extract distortion coefficients
                self.dist_coeffs = np.array(msg.d).reshape(-1, 1)
                self.camera_info_received = True
                self.get_logger().info('Camera intrinsic parameters received.')

    def detection_callback(self, msg: AprilTagDetectionArray):
        with self.camera_info_lock:
            if not self.camera_info_received:
                self.get_logger().warn('CameraInfo not received yet. Skipping detection.')
                return

        if not msg.detections:
            self.get_logger().info('No AprilTags detected in this frame.')
            return

        for detection in msg.detections:
            tag_id = detection.id
            self.get_logger().info(f'Processing AprilTag ID: {tag_id}')

            # Extract image points from detection
            img_points = np.array([
                [corner.x, corner.y] for corner in detection.corners
            ], dtype=np.float32)

            # Define object points based on tag size
            obj_points = np.array([
                [0, 0, 0],
                [self.tag_size, 0, 0],
                [self.tag_size, self.tag_size, 0],
                [0, self.tag_size, 0]
            ], dtype=np.float32)

            # Compute pose using solvePnP
            success, rotation_vector, translation_vector = cv2.solvePnP(
                obj_points,
                img_points,
                self.K,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not success:
                self.get_logger().error(f'Pose estimation failed for tag ID: {tag_id}')
                continue

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Convert rotation matrix to quaternion
            transformation_matrix = np.identity(4)
            transformation_matrix[:3, :3] = rotation_matrix
            quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)

            # Define the 90-degree rotation quaternion around Y-axis
            theta = np.pi / 2  # 90 degrees in radians
            q_rot = tf_transformations.quaternion_from_euler(0, theta, 0)  # Roll, Pitch, Yaw

            # Original quaternion from pose estimation
            q_orig = [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

            # Multiply the rotation quaternion with the original quaternion
            q_new = self.multiply_quaternions(q_rot, q_orig)

            # Normalize the resulting quaternion
            q_new = tf_transformations.unit_vector(q_new)

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header = msg.header  # Use the header from the detection message

            # Assign position
            pose_msg.pose.position.x = translation_vector[0][0]
            pose_msg.pose.position.y = translation_vector[1][0]
            pose_msg.pose.position.z = translation_vector[2][0]

            # Assign rotated orientation
            pose_msg.pose.orientation.x = q_new[0]
            pose_msg.pose.orientation.y = q_new[1]
            pose_msg.pose.orientation.z = q_new[2]
            pose_msg.pose.orientation.w = q_new[3]

            self.get_logger().info(
                f'Pose for tag ID {tag_id}: Position({pose_msg.pose.position.x:.2f}, '
                f'{pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f}) '
                f'Orientation({q_new[0]:.2f}, {q_new[1]:.2f}, '
                f'{q_new[2]:.2f}, {q_new[3]:.2f})'
            )

            # Publish the pose
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f'Published PoseStamped for tag ID: {tag_id}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AprilTagPosePublisher node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

