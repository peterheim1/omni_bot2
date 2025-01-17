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
import tf2_ros
import tf2_geometry_msgs  # Required for transforming PoseStamped messages
from math import atan2

class AprilTagPosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')

        # Hard-coded debug variable
        self.debug = True  # Set to True to enable debug logs, False to disable

        # Publisher for PoseStamped messages in base_link frame
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

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

    def get_yaw_from_quaternion(self, orientation):
        """
        Convert quaternion to yaw angle.

        Args:
            orientation (geometry_msgs.msg.Quaternion): Orientation in quaternion.

        Returns:
            float: Yaw angle in radians.
        """
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback function to handle incoming CameraInfo messages.
        Extracts and stores the camera intrinsic parameters.

        Args:
            msg (CameraInfo): Incoming CameraInfo message.
        """
        with self.camera_info_lock:
            if not self.camera_info_received:
                # Extract camera matrix
                self.K = np.array(msg.k).reshape((3, 3))
                # Extract distortion coefficients
                self.dist_coeffs = np.array(msg.d).reshape(-1, 1)
                self.camera_info_received = True
                self.get_logger().info('Camera intrinsic parameters received.')

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        Callback function to handle incoming AprilTagDetectionArray messages.
        Processes each detected AprilTag, estimates its pose, transforms it to the base_link frame,
        and publishes the transformed PoseStamped message.

        Args:
            msg (AprilTagDetectionArray): Incoming AprilTag detections.
        """
        with self.camera_info_lock:
            if not self.camera_info_received:
                self.get_logger().warn('CameraInfo not received yet. Skipping detection.')
                return

        if not msg.detections:
            if self.debug:
                self.get_logger().debug('No AprilTags detected in this frame.')
            return

        for detection in msg.detections:
            tag_id = detection.id
            if self.debug:
                self.get_logger().debug(f'Processing AprilTag ID: {tag_id}')

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
            # Initially set header to the camera frame
            pose_msg.header = msg.header  # frame_id: 'rear_camera_color_optical_frame'

            # Assign position
            pose_msg.pose.position.x = translation_vector[0][0]
            pose_msg.pose.position.y = translation_vector[1][0]
            pose_msg.pose.position.z = translation_vector[2][0]

            # Assign rotated orientation
            pose_msg.pose.orientation.x = q_new[0]
            pose_msg.pose.orientation.y = q_new[1]
            pose_msg.pose.orientation.z = q_new[2]
            pose_msg.pose.orientation.w = q_new[3]

            if self.debug:
                self.get_logger().debug(
                    f'Pose for tag ID {tag_id}: Position({pose_msg.pose.position.x:.2f}, '
                    f'{pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f}) '
                    f'Orientation({q_new[0]:.2f}, {q_new[1]:.2f}, '
                    f'{q_new[2]:.2f}, {q_new[3]:.2f})'
                )

            # Transform the PoseStamped message to base_link frame
            try:
                # Define the target frame
                target_frame = 'base_link'

                # Perform the transformation
                transformed_pose = self.tf_buffer.transform(
                    pose_msg,
                    target_frame,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                # Update the frame_id to base_link
                transformed_pose.header.frame_id = target_frame

                if self.debug:
                    yaw = self.get_yaw_from_quaternion(transformed_pose.pose.orientation)
                    self.get_logger().debug(
                        f'Transformed Pose for tag ID {tag_id}: Position({transformed_pose.pose.position.x:.2f}, '
                        f'{transformed_pose.pose.position.y:.2f}, {transformed_pose.pose.position.z:.2f}) '
                        f'Orientation (yaw)=({yaw:.2f} rad)'
                    )

                # Publish the transformed pose
                self.pose_publisher.publish(transformed_pose)
                if self.debug:
                    self.get_logger().debug(f'Published transformed PoseStamped for tag ID: {tag_id}')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Transform failed for tag ID {tag_id}: {e}. Pose not published.')
                continue

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
