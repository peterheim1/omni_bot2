#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Locate the 'omni_bot2' package share directory
    omni_bot2_share = FindPackageShare('omni_bot2').find('omni_bot2')

    # Path to the apriltag_ros configuration file
    tags_config = os.path.join(omni_bot2_share, 'config', 'tags_36h11.yaml')

    # Define the apriltag_ros node as a composable node
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag',
        namespace='apriltag',
        parameters=[tags_config],
        remappings=[
            ('image_rect', '/rear_camera/color/image_raw'),
            ('camera_info', '/rear_camera/color/camera_info')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create a container for the composable nodes
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            apriltag_node
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        container
    ])

