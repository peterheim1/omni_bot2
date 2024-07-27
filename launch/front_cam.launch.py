import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='front_camera',
            parameters=[{
                'image_size': [640,480],
                'time_per_frame': [1, 15],
                'Vertical Flip': 1,
                'camera_frame_id': 'front_camera_link_optical'
                }]
    )
    ])