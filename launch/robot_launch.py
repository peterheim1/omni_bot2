#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    

    robot_pkg_dir = LaunchConfiguration(
        'robot_pkg_dir',
        default=os.path.join(get_package_share_directory('omni_bot2'), 'launch'))
        
    #rs_cam_pkg_dir = LaunchConfiguration(
        #'rs_cam_pkg_dir',
        #default=os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/joy_teleop.launch.py']),
        ),
        
        #IncludeLaunchDescription(
            #PythonLaunchDescriptionSource([robot_pkg_dir, '/rear_cam_launch.py']),
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rs_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rsp_launch.py']),
        ),        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rplidar.launch.py']),
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/slam_launch.py']),
        ), 
        #IncludeLaunchDescription(
            #PythonLaunchDescriptionSource([robot_pkg_dir, '/navigation_launch.py']),
        #),
        Node(
            package='omni_bot2',
            executable='new_controller.py',
            name='base_driver',
            output='screen'),

         Node(
            package='omni_bot2',
            executable='odom_v2.py',
            name='odom_publisher',
            output='screen'),   

        Node(
            package='omni_bot2',
            executable='teensy_imu.py',
            name='imu_publisher',
            output='screen'),     
           
    ])