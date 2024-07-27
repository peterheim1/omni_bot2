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
    

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
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
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/joy_teleop.launch.py']),
        ),

        #IncludeLaunchDescription(
            #PythonLaunchDescriptionSource([lidar_pkg_dir, '/robbie_robot.launch.py']),
        #),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/rsp_launch.py']),
        ),        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/rplidar.launch.py']),
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/localization_launch.py']),
        ), 
        #IncludeLaunchDescription(
            #PythonLaunchDescriptionSource([lidar_pkg_dir, '/navigation_launch.py']),
        #),
        Node(
            package='omni_bot2',
            #namespace='robbie',
            executable='old_controller.py',
            name='base_driver',
            output='screen'),

         Node(
            package='omni_bot2',
            executable='odom_50hz.py',
            name='odom_driver',
            output='screen'),   
           
    ])