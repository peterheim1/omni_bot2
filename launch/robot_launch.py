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
        
    driver_pkg_dir = LaunchConfiguration(
        'driver_pkg_dir',
        default=os.path.join(get_package_share_directory('robot_control'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/joy_teleop.launch.py']),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/oakCam_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rs_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rplidar.launch.py']),
        ), 
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/rsp_launch.py']),
        ), 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([driver_pkg_dir, '/robot_control_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/slam_launch.py']),
        ), 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pkg_dir, '/april_launch.py']),
        ),

        

         Node(
            package="omni_bot2",
            executable="voice_serv.py",
            name="voice",
            output={
                "stdout": "screen",              
            },),
                 

        Node(
            package='omni_bot2',
            executable='teensy_imu.py',
            name='imu_publisher',
            output='screen'),   

        Node(
            package='omni_bot2',
            executable='dock_pose_pub.py',
            name='dock_republisher',
            output='screen'), 

        Node(
            package='omni_bot2',
            executable='docking_node.py',
            name='auto_dock_node',
            output='screen'),

       Node(
            package='omni_bot2',
            executable='docking_controller.py',
            name='joy_commander',
            output='screen'),     


    ])        