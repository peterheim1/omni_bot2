#!/usr/bin/env python3
"""
Launch file for robot_state_publisher and joint_state_publisher_gui.

This file converts a xacro file to a URDF description, publishes it via the
robot_state_publisher node, and launches the joint_state_publisher_gui node.
It also supports simulation time via a launch argument.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    
    

    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('omni_bot2'),
        'urdf',
        'dual_arm_robot.urdf.xacro'
    )

    # Robot state publisher node: converts xacro to URDF and publishes robot_description.
    # We wrap the xacro command output in a ParameterValue with value_type=str.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)
        }]
    )

    # Joint state publisher GUI node for joint state visualization and debugging
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            
        ]
    )
    
    # Joint state publisher GUI node for joint state visualization and debugging
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            
        ]
    )

    # Create and populate launch description
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    #ld.add_action(joint_state_publisher_gui_node)
    #ld.add_action(joint_state_publisher_node)

    return ld
