#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    suitee_xacro = os.path.join(get_package_share_directory('omni_bot2'), 'urdf', 'robbie.urdf.xacro')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
        
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot',
        output='screen',
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', suitee_xacro])
        }])
        
    
        
    joint_state_publisher_gui_node = Node(
            name='joint_state_publisher_gui',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )   
            

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    
    
    ld.add_action(robot_state_publisher_node)
    
    #ld.add_action(joint_state_publisher_gui_node)
    

    return ld

