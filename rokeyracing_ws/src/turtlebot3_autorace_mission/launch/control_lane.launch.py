#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Hyungyu Kim

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():

#     DeclareLaunchArgument(
#             'image_width', 
#             default_value='640',  
#             description='camera frame width'
#         ), 
#     DeclareLaunchArgument(
#             'image_height', 
#             default_value='480',  
#             description='camera frame height'
#         ), 

#     control_node = Node(
#             package='turtlebot3_autorace_mission',
#             executable='control_lane',
#             name='control_lane',
#             output='screen',
#             parameters=[{
#                 'image_width': LaunchConfiguration('image_width'),
#                 'image_height': LaunchConfiguration('image_height'),                                             
#             }],            
#             arguments=['--ros-args', '--log-level', 'INFO'],
#             remappings=[
#                 ('/control/lane', '/detect/lane'),
#                 ('/control/cmd_vel', '/cmd_vel')
#             ]
#         )
    
#     return LaunchDescription([
#         control_node
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_node = Node(
            package='turtlebot3_autorace_mission',
            executable='control_lane',
            name='control_lane',
            output='screen',
            remappings=[
                ('/control/lane', '/detect/lane'),
                # ('/control/cmd_vel', '/cmd_vel')
            ]
        )
    return LaunchDescription([
        control_node
    ])