# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('akros2_description') / 'urdf/akros2.urdf.xacro')]),
        value_type=str)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='uros_ns',
            default_value='drive',
            description='Namespace of the micro-ROS system'),
        
        DeclareLaunchArgument(
            name='uros',
            default_value='false',
            description='Enable Joint States from the micro-ROS node'),
        
        DeclareLaunchArgument(
            name='joint_state_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) and required (req_states) topics'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('uros_ns'), '/', LaunchConfiguration('joint_state_topic')])
            ]),
        
        Node(
            condition=UnlessCondition(LaunchConfiguration('uros')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('uros_ns'), '/', LaunchConfiguration('joint_state_topic')])
            ]),
    ])