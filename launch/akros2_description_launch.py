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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    
    urdf_file_path = os.path.join(get_package_share_directory('akros2_description'), 'urdf', 'akros2.urdf')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='micro_ros_ns',
            default_value='drive',
            description='Namespace of the micro-ROS system'),
        
        DeclareLaunchArgument(
            name='micro_ros',
            default_value='false',
            description='Enable Joint States from the micro-ROS node'),
        
        DeclareLaunchArgument(
            name='joint_state_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) and required (req_states) topics'),
        
        DeclareLaunchArgument(
            name='mesh_pub',
            default_value='true',
            description='Launches mesh publisher if set to true'),
        
        SetEnvironmentVariable('ROBOT_DESCRIPTION', 'file://' + urdf_file_path),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file_path).read()}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('micro_ros_ns'), '/', LaunchConfiguration('joint_state_topic')])
            ]),
        
        Node(
            condition=UnlessCondition(LaunchConfiguration('micro_ros')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('micro_ros_ns'), '/', LaunchConfiguration('joint_state_topic')])
            ]),
        
        Node(
            condition=IfCondition(LaunchConfiguration('mesh_pub')),
            package='akros2_description',
            executable='mesh_publisher',
            name='mesh_publisher',
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('micro_ros_ns'), '/', LaunchConfiguration('joint_state_topic')])
            ]),
        
        
    ])