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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    mecanum_robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('akros2_description') / 'urdf/akros2_mecanum/robot.urdf.xacro')]),
        value_type=str)

    omni_robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('akros2_description') / 'urdf/akros2_omni/robot.urdf.xacro')]),
        value_type=str)
        
    diff_robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('akros2_description') / 'urdf/akros2_diff/robot.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled), diff (2 wheeled)'),

        DeclareLaunchArgument(
            name='js_ext',
            default_value='True',
            description='Enable Joint States from external nodes (like the micro-ROS node). If False, enable Joint States from the Joint State Publisher'),

        DeclareLaunchArgument(
            name='js_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) and required (req_states) topics'),

        GroupAction(
            condition=LaunchConfigurationEquals('config', 'mecanum'),
            actions = [
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': mecanum_robot_description}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),

                Node(
                    condition=UnlessCondition(LaunchConfiguration('js_ext')),
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{'source_list': ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),
            ]),

        GroupAction(
            condition=LaunchConfigurationEquals('config', 'omni'),
            actions = [
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': omni_robot_description}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),

                Node(
                    condition=UnlessCondition(LaunchConfiguration('js_ext')),
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{'source_list': ['joint_f', 'joint_l', 'joint_r']}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),
            ]),
            
        GroupAction(
            condition=LaunchConfigurationEquals('config', 'diff'),
            actions = [
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': diff_robot_description}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),

                Node(
                    condition=UnlessCondition(LaunchConfiguration('js_ext')),
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{'source_list': ['joint_l', 'joint_r']}],
                    remappings=[
                        ('/joint_states', ['/', LaunchConfiguration('js_topic')])
                    ]),
            ]),
    ])
