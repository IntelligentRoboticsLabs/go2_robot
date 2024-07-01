# BSD 3-Clause License

# Copyright (c) 2024, Intelligent Robotics Lab
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    nodes = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='go2_description',
            description='Description package with robot URDF/xacro files made by manufacturer.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='go2_description.urdf',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to be added to the robot description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation/Gazebo clock if true',
        )
    )

    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('go2_description'), 'urdf', description_file]),
            ' ',
            'prefix:=', prefix
        ]
    )

    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content,
                                                                     value_type=str)

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param,
            'publish_frequency': 100.0,
            'frame_prefix': '',
            }],
        )
    )

    return LaunchDescription(declared_arguments + nodes)
