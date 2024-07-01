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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
  lidar = LaunchConfiguration('lidar')
  realsense = LaunchConfiguration('realsense')

  declare_lidar_cmd = DeclareLaunchArgument(
    'lidar',
    default_value='true',
    description='Launch hesai lidar driver'
  )

  declare_realsense_cmd = DeclareLaunchArgument(
    'realsense',
    default_value='true',
    description='Launch realsense driver'
  )

  robot_description_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('go2_description'),
      'launch/'), 'robot.launch.py'])
  )

  driver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('go2_driver'),
      'launch/'), 'go2_driver.launch.py'])
  )

  lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hesai_ros_driver'),
            'launch/'), 'start.py']),
        condition=IfCondition(PythonExpression([lidar]))
  )

  realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch/'), 'rs_launch.py']),
        condition=IfCondition(PythonExpression([realsense]))
  )

  ld = LaunchDescription()
  ld.add_action(declare_lidar_cmd)
  ld.add_action(declare_realsense_cmd)
  ld.add_action(robot_description_cmd)
  ld.add_action(driver_cmd)
  ld.add_action(lidar_cmd)
  ld.add_action(realsense_cmd)

  return ld