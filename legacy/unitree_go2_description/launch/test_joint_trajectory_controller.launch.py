# '''
# Author: Dyyt587 805207319@qq.com
# Date: 2024-09-17 23:07:35
# LastEditors: Dyyt587 805207319@qq.com
# LastEditTime: 2024-09-18 15:18:48
# FilePath: /GitHub/test_ws/adog_description/launch/test_joint_trajectory_controller.launch.py
# Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
# '''

# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("unitree_go2_description"),
            "config",
            "joint_trajectory_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="publisher_forward_position_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )
