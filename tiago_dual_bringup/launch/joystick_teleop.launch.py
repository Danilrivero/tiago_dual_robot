# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_pal.arg_utils import read_launch_argument

from launch_pal.arg_utils import LaunchArgumentsBase
from tiago_dual_description.launch_arguments import TiagoDualArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    arm_type_right: DeclareLaunchArgument = TiagoDualArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoDualArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoDualArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoDualArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoDualArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoDualArgs.ft_sensor_left
    base_type: DeclareLaunchArgument = TiagoDualArgs.base_type

    cmd_vel: DeclareLaunchArgument = DeclareLaunchArgument(
        name="cmd_vel",
        default_value="input_joy/cmd_vel",
        description="Joystick cmd_vel topic",
    )


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    launch_description.add_action(OpaqueFunction(function=create_joy_teleop_filename))

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[LaunchConfiguration('teleop_config')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))])

    launch_description.add_action(joy_teleop_node)

    pkg_dir = get_package_share_directory('tiago_dual_bringup')

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick',
        parameters=[os.path.join(pkg_dir, 'config', 'joy_teleop', 'joy_config.yaml')])

    launch_description.add_action(joy_node)

    torso_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='torso_controller')

    launch_description.add_action(torso_incrementer_server)

    head_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='head_controller')

    launch_description.add_action(head_incrementer_server)

    gripper_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='gripper_right_controller',
        condition=LaunchConfigurationEquals('end_effector_right', 'pal-gripper'))

    launch_description.add_action(gripper_incrementer_server)

    return


def create_joy_teleop_filename(context):

    base_type = read_launch_argument("base_type", context)
    pkg_dir = get_package_share_directory("tiago_dual_bringup")

    joy_teleop_file = f"joy_teleop_{base_type}.yaml"

    joy_teleop_path = os.path.join(
        pkg_dir,
        "config",
        "joy_teleop",
        joy_teleop_file,
    )

    return [SetLaunchConfiguration("teleop_config", joy_teleop_path)]
