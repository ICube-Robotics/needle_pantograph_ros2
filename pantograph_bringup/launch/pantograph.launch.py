# Copyright 2023 ICube Laboratory, University of Strasbourg
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

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition  # noqa: F401
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )

    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('pantograph_description'), 'config', 'pantograph.config.xacro']
            ),
            ' ',
            'use_sim:=', use_sim,
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    ros2_controllers = PathJoinSubstitution(
        [
            FindPackageShare('pantograph_description'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('pantograph_description'),
            'rviz',
            'display_robot.rviz',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers],
        output='both',
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    pantograph_mimick_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pantograph_mimick_controller'],
    )

    # pantograph_mock_motors_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['pantograph_mock_motors_controller'],
    #     condition=IfCondition(use_fake_hardware),
    # )
    # pantograph_mock_operator_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['pantograph_mock_operator_controller'],
    #     condition=IfCondition(use_fake_hardware),
    # )
    # effort_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_effort_controller'],
    # )

    stereo_cam_node = Node(
        package='pantograph_stereo_cam',
        executable='stereo_tracker',
        name='stereo_cam_node',
    )

    nodes = [
        control_node,
        rviz_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        pantograph_mimick_controller_spawner,
        # pantograph_mock_motors_controller_spawner,
        # pantograph_mock_operator_controller_spawner,
        # effort_controller_spawner,
        stereo_cam_node,

    ]

    return LaunchDescription(declared_arguments + nodes)
