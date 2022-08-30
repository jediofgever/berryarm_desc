# Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get hare directories of thorvald packages

    berryarm_desc_share_dir = get_package_share_directory(
        'berryarm_desc')

    # DECLARE THE ROBOT STATE PUBLISHER NODE
    xacro_file_name = 'arm.urdf.xacro'
    xacro_full_dir = os.path.join(
        berryarm_desc_share_dir, xacro_file_name)
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": False},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    #  INCLUDE RVIZ LAUNCH FILE IF use_rviz IS SET TO TRUE
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # output={'both': 'log'}, #change it to screen if you wanna see RVIZ output in terminal
        arguments=[
                   '--ros-args', '--log-level', 'ERROR'],
        # prefix=['xterm -e gdb -ex run --args'],
        # change it to screen if you wanna see RVIZ output in terminal
        # output={'both': 'screen'},
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.1'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               condition=IfCondition(
                                                   'True'),
                                               executable='spawn_entity.py',
                                               output='screen',
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description',
                                                   '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                                                   '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
                                               )

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(
        get_package_share_directory('gazebo_ros'), 'worlds/empty.world')
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s',
             'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen')

    position_goals = os.path.join(
        get_package_share_directory('berryarm_desc'),
        "rrbot_gazebo_forward_controller_position.yaml")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("berryarm_desc"),
                    "arm.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, position_goals],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui,
        declare_spawn_entity_to_gazebo_node,
        declare_start_gazebo_cmd,
        load_joint_state_controller,
        load_joint_trajectory_controller
        # control_node

    ])
