import os
import sys

import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_dir = get_package_share_directory("orthrus_interfaces")
simulation_description_path = os.path.join(pkg_dir)

robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("orthrus_interfaces"),
                "models",
                "orthrus",
                "urdf",
                "orthrus_real.urdf.xacro",
            ]
        ),
        " ",
    ]
)

robot_description = {"robot_description": robot_description_content}

robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("orthrus_control"),
        "config",
        "orthrus_controllers.yaml",
    ]
)

control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="screen",
)



node_robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
        {"robot_description": robot_description_content},
    ],
)

foxglove = ExecuteProcess(
    cmd=[
        "ros2",
        "launch",
        "foxglove_bridge",
        "foxglove_bridge_launch.xml",
        "use_compression:=true",
    ],
    output="log",
)

active_orthrus_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','orthrus_controller'],
    output='screen'
)


def generate_launch_description():
    return launch.LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=control_node,
                    on_exit=[active_orthrus_controller],
                )
            ),
            
            node_robot_state_publisher,
            control_node,
            #foxglove,
        ]
    )
