import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    pkg_dir = get_package_share_directory('orthrus_gazebo')
    simulation_description_path = os.path.join(pkg_dir)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [simulation_description_path, "models", "orthrus", "urdf", "orthrus_for_gazebo.urdf.xacro"]
            ),
        ]
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", "orthrus",
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '1.0'],
        output='screen'
    )

    active_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','orthrus_gazebo_joint_state'],
        output='screen'
    )

    
    active_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','orthrus_gazebo_joint_effort_controller'],
        output='screen'
    )

    active_imu_sensor_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','orthrus_gazebo_imu'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[active_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_joint_state_broadcaster,
                on_exit=[active_effort_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=active_effort_controller,
                on_exit=[active_imu_sensor_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])