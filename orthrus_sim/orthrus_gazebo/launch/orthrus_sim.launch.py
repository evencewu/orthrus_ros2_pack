import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():
            
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch/'), 'gzserver.launch.py']),
    ),

    pkg_dir = get_package_share_directory('orthrus_interfaces')
    simulation_description_path = os.path.join(pkg_dir)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [simulation_description_path, "models", "orthrus", "urdf", "orthrus.urdf.xacro"]
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

    controller_manager  =Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_content,[os.path.join(get_package_share_directory('orthrus_gazebo'), 'config/'), 'orthrus.yaml']]
    )
    
    active_orthrus_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','orthrus_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch/'), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[active_orthrus_controller],
            )
        ),
        
        node_robot_state_publisher,
        spawn_entity,
    ])