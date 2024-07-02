from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
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
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["orthrus_controller", "--controller-manager", "controller_manager"],
        output="both",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_state_pub_node,
                on_exit=[robot_controller_spawner],
            )
        )
    )
    
    foxglove = ExecuteProcess(
    cmd=['ros2', 
         'launch', 
         'foxglove_bridge', 
         'foxglove_bridge_launch.xml', 
         'use_compression:=true',
         'topic_whitelist:=["/tf", "/tf_static","/orthrus/visualization_marker","/orthrus/body_imu"]',
         'num_threads:=2',
         'min_qos_depth:=20',
         'max_qos_depth:=30'],
    output='log'
    )

        
    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        foxglove,
    ]

    return LaunchDescription(declared_arguments + nodes)