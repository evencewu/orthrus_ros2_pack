import os
 
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    gui = LaunchConfiguration("gui")


    model_arg = DeclareLaunchArgument(name='models', description='Absolute path to robot urdf file')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("orthrus_gazebo"), "models", "orthrus", "urdf",  "orthrus.urdf.xacro"]
            ),
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("orthrus_gazebo"),
            "config",
            "orthrus.yaml",
        ]
    )

    #rivz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_content}, robot_controllers],
        output="both",
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    #spawn the robot 
    orthrus_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", "othrus",
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '1.0']
        
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
    #rviz2,
    control_node,
    robot_state_publisher_node,
    joint_state_broadcaster_spawner,
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    orthrus_spawn,
])
