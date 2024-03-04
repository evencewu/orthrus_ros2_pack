import os
 
from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

    model_arg = DeclareLaunchArgument(name='models', description='Absolute path to robot urdf file')
    pkg_gazebo_ros = get_package_share_directory('orthrus_gazebo')


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

    #rivz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': use_sim_time}],
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

    active_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
        output='screen'
    )

    
    active_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','effort_controller'],
        output='screen'
    )

    delay_active_effort_controller = TimerAction(
        period=2.0,
        actions=[active_effort_controller]
    )

    return LaunchDescription([
    #rviz2,
    robot_state_publisher_node,
    ExecuteProcess(
        cmd=['gazebo', '--verbose', '-u' , '-s', 'libgazebo_ros_factory.so'],
        output='screen'),

    active_joint_state_broadcaster,
    active_effort_controller,
    orthrus_spawn,
])
