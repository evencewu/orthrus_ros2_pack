import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode

from launch.actions import TimerAction

use_sim_time = LaunchConfiguration('use_sim_time', default='true')

urdf = os.path.join(
    get_package_share_directory('orthrus_interfaces'),
    "models", "orthrus", "urdf", "orthrus_for_gazebo.urdf.xacro")
with open(urdf, 'r') as infp:
    robot_desc = infp.read()

def get_orthrus_ctrl(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'use_sim_time': use_sim_time}],
    )

def get_orthrus_state_publisher(package, executable, name):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf]
    )

def get_orthrus_gazebo_sim(package, executable):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), 'launch/'),executable])
    )