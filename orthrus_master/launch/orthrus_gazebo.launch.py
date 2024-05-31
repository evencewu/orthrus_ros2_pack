import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

use_sim_time = LaunchConfiguration('use_sim_time', default='true')

urdf = os.path.join(
    get_package_share_directory('orthrus_interfaces'),
    "models", "orthrus", "urdf", "orthrus.urdf")
with open(urdf, 'r') as infp:
    robot_desc = infp.read()

def get_orthrus_ctrl(package, executable, name):
    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'use_sim_time': use_sim_time}],
    )

def get_orthrus_state_publisher():
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf]
    )

def get_orthrus_gazebo_sim(package, executable):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), 'launch/'),executable])
    )

def generate_launch_description():
    orthrus_gazebo_sim = get_orthrus_gazebo_sim('orthrus_gazebo', 'orthrus_sim.launch.py')
    orthrus_state_publisher = get_orthrus_state_publisher()

    return launch.LaunchDescription(
        [
            orthrus_gazebo_sim,
            orthrus_state_publisher,
        ]
    )

