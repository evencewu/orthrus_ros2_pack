import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

#sys.path.append(os.path.join(get_package_share_directory('orthrus_master'), 'launch'))

use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
def get_orthrus_gazebo(package, executable, name):
    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'use_sim_time': use_sim_time}],
    )

def get_orthrus_gazebo_sim(package, executable):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), 'launch/'),executable])
    )

def generate_launch_description():
    orthrus_gazebo_node = get_orthrus_gazebo('orthrus_gazebo', 'orthrus_gazebo','orthrus_gazebo_node')
    orthrus_gazebo_sim = get_orthrus_gazebo_sim('orthrus_gazebo', 'orthrus_sim.launch.py')

    return launch.LaunchDescription(
        [
            orthrus_gazebo_sim,
            orthrus_gazebo_node,
        ]
    )