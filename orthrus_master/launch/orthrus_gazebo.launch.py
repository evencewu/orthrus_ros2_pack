import os
import sys

import launch
import launch_ros.actions
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import AnyLaunchDescriptionSource

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



def get_orthrus_gazebo_sim(package, executable):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), 'launch/'),executable])
    )
    
#ros2 launch foxglove_bridge foxglove_bridge_launch.xml

#foxglove_bridge_share_dir = get_package_share_directory('foxglove_bridge')
#xml_launch_file = os.path.join(foxglove_bridge_share_dir, 'launch', 'foxglove_bridge_launch.xml','use_compression:=true')
#foxglove = launch.actions.IncludeLaunchDescription(AnyLaunchDescriptionSource(xml_launch_file))

foxglove = ExecuteProcess(
    cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml', 'use_compression:=true'],
    output='log'
)

#from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

#foxglove_bridge = IncludeLaunchDescription(
#        XMLLaunchDescriptionSource(
#            [get_package_share_directory("foxglove_bridge"), '/launch/foxglove_bridge_launch.xml']),
#        launch_arguments={
#            'include_hidden': 'true',
#        }.items(),
#    )

def generate_launch_description():

    orthrus_gazebo_sim = get_orthrus_gazebo_sim('orthrus_gazebo', 'orthrus_sim.launch.py')

    return launch.LaunchDescription(
        [
            #RegisterEventHandler(
            #    event_handler = launch.event_handlers.OnProcessExit(
            #        target_action=orthrus_gazebo_sim,
            #        on_exit=[foxglove],
            #    )
            #),
            
            foxglove,
            orthrus_gazebo_sim,
        ]
    )