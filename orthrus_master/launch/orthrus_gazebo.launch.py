import os
import sys

import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_prefix,get_package_share_directory

from launch.launch_description_sources import AnyLaunchDescriptionSource

use_sim_time = LaunchConfiguration("use_sim_time", default="true")

def get_orthrus_gazebo_sim(package, executable):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package), "launch/"), executable]
        )
    )

foxglove = ExecuteProcess(
    cmd=[
        "ros2",
        "launch",
        "foxglove_bridge",
        "foxglove_bridge_launch.xml",
        "use_compression:=true",
        'topic_whitelist:=["/tf", "/tf_static","/orthrus/visualization_marker","/orthrus/body_imu"]',
        "num_threads:=2",
        "min_qos_depth:=20",
        "max_qos_depth:=30",
    ],
    output="log",
)

# from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# foxglove_bridge = IncludeLaunchDescription(
#        XMLLaunchDescriptionSource(
#            [get_package_share_directory("foxglove_bridge"), '/launch/foxglove_bridge_launch.xml']),
#        launch_arguments={
#            'include_hidden': 'true',
#        }.items(),
#    )


def generate_launch_description():

    orthrus_gazebo_sim = get_orthrus_gazebo_sim(
        "orthrus_gazebo", "orthrus_sim.launch.py"
    )

    return launch.LaunchDescription(
        [
            # RegisterEventHandler(
            #    event_handler = launch.event_handlers.OnProcessExit(
            #        target_action=orthrus_gazebo_sim,
            #        on_exit=[foxglove],
            #    )
            # ),
            foxglove,
            orthrus_gazebo_sim,
        ]
    )
