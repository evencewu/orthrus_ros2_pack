import os
import sys

from launch_ros.actions import ComposableNodeContainer 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import TimerAction

sys.path.append(os.path.join(get_package_share_directory('orthrus_master'), 'launch'))

def generate_launch_description():
    import common


    
    orthrus_ctrl_node = common.get_orthrus_ctrl('orthrus_controllers', 'orthrus_controllers','orthrus_ctrl_node')
    
    orthrus_gazebo_node = common.get_orthrus_gazebo('orthrus_gazebo', 'orthrus_gazebo','orthrus_gazebo_node')
    orthrus_gazebo_sim = common.get_orthrus_gazebo_sim('orthrus_gazebo', 'orthrus_sim.launch.py')
    return LaunchDescription(
        [
            orthrus_gazebo_sim,
            orthrus_ctrl_node,
            orthrus_gazebo_node,
        ]
    )