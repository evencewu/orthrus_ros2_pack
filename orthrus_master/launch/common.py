import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode

from launch.actions import TimerAction

#launch_params = yaml.safe_load(
#    open(
#        os.path.join(
#            get_package_share_directory("radar_master"),
#            "config",
#            "launch_params.yaml",
#        )
#    )
#)
#
#node_params = os.path.join(
#    get_package_share_directory("radar_master"), "config", "node_params.yaml"
#)

    
#def get_camera_node(package, plugin, name):
#    return ComposableNode(
#        package=package,
#        plugin=plugin,
#        name= name,
#        parameters=[node_params],
#        extra_arguments=[{"use_intra_process_comms": True}],
#    )

def get_orthrus_ctrl(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
    )
    
def get_orthrus_gazebo(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
    )